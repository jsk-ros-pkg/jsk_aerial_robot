import os, sys
import time
import numpy as np
import torch
from torch.utils.data import DataLoader, random_split
from progress_table import ProgressTable
from torchsummary import summary
from decimal import Decimal

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from dataset import TrajectoryDataset
from network_architecture.mlp import MLP
from network_architecture.vae import VAE
from vae_loss import VAELoss
from utils.data_utils import sanity_check_dataset, get_model_dir_and_file, read_dataset, log_metrics
from utils.model_utils import sanity_check_features_and_reg_dims, get_device
from utils.visualization_utils import plot_fitting
from config.configurations import NetworkConfig, ModelFitConfig


def main(test: bool = False, plot: bool = False, save: bool = True):
    device = get_device()

    train_ds_name = ModelFitConfig.train_ds_name
    train_ds_instance = ModelFitConfig.train_ds_instance
    sanity_check_dataset(train_ds_name, train_ds_instance)
    val_ds_name = ModelFitConfig.val_ds_name
    val_ds_instance = ModelFitConfig.val_ds_instance
    sanity_check_dataset(val_ds_name, val_ds_instance)

    # Define model input and output features
    state_feats = ModelFitConfig.state_feats
    u_feats = ModelFitConfig.u_feats
    y_reg_dims = ModelFitConfig.y_reg_dims

    # Set save path and populate metadata
    if save or ModelFitConfig.save_plots:
        save_file_path, save_file_name = get_model_dir_and_file(train_ds_name, train_ds_instance, NetworkConfig.model_name)
    # Raw data
    train_df = read_dataset(train_ds_name, train_ds_instance)
    val_df = read_dataset(val_ds_name, val_ds_instance)

    # === Datasets ===
    # TODO prune w.r.t. angular velocity as well
    print("Loading dataset...")
    train_dataset = TrajectoryDataset(
        train_df,
        state_feats,
        u_feats,
        y_reg_dims,
        save_file_path=save_file_path,
        save_file_name=save_file_name,
        mode="train",
    )
    val_dataset = TrajectoryDataset(
        val_df,
        state_feats,
        u_feats,
        y_reg_dims,
        save_file_path=save_file_path,
        save_file_name=save_file_name,
        mode="val",
    )
    print("Finished loading the dataset!")
    in_dim = train_dataset.x.shape[1]
    out_dim = train_dataset.y.shape[1]
    sanity_check_features_and_reg_dims(
        NetworkConfig.model_name, state_feats, u_feats, y_reg_dims, in_dim, out_dim, NetworkConfig.delay_horizon
    )

    if test:
        val_size = int(0.5 * len(val_dataset))
        test_size = len(val_dataset) - val_size
        val_dataset, test_dataset = random_split(val_dataset, [val_size, test_size])
    else:
        test_dataset = []

    # === Dataloaders ===
    train_dataloader, val_dataloader, test_dataloader = get_dataloaders(
        train_dataset, val_dataset, test_dataset, batch_size=NetworkConfig.batch_size, num_workers=NetworkConfig.num_workers
    )

    # === Neural Network ===
    print(f"\n{'='*50}")
    print(f"Using {NetworkConfig.model_type} architecture")
    print(f"{'='*50}\n")
    
    if NetworkConfig.model_type == "MLP":
        model = MLP(
            in_dim,
            NetworkConfig.hidden_sizes,
            out_dim,
            activation=NetworkConfig.activation,
            use_batch_norm=NetworkConfig.use_batch_norm,
            dropout_p=NetworkConfig.dropout_p,
            x_mean=torch.tensor(train_dataset.x_mean),
            x_std=torch.tensor(train_dataset.x_std),
            y_mean=torch.tensor(train_dataset.y_mean),
            y_std=torch.tensor(train_dataset.y_std),
        ).to(device)
        print(model)
        summary(model, (in_dim,))
    elif NetworkConfig.model_type == "VAE":
        model = VAE(
            in_dim,
            NetworkConfig.encoder_hidden_sizes,
            NetworkConfig.latent_dim,
            NetworkConfig.decoder_hidden_sizes,
            out_dim,
            activation=NetworkConfig.activation,
            use_batch_norm=NetworkConfig.use_batch_norm,
            dropout_p=NetworkConfig.dropout_p,
            x_mean=torch.tensor(train_dataset.x_mean),
            x_std=torch.tensor(train_dataset.x_std),
            y_mean=torch.tensor(train_dataset.y_mean),
            y_std=torch.tensor(train_dataset.y_std),
        ).to(device)
        print(model)
        summary(model, (in_dim,))
        print(f"Encoder input: {in_dim} -> Latent: {NetworkConfig.latent_dim} -> Decoder output: {out_dim} | Beta: {NetworkConfig.beta}")

    # === Loss function ===
    weight = torch.tensor(NetworkConfig.loss_weight).to(device)
    if len(NetworkConfig.loss_weight) != out_dim:
        raise ValueError("Loss weight doesn't match output dimension!")
    if NetworkConfig.model_type == "MLP":
        loss_function = l2_loss_function

    # === Optimizer ===
    optimizer = get_optimizer(model, NetworkConfig.learning_rate)
    if NetworkConfig.lr_scheduler is not None:
        if NetworkConfig.lr_scheduler == "ReduceLROnPlateau":
            lr_scheduler = torch.optim.lr_scheduler.ReduceLROnPlateau(
                optimizer, factor=0.1, threshold=0.005, patience=20
            )
        elif NetworkConfig.lr_scheduler == "LambdaLR":
            # Divide here since lambda func returns a multiplier for the base lr
            lr_func = lambda epoch: max(0.95**epoch, 1e-5 / 1e-3)
            lr_scheduler = torch.optim.lr_scheduler.LambdaLR(optimizer, lr_lambda=lr_func)
        elif NetworkConfig.lr_scheduler == "MultiStepLR":
            lr_scheduler = torch.optim.lr_scheduler.MultiStepLR(optimizer, milestones=NetworkConfig.lr_milestones, gamma=0.1)
        elif NetworkConfig.lr_scheduler == "LRScheduler":
            lr_scheduler = torch.optim.lr_scheduler.LRScheduler(optimizer)
        else:
            raise ValueError(f"Unsupported learning rate scheduler: {NetworkConfig.lr_scheduler}")
    learning_rates = []

    # === Logging ===
    total_losses = {"train": [], "val": []}
    if NetworkConfig.model_type == "VAE":
        total_losses["train_recon"] = []
        total_losses["train_kl"] = []
        total_losses["val_recon"] = []
        total_losses["val_kl"] = []
    inference_times = []
    table = ProgressTable(
        pbar_embedded=False,
        # pbar_style_embed="rich",
        pbar_style="rich",
        # pbar_style="angled alt red blue",
        num_decimal_places=6,
    )
    table.add_column("Epoch", color="white", width=9)
    table.add_column("Step", color="DIM", width=13)
    table.add_column("Train Loss", color="BRIGHT red", width=19)
    if NetworkConfig.model_type == "VAE":
        table.add_column("Recon Loss", color="yellow", width=15)
        table.add_column("KL Loss", color="LIGHTYELLOW_EX", width=15)
    if NetworkConfig.zero_out_lambda > 0.0:
        table.add_column("Zero-Out Reg", color="blue", width=15)
    if NetworkConfig.l1_lambda > 0.0:
        table.add_column("L1 Reg", color="green", width=15)
    if NetworkConfig.gradient_lambda > 0.0:
        table.add_column("Gradient", color="magenta", width=15)
    if NetworkConfig.consistency_lambda > 0.0:
        table.add_column("Consistency", color="cyan", width=15)
    table.add_column("Val Loss", color="BRIGHT green", width=19)
    table.add_column("Inference Time", width=17)
    table.add_column("LR", width=7)

    # === Training Loop ===
    print("==========================================")
    print(f"Starting training model {save_file_name}...")
    for t in table(NetworkConfig.num_epochs, show_throughput=False, show_eta=True):
        table["Epoch"] = f"{t+1}/{NetworkConfig.num_epochs}"
        
        # KL annealing for VAE
        if NetworkConfig.model_type == "VAE":
            if NetworkConfig.use_kl_annealing:
                # Gradually increase beta over epochs
                beta = min(1.0, (t + 1) / NetworkConfig.kl_annealing_epochs) * NetworkConfig.beta
            else:
                beta = NetworkConfig.beta
            loss_function = VAELoss(beta=beta, weight=weight, device=device)

        # === Train Step ===
        train_losses = train(train_dataloader, model, loss_function, weight, optimizer, device, table)
        total_losses["train"].append(train_losses)

        # === Validation ===
        val_losses, inference_time = inference(val_dataloader, model, loss_function, weight, device)
        table["Val Loss"] = val_losses
        table["Inference Time"] = f"{inference_time:.2f} ms"
        total_losses["val"].append(val_losses)
        inference_times.append(inference_time)

        # === Schedule learning rate ===
        if NetworkConfig.lr_scheduler == "ReduceLROnPlateau":
            # lr_scheduler.step(train_losses)
            lr_scheduler.step(val_losses)
        elif NetworkConfig.lr_scheduler in ["LambdaLR", "LRScheduler", "MultiStepLR"]:
            lr_scheduler.step()
        learning_rates.append(optimizer.param_groups[0]["lr"])
        table["LR"] = f"{Decimal(learning_rates[-1]):.0e}"
        table.next_row()

        # === Save model ===
        if save:
            if NetworkConfig.model_type == "MLP":
                save_dict = {
                    "state_dict": model.state_dict(),
                    "model_type": NetworkConfig.model_type,
                    "input_size": in_dim,
                    "hidden_sizes": NetworkConfig.hidden_sizes,
                    "output_size": out_dim,
                    "activation": NetworkConfig.activation,
                    "use_batch_norm": NetworkConfig.use_batch_norm,
                    "dropout_p": NetworkConfig.dropout_p,
                }
            elif NetworkConfig.model_type == "VAE":
                save_dict = {
                    "state_dict": model.state_dict(),
                    "model_type": NetworkConfig.model_type,
                    "input_size": in_dim,
                    "encoder_hidden_sizes": NetworkConfig.encoder_hidden_sizes,
                    "latent_dim": NetworkConfig.latent_dim,
                    "decoder_hidden_sizes": NetworkConfig.decoder_hidden_sizes,
                    "output_size": out_dim,
                    "activation": NetworkConfig.activation,
                    "use_batch_norm": NetworkConfig.use_batch_norm,
                    "dropout_p": NetworkConfig.dropout_p,
                }
            torch.save(save_dict, os.path.join(save_file_path, f"{save_file_name}.pt"))
    table.close()
    print("Training Finished!")

    # === Testing ===
    if test:
        test_losses, _ = inference(test_dataloader, model, loss_function, weight, device)
        total_losses["test"] = test_losses
        print(f"Test avg loss: {test_losses:>8f}")

    # === Store metrics ===
    log_metrics(total_losses, inference_times, learning_rates, save_file_path, save_file_name)
    if save:
        print(f"Model saved to results/{NetworkConfig.model_name}/{save_file_name}.pt")

    # === Plotting ===
    if plot:
        plot_fitting(total_losses, inference_times, learning_rates, save_file_path, save_file_name)
        halt = 1


def get_dataloaders(train_data, val_data, test_data, batch_size=64, num_workers=0):
    train_dataloader = DataLoader(train_data, batch_size=batch_size, shuffle=True, num_workers=num_workers)
    val_dataloader = DataLoader(val_data, batch_size=4096, shuffle=True, num_workers=num_workers)
    if len(test_data) == 0:
        test_dataloader = None
    else:
        test_dataloader = DataLoader(test_data, batch_size=4096, shuffle=True, num_workers=num_workers)
    return train_dataloader, val_dataloader, test_dataloader


def l2_loss_function(y, y_pred, weight):
    # Weight each dimension
    return torch.mean(torch.square(y - y_pred) * weight)


def get_optimizer(model, learning_rate):
    optimizer_class = getattr(torch.optim, NetworkConfig.optimizer)
    return optimizer_class(model.parameters(), lr=learning_rate, weight_decay=NetworkConfig.weight_decay)


def train(dataloader, model, loss_function, weight, optimizer, device, table):
    size = len(dataloader.dataset)
    model.train()
    loss_avg = 0.0
    mov_size = 0
    for x, y in dataloader:
        x, y = x.to(device), y.to(device)
        optimizer.zero_grad()
        if NetworkConfig.gradient_lambda > 0.0:
            x.requires_grad = True

        # === Forward pass ===
        if NetworkConfig.model_type == "MLP":
            y_pred = model(x)
            loss = loss_function(y, y_pred, weight)
        elif NetworkConfig.model_type == "VAE":
            y_pred, mu, logvar = model(x)
            loss, recon_loss, kl_loss = loss_function(y_pred, mu, logvar, y)  # recon_loss & kl_loss are only for logging

        # === Regularizations ===
        # Zero-output regularization
        if NetworkConfig.zero_out_lambda > 0.0:
            if NetworkConfig.model_type == "MLP":
                loss_reg = NetworkConfig.zero_out_lambda * loss_function(torch.zeros_like(y_pred), y_pred, weight)
            elif NetworkConfig.model_type == "VAE":
                raise NotImplementedError("Zero-out regularization not implemented for VAE yet.")
                # y_zero, _, _ = model(torch.zeros_like(x))
                # loss_reg = NetworkConfig.zero_out_lambda * loss_function(
                #     y_zero, torch.zeros_like(y_zero), weight
                # )
            loss += loss_reg
        # L1 regularization
        if NetworkConfig.l1_lambda > 0.0:
            loss_l1 = NetworkConfig.l1_lambda * sum(p.abs().sum() for p in model.parameters())
            loss += loss_l1
        # L2 regularization (weight decay) is handled by the optimizer

        # === Gradient penalty ===
        if NetworkConfig.gradient_lambda > 0.0:
            gradients = torch.autograd.grad(
                outputs=y_pred, inputs=x, grad_outputs=torch.ones_like(y_pred), create_graph=True
            )[0]
            gradient_penalty = torch.mean(torch.square(gradients))
            loss_gradient = NetworkConfig.gradient_lambda * gradient_penalty
            loss += loss_gradient

        # === Output consistency ===
        if NetworkConfig.consistency_lambda > 0.0:
            # Add relative noise based on input magnitude with standard normal distribution
            loss_consistency = 0.0
            for _ in range(NetworkConfig.consistency_num_samples):
                noise = torch.rand_like(x) * 2.0 - 1.0  # Uniform noise in [-1, 1]
                noisy_input = x + NetworkConfig.consistency_epsilon * model.x_std * noise  # element-wise noise scaling by standard variation
                if NetworkConfig.model_type == "MLP":
                    y_pred_noise = model(noisy_input)
                elif NetworkConfig.model_type == "VAE":
                    y_pred_noise, _, _ = model(noisy_input)
                loss_consistency += NetworkConfig.consistency_lambda * torch.mean(torch.square(y_pred - y_pred_noise) * weight)
            loss_consistency /= NetworkConfig.consistency_num_samples
            loss += loss_consistency

        # === Backpropagation ===
        loss.backward()
        optimizer.step()

        # === Logging ===
        # Weighted moving average
        curr_batch_size = x.shape[0]
        prev_mov_size = mov_size
        mov_size += curr_batch_size
        loss_avg = (loss_avg * prev_mov_size + loss.item() * curr_batch_size) / mov_size
        table["Step"] = f"{mov_size}/{size}"
        table["Train Loss"] = loss_avg
        if NetworkConfig.model_type == "VAE":
            table["Recon Loss"] = recon_loss
            table["KL Loss"] = kl_loss
        if NetworkConfig.zero_out_lambda > 0.0:
            table["Zero-Out Reg"] = loss_reg
        if NetworkConfig.l1_lambda > 0.0:
            table["L1 Reg"] = loss_l1
        if NetworkConfig.gradient_lambda > 0.0:
            table["Gradient"] = loss_gradient
        if NetworkConfig.consistency_lambda > 0.0:
            table["Consistency"] = loss_consistency
    return loss_avg


def inference(dataloader, model, loss_function, weight, device):
    """
    Combined inference function for validation and testing.
    """
    model.eval()
    loss_avg = 0.0
    mov_size = 0
    inference_times = []
    with torch.no_grad():
        for x, y in dataloader:
            x, y = x.to(device), y.to(device)

            # === Forward pass ===
            timer = time.time()
            if NetworkConfig.model_type == "MLP":
                y_pred = model(x)
                inference_times.append(time.time() - timer)
                loss = loss_function(y, y_pred, weight).cpu().numpy()
            elif NetworkConfig.model_type == "VAE":
                y_pred, mu, logvar = model(x)
                inference_times.append(time.time() - timer)
                loss, _, _ = loss_function(y_pred, mu, logvar, y)
                loss = loss.cpu().numpy()

            # === Logging ===
            # Weighted moving average
            batch_size = x.shape[0]
            prev_mov_size = mov_size
            mov_size += batch_size
            loss_avg = (loss_avg * prev_mov_size + loss * batch_size) / mov_size
            # TODO implement some form of accuracy metric
    time_avg = np.mean(inference_times) * 1000  # in ms
    return loss_avg, time_avg


if __name__ == "__main__":
    main(test=False, plot=True, save=True)
