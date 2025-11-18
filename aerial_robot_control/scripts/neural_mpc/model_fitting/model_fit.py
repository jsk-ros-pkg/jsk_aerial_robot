import os, sys
import time
import numpy as np
import torch
from torch.utils.data import DataLoader, random_split
from torch.nn import MSELoss
from progress_table import ProgressTable
from torchsummary import summary
from decimal import Decimal

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from dataset import TrajectoryDataset
from network_architecture.mlp import MLP
from utils.data_utils import sanity_check_dataset, get_model_dir_and_file, read_dataset, log_metrics
from utils.model_utils import sanity_check_features_and_reg_dims, get_device
from utils.visualization_utils import plot_fitting
from config.configurations import MLPConfig, ModelFitConfig


def main(test: bool = False, plot: bool = False, save: bool = True):
    device = get_device()

    ds_name = ModelFitConfig.ds_name
    ds_instance = ModelFitConfig.ds_instance
    sanity_check_dataset(ds_name, ds_instance)

    # Define model input and output features
    state_feats = ModelFitConfig.state_feats
    u_feats = ModelFitConfig.u_feats
    y_reg_dims = ModelFitConfig.y_reg_dims

    # Set save path and populate metadata
    if save or ModelFitConfig.save_plots:
        save_file_path, save_file_name = get_model_dir_and_file(ds_name, ds_instance, MLPConfig.model_name)

    # Raw data
    df = read_dataset(ds_name, ds_instance)

    # === Datasets ===
    # TODO prune w.r.t. angular velocity as well
    print("Loading dataset...")
    dataset = TrajectoryDataset(
        df,
        state_feats,
        u_feats,
        y_reg_dims,
        save_file_path=save_file_path,
        save_file_name=save_file_name,
    )
    print("Finished loading the dataset!")
    in_dim = dataset.x.shape[1]
    out_dim = dataset.y.shape[1]
    sanity_check_features_and_reg_dims(
        MLPConfig.model_name, state_feats, u_feats, y_reg_dims, in_dim, out_dim, MLPConfig.delay_horizon
    )

    train_size = int(0.8 * len(dataset))
    if test:
        val_size = int(0.1 * len(dataset))
        test_size = len(dataset) - train_size - val_size
    else:
        val_size = len(dataset) - train_size
        test_size = 0

    train_dataset, val_dataset, test_dataset = random_split(dataset, [train_size, val_size, test_size])

    # === Dataloaders ===
    train_dataloader, val_dataloader, test_dataloader = get_dataloaders(
        train_dataset, val_dataset, test_dataset, batch_size=MLPConfig.batch_size, num_workers=MLPConfig.num_workers
    )

    # === Neural Network ===
    model = MLP(
        in_dim,
        MLPConfig.hidden_sizes,
        out_dim,
        activation=MLPConfig.activation,
        use_batch_norm=MLPConfig.use_batch_norm,
        dropout_p=MLPConfig.dropout_p,
        x_mean=torch.tensor(dataset.x_mean),
        x_std=torch.tensor(dataset.x_std),
        y_mean=torch.tensor(dataset.y_mean),
        y_std=torch.tensor(dataset.y_std),
    ).to(device)
    print(model)
    summary(model, (in_dim,))

    # === Loss function ===
    if MLPConfig.l1_lambda > 0.0:
        loss_fn = loss_function_l1
    else:
        loss_fn = loss_function
    if len(MLPConfig.loss_weight) != out_dim:
        raise ValueError("Loss weight doesn't match output dimension!")
    weight = torch.tensor(MLPConfig.loss_weight).to(device)
    l1_lambda = torch.tensor(MLPConfig.l1_lambda).to(device)

    # === Optimizer ===
    optimizer = get_optimizer(model, MLPConfig.learning_rate)
    if MLPConfig.lr_scheduler is not None:
        if MLPConfig.lr_scheduler == "ReduceLROnPlateau":
            lr_scheduler = torch.optim.lr_scheduler.ReduceLROnPlateau(
                optimizer, factor=0.1, threshold=0.005, patience=20
            )
        elif MLPConfig.lr_scheduler == "LambdaLR":
            # Divide here since lambda func returns a multiplier for the base lr
            lr_func = lambda epoch: max(0.95**epoch, 1e-5 / 1e-3)
            lr_scheduler = torch.optim.lr_scheduler.LambdaLR(optimizer, lr_lambda=lr_func)
        elif MLPConfig.lr_scheduler == "LRScheduler":
            lr_scheduler = torch.optim.lr_scheduler.LRScheduler(optimizer)
        else:
            raise ValueError(f"Unsupported learning rate scheduler: {MLPConfig.lr_scheduler}")
    learning_rates = []

    # === Logging ===
    total_losses = {"train": [], "val": []}
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
    if MLPConfig.gradient_lambda > 0.0:
        table.add_column("Gradient", color="magenta", width=15)
    if MLPConfig.consistency_epsilon > 0.0:
        table.add_column("Consistency", color="cyan", width=15)
    if MLPConfig.symmetry_lambda > 0.0:
        table.add_column("Symmetry", color="yellow", width=15)
    table.add_column("Val Loss", color="BRIGHT green", width=19)
    table.add_column("Inference Time", width=17)
    table.add_column("LR", width=7)

    # === Training Loop ===
    print("==========================================")
    print(f"Starting training model {save_file_name}...")
    for t in table(MLPConfig.num_epochs, show_throughput=False, show_eta=True):
        table["Epoch"] = f"{t+1}/{MLPConfig.num_epochs}"

        # === Train Step ===
        train_losses = train(train_dataloader, model, loss_fn, weight, l1_lambda, optimizer, device, table)
        total_losses["train"].append(train_losses)

        # === Validation ===
        val_losses, inference_time = inference(val_dataloader, model, loss_fn, weight, l1_lambda, device)
        table["Val Loss"] = val_losses
        table["Inference Time"] = f"{inference_time:.2f} ms"
        total_losses["val"].append(val_losses)
        inference_times.append(inference_time)

        # === Schedule learning rate ===
        if MLPConfig.lr_scheduler == "ReduceLROnPlateau":
            # lr_scheduler.step(train_losses)
            lr_scheduler.step(val_losses)
        elif MLPConfig.lr_scheduler in ["LambdaLR", "LRScheduler"]:
            lr_scheduler.step()
        learning_rates.append(optimizer.param_groups[0]["lr"])
        table["Learning Rate"] = f"{Decimal(learning_rates[-1]):.0e}"
        table.next_row()

        # === Save model ===
        if save:
            save_dict = {
                "state_dict": model.state_dict(),
                "input_size": in_dim,
                "hidden_sizes": MLPConfig.hidden_sizes,
                "output_size": out_dim,
                "activation": MLPConfig.activation,
                "use_batch_norm": MLPConfig.use_batch_norm,
                "dropout_p": MLPConfig.dropout_p,
            }
            torch.save(save_dict, os.path.join(save_file_path, f"{save_file_name}.pt"))
    table.close()
    print("Training Finished!")

    # === Testing ===
    if test:
        test_losses, _ = inference(test_dataloader, model, loss_fn, weight, l1_lambda, device)
        total_losses["test"] = test_losses
        print(f"Test avg loss: {test_losses:>8f}")

    # === Store metrics ===
    log_metrics(total_losses, inference_times, learning_rates, save_file_path, save_file_name)
    if save:
        print(f"Model saved to results/{MLPConfig.model_name}/{save_file_name}.pt")

    # === Plotting ===
    if plot:
        plot_fitting(total_losses, inference_times, learning_rates, save_file_path, save_file_name)
        halt = 1


def get_dataloaders(training_data, val_data, test_data, batch_size=64, num_workers=0):
    train_dataloader = DataLoader(training_data, batch_size=batch_size, shuffle=True, num_workers=num_workers)
    val_dataloader = DataLoader(val_data, batch_size=4096, shuffle=True, num_workers=num_workers)
    if len(test_data) == 0:
        test_dataloader = None
    else:
        test_dataloader = DataLoader(test_data, batch_size=4096, shuffle=True, num_workers=num_workers)
    return train_dataloader, val_dataloader, test_dataloader


def loss_function(y, y_pred, weight, *_):
    # Weight each dimension
    return torch.mean(torch.square(y - y_pred) * weight)


def loss_function_l1(y, y_pred, weight, l1_lambda, model):
    # Weight each dimension
    ls_loss = torch.mean(torch.square(y - y_pred) * weight)
    # L1 regularization
    l1_loss = l1_lambda * sum(p.abs().sum() for p in model.parameters())
    return ls_loss + l1_loss


def get_optimizer(model, learning_rate):
    optimizer_class = getattr(torch.optim, MLPConfig.optimizer)
    return optimizer_class(model.parameters(), lr=learning_rate, weight_decay=MLPConfig.weight_decay)


def train(dataloader, model, loss_fn, weight, l1_lambda, optimizer, device, table):
    size = len(dataloader.dataset)
    model.train()
    loss_avg = 0.0
    mov_size = 0
    for x, y in dataloader:
        x, y = x.to(device), y.to(device)
        optimizer.zero_grad()
        if MLPConfig.gradient_lambda > 0.0:
            x.requires_grad = True

        # === Forward pass ===
        y_pred = model(x)

        # === Gradient penalty ===
        if MLPConfig.gradient_lambda > 0.0:
            gradients = torch.autograd.grad(
                outputs=y_pred, inputs=x, grad_outputs=torch.ones_like(y_pred), create_graph=True
            )[0]
            gradient_penalty = torch.mean(torch.square(gradients))
            loss_gradient = MLPConfig.gradient_lambda * gradient_penalty

        # === Output consistency ===
        if MLPConfig.consistency_lambda > 0.0:
            # Add relative noise based on input magnitude with standard normal distribution
            noise = torch.randn_like(x) * torch.abs(x)
            noisy_input = x + MLPConfig.consistency_epsilon * model.x_std * noise
            y_pred_noise = model(noisy_input)
            loss_consistency = MLPConfig.consistency_lambda * torch.mean(torch.square(y_pred - y_pred_noise) * weight)

        # === Output symmetry ===
        if MLPConfig.symmetry_lambda > 0.0:
            # Permutate inputs and outputs to enforce symmetry
            if x.shape[1] == 9:
                x_permuted = x.clone()
                # Swap thrusts (same arms)
                # x_permuted[:, 1], x_permuted[:, 2] = x[:, 2], x[:, 1]
                # x_permuted[:, 3], x_permuted[:, 4] = x[:, 4], x[:, 3]
                # Swap thrusts (opposite arms)
                x_permuted[:, 1], x_permuted[:, 3] = x[:, 3], x[:, 1]
                x_permuted[:, 2], x_permuted[:, 4] = x[:, 4], x[:, 2]
                # Swap servo angles (same arms)
                x_permuted[:, 5], x_permuted[:, 6] = x[:, 6], x[:, 5]
                x_permuted[:, 7], x_permuted[:, 8] = x[:, 8], x[:, 7]
                # Swap servo angles (opposite arms)
                # x_permuted[:, 5], x_permuted[:, 7] = x[:, 7], x[:, 5]
                # x_permuted[:, 6], x_permuted[:, 8] = x[:, 8], x[:, 6]
                # Permutated forward pass
                y_pred_permuted = model(x_permuted)
                # Permute output accordingly (swap accelerations in x & y)
                y_pred_permuted[:, 0], y_pred_permuted[:, 1] = y_pred_permuted[:, 1], y_pred_permuted[:, 0]
            else:
                raise NotImplementedError("Symmetry loss is only implemented for input dimension 9.")

            # Symmetry loss
            loss_symmetry = MLPConfig.symmetry_lambda * torch.mean(torch.square(y_pred - y_pred_permuted) * weight)

        # === Loss ===
        loss = loss_fn(y, y_pred, weight, l1_lambda, model)
        if MLPConfig.gradient_lambda > 0.0:
            loss += loss_gradient
        if MLPConfig.consistency_lambda > 0.0:
            loss += loss_consistency
        if MLPConfig.symmetry_lambda > 0.0:
            loss += loss_symmetry

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
        if MLPConfig.gradient_lambda > 0.0:
            table["Gradient"] = loss_gradient
        if MLPConfig.consistency_epsilon > 0.0:
            table["Consistency"] = loss_consistency
        if MLPConfig.symmetry_lambda > 0.0:
            table["Symmetry"] = loss_symmetry
    return loss_avg


def inference(dataloader, model, loss_fn, weight, l1_lambda, device):
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
            y_pred = model(x)
            inference_times.append(time.time() - timer)

            # === Loss ===
            loss = loss_fn(y, y_pred, weight, l1_lambda, model).cpu().numpy()

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
