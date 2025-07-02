import os
import time
import numpy as np
import torch
from torch.utils.data import DataLoader, random_split
from progress_table import ProgressTable
from torchsummary import summary

import ml_casadi.torch as mc    # Propietary library for approximated MLP [https://ieeexplore.ieee.org/document/10049101/]

import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from dataset import TrajectoryDataset
from network_architecture.naive_mlp import NaiveMLP
from network_architecture.normalized_mlp import NormalizedMLP
from utils.data_utils import sanity_check_dataset, get_model_dir_and_file, read_dataset
from utils.visualization_utils import plot_losses
from config.configurations import MLPConfig, ModelFitConfig, SimpleSimConfig


def main(network_name: str ="mlp", approximated_mlp: bool = False, test: bool = False, plot: bool = False, save: bool = True):
    device = get_device()

    ds_name = ModelFitConfig.ds_name
    ds_instance = ModelFitConfig.ds_instance
    sanity_check_dataset(ds_name, ds_instance)
    if save or plot: save_file_path, save_file_name = get_model_dir_and_file(network_name)  # Checks for settings in ModelFitConfig

    # === Raw data ===
    df = read_dataset(ds_name, ds_instance)

    # import pandas as pd
    # df = pd.read_csv("/home/johannes/ros/neural-mpc/ros_dd_mpc/data/simplified_sim_dataset/train/dataset_001.csv")

    # === Datasets ===
    # TODO prune wrt angular velocity as well
    dataset = TrajectoryDataset(df,
                                histogram_pruning_n_bins=MLPConfig.histogram_n_bins,
                                histogram_pruning_thresh=MLPConfig.histogram_thresh,
                                vel_cap=MLPConfig.vel_cap,
                                plot=False)
    in_dim = dataset.x.shape[1]
    out_dim = dataset.y.shape[1]

    train_size = int(0.8 * len(dataset))
    val_size = int(0.1 * len(dataset))
    test_size = len(dataset) - train_size - val_size

    train_dataset, val_dataset, test_dataset = \
        random_split(dataset, [train_size, val_size, test_size])

    # === Dataloaders ===
    train_dataloader, val_dataloader, test_dataloader = \
        get_dataloaders(train_dataset, val_dataset, test_dataset,
                        batch_size=MLPConfig.batch_size,
                        num_workers=MLPConfig.num_workers)

    # === Model ===
    if approximated_mlp:
        # TODO implement batch normalization and dropout?
        # TODO combine and call it "ApproximatedMLP"
        mlp = mc.nn.MultiLayerPerceptron(in_dim, MLPConfig.hidden_sizes[0],
                                         out_dim, len(MLPConfig.hidden_sizes),
                                         activation=MLPConfig.activation).to(device)
        model = NormalizedMLP(mlp,
                              torch.tensor(dataset.x_mean), torch.tensor(dataset.x_std),
                              torch.tensor(dataset.y_mean), torch.tensor(dataset.y_std)).to(device)
    else:
        model = NaiveMLP(in_dim, MLPConfig.hidden_sizes, out_dim,
                         activation=MLPConfig.activation,
                         use_batch_norm=MLPConfig.use_batch_norm,
                         dropout_p=MLPConfig.dropout_p,
                         x_mean=torch.tensor(dataset.x_mean), x_std=torch.tensor(dataset.x_std),
                         y_mean=torch.tensor(dataset.y_mean), y_std=torch.tensor(dataset.y_std)).to(device)
    print(model)
    summary(model, (in_dim,))

    # === Loss function ===
    loss_fn = loss_function

    # === Optimizer ===
    optimizer = get_optimizer(model, MLPConfig.learning_rate)

    # === Training Loop ===
    print("==========================================")
    print("Starting training...")
    total_losses = {"train": [], "val": []}
    inference_times = []
    table = ProgressTable(
        pbar_embedded=False,
        # pbar_style_embed="rich",
        pbar_style="rich",
        # pbar_style="angled alt red blue",
        num_decimal_places=6
    )

    table.add_column("Epoch", color="white", width=13)
    table.add_column("Step", color="DIM", width=13)
    table.add_column("Train Loss", color="red", width=25)
    table.add_column("Val Loss", color="BRIGHT green", width=25)
    table.add_column("Inference Time", width=19)
    if test:
        table.add_column("Test Loss", color="bold green", column_width=25)

    for t in table(MLPConfig.num_epochs, show_throughput=False, show_eta=True):
        table["Epoch"] = f"{t+1}/{MLPConfig.num_epochs}"

        # === Training ===
        train_losses = train(train_dataloader, model, loss_fn, optimizer, device, table)
        total_losses["train"].append(train_losses)

        # === Validation ===
        val_losses, inference_time = inference(val_dataloader, model, loss_fn, device, table)
        total_losses["val"].append(val_losses)
        inference_times.append(inference_time)
        table.next_row()

        # === Save model ===
        if save:
            save_dict = {
                'state_dict': model.state_dict(),
                'input_size': in_dim,
                'hidden_sizes': MLPConfig.hidden_sizes,
                'output_size': out_dim
            }
            torch.save(save_dict, os.path.join(save_file_path, f'{save_file_name}.pt'))
    table.close()
    print("Training Finished!")

    # === Testing ===
    if test:
        test_losses, _ = inference(test_dataloader, model, loss_fn, device, validation=False)
        total_losses["test"] = test_losses
        print(f"Test avg loss: {test_losses:>8f}")

    # === Plotting ===
    if plot:
        plot_losses(total_losses, inference_times, save_file_path, save_file_name)
        halt = 1


def get_dataloaders(training_data, val_data, test_data, batch_size=64, num_workers=0):
    train_dataloader = DataLoader(training_data, batch_size=batch_size, shuffle=True, num_workers=num_workers)
    val_dataloader = DataLoader(val_data, batch_size=4096, shuffle=True, num_workers=num_workers)
    test_dataloader = DataLoader(test_data, batch_size=4096, shuffle=True, num_workers=num_workers)
    return train_dataloader, val_dataloader, test_dataloader

def get_device():
    device = torch.accelerator.current_accelerator().type if torch.accelerator.is_available() else "cpu"
    print(f"Using {device} device")
    if torch.cuda.is_available():
        return torch.device("cuda")
    elif torch.backends.mps.is_available():
        return torch.device("mps")
    else:
        return torch.device("cpu")

def loss_function(y, y_pred):
    return torch.square(y - y_pred).mean()

def get_optimizer(model, learning_rate):
    optimizer_class = getattr(torch.optim, MLPConfig.optimizer)
    return optimizer_class(model.parameters(), lr=learning_rate)

def train(dataloader, model, loss_fn, optimizer, device, table):
    size = len(dataloader.dataset)
    model.train()
    loss_avg= 0.0
    mov_size = 0
    for x, y in table(dataloader, total=MLPConfig.num_epochs, description="Epoch"):
        x, y = x.to(device), y.to(device)
        
        # === Forward pass ===
        y_pred = model(x)

        # === Loss ===
        loss = loss_fn(y, y_pred)

        # === Backpropagation ===
        loss.backward()
        optimizer.step()
        optimizer.zero_grad()

        # === Logging ===
        # Weighted moving average
        curr_batch_size = x.shape[0]
        prev_mov_size = mov_size
        mov_size += curr_batch_size
        loss_avg = (loss_avg * prev_mov_size + loss.item() * curr_batch_size) / mov_size
        table["Step"] = f"{mov_size}/{size}"
        table["Train Loss"] = loss_avg
    return loss_avg


def inference(dataloader, model, loss_fn, device, table, validation=True):
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
            loss = loss_fn(y, y_pred).cpu().numpy()

            # === Logging ===
            # Weighted moving average
            batch_size = x.shape[0]
            prev_mov_size = mov_size
            mov_size += batch_size
            loss_avg = (loss_avg*prev_mov_size + loss * batch_size) / mov_size
            if validation:
                table["Val Loss"] = loss_avg
            else:
                table["Test Loss"] = loss_avg
            # TODO implement some form of accuracy metric
    time_avg = np.mean(inference_times) * 1000 # in ms
    table["Inference Time"] = f"{time_avg:.2f} ms"
    return loss_avg, time_avg


if __name__ == '__main__':
    model_name = "simple_mlp"  # or "normalized_mlp"
    main(model_name, approximated_mlp=True, test=False, plot=True, save=False)
