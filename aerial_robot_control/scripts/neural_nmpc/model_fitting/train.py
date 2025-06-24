import os
import subprocess

import numpy as np
import torch
from torch import nn
from torch.utils.data import DataLoader, random_split

import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from dataset import TrajectoryDataset
from network_architecture.simple_mlp import SimpleMLP
from network_architecture.normalized_mlp import NormalizedMLP
from utils.data_utils import sanity_check_dataset, get_model_dir_and_file, read_dataset
from utils.visualization_utils import plot_losses
from config.configurations import MLPConfig, ModelFitConfig, SimpleSimConfig


def main(network_name: str ="mlp", test: bool = False, plot: bool = False):
    device = get_device()

    ds_name = ModelFitConfig.ds_name
    ds_instance = ModelFitConfig.ds_instance
    sanity_check_dataset(ds_name, ds_instance)
    save_file_path, save_file_name, model_options = get_model_dir_and_file(ds_name, ds_instance, network_name)
    
    # # Retrieve metadata
    # nmpc_type = model_options['nmpc_type']
    # state_dim, control_dim = model_options['state_dim'], model_options['control_dim']

    # === Raw data ===
    df = read_dataset(ds_name, ds_instance)
    
    # === Datasets ===
    dataset = TrajectoryDataset(df)
    in_dim = dataset.inputs.shape[1]
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
    model = SimpleMLP(in_dim, out_dim).to(device)
    print(model)

    # === Loss function ===
    loss_fn = loss_function

    # === Optimizer ===2
    optimizer = get_optimizer(model, MLPConfig.learning_rate)

    # === Training Loop ===
    print("==========================================")
    print("Starting training...")
    total_losses = {"train": [], "val": []}
    for t in range(MLPConfig.num_epochs):
        print(f"Epoch {t+1}\n--------------------------------------------------------------")

        # === Training ===
        train_losses = train(train_dataloader, model, loss_fn, optimizer, device)
        total_losses["train"].append(train_losses)

        # === Validation ===
        val_losses = inference(val_dataloader, model, loss_fn, device)
        total_losses["val"].append(val_losses)
        print(f"----> Validation avg loss: {val_losses:>8f}")

        # === Save model ===
        save_dict = {
            'state_dict': model.state_dict(),
            'input_size': in_dim,
            'hidden_size': MLPConfig.hidden_neurons,
            'output_size': out_dim,
            'hidden_layers': MLPConfig.hidden_layers
        }
        torch.save(save_dict, os.path.join(save_file_path, f'{save_file_name}.pt'))
        print("Saved PyTorch Model!")
        print("______________________________________________________________________________")
    print("Training Finished!")

    # === Testing ===
    if test:
        test_losses = inference(test_dataloader, model, loss_fn, device)
        total_losses["test"] = test_losses
        print(f"Test avg loss: {test_losses:>8f}")

    # === Plotting ===
    if plot:
        plot_losses(total_losses)
    

def get_dataloaders(training_data, val_data, test_data, batch_size=64, num_workers=0):
    """
    Load training and test data into DataLoader objects.
    """
    # TODO understand why batch_size != 1 for val, test (from torch tutorial)
    train_dataloader = DataLoader(training_data, batch_size=batch_size, shuffle=True, num_workers=num_workers)
    val_dataloader = DataLoader(val_data, batch_size=batch_size, shuffle=True, num_workers=num_workers)
    test_dataloader = DataLoader(test_data, batch_size=batch_size, shuffle=True, num_workers=num_workers)

    return train_dataloader, val_dataloader, test_dataloader

def get_device():
    """
    Get the current device for PyTorch operations.
    """
    device = torch.accelerator.current_accelerator().type if torch.accelerator.is_available() else "cpu"
    print(f"Using {device} device")

    if torch.cuda.is_available():
        return torch.device("cuda")
    elif torch.backends.mps.is_available():
        return torch.device("mps")
    else:
        return torch.device("cpu")

def loss_function(y, y_pred, run_training=True):
    if run_training: 
        return torch.square(y - y_pred).mean()
    else:
        return torch.square(y - y_pred)

def get_optimizer(model, learning_rate=1e-4):
    return torch.optim.Adam(model.parameters(), lr=learning_rate)

def train(dataloader, model, loss_fn, optimizer, device):
    size = len(dataloader.dataset)
    losses = []
    model.train()
    for batch_idx, (x, y) in enumerate(dataloader):
        x, y = x.to(device), y.to(device)
        
        # === Forward pass ===
        y_pred = model(x)

        # === Loss ===
        loss = loss_fn(y, y_pred)

        # === Backpropagation ===
        loss.backward()
        optimizer.step()
        optimizer.zero_grad()

        losses.append(loss.item())

        if batch_idx % 10 == 0:
            loss = np.mean(losses)
            current = (batch_idx + 1) * len(x)
            print(f"Train avg loss: {loss:>7f}  [{current:>5d}/{size:>5d}]\n")
    loss = np.mean(losses)
    print(f"Train avg loss: {loss:>7f}  [{size:>5d}/{size:>5d}]\n")
    return loss


def inference(dataloader, model, loss_fn, device):
    losses = []
    model.eval()
    with torch.no_grad():
        for x, y in dataloader:
            x, y = x.to(device), y.to(device)

            # === Forward pass ===
            y_pred = model(x)

            # === Loss ===
            loss = loss_fn(y, y_pred, run_training=False).cpu().numpy()
            losses.append(loss)

            # === Accuracy ===
            # TODO implement some form of accuracy metric
            # correct += (y_pred.argmax(1) == y).type(torch.float).sum().item()
    loss_avg = np.mean(np.vstack(losses))
    return loss_avg


if __name__ == '__main__':
    model_name = "simple_mlp"  # or "normalized_mlp"
    main(model_name, test=True, plot=True)
