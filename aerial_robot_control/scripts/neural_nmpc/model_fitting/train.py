import os
import subprocess

import numpy as np
import torch
from torch import nn
from torch.utils.data import DataLoader

import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from network_architecture.simple_mlp import SimpleMLP
from network_architecture.normalized_mlp import NormalizedMLP
from utils.data_utils import safe_mkdir_recursive, get_model_dir_and_file, read_dataset
from config.configurations import MLPConfig, ModelFitConfig, SimpleSimConfig, DynModelConfig


def main(model_name: str ="mlp"):
    # Git commit hash for saving the model
    git_version = ''
    try:
        git_version = subprocess.check_output(['git', 'describe', '--always']).strip().decode("utf-8")
    except subprocess.CalledProcessError as e:
        print(e.returncode, e.output)
    print("The model will be saved using hash: %s" % git_version)

    ds_name = ModelFitConfig.ds_name
    state_dim = ModelFitConfig.ds_state_dim
    input_dim = ModelFitConfig.ds_input_dim
    sim_options = ModelFitConfig.ds_disturbances

    save_file_path, save_file_name = get_model_dir_and_file(git_version, model_name, state_dim, sim_options)
    safe_mkdir_recursive(save_file_path)
    if os.path.exists(save_file_name):
        print(f"File {save_file_name} already exists.")
        return

    # === Load data ===
    train_dataset = read_dataset(ds_name, "train", state_dim, input_dim, sim_options)
    val_dataset = read_dataset(ds_name, "val", state_dim, input_dim, sim_options)
    test_dataset = read_dataset(ds_name, "test", state_dim, input_dim, sim_options)

    # Load training and test data
    # TODO create own dataset class
    # gp_dataset_train = GPDataset(df_train, x_features, u_features, reg_y_dims,
    #                                 cap=x_cap, n_bins=hist_bins, thresh=hist_thresh)
    # gp_dataset_val = GPDataset(df_val, x_features, u_features, reg_y_dims,
    #                             cap=x_cap, n_bins=hist_bins, thresh=hist_thresh)

    # Data loaders
    train_dataloader, val_dataloader, test_dataloader = \
        get_dataloaders(train_dataset, val_dataset, test_dataset, batch_size=MLPConfig.batch_size)

    device = get_device()
    model = SimpleMLP().to(device)

    loss_fn = get_loss_function()

    optimizer = get_optimizer(model)

    train(train_dataloader, model, loss_fn, optimizer, device)

    print(model)

    epochs = 5
    for t in range(epochs):
        print(f"Epoch {t+1}\n-------------------------------")
        train(train_dataloader, model, loss_fn, optimizer, device)
        test(test_dataloader, model, loss_fn, device)
    print("Done!")

    # === Save the model ===
    save_dict = {
        'state_dict': model.state_dict(),
        'input_size': state_dim + input_dim,
        'hidden_size': MLPConfig.hidden_neurons,
        'output_size': state_dim,   # TODO maybe reduce number of predicted states
        'hidden_layers': MLPConfig.hidden_layers
    }
    torch.save(save_dict, os.path.join(save_file_path, f'{save_file_name}.pt'))
    print("Saved PyTorch Model!")

    # # === Load the model ===
    # model = SimpleMLP().to(device)
    # model.load_state_dict(torch.load("model.pth", weights_only=True))
    

def get_dataloaders(training_data, val_data, test_data, batch_size=64):
    """
    Load training and test data into DataLoader objects.
    """
    train_dataloader = DataLoader(training_data, batch_size=batch_size, shuffle=True)
    val_dataloader = DataLoader(val_data, batch_size=batch_size, shuffle=True)
    test_dataloader = DataLoader(test_data, batch_size=batch_size, shuffle=True)

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

def get_loss_function():
    return nn.CrossEntropyLoss()

def get_optimizer(model, learning_rate=1e-3):
    return torch.optim.SGD(model.parameters(), lr=learning_rate)

def train(dataloader, model, loss_fn, optimizer, device):
    size = len(dataloader.dataset)
    model.train()
    for batch_idx, (X, y) in enumerate(dataloader):
        X, y = X.to(device), y.to(device)

        # Compute prediction error
        pred = model(X)
        loss = loss_fn(pred, y)

        # Backpropagation
        loss.backward()
        optimizer.step()
        optimizer.zero_grad()

        if batch_idx % 100 == 0:
            loss, current = loss.item(), (batch_idx + 1) * len(X)
            print(f"loss: {loss:>7f}  [{current:>5d}/{size:>5d}]")


def test(dataloader, model, loss_fn, device):
    size = len(dataloader.dataset)
    num_batches = len(dataloader)
    model.eval()
    test_loss, correct = 0, 0
    with torch.no_grad():
        for X, y in dataloader:
            X, y = X.to(device), y.to(device)
            pred = model(X)
            test_loss += loss_fn(pred, y).item()
            correct += (pred.argmax(1) == y).type(torch.float).sum().item()
    test_loss /= num_batches
    correct /= size
    print(f"Test Error: \n Accuracy: {(100*correct):>0.1f}%, Avg loss: {test_loss:>8f} \n")


if __name__ == '__main__':
    model_name = "simple_mlp"  # or "normalized_mlp"
    main(model_name)
