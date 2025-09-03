#!/usr/bin/env python3

import torch
import casadi as cs
import numpy as np

import os, sys

sys.path.append(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))))
)
from batch_norm import BatchNorm1D


def test_batch_norm():
    """Test the BatchNorm1D implementation"""

    print("Starting BatchNorm1D test...")

    # Create a BatchNorm1D layer with 3 features
    num_features = 3
    batch_norm = BatchNorm1D(num_features)

    # Set the model to evaluation mode (uses running statistics)
    batch_norm.eval()

    print("Created BatchNorm1D with", num_features, "features")

    # Create some test input data
    # For BatchNorm1d, input shape should be (batch_size, num_features) or (batch_size, num_features, length)
    test_input = torch.randn(64, 3)  # batch size 64, 3 features

    print("Test input shape:", test_input.shape)
    print("Test input:", test_input)

    # Forward pass with PyTorch
    with torch.no_grad():
        pytorch_output = batch_norm(test_input)

    print("PyTorch forward pass completed")
    pytorch_np = pytorch_output.detach().cpu().numpy()

    # Convert input to CasADi format (transpose to match expected shape)
    casadi_output = np.zeros((test_input.shape[0], num_features))
    for b in range(test_input.shape[0]):
        casadi_input = cs.DM(test_input[b, :].detach().cpu().numpy().T)
        # Forward pass with CasADi
        casadi_output[b, :] = np.array(batch_norm.cs_forward(casadi_input).T)

    print("CasADi forward pass completed")

    # Compare outputs
    print("PyTorch output:", pytorch_np.flatten())
    print("CasADi output:", casadi_output.flatten())
    print("Difference:", np.abs(pytorch_np - casadi_output).max())

    # Check if they're close enough
    if np.allclose(pytorch_np, casadi_output, rtol=1e-5):
        print("✓ Test passed: PyTorch and CasADi outputs match!")
        return True
    else:
        print("✗ Test failed: Outputs don't match")
        return False


if __name__ == "__main__":
    test_batch_norm()
