import torch
import casadi as cs

from ml_casadi.torch.modules import TorchMLCasadiModule


class Dropout(TorchMLCasadiModule, torch.nn.Dropout):
    def cs_forward(self, x):
        """
        Apply dropout to the input tensor using CasADi operations.
        """
        return x  # Dropout is deactivated while using eval()
