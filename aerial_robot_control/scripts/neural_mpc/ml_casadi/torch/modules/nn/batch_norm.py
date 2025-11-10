import torch
import casadi as cs

from ml_casadi.torch.modules import TorchMLCasadiModule


# NEW!
class BatchNorm1D(TorchMLCasadiModule, torch.nn.BatchNorm1d):
    def cs_forward(self, x):
        """
        Normalize the input as in PyTorch but with casadi operations.
        """
        assert x.shape[1] == 1, "Casadi can not handle batches."

        # Get the parameters as numpy arrays
        running_mean = self.running_mean.detach().cpu().numpy()
        running_var = self.running_var.detach().cpu().numpy()
        weight = self.weight.detach().cpu().numpy()
        bias = self.bias.detach().cpu().numpy()

        # BatchNorm1d normalization formula from PyTorch documentation:
        # output = (input - running_mean) / sqrt(running_var + eps) * weight + bias
        normalized = (x - running_mean) / cs.sqrt(running_var + self.eps)
        output = normalized * weight + bias

        return output
