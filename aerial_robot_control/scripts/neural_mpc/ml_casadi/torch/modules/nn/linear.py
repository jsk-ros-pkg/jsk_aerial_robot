import torch
import casadi as cs

from ml_casadi.torch.modules import TorchMLCasadiModule


class Linear(TorchMLCasadiModule, torch.nn.Linear):
    def cs_forward(self, x):
        assert x.shape[1] == 1, "Casadi can not handle batches."
        y = cs.mtimes(self.weight.detach().cpu().numpy(), x)
        if self.bias is not None:
            y = y + self.bias.detach().cpu().numpy()
        return y
