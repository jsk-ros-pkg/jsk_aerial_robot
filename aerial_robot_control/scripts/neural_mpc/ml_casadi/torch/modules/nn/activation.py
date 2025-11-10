import torch
import casadi as cs

from ml_casadi.torch.modules import TorchMLCasadiModule


class Sigmoid(TorchMLCasadiModule, torch.nn.Sigmoid):
    def cs_forward(self, x):
        y = 1 / (1 + cs.exp(-x))
        return y


class Tanh(TorchMLCasadiModule, torch.nn.Tanh):
    def cs_forward(self, x):
        return cs.tanh(x)


class ReLU(TorchMLCasadiModule, torch.nn.ReLU):
    def cs_forward(self, x):
        return cs.if_else(x < 0.0, 0.0 * x, x)


class LeakyReLU(TorchMLCasadiModule, torch.nn.LeakyReLU):
    def cs_forward(self, x):
        return cs.if_else(x < 0.0, self.negative_slope * x, x)


class GELU(TorchMLCasadiModule, torch.nn.GELU):
    def cs_forward(self, x):
        return 0.5 * x * (1 + cs.tanh(cs.sqrt(2 / cs.pi) * (x + 0.044715 * cs.power(x, 3))))
