import torch.nn as nn
from ml_casadi.torch.modules import TorchMLCasadiModule
from ml_casadi.torch.modules.nn import Linear as mcLinear
from ml_casadi.torch.modules.nn import activation as mcActivations
from ml_casadi.torch.modules.nn.batch_norm import BatchNorm1D as mcBatchNorm1d
from ml_casadi.torch.modules.nn.dropout import Dropout as mcDropout


class NaiveMLP(TorchMLCasadiModule):
    def __init__(
        self,
        in_size,
        hidden_sizes,
        out_size,
        activation="relu",
        use_batch_norm=True,
        dropout_p=0.0,
        x_mean=None,
        x_std=None,
        y_mean=None,
        y_std=None,
    ):
        super().__init__()
        assert len(hidden_sizes) >= 1, "There must be at least one hidden layer"

        layers = []
        prev_size = in_size
        for i in range(len(hidden_sizes) + 1):  # +1 to compensate for input layer
            if i < len(hidden_sizes):
                next_size = hidden_sizes[i]
            else:
                next_size = hidden_sizes[-1]  # Repeat last hidden layer size
            # Fully connected layer
            layers.append(mcLinear(prev_size, next_size))
            # Batch normalization
            if use_batch_norm:
                layers.append(mcBatchNorm1d(next_size))
            # Activation function
            if activation == "ReLU":
                layers.append(mcActivations.ReLU())
            elif activation == "Tanh":
                layers.append(mcActivations.Tanh())
            elif activation == "Sigmoid":
                layers.append(mcActivations.Sigmoid())
            elif activation == "LeakyReLU":
                layers.append(mcActivations.LeakyReLU())
            elif activation == "GELU":
                layers.append(mcActivations.GELU())
            elif activation is None:
                pass  # Equal to lambda x: x
            else:
                raise ValueError(f"Unsupported activation function: {activation}")
            # Dropout
            if dropout_p > 0.0:
                layers.append(mcDropout(dropout_p))

            prev_size = next_size

        layers.append(mcLinear(prev_size, out_size))

        self.fully_connected_stack = nn.ModuleList(layers)

        # Input-Output Normalization
        self.register_buffer("x_mean", x_mean)
        self.register_buffer("x_std", x_std)
        self.register_buffer("y_mean", y_mean)
        self.register_buffer("y_std", y_std)

    def forward(self, x):
        # Input normalization
        x = (x - self.x_mean) / self.x_std
        # Forward pass
        for layer in self.fully_connected_stack:
            x = layer(x)
        # Output denormalization
        return (x * self.y_std) + self.y_mean

    def cs_forward(self, x):
        # Input normalization
        x = (x - self.x_mean.cpu().numpy()) / self.x_std.cpu().numpy()
        # Forward pass
        for layer in self.fully_connected_stack:
            x = layer(x)
        # Output denormalization
        return (x * self.y_std.cpu().numpy()) + self.y_mean.cpu().numpy()
