import torch
import torch.nn as nn
from ml_casadi.torch.modules import TorchMLCasadiModule
from ml_casadi.torch.modules.nn import Linear as mcLinear
from ml_casadi.torch.modules.nn import activation as mcActivations
from ml_casadi.torch.modules.nn.batch_norm import BatchNorm1D as mcBatchNorm1d
from ml_casadi.torch.modules.nn.dropout import Dropout as mcDropout


class MLP(TorchMLCasadiModule):
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

        # For ML Casadi library
        self.input_size = in_size
        self.output_size = out_size

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

    def compute_jacobian(self, x):
        """
        Compute the Jacobian of the network output with respect to the input at point x.
        
        Args:
            x: Input tensor of shape (batch_size, input_size) or (input_size,)
            
        Returns:
            Jacobian matrix of shape (batch_size, output_size, input_size) or (output_size, input_size)
        """
        x = x.clone().detach().requires_grad_(True)
        y = self.forward(x)
        
        # Handle both single sample and batch
        is_single = x.dim() == 1
        if is_single:
            x = x.unsqueeze(0)
            y = y.unsqueeze(0)
        
        batch_size = x.shape[0]
        jacobian = torch.zeros(batch_size, self.output_size, self.input_size, device=x.device)
        
        for i in range(self.output_size):
            # Compute gradient of output i with respect to input
            grad_outputs = torch.zeros_like(y)
            grad_outputs[:, i] = 1.0
            grads = torch.autograd.grad(
                outputs=y,
                inputs=x,
                grad_outputs=grad_outputs,
                create_graph=False,
                retain_graph=True
            )[0]
            jacobian[:, i, :] = grads
        
        if is_single:
            jacobian = jacobian.squeeze(0)
            
        return jacobian

    def linearize_at_point(self, x0):
        """
        Linearize the network around the working point x0.
        Returns the linear approximation: y ≈ y0 + J @ (x - x0)
        
        Args:
            x0: Working point tensor of shape (input_size,) or (batch_size, input_size)
            
        Returns:
            dict with:
                - 'y0': Output at the working point
                - 'J': Jacobian matrix at the working point
                - 'x0': The working point (for reference)
        """
        with torch.no_grad():
            y0 = self.forward(x0)
        
        J = self.compute_jacobian(x0)
        
        return {
            'y0': y0,
            'J': J,
            'x0': x0.clone()
        }
    
    def evaluate_linearized(self, x, linearization):
        """
        Evaluate the linearized model at point x.
        
        Args:
            x: Input point(s) where to evaluate
            linearization: Dict returned by linearize_at_point()
            
        Returns:
            Linearized output: y ≈ y0 + J @ (x - x0)
        """
        y0 = linearization['y0']
        J = linearization['J']
        x0 = linearization['x0']
        
        dx = x - x0
        
        # Handle both single sample and batch
        if dx.dim() == 1:
            dy = J @ dx
        else:
            # Batch matrix multiplication
            dy = torch.einsum('bij,bj->bi', J, dx)
        
        return y0 + dy
