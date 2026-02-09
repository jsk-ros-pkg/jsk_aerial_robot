import torch
import torch.nn as nn
import casadi as ca
import os, sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from ml_casadi.torch.modules import TorchMLCasadiModule
from ml_casadi.torch.modules.nn import Linear as mcLinear
from ml_casadi.torch.modules.nn import activation as mcActivations
from ml_casadi.torch.modules.nn.batch_norm import BatchNorm1D as mcBatchNorm1d
from ml_casadi.torch.modules.nn.dropout import Dropout as mcDropout


class VAE(TorchMLCasadiModule):
    """
    Variational Autoencoder for residual dynamics modeling in MPC framework.
    
    The VAE learns a probabilistic mapping from input (state + control) to output (dynamics residuals).
    During training, it learns to encode the input into a latent distribution and decode back to the output.
    During inference, it uses the mean of the latent distribution for deterministic predictions.
    """
    
    def __init__(
        self,
        in_size,
        encoder_hidden_sizes,
        latent_dim,
        decoder_hidden_sizes,
        out_size,
        activation="relu",
        use_batch_norm=False,
        dropout_p=0.0,
        x_mean=None,
        x_std=None,
        y_mean=None,
        y_std=None,
    ):
        super().__init__()
        assert len(encoder_hidden_sizes) >= 1, "There must be at least one hidden layer in encoder"
        assert len(decoder_hidden_sizes) >= 1, "There must be at least one hidden layer in decoder"
        
        # =============== Encoder ===============
        encoder_layers = []
        prev_size = in_size
        for i in range(len(encoder_hidden_sizes) + 1):  # +1 to compensate for input layer
            if i < len(encoder_hidden_sizes):
                next_size = encoder_hidden_sizes[i]
            else:
                next_size = encoder_hidden_sizes[-1]  # Repeat last hidden layer size
            # Fully connected layer
            encoder_layers.append(mcLinear(prev_size, next_size))
            # Batch normalization
            if use_batch_norm:
                encoder_layers.append(mcBatchNorm1d(next_size))
            # Activation function
            if activation == "ReLU":
                encoder_layers.append(mcActivations.ReLU())
            elif activation == "Tanh":
                encoder_layers.append(mcActivations.Tanh())
            elif activation == "Sigmoid":
                encoder_layers.append(mcActivations.Sigmoid())
            elif activation == "LeakyReLU":
                encoder_layers.append(mcActivations.LeakyReLU())
            elif activation == "GELU":
                encoder_layers.append(mcActivations.GELU())
            elif activation is None:
                pass  # Equal to lambda x: x
            else:
                raise ValueError(f"Unsupported activation function: {activation}")
            # Dropout
            if dropout_p > 0.0:
                encoder_layers.append(mcDropout(dropout_p))

            prev_size = next_size

        self.encoder = nn.ModuleList(encoder_layers)
        
        # =============== Latent space ===============
        # Simply one fully connected layer with mean and log-variance out sizes
        # Can be interpreted as last layer of encoder
        self.fc_mu = mcLinear(prev_size, latent_dim)
        self.fc_logvar = mcLinear(prev_size, latent_dim)
        
        # =============== Decoder ===============
        decoder_layers = []
        prev_size = latent_dim
        for i in range(len(decoder_hidden_sizes) + 1):  # +1 to compensate for input layer
            if i < len(decoder_hidden_sizes):
                next_size = decoder_hidden_sizes[i]
            else:
                next_size = decoder_hidden_sizes[-1]  # Repeat last hidden layer size
            # Fully connected layer
            decoder_layers.append(mcLinear(prev_size, next_size))
            # Batch normalization
            if use_batch_norm:
                decoder_layers.append(mcBatchNorm1d(next_size))
            # Activation function
            if activation == "ReLU":
                decoder_layers.append(mcActivations.ReLU())
            elif activation == "Tanh":
                decoder_layers.append(mcActivations.Tanh())
            elif activation == "Sigmoid":
                decoder_layers.append(mcActivations.Sigmoid())
            elif activation == "LeakyReLU":
                decoder_layers.append(mcActivations.LeakyReLU())
            elif activation == "GELU":
                decoder_layers.append(mcActivations.GELU())
            elif activation is None:
                pass  # Equal to lambda x: x
            else:
                raise ValueError(f"Unsupported activation function: {activation}")
            # Dropout
            if dropout_p > 0.0:
                decoder_layers.append(mcDropout(dropout_p))

            prev_size = next_size

        decoder_layers.append(mcLinear(prev_size, out_size))

        self.decoder = nn.ModuleList(decoder_layers)
        
        # Input-Output Normalization
        self.register_buffer("x_mean", x_mean)
        self.register_buffer("x_std", x_std)
        self.register_buffer("y_mean", y_mean)
        self.register_buffer("y_std", y_std)
        
        # For ML Casadi library
        self.input_size = in_size
        self.output_size = out_size
    
    def encode(self, x):
        """Encode input to latent distribution parameters."""
        for layer in self.encoder:
            x = layer(x)
        
        # Get mean and log-variance
        mu = self.fc_mu(x)
        logvar = self.fc_logvar(x)
        
        return mu, logvar

    def reparameterize(self, mu, logvar):
        """
        Reparameterization trick: z = mu + std * epsilon
        where epsilon ~ N(0, 1)
        This allows backpropagation through the sampling process by factoring out the randomness.
        """
        std = torch.exp(0.5 * logvar)
        eps = torch.randn_like(std)  # mean 0, variance 1
        return mu + eps * std

    def decode(self, z):
        """Decode latent variable to output."""
        for layer in self.decoder:
            z = layer(z)
        return z

    def forward(self, x):
        """Forward pass through VAE."""
        # Input normalization
        x = (x - self.x_mean) / self.x_std

        # Encoder
        mu, logvar = self.encode(x)
        
        # Latent sampling
        if self.training:  # property from torch.nn.Module, set by model.train() and model.eval()
            # During training, sample from latent distribution
            
            ########################################
            # Detach logvar to be able to use it as uncertainty estimate later
            # logvar = logvar.detach()
            ########################################

            z = self.reparameterize(mu, logvar)
        else:
            # During inference, use mean for deterministic prediction
            z = mu

        # Decoder
        y_pred = self.decode(z)

        # Output denormalization
        y_pred = (y_pred * self.y_std) + self.y_mean
        return y_pred, mu, logvar

    def cs_forward(self, x):
        # Input normalization
        x = (x - self.x_mean.cpu().numpy()) / self.x_std.cpu().numpy()
        
        # Encoder
        mu, logvar = self.encode(x)
        
        # Latent sampling: use mean for deterministic prediction

        # Decoder
        y_pred = self.decode(mu)
        
        # Output denormalization
        y_pred = (y_pred * self.y_std.cpu().numpy()) + self.y_mean.cpu().numpy()

        # Return output and its uncertainty (std)
        std = ca.exp(0.5 * logvar)
        return y_pred, std
