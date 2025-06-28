import torch.nn as nn

class NaiveMLP(nn.Module):
    def __init__(self, in_size, hidden_sizes, out_size, activation='relu', dropout_p=0.0,
                 x_mean=None, x_std=None, y_mean=None, y_std=None):

        super().__init__()
        assert len(hidden_sizes) >= 1, "There must be at least one hidden layer"

        layers = []
        prev_size = in_size
        for i in range(len(hidden_sizes) + 1):  # +1 to compensate for input layer
            if i < len(hidden_sizes):
                next_size = hidden_sizes[i]
            else:
                next_size = hidden_sizes[-1]    # Repeat last hidden layer size
            # Fully connected layer
            layers.append(nn.Linear(prev_size, next_size))
            # Batch normalization
            layers.append(nn.BatchNorm1d(next_size))
            # Activation function
            if activation == 'ReLU':
                layers.append(nn.ReLU())
            elif activation == 'Tanh':
                layers.append(nn.Tanh())
            elif activation == 'Sigmoid':
                layers.append(nn.Sigmoid())
            elif activation == 'LeakyReLU':
                layers.append(nn.LeakyReLU())
            elif activation is None:
                pass # Equal to lambda x: x
            else:
                raise ValueError(f"Unsupported activation function: {activation}")
            # Dropout
            if dropout_p > 0.0:
                layers.append(nn.Dropout(dropout_p))

            prev_size = next_size

        layers.append(nn.Linear(prev_size, out_size))

        self.fully_connected_stack = nn.Sequential(*layers)

        # Input-Output Normalization
        self.register_buffer('x_mean', x_mean)
        self.register_buffer('x_std', x_std)
        self.register_buffer('y_mean', y_mean)
        self.register_buffer('y_std', y_std)

    def forward(self, x):
        # Input normalization
        x = (x - self.x_mean) / self.x_std
        # Forward pass
        y = self.fully_connected_stack(x)
        # Output denormalization
        return (y * self.y_std) + self.y_mean
    
    def cs_forward(self, x):
        # Input normalization
        x = (x - self.x_mean.cpu().numpy()) / self.x_std.cpu().numpy()
        # Forward pass
        y = self.fully_connected_stack(x)
        # Output denormalization
        return (y * self.y_std.cpu().numpy()) + self.y_mean.cpu().numpy()