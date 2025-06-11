import os, sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))    # Add parent directory to path to allow relative imports
import ml_casadi.torch as mc

class NormalizedMLP(mc.TorchMLCasadiModule):
    def __init__(self, model, x_mean, x_std, y_mean, y_std):
        super().__init__()
        self.model = model
        self.input_size = self.model.input_size
        self.output_size = self.model.output_size
        self.register_buffer('x_mean', x_mean)
        self.register_buffer('x_std', x_std)
        self.register_buffer('y_mean', y_mean)
        self.register_buffer('y_std', y_std)

    def forward(self, x):
        return (self.model((x - self.x_mean) / self.x_std) * self.y_std) + self.y_mean

    def cs_forward(self, x):
        return (self.model((x - self.x_mean.cpu().numpy()) / self.x_std.cpu().numpy()) * self.y_std.cpu().numpy()) + self.y_mean.cpu().numpy()
