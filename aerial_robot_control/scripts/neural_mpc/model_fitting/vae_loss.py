import torch

class VAELoss:
    """
    Combined loss for VAE training:
    - Reconstruction loss (MSE between predicted and actual output)
    - KL divergence (regularization term for latent distribution)
    """
    
    def __init__(self, beta=1.0, weight=None, device='cpu'):
        """
        Args:
            beta: Weight for KL divergence term (KL annealing parameter)
            weight: Weight for each output dimension in reconstruction loss
            device: Device for tensors
        """
        self.beta = beta
        self.weight = weight if weight is not None else torch.ones(1).to(device)
        self.device = device
    
    def __call__(self, y_pred, mu, logvar, y_true):
        """
        Compute VAE loss.
        
        Args:
            y_pred: Predicted output from decoder
            mu: Mean of latent distribution
            logvar: Log-variance of latent distribution
            y_true: True output (ground truth)
        
        Returns:
            total_loss, reconstruction_loss, kl_loss
        """
        # Reconstruction loss (weighted MSE)
        recon_loss = torch.mean(torch.square(y_true - y_pred) * self.weight)
        
        # KL divergence loss
        # KL(N(mu, sigma) || N(0, 1)) = -0.5 * sum(1 + log(sigma^2) - mu^2 - sigma^2)
        kl_loss = -0.5 * torch.sum(1 + logvar - mu.pow(2) - logvar.exp())
        kl_loss = kl_loss / y_pred.shape[0]  # Normalize by batch size
        
        # Total loss
        total_loss = recon_loss + self.beta * kl_loss
        
        return total_loss, recon_loss, kl_loss
