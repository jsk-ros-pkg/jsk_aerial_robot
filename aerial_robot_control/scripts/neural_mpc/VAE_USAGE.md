# Variational Auto Encoder (VAE) for Residual Dynamics Modeling

## Overview

This implementation adds a **Variational Auto Encoder (VAE)** as an alternative to the standard MLP for residual dynamics modeling in the MPC framework. The VAE learns a probabilistic mapping from input (state + control) to output (dynamics residuals) through an encoder-decoder architecture with a latent space.

## Key Features

### VAE Architecture
- **Encoder**: Maps input to latent distribution parameters (mean and log-variance)
- **Latent Space**: Compressed probabilistic representation of the dynamics
- **Decoder**: Maps latent representation back to output predictions
- **Reparameterization Trick**: Enables gradient-based optimization during training
- **Deterministic Inference**: Uses latent mean for control applications (no sampling variance)

### Benefits for MPC
1. **Uncertainty Quantification**: VAE naturally models uncertainty through its probabilistic latent space
2. **Regularized Learning**: KL divergence prevents overfitting and encourages smooth latent representations
3. **Robustness**: Latent space regularization can improve generalization to unseen conditions
4. **Feature Learning**: Encoder learns compressed features that capture essential dynamics

## How to Switch Between MLP and VAE

### 1. Configuration File

In [config/configurations.py](config/configurations.py), set the model type in `NetworkConfig`:

```python
class NetworkConfig:
    # Choose between "MLP" or "VAE"
    model_type = "MLP"  # Change to "VAE" to use Variational Auto Encoder
```

### 2. VAE-Specific Configuration

The `VAEConfig` class contains all VAE-specific hyperparameters:

```python
class VAEConfig:
    # Architecture
    encoder_hidden_sizes = [64, 32]     # Encoder network layers
    latent_dim = 16                      # Latent space dimension (adjust based on complexity)
    decoder_hidden_sizes = [32, 64]     # Decoder network layers (often symmetric to encoder)
    
    # Loss weighting
    beta = 1.0                           # Weight for KL divergence (beta-VAE)
    use_kl_annealing = False            # Gradually increase beta during training
    kl_annealing_epochs = 10            # Epochs to reach full beta
    
    # Standard hyperparameters (same as MLP)
    activation = "GELU"
    use_batch_norm = False
    dropout_p = 0.0
    num_epochs = 30
    batch_size = 64
    learning_rate = 1e-3
    # ... etc
```

### 3. Running Training

Simply run the training script as usual:

```bash
cd /path/to/model_fitting
python model_fit.py
```

The script will automatically:
- Detect the model type from `NetworkConfig.model_type`
- Load the appropriate configuration (`NetworkConfig` or `VAEConfig`)
- Build the corresponding model architecture
- Use the appropriate loss function (MSE for MLP, VAE Loss for VAE)
- Save model with correct metadata

## VAE-Specific Hyperparameters

### Architecture Parameters

#### `latent_dim` (default: 16)
- **Purpose**: Dimension of the compressed latent representation
- **Guidance**:
  - Too small: May lose important information (underfitting)
  - Too large: Less compression, less regularization benefit
  - Start with: `latent_dim ≈ (input_dim + output_dim) / 2`
  - For your case: input_dim=12, output_dim=3, try 8-16

#### `encoder_hidden_sizes` & `decoder_hidden_sizes`
- **Purpose**: Define network depth and width
- **Guidance**:
  - Often symmetric or tapered towards latent space
  - Example: `[64, 32]` for encoder, `[32, 64]` for decoder
  - Deeper networks for more complex dynamics

### Training Parameters

#### `beta` (default: 1.0)
- **Purpose**: Weight for KL divergence term (beta-VAE)
- **Effect**:
  - `beta = 0`: No KL regularization (becomes autoencoder)
  - `beta = 1`: Standard VAE
  - `beta > 1`: Stronger disentanglement, may reduce reconstruction quality
  - `beta < 1`: Better reconstruction, less regularization
- **Recommendation**: Start with 1.0, adjust based on reconstruction quality

#### `use_kl_annealing` (default: False)
- **Purpose**: Gradually increase KL weight during training
- **Benefits**:
  - Prevents "posterior collapse" early in training
  - Helps with training stability
  - Better initial learning
- **Usage**: Set to `True` and specify `kl_annealing_epochs`

## Training Output

When training with VAE, you'll see additional metrics:

```
Epoch | Step      | Train Loss | Recon Loss | KL Loss | Val Loss | Inference Time | LR
1/30  | 1024/1024 | 0.025430   | 0.024500   | 0.009300| 0.023100 | 1.23 ms       | 1e-3
```

- **Train Loss**: Total loss (reconstruction + beta * KL)
- **Recon Loss**: MSE reconstruction loss (how well it predicts outputs)
- **KL Loss**: KL divergence (regularization term)
- **Val Loss**: Total validation loss

## Model Saving and Loading

### Saved Model Structure

The saved `.pt` file contains:

**For VAE:**
```python
{
    'state_dict': ...,              # Model weights
    'model_type': 'VAE',            # Model type identifier
    'input_size': 12,
    'encoder_hidden_sizes': [64, 32],
    'latent_dim': 16,
    'decoder_hidden_sizes': [32, 64],
    'output_size': 3,
    'activation': 'GELU',
    'use_batch_norm': False,
    'dropout_p': 0.0
}
```

**For MLP:**
```python
{
    'state_dict': ...,
    'model_type': 'MLP',            # Model type identifier
    'input_size': 12,
    'hidden_sizes': [32, 32],
    'output_size': 3,
    # ... etc
}
```

### Loading a Model

```python
import torch
from network_architecture.vae import VAE
from network_architecture.mlp import MLP

# Load checkpoint
checkpoint = torch.load('path/to/model.pt')

# Check model type and instantiate accordingly
if checkpoint['model_type'] == 'VAE':
    model = VAE(
        checkpoint['input_size'],
        checkpoint['encoder_hidden_sizes'],
        checkpoint['latent_dim'],
        checkpoint['decoder_hidden_sizes'],
        checkpoint['output_size'],
        activation=checkpoint['activation'],
        # ... other params
    )
elif checkpoint['model_type'] == 'MLP':
    model = MLP(
        checkpoint['input_size'],
        checkpoint['hidden_sizes'],
        checkpoint['output_size'],
        # ... other params
    )

model.load_state_dict(checkpoint['state_dict'])
```

## Using VAE for Control

### Deterministic Predictions

For MPC and control, use **deterministic** forward pass:

```python
# During inference/control
model.eval()
with torch.no_grad():
    output = model.forward_deterministic(input)  # Uses latent mean, no sampling
```

The VAE automatically handles this in `forward()` when `model.eval()` is set, but `forward_deterministic()` is explicit.

### CasADi Integration

The VAE is compatible with `ml_casadi` for symbolic differentiation in MPC:

```python
# CasADi forward pass (uses deterministic path)
y_casadi = model.cs_forward(x_casadi)

# Linearization for linear MPC
y_linear = model.sym_linearize(x, x0)
```

## Comparison: MLP vs VAE

| Aspect | MLP | VAE |
|--------|-----|-----|
| **Architecture** | Direct mapping | Encoder → Latent → Decoder |
| **Training** | MSE loss | Reconstruction + KL divergence |
| **Uncertainty** | Point estimates | Probabilistic (via latent dist.) |
| **Regularization** | Manual (L1/L2/dropout) | Built-in (KL divergence) |
| **Interpretability** | Hidden layers | Structured latent space |
| **Complexity** | Simpler | More complex |
| **Training Time** | Faster | Slightly slower |
| **Best for** | Simple, deterministic dynamics | Complex, noisy dynamics |

## Hyperparameter Tuning Guide

### Starting Point
1. Set `model_type = "VAE"` in `NetworkConfig`
2. Use symmetric encoder-decoder: `[64, 32]` and `[32, 64]`
3. Set `latent_dim = 16`
4. Keep `beta = 1.0`
5. Train and observe reconstruction vs KL loss

### If Reconstruction Loss is High
- Increase `latent_dim`
- Decrease `beta` (e.g., 0.5 or 0.1)
- Increase encoder/decoder capacity
- Enable KL annealing

### If KL Loss is Near Zero ("Posterior Collapse")
- Enable `use_kl_annealing = True`
- Increase `beta` gradually
- Decrease learning rate
- Reduce encoder capacity

### If Training is Unstable
- Enable KL annealing
- Reduce learning rate
- Add batch normalization
- Increase batch size

## Example Configuration

For your aerial robot dynamics (3D acceleration prediction):

```python
class VAEConfig:
    # Architecture optimized for dynamics modeling
    encoder_hidden_sizes = [48, 24]    # Smooth compression
    latent_dim = 12                     # Between input (12) and output (3)
    decoder_hidden_sizes = [24, 48]    # Symmetric expansion
    
    # Training with stability
    beta = 0.5                          # Prioritize reconstruction
    use_kl_annealing = True
    kl_annealing_epochs = 10
    
    activation = "GELU"                 # Smooth gradients
    use_batch_norm = False              # Try without first
    learning_rate = 1e-3
    lr_scheduler = "MultiStepLR"
    lr_milestones = [15, 25]
    
    # Regularization
    consistency_lambda = 1e-2           # Output smoothness
    consistency_epsilon = 0.2
```

## Troubleshooting

### Issue: High validation loss
**Solution**: 
- Reduce `beta` to prioritize reconstruction
- Increase `latent_dim`
- Enable KL annealing

### Issue: KL loss is zero
**Solution**: 
- Enable `use_kl_annealing`
- Increase `beta` after initial training
- Check encoder is not too powerful

### Issue: Slower than MLP
**Expected**: VAE has more parameters and computes KL divergence
**Mitigation**: Use smaller encoder/decoder if speed is critical

## References

- Original VAE paper: Kingma & Welling (2013) "Auto-Encoding Variational Bayes"
- β-VAE: Higgins et al. (2017) "β-VAE: Learning Basic Visual Concepts with a Constrained Variational Framework"
- For dynamics modeling: Wahlström et al. (2015) "Deep Kalman Filters"

## Support

If you encounter issues or have questions:
1. Check that `model_type` is correctly set in `NetworkConfig`
2. Verify all dependencies are installed (torch, ml_casadi, etc.)
3. Compare training metrics between MLP and VAE
4. Adjust hyperparameters gradually

---

**Note**: The existing MLP code remains unchanged. You can always switch back by setting `model_type = "MLP"`.
