import numpy as np
import matplotlib.pyplot as plt

# ============================================================
#  Potential functions satisfying gradient monotonic decay
# ============================================================

# Exponential decay
def pot_exp(x, umax, x0, eps=0.01):
    k = np.log(1/eps) / x0
    return umax * np.exp(-k * x)

# Rational decay: U = umax / (1 + a x)
def pot_rational(x, umax, x0, eps=0.01):
    a = (1/np.sqrt(eps) - 1) / x0
    return umax / (1 + a * x)

# Power-law decay: U = umax / (1 + b x)^p
def pot_powerlaw(x, umax, x0, p=2, eps=0.01):
    b = (eps**(-1/(p+1)) - 1) / x0
    return umax / (1 + b * x)**p

# ============================================================
#  Numeric derivative
# ============================================================

def dUdx(x, U):
    return np.gradient(U, x)

# ============================================================
#  Parameters
# ============================================================

umax = 1.0
x0 = 2.0
x = np.linspace(0, x0, 500)

# Potentials
U_exp = pot_exp(x, umax, x0)
U_rat = pot_rational(x, umax, x0)
U_pow = pot_powerlaw(x, umax, x0, p=2)

# Gradients
d_exp = dUdx(x, U_exp)
d_rat = dUdx(x, U_rat)
d_pow = dUdx(x, U_pow)

# ============================================================
#  Plot potentials
# ============================================================

plt.figure()
plt.plot(x, U_exp, label="Exponential")
plt.plot(x, U_rat, label="Rational")
plt.plot(x, U_pow, label="Power-law (p=2)")
plt.title("Potential vs Distance")
plt.xlabel("distance")
plt.ylabel("potential")
plt.legend()
plt.grid(True)
plt.show()

# ============================================================
#  Plot gradients
# ============================================================

plt.figure()
plt.plot(x, d_exp, label="Exponential")
plt.plot(x, d_rat, label="Rational")
plt.plot(x, d_pow, label="Power-law (p=2)")
plt.title("Gradient dU/dx vs Distance")
plt.xlabel("distance")
plt.ylabel("gradient")
plt.legend()
plt.grid(True)
plt.show()
