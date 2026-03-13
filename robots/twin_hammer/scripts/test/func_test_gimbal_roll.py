#!/usr/env python
import numpy as np
import matplotlib.pyplot as plt
import math
from scipy.optimize import fsolve

def func_polynominal(x,r):
  return (2/r**2)*(x**3) - (1/r**4)*(x**5)

def gaussian_equations(vars, r, e):
    # y = a * x * (1 - np.exp(-b * x**2))
    # dy/dx = a * [ (1 - np.exp(-b * x**2)) + 2b * x^2 * np.exp(-b * x^2) ]
    a, b = vars
    # 条件1: x = r で y = r
    eq1 = a * r * (1 - np.exp(-e * r**2)) + b * r**3 - r
    # 条件2: x = r で dy/dx = 1
    eq2 = a * (1 - np.exp(-e * r**2) + 2*e * r**2 * np.exp(-e * r**2)) + 3 * b * r**2 - 1
    return [eq1, eq2]

def function_gaussian(x,r,e):
  # return x * (1 - np.exp(-10 * x**2))

  # 初期推定値 (a, b)
  initial_guess = [1, 1]
  # 方程式を解く
  a, b = fsolve(gaussian_equations, initial_guess, args=(r,e))
  # print(f"解: a = {a}, b = {b}")
  return a*x*(1 - np.exp(-e * x**2)) + b*(x**3)

y_g_list = []
y_p_list = []
range = 0.8
gaussian_exp_param = 10
x = np.linspace(-range-0.2, range+0.2, 1000)
for i in x:
  if i<-range or i>range:
    y_g = i
    y_p = i
  else:
    y_g = function_gaussian(i,range,gaussian_exp_param)
    y_p = func_polynominal(i,range)
  y_g_list.append(y_g)
  y_p_list.append(y_p)

plt.figure(figsize=(8, 6))
plt.plot(x, y_g_list, label="Gaussian smoothing", color='orange')
plt.plot(x, y_p_list, label="polynominal", color='blue')
plt.axvline(-range, color='gray', linestyle='--', alpha=0.7)
plt.axvline(range, color='gray', linestyle='--', alpha=0.7)
plt.axhline(0, color='black', linewidth=0.5, alpha=0.7)
# plt.title("Function 2 (Gaussian smoothing)")
plt.xlabel("x")
plt.ylabel("y")
plt.legend()
plt.grid()
plt.show()