#!/usr/env python
import numpy as np
import matplotlib.pyplot as plt
import math
from scipy.optimize import fsolve

def exponential(x, base, k_exp):
  # return math.pow(base,x) * k_exp
  return pow(x,base) * k_exp

def exp_eqs(vars, base):
  x, k_exp = vars
  eq1 = exponential(x,base,k_exp) - x
  # eq2 = math.log(base) * exponential(x,base,k_exp) - 1
  eq2 = base * exponential(x,base-1,k_exp) - 1
  return [eq1,eq2]

def find_range_k_exp(base, initial_guess=(1.0,1.0)):
  range_sol, k_exp_sol = fsolve(exp_eqs, initial_guess, args=(base))
  return range_sol, k_exp_sol

def logarithm(x, base, k_log):
  # if x >= 0.0:
  #   return math.log(x,base) * k_log
  # else:
  #   return math.log(-x,base) * k_log
  return math.log(x,base) * k_log

def log_eqs(vars, base):
  x,k_log = vars
  eq1 = logarithm(x,base,k_log) - x
  eq2 = (k_log / (x*math.log(base))) - 1
  return [eq1,eq2]

def find_range_k_log(base, initial_guess=(1.0,1.0)):
  range_sol, k_log_sol = fsolve(log_eqs, initial_guess, args=(base))
  return range_sol, k_log_sol

exp_base = 1.45
log_base = 1.45
k_exp = 0.2
k_log = 1.5
a_log = k_log / (math.e*math.log(log_base))
range_log = math.e
y_exp_list = []
y_log_list = []
y_log_v2_list = []
x = np.linspace(-20, 20, 1000)
# range_exp, k_exp = find_range_k_exp(exp_base)
# print("range_exp = ",range_exp)
# print("k_exp = ", k_exp)
# range_log, k_log = find_range_k_log(log_base)
# print("range_log = ", range_log)
# print("k_log = ", k_log)
for i in x:
  if i >= 0:
    y_exp = exponential(i, exp_base, k_exp)
  if i < 0:
    y_exp = -exponential(-i, exp_base, k_exp)
  # if i>range_exp:
  #   y_exp = exponential(i,exp_base,k_exp)
  # elif i<-range_exp:
  #   y_exp = -exponential(-i,exp_base,k_exp)
  # else:
  #   y_exp = i
  y_exp_list.append(y_exp)
for i in x:
  if i>range_log:
    y_log = logarithm(i,log_base,k_log)
  elif i<-range_log:
    y_log = -logarithm(-i,log_base,k_log)
  else:
    y_log = i * a_log
  y_log_list.append(y_log)
for i in x:
  if i >= 0:
    y_log_v2 = logarithm(i+1,log_base,k_log)
  if i < 0:
    y_log_v2 = -logarithm(-(i-1),log_base,k_log)
  y_log_v2_list.append(y_log_v2)

plt.figure(figsize=(8, 6))
plt.plot(x, y_exp_list, label="exp", color='orange')
# plt.axvline(-range_exp, color='gray', linestyle='--', alpha=0.7)
# plt.axvline(range_exp, color='gray', linestyle='--', alpha=0.7)
plt.axhline(0, color='black', linewidth=0.5, alpha=0.7)
plt.xlabel("x")
plt.ylabel("y")
plt.legend()
plt.grid()
plt.show()

plt.figure(figsize=(8, 6))
plt.plot(x, y_log_list, label="log", color='orange')
plt.axvline(-range_log, color='gray', linestyle='--', alpha=0.7)
plt.axvline(range_log, color='gray', linestyle='--', alpha=0.7)
plt.axhline(0, color='black', linewidth=0.5, alpha=0.7)
plt.xlabel("x")
plt.ylabel("y")
plt.legend()
plt.grid()
plt.show()

plt.figure(figsize=(8, 6))
plt.plot(x, y_log_v2_list, label="log_v2", color='orange')
plt.axvline(-range_log, color='gray', linestyle='--', alpha=0.7)
plt.axvline(range_log, color='gray', linestyle='--', alpha=0.7)
# plt.axhline(0, color='black', linewidth=0.5, alpha=0.7)
plt.xlabel("x")
plt.ylabel("y")
plt.legend()
plt.grid()
plt.show()