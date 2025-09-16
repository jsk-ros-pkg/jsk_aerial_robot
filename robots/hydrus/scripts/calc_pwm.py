#!/usr/bin/env python


# ref1:
#         voltage: 25.2
#         max_thrust: 20.2552 # N
#         polynominal4: 2.54166562  # x10^4
#         polynominal3: -15.43369952  # x10^3
#         polynominal2: 26.10323352  # x10^2
#         polynominal1: 8.54686236  # x10^1
#         polynominal0: 53.25881203  # x10^0
# ref2:
#         voltage: 24.2
#         max_thrust: 18.3445 # N
#         polynominal4: 4.54333123  # x10^4
#         polynominal3: -23.12965516  # x10^3
#         polynominal2: 34.86255507  # x10^2
#         polynominal1: 6.83658356  # x10^1
#         polynominal0: 53.57564579  # x10^0
# ref3:
#         voltage: 23.2
#         max_thrust: 17.3934 # N
#         polynominal4: 3.91098427  # x10^4
#         polynominal3: -21.79964328  # x10^3
#         polynominal2: 33.15486115  # x10^2
#         polynominal1: 10.08123408  # x10^1
#         polynominal0: 53.32085963  # x10^0
# ref4:
#         voltage: 22.2
#         max_thrust: 15.9989 # N
#         polynominal4: 2.97715075  # x10^4
#         polynominal3: -19.74638039  # x10^3
#         polynominal2: 30.95809122  # x10^2
#         polynominal1: 13.21203700  # x10^1
#         polynominal0: 53.05390106  # x10^0

voltage = 24.2
min_pwm = 0.56
# max_pwm = 0.925
max_pwm = 0.7


def force_to_pwm(force):
    pwm = 53.25881203 + 8.54686236*0.1*force + 26.10323352*0.01*force**2 - 15.43369952*0.001*force**3 + 2.54166562*0.0001*force**4
    pwm = pwm / 100
    print("pwm (0~1): ", pwm)
    if pwm < min_pwm:
        pwm = min_pwm
    if pwm > max_pwm:
        pwm = max_pwm
    return pwm
