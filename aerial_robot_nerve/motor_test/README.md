## Setting
1. Set `NERVE_COMM = 0` and write firmware to spinal.
2. Connect like this: 
```
PC - spinal - ESC - rotor
|              |
|\ power supply/
|
\ force sensor 
```
3. Set IP address of the power supply to `192.168.0.2`.
4. Power on the power supply and set the voltage as you like (e.g. 25.2V).
6. In your terminal, do:
   ```
   $ roslaunch motor_test log.launch
   ```
   
   **note**: you can give your specific pwm config file as follows:
   ```
   $ roslaunch motor_test log.launch pwm_config_file:=xxxx
   ```
   
7. Start logging by `rosrun motor_test logging_start`.

## General calib for ESC:

Low PWM: 1000 (start with 1050)
High PWM: 1900
