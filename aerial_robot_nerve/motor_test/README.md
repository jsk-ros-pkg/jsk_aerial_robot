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
   $ roslaunch motor_test test.launch
   ```
   
   **parameter**:
   - `test_mode`: the test mode. Step mode (:=0) is to give continous steps command to the ESC, the problem is the bias of the force sensor increases during the steps commands. Thus, we recommand to use **one-shot** mode (:=1, default mode), which iteratively gives an increasing PWM value. The difference is this mode stops the motor rotation after each step, and re-calibrate the force sensor before the next step. So the bias of force sensor can be considered to be zero.
   - `run_duration`: the duration of motor rotation during each step.
   - `pwm_incremental_value`: the increamental value between two steps.
   - `min_pwm_value`/`max_pwm_value`: the min/max of the pwm value
   - `raise_duration`/`brake_duration`: these are the special parameters for one-shot mode, since we have to consider the raise-up phase after staring rotation and brake-down phase after stopping rotation. For small propellers (e.g. 5inch), these value can be small. For large propellers (e.g. 14inch), please increase there values

   **sample**:
   - for 5inch propeller:
     ```
     $ roslaunch motor_test test.launch max_pwm_value:=1900 raise_duration:=1.0 brake_duration:=2.0 run_duration:=2.0
     ```

   - for 5inch propeller:
     ```
     $ roslaunch motor_test test.launch max_pwm_value:=1200 raise_duration:=2  brake_duration:=5 run_duration:=2
     ```
     
7. Start logging by `rosrun motor_test logging_start`.

## General calib for ESC:

Low PWM: 1000 (start with 1050)
High PWM: 1900
