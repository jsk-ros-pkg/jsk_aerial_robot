# NuceleoH743ZI board

## 1. rosserial based on ethernet (UDP) with STM32H7 Nucleo

### requirement:

1. [STM32CubeMX](https://www.st.com/ja/development-tools/stm32cubemx.html), please use the version of [here](https://github.com/tongtybj/aerial_robot/blob/PR/feature/STM32H7_ETH/aerial_robot_nerve/stm32h7_nucleo/stm32h7_nucleo.ioc#L604).
2. [STM32CubeIDE](https://www.st.com/ja/development-tools/stm32cubeide.html)
3. [Nucleo-H743ZI](https://www.st.com/ja/evaluation-tools/nucleo-h743zi.html)

### compile:

0. prepare the `ros_lib` for MCU:

   ```
   $ catkin bt
   ```

1. build in STMCubeIDE by:

   ```
   $ rosrun stm32h7_nucleo cube_ide.sh
   ```

2. flash to the Nucleo-H743ZI via STM32CubeIDE

### usage:

1. create a new Ethernet connection with static IP address **192.168.25.100** in host PC. This IP address is hard-coding written [here](https://github.com/tongtybj/aerial_robot_private/blob/spinal_stm32h7/aerial_robot_nerve/spinal/stm32h7_nucleo/Src/main.cpp#L248)
   please check the connection by `ping`:

   ```
   $ ping 192.168.25.238
   ```

2. rosserial based on ethernet UDP. Please use [this branch](https://github.com/tongtybj/rosserial/tree/spinal)

   ```
   $ roslaunch spinal bridge.launch mode:=udp
   ```

3. perform the load test

   - check the published topic from device:

      ```
      $ rostopic hz /imu
      ```

      **note**: you can also add option `-w 2` to check the interval between two messages.

   - publish topic to device

      ```
      $ rosrun stm32h7_nucleo load_test.py
      ```

      **note**: you can see the log in the terminal launching `udp_socket.launch`, such as, `[ERROR] [1560932626.875748081]: 2, 3, 1, 1, 2`. This means the current temporal interval between two receive topics in MCU is 2 [ms], the max interval is 3 [ms], the min interval is 1 [ms]; the current sequence id interval between two receive topics in MCU is 1 [frame], the max interval is [frames].


#### Option: generate from STM32CubeMX:
please check the MPU settings for ethernet DMA, based on following website:

- https://community.st.com/s/article/FAQ-Ethernet-not-working-on-STM32H7x3

- https://www.keshikan.net/gohantabeyo/?p=563

- http://nemuisan.blog.bai.ne.jp/?eid=215813

- https://www.st.com/content/ccc/resource/technical/document/application_note/group0/bc/2d/f7/bd/fb/3f/48/47/DM00272912/files/DM00272912.pdf/jcr:content/translations/en.DM00272912.pdf

## 2. CANFD with two NucleoH743ZI boards

### External transmitter board:

https://drive.google.com/uc?export=view&id=1TKMwXH4Lm9L4QMkjgN_sgr2S7u54GGa7

#### cables:

- yellow: CAN_RX, No.25
- green:  CAN_TX, No.27
- red: 5V (only one board to provide power)
- black: GND (needs to connect two boards)


### Usage:


- Push the blue button when power the board, or push both the blue and black button when reset the board. You can see the blue led turn on, indicates entering the CANFD mode
- Relase the blue button to start the process. You can see the blue led turn off, but soon start blink.
- After Powering both two boards, push blue button of **one of** the boards to trigger CAN TX from this board. Then you can see the **green** leds of two boards start to blink. This means both boards receive CAN message from each other.
- Please read `spinal/mcu_project/Hydrus_Lib/CANDevice/test/canfd_test.*`.

### Classic CAN mode for Spianl-Neuron test

- The external transmitter board also support the connect of neuron. Please check https://drive.google.com/uc?export=view&id=1WYRR4KZtqjQvnGTJE5B0apQbLLGStGB4
- To check the calssi CAN mode for STM32F4 based Neuron. Please connect the neuron with general way.


## 3. UART rosserial mode

### cables:

- yellow: Rx (USE-serial board) - Tx (D53 in nucleo board)
- green:  Tx (USE-serial board) - Rx (D52 in nucleo board)
- black1: GND
- black2: A0 in nucleo board - 3.3V for jumping to uart mode

### command:
```
$ roslaunch spinal bridge.launch mode:=serial
```