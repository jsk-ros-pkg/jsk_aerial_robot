# rossrial based on ethernet (UDP) with STM32H7 Nucleo

## requirement:

1. [STM32CubeMX](https://www.st.com/ja/development-tools/stm32cubemx.html), please use the version of **v5.0.1**, and the version of `STM32H7` is **v1.3.0**.
2. [Truestudio](https://www.st.com/ja/development-tools/truestudio.html), please use version of **9.3.0**, otherwise, the flash would fail.
3. [Nucleo-H743ZI](https://www.st.com/ja/evaluation-tools/nucleo-h743zi.html)

## compile:

0. prepare the `ros_lib` for MCU:

   ```
   $ rosrun spinal make_libraries.py
   ```

1. build in Truestudio by:

   ```
   $ rosrun spinal true_studio.sh
   ```

2. flash to the Nucleo-H743ZI

## usage:

1. create a new Ethernet connection with static IP (192.168.25.xxx, xxx: except 238) in host PC.
   please check the connection by `ping`:

   ```
   $ ping 192.168.25.238
   ```

2. rosserial based on ethernet UDP.

   ```
   $ roslaunch rosserial_server udp_socket.launch client_addr:="192.168.25.238" client_port:=12345 server_port:=12345  --screen -v
   ```

3. perform the load test

   - check the published topic from device:

      ```
      $ rostopic hz /imu
      ```

      **note**: you can also add option `-w 2` to check the interval between two messages.

   - publish topic to device

      ```
      $ rosrun spinal load_test.py
      ```

      **note**: you can see the log in the terminal launching `udp_socket.launch`, such as, `[ERROR] [1560932626.875748081]: 2, 3, 1`. This means the current interval is 2 [ms], the max interval is 3 [ms], the min interval is 1 [ms].


### Option: generate from STM32CubeMX:
please check the MPU settings for ethernet DMA, based on following website:

- https://community.st.com/s/article/FAQ-Ethernet-not-working-on-STM32H7x3

- https://www.keshikan.net/gohantabeyo/?p=563

- http://nemuisan.blog.bai.ne.jp/?eid=215813

- https://www.st.com/content/ccc/resource/technical/document/application_note/group0/bc/2d/f7/bd/fb/3f/48/47/DM00272912/files/DM00272912.pdf/jcr:content/translations/en.DM00272912.pdf
