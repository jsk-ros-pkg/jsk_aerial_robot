# Spinal firmware

Spinal is the flight-controller firmware and the ROS interface for the aerial
robot. The current firmware projects support STM32F7, STM32H7, and STM32H7 v2.

The two communication links are independent:

- The host computer communicates with Spinal through rosserial over either a
  UART/USB serial device or Ethernet/UDP.
- Spinal communicates with Neuron boards through Classic CAN or CAN FD.

Neither selection is a runtime ROS parameter in the MCU firmware. Change the
matching firmware definitions, rebuild, and flash the affected boards before
changing the ROS launch configuration.

## Open a Spinal firmware project

Open the STM32CubeIDE project for the flight-controller board that is actually
installed:

```bash
rosrun spinal cube_ide.sh stm32H7_v2
# Other supported targets: stm32H7 and stm32F7
```

> 💡 You can also use STM32CubeProgrammer to flash `.bin` or `.elf` files separately.  
`st-flash write spinal.bin 0x08000000`

Do not modify one board variant and flash another. The Ethernet and CAN FD
instructions below apply to the STM32H7 and STM32H7 v2 projects; STM32F7 does
not provide the same FDCAN path.

## Select the rosserial transport

### UART/USB serial

Serial is the current default. In the appropriate board header, leave
`USE_ETH` undefined:

- [`stm32H7/Inc/main.h`](mcu_project/boards/stm32H7/Inc/main.h)
- [`stm32H7_v2/Inc/main.h`](mcu_project/boards/stm32H7_v2/Inc/main.h)

```cpp
// #define USE_ETH
```

The firmware then initializes rosserial on USART1. Start the host bridge in
serial mode:

```bash
roslaunch spinal bridge.launch \
  mode:=serial \
  serial_port:=/dev/flight_controller \
  serial_baud:=2000000
```

### Ethernet/UDP

Enable Ethernet in the matching Spinal board header:

```cpp
#define USE_ETH
```

Rebuild and flash Spinal. The current firmware uses these static endpoints:

| Endpoint | Address |
|---|---|
| Spinal | `192.168.25.238/24` |
| Host computer | `192.168.25.100/24` |
| Gateway | `192.168.25.253` |
| rosserial UDP source/destination port | `12345` |

The Spinal address and gateway are defined in
[`Src/lwip.c`](mcu_project/boards/stm32H7_v2/Src/lwip.c), and the host address
and UDP ports are defined in
[`Src/main.c`](mcu_project/boards/stm32H7_v2/Src/main.c). Configure the host
Ethernet interface as `192.168.25.100/24`, then start:

```bash
roslaunch spinal bridge.launch \
  mode:=udp \
  udp_addr:=192.168.25.238 \
  udp_port:=12345
```

[`bridge.launch`](launch/bridge.launch) supports both modes. The current Dragon
v1.5 sensor launch explicitly selects serial mode in
[`sensors.launch.xml`](../../robots/dragon/launch/includes/v1_5_202601/sensors.launch.xml).
For normal Dragon bringup, change that include to pass `mode="udp"`,
`udp_addr="192.168.25.238"`, and `udp_port="12345"`, or expose equivalent
top-level launch arguments.

The host and firmware selections must match. Run only one rosserial bridge;
starting both serial and UDP bridges in the same robot namespace creates a node
name and topic conflict.

## Select Classic CAN or CAN FD

### Hardware support and current configuration

| Board | Peripheral | Current source configuration |
|---|---|---|
| Spinal STM32F7 | bxCAN | Classic CAN only |
| Spinal STM32H7/H7 v2 | FDCAN | `FDCAN_FRAME_FD_BRS` |
| Neuron F1/F4 | bxCAN | Classic CAN only |
| Neuron G4 | FDCAN | `FDCAN_FRAME_CLASSIC` |

The current H7/G4 bit timing uses an 80 MHz FDCAN clock. It provides a 1 Mbps
nominal/arbitration phase and an 8 Mbps CAN-FD data phase. All nodes on the bus
must use identical nominal and data timings.

The peripheral format is configured in:

- Spinal H7 v2:
  [`Src/main.c`](mcu_project/boards/stm32H7_v2/Src/main.c), function
  `MX_FDCAN1_Init()`
- Spinal H7:
  [`Src/main.c`](mcu_project/boards/stm32H7/Src/main.c), function
  `MX_FDCAN1_Init()`
- Neuron G4:
  [`Core/Src/main.c`](../neuron/neuron_g4/Core/Src/main.c), function
  `MX_FDCAN1_Init()`

When changing the setting in STM32CubeIDE, update the corresponding `.ioc`
configuration too; otherwise later code generation can overwrite a direct edit
to `main.c`.

### Use Classic CAN

Use Classic CAN when any Neuron F1/F4 board or other Classic-CAN-only node is
connected. Configure both FDCAN-capable endpoints as:

```cpp
hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
```

Keep every CAN payload at eight bytes or fewer. The current CAN transport code
already selects `FDCAN_CLASSIC_CAN` for payloads of this size. A larger payload
must be split into multiple Classic CAN frames.

### Use CAN FD with bit-rate switching

CAN FD requires a Spinal H7/H7 v2 and Neuron G4 boards. Confirm that every node
and CAN transceiver on the bus supports CAN FD before proceeding; F1/F4 Neuron
boards cannot remain on a bus that transmits CAN-FD frames.

1. Configure both Spinal and every Neuron G4 as:

   ```cpp
   hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
   ```

2. Keep the current matching bit timings unless the whole bus is changed
   together:

   ```text
   NominalPrescaler = 1, NominalTimeSeg1 = 63, NominalTimeSeg2 = 16
   DataPrescaler    = 1, DataTimeSeg1    = 7,  DataTimeSeg2    = 2
   ```

3. Check the transmit-frame policy in both implementations:

   - [`Spinal CAN core`](mcu_project/lib/Jsk_Lib/CAN/can_core.cpp)
   - [`Neuron CAN core`](../neuron/neuronlib/CAN/can_core.cpp)

   The current policy sends payloads of eight bytes or fewer as Classic CAN and
   uses `FDCAN_FD_CAN` only for payload lengths `12`, `16`, `20`, `24`, `32`,
   `48`, or `64`. `FDCAN_BRS_ON` is already selected. Therefore, enabling
   `FDCAN_FRAME_FD_BRS` alone does not move the existing short messages to the
   8 Mbps data phase. To accelerate a short message, change its transmit policy
   to use `FDCAN_FD_CAN` on both sides of the protocol, or aggregate it into a
   supported CAN-FD payload length.

4. Rebuild and flash Spinal and all Neuron boards as one coordinated update.
   Do not operate the robot with mixed Classic-only and CAN-FD firmware.

After restarting the complete system, verify that every Neuron is enumerated:

```bash
rosservice call /dragon/get_board_info "{}"
```

If no Neuron responds, first restore Classic CAN on all boards, then recheck the
board variants, nominal/data timing, transceiver CAN-FD capability, bus
termination, and firmware versions.
