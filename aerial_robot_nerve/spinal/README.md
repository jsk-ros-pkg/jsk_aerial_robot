# Spinal firmware

Firmware of the Spinal flight controller (STM32F7 / STM32H7 / STM32H7 v2) and its ROS interface.

## Build and flash

```bash
rosrun spinal cube_ide.sh stm32H7_v2   # or stm32H7 / stm32F7
```

Build and flash the project in STM32CubeIDE, or flash the generated binary directly:

```bash
st-flash write spinal.bin 0x08000000
```

## Host link

rosserial runs at **2 Mbps** over USART1 by default (`spinal/launch/bridge.launch`).
To use Ethernet/UDP instead (H7 / H7 v2 only), uncomment `#define USE_ETH` in the board's `Inc/main.h`
and launch the bridge with `mode:=udp`.

## Spinal ⇄ Neuron CAN

| Board | Peripheral | Frame format |
|---|---|---|
| Spinal H7 / H7 v2 | FDCAN | CAN FD with BRS (1 Mbps nominal / 8 Mbps data) |
| Neuron G4 | FDCAN | CAN FD with BRS (1 Mbps nominal / 8 Mbps data) |
| Spinal F7, Neuron F1 / F4 | bxCAN | Classic CAN only |

Servo target / feedback / config records use CAN FD frames (64 / 16 / 48 bytes), so the servo protocol requires
Spinal H7 / H7 v2 with Neuron G4 boards. Spinal and **all** Neuron boards must be flashed together,
because the CAN protocol and the ROS messages changed at the same time.
