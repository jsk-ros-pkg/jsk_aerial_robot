#!/bin/bash

stm32cubeide `rospack find spinal`/mcu_project/boards/${1:-stm32F7}/STM32CubeIDE/.project
