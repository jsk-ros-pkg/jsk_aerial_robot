#!/bin/bash

if [ "$1" = "" ]; then
    echo -e "\e[33m No argument! Please select a board type from ['stm32F7', 'stm32H7', 'stm32H7_v2'], such as: \n \$ rosrun spinal cube_ide.sh stm32F7 \e[m"
    exit
elif [ "$1" = "stm32F7" ] || [ "$1" = "stm32H7" ] || [ "$1" = "stm32H7_v2" ]; then
    stm32cubeide `rospack find spinal`/mcu_project/boards/$1/STM32CubeIDE/.project
else
    echo -e "\e[33m We do not support board type '$1'. Please select a board type from ['stm32F7', 'stm32H7', 'stm32H7_v2'], such as: \n \$ rosrun spinal cube_ide.sh stm32F7 \e[m"
fi
