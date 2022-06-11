#!/bin/bash

if [ "$1" = "" ]
then
    echo -e "\e[33m No argument! please select board type from ['stm32F7', 'stm32H7'], such as: \n \$ rosrun spinal cube_ide.sh stm32F7 \e[m"
    exit
elif [ $1 = "stm32F7" -o $1 = "stm32H7" ]
then
    stm32cubeide `rospack find spinal`/mcu_project/boards/$1/STM32CubeIDE/.project
else
    echo -e "\e[33m We do not support board type of $1, please select board type from ['stm32F7', 'stm32H7'], such as: \n \$ rosrun spinal cube_ide.sh stm32F7 \e[m"
fi
