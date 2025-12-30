#!/bin/bash

if [ "$1" = "" ]
then
    echo -e "\e[33m No argument! please select board type from ['neuron_f4', 'neuron_g4'] \e[m"
    exit
elif [ $1 = "neuron_f4" -o $1 = "neuron_g4" ]
then
    stm32cubeide ../$1/STM32CubeIDE/.project
else
    echo -e "\e[33m We do not support board type of $1, please select board type from ['neuron_f4', 'neuron_g4']\e[m"
fi
