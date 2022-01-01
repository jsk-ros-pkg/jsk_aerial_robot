#!/usr/bin/env python

#####################################################################
# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

__usage__ = """
make_libraries.py generates the STM32 rosserial library files.  It
requires the location of your STM32 project folder.

rosrun spinal make_libraries.py  --output_path <output_path> --support_rtos --support_lwip
"""

import os
import rospkg
import rosserial_client
import argparse
from rosserial_client.make_library import *


# for copying files
from distutils.dir_util import copy_tree

ROS_TO_EMBEDDED_TYPES = {
    'bool'    :   ('bool',              1, PrimitiveDataType, []),
    'byte'    :   ('int8_t',            1, PrimitiveDataType, []),
    'int8'    :   ('int8_t',            1, PrimitiveDataType, []),
    'char'    :   ('uint8_t',           1, PrimitiveDataType, []),
    'uint8'   :   ('uint8_t',           1, PrimitiveDataType, []),
    'int16'   :   ('int16_t',           2, PrimitiveDataType, []),
    'uint16'  :   ('uint16_t',          2, PrimitiveDataType, []),
    'int32'   :   ('int32_t',           4, PrimitiveDataType, []),
    'uint32'  :   ('uint32_t',          4, PrimitiveDataType, []),
    'int64'   :   ('int64_t',           8, PrimitiveDataType, []),
    'uint64'  :   ('uint64_t',          8, PrimitiveDataType, []),
    'float32' :   ('float',             4, PrimitiveDataType, []),
    'float64' :   ('double',            8, PrimitiveDataType, []),
    'time'    :   ('ros::Time',         8, TimeDataType, ['ros/time']),
    'duration':   ('ros::Duration',     8, TimeDataType, ['ros/duration']),
    'string'  :   ('char*',             0, StringDataType, []),
    'Header'  :   ('std_msgs::Header',  0, MessageDataType, ['std_msgs/Header'])
}

rospack = rospkg.RosPack()
spinal_dir = rospack.get_path("spinal")

parser = argparse.ArgumentParser()
parser.add_argument('--save_path', default=os.path.join(spinal_dir, 'mcu_project/lib'),
                    help='path to save ros_lib')

parser.add_argument('--support_rtos', action='store_true',
                    help='whether support FreeRTOS')
parser.add_argument('--support_ethernet', action='store_true',
                    help='whether support ethernet (LWIP)')

args = parser.parse_args()
args.save_path = os.path.join(args.save_path, 'ros_lib')

# remove the old directory
if os.path.exists(args.save_path):
    shutil.rmtree(args.save_path)

# copy ros_lib stuff in
rosserial_client_copy_files(rospack, args.save_path + '/')

copy_tree(os.path.join(spinal_dir, "src/ros_lib"), args.save_path)

# generate messages
rosserial_generate(rospack, args.save_path, ROS_TO_EMBEDDED_TYPES)

# edit files according to options
filename = os.path.join(args.save_path, 'STM32Hardware.h')
with open(filename) as f:
    data_lines = f.read()

if args.support_rtos:
    data_lines = data_lines.replace("#define SUPPORT_RTOS 0", "#define SUPPORT_RTOS 1")
if args.support_ethernet:
    data_lines = data_lines.replace("#define SUPPORT_LWIP 0", "#define SUPPORT_LWIP 1")
with open(filename, mode="w") as f:
    f.write(data_lines)

