#!/usr/bin/env python
import subprocess
import time
import sys

sys.stdout.reconfigure(line_buffering=True)

def launch_node():
    process = subprocess.Popen(['roslaunch', 'ninja', 'bringup.launch', 'robot_id:=1'])
    print("ok")
    return process

if __name__ == '__main__':
    try:
        process = launch_node()
        while True:
            time.sleep(0.1)

    except KeyboardInterrupt:
        process.terminate()
        process.wait()
