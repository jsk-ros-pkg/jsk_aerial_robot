import rospy
import subprocess
from std_msgs.msg import Bool  # Import Bool message type

def callback(data):
    if data.data:  # Access 'data' attribute since 'data' is a Bool message
        subprocess.run("echo 0 | sudo tee /sys/class/gpio/gpio464/value", shell=True, check=True)
        subprocess.run("echo 1 | sudo tee /sys/class/gpio/gpio465/value", shell=True, check=True)
    else:
        subprocess.run("echo 0 | sudo tee /sys/class/gpio/gpio465/value", shell=True, check=True)
        subprocess.run("echo 1 | sudo tee /sys/class/gpio/gpio464/value", shell=True, check=True)

def listener():
    rospy.init_node('vim2arduino', anonymous=True)

    # Unexport GPIO pins before exporting (to avoid "Device busy" error)
    subprocess.run("echo 464 | sudo tee /sys/class/gpio/unexport", shell=True, check=False)
    subprocess.run("echo 465 | sudo tee /sys/class/gpio/unexport", shell=True, check=False)

    # Export and configure GPIO pins
    subprocess.run("echo 464 | sudo tee /sys/class/gpio/export", shell=True, check=True)
    subprocess.run("echo out | sudo tee /sys/class/gpio/gpio464/direction", shell=True, check=True)
    subprocess.run("echo 465 | sudo tee /sys/class/gpio/export", shell=True, check=True)
    subprocess.run("echo out | sudo tee /sys/class/gpio/gpio465/direction", shell=True, check=True)

    rospy.Subscriber("quadrotor/gripper", Bool, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
