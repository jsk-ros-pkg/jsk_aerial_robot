import rosbag
import matplotlib.pyplot as plt

if_save_csv = True

# Initialize a list to store the data
timestamp_list = []
gimbal_1_list = []
gimbal_2_list = []
gimbal_3_list = []
gimbal_4_list = []

# Specify the ROS bag file and topic name
# bag_file = "2023-12-30-15-35-17_beetle_Qa=1_fail.bag"
bag_file = "2023-12-30-17-56-53_beetle_servo_sysid.bag"
# bag_file = "beetle_body_rate_thrust_limit0.2_NoServoDelay_2023-12-26-12-04-34.bag"
gimbals_ctrl_topic_name = "/beetle1/gimbals_ctrl"
four_axis_topic_name = "/beetle1/four_axes/command"

# Open the ROS bag file
bag = rosbag.Bag(bag_file)

# Iterate through messages
for topic, msg, timestamp in bag.read_messages(topics=[gimbals_ctrl_topic_name, four_axis_topic_name]):
    if topic == gimbals_ctrl_topic_name:
        # Assuming the topic contains Float64 messages
        timestamp_list.append(timestamp.to_sec())
        gimbal_1_list.append(msg.position[0])
        gimbal_2_list.append(msg.position[1])
        gimbal_3_list.append(msg.position[2])
        gimbal_4_list.append(msg.position[3])

        print(
            f"gimbals_ctrl: {timestamp.to_sec()}, {msg.position[0]}, {msg.position[1]}, {msg.position[2]}, {msg.position[3]}"
        )

    # elif topic == four_axis_topic_name:
    #     thrusts = msg.base_thrust
    #     print(f"base_thrust: {timestamp}, {thrusts[0]}, {thrusts[1]}, {thrusts[2]}, {thrusts[3]}")

if if_save_csv:
    import csv
    import numpy as np

    with open("inter_" + bag_file + "_gimbals_ctrl.csv", "w", newline="") as csvfile:
        writer = csv.writer(csvfile)
        # writer.writerow(['timestamp', 'gimbal_1', 'gimbal_2', 'gimbal_3', 'gimbal_4'])
        # interpolate the data, timestamp from 435s to 465s with 50hz
        inter_timestamp_list = np.linspace(435, 465, 50 * (465 - 435))
        inter_gimbal_1_list = np.interp(inter_timestamp_list, timestamp_list, gimbal_1_list)
        inter_gimbal_2_list = np.interp(inter_timestamp_list, timestamp_list, gimbal_2_list)
        inter_gimbal_3_list = np.interp(inter_timestamp_list, timestamp_list, gimbal_3_list)
        inter_gimbal_4_list = np.interp(inter_timestamp_list, timestamp_list, gimbal_4_list)

        for i in range(len(inter_timestamp_list)):
            writer.writerow(
                [
                    inter_timestamp_list[i],
                    inter_gimbal_1_list[i],
                    inter_gimbal_2_list[i],
                    inter_gimbal_3_list[i],
                    inter_gimbal_4_list[i],
                ]
            )

# Close the ROS bag file
bag.close()

# Now you can work with the data as needed
# plot the curves

plt.plot(timestamp_list, gimbal_1_list)
plt.plot(timestamp_list, gimbal_2_list)
plt.plot(timestamp_list, gimbal_3_list)
plt.plot(timestamp_list, gimbal_4_list)
plt.show()
