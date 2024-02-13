'''
created by Jinjie LI, 2023/02/06
'''
import argparse
import rosbag
import csv
import numpy as np
import matplotlib.pyplot as plt

if_save_csv = True
TIME_START = 0
TIME_END = 30
FREQ_INTEP = 100


class DataStore:
    def __init__(self, number):
        self.timestamp_list = []
        for i in range(number):
            setattr(self, f'data_{i}_list', [])


def process_data(file_name):
    # Initialize a list to store the data
    cmd_data = DataStore(4)
    real_data = DataStore(4)

    # Specify the ROS bag file and topic name
    bag_file = file_name
    joint_states_topic_name = "/beetle1/joint_states"
    gimbals_ctrl_topic_name = "/beetle1/gimbals_ctrl"

    # Open the ROS bag file
    bag = rosbag.Bag(bag_file)

    # Iterate through messages
    for topic, msg, timestamp in bag.read_messages(topics=[joint_states_topic_name, gimbals_ctrl_topic_name]):
        if topic == gimbals_ctrl_topic_name:
            cmd_data.timestamp_list.append(timestamp.to_sec())
            for i in range(4):
                getattr(cmd_data, f'data_{i}_list').append(msg.position[i])

        if topic == joint_states_topic_name:
            real_data.timestamp_list.append(timestamp.to_sec())
            for i in range(4):
                getattr(real_data, f'data_{i}_list').append(msg.position[i])

    # Close the ROS bag file
    bag.close()

    # find the max first timestamp
    time_start = max(cmd_data.timestamp_list[0], real_data.timestamp_list[0])

    # normalize the timestamp
    cmd_data.timestamp_list = [t - time_start for t in cmd_data.timestamp_list]
    real_data.timestamp_list = [t - time_start for t in real_data.timestamp_list]

    # interpolate the data
    cmd_data.inter_timestamp_list = np.linspace(TIME_START, TIME_END, FREQ_INTEP * (TIME_END - TIME_START))
    cmd_data.inter_data_0_list = np.interp(cmd_data.inter_timestamp_list, cmd_data.timestamp_list, cmd_data.data_0_list)
    cmd_data.inter_data_1_list = np.interp(cmd_data.inter_timestamp_list, cmd_data.timestamp_list, cmd_data.data_1_list)
    cmd_data.inter_data_2_list = np.interp(cmd_data.inter_timestamp_list, cmd_data.timestamp_list, cmd_data.data_2_list)
    cmd_data.inter_data_3_list = np.interp(cmd_data.inter_timestamp_list, cmd_data.timestamp_list, cmd_data.data_3_list)

    real_data.inter_timestamp_list = np.linspace(TIME_START, TIME_END, FREQ_INTEP * (TIME_END - TIME_START))
    real_data.inter_data_0_list = np.interp(real_data.inter_timestamp_list, real_data.timestamp_list,
                                            real_data.data_0_list)
    real_data.inter_data_1_list = np.interp(real_data.inter_timestamp_list, real_data.timestamp_list,
                                            real_data.data_1_list)
    real_data.inter_data_2_list = np.interp(real_data.inter_timestamp_list, real_data.timestamp_list,
                                            real_data.data_2_list)
    real_data.inter_data_3_list = np.interp(real_data.inter_timestamp_list, real_data.timestamp_list,
                                            real_data.data_3_list)

    if if_save_csv:
        with open("inter_" + bag_file + "_gimbals_ctrl.csv", "w", newline="") as csvfile:
            writer = csv.writer(csvfile)
            for i in range(len(cmd_data.inter_timestamp_list)):
                writer.writerow(
                    [
                        cmd_data.inter_timestamp_list[i],
                        cmd_data.inter_data_0_list[i],
                        cmd_data.inter_data_1_list[i],
                        cmd_data.inter_data_2_list[i],
                        cmd_data.inter_data_3_list[i],
                    ]
                )
                print(
                    f"gimbals_ctrl: {cmd_data.inter_timestamp_list[i]}, {cmd_data.inter_data_0_list[i]}, {cmd_data.inter_data_1_list[i]}, {cmd_data.inter_data_2_list[i]}, {cmd_data.inter_data_3_list[i]}")

        with open("inter_" + bag_file + "_joint_states.csv", "w", newline="") as csvfile:
            writer = csv.writer(csvfile)
            for i in range(len(real_data.inter_timestamp_list)):
                writer.writerow(
                    [
                        real_data.inter_timestamp_list[i],
                        real_data.inter_data_0_list[i],
                        real_data.inter_data_1_list[i],
                        real_data.inter_data_2_list[i],
                        real_data.inter_data_3_list[i],
                    ]
                )
                print(
                    f"joint_states: {real_data.inter_timestamp_list[i]}, {real_data.inter_data_0_list[i]}, {real_data.inter_data_1_list[i]}, {real_data.inter_data_2_list[i]}, {real_data.inter_data_3_list[i]}")

    # Now you can work with the data as needed
    # plot the curves
    # draw a figure with four subplots, each subplot is two curves with cmd and real data
    fig, axs = plt.subplots(2, 2, figsize=(25, 10))
    plt.tight_layout(pad=3.0)
    for i in range(4):
        axs[i // 2, i % 2].plot(cmd_data.timestamp_list, getattr(cmd_data, f'data_{i}_list'), label='cmd')
        axs[i // 2, i % 2].plot(real_data.timestamp_list, getattr(real_data, f'data_{i}_list'), label='real')
        axs[i // 2, i % 2].set_title(f'Joint {i} data -- original')
        axs[i // 2, i % 2].set_xlabel('Time (s)')
        axs[i // 2, i % 2].set_ylabel('Position')
        axs[i // 2, i % 2].legend()
    plt.show()

    # draw a figure with four subplots, each subplot is two curves with interpolated cmd and real data
    fig, axs = plt.subplots(2, 2, figsize=(25, 10))
    plt.tight_layout(pad=3.0)
    for i in range(4):
        axs[i // 2, i % 2].plot(cmd_data.inter_timestamp_list, getattr(cmd_data, f'inter_data_{i}_list'), label='cmd')
        axs[i // 2, i % 2].plot(real_data.inter_timestamp_list, getattr(real_data, f'inter_data_{i}_list'), label='real')
        axs[i // 2, i % 2].set_title(f'Joint {i} data -- interpolated')
        axs[i // 2, i % 2].set_xlabel('Time (s)')
        axs[i // 2, i % 2].set_ylabel('Position')
        axs[i // 2, i % 2].legend()
    plt.show()

    return


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Process and visualize data from a rosbag file.')
    parser.add_argument('file_name', help='Name of the text file containing the data')
    args = parser.parse_args()

    process_data(args.file_name)
