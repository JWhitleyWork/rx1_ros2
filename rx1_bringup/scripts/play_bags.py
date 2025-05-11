#!/usr/bin/env python

import subprocess


# List of ROS bag files to play in sequence
rosbags = [
    '/home/lingkang/rosbag/dual_arm_hand_bag.bag',
    '/home/lingkang/rosbag/command_head.bag',
    '/home/lingkang/rosbag/right_arm_2.bag',
    '/home/lingkang/rosbag/command_head.bag',
    '/home/lingkang/rosbag/dual_arm_hand_bag.bag',
    '/home/lingkang/rosbag/command_head.bag',
    '/home/lingkang/rosbag/dual_arm_hand_bag.bag',
    '/home/lingkang/rosbag/command_head.bag',
    '/home/lingkang/rosbag/dual_arm_hand_bag.bag',
    '/home/lingkang/rosbag/command_head.bag',
]


# Function to play a single ROS bag file
def play_rosbag(bag_file):
    print(f'Playing {bag_file}')
    process = subprocess.Popen(['ros2', 'bag', 'play', '--clock', bag_file])
    process.wait()  # Wait for the current bag file to finish playing


if __name__ == '__main__':
    # Set use_sim_time to true
    subprocess.call(['ros2', 'param', 'set', '/use_sim_time', 'true'])

    # Play each ROS bag file in sequence
    for bag in rosbags:
        play_rosbag(bag)
    for bag in rosbags:
        play_rosbag(bag)
    for bag in rosbags:
        play_rosbag(bag)
    for bag in rosbags:
        play_rosbag(bag)
    for bag in rosbags:
        play_rosbag(bag)
