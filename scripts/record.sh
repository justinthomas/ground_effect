#!/bin/bash

ROBOT=QuadrotorAlpha

rosbag record \
  /$ROBOT/motors \
  /$ROBOT/odom \
  /$ROBOT/position_cmd \
  /$ROBOT/quad_decode_msg/output_data \
  /$ROBOT/so3_cmd \
  /$ROBOT/traj_num \
  /$ROBOT/traj_signal \
  /nanokontrol2
