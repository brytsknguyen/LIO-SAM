#!/bin/bash

# Compulsary arguments
export LAUNCH_FILE=$1;
export ROS_PKG=$2;
export ROSBAG_PATH=$3;
export CAPTURE_SCREEN=$4;
export LOG_DATA=$5;
export LOG_PATH=$6;
export LOOP_EN=$7;
ARG_IMU_TOPIC=${8:-/imu/imu}
IMU_TOPIC=$ARG_IMU_TOPIC

# Begin the processing

# Find the path to the package
ROS_PKG_DIR=$(rospack find $ROS_PKG)

# Notify the bag file
echo BAG FILE: $ROSBAG_PATH;

if $LOG_DATA
then

# Create the log director
mkdir -p $LOG_PATH/ ;
# Copy the config folders for later references
cp -R $ROS_PKG_DIR/config $LOG_PATH;
cp -R $ROS_PKG_DIR/launch $LOG_PATH;
cp -R $ROS_PKG_DIR/script $LOG_PATH;
# Create folder for BA output
mkdir -p $LOG_PATH/ba;

fi

#Notify the log file
echo LOG DIR: $LOG_PATH;


# Turn on the screen capture if selected
if $CAPTURE_SCREEN
then

echo CAPTURING SCREEN ON;
(
ffmpeg -video_size 1920x1080 -framerate 0.01 -f x11grab -i $DISPLAY+0,0 \
-loglevel quiet -y $LOG_PATH/screen.mp4
) &
FFMPEG_PID=$!

else

echo CAPTURING SCREEN OFF;
sleep 1;

fi

echo FFMPEG PID $FFMPEG_PID

if $LOG_DATA
then

echo LOGGING ON;

# Start the process
(

roslaunch $ROS_PKG $LAUNCH_FILE \
autorun:=1 \
loop_en:=$LOOP_EN \
bag_file:=$ROSBAG_PATH \
exp_log_dir:=$LOG_PATH/ba

)&

MAIN_PID=$!

# Log the topics
( sleep 1; rostopic echo -p --nostr --noarr /odometry/imu \
> $LOG_PATH/predict_odom.csv ) \
& \
( sleep 1; rostopic echo -p --nostr --noarr /lio_sam/mapping/odometry \
> $LOG_PATH/opt_odom.csv ) \
& \
( sleep 1; rostopic echo -b $ROSBAG_PATH -p --nostr --noarr /leica/pose/relative \
> $LOG_PATH/leica_pose.csv ) \
& \
( sleep 1; rostopic echo -b $ROSBAG_PATH -p --nostr --noarr /dji_sdk/imu \
> $LOG_PATH/dji_sdk_imu.csv ) \
& \
( sleep 1; rostopic echo -b $ROSBAG_PATH -p --nostr --noarr $IMU_TOPIC \
> $LOG_PATH/vn100_imu.csv ) \
& \
( sleep 1; rostopic echo -p --nostr --noarr /opt_stat \
> $LOG_PATH/opt_stat.csv ) \
& \

else

echo LOGGING OFF;
sleep 1;

fi

wait $MAIN_PID;

# Close the screen recorder
kill $FFMPEG_PID;

exit;