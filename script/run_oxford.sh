catkin build fast_lio ;
source /home/$USER/dev_ws/devel/setup.bash

# Get the current directory
CURR_DIR=$(pwd)
echo CURRENT DIR: $CURR_DIR

# Get the location of the slict package
ROS_PKG=fast_lio
echo FLIO DIR: $(rospack find $ROS_PKG)

CAPTURE_SCREEN=true;
LOG_DATA=true;

LAUNCH_FILE=run_oxford.launch
DATASET_LOC=/media/$USER/mySDSSD/NewerCollegeDataset


EPOC_DIR=/home/$USER/MATLAB_WS/SLICT/flio_oxford_20220917
#region Disable loop closure -----------------------------------------------------------------------------------------#

# OXFORD_SEQ=01_short_experiment
# ROSBAG_PATH=$DATASET_LOC/$OXFORD_SEQ/${OXFORD_SEQ}.bag
# LOG_PATH=$EPOC_DIR/result_$OXFORD_SEQ
# ./run_one_bag.sh $LAUNCH_FILE $ROS_PKG $ROSBAG_PATH $CAPTURE_SCREEN $LOG_DATA $LOG_PATH 0 '/os1_cloud_node/imu';

# OXFORD_SEQ=02_long_experiment
# ROSBAG_PATH=$DATASET_LOC/$OXFORD_SEQ/${OXFORD_SEQ}.bag
# LOG_PATH=$EPOC_DIR/result_$OXFORD_SEQ
# ./run_one_bag.sh $LAUNCH_FILE $ROS_PKG $ROSBAG_PATH $CAPTURE_SCREEN $LOG_DATA $LOG_PATH 0 '/os1_cloud_node/imu';

OXFORD_SEQ=05_quad_with_dynamics
ROSBAG_PATH=$DATASET_LOC/$OXFORD_SEQ/${OXFORD_SEQ}.bag
LOG_PATH=$EPOC_DIR/result_$OXFORD_SEQ
./run_one_bag.sh $LAUNCH_FILE $ROS_PKG $ROSBAG_PATH $CAPTURE_SCREEN $LOG_DATA $LOG_PATH 0 '/os1_cloud_node/imu';

OXFORD_SEQ=06_dynamic_spinning
ROSBAG_PATH=$DATASET_LOC/$OXFORD_SEQ/${OXFORD_SEQ}.bag
LOG_PATH=$EPOC_DIR/result_$OXFORD_SEQ
./run_one_bag.sh $LAUNCH_FILE $ROS_PKG $ROSBAG_PATH $CAPTURE_SCREEN $LOG_DATA $LOG_PATH 0 '/os1_cloud_node/imu';

OXFORD_SEQ=07_parkland_mound
ROSBAG_PATH=$DATASET_LOC/$OXFORD_SEQ/${OXFORD_SEQ}.bag
LOG_PATH=$EPOC_DIR/result_$OXFORD_SEQ
./run_one_bag.sh $LAUNCH_FILE $ROS_PKG $ROSBAG_PATH $CAPTURE_SCREEN $LOG_DATA $LOG_PATH 0 '/os1_cloud_node/imu';

#endregion Enable loop closure --------------------------------------------------------------------------------------#

