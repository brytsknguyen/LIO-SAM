catkin build slict ;
source /home/$USER/dev_ws/devel/setup.bash

# Get the current directory
CURR_DIR=$(pwd)
echo CURRENT DIR: $CURR_DIR

# Get the location of the slict package
ROS_PKG=slict
echo SLICT DIR: $(rospack find $ROS_PKG)

CAPTURE_SCREEN=true;
LOG_DATA=true;

LAUNCH_FILE=run_wkw.launch
DATASET_LOC=/media/$USER/mySataSSD3/ATVCollectedWKWNoDepth


EPOC_DIR=/home/$USER/MATLAB_WS/SLICT/wkw_20221017_noloop
#region Disable loop closure ---------------------------------------------------------------------------------------#


# Daytime sequences
# ATV_SEQ=daytime_01
# ROSBAG_PATH=$DATASET_LOC/${ATV_SEQ}.bag
# LOG_PATH=$EPOC_DIR/result_$ATV_SEQ
# ./run_one_bag.sh $LAUNCH_FILE $ROS_PKG $ROSBAG_PATH $CAPTURE_SCREEN $LOG_DATA $LOG_PATH 0 '/imu_vn_100/imu';

# ATV_SEQ=daytime_02
# ROSBAG_PATH=$DATASET_LOC/${ATV_SEQ}.bag
# LOG_PATH=$EPOC_DIR/result_$ATV_SEQ
# ./run_one_bag.sh $LAUNCH_FILE $ROS_PKG $ROSBAG_PATH $CAPTURE_SCREEN $LOG_DATA $LOG_PATH 0 '/imu_vn_100/imu';

ATV_SEQ=daytime_03
ROSBAG_PATH=$DATASET_LOC/${ATV_SEQ}.bag
LOG_PATH=$EPOC_DIR/result_$ATV_SEQ
./run_one_bag.sh $LAUNCH_FILE $ROS_PKG $ROSBAG_PATH $CAPTURE_SCREEN $LOG_DATA $LOG_PATH 0 '/imu_vn_100/imu';

ATV_SEQ=daytime_04
ROSBAG_PATH=$DATASET_LOC/${ATV_SEQ}.bag
LOG_PATH=$EPOC_DIR/result_$ATV_SEQ
./run_one_bag.sh $LAUNCH_FILE $ROS_PKG $ROSBAG_PATH $CAPTURE_SCREEN $LOG_DATA $LOG_PATH 0 '/imu_vn_100/imu';

ATV_SEQ=daytime_05
ROSBAG_PATH=$DATASET_LOC/${ATV_SEQ}.bag
LOG_PATH=$EPOC_DIR/result_$ATV_SEQ
./run_one_bag.sh $LAUNCH_FILE $ROS_PKG $ROSBAG_PATH $CAPTURE_SCREEN $LOG_DATA $LOG_PATH 0 '/imu_vn_100/imu';


# Nighttime sequences
ATV_SEQ=nighttime_01
ROSBAG_PATH=$DATASET_LOC/${ATV_SEQ}.bag
LOG_PATH=$EPOC_DIR/result_$ATV_SEQ
./run_one_bag.sh $LAUNCH_FILE $ROS_PKG $ROSBAG_PATH $CAPTURE_SCREEN $LOG_DATA $LOG_PATH 0 '/imu_vn_100/imu';

ATV_SEQ=nighttime_02
ROSBAG_PATH=$DATASET_LOC/${ATV_SEQ}.bag
LOG_PATH=$EPOC_DIR/result_$ATV_SEQ
./run_one_bag.sh $LAUNCH_FILE $ROS_PKG $ROSBAG_PATH $CAPTURE_SCREEN $LOG_DATA $LOG_PATH 0 '/imu_vn_100/imu';

ATV_SEQ=nighttime_03
ROSBAG_PATH=$DATASET_LOC/${ATV_SEQ}.bag
LOG_PATH=$EPOC_DIR/result_$ATV_SEQ
./run_one_bag.sh $LAUNCH_FILE $ROS_PKG $ROSBAG_PATH $CAPTURE_SCREEN $LOG_DATA $LOG_PATH 0 '/imu_vn_100/imu';

ATV_SEQ=nighttime_04
ROSBAG_PATH=$DATASET_LOC/${ATV_SEQ}.bag
LOG_PATH=$EPOC_DIR/result_$ATV_SEQ
./run_one_bag.sh $LAUNCH_FILE $ROS_PKG $ROSBAG_PATH $CAPTURE_SCREEN $LOG_DATA $LOG_PATH 0 '/imu_vn_100/imu';

ATV_SEQ=nighttime_05
ROSBAG_PATH=$DATASET_LOC/${ATV_SEQ}.bag
LOG_PATH=$EPOC_DIR/result_$ATV_SEQ
./run_one_bag.sh $LAUNCH_FILE $ROS_PKG $ROSBAG_PATH $CAPTURE_SCREEN $LOG_DATA $LOG_PATH 0 '/imu_vn_100/imu';


#endregion Enable loop closure -------------------------------------------------------------------------------------#



EPOC_DIR=/home/$USER/MATLAB_WS/SLICT/wkw_20221017_looped
#region Enable loop closure ----------------------------------------------------------------------------------------#


# # Daytime sequences
# ATV_SEQ=daytime_01
# ROSBAG_PATH=$DATASET_LOC/${ATV_SEQ}.bag
# LOG_PATH=$EPOC_DIR/result_$ATV_SEQ
# ./run_one_bag.sh $LAUNCH_FILE $ROS_PKG $ROSBAG_PATH $CAPTURE_SCREEN $LOG_DATA $LOG_PATH 1 '/imu_vn_100/imu';

# ATV_SEQ=daytime_02
# ROSBAG_PATH=$DATASET_LOC/${ATV_SEQ}.bag
# LOG_PATH=$EPOC_DIR/result_$ATV_SEQ
# ./run_one_bag.sh $LAUNCH_FILE $ROS_PKG $ROSBAG_PATH $CAPTURE_SCREEN $LOG_DATA $LOG_PATH 1 '/imu_vn_100/imu';

# ATV_SEQ=daytime_03
# ROSBAG_PATH=$DATASET_LOC/${ATV_SEQ}.bag
# LOG_PATH=$EPOC_DIR/result_$ATV_SEQ
# ./run_one_bag.sh $LAUNCH_FILE $ROS_PKG $ROSBAG_PATH $CAPTURE_SCREEN $LOG_DATA $LOG_PATH 1 '/imu_vn_100/imu';

# ATV_SEQ=daytime_04
# ROSBAG_PATH=$DATASET_LOC/${ATV_SEQ}.bag
# LOG_PATH=$EPOC_DIR/result_$ATV_SEQ
# ./run_one_bag.sh $LAUNCH_FILE $ROS_PKG $ROSBAG_PATH $CAPTURE_SCREEN $LOG_DATA $LOG_PATH 1 '/imu_vn_100/imu';

# ATV_SEQ=daytime_05
# ROSBAG_PATH=$DATASET_LOC/${ATV_SEQ}.bag
# LOG_PATH=$EPOC_DIR/result_$ATV_SEQ
# ./run_one_bag.sh $LAUNCH_FILE $ROS_PKG $ROSBAG_PATH $CAPTURE_SCREEN $LOG_DATA $LOG_PATH 1 '/imu_vn_100/imu';


# # Nighttime sequences
# ATV_SEQ=nighttime_01
# ROSBAG_PATH=$DATASET_LOC/${ATV_SEQ}.bag
# LOG_PATH=$EPOC_DIR/result_$ATV_SEQ
# ./run_one_bag.sh $LAUNCH_FILE $ROS_PKG $ROSBAG_PATH $CAPTURE_SCREEN $LOG_DATA $LOG_PATH 1 '/imu_vn_100/imu';

# ATV_SEQ=nighttime_02
# ROSBAG_PATH=$DATASET_LOC/${ATV_SEQ}.bag
# LOG_PATH=$EPOC_DIR/result_$ATV_SEQ
# ./run_one_bag.sh $LAUNCH_FILE $ROS_PKG $ROSBAG_PATH $CAPTURE_SCREEN $LOG_DATA $LOG_PATH 1 '/imu_vn_100/imu';

# ATV_SEQ=nighttime_03
# ROSBAG_PATH=$DATASET_LOC/${ATV_SEQ}.bag
# LOG_PATH=$EPOC_DIR/result_$ATV_SEQ
# ./run_one_bag.sh $LAUNCH_FILE $ROS_PKG $ROSBAG_PATH $CAPTURE_SCREEN $LOG_DATA $LOG_PATH 1 '/imu_vn_100/imu';

# ATV_SEQ=nighttime_04
# ROSBAG_PATH=$DATASET_LOC/${ATV_SEQ}.bag
# LOG_PATH=$EPOC_DIR/result_$ATV_SEQ
# ./run_one_bag.sh $LAUNCH_FILE $ROS_PKG $ROSBAG_PATH $CAPTURE_SCREEN $LOG_DATA $LOG_PATH 1 '/imu_vn_100/imu';

# ATV_SEQ=nighttime_05
# ROSBAG_PATH=$DATASET_LOC/${ATV_SEQ}.bag
# LOG_PATH=$EPOC_DIR/result_$ATV_SEQ
# ./run_one_bag.sh $LAUNCH_FILE $ROS_PKG $ROSBAG_PATH $CAPTURE_SCREEN $LOG_DATA $LOG_PATH 1 '/imu_vn_100/imu';


#endregion Enable loop closure -------------------------------------------------------------------------------------#