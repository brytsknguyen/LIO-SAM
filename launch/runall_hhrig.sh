catkin_make -C /home/$USER/dev_ws ;
source /home/$USER/dev_ws/devel/setup.bash


# Get the current directory
CURR_DIR=$(pwd)
# Get the location of the viral package
roscd lio_sam
PACKAGE_DIR=$(pwd)
# Return to the current dir, print the directions
cd $CURR_DIR
echo CURRENT DIR: $CURR_DIR
echo VIRAL DIR:   $PACKAGE_DIR

export EPOC_DIR=/home/$USER/nucssd2/MATLAB_WS/RAL_VIRAL_SLAM/hhrig_liosam
export DATASET_LOCATION=/home/$USER/DATASETS/hhrig/
# export DATASET_LOCATION=/media/$USER/myHPSSD/hhrig

export CAPTURE_SCREEN=false;
export LOG_DATA=false;

#region 0 UWB NO VIS --------------------------------------------------------------------------------------------------

# wait;
# ./run_one_bag_hhrig.sh $EPOC_DIR $DATASET_LOCATION $PACKAGE_DIR iot_01 $CAPTURE_SCREEN $LOG_DATA 450 0 0 0.75 -1;
# wait;
# ./run_one_bag_hhrig.sh $EPOC_DIR $DATASET_LOCATION $PACKAGE_DIR iot_02 $CAPTURE_SCREEN $LOG_DATA 450 0 0 0.75 -1;


# wait;
# ./run_one_bag_hhrig.sh $EPOC_DIR $DATASET_LOCATION $PACKAGE_DIR ac_01  $CAPTURE_SCREEN $LOG_DATA 450 0 0 0.75 -1;
# wait;
# ./run_one_bag_hhrig.sh $EPOC_DIR $DATASET_LOCATION $PACKAGE_DIR ac_02  $CAPTURE_SCREEN $LOG_DATA 450 0 0 0.75 -1;
wait;
./run_one_bag_hhrig.sh $EPOC_DIR $DATASET_LOCATION $PACKAGE_DIR ac_03  $CAPTURE_SCREEN $LOG_DATA 450 0 0 0.75 -1;


# wait;
# ./run_one_bag_hhrig.sh $EPOC_DIR $DATASET_LOCATION $PACKAGE_DIR ac_walk_01  $CAPTURE_SCREEN $LOG_DATA 450 0 0 0.75 -1;
# wait;
# ./run_one_bag_hhrig.sh $EPOC_DIR $DATASET_LOCATION $PACKAGE_DIR ac_walk_02  $CAPTURE_SCREEN $LOG_DATA 450 0 0 0.75 -1;

#endregion NO UWB NO VIS ----------------------------------------------------------------------------------------------


#region ## Poweroff ---------------------------------------------------------------------------------------------------

wait;
# poweroff

#endregion ## Poweroff ------------------------------------------------------------------------------------------------

