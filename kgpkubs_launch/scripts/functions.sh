# functions.lib
# Includes the set of common functions that can be used

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
SETUP_DIR=$DIR/../../../devel/setup.bash

launcher()
{
   if screen -ls | grep -q $1; then
      echo "WARNING! $1 is already running!"
   else
      screen -S $1 -d -m bash
      screen -S $1 -p 0 -X stuff "source $DIR/../../../devel/setup.bash; $2$(printf \\r)"
   fi
}

start_python_screen() 
{
   SCREEN_NAME=$1
   BASE_DIR=$2
   FILE_NAME=$3

   if screen -ls | grep -q $1; then
      echo "WARNING! screen \"${SCREEN_NAME}\" is already running!"
   else
      echo "Starting the screen \"${SCREEN_NAME}\""
      screen -S ${SCREEN_NAME} -d -m bash
      screen -r ${SCREEN_NAME} -p 0 -X stuff "source $SETUP_DIR; $cd ${BASE_DIR}; python ${FILE_NAME}$(printf \\r)"
   fi
}

start_ros_launch_file()
{
   SCREEN_NAME=$1
   LAUNCH_FILE=$2

   if screen -ls | grep -q $1; then
      echo "WARNING! ${SCREEN_NAME} is already running!"
   else
      echo "Starting the screen ${SCREEN_NAME}"
      screen -S ${SCREEN_NAME} -d -m bash
      screen -r ${SCREEN_NAME} -p 0 -X stuff "source $SETUP_DIR; roslaunch kgpkubs_launch $LAUNCH_FILE$(printf \\r)"
   fi
}

start_ros_node()
{
   SCREEN_NAME=$1
   PACKAGE_NAME=$2
   NODE_NAME=$3

   if screen -ls | grep -q $1; then
      echo "WARNING! ${SCREEN_NAME} is already running!"
   else
      echo "Starting the screen ${SCREEN_NAME}"
      screen -S ${SCREEN_NAME} -d -m bash
      screen -r ${SCREEN_NAME} -p 0 -X stuff "source $SETUP_DIR; rosrun ${PACKAGE_NAME} ${NODE_NAME}$(printf \\r)"
   fi
}
