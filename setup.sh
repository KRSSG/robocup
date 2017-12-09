#! /bin/bash
set -e
if [ "$#" -ne 1 ]; then
  echo "usage: ./install.sh <path-to-installation>"
  exit
fi

# Install dependencies
workspace_dir=$1

echo "Installing Dependencies"
sudo apt-get -qq --yes --force-yes update
sudo apt-get -qq --yes --force-yes install build-essential
sudo add-apt-repository -y ppa:ubuntu-toolchain-r/test
sudo apt-get -qq --yes --force-yes update
sudo apt-get -qq --yes --force-yes install "g++-4.9"
sudo apt-get -qq --yes --force-yes install cmake qt5-default libqt5svg5-dev libprotobuf-dev protobuf-compiler libode-dev screen

mkdir temp_dir && cd temp_dir
wget https://storage.googleapis.com/google-code-archive-downloads/v2/code.google.com/vartypes/vartypes-0.7.tar.gz
tar xfz vartypes-0.7.tar.gz
cd vartypes-0.7
mkdir build && cd build
cmake ..
make --quiet -j8
sudo make install
cd ../../../
rm -rf temp_dir

# Install latest cmake
echo "Installing latest cmake"
mkdir temp_dir && cd temp_dir
wget https://cmake.org/files/v3.8/cmake-3.8.0.tar.gz
tar xf cmake-3.8.0.tar.gz
cd cmake-3.8.0
./configure
make --quiet -j8
sudo make install
cd ../../
rm -rf temp_dir

# Call the ros-install script here
echo "Installing ROS"
sudo chmod +x ros_install.sh
bash ros_install.sh

cd $workspace_dir
source "/opt/ros/jade/setup.bash"

cd src
