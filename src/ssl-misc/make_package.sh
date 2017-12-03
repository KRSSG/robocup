#!/bin/bash

#################################################################
#								#
#			KgpKubs - 2017				#
#								#
# Create a new ROS package with gitignore, travis and more	#
# To use:							#
# 		./make_package.sh <pkg_name> <pkg_type>		#
#								#
#	* <pkg_name> ->		Name of package 		#
#	* <pkg_type> ->		Type of package -> "cpp" / "py"	#
#								#
#################################################################

PKG_DIR=$1
PKG_TYPE=$2

cd ..

if [ $PKG_TYPE == "py" ]; then
	PKG_DIR="${PKG_DIR}_py"
fi

catkin_create_pkg $PKG_DIR roscpp rospy std_msgs
cd $PKG_DIR

#initialise with git
curl https://raw.githubusercontent.com/github/gitignore/master/ROS.gitignore > .gitignore
git init

#add README.md
touch README.md

#add type specific files
if [ $PKG_TYPE == "py" ]; then
	rm -rf include/
	rm -rf src/
	mkdir scripts/
fi

#run git
git add *
git add .gitignore
git commit -m "Initial commit"

cd ..
