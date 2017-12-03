		##################################################################
		##								##
		##	 	ROS Jade installation script			##
		##								##
		##			KgpKubs - 2017				##
		##################################################################

####################### For proxy 
# KGP_PROXY="10.3.100.207"
# KGP_PORT="8080"
# export http_proxy="http://$KGP_PROXY:$KGP_PORT"
# export https_proxy="https://$KGP_PROXY:$KGP_PORT"
#######################

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo -E apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo -E apt-get -qq --yes update
sudo -E apt-get -qq install ros-jade-desktop-full

sudo -E rosdep init
rosdep update

echo "source /opt/ros/jade/setup.bash" >> ~/.bashrc
source ~/.bashrc

