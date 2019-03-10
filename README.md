# RobocupSSL
## Requirement
* Ubuntu 16.04
* ROS-Kinetic
* OMPL
* Python-memcache
```bash
$ sudo apt install memcached python-memcache
```
* PyQt4
## Run 
```bash
$ ./kgpkubs_launch/scripts/ssl.sh
$ python bs.py
$ python run_gui.py
$ rosrun ompl_planner listener_ompl 
$ python test_role.py (or  python test_tactic.py)
```
in five different terminals !!
