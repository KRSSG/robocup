# RobocupSSL [![Build Status](https://travis-ci.com/KRSSG/robocup.svg?branch=fsm)](https://travis-ci.com/KRSSG/robocup)
## Requirement
* Ubuntu 16.04
* ROS-Kinetic
* OMPL
* PyQt4

## Installation
* Follow this doc for installation instructions: https://docs.google.com/document/d/1dslgGWAXHz8jKqUmEGex7JvCxIh3FS-td155nmqGrdw/edit?usp=sharing
* Install OMPL without python bindings from http://ompl.kavrakilab.org/installation.html
* Install Protobuf version 2.6

## Run 
```bash
$ ./kgpkubs_launch/scripts/ssl.sh
$ python test_role.py (or  python test_tactic.py)
```

## Documentation
We use Doxygen to document our code and build a website that help us to browse through code. You can build documentation of codebase using `$ doxygen Doxyfile` and browse documentation at `Documentation/html/index.html`. 
