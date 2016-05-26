### Skill based system for ROS (SkiROS) standard library from the RVMI lab, Aalborg University Copenhagen, Denmark

[www.rvmi.aau.dk(RVMI webpage)](http://homes.m-tech.aau.dk/mrp/skiros/)

Last readME update: 25/05/2016  

**Compatibility**: Has been tested with Ubuntu 14.04 and ROS Indigo.

The repository is a collection of plugins for the SkiROS system, for more information refer to the SkiROS package readME.  

This repository provide SkiROS with a minimal collection of plugins to try out the framework. In particular, it is provided a task planner, a reasoner for spatial relations, a set of fake skills (drive, pick and place) and a fake module to locate objects.

### Included (meta)packages 
* **skiros_std_lib**: holds launch file to launch a skiros with fake skills and an example world model. script to install dependencies. (meta-package)  
* **skiros_lib_modules**: holds locate dummy module
* **skiros_lib_skill_dummy**: holds drive, pick and place, dummy skills
* **skiros_lib_reasoner**: holds aau spatial reasoner, to reason about poses and sizes
* **task_planners**: holds fast downward planner implementation
* **utils**: holds support packages

### Dependencies
* [**skiros**](git@git.rvmi.aau.dk:aau-projects/skiros.git) core framework
* [**fast-downward**](http://gki.informatik.uni-freiburg.de/tools/tfd/downloads.html) task planner

### Install

To install the fast-downward planner, user can run the script "skiros_std_lib/scripts/install_dependencies.sh", specifing the folder of installation (note: this should never change afterwards).

The packages can be compiled using catkin.

### Quick start
These plugins can be loaded into the SkiROS framework. To launch the system:

* roslaunch skiros_std_lib skiros_system_fake_skills.launch


