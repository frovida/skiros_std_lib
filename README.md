### Skill based system for ROS (SkiROS) standard library from the RVMI lab, Aalborg University Copenhagen, Denmark

[www.rvmi.aau.dk(RVMI webpage)](http://homes.m-tech.aau.dk/mrp/skiros/)

Last readME update: 25/05/2016  

The repository is a collection of plugins for the SkiROS system, for more information refer to the SkiROS package readME.  

This collection of plugins provide SkiROS with a minimal set to try out the framework. 

### Included (meta)packages 
* **skiros_std_lib**: holds launch file to launch a skiros with fake skills and an example world model. script to install dependencies. (meta-package)  
* **skiros_lib_modules**: holds locate dummy module
* **skiros_lib_skill_dummy**: holds drive, pick and place, dummy skills
* **skiros_lib_reasoner**: holds aau spatial reasoner, to reason about poses and sizes
* **task_planners**: holds fast downward planner implementation
* **utils**: holds support packages

### Dependencies
* [**skiros**](git@git.rvmi.aau.dk:aau-projects/skiros.git) 
* [**downward**](http://gki.informatik.uni-freiburg.de/tools/tfd/downloads.html) task planner

### Install
Run the script "skiros_std_lib/scripts/install_dependencies.sh directory" where "directory" should specify the install directory

### Quick start
These plugins can be executed into SkiROS. For an example on how to launch it, refer to the SkiROS package.



