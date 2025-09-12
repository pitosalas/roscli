# Claude instructions for roscli 


* Code is python
* Uses only ROS2
* functions and methods no longer than 50 lines
* Write idiomatic python
* Look for libraries that exist instead of reinventing
* Don't go overboard on error checking
* Give methods intention revealing names
* Use classes and put them in a seprate file
* Put data classes in the file where they are constructed
* name files after the class defined in the file
* no files with more than about 300 lines
* use python latest and ros2 compliant package management and building with colcon
* prefer async/await over threading when there is a choice
* avoid if/else statements that are nested more than 1 deep
* avoid 1 line methods
* look for code duplication and make the code DRY if it makes sense

