# kinematics_animation
A robotic manipulator visualization and animation tool based on kinematics

## Dependencies
* scipy `pip install --upgrade scipy`

## Installation
1. Clone the repository (or download accordingly) to src folder of ros workspace
1. Go back to ros workspace then run `catkin_make` command 
1. Please run `roscd kinematics_animation/scripts && chmod +x *.py` to make python scripts exectuable 

## Run (baxter animation)
1. Change the directory to ros workspace
1. Make sure to source the `setup.bash` ros workspace. Alternativily, run the following command `source devel/setup.bash`
1. Use `roslaunch` to invoke animation `roslaunch kinematics_animation baxter.launch`
    1. Alternativily, the csv file can be provided as command line argument `roslaunch kinematics_animation baxter.launch file:=/home/ravi/Desktop/joint_states.csv`

## Add a new robot
1. Create a launch file, similar to existing launch file located inside `launch` directory
1. Change the `file` argument as per new robot
1. Provide the `urdf.xacro` file of the new robot as `robot_description` parameter
1. Provide the rviz file as per new robot under `rviz` package
1. Create a new python script to read the input csv file. Make sure that the newly created python script must publish all the joint state

## Issues (or Error Reporting)
Please check [here](https://github.com/ravijo/kinematics_animation/issues) and create issues accordingly.
