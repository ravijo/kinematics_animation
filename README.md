# kinematics_animation
A robotic manipulator visualization and animation tool based on kinematics

## Dependencies
* scipy `pip install --upgrade scipy`

## Installation
1. Change the directory to src folder of ros workspace
2. Clone the repo (or download accordingly)
3. After successful cloning, go back to ros workspace
4. Run `catkin_make` command 

## Run (baxter animation)
1. Change the directory to ros workspace
2. Make sure to source the setup.bash ros workspace file by following command `source devel/setup.bash`
3. Use `roslaunch` to invoke animation `roslaunch kinematics_animation baxter.launch`
4. Alternativily, the csv file can be provided as command line argument `roslaunch kinematics_animation baxter.launch file:=/home/ravi/Desktop/joint_states.csv`

## Add a new robot
1. Create a launch file, similar to existing launch file located inside luanch directory
2. Change the `file` argument as per new robot
3. Provide the `urdf.xacro` file of the new robot as `robot_description` parameter
4. Provide the rviz file as per new robot under `rviz` package
5. Create a new python script to read the input csv file
6. The newly created python script must publish all the joint state
