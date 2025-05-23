# my_robot_bringup
 my_robot_gazebo.launch.py file requires the my_robot_description package. This is because the following lines load the robot model from that package:  The launch file uses the URDF/Xacro file from my_robot_description to publish the robot model and spawn it in Gazebo. You must have the my_robot_description package in your workspace for this launch file to work.

 ![image](https://github.com/user-attachments/assets/1646947d-643d-4157-bad1-2490ba12b109)
![image](https://github.com/user-attachments/assets/62717f21-8846-411f-b60f-ae4e83d6e753)


🛠️ my_robot_bringup

This package provides launch files and configurations to initialize and simulate a mobile robot in Gazebo and RViz2 within the ROS 2 Humble environment. It supports joystick teleoperation and integrates with the my_robot_description package for robot model definitions.

📁 Directory Structure


my_robot_bringup/

├── launch/

│   ├── my_robot_gazebo.launch.py   # Launches Gazebo simulation

│   └── my_robot_rviz.launch.py     # Launches RViz2 visualization

├── rviz/

│   └── my_robot.rviz               # RViz2 configuration file

├── worlds/

│   └── my_world.world              # Custom Gazebo world file

├── CMakeLists.txt

├── package.xml

└── README.md

🚀 Installation & Build

Clone the repository into your ROS 2 workspace:


cd ~/ros2_ws/src

git clone https://github.com/seyit05/my_robot_bringup.git

Ensure that the my_robot_description package is also present in your workspace:


git clone https://github.com/seyit05/my_robot_description.git

Install dependencies:

cd ~/ros2_ws

rosdep install --from-paths src --ignore-src -r -y

Build the workspace:

colcon build

source install/setup.bash

🧪 Usage

Launch Gazebo Simulation


ros2 launch my_robot_bringup my_robot_gazebo.launch.py

This command will spawn the robot model in the Gazebo environment using the specified world file.

Launch RViz2 Visualization


ros2 launch my_robot_bringup my_robot_rviz.launch.py

This command will open RViz2 with the pre-configured settings to visualize the robot's state and sensor data.


🎮 Joystick Teleoperation

To control the robot using a joystick:

GitHub

Ensure that a joystick is connected to your system.

Install the joy and teleop_twist_joy packages if not already installed:

sudo apt install ros-humble-joy ros-humble-teleop-twist-joy

Launch the joystick driver:


ros2 run joy joy_node

Launch the teleoperation node:


ros2 run teleop_twist_joy teleop_node

Ensure that the cmd_vel topic from the teleoperation node is correctly remapped to the robot's velocity command topic if necessary.

🔧 Dependencies

ROS 2 Humble (Ubuntu 22.04)

my_robot_description package

gazebo_ros

robot_state_publisher

joint_state_publisher_gui

rviz2

joy

teleop_twist_joy


Install missing dependencies using:


sudo apt install ros-humble-gazebo-ros-pkgs \
                 ros-humble-robot-state-publisher \
                 ros-humble-joint-state-publisher-gui \
                 ros-humble-rviz2 \
                 ros-humble-joy \
                 ros-humble-teleop-twist-joy
                 
📄 License

This project is licensed under the MIT License. See the LICENSE file for details.

Feel free to integrate this README.md into your repository. If you need further customization or additional features, don't hesitate to ask!
