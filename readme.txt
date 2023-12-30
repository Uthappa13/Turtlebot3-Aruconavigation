1. ENPM809Y: RWA3 (Group-8)
	The goal of the assignment is to make the turtlebot waffle navigate through the provided maze based on the instructions given.

2. Instructions:
	(a) After putting the group8 package in the required workspace, build the workspace and launch the maze provided.
		ros2 launch turtlebot3_gazebo maze.launch.py
	(b) In a new terminal, launch the broadcaster using the below code
		ros2 run group8 broadcaster
	(b) After getting the message that the broadcaster has started, in a new terminal, launch the waffle.launch.py launch file to run all the nodes for the turtlebot to successfully complete the maze.
		ros2 launch group8 waffle.launch.py

	