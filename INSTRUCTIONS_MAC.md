# How to Run Gazebo on a Mac

- ROS and gzserver run in an Ubuntu 20.04 VM under Parallels
	- 3D acceleration should be on
- gzclient runs natively under macOS

## To Launch

- Make sure the VM is running
- Connect using VS Code (connect over ssh to localhost:2222)
- In the VM (VS Code prefered), in seperate terminal windows, run:
	- `roscore`
	- Whatever roslaunch command you want to run, but with the `gui` option set to `false`
		- **Example:** `roslaunch neato_gazebo neato_gauntlet_world.launch gui:=false`
- On macOS:
	- Launch Gazebo: `GAZEBO_MASTER_IP=127.0.0.1:11345 gzclient`
		- You'll need to relaunch this if you restart the gazebo server

