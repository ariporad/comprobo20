# How to Run CompRobo (ROS1 & Gazebo et al.) on a Mac

> These instructions have been tested on a 2021 Macbook Pro with an M1 Pro chip, running macOS 12.0.1.

## Overview
- Everything _except the Gazebo UI_ runs in a Parallels VM (mine is Ubuntu 20.04).
- For Gazebo, the gzserver runs in the VM and the UI (gzclient) runs natively on macOS.

## Setup Details

> **Note:** On my machine, the VM is named `ubuntu` (in Parallels), and my CompRobo repo lives (on my Mac) at `/Users/ariporad/work/CompRobo/catkin_ws/src/comprobo20`. If your VM name/path are different, this all should still work but you'll need to substitute appropriately.

- For the most part, just setup a Parallels VM running Ubuntu the normal way. Make sure the following is true:
	- Under Parallels Preferences > Hardware > Graphics > Advanced, 3D Acceleration is **ON**.
	- It's best if your Ubuntu username is the same as your Mac username, although probably not required.
	- Under Parallels Preferences > Options > Sharing > Custom Folders, select the `comprobo20` folder (on your Mac). Leave the name as-is and make sure the permissions are read-write.
		- This will mount your the folder to `/media/psf/comprobo20` in Ubuntu.
	- Under Parallels Preferences > Network > Shared > Port Forwarding, use the following:
		- `Source: 2222 -> Destination: 22` (SSH)
		- `Source 11345 -> Destination: 11345` (Gazebo)
		- `Source 11311 -> Destination: 11311` (ROS)
	- **ProTip:** You can run commands in the VM either by opening a GUI terminal in Ubuntu, or by SSHing in from your Mac with `ssh ubuntu`.
- Install ROS in the VM as you normally would. ([CompRobo 2020 Instructions](https://comprobo20.github.io/How%20to/setup_your_environment))
	- Skip the the NVIDIA section.
	- Note that the CompRobo 2020 setup instructions are for an all-virtual class. To get the physical Neatos working (_Thanks Paul!_) you'll need to also run `sudo apt install libgstreamer*1.0* ros-noetic-pointcloud-to-laserscan hping3` and `sudo setcap cap_net_raw+ep /usr/sbin/hping3` before (re-)running `catkin_make`
- Now here comes the weird bit: _In the VM_, make empty folders that correspond to the path of the `comprobo20` folder _on your Mac_: `mkdir -p /Users/ariporad/work/CompRobo/catkin_ws/src`.
	- This is a weird thing to do. (On Mac, `/Users/username` is the home directory, but it's `/home/username` on Linux.) However, running the Gazebo client on the Mac side _will not work_ without it.
		_ **Why?** Gazebo expects model files to be in the same _absolute path_ on the client and the server. This is a bad idea on their part, but that's life. Here, we're making sure that the paths in Ubuntu will match the paths in macOS.
			- We do this on the VM side since we care much less about messing/dirtying up the VM than our real computer.
	- Now, symlink the mounted repo into this directory: `ln -s /media/psf/comprobo20 comprobo20`.
		- If you want you can also link it to the Ubuntu home directory: `ln -s /Users/ariporad/work/CompRobo/catkin_ws ~/catkin_ws`
- Have the VM's `.bashrc` file source the setup file in this repo: `echo "source /Users/ariporad/work/CompRobo/catkin_ws/src/comprobo20/setup_vm.sh" >> ~/.bashrc`

## To Launch
- Make sure the VM is running in Parallels
- Just run commands normally (you can `ssh ubuntu` from macOS to run commands)
- If you want to use RViz, you'll need to use the VM's GUI
- I suggest editing using VS Code's remote SSH functionality to the VM

## To Run Gazebo

> Don't forget to run `roscore` before doing anything else!

- In the VM, in seperate terminal windows, run:
	- Whatever roslaunch command you want to run, but with the `gui` option set to `false`
		- **Example:** `roslaunch neato_gazebo neato_gauntlet_world.launch gui:=false`
- On macOS:
	- Launch Gazebo using the `gzclient_mac.sh` script in this repo

## To Connect to a Real Neato

> Don't forget to run `roscore` before doing anything else!

- Turn the Neato on
- Run `roslaunch neato_node bringup_minimal.launch host:=<Neato IP address>`