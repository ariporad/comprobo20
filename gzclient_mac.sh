#!/usr/bin/env bash

##
# Launch gzclient on macOS, connecting to gzserver running in a local Ubuntu VM.
#
# Make sure your system is configured as described in INSTRUCTIONS_MAC.md.
##

COMPROBO=$(cd $(dirname ${BASH_SOURCE}) > /dev/null; pwd)

export GAZEBO_MODEL_PATH=$COMPROBO/*/model

source "$(brew --prefix gazebo11)/share/gazebo-11/setup.bash"

echo "Starting Gazbeo... (CompRobo: $COMPROBO)"
echo ""

if [[ "$*" != *"--verbose"* ]]; then
	echo "ProTip: You probably want to run this script with --verbose to make sure it's working right."
	echo ""
fi

echo "NOTE: If you restart gzserver in the VM, you'll need to restart this."
echo ""


GAZEBO_MASTER_IP=127.0.0.1:11345 gzclient $@

echo "Gazebo Finished"