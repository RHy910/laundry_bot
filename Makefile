# ROS2 Makefile — place in ~/ros2_ws/
SHELL := /bin/bash
ROS_DISTRO ?= humble
PACKAGE     := laundry_dt
LAUNCH_FILE := launch.py

SOURCE := source /opt/ros/$(ROS_DISTRO)/setup.bash && source install/setup.bash

.PHONY: go build launch clean rebuild

## Build + source + launch in one command
go:
	source /opt/ros/$(ROS_DISTRO)/setup.bash && \
	colcon build --packages-select $(PACKAGE) && \
	$(SOURCE) && \
	ros2 launch $(PACKAGE) $(LAUNCH_FILE)

## Build only
build:
	source /opt/ros/$(ROS_DISTRO)/setup.bash && \
	colcon build --packages-select $(PACKAGE)

## Launch only (assumes already built)
launch:
	$(SOURCE) && \
	ros2 launch $(PACKAGE) $(LAUNCH_FILE)

## Full clean rebuild then launch
rebuild:
	rm -rf build/ install/ log/
	source /opt/ros/$(ROS_DISTRO)/setup.bash && \
	colcon build --packages-select $(PACKAGE) && \
	$(SOURCE) && \
	ros2 launch $(PACKAGE) $(LAUNCH_FILE)

## Remove build artifacts
clean:
	rm -rf build/ install/ log/