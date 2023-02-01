#!/bin/sh
cd catkin_ws
sudo cp -r src/simple_scene/worlds/* /usr/share/gazebo-11/worlds/
sudo cp -r src/simple_scene/models/* ~/.gazebo/models/
git submodule init
git submodule update
cd ..