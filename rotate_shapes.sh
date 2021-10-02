#!/usr/bin/env bash

source devel/setup.bash

rm -rf data/rotated

mkdir -p data/rotated/sphere
rosrun rotate_pcd rotate_pcd data/basic_sphere.pcd data/rotated/sphere

mkdir -p data/rotated/cube
rosrun rotate_pcd rotate_pcd data/basic_cube.pcd data/rotated/cube
