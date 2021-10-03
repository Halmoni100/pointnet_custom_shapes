#!/usr/bin/env bash

source devel/setup.bash

rm -rf data/transformed

mkdir -p data/transformed/sphere
rosrun transform_pcd transform_pcd \
  --input data/basic_sphere.pcd \
  --output data/transformed/sphere \
  --rotate 7 \
  --rotationsRatio 0.8 \
  --pertubationRatio 0.25

mkdir -p data/transformed/cube
rosrun transform_pcd transform_pcd \
  --input data/basic_cube.pcd \
  --output data/transformed/cube \
  --rotate 7 \
  --rotationsRatio 0.8 \
  --pertubationRatio 0.25

mkdir -p data/transformed/flat_square
rosrun transform_pcd transform_pcd \
  --input data/basic_cube.pcd \
  --output data/transformed/flat_square \
  --rotate 7 \
  --rotationsRatio 0.8 \
  --pertubationRatio 0.25 \
  --numX 1 \
  --minX 0.2 \
  --maxX 0.2

mkdir -p data/transformed/tetrahedron
rosrun transform_pcd transform_pcd \
  --input data/tetrahedron.pcd \
  --output data/transformed/tetrahedron \
  --rotate 7 \
  --rotationsRatio 0.8 \
  --pertubationRatio 0.25
