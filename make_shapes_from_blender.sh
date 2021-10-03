#!/usr/bin/env bash

# Sphere
blender blender/blank.blend --background --python blender/make_sphere_blender.py
pcl_mesh_sampling -no_vis_result -n_samples 1024 data/basic_sphere.obj data/basic_sphere.pcd

# Cube
blender blender/blank.blend --background --python blender/make_cube_blender.py
pcl_mesh_sampling -no_vis_result -n_samples 1024 data/basic_cube.obj data/basic_cube.pcd

# Tetrahedron
blender blender/blank.blend --background --python blender/make_tetrahedron_blender.py
pcl_mesh_sampling -no_vis_result -n_samples 1024 data/tetrahedron.obj data/tetrahedron.pcd
