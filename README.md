# Grasp Planning Engine

## Usage
Compile and launch the program as exemplified in the following code block, where ```--opt arg``` is to be substituted with the program options tabulated below.
```
colcon build
./build/grasp-planning-engine/gp_node --opt arg
```

Option | Args | Description
--- |--- | ---
  |--help |  | Print help message
  |--global_analysis |  | Perform a global graspability analysis |
  |--local_analysis | {x,y,z} | Perform a local analysis at a defined target 3D point. Coordinates in [m] of type float wrapped in curly braces {} and with no spaces, i.e. {0.1,-9.0,3} |
  |--gp_algorithm | uint | Select grasp planning algorithm to use in idx range [1-4] |
  |--target | string | Target tree filepath |
  |--gripper | string | Gripper tree filepath |
  | --use_simple_gripper | | Use a simple gripper model instead of importing a gripper tree |


## Algorithms

### Voxel superimposition (1)
Simply count the number of voxels within the graspable region that collide with voxels from the target

### Surface normals histogram (2)
Simply calculate the average surface normal of the region of the target colliding with the graspable voxels, and compare it against the ideal surface normal

### Hybrid approach (3)
(1) and (2) methods combined, resulting in a weighed average score

### Surface normals with discrete scoring (4)
Cast rays in both directions between antipodal planes and check each colliding target node surface normal to assess for grasping fitness