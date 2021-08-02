# Grasp Planning Engine

## Usage
Compile and launch the program as exemplified in the following code block, where ```--opt arg``` is to be substituted with the program options tabulated below.
```
colcon build
./build/grasp-planning-engine/gp_node --opt arg
```

Option \<Argument> | Description
--- | ---
|```--help``` | Print options description. Options declared will be serially executed in the order listed here |
|```--target <tree_path>``` | Target tree filepath |
|```--gripper <tree_path>``` | Gripper tree filepath |
|```--use_simple_gripper```  | Use a simple gripper model instead of importing a gripper tree |
|```--write_target <save_path{=target.ot}>``` | Write target octree to file |
|```--write_gripper <save_path{=gripper.ot}>``` | Write gripper octree to file |
|```--write_color_target <save_path{=colortree_target.ot}>``` | Write to file ColorOcTree version of target octree |
|```--write_color_gripper <save_path{=colortree_gripper.ot}>``` | Write to file ColorOcTree version of gripper octree |
|```--write_surface_normals_density}``` | Visualise the surface normals density of the target tree |
|```--gp_algorithm <idx>``` | Select grasp planning algorithm to use in idx range [1-7] |
|```--global_analysis``` | Perform a global graspability analysis |
|```--local_analysis=<{x,y,z}>``` | Perform a local analysis at a defined target 3D point. Must pass arg with no spaces and with equal sign (i.e. ```--local_analysis={x,y,z}```) [m]


## Algorithms

### Voxel superimposition (1)
Count the number of target voxels within the graspable region of the gripper, and subtract the number of voxels that collide against the body of the gripper.
  
  Tuneable parameters are: Reward, Penalty

### Surface normals histogram (2)
Build histogram of alignment angles between gripper antipodal plates normal direction and the target surface normal that they touch. Score is a combination of standard deviation of histogram and mean angle between the two vectors. Collisions of target body against the gripper apply a light penalty to the score.
  
  Tuneable parameters are: Mean/std_dev fraction, std_dev saturation
  
### Voxel superimposition + surface normals histogram (3)
(1) and (2) methods combined, resulting in a weighed average score (default is 50% weight each)

### Surface normals with discrete scoring (4)
Cast rays in both directions between both antipodal gripper planes and note the first target node (in between the gripper planes) that gets hit by the ray. Similar to Method 2 but with discrete (individual scores), which cross the 0 graspability score at a defined angle value, instead of histogram generation. Collisions of target body against the gripper apply a light penalty to the score.
  
  Tuneable parameter is: Zero-score crossing
  
### Coplanarity contact points (5)
Compute coplanarity of (separate) target surface contact points hit by rays cast from both antipodal gripper planes, using the disparity between the median and mean as well as the standard deviation of the height difference. Collisions of target body against the gripper apply a light penalty to the score.
  
  Tuneable parameters are: disparity/std_dev fraction, std_dev saturation, discretisation steps
  
### Voxel superimposition + surface normals with discrete scoring (6)
(1) and (4) methods combined, resulting in a weighed average score (default is 50% weight each)
  
### Voxel superimposition + coplanarity contact points (7)
(1) and (5) methods combined, resulting in a weighed average score (default is 50% weight each)
