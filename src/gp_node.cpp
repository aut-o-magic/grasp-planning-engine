#include <cstdio>
#include <chrono>
#include <octomap/OcTree.h>
//#include <Eigen/StdVector>
//#include <Eigen/Geometry>

#include "gp_engine.cpp"

using namespace std::chrono_literals;

std::string target_octree_filename = "assets/bt/ENVISAT_midres.bt";
std::string gripper_octree_filename = "assets/bt/2F85_gripper_midres.bt";

int main()
{
    //graspQualityMap gqm{0.1};
    //gqm.build_simple_gripper();
    
    octomap::OcTree target_tree(target_octree_filename);
    octomap::OcTree gripper_tree(gripper_octree_filename);

    // xyz min -0.04,0.115,0.085
    octomap::point3d min_point3d{-0.04,0.115,0.085};
    // xyz max 0.04,0.145,0.102
    octomap::point3d max_point3d{0.04,0.145,0.102};

    graspQualityMap gqm{0.1};
    gqm.set_target_tree(target_tree);
    gqm.set_gripper_tree(gripper_tree, min_point3d, max_point3d);
    
    octomap::ColorOcTree grasp_visualisation{gqm.visualise_grasp()};

    grasp_visualisation.write("grasp_visualisation.ot");
    
    //gqm.update_global_grasp_quality(0);
    
    //((octomap::ColorOcTree)*gqm.get_gripper_tree()).write("grippercolor.ot");
    return 0;
}