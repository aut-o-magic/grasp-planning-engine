#include <cstdio>
#include <chrono>
#include <octomap/OcTree.h>
//#include <Eigen/StdVector>
//#include <Eigen/Geometry>

#include "gp_engine.cpp"

using namespace std::chrono_literals;

std::string target_bintree_filename = "assets/bt/ENVISAT_midres.bt";
std::string target_octree_filename = "assets/ot/ENVISAT_midres_4.ot"; // ! Precompiled with 4 angle discretisations
std::string gripper_bintree_filename = "assets/bt/2F85_gripper_midres.bt";
std::string gripper_octree_filename = "assets/ot/2F85_gripper_midres.ot";

int main()
{    
    // Graspable gripper BBX definition
    octomap::point3d min_point3d{-0.04,0.115,0.085}; // xyz min -0.04,0.115,0.085
    octomap::point3d max_point3d{0.04,0.145,0.102}; // xyz max 0.04,0.145,0.102
    octomap::point3d gripper_normal{0,1,0}; // y-axis points towards target surface normal in 2F85 gripper model

    // Initialise Grasp Quality Map object handle
    graspQualityMap gqm{0.1};

    #define ot // bt -> import from binarytrees, ot-> from octrees (precompiled, faster)
    #ifdef bt
    octomap::OcTree* target_tree = new octomap::OcTree(target_bintree_filename);
    octomap::OcTree* gripper_tree = new octomap::OcTree(gripper_bintree_filename);
    #endif
    #ifdef ot
    octomap::OcTreeGraspQuality* target_tree = new octomap::OcTreeGraspQuality(target_octree_filename);
    octomap::OcTreeGripper* gripper_tree = new octomap::OcTreeGripper(gripper_octree_filename);
    #endif
    
    gqm.set_target_tree(target_tree);
    gqm.set_gripper_tree(gripper_tree, gripper_normal, min_point3d, max_point3d);

    gqm.update_global_grasp_quality(0);

    //gqm.get_target_tree()->write("ENVISAT_midres.ot");
    
    //((octomap::ColorOcTree)*gqm.get_target_tree()).write("targetColor.ot");
    //((octomap::ColorOcTree)*gqm.get_gripper_tree()).write("gripperColor.ot");
    return 0;
}