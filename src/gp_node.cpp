#include <cstdio>
#include <chrono>
#include <octomap/OcTree.h>
//#include <Eigen/StdVector>
//#include <Eigen/Geometry>

#include "gp_engine.cpp"

using namespace std::chrono_literals;

std::string target_bintree_filename = "assets/bt/ENVISAT_midres.bt";
std::string target_octree_filename = "assets/ot/ENVISAT_midres.ot";
std::string gripper_bintree_filename = "assets/bt/2F85_gripper_midres.bt";

int main()
{    
    #define ot // bt -> import from binarytrees, ot-> from object trees (precompiled, faster)
    #ifdef bt
    octomap::OcTree target_tree(target_bintree_filename);
    octomap::OcTree gripper_tree(gripper_bintree_filename);
    #endif
    #ifdef ot
    octomap::OcTreeGraspQuality* target_tree = new octomap::OcTreeGraspQuality(target_octree_filename);
    octomap::OcTree gripper_tree(gripper_bintree_filename); // TODO Compile and switch to .ot model
    #endif


    // xyz min -0.04,0.115,0.085
    octomap::point3d min_point3d{-0.04,0.115,0.085};
    // xyz max 0.04,0.145,0.102
    octomap::point3d max_point3d{0.04,0.145,0.102};

    graspQualityMap gqm{0.1};
    
    gqm.set_target_tree(target_tree);
    gqm.set_gripper_tree(gripper_tree, min_point3d, max_point3d);
    
    gqm.update_global_grasp_quality(0);

    //gqm.get_target_tree()->write("ENVISAT_midres.ot");
    
    //((octomap::ColorOcTree)*gqm.get_target_tree()).write("targetcolor.ot");
    return 0;
}