#include <cstdio>
#include <chrono>
#include <octomap/OcTree.h>
//#include <Eigen/StdVector>
//#include <Eigen/Geometry>

#include "gp_engine.cpp"
#include "gp_utils.cpp"

using namespace std::chrono_literals;

std::string target_bintree_filename = "assets/bt/ENVISAT_midres.bt";
std::string target_octree_filename = "assets/ot/ENVISAT_midres_4.ot"; // ! 4 angle discretisations
std::string gripper_bintree_filename = "assets/bt/2F85_gripper_midres.bt";
std::string gripper_octree_filename = "assets/ot/2F85_gripper_midres.ot";

int main()
{   
    // Graspable gripper BBX definition
    octomap::point3d min_point3d{-0.04,0.115,0.085}; // xyz min -0.04,0.115,0.085
    octomap::point3d max_point3d{0.04,0.145,0.102}; // xyz max 0.04,0.145,0.102

    // * Initialise Grasp Quality Map object handle
    graspQualityMap gqm{0.01};

    // * Load objects
    #define replay // bt -> import from binarytrees, ot-> from octrees (precompiled, faster), sg-> simple gripper (with ot target), replay -> Load output from previous run and analyse thoroughly
    #ifdef bt
    octomap::OcTree* target_tree = new octomap::OcTree(target_bintree_filename);
    octomap::point3d gripper_normal{0,1,0}; // y-axis points towards target surface normal in 2F85 gripper model
    octomap::OcTree* gripper_tree = new octomap::OcTree(gripper_bintree_filename);
    gqm.set_gripper_tree(gripper_tree, gripper_normal, min_point3d, max_point3d);
    #endif
    #ifdef ot
    octomap::OcTreeGraspQuality* target_tree = new octomap::OcTreeGraspQuality(target_octree_filename);
    octomap::OcTreeGripper* gripper_tree = new octomap::OcTreeGripper(gripper_octree_filename);
    gqm.set_gripper_tree(gripper_tree);
    #endif
    #ifdef sg
    octomap::OcTreeGraspQuality* target_tree = new octomap::OcTreeGraspQuality(target_octree_filename);
    gqm.set_simple_gripper(min_point3d, max_point3d);
    #endif
    #ifdef replay
    octomap::OcTreeGraspQuality* target_tree = new octomap::OcTreeGraspQuality("out_target.ot");
    octomap::OcTreeGripper* gripper_tree = new octomap::OcTreeGripper(gripper_octree_filename);
    gqm.set_gripper_tree(gripper_tree);
    #endif
    gqm.set_target_tree(target_tree);

    #ifdef replay
    // * Scratchpad to replay grasp analyses
    octomap::point3d coord_node{-8.55646, -0.0842408, -1.43209};
    octomap::OcTreeGraspQualityNode* node = gqm.get_target_tree()->search(coord_node);
    octomap::OcTreeGraspQuality::iterator node_it{GraspPlanningUtils::nodeToIterator(node, gqm.get_target_tree())};
    Eigen::Affine3f T{gqm.analyse_local_grasp_quality(node_it, 0)};
    gqm.write_grasp_visualisations(T);
    return 0;
    #endif

    gqm.analyse_global_grasp_quality(0);

    // * Output results
    gqm.get_target_tree()->writeGraspQualityHistogram("gq_histogram");
    gqm.get_target_tree()->write("out_target.ot");
    ((octomap::ColorOcTree)*gqm.get_target_tree()).write("out_color_target.ot");
    //((octomap::ColorOcTree)*gqm.get_gripper_tree()).write("gripperColor.ot");

    return 0;
}