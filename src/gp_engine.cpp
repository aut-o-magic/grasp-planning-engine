#include <octomap/octomap.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_grasping/OcTreeGraspQuality.hpp>
#include <octomap_grasping/OcTreeGripper.hpp>
#include <octomap/ColorOcTree.h>

class graspQualityMap
{
public:
    graspQualityMap(const double resolution=0.1) : sensor_origin_{0,0,0}, target_tree_{resolution}, gripper_tree_{resolution} 
    {
        //build_simple_gripper();
        std::string fileName = "assets/bt/ENVISAT_midres.bt";
        octomap::OcTree tree(fileName);
        target_tree_.importOcTree(tree);

        // Copy tree occupancy contents and convert grasping surface flag to Red/Green
        for(octomap::OcTreeGraspQuality::leaf_iterator it = target_tree_.begin_leafs(), end=target_tree_.end_leafs(); it!= end; ++it)
        {
            // cannot use node key as it is only valid for the previous node
            octomap::OcTreeKey node_key = it.getKey();
            //target_tree_.updateNode(node_key, true, true);
            Eigen::Matrix<float,2,18> gq;
            Eigen::Vector3f normal{1,0,0};
            gq.row(0).setLinSpaced(0.0, M_PI_2);
            gq.row(1).setRandom();
            target_tree_.setNodeGraspQuality(node_key, normal, gq);
        }
        target_tree_.updateInnerOccupancy();
        target_tree_.toColorOcTree().write("outtarget.ot");
    }

    // Helper function to generate a simple anti-podal gripper model and save it as an .ot file
    void build_simple_gripper()
    {
        // anti-podal plates
        for (double z = 0; z <= 0.5; z = z + 0.5)
        {
            //xy rectangle plane
            for (double x = 0; x < 0.2; x = x + 0.1)
            {
                for (double y = 0; y < 0.5; y = y + 0.1)
                {
                    octomap::point3d node_point3d{x,y,z};
                    gripper_tree_.updateNode(node_point3d, true);
                    gripper_tree_.setNodeIsGraspingSurface(node_point3d, true);
                }
            }
        }

        // back plate (end stop)
        for (double z = 0; z < 0.5; z = z + 0.1)
        {
            for (double x = 0; x < 0.2; x = x + 0.1)
            {
                gripper_tree_.updateNode(octomap::point3d{x,0,z}, true);
            }
        }

        //gripper_tree_.updateNode(octomap::point3d{1,1,1}, true);

        // save tree
        gripper_tree_.toColorOcTree().write("colorgripper.ot");
    }

    void setter_target_tree(octomap::OcTreeGraspQuality octree)
    {
        // TODO use iterator with copyData node member function
        std::string fileName = "assets/bt/ENVISAT_midres.bt";
        octomap::OcTree tree(fileName);
    }

    void setter_gripper_tree(octomap::OcTreeGripper octree)
    {
        //TODO
    }

    // visualiser...
    // can use castRay to determine distance to closest voxel...
private:
    octomap::point3d sensor_origin_;
    octomap::OcTreeGraspQuality target_tree_;
    octomap::OcTreeGripper gripper_tree_;
};