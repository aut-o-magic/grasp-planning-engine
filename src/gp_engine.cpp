#include <octomap/octomap.h>
#include "octomap_grasping/OcTreeGraspQuality.hpp"
#include "octomap_grasping/OcTreeGripper.hpp"
#include <octomap/ColorOcTree.h>

// Grasp planning algorithms legend
#define GP_ONLYVOXELSUPERIMPOSITION 0 // Simply count the number of voxels within the graspable region that collide with voxels from the target
#define GP_ONLYSURFACENORMALS 1 // Simply calculate the average surface normal of the region of the target colliding with the graspable voxels, and compare it against the ideal surface normal
#define GP_VOXELSUPERIMPOSITIONANDSURFACENORMALS 2 // 0 and 1 methods combined

class graspQualityMap
{
public:
    graspQualityMap(const double resolution=0.1) : sensor_origin_{0,0,0}, target_tree_{resolution}, gripper_tree_{resolution} 
    {
        // ! TEMPORARY CODE BELOW
        /*
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
            double rn{rand()};
            double randmax{RAND_MAX};
            gq.row(1).setConstant(rn/randmax);
            target_tree_.setNodeGraspQuality(node_key, normal, gq);
        }
        target_tree_.updateInnerOccupancy();
        target_tree_.toColorOcTree().write("outtarget.ot");
        */
    }

    /** 
     * Helper function to generate a simple anti-podal gripper model and save it as an .ot file
     * ! DEBUG only
    */
    void build_simple_gripper()
    {
        // grasping surface anti-podal plates
        for (double z = 0.05; z <= 0.5; z = z + 0.05)
        {
            //xy rectangle plane
            for (double x = 0; x < 0.2; x = x + 0.05)
            {
                for (double y = 0.1; y < 0.5; y = y + 0.05)
                {
                    octomap::point3d node_point3d{x,y,z};
                    gripper_tree_.updateNode(node_point3d, true);
                    gripper_tree_.setNodeIsGraspingSurface(node_point3d, true);
                }
            }
        }

        // non-grasping surface anti-podal plates
        double z = -0.05;
        {
            //xy rectangle plane
            for (double x = 0; x < 0.2; x = x + 0.1)
            {
                for (double y = 0.1; y < 0.5; y = y + 0.1)
                {
                    octomap::point3d node_point3d{x,y,z};
                    gripper_tree_.updateNode(node_point3d, true);
                    gripper_tree_.setNodeIsGraspingSurface(node_point3d, false);
                }
            }
        }

        z = 0.65;
        {
            //xy rectangle plane
            for (double x = 0; x < 0.2; x = x + 0.1)
            {
                for (double y = 0.1; y < 0.5; y = y + 0.1)
                {
                    octomap::point3d node_point3d{x,y,z};
                    gripper_tree_.updateNode(node_point3d, true);
                    gripper_tree_.setNodeIsGraspingSurface(node_point3d, false);
                }
            }
        }

        // back plate (end stop)
        for (double z = -0.05; z < 0.65; z = z + 0.1)
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

    /**
     * Setter for target tree attribute. Be sure to run update_grasp_quality() to determine the grasp quality of the target
     * @param octree input octree
     */
    void set_target_tree(const octomap::OcTree& octree)
    {
        target_tree_.importOcTree(octree);
    }

    /**
     * Setter for target tree attribute
     * @param octree input octree
     */
    void set_target_tree(const octomap::OcTreeGraspQuality& octree)
    {
        target_tree_ = octree;
    }

    /**
     * Setter for gripper tree attribute and draw graspable bounding box (BBX)
     * @param octree input octree
     * @param min_BBX minimum coordinate of BBX in meters
     * @param max_BBX maximum coordinate of BBX in meters
     */
    void set_gripper_tree(const octomap::OcTree& octree, const octomap::point3d& min_BBX, const octomap::point3d& max_BBX)
    {
        gripper_tree_.importOcTree(octree);
        set_graspable_region(min_BBX, max_BBX);
        //set_graspable_region(min_BBX.z(), max_BBX.z() ,min_BBX.y() ,max_BBX.y() ,min_BBX.x() ,max_BBX.x());
    }

    /**
     * Setter for gripper tree attribute
     * @param octree input octree
     */
    void set_gripper_tree(const octomap::OcTreeGripper& octree)
    {
        gripper_tree_ = octree;
    }

    const octomap::OcTreeGripper* get_gripper_tree() const {return &gripper_tree_;}

    const octomap::OcTreeGraspQuality* get_target_tree() const {return &target_tree_;}

    /**
     * Perform geometric fitting to update the grasp quality metric of the target surface
     * TODO Implement function
     * ! Not finished
     * @param algorithm_select Implementation index of a specific grasp planning algorithm, listed as macros at top of source file
     */
    void update_grasp_quality(unsigned int algorithm_select = 0)
    {
        float score;
        switch(algorithm_select) {
            case GP_ONLYVOXELSUPERIMPOSITION:
                score = gp_voxelsuperimposition();
                break;
            case GP_ONLYSURFACENORMALS:
                score = gp_surfacenormals();
                break;
            case GP_VOXELSUPERIMPOSITIONANDSURFACENORMALS:
                score = gp_voxelsuperimposition_surfacenormals();
                break;
            default:
                std::cerr << "Invalid grasp planning algorithm selected" << std::endl;
                return;
        }
        std::cout << "Grasp quality score: " << score << std::endl;
    }

    // visualiser...
    // can use castRay to determine distance to closest voxel...
private:
    /**
     * Count the number of voxels within the graspable region that collide with voxels from the target
     * TODO algorithm
     */
    float gp_voxelsuperimposition()
    {
        float score{0};


        return score;
    }

    /**
     * Calculate the average surface normal of the region of the target colliding with the graspable voxels, and compare it against the ideal surface normal
     * TODO algorithm
     */
    float gp_surfacenormals()
    {
        float score{0};
        // ? Can create a sub-octree that is just the area of the target that COLLIDES with the graspable voxels

        return score;
    }

    /**
     * Check for voxel superimposition score and surface normals
     * @param voxel_normal_ratio Percentage (in 1 scale) weight given to voxel superimposition method against surface normal method
     */
    float gp_voxelsuperimposition_surfacenormals(float voxel_normal_ratio = 0.5)
    {
        float score{gp_voxelsuperimposition() * voxel_normal_ratio + gp_surfacenormals() * (1-voxel_normal_ratio)};
        return score;
    }

    /**
     * Sets graspable region of gripper octree as BBX (Bounding Box)
     * @param min minimum coordinate of bounding box in meters
     * @param max maximum coordinate of bounding box in meters
     */
    void set_graspable_region(const octomap::point3d& min, const octomap::point3d& max)
    {
        float logodds = gripper_tree_.getClampingThresMaxLog() - gripper_tree_.getClampingThresMinLog();
        octomap::OcTreeKey minKey(0,0,0);
        octomap::OcTreeKey maxKey(0,0,0);
        gripper_tree_.coordToKeyChecked(min, minKey);
        gripper_tree_.coordToKeyChecked(max, maxKey);
        
        octomap::OcTreeKey k;
        for (k[0] = minKey[0]; k[0] < maxKey[0]; ++k[0]){
            for (k[1] = minKey[1]; k[1] < maxKey[1]; ++k[1]){
                for (k[2] = minKey[2]; k[2] < maxKey[2]; ++k[2]){
                    octomap::OcTreeGripperNode* n = gripper_tree_.search(k);
                    if(!n)
                    {
                        octomap::OcTreeGripperNode* nn = gripper_tree_.updateNode(k, logodds);
                        nn->setIsGraspingSurface(true);
                    }
                }
            }
        }
    }


    /**
     * Sets the graspable region of the gripper octree as a rectangular volume
     * ! Not reliable, will be deleted. Use the other overloaded method available
     * @param z_min minimum Z in meters
     * @param z_max maximum Z in meters
     * @param y_min minimum Y in meters
     * @param y_max maximum Y in meters
     * @param x_min minimum X in meters
     * @param x_max maximum X in meters
     */
    void set_graspable_region(double z_min, double z_max, double y_min, double y_max, double x_min, double x_max)
    {
        float logodds = gripper_tree_.getClampingThresMaxLog() - gripper_tree_.getClampingThresMinLog();
        for (double z = z_min; z <= z_max; z = z + gripper_tree_.getResolution())
        {
            for (double y = y_min; y <= y_max; y = y + gripper_tree_.getResolution())
            {
                for (double x = x_min; x <= x_max; x = x + gripper_tree_.getResolution())
                {
                    octomap::point3d node_point3d{x,y,z};
                    octomap::OcTreeGripperNode* n = gripper_tree_.updateNode(node_point3d, true);
                    n->setIsGraspingSurface(true);
                }
            }
        }
    }

    octomap::point3d sensor_origin_;
    octomap::OcTreeGraspQuality target_tree_;
    octomap::OcTreeGripper gripper_tree_;
};