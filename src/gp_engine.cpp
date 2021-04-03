#include <octomap/octomap.h>
#include "octomap_grasping/OcTreeGraspQuality.hpp"
#include "octomap_grasping/OcTreeGripper.hpp"
#include <octomap/ColorOcTree.h>

// Grasp planning algorithms definitions
#define GP_ONLYVOXELSUPERIMPOSITION 0 // Simply count the number of voxels within the graspable region that collide with voxels from the target
#define GP_ONLYSURFACENORMALS 1 // Simply calculate the average surface normal of the region of the target colliding with the graspable voxels, and compare it against the ideal surface normal
#define GP_VOXELSUPERIMPOSITIONANDSURFACENORMALS 2 // 0 and 1 methods combined

class graspQualityMap
{
public:
    graspQualityMap(const double resolution=0.1) : sensor_origin_{0,0,0}, target_tree_{resolution}, gripper_tree_{resolution} 
    {

    }

    /** 
     * Helper function to generate a simple anti-podal gripper model and save it as an .ot file
     * ! DEBUG only
    */
    void build_simple_gripper()
    {
        // grasping surface anti-podal plates
        /*
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
        */
        octomap::point3d min{0,0.1,0.05};
        octomap::point3d max{0.2,0.5,0.6};
        
        add_graspable_region(min, max);
        /*
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
        */
        // save tree
        ((octomap::ColorOcTree)gripper_tree_).write("simple_gripper.ot");
    }

    /**
     * Setter for target tree attribute. Be sure to run update_global_grasp_quality() to determine the grasp quality of the target
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
        // TODO try setting import tree resolution to same as this?
        gripper_tree_.importOcTree(octree);
        add_graspable_region(min_BBX, max_BBX);
        //add_graspable_region(min_BBX.z(), max_BBX.z() ,min_BBX.y() ,max_BBX.y() ,min_BBX.x() ,max_BBX.x());
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
    void update_global_grasp_quality(unsigned int algorithm_select = 0)
    {
        float highest_score{0}; // ? Is this the most relevant metric?
        Eigen::Vector3f highest_score_gripper_normal{1,0,0}; // Gripper normal of highest score
        switch(algorithm_select) {
            case GP_ONLYVOXELSUPERIMPOSITION:
                std::cout << "ONLYVOXELSUPERIMPOSITION SELECTED" << std::endl; // ! DEBUG Call
                
                for (octomap::OcTreeGraspQuality::leaf_iterator it = target_tree_.begin_leafs(), end=target_tree_.end_leafs(); it!= end; ++it)
                {
                    // Step 1: Get surface normal
                    octomap::point3d_collection normals;
                    //print_query_info(it.getCoordinate(), &(*it));
                    
                    normals = get_surface_normals(target_tree_, it.getCoordinate());

                    // TODO continue here...
                    /*
                    // Step 2: Iterate over bounding box of size of gripper on the target, rotating the gripper over all the orientations
                    octomap::OcTreeGraspQualityNode::GraspQuality gq = it->getGraspQuality();
                    for (int i=0; i < gq.angle_quality.cols(); ++i)
                    {
                        //gripper_tree_.getNormals() //gq.angle_quality(0,i) // rotate gripper
                    }

                    //it->setGraspQuality()
                    //highest_score = gp_voxelsuperimposition();
                    */
                }
                
                break;
            case GP_ONLYSURFACENORMALS:
                highest_score = gp_surfacenormals();
                break;
            case GP_VOXELSUPERIMPOSITIONANDSURFACENORMALS:
                highest_score = gp_voxelsuperimposition_surfacenormals();
                break;
            default:
                std::cerr << "Invalid grasp planning algorithm selected" << std::endl;
                return;
        }
        std::cout << "Grasp quality score: " << highest_score << ", at " << highest_score_gripper_normal << std::endl;
    }

    // visualiser...
    // can use castRay to determine distance to closest voxel...
private:
    /**
     * Simple std::out formatter for node occupancy queries
     * @param query 3D point being queried
     * @param node corresponding octree node
     */
    void print_query_info(octomap::point3d query, octomap::OcTreeNode* node)
    {
        if (node) // if not NULL
        {
            std::cout << "occupancy probability at " << query << ":\t " << node->getOccupancy() << std::endl;
        }
        else 
            std::cout << "occupancy probability at " << query << ":\t is unknown" << std::endl;    
    }

    /**
     * Compute the surface normals of the octree at the target point
     * @param tree Input OcTree
     * @param point3d Point at which to compute the surface normals
     * @returns Collection of surface normals normalised vectors
     */
    template<typename NODE>
    octomap::point3d_collection get_surface_normals(const octomap::OccupancyOcTreeBase<NODE>& tree, const octomap::point3d& point3d)
    {
        const double angle_threshold_same_vector = 0.01; // rad (0.01rad = 0.573deg)
        octomap::point3d_collection normals;
        octomap::point3d_collection filtered_normals;
        if (!tree.getNormals(point3d, normals, false)) // run octomap surface reconstruction function considering unknown measurements as FREE
        {
            std::cerr << "[gp_engine::get_surface_normals()] call failed" << std::endl;
            return normals;
        }
        // loop through normal vector collection and remove repeated entries
        for (unsigned int i=0; i < normals.size(); ++i)
        {
            octomath::Vector3 current_vector{normals[i]};
            // check if vector already exists in filtered collection, if not push it there
            if (filtered_normals.empty()) filtered_normals.push_back(current_vector);
            else 
            {
                bool already_exists{false}; // flag to hold whether vector already exists in filtered normals
                for (unsigned int j=0; j < filtered_normals.size(); ++j)
                {
                    if (current_vector.angleTo(filtered_normals[j]) < angle_threshold_same_vector) already_exists = true;
                }
                // if vector normal doesnt already exist in filtered collection, push it there
                if (!already_exists) filtered_normals.push_back(current_vector);
            }
        }
        return filtered_normals;
    }

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
     * Adds a graspable region of gripper octree as BBX (Bounding Box)
     * @param min minimum corner coordinate of bounding box in meters
     * @param max maximum corner coordinate of bounding box in meters
     */
    void add_graspable_region(const octomap::point3d& min, const octomap::point3d& max)
    {
        //float logodds = gripper_tree_.getClampingThresMaxLog() - gripper_tree_.getClampingThresMinLog(); // TODO check if this can be deleted
        octomap::OcTreeKey minKey(0,0,0);
        octomap::OcTreeKey maxKey(0,0,0);
        gripper_tree_.coordToKeyChecked(min, minKey);
        gripper_tree_.coordToKeyChecked(max, maxKey);
        
        octomap::OcTreeKey k;
        for (k[0] = minKey[0]; k[0] < maxKey[0]; ++k[0]){
            for (k[1] = minKey[1]; k[1] < maxKey[1]; ++k[1]){
                for (k[2] = minKey[2]; k[2] < maxKey[2]; ++k[2]){
                    octomap::OcTreeGripperNode* n = gripper_tree_.search(k);
                    if(!n) // If node has not been populated yet
                    {
                        octomap::OcTreeGripperNode* nn = this->gripper_tree_.updateNode(k, true);
                        nn->setIsGraspingSurface(true);
                    } else { // If node exists
                        //n->setLogOdds(logodds); // TODO Update to boolean occupancy, not logodds
                        //octomap::OcTreeGripperNode* nn = gripper_tree_.updateNode(k, logodds);
                        n->setIsGraspingSurface(true);
                    
                    }
                }
            }
        }
    }

    octomap::point3d sensor_origin_;
    octomap::OcTreeGraspQuality target_tree_;
    octomap::OcTreeGripper gripper_tree_;
};