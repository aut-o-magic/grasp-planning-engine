#include "octomap_grasping/OcTreeGraspQuality.hpp"
#include "octomap_grasping/OcTreeGripper.hpp"
#include "gq_algorithms.cpp"
#include "grasp_visualisations.cpp"

#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>
#include <chrono>
#include <iomanip> // std::setw, std::setfill
#include <functional> // std::function, std::ref
#include <memory> // std::make_shared, std::shared_ptr
//#include <thread> // std::thread
//#include <deque> // std::deque

// Grasp planning algorithms definitions
#define GP_ONLYVOXELSUPERIMPOSITION 0 // Simply count the number of voxels within the graspable region that collide with voxels from the target
#define GP_ONLYSURFACENORMALS 1 // Simply calculate the average surface normal of the region of the target colliding with the graspable voxels, and compare it against the ideal surface normal
#define GP_VOXELSUPERIMPOSITIONANDSURFACENORMALS 2 // 0 and 1 methods combined
#define GP_RAYCASTINGANTIPODALPLANES 3 // Cast rays in both directions between antipodal planes and check each colliding target node surface normal to assess for grasping fitness

/**
 * Pause program execution, prompting a key-press in the terminal to continue
 */
inline void pause()
{
    do 
    {
        std::cout << '\n' << "Press a key to continue...";
    } while (std::cin.get() != '\n');
}

class graspQualityMap
{
public:
    graspQualityMap(const double resolution=0.1) : target_tree_{new octomap::OcTreeGraspQuality(resolution)}, gripper_tree_{new octomap::OcTreeGripper(resolution)}, sensor_origin_{0,0,0}
    {}

    /** 
     * Helper function to generate a simple anti-podal gripper model and save it as an .ot file
     * ! DEBUG only
    */
    void set_simple_gripper(const octomap::point3d& min_point3d, const octomap::point3d& max_point3d)
    {
        std::cout << "[set_simple_gripper] started..." << std::endl;
        // * Non-graspable shell region
        octomap::point3d grasping_normal{0,1,0}; // Normal direction for gripping is {0,1,0}
        this->gripper_tree_->setGraspingNormal(grasping_normal);
        const double res{this->gripper_tree_->getResolution()};
        const octomap::point3d min_shell{min_point3d - octomap::point3d(res,res+res,0)};
        const octomap::point3d max_shell{max_point3d + octomap::point3d(res+res,0,0)};
        octomap::OcTreeKey minKey{};
        octomap::OcTreeKey maxKey{};
        gripper_tree_->coordToKeyChecked(min_shell, minKey);
        gripper_tree_->coordToKeyChecked(max_shell, maxKey);

        octomap::OcTreeKey k;
        for (k[0] = minKey[0]; k[0] < maxKey[0]; ++k[0]){
            for (k[1] = minKey[1]; k[1] < maxKey[1]; ++k[1]){
                for (k[2] = minKey[2]; k[2] < maxKey[2]; ++k[2]){
                    octomap::OcTreeGripperNode* n = gripper_tree_->search(k);
                    if(!n) // If node has not been populated yet
                    {
                        octomap::OcTreeGripperNode* nn = this->gripper_tree_->updateNode(k, true);
                        nn->setIsGraspingSurface(false);
                    }
                    else // If node exists
                    {
                        if (n->getOccupancy() < 0.5) // if node is free
                            n->setLogOdds(this->gripper_tree_->getProbHitLog());
                        n->setIsGraspingSurface(false);
                    }
                }
            }
        }

        // * Override nodes within graspable region
        octomap::point3d grasp_center_point{add_graspable_region(min_point3d, max_point3d)};
        this->gripper_tree_->setOrigin(grasp_center_point);
    }

    /**
     * Setter for target tree attribute. Be sure to run update_global_grasp_quality() to determine the grasp quality of the target
     * @param octree input octree
     */
    void set_target_tree(octomap::OcTree* octree)
    {
        std::cout << "[set_target_tree] started..." << std::endl;
        target_tree_->importOcTree(*octree);
    }

    /**
     * Setter for target tree attribute
     * @param octree input octree
     */
    void set_target_tree(octomap::OcTreeGraspQuality* octree)
    {
        std::cout << "[set_target_tree] started..." << std::endl;
        target_tree_ = octree;
    }

    /**
     * Setter for gripper tree attribute and draw graspable bounding box (BBX)
     * @param octree input octree
     * @param gripper_normal Normal vector of the gripper towards the target surface
     */
    void set_gripper_tree(octomap::OcTree* octree, const octomap::point3d& gripper_normal)
    {
        std::cout << "[set_gripper_tree] started..." << std::endl;
        gripper_tree_->importOcTree(*octree);
        gripper_tree_->setGraspingNormal(gripper_normal);
    }    

    /**
     * Setter for gripper tree attribute and draw graspable bounding box (BBX)
     * @param octree input octree
     * @param gripper_normal Normal vector of the gripper towards the target surface
     * @param min_BBX minimum coordinate of BBX in meters
     * @param max_BBX maximum coordinate of BBX in meters
     */
    void set_gripper_tree(octomap::OcTree* octree, const octomap::point3d& gripper_normal, const octomap::point3d& min_BBX, const octomap::point3d& max_BBX)
    {
        std::cout << "[set_gripper_tree] started..." << std::endl;
        gripper_tree_->importOcTree(*octree);
        gripper_tree_->setGraspingNormal(gripper_normal);
        octomap::point3d grasp_center_point{add_graspable_region(min_BBX, max_BBX)};
        this->gripper_tree_->setOrigin(grasp_center_point);
    }

    /**
     * Setter for gripper tree attribute and draw graspable bounding box (BBX)
     * @param octree input gripper octree
     */
    void set_gripper_tree(octomap::OcTreeGripper* octree)
    {
        std::cout << "[set_gripper_tree] started..." << std::endl;
        gripper_tree_ = octree;
    }

    /**
     * Setter for gripper tree attribute and draw graspable bounding box (BBX)
     * @param octree input gripper octree
     * @param gripper_normal Normal vector of the gripper towards the target surface
     * @param min_BBX minimum coordinate of BBX in meters
     * @param max_BBX maximum coordinate of BBX in meters
     */
    void set_gripper_tree(octomap::OcTreeGripper* octree, const octomap::point3d& min_BBX, const octomap::point3d& max_BBX)
    {
        std::cout << "[set_gripper_tree] started..." << std::endl;
        gripper_tree_ = octree;
        octomap::point3d grasp_center_point{add_graspable_region(min_BBX, max_BBX)};
        this->gripper_tree_->setOrigin(grasp_center_point);
    }

    const octomap::OcTreeGripper* get_gripper_tree() const {return gripper_tree_;}

    const octomap::OcTreeGraspQuality* get_target_tree() const {return target_tree_;}

    /**
     * Analyse graspability of a single target-tree node
     * @param it_node Target node iterator
     * @param algorithm_select Implementation index of a specific grasp planning algorithm, listed as macros at top of source file
     * @returns Gripper transformation in target reference frame
     */ 
    Eigen::Affine3f analyse_local_grasp_quality(octomap::OcTreeGraspQuality::iterator it_node, unsigned int algorithm_select = 0)
    {
        std::cout << "[analyse_local_grasp_quality] started..." << std::endl;
        octomap::OcTreeGraspQualityNode::GraspQuality gq{it_node->getGraspQuality()};
        Eigen::Affine3f Tbest;
        float (*gq_function)(const Eigen::Affine3f &T, const octomap::OcTreeGraspQuality *target_tree_, octomap::OcTreeGripper *gripper_tree_); // Grasp Quality function handle

        switch(algorithm_select) {
        case GP_ONLYVOXELSUPERIMPOSITION:
        {
            std::cout << "ONLYVOXELSUPERIMPOSITION SELECTED" << std::endl;
            gq_function = GraspQualityMethods::gq_voxelsuperimposition;
            break;
        }
        case GP_ONLYSURFACENORMALS:
            std::cout << "ONLYSURFACENORMALS SELECTED" << std::endl;
            gq_function = GraspQualityMethods::gq_surfacenormals;
            break;
        case GP_VOXELSUPERIMPOSITIONANDSURFACENORMALS:
            std::cout << "VOXELSUPERIMPOSITIONANDSURFACENORMALS SELECTED" << std::endl;
            gq_function = GraspQualityMethods::gq_voxelsuperimposition_surfacenormals;
            break;
        case GP_RAYCASTINGANTIPODALPLANES:
            std::cout << "RAYCASTINGANTIPODALPLANES SELECTED" << std::endl;
            gq_function = GraspQualityMethods::gq_raycasting;
            break;
        default:
            std::cerr << "ERROR. Invalid grasp planning algorithm selected" << std::endl;
            return Tbest;
        }

        node_gq_analysis(it_node, this->target_tree_, this->gripper_tree_, gq_function, gq, Tbest);

        std::cout << "*** Best grasp candidate at " << it_node.getCoordinate() << " has a score of " << gq.angle_quality.row(1).maxCoeff() << " ***" << std::endl;

        return Tbest;
    }

    /**
     * Perform geometric fitting to update the grasp quality metric of the target surface
     * @param algorithm_select Implementation index of a specific grasp planning algorithm, listed as macros at top of source file
     * @note Grasp planning algorithms definitions:
     * 0 -> GP_ONLYVOXELSUPERIMPOSITION: Simply count the number of voxels within the graspable region that collide with voxels from the target
     * 1 -> GP_ONLYSURFACENORMALS: Simply calculate the average surface normal of the region of the target colliding with the graspable voxels, and compare it against the ideal surface normal
     * 2 -> GP_VOXELSUPERIMPOSITIONANDSURFACENORMALS: 0 and 1 methods combined
     * 3 -> GP_RAYCASTINGANTIPODALPLANES: Cast rays in both directions between antipodal planes and check each colliding target node surface normal to assess for grasping fitness
     */
    void analyse_global_grasp_quality(unsigned int algorithm_select = 0)
    {
        std::cout << "[update_global_grasp_quality] started..." << std::endl;
        auto start = std::chrono::high_resolution_clock::now();
        float (*gq_function)(const Eigen::Affine3f &T, const octomap::OcTreeGraspQuality *target_tree_, octomap::OcTreeGripper *gripper_tree_); // Grasp Quality function handle
        
        // grasp planning algorithm select
        switch(algorithm_select) {
            case GP_ONLYVOXELSUPERIMPOSITION:
            {
                std::cout << "ONLYVOXELSUPERIMPOSITION SELECTED" << std::endl;
                gq_function = GraspQualityMethods::gq_voxelsuperimposition;
                break;
            }
            case GP_ONLYSURFACENORMALS:
                std::cout << "ONLYSURFACENORMALS SELECTED" << std::endl;
                gq_function = GraspQualityMethods::gq_surfacenormals;
                break;
            case GP_VOXELSUPERIMPOSITIONANDSURFACENORMALS:
                std::cout << "VOXELSUPERIMPOSITIONANDSURFACENORMALS SELECTED" << std::endl;
                gq_function = GraspQualityMethods::gq_voxelsuperimposition_surfacenormals;
                break;
            case GP_RAYCASTINGANTIPODALPLANES:
                std::cout << "RAYCASTINGANTIPODALPLANES SELECTED" << std::endl;
                gq_function = GraspQualityMethods::gq_raycasting;
                break;
            default:
                std::cerr << "ERROR. Invalid grasp planning algorithm selected" << std::endl;
                return;
        }
        
        float best_global_score{-1}; // -1 will get overwritten in first iteration as gq is [0,1]
        octomap::point3d best_node{}; // Target node associated with highest graspability score

        // expand target_tree for complete analysis
        target_tree_->expand();
        // progress tracking variables
        size_t total_nodes{target_tree_->calcNumNodes()};
        std::cout << "Total nodes: " << total_nodes << std::endl;
        size_t current_node{0};

        for (octomap::OcTreeGraspQuality::leaf_iterator it_node = target_tree_->begin_leafs(), end = target_tree_->end_leafs(); it_node != end; ++it_node)
        {
            // Dispatch threaded worker to analyse node
            octomap::OcTreeGraspQualityNode::GraspQuality gq{node_gq_analysis(it_node, gq_function)};

            if (gq.angle_quality.row(1).maxCoeff() > best_global_score)
            {
                best_global_score = gq.angle_quality.row(1).maxCoeff();
                best_node = it_node.getCoordinate();
            }

            // Write to node
            it_node->setGraspQuality(gq);

            // Print progress and advance tracker
            if (current_node % (total_nodes/100) == 0) 
                std::cout << '\r' << std::setw(2) << std::setfill('0') << (current_node*100)/total_nodes << "%" << std::flush;
            current_node++;
        }
        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::seconds>(stop - start);
        std::cout << ">>>Grasp quality score: " << best_global_score << ", at:\n" << best_node << std::endl;
        std::cout << "[update_global_grasp_quality] finished (" << duration.count() << "s)" << std::endl;
    }

    /**
     * Compute the surface normals of the octree at the target point
     * @param tree Input OcTree
     * @param point3d Point at which to compute the surface normals
     * @returns Collection of surface normals normalised vectors
     */
    template<typename NODE>
    static octomap::point3d_collection get_surface_normals(const octomap::OccupancyOcTreeBase<NODE>* tree, const octomap::point3d& point3d)
    {
        const double angle_threshold_same_vector = 0.01; // rad (0.01rad = 0.573deg)
        octomap::point3d_collection normals;
        octomap::point3d_collection filtered_normals;
        if (!tree->getNormals(point3d, normals, false)) // run octomap surface reconstruction function considering unknown measurements as FREE
        {
            std::cerr << "[gp_engine::get_surface_normals()] call failed" << std::endl;
            return normals;
        }
        // loop through normal vector collection and remove repeated entries
        bool already_exists{false}; // flag to hold whether vector already exists in filtered normals
        for (unsigned int i=0; i < normals.size(); ++i)
        {
            octomath::Vector3 current_vector{normals[i]};
            
            // check if vector already exists in filtered collection, if not push it there
            for (unsigned int j=0; j < filtered_normals.size(); ++j)
            {
                if (current_vector.angleTo(filtered_normals[j]) < angle_threshold_same_vector) already_exists = true;
            }
            // if vector normal doesnt already exist in filtered collection, push it there
            if (!already_exists) filtered_normals.push_back(current_vector);
            already_exists = false; // reset flag
        }
        return filtered_normals;
    }

    /**
     * Convenience function for writing both local and global grasp visualisations to file
     * @param TF Gripper affine transformation matrix
     */
    void write_grasp_visualisations(const Eigen::Affine3f& TF)
    {
        GraspVisualisations::visualise_local_grasp(target_tree_, gripper_tree_, false, TF).write("local_grasp_visual.ot");
        GraspVisualisations::visualise_global_grasp(target_tree_, gripper_tree_, TF).write("global_grasp_visual.ot");
        pause();
    }

private:
    /**
     * Grasp quality analysis worker using *this object attributes
     * @param it_node Target node iterator
     * @param gq_virtual Grasp quality method to use
     * @returns GraspQuality object for the node
     */
    template<typename gq_method>
    octomap::OcTreeGraspQualityNode::GraspQuality node_gq_analysis(const octomap::OcTreeGraspQuality::leaf_iterator& it_node, gq_method& gq_virtual) const
    {
        octomap::OcTreeGraspQualityNode::GraspQuality gq{it_node->getGraspQuality()};
        Eigen::Affine3f Tbest;
        node_gq_analysis(it_node, this->target_tree_, this->gripper_tree_, gq_virtual, gq, Tbest);
        return gq;
    }
    /**
     * Grasp quality analysis worker
     * @param it_node Target node iterator
     * @param target_tree target tree
     * @param gripper_tree gripper tree
     * @param gq_virtual Grasp quality method to use
     * @param gq Grasp quality node attribute to populate
     * @param Tbest Transformation of gripper transformation for best grasp candidate
     * @returns Graspability score of best grasp candidate
     */
    template<typename gq_method>
    static void node_gq_analysis(const octomap::OcTreeGraspQuality::leaf_iterator& it_node, const octomap::OcTreeGraspQuality* target_tree, octomap::OcTreeGripper* gripper_tree, gq_method& gq_virtual, octomap::OcTreeGraspQualityNode::GraspQuality& gq, Eigen::Affine3f& Tbest)
    {
        // * Retrieve surface normals and gripper translation (t0)
        octomap::point3d_collection normals = get_surface_normals(target_tree, it_node.getCoordinate());
        if (normals.empty()) return; // if there is no normal to the surface, terminate early
        Eigen::Vector3f coordinates_node{it_node.getCoordinate().x(), it_node.getCoordinate().y(), it_node.getCoordinate().z()};
        Eigen::Affine3f T0{Eigen::Affine3f::Identity()}; // T0 is transformation from gripper origin in world (target) frame to grasping point and pointing vector as dictated by each output of marching cubes algorithm 
        T0.translation() = coordinates_node;

        // Keep record of best nodal score
        float best_nodal_score{-1}; 

        // * Iterate over each gripper angle discretisation (R01)
        for (unsigned int i=0; i<gq.angle_quality.row(0).size(); ++i)
        {
            float best_local_score{-1}; // grasp quality values will always be at least 0 so this will get overwritten on first iteration

            // * Iterate over each solution of the surface reconstruction algorithm (R0)
            for (octomap::point3d_collection::iterator it_norm = normals.begin(), end = normals.end(); it_norm != end; ++it_norm)
            {
                // rotation between gripper origin pointing vec and normal is R0 // ! gripper does not necessary stay horizontal with this convention, but results are reproduceable
                it_norm->normalize(); // Assure normalised normal vector

                float rot_angle{(float)gripper_tree->getGraspingNormal().angleTo(*it_norm)};
                octomap::point3d rot_axis{gripper_tree->getGraspingNormal().cross(*it_norm)};
                Eigen::Vector3f eigen_rot_axis{rot_axis.x(), rot_axis.y(), rot_axis.z()};
                Eigen::AngleAxisf rotation{rot_angle, eigen_rot_axis};
                T0.linear() = rotation.toRotationMatrix(); // rotation component is called "linear part" in Eigen library https://eigen.tuxfamily.org/dox/classEigen_1_1Transform.html

                // * Apply gripper angle rotation
                Eigen::Affine3f Ti{T0};
                float angle{gq.angle_quality.row(0)(i)};
                Eigen::Vector3f norm_eigen{it_norm->x(), it_norm->y(), it_norm->z()};
                Eigen::AngleAxisf rot{angle, norm_eigen}; // rotate around pointing vector axis
                Ti.rotate(rot);

                // * Calculate grasp quality for each normal candidate and record best one
                float score = gq_virtual(Ti, target_tree, gripper_tree);
                if (score > best_local_score)
                {
                    best_local_score = score;

                    // record best nodal grasp candidate
                    if (best_local_score > best_nodal_score)
                    {
                        Tbest = Ti;
                        best_nodal_score = best_local_score;
                    }
                }
            }
            // Write to gq object
            gq.angle_quality.row(1)(i) = best_local_score;
        }
    }

    /**
     * Compute vector with max composite of each coordinate
     * @param __vector1 First vector
     * @param __vector2 Second vector
     * @returns Max composite vector
     */
    octomap::point3d max_composite_vector(const octomap::point3d& __vector1, const octomap::point3d& __vector2) const
    {
        octomap::point3d output;
        for (unsigned int i=0; i<3; ++i)
        {
            output(i) = std::max(__vector1(i),__vector2(i));
        }
        return output;
    }

    /**
     * Compute vector with min composite of each coordinate
     * @param __vector1 First vector
     * @param __vector2 Second vector
     * @returns Min composite vector
     */
    octomap::point3d min_composite_vector(const octomap::point3d& __vector1, const octomap::point3d& __vector2) const
    {
        octomap::point3d output;
        for (unsigned int i=0; i<3; ++i)
        {
            output(i) = std::min(__vector1(i),__vector2(i));
        }
        return output;
    }

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
     * Adds a graspable region of gripper octree as BBX (Bounding Box)
     * @param min minimum corner coordinate of bounding box in meters
     * @param max maximum corner coordinate of bounding box in meters
     * @returns center of newly added graspable region
     */
    octomap::point3d add_graspable_region(const octomap::point3d& min, const octomap::point3d& max)
    {
        octomap::OcTreeKey minKey(0,0,0);
        octomap::OcTreeKey maxKey(0,0,0);
        gripper_tree_->coordToKeyChecked(min, minKey);
        gripper_tree_->coordToKeyChecked(max, maxKey);

        octomap::OcTreeKey k;
        for (k[0] = minKey[0]; k[0] < maxKey[0]; ++k[0]){
            for (k[1] = minKey[1]; k[1] < maxKey[1]; ++k[1]){
                for (k[2] = minKey[2]; k[2] < maxKey[2]; ++k[2]){
                    octomap::OcTreeGripperNode* n = gripper_tree_->search(k);
                    if(!n) // If node has not been populated yet
                    {
                        octomap::OcTreeGripperNode* nn = this->gripper_tree_->updateNode(k, true);
                        nn->setIsGraspingSurface(true);
                    } 
                    else // If node exists
                    {
                        // TODO Check this updates node at lowest depth level
                        if (n->getOccupancy() < 0.5) // if node is free
                            n->setLogOdds(this->gripper_tree_->getProbHitLog());
                        n->setIsGraspingSurface(true);
                    }
                }
            }
        }
        octomap::point3d center_bbx{(max-min)*0.5 + min};
        return center_bbx;
    }

    octomap::OcTreeGraspQuality* target_tree_;
    octomap::OcTreeGripper* gripper_tree_;
    octomap::point3d sensor_origin_;
};