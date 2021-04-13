#include <octomap/octomap.h>
#include "octomap_grasping/OcTreeGraspQuality.hpp"
#include "octomap_grasping/OcTreeGripper.hpp"
#include <octomap/ColorOcTree.h>
#include <thread> // std::thread
#include <functional> // std::ref
#include <deque> // std::deque
#include <condition_variable>
#include <chrono>

// Grasp planning algorithms definitions
#define GP_ONLYVOXELSUPERIMPOSITION 0 // Simply count the number of voxels within the graspable region that collide with voxels from the target
#define GP_ONLYSURFACENORMALS 1 // Simply calculate the average surface normal of the region of the target colliding with the graspable voxels, and compare it against the ideal surface normal
#define GP_VOXELSUPERIMPOSITIONANDSURFACENORMALS 2 // 0 and 1 methods combined

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
    graspQualityMap(const double resolution=0.1) : sensor_origin_{0,0,0}, target_tree_{resolution}, gripper_tree_{resolution}, normal_gripper_{0,1,0}
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
    void set_target_tree(octomap::OcTree* octree)
    {
        std::cout << "[set_target_tree] started..." << std::endl;
        target_tree_.importOcTree(*octree);
    }

    /**
     * Setter for target tree attribute
     * @param octree input octree
     */
    void set_target_tree(octomap::OcTreeGraspQuality* octree)
    {
        std::cout << "[set_target_tree] started..." << std::endl;
        target_tree_ = *octree;
    }

    /**
     * Setter for gripper tree attribute and draw graspable bounding box (BBX)
     * @param octree input octree
     * @param gripper_normal Normal vector of the gripper towards the target surface
     */
    void set_gripper_tree(octomap::OcTree* octree, const octomap::point3d& gripper_normal)
    {
        std::cout << "[set_gripper_tree] started..." << std::endl;
        gripper_tree_.importOcTree(*octree);
        this->normal_gripper_ = gripper_normal;
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
        gripper_tree_.importOcTree(*octree);
        this->normal_gripper_ = gripper_normal;
        octomap::point3d grasp_center_point{add_graspable_region(min_BBX, max_BBX)};
        this->gripper_tree_.setOrigin(grasp_center_point);
    }

    /**
     * Setter for gripper tree attribute and draw graspable bounding box (BBX)
     * @param octree input gripper octree
     * @param gripper_normal Normal vector of the gripper towards the target surface
     */
    void set_gripper_tree(octomap::OcTreeGripper* octree, const octomap::point3d& gripper_normal)
    {
        std::cout << "[set_gripper_tree] started..." << std::endl;
        gripper_tree_ = *octree;
        this->normal_gripper_ = gripper_normal;
    }

    /**
     * Setter for gripper tree attribute and draw graspable bounding box (BBX)
     * @param octree input gripper octree
     * @param gripper_normal Normal vector of the gripper towards the target surface
     * @param min_BBX minimum coordinate of BBX in meters
     * @param max_BBX maximum coordinate of BBX in meters
     */
    void set_gripper_tree(octomap::OcTreeGripper* octree, const octomap::point3d& gripper_normal, const octomap::point3d& min_BBX, const octomap::point3d& max_BBX)
    {
        std::cout << "[set_gripper_tree] started..." << std::endl;
        gripper_tree_ = *octree;
        this->normal_gripper_ = gripper_normal;
        octomap::point3d grasp_center_point{add_graspable_region(min_BBX, max_BBX)};
        this->gripper_tree_.setOrigin(grasp_center_point);
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
        std::cout << "[update_global_grasp_quality] started..." << std::endl;
        float highest_score{0}; // ? Is this the most relevant metric?
        Eigen::Vector3f highest_score_gripper_normal{1,0,0}; // Gripper normal of highest score

        // expand target_tree for complete analysis
        target_tree_.expand();

        // grasp planning algorithm select
        switch(algorithm_select) {
            case GP_ONLYVOXELSUPERIMPOSITION:
            {
                std::cout << "ONLYVOXELSUPERIMPOSITION SELECTED" << std::endl;

                for (octomap::OcTreeGraspQuality::leaf_iterator it = target_tree_.begin_leafs(), end=target_tree_.end_leafs(); it!= end; ++it) // ? Start at the back to avoid solar panels at the beginning?
                {
                    // * Step 1: Get surface normals
                    octomap::point3d_collection normals;
                    //print_query_info(it.getCoordinate(), &(*it));
                    normals = get_surface_normals(target_tree_, it.getCoordinate());

                    // * Step 2: Find best surface normal candidate
                    Eigen::Affine3f TF{Eigen::Affine3f::Identity()};
                    Eigen::Vector3f coordinates_node{it.getCoordinate().x(), it.getCoordinate().y(), it.getCoordinate().z()};
                    //octomap::point3d normal_gripper{0,1,0}; // Set as class attribute
                    TF.translation() = coordinates_node;


                    // Analyse surface normals collection
                    float best_score{-999999}; // TODO Set to a meaninful number
                    octomap::point3d best_normal{};
                    std::cout << "Surface normals:" << std::endl;
                    for (unsigned int i = 0; i < normals.size(); i++)
                    {
                        octomap::point3d normal_node{normals[i]};
                        std::cout << normal_node << std::endl;

                        // Compute gripper transformation
                        float rot_angle{(float)normal_gripper_.angleTo(normal_node.normalized())};
                        octomap::point3d rot_axis{normal_gripper_.cross(normal_node.normalized())};
                        Eigen::Vector3f euler_rot_axis{rot_axis.x(), rot_axis.y(), rot_axis.z()};
                        Eigen::AngleAxisf rotation{rot_angle, euler_rot_axis};
                        TF.linear() = rotation.toRotationMatrix(); // rotation component is called "linear part" in Eigen library https://eigen.tuxfamily.org/dox/classEigen_1_1Transform.html

                        // Calculate grasp quality
                        // ? is there no more efficient way of checking which surface normal to use?

                        float score = gp_voxelsuperimposition(TF);

                        if (score > best_score)
                        {
                            best_score = score;
                            best_normal = normal_node;
                        }

                        // Visualise grasp attempt
                        #if 1
                        std::cout << "Score: " << score << std::endl;
                        std::cout << "Normal: " << normal_node << std::endl;
                        visualise_local_grasp(false, TF).write("local_grasp_visual.ot");
                        visualise_global_grasp(TF).write("global_grasp_visual.ot");
                        pause();
                        #endif
                    }

                    // * Step 3: Rotate the gripper over all the orientations and store gq in target octree
                    // TODO continue here...
                    /*

                    //it->setGraspQuality()
                    //highest_score = gp_voxelsuperimposition();
                    */
                }

                break;
            }
            case GP_ONLYSURFACENORMALS:
                //highest_score = gp_surfacenormals();
                break;
            case GP_VOXELSUPERIMPOSITIONANDSURFACENORMALS:
                //highest_score = gp_voxelsuperimposition_surfacenormals();
                break;
            default:
                std::cerr << "Invalid grasp planning algorithm selected" << std::endl;
                return;
        }
        std::cout << "Grasp quality score: " << highest_score << ", at " << highest_score_gripper_normal << std::endl;
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


    // *** VISUALISATION METHODS *** --------------------------------------

    /**
     * Visualise surface normals density of target tree using filtered marching cubes surface reconstruction algorithm.
     * Density color coding: white = 0, red = 1, green = 2, blue = 3, black => 4
     */
    void visualise_surface_normals_density()
    {
        std::cout << "[visualise_surface_normals_density] started..." << std::endl;
        octomap::ColorOcTree color_tree_normals_density{this->target_tree_.getResolution()};
        unsigned int normal0{0};
        unsigned int normal1{0};
        unsigned int normal2{0};
        unsigned int normal3{0};
        unsigned int normal3plus{0};

        // expand target_tree for complete analysis
        target_tree_.expand();

        for (octomap::OcTreeGraspQuality::leaf_iterator it = target_tree_.begin_leafs(), end=target_tree_.end_leafs(); it!= end; ++it)
        {
            octomap::point3d_collection normals;
            //print_query_info(it.getCoordinate(), &(*it));
            normals = get_surface_normals(target_tree_, it.getCoordinate());

            // Populate color_tree_normals_density object
            octomap::ColorOcTreeNode* snn = color_tree_normals_density.updateNode(it.getCoordinate(), true);
            octomap::ColorOcTreeNode::Color color{0,0,0};
            // white = 0, red = 1, green = 2, blue = 3, black => 4
            if (normals.size() == 0) {normal0++;}
            else if (normals.size() == 1) {color.r = 255; normal1++;}
            else if (normals.size() == 2) {color.g = 255; normal2++;}
            else if (normals.size() == 3) {color.b = 255; normal3++;}
            else {color = octomap::ColorOcTreeNode::Color{255, 255, 255}; normal3plus++;}
            uint8_t r = std::max((unsigned long)0, 255-(normals.size()*51));
            uint8_t g = std::min((unsigned long)255, 0+(normals.size()*51));
            snn->setColor(r,g,0);
        }

        color_tree_normals_density.updateInnerOccupancy();

        std::cout << "[Surface normals density study]:" << std::endl << "Size collection: [0]=" << normal0 << " [1]=" << normal1 << " [2]=" << normal2 << " [3]=" << normal3 << " [3+]=" << normal3plus << std::endl;
        color_tree_normals_density.write("color_tree_normals_density.ot");
    }

    /**
     * Local grasp visualiser showing the region only in the BBX of the gripper octree
     * @param show_target_voxels Show/hide the target octree voxels
     * @param T homogenous transformation from origin to gripper grasping pose
     * @param BBX_margin Add additional margins to the BBX drawn for better occupancy context
     * @returns ColorOcTree visualisation with: \n Green -> Positive overlapping voxels; \n Red -> Negative overlapping voxels; \n Light blue -> Non-interacting Gripper voxels; \n Dark blue -> Non-interacting Target voxels.
     */
    octomap::ColorOcTree visualise_local_grasp(bool show_target_voxels = false, const Eigen::Affine3f& T = Eigen::Affine3f::Identity(), float BBX_margin = 0)
    {
        //#define ALWAYS_CARTESIAN_BBX_METHOD 
        // ndef-> Use Node ptr iteration (faster) when target voxels don't have to be shown,
        // def-> Always use BBX cartesian iteration regardless of show_target_voxels parameter
        
        std::cout << "[visualise_local_grasp] started..." << std::endl;
        octomap::ColorOcTree color_tree{std::min(this->gripper_tree_.getResolution(),this->target_tree_.getResolution())};

        #ifndef ALWAYS_CARTESIAN_BBX_METHOD
        if (show_target_voxels)
        {
        #endif
            // *** Method 0 *** Spatial BBX iteration
            double x,y,z;
            this->gripper_tree_.getMetricMax(x,y,z);
            x += BBX_margin;
            y += BBX_margin;
            z += BBX_margin;
            octomap::point3d max_bbx{(float)x,(float)y,(float)z};
            this->gripper_tree_.getMetricMin(x,y,z);
            x -= BBX_margin;
            y -= BBX_margin;
            z -= BBX_margin;
            octomap::point3d min_bbx{(float)x,(float)y,(float)z};
            color_tree.setBBXMax(max_bbx);
            color_tree.setBBXMin(min_bbx);

            for (float x = color_tree.getBBXMin().x(); x < color_tree.getBBXMax().x(); x = x + (float)color_tree.getResolution()/2)
            {
                for (float y = color_tree.getBBXMin().y(); y < color_tree.getBBXMax().y(); y = y + (float)color_tree.getResolution()/2)
                {
                    for (float z = color_tree.getBBXMin().z(); z < color_tree.getBBXMax().z(); z = z + (float)color_tree.getResolution()/2)
                    {
                        // transform coordinates according to T
                        Eigen::Vector3f coord_g{x,y,z};
                        octomap::point3d gripper3d{x,y,z};
                        Eigen::Vector3f coord_w{T * coord_g};
                        octomap::point3d world3d{coord_w.x(), coord_w.y(), coord_w.z()};

                        // search for corresponding nodes in octrees
                        octomap::OcTreeGripperNode* gn = gripper_tree_.search(coord_g.x(), coord_g.y(), coord_g.z());
                        octomap::OcTreeGraspQualityNode* tn = target_tree_.search(world3d);

                        // colorise nodes
                        octomap::ColorOcTreeNode::Color color{0,0,0};
                        if (gn) // if gripper node present
                        {
                            octomap::ColorOcTreeNode* n = color_tree.updateNode(gripper3d, true);
                            if (tn)
                            {
                                if (gn->isGraspingSurface()) color.g = 255;
                                else color.r = 255;
                            }
                            else color.b = 255; // light blue for gripper-only nodes
                            n->setColor(color);
                        }
                        else if (tn && show_target_voxels)
                        {
                            octomap::ColorOcTreeNode* n = color_tree.updateNode(gripper3d, true);
                            color.b = 50; // dark blue for target-only nodes
                            n->setColor(color);
                        }
                    }
                }
            }
        #ifndef ALWAYS_CARTESIAN_BBX_METHOD
        }
        else
        {
            // *** Method 1 *** Iterate over octree node ptrs, does not support showing target voxels

            gripper_tree_.expand();

            // iterate over gripper octree
            for (octomap::OcTreeGripper::leaf_iterator it = gripper_tree_.begin_leafs(), end=gripper_tree_.end_leafs(); it!= end; ++it)
            {
                // transform coordinates according to T
                octomap::point3d gripper3d{it.getCoordinate()};
                Eigen::Vector3f coord_g{gripper3d.x(), gripper3d.y(), gripper3d.z()};
                Eigen::Vector3f coord_w{T * coord_g};
                octomap::point3d world3d{coord_w.x(), coord_w.y(), coord_w.z()};

                // colorise nodes
                octomap::OcTreeGraspQualityNode* n = target_tree_.search(world3d);
                octomap::ColorOcTreeNode::Color color{0,0,0};
                if (n) // if not unknown, occupied by both octrees
                {
                    if (it->isGraspingSurface()) color.g = 255;
                    else color.r = 255;
                }
                else color.b = 255; // only populated by gripper

                octomap::ColorOcTreeNode* nn = color_tree.updateNode(gripper3d, true);
                nn->setColor(color);
            }
        }
        #endif

        // ! Below code plots voxels with spacing between them due to lower resolution than color_tree, hence it does not work as desired
        #if 0
        if (show_target_voxels) // if target voxels shall be plotted, iterate over local bbx of target tree
        {
            target_tree_.expand();

            double x,y,z;
            this->gripper_tree_.getMetricMax(x,y,z);
            x += BBX_margin;
            y += BBX_margin;
            z += BBX_margin;
            octomap::point3d max_bbx{(float)x,(float)y,(float)z};
            this->gripper_tree_.getMetricMin(x,y,z);
            x -= BBX_margin;
            y -= BBX_margin;
            z -= BBX_margin;
            octomap::point3d min_bbx{(float)x,(float)y,(float)z};
            color_tree.setBBXMax(max_bbx);
            color_tree.setBBXMin(min_bbx);

            // iterate over target octree
            for (octomap::OcTreeGraspQuality::leaf_iterator it = target_tree_.begin_leafs(), end=target_tree_.end_leafs(); it!= end; ++it)
            {
                octomap::point3d world3d{it.getCoordinate()};

                Eigen::Vector3f coord_w{world3d.x(), world3d.y(), world3d.z()};
                Eigen::Vector3f coord_g{T.inverse() * coord_w}; // TODO optimise inverse call
                octomap::point3d gripper3d{coord_g.x(), coord_g.y(), coord_g.z()};

                octomap::ColorOcTreeNode* n = color_tree.search(gripper3d);
                if (!n && color_tree.inBBX(gripper3d)) // if not null already exists, and if outside of BBX not within grasp local region
                {
                    octomap::ColorOcTreeNode* nn = color_tree.updateNode(gripper3d, true);
                    nn->setColor(0,0,50); // dark blue for target-only nodes
                }
            }
        }
        #endif

        color_tree.updateInnerOccupancy();
        return color_tree;
    }

    /**
     * Grasp visualiser using target and gripper models stored in object
     * @param T homogenous transformation from origin to gripper grasping pose
     * @returns ColorOcTree visualisation with: \n Green -> Positive overlapping voxels; \n Red -> Negative overlapping voxels; \n Light blue -> Non-interacting Gripper voxels; \n Dark blue -> Non-interacting Target voxels.
     * TODO Make multithreaded search execution in method 0 work
     */
    octomap::ColorOcTree visualise_global_grasp(const Eigen::Affine3f& T = Eigen::Affine3f::Identity())
    {
        std::cout << "[visualise_global_grasp] started..." << std::endl;
        #define ITERATION_METHOD 1 // 0 = spatial iteration, 1 = octree nodes iteration
        octomap::ColorOcTree color_tree{std::max(target_tree_.getResolution(),gripper_tree_.getResolution())};

        //color_tree.expand(); // ? Necessary? probably not

        #if ITERATION_METHOD==0
        // *** Method 0 *** Spatial BBX iteration
        // set scene BBX
        double xt, xg, yt, yg, zt, zg;
        target_tree_.getMetricMax(xt,yt,zt);
        gripper_tree_.getMetricMax(xg,yg,zg);
        octomap::point3d max_bbx{(float)std::max(xt,xg), (float)std::max(yt, yg), (float)std::max(zt, zg)};
        target_tree_.getMetricMin(xt,yt,zt);
        gripper_tree_.getMetricMin(xg,yg,zg);
        octomap::point3d min_bbx{(float)std::min(xt,xg), (float)std::min(yt, yg), (float)std::min(zt, zg)};
        color_tree.setBBXMax(max_bbx);
        color_tree.setBBXMin(min_bbx);

        // ? Comment out/remove cout
        std::cout << "Resolution target: " << target_tree_.getResolution() << ", resolution gripper: " << gripper_tree_.getResolution() << ", color tree resolution: " << color_tree.getResolution() << std::endl;
        std::cout << "minBBX: " << min_bbx << std::endl;
        std::cout << "maxBBX: " << max_bbx << std::endl;
        std::cout << "MetricSize: " << color_tree.getBBXBounds() << std::endl;
        
        // set up variables for progress tracking
        //unsigned int total_x_steps{(color_tree.getBBXMax().x() - color_tree.getBBXMin().x()) / color_tree.getResolution()/2};
        unsigned int current_x_step{0};
        
        // Spatially iterate over BBX
        for (float x = color_tree.getBBXMin().x(); x < color_tree.getBBXMax().x(); x = x + (float)color_tree.getResolution()/2)
        {
            for (float y = color_tree.getBBXMin().y(); y < color_tree.getBBXMax().y(); y = y + (float)color_tree.getResolution()/2)
            {
                for (float z = color_tree.getBBXMin().z(); z < color_tree.getBBXMax().z(); z = z + (float)color_tree.getResolution()/2)
                {
                    // transform coordinates according to T
                    Eigen::Vector3f coord_g{x,y,z};
                    Eigen::Vector3f coord_w{T * coord_g};
                    octomap::point3d world3d{coord_w.x(), coord_w.y(), coord_w.z()};

                    // search for corresponding nodes in octrees
                    octomap::OcTreeGripperNode* gn = gripper_tree_.search(x,y,z);
                    //std::thread th1(threaded_octree_search, std::ref(gripper_tree_), x, y, z, std::ref(gn));

                    octomap::OcTreeGraspQualityNode* tn = target_tree_.search(world3d);

                    // colorise nodes
                    octomap::ColorOcTreeNode::Color color{0,0,0};
                    if (!tn && !gn) // if both nodes null, node is free
                    {
                        //color_tree.updateNode(x, y, z, false); // ? By understanding unknown state as free, we don't really need to explicitely set to free
                    }
                    else // occupied
                    {
                        octomap::ColorOcTreeNode* n = color_tree.updateNode(world3d, true);

                        if (!tn != !gn) // XOR
                        {
                            // node is not occupied by both octrees
                            if (tn) color.b = 100; // dark blue
                            else color.b = 255; // light blue
                        }
                        else // node occupied by both gripper and target trees
                        {
                            if (gn->isGraspingSurface()) color.g = 255;
                            else color.r = 255;
                        }
                        n->setColor(color);
                    }
                }
            }

            // print progress
            std::cout << '\r' << current_x_step << "steps" << std::flush;
            current_x_step++;
        }
        #endif

        #if ITERATION_METHOD==1
        // *** Method 1 *** Iterate over both octree nodes

        target_tree_.expand();
        gripper_tree_.expand();

        // iterate over target octree
        for (octomap::OcTreeGraspQuality::leaf_iterator it = target_tree_.begin_leafs(), end=target_tree_.end_leafs(); it!= end; ++it)
        {
            octomap::point3d world3d{it.getCoordinate()};
            
            // colorise nodes
            octomap::ColorOcTreeNode::Color color{0,0,0};
            octomap::ColorOcTreeNode* n = color_tree.updateNode(world3d, true);
            color.b = 50;
            n->setColor(color);
        }

        // iterate over gripper octree
        for (octomap::OcTreeGripper::leaf_iterator it = gripper_tree_.begin_leafs(), end=gripper_tree_.end_leafs(); it!= end; ++it)
        {
            // transform coordinates according to T
            octomap::point3d gripper3d{it.getCoordinate()};
            Eigen::Vector3f coord_g{gripper3d.x(), gripper3d.y(), gripper3d.z()};
            Eigen::Vector3f coord_w{T * coord_g};
            octomap::point3d world3d{coord_w.x(), coord_w.y(), coord_w.z()};

            // colorise nodes
            octomap::ColorOcTreeNode* n = color_tree.search(world3d);
            octomap::ColorOcTreeNode::Color color{0,0,0};
            if (n) // if not unknown
            {
                int color_b_node{n->getColor().b};
                if (0 < color_b_node && color_b_node < 255) // if node is blue but not _fully_ blue (255b is associated with gripper-only nodes)
                {
                    if (it->isGraspingSurface()) color.g = 255;
                    else color.r = 255;
                    n->setColor(color);
                }
            }
            else // only populated by gripper
            {
                octomap::ColorOcTreeNode* nn = color_tree.updateNode(world3d, true);
                color.b = 255;
                nn->setColor(color);
            }
        }
        #endif
        color_tree.updateInnerOccupancy();
        //color_tree.prune();
        return color_tree;
    }

    // can use castRay to determine distance to closest voxel...

private:
    /**
     * Task definition for octree search using threaded workers
     * @param octree Octree
     * @param x x-coordiante
     * @param y y-coordinate
     * @param z z-coordinate
     * @param n octree node pointer found
     * ! Still not used/working
     */
    template<typename NODE>
    static void threaded_octree_search(const octomap::OccupancyOcTreeBase<NODE>& tree, double x, double y, double z, NODE& n) 
    {
        n = tree.search(x, y, z);
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
     * Count the number of voxels within the graspable region that collide with voxels from the target
     * @param T homogenous transformation from origin to gripper grasping pose
     */
    float gp_voxelsuperimposition(const Eigen::Affine3f& T)
    {
        float score{0};
        float reward{1}; // reward for positive node interaction
        float penalty{1}; // penalty for negative node interaction

        gripper_tree_.expand(); // expand to standardise the size of all voxels in score counting

        for (octomap::OcTreeGripper::leaf_iterator it = gripper_tree_.begin_leafs(), end=gripper_tree_.end_leafs(); it!= end; ++it)
        {
            octomap::point3d coord{it.getCoordinate()};
            
            // transform coordinates according to T
            Eigen::Vector3f coord_g{coord.x(),coord.y(),coord.z()};
            Eigen::Vector3f coord_w{T * coord_g};
            octomap::point3d world3d{coord_w.x(), coord_w.y(), coord_w.z()};

            octomap::OcTreeGraspQualityNode* n = target_tree_.search(world3d);
            if (n) // if target node exists
            {
                if(it->isGraspingSurface()) score += reward;
                else score -= penalty;
            }
        }
        return score;
    }

    /**
     * Calculate the average surface normal of the region of the target colliding with the graspable voxels, and compare it against the ideal surface normal
     * @param T homogenous transformation from origin to gripper grasping pose
     * TODO algorithm
     */
    float gp_surfacenormals(const Eigen::Affine3f& T)
    {
        float score{0};
        // ? Can create a sub-octree that is just the area of the target that COLLIDES with the graspable voxels

        return score;
    }

    /**
     * Check for voxel superimposition score and surface normals
     * @param T homogenous transformation from origin to gripper grasping pose
     * @param voxel_normal_ratio Percentage (in 1 scale) weight given to voxel superimposition method against surface normal method
     */
    float gp_voxelsuperimposition_surfacenormals(const Eigen::Affine3f& T, float voxel_normal_ratio = 0.5)
    {
        float score{gp_voxelsuperimposition(T) * voxel_normal_ratio + gp_surfacenormals(T) * (1-voxel_normal_ratio)};
        return score;
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
                    } 
                    else // If node exists
                    {
                        // TODO Check this updates node at lowest depth level
                        if (n->getOccupancy() < 0.5) // if node is free
                            n->setLogOdds(this->gripper_tree_.getProbHitLog());
                        n->setIsGraspingSurface(true);
                    }
                }
            }
        }
        octomap::point3d center_bbx{(max-min)*0.5 + min};
        std::cout << "min" << min << std::endl << "max" << max << std::endl;
        std::cout << "[add_graspable_region] center of graspable region is at: " << center_bbx << std::endl;
        return center_bbx;
    }

    octomap::point3d sensor_origin_;
    octomap::OcTreeGraspQuality target_tree_;
    octomap::OcTreeGripper gripper_tree_;
    octomap::point3d normal_gripper_;
};