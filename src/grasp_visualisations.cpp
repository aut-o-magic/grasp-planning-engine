#pragma once

#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>
#include "octomap_grasping/OcTreeGraspQuality.hpp"
#include "octomap_grasping/OcTreeGripper.hpp"
#include "gp_utils.cpp"

/**
 * Translated copy of the origin of a color octree
 * @param tree Original color octree ptr
 * @param translation Linear translation vector to new origin
 * @returns New color octree with translated origin
 * ? Could be made to also rotate the octree but that seems unnecessary
 */
inline octomap::ColorOcTree translated_ColorOcTree(octomap::ColorOcTree& tree, const octomap::point3d& translation)
{
    tree.expand();
    
    // match translation to resolution steps
    float res{(float)tree.getResolution()};
    float x{std::round(translation.x()/res)*res};
    float y{std::round(translation.y()/res)*res};
    float z{std::round(translation.z()/res)*res};
    octomap::point3d matched_translation{x,y,z};

    // new octree
    octomap::ColorOcTree new_tree{tree.getResolution()};

    // iterate over current tree and copy transformed nodes to new tree
    for (octomap::ColorOcTree::leaf_iterator it = tree.begin_leafs(), end=tree.end_leafs(); it!= end; ++it)
    {
        octomap::point3d coord{it.getCoordinate() - matched_translation};
        octomap::ColorOcTreeNode* n = new_tree.updateNode(coord, it->getLogOdds(), true);
        n->setColor(it->getColor());

    }
    new_tree.updateInnerOccupancy();
    return new_tree;
}

namespace GraspVisualisations
{
    /**
     * Visualise surface normals density of target tree using filtered marching cubes surface reconstruction algorithm.
     * Density color coding: white = 0, red = 1, green = 2, blue = 3, black => 4
     * @param target_tree_ Target object octree
     */
    inline void visualise_surface_normals_density(const octomap::OcTreeGraspQuality* target_tree_)
    {
        std::cout << "[visualise_surface_normals_density] started..." << std::endl;
        octomap::ColorOcTree color_tree_normals_density{target_tree_->getResolution()};
        std::vector<unsigned int> normals_vec(5); // (0,1,2,3,3+)

        for (octomap::OcTreeGraspQuality::leaf_iterator it = target_tree_->begin_leafs(), end=target_tree_->end_leafs(); it!= end; ++it)
        {
            octomap::point3d_collection normals;// = GraspPlanningUtils::get_filtered_surface_normals(target_tree_, it.getCoordinate());
            if(!target_tree_->getNormal(it.getKey(), normals)) std::cerr << "[visualise_surface_normals_density] getNormal returned false" << std::endl; // ! Toggle to use getNormal()

            // Populate color_tree_normals_density object
            octomap::ColorOcTreeNode* snn = color_tree_normals_density.updateNode(it.getCoordinate(), it->getLogOdds(), true);
            octomap::ColorOcTreeNode::Color color{0,0,0};
            // black = 0, 1 <= red ... green
            if (!normals.empty())
            {
                color.r = std::max((unsigned long)0, 255-(normals.size()*51));
                color.g = std::min((unsigned long)255, 0+(normals.size()*51));
            }
            normals_vec[std::min(normals.size(),normals_vec.size()-1)]++;            
            snn->setColor(color);
        }

        color_tree_normals_density.updateInnerOccupancy();

        std::cout << "[Surface normals density study]:" << std::endl << "Size collection: [0]=" << normals_vec[0] << " [1]=" << normals_vec[1] << " [2]=" << normals_vec[2] << " [3]=" << normals_vec[3] << " [3+]=" << normals_vec[4] << std::endl;
        color_tree_normals_density.write("color_tree_normals_density.ot");
    }

    /**
     * Local grasp visualiser showing the region only in the BBX of the gripper octree
     * @param target_tree_ Target object octree
     * @param gripper_tree_ Gripper octree
     * @param show_target_voxels Show/hide the target octree voxels
     * @param T homogenous transformation from origin to gripper grasping pose
     * @param BBX_margin Add additional margins to the BBX drawn for better occupancy context
     * @returns ColorOcTree visualisation with: \n Green -> Positive overlapping voxels; \n Red -> Negative overlapping voxels; \n Light blue -> Non-interacting Gripper voxels; \n Dark blue -> Non-interacting Target voxels.
     */
    inline octomap::ColorOcTree visualise_local_grasp(const octomap::OcTreeGraspQuality* target_tree_, const octomap::OcTreeGripper* gripper_tree_, bool show_target_voxels = false, const Eigen::Affine3f& T = Eigen::Affine3f::Identity(), float BBX_margin = 0)
    {
        //#define ALWAYS_CARTESIAN_BBX_METHOD 
        // ndef-> Use Node ptr iteration (faster) when target voxels don't have to be shown,
        // def-> Always use BBX cartesian iteration regardless of show_target_voxels parameter
        
        std::cout << "[visualise_local_grasp] started..." << std::endl;
        octomap::ColorOcTree color_tree{std::min(gripper_tree_->getResolution(), target_tree_->getResolution())};

        #ifndef ALWAYS_CARTESIAN_BBX_METHOD
        if (show_target_voxels)
        {
        #endif
            // *** Method 0 *** Spatial BBX iteration
            double x,y,z;
            gripper_tree_->getMetricMax(x,y,z);
            x += BBX_margin;
            y += BBX_margin;
            z += BBX_margin;
            octomap::point3d max_bbx{(float)x,(float)y,(float)z};
            gripper_tree_->getMetricMin(x,y,z);
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
                        octomap::point3d gripper3d{x,y,z};
                        octomap::point3d world3d{GraspPlanningUtils::transform_point3d(T, gripper3d)};

                        // search for corresponding nodes in octrees
                        octomap::OcTreeGripperNode* gn = gripper_tree_->search(gripper3d);
                        octomap::OcTreeGraspQualityNode* tn = target_tree_->search(world3d);

                        // colorise nodes
                        octomap::ColorOcTreeNode::Color color{0,0,0};
                        float log_odds;
                        if (gn) // if gripper node present
                        {
                            if (tn && target_tree_->isNodeOccupied(tn)) // if target node is occupied
                            {
                                log_odds = tn->getLogOdds();
                                if (gn->isGraspingSurface()) color.g = 255;
                                else color.r = 255;
                            }
                            else // gripper-only nodes
                            {
                                log_odds = gn->getLogOdds();
                                if (gn->isGraspingSurface()) color.g = 255; // green for grasping (free) nodes
                                else color.b = 255; // light blue for structure nodes
                            }
                        }
                        else if (tn && show_target_voxels)
                        {
                            log_odds = tn->getLogOdds();
                            color.b = 50; // dark blue for target-only nodes
                            
                        }
                        else continue;
                        octomap::ColorOcTreeNode* n = color_tree.updateNode(gripper3d, log_odds, true);
                        n->setColor(color);
                    }
                }
            }
        #ifndef ALWAYS_CARTESIAN_BBX_METHOD
        }
        else
        {
            // *** Method 1 *** Iterate over octree node ptrs, does not support showing target voxels
            // iterate over gripper octree
            for (octomap::OcTreeGripper::leaf_iterator it = gripper_tree_->begin_leafs(), end=gripper_tree_->end_leafs(); it!= end; ++it)
            {
                // transform coordinates according to T
                octomap::point3d gripper3d{it.getCoordinate()};
                octomap::point3d world3d{GraspPlanningUtils::transform_point3d(T, gripper3d)};

                // colorise nodes
                octomap::OcTreeGraspQualityNode* n = target_tree_->search(world3d);
                octomap::ColorOcTreeNode::Color color{0,0,0};
                float log_odds;
                if (n && target_tree_->isNodeOccupied(n)) // if target has an occupied node in that location
                {
                    log_odds = n->getLogOdds();
                    if (it->isGraspingSurface()) color.g = 255;
                    else color.r = 255;
                }
                else // only populated by gripper
                {
                    log_odds = it->getLogOdds();
                    if (it->isGraspingSurface()) color.g = 255;
                    else color.b = 255; 
                }
                octomap::ColorOcTreeNode* nn = color_tree.updateNode(gripper3d, log_odds, true);
                nn->setColor(color);
            }
        }
        #endif

        color_tree.updateInnerOccupancy();
        return color_tree;
    }

    /**
     * Grasp visualiser using target and gripper models stored in object, origin set to gripper
     * @param target_tree_ Target object octree
     * @param gripper_tree_ Gripper octree
     * @param T homogenous transformation from origin to gripper grasping pose
     * @returns ColorOcTree visualisation with: \n Green -> Positive overlapping voxels; \n Red -> Negative overlapping voxels; \n Yellow -> Non-interacting Gripper voxels; \n Light blue -> Non-interacting Target voxels.
     * TODO Make multithreaded search execution in method 0 work
     */
    inline octomap::ColorOcTree visualise_global_grasp(const octomap::OcTreeGraspQuality* target_tree_, const octomap::OcTreeGripper* gripper_tree_, const Eigen::Affine3f& T = Eigen::Affine3f::Identity())
    {
        std::cout << "[visualise_global_grasp] started..." << std::endl;
        octomap::ColorOcTree color_tree{std::max(target_tree_->getResolution(),gripper_tree_->getResolution())};
        octomap::point3d origin_offset{T.translation().x(), T.translation().y(), T.translation().z()}; // origin of color_tree will be offset by this amount to focus on gripper
        
        // * Iterate over both octree nodes
        // iterate over target octree
        for (octomap::OcTreeGraspQuality::leaf_iterator it = target_tree_->begin_leafs(), end=target_tree_->end_leafs(); it!= end; ++it)
        {
            octomap::point3d world3d{it.getCoordinate()};
            
            // colorise nodes
            octomap::ColorOcTreeNode::Color color{0,0,0};
            octomap::ColorOcTreeNode* n = color_tree.updateNode(world3d, it->getLogOdds(), true);
            color.b = 255;
            n->setColor(color);
        }
        color_tree.updateInnerOccupancy();

        // iterate over gripper octree
        for (octomap::OcTreeGripper::leaf_iterator it = gripper_tree_->begin_leafs(), end=gripper_tree_->end_leafs(); it!= end; ++it)
        {
            // transform coordinates according to T
            octomap::point3d gripper3d{it.getCoordinate()};
            octomap::point3d world3d{GraspPlanningUtils::transform_point3d(T, gripper3d)};

            // colorise nodes
            octomap::ColorOcTreeNode* n = color_tree.search(world3d);
            octomap::ColorOcTreeNode::Color color{0,0,0};
            if (n && color_tree.isNodeOccupied(n)) // if occupied
            {
                int color_b_node{n->getColor().b};
                if (color_b_node == 255) // if node fully blue (255b is associated with target nodes)
                {
                    if (it->isGraspingSurface()) color.g = 255;
                    else color.r = 255;
                    n->setColor(color);
                }
            }
            else // only populated by gripper
            {
                octomap::ColorOcTreeNode* nn = color_tree.updateNode(world3d, it->getLogOdds(), true);
                if (it->isGraspingSurface()) color.g = 255; // grasping (free) nodes
                else {color.r = 255; color.g = 255;}
                nn->setColor(color);
            }
        }
        color_tree.updateInnerOccupancy();

        return translated_ColorOcTree(color_tree, origin_offset);
    }
}