#pragma once

#include <octomap/octomap.h>
#include "octomap_grasping/OcTreeGraspQuality.hpp"
#include "octomap_grasping/OcTreeGripper.hpp"
#include "gp_utils.cpp"

namespace GraspQualityMethods
{
    static std::vector<std::pair<octomap::OcTreeGripper::iterator, octomap::OcTreeGripper::iterator>> grasping_pairs; // TODO Set as private --somehow?

    /**
     * Count the number of voxels within the graspable region that collide with voxels from the target
     * @param T homogenous transformation from origin to gripper grasping pose
     * @param target_tree_ Target object octree
     * @param gripper_tree_ Gripper octree
     * @returns Normalised grasp quality score between [0,1]
     * ? Alternative approach would be with computeRay(). May be much faster?
     */
    static float gq_voxelsuperimposition(const Eigen::Affine3f& T, const octomap::OcTreeGraspQuality* target_tree_, octomap::OcTreeGripper* gripper_tree_)
    {
        float score{0};
        float reward{1}; // reward for positive node interaction
        float penalty{1}; // penalty for negative node interaction

        gripper_tree_->expand(); // expand to standardise the size of all voxels in score counting

        for (octomap::OcTreeGripper::leaf_iterator it = gripper_tree_->begin_leafs(), end=gripper_tree_->end_leafs(); it!= end; ++it)
        {
            octomap::point3d coord{it.getCoordinate()};
            
            // transform coordinates according to T
            Eigen::Vector3f coord_g{coord.x(),coord.y(),coord.z()};
            Eigen::Vector3f coord_w{T * coord_g};
            octomap::point3d world3d{coord_w.x(), coord_w.y(), coord_w.z()};

            octomap::OcTreeGraspQualityNode* n = target_tree_->search(world3d);
            if (n && n->getOccupancy() > 0.5) // if target node is occupied
            {
                if(it->isGraspingSurface()) score += reward;
                else score -= penalty;
            }
        }
        float score_norm{std::max(score/gripper_tree_->getNumGraspableVoxels(),0.0F)}; // constrained between [0,1]
        return score_norm;
    }

    /**
     * Calculate the average surface normal of the region of the target colliding with the graspable voxels, and compare it against the ideal surface normal
     * @param T homogenous transformation from origin to gripper grasping pose
     * @param target_tree_ Target object octree
     * @param gripper_tree_ Gripper octree
     * @returns Normalised grasp quality score between [0,1]
     * TODO algorithm
     * TODO avoid hardcoding grasping plane here, in graspingPairs fcn call
     */
    static float gq_surfacenormals(const Eigen::Affine3f& T, const octomap::OcTreeGraspQuality* target_tree_, octomap::OcTreeGripper* gripper_tree_)
    {
        if (grasping_pairs.empty()) grasping_pairs = GraspPlanningUtils::graspingPairs("yz", gripper_tree_); // only initialise once, as this fcn call is slow

        std::vector<int> histogram_angles_left (360,0); // create 360 fields populated with 0s
        std::vector<int> histogram_angles_right (360,0); // create 360 fields populated with 0s

        for(auto it = grasping_pairs.begin(); it != grasping_pairs.end(); ++it)
        {
            // retrieve pair coordinates
            octomap::point3d coord_g_left_3d{it->first.getCoordinate()};
            octomap::point3d coord_g_right_3d{it->second.getCoordinate()};

            // transform coordinates according to T
            Eigen::Vector3f coord_g_left{coord_g_left_3d.x(),coord_g_left_3d.y(),coord_g_left_3d.z()};
            Eigen::Vector3f coord_g_right{coord_g_right_3d.x(),coord_g_right_3d.y(),coord_g_right_3d.z()};
            Eigen::Vector3f coord_w_left{T * coord_g_left};
            Eigen::Vector3f coord_w_right{T * coord_g_right};

            octomap::point3d coord_w_left_3d{coord_w_left.x(), coord_w_left.y(), coord_w_left.z()};
            octomap::point3d coord_w_right_3d{coord_w_right.x(), coord_w_right.y(), coord_w_right.z()};
            octomap::point3d direction{(coord_w_right_3d-coord_w_left_3d).normalized()}; // First to second
            octomap::point3d hit_left;
            octomap::point3d hit_right;

            // build histogram of angles
            if (target_tree_->castRay(coord_w_left_3d, direction, hit_left, true)) // if occupied node was hit
            {
                octomap::OcTreeGraspQualityNode* n = target_tree_->search(hit_left);
                if (n && n->getOccupancy() > 0.5) // if occupied
                {
                    octomap::point3d_collection points = GraspPlanningUtils::get_surface_normals(target_tree_, hit_left);
                    for (octomap::point3d_collection::iterator it_3d = points.begin(); it_3d != points.end(); ++it_3d)
                    {
                        float rot_angle_rad{(float)gripper_tree_->getGraspingNormal().angleTo(*it_3d)};
                        int angle_deg = (int)(rot_angle_rad/(M_PI)*180) + 180;
                        if (angle_deg < 0 || angle_deg >360) std::cout << "OUTOFBOUNDS ANGLEDEG" << std::endl; // ! Test properly and remove check
                        ++histogram_angles_left[angle_deg];
                    }
                }
            }
            if (target_tree_->castRay(coord_w_right_3d, -direction, hit_right, true)) // if occupied node was hit
            {
                octomap::OcTreeGraspQualityNode* n = target_tree_->search(hit_right);
                if (n && n->getOccupancy() > 0.5) // if occupied
                {
                    octomap::point3d_collection points = GraspPlanningUtils::get_surface_normals(target_tree_, hit_right);
                    for (octomap::point3d_collection::iterator it_3d = points.begin(); it_3d != points.end(); ++it_3d)
                    {
                        float rot_angle_rad{(float)gripper_tree_->getGraspingNormal().angleTo(*it_3d)};
                        int angle_deg = (int)(rot_angle_rad/(M_PI)*180) + 180;
                        if (angle_deg < 0 || angle_deg >360) std::cout << "OUTOFBOUNDS ANGLEDEG" << std::endl; // ! Test properly and remove check
                        ++histogram_angles_right[angle_deg];
                    }
                }
            }
        }

        // TODO Quantify score based on angles histograms
        float score{0};

        return score;
    }

    /**
     * Check for voxel superimposition score and surface normals
     * @param T homogenous transformation from origin to gripper grasping pose
     * @param target_tree_ Target object octree
     * @param gripper_tree_ Gripper octree
     * @param voxel_normal_ratio Percentage (in 1 scale) weight given to voxel superimposition method against surface normal method
     * TODO find a way to pass weight ratio
     */
    static float gq_voxelsuperimposition_surfacenormals(const Eigen::Affine3f& T, const octomap::OcTreeGraspQuality* target_tree_, octomap::OcTreeGripper* gripper_tree_)//, float voxel_normal_ratio = 0.5)
    {
        float voxel_normal_ratio{0.5};
        float score{gq_voxelsuperimposition(T, target_tree_, gripper_tree_) * voxel_normal_ratio + gq_surfacenormals(T, target_tree_, gripper_tree_) * (1-voxel_normal_ratio)};
        return score;
    }

    /**
     * Cast ray between every node-pair in the surface of the anti-podal grasping plates of the gripper and compare surface normal of node it collides against
     * @param T homogenous transformation from origin to gripper grasping pose
     * @param target_tree_ Target object octree
     * @param gripper_tree_ Gripper octree
     * @returns Normalised grasp quality score between [0,1]
     * TODO algorithm
     */
    static float gq_raycasting(const Eigen::Affine3f& T, const octomap::OcTreeGraspQuality* target_tree_, octomap::OcTreeGripper* gripper_tree_)
    {
        float score{0};


        return score;
    }
}

