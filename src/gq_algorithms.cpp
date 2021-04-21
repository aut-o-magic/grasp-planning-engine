#pragma once

#include <octomap/octomap.h>
#include "octomap_grasping/OcTreeGraspQuality.hpp"
#include "octomap_grasping/OcTreeGripper.hpp"

namespace GraspQualityMethods
{
    /**
     * Count the number of voxels within the graspable region that collide with voxels from the target
     * @param T homogenous transformation from origin to gripper grasping pose
     * @param target_tree_ Target object octree
     * @param gripper_tree_ Gripper octree
     * @returns Normalised grasp quality score between [0,1]
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
            if (n) // if target node exists
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
     */
    static float gq_surfacenormals(const Eigen::Affine3f& T, const octomap::OcTreeGraspQuality* target_tree_, octomap::OcTreeGripper* gripper_tree_)
    {
        float score{0};
        // ? Can create a sub-octree that is just the area of the target that COLLIDES with the graspable voxels

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

