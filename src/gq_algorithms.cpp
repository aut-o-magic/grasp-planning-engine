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
    inline float gq_voxelsuperimposition(const Eigen::Affine3f& T, const octomap::OcTreeGraspQuality* target_tree_, const octomap::OcTreeGripper* gripper_tree_)
    {
        float score{0};
        float reward{1}; // reward for positive node interaction
        float penalty{1}; // penalty for negative node interaction

        for (octomap::OcTreeGripper::leaf_iterator it = gripper_tree_->begin_leafs(), end=gripper_tree_->end_leafs(); it!= end; ++it)
        {
            octomap::point3d gripper3d{it.getCoordinate()};
            octomap::point3d world3d{GraspPlanningUtils::transform_point3d(T, gripper3d)};

            octomap::OcTreeGraspQualityNode* n = target_tree_->search(world3d);
            if (n && target_tree_->isNodeOccupied(n)) // if target node is occupied
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
     * TODO avoid hardcoding grasping plane here, in graspingPairs fcn call
     */
    inline float gq_surfacenormals(const Eigen::Affine3f& T, const octomap::OcTreeGraspQuality* target_tree_, const octomap::OcTreeGripper* gripper_tree_)
    {
        if (grasping_pairs.empty()) grasping_pairs = GraspPlanningUtils::graspingPairs("yz", gripper_tree_); // only initialise once, as this fcn call is slow

        // rotate grasping normal gripper
        octomap::point3d graspingnormal = gripper_tree_->getGraspingNormal();
        Eigen::Vector3f graspingnormal_eigen{graspingnormal.x(), graspingnormal.y(), graspingnormal.z()};
        Eigen::Vector3f graspingnormal_rotated_eigen = T.rotation() * graspingnormal_eigen;
        octomap::point3d graspingnormal_rotated{graspingnormal_rotated_eigen.x(), graspingnormal_rotated_eigen.y(), graspingnormal_rotated_eigen.z()};
        graspingnormal_rotated.normalize();

        std::vector<int> histogram_angles_left (91,0); // create 91 (0,90) fields populated with 0s
        std::vector<int> histogram_angles_right (91,0); // create 91 (0,90) fields populated with 0s

        for(auto it = grasping_pairs.begin(); it != grasping_pairs.end(); ++it)
        {
            // retrieve pair coordinates
            octomap::point3d gripper3d_left{it->first.getCoordinate()};
            octomap::point3d gripper3d_right{it->second.getCoordinate()};

            // transform coordinates according to T
            octomap::point3d world3d_left{GraspPlanningUtils::transform_point3d(T, gripper3d_left)};
            octomap::point3d world3d_right{GraspPlanningUtils::transform_point3d(T, gripper3d_right)};
            
            octomap::point3d direction{(world3d_right-world3d_left).normalized()}; // First to second
            const double distance{abs(world3d_right.distance(world3d_left))};
            octomap::point3d hit_left;
            octomap::point3d hit_right;

            // build histogram of angles
            if (target_tree_->castRay(world3d_left, direction, hit_left, true, distance)) // if occupied node was hit
            {
                octomap::OcTreeGraspQualityNode* n = target_tree_->search(hit_left);
                if (n && target_tree_->isNodeOccupied(n)) // if occupied
                {
                    octomap::point3d_collection points = GraspPlanningUtils::get_surface_normals(target_tree_, hit_left);
                    if (!points.empty())
                    {
                        int best_normal_option_angle = -1;
                        for (octomap::point3d_collection::iterator it_3d = points.begin(); it_3d != points.end(); ++it_3d)
                        {
                            float rot_angle_rad{GraspPlanningUtils::safe_angleTo(graspingnormal_rotated,*it_3d)};
                            int angle_deg = (int)(abs(cos((rot_angle_rad))*90)); // angle always between (0-90)
                            if (angle_deg < 0 || angle_deg >90) std::cerr << "OUTOFBOUNDS left ANGLEDEG=" << angle_deg << std::endl;
                            if (angle_deg > best_normal_option_angle) best_normal_option_angle = angle_deg;
                        }
                        ++histogram_angles_left[best_normal_option_angle];
                    }
                }
            }
            if (target_tree_->castRay(world3d_right, -direction, hit_right, true, distance)) // if occupied node was hit
            {
                octomap::OcTreeGraspQualityNode* n = target_tree_->search(hit_right);
                if (n && target_tree_->isNodeOccupied(n)) // if occupied
                {
                    octomap::point3d_collection points = GraspPlanningUtils::get_surface_normals(target_tree_, hit_right);
                    if (!points.empty())
                    {
                        int best_normal_option_angle = -1;
                        for (octomap::point3d_collection::iterator it_3d = points.begin(); it_3d != points.end(); ++it_3d)
                        {
                            float rot_angle_rad{GraspPlanningUtils::safe_angleTo(graspingnormal_rotated,*it_3d)};
                            int angle_deg = (int)(abs(cos((rot_angle_rad))*90)); // angle always between (0-90)
                            if (angle_deg < 0 || angle_deg >90) std::cerr << "OUTOFBOUNDS right ANGLEDEG=" << angle_deg << std::endl;
                            if (angle_deg > best_normal_option_angle) best_normal_option_angle = angle_deg;
                        }
                        ++histogram_angles_right[best_normal_option_angle];
                    }
                }
            }
        }
        #ifdef write_csv
        std::ofstream myfile;
        myfile.open("method2histogram.csv");
        myfile << "angle,left,right\n";
        for (unsigned int i=0; i <= histogram_angles_left.size()-1; ++i)
        {
            myfile << i << "," << histogram_angles_left[i] << "," << histogram_angles_right[i] << "\n";
            std::cout << "histo[" << i << "]: left=" << histogram_angles_left[i] << ", right=" << histogram_angles_right[i] << std::endl;
        }
        myfile.close();
        #endif

        // merge histograms into eigen array
        Eigen::Array<int, 91, 1> histogram_combined;
        std::transform(histogram_angles_left.begin(), histogram_angles_left.end(), histogram_angles_right.begin(), histogram_combined.data(), std::plus<int>());
        
        // calculate mean and std dev
        const unsigned int samples = histogram_combined.sum();
        if (samples == 0) return 0.0F;
        const int mean = (histogram_combined * Eigen::Array<int, 91, 1>::LinSpaced(0,90)).sum()/samples;
        const float std_dev = sqrt((histogram_combined * Eigen::Array<int, 91, 1>::LinSpaced(0-mean,90-mean)).square().sum() / samples);
        
        /* median unused right now
        // calculate median
        int median = 0;
        int accumulator = 0;
        int i = 0;
        while (accumulator < (int)samples/2)
        {
            int new_bin = histogram_combined[i];
            if (abs((int)samples/2 - accumulator - new_bin) < abs((int)samples/2 - accumulator)) median = i;
            accumulator += new_bin;
            ++i;
        }
        */
        //std::cout << "Samples = " << samples << ", mean = " << mean << ", median = " << median << ", std dev = " << std_dev << std::endl;

        // assign score based on heuristic linear regression slopes
        const float weight_mean{0.5F}; // 50% of total score comes from the mean, other 50% from std dev
        float score = ((float)mean)/90.0F * weight_mean + 100.0F/std::max(std_dev,100.0F) * (1.0F-weight_mean); // TODO Finetune std_dev formula for score
        //std::cout << "*****score = " << score << ", meanscore = " << ((float)mean)/90.0F << "stdscore = " << 100.0F/std::max(std_dev,100.0F) << std::endl;
        // ? Maybe use median and mean, instead of std dev?
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
    inline float gq_voxelsuperimposition_surfacenormals(const Eigen::Affine3f& T, const octomap::OcTreeGraspQuality* target_tree_, const octomap::OcTreeGripper* gripper_tree_)//, float voxel_normal_ratio = 0.5)
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
     */
    inline float gq_raycasting(const Eigen::Affine3f& T, const octomap::OcTreeGraspQuality* target_tree_, const octomap::OcTreeGripper* gripper_tree_)
    {
        if (grasping_pairs.empty()) grasping_pairs = GraspPlanningUtils::graspingPairs("yz", gripper_tree_); // only initialise once, as this fcn call is slow

        // rotate grasping normal gripper
        octomap::point3d graspingnormal = gripper_tree_->getGraspingNormal();
        Eigen::Vector3f graspingnormal_eigen{graspingnormal.x(), graspingnormal.y(), graspingnormal.z()};
        Eigen::Vector3f graspingnormal_rotated_eigen = T.rotation() * graspingnormal_eigen;
        octomap::point3d grasping_normal_rotated{graspingnormal_rotated_eigen.x(), graspingnormal_rotated_eigen.y(), graspingnormal_rotated_eigen.z()};
        grasping_normal_rotated.normalize();

        float score{0};

        for(auto it = grasping_pairs.begin(); it != grasping_pairs.end(); ++it)
        {
            // retrieve pair coordinates
            octomap::point3d gripper3d_left{it->first.getCoordinate()};
            octomap::point3d gripper3d_right{it->second.getCoordinate()};

            // transform coordinates according to T
            octomap::point3d world3d_left{GraspPlanningUtils::transform_point3d(T, gripper3d_left)};
            octomap::point3d world3d_right{GraspPlanningUtils::transform_point3d(T, gripper3d_right)};
            
            octomap::point3d direction{(world3d_right-world3d_left).normalized()}; // First to second
            const double distance{abs(world3d_right.distance(world3d_left))};
            octomap::point3d hit_left;
            octomap::point3d hit_right;

            // build histogram of angles
            if (target_tree_->castRay(world3d_left, direction, hit_left, true, distance)) // if occupied node was hit
            {
                octomap::OcTreeGraspQualityNode* n = target_tree_->search(hit_left);
                if (n && target_tree_->isNodeOccupied(n)) // if occupied
                {
                    octomap::point3d_collection points = GraspPlanningUtils::get_surface_normals(target_tree_, hit_left);
                    int best_normal_option_angle = -1;
                    for (octomap::point3d_collection::iterator it_3d = points.begin(); it_3d != points.end(); ++it_3d)
                    {
                        float rot_angle_rad{GraspPlanningUtils::safe_angleTo(grasping_normal_rotated,*it_3d)};
                        int angle_deg = (int)(abs(cos((rot_angle_rad))*90)); // angle always between (0-90)
                        if (angle_deg < 0 || angle_deg >90) std::cerr << "OUTOFBOUNDS left ANGLEDEG=" << angle_deg << std::endl;
                        if (angle_deg > best_normal_option_angle) best_normal_option_angle = angle_deg;
                    }
                    score += ((float)(best_normal_option_angle-45))/45.0F; // a 45 deg angle would give 0 reward/penalty, a 0 or 90 would give (1) penalty/reward, respectively.
                }
            }
            if (target_tree_->castRay(world3d_right, -direction, hit_right, true, distance)) // if occupied node was hit
            {
                octomap::OcTreeGraspQualityNode* n = target_tree_->search(hit_right);
                if (n && target_tree_->isNodeOccupied(n)) // if occupied
                {
                    octomap::point3d_collection points = GraspPlanningUtils::get_surface_normals(target_tree_, hit_right);
                    int best_normal_option_angle = -1;
                    for (octomap::point3d_collection::iterator it_3d = points.begin(); it_3d != points.end(); ++it_3d)
                    {
                        float rot_angle_rad{GraspPlanningUtils::safe_angleTo(grasping_normal_rotated,*it_3d)};
                        int angle_deg = (int)(abs(cos((rot_angle_rad))*90)); // angle always between (0-90)
                        if (angle_deg < 0 || angle_deg >90) std::cerr << "OUTOFBOUNDS right ANGLEDEG=" << angle_deg << std::endl;
                        if (angle_deg > best_normal_option_angle) best_normal_option_angle = angle_deg;
                    }
                    score += ((float)(best_normal_option_angle-45))/45.0F; // a 45 deg angle would give 0 reward/penalty, a 0 or 90 would give (1) penalty/reward, respectively.
                }
            }
        }
        // normalise score
        score /= (float)grasping_pairs.size()*2.0F; // divide by sample size
        if (score>1.0F)
        {
            std::cerr << "[gq_raycasting] score is " << score << ", not properly normalised" << std::endl;
        }

        return score;
    }
}

