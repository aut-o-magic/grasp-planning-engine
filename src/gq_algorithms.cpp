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
        // ! Weights
        const float reward{1}; // reward for positive node interaction
        const float penalty{1}; // penalty for negative node interaction

        float score{0};

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
        // ! Weights
        const float weight_mean{0.5F}; // % (1-based) of total score comes from the mean, other fraction from std dev
        const float std_saturation{10.0F};

        if (grasping_pairs.empty()) grasping_pairs = GraspPlanningUtils::graspingPairs("yz", gripper_tree_); // only initialise once, as this fcn call is slow

        // rotate grasping normal gripper
        octomap::point3d graspingnormal = gripper_tree_->getGraspingNormal();
        Eigen::Vector3f graspingnormal_eigen{graspingnormal.x(), graspingnormal.y(), graspingnormal.z()};
        Eigen::Vector3f graspingnormal_rotated_eigen = T.linear() * graspingnormal_eigen;
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
                    octomap::point3d_collection points;// = GraspPlanningUtils::get_filtered_surface_normals(target_tree_, hit_left);
                    target_tree_->getNormal(hit_left, points); // ! Toggle to use getNormal()
                    if (!points.empty())
                    {
                        int best_option_angle_deg{91};
                        for (octomap::point3d_collection::iterator it_3d = points.begin(); it_3d != points.end(); ++it_3d)
                        {
                            float rot_angle_rad{GraspPlanningUtils::safe_angleTo(direction,*it_3d)};
                            int angle_deg = (int)(abs(cos((rot_angle_rad))*90)); // angle always between (0-90)
                            if (angle_deg < 0 || angle_deg >90) std::cerr << "OUTOFBOUNDS left ANGLEDEG=" << angle_deg << std::endl;
                            if (angle_deg < best_option_angle_deg) best_option_angle_deg = angle_deg;
                        }
                        ++histogram_angles_left[best_option_angle_deg];
                    }
                }
            }
            if (target_tree_->castRay(world3d_right, -direction, hit_right, true, distance)) // if occupied node was hit
            {
                octomap::OcTreeGraspQualityNode* n = target_tree_->search(hit_right);
                if (n && target_tree_->isNodeOccupied(n)) // if occupied
                {
                    octomap::point3d_collection points;// = GraspPlanningUtils::get_filtered_surface_normals(target_tree_, hit_right);
                    target_tree_->getNormal(hit_right, points); // ! Toggle to use getNormal()
                    if (!points.empty())
                    {
                        int best_option_angle_deg{91};
                        for (octomap::point3d_collection::iterator it_3d = points.begin(); it_3d != points.end(); it_3d++)
                        {
                            float rot_angle_rad{GraspPlanningUtils::safe_angleTo(direction,*it_3d)};
                            int angle_deg = (int)(abs(cos((rot_angle_rad))*90)); // angle always between (0-90)
                            if (angle_deg < 0 || angle_deg >90) std::cerr << "OUTOFBOUNDS right ANGLEDEG=" << angle_deg << std::endl;
                            if (angle_deg < best_option_angle_deg) best_option_angle_deg = angle_deg;
                        }
                        ++histogram_angles_right[best_option_angle_deg];
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
        unsigned int samples{(unsigned int)histogram_combined.sum()};
        if (samples == 0U) return 0.0F; // if there were no hits the score is zero, no need for further processing

        // calculate mean and std dev
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

        // calculate number of negative voxel interactions target-gripper to substract from score
        const int collisions = GraspPlanningUtils::negative_collisions(gripper_tree_, target_tree_, T);

        // assign score based on heuristic linear regression slopes
        float score = (90-(float)mean)/90.0F * weight_mean + std_saturation/std::max(std_dev,std_saturation) * (1.0F-weight_mean);
        // ? Maybe use median and mean, instead of std dev?
        float fraction_samples = ((float)(std::max(((int)samples)-collisions,0)))/((float)grasping_pairs.size()*2.0F);

        if (score*fraction_samples > 1.0F)
        {
            std::cerr << "SCORE OVER 1 (" << score*fraction_samples << ")" << std::endl;
            std::cout << "Mean score = " << (90-(float)mean)/90.0F << ", stddev score = " << std_saturation/std::max(std_dev,std_saturation) << ", samples = " << samples << ", grasping_pairs.size() = " << grasping_pairs.size() << ", fraction_samples = " << fraction_samples << ", score = " << score << ", normalised score = " << score*fraction_samples << std::endl;
        }
        return score*fraction_samples; // normalise score accounting for % of nodes hit from total
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
        // ! Weights
        const float voxel_normal_ratio{0.5};
        
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
        // ! Weights
        const int zero_crossing{45};
        float score{0};

        if (grasping_pairs.empty()) grasping_pairs = GraspPlanningUtils::graspingPairs("yz", gripper_tree_); // only initialise once, as this fcn call is slow

        // rotate grasping normal gripper
        octomap::point3d graspingnormal = gripper_tree_->getGraspingNormal();
        Eigen::Vector3f graspingnormal_eigen{graspingnormal.x(), graspingnormal.y(), graspingnormal.z()};
        Eigen::Vector3f graspingnormal_rotated_eigen = T.rotation() * graspingnormal_eigen;
        octomap::point3d grasping_normal_rotated{graspingnormal_rotated_eigen.x(), graspingnormal_rotated_eigen.y(), graspingnormal_rotated_eigen.z()};
        grasping_normal_rotated.normalize();

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
                octomap::point3d_collection points;// = GraspPlanningUtils::get_filtered_surface_normals(target_tree_, hit_left);
                if (n && target_tree_->isNodeOccupied(n) && target_tree_->getNormal(hit_left, points)) // if occupied and has normal (is in surface) // ! Toggle to use getNormal()
                {
                    int best_normal_option_angle = 91;
                    for (octomap::point3d_collection::iterator it_3d = points.begin(); it_3d != points.end(); ++it_3d)
                    {
                        float rot_angle_rad{GraspPlanningUtils::safe_angleTo(direction,*it_3d)};
                        int angle_deg = (int)(abs(cos((rot_angle_rad))*90)); // angle always between (0-90)
                        if (angle_deg < 0 || angle_deg >90) std::cerr << "OUTOFBOUNDS left ANGLEDEG=" << angle_deg << std::endl;
                        if (angle_deg < best_normal_option_angle) best_normal_option_angle = angle_deg;
                    }
                    score += std::max(((float)(zero_crossing-best_normal_option_angle))/((float)zero_crossing),-1.0F); // an angle equal to zero_crossing would give 0 reward/penalty, a 90deg would give (1) reward, and a decreasing angle linearly increases penalty up until a -1.0F penalty.
                }
            }
            if (target_tree_->castRay(world3d_right, -direction, hit_right, true, distance)) // if occupied node was hit
            {
                octomap::OcTreeGraspQualityNode* n = target_tree_->search(hit_right);
                octomap::point3d_collection points;// = GraspPlanningUtils::get_filtered_surface_normals(target_tree_, hit_right);
                if (n && target_tree_->isNodeOccupied(n) && target_tree_->getNormal(hit_right, points)) // if occupied and has normal (is in surface) // ! Toggle to use getNormal()
                {
                    int best_normal_option_angle = 91;
                    for (octomap::point3d_collection::iterator it_3d = points.begin(); it_3d != points.end(); ++it_3d)
                    {
                        float rot_angle_rad{GraspPlanningUtils::safe_angleTo(direction,*it_3d)};
                        int angle_deg = (int)(abs(cos((rot_angle_rad))*90)); // angle always between (0-90)
                        if (angle_deg < 0 || angle_deg >90) std::cerr << "OUTOFBOUNDS right ANGLEDEG=" << angle_deg << std::endl;
                        if (angle_deg < best_normal_option_angle) best_normal_option_angle = angle_deg;
                    }
                    score += std::max(((float)(zero_crossing-best_normal_option_angle))/((float)zero_crossing),-1.0F); // an angle equal to zero_crossing would give 0 reward/penalty, a 90deg would give (1) reward, and a decreasing angle linearly increases penalty up until a -1.0F penalty.
                }
            }
        }

        // calculate number of negative voxel interactions target-gripper to substract from score
        const float collisions = GraspPlanningUtils::negative_collisions(gripper_tree_, target_tree_, T);

        // normalise score
        score *= std::max(((float)grasping_pairs.size()*2.0F - collisions)/((float)grasping_pairs.size()*2.0F), 0.0F)/((float)grasping_pairs.size()*2.0F); // fraction out number of collision measurements from grasping pairs size and divide by sample size to normalise
        if (score>1.0F)
        {
            std::cerr << "[gq_raycasting] score is " << score << ", not properly normalised" << std::endl;
        }

        return score;
    }

    /**
     * Compute coplanarity of (separate) target surface contact points hit by rays cast from both antipodal gripper planes
     * @param T homogenous transformation from origin to gripper grasping pose
     * @param target_tree_ Target object octree
     * @param gripper_tree_ Gripper octree
     * @returns Normalised grasp quality score between [0,1]
     */
    inline float pairs_coplanarity(const Eigen::Affine3f& T, const octomap::OcTreeGraspQuality* target_tree_, const octomap::OcTreeGripper* gripper_tree_)
    {
        // ! Weights
        const float weight_mean{0.5F}; // % (1-based) of total score comes from the abs(mean-median), other fraction from std dev
        const float std_saturation{5.0F}; // in number of bins
        const unsigned int bins_steps{100}; // number of discretisations between grasping plates (resolution)


        if (grasping_pairs.empty()) grasping_pairs = GraspPlanningUtils::graspingPairs("yz", gripper_tree_); // only initialise once, as this fcn call is slow

        // rotate grasping normal gripper
        octomap::point3d graspingnormal = gripper_tree_->getGraspingNormal();
        Eigen::Vector3f graspingnormal_eigen{graspingnormal.x(), graspingnormal.y(), graspingnormal.z()};
        Eigen::Vector3f graspingnormal_rotated_eigen = T.linear() * graspingnormal_eigen;
        octomap::point3d graspingnormal_rotated{graspingnormal_rotated_eigen.x(), graspingnormal_rotated_eigen.y(), graspingnormal_rotated_eigen.z()};
        graspingnormal_rotated.normalize();


        std::vector<int> histogram_graspdepth_left (bins_steps,0); // create 101 bins (0,100) fields populated with 0s. the 100 bins span the entire distance between antipodal grasping plates
        std::vector<int> histogram_graspdepth_right (bins_steps,0); // create 101 bins (0,100) fields populated with 0s. the 100 bins span the entire distance between antipodal grasping plates

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
                if (n && target_tree_->isNodeOccupied(n)) // if occupied and not the first node
                {
                    ++histogram_graspdepth_left[((unsigned int)abs(world3d_left.distance(hit_left)/distance))*bins_steps];
                }
            }
            if (target_tree_->castRay(world3d_right, -direction, hit_right, true, distance)) // if occupied node was hit
            {
                octomap::OcTreeGraspQualityNode* n = target_tree_->search(hit_right);
                if (n && target_tree_->isNodeOccupied(n)) // if occupied
                {
                    ++histogram_graspdepth_right[((unsigned int)abs(world3d_right.distance(hit_right)/distance))*bins_steps];
                }
            }
        }

        // merge histograms into eigen array
        Eigen::Array<int, bins_steps, 1> histogram_combined;
        std::transform(histogram_graspdepth_left.begin(), histogram_graspdepth_left.end(), histogram_graspdepth_right.begin(), histogram_combined.data(), std::plus<int>());
        unsigned int samples{(unsigned int)histogram_combined.sum()};
        if (samples == 0U) return 0.0F; // if there were no hits the score is zero, no need for further processing

        // calculate mean and std dev
        const int mean = (histogram_combined * Eigen::Array<int, bins_steps, 1>::LinSpaced(0,bins_steps)).sum()/samples;
        const float std_dev = sqrt((histogram_combined * Eigen::Array<int, bins_steps, 1>::LinSpaced(0-mean,bins_steps-mean)).square().sum() / samples);
        
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

        // calculate number of negative voxel interactions target-gripper to substract from score
        const int collisions = GraspPlanningUtils::negative_collisions(gripper_tree_, target_tree_, T);

        // assign score
        float score = ((float)(bins_steps-abs(mean-median))/bins_steps) * weight_mean + std_saturation/std::max(std_dev,std_saturation) * (1.0F-weight_mean);

        float fraction_samples = ((float)(std::max(((int)samples)-collisions,0)))/((float)grasping_pairs.size()*2.0F);
        if (score*fraction_samples > 1.0F)
        {
            std::cerr << "SCORE OVER 1 (" << score*fraction_samples << ")" << std::endl;
            std::cout << "mean score = " << mean << ", median score = " << median << ", mean-median score = " << (bins_steps-abs(mean-median))/bins_steps << ", stddev score = " << std_saturation/std::max(std_dev,std_saturation) << ", samples = " << samples << ", grasping_pairs.size() = " << grasping_pairs.size() << ", collisions = " << collisions << ", fraction_samples = " << fraction_samples << ", score = " << score << ", normalised score = " << score*fraction_samples << std::endl;
        }
        return score*fraction_samples; // normalise score accounting for % of nodes hit from total
    }

    /**
     * Check for voxel superimposition (method 1) score and ray casting (method 4)
     * @param T homogenous transformation from origin to gripper grasping pose
     * @param target_tree_ Target object octree
     * @param gripper_tree_ Gripper octree
     * @param voxel_normal_ratio Percentage (in 1 scale) weight given to voxel superimposition method against surface normal method
     * TODO find a way to pass weight ratio
     */
    inline float gq_voxelsuperimposition_raycasting(const Eigen::Affine3f& T, const octomap::OcTreeGraspQuality* target_tree_, const octomap::OcTreeGripper* gripper_tree_)//, float voxel_normal_ratio = 0.5)
    {
        // ! Weights
        const float voxel_normal_ratio{0.5};
        
        float score{gq_voxelsuperimposition(T, target_tree_, gripper_tree_) * voxel_normal_ratio + gq_raycasting(T, target_tree_, gripper_tree_) * (1-voxel_normal_ratio)};
        return score;
    }

    /**
     * Check for voxel superimposition (method 1) score and coplanarity (method 5)
     * @param T homogenous transformation from origin to gripper grasping pose
     * @param target_tree_ Target object octree
     * @param gripper_tree_ Gripper octree
     * @param voxel_normal_ratio Percentage (in 1 scale) weight given to voxel superimposition method against surface normal method
     * TODO find a way to pass weight ratio
     */
    inline float gq_voxelsuperimposition_coplanarity(const Eigen::Affine3f& T, const octomap::OcTreeGraspQuality* target_tree_, const octomap::OcTreeGripper* gripper_tree_)//, float voxel_normal_ratio = 0.5)
    {
        // ! Weights
        const float voxel_normal_ratio{0.5};
        
        float score{gq_voxelsuperimposition(T, target_tree_, gripper_tree_) * voxel_normal_ratio + pairs_coplanarity(T, target_tree_, gripper_tree_) * (1-voxel_normal_ratio)};
        return score;
    }
}

