#pragma once

#include <octomap/octomap.h>
#include "octomap_grasping/OcTreeGripper.hpp"
#include <limits>

namespace GraspPlanningUtils
{
    /**
     * Apply a spatial transformation to a point3d data type
     * @param T Affine transformation
     * @param point3d Point3d to transform
     * @returns Transformed point3d
     */
    static octomap::point3d transform_point3d(const Eigen::Affine3f& T, const octomap::point3d& point3d)
    {
        Eigen::Vector3f coord_0{point3d.x(), point3d.y(), point3d.z()};
        Eigen::Vector3f coord_1{T * coord_0};
        octomap::point3d coord_1_3d{coord_1.x(), coord_1.y(), coord_1.z()};
        return coord_1_3d;
    }

    /**
     * Simple std::out formatter for node occupancy queries
     * @param query 3D point being queried
     * @param node corresponding octree node
     */
    static void print_query_info(const octomap::point3d& query, const octomap::OcTreeNode* node)
    {
        if (node) // if not NULL
        {
            std::cout << "occupancy probability at " << query << ":\t " << node->getOccupancy() << std::endl;
        }
        else 
            std::cout << "occupancy probability at " << query << ":\t is unknown" << std::endl;    
    }

    /**
     * Compute vector with max composite of each coordinate
     * @param __vector1 First vector
     * @param __vector2 Second vector
     * @returns Max composite vector
     */
    static octomap::point3d max_composite_vector(const octomap::point3d& __vector1, const octomap::point3d& __vector2)
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
    static octomap::point3d min_composite_vector(const octomap::point3d& __vector1, const octomap::point3d& __vector2)
    {
        octomap::point3d output;
        for (unsigned int i=0; i<3; ++i)
        {
            output(i) = std::min(__vector1(i),__vector2(i));
        }
        return output;
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
        const double angle_threshold_same_vector = 0.01; // rad (0.01rad = 0.573deg) // TODO set to a meaningful value
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
     * Utility that type casts an octree node to its corresponding iterator
     * @param node Octree node ptr
     * @param tree Octree ptr
     * @returns Octree iterator pointing to the provided node
     */
    template<typename NODE>
    static typename octomap::OccupancyOcTreeBase<NODE>::iterator nodeToIterator(const NODE* node, const octomap::OccupancyOcTreeBase<NODE>* tree)
    {
        for (typename octomap::OccupancyOcTreeBase<NODE>::leaf_iterator it = tree->begin_leafs(), end = tree->end_leafs(); it != end; ++it)
        {
            if(*it == *node) return it;
        }
        return NULL;
    }

    /**
     * Utility to retrieve the grasping pairs on each edge of the grasping region of the gripper.
     * @param axes Axes plane in the gripper reference frame of the grasping plates (i.e. "xz")
     * @param tree Gripper octree ptr
     * @returns Vector of node iterator pairs
     */
    static std::vector<std::pair<octomap::OcTreeGripper::iterator,octomap::OcTreeGripper::iterator>> graspingPairs(const std::string axes, const octomap::OcTreeGripper* tree)
    {
        std::vector<std::pair<octomap::OcTreeGripper::iterator, octomap::OcTreeGripper::iterator>> vector_pair{};
        // verify formatting correctness of input axes
        if (axes.length() != 2)
        {
            std::cerr << "[graspingPairs] ERROR: axes argument not 2 letters long" << std::endl;
            return vector_pair;
        }
        // verify input axes are non-repeating combinations of {x,y,z}
        if (((int)axes[0])-120 > 2 || ((int)axes[1])-120 > 2 || axes[0] == axes[1])
        {            
            std::cerr << "[graspingPairs] ERROR: invalid axes provided" << std::endl;
            return vector_pair;
        }

        // search axes parameter to find normal axis to plane
        std::string possible_axes{"xyz"};
        std::size_t normal_axis_loc = possible_axes.find_first_not_of(axes);
        const unsigned int normal_axis_idx{(unsigned int)(((int)possible_axes[normal_axis_loc])-120)};

        const float max_f{std::numeric_limits<float>::max()};
        const float min_f{-std::numeric_limits<float>::max()};
        octomap::point3d min{max_f, max_f, max_f};
        octomap::point3d max{min_f, min_f, min_f};

        // Iterate over tree and record max/min BBX coordinates
        for (octomap::OcTreeGripper::iterator it = tree->begin(), end = tree->end(); it != end; ++it)
        {
            if (it->isGraspingSurface())
            {
                octomap::point3d it_coord{it.getCoordinate()};
                min = min_composite_vector(min, it_coord);
                max = max_composite_vector(max, it_coord);
            }
        }

        // loop again storing matching node iterators into vector stacks for both boundary planes
        std::vector<octomap::OcTreeGripper::iterator> nodes_left;
        std::vector<octomap::OcTreeGripper::iterator> nodes_right;

        for (octomap::OcTreeGripper::iterator it = tree->begin(), end = tree->end(); it != end; ++it)
        {
            if (it->isGraspingSurface())
            {
                octomap::point3d it_coord{it.getCoordinate()};
                if (it_coord(normal_axis_idx) == min(normal_axis_idx)) nodes_left.push_back(it);
                else if (it_coord(normal_axis_idx) == max(normal_axis_idx)) nodes_right.push_back(it);
            }
        }

        // check stacks are of different size
        if (nodes_left.size() != nodes_right.size()) std::cerr << "[graspingPairs] ERROR: node stacks on each side of BBX have different lengths (" << nodes_left.size() << ", " << nodes_right.size() << ")" << std::endl;

        // match nodes in vector stacks into mirrored pairs
        for (std::vector<octomap::OcTreeGripper::iterator>::iterator it_left = nodes_left.begin(); it_left != nodes_left.end(); ++it_left)
        {
            octomap::point3d left_coord{it_left->getCoordinate()};
            for (std::vector<octomap::OcTreeGripper::iterator>::iterator it_right = nodes_right.begin(); it_right != nodes_right.end(); ++it_right)
            {
                octomap::point3d right_coord{it_right->getCoordinate()};
                bool match{false};
                for (unsigned int i=0; i<3; ++i)
                {
                    if (i == normal_axis_idx) continue; // skip the normal axis
                    if (left_coord(i) == right_coord(i))
                    {
                        if (match) // hence this is the second axis in the plane that has matching coordinates (i.e. nodes are mirrored pair)
                        {
                            std::pair<octomap::OcTreeGripper::iterator, octomap::OcTreeGripper::iterator> pair(*it_left, *it_right);
                            vector_pair.push_back(pair);
                        }
                        else // first axis that has matching coordinates
                        {
                            match = true;
                        }
                    }
                }
            }
        }
        // check resulting vector_pair has same length as vector stacks (i.e. no nodes were missed)
        if (nodes_left.size() != vector_pair.size()) std::cerr << "[graspingPairs] ERROR: resulting vector pair and node stacks have different lengths" << std::endl;
        return vector_pair;
    }
}