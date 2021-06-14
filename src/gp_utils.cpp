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
    inline octomap::point3d transform_point3d(const Eigen::Affine3f& T, const octomap::point3d& point3d)
    {
        Eigen::Vector3f coord_0{point3d.x(), point3d.y(), point3d.z()};
        Eigen::Vector3f coord_1{T * coord_0};
        octomap::point3d coord_1_3d{coord_1.x(), coord_1.y(), coord_1.z()};
        return coord_1_3d;
    }

    /**
     * Calculate number of negative interactions between gripper and target octrees
     * @param source source tree, will be transformed to target reference frame
     * @param target target tree, anchored in its reference frame
     * @param T transformation to transform source to target reference frames
     */
    template<typename NODE_A, typename NODE_B>
    inline int negative_collisions(const octomap::OccupancyOcTreeBase<NODE_A>* source, const octomap::OccupancyOcTreeBase<NODE_B>* target, const Eigen::Affine3f& T)
    {
        int collisions{0};
        for (octomap::OcTreeGripper::leaf_iterator it = source->begin_leafs(), end=source->end_leafs(); it!= end; ++it)
        {
            octomap::point3d tgt3d{transform_point3d(T, it.getCoordinate())};
            octomap::OcTreeGraspQualityNode* n = target->search(tgt3d);
            if (n && target->isNodeOccupied(n)) // if target node is occupied
            {
                if(!it->isGraspingSurface()) ++collisions;
            }
        }
        return collisions;
    }

    /**
     * Calculate the angle between two vectors safetly, handling floating-point imprecision safely
     * @param lhs Vector 1
     * @param rhs Vector 2
     * @returns angle between vector in radians
     */
    inline float safe_angleTo(const octomap::point3d& lhs, const octomap::point3d& rhs)
    {
        double dotProduct = lhs.dot(rhs);
        double len1 = lhs.norm();
        double len2 = rhs.norm();
        float op = std::max<float>(std::min<float>((float)(dotProduct / (len1*len2)),1.0F),-1.0F); // clamp between (-1.0,1.0)
        return acos(op);
    }

    /**
     * Simple std::out formatter for node occupancy queries
     * @param query 3D point being queried
     * @param node corresponding octree node
     */
    inline void print_query_info(const octomap::point3d& query, const octomap::OcTreeNode* node)
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
    inline octomap::point3d max_composite_vector(const octomap::point3d& __vector1, const octomap::point3d& __vector2)
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
    inline octomap::point3d min_composite_vector(const octomap::point3d& __vector1, const octomap::point3d& __vector2)
    {
        octomap::point3d output;
        for (unsigned int i=0; i<3; ++i)
        {
            output(i) = std::min(__vector1(i),__vector2(i));
        }
        return output;
    }

    /**
     * Compute the surface normals of the octree at the target point. Uses the original Marching Cubes octomap implementation of getNormals()
     * @param tree Input OcTree
     * @param point3d Point at which to compute the surface normals
     * @returns Collection of surface normals normalised vectors
     */
    template<typename NODE>
    inline octomap::point3d_collection get_filtered_surface_normals(const octomap::OccupancyOcTreeBase<NODE>* tree, const octomap::point3d& point3d)
    {
        const double angle_threshold_same_vector = 0.01; // rad (0.01rad = 0.573deg) // TODO set to a meaningful value
        octomap::point3d_collection normals;
        octomap::point3d_collection filtered_normals;
        if (!tree->getNormals(point3d, normals, false)) // run octomap surface reconstruction function considering unknown measurements as FREE
        {
            std::cerr << "[GraspPlanningUtils::get_filtered_surface_normals()] call to getNormals of tree failed" << std::endl;
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
                if (safe_angleTo(current_vector,filtered_normals[j]) < angle_threshold_same_vector) already_exists = true;
            }
            // if vector normal doesnt already exist in filtered collection, push it there
            if (!already_exists) filtered_normals.push_back(current_vector);
            already_exists = false; // reset flag
        }
        return filtered_normals;
    }

    /**
     * Utility that searches 3d coordinates within tree to provide its corresponding iterator ptr, similar to the member fcn search() that returns the node ptr. Time complexity of O(n), do not overuse.
     * @param coord 3D coordinates to search
     * @param tree Octree ptr
     * @returns Octree iterator pointing to the node populating the provided coordinates. Returns NULL if no node is in the coordinates
     */
    template<typename NODE>
    inline typename octomap::OccupancyOcTreeBase<NODE>::iterator searchIterator(const octomap::point3d &coord, const octomap::OccupancyOcTreeBase<NODE>* tree)
    {
        NODE* node = tree->search(coord);
        if (!node) return NULL; // return null if node is unknown
        for (typename octomap::OccupancyOcTreeBase<NODE>::leaf_iterator it = tree->begin_leafs(), end = tree->end_leafs(); it != end; ++it)
        {
            if (&(*it) == node) // if ptrs point to the same memory address, they are the same node
            {
                return it;
            }
        }
        return NULL;
    }

    /**
     * Utility to retrieve the grasping pairs on each edge of the grasping region of the gripper.
     * @param axes Axes plane in the gripper reference frame of the grasping plates (i.e. "xz")
     * @param tree Gripper octree ptr
     * @returns Vector of node iterator pairs
     */
    inline std::vector<std::pair<octomap::OcTreeGripper::iterator,octomap::OcTreeGripper::iterator>> graspingPairs(const std::string axes, const octomap::OcTreeGripper* tree)
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

    /**
     * Generate a copy of the tree nodes with the provided node resolution
     * @param tree Input tree to be rescaled
     * @param resolution New tree resolution
     * @returns Copy of the tree nodes with the new resolution
     * ! Not working, see ! comment below
     */
    template<typename TREE>
    inline TREE* rescaleTree(const TREE* tree, const double &resolution)
    {
        std::cout << "[rescaleTree] started..." << std::endl;
        TREE* out_tree = new TREE(*tree); // copying tree to copy any class attributes (metadata). Not an efficient call, but the only one currently supported for all octree types
        out_tree->clear(); // delete tree structure // ! This tries deleting the imported tree structure, need to copy contents in constructor, instead of the entire tree like that.
        out_tree->setResolution(resolution);
        octomap::OcTreeKey minKey, maxKey, k;
        double xmin, ymin, zmin, xmax, ymax, zmax;
        tree->getMetricMin(xmin,ymin,zmin);
        tree->getMetricMax(xmax,ymax,zmax);
        out_tree->coordToKeyChecked(xmin, ymin, zmin, minKey);
        out_tree->coordToKeyChecked(xmax, ymax, zmax, maxKey);

        for (k[0] = minKey[0]; k[0] <= maxKey[0]; ++k[0]){
            for (k[1] = minKey[1]; k[1] <= maxKey[1]; ++k[1]){
                for (k[2] = minKey[2]; k[2] <= maxKey[2]; ++k[2]){
                    auto* node_ptr = tree->search(out_tree->keyToCoord(k)); // TODO substitute auto for an actual type
                    if(node_ptr) // If node exists
                    {
                        out_tree->updateNode(k, node_ptr->getLogOdds(), true)->copyData(*node_ptr);
                    }
                }
            }
        }
        out_tree->updateInnerOccupancy();
        out_tree->expand();
        return out_tree;
    }

    /**
     * Generate a copy of the tree nodes within the defined BBX with the provided node resolution
     * @param tree Input tree to be rescaled
     * @param resolution New tree resolution
     * @param min Minimum corner coordinate of bounding box in meters
     * @param max Maximum corner coordinate of bounding box in meters
     * @returns Copy of the tree nodes within the BBX with the new resolution
     * ! Not working yet, due to problem with BBX-less impl of this fcn
     */
    template<typename TREE>
    inline TREE* rescaleTree(const TREE* tree, const double &resolution, const octomap::point3d& min, const octomap::point3d& max)
    {
        // * Copy subset of the tree structure within (min, max)
        TREE* tree_bbx = new TREE(*tree); // copying tree to copy any class attributes (metadata). Not an efficient call, but the only one currently supported for all octree types
        tree_bbx->clear(); // delete tree structure

        for(typename TREE::leaf_bbx_iterator it = tree->begin_leafs_bbx(min, max), end = tree->end_leafs_bbx(); it != end; ++it)
        {
            octomap::OcTreeKey key;
            if (tree_bbx->coordToKeyChecked(it.getCoordinate(), key))
            {
                tree_bbx->updateNode(key, it->getLogOdds(), true)->copyData(*it);
            }
            else std::cerr << "[rescaleTree] Attempted to copy invalid node" << std::endl;
            if (!(tree_bbx->search(key) == *it)) std::cerr << "NODES NOT IDENTICAL, FAIL" << std::endl; // ! remove DEBUG
        }
        tree_bbx->updateInnerOccupancy();
        return rescaleTree(tree_bbx, resolution); // feed subset of tree to generic overload for actual rescaling
    }

    /**
     * Generate a fast copy of the tree structure with the provided node resolution. This fast method will not preserve the tree class attributes (metadata), only the tree structure!
     * @param tree Input tree to be rescaled
     * @param resolution New tree resolution
     * @returns Copy of the tree nodes with the new resolution
     */
    template<typename TREE>
    inline TREE* rescaleTreeStructure(const TREE* tree, const double &resolution)
    {
        TREE* out_tree = new TREE(resolution); // copying tree to copy any class attributes (metadata). Not an efficient call, but the only one currently supported for all octree types

        octomap::OcTreeKey minKey, maxKey, k;
        double xmin, ymin, zmin, xmax, ymax, zmax;
        tree->getMetricMin(xmin,ymin,zmin);
        tree->getMetricMax(xmax,ymax,zmax);
        if (out_tree->coordToKeyChecked(xmin, ymin, zmin, minKey) && out_tree->coordToKeyChecked(xmax, ymax, zmax, maxKey))
        {
            for (k[0] = minKey[0]; k[0] <= maxKey[0]; ++k[0]){
                for (k[1] = minKey[1]; k[1] <= maxKey[1]; ++k[1]){
                    for (k[2] = minKey[2]; k[2] <= maxKey[2]; ++k[2]){
                        auto* node_ptr = tree->search(out_tree->keyToCoord(k)); // TODO substitute auto for an actual type
                        if(node_ptr) // If node exists
                        {
                            out_tree->updateNode(k, node_ptr->getLogOdds(), true)->copyData(*node_ptr);
                        }
                    }
                }
            }
            if (out_tree->getRoot()) // NULL if tree is empty
            {
                out_tree->updateInnerOccupancy();
                out_tree->expand();
            }
            else 
            {
                std::cerr << "[rescaleTreeStructure] empty outgoing tree" << std::endl;
            }
        }
        else
        {
            std::cerr << "[rescaleTreeStructure] incoming tree has invalid dimensions" << std::endl;
        }
        return out_tree;
    }

    /**
     * Generate a fast copy of the tree structure within the defined BBX with the provided node resolution. This fast method will not preserve the tree class attributes (metadata), only the tree structure!
     * @param tree Input tree to be rescaled
     * @param resolution New tree resolution
     * @param min Minimum corner coordinate of bounding box in meters
     * @param max Maximum corner coordinate of bounding box in meters
     * @returns Copy of the tree nodes within the BBX with the new resolution
     */
    template<typename TREE>
    inline TREE* rescaleTreeStructure(const TREE* tree, const double &resolution, const octomap::point3d& min, const octomap::point3d& max)
    {
        // * Copy subset of the tree structure within (min, max)
        TREE* tree_bbx = new TREE(tree->getResolution());

        for(typename TREE::leaf_bbx_iterator it = tree->begin_leafs_bbx(min, max), end = tree->end_leafs_bbx(); it != end; ++it)
        {
            octomap::OcTreeKey key;
            if (tree_bbx->coordToKeyChecked(it.getCoordinate(), key))
            {
                tree_bbx->updateNode(key, it->getLogOdds(), true)->copyData(*it);
            }
            else std::cerr << "[rescaleTreeStructure] Attempted to copy invalid node" << std::endl;
        }
        if (tree_bbx->getRoot()) // NULL if tree is empty
        {
            tree_bbx->updateInnerOccupancy();
            tree_bbx->expand();
        }
        else 
        {
            std::cerr << "[rescaleTreeStructure] empty outgoing bbx tree. BBX too narrow?" << std::endl;
            std::cerr << "Min BBX: \n" << min << std::endl;
            std::cerr << "Max BBX: \n" << max << std::endl;
            return tree_bbx;
        }
        return rescaleTreeStructure(tree_bbx, resolution); // feed subset of tree to generic overload for actual rescaling
    }
}