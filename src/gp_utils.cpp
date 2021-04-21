#include <octomap/octomap.h>

namespace GraspPlanningUtils
{
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
}