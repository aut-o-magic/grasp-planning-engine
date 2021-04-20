#include <octomap/octomap.h>

namespace GraspPlanningUtils
{
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