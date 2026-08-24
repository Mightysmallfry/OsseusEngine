#ifndef OSSEUSENGINE_BARNESHUT_H
#define OSSEUSENGINE_BARNESHUT_H

#include "Osseus/physics/Octree.h"
#include "Osseus/system/BodyManager.h"
#include "Osseus/system/ForceManager.h"

namespace osseus {

    // Approximates n-body gravitational forces by walking an Octree
    // once per body. A node is treated as a single point mass at its
    // center of mass when it's small/far enough (width / distance 
    // theta); otherwise the walk recurses into its children.
    class BarnesHut {
    public:
        void Evaluate(const Octree& tree, const std::vector<Handle>& handles,
             const std::vector<BodyData>& bodies, ForceManager& forceManager);

    };

} // osseus

#endif //OSSEUSENGINE_BARNESHUT_H