#include "Osseus/physics/BarnesHut.h"

#include <algorithm>
#include <cmath>

namespace osseus {

    void BarnesHut::Evaluate(const Octree& tree, const std::vector<Handle>& handles,
                             const std::vector<BodyData>& bodies, ForceManager& forceManager) {

        if (!forceManager.HasUniversals()) {
            return;
        }
        const OctNode& root = tree.GetRoot();

        const std::size_t count = std::min(handles.size(), bodies.size());
        for (std::size_t i = 0; i < count; ++i) {
            const Handle handle = handles[i];
            const BodyData& body = bodies[i];
            if (body.invMass == 0.0) {
                continue;
            } // static bodies don't move; no force needed

            // Spin through accumulating all forces from the forceManager
            Vector3 force = Vector3::Zero();
            for (UniversalForceEvaluator* UForce : forceManager.GetUniversals()) {
                force += UForce->CalculateForce(root, handle, body);
            }
            forceManager.NetForces()[handle.index] += force;
        }
    }

} // namespace osseus