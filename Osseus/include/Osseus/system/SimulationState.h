#ifndef OSSEUSENGINE_SIMULATIONSTATE_H
#define OSSEUSENGINE_SIMULATIONSTATE_H

#include <cstddef>
#include <vector>

#include "Osseus/math/Vector3.h"
#include "Osseus/physics/Octree.h"
#include "Osseus/system/BodyManager.h"
#include "Osseus/system/ForceManager.h"
#include "Osseus/system/Handle.h"

namespace osseus {

    class BodyManager;

    class SimulationState {
        public:
        SimulationState() = default;

        explicit SimulationState(const BodyManager& bodyManager, const ForceManager& forceManager);

        SimulationState(const SimulationState& other);

        void CopyFrom(const BodyManager& bodyManager, const ForceManager& forceManager);

        void RebuildOctree();

        void Resize(std::size_t size);

        std::vector<BodyData>& GetBodies() {
            return bodyData_;
        }
        std::vector<Handle>& GetHandles() {
            return handles_;
        }
        std::vector<Vector3>& GetNetForces() {
            return netForces_;
        }

        Octree& GetOctree();
        const Octree& GetOctree() const;

        private:
        std::vector<BodyData> bodyData_;
        std::vector<Handle> handles_;
        std::vector<Vector3> netForces_;
        std::vector<UniversalForceEvaluator*> universalForces_;

        Octree octree_;
    };

} // namespace osseus

#endif // OSSEUSENGINE_SIMULATIONSTATE_H