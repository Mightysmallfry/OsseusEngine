#ifndef OSSEUSENGINE_SIMULATIONSTATE_H
#define OSSEUSENGINE_SIMULATIONSTATE_H

#include <cstddef>
#include <vector>

#include "Osseus/math/Vector3.h"
#include "Osseus/system/Handle.h"
#include "Osseus/physics/Octree.h"

namespace osseus {

    class BodyManager;

    class SimulationState
    {
    public:
        SimulationState() = default;

        explicit SimulationState(const BodyManager& bodyManager);

        void CopyFrom(const BodyManager& bodyManager);
        void CopyState(const SimulationState& simState);

        void RebuildOctree(const BodyManager& bodyManager);

        void Resize(std::size_t size);

        Vector3& GetPosition(Handle handle);
        const Vector3& GetPosition(Handle handle) const;

        Vector3& GetVelocity(Handle handle);
        const Vector3& GetVelocity(Handle handle) const;

        Octree& GetOctree();
        const Octree& GetOctree() const;

        std::size_t Size() const;

    private:
        std::vector<Vector3> positions_;
        std::vector<Vector3> velocities_;

        Octree octree_;
    };

} // namespace osseus

#endif // OSSEUSENGINE_SIMULATIONSTATE_H