#ifndef OSSEUSENGINE_SIMULATIONSTATE_CPP
#define OSSEUSENGINE_SIMULATIONSTATE_CPP

#include "Osseus/system/SimulationState.h"

#include "Osseus/system/BodyManager.h"

namespace osseus {

    SimulationState::SimulationState(const BodyManager& bodyManager)
    {
        CopyFrom(bodyManager);
    }

    void SimulationState::CopyFrom(const BodyManager& bodyManager)
    {
        Resize(bodyManager.Data().size());

        for (Handle handle : bodyManager.Handles())
        {
            const BodyData* body = bodyManager.GetBody(handle);

            if (body == nullptr)
            {
                continue;
            }

            positions_[handle.index] = body->position;
            velocities_[handle.index] = body->velocity;
        }

        RebuildOctree(bodyManager);
    }

    void SimulationState::CopyState(const SimulationState& simState){
        positions_ = simState.positions_;
        velocities_= simState.velocities_;
    }

    void SimulationState::RebuildOctree(const BodyManager& bodyManager)
    {
        octree_.Clear();

        for (Handle handle : bodyManager.Handles())
        {
            const BodyData* body = bodyManager.GetBody(handle);

            if (body == nullptr)
            {
                continue;
            }

            octree_.Insert(
                handle,
                positions_[handle.index],
                body->mass
            );
        }
    }

    void SimulationState::Resize(std::size_t size)
    {
        positions_.resize(size);
        velocities_.resize(size);
    }

    Vector3& SimulationState::GetPosition(Handle handle)
    {
        return positions_[handle.index];
    }

    const Vector3& SimulationState::GetPosition(Handle handle) const
    {
        return positions_[handle.index];
    }

    Vector3& SimulationState::GetVelocity(Handle handle)
    {
        return velocities_[handle.index];
    }

    const Vector3& SimulationState::GetVelocity(Handle handle) const
    {
        return velocities_[handle.index];
    }

    Octree& SimulationState::GetOctree()
    {
        return octree_;
    }

    const Octree& SimulationState::GetOctree() const
    {
        return octree_;
    }

    std::size_t SimulationState::Size() const
    {
        return positions_.size();
    }

} // namespace osseus

#endif // OSSEUSENGINE_SIMULATIONSTATE_CPP