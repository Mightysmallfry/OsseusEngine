#ifndef OSSEUSENGINE_SIMULATIONSTATE_CPP
#define OSSEUSENGINE_SIMULATIONSTATE_CPP

#include "Osseus/system/SimulationState.h"

#include "Osseus/system/BodyManager.h"

namespace osseus {

    SimulationState::SimulationState(const BodyManager& bodyManager, 
        const ForceManager& forceManager)
    {
        CopyFrom(bodyManager, forceManager);
    }

    SimulationState::SimulationState(const SimulationState& other)
    : bodyData(other.bodyData), handles(other.handles), netForces(other.netForces) {}

    void SimulationState::CopyFrom(const BodyManager& bodyManager, const ForceManager& forceManager)
    {
        bodyData = bodyManager.Data();
        handles = bodyManager.Handles();
        netForces = forceManager.NetForces();
        universalForces_ = forceManager.GetUniversals();

        RebuildOctree();
    }


    void SimulationState::RebuildOctree()
    {
        octree_.Clear();

        for (Handle handle : handles)
        {
            const BodyData& body = bodyData[handle.index];

            octree_.Insert(
                handle,
                body.position,
                body.mass
            );
        }
    }


    Octree& SimulationState::GetOctree()
    {
        return octree_;
    }

    const Octree& SimulationState::GetOctree() const
    {
        return octree_;
    }


} // namespace osseus

#endif // OSSEUSENGINE_SIMULATIONSTATE_CPP