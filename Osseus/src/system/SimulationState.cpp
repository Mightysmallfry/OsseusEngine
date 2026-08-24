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
    : bodyData_(other.bodyData_), handles_(other.handles_), netForces_(other.netForces_) {}

    void SimulationState::CopyFrom(const BodyManager& bodyManager, const ForceManager& forceManager)
    {
        bodyData_ = bodyManager.Data();
        handles_ = bodyManager.Handles();
        netForces_ = forceManager.NetForces();
        universalForces_ = forceManager.GetUniversals();

        RebuildOctree();
    }


    void SimulationState::RebuildOctree()
    {
        octree_.Clear();

        for (Handle handle : handles_)
        {
            const BodyData& body = bodyData_[handle.index];

            octree_.Insert(
                handle,
                body.position,
                body.mass,
                body.charge
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