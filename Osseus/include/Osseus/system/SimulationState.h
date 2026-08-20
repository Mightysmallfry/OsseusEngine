#ifndef OSSEUSENGINE_SIMULATIONSTATE_H
#define OSSEUSENGINE_SIMULATIONSTATE_H

#include <cstddef>
#include <vector>

#include "Osseus/system/ForceManager.h"
#include "Osseus/system/BodyManager.h"
#include "Osseus/math/Vector3.h"
#include "Osseus/system/Handle.h"
#include "Osseus/physics/Octree.h"

namespace osseus {

    class BodyManager;

    class SimulationState
    {
    public:
        SimulationState() = default;
        
        explicit SimulationState(const BodyManager& bodyManager, 
            const ForceManager& forceManager);
            
        SimulationState(const SimulationState& other);

        void CopyFrom(const BodyManager& bodyManager, const ForceManager& forceManager);

        void RebuildOctree();

        void Resize(std::size_t size);
    
        std::vector<BodyData>& GetBodies() { return bodyData; } 
        std::vector<Handle>& GetHandles() { return handles; } 
        std::vector<Vector3>& GetNetForces() { return netForces; } 

        Octree& GetOctree();
        const Octree& GetOctree() const;


    private:
        std::vector<BodyData> bodyData;
        std::vector<Handle> handles;
        std::vector<Vector3> netForces;
        std::vector<UniversalForceEvaluator*> universalForces_;

        Octree octree_;
    };

} // namespace osseus

#endif // OSSEUSENGINE_SIMULATIONSTATE_H