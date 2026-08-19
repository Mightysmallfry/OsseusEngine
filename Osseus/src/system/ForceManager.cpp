#ifndef OSSEUSENGINE_FORCEMANAGER_CPP
#define OSSEUSENGINE_FORCEMANAGER_CPP

#include <algorithm>
#include <typeinfo>

#include "Osseus/system/ForceManager.h"

namespace osseus {

    const Vector3& ForceManager::Get(Handle handle) const
    {
        return netForces_[handle.index];
    }

    void ForceManager::Resize(std::size_t count)
    {
        netForces_.resize(count);
    }

    void ForceManager::Clear()
    {
        std::fill(
            netForces_.begin(),
            netForces_.end(),
            Vector3::Zero()
        );
    }

    void ForceManager::Add(Handle handle, const Vector3& force)
    {
        netForces_[handle.index] += force;
    }

    void ForceManager::ClearForceOf(Handle handle)
    {
        netForces_[handle.index] = Vector3::Zero();
    }

    void ForceManager::ClearUniversals()
    {
        universalForces_.clear();
    }

    int ForceManager::AddUniversal(
        UniversalForceEvaluator* universalForce)
    {
        for (const auto& existing : universalForces_)
        {
            if (typeid(*existing) == typeid(*universalForce))
            {
                return -1;
            }
        }

        universalForces_.push_back(universalForce);
        return 0;
    }

    const std::vector<UniversalForceEvaluator*>&
    ForceManager::GetUniversals() const
    {
        return universalForces_;
    }

}

#endif