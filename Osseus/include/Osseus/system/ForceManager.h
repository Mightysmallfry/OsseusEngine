#ifndef OSSEUSENGINE_FORCEMANAGER_H
#define OSSEUSENGINE_FORCEMANAGER_H

#include <vector>

#include "Osseus/math/Vector3.h"
#include "Osseus/system/Handle.h"
#include "Osseus/physics/UniversalForce.h"

namespace osseus {

    class ForceManager
    {
    public:
        void Resize(std::size_t count);

        void Clear();
        void Add(Handle handle, const Vector3& force);
        void ClearForceOf(Handle handle);

        void ClearUniversals();
        int AddUniversal(UniversalForceEvaluator* universalForce);

        [[nodiscard]]
        const Vector3& Get(Handle handle) const;

        [[nodiscard]]
        const std::vector<UniversalForceEvaluator*>&
        GetUniversals() const;

    private:
        std::vector<Vector3> netForces_;
        std::vector<UniversalForceEvaluator*> universalForces_;
    };

}

#endif