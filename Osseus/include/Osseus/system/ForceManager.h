#ifndef OSSEUSENGINE_FORCEMANAGER_H
#define OSSEUSENGINE_FORCEMANAGER_H

#include <vector>

#include "Osseus/math/Vector3.h"
#include "Osseus/physics/UniversalForce.h"
#include "Osseus/system/BodyManager.h"
#include "Osseus/system/Handle.h"

namespace osseus {

    class ForceManager {
        public:
        ForceManager() = default;
        explicit ForceManager(const BodyManager& bodyManager);

        void Register(Handle handle);

        void Resize(std::size_t count);

        void Clear();
        void Add(Handle handle, const Vector3& force);
        void ClearForceOf(Handle handle);

        void ClearUniversals();
        int AddUniversal(UniversalForceEvaluator* universalForce);
        bool HasUniversals();

        [[nodiscard]]
        const Vector3& Get(Handle handle) const;

        std::vector<UniversalForceEvaluator*>& GetUniversals() {
            return universalForces_;
        }
        const std::vector<UniversalForceEvaluator*>& GetUniversals() const {
            return universalForces_;
        }

        std::vector<Vector3>& NetForces() {
            return netForces_;
        }
        const std::vector<Vector3>& NetForces() const {
            return netForces_;
        }

        private:
        std::vector<Vector3> netForces_;
        std::vector<UniversalForceEvaluator*> universalForces_;
    };

} // namespace osseus

#endif