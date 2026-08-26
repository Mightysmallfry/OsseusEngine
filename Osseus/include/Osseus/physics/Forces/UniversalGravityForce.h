#ifndef OSSEUSENGINE_UNIVERSALGRAVITYFORCE_H
#define OSSEUSENGINE_UNIVERSALGRAVITYFORCE_H

#include "Osseus/physics/Constants.h"
#include "Osseus/physics/UniversalForce.h"

namespace osseus {

    class UniversalGravity : public UniversalForceEvaluator {
        public:
        Vector3 CalculateForce(const OctNode& sourceNode, Handle handle, const BodyData& body) override;

        private:
        Vector3 PointForce(const Vector3& sourcePosition, double sourceMass, const Vector3& queryPosition,
                           double queryMass);
    };

} // namespace osseus

#endif // OSSEUSENGINE_UNIVERSALGRAVITYFORCE_H