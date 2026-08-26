#ifndef OSSEUSENGINE_UNIVERSALEMFORCE_H
#define OSSEUSENGINE_UNIVERSALEMFORCE_H

#include "Osseus/physics/Constants.h"
#include "Osseus/physics/UniversalForce.h"

namespace osseus {

    class UniversalElectroMag : public UniversalForceEvaluator {
        public:
        Vector3 CalculateForce(const OctNode& sourceNode, Handle handle, const BodyData& body) override;

        private:
        Vector3 PointForce(const Vector3& sourcePosition, double sourceMass, const Vector3& queryPosition,
                           double queryMass);

        Vector3 DipoleForce(const Vector3& dipoleMoment, const Vector3& sourcePosition, const Vector3& queryPosition,
                            double queryMass);
    };

} // namespace osseus

#endif // OSSEUSENGINE_UNIVERSALEMFORCE_H