#ifndef OSSEUSENGINE_UNIVERSALFORCEEVALUATOR_H
#define OSSEUSENGINE_UNIVERSALFORCEEVALUATOR_H

#include "Osseus/interfaces/IForceEvaluator.h"
#include "Osseus/physics/Octree.h"
#include "Osseus/system/Handle.h"
#include "Osseus/system/ApproximationMode.h"

namespace osseus {

    class UniversalForceEvaluator : public IForceEvaluator {
        public:
        UniversalForceEvaluator();
        ~UniversalForceEvaluator() override = default;
        virtual Vector3 CalculateForce(const OctNode& sourceNode, Handle handle, const BodyData& body) = 0;

        void SetApproximationMode(ApproximationMode mode);
        ApproximationMode GetApproximationMode();

        void SetApproximationData(ApproximationData data);

        protected:
        void SetTheta(double newTheta);
        [[nodiscard]] double GetTheta() const;

        void SetSoftening(double newSoftening);
        [[nodiscard]] double GetSoftening() const;

        ApproximationMode approximationMode {ApproximationMode::NORMAL};
        // Initialized off of approximationMode and data
        double theta_;    // Accuracy or Speed, 0 -> 1.0, strict to loose
        double softening_; // Softens distance calculations
    };

} // namespace osseus

#endif // OSSEUSENGINE_UNIVERSALFORCEEVALUATOR_H