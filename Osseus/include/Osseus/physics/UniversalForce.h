#ifndef OSSEUSENGINE_UNIVERSALFORCEEVALUATOR_H
#define OSSEUSENGINE_UNIVERSALFORCEEVALUATOR_H

#include "Osseus/interfaces/IForceEvaluator.h"
#include "Osseus/physics/Octree.h"
#include "Osseus/system/Handle.h"

namespace osseus {

    class UniversalForceEvaluator : public IForceEvaluator {
    public:
        ~UniversalForceEvaluator() override = default;
        virtual Vector3 CalculateForce(const OctNode& sourceNode, Handle handle, const BodyData& body) = 0;
    
        
        void SetTheta(double newTheta);
        [[nodiscard]] double GetTheta() const;

        void SetSoftening(double newSoftening);
        [[nodiscard]] double GetSoftening() const;
    
    protected:
        double theta_{ 0.25 }; // Accuracy or Speed, 0 -> 1.0, strict to loose
        double softening_{ 0.0 }; // Softens distance calculations
    
    };

} // namespace osseus

#endif // OSSEUSENGINE_UNIVERSALFORCEEVALUATOR_H