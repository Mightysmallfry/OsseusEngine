#include "Osseus/physics/UniversalForce.h"

namespace osseus {



    void UniversalForceEvaluator::SetTheta(double newTheta) { theta_ = newTheta; }
    double UniversalForceEvaluator::GetTheta() const { return theta_; }

    void UniversalForceEvaluator::SetSoftening(double newSoftening) { softening_ = newSoftening; }
    double UniversalForceEvaluator::GetSoftening() const { return softening_; }

}