#include "Osseus/physics/UniversalForce.h"

namespace osseus {

    UniversalForceEvaluator::UniversalForceEvaluator() {
        SetApproximationMode(approximationMode);
    }

    void UniversalForceEvaluator::SetApproximationMode(ApproximationMode mode) {
        approximationMode = mode;
        
        ApproximationData guidelines = Approximations::Get(mode);

        if (mode == ApproximationMode::CUSTOM) {
            std::cerr << "[Warning] custom approximation mode has been selected, ensure approximation data has been handed forward\n";
        }
        SetApproximationData(guidelines);
    }

    ApproximationMode UniversalForceEvaluator::GetApproximationMode() {
        return approximationMode;
    }

    void UniversalForceEvaluator::SetApproximationData(ApproximationData data) {
        SetTheta(data.theta);
        SetSoftening(data.softening);
    }


    void UniversalForceEvaluator::SetTheta(double newTheta) {
        theta_ = newTheta;
    }
    double UniversalForceEvaluator::GetTheta() const {
        return theta_;
    }

    void UniversalForceEvaluator::SetSoftening(double newSoftening) {
        softening_ = newSoftening;
    }
    double UniversalForceEvaluator::GetSoftening() const {
        return softening_;
    }

} // namespace osseus