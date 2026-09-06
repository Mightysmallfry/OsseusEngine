#ifndef OSSEUSENGINE_APPROXMODE_H
#define OSSEUSENGINE_APPROXMODE_H

#include <iostream>

namespace osseus {
    enum class ApproximationMode {
        EXACT,
        NORMAL,
        LIGHT,
        CUSTOM
    };

    struct ApproximationData {
        double theta = 0.25;
        double softening = 0.01;
    };

    class Approximations {
        public:
        static constexpr ApproximationData Get(ApproximationMode mode) {

            switch (mode) {
            case ApproximationMode::EXACT:
                return ApproximationData{0.0, 0.0};
            case ApproximationMode::NORMAL:
                return ApproximationData{0.25, 0.01};
            case ApproximationMode::LIGHT:
                return ApproximationData{0.6, 0.05};

            case ApproximationMode::CUSTOM:
            default:
                return ApproximationData{0.25, 0.01};
            };
        }

        private:
    };
} // namespace osseus

#endif