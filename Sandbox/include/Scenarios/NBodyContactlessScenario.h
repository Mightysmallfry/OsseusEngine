#ifndef OSSEUS_SANDBOX_NBODYCONTACTLESSSCENARIO_H
#define OSSEUS_SANDBOX_NBODYCONTACTLESSSCENARIO_H

#include "IScenario.h"

namespace sandbox {

    class NBodyContactlessScenario : public IScenario {
        public:        
        void Initialize(osseus::PhysicsWorld& world, double width, double height) override;
        void Update(osseus::PhysicsWorld& world) override;

        private:
        osseus::UniversalGravity universalGravity_;

        double boundaryRadius_ = 0.0;
        double particleRadius_ = 5.0;
        double staticRadius_ = 5.0;
    };

} // namespace sandbox

#endif