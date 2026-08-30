#ifndef OSSEUS_SANDBOX_ORBITSCENARIO_H
#define OSSEUS_SANDBOX_ORBITSCENARIO_H

#include "IScenario.h"

namespace sandbox {

    class OrbitScenario : public IScenario {
        public:        
        void Initialize(osseus::PhysicsWorld& world, double width, double height) override;
        void Update(osseus::PhysicsWorld& world) override;

        private:
        osseus::UniversalGravity universalGravity_;

        double boundaryRadius_ = 0.0;
        double particleRadius_ = 2.0;
        double staticRadius_ = 20.0;
    };

} // namespace sandbox

#endif