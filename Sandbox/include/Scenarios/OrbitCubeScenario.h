#ifndef OSSEUS_SANDBOX_ORBITCUBESCENARIO_H
#define OSSEUS_SANDBOX_ORBITCUBESCENARIO_H

#include "IScenario.h"

namespace sandbox {

    class OrbitCubeScenario : public IScenario {
        public:        
        void Initialize(osseus::PhysicsWorld& world, double width, double height) override;
        void Update(osseus::PhysicsWorld& world) override;

        private:
        osseus::UniversalGravity universalGravity_;

        const double staticMass = 500000.0;

        const std::size_t bodyCount = 1000;
        const double mass = 1.0;
        const double speed = 50.0;

        double boundaryRadius_ = 0.0;
        double halfExtent_ = 3.0;
        double staticRadius_ = 20.0;
    };

} // namespace sandbox

#endif