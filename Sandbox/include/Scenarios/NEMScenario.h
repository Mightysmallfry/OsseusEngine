#ifndef OSSEUS_SANDBOX_NEMSCENARIO_H
#define OSSEUS_SANDBOX_NEMSCENARIO_H

#include "IScenario.h"

namespace sandbox {

    class NEMScenario : public IScenario {
        public:        
        void Initialize(osseus::PhysicsWorld& world, double width, double height) override;
        void Update(osseus::PhysicsWorld& world) override;

        private:
        osseus::UniversalElectroMag emForce_;
        osseus::UniversalGravity gravity_;

        const std::size_t bodyCount = 100;
        const double mass = 1.0;
        const double chargeMagnitude = 50.0;

        double boundaryRadius_ = 0.0;
        double particleRadius_ = 5.0;
        double staticRadius_ = 5.0;
        
        const double G = OsseusConstants::GravitationalConstant;
        const double orbitalSpeed = 0;
    };

} // namespace sandbox

#endif