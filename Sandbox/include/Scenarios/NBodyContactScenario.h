#ifndef OSSEUS_SANDBOX_NBODYCONTACTSCENARIO_H
#define OSSEUS_SANDBOX_NBODYCONTACTSCENARIO_H

#include "IScenario.h"

namespace sandbox {

    class NBodyContactScenario : public IScenario {
        public:        
        void Initialize(osseus::PhysicsWorld& world, double width, double height) override;
        void Update(osseus::PhysicsWorld& world) override;

        private:
        osseus::UniversalGravity universalGravity_;

        const std::size_t bodyCount = 100;
        const double mass = 100.0;

        double boundaryRadius_ = 0.0;
        double particleRadius_ = 5.0;
        double staticRadius_ = 5.0;
        
        const double G = OsseusConstants::GravitationalConstant;
        const double orbitalSpeed = 0;
    };

} // namespace sandbox

#endif