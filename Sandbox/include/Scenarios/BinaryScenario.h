#ifndef OSSEUS_SANDBOX_BINARYSCENARIO_H
#define OSSEUS_SANDBOX_BINARYSCENARIO_H

#include "IScenario.h"
#include "Osseus/Osseus.h"

    namespace sandbox {

    class BinaryScenario : public IScenario {
        public:
        void Initialize(osseus::PhysicsWorld& world, double width, double height) override;

        void Update(osseus::PhysicsWorld& world) override;

        private:
        // Simulation bounds
        double width_ = 0.0;
        double height_ = 0.0;

        // Binary system
        double starMass_ = 125000.0;
        double starRadius_ = 20.0;
        double orbitalRadius_ = 0.0;
        double orbitalSpeed_ = 0.0;

        // Physics
        osseus::UniversalGravity universalGravity_;

        // Bodies
        osseus::Handle star1_;
        osseus::Handle star2_;
    };

} // namespace sandbox

#endif
