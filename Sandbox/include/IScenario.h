#ifndef OSSEUS_SANDBOX_ISCENARIO_H
#define OSSEUS_SANDBOX_ISCENARIO_H

#include <Osseus/Osseus.h>
#include <SFML/Graphics.hpp>

#include <memory>
#include <vector>

namespace sandbox {

    struct RenderObject {
        osseus::Handle handle;
        std::unique_ptr<sf::Shape> shape;
    };

    class IScenario {
        public:
        virtual ~IScenario() = default;

        virtual void Initialize(osseus::PhysicsWorld& world, double width, double height) = 0;
        virtual void Update(osseus::PhysicsWorld& world) = 0;

        const std::vector<RenderObject>& GetRenderObjects() const {
            return renderObjects_;
        }

        protected:
        std::vector<RenderObject> renderObjects_;
    };

} // namespace sandbox

#endif