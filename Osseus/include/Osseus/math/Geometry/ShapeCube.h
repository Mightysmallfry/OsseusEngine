#ifndef OSSEUSENGINE_SHAPECUBE_H
#define OSSEUSENGINE_SHAPECUBE_H

#include "Osseus/interfaces/IShape.h"

namespace osseus {
    // Axis-aligned cube — one half-extent shared by every axis, so it
    // can't be stretched into a non-cube box. No rotation support (yet):
    // faces stay aligned with the world axes, same as the AABB used for
    // broad phase.
    class ShapeCube : public IShape {
        public:
        explicit ShapeCube(double _halfExtent = 0.5) : halfExtent_(_halfExtent) {
        }

        AABB ComputeBoundingBox(const Vector3& position) const override {
            const Vector3 extents(halfExtent_, halfExtent_, halfExtent_);
            return AABB{position - extents, position + extents};
        }

        // Furthest vertex along `direction`: each axis independently
        // picks whichever face (+halfExtent or -halfExtent) is further
        // along that axis's component of the direction. Same no-rotation
        // caveat as ComputeBoundingBox above — this picks the corner in
        // world axes, not the cube's own (currently identical) axes.
        Vector3 Support(const Vector3& position, const Vector3& direction) const override {
            const Vector3 corner(direction.x >= 0.0 ? halfExtent_ : -halfExtent_,
                                 direction.y >= 0.0 ? halfExtent_ : -halfExtent_,
                                 direction.z >= 0.0 ? halfExtent_ : -halfExtent_);
            return position + corner;
        }

        double GetHalfExtent() const {
            return halfExtent_;
        }

        private:
        double halfExtent_;
    };
} // namespace osseus

#endif // OSSEUSENGINE_SHAPECUBE_H
