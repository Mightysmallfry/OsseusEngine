//
// Created by MightySmallFry on 7/24/2026.
//

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
        explicit ShapeCube(double _halfExtent = 0.5) : halfExtent(_halfExtent) {}

        AABB ComputeBoundingBox(const Vector3 &position) const override {
            const Vector3 extents(halfExtent, halfExtent, halfExtent);
            return AABB{ position - extents, position + extents };
        }

        double GetHalfExtent() const { return halfExtent; }

    private:
        double halfExtent;
    };
} // osseus

#endif //OSSEUSENGINE_SHAPECUBE_H
