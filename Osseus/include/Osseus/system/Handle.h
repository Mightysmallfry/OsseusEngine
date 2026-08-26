//
// Created by MightySmallFry on 7/19/2026.
//

#ifndef OSSEUSENGINE_PHYSICSHANDLE_H
#define OSSEUSENGINE_PHYSICSHANDLE_H

#include <stdint.h>

namespace osseus {
    struct Handle {
        uint32_t index;
        uint32_t generation;

        constexpr bool operator==(const Handle& other) const noexcept {
            return index == other.index && generation == other.generation;
        }
        constexpr bool operator!=(const Handle& other) const noexcept {
            return !(*this == other);
        }
    };
} // namespace osseus

#endif // OSSEUSENGINE_PHYSICSHANDLE_H