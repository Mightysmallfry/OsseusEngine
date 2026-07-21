//
// Created by MightySmallFry on 7/20/2026.
//

#include "../../include/Osseus/system/Registry.h"

namespace osseus {
    Handle Registry::CreateHandle() {
        uint32_t index;

        if (!freeList.empty()) {
            index = freeList.back();
            freeList.pop_back();
            alive[index] = true;
        } else {
            index = static_cast<uint32_t>(generations.size());
            generations.push_back(0);
            alive.push_back(true);
        }

        return Handle{ .index = index, .generation = generations[index] };
    }

    void Registry::Destroy(Handle handle) {
        if (!IsValid(handle)) { return; }

        alive[handle.index] = false;
        generations[handle.index]++;
        freeList.push_back(handle.index);
    }

    bool Registry::IsValid(Handle handle) const {
        if (handle.index >= generations.size()) {
            return false;
        }
        return alive[handle.index] && generations[handle.index] == handle.generation;
    }
} // osseus