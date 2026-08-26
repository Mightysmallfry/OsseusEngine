//
// Created by MightySmallFry on 7/20/2026.
//

#include "../../include/Osseus/system/Registry.h"

namespace osseus {
    Handle Registry::CreateHandle() {
        uint32_t index;

        if (!freeList_.empty()) {
            index = freeList_.back();
            freeList_.pop_back();
            alive_[index] = true;
        } else {
            index = static_cast<uint32_t>(generations_.size());
            generations_.push_back(0);
            alive_.push_back(true);
        }

        return Handle{.index = index, .generation = generations_[index]};
    }

    void Registry::Destroy(Handle handle) {
        if (!IsValid(handle)) {
            return;
        }

        alive_[handle.index] = false;
        generations_[handle.index]++;
        freeList_.push_back(handle.index);
    }

    bool Registry::IsValid(Handle handle) const {
        if (handle.index >= generations_.size()) {
            return false;
        }
        return alive_[handle.index] && generations_[handle.index] == handle.generation;
    }
} // namespace osseus