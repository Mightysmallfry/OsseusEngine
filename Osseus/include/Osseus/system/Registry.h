//
// Created by MightySmallFry on 7/20/2026.
//

#ifndef OSSEUSENGINE_REGISTRY_H
#define OSSEUSENGINE_REGISTRY_H

#include <vector>
#include <cstdint>

#include "Handle.h"

namespace osseus {
    class Registry {
    public:
        Handle CreateHandle();
        void Destroy(Handle handle);
        bool IsValid(Handle handle) const;
    private:
        std::vector<uint32_t> generations;
        std::vector<uint32_t> freeList;
        std::vector<bool> alive;
    };
} // osseus

#endif //OSSEUSENGINE_REGISTRY_H
