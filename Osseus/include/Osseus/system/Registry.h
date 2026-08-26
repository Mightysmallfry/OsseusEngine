//
// Created by MightySmallFry on 7/20/2026.
//

#ifndef OSSEUSENGINE_REGISTRY_H
#define OSSEUSENGINE_REGISTRY_H

#include <cstdint>
#include <vector>

#include "Handle.h"

namespace osseus {
    class Registry {
        public:
        Handle CreateHandle();
        void Destroy(Handle handle);
        bool IsValid(Handle handle) const;

        private:
        std::vector<uint32_t> generations_;
        std::vector<uint32_t> freeList_;
        std::vector<bool> alive_;
    };
} // namespace osseus

#endif // OSSEUSENGINE_REGISTRY_H
