//
// Created by MightySmallFry on 7/19/2026.
//

#ifndef OSSEUSENGINE_SPARSESET_H
#define OSSEUSENGINE_SPARSESET_H

#include <vector>

#include "Handle.h"

namespace osseus {
    template<typename T>
    class SparseSet {
    public:
        void Insert(Handle handle, T value);
        void Remove(Handle handle);

        T* Get(Handle handle);
        const T* Get(Handle handle) const;

        bool Contains(Handle handle) const;

        std::vector<T>& Data();
        const std::vector<T>& Data() const;
        const std::vector<Handle>& Handles() const;

        size_t size() const;

    private:
        std::vector<uint32_t> sparseData;       // Handle.index -> position for dense
        std::vector<Handle> denseHandles;       // Position in dense -> owning handle
        std::vector<T> dense;                   // The Actual Data, no gaps
    };
} // osseus

#include "SparseSet.tpp"

#endif //OSSEUSENGINE_SPARSESET_H
