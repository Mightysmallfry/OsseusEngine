//
// Created by MightySmallFry on 7/20/2026.
//

#include "Osseus/system/SparseSet.h"

namespace osseus {
    template<typename T>
    std::vector<T>& SparseSet<T>::Data() {
        return dense;
    }

    template<typename T>
    const std::vector<T>& SparseSet<T>::Data() const {
        return dense;
    }

    template<typename T>
    const std::vector<Handle>& SparseSet<T>::Handles() const {
        return denseHandles;
    }
    
    template<typename T>
    void SparseSet<T>::Insert(Handle handle, T value) {
        if (Contains(handle)) {
            dense[sparseData[handle.index]] = std::move(value);
            return;
        }
        if (sparseData.size() <= handle.index) {
            sparseData.resize(handle.index + 1);
        }
        sparseData[handle.index] = static_cast<uint32_t>(dense.size());
        denseHandles.push_back(handle);
        dense.push_back(std::move(value));
    }

    template<typename T>
    void SparseSet<T>::Remove(Handle handle) {
        if (!Contains(handle)) { return; }

        uint32_t removedIndex = sparseData[handle.index];
        uint32_t lastIndex = static_cast<uint32_t>(dense.size() - 1);

        dense[removedIndex] = std::move(dense[lastIndex]);
        denseHandles[removedIndex] = denseHandles[lastIndex];

        sparseData[denseHandles[removedIndex].index] = removedIndex;

        dense.pop_back();
        denseHandles.pop_back();
    }

    template<typename T>
    T * SparseSet<T>::Get(Handle handle) {
        if (!Contains(handle)) { return nullptr; }
        return &dense[sparseData[handle.index]];
    }

    template<typename T>
    const T * SparseSet<T>::Get(Handle handle) const {
        if (!Contains(handle)) { return nullptr; }
        return &dense[sparseData[handle.index]];
    }

    template<typename T>
    bool SparseSet<T>::Contains(Handle handle) const {
        if (handle.index >= sparseData.size()) {
            return false;
        }

        uint32_t index = sparseData[handle.index];
        if (index >= denseHandles.size()) {
            return false;
        }

        return denseHandles[index].index == handle.index;
    }

    template<typename T>
    size_t SparseSet<T>::size() const {
        return dense.size();
    }
}
