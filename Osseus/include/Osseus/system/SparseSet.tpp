//
// Created by MightySmallFry on 7/20/2026.
//

#include "Osseus/system/SparseSet.h"

namespace osseus {
    template<typename T>
    std::vector<T>& SparseSet<T>::Data() {
        return dense_;
    }

    template<typename T>
    const std::vector<T>& SparseSet<T>::Data() const {
        return dense_;
    }

    template<typename T>
    const std::vector<Handle>& SparseSet<T>::Handles() const {
        return denseHandles_;
    }
    
    template<typename T>
    void SparseSet<T>::Insert(Handle handle, T value) {
        if (Contains(handle)) {
            dense_[sparseData_[handle.index]] = std::move(value);
            return;
        }
        if (sparseData_.size() <= handle.index) {
            sparseData_.resize(handle.index + 1);
        }
        sparseData_[handle.index] = static_cast<uint32_t>(dense_.size());
        denseHandles_.push_back(handle);
        dense_.push_back(std::move(value));
    }

    template<typename T>
    void SparseSet<T>::Remove(Handle handle) {
        if (!Contains(handle)) { return; }

        uint32_t removedIndex = sparseData_[handle.index];
        uint32_t lastIndex = static_cast<uint32_t>(dense_.size() - 1);

        dense_[removedIndex] = std::move(dense_[lastIndex]);
        denseHandles_[removedIndex] = denseHandles_[lastIndex];

        sparseData_[denseHandles_[removedIndex].index] = removedIndex;

        dense_.pop_back();
        denseHandles_.pop_back();
    }

    template<typename T>
    T * SparseSet<T>::Get(Handle handle) {
        if (!Contains(handle)) { return nullptr; }
        return &dense_[sparseData_[handle.index]];
    }

    template<typename T>
    const T * SparseSet<T>::Get(Handle handle) const {
        if (!Contains(handle)) { return nullptr; }
        return &dense_[sparseData_[handle.index]];
    }

    template<typename T>
    bool SparseSet<T>::Contains(Handle handle) const {
        if (handle.index >= sparseData_.size()) {
            return false;
        }

        uint32_t index = sparseData_[handle.index];
        if (index >= denseHandles_.size()) {
            return false;
        }

        return denseHandles_[index].index == handle.index;
    }

    template<typename T>
    size_t SparseSet<T>::size() const {
        return dense_.size();
    }
}
