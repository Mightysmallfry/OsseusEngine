//
// Created by MightySmallFry on 7/21/2026.
//

#include "../../include/Osseus/system/ShapeManager.h"

namespace osseus {
    void ShapeManager::AddShape(Handle handle, std::unique_ptr<IShape> shape) {
        shapes_.Insert(handle, std::move(shape));
    }

    void ShapeManager::RemoveShape(Handle handle) {
        shapes_.Remove(handle);
    }

    void ShapeManager::Register(Handle handle) {
        AddShape(handle, std::make_unique<ShapePoint>());
    }

    IShape* ShapeManager::GetShape(Handle handle) {
        std::unique_ptr<IShape>* slot = shapes_.Get(handle);
        return slot ? slot->get() : nullptr;
    }

    const IShape* ShapeManager::GetShape(Handle handle) const {
        const std::unique_ptr<IShape>* slot = shapes_.Get(handle);
        return slot ? slot->get() : nullptr;
    }
} // osseus