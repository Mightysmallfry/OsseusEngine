//
// Created by MightySmallFry on 7/21/2026.
//

#ifndef OSSEUSENGINE_SHAPEMANAGER_H
#define OSSEUSENGINE_SHAPEMANAGER_H
#include "SparseSet.h"
#include <memory>

#include "Osseus/interfaces/IShape.h"
#include "Osseus/math/Geometry/ShapePoint.h"

namespace osseus {
    class ShapeManager {
        public:
        void AddShape(Handle handle, std::unique_ptr<IShape> shape);
        void RemoveShape(Handle handle);

        void Register(Handle handle);

        IShape* GetShape(Handle handle);
        const IShape* GetShape(Handle handle) const;

        const std::vector<std::unique_ptr<IShape>>& Data() const {
            return shapes_.Data();
        }
        const std::vector<Handle>& Handles() const {
            return shapes_.Handles();
        }

        private:
        SparseSet<std::unique_ptr<IShape>> shapes_;
    };
} // namespace osseus

#endif // OSSEUSENGINE_SHAPEMANAGER_H
