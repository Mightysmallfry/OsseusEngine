#ifndef OSSEUSENGINE_FORCEMANAGER_H
#define OSSEUSENGINE_FORCEMANAGER_H

#include <vector>

#include "Osseus/math/Vector3.h"
#include "Osseus/system/Handle.h"


namespace osseus{
    class ForceManager
    {
    public:
        void Resize(std::size_t count);
    
        void Clear();
    
        void Add(Handle handle, const Vector3& force);
    
        [[nodiscard]]
        const Vector3& Get(Handle handle) const;
    
    private:
        std::vector<Vector3> forces_;
    };
}
    

#endif