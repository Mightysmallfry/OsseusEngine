#ifndef OSSEUSENGINE_EPA_H
#define OSSEUSENGINE_EPA_H
#include <utility>
#include <vector>

#include "GJK.h"
#include "Contact.h"
#include "Osseus/interfaces/IShape.h"
#include "Osseus/system/Handle.h"

namespace osseus {
    // Expanding Polytope Algorithm. Once GJK confirms an overlap it
    // leaves behind a tetrahedron enclosing the origin; EPA inflates
    // that tetrahedron toward the Minkowski surface to recover the
    // penetration depth, separating normal, and contact points. Like
    // GJK, it only calls IShape::Support, so it works for any shape pair.
    class EPA {
    public:
        static Contact Resolve(const IShape& shapeA, const Vector3& posA, Handle handleA,
                                const IShape& shapeB, const Vector3& posB, Handle handleB,
                                const GJKSimplex& startingSimplex);

    private:
        struct Face {
            int a;
            int b;
            int c;
            Vector3 normal;
            double distance;
        };

        static Face MakeFace(const std::vector<GJKSupportPoint>& polytope, int a, int b, int c);
        static void AddUniqueEdge(std::vector<std::pair<int, int>>& edges, int a, int b);
        static Contact BuildContact(const std::vector<GJKSupportPoint>& polytope, const Face& face,
                                     Handle handleA, Handle handleB);
    };
} // osseus

#endif //OSSEUSENGINE_EPA_H
