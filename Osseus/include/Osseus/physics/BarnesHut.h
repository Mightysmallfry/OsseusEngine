#ifndef OSSEUSENGINE_BARNESHUT_H
#define OSSEUSENGINE_BARNESHUT_H

#include "Osseus/physics/Octree.h"
#include "Osseus/system/BodyManager.h"
#include "Osseus/system/ForceManager.h"

namespace osseus {

    // Approximates n-body gravitational forces by walking an Octree
    // once per body. A node is treated as a single point mass at its
    // center of mass when it's small/far enough (width / distance 
    // theta); otherwise the walk recurses into its children.
    class BarnesHut {
    public:
        explicit BarnesHut(double theta = 0.5, double G = 1.0, double softening = 1e-6);

        // Computes each body's net gravitational force from every
        // other body in `tree` and adds it into `forceManager` via
        // ForceManager::Add. Does not clear forceManager first --
        // callers own that decision.
        void Evaluate(const Octree& tree, const BodyManager& bodyManager, ForceManager& forceManager) const;

        void SetTheta(double newTheta);
        [[nodiscard]] double GetTheta() const;

        void SetG(double newG);
        [[nodiscard]] double GetG() const;

        void SetSoftening(double newSoftening);
        [[nodiscard]] double GetSoftening() const;

    private:
        // Accumulates the force on (queryHandle, queryPosition, queryMass)
        // contributed by `node` -- either as one point-mass contribution
        // (node accepted by the MAC) or by recursing into its children
        // (node rejected, or it's the query body's own subtree).
        [[nodiscard]] Vector3 CalculateForce(const OctNode& node, Handle queryHandle,
                                              const Vector3& queryPosition, double queryMass) const;

        // Newtonian gravitational force from a point mass at
        // sourcePosition/sourceMass acting on queryPosition/queryMass,
        // softened to avoid a singularity as distance -> 0.
        [[nodiscard]] Vector3 PointForce(const Vector3& sourcePosition, double sourceMass,
                                          const Vector3& queryPosition, double queryMass) const;

        double theta_;
        double G_;
        double softening_;
    };

} // osseus

#endif //OSSEUSENGINE_BARNESHUT_H