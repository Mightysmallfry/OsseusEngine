//
// Created by MightySmallFry on 7/20/2026.
//

#ifndef OSSEUSENGINE_SOLVER_H
#define OSSEUSENGINE_SOLVER_H
#include <vector>

#include "Collision/Contact.h"
#include "Osseus/system/BodyManager.h"

namespace osseus {
    // Impulse-based contact resolution with Baumgarte-style positional
    // correction to counteract the sinking that discrete timesteps
    // otherwise leave behind.
    class Solver {
    public:
        void SetRestitution(double newRestitution) { restitution = newRestitution; }
        double GetRestitution() const { return restitution; }

        void ResolveContacts(const std::vector<Contact>& contacts, BodyManager& bodyManager) const;

    private:
        double restitution{ 0.2 };
        double positionalCorrectionPercent{ 0.8 };
        double positionalSlop{ 0.01 };
    };
} // osseus

#endif //OSSEUSENGINE_SOLVER_H
