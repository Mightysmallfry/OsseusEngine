#ifndef OSSEUSENGINE_BAUMGARTE_H
#define OSSEUSENGINE_BAUMGARTE_H

#include <vector>

#include "Collision/Contact.h"
#include "Osseus/system/BodyManager.h"

namespace osseus {
    // Impulse-based contact resolution with Baumgarte-style positional
    // correction to counteract the sinking that discrete timesteps
    // otherwise leave behind.
    class Baumgarte {
        public:
        void SetRestitution(double newRestitution) {
            restitution_ = newRestitution;
        }
        double GetRestitution() const {
            return restitution_;
        }

        void ResolveContacts(const std::vector<Contact>& contacts, BodyManager& bodyManager) const;

        private:
        double restitution_{0.2};
        double positionalCorrectionPercent_{0.8};
        double positionalSlop_{0.01};
    };
} // namespace osseus

#endif // OSSEUSENGINE_SOLVER_H
