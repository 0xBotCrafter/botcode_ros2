#ifndef SPIN_CONTROLLER_HPP
#define SPIN_CONTROLLER_HPP

#include "chapt8_controllers/base_controller.hpp"

namespace controllers
{
    class SpinController : public BaseController
    {
    public:
        void start() override;
        void stop() override;
    };

} // namespace controllers

#endif // SPIN_CONTROLLER_HPP