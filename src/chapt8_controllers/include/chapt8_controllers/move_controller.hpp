#ifndef MOVE_CONTROLLER_HPP
#define MOVE_CONTROLLER_HPP

#include "chapt8_controllers/base_controller.hpp"

namespace controllers
{
    class MoveController : public BaseController
    {
    public:
        void start() override;
        void stop() override;
    };

} // namespace controllers

#endif // MOVE_CONTROLLER_HPP