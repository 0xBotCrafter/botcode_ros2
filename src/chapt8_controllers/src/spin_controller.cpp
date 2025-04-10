#include <iostream>
#include "chapt8_controllers/spin_controller.hpp"

namespace controllers
{
    void SpinController::start()
    {
        // 实现旋转运动控制逻辑
        std::cout << "SpinController::start" << std::endl;
    }
    void SpinController::stop()
    {
        // 停止运动控制
        std::cout << "SpinController::stop" << std::endl;
    }
} // namespace controllers

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(controllers::SpinController, controllers::BaseController)