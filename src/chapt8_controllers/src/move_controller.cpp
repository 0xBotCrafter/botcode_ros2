#include <iostream>
#include "chapt8_controllers/move_controller.hpp"

namespace controllers
{
    void MoveController::start()
    {
        // 实现平移运动控制逻辑
        std::cout << "MoveController::start" << std::endl;
    }
    void MoveController::stop()
    {
        // 停止运动控制
        std::cout << "MoveController::stop" << std::endl;
    }
} // namespace controllers

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(controllers::MoveController, controllers::BaseController)