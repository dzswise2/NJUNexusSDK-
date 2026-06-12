/*
 * @Author: Infra Embedded
 * @Date: 2025-01-07 10:00:00
 * @LastEditors: JiangHu JiangHu@pegasus-ai.cn
 * @LastEditTime: 2025-01-07 10:00:00
 * @FilePath: teleop_adapter/src/nodes/teleop_arm_driver_main.cpp
 * @Description: 远程操控机械臂驱动节点主入口
 *
 * Copyright (c) 2025 by Pegasus, All Rights Reserved.
 */

/*******************************************************************************
 * Include
 ******************************************************************************/
#include <rclcpp/rclcpp.hpp>
#include "teleop_adapter/nodes/teleop_arm_driver_node.hpp"
#include <memory>

/*******************************************************************************
 * Function
 ******************************************************************************/
/**
 * @brief  主函数：ROS 2 程序的入口
 * @param  argc  命令行参数计数
 * @param  argv  命令行参数数组
 * @retval 0 正常退出，非0 异常退出
 */
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<teleop_adapter::TeleopArmDriverNode>();
        
        if (!node->initialize()) {
            RCLCPP_ERROR(rclcpp::get_logger("table_arm_driver_main"), 
                        "Failed to initialize Table Arm Driver Node");
            return 1;
        }
        
        RCLCPP_INFO(rclcpp::get_logger("table_arm_driver_main"), 
                   "Table Arm Driver Node started successfully");
        
        rclcpp::spin(node);
        
        node->shutdown();
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("table_arm_driver_main"), 
                    "Exception: %s", e.what());
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}