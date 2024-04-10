#include "spdlog_ros/ros2_sink.hpp"
#include "spdlog/spdlog.h"

int mainWithRos(int argc, char** argv)
{

  rclcpp::init(argc, argv);

  // Create a ROS2 node
  auto node = rclcpp::Node::make_shared("spdlog_ros_example");
  // Using ROS is optional
  auto ros_sink = std::make_shared<spdlog_ros::RosSink>(node);

  // Create an async logger that logs to the console and ROS2
  auto logger = spdlog_ros::CreateAsyncLogger("my_logger", {ros_sink});
  // Optionally, make this the default logger, accessible globally
  spdlog::set_default_logger(logger);

  // Log some messages
  logger->info("Hello, world!");
  logger->warn("This is a warning!");
  logger->error("This is an error!");
  spdlog::info("This message is logged using the default logger");

  return 0;
}

int mainWithoutROS(int, char**)
{
  // Create a logger with a name
  auto logger = spdlog::get("my_logger");
  // Optionally, make this the default logger, accessible globally
  spdlog::set_default_logger(logger);

  // Log some messages
  logger->info("Hello, world!");
  logger->warn("This is a warning!");
  logger->error("This is an error!");
  spdlog::info("This message is logged using the default logger");

  return 0;
}

int main(int argc, char** argv)
{
  return mainWithRos(argc, argv);
}