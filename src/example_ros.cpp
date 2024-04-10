#include "spdlog_ros/ros_sink.hpp"
#include "spdlog/spdlog.h"

int mainWithRos(int argc, char** argv)
{
  ros::init(argc, argv, "spdlog_ros_example");

  // Create a ROS2 node
  ros::NodeHandle node;
  // Using ROS is optional
  auto ros_sink = std::make_shared<spdlog_ros::RosSink>(node);

  // Create an async logger that logs to the console and ROS2
  auto logger = spdlog_ros::CreateAsyncLogger("my_logger", {ros_sink});
  // Optionally, make this the default logger, accessible globally
  spdlog::set_default_logger(logger);

  ros::Rate loop_rate(0.5);

  while (ros::ok())
  {
    // Log some messages
    logger->info("Hello, world!");
    logger->warn("This is a warning!");
    logger->error("This is an error!");
    spdlog::info("This message is logged using the default logger");

    ROS_INFO("This info is logged using ROS logging");
    ROS_ERROR("This error is logged using ROS logging");
    ros::spinOnce();

    loop_rate.sleep();
  }
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