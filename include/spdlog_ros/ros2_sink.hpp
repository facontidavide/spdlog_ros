#pragma once

#include "spdlog/sinks/sink.h"
#include "spdlog/logger.h"
#include "rclcpp/rclcpp.hpp"

namespace spdlog_ros
{   

class RosSink : public spdlog::sinks::sink 
{
public:
  RosSink(rclcpp::Node::SharedPtr node);

  RosSink(const RosSink& other) = delete;
  RosSink& operator=(const RosSink& other) = delete;

  ~RosSink() override;

  void log(const spdlog::details::log_msg& msg) override;

  void flush() override {}

  void set_pattern(const std::string&) override {}

  void set_formatter(std::unique_ptr<spdlog::formatter>) override {}

private:

  struct Pimpl;
  std::unique_ptr<Pimpl> pimpl_;
};

void EnableRosLogger(const rclcpp::Node::SharedPtr& node);

std::shared_ptr<spdlog::logger> CreateAsyncLogger(const std::string& name, std::vector<spdlog::sink_ptr> sinks = {});

}  // namespace spdlog_ros