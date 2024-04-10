#pragma once

#include "spdlog/sinks/sink.h"
#include "spdlog/logger.h"
#include "ros/ros.h"

namespace spdlog_ros
{   

class RosSink : public spdlog::sinks::sink 
{
public:
  RosSink(ros::NodeHandle& node);

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

std::shared_ptr<spdlog::logger> CreateAsyncLogger(const std::string& name, std::vector<spdlog::sink_ptr> sinks = {});

}  // namespace spdlog_ros