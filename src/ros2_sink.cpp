#include "spdlog_ros/ros2_sink.hpp"

#include "rcl_interfaces/msg/log.hpp"
#include "rcl/log_level.h"

#include "spdlog/spdlog.h"
#include "spdlog/async.h"
#include "spdlog/sinks/stdout_color_sinks.h"

namespace spdlog_ros
{

std::shared_ptr<spdlog::logger> CreateAsyncLogger(const std::string& name, std::vector<spdlog::sink_ptr> sinks)
{
  static auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
  static bool oninit = true;
  if(oninit)
  {
    spdlog::init_thread_pool(32768, 1); // queue with max 32k items 1 backing thread.
    oninit = false;
  }
  sinks.push_back(console_sink);
  return std::make_shared<spdlog::async_logger>(name, sinks.begin(), sinks.end(), spdlog::thread_pool(), spdlog::async_overflow_policy::overrun_oldest);
}

RosSink::~RosSink() {}

//Pimpl idiom
struct RosSink::Pimpl
{
    rclcpp::Node::SharedPtr node;
    rclcpp::Publisher<rcl_interfaces::msg::Log>::SharedPtr log_publisher;
     std::mutex mutex;

    RCUTILS_LOG_SEVERITY convertSeverityToROS(spdlog::level_t level)
    {
        switch (level)
        {
        case spdlog::level::trace:
            return RCUTILS_LOG_SEVERITY_DEBUG;
        case spdlog::level::debug:
            return RCUTILS_LOG_SEVERITY_DEBUG;
        case spdlog::level::info:
            return RCUTILS_LOG_SEVERITY_INFO;
        case spdlog::level::warn:
            return RCUTILS_LOG_SEVERITY_WARN;
        case spdlog::level::err:
            return RCUTILS_LOG_SEVERITY_ERROR;
        case spdlog::level::critical:
            return RCUTILS_LOG_SEVERITY_FATAL;
        case spdlog::level::off:
            return RCUTILS_LOG_SEVERITY_UNSET;
        default:
            return RCUTILS_LOG_SEVERITY_UNSET;
        }
    }
};

RosSink::RosSink(rclcpp::Node::SharedPtr node): pimpl_(std::make_unique<Pimpl>())
{
    pimpl_->node = node;
    pimpl_->log_publisher = pimpl_->node->create_publisher<rcl_interfaces::msg::Log>("rosout", rclcpp::RosoutQoS());
}


void RosSink::log(const spdlog::details::log_msg& msg)
{
  std::lock_guard<std::mutex> lock(pimpl_->mutex);
    
  // Publish the log message
  rcl_interfaces::msg::Log log_msg;

  log_msg.level = pimpl_->convertSeverityToROS(msg.level);
  log_msg.name = fmt::format("{}", msg.logger_name);
  log_msg.msg = fmt::format("{}", msg.payload);
  log_msg.stamp.sec = msg.time.time_since_epoch().count() / 1000000000;
  log_msg.stamp.nanosec = msg.time.time_since_epoch().count() % 1000000000;
  
  if(msg.source.filename)
  {
    log_msg.file = msg.source.filename;
  }
  if(msg.source.funcname)
  {
    log_msg.function = msg.source.funcname;
  }
  log_msg.line = msg.source.line;

  pimpl_->log_publisher->publish(log_msg);
}

}  // namespace spdlog_ros