#include "spdlog_ros/ros_sink.hpp"

#include "rosgraph_msgs/Log.h"

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
    ros::Publisher log_publisher;
    std::mutex mutex;

    int8_t convertSeverityToROS(spdlog::level::level_enum level)
    {
        switch (level)
        {
        case spdlog::level::trace:
            return rosgraph_msgs::Log::DEBUG;
        case spdlog::level::debug:
            return rosgraph_msgs::Log::DEBUG;
        case spdlog::level::info:
            return rosgraph_msgs::Log::INFO;
        case spdlog::level::warn:
            return rosgraph_msgs::Log::WARN;
        case spdlog::level::err:
            return rosgraph_msgs::Log::ERROR;
        case spdlog::level::critical:
            return rosgraph_msgs::Log::FATAL;
        default:
            return 0;
        }
    }
};

RosSink::RosSink(ros::NodeHandle& node): pimpl_(std::make_unique<Pimpl>())
{
    pimpl_->log_publisher = node.advertise<rosgraph_msgs::Log>("rosout", 10);
}


void RosSink::log(const spdlog::details::log_msg& msg)
{
  std::lock_guard<std::mutex> lock(pimpl_->mutex);
    
  // Publish the log message
  rosgraph_msgs::Log log_msg;

  log_msg.level = pimpl_->convertSeverityToROS(msg.level);
  log_msg.name = fmt::format("{}", msg.logger_name);
  log_msg.msg = fmt::format("{}", msg.payload);
  auto sec = msg.time.time_since_epoch().count() / 1000000000;
  auto nsec = msg.time.time_since_epoch().count() % 1000000000;
  log_msg.header.stamp = ros::Time(sec, nsec);
  
  if(msg.source.filename)
  {
    log_msg.file = msg.source.filename;
  }
  if(msg.source.funcname)
  {
    log_msg.function = msg.source.funcname;
  }
  log_msg.line = msg.source.line;
  log_msg.topics = {"/rosout"};

  pimpl_->log_publisher.publish(log_msg);
}

}  // namespace spdlog_ros