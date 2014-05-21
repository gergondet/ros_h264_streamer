#ifndef _H_ROS_H264_STREAMER_H_
#define _H_ROS_H264_STREAMER_H_

#include <string>
#include <boost/shared_ptr.hpp>
#include <ros/ros.h>

namespace ros_h264_streamer
{

struct H264StreamerImpl;


class H264Streamer
{
public:
  struct Config
  {
    Config()
    : use_udp(true), is_server(true),
      port(10000), host("127.0.0.1"),
      camera_topic("/camera/rgb/image_raw")
    {}
    bool use_udp;
    bool is_server;
    short port;
    std::string host;
    std::string camera_topic;
  };

  H264Streamer(H264Streamer::Config & conf, ros::NodeHandle & nh);
private:
  boost::shared_ptr<H264StreamerImpl> impl;
};

} // namespace ros_h264_streamer

#endif
