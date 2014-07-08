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
    : udp(true), server(true),
      port(10000), host("127.0.0.1"),
      camera_topic("/camera/rgb/image_raw"),
      fps_num(30), fps_den(1)
    {}
    bool udp;
    bool server;
    short port;
    std::string host;
    std::string camera_topic;
    int fps_num;
    int fps_den;
  };

  H264Streamer(H264Streamer::Config & conf, ros::NodeHandle & nh);
private:
  boost::shared_ptr<H264StreamerImpl> impl;
};

} // namespace ros_h264_streamer

#endif
