#ifndef _H_ROS_H264_RECEIVER_H_
#define _H_ROS_H264_RECEIVER_H_

#include <string>
#include <boost/shared_ptr.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

namespace ros_h264_streamer
{

struct H264ReceiverImpl;


class H264Receiver
{
public:
  struct Config
  {
    Config()
    : width(640), height(480),
      use_udp(true), is_server(true),
      port(10000), host("127.0.0.1"),
      publish_topic("/ros_h264_receiver/rgb/image_raw"), frame_id("/ros_h264_receiver_rgb_optical_frame")

    {}
    int width;
    int height;
    bool use_udp;
    bool is_server;
    short port;
    std::string host;
    std::string publish_topic;
    std::string frame_id;
  };

  H264Receiver(H264Receiver::Config & conf, ros::NodeHandle & nh);

  /* Returns true if we are returning a new image compared to last call */
  bool getLatestImage(sensor_msgs::ImagePtr & img);
private:
  boost::shared_ptr<H264ReceiverImpl> impl;
};

} // namespace ros_h264_Receiver

#endif
