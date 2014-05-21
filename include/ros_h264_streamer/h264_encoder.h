#ifndef _H_ROS_H264_ENCODER_H_
#define _H_ROS_H264_ENCODER_H_

#include <sensor_msgs/Image.h>
#include <stdint.h>

struct x264_param_t;

namespace ros_h264_streamer
{

struct H264EncoderResult
{
  H264EncoderResult() : frame_size(0), frame_data(0) {}

  int frame_size;
  uint8_t * frame_data;
};

struct H264EncoderImpl;

class H264Encoder
{
public:
  H264Encoder(int width, int height, int fps, const std::string & encoding);

  H264EncoderResult encode(const sensor_msgs::ImageConstPtr & img, uint64_t pts = 0);
private:
  boost::shared_ptr<H264EncoderImpl> impl;
};

} // namespace ros_h264_streamer

#endif
