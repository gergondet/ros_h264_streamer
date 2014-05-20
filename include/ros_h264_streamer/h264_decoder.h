#ifndef _H_ROS_H264_DECODER_H_
#define _H_ROS_H264_DECODER_H_

#include <sensor_msgs/Image.h>

namespace ros_h264_streamer
{

struct H264DecoderImpl;

class H264Decoder
{
public:
  H264Decoder(int width, int height);

  /* Decode data into a sensor_msgs::ImagePtr, the header is untouched */
  int decode(int frame_size, uint8_t * frame_data, sensor_msgs::ImagePtr & out);
private:
  boost::shared_ptr<H264DecoderImpl> impl;
};

} // ros_h264_streamer

#endif
