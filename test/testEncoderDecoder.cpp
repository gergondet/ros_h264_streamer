#include <iostream>
#include <ros_h264_streamer/h264_encoder.h>
#include <ros_h264_streamer/h264_decoder.h>

#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>

int main(int argc, char * argv[])
{
  if(argc < 2)
  {
    std::cerr << "[Usage] " << argv[0] << " image_file" << std::endl;
    return 0;
  }

  ros::Time::init();

  std_msgs::Header header;
  cv_bridge::CvImage cvmsg(header, "bgr8", cv::imread(argv[1]));
  sensor_msgs::ImagePtr msg = cvmsg.toImageMsg();

  std::cout << "Image loaded, image size: " << msg->width << "x" << msg->height << ", encoding: " << msg->encoding << std::endl;

  ros_h264_streamer::H264Encoder encoder(msg->width, msg->height, 30, msg->encoding);
  ros::Time before_encoding = ros::Time::now();
  ros_h264_streamer::H264EncoderResult res = encoder.encode(msg);
  ros::Time after_encoding = ros::Time::now();

  ros::Duration encoding_duration = after_encoding - before_encoding;
  std::cout << "Image encoded, encoded size is " << res.frame_size << std::endl;
  std::cout << "Encoding took " << encoding_duration << std::endl;

  sensor_msgs::ImagePtr out(new sensor_msgs::Image);
  ros_h264_streamer::H264Decoder decoder(msg->width, msg->height);
  ros::Time before_decoding = ros::Time::now();
  int len = decoder.decode(res.frame_size, res.frame_data, out); 
  ros::Time after_decoding = ros::Time::now();

  std::cout << "Image decoded, decoded image size is: " << out->width << "x" << out->height << std::endl;
  std::cout << "Decoding took " << (after_decoding - before_decoding) << std::endl;

  cv_bridge::CvImagePtr cvout = cv_bridge::toCvCopy(out);
  std::cout << "Saving to disk" << std::endl;
  cv::imwrite("testEncoderDecoderResult.jpg", cvout->image);

  double fps = (double)encoding_duration.toNSec();
  fps = floor(1e9/fps);
  std::cout << "Could encode at (roughly) " << fps << "FPS" << std::endl;

  return 0;
}
