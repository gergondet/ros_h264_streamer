#include <ros_h264_streamer/h264_receiver.h>
#include <sensor_msgs/Image.h>

#include <sstream>
#include <iomanip>

int main(int argc, char * argv[])
{
  ros::init(argc, argv, "ros_h264_streamer_test_udp_client_receiver");

  ros::NodeHandle nh;

  ros_h264_streamer::H264Receiver::Config conf;
  conf.width = 640;
  conf.height = 480;
  conf.udp = true;
  conf.server = true;
  conf.port = 10000;
  conf.host = "127.0.0.1";
  conf.publish_topic = "/ros_h264_receiver/rgb/image_raw";
  conf.frame_id = "/ros_h264_receiver_rgb_optical_frame";

  ros_h264_streamer::H264Receiver receiver(conf, nh);

  while(ros::ok())
  {
    ros::spin();
  }
}
