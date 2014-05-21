#include <ros_h264_streamer/h264_streamer.h>

int main(int argc, char * argv[])
{
  ros::init(argc, argv, "ros_h264_streamer_test_udp_server_streamer");

  ros::NodeHandle nh;

  ros_h264_streamer::H264Streamer::Config conf;
  conf.use_udp = true;
  conf.is_server = true;
  conf.port = 10000;
  conf.host = "127.0.0.1";
  conf.camera_topic = "/camera/rgb/image_raw";

  ros_h264_streamer::H264Streamer streamer(conf, nh);

  while(ros::ok())
  {
    ros::spin();
  }
}
