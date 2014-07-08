#include <ros_h264_streamer/h264_streamer.h>

#include <boost/program_options.hpp>
namespace po = boost::program_options;

int main(int argc, char * argv[])
{
  ros::init(argc, argv, "ros_h264_streamer_test_streamer");
  ros::NodeHandle nh;

  ros_h264_streamer::H264Streamer::Config conf;
  conf.udp = false;
  conf.server = false;
  conf.port = 10000;
  conf.host = "127.0.0.1";
  conf.camera_topic = "/camera/rgb/image_raw";

  po::options_description desc("testStreamer options");
  desc.add_options()
    ("help", "display help message")
    ("udp,u", po::value<bool>(&conf.udp)->default_value("udp"), "connection type; valid values are tcp and udp")
    ("server,s", po::value<bool>(&conf.server)->default_value(false), "is this a server?")
    ("port,p", po::value<short>(&conf.port)->default_value(10000), "connection port")
    ("host,h", po::value<std::string>(&conf.host)->default_value("127.0.0.1"), "connection host (irrelevant for server mode")
    ("topic,t", po::value<std::string>(&conf.camera_topic)->default_value("/camera/rgb/image_raw"), "camera topic to stream")
    ("fps_num", po::value<int>(&conf.fps_num)->default_value(30), "FPS numerator value, will be also interpreted as camera real fps")
    ("fps_den", po::value<int>(&conf.fps_den)->default_value(1), "FPS denominator value, will be interpreted as sub-sampling denominator");

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  if(vm.count("help"))
  {
    std::cout << desc << std::endl;
    return 1;
  }

  ros_h264_streamer::H264Streamer streamer(conf, nh);

  while(ros::ok())
  {
    ros::spin();
  }

  return 0;
}
