#include <ros_h264_streamer/h264_receiver.h>

#include <boost/program_options.hpp>
namespace po = boost::program_options;

int main(int argc, char * argv[])
{
  ros::init(argc, argv, "ros_h264_streamer_test_receiver");
  ros::NodeHandle nh;

  ros_h264_streamer::H264Receiver::Config conf;
  conf.width = 640; conf.height = 480;
  conf.udp = false;
  conf.server = false;
  conf.port = 10000;
  conf.host = "127.0.0.1";

  conf.publish = true;
  conf.publish_topic = "/ros_h264_receiver/rgb/image_raw";

  po::options_description desc("testStreamer options");
  desc.add_options()
    ("help", "display help message")
    ("width", po::value<int>(&conf.width)->default_value(640), "image width")
    ("height", po::value<int>(&conf.height)->default_value(480), "image height")
    ("udp,u", po::value<bool>(&conf.udp)->default_value("udp"), "connection type; valid values are tcp and udp")
    ("server,s", po::value<bool>(&conf.server)->default_value(false), "is this a server?")
    ("port,p", po::value<short>(&conf.port)->default_value(10000), "connection port")
    ("host,h", po::value<std::string>(&conf.host)->default_value("127.0.0.1"), "connection host (irrelevant for server mode");

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  if(vm.count("help"))
  {
    std::cout << desc << std::endl;
    return 1;
  }

  ros_h264_streamer::H264Receiver receiver(conf, nh);

  while(ros::ok())
  {
    ros::spin();
  }

  return 0;
}
