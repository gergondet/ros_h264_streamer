#include <ros/ros.h>
#include <ros_h264_streamer/h264_encoder.h>
#include <image_transport/image_transport.h>

#include <boost/program_options.hpp>
namespace po = boost::program_options;

#ifdef HAS_DGBRIDGE_MSGS
#include <dynamic_graph_bridge_msgs/RunCommand.h>
#endif

extern "C"
{
#include "x264.h"
#include "output/output.h"
}

struct MKVRecorder
{
  MKVRecorder(const std::string & topic, const std::string & camera_name)
  : nh(), it(nh), topic(topic), camera_name(camera_name),
    encoder(0), pts(0),
    hout(0), t0(0)
  {
    sub = it.subscribe(topic, 1, &MKVRecorder::imageCallback, this);
  }

  ~MKVRecorder()
  {
    delete encoder;
    if(hout)
    {
      std::cout << "Closing file" << std::endl;
      mkv_output.close_file(hout, 0, 0);
    }
  }

  void imageCallback(const sensor_msgs::ImageConstPtr & msg)
  {
    if(!encoder)
    {
      /* 20 is a high yet effective quality of recording */
      encoder = new ros_h264_streamer::H264Encoder(msg->width, msg->height, 20, 30, 1, msg->encoding, false);
      InitMKVRecorder(encoder->GetParameters());
    }
    ros_h264_streamer::H264EncoderResult res = encoder->encode(msg, pts);
    pts = static_cast<x264_picture_t*>(encoder->GetPicIn())->i_pts;
    pts++;
    if(res.frame_size)
    {
      mkv_output.write_frame(hout, res.frame_data, res.frame_size, static_cast<x264_picture_t*>(encoder->GetPicOut()));
    }
  }

  void spin()
  {
    while(ros::ok())
    {
      ros::spin();
    }
  }
private:
  void InitMKVRecorder(x264_param_t * param)
  {
    UpdateT0();
    std::stringstream ss; ss << t0 << "_" << camera_name << ".mkv";
    cli_output_opt_t output_opt;
    memset(&output_opt, 0, sizeof(cli_output_opt_t));

    if(mkv_output.open_file( const_cast<char*>(ss.str().c_str()), &hout, &output_opt))
    {
      std::cerr << "Failed to open " << ss.str() << std::endl;
      throw(std::string("Failed to open output file"));
    }

    if(mkv_output.set_param(hout, param))
    {
      std::cerr << "Failed to set encoder params on " << ss.str() << std::endl;
      throw(std::string("Failed to set encoder params"));
    }

    x264_nal_t * header;
    int i_nal;
    if(x264_encoder_headers(encoder->GetEncoder(), &header, &i_nal) < 0)
    {
      std::cerr << "x264_encoder_headers failed" << std::endl;
      throw(std::string("x264_encoder_headers failed"));
    }
    if(mkv_output.write_headers(hout, header) < 0)
    {
      std::cerr << "Failed to write headers to " << ss.str() << std::endl;
      throw(std::string("Failed to write headers"));
    }
  }

  void UpdateT0()
  {
#ifdef HAS_DGBRIDGE_MSGS
    try
    {
      dynamic_graph_bridge_msgs::RunCommand cmd;
      cmd.request.input = "solver.sot.control.time";
      ros::service::call("run_command", cmd);
      std::stringstream ss;
      ss << cmd.response.result;
      ss >> t0;
    }
    catch(...)
    {
      t0 = 0;
    }
#else
    t0 = 0;
#endif
  }

  ros::NodeHandle nh;
  image_transport::ImageTransport it;
  std::string topic;
  std::string camera_name;
  image_transport::Subscriber sub;

  /* Encoder related */
  ros_h264_streamer::H264Encoder * encoder;
  uint64_t pts;

  /* Output */
  hnd_t hout;
  unsigned int t0;
};

int main(int argc, char * argv[])
{
  std::string record_topic("/camera/rgb/image_raw");
  std::string camera_name("camera_rgb");
  /* options */
  po::options_description desc("mkv_recorder options");
  desc.add_options()
    ("help", "display help message")
    ("topic,t", po::value<std::string>(&record_topic)->default_value("/camera/rgb/image_raw"), "topic to record")
    ("name,n", po::value<std::string>(&camera_name)->default_value("camera_rgb"), "camera name, the output file will be [record_time]_[camera_name].mkv");

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  if(vm.count("help"))
  {
    std::cout << desc << std::endl;
    return 1;
  }

  /* Launch the application */
  std::stringstream ss; ss << camera_name << "_mkv_recorder";
  ros::init(argc, argv, ss.str().c_str());
  MKVRecorder mkv_recorder(record_topic, camera_name);
  mkv_recorder.spin();
  return 0;
}
