#include <boost/asio.hpp>
#include <boost/thread.hpp>

#include <ros_h264_streamer/h264_streamer.h>

#include <ros/ros.h>

#include <ros_h264_streamer/h264_encoder.h>
#include <image_transport/image_transport.h>

#include "private/net_buffer_size.h"

using boost::asio::ip::udp;
using boost::asio::ip::tcp;

namespace ros_h264_streamer
{

struct H264StreamerNetImpl
{
  H264StreamerNetImpl()
  : io_service_(), io_service_th_(0), request_data(0), chunk_data(0)
  {
    request_data = new char[ros_h264_streamer_private::_request_size];
    chunk_data = new unsigned char[ros_h264_streamer_private::_video_chunk_size];
    CleanRequestData();
    CleanChunkData();
  }

  ~H264StreamerNetImpl()
  {
    io_service_.stop();
    if(io_service_th_)
    {
      io_service_th_->join();
      delete io_service_th_;
    }
    delete[] request_data;
    delete[] chunk_data;
  }

  void StartIOService()
  {
    io_service_th_ = new boost::thread(boost::bind(&boost::asio::io_service::run, &io_service_));
  }

  void HandleNewData(H264EncoderResult & res)
  {
    int data_size = 0;
    uint8_t chunkID = 0;
    do
    {
      CleanChunkData();
      data_size = std::min(res.frame_size, ros_h264_streamer_private::_video_chunk_size -1 );
      chunk_data[0] = chunkID;
      std::memcpy(&chunk_data[1], &res.frame_data[chunkID*(ros_h264_streamer_private::_video_chunk_size - 1)], data_size);
      SendData(data_size);
      chunkID++;
      res.frame_size -= data_size;
    }
    while(data_size == ros_h264_streamer_private::_video_chunk_size - 1);
  }

  virtual void SendData(int frame_size) = 0;

  boost::asio::io_service io_service_;
  boost::thread * io_service_th_;

  char * request_data;
  void CleanRequestData() { memset(request_data, 0, ros_h264_streamer_private::_request_size); }
  unsigned char * chunk_data;
  void CleanChunkData() { memset(chunk_data, 0, ros_h264_streamer_private::_video_chunk_size); }
};

struct H264StreamerUDPServer : public H264StreamerNetImpl
{
  H264StreamerUDPServer(short port)
  : socket(0), has_client(false), client_endpoint()
  {
    socket = new udp::socket(io_service_);
    socket->open(udp::v4());
    socket->bind(udp::endpoint(udp::v4(), port));
    socket->async_receive_from(
      boost::asio::buffer(request_data, ros_h264_streamer_private::_request_size), request_endpoint,
      boost::bind(&H264StreamerUDPServer::handle_receive_from, this,
        boost::asio::placeholders::error,
        boost::asio::placeholders::bytes_transferred));
  }

  ~H264StreamerUDPServer()
  {
    socket->close();
    delete socket;
  }

  void SendData(int frame_size)
  {
    if(has_client)
    {
      boost::system::error_code error;
      socket->send_to(
        boost::asio::buffer(chunk_data, frame_size),
        client_endpoint, 0, error);
      if(error)
      {
        std::cerr << "[ros_h264_streamer] H264Streamer UDP server got the error while sending data: " << std::endl << error.message() << std::endl;
        has_client = false;
      }
    }
  }

  void handle_receive_from(const boost::system::error_code & error, size_t bytes_recvd)
  {
    if(!error && bytes_recvd > 0)
    {
      has_client = true;
      client_endpoint = request_endpoint;
    }
    else if(error)
    {
      std::cerr << "[ros_h264_streamer] H264Streamer UDP server got the error while receiving data: " << std::endl << error.message() << std::endl;
    }
    socket->async_receive_from(
      boost::asio::buffer(request_data, ros_h264_streamer_private::_request_size), request_endpoint,
      boost::bind(&H264StreamerUDPServer::handle_receive_from, this,
        boost::asio::placeholders::error,
        boost::asio::placeholders::bytes_transferred));
  }
    
private:
  udp::socket * socket;
  bool has_client;
  udp::endpoint request_endpoint;
  udp::endpoint client_endpoint;
};

struct H264StreamerUDPClient : public H264StreamerNetImpl
{
  H264StreamerUDPClient(const std::string & host, short port)
  {
  }

  void SendData(int frame_size)
  {
  }
};

struct H264StreamerTCPServer : public H264StreamerNetImpl
{
  H264StreamerTCPServer(short port)
  {
  }

  void SendData(int frame_size)
  {
  }
};

struct H264StreamerTCPClient : public H264StreamerNetImpl
{
  H264StreamerTCPClient(const std::string & host, short port)
  {
  }

  void SendData(int frame_size)
  {
  }
};

struct H264StreamerImpl
{
public:
  H264StreamerImpl(H264Streamer::Config & conf, ros::NodeHandle & nh)
  : nh(nh), it(nh), conf(conf), net_impl(0), encoder(0)
  {
    sub = it.subscribe(conf.camera_topic, 1, &H264StreamerImpl::imageCallback, this);
    if(conf.use_udp)
    {
      if(conf.is_server)
      {
        net_impl = new H264StreamerUDPServer(conf.port);
      }
      else
      {
        net_impl = new H264StreamerUDPClient(conf.host, conf.port);
      }
    }
    else
    {
      if(conf.is_server)
      {
        net_impl = new H264StreamerTCPServer(conf.port);
      }
      else
      {
        net_impl = new H264StreamerTCPClient(conf.host, conf.port);
      }
    }
    net_impl->StartIOService();
  }

  ~H264StreamerImpl()
  {
    sub.shutdown();
    delete encoder;
  }

  void imageCallback(const sensor_msgs::ImageConstPtr & msg)
  {
    if(!encoder)
    {
      encoder = new H264Encoder(msg->width, msg->height, 30, msg->encoding);
    }
    H264EncoderResult res = encoder->encode(msg);
    net_impl->HandleNewData(res);
  }
private:
  ros::NodeHandle & nh;
  image_transport::ImageTransport it;
  image_transport::Subscriber sub;
  H264Streamer::Config & conf;

  H264StreamerNetImpl * net_impl;

  H264Encoder * encoder;
};

H264Streamer::H264Streamer(H264Streamer::Config & conf, ros::NodeHandle & nh)
: impl(new H264StreamerImpl(conf, nh))
{
}

} // namespace ros_h264_streamer
