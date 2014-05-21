#include <boost/asio.hpp>
#include <boost/thread.hpp>

#include <ros_h264_streamer/h264_receiver.h>

#include "private/net_buffer_size.h"

#include <ros/ros.h>

#include <ros_h264_streamer/h264_decoder.h>
#include <image_transport/image_transport.h>

#include "private/net_buffer_size.h"

using boost::asio::ip::udp;
using boost::asio::ip::tcp;

namespace ros_h264_streamer
{

struct H264ReceiverNetImpl
{
  H264ReceiverNetImpl(H264Receiver::Config & conf, ros::NodeHandle & nh)
  : io_service(), io_service_th(0), request_data(0), chunk_data(0),
    frame_data_size(0), full_frame_data_size(0), frame_data(0),
    has_new_data(false), img(new sensor_msgs::Image),
    decoder(conf.width, conf.height),
    it(nh), pub()
  {
    request_data = new char[ros_h264_streamer_private::_request_size];
    chunk_data = new unsigned char[ros_h264_streamer_private::_video_chunk_size];
    frame_data_size = 0;
    full_frame_data_size = ros_h264_streamer_private::_video_chunk_size;
    frame_data = new uint8_t[full_frame_data_size];
    CleanRequestData();
    CleanChunkData();
    CleanFrameData();

    img->header.seq = 0;
    img->header.frame_id = conf.frame_id;
    pub = it.advertise(conf.publish_topic, 1);
  }

  ~H264ReceiverNetImpl()
  {
    io_service.stop();
    if(io_service_th)
    {
      io_service_th->join();
      delete io_service_th;
    }
    delete[] request_data;
    delete[] chunk_data;
    delete[] frame_data;
  }

  void StartIOService()
  {
    io_service_th = new boost::thread(boost::bind(&boost::asio::io_service::run, &io_service));
  }

  void HandleVideoChunk(size_t bytes_recvd)
  {
    uint8_t chunkID = chunk_data[0];
    frame_data_size += bytes_recvd;
    if(frame_data_size > full_frame_data_size)
    {
      std::cerr << "[ros_h264_streamer] H264Receiver needs to re-allocate frame data" << std::endl;
      delete[] frame_data;
      frame_data = new uint8_t[2*frame_data_size];
      full_frame_data_size = 2*frame_data_size;
    }
    memcpy(&frame_data[chunkID*(ros_h264_streamer_private::_video_chunk_size-1)], &chunk_data[1], bytes_recvd - 1);
    if(bytes_recvd < ros_h264_streamer_private::_video_chunk_size)
    {
      decoder.decode(frame_data_size, frame_data, img);
      has_new_data = true;
      frame_data_size = 0;
      CleanFrameData();
      img->header.seq++;
      img->header.stamp = ros::Time::now();
      pub.publish(*img);
    }
    CleanChunkData();
  }

  bool getLatestImage(sensor_msgs::ImagePtr & img_in)
  {
    if(has_new_data)
    {
      *img_in = *img;
      has_new_data = false;
    }
  }

  boost::asio::io_service io_service;
  boost::thread * io_service_th;

  char * request_data;
  void CleanRequestData() { memset(request_data, 0, ros_h264_streamer_private::_request_size); }
  unsigned char * chunk_data;
  void CleanChunkData() { memset(chunk_data, 0, ros_h264_streamer_private::_video_chunk_size); }

  int frame_data_size;
  int full_frame_data_size;
  uint8_t * frame_data;
  void CleanFrameData() { memset(frame_data, 0, full_frame_data_size); }
  bool has_new_data;
  sensor_msgs::ImagePtr img;
  H264Decoder decoder;

  image_transport::ImageTransport it;
  image_transport::Publisher pub;
};

struct H264ReceiverUDPServer : public H264ReceiverNetImpl
{
  H264ReceiverUDPServer(H264Receiver::Config & conf, ros::NodeHandle & nh)
  : H264ReceiverNetImpl(conf, nh),
    socket(0), client_endpoint()
  {
    socket = new udp::socket(io_service);
    socket->open(udp::v4());
    socket->bind(udp::endpoint(udp::v4(), conf.port));
    socket->async_receive_from(
      boost::asio::buffer(chunk_data, ros_h264_streamer_private::_video_chunk_size), client_endpoint,
      boost::bind(&H264ReceiverUDPServer::handle_receive_from, this,
        boost::asio::placeholders::error,
        boost::asio::placeholders::bytes_transferred));
  }

  ~H264ReceiverUDPServer()
  {
    socket->close();
    delete socket;
  }

  void handle_receive_from(const boost::system::error_code & error, size_t bytes_recvd)
  {
    if(!error && bytes_recvd > 0)
    {
      HandleVideoChunk(bytes_recvd);
    }
    else if(error)
    {
      std::cerr << "[ros_h264_streamer] H264Receiver UDP server got the error while receiving data: " << std::endl << error.message() << std::endl;
    }
    socket->async_receive_from(
      boost::asio::buffer(chunk_data, ros_h264_streamer_private::_video_chunk_size), client_endpoint,
      boost::bind(&H264ReceiverUDPServer::handle_receive_from, this,
        boost::asio::placeholders::error,
        boost::asio::placeholders::bytes_transferred));
  }

private:
  udp::socket * socket;
  udp::endpoint client_endpoint;
};

struct H264ReceiverUDPClient : public H264ReceiverNetImpl
{
  H264ReceiverUDPClient(H264Receiver::Config & conf, ros::NodeHandle & nh)
  : H264ReceiverNetImpl(conf, nh),
    timeout_timer(io_service, boost::posix_time::seconds(1))
  {
    udp::resolver resolver(io_service);
    std::stringstream ss;
    ss << conf.port;
    udp::resolver::query query(udp::v4(), conf.host, ss.str());
    server_endpoint = *resolver.resolve(query);

    socket = new udp::socket(io_service);
    socket->open(udp::v4());
    SendVideoRequest();
  }

  void SendVideoRequest()
  {
    std::string request_ = "get";
    socket->async_send_to(
      boost::asio::buffer(request_.c_str(), request_.size() + 1),
      server_endpoint,
      boost::bind(&H264ReceiverUDPClient::handle_send_to, this,
        boost::asio::placeholders::error,
        boost::asio::placeholders::bytes_transferred));
  }

  void ReceiveData()
  {
    timeout_timer.cancel();
    socket->async_receive_from(
      boost::asio::buffer(chunk_data, ros_h264_streamer_private::_video_chunk_size), client_endpoint,
      boost::bind(&H264ReceiverUDPClient::handle_receive_from, this,
        boost::asio::placeholders::error,
        boost::asio::placeholders::bytes_transferred));
    timeout_timer.expires_from_now(boost::posix_time::seconds(1));
    timeout_timer.async_wait(boost::bind(&H264ReceiverUDPClient::handle_timeout, this, boost::asio::placeholders::error));
  }

  void handle_send_to(const boost::system::error_code & error, size_t bytes_send)
  {
    if(!error)
    {
      ReceiveData();
    }
    else
    {
      std::cerr << "[ros_h264_streamer] H264Receiver UDP client error when trying to send to server: " << std::endl << error.message() << std::endl;
      std::cerr << "[ros_h264_streamer] Retrying in one second" << std::endl;
      sleep(1);
      SendVideoRequest();
    }
  }

  void handle_receive_from(const boost::system::error_code & error, size_t bytes_recvd)
  {
    if(!error && bytes_recvd > 0)
    {
      HandleVideoChunk(bytes_recvd);
      ReceiveData();
    }
    else if(error)
    {
      std::cerr << "[ros_h264_streamer] H264Receiver UDP server got the error while receiving data: " << std::endl << error.message() << std::endl;
      SendVideoRequest();
    }
  }

  void handle_timeout(const boost::system::error_code & error)
  {
    if(error != boost::asio::error::operation_aborted)
    {
      SendVideoRequest();
    }
  }

private:
  udp::socket * socket;
  udp::endpoint server_endpoint;
  udp::endpoint client_endpoint;
  boost::asio::deadline_timer timeout_timer;
};

struct H264ReceiverTCPServer : public H264ReceiverNetImpl
{
  H264ReceiverTCPServer(H264Receiver::Config & conf, ros::NodeHandle & nh)
  : H264ReceiverNetImpl(conf, nh)
  {
  }
};

struct H264ReceiverTCPClient : public H264ReceiverNetImpl
{
  H264ReceiverTCPClient(H264Receiver::Config & conf, ros::NodeHandle & nh)
  : H264ReceiverNetImpl(conf, nh)
  {
  }
};

struct H264ReceiverImpl
{
public:
  H264ReceiverImpl(H264Receiver::Config & conf, ros::NodeHandle & nh)
  : net_impl(0)
  {
    if(conf.use_udp)
    {
      if(conf.is_server)
      {
        net_impl = new H264ReceiverUDPServer(conf, nh);
      }
      else
      {
        net_impl = new H264ReceiverUDPClient(conf, nh);
      }
    }
    else
    {
      if(conf.is_server)
      {
        net_impl = new H264ReceiverTCPServer(conf, nh);
      }
      else
      {
        net_impl = new H264ReceiverTCPClient(conf, nh);
      }
    }
    net_impl->StartIOService();
  }

  ~H264ReceiverImpl()
  {
    delete net_impl;
  }

  bool getLatestImage(sensor_msgs::ImagePtr & img)
  {
    return net_impl->getLatestImage(img);
  }
private:
  H264ReceiverNetImpl * net_impl;
};

H264Receiver::H264Receiver(H264Receiver::Config & conf, ros::NodeHandle & nh)
: impl(new H264ReceiverImpl(conf, nh))
{
}

bool H264Receiver::getLatestImage(sensor_msgs::ImagePtr & img)
{
  return impl->getLatestImage(img);
}

} // namespace ros_h264_Receiver
