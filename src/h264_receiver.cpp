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

enum H264ReceiverProtocol
{
  ChunkIDPlusData = 1,
  FrameSizePlusData
};

struct H264ReceiverNetImpl
{
  H264ReceiverNetImpl(H264Receiver::Config & conf, ros::NodeHandle & nh)
  : io_service(), io_service_th(0), request_data(0), protocol(ChunkIDPlusData), video_chunk_size(0), chunk_data(0),
    frame_data_size(0), full_frame_data_size(0), frame_data(0),
    has_new_data(false), img(new sensor_msgs::Image),
    decoder(conf.width, conf.height),
    publish(conf.publish), it(nh), pub()
  {
  }

  void InitBuffers(H264Receiver::Config & conf)
  {
    SetVideoChunkSize();
    request_data = new char[ros_h264_streamer_private::_request_size];
    chunk_data = new unsigned char[video_chunk_size];
    frame_data_size = 0;
    frame_data = new uint8_t[full_frame_data_size];
    CleanRequestData();
    CleanChunkData();
    CleanFrameData();

    if(publish)
    {
      img->header.seq = 0;
      img->header.frame_id = conf.frame_id;
      pub = it.advertise(conf.publish_topic, 1);
    }
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

  virtual void SetVideoChunkSize() = 0;

  virtual void ReceiveMissingData(size_t bytes_recvd, size_t missing_data_size) {}

  void StartIOService()
  {
    io_service_th = new boost::thread(boost::bind(&boost::asio::io_service::run, &io_service));
  }

  void ResizeFrameData()
  {
    std::cerr << "[ros_h264_streamer] H264Receiver needs to re-allocate frame data" << std::endl;
    uint8_t * old_frame_data = frame_data;
    uint8_t * new_frame_data = new uint8_t[2*frame_data_size];
    memcpy(new_frame_data, frame_data, full_frame_data_size);
    full_frame_data_size = 2*frame_data_size;
    frame_data = new_frame_data;
    delete[] old_frame_data;
  }

  void HandleVideoChunk(size_t bytes_recvd)
  {
    bool decode_frame_data = false;
    if(protocol == ChunkIDPlusData)
    {
      uint8_t chunkID = chunk_data[0];
      frame_data_size += bytes_recvd;
      if(frame_data_size > full_frame_data_size)
      {
        ResizeFrameData();
      }
      if(chunkID*(video_chunk_size-1) + bytes_recvd - 1 < full_frame_data_size)
      {
        memcpy(&frame_data[chunkID*(video_chunk_size-1)], &chunk_data[1], bytes_recvd - 1);
        if(bytes_recvd < video_chunk_size)
        {
          decode_frame_data = true;
        }
      }
      else
      {
          frame_data_size = 0;
          CleanFrameData();
      }
      CleanChunkData();
    }
    else
    {
      memcpy(&frame_data_size, chunk_data, sizeof(int));
      memcpy(frame_data, &chunk_data[sizeof(int)], bytes_recvd - sizeof(int));
      CleanChunkData();
      if(frame_data_size < 0 || bytes_recvd > frame_data_size + sizeof(int) || frame_data_size > full_frame_data_size)
      {
        CleanFrameData();
        return;
      }
      if(bytes_recvd != frame_data_size + sizeof(int))
      {
        ReceiveMissingData(bytes_recvd, frame_data_size + sizeof(int) - bytes_recvd);
      }
      decode_frame_data = true;
    }
    if(decode_frame_data)
    {
      int data_decoded = decoder.decode(frame_data_size, frame_data, img);
      if(data_decoded > 0 && publish)
      {
        has_new_data = true;
        img->header.seq++;
        img->header.stamp = ros::Time::now();
        pub.publish(*img);
      }
      CleanFrameData();
    }
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
  H264ReceiverProtocol protocol;
  int video_chunk_size;
  void CleanChunkData() { memset(chunk_data, 0, video_chunk_size); }

  int frame_data_size;
  int full_frame_data_size;
  uint8_t * frame_data;
  void CleanFrameData() { memset(frame_data, 0, full_frame_data_size); }
  bool has_new_data;
  sensor_msgs::ImagePtr img;
  H264Decoder decoder;

  bool publish;
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
      boost::asio::buffer(chunk_data, video_chunk_size), client_endpoint,
      boost::bind(&H264ReceiverUDPServer::handle_receive_from, this,
        boost::asio::placeholders::error,
        boost::asio::placeholders::bytes_transferred));
  }

  ~H264ReceiverUDPServer()
  {
    socket->close();
    delete socket;
  }

  void SetVideoChunkSize()
  {
    video_chunk_size = ros_h264_streamer_private::_udp_video_chunk_size;
    full_frame_data_size = 3*video_chunk_size;
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
      boost::asio::buffer(chunk_data, video_chunk_size), client_endpoint,
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

  void SetVideoChunkSize()
  {
    video_chunk_size = ros_h264_streamer_private::_udp_video_chunk_size;
    full_frame_data_size = 3*video_chunk_size;
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
      boost::asio::buffer(chunk_data, video_chunk_size), client_endpoint,
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
  : H264ReceiverNetImpl(conf, nh),
    socket(0), acceptor(io_service, tcp::endpoint(tcp::v4(), conf.port))
  {
    AcceptConnection();
  }

  void SetVideoChunkSize()
  {
    video_chunk_size = ros_h264_streamer_private::_tcp_video_chunk_size;
    full_frame_data_size = 90*video_chunk_size;
  }

  ~H264ReceiverTCPServer()
  {
    acceptor.close();
    if(socket)
    {
      socket->close();
    }
    delete socket;
  }

  void ReceiveData()
  {
    if(socket)
    {
      socket->async_receive(
        boost::asio::buffer(chunk_data, video_chunk_size),
        boost::bind(&H264ReceiverTCPServer::handle_receive, this,
          boost::asio::placeholders::error,
          boost::asio::placeholders::bytes_transferred));
    }
  }

  void AcceptConnection()
  {
    tcp::socket * nsocket = new tcp::socket(io_service);
    acceptor.async_accept(*nsocket,
      boost::bind(&H264ReceiverTCPServer::handle_accept, this, nsocket, boost::asio::placeholders::error));
  }

  void handle_accept(tcp::socket * socket_in, const boost::system::error_code& error)
  {
    delete socket;
    if(!error)
    {
      socket = socket_in;
      ReceiveData();
    }
    AcceptConnection();
  }

  void ReceiveMissingData(size_t bytes_recvd, size_t missing_data_size)
  {
    if(socket)
    {
      size_t data_left = missing_data_size;
      while(data_left > 0)
      {
        int nbytes_recvd = socket->read_some(boost::asio::buffer(chunk_data, std::min((size_t)video_chunk_size, data_left)));
        memcpy(&frame_data[bytes_recvd-sizeof(int)], chunk_data, nbytes_recvd);
        CleanChunkData();
        data_left -= nbytes_recvd;
        bytes_recvd += nbytes_recvd;
      }
    }
  }

  void handle_receive(const boost::system::error_code & error, size_t bytes_recvd)
  {
    if(!error && bytes_recvd > 0)
    {
      HandleVideoChunk(bytes_recvd);
      ReceiveData();
    }
    else if(error)
    {
      std::cerr << "[ros_h264_streamer] H264Receiver TCP server got the error while receiving data: " << std::endl << error.message() << std::endl;
    }
  }

private:
  tcp::socket * socket;
  tcp::acceptor acceptor;
};

struct H264ReceiverTCPClient : public H264ReceiverNetImpl
{
  H264ReceiverTCPClient(H264Receiver::Config & conf, ros::NodeHandle & nh)
  : H264ReceiverNetImpl(conf, nh), socket(0)
  {
    tcp::resolver resolver(io_service);
    std::stringstream ss;
    ss << conf.port;
    tcp::resolver::query query(tcp::v4(), conf.host, ss.str());
    server_endpoint = *resolver.resolve(query);

    ConnectToServer();
  }

  void SetVideoChunkSize()
  {
    video_chunk_size = ros_h264_streamer_private::_tcp_video_chunk_size;
    full_frame_data_size = 90*video_chunk_size;
  }

  void ConnectToServer()
  {
    delete socket;
    socket = new tcp::socket(io_service);
    socket->async_connect(server_endpoint,
      boost::bind(&H264ReceiverTCPClient::handle_connect, this,
        boost::asio::placeholders::error));
  }

  void ReceiveData()
  {
    socket->async_receive(
      boost::asio::buffer(chunk_data, video_chunk_size), 0,
      boost::bind(&H264ReceiverTCPClient::handle_receive, this,
        boost::asio::placeholders::error,
        boost::asio::placeholders::bytes_transferred));
  }

  void ReceiveMissingData(size_t bytes_recvd, size_t missing_data_size)
  {
    if(socket)
    {
      size_t data_left = missing_data_size;
      while(data_left > 0)
      {
        int nbytes_recvd = socket->read_some(boost::asio::buffer(chunk_data, std::min((size_t)video_chunk_size, data_left)));
        memcpy(&frame_data[bytes_recvd-sizeof(int)], chunk_data, nbytes_recvd);
        CleanChunkData();
        data_left -= nbytes_recvd;
        bytes_recvd += nbytes_recvd;
      }
    }
  }

  void handle_connect(const boost::system::error_code & error)
  {
    if(!error)
    {
      ReceiveData();
    }
    else
    {
      sleep(1);
      ConnectToServer();
    }
  }

  void handle_receive(const boost::system::error_code & error, size_t bytes_recvd)
  {
    if(!error && bytes_recvd > 0)
    {
      HandleVideoChunk(bytes_recvd);
      ReceiveData();
    }
    else if(error)
    {
      std::cerr << "[ros_h264_streamer] H264Receiver TCP client got the error while receiving data: " << std::endl << error.message() << std::endl;
      ConnectToServer();
    }
  }

private:
  tcp::socket * socket;
  tcp::endpoint server_endpoint;
};

struct H264ReceiverImpl
{
public:
  H264ReceiverImpl(H264Receiver::Config & conf, ros::NodeHandle & nh)
  : net_impl(0)
  {
    if(conf.udp)
    {
      if(conf.server)
      {
        net_impl = new H264ReceiverUDPServer(conf, nh);
      }
      else
      {
        net_impl = new H264ReceiverUDPClient(conf, nh);
      }
      net_impl->protocol = ChunkIDPlusData;
    }
    else
    {
      if(conf.server)
      {
        net_impl = new H264ReceiverTCPServer(conf, nh);
      }
      else
      {
        net_impl = new H264ReceiverTCPClient(conf, nh);
      }
      net_impl->protocol = FrameSizePlusData;
    }
  }

  ~H264ReceiverImpl()
  {
    delete net_impl;
  }

  void Init(H264Receiver::Config & conf)
  {
    net_impl->InitBuffers(conf);
    net_impl->StartIOService();
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
  impl->Init(conf);
}

bool H264Receiver::getLatestImage(sensor_msgs::ImagePtr & img)
{
  return impl->getLatestImage(img);
}

} // namespace ros_h264_Receiver
