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

enum H264StreamerProtocol
{
  ChunkIDPlusData = 1,
  FrameSizePlusData
};

struct H264StreamerNetImpl
{
  H264StreamerNetImpl()
  : io_service(), io_service_th(0), request_data(0), chunk_data_size(0), chunk_data(0), protocol(ChunkIDPlusData)
  {
  }

  void InitBuffers()
  {
    SetChunkDataSize();
    request_data = new char[ros_h264_streamer_private::_request_size];
    chunk_data = new unsigned char[chunk_data_size];
    CleanRequestData();
    CleanChunkData();
  }

  ~H264StreamerNetImpl()
  {
    io_service.stop();
    if(io_service_th)
    {
      io_service_th->join();
      delete io_service_th;
    }
    delete[] request_data;
    delete[] chunk_data;
  }

  virtual void SetChunkDataSize() = 0;

  void IOServiceThread()
  {
    while(ros::ok())
    {
      io_service.run();
      io_service.reset();
    }
  }

  void StartIOService()
  {
    io_service_th = new boost::thread(boost::bind(&H264StreamerNetImpl::IOServiceThread, this));
  }

  void HandleNewData(H264EncoderResult & res)
  {
    if(protocol == ChunkIDPlusData)
    {
      int data_size = 0;
      uint8_t chunkID = 0;
      do
      {
        CleanChunkData();
        data_size = std::min(res.frame_size, chunk_data_size - 1 );
        chunk_data[0] = chunkID;
        std::memcpy(&chunk_data[1], &res.frame_data[chunkID*(chunk_data_size - 1)], data_size);
        SendData(data_size + 1);
        chunkID++;
        res.frame_size -= data_size;
      }
      while(res.frame_size > 0);
    }
    else
    {
      if(chunk_data_size < res.frame_size + sizeof(unsigned int))
      {
        delete[] chunk_data;
        chunk_data_size = 2*res.frame_size + sizeof(int);
        chunk_data = new uint8_t[chunk_data_size];
      }
      CleanChunkData();
      std::memcpy(chunk_data, &res.frame_size, sizeof(int));
      std::memcpy(&chunk_data[sizeof(int)], res.frame_data, res.frame_size);
      SendData(res.frame_size + sizeof(int));
    }
  }

  virtual void SendData(int frame_size) = 0;

  boost::asio::io_service io_service;
  boost::thread * io_service_th;

  char * request_data;
  void CleanRequestData() { memset(request_data, 0, ros_h264_streamer_private::_request_size); }
  int chunk_data_size;
  unsigned char * chunk_data;
  void CleanChunkData() { memset(chunk_data, 0, chunk_data_size); }

  H264StreamerProtocol protocol;
};

struct H264StreamerUDPServer : public H264StreamerNetImpl
{
  H264StreamerUDPServer(short port)
  : socket(0), has_client(false), client_endpoint()
  {
    socket = new udp::socket(io_service);
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

  void SetChunkDataSize()
  {
    chunk_data_size = ros_h264_streamer_private::_udp_video_chunk_size;
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
    udp::resolver resolver(io_service);
    std::stringstream ss;
    ss << port;
    udp::resolver::query query(udp::v4(), host, ss.str());
    server_endpoint = *resolver.resolve(query);

    socket = new udp::socket(io_service);
    socket->open(udp::v4());
  }

  void SetChunkDataSize()
  {
    chunk_data_size = ros_h264_streamer_private::_udp_video_chunk_size;
  }

  void SendData(int frame_size)
  {
    boost::system::error_code error;
    socket->send_to(
      boost::asio::buffer(chunk_data, frame_size),
      server_endpoint, 0, error);
    if(error)
    {
      std::cerr << "[ros_h264_streamer] H264Streamer UDP client got the error while sending data: " << std::endl << error.message() << std::endl;
    }
  }
private:
  udp::socket * socket;
  udp::endpoint server_endpoint;
};

struct H264StreamerTCPServer : public H264StreamerNetImpl
{
  H264StreamerTCPServer(short port)
  : acceptor(io_service, tcp::endpoint(tcp::v4(), port)),
    socket(0)
  {
    AcceptConnection();
  }

  ~H264StreamerTCPServer()
  {
    acceptor.close();
    if(socket)
    {
      socket->close();
    }
    delete socket;
  }

  void SetChunkDataSize()
  {
    chunk_data_size = ros_h264_streamer_private::_tcp_video_chunk_size;
  }

  void SendData(int frame_size)
  {
    if(socket)
    {
      socket->async_send(
        boost::asio::buffer(chunk_data, frame_size),
        boost::bind(&H264StreamerTCPServer::handle_send, this,
          boost::asio::placeholders::error,
          boost::asio::placeholders::bytes_transferred));
    }
  }

  void AcceptConnection()
  {
    tcp::socket * nsocket = new tcp::socket(io_service);
    acceptor.async_accept(*nsocket,
      boost::bind(&H264StreamerTCPServer::handle_accept, this, nsocket, boost::asio::placeholders::error));
  }

  void handle_accept(tcp::socket * socket_in, const boost::system::error_code& error)
  {
    delete socket;
    if(!error)
    {
      socket = socket_in;
    }
    AcceptConnection();
  }

  void handle_send(const boost::system::error_code & error, size_t bytes_send)
  {
    if(error)
    {
      socket->close();
      delete socket;
      socket = 0;
    }
  }
private:
  tcp::acceptor acceptor;
  tcp::socket * socket;
};

struct H264StreamerTCPClient : public H264StreamerNetImpl
{
  H264StreamerTCPClient(const std::string & host, short port)
  : socket(0), server_endpoint(), ready(false)
  {
    tcp::resolver resolver(io_service);
    std::stringstream ss;
    ss << port;
    tcp::resolver::query query(tcp::v4(), host, ss.str());
    server_endpoint = *resolver.resolve(query);

    ConnectToServer();
  }

  void SetChunkDataSize()
  {
    chunk_data_size = ros_h264_streamer_private::_tcp_video_chunk_size;
  }

  void ConnectToServer()
  {
    delete socket;
    socket = new tcp::socket(io_service);
    socket->async_connect(server_endpoint,
      boost::bind(&H264StreamerTCPClient::handle_connect, this,
        boost::asio::placeholders::error));
  }

  void SendData(int frame_size)
  {
    if(ready)
    {
      socket->async_send(
        boost::asio::buffer(chunk_data, frame_size), 0,
        boost::bind(&H264StreamerTCPClient::handle_send, this,
          boost::asio::placeholders::error,
          boost::asio::placeholders::bytes_transferred));
    }
  }

  void handle_connect(const boost::system::error_code & error)
  {
    if(!error)
    {
      ready = true;
    }
    else
    {
      ready = false;
      sleep(1);
      ConnectToServer();
    }
  }

  void handle_send(const boost::system::error_code & error, size_t bytes_send)
  {
    if(error)
    {
      ready = false;
      std::cerr << "[ros_h264_streamer] H264Streamer TCP client got error when sending: " << std::endl << error.message() << std::endl;
      std::cerr << "[ros_h264_streamer] Trying to reconnect" << std::endl;
      ConnectToServer();
    }
  }
private:
  tcp::socket * socket;
  tcp::endpoint server_endpoint;
  bool ready;
};

struct H264StreamerImpl
{
public:
  H264StreamerImpl(H264Streamer::Config & conf, ros::NodeHandle & nh)
  : nh(nh), it(nh), conf(conf), net_impl(0), encoder(0)
  {
    if(conf.udp)
    {
      if(conf.server)
      {
        net_impl = new H264StreamerUDPServer(conf.port);
      }
      else
      {
        net_impl = new H264StreamerUDPClient(conf.host, conf.port);
      }
      net_impl->protocol = ChunkIDPlusData;
    }
    else
    {
      if(conf.server)
      {
        net_impl = new H264StreamerTCPServer(conf.port);
      }
      else
      {
        net_impl = new H264StreamerTCPClient(conf.host, conf.port);
      }
      net_impl->protocol = FrameSizePlusData;
    }
  }

  void Init()
  {
    net_impl->InitBuffers();
    net_impl->StartIOService();
    sub = it.subscribe(conf.camera_topic, 1, &H264StreamerImpl::imageCallback, this);
  }

  ~H264StreamerImpl()
  {
    sub.shutdown();
    delete encoder;
    delete net_impl;
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
  impl->Init();
}

} // namespace ros_h264_streamer
