#include <ros_h264_streamer/h264_decoder.h>

#include <sensor_msgs/image_encodings.h>

#include <stdint.h>
#ifndef UINT64_C
typedef uint64_t UINT64_C;
#endif

extern "C"
{
#include "libswscale/swscale.h"
#include "libavutil/imgutils.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "libavcodec/avcodec.h"
#include "libavutil/mathematics.h"
#include "libavutil/samplefmt.h"
}

namespace ros_h264_streamer
{

struct H264DecoderImpl
{
public:
  H264DecoderImpl(int width, int height)
  : m_width(width), m_height(height)
  {
    m_stride = 3*m_width;

    /* For YUV420P -> BGR8 conversion */
    m_convert_ctx = sws_getContext(m_width, m_height, PIX_FMT_YUV420P, m_width, m_height, PIX_FMT_BGR24, SWS_FAST_BILINEAR, NULL, NULL, NULL);

    av_init_packet(&avpkt);

    /* Find H264 codec */
    avcodec_register_all();
    codec = avcodec_find_decoder(CODEC_ID_H264);
    if(!codec)
    {
        std::cerr << "[ros_h264_streamer] H264Decoder cannot find H.264 codec in ffmpeg" << std::endl;
        throw(std::string("H264 codec not found, check that your ffmpeg is built with H264 support"));
    }

    c = avcodec_alloc_context3(codec);
    picture = avcodec_alloc_frame();
    c->width  = m_width;
    c->height = m_height;

    if(avcodec_open2(c, codec, 0) < 0)
    {
        std::cerr << "[ros_h264_streamer] H264Decoder cannot open codec" << std::endl;
        throw(std::string("Cannot open codec"));
    }
  }

  ~H264DecoderImpl()
  {
    avcodec_close(c);
    av_free(c);
    av_free(picture);
  }

  int decode(int frame_size, uint8_t * frame_data, sensor_msgs::ImagePtr & out)
  {
    avpkt.size = frame_size;
    avpkt.data = frame_data;
    int got_picture;
    int len = avcodec_decode_video2(c, picture, &got_picture, &avpkt);
    if(len < 0)
    {
        return len;
    }
    out->data.resize(m_width*m_height*3);
    uint8_t *buf_out[4]={&(out->data[0]),NULL,NULL,NULL};
    sws_scale(m_convert_ctx, (const uint8_t* const*)picture->data, picture->linesize, 0, m_height, buf_out, &m_stride);

    out->width = m_width;
    out->height = m_height;
    out->step = m_stride;
    out->encoding = sensor_msgs::image_encodings::BGR8;

    return len;
  }
private:
  int m_width;
  int m_height;
  int m_stride;

  struct SwsContext * m_convert_ctx;

  AVCodec * codec;
  AVCodecContext * c;
  AVFrame *picture;
  AVPacket avpkt;
};

H264Decoder::H264Decoder(int width, int height)
: impl(new H264DecoderImpl(width, height))
{
}

int H264Decoder::decode(int frame_size, uint8_t * frame_data, sensor_msgs::ImagePtr & out)
{
  impl->decode(frame_size, frame_data, out);
}

} // namespace ros_h264_streamer
