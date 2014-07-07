#ifndef _H_H264_API_H_
#define _H_H264_API_H_
  #ifdef WIN32
    #define H264_API __declspec(dllexport)
  #else
    #define H264_API
  #endif
#endif