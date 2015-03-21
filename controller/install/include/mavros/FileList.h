/* Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of Willow Garage, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Auto-generated by gensrv_cpp from file /home/lintang-sutawika/krai/trui-bot-prj/controller/src/mavros/srv/FileList.srv
 *
 */


#ifndef MAVROS_MESSAGE_FILELIST_H
#define MAVROS_MESSAGE_FILELIST_H

#include <ros/service_traits.h>


#include <mavros/FileListRequest.h>
#include <mavros/FileListResponse.h>


namespace mavros
{

struct FileList
{

typedef FileListRequest Request;
typedef FileListResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct FileList
} // namespace mavros


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::mavros::FileList > {
  static const char* value()
  {
    return "1647c627907cac72fa2d06e0e5f96aac";
  }

  static const char* value(const ::mavros::FileList&) { return value(); }
};

template<>
struct DataType< ::mavros::FileList > {
  static const char* value()
  {
    return "mavros/FileList";
  }

  static const char* value(const ::mavros::FileList&) { return value(); }
};


// service_traits::MD5Sum< ::mavros::FileListRequest> should match 
// service_traits::MD5Sum< ::mavros::FileList > 
template<>
struct MD5Sum< ::mavros::FileListRequest>
{
  static const char* value()
  {
    return MD5Sum< ::mavros::FileList >::value();
  }
  static const char* value(const ::mavros::FileListRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::mavros::FileListRequest> should match 
// service_traits::DataType< ::mavros::FileList > 
template<>
struct DataType< ::mavros::FileListRequest>
{
  static const char* value()
  {
    return DataType< ::mavros::FileList >::value();
  }
  static const char* value(const ::mavros::FileListRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::mavros::FileListResponse> should match 
// service_traits::MD5Sum< ::mavros::FileList > 
template<>
struct MD5Sum< ::mavros::FileListResponse>
{
  static const char* value()
  {
    return MD5Sum< ::mavros::FileList >::value();
  }
  static const char* value(const ::mavros::FileListResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::mavros::FileListResponse> should match 
// service_traits::DataType< ::mavros::FileList > 
template<>
struct DataType< ::mavros::FileListResponse>
{
  static const char* value()
  {
    return DataType< ::mavros::FileList >::value();
  }
  static const char* value(const ::mavros::FileListResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // MAVROS_MESSAGE_FILELIST_H