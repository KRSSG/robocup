//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
  \file    robocup_ssl_server.cpp
  \brief   C++ Implementation: robocup_ssl_server
  \author  Stefan Zickler, 2009
*/
//========================================================================
#include "robocup_ssl_server.h"

RoboCupSSLServer::RoboCupSSLServer(int port,
                     string net_address,
                     string net_interface)
{
  _port=port;
  _net_address=net_address;
  _net_interface=net_interface;

}


RoboCupSSLServer::~RoboCupSSLServer()
{
}

void RoboCupSSLServer::close() {
  mc.close();
}

bool RoboCupSSLServer::open() {
  close();
  if(!mc.open(_port,true,true)) {
    fprintf(stderr,"Unable to open UDP network port: %d\n",_port);
    fflush(stderr);
    return(false);
  }

  Net::Address multiaddr,interface;
  multiaddr.setHost(_net_address.c_str(),_port);
  if(_net_interface.length() > 0){
    interface.setHost(_net_interface.c_str(),_port);
  }else{
    interface.setAny();
  }

  if(!mc.addMulticast(multiaddr,interface)) {
    fprintf(stderr,"Unable to setup UDP multicast\n");
    fflush(stderr);
    return(false);
  }
  return(true);
}

bool RoboCupSSLServer::send(const SSL_DetectionFrame & frame) {
  SSL_WrapperPacket pkt;
  SSL_DetectionFrame * nframe = pkt.mutable_detection();
  nframe->CopyFrom(frame);
  return sendWrapperPacket<SSL_WrapperPacket>(pkt);
}

bool RoboCupSSLServer::send(const SSL_GeometryData & geometry) {
  SSL_WrapperPacket pkt;
  SSL_GeometryData * gdata = pkt.mutable_geometry();
  gdata->CopyFrom(geometry);
  return sendWrapperPacket<SSL_WrapperPacket>(pkt);
}

bool RoboCupSSLServer::sendLegacyMessage(const SSL_DetectionFrame& frame) {
  RoboCup2014Legacy::Wrapper::SSL_WrapperPacket pkt;
  SSL_DetectionFrame * nframe = pkt.mutable_detection();
  nframe->CopyFrom(frame);
  return sendWrapperPacket<RoboCup2014Legacy::Wrapper::SSL_WrapperPacket>(pkt);
}

bool RoboCupSSLServer::sendLegacyMessage(
    const RoboCup2014Legacy::Geometry::SSL_GeometryData& geometry) {
  RoboCup2014Legacy::Wrapper::SSL_WrapperPacket pkt;
  RoboCup2014Legacy::Geometry::SSL_GeometryData * gdata = pkt.mutable_geometry();
  gdata->CopyFrom(geometry);
  return sendWrapperPacket<RoboCup2014Legacy::Wrapper::SSL_WrapperPacket>(pkt);
}
