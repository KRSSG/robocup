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
\file    main.cpp
\brief   Main Entry point for the graphicalClient binary`
\author  Joydeep Biswas (C) 2011
*/
//========================================================================

#include <stdio.h>
#include <QtGui>
#include <QApplication>
#include "soccerview.h"
#include "timer.h"
#include "robocup_ssl_debug_client.h"
GLSoccerView *view;

bool runApp = true;

class MyThread : public QThread
{  
protected:
  void run()
  {
    static const double minDuration = 0.01; //100FPS
    RoboCupSSLClient client;
    RoboCupSSLDebugClient dclient;
    dclient.open(false);
    client.open(false);
    sslDebug_Data dpacket;
    SSL_WrapperPacket packet;
    while(runApp) {
      while (client.receive(packet)) {
        if (packet.has_detection()) {
          SSL_DetectionFrame detection = packet.detection();
          view->updateDetection(detection);
        }
        if (packet.has_geometry()) {
          view->updateFieldGeometry(packet.geometry().field());
        }
      }
      while (dclient.receive(dpacket)) {
        printf("got a dpacket!\n");
        view->updateDebugData(dpacket);
      }
      Sleep(minDuration);
    }
  }
  
public:
  MyThread(QObject* parent = 0){}
  ~MyThread(){}
};

int main(int argc, char **argv)
{
  QApplication app(argc, argv);  
  view = new GLSoccerView();
  view->show();
  MyThread thread;
  thread.start();
  int retVal = app.exec();
  runApp = false;
  thread.wait();
  return retVal;
}

