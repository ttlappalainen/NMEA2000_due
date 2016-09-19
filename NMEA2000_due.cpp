/* 
NMEA2000_due.cpp

2015 Copyright (c) Kave Oy, www.kave.fi  All right reserved.

Author: Timo Lappalainen

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-
  1301  USA
  
Inherited NMEA2000 object for Arduino Due internal CAN
based setup. See also NMEA2000 library.
*/

#include <NMEA2000_due.h> 
#include <due_can.h>  // https://github.com/collin80/due_can


//*****************************************************************************
tNMEA2000_due::tNMEA2000_due() : tNMEA2000() {
}

//*****************************************************************************
bool tNMEA2000_due::CANSendFrame(unsigned long id, unsigned char len, const unsigned char *buf, bool /* wait_sent */) {
  CAN_FRAME output;
  bool result;
//  int TryCount=0;
  
    output.extended=true;
    output.id = id;
    output.length = len;
    for (int i=0; i<len && i<8; i++) output.data.bytes[i]=buf[i];
    
    result=CAN.sendFrame(output);
    
    return result;
}

//*****************************************************************************
bool tNMEA2000_due::CANOpen() {
    Can0.begin(CAN_BPS_250K);
  //By default there are 7 mailboxes for each device that are RX boxes
  //This sets each mailbox to have an open filter that will accept extended
  //or standard frames
  int filter;
  //extended
  for (filter = 0; filter < 3; filter++) {
  	Can0.setRXFilter(filter, 0, 0, true);
  }  
  //standard
  for (int filter = 3; filter < 7; filter++) {
  	Can0.setRXFilter(filter, 0, 0, false);
  }  
    return true;
}

//*****************************************************************************
bool tNMEA2000_due::CANGetFrame(unsigned long &id, unsigned char &len, unsigned char *buf) {
  bool HasFrame=false;
  CAN_FRAME incoming;

    if ( Can0.available() > 0 ) {           // check if data coming
        Can0.read(incoming); 
        id=incoming.id;
        len=incoming.length;
        for (int i=0; i<len && i<8; i++) buf[i]=incoming.data.bytes[i];
        HasFrame=true;
    }
    
    return HasFrame;
}
