/* 
NMEA2000_due.cpp

Copyright (c) 2015-2016 Timo Lappalainen, Kave Oy, www.kave.fi

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to use,
copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
  
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
