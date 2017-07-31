/* 
NMEA2000_due.cpp

Copyright (c) 2015-2017 Timo Lappalainen, Kave Oy, www.kave.fi

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

#include "NMEA2000_due.h"


//*****************************************************************************
tNMEA2000_due::tNMEA2000_due() : tNMEA2000() {
#if defined(DUE_CAN_MAILBOX_TX_BUFFER_SUPPORT)
  NumTxMailBoxes=2;
#else
  NumTxMailBoxes=1;
#endif
  CANbus=&Can0;
}

//*****************************************************************************
bool tNMEA2000_due::CANSendFrame(unsigned long id, unsigned char len, const unsigned char *buf, bool  wait_sent ) {
  CAN_FRAME output;
  bool result;
  
    output.extended=true;
    output.id = id;
    output.length = len;
    for (int i=0; i<len && i<8; i++) output.data.bytes[i]=buf[i];
    
#if defined(DUE_CAN_MAILBOX_TX_BUFFER_SUPPORT)
  if ( wait_sent ) {
    result=Can0.sendFrame(output,Can0.getLastTxBox());
  } else {
    result=Can0.sendFrame(output);
  }
#else
    result=Can0.sendFrame(output);
#endif
    
    return result;
}

//*****************************************************************************
bool tNMEA2000_due::CANOpen() {
    Can0.begin(CAN_BPS_250K);
    Can0.setNumTXBoxes(NumTxMailBoxes); // Just one send box
    
#if defined(DUE_CAN_DYNAMIC_BUFFER_SUPPORT)
  if ( MaxCANReceiveFrames==0 ) MaxCANReceiveFrames=32; // Use default, if not set
  if ( MaxCANReceiveFrames<10 ) MaxCANReceiveFrames=10; // Do not allow less that 10 - DUE should have enough memory.
  Can0.setRxBufferSize(MaxCANReceiveFrames);
#if !defined(DUE_CAN_MAILBOX_TX_BUFFER_SUPPORT)
  if (MaxCANSendFrames<30 ) MaxCANSendFrames=30;
  Can0.setTxBufferSize(MaxCANSendFrames-4);
  MaxCANSendFrames=4;
#endif
#endif
  //By default there are 7 mailboxes for each device that are RX boxes
  //This sets each mailbox to have an open filter that will accept extended
  //or standard frames
  uint8_t mailbox;
  //extended
  for (mailbox = 0; mailbox < Can0.getNumRxBoxes()-1; mailbox++) {
  	Can0.setRXFilter(mailbox, 0, 0, true);
  }  
  //standard
  for (; mailbox < Can0.getNumRxBoxes(); mailbox++) {
  	Can0.setRXFilter(mailbox, 0, 0, false);
  }  
    return true;
}

//*****************************************************************************
bool tNMEA2000_due::CANGetFrame(unsigned long &id, unsigned char &len, unsigned char *buf) {
  bool HasFrame=false;
  CAN_FRAME incoming;

    if ( Can0.rx_avail() ) {           // check if data coming
        Can0.read(incoming); 
        id=incoming.id;
        len=incoming.length;
        for (int i=0; i<len && i<8; i++) buf[i]=incoming.data.bytes[i];
        HasFrame=true;
    }
    
    return HasFrame;
}

#if defined(DUE_CAN_MAILBOX_TX_BUFFER_SUPPORT)
//*****************************************************************************
void tNMEA2000_due::InitCANFrameBuffers() {

  if (MaxCANSendFrames<30 ) MaxCANSendFrames=30;
  
  uint16_t TotalFrames=MaxCANSendFrames;
  MaxCANSendFrames=4; // we do not need libary internal buffer since driver has them.
  uint16_t CANGlobalBufSize=TotalFrames-MaxCANSendFrames;

// With this support system can have different buffers for single and fast packet messages.
// After message has been sent to driver, it buffers it automatically and sends it by interrupt.
// We may need to make these possible to set.
//  uint16_t HighPriorityBufferSize=CANGlobalBufSize / 10;
//  HighPriorityBufferSize=(HighPriorityBufferSize<15?HighPriorityBufferSize:15); // just guessing
//  CANGlobalBufSize-=HighPriorityBufferSize;
  uint16_t FastPacketBufferSize= (CANGlobalBufSize * 9 / 10);
  CANGlobalBufSize-=FastPacketBufferSize;

//  Can0.setMailBoxTxBufferSize(CANbus->getFirstTxBox(),HighPriorityBufferSize); // Highest priority buffer
  Can0.setMailBoxTxBufferSize(Can0.getLastTxBox(),FastPacketBufferSize); // Fastpacket buffer
  Can0.setTxBufferSize(CANGlobalBufSize);
  
  tNMEA2000::InitCANFrameBuffers(); // call main initialization
}
#endif

