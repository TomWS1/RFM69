// **********************************************************************************
// Driver definition for HopeRF RFM69W/RFM69HW/RFM69CW/RFM69HCW, Semtech SX1231/1231H
// **********************************************************************************
// Copyright Felix Rusu (2014), felix@lowpowerlab.com
// http://lowpowerlab.com/
// **********************************************************************************
// License
// **********************************************************************************
// This program is free software; you can redistribute it 
// and/or modify it under the terms of the GNU General    
// Public License as published by the Free Software       
// Foundation; either version 3 of the License, or        
// (at your option) any later version.                    
//                                                        
// This program is distributed in the hope that it will   
// be useful, but WITHOUT ANY WARRANTY; without even the  
// implied warranty of MERCHANTABILITY or FITNESS FOR A   
// PARTICULAR PURPOSE. See the GNU General Public        
// License for more details.                              
//                                                        
// You should have received a copy of the GNU General    
// Public License along with this program.
// If not, see <http://www.gnu.org/licenses/>.
//                                                        
// Licence can be viewed at                               
// http://www.gnu.org/licenses/gpl-3.0.txt
//
// Please maintain this license information along with authorship
// and copyright notices in any redistribution of this code
// **********************************************************************************
#ifndef RFM69_h
#define RFM69_h
#include <Arduino.h>            // assumes Arduino IDE v1.0 or greater
#include <SPI.h>

#define RF69_MAX_DATA_LEN       61 // to take advantage of the built in AES/CRC we want to limit the frame size to the internal FIFO size (66 bytes - 3 bytes overhead - 2 bytes crc)
#define RF69_SPI_CS             SS // SS is the SPI slave select pin, for instance D10 on ATmega328

// INT0 on AVRs should be connected to RFM69's DIO0 (ex on ATmega328 it's D2, on ATmega644/1284 it's D2)
#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__) || defined(__AVR_ATmega88) || defined(__AVR_ATmega8__) || defined(__AVR_ATmega88__)
  #define RF69_IRQ_PIN          2
  #define RF69_IRQ_NUM          0
#elif defined(__AVR_ATmega644P__) || defined(__AVR_ATmega1284P__)
  #define RF69_IRQ_PIN          2
  #define RF69_IRQ_NUM          2
#elif defined(__AVR_ATmega32U4__)
  #define RF69_IRQ_PIN          3
  #define RF69_IRQ_NUM          0
#endif


#define CSMA_LIMIT              -90 // upper RX signal sensitivity threshold in dBm for carrier sense access
#define RF69_MODE_SLEEP         0 // XTAL OFF
#define RF69_MODE_STANDBY       1 // XTAL ON
#define RF69_MODE_SYNTH         2 // PLL ON
#define RF69_MODE_RX            3 // RX MODE
#define RF69_MODE_TX            4 // TX MODE

// available frequency bands
#define RF69_315MHZ            31 // non trivial values to avoid misconfiguration
#define RF69_433MHZ            43
#define RF69_868MHZ            86
#define RF69_915MHZ            91

#define null                  0
#define COURSE_TEMP_COEF    -90 // puts the temperature reading in the ballpark, user can fine tune the returned value
#define RF69_BROADCAST_ADDR 255
#define RF69_CSMA_LIMIT_MS 1000
#define RF69_TX_LIMIT_MS   1000
#define RF69_FSTEP  61.03515625 // == FXOSC / 2^19 = 32MHz / 2^19 (p13 in datasheet)

class RFM69 {
  public:
    static volatile uint8_t DATA[RF69_MAX_DATA_LEN]; // recv/xmit buf, including header & crc bytes
    static volatile uint8_t DATALEN;
    static volatile uint8_t SENDERID;
    static volatile uint8_t TARGETID; // should match _address
    static volatile uint8_t PAYLOADLEN;
    static volatile uint8_t ACK_REQUESTED;
    static volatile uint8_t ACK_RECEIVED; // should be polled immediately after sending a packet with ACK request
    static volatile int16_t RSSI; // most accurate RSSI during reception (closest to the reception)
    static volatile uint8_t _mode; // should be protected?
    static volatile byte ACK_RSSI_REQUESTED;  // new flag in CTL byte to request RSSI with ACK (could potentially be merged with ACK_REQUESTED)

    RFM69(uint8_t slaveSelectPin=RF69_SPI_CS, uint8_t interruptPin=RF69_IRQ_PIN, bool isRFM69HW=false, uint8_t interruptNum=RF69_IRQ_NUM) {
      _slaveSelectPin = slaveSelectPin;
      _interruptPin = interruptPin;
      _interruptNum = interruptNum;
      _mode = RF69_MODE_STANDBY;
      _promiscuousMode = false;
      _powerLevel = 31;
      _isRFM69HW = isRFM69HW;
    }

    bool initialize(uint8_t freqBand, uint8_t ID, uint8_t networkID=1);
    void setAddress(uint8_t addr);
    void setNetwork(uint8_t networkID=1);
    bool canSend();
    void send(uint8_t toAddress, const void* buffer, uint8_t bufferSize, bool requestACK=false);
    bool sendWithRetry(uint8_t toAddress, const void* buffer, uint8_t bufferSize, uint8_t retries=2, uint8_t retryWaitTime=40); // 40ms roundtrip req for 61byte packets
    bool receiveDone();
    bool ACKReceived(uint8_t fromNodeID);
    bool ACKRequested();
    void sendACK(const void* buffer = "", uint8_t bufferSize=0);
    uint32_t getFrequency();
    void setFrequency(uint32_t freqHz);
    void encrypt(const char* key);
    void setCS(uint8_t newSPISlaveSelect);
    int16_t readRSSI(bool forceTrigger=false);
    void promiscuous(bool onOff=true);
    void setHighPower(bool onOFF=true, byte PA_ctl=0x60); //have to call it after initialize for RFM69HW
    void setPowerLevel(uint8_t level); // reduce/increase transmit power level
    int  enableAutoPower(int targetRSSI=-69);   // TWS: New method to enable/disable auto Power control
    int  getAckRSSI(void);                      // TWS: New method to retrieve the ack'd RSSI (if any)
    byte setLNA(byte newReg);                   // TWS: function to control LNA reg for power testing purposes
    void sleep();
    uint8_t readTemperature(uint8_t calFactor=0); // get CMOS temperature (8bit)
    void rcCalibration(); // calibrate the internal RC oscillator for use in wide temperature variations - see datasheet section [4.3.5. RC Timer Accuracy]
    void setMode(byte mode);  // TWS: moved from protected to try to build block()/unblock() wrapper

    // allow hacking registers by making these public
    uint8_t readReg(uint8_t addr);
    void writeReg(uint8_t addr, uint8_t val);
    void readAllRegs();
    int  _targetRSSI;     // if non-zero then this is the desired end point RSSI for our transmission
    byte _transmitLevel;  // saved powerLevel in case we do auto power adjustment, this value gets dithered

  protected:
    static void isr0();
    void virtual interruptHandler();
    void sendFrame(byte toAddress, const void* buffer, byte size, bool requestACK=false, bool sendACK=false, bool sendRSSI=false, int lastRSSI=0);

    static RFM69* selfPointer;
    uint8_t _slaveSelectPin;
    uint8_t _interruptPin;
    uint8_t _interruptNum;
    uint8_t _address;
    bool    _promiscuousMode;
    uint8_t _powerLevel;
    bool    _isRFM69HW;
    uint8_t _SPCR;
    uint8_t _SPSR;
    uint8_t _SREG;
    int16_t _ackRSSI;         // this contains the RSSI our destination Ack'd back to us (if we enabledAutoPower)
    bool    _powerBoost;      // this controls whether we need to turn on the highpower regs based on the setPowerLevel input
    uint8_t _PA_Reg;          // saved and derived PA control bits so we don't have to spend time reading back from SPI port

    void receiveBegin();
//    void setMode(uint8_t mode);  // TWS: moved to public to create blocking method
    void setHighPowerRegs(bool onOff);
    void select();
    void unselect();
    
#ifdef SPI_HAS_TRANSACTION
    SPISettings _settings;
#endif
};

#endif
