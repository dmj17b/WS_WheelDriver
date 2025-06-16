/**
 * LS7466 Quadrature Decoder Library
 * 
 * This library provides an interface for the LS7466 quadrature decoder chip 
 * through SPI communication. It closely follows the example implementation
 * provided in the datasheet.
 * 
 * Updated: March 18, 2025
 */

 #ifndef LS7466_H
 #define LS7466_H
 
 #include <Arduino.h>
 #include <SPI.h>
 
 /* MCR0 configuration data (added from example) */
 #define QUADRX1 0x00      // non-quadrature mode
 #define QUADRX2 0x01      // X2 quadrature mode
 #define QUADRX4 0x02      // X4 quadrature mode
 #define FREE_RUN 0x00     // free-running mode
 #define SNGCYC 0x04       // single-cycle mode
 #define INDX_LCNT 0x00    // index input in non-synchronous mode
 #define INDX_LCNTRL 0x08  // index input in synchronous mode
 #define ACTV_HI_INDX 0x00 // active high index
 #define ACTV_LOW_INDX 0x10 // active low index
 
 /* MCR1 configuration data; any of these data segments can be ORed together */
 //Flag modes
 #define NO_FLAGS 0x00     // no flag
 #define INDX_FLAG 0x10    // INDX flag
 #define EQL_FLAG 0x20     // EQL flag
 #define BW_FLAG 0x40      // BW flag
 #define CY_FLAG 0x80      // CY flag
 #define DYNAMIC_FLAG 0x00 // FLAG/ out is dynamic
 #define LATCHED_FLAG 0x08 // FLAG/ out is latched
 //Enable/disable counter
 #define EN_CNTR 0x00      // counting enabled
 #define DIS_CNTR 0x04     // counting disabled
 //data width
 #define BYTE_3 0x00       // 3-byte mode
 #define BYTE_2 0x01       // 2-byte mode
 #define BYTE_1 0x02       // 1-byte mode
 
 /* LS7466 op-code list */
 #define RST_MCR0x 0x08    // reset x-axis MCR0
 #define RST_MCR0y 0x0a    // reset y-axis MCR0
 #define RST_MCR0xy 0x0c   // reset both-axis MCR0
 #define RST_MCR1x 0x10    // reset x-axis MCR1
 #define RST_MCR1y 0x12    // reset y-axis MCR1
 #define RST_MCR1xy 0x14   // reset both-axis MCR1
 #define RST_CNTRx 0x20    // reset x-axis CNTR
 #define RST_CNTRy 0x22    // reset y-axis CNTR
 #define RST_CNTRxy 0x24   // reset both-axis CNTR
 #define RST_SSTRx 0x30    // reset x-axis SSTR
 #define RST_SSTRy 0x32    // reset y-axis SSTR
 #define RST_SSTRxy 0x34   // reset both-axis SSTR
 #define RD_MCR0x 0x48     // read x-axis MCR0
 #define RD_MCR0y 0x4a     // read y-axis MCR0
 #define RD_MCR1x 0x50     // read x-axis MCR1
 #define RD_MCR1y 0x52     // read y-axis MCR1
 #define RD_CNTRx 0x60     // read x-axis CNTR
 #define RDC_LDSx 0x61     // read CNTR and load SSTR in x-axis
 #define RD_CNTRy 0x62     // read y-axis CNTR
 #define RDC_LDSy 0x63     // read CNTR and load SSTR in y-axis
 #define RD_ODRx 0x68      // read x-axis ODR
 #define RD_ODRy 0x6a      // read y-axis ODR
 #define RD_SSTRx 0x70     // read x-axis SSTR
 #define RD_SSTRy 0x72     // read y-axis SSTR
 #define WR_MCR0x 0x88     // write to x-axis MCR0
 #define WR_MCR0y 0x8a     // write to y-axis MCR0
 #define WR_MCR0xy 0x8c    // write to both-axis MCR0
 #define WR_MCR1x 0x90     // write to x-axis MCR1
 #define WR_MCR1y 0x92     // write to y-axis MCR1
 #define WR_MCR1xy 0x94    // write to both-axis MCR1
 #define WR_IDRx 0x98      // write to x-axis IDR
 #define WR_IDRy 0x9a      // write to y-axis IDR
 #define WR_IDRxy 0x9c     // write to both-axis IDR
 #define LOAD_CNTRx 0xe0   // load x-axis CNTR
 #define LOAD_CNTRy 0xe2   // load y-axis CNTR
 #define LOAD_CNTRxy 0xe4  // load both-axis CNTR
 #define LOAD_ODRx 0xe8    // load x-axis ODR
 #define LOAD_ODRy 0xea    // load y-axis ODR
 #define LOAD_ODRxy 0xec   // load both-axis ODR
 #define LOAD_SSTRx 0xf0   // load x-axis SSTR
 #define LOAD_SSTRy 0xf2   // load y-axis SSTR
 #define LOAD_SSTRxy 0xf4  // load both-axis SSTR
 
 class LS7466 {
 public:
     // Constructor
     LS7466(uint8_t csPin);
     
     // Initialization
     void begin();
     
     // Basic operations matching the example code
     void loadResetReg(uint8_t opCode);
     void singleByteWR(uint8_t opCode, uint8_t data);
     uint8_t singleByteRD(uint8_t opCode);
     
     // Multi-byte operations
     long readMultiByteCounter(uint8_t axis);
     void writeMultiByteCounter(uint8_t axis, long value);
     
     // Enhanced operations
     void configMCR0(uint8_t axis, uint8_t config);
     void configMCR1(uint8_t axis, uint8_t config);
     void resetCounter(uint8_t axis);
     long readCounter(uint8_t axis);
     long readCounterAndLoadSnapshot(uint8_t axis);
     long readSnapshot(uint8_t axis);
     
     // Constants for axis selection
     static const uint8_t AXIS_X = 0;
     static const uint8_t AXIS_Y = 1;
     static const uint8_t AXIS_BOTH = 2;
     
 private:
     uint8_t _csPin;
     uint8_t _dataWidth[2];  // Store data width setting for each axis
     
     // Helper functions
     uint8_t getAxisCommand(uint8_t baseCommand, uint8_t axis);
 };
 
 /**
  * Implementation of the LS7466 class methods
  */
 
 LS7466::LS7466(uint8_t csPin) : _csPin(csPin) {
     // Default to 3-byte mode
     _dataWidth[0] = BYTE_3;
     _dataWidth[1] = BYTE_3;
 }
 
 void LS7466::begin() {
     // Set up chip select pin
     pinMode(_csPin, OUTPUT);
     digitalWrite(_csPin, HIGH);  // Active low
     
     // Initialize SPI with settings similar to the example
     SPI.begin();
     delay(1);
     SPI.setDataMode(SPI_MODE0);  // CPOL=0, CPHA=0
     delay(1);   
     SPI.setBitOrder(MSBFIRST);   // MSB first
     delay(1);
     SPI.setClockDivider(SPI_CLOCK_DIV4); // fosc/4
     delay(1);
     // Reset all registers
     loadResetReg(RST_CNTRxy);
     loadResetReg(RST_SSTRxy);
 }
 
 void LS7466::loadResetReg(uint8_t opCode) {
     digitalWrite(_csPin, HIGH);  // Ensure deselected
     digitalWrite(_csPin, LOW);   // Select device
     
     // Send command to LS7466
     SPI.transfer(opCode);
     
     digitalWrite(_csPin, HIGH);  // Deselect device
 }
 
 void LS7466::singleByteWR(uint8_t opCode, uint8_t data) {
     digitalWrite(_csPin, HIGH);  // Ensure deselected
     delayMicroseconds(5);
     digitalWrite(_csPin, LOW);   // Select device
     delayMicroseconds(5);

     // Send command to LS7466
     SPI.transfer(opCode);
     
     // Send data to be written to LS7466 Register
     SPI.transfer(data);
     
     digitalWrite(_csPin, HIGH);  // Deselect device
     delayMicroseconds(5);

    }
 
 uint8_t LS7466::singleByteRD(uint8_t opCode) {
     uint8_t data;
     
     digitalWrite(_csPin, HIGH);  // Ensure deselected
     delayMicroseconds(5);
     digitalWrite(_csPin, LOW);   // Select device
     delayMicroseconds(5);

     // Send RD OP Code
     SPI.transfer(opCode);
     
     // Dummy Transfer to RD Data
     data = SPI.transfer(0xFF);
     
     digitalWrite(_csPin, HIGH);  // Deselect device
     delayMicroseconds(5);

     return data;
 }
 
 long LS7466::readMultiByteCounter(uint8_t axis) {
    long result = 0;
    uint8_t byteL, byteM, byteH;
    uint8_t opCode;
    uint8_t dataWidth;
    
    // Select appropriate op code and data width
    if (axis == AXIS_X) {
        opCode = RD_CNTRx;
        dataWidth = _dataWidth[0];
    } else {
        opCode = RD_CNTRy;
        dataWidth = _dataWidth[1];
    }
    
    digitalWrite(_csPin, HIGH);
    delayMicroseconds(5);
    digitalWrite(_csPin, LOW);
    delayMicroseconds(5);
    
    // Send RD OP Code
    SPI.transfer(opCode);
    
    // Read bytes in the correct order (LSB first per LS7466 datasheet)
    byteL = SPI.transfer(0xFF);  // LSB
    byteM = SPI.transfer(0xFF);  // Middle byte
    byteH = SPI.transfer(0xFF);  // MSB
    
    digitalWrite(_csPin, HIGH);
    
    
    // Combine bytes correctly according to data width
    if (dataWidth == BYTE_1) {
        result = byteL;
        // Sign extend if negative (bit 7 is set)
        if (result & 0x80) {
            result |= 0xFFFFFF00;
        }
    } else if (dataWidth == BYTE_2) {
        result = (byteL << 8) | byteM;
        // Sign extend if negative (bit 15 is set)
        if (result & 0x8000) {
            result |= 0xFFFF0000;
        }
    } else {  // BYTE_3
        result = (byteL << 16) | (byteM << 8) | byteH;
        // Sign extend if negative (bit 23 is set)
        if (result & 0x800000) {
            result |= 0xFF000000;
        }
    }
    
    return result;
}
 
 
 void LS7466::writeMultiByteCounter(uint8_t axis, long value) {
     uint8_t opCode;
     uint8_t dataWidth;
     
     // Select appropriate op code and data width
     if (axis == AXIS_X) {
         opCode = LOAD_CNTRx;
         dataWidth = _dataWidth[0];
     } else {
         opCode = LOAD_CNTRy;
         dataWidth = _dataWidth[1];
     }
     
     digitalWrite(_csPin, HIGH);  // Ensure deselected
     digitalWrite(_csPin, LOW);   // Select device
     
     // Send LOAD OP Code
     SPI.transfer(opCode);
     
     // Write 1, 2, or 3 bytes depending on the data width setting
     if (dataWidth == BYTE_1) {
         SPI.transfer(value & 0xFF);
     } else if (dataWidth == BYTE_2) {
         SPI.transfer(value & 0xFF);
         SPI.transfer((value >> 8) & 0xFF);
     } else {  // BYTE_3
         SPI.transfer(value & 0xFF);
         SPI.transfer((value >> 8) & 0xFF);
         SPI.transfer((value >> 16) & 0xFF);
     }
     
     digitalWrite(_csPin, HIGH);  // Deselect device
 }
 
 void LS7466::configMCR0(uint8_t axis, uint8_t config) {
     uint8_t opCode = getAxisCommand(WR_MCR0x, axis);
     singleByteWR(opCode, config);
 }
 
 void LS7466::configMCR1(uint8_t axis, uint8_t config) {
     uint8_t opCode = getAxisCommand(WR_MCR1x, axis);
     
     // Store data width setting
     if (axis == AXIS_X || axis == AXIS_BOTH) {
         _dataWidth[0] = config & 0x03;
     }
     if (axis == AXIS_Y || axis == AXIS_BOTH) {
         _dataWidth[1] = config & 0x03;
     }
     
     singleByteWR(opCode, config);
 }
 
 void LS7466::resetCounter(uint8_t axis) {
     uint8_t opCode = getAxisCommand(RST_CNTRx, axis);
     loadResetReg(opCode);
 }
 
 long LS7466::readCounter(uint8_t axis) {
     if (axis == AXIS_BOTH) {
         // Cannot read both at once, default to X
         axis = AXIS_X;
     }
     return readMultiByteCounter(axis);
 }
 
 long LS7466::readCounterAndLoadSnapshot(uint8_t axis) {
     long result = 0;
     uint8_t opCode;
     uint8_t dataWidth;
     
     // Select appropriate op code and data width
     if (axis == AXIS_X) {
         opCode = RDC_LDSx;
         dataWidth = _dataWidth[0];
     } else {
         opCode = RDC_LDSy;
         dataWidth = _dataWidth[1];
     }
     
     digitalWrite(_csPin, HIGH);  // Ensure deselected
     digitalWrite(_csPin, LOW);   // Select device
     
     // Send RD OP Code
     SPI.transfer(opCode);
     
     // Read 1, 2, or 3 bytes depending on the data width setting
     if (dataWidth == BYTE_1) {
         result = SPI.transfer(0xFF);
     } else if (dataWidth == BYTE_2) {
         result = SPI.transfer(0xFF);
         result |= ((long)SPI.transfer(0xFF) << 8);
     } else {  // BYTE_3
         result = SPI.transfer(0xFF);
         result |= ((long)SPI.transfer(0xFF) << 8);
         result |= ((long)SPI.transfer(0xFF) << 16);
     }
     
     digitalWrite(_csPin, HIGH);  // Deselect device
     
     return result;
 }
 
 long LS7466::readSnapshot(uint8_t axis) {
     long result = 0;
     uint8_t opCode;
     uint8_t dataWidth;
     
     // Select appropriate op code and data width
     if (axis == AXIS_X) {
         opCode = RD_SSTRx;
         dataWidth = _dataWidth[0];
     } else {
         opCode = RD_SSTRy;
         dataWidth = _dataWidth[1];
     }
     
     digitalWrite(_csPin, HIGH);  // Ensure deselected
     digitalWrite(_csPin, LOW);   // Select device
     
     // Send RD OP Code
     SPI.transfer(opCode);
     
     // Read 1, 2, or 3 bytes depending on the data width setting
     if (dataWidth == BYTE_1) {
         result = SPI.transfer(0xFF);
     } else if (dataWidth == BYTE_2) {
         result = SPI.transfer(0xFF);
         result |= ((long)SPI.transfer(0xFF) << 8);
     } else {  // BYTE_3
         result = SPI.transfer(0xFF);
         result |= ((long)SPI.transfer(0xFF) << 8);
         result |= ((long)SPI.transfer(0xFF) << 16);
     }
     
     digitalWrite(_csPin, HIGH);  // Deselect device
     
     return result;
 }
 
 uint8_t LS7466::getAxisCommand(uint8_t baseCommand, uint8_t axis) {
     if (axis == AXIS_Y) {
         return baseCommand + 2;
     } else if (axis == AXIS_BOTH) {
         return baseCommand + 4;
     }
     return baseCommand;  // Default to X axis
 }
 
 #endif // LS7466_H