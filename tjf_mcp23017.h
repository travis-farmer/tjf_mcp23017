/***************************************************
  This is a library for the MCP23017 i2c port expander

  These displays use I2C to communicate, 2 pins are required to
  interface
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  BSD license, all text above must be included in any redistribution

  Altered by Travis Farmer to allow for multiple MCPs, to be used for
  DCC++.
  Other contributions:
  https://github.com/hdowne
 ****************************************************/

#ifndef _tjf_mcp23017_H_
#define _tjf_mcp23017_H_

// Don't forget the Wire library
#ifdef __AVR_ATtiny85__
#include <TinyWireM.h>
#else
#include <Wire.h>
#endif

class tjf_mcp23017 {
public:
  tjf_mcp23017(const size_t size) { mcpAddrs = new uint32_t[size]; }
  ~tjf_mcp23017() { delete mcpAddrs; }
  void begin(void);

  void pinMode(uint8_t p, uint8_t d);
  void digitalWrite(uint16_t p, uint8_t d);
  void pullUp(uint16_t p, uint8_t d);
  uint8_t digitalRead(uint16_t p);

  void writeGPIOAB(uint8_t addr, uint16_t);
  uint16_t readGPIOAB(uint8_t addr);
  uint8_t readGPIO(uint8_t addr, uint8_t b);

  void setupInterrupts(uint8_t addr, uint8_t mirroring, uint8_t open, uint8_t polarity);
  void setupInterruptPin(uint16_t p, uint8_t mode);
  uint8_t getLastInterruptPin(uint8_t addr);
  uint8_t getLastInterruptPinValue(uint8_t addr);
  void addMCP(uint8_t addr);
 private:
  uint8_t i2caddr;
  uint8_t bitForPin(uint16_t pin);
  uint8_t regForPin(uint16_t pin, uint8_t portAaddr, uint8_t portBaddr);
  uint8_t MCPpin(uint16_t pin);  //added
  uint8_t mcpForPin(uint16_t pin);
  //uint8_t tcaForAddr(uint8_t addr);
  //uint8_t tcaForPin(uint16_t pin);
  uint32_t* mcpAddrs;
  uint8_t addrCount = 0;
  uint8_t readRegister(uint8_t mcpAddr, uint8_t addr);
  void writeRegister(uint8_t mcpAddr, uint8_t addr, uint8_t value);
  uint8_t TCAADDR = 0x70;
  //void tcaSelect(uint8_t port);

  /**
   * Utility private method to update a register associated with a pin (whether port A/B)
   * reads its value, updates the particular bit, and writes its value.
   */
  void updateRegisterBit(uint16_t p, uint8_t pValue, uint8_t portAaddr, uint8_t portBaddr);

};

#define MCP23017_ADDRESS 0x20

// registers
#define MCP23017_IODIRA 0x00
#define MCP23017_IPOLA 0x02
#define MCP23017_GPINTENA 0x04
#define MCP23017_DEFVALA 0x06
#define MCP23017_INTCONA 0x08
#define MCP23017_IOCONA 0x0A
#define MCP23017_GPPUA 0x0C
#define MCP23017_INTFA 0x0E
#define MCP23017_INTCAPA 0x10
#define MCP23017_GPIOA 0x12
#define MCP23017_OLATA 0x14

#define MCP23017_IODIRB 0x01
#define MCP23017_IPOLB 0x03
#define MCP23017_GPINTENB 0x05
#define MCP23017_DEFVALB 0x07
#define MCP23017_INTCONB 0x09
#define MCP23017_IOCONB 0x0B
#define MCP23017_GPPUB 0x0D
#define MCP23017_INTFB 0x0F
#define MCP23017_INTCAPB 0x11
#define MCP23017_GPIOB 0x13
#define MCP23017_OLATB 0x15

#define MCP23017_INT_ERR 255

#endif
