

#ifdef __AVR_ATtiny85__
#include <TinyWireM.h>
#define Wire TinyWireM
#else
#include <Wire.h>
#endif


#ifdef __AVR
#include <avr/pgmspace.h>
#elif defined(ESP8266)
#include <pgmspace.h>
#endif
#include <tjf_mcp23017.h>

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

//void tjf_mcp23017::tcaSelect(uint8_t port) {
//  if (port > 7) return;
//
//  Wire.beginTransmission(TCAADDR);
//  Wire.write(1 << port);
//  Wire.endTransmission();
//}

// minihelper to keep Arduino backward compatibility
static inline void wiresend(uint8_t x)
{
#if ARDUINO >= 100
  Wire.write((uint8_t) x);
#else
  Wire.send(x);
#endif
}

static inline uint8_t wirerecv(void)
{
#if ARDUINO >= 100
  return Wire.read();
#else
  return Wire.receive();
#endif
}

uint8_t tjf_mcp23017::MCPpin(uint16_t pinRaw)    //added
{
  return (pinRaw - ((pinRaw/16)*16));

}

/**
 * Bit number associated to a give Pin
 */
uint8_t tjf_mcp23017::bitForPin(uint16_t pin)
{
  return pin%8;
}

uint8_t tjf_mcp23017::mcpForPin(uint16_t pinRaw)
{
  return mcpAddrs[pinRaw / 16];

}


//uint8_t tjf_mcp23017::tcaForAddr(uint8_t addr)
//{
//    return (addr / 16);
//}

//uint8_t tjf_mcp23017::tcaForPin(uint16_t pinRaw)
//{
//    return (pinRaw / 128);
//}

void tjf_mcp23017::addMCP(uint8_t addr)
{
  mcpAddrs[addrCount] = addr;
  addrCount++;
}
/**
 * Register address, port dependent, for a given PIN
 */
uint8_t tjf_mcp23017::regForPin(uint16_t xMCPpin, uint8_t portAaddr, uint8_t portBaddr)
{

  if ((uint8_t)xMCPpin < 8)
  {
    return portAaddr;
  }
  else
  {
    return portBaddr;
  }
}

/**
 * Reads a given register
 */
uint8_t tjf_mcp23017::readRegister(uint8_t mcpAddr, uint8_t addr)
{
  // read the current GPINTEN
  //tcaSelect(tcaForAddr(mcpAddr));
  Wire.beginTransmission(MCP23017_ADDRESS | mcpAddr);
  wiresend(addr);
  Wire.endTransmission();
  Wire.requestFrom(MCP23017_ADDRESS | mcpAddr, 1);
  return wirerecv();
  Wire.endTransmission();
}


/**
 * Writes a given register
 */
void tjf_mcp23017::writeRegister(uint8_t mcpAddr, uint8_t regAddr, uint8_t regValue)
{
  // Write the register
  //tcaSelect(tcaForAddr(mcpAddr));
  Wire.beginTransmission(MCP23017_ADDRESS | mcpAddr);
  wiresend(regAddr);
  wiresend(regValue);
  Wire.endTransmission();
}


/**
 * Helper to update a single bit of an A/B register.
 * - Reads the current register value
 * - Writes the new register value
 */
void tjf_mcp23017::updateRegisterBit(uint16_t pinRaw, uint8_t pValue, uint8_t portAaddr, uint8_t portBaddr)
{
  uint8_t regValue;
  uint8_t regAddr=regForPin(MCPpin(pinRaw),portAaddr,portBaddr);
  uint8_t bit=bitForPin(MCPpin(pinRaw));
  regValue = readRegister(mcpForPin(pinRaw),regAddr);
  // set the value for the particular bit
  bitWrite(regValue,bit,pValue);
  writeRegister(mcpForPin((uint8_t)pinRaw), regAddr,regValue);
}

////////////////////////////////////////////////////////////////////////////////

/**
 * Initializes the MCP23017.
 */
void tjf_mcp23017::begin(void)
{
  Wire.begin();


  // set defaults!
  // all inputs on port A and B
  for (byte B = 0; B < addrCount; B++)
  {
    writeRegister(mcpAddrs[B], MCP23017_IODIRA,0xff);
    writeRegister(mcpAddrs[B], MCP23017_IODIRB,0xff);
  }

}

/**
 * Sets the pin mode to either INPUT or OUTPUT
 */
void tjf_mcp23017::pinMode(uint8_t pinRaw, uint8_t d)
{
  updateRegisterBit(pinRaw,(d==INPUT),MCP23017_IODIRA,MCP23017_IODIRB);
}

/**
 * Reads all 16 pins (port A and B) into a single 16 bits variable.
 */
uint16_t tjf_mcp23017::readGPIOAB(uint8_t addr)
{
  uint16_t ba = 0;
  uint8_t a;

  //tcaSelect(tcaForAddr(addr));
  // read the current GPIO output latches
  Wire.beginTransmission(MCP23017_ADDRESS | addr);
  wiresend(MCP23017_GPIOA);
  Wire.endTransmission();

  Wire.requestFrom(MCP23017_ADDRESS | addr, 2);
  a = wirerecv();
  ba = wirerecv();
  ba <<= 8;
  ba |= a;

  return ba;
}

/**
 * Read a single port, A or B, and return its current 8 bit value.
 * Parameter b should be 0 for GPIOA, and 1 for GPIOB.
 */
uint8_t tjf_mcp23017::readGPIO(uint8_t addr, uint8_t b)
{

  //tcaSelect(tcaForAddr(addr));
  // read the current GPIO output latches
  Wire.beginTransmission(MCP23017_ADDRESS | addr);
  if (b == 0)
    wiresend(MCP23017_GPIOA);
  else
  {
    wiresend(MCP23017_GPIOB);
  }
  Wire.endTransmission();

  Wire.requestFrom(MCP23017_ADDRESS | addr, 1);
  return wirerecv();
}

/**
 * Writes all the pins in one go. This method is very useful if you are implementing a multiplexed matrix and want to get a decent refresh rate.
 */
void tjf_mcp23017::writeGPIOAB(uint8_t addr, uint16_t ba)
{
  //tcaSelect(tcaForAddr(addr));
  Wire.beginTransmission(MCP23017_ADDRESS | addr);
  wiresend(MCP23017_GPIOA);
  wiresend(ba & 0xFF);
  wiresend(ba >> 8);
  Wire.endTransmission();
}

void tjf_mcp23017::digitalWrite(uint16_t pinRaw, uint8_t d)
{
  uint8_t gpio;
  uint8_t bit=bitForPin(MCPpin(pinRaw));

  // read the current GPIO output latches
  uint8_t regAddr=regForPin(MCPpin(pinRaw),MCP23017_OLATA,MCP23017_OLATB);
  gpio = readRegister(mcpForPin(pinRaw), regAddr);

  // set the pinRaw and direction
  bitWrite(gpio,bit,d);

  // write the new GPIO
  regAddr=regForPin(MCPpin(pinRaw),MCP23017_GPIOA,MCP23017_GPIOB);
  writeRegister(mcpForPin(pinRaw), regAddr,gpio);
}

void tjf_mcp23017::pullUp(uint16_t pinRaw, uint8_t d)
{
  updateRegisterBit(pinRaw,d,MCP23017_GPPUA,MCP23017_GPPUB);
}

uint8_t tjf_mcp23017::digitalRead(uint16_t pinRaw)
{

  uint8_t bit=bitForPin(MCPpin(pinRaw));
  uint8_t regAddr=regForPin(MCPpin(pinRaw),MCP23017_GPIOA,MCP23017_GPIOB);


  return (readRegister(mcpForPin(pinRaw), regAddr) >> bit) & 0x1;
}

/**
 * Configures the interrupt system. both port A and B are assigned the same configuration.
 * Mirroring will OR both INTA and INTB pins.
 * Opendrain will set the INT pin to value or open drain.
 * polarity will set LOW or HIGH on interrupt.
 * Default values after Power On Reset are: (false,flase, LOW)
 * If you are connecting the INTA/B pin to arduino 2/3, you should configure the interupt handling as FALLING with
 * the default configuration.
 */
void tjf_mcp23017::setupInterrupts(uint8_t addr, uint8_t mirroring, uint8_t openDrain, uint8_t polarity)
{
  // configure the port A
  uint8_t ioconfValue=readRegister(addr,MCP23017_IOCONA);
  bitWrite(ioconfValue,6,mirroring);
  bitWrite(ioconfValue,2,openDrain);
  bitWrite(ioconfValue,1,polarity);
  writeRegister(addr,MCP23017_IOCONA,ioconfValue);

  // Configure the port B
  ioconfValue=readRegister(addr,MCP23017_IOCONB);
  bitWrite(ioconfValue,6,mirroring);
  bitWrite(ioconfValue,2,openDrain);
  bitWrite(ioconfValue,1,polarity);
  writeRegister(addr,MCP23017_IOCONB,ioconfValue);
}

/**
 * Set's up a pin for interrupt. uses arduino MODEs: CHANGE, FALLING, RISING.
 *
 * Note that the interrupt condition finishes when you read the information about the port / value
 * that caused the interrupt or you read the port itself. Check the datasheet can be confusing.
 *
 */
void tjf_mcp23017::setupInterruptPin(uint16_t pinRaw, uint8_t mode)
{

  // set the pin interrupt control (0 means change, 1 means compare against given value);
  updateRegisterBit(pinRaw,(mode!=CHANGE),MCP23017_INTCONA,MCP23017_INTCONB);
  // if the mode is not CHANGE, we need to set up a default value, different value triggers interrupt

  // In a RISING interrupt the default value is 0, interrupt is triggered when the pin goes to 1.
  // In a FALLING interrupt the default value is 1, interrupt is triggered when pin goes to 0.
  updateRegisterBit(pinRaw,(mode==FALLING),MCP23017_DEFVALA,MCP23017_DEFVALB);

  // enable the pin for interrupt
  updateRegisterBit(pinRaw,HIGH,MCP23017_GPINTENA,MCP23017_GPINTENB);

}

uint8_t tjf_mcp23017::getLastInterruptPin(uint8_t addr)
{
  uint8_t intf;

  // try port A
  intf=readRegister(addr,MCP23017_INTFA);
  for(unsigned int i=0; i<8; i++)
    if (bitRead(intf,i))
      return i;

  // try port B
  intf=readRegister(addr,MCP23017_INTFB);
  for(unsigned int i=0; i<8; i++)
    if (bitRead(intf,i))
      return i+8;

  return MCP23017_INT_ERR;

}
uint8_t tjf_mcp23017::getLastInterruptPinValue(uint8_t addr)
{
  uint8_t intPin=getLastInterruptPin(addr);
  if(intPin!=MCP23017_INT_ERR)
  {
    uint8_t intcapreg=regForPin(intPin,MCP23017_INTCAPA,MCP23017_INTCAPB);
    uint8_t bit=bitForPin(intPin);
    return (readRegister(mcpForPin(intPin),intcapreg)>>bit) & (0x01);
  }

  return MCP23017_INT_ERR;
}


