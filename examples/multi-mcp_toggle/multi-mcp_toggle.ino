#include <Wire.h>
#include <tjf_mcp23017.h>

tjf_mcp23017 mcp(2);

unsigned long LastDelay = 0UL;
unsigned long Delay = 300Ul;
bool LastState = false;

void setup() {

  mcp.addMCP(0x01); // address (001) of first MCP (decimal value of the binary equivalent of address pins A0, A1, A2)
  mcp.addMCP(0x02); // address (010) of second MCP
  mcp.begin();      // use default address 0

  mcp.pinMode(0, OUTPUT); // pins are assigned to MCPs in order of addition above. first is 0 to 15
  mcp.pinMode(16, OUTPUT); // second MCP handles pins 16 to 31
}


void loop() {
  if ((millis() - LastDelay) > Delay)
  {
    if (LastState == true)
    {
      mcp.digitalWrite(0, HIGH);
      mcp.digitalWrite(16, LOW);
      LastState = false;
    }
    else
    {
      mcp.digitalWrite(0, LOW);
      mcp.digitalWrite(16, HIGH);
      LastState = true;
    }
    LastDelay = millis();
  }
}
