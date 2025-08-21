#include <ACAN2517FD.h>

static const uint8_t MCP2517_SCK = 12; // SCK input of MCP2517
static const uint8_t MCP2517_SDI = 11; // SDI input of MCP2517
static const uint8_t MCP2517_SDO = 13; // SDO output of MCP2517

static const uint8_t MCP2517_CS = 10; // CS input of MCP2517
static const uint8_t MCP2517_INT = 9; // INT output of MCP2517

ACAN2517FD can(MCP2517_CS, SPI, MCP2517_INT);

void initCanBus()
{
  ACAN2517FDSettings settings(
      ACAN2517FDSettings::OSC_40MHz, 1000ll * 1000ll, DataBitRateFactor::x1);

  settings.mArbitrationSJW = 2;
  settings.mDriverTransmitFIFOSize = 1;
  settings.mDriverReceiveFIFOSize = 2;

  SPI.begin();
  delay(10);

  const uint32_t errorCode = can.begin(settings, []
                                       { can.isr(); });

  if (errorCode != 0)
    syserr("Error initializing Can Bus with error: " + String(errorCode), false);

  delay(100);
}