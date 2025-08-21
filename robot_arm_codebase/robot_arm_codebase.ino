#include "utils.hpp"
#include "lib/types.hpp"
#include "lib/IKUtils.hpp"
#include "lib/FKUtils.hpp"
#include "lib/JacobianUtils.hpp"
#include "canbusUtils.hpp"
#include "motorDriverUtils.hpp"
#include "systemStatesUtils.hpp"
#include "serialUI.hpp"

void setup()
{
  initComms();
  initCanBus();
  initMotors();

  // esp_reset_reason_t resetReason = esp_reset_reason();

  globalStats.begin();
}

void loop()
{
  system_run();
  checkSerial();
  globalStats.report();
}
