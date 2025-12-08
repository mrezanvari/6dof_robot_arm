#include "esp_partition.h"

void printPartitions()
{
  Serial.println("\n--- Partition Table ---");

  esp_partition_iterator_t it = esp_partition_find(ESP_PARTITION_TYPE_ANY, ESP_PARTITION_SUBTYPE_ANY, NULL);

  for (; it != NULL; it = esp_partition_next(it))
  {
    const esp_partition_t *part = esp_partition_get(it);
    Serial.printf("Name: %s, Type: 0x%02X, Subtype: 0x%02X, Size: %d bytes, Address: 0x%X\n",
                  part->label, part->type, part->subtype, part->size, part->address);
  }

  esp_partition_iterator_release(it);
}

Coor tempCoor;

void checkSerial()
{
  vector<String> cmd; // will hold all of the commands and values for processing...

  if (!Serial.available())
    return;

  String serialIn = Serial.readStringUntil('\n');
  serialIn.remove(serialIn.length() - 1); // remove \r

  String token; // tokenizer
  int index = 0;
  int cmdIndex = -1;

  while (serialIn.length() > 0)
  {
    char firstChar = serialIn.charAt(0);
    if (firstChar == '\'' || firstChar == '\"')
    {
      char quote = firstChar;
      serialIn.remove(0, 1);
      index = serialIn.indexOf(String(quote));
      if (index == -1)
      {
        token = serialIn;
        serialIn = "";
      }
      else
      {
        token = serialIn.substring(0, index);
        serialIn.remove(0, index + 1);
      }
    }
    else
    {
      index = serialIn.indexOf(" ");
      if (index == -1)
        index = serialIn.length();

      token = serialIn.substring(0, index);
      serialIn.remove(0, index);
    }

    while (serialIn.length() > 0 && serialIn.charAt(0) == ' ')
      serialIn.remove(0, 1);

    cmd.push_back(token);
  }

  int cmdSize = cmd.size();

  if (cmdSize <= 0)
    return;

  if (cmd[0].equalsIgnoreCase(F("reset")) || cmd[0].equalsIgnoreCase(F("r")))
    esp_restart();

  if (cmd[0].equalsIgnoreCase("hey"))
  {
    Serial.println(F("Hey!"));
    return;
  }

  if (cmd[0].equalsIgnoreCase(F(("help"))) || cmd[0].equalsIgnoreCase(F("h")))
  {
    Serial.println(F("\n                                                           ..::Robot Settings::.."));
    Serial.println(F("-------------------------------------------------------------------------------------------------------------------------------------------------------\n\nCommands:\n"));
    Serial.println(F("hey: Check for connection."));
    Serial.println(F("set led [R] [G] [B]: change the color of the LED"));
    Serial.println(F("home: change operation mode to HOMING -> begin homing sequence"));
    Serial.println(F("ik: change operation mode to INVERSE_KINEMATICS -> IK mode"));
    Serial.println(F("fk: change operation mode to FORWARD_KINEMATICS"));
    Serial.println(F("ic: change operation mode to IMPEDANCE_CONTROL -> NOT IMPLEMENTED"));
    Serial.println(F("idle: change operation mode to  IDLE -> reads serial comands and brake the motors while printing the motor positions as angle in degrees"));
    Serial.println(F("probe: change operation mode to PROBE -> precise FK mode"));
    Serial.println(F("dev: change operation mode to DEV -> mode used for development and testing"));
    Serial.println(F("jacobi: change operation mode to FULL_JACOBI -> jacobian differencial IK mode"));
    Serial.println(F("set pos [x] [y] [z] [phi] [theta] [psi]: set position/orientation in IK/Jacobian modes"));
    Serial.println(F("                                             - 'nan' can be used to keep the same value for the variable"));
    Serial.println(F("                                             - xyz values can be set alone to keep the orientation the same"));
    Serial.println(F("                                                   - eg.--> set pos nan 200 nan "));
    Serial.println(F("                                                   -        set pos 100 200 -300.20 "));
    Serial.println(F("                                                   -        set pos nan nan nan 90 120 45"));
    Serial.println(F("set trace [x] [y] [z] [phi] [theta] [psi]: set jogging for position and orientation (incremental movement)"));
    Serial.println(F("                                             - 'nan' can be used to keep the same value for the variable"));
    Serial.println(F("                                             - xyz values can be set alone to keep the orientation the same"));
    Serial.println(F("                                                  - eg.--> set trace 0 0.1 0"));
    Serial.println(F("                                                  -        set trace 0 0 0 -0.1 0 0"));
    Serial.println(F("                                                  -        set pos nan nan nan 90 120 45"));
    Serial.println(F("set ang [motor_num] [angle_as_degrees] [velocity]: move motor to angle (degrees) with velocity. Equivalent to MJBots \"d pos\" command."));
    Serial.println(F("                                                  - MUST only be used in IDLE mode"));
    Serial.println(F("                                                  - Motor num is 1-6"));
    Serial.println(F("set lock: set the locking for FK/PROBE modes: brake or releases the motors during FK and Probing modes"));
    Serial.println(F("set zero: zeroes the motors at current position"));
    Serial.println(F("          - MUST only be used during IDLE and with care since IK will be based on the zero position of motors"));
    Serial.println(F("set gain [gain]: sets the gain in jacobian mode"));
    Serial.println(F("set interval [ms]: sets the update interval delay for INVERSE_KINEMATICS, FULL_JACOBI and DEV modes"));
    Serial.println(F("set vel [velocity]: set the globalVelocity value. Default is 0.7"));
    Serial.println(F("set acc [acceleration]: set the globalAcceleration value. Default is 5."));
    Serial.println(F("exec motor_num \"cmd\": execute MJBots commands on a given motor. Commands must be passed with double or single quotes"));
    Serial.println(F("toggle debug: toggles the debug mode -> activates syslogs"));
    Serial.println(F("monitor switches: will show the state of upper and lower limit switches"));
    Serial.println(F("check partitions: will show LittleFS partitions and files on the ESP32 flash"));
    Serial.println(F("reset/r: will reset the device"));
    Serial.println(F("help/h: will show all the commands\n"));
    Serial.println(F("------------------------------------------------------------------------< 2025 >-----------------------------------------------------------------------"));
    return;
  }

  else if (cmd[0].equalsIgnoreCase(F("home")))
    currentSystemState = HOMING;

  else if (cmd[0].equalsIgnoreCase(F("ik")))
    currentSystemState = INVERSE_KINEMATICS;

  else if (cmd[0].equalsIgnoreCase(F("fk")))
    currentSystemState = FORWARD_KINEMATICS;

  else if (cmd[0].equalsIgnoreCase(F("ic")))
    currentSystemState = IMPEDANCE_CONTROL;

  else if (cmd[0].equalsIgnoreCase(F("idle")))
    currentSystemState = IDLE;

  else if (cmd[0].equalsIgnoreCase(F("probe")))
    currentSystemState = PROBE;

  else if (cmd[0].equalsIgnoreCase(F("dev")))
    currentSystemState = DEV;

  else if (cmd[0].equalsIgnoreCase(F("jacobi")))
    currentSystemState = FULL_JACOBI;

  // switch case cannot be used for string in arduino ide!
  else if (cmd[0].equalsIgnoreCase(F("set")))
  {
    if (cmd[1].equalsIgnoreCase(F("led")))
    {
      if (cmdSize < 5)
      {
        Serial.println(F("Command Error!  use \"help\""));
        return;
      }

      rgbLedWrite(RGB_BUILTIN, cmd[2].toInt(), cmd[3].toInt(), cmd[4].toInt());
    }

    else if (cmd[1].equalsIgnoreCase(F("pos")))
    {
      if (cmdSize != 5 && cmdSize != 8)
      {
        Serial.println(F("Command Error!  use \"help\""));
        return;
      }

      tempPos.x = cmd[2].equalsIgnoreCase(F("nan")) ? tempPos.x : cmd[2].toDouble();
      tempPos.y = cmd[3].equalsIgnoreCase(F("nan")) ? tempPos.y : cmd[3].toDouble();
      tempPos.z = cmd[4].equalsIgnoreCase(F("nan")) ? tempPos.z : cmd[4].toDouble();
      tempPos.axisType = Coor::CoorType::Y_UP;

      if (cmdSize == 8)
      {
        tempOrientation.phi = cmd[5].equalsIgnoreCase(F("nan")) ? tempOrientation.phi : rad(cmd[5].toDouble());
        tempOrientation.theta = cmd[6].equalsIgnoreCase(F("nan")) ? tempOrientation.theta : rad(cmd[6].toDouble());
        tempOrientation.psi = cmd[7].equalsIgnoreCase(F("nan")) ? tempOrientation.psi : rad(cmd[7].toDouble());
      }

      //  very important to set all trace to 0 to move to correct position properly
      globalTraceCoor.x = 0;
      globalTraceCoor.y = 0;
      globalTraceCoor.z = 0;

      globalTraceOrientation.phi = globalTraceOrientation.psi = globalTraceOrientation.theta = 0;

      Serial.printf("Set pos to x:%.2f y:%.2f z:%.2f and orientation to phi:%.2f theta:%.2f psi:%.2f\r\n", tempPos.x, tempPos.y, tempPos.z, tempOrientation.phi, tempOrientation.theta, tempOrientation.psi);
    }

    else if (cmd[1].equalsIgnoreCase(F("trace")))
    {
      if (cmdSize != 5 && cmdSize != 8)
      {
        Serial.println(F("Command Error!  use \"help\""));
        return;
      }

      globalTraceCoor.x = cmd[2].equalsIgnoreCase(F("nan")) ? globalTraceCoor.x : cmd[2].toDouble();
      globalTraceCoor.y = cmd[3].equalsIgnoreCase(F("nan")) ? globalTraceCoor.y : cmd[3].toDouble();
      globalTraceCoor.z = cmd[4].equalsIgnoreCase(F("nan")) ? globalTraceCoor.z : cmd[4].toDouble();

      if (cmdSize == 8)
      {
        globalTraceOrientation.phi = cmd[5].equalsIgnoreCase(F("nan")) ? globalTraceOrientation.phi : rad(cmd[5].toDouble());
        globalTraceOrientation.theta = cmd[6].equalsIgnoreCase(F("nan")) ? globalTraceOrientation.theta : rad(cmd[6].toDouble());
        globalTraceOrientation.psi = cmd[7].equalsIgnoreCase(F("nan")) ? globalTraceOrientation.psi : rad(cmd[7].toDouble());
      }

      Serial.printf("Set trace to x:%.2f y:%.2f z:%.2f and orientation to phi:%.2f theta:%.2f psi:%.2f\r\n", tempPos.x, tempPos.y, tempPos.z, tempOrientation.phi, tempOrientation.theta, tempOrientation.psi);
    }

    else if (cmd[1].equalsIgnoreCase(F("ang")))
    {
      if (cmdSize < 4 || cmdSize > 5)
      {
        Serial.println(F("Command Error!  use \"help\""));
        return;
      }

      int motor_ind = cmd[2].toInt();
      if (motor_ind > jointMotors.size() || motor_ind <= 0)
      {
        Serial.println(F("Index not permitted read \"help\""));
        return;
      }

      double ang = cmd[3].equalsIgnoreCase(F("nan")) ? std::numeric_limits<double>::quiet_NaN() : cmd[3].toDouble();
      double pos = (motor_ind < 3) ? mot_a(ang) : wmot_a(ang);
      double vel = 0.4;
      if (cmdSize == 5)
        vel = cmd[4].toDouble();
      Serial.printf("angle set for motor %d as % .3f which corresponds to % .5f motor postion with velocity %.3f\r\n", motor_ind, ang, pos, vel);

      String cmd = "d pos " + String(pos) + " 0 nan v" + String(vel);
      jointMotors[motor_ind - 1].DiagnosticCommand(cmd);
      return;
    }

    else if (cmd[1].equalsIgnoreCase(F("lock")))
      FK_motorLock = !FK_motorLock;

    else if (cmd[1].equalsIgnoreCase(F("zero")))
    {
      if (cmdSize != 2)
      {
        Serial.println(F("Command Error!  use \"help\""));
        return;
      }

      String command = "d exact 0";
      mjbots::moteus::RequireReindex::Command rrcmd;

      baseJointMotor.SetStop();
      lowerJointMotor.SetStop();
      upperJointMotor.SetStop();
      wristBaseJointMotor.SetStop();
      wristLowerJointMotor.SetStop();
      wristUpperJointMotor.SetStop();
      baseJointMotor.DiagnosticCommand(F("conf set motor_position.output.sign -1"));
      lowerJointMotor.DiagnosticCommand(F("conf set motor_position.output.sign -1"));
      upperJointMotor.DiagnosticCommand(F("conf set motor_position.output.sign -1"));
      wristBaseJointMotor.DiagnosticCommand(F("conf set motor_position.output.sign -1"));
      wristLowerJointMotor.DiagnosticCommand(F("conf set motor_position.output.sign -1"));
      wristUpperJointMotor.DiagnosticCommand(F("conf set motor_position.output.sign -1"));
      delay(10);
      baseJointMotor.SetRequireReindex(rrcmd);
      lowerJointMotor.SetRequireReindex(rrcmd);
      upperJointMotor.SetRequireReindex(rrcmd);
      wristLowerJointMotor.SetRequireReindex(rrcmd);
      wristBaseJointMotor.SetRequireReindex(rrcmd);
      wristUpperJointMotor.SetRequireReindex(rrcmd);
      delay(10);
      baseJointMotor.DiagnosticCommand(command);
      lowerJointMotor.DiagnosticCommand(command);
      upperJointMotor.DiagnosticCommand(command);
      wristBaseJointMotor.DiagnosticCommand(command);
      wristLowerJointMotor.DiagnosticCommand(command);
      wristUpperJointMotor.DiagnosticCommand(command);
      baseJointMotor.SetStop();
      lowerJointMotor.SetStop();
      upperJointMotor.SetStop();
      wristBaseJointMotor.SetStop();
      wristLowerJointMotor.SetStop();
      wristUpperJointMotor.SetStop();

      Serial.println(F("Arm and Wrist zeroed"));

      return;
    }

    else if (cmd[1].equalsIgnoreCase(F("gain")))
    {
      if (cmdSize != 3)
      {
        Serial.println(F("Command Error!  use \"help\""));
        return;
      }

      globalJacobiGain = cmd[2].equalsIgnoreCase("nan") ? 1 : cmd[2].toDouble();
      Serial.printf("Set globalJacobiGain to: %.3f\r\n", globalJacobiGain);
    }

    else if (cmd[1].equalsIgnoreCase(F("interval")))
    {
      if (cmdSize != 3)
      {
        Serial.println(F("Command Error!  use \"help\""));
        return;
      }

      updteInterval = cmd[2].toFloat();

      Serial.printf("Set interval to %1.2f\r\n", updteInterval);
    }

    else if (cmd[1].equalsIgnoreCase("vel"))
    {
      if (cmdSize != 3)
      {
        Serial.println(F("Command Error!  use \"help\""));
        return;
      }

      globalVelocity = cmd[2].equalsIgnoreCase(F("nan")) ? 1 : cmd[2].toDouble();

      Serial.printf("Set vel to %1.2f\r\n", globalVelocity);
    }

    else if (cmd[1].equalsIgnoreCase(F("acc")))
    {
      if (cmdSize != 3)
      {
        Serial.println(F("Command Error!  use \"help\""));
        return;
      }

      globalAccel = cmd[2].equalsIgnoreCase(F("nan")) ? std::numeric_limits<double>::quiet_NaN() : cmd[2].toDouble();

      Serial.printf("Set accel to %1.2f\r\n", globalAccel);
    }
  }

  else if (cmd[0].equalsIgnoreCase(F("exec")))
  {
    if (cmdSize != 3)
    {
      Serial.println(F("Command Error!  use \"help\""));
      return;
    }

    int motor_ind = cmd[1].toInt();
    if (motor_ind > jointMotors.size())
    {
      Serial.println(F("Index not permitted read \"help\""));
      return;
    }

    Serial.printf("exec was ran with for motor %d with following cmd:\r\n%s\r\n", motor_ind, cmd[2].c_str());

    jointMotors[motor_ind - 1].DiagnosticCommand(cmd[2].c_str());
  }

  else if (cmd[0].equalsIgnoreCase(F("toggle")))
  {
    if (cmd[1].equalsIgnoreCase(F("debug")))
    {
      DEBUG = !DEBUG;
      Serial.print(F("Debug "));
      Serial.println(DEBUG ? "On" : "Off");
    }
  }

  else if (cmd[0].equalsIgnoreCase(F("monitor")))
  {
    DEBUG = true;
    if (cmd[1].equalsIgnoreCase(F("switches")))
    {
      while (1)
      {
        syslog(F("UPPER_JOINT_LIMIT_SWITCH:"), digitalRead(UPPER_JOINT_LIMIT_SWITCH));
        syslog(F("LOWER_JOINT_LIMIT_SWITCH:"), digitalRead(LOWER_JOINT_LIMIT_SWITCH), true, true);
      }
    }
  }

  else if (cmd[0].equalsIgnoreCase(F("check")))
  {
    if (cmd[1].equalsIgnoreCase(F("partition")))
    {
      printPartitions();
      Serial.println(F("----------------------------- LittleFS -----------------------------"));
      listFiles();
      size_t totalBytes = LittleFS.totalBytes();
      size_t usedBytes = LittleFS.usedBytes();
      Serial.printf("LittleFS Partition Size: %d bytes\n", totalBytes);
      Serial.printf("Used Space: %d bytes\n", usedBytes);
      Serial.printf("Free Space: %d bytes\n", totalBytes - usedBytes);
    }
  }

  else
    Serial.println(F("Command not found, use h/help for info."));

  syslog(F("OK"));
}