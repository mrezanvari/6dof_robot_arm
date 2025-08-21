#include <Moteus.h>
#include <Vector.h>

Moteus baseJointMotor(can, []()
                      {
  Moteus::Options options;
  options.id = 1;
  return options; }());

Moteus lowerJointMotor(can, []()
                       {
  Moteus::Options options;
  options.id = 2;
  return options; }());

Moteus upperJointMotor(can, []()
                       {
  Moteus::Options options;
  options.id = 3;
  return options; }());

Moteus wristBaseJointMotor(can, []()
                           {
  Moteus::Options options;
  options.id = 4;
  return options; }());

Moteus wristLowerJointMotor(can, []()
                            {
  Moteus::Options options;
  options.id = 5;
  return options; }());

Moteus wristUpperJointMotor(can, []()
                            {
  Moteus::Options options;
  options.id = 6;
  return options; }());

Moteus::PositionMode::Command motor_cmd[6];
Moteus::PositionMode::Format motor_position_fmt;
Moteus::Query::Format motor_query_fmt;

const float lowerJointLimitSwitchOffset = 0.30;
const float upperJointLimitSwitchOffset = 0.0;
const float lowerJointHome = 0 + lowerJointLimitSwitchOffset;
const float upperJointHome = 3.325 + upperJointLimitSwitchOffset;
// const float upperJointMax = -upperJointHome;
const float lowerJointMax = -4.5;
const float baseJointLimitSwitchOffset = 0.0;
const float baseJointHome = 0 + baseJointLimitSwitchOffset; // ---> base home postion must be 0 towards the Z postion
const float baseJointMax = -4.5;

const size_t joints = 3;
vector<Moteus> jointMotors;
size_t selectedMotorIndex = 0;
// Moteus* selectedMotorOBJ = &jointMotors[selectedMotorIndex];

// void selectMotor(int selectionIndex)
// {
//   selectedMotorIndex = selectionIndex <= joints - 1 ? selectionIndex : (selectionIndex % joints);
//   selectedMotorOBJ = &jointMotors[selectedMotorIndex];
// }

void initMotors()
{
  motor_position_fmt.velocity_limit = Moteus::kFloat;
  motor_position_fmt.accel_limit = Moteus::kFloat;
  motor_position_fmt.position = Moteus::kFloat;
  motor_position_fmt.velocity = Moteus::kFloat;
  motor_position_fmt.maximum_torque = Moteus::kFloat;
  // motor_position_fmt.feedforward_torque = Moteus::kFloat;
  motor_query_fmt.trajectory_complete = Moteus::kInt8;

  // TODO: load these from a .hpp, or put on json on littlefs

  baseJointMotor.DiagnosticCommand("conf set servo.pid_position.kp 32.0");
  baseJointMotor.DiagnosticCommand("conf set servo.pid_position.ki 320.0");
  baseJointMotor.DiagnosticCommand("conf set servo.pid_position.kd 0.5");
  baseJointMotor.DiagnosticCommand("conf set servo.pid_position.ilimit 0.48");
  // baseJointMotor.DiagnosticCommand("conf set motor_position.rotor_to_output_ratio 0.1111111111"); // will affect the PID, must first resolve ALL issues then fix this
  delay(10);

  lowerJointMotor.DiagnosticCommand("conf set servo.pid_position.kp 64.0");
  lowerJointMotor.DiagnosticCommand("conf set servo.pid_position.ki 200.0");
  lowerJointMotor.DiagnosticCommand("conf set servo.pid_position.kd 0.6");
  lowerJointMotor.DiagnosticCommand("conf set servo.pid_position.ilimit 0.40");
  // lowerJointMotor.DiagnosticCommand("conf set motor_position.rotor_to_output_ratio 0.1111111111");

  delay(10);
  upperJointMotor.DiagnosticCommand("conf set servo.pid_position.kp 64.0");
  upperJointMotor.DiagnosticCommand("conf set servo.pid_position.ki 640.0");
  upperJointMotor.DiagnosticCommand("conf set servo.pid_position.kd 0.4");
  upperJointMotor.DiagnosticCommand("conf set servo.pid_position.ilimit 0.45");
  // upperJointMotor.DiagnosticCommand("conf set motor_position.rotor_to_output_ratio 0.1111111111");

  // wristBaseJointMotor.DiagnosticCommand("conf set motor_position.rotor_to_output_ratio 0.125");
  // wristLowerJointMotor.DiagnosticCommand("conf set motor_position.rotor_to_output_ratio 0.125");
  // wristUpperJointMotor.DiagnosticCommand("conf set motor_position.rotor_to_output_ratio 0.125");

  delay(10);
  baseJointMotor.SetStop();
  delay(10);
  lowerJointMotor.SetStop();
  delay(10);
  upperJointMotor.SetStop();

  delay(10);
  wristBaseJointMotor.SetStop();
  delay(10);
  wristLowerJointMotor.SetStop();
  delay(10);
  wristUpperJointMotor.SetStop();
  delay(100);

  jointMotors.push_back(baseJointMotor);
  jointMotors.push_back(lowerJointMotor);
  jointMotors.push_back(upperJointMotor);
  jointMotors.push_back(wristBaseJointMotor);
  jointMotors.push_back(wristLowerJointMotor);
  jointMotors.push_back(wristUpperJointMotor);
}

void brakeAllMotors()
{
  baseJointMotor.SetBrake();
  lowerJointMotor.SetBrake();
  upperJointMotor.SetBrake();
  wristBaseJointMotor.SetBrake();
  wristLowerJointMotor.SetBrake();
  wristUpperJointMotor.SetBrake();
}

uint16_t gLoopCount = 0;
static uint32_t gNextSendMillis = 0;

void pidTunningConfigPlot(Moteus motor)
{
  // // baseJointMotor.SetBrake();
  //    const auto time = millis();
  //   if (gNextSendMillis >= time)
  //     return;;

  //   gNextSendMillis += 20;
  //   gLoopCount++;
  //   rgbLedWrite(RGB_BUILTIN, 10, 0, 10);

  //   // for(int i = 0; i < joints; i++) // not efficient for every cycle but this is not a process heavy or crucial task so we can afford it.
  //   //   if(motor.options.id != jointMotors[i].options.id)
  //   //     jointMotors[i].SetBrake();

  //   if(userGlobalPos == std::numeric_limits<double>::quiet_NaN())
  //     motor.DiagnosticCommand("d pos nan 0 nan");

  //   else
  //   {
  //     motor_cmd.position = userGlobalPos;
  //     motor_cmd.position = (gNextSendMillis / 2000) % 2 ? userGlobalPos + 0.5 : userGlobalPos + 0.1;
  //     motor_cmd.velocity = 0.0;
  //     //motor_cmd.velocity_limit = 2.0;
  //     // motor_cmd.accel_limit = 3.0;
  //     motor.SetPosition(motor_cmd, &motor_position_fmt);
  //   }
  //   Serial.print("pos:");
  //   Serial.print(motor.last_result().values.position);
  //   Serial.print(" st:");
  //   Serial.print(motor_cmd.position);
  //   // Serial.print(" torque:");
  //   // Serial.print(motor.last_result().values.torque);
  //   Serial.print(" min:");
  //   Serial.print(userGlobalPos - 1);
  //   Serial.print(" max:");
  //   Serial.println(userGlobalPos + 1);
}