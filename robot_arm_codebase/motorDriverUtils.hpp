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

const float lowerJointLimitSwitchOffset = -0.25;
const float upperJointLimitSwitchOffset = -0.19;
const float lowerJointHome = 0 + lowerJointLimitSwitchOffset;
const float upperJointHome = mot_a(-40) + upperJointLimitSwitchOffset;
// const float upperJointMax = -upperJointHome;
const float lowerJointMax = -4.5;
const float baseJointLimitSwitchOffset = 0.0;
const float baseJointHome = 0 + baseJointLimitSwitchOffset; // ---> base home postion must be 0 towards the Z postion
const float baseJointMax = -4.5;

vector<Moteus> jointMotors;

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

void stopAllMotors()
{
  baseJointMotor.SetStop();
  lowerJointMotor.SetStop();
  upperJointMotor.SetStop();
  wristBaseJointMotor.SetStop();
  wristLowerJointMotor.SetStop();
  wristUpperJointMotor.SetStop();
}

void lockMotor(Moteus &motor, bool useDiagnoseProtocol = true)
{
  if (useDiagnoseProtocol)
  {
    String cmd = "d pos nan 0 nan";
    motor.DiagnosticCommand(cmd);
    return;
  }
  Moteus::PositionMode::Command motor_cmd;
  motor_cmd.velocity = NaN;
  motor_cmd.position = NaN;
  motor_cmd.maximum_torque = NaN;

  motor.SetPosition(motor_cmd);
}

void setMotorPositionVelocity(Moteus &motor, double &position, double &velocity)
{
  Moteus::PositionMode::Command motor_cmd;
  motor_cmd.velocity = NaN;
  motor_cmd.maximum_torque = NaN;
  motor_cmd.position = position;
  motor_cmd.velocity_limit = abs(velocity);
  motor.SetPosition(motor_cmd, &motor_position_fmt, &motor_query_fmt);
}

void setMotorPositionVelocityAccel(Moteus &motor, double &position, double &velocity, double &acceleration)
{
  Moteus::PositionMode::Command motor_cmd;
  motor_cmd.velocity = 0.0;
  motor_cmd.maximum_torque = NaN;
  motor_cmd.position = position;
  motor_cmd.velocity_limit = velocity;
  motor_cmd.accel_limit = acceleration;
  motor.SetPosition(motor_cmd, &motor_position_fmt, &motor_query_fmt);
}

void setMotorVelocity(Moteus &motor, double &velocity)
{
  Moteus::PositionMode::Command motor_cmd;
  motor_cmd.velocity = velocity;
  motor_cmd.maximum_torque = NaN;
  motor_cmd.position = NaN;
  motor.SetPosition(motor_cmd, &motor_position_fmt, &motor_query_fmt);
}