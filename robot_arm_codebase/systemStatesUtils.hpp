#include <math.h>
#include <bitset>

enum SystemStates
{
  BOOTING,
  HOMING,
  IDLE,
  INVERSE_KINEMATICS,
  FORWARD_KINEMATICS,
  IMPEDANCE_CONTROL,
  PROBE,
  SYS_ERR,
  DEV,
};

enum HomingSubStates
{
  // substates of each state/feature
  // during homing we might need other substates to properly home the arm
  BEGIN_HOMING,
  PROBING_UPPER_JOINT_LIMIT,
  PROBING_LOWER_JOINT_LIMIT,
  HOMING_BASE,
  END_HOMING,
};

SystemStates currentSystemState = BOOTING;
HomingSubStates currentHomingState = BEGIN_HOMING;

JointAngle tempAngle;
MotorPosition currentPosition;
MotorPosition currentMotorPosition;
bool mayProceed = false;
double globalVelocity = 0.7, globalAccel = 5;
pair<Coor, vector<Matrix4d>> FK_out;
Coor FK_coor;
MatrixXd J;
JacobiSVD<MatrixXd> J_svd;
Vector3d velocities_last;
Vector3d velocities;
bool FK_motorLock = true;
vector<Coor> motionQuery;
size_t queryIndex = 0;
size_t probeCount = 0;
const size_t probeLimit = 1;
float probVal = 0.0;
int moveDir = 1;
Coor globalUserPos;
Coor globalTraceCoor;
Orientation globalUserOrientation;
Orientation globalTraceOrientation;
Coor tempPos;
Orientation tempOrientation;
Coor diffCoor;

Orientation devOrientation(
    rad(90),
    rad(90),
    0);

void printMotors()
{
  printf("θ1: %1.3f θ2: %1.3f θ3: %1.3f θ4: %1.3f θ5: %1.3f θ6: %1.3f deg\r\n",
         deg(mpos2rad(baseJointMotor.last_result().values.position)),
         deg(mpos2rad(lowerJointMotor.last_result().values.position)),
         deg(mpos2rad(upperJointMotor.last_result().values.position)),
         deg(wmpos2rad(wristBaseJointMotor.last_result().values.position)),
         deg(wmpos2rad(wristLowerJointMotor.last_result().values.position)),
         deg(wmpos2rad(wristUpperJointMotor.last_result().values.position)));
}

void run_homing()
{
  switch (currentHomingState)
  {
  case BEGIN_HOMING:
    initMotors();
    // baseJointMotor.SetBrake();
    lowerJointMotor.SetBrake();
    upperJointMotor.SetBrake();
    wristLowerJointMotor.SetBrake();
    Serial.println(F("Homing..."));
    Serial.println(F("Please move the lower joint to upwards position and make sure the arm can move freely, then press the button."));
    probeCount = 0;
    probVal = 0.0;
    while (usrKey_up)
    {
      rgbLedWrite(RGB_BUILTIN, 50, 50, 0);
      delay(1);
    }

    Serial.println(F("Probing begins once the button is release, be careful!"));

    while (usrKey_down)
    {
      rgbLedWrite(RGB_BUILTIN, 50, 50, 0);
      delay(1);
    }
    baseJointMotor.DiagnosticCommand("conf set motor_position.output.sign -1");
    lowerJointMotor.DiagnosticCommand("conf set motor_position.output.sign -1");
    upperJointMotor.DiagnosticCommand("conf set motor_position.output.sign -1");
    wristBaseJointMotor.DiagnosticCommand("conf set motor_position.output.sign -1");
    wristLowerJointMotor.DiagnosticCommand("conf set motor_position.output.sign -1");
    wristUpperJointMotor.DiagnosticCommand("conf set motor_position.output.sign -1");
    delay(10);
    baseJointMotor.SetStop();
    lowerJointMotor.SetStop();
    upperJointMotor.SetStop();
    wristBaseJointMotor.SetStop();
    wristLowerJointMotor.SetStop();
    wristUpperJointMotor.SetStop();
    delay(10);
    brakeAllMotors();
    currentHomingState = PROBING_UPPER_JOINT_LIMIT;
    // motor_cmd[2].position = upperJointMotor.last_result().values.position;
    Serial.println(F("Probing upper joint..."));
    break;
  case PROBING_UPPER_JOINT_LIMIT:
    rgbLedWrite(RGB_BUILTIN, 0, 20, 20);
    lockMotor(lowerJointMotor);
    motor_cmd[2].velocity = probeCount <= 0 ? -0.6 : -0.1;
    motor_cmd[2].accel_limit = probeCount <= 0 ? 3.0 : 1.0;
    motor_cmd[2].velocity_limit = 3.0;
    motor_cmd[2].position = NaN;
    upperJointMotor.SetPosition(motor_cmd[2]);

    if (upperJointAtLimit && probeCount < probeLimit)
    {
      upperJointMotor.SetBrake();
      Serial.print(F("Hit limit at position: "));
      probVal = upperJointMotor.last_result().values.position;
      Serial.println(probVal);
      motor_cmd[2] = Moteus::PositionMode::Command();
      motor_cmd[2].velocity = NaN;
      motor_cmd[2].accel_limit = 3.0;
      motor_cmd[2].position = (probVal + 0.3);
      upperJointMotor.SetPositionWaitComplete(motor_cmd[2], 0.01, &motor_position_fmt);
      probeCount++;
    }

    else if (upperJointAtLimit && probeCount >= probeLimit)
    {
      lockMotor(upperJointMotor, false);
      delay(10);
      Serial.println(F("Probing upper joint complete!"));
      Serial.print(F("Hit limit at position: "));
      Serial.println(upperJointMotor.last_result().values.position);
      Serial.println("Homing upper joint...");
      delay(5);
      mjbots::moteus::RequireReindex::Command rrcmd;
      upperJointMotor.SetRequireReindex(rrcmd);
      String command = "d exact " + String(upperJointHome);
      upperJointMotor.DiagnosticCommand(command);
      upperJointMotor.SetBrake();
      delay(5);
      motor_cmd[2].velocity = NaN;
      motor_cmd[2].velocity_limit = 4.0;
      motor_cmd[2].position = mot_a(180);
      upperJointMotor.SetPositionWaitComplete(motor_cmd[2], 0.02, &motor_position_fmt);
      upperJointMotor.SetBrake();
      Serial.print(F("This pos is: "));
      Serial.println(upperJointMotor.last_result().values.position);
      currentHomingState = PROBING_LOWER_JOINT_LIMIT;
      lockMotor(upperJointMotor);
      lowerJointMotor.SetBrake();
      probeCount = 0;
    }
    break;
  case PROBING_LOWER_JOINT_LIMIT:

    rgbLedWrite(RGB_BUILTIN, 20, 0, 20);
    motor_cmd[1].velocity = probeCount <= 0 ? -0.6 : -0.1;
    motor_cmd[1].accel_limit = probeCount <= 0 ? 3.0 : 1.0;
    motor_cmd[1].velocity_limit = 3.0;
    motor_cmd[1].position = NaN;
    lowerJointMotor.SetPosition(motor_cmd[1]);

    if (lowerJointAtLimit && probeCount < probeLimit)
    {
      lockMotor(lowerJointMotor, false);
      Serial.print(F("Hit limit at position: "));
      probVal = lowerJointMotor.last_result().values.position;
      Serial.println(probVal);
      Serial.println(F("Homing lower joint..."));
      motor_cmd[1] = Moteus::PositionMode::Command();
      motor_cmd[1].velocity = NaN;
      motor_cmd[1].velocity_limit = 1.0;
      motor_cmd[1].accel_limit = 3.0;
      motor_cmd[1].position = (probVal + 0.3);
      Serial.print(F("Going back to:"));
      Serial.println(motor_cmd[1].position);
      lowerJointMotor.SetPositionWaitComplete(motor_cmd[1], 0.02, &motor_position_fmt);
      probeCount++;
    }
    else if (lowerJointAtLimit && probeCount >= probeLimit)
    {
      lockMotor(lowerJointMotor, false);
      delay(50);
      Serial.println(F("Probing lower joint complete!"));
      Serial.print(F("Hit limit at position: "));
      Serial.println(lowerJointMotor.last_result().values.position);
      Serial.println(F("Homing lower joint..."));
      delay(2);
      mjbots::moteus::RequireReindex::Command rrcmd;
      lowerJointMotor.SetRequireReindex(rrcmd);
      String command = "d exact " + String(lowerJointHome);
      lowerJointMotor.DiagnosticCommand(command);
      lowerJointMotor.SetBrake();
      delay(5);
      motor_cmd[1] = Moteus::PositionMode::Command();
      motor_cmd[1].velocity = NaN;
      motor_cmd[1].position = mot_a(90);
      motor_cmd[1].accel_limit = 1.0;

      motor_cmd[2] = Moteus::PositionMode::Command();
      motor_cmd[2].velocity = NaN;
      motor_cmd[2].position = mot_a(90);
      motor_cmd[2].accel_limit = 3.0;
      Serial.print(F("Going to:"));
      Serial.println(motor_cmd[1].position);
      lowerJointMotor.SetPositionWaitComplete(motor_cmd[1], 0.02, &motor_position_fmt);
      lockMotor(lowerJointMotor);

      upperJointMotor.SetBrake();
      Serial.print(F("Going to:"));
      Serial.println(motor_cmd[2].position);
      upperJointMotor.SetPositionWaitComplete(motor_cmd[2], 0.02, &motor_position_fmt);
      lockMotor(upperJointMotor);
      probeCount = 0;
      Serial.println(F("\r\nPlease rotate the base clockwise until the center point matches the far right position."));
      Serial.println(F("Additionally, straighten joint 5 so its pointing up. Finally, rotate joint 4 so the hexagonal"));
      Serial.println(F("logo aligns with the blue logo behind the joint 3"));
      Serial.println(F("Once in the correct position, press the key."));
      currentHomingState = HOMING_BASE;
    }
    break;
  case HOMING_BASE:
    lockMotor(lowerJointMotor);
    lockMotor(upperJointMotor);

    baseJointMotor.SetStop();
    wristBaseJointMotor.SetStop();
    wristLowerJointMotor.SetStop();
    wristUpperJointMotor.SetStop();

    if (usrKey_down)
    {
      Serial.println(F("The base will begin moving when you let go, be careful!"));
      while (usrKey_down)
      {
        rgbLedWrite(RGB_BUILTIN, 50, 50, 0);
        delay(1);
      }

      probeCount++;
      mjbots::moteus::RequireReindex::Command rrcmd;
      baseJointMotor.SetRequireReindex(rrcmd);
      String command = "d exact " + String(baseJointHome);
      baseJointMotor.DiagnosticCommand(command);
      command = F("d exact 0");
      wristBaseJointMotor.SetRequireReindex(rrcmd);
      wristBaseJointMotor.DiagnosticCommand(command);
      wristLowerJointMotor.SetRequireReindex(rrcmd);
      wristLowerJointMotor.DiagnosticCommand(command);
      wristUpperJointMotor.SetRequireReindex(rrcmd);
      wristUpperJointMotor.DiagnosticCommand(command);
      delay(5);
      baseJointMotor.SetStop();
      wristBaseJointMotor.SetStop();
      wristLowerJointMotor.SetStop();
      wristUpperJointMotor.SetStop();
      delay(1100);
    }

    if (probeCount >= probeLimit) // baseAtLimit --> add new switch for homing
    {
      motor_cmd[0] = Moteus::PositionMode::Command();
      motor_cmd[0].accel_limit = 3.0;
      motor_cmd[0].velocity = NaN;
      motor_cmd[0].position = mot_a(90);
      baseJointMotor.SetPositionWaitComplete(motor_cmd[0], 0.02, &motor_position_fmt);
      baseJointMotor.SetStop();
      lowerJointMotor.SetStop();
      upperJointMotor.SetStop();
      currentHomingState = END_HOMING;
    }

    break;
  case END_HOMING:
    Serial.println(F("Homing Complete!"));
    isHomed = true;
    currentSystemState = IDLE;
    currentHomingState = BEGIN_HOMING;
    // Perhaps move to a safe position and end homing there?
    break;
  default:
    NOT_IMPLEMENTED();
    break;
  }
}

void system_run()
{
  switch (currentSystemState)
  {
  case BOOTING:
    delay(1);
    rgbLedWrite(RGB_BUILTIN, 0, 0, 50);
    delay(1);
    while (usrKey_up)
      checkSerial(); // wait for key press then begin

    while (usrKey_down)
    {
      rgbLedWrite(RGB_BUILTIN, 50, 50, 0);
      delay(1);
    }
    digitalWrite(RGB_BUILTIN, LOW);
    currentSystemState = IDLE;
    delay(1000);

    globalUserPos.x = 367.569;
    globalUserPos.y = 321.096;
    globalUserPos.z = -362.296;

    globalUserOrientation.phi = rad(134.6);
    globalUserOrientation.theta = rad(56);
    globalUserOrientation.psi = 0;

    globalTraceCoor.x = 0;
    globalTraceCoor.y = 0;
    globalTraceCoor.z = 0;

    globalTraceOrientation.phi = 0;
    globalTraceOrientation.theta = 0;
    globalTraceOrientation.psi = 0;

    // all of this here is quite dangerous but there is no better way to initialize the values
    tempPos = globalUserPos;
    tempOrientation = globalUserOrientation;
    mayProceed = IK(tempPos, tempOrientation, &tempAngle);
    if (mayProceed)
      currentPosition = tempAngle.toMotorPosition();

    velocities_last = Vector3d{0, 0, 0};

    break;

  case HOMING:
    run_homing();
    break;

  case IDLE:
    baseJointMotor.SetBrake();
    upperJointMotor.SetBrake();
    lowerJointMotor.SetBrake();
    wristBaseJointMotor.SetBrake();
    wristLowerJointMotor.SetBrake();
    wristUpperJointMotor.SetBrake();

    printMotors();

    break;

  case INVERSE_KINEMATICS:
  {
    tempPos.x += globalTraceCoor.x;
    tempPos.y += globalTraceCoor.y;
    tempPos.z += globalTraceCoor.z;

    tempOrientation.phi += globalTraceOrientation.phi;
    tempOrientation.theta += globalTraceOrientation.theta;
    tempOrientation.psi += globalTraceOrientation.psi;

    IKSolution newSolutions = solveFullIK(tempPos, tempOrientation, &tempAngle);
    // mayProceed = IK(tempPos, tempOrientation, &tempAngle);
    mayProceed = (newSolutions.validationFlags.bits > 0);

    if (baseJointMotor.last_result().values.trajectory_complete &&
        lowerJointMotor.last_result().values.trajectory_complete &&
        upperJointMotor.last_result().values.trajectory_complete &&
        wristBaseJointMotor.last_result().values.trajectory_complete &&
        wristLowerJointMotor.last_result().values.trajectory_complete &&
        wristUpperJointMotor.last_result().values.trajectory_complete &&
        mayProceed)
    {
      currentPosition = tempAngle.toMotorPosition();
      globalUserOrientation = tempOrientation;
      globalUserPos = tempPos;
    }

    else
    {
      tempPos = globalUserPos;
      tempOrientation = globalUserOrientation;
    }

    motor_cmd[0] = Moteus::PositionMode::Command();
    motor_cmd[0].position = currentPosition.pos1;
    motor_cmd[0].velocity = 0.0;
    motor_cmd[0].maximum_torque = NaN;
    motor_cmd[0].velocity_limit = globalVelocity;
    motor_cmd[0].accel_limit = globalAccel;
    baseJointMotor.SetPosition(motor_cmd[0], &motor_position_fmt, &motor_query_fmt);

    motor_cmd[1] = Moteus::PositionMode::Command();
    motor_cmd[1].position = currentPosition.pos2;
    motor_cmd[1].velocity = 0.0;
    motor_cmd[1].maximum_torque = NaN;
    motor_cmd[1].velocity_limit = globalVelocity;
    motor_cmd[1].accel_limit = globalAccel;
    lowerJointMotor.SetPosition(motor_cmd[1], &motor_position_fmt, &motor_query_fmt);

    motor_cmd[2] = Moteus::PositionMode::Command();
    motor_cmd[2].position = currentPosition.pos3;
    motor_cmd[2].velocity = 0.0;
    motor_cmd[2].maximum_torque = NaN;
    motor_cmd[2].velocity_limit = globalVelocity;
    motor_cmd[2].accel_limit = globalAccel;
    upperJointMotor.SetPosition(motor_cmd[2], &motor_position_fmt, &motor_query_fmt);

    motor_cmd[3] = Moteus::PositionMode::Command();
    motor_cmd[3].position = currentPosition.pos4;
    motor_cmd[3].velocity = 0.0;
    motor_cmd[3].maximum_torque = NaN;
    motor_cmd[3].velocity_limit = globalVelocity;
    motor_cmd[3].accel_limit = globalAccel;
    wristBaseJointMotor.SetPosition(motor_cmd[3], &motor_position_fmt, &motor_query_fmt);

    motor_cmd[4] = Moteus::PositionMode::Command();
    motor_cmd[4].position = currentPosition.pos5;
    motor_cmd[4].velocity = 0.0;
    motor_cmd[4].maximum_torque = NaN;
    motor_cmd[4].velocity_limit = globalVelocity;
    motor_cmd[4].accel_limit = globalAccel;
    wristLowerJointMotor.SetPosition(motor_cmd[4], &motor_position_fmt, &motor_query_fmt);

    motor_cmd[5] = Moteus::PositionMode::Command();
    motor_cmd[5].position = currentPosition.pos6;
    motor_cmd[5].velocity = 0.0;
    motor_cmd[5].maximum_torque = NaN;
    motor_cmd[5].velocity_limit = globalVelocity;
    motor_cmd[5].accel_limit = globalAccel;
    wristUpperJointMotor.SetPosition(motor_cmd[5], &motor_position_fmt, &motor_query_fmt);

    currentMotorPosition = MotorPosition(
        baseJointMotor.last_result().values.position,
        lowerJointMotor.last_result().values.position,
        upperJointMotor.last_result().values.position,
        wristBaseJointMotor.last_result().values.position,
        wristLowerJointMotor.last_result().values.position,
        wristUpperJointMotor.last_result().values.position);

    FK_out = FK(currentMotorPosition.toJointAngle());
    FK_coor = FK_out.first;

    Serial.printf("x:% 3.3f y:% 3.3f z:% 3.3f │ FK x:% 3.3f y:% 3.3f z:% 3.3f │ orientation phi:% 3.3f theta:% 3.3f psi:% 3.3f │ trajectory_complete: [%1d, %1d, %1d, %1d, %1d, %1d]\r\n",
                  globalUserPos.x,
                  globalUserPos.y,
                  globalUserPos.z,
                  FK_coor.x,
                  FK_coor.y,
                  FK_coor.z,
                  deg(globalUserOrientation.phi),
                  deg(globalUserOrientation.theta),
                  deg(globalUserOrientation.psi),
                  baseJointMotor.last_result().values.trajectory_complete,
                  lowerJointMotor.last_result().values.trajectory_complete,
                  upperJointMotor.last_result().values.trajectory_complete,
                  wristBaseJointMotor.last_result().values.trajectory_complete,
                  wristLowerJointMotor.last_result().values.trajectory_complete,
                  wristUpperJointMotor.last_result().values.trajectory_complete);

    delay(updteInterval);
    break;
  }

  case FORWARD_KINEMATICS:
  {
    if (FK_motorLock)
    {
      baseJointMotor.SetBrake();
      lowerJointMotor.SetBrake();
      upperJointMotor.SetBrake();
      wristBaseJointMotor.SetBrake();
      wristLowerJointMotor.SetBrake();
      wristUpperJointMotor.SetBrake();
    }
    else
    {
      baseJointMotor.SetStop();
      lowerJointMotor.SetStop();
      upperJointMotor.SetStop();
      wristBaseJointMotor.SetStop();
      wristLowerJointMotor.SetStop();
      wristUpperJointMotor.SetStop();
    }

    currentMotorPosition = MotorPosition(
        baseJointMotor.last_result().values.position,
        lowerJointMotor.last_result().values.position,
        upperJointMotor.last_result().values.position,
        wristBaseJointMotor.last_result().values.position,
        wristLowerJointMotor.last_result().values.position,
        wristUpperJointMotor.last_result().values.position);

    FK_out = FK(currentMotorPosition.toJointAngle());
    FK_coor = FK_out.first;
    J = createJacobianMatrix(FK_out.second);
    J_svd.compute(J, ComputeThinU | ComputeThinV);
    auto singular_out = IsSingular(J);
    bool isAtSingularity = singular_out.first;

    Serial.printf("x:% 3.3f y:% 3.3f z:% 3.3f │ θ1: %1.3f θ2: %1.3f θ3: %1.3f θ4: %1.3f θ5: %1.3f θ6: %1.3f deg | ∞: %d │ rank: %d | J11 det: % 10.8f J22 det:% .20G\r\n",
                  FK_coor.x,
                  FK_coor.y,
                  FK_coor.z,
                  deg(mpos2rad(baseJointMotor.last_result().values.position)),
                  deg(mpos2rad(lowerJointMotor.last_result().values.position)),
                  deg(mpos2rad(upperJointMotor.last_result().values.position)),
                  deg(wmpos2rad(wristBaseJointMotor.last_result().values.position)),
                  deg(wmpos2rad(wristLowerJointMotor.last_result().values.position)),
                  deg(wmpos2rad(wristUpperJointMotor.last_result().values.position)),
                  isAtSingularity,
                  J_svd.rank(),
                  singular_out.second.first,
                  singular_out.second.second);

    rgbLedWrite(RGB_BUILTIN, 255 * isAtSingularity, 0, 0);

    break;
  }

  case PROBE:
    if (FK_motorLock)
    {
      baseJointMotor.SetBrake();
      lowerJointMotor.SetBrake();
      upperJointMotor.SetBrake();
      wristBaseJointMotor.SetBrake();
      wristLowerJointMotor.SetBrake();
    }
    else
    {
      baseJointMotor.SetStop();
      lowerJointMotor.SetStop();
      upperJointMotor.SetStop();
      wristBaseJointMotor.SetStop();
      wristLowerJointMotor.SetStop();
    }

    currentMotorPosition = MotorPosition(
        baseJointMotor.last_result().values.position,
        lowerJointMotor.last_result().values.position,
        upperJointMotor.last_result().values.position,
        wristBaseJointMotor.last_result().values.position,
        wristLowerJointMotor.last_result().values.position,
        wristUpperJointMotor.last_result().values.position);

    FK_coor = FK_precise(currentMotorPosition.toJointAngle());

    Serial.printf("x:% 3.3f y:% 3.3f z:% 3.3f | dist: %f\r\n",
                  abs(diffCoor.x - FK_coor.x),
                  abs(diffCoor.y - FK_coor.y),
                  abs(diffCoor.z - FK_coor.z),
                  abs(sqrt(pow((diffCoor.x - FK_coor.x), 2) + pow((diffCoor.y - FK_coor.y), 2) + pow((diffCoor.z - FK_coor.z), 2))));

    if (usrKey_down)
    {
      Serial.println(F("Release key to set origin."));
      rgbLedWrite(RGB_BUILTIN, 0, 0, 100);
      while (usrKey_down)
        ;
      diffCoor = FK_coor;
      digitalWrite(RGB_BUILTIN, LOW);
    }
    break;

  case DEV:
  {
    devOrientation.theta += rad((moveDir * 0.1));
    // devOrientation.phi += rad((-moveDir * 0.1));

    // TODO: Make the ball rotate around itself -> this would be a starting point:
    // double thet = constrain(rad(30) + sin(millis() * 0.0001), rad(30), rad(100));
    // double ph = constrain(rad(90) + cos(millis() * 0.0001), rad(90), rad(180));
    // printf("theta:% .5f phi:% .5f t2:% .5f p2:% .5f\r\n", deg(thet), deg(ph), deg(devOrientation.theta), deg(devOrientation.phi));

    if (devOrientation.theta > rad(180))
      moveDir = -1;
    else if (devOrientation.theta <= rad(60))
      moveDir = 1;

    Coor newIKCoor(
        410,
        215,
        0);

    JointAngle IKOut;
    IKSolution fullIKSolution = solveFullIK(newIKCoor, devOrientation, &IKOut);

    currentPosition = IKOut.toMotorPosition();
    baseJointMotor.SetBrake();

    currentMotorPosition = MotorPosition(
        baseJointMotor.last_result().values.position,
        lowerJointMotor.last_result().values.position,
        upperJointMotor.last_result().values.position,
        wristBaseJointMotor.last_result().values.position,
        wristLowerJointMotor.last_result().values.position,
        wristUpperJointMotor.last_result().values.position);

    FK_out = FK(currentMotorPosition.toJointAngle());
    FK_coor = FK_out.first;
    J = createJacobianMatrix(FK_out.second);
    auto singular_out = IsSingular(J);
    bool isAtSingularity = singular_out.first;

    VectorXd velocities(6);
    velocities << globalVelocity,
        globalVelocity,
        globalVelocity,
        globalVelocity,
        globalVelocity,
        globalVelocity;

    velocities = getJointVelocities(currentMotorPosition.toJointAngle(), IKOut, 6);

    motor_cmd[1] = Moteus::PositionMode::Command();
    motor_cmd[1].position = currentPosition.pos2;
    motor_cmd[1].velocity = NaN;
    motor_cmd[1].maximum_torque = NaN;
    motor_cmd[1].velocity_limit = abs(velocities(1));
    lowerJointMotor.SetPosition(motor_cmd[1], &motor_position_fmt, &motor_query_fmt);

    motor_cmd[2] = Moteus::PositionMode::Command();
    motor_cmd[2].position = currentPosition.pos3;
    motor_cmd[2].velocity = NaN;
    motor_cmd[2].maximum_torque = NaN;
    motor_cmd[2].velocity_limit = abs(velocities(2));
    ;
    upperJointMotor.SetPosition(motor_cmd[2], &motor_position_fmt, &motor_query_fmt);

    motor_cmd[3] = Moteus::PositionMode::Command();
    motor_cmd[3].position = currentPosition.pos4;
    motor_cmd[3].velocity = NaN; // abs(velocities(3));
    motor_cmd[3].maximum_torque = NaN;
    motor_cmd[3].velocity_limit = abs(velocities(3));
    wristBaseJointMotor.SetPosition(motor_cmd[3], &motor_position_fmt, &motor_query_fmt);

    motor_cmd[4] = Moteus::PositionMode::Command();
    motor_cmd[4].position = currentPosition.pos5;
    motor_cmd[4].velocity = NaN;
    motor_cmd[4].maximum_torque = NaN;
    motor_cmd[4].velocity_limit = abs(velocities(4));
    wristLowerJointMotor.SetPosition(motor_cmd[4], &motor_position_fmt, &motor_query_fmt);

    motor_cmd[5] = Moteus::PositionMode::Command();
    motor_cmd[5].position = currentPosition.pos6;
    motor_cmd[5].velocity = NaN;
    motor_cmd[5].maximum_torque = NaN;
    motor_cmd[5].velocity_limit = abs(velocities(5));
    wristUpperJointMotor.SetPosition(motor_cmd[5], &motor_position_fmt, &motor_query_fmt);

    Serial.printf("x:% 3.3f y:% 3.3f z:% 3.3f │ t0: %1.3f t1: %1.3f t2: %1.3f t3: %1.3f t4: %1.3f t5: %1.3f | phi:% 1.3f theta:% 1.3f psi:% 1.3f | %s | ∞: %d \r\n",
                  FK_coor.x,
                  FK_coor.y,
                  FK_coor.z,
                  // deg(IKOut.theta1),
                  // deg(IKOut.theta2),
                  // deg(IKOut.theta3),
                  // deg(IKOut.theta4),
                  // deg(IKOut.theta5),
                  // deg(IKOut.theta6),
                  velocities(0),
                  velocities(1),
                  velocities(2),
                  velocities(3),
                  velocities(4),
                  velocities(5),
                  deg(devOrientation.phi),
                  deg(devOrientation.theta),
                  deg(devOrientation.psi),
                  "nan", // bitset<8>(fullIKSolution.validationFlags.bits).to_string().c_str(),
                  isAtSingularity);

    delay(updteInterval);
    break;
  }

  case IMPEDANCE_CONTROL:
    NOT_IMPLEMENTED();
    break;

  case SYS_ERR:
    NOT_IMPLEMENTED();
    break;
  }
}

void NOT_IMPLEMENTED()
{
  brakeAllMotors();
  while (1)
  {
    for (int i = 0; i <= 255; i++)
    {
      rgbLedWrite(RGB_BUILTIN, 255 - i, i, 0);
      delay(10);
    }

    for (int i = 0; i <= 255; i++)
    {
      rgbLedWrite(RGB_BUILTIN, 0, 255 - i, i);
      delay(10);
    }

    for (int i = 0; i <= 255; i++)
    {
      rgbLedWrite(RGB_BUILTIN, i, 0, 255 - i);
      delay(10);
    }

    rgbLedWrite(RGB_BUILTIN, 255, 0, 0);
    delay(100);
    rgbLedWrite(RGB_BUILTIN, 0, 255, 0);
    delay(100);
    rgbLedWrite(RGB_BUILTIN, 0, 0, 255);
    delay(100);

    for (int i = 0; i < 3; i++)
    {
      rgbLedWrite(RGB_BUILTIN, 255, 255, 255);
      delay(50);
      rgbLedWrite(RGB_BUILTIN, 0, 0, 0);
      delay(50);
    }

    for (int i = 0; i < 5; i++)
    {
      rgbLedWrite(RGB_BUILTIN, random(0, 256), random(0, 256), random(0, 256));
      delay(200);
    }
  }
}