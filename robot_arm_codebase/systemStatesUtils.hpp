#include <math.h>

Coor globalUserPos;

enum SystemStates
{
  BOOTING,
  HOMING,
  IDLE,
  MOVING,
  HOLDING,
  LEARNING,
  DIFF,
  SYS_ERR,
  DEV,
  PID_TUNNING,
};

enum HomingSubStates
{
  // substates of each state/feature
  // during homing i.g. we might need other substates to properly home the arm
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
Coor globalTraceCoor;
Coor tempPos;
bool mayProceed = false;
double globalVelocity = 0.7, globalAccel = 5;
pair<Coor, vector<Matrix4d>> FK_out;
Coor FK_coor;
Coor diffCoor;
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

void initMotions()
{
  Coor pos1;
  Coor pos2;

  pos1.x = 195;
  pos1.y = 60;
  pos1.z = 0;

  pos2.x = 100;
  pos2.y = 60;
  pos2.z = -350;

  motionQuery.push_back(pos1);
  motionQuery.push_back(pos2);
}

void printMotors()
{
  printf("θ1: %1.3f θ2: %1.3f θ3: %1.3f θ4: %1.3f θ5: %1.3f θ6: %1.3f deg │ trajectory_complete: [%1d, %1d, %1d, %1d, %1d, %1d]\r\n",
         deg(mpos2rad(baseJointMotor.last_result().values.position)),
         deg(mpos2rad(lowerJointMotor.last_result().values.position)),
         deg(mpos2rad(upperJointMotor.last_result().values.position)),
         deg(wmpos2rad(wristBaseJointMotor.last_result().values.position)),
         deg(wmpos2rad(wristLowerJointMotor.last_result().values.position)),
         deg(wmpos2rad(wristUpperJointMotor.last_result().values.position)),
         baseJointMotor.last_result().values.trajectory_complete,
         lowerJointMotor.last_result().values.trajectory_complete,
         upperJointMotor.last_result().values.trajectory_complete,
         wristBaseJointMotor.last_result().values.trajectory_complete,
         wristLowerJointMotor.last_result().values.trajectory_complete,
         wristUpperJointMotor.last_result().values.trajectory_complete);
}

void run_homing()
{
  switch (currentHomingState)
  {
  case BEGIN_HOMING:
    initMotors();
    // baseJointMotor.SetBrake();
    lowerJointMotor.SetBrake();
    Serial.println("Homing...");
    Serial.println("Please move the lower joint to upwards position and make sure the arm can move freely, then press the button.");
    probeCount = 0;
    probVal = 0.0;
    while (usrKey_up)
    {
      rgbLedWrite(RGB_BUILTIN, 50, 50, 0);
      delay(1);
    }

    Serial.println("Probing begins once the button is release, be careful!");

    while (usrKey_down)
    {
      rgbLedWrite(RGB_BUILTIN, 50, 50, 0);
      delay(1);
    }
    currentHomingState = PROBING_UPPER_JOINT_LIMIT;
    motor_cmd[2].position = upperJointMotor.last_result().values.position;
    Serial.println("Probing upper joint...");
    break;
  case PROBING_UPPER_JOINT_LIMIT:
    rgbLedWrite(RGB_BUILTIN, 0, 20, 20);

    motor_cmd[2].velocity = probeCount <= 0 ? 0.6 : 0.1;
    motor_cmd[2].accel_limit = probeCount <= 0 ? 3.0 : 1.0;
    motor_cmd[2].velocity_limit = 3.0;
    motor_cmd[2].position = NaN;
    upperJointMotor.SetPosition(motor_cmd[2]);

    if (upperJointAtLimit && probeCount < probeLimit)
    {
      upperJointMotor.SetBrake();
      Serial.print("Hit limit at position: ");
      probVal = upperJointMotor.last_result().values.position;
      Serial.println(probVal);
      motor_cmd[2].velocity = NaN;
      motor_cmd[2].accel_limit = 3.0;
      motor_cmd[2].position = (probVal - 0.3);
      upperJointMotor.SetPositionWaitComplete(motor_cmd[2], 0.02, &motor_position_fmt);
      probeCount++;
    }

    else if (upperJointAtLimit && probeCount >= probeLimit)
    {
      upperJointMotor.SetBrake(); // use d pos nan 0 nan
      delay(50);
      Serial.println("Probing upper joint complete!");
      Serial.print("Hit limit at position: ");
      Serial.println(upperJointMotor.last_result().values.position);
      Serial.println("Homing upper joint...");
      mjbots::moteus::RequireReindex::Command rrcmd;
      upperJointMotor.SetRequireReindex(rrcmd);
      String command = "d exact " + String(upperJointHome);
      upperJointMotor.DiagnosticCommand(command);
      motor_cmd[2].velocity = NaN;
      motor_cmd[2].velocity_limit = 4.0;
      motor_cmd[2].position = -2.25;
      upperJointMotor.SetPositionWaitComplete(motor_cmd[2], 0.02, &motor_position_fmt);
      upperJointMotor.SetBrake();
      Serial.print("This pos is: ");
      Serial.println(upperJointMotor.last_result().values.position);
      currentHomingState = PROBING_LOWER_JOINT_LIMIT;
      probeCount = 0;
    }
    break;
  case PROBING_LOWER_JOINT_LIMIT:

    rgbLedWrite(RGB_BUILTIN, 20, 0, 20);
    motor_cmd[1].velocity = probeCount <= 0 ? 0.6 : 0.1;
    motor_cmd[1].accel_limit = probeCount <= 0 ? 3.0 : 1.0;
    motor_cmd[1].velocity_limit = 3.0;
    motor_cmd[1].position = NaN;
    lowerJointMotor.SetPosition(motor_cmd[1]);

    if (lowerJointAtLimit && probeCount < probeLimit)
    {
      lowerJointMotor.SetBrake();
      Serial.print("Hit limit at position: ");
      probVal = lowerJointMotor.last_result().values.position;
      Serial.println(probVal);
      Serial.println("Homing lower joint...");
      motor_cmd[1] = Moteus::PositionMode::Command();
      motor_cmd[1].velocity = NaN;
      motor_cmd[1].velocity_limit = 1.0;
      motor_cmd[1].accel_limit = 3.0;
      motor_cmd[1].position = (probVal - 0.3);
      lowerJointMotor.SetPositionWaitComplete(motor_cmd[1], 0.02, &motor_position_fmt);
      probeCount++;
    }
    else if (lowerJointAtLimit && probeCount >= probeLimit)
    {
      lowerJointMotor.SetBrake();
      delay(50);
      Serial.println("Probing lower joint complete!");
      Serial.print("Hit limit at position: ");
      Serial.println(lowerJointMotor.last_result().values.position);
      Serial.println("Homing lower joint...");
      mjbots::moteus::RequireReindex::Command rrcmd;
      lowerJointMotor.SetRequireReindex(rrcmd);
      String command = "d exact " + String(lowerJointHome);
      lowerJointMotor.DiagnosticCommand(command);
      motor_cmd[1] = Moteus::PositionMode::Command();
      motor_cmd[1].velocity = NaN;
      motor_cmd[1].position = lowerJointMax / 2;
      motor_cmd[1].accel_limit = 1.0;

      motor_cmd[2] = Moteus::PositionMode::Command();
      motor_cmd[2].velocity = NaN;
      motor_cmd[2].position = 0;
      motor_cmd[2].accel_limit = 3.0;
      lowerJointMotor.SetPositionWaitComplete(motor_cmd[1], 0.02, &motor_position_fmt);
      lowerJointMotor.SetBrake();

      upperJointMotor.SetPositionWaitComplete(motor_cmd[2], 0.02, &motor_position_fmt);
      upperJointMotor.SetBrake();
      probeCount = 0;
      Serial.println("\r\nPlease rotate the base clockwise until the center point matches the far right position.");
      Serial.println("Additionally, make sure to put the wrist joints at their corresponding home position.");
      Serial.println("Once in the correct position, press the key.");
      currentHomingState = HOMING_BASE;
    }
    break;
  case HOMING_BASE:
    motor_cmd[1] = Moteus::PositionMode::Command();
    motor_cmd[1].velocity = NaN;
    motor_cmd[1].position = lowerJointMax / 2;
    ;
    motor_cmd[1].maximum_torque = NaN;

    motor_cmd[2] = Moteus::PositionMode::Command();
    motor_cmd[2].velocity = NaN;
    motor_cmd[2].position = 0;
    motor_cmd[2].maximum_torque = NaN;
    lowerJointMotor.SetPosition(motor_cmd[1]);
    upperJointMotor.SetPosition(motor_cmd[2]);

    baseJointMotor.SetStop();

    if (usrKey_down)
    {
      Serial.println("The base will begin moving when you let go, be careful!");
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
      command = "d exact 0";
      wristBaseJointMotor.SetRequireReindex(rrcmd);
      wristBaseJointMotor.DiagnosticCommand(command);
      wristLowerJointMotor.SetRequireReindex(rrcmd);
      wristLowerJointMotor.DiagnosticCommand(command);
      wristUpperJointMotor.SetRequireReindex(rrcmd);
      wristUpperJointMotor.DiagnosticCommand(command);
      delay(1100);
    }

    if (probeCount >= probeLimit) // baseAtLimit --> add new switch for homing
    {
      motor_cmd[0] = Moteus::PositionMode::Command();
      motor_cmd[0].accel_limit = 3.0;
      motor_cmd[0].velocity = NaN;
      motor_cmd[0].position = baseJointMax / 2;
      baseJointMotor.SetPositionWaitComplete(motor_cmd[0], 0.02, &motor_position_fmt);
      baseJointMotor.SetBrake();

      baseJointMotor.SetStop();
      lowerJointMotor.SetStop();
      upperJointMotor.SetStop();
      currentHomingState = END_HOMING;
    }

    break;
  case END_HOMING:
    Serial.println("Homing Complete!");
    isHomed = true;
    currentSystemState = IDLE;
    currentHomingState = BEGIN_HOMING;
    baseJointMotor.SetBrake();
    lowerJointMotor.SetBrake();
    upperJointMotor.SetBrake();
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

    // globalUserPos.x = 195;
    // globalUserPos.y = 60;
    // globalUserPos.z = -135;

    globalUserPos.x = 280;
    globalUserPos.y = 100;
    globalUserPos.z = -200;

    globalTraceCoor.x = 0;
    globalTraceCoor.y = 0;
    globalTraceCoor.z = 0;

    velocities_last = Vector3d{0, 0, 0};

    initMotions();

    break;

  case HOMING:
    run_homing();
    break;

  case IDLE:
    baseJointMotor.Poll();
    upperJointMotor.Poll();
    lowerJointMotor.Poll();
    wristBaseJointMotor.Poll();
    wristLowerJointMotor.Poll();
    wristUpperJointMotor.Poll();

    printMotors();

    break;

  case MOVING:
  {
    tempPos.x += globalTraceCoor.x;
    tempPos.y += globalTraceCoor.y;
    tempPos.z += globalTraceCoor.z;

    mayProceed = IK(tempPos, &tempAngle);

    if (baseJointMotor.last_result().values.trajectory_complete &&
        lowerJointMotor.last_result().values.trajectory_complete &&
        upperJointMotor.last_result().values.trajectory_complete &&
        mayProceed)
    {
      currentPosition = tempAngle.toMotorPosition();
      globalUserPos = tempPos;
    }

    else
      tempPos = globalUserPos;
    //  globalTraceCoor.y = -globalTraceCoor.y;

    motor_cmd[0] = Moteus::PositionMode::Command();
    motor_cmd[0].position = currentPosition.pos1;
    motor_cmd[0].velocity = 0.0;
    motor_cmd[0].maximum_torque = std::numeric_limits<float>::quiet_NaN();
    motor_cmd[0].velocity_limit = globalVelocity;
    motor_cmd[0].accel_limit = globalAccel;
    baseJointMotor.SetPosition(motor_cmd[0], &motor_position_fmt, &motor_query_fmt);

    motor_cmd[1] = Moteus::PositionMode::Command();
    motor_cmd[1].position = currentPosition.pos2;
    motor_cmd[1].velocity = 0.0;
    motor_cmd[1].maximum_torque = std::numeric_limits<float>::quiet_NaN();
    motor_cmd[1].velocity_limit = globalVelocity;
    motor_cmd[1].accel_limit = globalAccel;
    lowerJointMotor.SetPosition(motor_cmd[1], &motor_position_fmt, &motor_query_fmt);

    motor_cmd[2] = Moteus::PositionMode::Command();
    motor_cmd[2].position = currentPosition.pos3;
    motor_cmd[2].velocity = 0.0;
    motor_cmd[2].maximum_torque = std::numeric_limits<float>::quiet_NaN();
    motor_cmd[2].velocity_limit = globalVelocity;
    motor_cmd[2].accel_limit = globalAccel;
    upperJointMotor.SetPosition(motor_cmd[2], &motor_position_fmt, &motor_query_fmt);

    motor_cmd[5] = Moteus::PositionMode::Command();
    motor_cmd[5].position = std::numeric_limits<float>::quiet_NaN();
    motor_cmd[5].velocity = 1;
    wristUpperJointMotor.SetPosition(motor_cmd[5], &motor_position_fmt, &motor_query_fmt);

    wristBaseJointMotor.SetBrake();
    wristLowerJointMotor.SetBrake();

    currentMotorPosition = MotorPosition(
        baseJointMotor.last_result().values.position,
        lowerJointMotor.last_result().values.position,
        upperJointMotor.last_result().values.position,
        wristBaseJointMotor.last_result().values.position,
        wristLowerJointMotor.last_result().values.position,
        wristUpperJointMotor.last_result().values.position);

    FK_out = FK(currentMotorPosition.toJointAngle());
    FK_coor = FK_out.first;

    Serial.printf("x:% 3.3f y:% 3.3f z:% 3.3f │ FK x:% 3.3f y:% 3.3f z:% 3.3f\r\n",
                  globalUserPos.x,
                  globalUserPos.y,
                  globalUserPos.z,
                  FK_coor.x,
                  FK_coor.y,
                  FK_coor.z);
    // J = createJacobianMatrix(FK_out.second);
    // VectorXd v{{0,   // linear vx
    //             100, // linear vy
    //             0,   // linear vz
    //             0,   // angular vx
    //             0,   // angular vy
    //             0}}; // angular vz

    // // units are messed up, values are too small! fix this
    // velocities = getJointVelocities(J, v); // devide by 2*M_PI for rev/sec
    // velocities /= 2 * M_PI;

    // Vector3d deltaV = (velocities_last - velocities);
    // Serial.printf("x:% 3.3f y:% 3.3f z:% 3.3f │ FK x:% 3.3f y:% 3.3f z:% 3.3f │ JVel v1:% 1.5f v2:% 1.5f v3:% 1.5f │ MVel v1:% 1.5f v2:% 1.5f v3:% 1.5f \r\n",
    //               globalUserPos.x,
    //               globalUserPos.y,
    //               globalUserPos.z,
    //               FK_coor.x,
    //               FK_coor.y,
    //               FK_coor.z,
    //               deltaV(0),
    //               deltaV(1),
    //               deltaV(2),
    //               baseJointMotor.last_result().values.velocity,
    //               lowerJointMotor.last_result().values.velocity,
    //               upperJointMotor.last_result().values.velocity);
    // // printMotors();
    // // printJacobian();
    // velocities_last = velocities;
    delay(updteInterval);
    break;
  }

  case HOLDING:
  {
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

  case LEARNING:

    NOT_IMPLEMENTED();
    break;

  case SYS_ERR:
    NOT_IMPLEMENTED();
    break;

  case DEV:
    break;

  case DIFF:
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
      Serial.println("Release key to set origin.");
      rgbLedWrite(RGB_BUILTIN, 0, 0, 100);
      while (usrKey_down)
        ;
      diffCoor = FK_coor;
      digitalWrite(RGB_BUILTIN, LOW);
    }
    break;

  case PID_TUNNING:
    // pidTunningConfigPlot(*selectedMotorOBJ);
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