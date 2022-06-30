#include <stdio.h>
#include <string.h>
#include "iostream"
#include <sstream>

#include <msgpack.hpp>

#include <os_generic.h>

#include <assert.h>
#include <signal.h>
#include <stdlib.h>

#include <sw/redis++/redis++.h>

#include "utils/LoopTimer.h"

#include <frc/MathUtil.h>

#include <frc/controller/PIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/controller/HolonomicDriveController.h>

#include <frc/kinematics/MecanumDriveKinematics.h>
#include <frc/kinematics/MecanumDriveWheelSpeeds.h>

#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/trajectory/TrapezoidProfile.h>

#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>

#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/math.h>

#include <wpi/numbers>
#include <wpi/json.h>

#include <math.h>

#include <chrono>

#include <inttypes.h>

#include <survive.h>

#include <cpr/cpr.h>

using namespace std;
using namespace sw::redis;
using namespace std::chrono;

using json = wpi::json;

using radians_per_second_squared_t =
    units::compound_unit<units::radians,
                         units::inverse<units::squared<units::second>>>;

const string STARTUP_TIMESTAMP_KEY ="rover_startup_timestamp";

const string POSE_DATA_KEY = "rover_pose";
const string POSE_VELOCITY_DATA_KEY = "rover_pose_velocity";
const string WHEEL_VELOCITY_COMMAND_KEY = "rover_wheel_velocity_command";

const string WHEEL_VOLTAGE_COMMAND_KEY = "rover_wheel_voltage_command";
const string WHEEL_STATUS_KEY = "rover_wheel_status";

const string TRAJECTORY_PROFILE_KEY = "rover_trajectory_profile";
const string TRAJECTORY_SAMPLE_KEY = "rover_trajectory_sample";
const string RADAR_SAMPLE_POINT_KEY = "radar_sample_point";

const string PARAMETERS_KEY = "rover_parameters";
const string TRAJECTORY_KEY = "rover_trajectory";
const string COMMAND_KEY = "rover_command";
const string CONTROLLER_KEY = "rover_controller";

const string CONTROLLER_STATE_KEY = "rover_control_state";

const string STOP_WHEELS_COMMAND_KEY = "STOP_WHEELS";
const string RUN_TRAJECTORY_COMMAND_KEY = "RUN_ACTIVE_TRAJECTORY";

struct PoseMessage {
  FLT timestamp = 0.0;

  std::array<FLT, 3> pos = {0.0, 0.0, 0.0};
  std::array<FLT, 4> rot = {0.0, 0.0, 0.0, 0.0};

  MSGPACK_DEFINE_MAP(timestamp, pos, rot)
};

struct VelocityMessage {
  FLT timestamp;

  std::array<FLT, 3> pos   = {0.0, 0.0, 0.0};
  std::array<FLT, 3> theta = {0.0, 0.0, 0.0};

  MSGPACK_DEFINE_MAP(timestamp, pos, theta)
};

struct ControllerStateMessage {
  ControllerStateMessage(
    int64_t _timestamp,
    
    double _targetChassisVelocityX,
    double _targetChassisVelocityY,

    FLT _velocityX,
    FLT _velocityY,


    double _xPositionError,
    double _xVelocityError,
    double _yPositionError,
    double _yVelocityError,
    double _thetaPositionError,
    double _thetaVelocityError,
    double _xSetpoint,
    double _ySetpoint,
    double _thetaSetpoint,
    bool _xAtSetpoint,
    bool _yAtSetpoint,
    bool _thetaAtSetpoint,
    bool _atPoseReference,
    double _poseErrorX,
    double _poseErrorY,
    double _rotationError,

    double _wheelFrontLeftVelocityError,
    double _wheelFrontRightVelocityError,
    double _wheelRearLeftVelocityError,
    double _wheelRearRightVelocityError,

    double _wheelFrontLeftAccelError,
    double _wheelFrontRightAccelError,
    double _wheelRearLeftAccelError,
    double _wheelRearRightAccelError,

    bool _wheelFrontLeftAtSetpoint,
    bool _wheelFrontRightAtSetpoint,
    bool _wheelRearLeftAtSetpoint,
    bool _wheelRearRightAtSetpoint,

    double _wheelFrontLeftSetpoint,
    double _wheelFrontRightSetpoint,
    double _wheelRearLeftSetpoint,
    double _wheelRearRightSetpoint,
    double _frontLeftFeedforward,
    double _frontRightFeedforward,
    double _backLeftFeedforward,
    double _backRightFeedforward,
    double _frontLeftOutput,
    double _frontRightOutput,
    double _backLeftOutput,
    double _backRightOutput,

    double _frontLeftMotorOutput,
    double _frontRightMotorOutput,
    double _backLeftMotorOutput,
    double _backRightMotorOutput
  ) {
    timestamp = _timestamp;

    targetChassisVelocityX = _targetChassisVelocityX;
    targetChassisVelocityY = _targetChassisVelocityY;

    velocityX = _velocityX;
    velocityY = _velocityY;

    xPositionError = _xPositionError;
    xVelocityError = _xVelocityError;
    
    yPositionError = _yPositionError;
    yVelocityError = _yVelocityError;
    
    thetaPositionError = _thetaPositionError;
    thetaVelocityError = _thetaVelocityError;
    
    xSetpoint = _xSetpoint;
    ySetpoint = _ySetpoint;
    thetaSetpoint = _thetaSetpoint;

    xAtSetpoint = _xAtSetpoint;
    yAtSetpoint = _yAtSetpoint;
    thetaAtSetpoint = _thetaAtSetpoint;

    atPoseReference = _atPoseReference;

    poseErrorX = _poseErrorX;
    poseErrorY = _poseErrorY;

    rotationError = _rotationError;

    wheelFrontLeftVelocityError = _wheelFrontLeftVelocityError;
    wheelFrontRightVelocityError = _wheelFrontRightVelocityError;
    wheelRearLeftVelocityError = _wheelRearLeftVelocityError;
    wheelRearRightVelocityError = _wheelRearRightVelocityError;
                                                              
    wheelFrontLeftAccelError = _wheelFrontLeftAccelError;
    wheelFrontRightAccelError = _wheelFrontRightAccelError;
    wheelRearLeftAccelError = _wheelRearLeftAccelError;
    wheelRearRightAccelError = _wheelRearRightAccelError;
                                                              
    wheelFrontLeftAtSetpoint = _wheelFrontLeftAtSetpoint;
    wheelFrontRightAtSetpoint = _wheelFrontRightAtSetpoint;
    wheelRearLeftAtSetpoint = _wheelRearLeftAtSetpoint;
    wheelRearRightAtSetpoint = _wheelRearRightAtSetpoint;

    wheelFrontLeftSetpoint = _wheelFrontLeftSetpoint;
    wheelFrontRightSetpoint = _wheelFrontRightSetpoint;
    wheelRearLeftSetpoint = _wheelRearLeftSetpoint;
    wheelRearRightSetpoint = _wheelRearRightSetpoint;

    frontLeftFeedforward = _frontLeftFeedforward;
    frontRightFeedforward = _frontRightFeedforward;
    backLeftFeedforward = _backLeftFeedforward;
    backRightFeedforward = _backRightFeedforward;

    frontLeftOutput = _frontLeftOutput;
    frontRightOutput = _frontRightOutput; 
    backLeftOutput = _backLeftOutput; 
    backRightOutput = _backRightOutput;

    frontLeftMotorOutput = _frontLeftMotorOutput;
    frontRightMotorOutput = _frontRightMotorOutput;
    backLeftMotorOutput = _backLeftMotorOutput;
    backRightMotorOutput = _backRightMotorOutput;
  }

  int64_t timestamp = 0;

  double targetChassisVelocityX = 0;
  double targetChassisVelocityY = 0;

  FLT velocityX = 0;
  FLT velocityY = 0;

  double xPositionError = 0;
  double xVelocityError = 0;

  double yPositionError = 0;
  double yVelocityError = 0;

  double thetaPositionError = 0;
  double thetaVelocityError = 0;

  double xSetpoint = 0;
  double ySetpoint = 0;
  double thetaSetpoint = 0;

  bool xAtSetpoint = false;
  bool yAtSetpoint = false;
  bool thetaAtSetpoint = false;

  bool atPoseReference = false;

  double poseErrorX = 0;
  double poseErrorY = 0;

  double rotationError = 0;

  double wheelFrontLeftVelocityError = 0;
  double wheelFrontRightVelocityError = 0;
  double wheelRearLeftVelocityError = 0;
  double wheelRearRightVelocityError = 0;

  double wheelFrontLeftAccelError = 0;
  double wheelFrontRightAccelError = 0;
  double wheelRearLeftAccelError = 0;
  double wheelRearRightAccelError = 0;

  bool wheelFrontLeftAtSetpoint = false;
  bool wheelFrontRightAtSetpoint = false;
  bool wheelRearLeftAtSetpoint = false;
  bool wheelRearRightAtSetpoint = false;

  double wheelFrontLeftSetpoint = 0;
  double wheelFrontRightSetpoint = 0;
  double wheelRearLeftSetpoint = 0;
  double wheelRearRightSetpoint = 0;

  double frontLeftFeedforward = 0;
  double frontRightFeedforward = 0;
  double backLeftFeedforward = 0;
  double backRightFeedforward = 0;

  double frontLeftOutput = 0;
  double frontRightOutput = 0;
  double backLeftOutput = 0;
  double backRightOutput = 0;

  double frontLeftMotorOutput = 0;
  double frontRightMotorOutput = 0;
  double backLeftMotorOutput = 0;
  double backRightMotorOutput = 0;

  MSGPACK_DEFINE_MAP(
    timestamp,

    targetChassisVelocityX,
    targetChassisVelocityY,

    velocityX,
    velocityY,

    xPositionError,
    xVelocityError,

    yPositionError,
    yVelocityError,

    thetaPositionError,
    thetaVelocityError,

    xSetpoint,
    ySetpoint,
    thetaSetpoint,

    xAtSetpoint,
    yAtSetpoint,
    thetaAtSetpoint,

    atPoseReference,

    poseErrorX,
    poseErrorY,
    rotationError,

    wheelFrontLeftVelocityError,
    wheelFrontRightVelocityError,
    wheelRearLeftVelocityError,
    wheelRearRightVelocityError,

    wheelFrontLeftAccelError,
    wheelFrontRightAccelError,
    wheelRearLeftAccelError,
    wheelRearRightAccelError,

    wheelFrontLeftAtSetpoint,
    wheelFrontRightAtSetpoint,
    wheelRearLeftAtSetpoint,
    wheelRearRightAtSetpoint,

    wheelFrontLeftSetpoint,
    wheelFrontRightSetpoint,
    wheelRearLeftSetpoint,
    wheelRearRightSetpoint,

    frontLeftFeedforward,
    frontRightFeedforward,
    backLeftFeedforward,
    backRightFeedforward,

    frontLeftOutput,
    frontRightOutput,
    backLeftOutput,
    backRightOutput,

    frontLeftMotorOutput,
    frontRightMotorOutput,
    backLeftMotorOutput,
    backRightMotorOutput
  )
};

struct ParametersMessage {
  FLT timestamp = 0.0;

  double maxVelocity = 0.3;
  double maxAcceleration = 0.3;
  double maxAngularVelocity = 0.3;
  double maxAngularAcceleration = 0.3;

  double trackWidth = 0.2;
  double wheelBase  = 0.2;

  double wheelDiameter = 0.09;

  int controllerUpdateRate = 100;

  double maxXPosition = 0.6;
  double maxYPosition = 0.6;

  double linearTolerance = 0.01;
  double angularTolerance = 2.0 * M_PI / 180;

  double poseToleranceX = 0.01;
  double poseToleranceY = 0.01;
  double poseToleranceTheta = 0.02;

  double xControllerP = 1.5;
  double xControllerI = 0.0;
  double xControllerD = 0.0;

  double yControllerP = 1.0;
  double yControllerI = 0.0;
  double yControllerD = 0.0;

  double thetaControllerP = 1.5;
  double thetaControllerI = 0.0;
  double thetaControllerD = 0.0;

  double frontLeftWheelControllerP = 1.0;
  double frontLeftWheelControllerI = 0.0;
  double frontLeftWheelControllerD = 0.0;

  double frontRightWheelControllerP = 1.0;
  double frontRightWheelControllerI = 0.0;
  double frontRightWheelControllerD = 0.0;

  double backLeftWheelControllerP = 1.0;
  double backLeftWheelControllerI = 0.0;
  double backLeftWheelControllerD = 0.0;

  double backRightWheelControllerP = 1.0;
  double backRightWheelControllerI = 0.0;
  double backRightWheelControllerD = 0.0;

  double wheelMotorFeedforwardkS = 0.0797;
  double wheelMotorFeedforwardkV = 0.012;
  double wheelMotorFeedforwardkA = 0.00002;

  int16_t minWheelVoltage = -30000;
  int16_t maxWheelVoltage = 30000;

  MSGPACK_DEFINE_MAP(
    timestamp,

    maxVelocity,
    maxAcceleration,
    maxAngularVelocity,
    maxAngularAcceleration,

    trackWidth,
    wheelBase,
    wheelDiameter,

    controllerUpdateRate,

    maxXPosition,
    maxYPosition,

    linearTolerance,
    angularTolerance,

    poseToleranceX,
    poseToleranceY,
    poseToleranceTheta,

    xControllerP,
    xControllerI,
    xControllerD,

    yControllerP,
    yControllerI,
    yControllerD,

    thetaControllerP,
    thetaControllerI,
    thetaControllerD,

    frontLeftWheelControllerP,
    frontLeftWheelControllerI,
    frontLeftWheelControllerD,

    frontRightWheelControllerP,
    frontRightWheelControllerI,
    frontRightWheelControllerD,

    backLeftWheelControllerP,
    backLeftWheelControllerI,
    backLeftWheelControllerD,

    backRightWheelControllerP,
    backRightWheelControllerI,
    backRightWheelControllerD,

    wheelMotorFeedforwardkS,
    wheelMotorFeedforwardkV,
    wheelMotorFeedforwardkA,

    minWheelVoltage,
    maxWheelVoltage
  )
};

struct WheelVoltageMessage {
  uint32_t timestamp; 
  int16_t voltage[4];  

  MSGPACK_DEFINE_MAP(timestamp, voltage)
};

struct WheelStatusMessage {
  uint32_t timestamp;

  float angle[4];
  float velocity[4];
  float torque[4];
  float temperature[4];

  MSGPACK_DEFINE_MAP(timestamp, angle, velocity, torque, temperature)
};

struct WheelVelocityMessage {
  uint32_t timestamp; 
  int16_t velocity[4];  

  MSGPACK_DEFINE_MAP(timestamp, velocity)
};

# define TAU M_PI * 2
# define WHEEL_RADIUS 45


double rpmToVelocity(int16_t rpm) {
  return ((double(rpm) / 60) * (M_PI * 2) * (WHEEL_RADIUS * 0.001));
}

int16_t velocityToRPM(units::meters_per_second_t speed) {
  return int16_t ((60 * speed) / ((WHEEL_RADIUS * 0.001 * M_PI) * 2));
}

Redis* redis;
Redis* controllerStateRedisClient;

void stopWheels() {
  WheelVoltageMessage message { 0, { 0, 0, 0, 0 } };

  std::stringstream packed;
  msgpack::pack(packed, message);
  
  packed.seekg(0);

  redis->set(WHEEL_VOLTAGE_COMMAND_KEY, packed.str()); 
}

static volatile int keepRunning = 1;

void intHandler(int dummy) {
	if (keepRunning == 0) {
		stopWheels();

    exit(-1);
	}
	keepRunning = 0;
}

struct poseInfo {
  poseInfo(PoseMessage _message, VelocityMessage _messageVelocity, frc::Pose2d _pose) {
    message = _message;
    messageVelocity = _messageVelocity;
    pose    = _pose;

  }

  PoseMessage message;
  VelocityMessage messageVelocity;
  frc::Pose2d pose;
};

poseInfo getCurrentPose() {
  auto pose = redis->get(POSE_DATA_KEY);

  auto velocity = redis->get(POSE_VELOCITY_DATA_KEY);
    
  string s = *pose;
  msgpack::object_handle oh = msgpack::unpack(s.data(), s.size());

  msgpack::object deserialized = oh.get();

  PoseMessage poseMessage;
  deserialized.convert(poseMessage);

  string sV = *velocity;
  msgpack::object_handle ohV = msgpack::unpack(sV.data(), sV.size());

  msgpack::object deserializedV = ohV.get();

  VelocityMessage velocityMessage;
  deserializedV.convert(velocityMessage);

  frc::Rotation2d heading = frc::Rotation2d(units::radian_t (
    atan2(2*((poseMessage.rot[0]*poseMessage.rot[3])+(poseMessage.rot[1]*poseMessage.rot[2])),1-(2*((poseMessage.rot[2]*poseMessage.rot[2])+(poseMessage.rot[3]*poseMessage.rot[3]))))));

  //frc::Rotation2d noHeading = frc::Rotation2d{0_deg};

  frc::Pose2d robotPose{units::meter_t(poseMessage.pos[0] * -1), units::meter_t(poseMessage.pos[1] * -1), heading};

  poseInfo p = { poseMessage, velocityMessage, robotPose };

  return p;
}

WheelStatusMessage getCurrentWheelStatus() {
  auto wheelStatus = redis->get(WHEEL_STATUS_KEY);

  string s = *wheelStatus;
  msgpack::object_handle oh = msgpack::unpack(s.data(), s.size());

  msgpack::object deserialized = oh.get();

  WheelStatusMessage wheelMessage;
  deserialized.convert(wheelMessage);

  return wheelMessage;
}

int64_t startupTimestamp;

ParametersMessage lastParametersMessage;

units::meters_per_second_t maxVelocity;
units::meters_per_second_squared_t maxAcceleration;
units::radians_per_second_t maxAngularVelocity;
units::unit_t<radians_per_second_squared_t> maxAngularAcceleration;

units::meter_t trackWidth;
units::meter_t wheelBase;
units::meter_t wheelDiameter;

int controllerUpdateRate;

double maxXPosition;
double maxYPosition;

units::meter_t linearTolerance;
units::radian_t angularTolerance;

units::meter_t poseToleranceX;
units::meter_t poseToleranceY;
units::radian_t poseToleranceTheta;

double xControllerP;
double xControllerI;
double xControllerD;

double yControllerP;
double yControllerI;
double yControllerD;

double thetaControllerP;
double thetaControllerI;
double thetaControllerD;

double frontLeftWheelControllerP = 1.0;
double frontLeftWheelControllerI = 0.0;
double frontLeftWheelControllerD = 0.0;

double frontRightWheelControllerP = 1.0;
double frontRightWheelControllerI = 0.0;
double frontRightWheelControllerD = 0.0;

double backLeftWheelControllerP = 1.0;
double backLeftWheelControllerI = 0.0;
double backLeftWheelControllerD = 0.0;

double backRightWheelControllerP = 1.0;
double backRightWheelControllerI = 0.0;
double backRightWheelControllerD = 0.0;

double wheelMotorFeedforwardkS = 0.0797;
double wheelMotorFeedforwardkV = 0.012;
double wheelMotorFeedforwardkA = 0.00002;

int16_t minWheelVoltage = -30000;
int16_t maxWheelVoltage = 30000;

frc::MecanumDriveKinematics* driveKinematics;
frc::HolonomicDriveController* controller;

frc2::PIDController* frontLeftWheelController;
frc2::PIDController* frontRightWheelController;
frc2::PIDController* backLeftWheelController;
frc2::PIDController* backRightWheelController;

frc::SimpleMotorFeedforward<units::meter>* wheelMotorFeedforward;

frc2::PIDController* xController;
frc2::PIDController* yController;
frc::ProfiledPIDController<units::radian>* thetaController;

frc::TrapezoidProfile<units::radian>::Constraints* thetaConstraints;

std::vector<frc::Translation2d> activeWaypoints;
std::vector<std::vector<std::vector<frc::Translation2d>>> activeScanPatterns;

units::meters_per_second_t scanPatternMaxVelocity;
units::meters_per_second_squared_t scanPatternMaxAcceleration;

frc::Trajectory activeTrajectory;

int64_t lastTrajectoryTime  = 0;
int64_t trajectoryStartTime = 0;

sw::redis::Subscriber* subscriber;

void updateParameters(ParametersMessage parametersMessage) {
  lastParametersMessage = parametersMessage;

  maxVelocity = units::meters_per_second_t (parametersMessage.maxVelocity);
  maxAcceleration = units::meters_per_second_squared_t (parametersMessage.maxAcceleration);
  maxAngularVelocity = units::radians_per_second_t (parametersMessage.maxAngularVelocity);
  maxAngularAcceleration = units::unit_t<radians_per_second_squared_t>(parametersMessage.maxAngularAcceleration);

  trackWidth = units::meter_t (parametersMessage.trackWidth);
  wheelBase =  units::meter_t (parametersMessage.wheelBase);
  wheelDiameter = units::meter_t (parametersMessage.wheelDiameter);

  linearTolerance = units::meter_t (parametersMessage.linearTolerance);
  angularTolerance = units::radian_t (parametersMessage.angularTolerance);

  poseToleranceX = units::meter_t (parametersMessage.poseToleranceX);
  poseToleranceY = units::meter_t (parametersMessage.poseToleranceY);
  poseToleranceTheta = units::radian_t (parametersMessage.poseToleranceTheta);
  
  maxXPosition = parametersMessage.maxXPosition;
  maxYPosition = parametersMessage.maxYPosition;

  driveKinematics = new frc::MecanumDriveKinematics(
    frc::Translation2d(wheelBase / 2, trackWidth / 2),
    frc::Translation2d(wheelBase / 2, -trackWidth / 2),
    frc::Translation2d(-wheelBase / 2, trackWidth / 2),
    frc::Translation2d(-wheelBase / 2, -trackWidth / 2)
  );

  controllerUpdateRate = parametersMessage.controllerUpdateRate;

  frontLeftWheelControllerP = parametersMessage.frontLeftWheelControllerP; 
  frontLeftWheelControllerI = parametersMessage.frontLeftWheelControllerI; 
  frontLeftWheelControllerD = parametersMessage.frontLeftWheelControllerD; 

  frontRightWheelControllerP = parametersMessage.frontRightWheelControllerP; 
  frontRightWheelControllerI = parametersMessage.frontRightWheelControllerI; 
  frontRightWheelControllerD = parametersMessage.frontRightWheelControllerD; 

  backLeftWheelControllerP = parametersMessage.backLeftWheelControllerP; 
  backLeftWheelControllerI = parametersMessage.backLeftWheelControllerI; 
  backLeftWheelControllerD = parametersMessage.backLeftWheelControllerD; 

  backRightWheelControllerP = parametersMessage.backRightWheelControllerP;
  backRightWheelControllerI = parametersMessage.backRightWheelControllerI;
  backRightWheelControllerD = parametersMessage.backRightWheelControllerD; 

  wheelMotorFeedforwardkS = parametersMessage.wheelMotorFeedforwardkS;
  wheelMotorFeedforwardkV = parametersMessage.wheelMotorFeedforwardkV;
  wheelMotorFeedforwardkA = parametersMessage.wheelMotorFeedforwardkA;

  minWheelVoltage = parametersMessage.minWheelVoltage;
  maxWheelVoltage = parametersMessage.maxWheelVoltage;

  if(frontLeftWheelController == NULL) {
    frontLeftWheelController = new frc2::PIDController(frontLeftWheelControllerP, frontLeftWheelControllerI, frontLeftWheelControllerD, units::second_t (1 / controllerUpdateRate));
  } else {
    frontLeftWheelController->SetPID(frontLeftWheelControllerP, frontLeftWheelControllerI, frontLeftWheelControllerD);
  }

  if(frontRightWheelController == NULL) {
    frontRightWheelController = new frc2::PIDController(frontRightWheelControllerP, frontRightWheelControllerI, frontRightWheelControllerD, units::second_t (1 / controllerUpdateRate));
  } else {
    frontRightWheelController->SetPID(frontRightWheelControllerP, frontRightWheelControllerI, frontRightWheelControllerD);
  }

  if(backLeftWheelController == NULL) {
    backLeftWheelController = new frc2::PIDController(backLeftWheelControllerP, backLeftWheelControllerI, backLeftWheelControllerD, units::second_t (1 / controllerUpdateRate));
  } else {
    backLeftWheelController->SetPID(backLeftWheelControllerP, backLeftWheelControllerI, backLeftWheelControllerD);
  }

  if(backRightWheelController == NULL) {
    backRightWheelController = new frc2::PIDController(backRightWheelControllerP, backRightWheelControllerI, backRightWheelControllerD, units::second_t (1 / controllerUpdateRate));
  } else {
    backRightWheelController->SetPID(backRightWheelControllerP, backRightWheelControllerI, backRightWheelControllerD);
  }

  wheelMotorFeedforward = new frc::SimpleMotorFeedforward<units::meter>(units::volt_t (wheelMotorFeedforwardkS), units::volt_t(wheelMotorFeedforwardkV) / 1_mps, units::volt_t(wheelMotorFeedforwardkA) / 1_mps_sq);

  xControllerP = parametersMessage.xControllerP;
  xControllerI = parametersMessage.xControllerI;
  xControllerD = parametersMessage.xControllerD;
  
  if(xController == NULL) {
    xController = new frc2::PIDController(xControllerP, xControllerI, xControllerD);
  } else {
    controller->getXController().SetPID(xControllerP, xControllerI, xControllerD);
  }

  yControllerP = parametersMessage.yControllerP;
  yControllerI = parametersMessage.yControllerI;
  yControllerD = parametersMessage.yControllerD;

  if(yController == NULL) {
    yController = new frc2::PIDController(yControllerP, yControllerI, yControllerD);
  } else {
    controller->getYController().SetPID(yControllerP, yControllerI, yControllerD);
  }

  thetaControllerP = parametersMessage.thetaControllerP;
  thetaControllerI = parametersMessage.thetaControllerI;
  thetaControllerD = parametersMessage.thetaControllerD;

  if(thetaConstraints == NULL) {
    thetaConstraints = new frc::TrapezoidProfile<units::radian>::Constraints(maxAngularVelocity, maxAngularAcceleration);
  } else {
    thetaConstraints->maxVelocity = maxAngularVelocity;
    thetaConstraints->maxAcceleration = maxAngularAcceleration;
  }

  if(thetaController == NULL) {
    thetaController = new frc::ProfiledPIDController<units::radian>(
      thetaControllerP,
      thetaControllerI,
      thetaControllerD,
      *thetaConstraints
    );
  } else {
    controller->getThetaController().SetPID(thetaControllerP, thetaControllerI, thetaControllerD);
  }

  if(controller == NULL) {
    controller = new frc::HolonomicDriveController(
      (frc2::PIDController) *xController,
      (frc2::PIDController) *yController,
      (frc::ProfiledPIDController<units::radian>) *thetaController
    );
  }

  controller->SetTolerance(frc::Pose2d{poseToleranceX, poseToleranceY, frc::Rotation2d{poseToleranceTheta}});

  printf("X P: %lf, X I: %lf, X D: %lf \n", controller->getXController().GetP(), controller->getXController().GetI(), controller->getXController().GetD());
  printf("Y P: %lf, Y I: %lf, Y D: %lf \n", controller->getYController().GetP(), controller->getYController().GetI(), controller->getYController().GetD());
}

void updateParametersFromString(string parametersString) {
  msgpack::object_handle oh = msgpack::unpack(parametersString.data(), parametersString.size());

  msgpack::object deserialized = oh.get();
  
  std::cout << "updated parameters: " << deserialized << std::endl;

  ParametersMessage parametersMessage;
  deserialized.convert(parametersMessage);

  updateParameters(parametersMessage);
}

void generateTrajectoriesFromActiveScanPatterns(units::meters_per_second_t trajectoryMaxVelocity, units::meters_per_second_squared_t trajectoryMaxAcceleration) {
  poseInfo currentPose = getCurrentPose();

  frc::TrajectoryConfig trajectoryConfig = frc::TrajectoryConfig(trajectoryMaxVelocity, trajectoryMaxAcceleration);
  trajectoryConfig.SetKinematics(*driveKinematics);
  //trajectoryConfig.SetReversed(true);

  for(auto pattern = activeScanPatterns.begin(); pattern != activeScanPatterns.end(); ++pattern) {
    
    for(auto line = pattern->begin(); line != pattern->end(); ++line) {
      
      for(auto sample2d = line->begin(), prev = line->end(); sample2d != line->end(); prev = sample2d, ++sample2d) {
        if(prev != line->end()) {
          frc::Pose2d p0 = frc::Pose2d(*prev, frc::Rotation2d(88_deg));
          frc::Pose2d p1 = frc::Pose2d(*sample2d, frc::Rotation2d(88_deg));

          frc::Trajectory t = frc::TrajectoryGenerator::GenerateTrajectory(p0, {}, p1, trajectoryConfig);
          units::time::second_t totalTime = t.TotalTime();
  
          json profiledTrajectoryJSON = json::array(); 
          for (int i = 0; i <= int(ceil(totalTime.value() / (controllerUpdateRate * 0.001))); i++) {
            frc::Trajectory::State state = t.Sample(units::second_t (i * controllerUpdateRate * 0.001));

            json stateTrajectoryJSON;
            frc::to_json(stateTrajectoryJSON, state);

            profiledTrajectoryJSON.push_back(stateTrajectoryJSON);
          }

          printf("trajectory total time: %f \n", (double)totalTime.value());

          redis->publish(TRAJECTORY_PROFILE_KEY, profiledTrajectoryJSON.dump()); 

        }
      }
    }
  }
}

void profileActiveTrajectory() {
  units::time::second_t totalTime = activeTrajectory.TotalTime();

  json profiledTrajectoryJSON = json::array(); 
  for (int i = 0; i <= int(ceil(totalTime.value() / (controllerUpdateRate * 0.001))); i++) {
    frc::Trajectory::State state = activeTrajectory.Sample(units::second_t (i * controllerUpdateRate * 0.001));

    json stateTrajectoryJSON;
    frc::to_json(stateTrajectoryJSON, state);

    profiledTrajectoryJSON.push_back(stateTrajectoryJSON);
  }

  printf("trajectory total time: %f \n", (double)totalTime.value());

  redis->publish(TRAJECTORY_PROFILE_KEY, profiledTrajectoryJSON.dump()); 
}

void generateTrajectoryFromActiveWaypoints(units::meters_per_second_t trajectoryMaxVelocity, units::meters_per_second_squared_t trajectoryMaxAcceleration) {
  poseInfo currentPose = getCurrentPose();
  
  frc::TrajectoryConfig trajectoryConfig = frc::TrajectoryConfig(trajectoryMaxVelocity, trajectoryMaxAcceleration);
  trajectoryConfig.SetKinematics(*driveKinematics);

  activeTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(currentPose.pose,
      activeWaypoints, currentPose.pose, trajectoryConfig);

  units::time::second_t totalTime = activeTrajectory.TotalTime();
  
  json profiledTrajectoryJSON = json::array(); 
  for (int i = 0; i <= int(ceil(totalTime.value() / (controllerUpdateRate * 0.001))); i++) {
    frc::Trajectory::State state = activeTrajectory.Sample(units::second_t (i * controllerUpdateRate * 0.001));

    json stateTrajectoryJSON;
    frc::to_json(stateTrajectoryJSON, state);

    profiledTrajectoryJSON.push_back(stateTrajectoryJSON);
  }

  printf("trajectory total time: %f \n", (double)totalTime.value());

  redis->publish(TRAJECTORY_PROFILE_KEY, profiledTrajectoryJSON.dump()); 
}

void updateActiveScanPatternsFromJSON(std::string trajectoryJSONString) {
  activeScanPatterns.clear();

  json trajectoryJSON = json::parse(trajectoryJSONString);

  if(trajectoryJSON.find("samplePoints") != trajectoryJSON.end()) {
    
    for(json::iterator pattern = trajectoryJSON["samplePoints"].begin(); pattern != trajectoryJSON["samplePoints"].end(); ++pattern) {
      
      std::vector< vector<frc::Translation2d> > scanPattern;
      
      for(json::iterator line = pattern.value().begin(); line != pattern.value().end(); ++line) {

        vector<frc::Translation2d> lineSamples;

        for(json::iterator samplePoint = line.value().begin(); samplePoint != line.value().end(); ++samplePoint) {
          
          auto point = samplePoint.value().get<std::vector<float>>();

          frc::Translation2d sample2d = frc::Translation2d{units::meter_t {point[0]}, units::meter_t {point[1]}};

          std::cout << sample2d.X().value() << std::endl;
          std::cout << sample2d.Y().value() << std::endl;


          lineSamples.push_back(sample2d);
        }

        scanPattern.push_back(lineSamples);
      }

      activeScanPatterns.push_back(scanPattern);
    }
  }

  if(trajectoryJSON.find("maxVelocity") != trajectoryJSON.end()) {
    scanPatternMaxVelocity = units::meters_per_second_t (trajectoryJSON["maxVelocity"].get<double>());
  }

  if(trajectoryJSON.find("maxAcceleration") != trajectoryJSON.end()) {
    scanPatternMaxAcceleration = units::meters_per_second_squared_t (trajectoryJSON["maxAcceleration"].get<double>());
  }

  if(activeScanPatterns.size() > 0) {
    //generateTrajectoriesFromActiveScanPatterns(trajectoryMaxVelocity, trajectoryMaxAcceleration);
  }
}

void updateActiveWaypointsFromJSON(std::string waypointsJSONString) {
  
  activeWaypoints.clear();
  
  json waypointJSON = json::parse(waypointsJSONString);
  
  if(waypointJSON.find("waypoints") != waypointJSON.end()) {
    
    poseInfo currentPose = getCurrentPose();

    //activeWaypoints.push_back(currentPose.pose.Translation());

    for(json::iterator it = waypointJSON["waypoints"].begin(); it != waypointJSON["waypoints"].end(); ++it) {
      frc::Pose2d p;

      frc::from_json(it.value(), p);

      activeWaypoints.push_back(p.Translation());
    }

    //activeWaypoints.push_back(currentPose.pose.Translation());
  }

  units::meters_per_second_t trajectoryMaxVelocity = maxVelocity;
  units::meters_per_second_squared_t trajectoryMaxAcceleration = maxAcceleration;

  if(waypointJSON.find("maxVelocity") != waypointJSON.end()) {
    trajectoryMaxVelocity = units::meters_per_second_t (waypointJSON["maxVelocity"].get<double>());
  }

  if(waypointJSON.find("maxAcceleration") != waypointJSON.end()) {
    trajectoryMaxAcceleration = units::meters_per_second_squared_t (waypointJSON["maxAcceleration"].get<double>());
  }

  if(activeWaypoints.size() > 0) {
    generateTrajectoryFromActiveWaypoints(trajectoryMaxVelocity, trajectoryMaxAcceleration);
  }
}

void dumpWaypoints() {
  for(frc::Translation2d p : activeWaypoints) {
    json waypointJSON;
    frc::to_json(waypointJSON, p);

    std::cout << "waypoints: " << waypointJSON.dump() << std::endl;
  }
}

void printMsgpackMessage(std::string packed) {
  auto oh = msgpack::unpack(packed.data(), packed.size());
  std::cout << oh.get() << std::endl;
}

void publishControlState(
  frc::ChassisSpeeds targetChassisSpeeds,
  frc::MecanumDriveWheelSpeeds targetWheelSpeeds,

  units::volt_t frontLeftFeedforward,
  units::volt_t frontRightFeedforward,
  units::volt_t backLeftFeedforward,
  units::volt_t backRightFeedforward,

  double frontLeftOutput,
  double frontRightOutput,
  double backLeftOutput,
  double backRightOutput,

  double frontLeftMotorOutput,
  double frontRightMotorOutput,
  double backLeftMotorOutput,
  double backRightMotorOutput,

  poseInfo currentPose

  ) {
  int64_t currentTimestamp = duration_cast<microseconds>(system_clock::now().time_since_epoch()).count();
  int64_t timestamp = currentTimestamp - startupTimestamp;

  double xPositionError = controller->getXController().GetPositionError();
  double xVelocityError = controller->getXController().GetVelocityError();

  double yPositionError = controller->getYController().GetPositionError();
  double yVelocityError = controller->getYController().GetVelocityError();
  
  frc::ProfiledPIDController<units::radian>::Distance_t thetaPositionError = controller->getThetaController().GetPositionError();
  frc::ProfiledPIDController<units::radian>::Velocity_t thetaVelocityError = controller->getThetaController().GetVelocityError();
  
  double xSetpoint = controller->getXController().GetSetpoint();
  double ySetpoint = controller->getYController().GetSetpoint();
  
  frc::ProfiledPIDController<units::radian>::State thetaState = controller->getThetaController().GetSetpoint();

  bool xAtSetpoint = controller->getXController().AtSetpoint();
  bool yAtSetpoint = controller->getYController().AtSetpoint();
  bool thetaAtSetpoint = controller->getThetaController().AtSetpoint();

  bool atPoseReference = controller->AtReference();

  frc::Pose2d poseError = controller->getPoseError();
  frc::Rotation2d rotationError = controller->getRotationError();

  double targetChassisVelocityX = targetChassisSpeeds.vx.value();
  double targetChassisVelocityY = targetChassisSpeeds.vy.value();

  // wheel controllers are actually controlling velocity (not position), so the below is not a mistake

  double wheelFrontLeftVelocityError = frontLeftWheelController->GetPositionError();
  double wheelFrontRightVelocityError = frontRightWheelController->GetPositionError();
  double wheelRearLeftVelocityError = backLeftWheelController->GetPositionError();
  double wheelRearRightVelocityError = backRightWheelController->GetPositionError();

  double wheelFrontLeftAccelError = frontLeftWheelController->GetVelocityError();
  double wheelFrontRightAccelError = frontRightWheelController->GetVelocityError();
  double wheelRearLeftAccelError = backLeftWheelController->GetVelocityError();
  double wheelRearRightAccelError = backRightWheelController->GetVelocityError();

  bool wheelFrontLeftAtSetpoint = frontLeftWheelController->AtSetpoint();
  bool wheelFrontRightAtSetpoint = frontRightWheelController->AtSetpoint();
  bool wheelRearLeftAtSetpoint =  backLeftWheelController->AtSetpoint();
  bool wheelRearRightAtSetpoint = backRightWheelController->AtSetpoint();

  FLT velocityX = currentPose.messageVelocity.pos[0];
  FLT velocityY = currentPose.messageVelocity.pos[1];

  ControllerStateMessage message = {
    timestamp,

    targetChassisVelocityX,
    targetChassisVelocityY,

    velocityX,
    velocityY,

    xPositionError,    
    xVelocityError,

    yPositionError,
    yVelocityError,

    double (thetaPositionError),
    double (thetaVelocityError),

    xSetpoint,
    ySetpoint,
    double (thetaState.position),

    xAtSetpoint,
    yAtSetpoint,
    thetaAtSetpoint,

    atPoseReference,

    double (poseError.X()),
    double (poseError.Y()),

    double (rotationError.Radians()),

    wheelFrontLeftVelocityError,
    wheelFrontRightVelocityError,
    wheelRearLeftVelocityError,
    wheelRearRightVelocityError,

    wheelFrontLeftAccelError,
    wheelFrontRightAccelError,
    wheelRearLeftAccelError,
    wheelRearRightAccelError,

    wheelFrontLeftAtSetpoint,
    wheelFrontRightAtSetpoint,
    wheelRearLeftAtSetpoint,
    wheelRearRightAtSetpoint,

    double (targetWheelSpeeds.frontLeft.value()),
    double (targetWheelSpeeds.frontRight.value()),
    double (targetWheelSpeeds.rearLeft.value()),
    double (targetWheelSpeeds.rearRight.value()),

    double (frontLeftFeedforward.value()),
    double (frontRightFeedforward.value()),
    double (backLeftFeedforward.value()),
    double (backRightFeedforward.value()),

    frontLeftOutput,
    frontRightOutput,
    backLeftOutput,
    backRightOutput,

    frontLeftMotorOutput,
    frontRightMotorOutput,
    backLeftMotorOutput,
    backRightMotorOutput
  };

  std::stringstream packed;
  msgpack::pack(packed, message);

  packed.seekg(0);

  //printMsgpackMessage(packed.str());

  controllerStateRedisClient->publish(CONTROLLER_STATE_KEY, packed.str());
}

enum messageTypes {
  PARAMETERS_MESSAGE,
  TRAJECTORY_MESSAGE,
  COMMAND_MESSAGE,

  CONTROLLER_MESSAGE,

  UNKNOWN_MESSAGE
};

enum commandTypes {
  STOP_WHEELS_COMMAND,
  RUN_TRAJECTORY_COMMAND,

  UNKNOWN_COMMAND
};

messageTypes getMessageType(std::string const& messageType) {
  if(messageType == PARAMETERS_KEY) return PARAMETERS_MESSAGE;
  if(messageType == TRAJECTORY_KEY) return TRAJECTORY_MESSAGE;
  if(messageType == COMMAND_KEY) return COMMAND_MESSAGE;
  if(messageType == CONTROLLER_KEY) return CONTROLLER_MESSAGE;

  return UNKNOWN_MESSAGE;
}

commandTypes getCommandType(std::string const& commandType) {
  if(commandType == STOP_WHEELS_COMMAND_KEY) return STOP_WHEELS_COMMAND;
  if(commandType == RUN_TRAJECTORY_COMMAND_KEY) return RUN_TRAJECTORY_COMMAND;

  return UNKNOWN_COMMAND;
}

double normalizeMotorVoltage(double wheelControllerOutput, units::volt_t wheelFeedforward) {

  return (wheelControllerOutput + wheelFeedforward.value()) * maxWheelVoltage;
}

frc::MecanumDriveWheelSpeeds targetWheelSpeeds;
frc::ChassisSpeeds targetChassisSpeeds;

units::volt_t frontLeftFeedforward;
units::volt_t frontRightFeedforward;
units::volt_t backLeftFeedforward;
units::volt_t backRightFeedforward;

double frontLeftOutput;
double frontRightOutput;
double backLeftOutput;
double backRightOutput;

void runActiveTrajectory() {
  LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(controllerUpdateRate);

  poseInfo startingRobotPose = getCurrentPose();

  /*auto waypoints = std::vector{startingRobotPose.pose,
                               frc::Pose2d{0.3_m, 0.0_m, startingRobotPose.pose.Rotation() }};
  auto trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
      waypoints, {0.4_mps, 0.4_mps_sq});*/

  redis->publish(CONTROLLER_KEY, "runActiveTrajectory"); 

  lastTrajectoryTime  = duration_cast<microseconds>(system_clock::now().time_since_epoch()).count();
  trajectoryStartTime = duration_cast<microseconds>(system_clock::now().time_since_epoch()).count();

  bool trajectoryComplete = false;

  while(keepRunning && !trajectoryComplete) {
    /*try {
      subscriber->consume();
    } catch (const Error &err) {}*/
    
    timer.waitForNextLoop();

    int64_t currentTime = duration_cast<microseconds>(system_clock::now().time_since_epoch()).count();
    int64_t dt = currentTime - lastTrajectoryTime;

    lastTrajectoryTime = currentTime;
    int64_t elapsedTime = currentTime - trajectoryStartTime;

    poseInfo robotPose = getCurrentPose();
    
    if (abs(robotPose.pose.X().value()) > maxXPosition || abs(robotPose.pose.Y().value()) > maxYPosition) {
      std::cout << "Out of bounds!" << std::endl;
      
      redis->publish(CONTROLLER_KEY, "boundsException"); 

      stopWheels();

      continue;
    }

    if(units::second_t(elapsedTime * 0.000001) > activeTrajectory.TotalTime()) {
      trajectoryComplete = true;

      redis->publish(CONTROLLER_KEY, "trajectoryComplete");

      std::cout << "Trajectory complete!" << std::endl;

      continue; 
    }

    frc::Trajectory::State state = activeTrajectory.Sample(units::second_t (elapsedTime * .000001));
    json stateTrajectoryJSON;
    
    frc::to_json(stateTrajectoryJSON, state);

    json trajectoryInfoJSON;

    trajectoryInfoJSON["timestamp"] = robotPose.message.timestamp;
    trajectoryInfoJSON["trajectory"] = stateTrajectoryJSON;

    redis->publish(TRAJECTORY_SAMPLE_KEY, trajectoryInfoJSON.dump()); 
    
    targetChassisSpeeds = controller->Calculate(robotPose.pose, state, startingRobotPose.pose.Rotation());
    
    targetWheelSpeeds = driveKinematics->ToWheelSpeeds(targetChassisSpeeds);

    frontLeftFeedforward  = wheelMotorFeedforward->Calculate(targetWheelSpeeds.frontLeft);
    frontRightFeedforward = wheelMotorFeedforward->Calculate(targetWheelSpeeds.frontRight);
    backLeftFeedforward   = wheelMotorFeedforward->Calculate(targetWheelSpeeds.rearLeft);
    backRightFeedforward  = wheelMotorFeedforward->Calculate(targetWheelSpeeds.rearRight);

    WheelStatusMessage wheelStatus = getCurrentWheelStatus();

    // 1 - back right, 2 - front right, 3 - front left, 4 - back left

    frontLeftOutput  = frontLeftWheelController->Calculate(  rpmToVelocity(wheelStatus.velocity[2]),  targetWheelSpeeds.frontLeft.value());
    frontRightOutput = frontRightWheelController->Calculate( rpmToVelocity(wheelStatus.velocity[1]),  targetWheelSpeeds.frontRight.value());
    backLeftOutput   = backLeftWheelController->Calculate(   rpmToVelocity(wheelStatus.velocity[3]),  targetWheelSpeeds.rearLeft.value());
    backRightOutput  = backRightWheelController->Calculate(  rpmToVelocity(wheelStatus.velocity[0]),  targetWheelSpeeds.rearRight.value());

    // wheel data is
    // 3 - back left
    // 2 - front left
    // 0 - back right
    // 1 - front right

    int64_t currentMicro = duration_cast<microseconds>(system_clock::now().time_since_epoch()).count();

    WheelVoltageMessage message = {
      uint32_t(currentMicro - startupTimestamp),
      
      {
        int16_t (normalizeMotorVoltage(backRightOutput, backRightFeedforward)),
        int16_t (normalizeMotorVoltage(frontRightOutput, frontRightFeedforward)),
        int16_t (normalizeMotorVoltage(frontLeftOutput, frontLeftFeedforward)),
        int16_t (normalizeMotorVoltage(backLeftOutput, backLeftFeedforward))
      }
    };

    std::stringstream packed;
    msgpack::pack(packed, message);

    packed.seekg(0);
    
    redis->set(WHEEL_VOLTAGE_COMMAND_KEY, packed.str()); 
  }

  stopWheels();
}

void runActiveScanPattern() {
  poseInfo startingRobotPose = getCurrentPose();

  redis->publish(CONTROLLER_KEY, "runActiveScanPattern"); 
 
  int patternIndex = 0;
  for(auto pattern = activeScanPatterns.begin(); pattern != activeScanPatterns.end(); ++pattern) {
    
    int lineIndex = 0;
    for(auto line = pattern->begin(); line != pattern->end(); ++line) {
      if(line == pattern->begin()) {
        poseInfo currentPose = getCurrentPose();

        //auto startingSample2d = line->begin();

        //frc::Pose2d firstLinePosition = frc::Pose2d(*startingSample2d, frc::Rotation2d(0_deg));

        //activeTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(currentPose.pose, {}, firstLinePosition, trajectoryConfig);
        //profileActiveTrajectory();

        //runActiveTrajectory();
      }

      int sampleIndex = 0;
      for(auto sample2d = line->begin(), lastSample2d = line->end(); sample2d != line->end(); lastSample2d = sample2d, ++sample2d) {
        poseInfo currentPose = getCurrentPose();
          
        //frc::Pose2d p0 = frc::Pose2d(*lastSample2d, frc::Rotation2d(88_deg));
        frc::Pose2d samplePose = frc::Pose2d(*sample2d, currentPose.pose.Rotation());
        
        frc::TrajectoryConfig trajectoryConfig = frc::TrajectoryConfig(scanPatternMaxVelocity, scanPatternMaxAcceleration);
        trajectoryConfig.SetKinematics(*driveKinematics);

        json currentPoseJSON;
        frc::to_json(currentPoseJSON, currentPose.pose);

        json samplePoseJSON;
        frc::to_json(samplePoseJSON, samplePose);

        std::cout << "currentPose:" << std::endl;
        std::cout << currentPoseJSON.dump() << std::endl;

        std::cout << "samplePose:" << std::endl;
        std::cout << samplePoseJSON.dump() << std::endl;

        bool generatedTrajectory = false;
        try {
          activeTrajectory = frc::TrajectoryGenerator::GenerateTrajectory({currentPose.pose, samplePose}, trajectoryConfig);
          generatedTrajectory = true;
        } catch(const std::exception& e) {
          std::cout << "Error generating trajectory:" << e.what() << std::endl;
        }

        if(generatedTrajectory) {
          profileActiveTrajectory();
          runActiveTrajectory();

          poseInfo currentPose = getCurrentPose();

          cpr::Response r = cpr::Get(
            cpr::Url{"http://localhost:9005/scan?patternIndex=" + 
              std::to_string(patternIndex) + 
              "&lineIndex=" + std::to_string(lineIndex) + 
              "&sampleIndex=" + std::to_string(sampleIndex)});

          std::cout << "radar data capture status code: " << std::to_string(r.status_code) << std::endl;
          std::cout << r.text << std::endl;

          if(r.status_code == 200) {
            std::stringstream packed;
            msgpack::pack(packed, currentPose.message);

            packed.seekg(0);
          
            redis->publish(RADAR_SAMPLE_POINT_KEY, packed.str());
          }
        }

        sampleIndex++;
      }

      lineIndex++;
    }
  
    patternIndex++;
  }
  
  stopWheels();
}

void publishControllerStateTask() {
  ConnectionOptions redisConnectionOptions;
  redisConnectionOptions.type = ConnectionType::UNIX;
  redisConnectionOptions.path = "/var/run/redis/redis-server.sock";
  redisConnectionOptions.socket_timeout = std::chrono::milliseconds(100);

  controllerStateRedisClient = new Redis(redisConnectionOptions);

  printf("Started controller state publish task\n");

  LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(controllerUpdateRate);

  while(keepRunning) {
    timer.waitForNextLoop();
    
    publishControlState(
      targetChassisSpeeds,
      targetWheelSpeeds,

      frontLeftFeedforward,
      frontRightFeedforward,
      backLeftFeedforward,
      backRightFeedforward,

      frontLeftOutput,
      frontRightOutput,
      backLeftOutput,
      backRightOutput,

      normalizeMotorVoltage(frontLeftOutput, frontLeftFeedforward),
      normalizeMotorVoltage(frontRightOutput, frontRightFeedforward),
      normalizeMotorVoltage(backLeftOutput, backLeftFeedforward),
      normalizeMotorVoltage(backRightOutput, backRightFeedforward),

      getCurrentPose()
    );
  }
}

int main(int argc, char **argv) {
	signal(SIGABRT, intHandler);
	signal(SIGTERM, intHandler);
	signal(SIGINT, intHandler);

  setbuf(stdout, NULL);
  
  ConnectionOptions redisConnectionOptions;
  redisConnectionOptions.type = ConnectionType::UNIX;
  redisConnectionOptions.path = "/var/run/redis/redis-server.sock";
  redisConnectionOptions.socket_timeout = std::chrono::milliseconds(100);

  redis = new Redis(redisConnectionOptions);

  auto timestamp = redis->get(STARTUP_TIMESTAMP_KEY);
  if(timestamp) {
    string timestampString = *timestamp;
    
    startupTimestamp = int64_t(atoll(timestampString.c_str()));
  } else {
    std::cout << "Startup timestamp not set or invalid" << std::endl;

    exit(0);
  }

  stopWheels();

  auto parameters = redis->get(PARAMETERS_KEY);

  if(parameters) {
    string parametersString = *parameters;

    updateParametersFromString(parametersString);
  } else {
    
    ParametersMessage parametersMessage;

    std::stringstream packed;
    msgpack::pack(packed, parametersMessage);
  
    packed.seekg(0);

    std::string str(packed.str());

    msgpack::object_handle oh =
      msgpack::unpack(str.data(), str.size());

    msgpack::object deserialized = oh.get();
    std::cout << "reset parameters: " << deserialized << std::endl;

    redis->set(PARAMETERS_KEY, packed.str());

    updateParameters(parametersMessage);
  }

  poseInfo robotPose = getCurrentPose();

  auto waypoints = redis->get(TRAJECTORY_KEY);

  if(waypoints) {
    string waypointsString = *waypoints;

    updateActiveWaypointsFromJSON(waypointsString);
    
  } else {
    //activeWaypoints.push_back(robotPose.pose);
    //activeWaypoints.push_back(frc::Pose2d{0.0_m, 0.0_m, robotPose.pose.Rotation()});
    //activeWaypoints.push_back(frc::Pose2d{0.5_m, -0.5_m, robotPose.pose.Rotation()}); 
    //activeWaypoints.push_back(frc::Pose2d{0.5_m, 0.5_m, robotPose.pose.Rotation()}); 
    //activeWaypoints.push_back(frc::Pose2d{0.5_m, -0.3_m, robotPose.pose.Rotation()}); 
    //activeWaypoints.push_back(frc::Pose2d{0.5_m, -0.2_m, robotPose.pose.Rotation()}); 

    /*activeWaypoints.push_back(robotPose.pose);
    activeWaypoints.push_back(frc::Pose2d{0.3_m, 0.0_m, robotPose.pose.Rotation()});
    activeWaypoints.push_back(frc::Pose2d{0.3_m, 0.3_m, robotPose.pose.Rotation()}); 
    activeWaypoints.push_back(frc::Pose2d{0.0_m, 0.0_m, robotPose.pose.Rotation()}); 
    activeWaypoints.push_back(frc::Pose2d{-0.3_m, 0.0_m, robotPose.pose.Rotation()}); 
    activeWaypoints.push_back(frc::Pose2d{-0.3_m, -0.3_m, robotPose.pose.Rotation()});  

    generateTrajectoryFromActiveWaypoints(0.4_mps, 0.4_mps_sq);*/
  }

  dumpWaypoints();
  
  sw::redis::Subscriber sub = redis->subscriber();
  subscriber = &sub;

  subscriber->on_message([](std::string channel, std::string msg) {
    switch (getMessageType(channel)) {
      case PARAMETERS_MESSAGE:
        updateParametersFromString(msg);
        
        break;
        
      case TRAJECTORY_MESSAGE:
        std::cout << "got trajectory! " << msg << std::endl;

        updateActiveScanPatternsFromJSON(msg);

        break;

      case COMMAND_MESSAGE:
        json commandMessageJSON = json::parse(msg);

        if(commandMessageJSON.find("command") != commandMessageJSON.end()) {
          switch (getCommandType(commandMessageJSON["command"].get<std::string>())) {
            case STOP_WHEELS_COMMAND:
              stopWheels();

            case RUN_TRAJECTORY_COMMAND:
              stopWheels();

              runActiveScanPattern();
            break;
          }
        }
    }
  });

  subscriber->subscribe(PARAMETERS_KEY);
  subscriber->subscribe(TRAJECTORY_KEY);
  subscriber->subscribe(COMMAND_KEY);

  thread publishControllerStateThread(publishControllerStateTask);

  while(keepRunning) {
    try {
      subscriber->consume();
    } catch (const Error &err) {}
  }

  stopWheels();

	return 0;
}
