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

const string PARAMETERS_KEY = "rover_parameters";
const string TRAJECTORY_KEY = "rover_trajectory";
const string COMMAND_KEY = "rover_command";

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
  poseInfo(PoseMessage _message, frc::Pose2d _pose) {
    message = _message;
    pose    = _pose;
  }

  PoseMessage message;
  frc::Pose2d pose;
};

poseInfo getCurrentPose() {
  auto pose = redis->get(POSE_DATA_KEY);
    
  string s = *pose;
  msgpack::object_handle oh = msgpack::unpack(s.data(), s.size());

  msgpack::object deserialized = oh.get();

  PoseMessage poseMessage;
  deserialized.convert(poseMessage);

  frc::Rotation2d heading = frc::Rotation2d(units::radian_t (
    atan2(2*((poseMessage.rot[0]*poseMessage.rot[3])+(poseMessage.rot[1]*poseMessage.rot[2])),1-(2*((poseMessage.rot[2]*poseMessage.rot[2])+(poseMessage.rot[3]*poseMessage.rot[3]))))));

  //frc::Rotation2d noHeading = frc::Rotation2d{0_deg};

  frc::Pose2d robotPose{units::meter_t(poseMessage.pos[0] * -1), units::meter_t(poseMessage.pos[1] * -1), heading};

  poseInfo p = { poseMessage, robotPose };

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
}

void updateParametersFromString(string parametersString) {
  msgpack::object_handle oh = msgpack::unpack(parametersString.data(), parametersString.size());

  msgpack::object deserialized = oh.get();
  
  std::cout << "updated parameters: " << deserialized << std::endl;

  ParametersMessage parametersMessage;
  deserialized.convert(parametersMessage);

  updateParameters(parametersMessage);
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

void updateActiveWaypointsFromJSON(std::string waypointsJSONString) {
  
  activeWaypoints.clear();
  
  std::cout << "here 1" << std::endl;

  json waypointJSON = json::parse(waypointsJSONString);
  
  std::cout << "pared!" << std::endl;

  if(waypointJSON.find("waypoints") != waypointJSON.end()) {
    
    std::cout << "here 2" << std::endl;

    poseInfo currentPose = getCurrentPose();

    //activeWaypoints.push_back(currentPose.pose.Translation());

    for(json::iterator it = waypointJSON["waypoints"].begin(); it != waypointJSON["waypoints"].end(); ++it) {
      std::cout << "here 3" << std::endl;

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
  double backRightMotorOutput

  ) {
  int64_t currentTimestamp = duration_cast<microseconds>(system_clock::now().time_since_epoch()).count();
  int64_t timestamp = currentTimestamp - startupTimestamp;

  double xPositionError = controller->getXController().GetPositionError();
  double xVelocityError = controller->getXController().GetVelocityError();

  double yPositionError = controller->getYController().GetPositionError();
  double yVelocityError = controller->getYController().GetVelocityError();
  
  std::cout << "error: " << xPositionError << std::endl;
  std::cout << "error: " << yPositionError << std::endl;

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

  ControllerStateMessage message = {
    timestamp,

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

  redis->publish(CONTROLLER_STATE_KEY, packed.str());
}

enum messageTypes {
  PARAMETERS_MESSAGE,
  TRAJECTORY_MESSAGE,
  COMMAND_MESSAGE,

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

void runActiveTrajectory() {
  LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(controllerUpdateRate);

  lastTrajectoryTime  = duration_cast<microseconds>(system_clock::now().time_since_epoch()).count();
  trajectoryStartTime = duration_cast<microseconds>(system_clock::now().time_since_epoch()).count();

  bool trajectoryComplete = false;

  poseInfo startingRobotPose = getCurrentPose();

  /*auto waypoints = std::vector{startingRobotPose.pose,
                               frc::Pose2d{0.3_m, 0.0_m, startingRobotPose.pose.Rotation() }};
  auto trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
      waypoints, {0.4_mps, 0.4_mps_sq});*/

  while(keepRunning && !trajectoryComplete) {
    /*try {
      subscriber->consume();
    } catch (const Error &err) {}*/
    
    std::cout << "next loop" << std::endl;

    timer.waitForNextLoop();

    int64_t currentTime = duration_cast<microseconds>(system_clock::now().time_since_epoch()).count();
    int64_t dt = currentTime - lastTrajectoryTime;

    lastTrajectoryTime = currentTime;
    int64_t elapsedTime = currentTime - trajectoryStartTime;

    poseInfo robotPose = getCurrentPose();
    
    std::cout << "got current pose" << std::endl;

    if (abs(robotPose.pose.X().value()) > maxXPosition || abs(robotPose.pose.Y().value()) > maxYPosition) {
      std::cout << "Out of bounds!" << std::endl;

      stopWheels();

      continue;
    }

    if(units::second_t(elapsedTime * 0.000001) > activeTrajectory.TotalTime()) {
      trajectoryComplete = true;

      std::cout << "Trajectory complete!" << std::endl;

      continue; 
    }

    frc::Trajectory::State state = activeTrajectory.Sample(units::second_t (elapsedTime * .000001));
    json stateTrajectoryJSON;
    
    std::cout << "sample trajectory" << std::endl;

    frc::to_json(stateTrajectoryJSON, state);

    json trajectoryInfoJSON;

    trajectoryInfoJSON["timestamp"] = robotPose.message.timestamp;
    trajectoryInfoJSON["trajectory"] = stateTrajectoryJSON;

    redis->publish(TRAJECTORY_SAMPLE_KEY, trajectoryInfoJSON.dump()); 
    
    std::cout << "publish trajectory" << std::endl;

    frc::ChassisSpeeds targetChassisSpeeds = controller->Calculate(robotPose.pose, state, startingRobotPose.pose.Rotation());
    
    frc::MecanumDriveWheelSpeeds targetWheelSpeeds = driveKinematics->ToWheelSpeeds(targetChassisSpeeds);

    units::volt_t frontLeftFeedforward  = wheelMotorFeedforward->Calculate(targetWheelSpeeds.frontLeft);
    units::volt_t frontRightFeedforward = wheelMotorFeedforward->Calculate(targetWheelSpeeds.frontRight);
    units::volt_t backLeftFeedforward   = wheelMotorFeedforward->Calculate(targetWheelSpeeds.rearLeft);
    units::volt_t backRightFeedforward  = wheelMotorFeedforward->Calculate(targetWheelSpeeds.rearRight);

    WheelStatusMessage wheelStatus = getCurrentWheelStatus();

    // 1 - back right, 2 - front right, 3 - front left, 4 - back left

    double frontLeftOutput  = frontLeftWheelController->Calculate(  rpmToVelocity(wheelStatus.velocity[2]),  targetWheelSpeeds.frontLeft.value());
    double frontRightOutput = frontRightWheelController->Calculate( rpmToVelocity(wheelStatus.velocity[1]),  targetWheelSpeeds.frontRight.value());
    double backLeftOutput   = backLeftWheelController->Calculate(   rpmToVelocity(wheelStatus.velocity[3]),  targetWheelSpeeds.rearLeft.value());
    double backRightOutput  = backRightWheelController->Calculate(  rpmToVelocity(wheelStatus.velocity[0]),  targetWheelSpeeds.rearRight.value());

    // wheel data is
    // 3 - back left
    // 2 - front left
    // 0 - back right
    // 1 - front right

    publishControlState(
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
      normalizeMotorVoltage(backRightOutput, backRightFeedforward)
    );
    
    /*std::cout << "publish control state" << std::endl;
    
    std::cout << "PID Controller outputs:";
    std::cout << backRightOutput << std::endl;
    std::cout << frontRightOutput << std::endl;
    std::cout << frontLeftOutput << std::endl;
    std::cout << backLeftOutput << std::endl;

    std::cout << "feedforward outputs:";

    std::cout << backRightFeedforward.value() << std::endl;
    std::cout << frontRightFeedforward.value() << std::endl;
    std::cout << frontLeftFeedforward.value() << std::endl;
    std::cout << backLeftFeedforward.value() << std::endl;
*/

    /*std::cout << "Target wheel speeds outputs:";
    std::cout << targetWheelSpeeds.rearRight.value() << std::endl;
    std::cout << targetWheelSpeeds.frontRight.value() << std::endl;
    std::cout << targetWheelSpeeds.frontLeft.value() << std::endl;
    std::cout << targetWheelSpeeds.rearLeft.value() << std::endl;

    // 0 - back right
    // 1 - front right
    // 2 - front left
    // 3 - back left 
    /*WheelVoltageMessage message = {
      uint32_t(elapsedTime), 
      
      {
        int16_t (targetWheelSpeeds.rearRight * maxWheelVoltage),
        int16_t (targetWheelSpeeds.frontRight * maxWheelVoltage),
        int16_t (targetWheelSpeeds.frontLeft * maxWheelVoltage),
        int16_t (targetWheelSpeeds.rearLeft * maxWheelVoltage)
      }
    };*/
    
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
    
    std::cout << "packed message" << std::endl;

    redis->set(WHEEL_VOLTAGE_COMMAND_KEY, packed.str()); 
    
    std::cout << "set packed message" << std::endl;

    packed.seekg(0);

    std::string str(packed.str());

    msgpack::object_handle oh =
    msgpack::unpack(str.data(), str.size());

    msgpack::object deserialized = oh.get();
    std::cout << deserialized << std::endl;
  }

  stopWheels();

}

int main(int argc, char **argv) {
	signal(SIGABRT, intHandler);
	signal(SIGTERM, intHandler);
	signal(SIGINT, intHandler);

  setbuf(stdout, NULL);
  
  ConnectionOptions redisConnectionOpts;

  redisConnectionOpts.host = "127.0.0.1";
  redisConnectionOpts.port = 6379;
  redisConnectionOpts.socket_timeout = std::chrono::milliseconds(5); 
  
  redis = new Redis(redisConnectionOpts);

  auto timestamp = redis->get(STARTUP_TIMESTAMP_KEY);
  if(timestamp) {
    string timestampString = *timestamp;
    
    startupTimestamp = atoll(timestampString.c_str());
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

        updateActiveWaypointsFromJSON(msg);

        break;

      case COMMAND_MESSAGE:
        json commandMessageJSON = json::parse(msg);

        if(commandMessageJSON.find("command") != commandMessageJSON.end()) {
          switch (getCommandType(commandMessageJSON["command"].get<std::string>())) {
            case STOP_WHEELS_COMMAND:
              stopWheels();

            case RUN_TRAJECTORY_COMMAND:
              stopWheels();

              runActiveTrajectory();
            break;
          }
        }
    }
  });

  subscriber->subscribe(PARAMETERS_KEY);
  subscriber->subscribe(TRAJECTORY_KEY);
  subscriber->subscribe(COMMAND_KEY);

  while(keepRunning) {
    try {
      subscriber->consume();
    } catch (const Error &err) {}
  }

  stopWheels();

	return 0;
}
