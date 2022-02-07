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

#include <Eigen/Dense>

#include "utils/LoopTimer.h"

#include <frc/MathUtil.h>

#include <frc/controller/PIDController.h>
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

#define FLT float

using namespace sw::redis;
using namespace std::chrono;

using json = wpi::json;

using radians_per_second_squared_t =
    units::compound_unit<units::radians,
                         units::inverse<units::squared<units::second>>>;

static volatile int keepRunning = 1;

void intHandler(int dummy) {
	if (keepRunning == 0) {
		exit(-1);
	}
	keepRunning = 0;
}

using namespace std;

const string STARTUP_TIMESTAMP_KEY ="rover_startup_timestamp";

const string POSE_DATA_KEY = "rover_pose";
const string POSE_VELOCITY_DATA_KEY = "rover_pose_velocity";
const string WHEEL_VELOCITY_COMMAND_KEY = "rover_wheel_velocity_command";
const string TRAJECTORY_SAMPLE_KEY = "rover_trajectory_sample";

const string PARAMETERS_KEY = "rover_parameters";
const string TRAJECTORY_KEY = "rover_trajectory";

const string CONTROLLER_STATE_KEY = "rover_control_state";

struct PoseMessage {
  FLT timestamp;

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
    double _xAtSetpoint,
    double _yAtSetpoint,
    double _thetaAtSetpoint
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
    thetaAtSetpoint
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

  double xControllerP = 1.5;
  double xControllerI = 0.0;
  double xControllerD = 0.0;

  double yControllerP = 1.0;
  double yControllerI = 0.0;
  double yControllerD = 0.0;

  double thetaControllerP = 1.5;
  double thetaControllerI = 0.0;
  double thetaControllerD = 0.0;

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

    xControllerP,
    xControllerI,
    xControllerD,

    yControllerP,
    yControllerI,
    yControllerD,

    thetaControllerP,
    thetaControllerI,
    thetaControllerD
  )
};

struct wheelVelocityMessage {
  uint32_t timestamp; 
  int16_t velocity[4];  

  MSGPACK_DEFINE_MAP(timestamp, velocity)
};

# define TAU M_PI * 2
# define WHEEL_RADIUS 45

int16_t velocityToRPM(units::meters_per_second_t speed) {
  return int16_t ((60 * speed) / ((WHEEL_RADIUS * 0.001 * M_PI) * 2));
}

Redis* redis;

void stopWheels() {
  wheelVelocityMessage message { 0, { 0, 0, 0, 0 } };

  std::stringstream packed;
  msgpack::pack(packed, message);
  
  packed.seekg(0);

  redis->set(WHEEL_VELOCITY_COMMAND_KEY, packed.str()); 
}

frc::Pose2d getCurrentPose() {
  auto pose = redis->get(POSE_DATA_KEY);
    
  string s = *pose;
  msgpack::object_handle oh = msgpack::unpack(s.data(), s.size());

  msgpack::object deserialized = oh.get();

  PoseMessage poseMessage { 0, { 0, 0, 0 }, { 0, 0, 0, 0 } };
  deserialized.convert(poseMessage);

  frc::Rotation2d heading = frc::Rotation2d(units::radian_t (
    atan2(2*((poseMessage.rot[0]*poseMessage.rot[3])+(poseMessage.rot[1]*poseMessage.rot[2])),1-(2*((poseMessage.rot[2]*poseMessage.rot[2])+(poseMessage.rot[3]*poseMessage.rot[3]))))));

  //frc::Rotation2d noHeading = frc::Rotation2d{0_deg};

  frc::Pose2d robotPose{units::meter_t(poseMessage.pos[0] * -1), units::meter_t(poseMessage.pos[1] * -1), heading};

  return robotPose;
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

double xControllerP;
double xControllerI;
double xControllerD;

double yControllerP;
double yControllerI;
double yControllerD;

double thetaControllerP;
double thetaControllerI;
double thetaControllerD;

frc::MecanumDriveKinematics* driveKinematics;
frc::HolonomicDriveController* controller;

frc2::PIDController* xController;
frc2::PIDController* yController;
frc::ProfiledPIDController<units::radian>* thetaController;

frc::TrapezoidProfile<units::radian>::Constraints* thetaConstraints;

std::vector<frc::Pose2d> activeWaypoints;

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
  
  maxXPosition = parametersMessage.maxXPosition;
  maxYPosition = parametersMessage.maxYPosition;

  driveKinematics = new frc::MecanumDriveKinematics(
    frc::Translation2d(wheelBase / 2, trackWidth / 2),
    frc::Translation2d(wheelBase / 2, -trackWidth / 2),
    frc::Translation2d(-wheelBase / 2, trackWidth / 2),
    frc::Translation2d(-wheelBase / 2, -trackWidth / 2)
  );

  controllerUpdateRate = parametersMessage.controllerUpdateRate;

  xControllerP = parametersMessage.xControllerP;
  xControllerI = parametersMessage.xControllerI;
  xControllerD = parametersMessage.xControllerD;
  
  if(xController == NULL) {
    xController = new frc2::PIDController(xControllerP, xControllerI, xControllerD);
  } else {
    xController->SetPID(xControllerP, xControllerI, xControllerD);
  }

  yControllerP = parametersMessage.yControllerP;
  yControllerI = parametersMessage.yControllerI;
  yControllerD = parametersMessage.yControllerD;

  if(yController == NULL) {
    yController = new frc2::PIDController(yControllerP, yControllerI, yControllerD);
  } else {
    yController->SetPID(yControllerP, yControllerI, yControllerD);
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
    thetaController->SetPID(thetaControllerP, thetaControllerI, thetaControllerD);
  }

  if(controller == NULL) {
    controller = new frc::HolonomicDriveController(
      *xController,
      *yController,
      *thetaController
    );
  }
}

void updateParametersFromString(string parametersString) {
  msgpack::object_handle oh = msgpack::unpack(parametersString.data(), parametersString.size());

  msgpack::object deserialized = oh.get();
  
  std::cout << "updated parameters: " << deserialized << std::endl;

  ParametersMessage parametersMessage;
  deserialized.convert(parametersMessage);

  updateParameters(parametersMessage);
}

void updateActiveWaypointsFromJSON(std::string waypointsJSONString) {
  
  activeWaypoints.clear();
  
  json waypointJSON = json::parse(waypointsJSONString);

  if(waypointJSON.find("waypoints") != waypointJSON.end()) {
  
    for(json::iterator it = waypointJSON["waypoints"].begin(); it != waypointJSON["waypoints"].end(); ++it) {
      frc::Pose2d p;

      frc::from_json(it.value(), p);

      activeWaypoints.push_back(p);
    }
  }
}

void dumpWaypoints() {
  for(frc::Pose2d p : activeWaypoints) {
    json waypointJSON;
    frc::to_json(waypointJSON, p);

    std::cout << "waypoints: " << waypointJSON.dump() << std::endl;
  }
}

void publishControlState() {
  int64_t currentTimestamp = duration_cast<microseconds>(system_clock::now().time_since_epoch()).count();
  int64_t timestamp = currentTimestamp - startupTimestamp;

  double xPositionError = xController->GetPositionError();
  double xVelocityError = xController->GetVelocityError();
  
  double yPositionError = yController->GetPositionError();
  double yVelocityError = yController->GetVelocityError();
  
  frc::ProfiledPIDController<units::radian>::Distance_t thetaPositionError = thetaController->GetPositionError();
  frc::ProfiledPIDController<units::radian>::Velocity_t thetaVelocityError = thetaController->GetVelocityError();
  
  double xSetpoint = xController->GetSetpoint();
  double ySetpoint = yController->GetSetpoint();
  
  frc::ProfiledPIDController<units::radian>::State thetaState = thetaController->GetSetpoint();

  double xAtSetpoint = xController->AtSetpoint();
  double yAtSetpoint = yController->AtSetpoint();
  double thetaAtSetpoint = thetaController->AtSetpoint();


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
    thetaAtSetpoint
  };

  std::stringstream packed;
  msgpack::pack(packed, message);

  packed.seekg(0);

  redis->publish(CONTROLLER_STATE_KEY, packed.str());
}

enum messageTypes {
  PARAMETERS_MESSAGE,
  TRAJECTORY_MESSAGE,

  UNKNOWN_MESSAGE
};

messageTypes getMessageType(std::string const& messageType) {
  if(messageType == PARAMETERS_KEY) return PARAMETERS_MESSAGE;
  if(messageType == TRAJECTORY_KEY) return TRAJECTORY_MESSAGE;

  return UNKNOWN_MESSAGE;
}

int main(int argc, char **argv) {
	signal(SIGABRT, intHandler);
	signal(SIGTERM, intHandler);
	signal(SIGINT, intHandler);

  setbuf(stdout, NULL);
  
  ConnectionOptions redisConnectionOpts;

  redisConnectionOpts.host = "127.0.0.1";
  redisConnectionOpts.port = 6379;
  redisConnectionOpts.socket_timeout = std::chrono::milliseconds(1); 
  
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

  frc::Pose2d robotPose = getCurrentPose();

  auto waypoints = redis->get(TRAJECTORY_KEY);

  if(waypoints) {
    string waypointsString = *waypoints;

    updateActiveWaypointsFromJSON(waypointsString);

  } else {
    activeWaypoints.push_back(robotPose);
    activeWaypoints.push_back(frc::Pose2d{0.5_m, 0.0_m, robotPose.Rotation()}); 
  }

  dumpWaypoints();
  
  auto subscriber = redis->subscriber();

  subscriber.on_message([](std::string channel, std::string msg) {
    switch (getMessageType(channel)) {
      case PARAMETERS_MESSAGE:
        updateParametersFromString(msg);
        
        break;
        
      case TRAJECTORY_MESSAGE:
        std::cout << "got trajectory!" << std::endl;

        updateActiveWaypointsFromJSON(msg);

        break;
    }
  });

  subscriber.subscribe(PARAMETERS_KEY);
  subscriber.subscribe(TRAJECTORY_KEY);

  auto trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
      activeWaypoints, {0.4_mps, 0.4_mps_sq});

  units::time::second_t totalTime = trajectory.TotalTime();
  
  json sampledTrajectoryJSON = json::array(); 
  for (int i = 0; i <= int(ceil(totalTime.value() / (controllerUpdateRate * 0.001))); i++) {
    frc::Trajectory::State state = trajectory.Sample(units::second_t (i * controllerUpdateRate * 0.001));

    json stateTrajectoryJSON;
    frc::to_json(stateTrajectoryJSON, state);

    sampledTrajectoryJSON.push_back(stateTrajectoryJSON);
  }

  redis->publish(TRAJECTORY_SAMPLE_KEY, sampledTrajectoryJSON.dump()); 
  
  LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(controllerUpdateRate);

  int64_t lastTime = duration_cast<microseconds>(system_clock::now().time_since_epoch()).count();
  int64_t startTime = duration_cast<microseconds>(system_clock::now().time_since_epoch()).count();

  printf("total time: %f \n", (double)totalTime.value());

  frc::Pose2d startingRobotPose = getCurrentPose();

  while(keepRunning) {
    try {
      subscriber.consume();
    } catch (const Error &err) {}

    timer.waitForNextLoop();

    int64_t currentTime = duration_cast<microseconds>(system_clock::now().time_since_epoch()).count();
    int64_t dt = currentTime - lastTime;

    lastTime = currentTime;

    int64_t elapsedTime = currentTime - startTime;

    frc::Pose2d robotPose = getCurrentPose();

    if (abs(robotPose.X().value()) > maxXPosition || abs(robotPose.Y().value()) > maxYPosition) {
      std::cout << "Out of bounds!" << std::endl;

      stopWheels();

      continue;
    }

    publishControlState();
    
    //printf("elapsed time: %ld \n", elapsedTime);
    //if(totalTime > (units::second_t (elapsedTime / 1000000))) {
      frc::Trajectory::State state = trajectory.Sample(units::second_t (dt * 1000000));
     
     // continue;
      frc::ChassisSpeeds targetChassisSpeeds = controller->Calculate(robotPose, state, startingRobotPose.Rotation());
      
      frc::MecanumDriveWheelSpeeds targetWheelSpeeds = driveKinematics->ToWheelSpeeds(targetChassisSpeeds);
      
      wheelVelocityMessage message = { 
        uint32_t(elapsedTime), 
        
        {
          velocityToRPM(targetWheelSpeeds.frontRight),
          velocityToRPM(targetWheelSpeeds.frontLeft),
          velocityToRPM(targetWheelSpeeds.rearLeft),
          velocityToRPM(targetWheelSpeeds.rearRight)
        }
      };

      //printf("%f %f %f %f", (double)targetWheelSpeeds.frontRight, (double)targetWheelSpeeds.frontLeft, (double)targetWheelSpeeds.rearLeft, (double)targetWheelSpeeds.rearRight);

      std::stringstream packed;
      msgpack::pack(packed, message);
  
      packed.seekg(0);

      redis->set(WHEEL_VELOCITY_COMMAND_KEY, packed.str()); 

        packed.seekg(0);

        std::string str(packed.str());

        msgpack::object_handle oh =
        msgpack::unpack(str.data(), str.size());

        msgpack::object deserialized = oh.get();
        std::cout << deserialized << std::endl;

    //} else {
    //  stopWheels();
   // }

  }


  stopWheels();

  double end_time = timer.elapsedTime();

  //auto& endPose = trajectory.States().back().pose;


	return 0;
}
