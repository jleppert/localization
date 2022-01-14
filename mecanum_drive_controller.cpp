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
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/trajectory/TrapezoidProfile.h>

#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Pose2d.h>
#include "frc/geometry/Rotation2d.h"

#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/math.h>

#include <wpi/numbers>

#include <math.h>

#include <chrono>

#include <inttypes.h>

#define FLT float

using namespace sw::redis;
using namespace std::chrono;

static volatile int keepRunning = 1;

void intHandler(int dummy) {
	if (keepRunning == 0) {
		exit(-1);
	}
	keepRunning = 0;
}

using namespace std;

const string POSE_DATA_KEY = "rover_pose";
const string POSE_VELOCITY_DATA_KEY = "rover_pose_velocity";

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

static constexpr units::meter_t kTolerance{1 / 12.0};
static constexpr units::radian_t kAngularTolerance{2.0 * wpi::numbers::pi / 180.0};
 
constexpr auto kTrackWidth =
    200_mm;  // Distance between centers of right and left wheels on robot
constexpr auto kWheelBase =
    200_mm;  // Distance between centers of front and back wheels on robot

using radians_per_second_squared_t =
    units::compound_unit<units::radians,
                         units::inverse<units::squared<units::second>>>;

constexpr double kWheelDiameterMeters = 90 * 0.001;

constexpr auto kMaxSpeed = units::meters_per_second_t(0.3);
constexpr auto kMaxAcceleration = units::meters_per_second_squared_t(0.3);
constexpr auto kMaxAngularSpeed = units::radians_per_second_t(0.3);
constexpr auto kMaxAngularAcceleration =
    units::unit_t<radians_per_second_squared_t>(0.3);

constexpr double kPXController = 0.25;
constexpr double kPYController = 0.25;
constexpr double kPThetaController = 0.5;

const frc::MecanumDriveKinematics kDriveKinematics{
    frc::Translation2d(kWheelBase / 2, kTrackWidth / 2),
    frc::Translation2d(kWheelBase / 2, -kTrackWidth / 2),
    frc::Translation2d(-kWheelBase / 2, kTrackWidth / 2),
    frc::Translation2d(-kWheelBase / 2, -kTrackWidth / 2)
};

frc::MecanumDriveKinematics m_kinematics{ kDriveKinematics };

struct wheelVelocityMessage {
  uint32_t timestamp; 
  int16_t velocity[4];  

  MSGPACK_DEFINE_MAP(timestamp, velocity)
};

# define TAU M_PI * 2

int16_t velocityToRPM(units::meters_per_second_t speed) {
  //return int16_t ((speed * 60) / TAU); 

  return int16_t ((60 * speed) / ((45 * 0.001 * M_PI) * 2));
}

const string WHEEL_VELOCITY_COMMAND_KEY = "rover_wheel_velocity_command";


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

  printf("xy: %f %f \n", poseMessage.pos[0] * -1, poseMessage.pos[1] * -1);

  Eigen::Quaternionf orientation = Eigen::Quaternionf(poseMessage.rot[0], poseMessage.rot[1], poseMessage.rot[2], poseMessage.rot[3]);
  auto euler = orientation.toRotationMatrix().eulerAngles(0, 1, 2);
  std::cout << "Euler from quaternion in roll, pitch, yaw"<< std::endl << euler << std::endl;

  frc::Rotation2d heading = frc::Rotation2d(units::radian_t (orientation.toRotationMatrix().eulerAngles(0, 1, 2)[2]));

  //std::cout << "pose2d rotation" << std::endl << heading.Radians().value() << std::endl;

  //frc::Rotation2d heading2 = frc::Rotation2d{0_deg};

  frc::Pose2d robotPose{units::meter_t(poseMessage.pos[0] * -1), units::meter_t(poseMessage.pos[1] * -1), heading};

  return robotPose;
}

int main(int argc, char **argv) {
	signal(SIGABRT, intHandler);
	signal(SIGTERM, intHandler);
	signal(SIGINT, intHandler);

  setbuf(stdout, NULL);

  redis = new Redis("tcp://127.0.0.1:6379");


  frc::HolonomicDriveController controller{
      frc2::PIDController{1.5, 0.0, 0.0}, 

      frc2::PIDController{1.0, 0.0, 0.0},
      
      frc::ProfiledPIDController<units::radian>{
          0.1, 0.0, 0.0,
          frc::TrapezoidProfile<units::radian>::Constraints{ kMaxAngularSpeed, kMaxAngularAcceleration } }};

  frc::Pose2d robotPose = getCurrentPose();


  auto waypoints = std::vector{robotPose,
                               frc::Pose2d{0.5_m, 0.0_m, robotPose.Rotation() }};

  auto trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
      waypoints, {0.4_mps, 0.4_mps_sq});

  constexpr auto kDt = 0.02_s;

  LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(100); 

  auto totalTime = trajectory.TotalTime();

  int64_t lastTime = duration_cast<microseconds>(system_clock::now().time_since_epoch()).count();
  int64_t startTime = duration_cast<microseconds>(system_clock::now().time_since_epoch()).count();

  printf("total time: %f \n", totalTime);

  frc::Pose2d startingRobotPose = getCurrentPose();

  while(keepRunning) {
    timer.waitForNextLoop();
    
    int64_t currentTime = duration_cast<microseconds>(system_clock::now().time_since_epoch()).count();
    int64_t dt = currentTime - lastTime;

    lastTime = currentTime;

    int64_t elapsedTime = currentTime - startTime;

    frc::Pose2d robotPose = getCurrentPose();
    
    printf("elapsed time: %ld \n", elapsedTime);
    //if(totalTime > (units::second_t (elapsedTime / 1000000))) {
      frc::Trajectory::State state = trajectory.Sample(units::second_t (dt * 1000000));
     

      frc::ChassisSpeeds targetChassisSpeeds = controller.Calculate(robotPose, state, startingRobotPose.Rotation());
      
      frc::MecanumDriveWheelSpeeds targetWheelSpeeds = m_kinematics.ToWheelSpeeds(targetChassisSpeeds);
      
      wheelVelocityMessage message = { 
        uint32_t(elapsedTime), 
        
        {
          velocityToRPM(targetWheelSpeeds.frontRight),
          velocityToRPM(targetWheelSpeeds.frontLeft),
          velocityToRPM(targetWheelSpeeds.rearLeft),
          velocityToRPM(targetWheelSpeeds.rearRight)
        }
      };

      printf("%f %f %f %f", (double)targetWheelSpeeds.frontRight, (double)targetWheelSpeeds.frontLeft, (double)targetWheelSpeeds.rearLeft, (double)targetWheelSpeeds.rearRight);

      std::stringstream packed;
      msgpack::pack(packed, message);
  
      packed.seekg(0);

      //redis->set(WHEEL_VELOCITY_COMMAND_KEY, packed.str()); 

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
