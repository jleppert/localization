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

#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/trajectory/TrapezoidProfile.h>

#include <frc/geometry/Translation2d.h>

#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/math.h>

#include <wpi/numbers>

#include <chrono>

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
    205_mm;  // Distance between centers of right and left wheels on robot
constexpr auto kWheelBase =
    205_mm;  // Distance between centers of front and back wheels on robot

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


/*
function remapWheels(wheels) {
      return [
        wheels[2],
        wheels[3],
        wheels[1],
        wheels[0]
      ];
    }

    function linearVelocityToRPM(wheels) {
      return wheels.map(v => {
        return (v * 60 / (Math.PI * 2));
      });
    }

*/


frc::MecanumDriveKinematics m_kinematics{ kDriveKinematics };

int main(int argc, char **argv) {
	signal(SIGABRT, intHandler);
	signal(SIGTERM, intHandler);
	signal(SIGINT, intHandler);

  auto redis = Redis("tcp://127.0.0.1:6379");


  frc::HolonomicDriveController controller{
      frc2::PIDController{1.0, 0.0, 0.0}, frc2::PIDController{1.0, 0.0, 0.0},
      frc::ProfiledPIDController<units::radian>{
          1.0, 0.0, 0.0,
          frc::TrapezoidProfile<units::radian>::Constraints{ kMaxAngularSpeed, kMaxAngularAcceleration } }};

  frc::Pose2d robotPose{0_m, 0_m, frc::Rotation2d{0_deg}};


  auto waypoints = std::vector{frc::Pose2d{0.5_m, 0_m, 0_rad},
                               frc::Pose2d{0_m, 0_m, 0_rad}};

  auto trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
      waypoints, {0.3_mps, 0.2_mps_sq});

  constexpr auto kDt = 0.02_s;

  LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(100); 

  auto totalTime = trajectory.TotalTime();

  int64_t lastTime = duration_cast<microseconds>(system_clock::now().time_since_epoch()).count();
  int64_t startTime = duration_cast<microseconds>(system_clock::now().time_since_epoch()).count();

  while(keepRunning) {
    timer.waitForNextLoop();
    
    int64_t currentTime = duration_cast<microseconds>(system_clock::now().time_since_epoch()).count();
    int64_t dt = currentTime - lastTime;

    lastTime = currentTime;

    int64_t elapsedTime = currentTime - startTime;

    if(totalTime > (units::second_t (elapsedTime * 1000000))) {
      auto state = trajectory.Sample(units::second_t (dt * 1000000));

      auto targetChassisSpeeds = controller.Calculate(robotPose, state, 0_rad);
      
      auto targetWheelSpeeds = m_kinematics.ToWheelSpeeds(targetChassisSpeeds);

    }


  }

  double end_time = timer.elapsedTime();

  //auto& endPose = trajectory.States().back().pose;


	return 0;
}
