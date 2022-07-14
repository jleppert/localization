#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "iostream"
#include <sstream>

#include <os_generic.h>

#include <assert.h>
#include <signal.h>
#include <stdlib.h>

#include <unistd.h>

#include <msgpack.hpp>

#include <chrono>

#include <sw/redis++/redis++.h>

#include <sl/Camera.hpp>

using namespace std::chrono;

using namespace sl;

using namespace std;
using namespace sw::redis;

int64_t startupTimestamp;

Redis* redisClient;

static volatile int keepRunning = 1;

void intHandler(int dummy) {
	if (keepRunning == 0) {
		exit(-1);
	}
	keepRunning = 0;
}

const string POSE_KEY = "rover_camera_pose";
const string STARTUP_TIMESTAMP_KEY ="rover_startup_timestamp";

struct CameraPoseMessage {
  CameraPoseMessage(int64_t _timestamp, long long unsigned int _cameraTimestamp, sl::Pose _pose) {
    timestamp = _timestamp;
    cameraTimestamp = _cameraTimestamp;

    sl::Translation translation = _pose.getTranslation();
    sl::Orientation orientation = _pose.getOrientation();

    pos[0] = translation.tx;
    pos[1] = translation.ty;
    pos[2] = translation.tz;

    rot[0] = orientation.ox;
    rot[1] = orientation.oy;
    rot[2] = orientation.oz;
    rot[3] = orientation.oz;
  }

  int64_t timestamp;
  long long unsigned int cameraTimestamp;

  std::array<float, 3> pos = {0.0, 0.0, 0.0};
  std::array<float, 4> rot = {0.0, 0.0, 0.0, 0.0};

  MSGPACK_DEFINE_MAP(timestamp, cameraTimestamp, pos, rot)
};

void printMsgpackMessage(std::string packed) {
  auto oh = msgpack::unpack(packed.data(), packed.size());
  std::cout << oh.get() << std::endl;
}

int main(int argc, char **argv) {
	signal(SIGABRT, intHandler);
	signal(SIGTERM, intHandler);
	signal(SIGINT, intHandler);

  ConnectionOptions redisConnectionOptions;
  redisConnectionOptions.type = ConnectionType::UNIX;
  redisConnectionOptions.path = "/var/run/redis/redis-server.sock";

  redisClient = new Redis(redisConnectionOptions);
  
  auto timestamp = redisClient->get(STARTUP_TIMESTAMP_KEY);
  if(timestamp) {
    string timestampString = *timestamp;
    
    startupTimestamp = int64_t(atoll(timestampString.c_str()));
  } else {
    std::cout << "Startup timestamp not set or invalid" << std::endl;

    exit(0);
  }

  Camera zed;
  InitParameters cameraParams;
  cameraParams.coordinate_units = UNIT::METER;
  cameraParams.coordinate_system = COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP;
  cameraParams.sdk_verbose = true;
  
  /*
      sl::RESOLUTION::HD2K
      sl::RESOLUTION::HD1080
      sl::RESOLUTION::HD720
      sl::RESOLUTION::VGA
  */
  cameraParams.camera_resolution = sl::RESOLUTION::HD2K;

  auto returnedState = zed.open(cameraParams);
  if (returnedState != ERROR_CODE::SUCCESS) {
    std::cout << "Camera Open failed, exiting" << std::endl;
    return EXIT_FAILURE;
  }

  PositionalTrackingParameters positionTrackingParams;
  positionTrackingParams.enable_area_memory = true;

  returnedState = zed.enablePositionalTracking(positionTrackingParams);
  if (returnedState != ERROR_CODE::SUCCESS) {
    std::cout << "Enabling positionnal tracking API failure, exiting" << std::endl;
    zed.close();
    return EXIT_FAILURE;
  }

  Pose cameraPose;
  POSITIONAL_TRACKING_STATE trackingState;

  while(keepRunning) {
    if (zed.grab() == ERROR_CODE::SUCCESS) {
      trackingState = zed.getPosition(cameraPose, REFERENCE_FRAME::WORLD);

      if (trackingState == POSITIONAL_TRACKING_STATE::OK) {
        sl::Translation translation = cameraPose.getTranslation();
        sl::Orientation orientation = cameraPose.getOrientation();

        int64_t currentMicro = duration_cast<microseconds>(system_clock::now().time_since_epoch()).count();

        CameraPoseMessage message = {
          uint32_t(currentMicro - startupTimestamp),

          cameraPose.timestamp, 
          cameraPose
        };

        std::stringstream packed;
        msgpack::pack(packed, message);

        packed.seekg(0);

        redisClient->set(POSE_KEY, packed.str());
      }
    }
  }

  zed.close();

	return 0;
}
