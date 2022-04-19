#include <stdio.h>
#include <string.h>
#include "iostream"
#include <sstream>

#include <msgpack.hpp>

#define EIGEN_MATRIXBASE_PLUGIN "MatrixBaseAddons.h"

#include <survive_api.h>

#include <survive.h>

#include <os_generic.h>

#include <assert.h>
#include <signal.h>
#include <stdlib.h>

#include <sw/redis++/redis++.h>

using namespace sw::redis;

static volatile int keepRunning = 1;
SurviveSimpleContext *actx; 

void intHandler(int dummy) {
	if (keepRunning == 0) {
		survive_simple_close(actx);
		exit(-1);
	}
	keepRunning = 0;
}

using namespace std;

const string POSE_DATA_KEY = "rover_pose";
const string POSE_VELOCITY_DATA_KEY = "rover_pose_velocity";
const string POSE_CONFIG_KEY = "rover_pose_config";
const string BASE_POSE_KEY = "rover_base_pose";

static void log_fn(SurviveSimpleContext *actx, SurviveLogLevel logLevel, const char *msg) {
	fprintf(stderr, "(%7.3f) SimpleApi: %s\n", survive_simple_run_time(actx), msg);
}

struct PoseMessage {
  PoseMessage(FLT _timestamp, SurvivePose _pose) {
    timestamp = _timestamp;

    pos[0] = _pose.Pos[0];
    pos[1] = _pose.Pos[1];
    pos[2] = _pose.Pos[2];

    rot[0] = _pose.Rot[0];
    rot[1] = _pose.Rot[1];
    rot[2] = _pose.Rot[2];
    rot[3] = _pose.Rot[3];
  }

  FLT timestamp;

  std::array<FLT, 3> pos = {0.0, 0.0, 0.0};
  std::array<FLT, 4> rot = {0.0, 0.0, 0.0, 0.0};

  MSGPACK_DEFINE_MAP(timestamp, pos, rot)
};

struct VelocityMessage {
  VelocityMessage(FLT _timestamp, SurviveVelocity _pose, SurviveAngularVelocity _theta) {
    timestamp = _timestamp;

    pos[0] = _pose.Pos[0];
    pos[1] = _pose.Pos[1];
    pos[2] = _pose.Pos[2];

    theta[0] = _theta[0];
    theta[1] = _theta[1];
    theta[2] = _theta[2];
  }

  FLT timestamp;

  std::array<FLT, 3> pos   = {0.0, 0.0, 0.0};
  std::array<FLT, 3> theta = {0.0, 0.0, 0.0};

  MSGPACK_DEFINE_MAP(timestamp, pos, theta)
};


int main(int argc, char **argv) {
	signal(SIGABRT, intHandler);
	signal(SIGTERM, intHandler);
	signal(SIGINT, intHandler);

  ConnectionOptions redisConnectionOptions;
  redisConnectionOptions.type = ConnectionType::UNIX;
  redisConnectionOptions.path = "/var/run/redis/redis-server.sock";
  redisConnectionOptions.socket_timeout = std::chrono::milliseconds(5);

  auto redis = Redis(redisConnectionOptions);

	actx = survive_simple_init_with_logger(argc, argv, log_fn);
	
	double start_time = OGGetAbsoluteTime();
	survive_simple_start_thread(actx);

	for (const SurviveSimpleObject *it = survive_simple_get_first_object(actx); it != 0;
		 it = survive_simple_get_next_object(actx, it)) {
		printf("Found '%s'\n", survive_simple_object_name(it));
	}

	struct SurviveSimpleEvent event = {};
	
  SurvivePose lastPose = { .Pos = { 0.0, 0.0, 0.0 }, .Rot = { 0.0, 0.0, 0.0, 0.0 } };
  FLT lastPoseMessageTime = 0.0;

	while (survive_simple_wait_for_event(actx, &event) != SurviveSimpleEventType_Shutdown) {
		//printf("%d Got event!\n", event.event_type);

		switch (event.event_type) {
		case SurviveSimpleEventType_PoseUpdateEvent: {
			const struct SurviveSimplePoseUpdatedEvent *pose_event = survive_simple_get_pose_updated_event(&event);
	
      if(survive_simple_object_get_type(pose_event->object) == SurviveSimpleObject_LIGHTHOUSE) {
        
        FLT timecode = survive_simple_run_time(actx);

        SurvivePose pose = pose_event->pose;
       
        printf("%s %s (%7.3f): %f %f %f %f %f %f %f\n", survive_simple_object_name(pose_event->object),
             survive_simple_serial_number(pose_event->object), timecode, pose.Pos[0], pose.Pos[1], pose.Pos[2],
             pose.Rot[0], pose.Rot[1], pose.Rot[2], pose.Rot[3]);


        PoseMessage message = {timecode, pose};

        printf("Timecode %7.3f \n", timecode);
        printf("BASE POS: %f %f %f \n", message.pos[0], message.pos[1], message.pos[2]);
        printf("BASE ROT: %f %f %f %f \n", message.rot[0], message.rot[1], message.rot[2], message.rot[3]);


        std::stringstream packed;
        msgpack::pack(packed, message);

        packed.seekg(0);

        redis.set(BASE_POSE_KEY + "-" + survive_simple_object_name(pose_event->object) + '-' + survive_simple_serial_number(pose_event->object), packed.str());

        packed.seekg(0);

        std::string str(packed.str());

        msgpack::object_handle oh =
        msgpack::unpack(str.data(), str.size());

        msgpack::object deserialized = oh.get();
        std::cout << deserialized << std::endl;

      } else {

        SurvivePose pose = pose_event->pose;
        SurviveVelocity velocity = pose_event->velocity;

        FLT timecode = pose_event->time;
       
        PoseMessage message = {timecode, pose};

        SurviveAngularVelocity thetaVel = {0.0, 0.0, 0.0};

        if(lastPoseMessageTime != 0.0) {
          survive_find_ang_velocity(thetaVel, timecode - lastPoseMessageTime, lastPose.Rot, pose.Rot); 
        }

        VelocityMessage velMessage = {timecode, velocity, thetaVel};

        std::stringstream packed;
        msgpack::pack(packed, message);

        packed.seekg(0);

        std::stringstream packedVel;
        msgpack::pack(packedVel, velMessage);

        packedVel.seekg(0);

        redis.set(POSE_DATA_KEY, packed.str());
        redis.set(POSE_VELOCITY_DATA_KEY, packedVel.str());

        lastPoseMessageTime = timecode;
        lastPose = pose;

        /*printf("%s %s (%7.3f): %f %f %f %f %f %f %f\n", survive_simple_object_name(pose_event->object),
             survive_simple_serial_number(pose_event->object), timecode, pose.Pos[0], pose.Pos[1], pose.Pos[2],
             pose.Rot[0], pose.Rot[1], pose.Rot[2], pose.Rot[3]);*/
      }
			break;
		}
		case SurviveSimpleEventType_ButtonEvent: {
			const struct SurviveSimpleButtonEvent *button_event = survive_simple_get_button_event(&event);
			SurviveObjectSubtype subtype = survive_simple_object_get_subtype(button_event->object);
			printf("%s input %s (%d) ", survive_simple_object_name(button_event->object),
				   SurviveInputEventStr(button_event->event_type), button_event->event_type);

			FLT v1 = survive_simple_object_get_input_axis(button_event->object, SURVIVE_AXIS_TRACKPAD_X) / 2. + .5;

			if (button_event->button_id != 255) {
				printf(" button %16s (%2d) ", SurviveButtonsStr(subtype, button_event->button_id),
					   button_event->button_id);

				if (button_event->button_id == SURVIVE_BUTTON_SYSTEM) {
					FLT v = 1 - survive_simple_object_get_input_axis(button_event->object, SURVIVE_AXIS_TRIGGER);
					survive_simple_object_haptic(button_event->object, 30, v, .5);
				}
			}
			for (int i = 0; i < button_event->axis_count; i++) {
				printf(" %20s (%2d) %+5.4f   ", SurviveAxisStr(subtype, button_event->axis_ids[i]),
					   button_event->axis_ids[i], button_event->axis_val[i]);
			}
			printf("\n");
			break;
		}
		case SurviveSimpleEventType_ConfigEvent: {
			const struct SurviveSimpleConfigEvent *cfg_event = survive_simple_get_config_event(&event);
			printf("(%f) %s received configuration of length %u type %d-%d\n", cfg_event->time,
				   survive_simple_object_name(cfg_event->object), (unsigned)strlen(cfg_event->cfg),
				   survive_simple_object_get_type(cfg_event->object),
				   survive_simple_object_get_subtype(cfg_event->object));

      redis.set(POSE_CONFIG_KEY, cfg_event->cfg);
			break;
		}
		case SurviveSimpleEventType_DeviceAdded: {
			const struct SurviveSimpleObjectEvent *obj_event = survive_simple_get_object_event(&event);
			printf("(%f) Found '%s'\n", obj_event->time, survive_simple_object_name(obj_event->object));
			break;
		}
		case SurviveSimpleEventType_None:
			break;
		}
	}

	printf("Cleaning up\n");
	survive_simple_close(actx);

	return 0;
}
