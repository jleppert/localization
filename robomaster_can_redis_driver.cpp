#include <stdio.h>
#include <string.h>
#include "iostream"
#include <sstream>

#include <os_generic.h>

#include <assert.h>
#include <signal.h>
#include <stdlib.h>

#include <msgpack.hpp>

#include <can_streambuf.hpp>
#include <crc.hpp>
#include <protocol.hpp>
#include <dds.hpp>
#include <chassis.hpp>

#include <chrono>
#include <thread>

#include <mutex>

#include <deque>
#include <iomanip>

#include <sw/redis++/redis++.h>

using robomaster::dds::metadata;
using robomaster::dds::wheel_encoders;
using robomaster::dds::battery;

using namespace std;
using namespace sw::redis;

Redis* odometryMonitorRedisClient;

std::mutex mutex1;

static volatile int keepRunning = 1;

void intHandler(int dummy) {
	if (keepRunning == 0) {
		exit(-1);
	}
	keepRunning = 0;
}

const string WHEEL_ENCODER_KEY = "rover_wheel_encoder";
const string WHEEL_VELOCITY_KEY = "rover_pose_config";
const string BATTERY_STATE_KEY = "rover_battery_state";

/*void wheelVelocityTask() {
	wheelVelocityRedisClient = new RedisClient();
	wheelVelocityRedisClient->connect();

  auto can = can_streambuf("can0", 0x201);
  std::iostream io(&can);

  robomaster::command::chassis chassis(io);

  chassis.send_workmode(1);

  while(true) {
    chassis.send_heartbeat();
    chassis.send_wheel_speed(30, 0, 0, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}*/


struct wheelEncoderMessage {
  int16_t rpm[4];
  uint16_t enc[4];
  //uint32_t timer[4];
  //uint8_t state[4];
  
  MSGPACK_DEFINE_MAP(rpm, enc)
};

struct batteryStateMessage {
  uint16_t adc_val;
  int16_t temperature;
  int32_t current;
  uint8_t percent;

  MSGPACK_DEFINE_MAP(adc_val, temperature, current, percent) 
};


int wheelCount = 0;
void wheelMonitor(const metadata&, const wheel_encoders& wheel_encoders) {
  
  wheelEncoderMessage message;
 
  memcpy(&message.rpm, wheel_encoders.rpm, sizeof(message.rpm)); 
  memcpy(&message.enc, wheel_encoders.enc, sizeof(message.enc)); 

  std::stringstream packed;
  msgpack::pack(packed, message);
  
  packed.seekg(0);

  odometryMonitorRedisClient->set(WHEEL_ENCODER_KEY, packed.str());
}


void batteryMonitor(const metadata&, const battery& battery) {
  batteryStateMessage message;
 
  memcpy(&message.adc_val, &battery.adc_val, sizeof(message.adc_val)); 
  memcpy(&message.temperature, &battery.temperature, sizeof(message.temperature)); 
  memcpy(&message.current, &battery.current, sizeof(message.current)); 
  memcpy(&message.percent, &battery.percent, sizeof(message.percent)); 

  std::stringstream packed;
  msgpack::pack(packed, message);
  
  packed.seekg(0);

  odometryMonitorRedisClient->set(BATTERY_STATE_KEY, packed.str());
}

void odometryMonitorTask() {
  odometryMonitorRedisClient = new Redis("tcp://127.0.0.1:6379");

  auto can_in = can_streambuf("can0", 0x202);
  auto can_cfg = can_streambuf("can0", 0x201);
  std::iostream in(&can_in);
  std::iostream out(&can_cfg);
  
  robomaster::dds::dds dds(in, out);

  dds.subscribe(std::function<void(const metadata&, const battery&)>(batteryMonitor), 1); 
  dds.subscribe(std::function<void(const metadata&, const wheel_encoders&)>(wheelMonitor), 20); 

  out.flush();

  printf("started battery monitor!");


  while (true) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000 / 10));
  }
}

int main(int argc, char **argv) {
	signal(SIGABRT, intHandler);
	signal(SIGTERM, intHandler);
	signal(SIGINT, intHandler);

  setbuf(stdout, NULL);

  printf("started!");

  thread odometryMonitorThread(odometryMonitorTask);

  odometryMonitorThread.join();

	return 0;
}
