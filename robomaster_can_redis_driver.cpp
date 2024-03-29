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

#include <deque>
#include <iomanip>

#include <sw/redis++/redis++.h>

using robomaster::dds::metadata;
using robomaster::dds::wheel_encoders;
using robomaster::dds::battery;

using namespace std::chrono;

using namespace std;
using namespace sw::redis;

uint64_t startupTimestamp;

Redis* odometryMonitorRedisClient;
Redis* commandReceiverRedisClient;

static volatile int keepRunning = 1;

void intHandler(int dummy) {
	if (keepRunning == 0) {
		exit(-1);
	}
	keepRunning = 0;
}

const string WHEEL_ENCODER_KEY = "rover_wheel_encoder";
const string BATTERY_STATE_KEY = "rover_battery_state";
const string WHEEL_VELOCITY_COMMAND_KEY = "rover_wheel_velocity_command";
const string WHEEL_VELOCITY_SET_KEY = "rover_wheel_velocity_output";
const string STARTUP_TIMESTAMP_KEY ="rover_startup_timestamp";

struct wheelVelocityMessage {
  uint32_t timestamp; 
  int16_t velocity[4];  

  MSGPACK_DEFINE_MAP(timestamp, velocity)
};

void wheelVelocityTask() {
  commandReceiverRedisClient = new Redis("tcp://127.0.0.1:6379");

  commandReceiverRedisClient->set(STARTUP_TIMESTAMP_KEY, std::to_string(startupTimestamp));

  auto can = can_streambuf("can0", 0x201);
  std::iostream io(&can);

  robomaster::command::chassis chassis(io);

  chassis.send_workmode(1);

  printf("Started wheel velocity task \n");

  // reset wheel velocity command message
  wheelVelocityMessage message { 0, { 0, 0, 0, 0 } };

  std::stringstream packed;
  msgpack::pack(packed, message);
  
  packed.seekg(0);

  commandReceiverRedisClient->set(WHEEL_VELOCITY_COMMAND_KEY, packed.str()); 

  printf("Reset wheel speed command key \n");

  while(true) {
    chassis.send_heartbeat();
    
    auto val = commandReceiverRedisClient->get(WHEEL_VELOCITY_COMMAND_KEY);

    if(val) {
      string s = *val;
      msgpack::object_handle oh = msgpack::unpack(s.data(), s.size());

      msgpack::object deserialized = oh.get();

      wheelVelocityMessage message { 0, { 0, 0, 0, 0 } };
      deserialized.convert(message);

      chassis.send_wheel_speed(
        message.velocity[0], 
        message.velocity[1], 
        message.velocity[2], 
        message.velocity[3]
      );

      int64_t currentMicro = duration_cast<microseconds>(system_clock::now().time_since_epoch()).count();
      message.timestamp = uint64_t(currentMicro - startupTimestamp);

      std::stringstream packed;
      msgpack::pack(packed, message);
  
      packed.seekg(0);

      commandReceiverRedisClient->set(WHEEL_VELOCITY_SET_KEY, packed.str()); 
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

struct wheelEncoderMessage {
  int16_t rpm[4];
  uint16_t enc[4];
  //uint32_t timer[4];
  //uint8_t state[4];

  uint64_t timestamp;
  
  MSGPACK_DEFINE_MAP(timestamp, rpm, enc)
};

struct batteryStateMessage {
  uint16_t adc_val;
  int16_t temperature;
  int32_t current;
  uint8_t percent;

  uint64_t timestamp;

  MSGPACK_DEFINE_MAP(timestamp, adc_val, temperature, current, percent) 
};

int wheelCount = 0;
void wheelMonitor(const metadata& metadata, const wheel_encoders& wheel_encoders) {
  
  wheelEncoderMessage message;
 
  memcpy(&message.rpm, wheel_encoders.rpm, sizeof(message.rpm)); 
  memcpy(&message.enc, wheel_encoders.enc, sizeof(message.enc)); 
  
  memcpy(&message.timestamp, &metadata.time_ms, sizeof(message.timestamp)); 

  std::stringstream packed;
  msgpack::pack(packed, message);
  
  packed.seekg(0);

  odometryMonitorRedisClient->set(WHEEL_ENCODER_KEY, packed.str());
}


void batteryMonitor(const metadata& metadata, const battery& battery) {
  batteryStateMessage message;
 
  memcpy(&message.adc_val, &battery.adc_val, sizeof(message.adc_val)); 
  memcpy(&message.temperature, &battery.temperature, sizeof(message.temperature)); 
  memcpy(&message.current, &battery.current, sizeof(message.current)); 
  memcpy(&message.percent, &battery.percent, sizeof(message.percent)); 

  memcpy(&message.timestamp, &metadata.time_ms, sizeof(message.timestamp)); 

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

  printf("Started odometry monitor task \n");

  while (true) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000 / 10));
  }
}

int main(int argc, char **argv) {
	signal(SIGABRT, intHandler);
	signal(SIGTERM, intHandler);
	signal(SIGINT, intHandler);

  startupTimestamp = uint64_t(duration_cast<microseconds>(system_clock::now().time_since_epoch()).count());
  
  printf("Startup timestamp: %s \n", std::to_string(startupTimestamp).c_str());

  thread odometryMonitorThread(odometryMonitorTask);
  thread wheelVelocityThread(wheelVelocityTask);

  odometryMonitorThread.join();
  wheelVelocityThread.join();

	return 0;
}
