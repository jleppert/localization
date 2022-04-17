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

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <set>

#include <chrono>
#include <thread>

#include <deque>
#include <iomanip>

#include <iostream>
#include <fstream>

#include <sw/redis++/redis++.h>

#define REVERSE -1

using namespace std::chrono;

using namespace std;
using namespace sw::redis;

int64_t startupTimestamp;

Redis* odometryMonitorRedisClient;
Redis* commandReceiverRedisClient;

static volatile int keepRunning = 1;

void intHandler(int dummy) {
	if (keepRunning == 0) {
		exit(-1);
	}
	keepRunning = 0;
}

const string WHEEL_VOLTAGE_COMMAND_KEY = "rover_wheel_voltage_command";
const string WHEEL_VOLTAGE_SET_KEY = "rover_wheel_voltage_output";
const string WHEEL_STATUS_KEY = "rover_wheel_status";
const string STARTUP_TIMESTAMP_KEY ="rover_startup_timestamp";

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

void wheelVoltageTask() {
  commandReceiverRedisClient = new Redis("tcp://127.0.0.1:6379");

  commandReceiverRedisClient->set(STARTUP_TIMESTAMP_KEY, std::to_string(startupTimestamp));

  int canSocket;

  if ((canSocket = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
    //perror("Socket");
    printf("Error opening socket! \n");
    exit(1);
  }

  struct ifreq ifr;

  strcpy(ifr.ifr_name, "can0");
  ioctl(canSocket, SIOCGIFINDEX, &ifr);

  struct sockaddr_can addr;
  memset(&addr, 0, sizeof(addr));

  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (bind(canSocket, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
   printf("Error binding! \n");
   exit(1);
   //perror("Bind");
  }

  printf("Started wheel voltage task \n");

  // reset wheel velocity command message
  WheelVoltageMessage message { 0, { 0, 0, 0, 0 } };

  std::stringstream packed;
  msgpack::pack(packed, message);
  
  packed.seekg(0);

  commandReceiverRedisClient->set(WHEEL_VOLTAGE_COMMAND_KEY, packed.str()); 

  printf("Reset wheel voltage command key \n");

  while(true) {
    auto val = commandReceiverRedisClient->get(WHEEL_VOLTAGE_COMMAND_KEY);

    if(val) {
      string s = *val;
      msgpack::object_handle oh = msgpack::unpack(s.data(), s.size());

      msgpack::object deserialized = oh.get();

      WheelVoltageMessage message { 0, { 0, 0, 0, 0 } };
      deserialized.convert(message);

      struct can_frame {
        canid_t can_id;  /* 32 bit CAN_ID + EFF/RTR/ERR flags */
        __u8    can_dlc; /* frame payload length in byte (0 .. 8) */
        __u8    __pad;   /* padding */
        __u8    __res0;  /* reserved / padding */
        __u8    __res1;  /* reserved / padding */
        __u8    data[8] __attribute__((aligned(8)));
      };

      struct can_frame frame;
      frame.can_id = 0x1ff;
      frame.can_dlc = 8;
      
      frame.data[0] = message.voltage[0] >> 8;
      frame.data[1] = message.voltage[0] && 0xFF;

      frame.data[2] = message.voltage[1] >> 8;
      frame.data[3] = message.voltage[1] && 0xFF;

      // readings from the opposite side need to be reversed

      frame.data[4] = (message.voltage[2] * REVERSE) >> 8;
      frame.data[5] = (message.voltage[2] * REVERSE) && 0xFF;

      frame.data[6] = (message.voltage[3] * REVERSE) >> 8;
      frame.data[7] = (message.voltage[3] * REVERSE) && 0xFF;

      if (write(canSocket, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        printf("Error writing frame! \n");
        exit(1);
      }

      int64_t currentMicro = duration_cast<microseconds>(system_clock::now().time_since_epoch()).count();
      message.timestamp = uint32_t(currentMicro - startupTimestamp);

      std::stringstream packed;
      msgpack::pack(packed, message);
  
      packed.seekg(0);

      commandReceiverRedisClient->set(WHEEL_VOLTAGE_SET_KEY, packed.str()); 
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

void printMsgpackMessage(std::string packed) {
  auto oh = msgpack::unpack(packed.data(), packed.size());
  std::cout << oh.get() << std::endl;
}

std::ofstream motorDataRaw;
std::ofstream motorDataAvg;

void logMotorDataToFile(int motorId, struct can_frame frame, WheelStatusMessage message) {
  motorDataRaw 
    << to_string(frame.can_id) << "," 
    << to_string(motorId) << "," 
    << to_string(message.angle[motorId]) << "," 
    << to_string(message.velocity[motorId]) << "," 
    << to_string(message.torque[motorId]) << "," 
    << to_string(message.temperature[motorId]) << "\n";
}

void logAvgMotorDataToFile(WheelStatusMessage message) {
  for(int id = 0; id < 4; id++) {
    motorDataAvg
      << to_string(message.timestamp) << ","
      << to_string(id) << ","
      << to_string(message.angle[id]) << ","
      << to_string(message.velocity[id]) << ","
      << to_string(message.torque[id]) << ","
      << to_string(message.temperature[id]) << "\n";
  }
}

int16_t be16_to_cpu_signed(const uint8_t data[2]) {
  
  int16_t r;
  memcpy(&r, data, sizeof r);
  return __builtin_bswap16(r);
}

int16_t le16_to_cpu_signed(const uint8_t data[2]) {
  
  int16_t r;
  uint16_t u = (unsigned)data[0] | ((unsigned)data[1] << 8);
  memcpy(&r, &u, sizeof r);
  
  return r;
}

// uncomment to enable logging motor data to CSV file for debug purposes
#define ENABLE_LOGGING 0

void wheelMonitorTask() {  
  if(ENABLE_LOGGING) {
    motorDataRaw.open ("motor_data_raw.csv");
    motorDataRaw << "can_id, motor_id, angle, velocity, torque, temperature\n";

    motorDataAvg.open ("motor_data_average.csv");
    motorDataAvg << "timestamp, motor_id, angle, velocity, torque, temperature\n";
  }

  odometryMonitorRedisClient = new Redis("tcp://127.0.0.1:6379");

  int canSocket;

  if ((canSocket = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
    printf("Error opening socket! \n");
    exit(1);
  }

  struct ifreq ifr;

  strcpy(ifr.ifr_name, "can0");
  ioctl(canSocket, SIOCGIFINDEX, &ifr);

  struct sockaddr_can addr;
  memset(&addr, 0, sizeof(addr));

  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (bind(canSocket, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
   printf("Error binding! \n");
   exit(1);
  }

  printf("Started wheel monitor task \n");

  // 1 KHz publish rate, how many messages to buffer for averaging, 10 ms
  float messageCount = 100;

  while (true) {
    WheelStatusMessage message;

    std::vector<WheelStatusMessage> messages;

    while(messages.size() < messageCount) {
    
      set <int> ids;  
      while (ids.size() != 4) {
        struct can_frame frame;
        int nbytes = read(canSocket, &frame, sizeof(struct can_frame));
        
        if(nbytes < 0) {
          printf("Skipping empty can frame \n");
          continue;
        }

        if(nbytes < sizeof(struct can_frame)) {
          printf("Skipping incomplete can frame \n");
          continue;
        }

        // ignore our own velocity frame
        if(frame.can_id == 0x1FF) continue;

        int id = frame.can_id - 0x205;

        ids.insert(id);
        
        uint8_t angleData[2] = {frame.data[0], frame.data[1]};
        message.angle[id] = be16_to_cpu_signed(angleData);

        uint8_t velocityData[2] = {frame.data[2], frame.data[3]};

        // motors on the opposite side need to be reversed
        message.velocity[id] = be16_to_cpu_signed(velocityData) * ((id == 2 || id == 3) ? REVERSE : 1);
        
        uint8_t torqueData[2] = {frame.data[4], frame.data[5]};
        message.torque[id] = be16_to_cpu_signed(torqueData);
        
        message.temperature[id] = frame.data[6];
        
        if(ENABLE_LOGGING) logMotorDataToFile(id, frame, message);
      }

      messages.push_back(message);
    }

    WheelStatusMessage averageMessage;
    
    std::array<int64_t, 4> angle = {0, 0, 0, 0};
    std::array<int64_t, 4> velocity = {0, 0, 0, 0};
    std::array<int64_t, 4> torque = {0, 0, 0, 0};
    std::array<int64_t, 4> temperature = {0, 0, 0, 0};

    for(int i = 0; i < messageCount; i++) {
      for(int j = 0; j < 4; j++) {
        angle[j] = angle[j] + messages[i].angle[j];
        velocity[j] = velocity[j] + messages[i].velocity[j];
        torque[j] = torque[j] + messages[i].torque[j];
        temperature[j] = temperature[j] + messages[i].temperature[j];

        averageMessage.angle[j] = angle[j] / messageCount;
        averageMessage.velocity[j] = velocity[j] / messageCount;
        averageMessage.torque[j] = torque[j] / messageCount;
        averageMessage.temperature[j] = temperature[j] / messageCount;
      }
    }

    int64_t currentMicro = duration_cast<microseconds>(system_clock::now().time_since_epoch()).count();
    message.timestamp = uint32_t(currentMicro - startupTimestamp);

    if(ENABLE_LOGGING) logAvgMotorDataToFile(message);

    std::stringstream packed;
    msgpack::pack(packed, averageMessage);

    packed.seekg(0);
    
    if(ENABLE_LOGGING) {
      logAvgMotorDataToFile(message);
      printMsgpackMessage(packed.str());
    }

    odometryMonitorRedisClient->set(WHEEL_STATUS_KEY, packed.str());

    //std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

int main(int argc, char **argv) {
	signal(SIGABRT, intHandler);
	signal(SIGTERM, intHandler);
	signal(SIGINT, intHandler);

  startupTimestamp = duration_cast<microseconds>(system_clock::now().time_since_epoch()).count(); 
  
  printf("Startup timestamp: %s \n", std::to_string(startupTimestamp).c_str());

  thread wheelVoltageThread(wheelVoltageTask);
  thread wheelMonitorThread(wheelMonitorTask);

  wheelVoltageThread.join();
  wheelMonitorThread.join();


	return 0;
}
