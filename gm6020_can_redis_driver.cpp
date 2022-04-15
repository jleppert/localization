#include <stdio.h>
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

  int16_t angle[4];
  int16_t velocity[4];
  int16_t torque[4];
  int8_t temperature[4];

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

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

void wheelMonitorTask() {  
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

  struct can_frame frame;
  
  while (true) {
    WheelStatusMessage message;

    set <int> ids;
    while (ids.size() != 4) {
      read(canSocket, &frame, sizeof(struct can_frame));
      int id = frame.can_id - 0x205;

      ids.insert(id);

      message.angle[id] = (frame.data[0] << 8) | frame.data[1];
      message.velocity[id] = ((frame.data[2] << 8) | frame.data[3]) * ((id == 2 || id == 3) ? REVERSE : 1);
      message.torque[id] = (frame.data[4] << 8) | frame.data[5];
      message.temperature[id] = frame.data[6];
    }

    int64_t currentMicro = duration_cast<microseconds>(system_clock::now().time_since_epoch()).count();
    message.timestamp = uint32_t(currentMicro - startupTimestamp);

    std::stringstream packed;
    msgpack::pack(packed, message);

    packed.seekg(0);

    odometryMonitorRedisClient->set(WHEEL_STATUS_KEY, packed.str());

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

void printMsgpackMessage(std::stringstream packed) {
  auto oh = msgpack::unpack(packed.str().data(), packed.str().size());
  std::cout << oh.get() << std::endl;
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
