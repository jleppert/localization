#include <stdio.h>
#include <string.h>
#include "iostream"
#include "threespace_api_export.h"

#include "utils/LoopTimer.h"

#include <msgpack.hpp>

#include <survive.h>

#include <sw/redis++/redis++.h>

#include <chrono>

#include <signal.h>
bool runloop = true;
void sighandler(int){runloop = false;}

using namespace std;
using namespace std::chrono;
using namespace sw::redis;

Redis* redis;

struct IMUMessage {
  IMUMessage(FLT _timestamp, TSS_Stream_Packet packet) {
    timestamp = _timestamp;

    ahrs[0] = packet.taredOrientEuler[0];
    ahrs[1] = packet.taredOrientEuler[1];
    ahrs[2] = packet.taredOrientEuler[2];
  }

  FLT timestamp = 0.0;

  std::array<FLT, 3> ahrs = {0.0, 0.0, 0.0};


  MSGPACK_DEFINE_MAP(timestamp, ahrs)
};

const string ROVER_IMU = "rover_imu";

int main(int argc, char **argv)
{
	TSS_ComPort port;
	port.port_name = "/dev/ttyS0";
	tss_device_id device_id;
	TSS_ERROR error = TSS_NO_ERROR;

	printf("====Creating a Three Space Device from Search====\n");
	//tss_findSensorPorts(TSS_FIND_ALL_KNOWN ^ TSS_DONGLE);

	/*error = tss_getNextSensorPort("/dev/ttyAMA0", &port.device_type, &port.connection_type);
	if(error != TSS_NO_ERROR)
	{
		printf("Failed to get the port!\n");
		tss_deinitAPI();
		return 0;
	}*/

	error = tss_createSensor(port.port_name, &device_id);
	if(error)
	{
		printf("Failed to create TSS Sensor on %s!\n", port.port_name);
		tss_deinitAPI();
		return 0;
	}
	printf("====Found Device====\n");

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// redis client
  redis = new Redis("tcp://127.0.0.1:6379");

	// set mode to IMU for fast readings
	U32 timestamp = 0;
	error = tss_sensor_setFilterMode(device_id, 1, &timestamp);
	// error = tss_sensor_setUARTBaudRate(device_id, 921600, &timestamp);

	// setup streaming
	//int data_to_stream = TSS_STREAM_CORRECTED_GYROSCOPE_DATA + TSS_STREAM_CORRECTED_ACCELEROMETER_DATA + TSS_STREAM_LINEAR_ACCELERATION;
	int data_to_stream = TSS_STREAM_TARED_ORIENTATION_AS_EULER_ANGLES;
	error = tss_sensor_startStreamingWired(device_id, data_to_stream, 1000, TSS_STREAM_DURATION_INFINITE, 0);
	printf("====Starting Streaming====\n");

	TSS_Stream_Packet packet;

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(100); 
	double current_time = 0;
	double prev_time = 0;
	double dt = 0;
	bool fTimerDidSleep = true;
	double start_time = timer.elapsedTime(); //secs
	
  int64_t startupTimestamp = duration_cast<microseconds>(system_clock::now().time_since_epoch()).count();

  while(runloop)
	{
		timer.waitForNextLoop();

		error = tss_sensor_getLastStreamingPacket(device_id, &packet);
		
    int64_t currentMicro = duration_cast<microseconds>(system_clock::now().time_since_epoch()).count();
    uint32_t timestamp = uint32_t(currentMicro - startupTimestamp);

    IMUMessage message = {timestamp, packet};

    std::stringstream packed;
    msgpack::pack(packed, message);

    packed.seekg(0);

    redis->set(ROVER_IMU, packed.str());
	}

	double end_time = timer.elapsedTime();
	std::cout << "\n";
	std::cout << "3Space Sensor Loop run time  : " << end_time << " seconds\n";
	std::cout << "3Space Sensor Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "3Space Sensor Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";

	tss_sensor_stopStreamingWired(device_id);
	tss_removeSensor(device_id);
	tss_deinitAPI();

	return 1;
}
