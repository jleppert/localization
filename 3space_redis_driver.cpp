/********************************************//**
* This example demonstrates streaming basic data from the sensor
* This is compatible with all sensors plugged in via USB or Bluetooth
* Will not work with the dongle or wireless sensor wirelessly see
* streaming_information_wireless for a wireless example.
***********************************************/
#include <stdio.h>
#include <string.h>
#include "iostream"
#include "threespace_api_export.h"

#include "utils/RedisClient.h"
#include "utils/LoopTimer.h"

#include <chrono>

#include <signal.h>
bool runloop = true;
void sighandler(int){runloop = false;}

using namespace std;

const string ACCELEROMETER_DATA_KEY =      "sai2::3spaceSensor::data::accelerometer";        // in local frame
const string GYROSCOPE_DATA_KEY =          "sai2::3spaceSensor::data::gyroscope";            // in local frame
const string LINEAR_ACCELERATION_KEY =     "sai2::3spaceSensor::data::linear_acceleration";  // acceleration gravity compensated in global frame

int main()
{
	TSS_ComPort port;
	port.port_name = new char[64];
	tss_device_id device_id;
	TSS_ERROR error = TSS_NO_ERROR;

	printf("====Creating a Three Space Device from Search====\n");
	tss_findSensorPorts(TSS_FIND_ALL_KNOWN ^ TSS_DONGLE);

	error = tss_getNextSensorPort(port.port_name, &port.device_type, &port.connection_type);
	if(error != TSS_NO_ERROR)
	{
		printf("Failed to get the port!\n");
		tss_deinitAPI();
		return 0;
	}

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
	RedisClient* redis_client = new RedisClient();
	redis_client->connect();

	// set mode to IMU for fast readings
	U32 timestamp = 0;
	error = tss_sensor_setFilterMode(device_id, 0, &timestamp);
	// error = tss_sensor_setUARTBaudRate(device_id, 921600, &timestamp);

	// setup streaming
	int data_to_stream = TSS_STREAM_CORRECTED_GYROSCOPE_DATA + TSS_STREAM_CORRECTED_ACCELEROMETER_DATA + TSS_STREAM_LINEAR_ACCELERATION;
	error = tss_sensor_startStreamingWired(device_id, data_to_stream, 1000, TSS_STREAM_DURATION_INFINITE, 0);
	printf("====Starting Streaming====\n");

	TSS_Stream_Packet packet;

	Eigen::Vector3d accelerometer_data = Eigen::Vector3d::Zero();
	Eigen::Vector3d gyroscope_data = Eigen::Vector3d::Zero();
	Eigen::Vector3d linear_acceleration = Eigen::Vector3d::Zero();

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000); 
	double current_time = 0;
	double prev_time = 0;
	double dt = 0;
	bool fTimerDidSleep = true;
	double start_time = timer.elapsedTime(); //secs
	while(runloop)
	{
		timer.waitForNextLoop();

		error = tss_sensor_getLastStreamingPacket(device_id, &packet);

		for(int i = 0 ; i < 3 ; i++)
		{
			accelerometer_data(i) = packet.correctedAccelerometerData[i];
			gyroscope_data(i) = packet.correctedGyroscopeData[i];
			linear_acceleration(i) = packet.linearAcceleration[i];
		}

		redis_client->setEigenMatrixJSON(ACCELEROMETER_DATA_KEY, accelerometer_data);
		redis_client->setEigenMatrixJSON(GYROSCOPE_DATA_KEY, gyroscope_data);
		redis_client->setEigenMatrixJSON(LINEAR_ACCELERATION_KEY, linear_acceleration);
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