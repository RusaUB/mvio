#pragma once
#include <iostream>
#include <opencv2/opencv.hpp>
#include <optional>
#include <string>

enum class SensorType {
	IMU,
	IMG
};

struct Sensor {
	double timestamp;
	SensorType type;

	Sensor(double t, SensorType ty) : timestamp(t), type(ty) {};
	virtual ~Sensor() = default;
};

struct IMU {
	double ax, ay, az;
	double gx, gy, gz;
	IMU(double _ax, double _ay, double _az, double _gx, double _gy, double _gz) 
		: ax(_ax), ay(_ay), az(_az), gx(_gx), gy(_gy), gz(_gz) {}

};

struct ImuData : public Sensor, public IMU {
	ImuData(double t, double _ax, double _ay, double _az, double _gx, double _gy, double _gz) 
		: Sensor(t, SensorType::IMU), IMU(_ax,_ay,_az,_gx,_gy,_gz) {}
};

struct ImageData : public Sensor {
	std::string filePath;
	std::optional<cv::Mat> image; // For live sources

	ImageData(double t, std::string path) : Sensor(t, SensorType::IMG), filePath(path) {};
	ImageData(double t, cv::Mat img) : Sensor(t, SensorType::IMG), image(img) {};
};
