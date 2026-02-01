#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <mvio/vio/imu.h>
#include <mvio/sensor/core.h>

void ImuPreintegrator::reset(){ 
	delta_p.setZero();
	delta_v.setZero();
	delta_R.setIdentity();
	last_timestamp = -1.0;
}

void ImuPreintegrator::propagate(const std::vector<ImuData>& imuBuffer) {
	if (imuBuffer.empty()) return;

	if (last_timestamp < 0) {
		last_timestamp = imuBuffer.front().timestamp; 
	}

	for (const auto& imu : imuBuffer){ 
		double current_time = imu.timestamp; 
		double dt = current_time - last_timestamp;

		if (dt <= 1e-9) { 
			continue; 
		}

		Eigen::Vector3d acc_raw;
		Eigen::Vector3d gyro_raw;

		acc_raw << imu.ax, imu.ay, imu.az;
		gyro_raw << imu.gx, imu.gy, imu.gz;

		Eigen::Vector3d acc_corrected = acc_raw - bias_a;
		Eigen::Vector3d gyro_corrected = gyro_raw - bias_g;

		delta_p = delta_p + delta_v * dt + 0.5 * delta_R * acc_corrected * dt * dt;
		delta_v = delta_v + delta_R * acc_corrected * dt;

		Eigen::Vector3d angle_axis_vec = gyro_corrected * dt;
		double angle = angle_axis_vec.norm();

		Eigen::Matrix3d dR_step;
		if (angle > 1e-8) {
			Eigen::Vector3d axis = angle_axis_vec.normalized();
			dR_step = Eigen::AngleAxisd(angle, axis).toRotationMatrix();
		} else {
			dR_step.setIdentity(); 
		}

		delta_R = delta_R * dR_step;

		last_timestamp = current_time; 
	}
}
