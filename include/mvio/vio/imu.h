#pragma once
#include <iostream>
#include <vector>
#include <Eigen/Dense>

#include <mvio/sensor/core.h>

class ImuPreintegrator {
	public:
		Eigen::Vector3d delta_p;
		Eigen::Vector3d delta_v;
		Eigen::Matrix3d delta_R; 

		Eigen::Vector3d bias_a;
		Eigen::Vector3d bias_g;

		double last_timestamp; 

		ImuPreintegrator() = default;
		ImuPreintegrator(const Eigen::Vector3d& init_ba, const Eigen::Vector3d& init_bg) 
			: bias_a(init_ba), bias_g(init_bg) {
				reset();
			}

		void reset();

		void propagate(const std::vector<ImuData>& imuBuffer);
};
