#pragma once
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <mvio/vio/imu.h>
#include <vector>

#include <mvio/estimators/estimator.h>

class KalmanFilter : public IEstimator {
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

			Eigen::Vector3d p;
		Eigen::Vector3d v;
		Eigen::Quaterniond q;
		Eigen::Vector3d ba;
		Eigen::Vector3d bg;

		Eigen::Matrix<double, 15, 15> P;

		double noise_acc;
		double noise_gyro;
		double noise_acc_bias;
		double noise_gyro_bias;

		Eigen::Vector3d g;

		KalmanFilter();

		void init(const Eigen::Vector3d &p0, const Eigen::Vector3d &v0,
				const Eigen::Quaterniond &q0, const Eigen::Vector3d &ba0,
				const Eigen::Vector3d &bg0, double acc_noise, double gyro_noise,
				double acc_bias_noise, double gyro_bias_noise) override;

		void predict(const std::vector<ImuData> &imu_data) override;

		void update_pose(const Eigen::Vector3d &p_meas,
				const Eigen::Quaterniond &q_meas,
				const Eigen::Matrix<double, 6, 6> &R_meas) override;

		Eigen::Vector3d getPosition() const override { return p; }
		Eigen::Quaterniond getRotation() const override { return q; }
		Eigen::Vector3d getVelocity() const override { return v; }
		Eigen::Vector3d getAccelBias() const override { return ba; }
		Eigen::Vector3d getGyroBias() const override { return bg; }

		void setGravity(const Eigen::Vector3d &g_new) override { g = g_new; }

		static Eigen::Matrix3d skew(const Eigen::Vector3d &v);
};
