#pragma once
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <mvio/vio/imu.h>
#include <vector>

/**
 * @brief Interface for generic state estimators (e.g., Kalman Filter, Factor
 * Graph).
 */
class IEstimator {
	public:
		virtual ~IEstimator() = default;

		/**
		 * @brief Initialize the estimator with the starting state.
		 *
		 * @param p0 Initial position
		 * @param v0 Initial velocity
		 * @param q0 Initial orientation (Quaternion)
		 * @param ba0 Initial accelerometer bias
		 * @param bg0 Initial gyroscope bias
		 * @param acc_noise Accelerometer noise density
		 * @param gyro_noise Gyroscope noise density
		 * @param acc_bias_noise Accelerometer bias random walk
		 * @param gyro_bias_noise Gyroscope bias random walk
		 */
		virtual void init(const Eigen::Vector3d &p0, const Eigen::Vector3d &v0,
				const Eigen::Quaterniond &q0, const Eigen::Vector3d &ba0,
				const Eigen::Vector3d &bg0, double acc_noise,
				double gyro_noise, double acc_bias_noise,
				double gyro_bias_noise) = 0;

		/**
		 * @brief Predict the next state using IMU measurements.
		 *
		 * @param imu_data Vector of IMU measurements for propagation.
		 */
		virtual void predict(const std::vector<ImuData> &imu_data) = 0;

		/**
		 * @brief Update the state with a pose measurement (e.g., from visual
		 * odometry).
		 *
		 * @param p_meas Measured position
		 * @param q_meas Measured orientation
		 * @param R_meas Measurement covariance matrix (6x6)
		 */
		virtual void update_pose(const Eigen::Vector3d &p_meas,
				const Eigen::Quaterniond &q_meas,
				const Eigen::Matrix<double, 6, 6> &R_meas) = 0;

		// Accessors
		virtual Eigen::Vector3d getPosition() const = 0;
		virtual Eigen::Quaterniond getRotation() const = 0;
		virtual Eigen::Vector3d getVelocity() const = 0;
		virtual Eigen::Vector3d getAccelBias() const = 0;
		virtual Eigen::Vector3d getGyroBias() const = 0;

		/**
		 * @brief Set the gravity vector used by the estimator.
		 *
		 * @param g_new Gravity vector (usually in world frame).
		 */
		virtual void setGravity(const Eigen::Vector3d &g_new) = 0;
};
