#include <iostream>
#include <mvio/estimators/kalman_filter.h>

KalmanFilter::KalmanFilter() {
	p.setZero();
	v.setZero();
	q.setIdentity();
	ba.setZero();
	bg.setZero();
	P.setIdentity();

	noise_acc = 0.01;
	noise_gyro = 0.01;
	noise_acc_bias = 0.0001;
	noise_gyro_bias = 0.0001;

	g << 0, 0, -9.81;
}

void KalmanFilter::init(const Eigen::Vector3d &p0, const Eigen::Vector3d &v0,
		const Eigen::Quaterniond &q0,
		const Eigen::Vector3d &ba0, const Eigen::Vector3d &bg0,
		double acc_noise, double gyro_noise,
		double acc_bias_noise, double gyro_bias_noise) {
	p = p0;
	v = v0;
	q = q0;
	ba = ba0;
	bg = bg0;

	noise_acc = acc_noise;
	noise_gyro = gyro_noise;
	noise_acc_bias = acc_bias_noise;
	noise_gyro_bias = gyro_bias_noise;

	P.setZero();
	P.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * 1e-4;
	P.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * 1e-2;
	P.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * 1e-4;
	P.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * 1e-2;
	P.block<3, 3>(12, 12) = Eigen::Matrix3d::Identity() * 1e-3;
}

Eigen::Matrix3d KalmanFilter::skew(const Eigen::Vector3d &v) {
	Eigen::Matrix3d m;
	m << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
	return m;
}

void KalmanFilter::predict(const std::vector<ImuData> &imu_data) {
	if (imu_data.empty())
		return;

	double last_time = -1.0;

	for (const auto &imu : imu_data) {
		if (last_time < 0) {
			last_time = imu.timestamp;
			continue;
		}

		double dt = imu.timestamp - last_time;
		if (dt <= 1e-6)
			continue;
		last_time = imu.timestamp;

		Eigen::Vector3d am(imu.ax, imu.ay, imu.az);
		Eigen::Vector3d wm(imu.gx, imu.gy, imu.gz);

		Eigen::Vector3d a = am - ba;
		Eigen::Vector3d w = wm - bg;

		Eigen::Matrix3d R = q.toRotationMatrix();

		Eigen::Vector3d p_new = p + v * dt + 0.5 * (R * a + g) * dt * dt;
		Eigen::Vector3d v_new = v + (R * a + g) * dt;

		Eigen::Vector3d angle_axis = w * dt;
		Eigen::Quaterniond dq;
		if (angle_axis.norm() > 1e-8) {
			dq = Eigen::Quaterniond(
					Eigen::AngleAxisd(angle_axis.norm(), angle_axis.normalized()));
		} else {
			dq = Eigen::Quaterniond(1, 0, 0, 0);
		}
		Eigen::Quaterniond q_new = (q * dq).normalized();

		Eigen::Matrix<double, 15, 15> Fx =
			Eigen::Matrix<double, 15, 15>::Identity();

		Fx.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;

		Fx.block<3, 3>(3, 6) = -R * skew(a) * dt;

		Fx.block<3, 3>(3, 9) = -R * dt;

		Fx.block<3, 3>(6, 6) = dq.toRotationMatrix().transpose();

		Fx.block<3, 3>(6, 12) = -Eigen::Matrix3d::Identity() * dt;

		Eigen::Matrix<double, 15, 15> Qi = Eigen::Matrix<double, 15, 15>::Zero();
		Qi.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * noise_acc * dt * dt;
		Qi.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * noise_gyro * dt * dt;
		Qi.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * noise_acc_bias * dt;
		Qi.block<3, 3>(12, 12) = Eigen::Matrix3d::Identity() * noise_gyro_bias * dt;

		P = Fx * P * Fx.transpose() + Qi;

		p = p_new;
		v = v_new;
		q = q_new;
	}
}

void KalmanFilter::update_pose(const Eigen::Vector3d &p_meas,
		const Eigen::Quaterniond &q_meas,
		const Eigen::Matrix<double, 6, 6> &R_meas) {
	Eigen::Vector3d r_p = p_meas - p;

	Eigen::Quaterniond dq = q_meas * q.inverse();
	Eigen::Vector3d r_theta = 2.0 * dq.vec();
	if (dq.w() < 0)
		r_theta = -r_theta;

	Eigen::Matrix<double, 6, 1> r;
	r << r_p, r_theta;

	Eigen::Matrix<double, 6, 15> H = Eigen::Matrix<double, 6, 15>::Zero();
	H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
	H.block<3, 3>(3, 6) = Eigen::Matrix3d::Identity();

	Eigen::Matrix<double, 6, 6> S = H * P * H.transpose() + R_meas;
	Eigen::Matrix<double, 15, 6> K = P * H.transpose() * S.inverse();

	Eigen::Matrix<double, 15, 1> dx = K * r;

	p += dx.segment<3>(0);
	v += dx.segment<3>(3);

	Eigen::Vector3d dtheta = dx.segment<3>(6);
	Eigen::Quaterniond dq_corr;
	if (dtheta.norm() > 1e-8) {
		dq_corr = Eigen::Quaterniond(
				Eigen::AngleAxisd(dtheta.norm(), dtheta.normalized()));
	} else {
		dq_corr = Eigen::Quaterniond(1, 0, 0, 0);
	}
	q = (dq_corr * q).normalized();

	ba += dx.segment<3>(9);
	bg += dx.segment<3>(12);

	Eigen::Matrix<double, 15, 15> I = Eigen::Matrix<double, 15, 15>::Identity();
	P = (I - K * H) * P;

	P = 0.5 * (P + P.transpose().eval());
}
