#include <mvio/pipeline/Pipeline.h>

#include <iostream>
#include <opencv2/opencv.hpp>

#include <mvio/sensor/PhoneSource.h>
#include <mvio/sensor/RpiSource.h>
#include <mvio/sensor/source.h>

namespace mvio {

	Pipeline::Pipeline(const Config &config, const std::atomic<bool> &stop_signal)
		: config_(config), lastImuMeasurement_(0.0, 0, 0, 0, 0, 0, 0) {

			std::cout << "Pipeline: Initializing. DataSource = " << config_.dataSourceType
				<< std::endl;

			if (config_.dataSourceType == "rpi") {
				std::cout << "Pipeline: Creating RpiSource..." << std::endl;
				sensorSource_ = std::make_unique<RpiSource>(stop_signal);
				std::cout << "Pipeline: RpiSource created." << std::endl;
			} else if (config_.dataSourceType == "phone") {
				std::cout << "Pipeline: Creating PhoneSource..." << std::endl;
				sensorSource_ = std::make_unique<PhoneSource>(stop_signal, config_);
			} else {
				// Default to folder
				std::cout << "Pipeline: Creating FolderSource..." << std::endl;
				sensorSource_ = std::make_unique<FolderSource>(
						stop_signal, config_.imagePath, config_.imuPath,
						config_.simulateRealtime);
			}

			vo_ = std::make_unique<VisualOdometry3Dto2D>(config_);

			vo_->imu.bias_a << config_.imuInitBiasAx, config_.imuInitBiasAy,
				config_.imuInitBiasAz;
			vo_->imu.bias_g << config_.imuInitBiasGx, config_.imuInitBiasGy,
				config_.imuInitBiasGz;
		}

	Pipeline::~Pipeline() = default;

	void Pipeline::run() {
		while (true) {
			std::unique_ptr<Sensor> s = sensorSource_->recv();
			if (!s) {
				break;
			}

			if (s->type == SensorType::IMG) {
				processImage(static_cast<ImageData *>(s.get()));
			} else if (s->type == SensorType::IMU) {
				bufferImu(static_cast<ImuData *>(s.get()));
			}
		}

		vo_->saveTrajectory("results");
	}

	void Pipeline::processImage(const ImageData *img) {
		if (!img)
			return;

		std::cout << "IMG: " << img->timestamp << " " << img->filePath << std::endl;
		double currTime = img->timestamp;

		std::vector<ImuData> currentFrameImu;
		currentFrameImu.push_back(lastImuMeasurement_);

		auto it = imuDataQueue_.begin();
		while (it != imuDataQueue_.end()) {
			if (it->timestamp <= lastFrameTime_) {
				it = imuDataQueue_.erase(it);
				continue;
			}

			if (it->timestamp <= currTime) {
				currentFrameImu.push_back(*it);
				lastImuMeasurement_ = *it;
				++it;
			} else {
				currentFrameImu.push_back(*it);
				break;
			}
		}

		std::cout << "Processed " << currentFrameImu.size() << " IMU measurements."
			<< std::endl;

		if (!currentFrameImu.empty()) {
			double avg_ax = 0, avg_ay = 0, avg_az = 0;
			double avg_gx = 0, avg_gy = 0, avg_gz = 0;
			for (const auto &imu : currentFrameImu) {
				avg_ax += imu.ax;
				avg_ay += imu.ay;
				avg_az += imu.az;
				avg_gx += imu.gx;
				avg_gy += imu.gy;
				avg_gz += imu.gz;
			}
			size_t N = currentFrameImu.size();
			avg_ax /= N;
			avg_ay /= N;
			avg_az /= N;
			avg_gx /= N;
			avg_gy /= N;
			avg_gz /= N;

			std::cout << "Avg Acc: [" << avg_ax << ", " << avg_ay << ", " << avg_az
				<< "]" << std::endl;
			std::cout << "Avg Gyro: [" << avg_gx << ", " << avg_gy << ", " << avg_gz
				<< "]" << std::endl;
		}

		cv::Mat frame;
		if (img->image) {
			frame = *img->image;
			if (frame.channels() == 3) {
				cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);
			}
		} else {
			frame = cv::imread(img->filePath, 0);
		}

		if (frame.empty()) {
			std::cerr << "Failed to load image" << std::endl;
			return;
		}

		vo_->update(frame, currentFrameImu, currTime);
		lastFrameTime_ = currTime;

		if (config_.showStream) {
			cv::Mat displayFrame;
			if (frame.channels() == 1) {
				cv::cvtColor(frame, displayFrame, cv::COLOR_GRAY2BGR);
			} else {
				displayFrame = frame.clone();
			}

			if (config_.showFeatures) {
				for (const auto &f : vo_->features) {
					cv::circle(displayFrame, f.point, 3, cv::Scalar(0, 255, 0), 1);
					cv::putText(displayFrame, std::to_string(f.id), f.point,
							cv::FONT_HERSHEY_PLAIN, 0.8, cv::Scalar(0, 0, 255), 1);
				}
			}

			cv::imshow("VIO Stream", displayFrame);
			cv::waitKey(1);
		}
	}

	void Pipeline::bufferImu(const ImuData *imu) {
		if (imu) {
			imuDataQueue_.push_back(*imu);
			if (config_.saveImu) {
				vo_->saveImuMeasurement(*imu);
			}
		}
	}

} // namespace mvio
