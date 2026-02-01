#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <mvio/sensor/PhoneSource.h>
#include <sstream>
#include <thread>

namespace mvio {

	namespace fs = std::filesystem;

	PhoneSource::PhoneSource(const std::atomic<bool> &stop_signal,
			const Config &config)
		: DataSource(stop_signal), dataPath_(config.imagePath),
		simulate_realtime_(config.simulateRealtime), currentImageIndex_(0),
		currentImuIndex_(0), prevTimestamp_(-1.0), timeOffset_(0.0) {

			fs::path framesDir(dataPath_);
			if (!fs::is_directory(framesDir) &&
					fs::exists(framesDir.parent_path() / "frames")) {
				framesDir = framesDir.parent_path() / "frames";
			}

			fs::path imuFile = framesDir.parent_path() / "imu.csv";

			// 1. Load Images
			if (fs::exists(framesDir) && fs::is_directory(framesDir)) {
				for (const auto &entry : fs::directory_iterator(framesDir)) {
					if (entry.is_regular_file()) {
						std::string ext = entry.path().extension().string();
						std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
						if (ext == ".png" || ext == ".jpg" || ext == ".jpeg") {
							imageFiles_.push_back(entry.path().string());
						}
					}
				}
			}

			// Sort images by timestamp derived from filename
			std::sort(imageFiles_.begin(), imageFiles_.end(),
					[](const std::string &a, const std::string &b) {
					fs::path pa(a);
					fs::path pb(b);
					try {
					double ta = std::stod(pa.stem().string());
					double tb = std::stod(pb.stem().string());
					return ta < tb;
					} catch (...) {
					return a < b;
					}
					});

			std::cout << "[PhoneSource] Loaded " << imageFiles_.size() << " frames from "
				<< framesDir << std::endl;

			// 2. Load IMU Data
			if (fs::exists(imuFile)) {
				std::ifstream file(imuFile);
				std::string line;

				while (std::getline(file, line)) {
					if (line.empty() || line[0] == '#')
						continue;

					// Heuristic to skip header
					if (!isdigit(line[0]) && line[0] != '-' && line[0] != '.') {
						continue;
					}

					std::stringstream ss(line);
					std::string val;
					std::vector<double> row;
					while (std::getline(ss, val, ',')) {
						try {
							row.push_back(std::stod(val));
						} catch (...) {
							break;
						}
					}

					// Expected format: timestamp, gx, gy, gz, ax, ay, az
					if (row.size() >= 7) {
						// ImuData(t, ax, ay, az, gx, gy, gz)
						imuData_.emplace_back(row[0], row[4], row[5], row[6], row[1], row[2],
								row[3]);
					}
				}

				std::sort(imuData_.begin(), imuData_.end(),
						[](const ImuData &a, const ImuData &b) {
						return a.timestamp < b.timestamp;
						});
			}

			std::cout << "[PhoneSource] Loaded " << imuData_.size()
				<< " IMU entries from " << imuFile << std::endl;

			// 3. Shake Synchronization Logic
			if (config.useShakeSync && !imuData_.empty() && !imageFiles_.empty()) {
				std::cout << "[PhoneSource] Analyzing IMU for shake sync (Threshold: "
					<< config.shakeAccThreshold << " m/s^2)..." << std::endl;

				double startT = imuData_.front().timestamp;
				double shakeT = -1.0;

				for (const auto &imu : imuData_) {
					if (imu.timestamp > startT + config.shakeSearchWindow)
						break;

					double accNorm =
						std::sqrt(imu.ax * imu.ax + imu.ay * imu.ay + imu.az * imu.az);
					if (accNorm > config.shakeAccThreshold) {
						shakeT = imu.timestamp;
						break;
					}
				}

				if (shakeT > 0.0) {
					// Assume first image corresponds to this shake event
					// We want Image[0].timestamp (virtual) = shakeT
					// Current Image[0].timestamp (real) = t_img_0
					// So we add offset = shakeT - t_img_0 to all images

					double img0_time = 0.0;
					try {
						img0_time = std::stod(fs::path(imageFiles_[0]).stem().string());
					} catch (...) {
					}

					if (img0_time > 0.0) {
						timeOffset_ = shakeT - img0_time;
						std::cout << "[PhoneSource] Shake detected at T=" << shakeT
							<< ". Aligning video (Offset: " << timeOffset_ << "s)."
							<< std::endl;

						// Fast-forward to settle time
						double startTime = shakeT + config.shakeSettleTime;

						// Skip IMU
						size_t skippedImu = 0;
						while (currentImuIndex_ < imuData_.size() &&
								imuData_[currentImuIndex_].timestamp < startTime) {
							currentImuIndex_++;
							skippedImu++;
						}

						// Skip Images (considering offset)
						size_t skippedImg = 0;
						while (currentImageIndex_ < imageFiles_.size()) {
							double t = 0.0;
							try {
								t = std::stod(
										fs::path(imageFiles_[currentImageIndex_]).stem().string());
							} catch (...) {
							}
							if (t + timeOffset_ < startTime) {
								currentImageIndex_++;
								skippedImg++;
							} else {
								break;
							}
						}

						std::cout << "[PhoneSource] Skipped " << skippedImu
							<< " IMU samples and " << skippedImg << " frames to settle."
							<< std::endl;
					}
				} else {
					std::cout << "[PhoneSource] No shake detected within search window."
						<< std::endl;
				}
			}
		}

	std::unique_ptr<Sensor> PhoneSource::recv() {
		if (stop_signal.load())
			return nullptr;

		std::unique_ptr<Sensor> nextSensor = nullptr;
		bool hasImage = currentImageIndex_ < imageFiles_.size();
		bool hasImu = currentImuIndex_ < imuData_.size();

		double imgTimeRaw = 0.0;
		if (hasImage) {
			try {
				imgTimeRaw =
					std::stod(fs::path(imageFiles_[currentImageIndex_]).stem().string());
			} catch (...) {
			}
		}

		// Apply offset to comparison
		double imgTimeSynced = imgTimeRaw + timeOffset_;

		if (hasImage && hasImu) {
			if (imuData_[currentImuIndex_].timestamp < imgTimeSynced) {
				nextSensor = std::make_unique<ImuData>(imuData_[currentImuIndex_++]);
			} else {
				nextSensor = std::make_unique<ImageData>(
						imgTimeSynced, imageFiles_[currentImageIndex_++]);
			}
		} else if (hasImage) {
			nextSensor = std::make_unique<ImageData>(imgTimeSynced,
					imageFiles_[currentImageIndex_++]);
		} else if (hasImu) {
			nextSensor = std::make_unique<ImuData>(imuData_[currentImuIndex_++]);
		}

		if (nextSensor) {
			if (simulate_realtime_ && prevTimestamp_ >= 0.0) {
				double diff = nextSensor->timestamp - prevTimestamp_;
				if (diff > 0) {
					while (diff > 0) {
						if (stop_signal.load())
							return nullptr;
						double sleepTime = std::min(diff, 0.05);
						std::this_thread::sleep_for(std::chrono::duration<double>(sleepTime));
						diff -= sleepTime;
					}
				}
			}
			prevTimestamp_ = nextSensor->timestamp;
		}

		return nextSensor;
	}

} // namespace mvio
