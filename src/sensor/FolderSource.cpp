#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <mvio/sensor/source.h>
#include <sstream>
#include <thread>
#include <vector>

namespace fs = std::filesystem;

FolderSource::FolderSource(const std::atomic<bool> &stop_signal,
		std::string imgPath, std::string imuP,
		bool sim_realtime)
	: DataSource(stop_signal), imagePath(imgPath), imuPath(imuP),
	currentImageIndex(0), currentImuIndex(0), prevTimestamp(-1.0),
	simulate_realtime(sim_realtime) {
		// Load Images
		if (fs::exists(imagePath) && fs::is_directory(imagePath)) {
			for (const auto &entry : fs::directory_iterator(imagePath)) {
				if (entry.is_regular_file()) {
					std::string ext = entry.path().extension().string();
					std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
					if (ext == ".png" || ext == ".jpg" || ext == ".jpeg") {
						imageFiles.push_back(entry.path().string());
					}
				}
			}
		}

		// Sort images based on timestamp in filename
		std::sort(imageFiles.begin(), imageFiles.end(),
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

		std::cout << "[FolderSource] Loaded " << imageFiles.size() << " images from "
			<< imagePath << std::endl;

		// Load IMU Data
		if (fs::exists(imuPath)) {
			std::ifstream file(imuPath);
			std::string line;

			while (std::getline(file, line)) {
				if (line.empty())
					continue;
				if (line[0] == '#')
					continue; // Skip comments
						  // specific check for header: if first char is not digit, skip
				if (!isdigit(line[0]) && line[0] != '-')
					continue;

				std::stringstream ss(line);
				std::string val;
				std::vector<double> row;
				while (std::getline(ss, val, ',')) {
					try {
						row.push_back(std::stod(val));
					} catch (...) {
						break; // stop parsing line on error
					}
				}

				if (row.size() >= 7) {
					double t = row[0];

					// ImuData constructor: t, ax, ay, az, gx, gy, gz
					imuData.emplace_back(t, row[4], row[5], row[6], row[1], row[2], row[3]);
				}
			}

			// Ensure IMU data is sorted
			std::sort(imuData.begin(), imuData.end(),
					[](const ImuData &a, const ImuData &b) {
					return a.timestamp < b.timestamp;
					});
		}
		std::cout << "[FolderSource] Loaded " << imuData.size()
			<< " IMU entries from " << imuPath << std::endl;
	}

std::unique_ptr<Sensor> FolderSource::recv() {
	if (stop_signal.load()) {
		return nullptr;
	}

	std::unique_ptr<Sensor> nextSensor = nullptr;

	// Check if we have items in both streams
	bool hasImage = currentImageIndex < imageFiles.size();
	bool hasImu = currentImuIndex < imuData.size();

	if (hasImage && hasImu) {
		// Peek at timestamps
		double imgTime = 0.0;
		try {
			imgTime =
				std::stod(fs::path(imageFiles[currentImageIndex]).stem().string());
		} catch (...) {
		}

		double imuTime = imuData[currentImuIndex].timestamp;

		if (imuTime < imgTime) {
			// Return IMU
			nextSensor = std::make_unique<ImuData>(imuData[currentImuIndex]);
			currentImuIndex++;
		} else {
			// Return Image
			nextSensor =
				std::make_unique<ImageData>(imgTime, imageFiles[currentImageIndex]);
			currentImageIndex++;
		}
	} else if (hasImage) {
		// Only images left
		double imgTime = 0.0;
		try {
			imgTime =
				std::stod(fs::path(imageFiles[currentImageIndex]).stem().string());
		} catch (...) {
		}
		nextSensor =
			std::make_unique<ImageData>(imgTime, imageFiles[currentImageIndex]);
		currentImageIndex++;
	} else if (hasImu) {
		// Only IMU left
		nextSensor = std::make_unique<ImuData>(imuData[currentImuIndex]);
		currentImuIndex++;
	}

	if (nextSensor) {
		if (simulate_realtime && prevTimestamp >= 0.0) {
			double diff = nextSensor->timestamp - prevTimestamp;
			if (diff > 0) {
				// Sleep in small chunks to be interruptible
				while (diff > 0) {
					if (stop_signal.load()) {
						return nullptr;
					}
					double sleepTime = std::min(diff, 0.05); // 50ms chunks
					std::this_thread::sleep_for(std::chrono::duration<double>(sleepTime));
					diff -= sleepTime;
				}
			}
		}
		prevTimestamp = nextSensor->timestamp;
	}

	return nextSensor;
}
