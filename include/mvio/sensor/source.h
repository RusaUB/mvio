#pragma once
#include <atomic>
#include <iostream>
#include <memory>
#include <mvio/sensor/core.h>
#include <string>
#include <vector>

struct DataSource {
	const std::atomic<bool> &stop_signal;
	explicit DataSource(const std::atomic<bool> &stop_signal)
		: stop_signal(stop_signal) {}
	virtual std::unique_ptr<Sensor> recv() = 0;
	virtual ~DataSource() = default;
};

struct FolderSource : public DataSource {
	std::string imagePath;
	std::string imuPath;
	std::vector<std::string> imageFiles;
	std::vector<ImuData> imuData;
	size_t currentImageIndex;
	size_t currentImuIndex;
	double prevTimestamp;
	bool simulate_realtime;

	FolderSource(const std::atomic<bool> &stop_signal, std::string imgPath,
			std::string imuPath, bool simulate_realtime = true);

	std::unique_ptr<Sensor> recv() override;
};
