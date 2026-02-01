#pragma once
#include <atomic>
#include <filesystem>
#include <mvio/config.h>
#include <mvio/sensor/core.h>
#include <mvio/sensor/source.h>
#include <string>
#include <vector>

namespace mvio {

	/**
	 * @brief DataSource implementation for processed iPhone data (Matlab Mobile)
	 * Expects a directory containing:
	 * - imu.csv (timestamp, gx, gy, gz, ax, ay, az)
	 * - frames/ (images named with timestamp, e.g. 123.456.png)
	 */
	class PhoneSource : public DataSource {
		public:
			PhoneSource(const std::atomic<bool> &stop_signal, const Config &config);
			~PhoneSource() override = default;

			std::unique_ptr<Sensor> recv() override;

		private:
			std::string dataPath_;
			bool simulate_realtime_;

			// Data Containers
			std::vector<std::string> imageFiles_;
			std::vector<ImuData> imuData_;

			// State
			size_t currentImageIndex_;
			size_t currentImuIndex_;
			double prevTimestamp_;
			double timeOffset_ = 0.0;
	};

} // namespace mvio
