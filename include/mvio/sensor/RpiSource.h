#pragma once
#include <atomic>
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <mvio/sensor/core.h>
#include <mvio/sensor/source.h>
#include <opencv2/opencv.hpp>
#include <optional>
#include <thread>

namespace mvio {

	class RpiSource : public DataSource {
		public:
			explicit RpiSource(const std::atomic<bool> &stop_signal);
			~RpiSource() override;

			std::unique_ptr<Sensor> recv() override;

		private:
			void imuLoop();
			void cameraLoop();

			// Hardware Handles
			int i2c_fd_;
			cv::VideoCapture cap_;

			// Threading
			std::atomic<bool> running_;
			std::thread imuThread_;
			std::thread cameraThread_;

			// Data Queue
			std::deque<std::unique_ptr<Sensor>> queue_;
			std::mutex queueMutex_;
			std::condition_variable queueCv_;

			// Time synchronization
			double startTime_;
	};

} // namespace mvio
