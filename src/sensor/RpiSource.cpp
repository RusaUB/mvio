#include <chrono>
#include <cmath>
#include <iostream>
#include <mvio/sensor/RpiSource.h>
#include <thread>

#ifdef __linux__
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>
#endif

// MPU6050 Registers
#define MPU6050_ADDR 0x68
#define PWR_MGMT_1 0x6B
#define ACCEL_CONFIG 0x1C
#define GYRO_CONFIG 0x1B
#define ACCEL_XOUT_H 0x3B

namespace mvio {

	RpiSource::RpiSource(const std::atomic<bool> &stop_signal)
		: DataSource(stop_signal), running_(true), i2c_fd_(-1) {
			std::cout << "[RpiSource] Constructor called." << std::endl;

			// Capture start time to normalize timestamps to start from 0
			auto now = std::chrono::steady_clock::now();
			startTime_ = std::chrono::duration<double>(now.time_since_epoch()).count();

#ifdef __linux__
			// Initialize MPU6050
			const char *i2c_device = "/dev/i2c-1";
			i2c_fd_ = open(i2c_device, O_RDWR);
			if (i2c_fd_ < 0) {
				std::cerr << "Failed to open I2C bus: " << i2c_device << std::endl;
			} else {
				if (ioctl(i2c_fd_, I2C_SLAVE, MPU6050_ADDR) < 0) {
					std::cerr << "Failed to acquire bus access and/or talk to slave."
						<< std::endl;
				} else {
					// Wake up MPU6050
					uint8_t buf[2];
					buf[0] = PWR_MGMT_1;
					buf[1] = 0x00;
					write(i2c_fd_, buf, 2);

					// Configure Accel (e.g., +/- 16g) - 0x18
					buf[0] = ACCEL_CONFIG;
					buf[1] = 0x18;
					write(i2c_fd_, buf, 2);

					// Configure Gyro (e.g., +/- 2000 dps) - 0x18
					buf[0] = GYRO_CONFIG;
					buf[1] = 0x18;
					write(i2c_fd_, buf, 2);
				}
			}
#else
			std::cerr
				<< "RpiSource: Not running on Linux. I2C device /dev/i2c-1 unavailable."
				<< std::endl;
#endif

#ifdef __linux__
			std::string pipeline = "libcamerasrc ! video/x-raw, width=848, height=480, "
				"framerate=30/1 ! videoconvert ! appsink";
			std::cout << "[RpiSource] Attempting to open camera with GStreamer pipeline: "
				<< pipeline << std::endl;
			cap_.open(pipeline, cv::CAP_GSTREAMER);

			if (!cap_.isOpened()) {
				std::cerr << "[RpiSource] Failed to open with GStreamer/libcamerasrc. "
					"Trying legacy V4L2 /dev/video0..."
					<< std::endl;
				cap_.open(0, cv::CAP_V4L2);
			}
#else
			cap_.open(0, cv::CAP_ANY);
#endif

			if (!cap_.isOpened()) {
				std::cerr << "[RpiSource] Failed to open camera (index 0 or pipeline)."
					<< std::endl;
			} else {
				std::cout << "[RpiSource] Camera initialized successfully." << std::endl;
				if (cap_.getBackendName() != "GSTREAMER") {
					cap_.set(cv::CAP_PROP_FPS, 30);
					cap_.set(cv::CAP_PROP_FRAME_WIDTH, 848);
					cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
				}
			}

			imuThread_ = std::thread(&RpiSource::imuLoop, this);
			cameraThread_ = std::thread(&RpiSource::cameraLoop, this);
		}

	RpiSource::~RpiSource() {
		running_ = false;
		queueCv_.notify_all();

		if (imuThread_.joinable())
			imuThread_.join();
		if (cameraThread_.joinable())
			cameraThread_.join();

#ifdef __linux__
		if (i2c_fd_ >= 0)
			close(i2c_fd_);
#endif
	}

	std::unique_ptr<Sensor> RpiSource::recv() {
		std::unique_lock<std::mutex> lock(queueMutex_);

		// Check stop signal immediately
		if (stop_signal.load()) {
			running_ = false;
			return nullptr;
		}

		// Wait until queue is not empty OR not running OR stop signal
		// using wait_for to periodically check stop_signal
		while (queue_.empty() && running_ && !stop_signal.load()) {
			queueCv_.wait_for(lock, std::chrono::milliseconds(100));

			// Check stop signal after wake up
			if (stop_signal.load()) {
				running_ = false;
				return nullptr;
			}
		}

		if ((!running_) && queue_.empty()) {
			return nullptr;
		}

		if (queue_.empty()) {
			return nullptr;
		}

		auto sensor = std::move(queue_.front());
		queue_.pop_front();
		return sensor;
	}

	void RpiSource::cameraLoop() {
		while (running_) {

			if (!cap_.isOpened()) {
				std::this_thread::sleep_for(std::chrono::milliseconds(100));
				continue;
			}

			cv::Mat frame;
			if (cap_.read(frame)) {

				auto now = std::chrono::steady_clock::now();
				double ts =
					std::chrono::duration<double>(now.time_since_epoch()).count() -
					startTime_;

				static int frameCount = 0;
				if (++frameCount % 30 == 0) {
					std::cout << "[RpiSource] Captured frame " << frameCount
						<< " at ts=" << ts << std::endl;
				}

				auto imgData = std::make_unique<ImageData>(ts, frame);

				{
					std::lock_guard<std::mutex> lock(queueMutex_);
					queue_.push_back(std::move(imgData));
				}
				queueCv_.notify_one();
			} else {
				std::this_thread::sleep_for(std::chrono::milliseconds(10));
			}
		}
	}

	void RpiSource::imuLoop() {
#ifdef __linux__
		// 14 bytes: Accel (6), Temp (2), Gyro (6)
		uint8_t buf[14];
		uint8_t reg = ACCEL_XOUT_H;

		// Scale factors for settings (+/- 16g, +/- 2000 dps)
		// 16g -> 2048 LSB/g. 2000dps -> 16.4 LSB/dps
		const double ACCEL_SCALE = 2048.0;
		const double GYRO_SCALE = 16.4;
		const double G_TO_MS2 = 9.81;

		auto delay = std::chrono::milliseconds(5);

		while (running_) {
			if (i2c_fd_ < 0) {
				std::this_thread::sleep_for(std::chrono::milliseconds(100));
				continue;
			}

			// Set register pointer
			if (write(i2c_fd_, &reg, 1) != 1) {
				// Error or retry
				std::this_thread::sleep_for(delay);
				continue;
			}

			if (read(i2c_fd_, buf, 14) != 14) {
				std::this_thread::sleep_for(delay);
				continue;
			}

			auto now = std::chrono::steady_clock::now();
			double ts = std::chrono::duration<double>(now.time_since_epoch()).count() -
				startTime_;

			// Parse Big Endian
			int16_t ax_raw = (buf[0] << 8) | buf[1];
			int16_t ay_raw = (buf[2] << 8) | buf[3];
			int16_t az_raw = (buf[4] << 8) | buf[5];
			// Temp at buf[6], buf[7] - ignored
			int16_t gx_raw = (buf[8] << 8) | buf[9];
			int16_t gy_raw = (buf[10] << 8) | buf[11];
			int16_t gz_raw = (buf[12] << 8) | buf[13];

			double ax = (double)ax_raw / ACCEL_SCALE * G_TO_MS2;
			double ay = (double)ay_raw / ACCEL_SCALE * G_TO_MS2;
			double az = (double)az_raw / ACCEL_SCALE * G_TO_MS2;

			// Radians/sec
			double gx = ((double)gx_raw / GYRO_SCALE) * (M_PI / 180.0);
			double gy = ((double)gy_raw / GYRO_SCALE) * (M_PI / 180.0);
			double gz = ((double)gz_raw / GYRO_SCALE) * (M_PI / 180.0);

			auto imuData = std::make_unique<ImuData>(ts, ax, ay, az, gx, gy, gz);

			{
				std::lock_guard<std::mutex> lock(queueMutex_);
				queue_.push_back(std::move(imuData));
			}
			queueCv_.notify_one();

			std::this_thread::sleep_for(delay);
		}
#else
		while (running_) {
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}
#endif
	}

} // namespace mvio
