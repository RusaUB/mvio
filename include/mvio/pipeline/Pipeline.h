#pragma once

#include <deque>
#include <memory>
#include <string>

#include <mvio/config.h>
#include <mvio/sensor/core.h>
#include <mvio/vio/vo.h>

struct DataSource;

namespace mvio {

	/// @brief Main processing pipeline for the MVIO system.
	///
	/// Encapsulates sensor data fetching, synchronization, and the VIO update loop.
	class Pipeline {
		public:
			/// @brief Construct a new Pipeline object.
			/// @param config Configuration object containing paths and system parameters.
			Pipeline(const Config &config, const std::atomic<bool> &stop_signal);

			/// @brief Destroy the Pipeline object.
			~Pipeline();

			/// @brief Run the main processing loop.
			///
			/// This method blocks until the sensor stream is exhausted or an error
			/// occurs.
			void run();

		private:
			/// @brief Process a single image frame.
			/// @param img Pointer to the image data.
			void processImage(const ImageData *img);

			/// @brief Buffer incoming IMU data.
			/// @param imu Pointer to the IMU data.
			void bufferImu(const ImuData *imu);

		private:
			// Dependencies
			Config config_;
			std::unique_ptr<DataSource> sensorSource_;

			// VIO System
			std::unique_ptr<VisualOdometry3Dto2D> vo_;

			// State
			double lastFrameTime_ = -1.0;
			std::deque<ImuData> imuDataQueue_;
			ImuData lastImuMeasurement_;
	};

} // namespace mvio
