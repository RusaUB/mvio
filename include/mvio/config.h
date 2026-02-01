#pragma once
#include <filesystem>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

struct Config {
	std::string imagePath;
	std::string imuPath;
	bool simulateRealtime;
	std::string dataSourceType; // "folder", "rpi", "phone"
	bool showStream;
	bool showFeatures;
	bool saveFrames;
	bool saveImu;
	double averageThreshold;

	// Shake Synchronization
	bool useShakeSync;
	double shakeAccThreshold;
	double shakeSearchWindow;
	double shakeSettleTime;

	// VO Parameters
	int minFeatures;
	double ransacThreshold;
	double ransacProb;

	// PnP Parameters (3D-2D)
	int pnpMinInliers;
	double pnpReprojectionError;
	double pnpConfidence;

	// VIO Initialization & Tracking Parameters
	int vioMinInitBuffer;
	double vioMinDt;
	double vioScaleMin;
	double vioScaleMax;
	double vioGravityMin;
	double vioGravityMax;

	int min3DPoints;
	double maxPointDepth;
	double minPointDepth;
	double maxPointLateral;
	double maxTranslationJump;

	int vioInitBufferSize;
	int vioMaxInitBufferSize;

	// IMU Initial Biases
	double imuInitBiasAx;
	double imuInitBiasAy;
	double imuInitBiasAz;
	double imuInitBiasGx;
	double imuInitBiasGy;
	double imuInitBiasGz;

	// Kalman Filter Parameters
	double kfAccNoise;
	double kfGyroNoise;
	double kfAccBiasNoise;
	double kfGyroBiasNoise;
	double kfPosCov;
	double kfRotCov;

	// Camera Intrinsics
	double fx, fy, cx, cy;
	double k1, k2, p1, p2;

	// Extrinsics (Body to Camera)
	cv::Mat T_bc_rot;   // 3x3 Rotation Matrix
	cv::Mat T_bc_trans; // 3x1 Translation Vector

	Config() {
		// Default values
		imagePath = "../dataset/infra1";
		imuPath = "../dataset/imu_px4.csv";
		simulateRealtime = false;
		dataSourceType = "folder";
		showStream = false;
		showFeatures = true;
		saveFrames = false;
		saveImu = false;
		averageThreshold = 20.0;

		useShakeSync = false;
		shakeAccThreshold = 30.0;
		shakeSearchWindow = 5.0;
		shakeSettleTime = 1.0;

		minFeatures = 200;
		ransacThreshold = 1.0;
		ransacProb = 0.999;

		pnpMinInliers = 10;
		pnpReprojectionError = 1.0;
		pnpConfidence = 0.99;

		vioMinInitBuffer = 4;
		vioMinDt = 0.001;
		vioScaleMin = 0.1;
		vioScaleMax = 100.0;
		vioGravityMin = 8.0;
		vioGravityMax = 12.0;

		min3DPoints = 6;
		maxPointDepth = 50.0;
		minPointDepth = 0.1;
		maxPointLateral = 500.0;
		maxTranslationJump = 10.0;

		vioInitBufferSize = 15;
		vioMaxInitBufferSize = 100;

		// Default IMU biases (zero)
		imuInitBiasAx = 0.0;
		imuInitBiasAy = 0.0;
		imuInitBiasAz = 0.0;
		imuInitBiasGx = 0.0;
		imuInitBiasGy = 0.0;
		imuInitBiasGz = 0.0;

		kfAccNoise = 0.01;
		kfGyroNoise = 0.01;
		kfAccBiasNoise = 1e-4;
		kfGyroBiasNoise = 1e-4;
		kfPosCov = 0.1;
		kfRotCov = 0.05;

		// Default Camera Intrinsics
		fx = 426.88;
		fy = 429.44;
		cx = 424.99;
		cy = 245.67;
		k1 = 0.0;
		k2 = 0.0;
		p1 = 0.0;
		p2 = 0.0;

		// Default Extrinsics (Identity)
		T_bc_rot = cv::Mat::eye(3, 3, CV_64F);
		T_bc_trans = cv::Mat::zeros(3, 1, CV_64F);
	}

	void parseFile(cv::FileStorage &fs, const std::string &filename = "") {
		if (!fs["imagePath"].empty())
			fs["imagePath"] >> imagePath;
		if (!fs["imuPath"].empty())
			fs["imuPath"] >> imuPath;
		if (!fs["simulateRealtime"].empty())
			fs["simulateRealtime"] >> simulateRealtime;
		if (!fs["averageThreshold"].empty())
			fs["averageThreshold"] >> averageThreshold;
		if (!fs["useShakeSync"].empty())
			fs["useShakeSync"] >> useShakeSync;
		if (!fs["shakeAccThreshold"].empty())
			fs["shakeAccThreshold"] >> shakeAccThreshold;
		if (!fs["shakeSearchWindow"].empty())
			fs["shakeSearchWindow"] >> shakeSearchWindow;
		if (!fs["shakeSettleTime"].empty())
			fs["shakeSettleTime"] >> shakeSettleTime;
		if (!fs["dataSourceType"].empty())
			fs["dataSourceType"] >> dataSourceType;
		if (!fs["showStream"].empty())
			fs["showStream"] >> showStream;
		if (!fs["showFeatures"].empty())
			fs["showFeatures"] >> showFeatures;
		if (!fs["saveFrames"].empty())
			fs["saveFrames"] >> saveFrames;
		if (!fs["saveImu"].empty())
			fs["saveImu"] >> saveImu;

		if (!fs["minFeatures"].empty())
			fs["minFeatures"] >> minFeatures;
		if (!fs["ransacThreshold"].empty())
			fs["ransacThreshold"] >> ransacThreshold;
		if (!fs["ransacProb"].empty())
			fs["ransacProb"] >> ransacProb;

		if (!fs["pnpMinInliers"].empty())
			fs["pnpMinInliers"] >> pnpMinInliers;
		if (!fs["pnpReprojectionError"].empty())
			fs["pnpReprojectionError"] >> pnpReprojectionError;
		if (!fs["pnpConfidence"].empty())
			fs["pnpConfidence"] >> pnpConfidence;

		if (!fs["vioMinInitBuffer"].empty())
			fs["vioMinInitBuffer"] >> vioMinInitBuffer;
		if (!fs["vioMinDt"].empty())
			fs["vioMinDt"] >> vioMinDt;
		if (!fs["vioScaleMin"].empty())
			fs["vioScaleMin"] >> vioScaleMin;
		if (!fs["vioScaleMax"].empty())
			fs["vioScaleMax"] >> vioScaleMax;
		if (!fs["vioGravityMin"].empty())
			fs["vioGravityMin"] >> vioGravityMin;
		if (!fs["vioGravityMax"].empty())
			fs["vioGravityMax"] >> vioGravityMax;

		if (!fs["min3DPoints"].empty())
			fs["min3DPoints"] >> min3DPoints;
		if (!fs["maxPointDepth"].empty())
			fs["maxPointDepth"] >> maxPointDepth;
		if (!fs["minPointDepth"].empty())
			fs["minPointDepth"] >> minPointDepth;
		if (!fs["maxPointLateral"].empty())
			fs["maxPointLateral"] >> maxPointLateral;
		if (!fs["maxTranslationJump"].empty())
			fs["maxTranslationJump"] >> maxTranslationJump;

		if (!fs["vioInitBufferSize"].empty())
			fs["vioInitBufferSize"] >> vioInitBufferSize;
		if (!fs["vioMaxInitBufferSize"].empty())
			fs["vioMaxInitBufferSize"] >> vioMaxInitBufferSize;

		// Load IMU initial biases
		if (!fs["imuInitBiasAx"].empty())
			fs["imuInitBiasAx"] >> imuInitBiasAx;
		if (!fs["imuInitBiasAy"].empty())
			fs["imuInitBiasAy"] >> imuInitBiasAy;
		if (!fs["imuInitBiasAz"].empty())
			fs["imuInitBiasAz"] >> imuInitBiasAz;
		if (!fs["imuInitBiasGx"].empty())
			fs["imuInitBiasGx"] >> imuInitBiasGx;
		if (!fs["imuInitBiasGy"].empty())
			fs["imuInitBiasGy"] >> imuInitBiasGy;
		if (!fs["imuInitBiasGz"].empty())
			fs["imuInitBiasGz"] >> imuInitBiasGz;

		if (!fs["kfAccNoise"].empty())
			fs["kfAccNoise"] >> kfAccNoise;
		if (!fs["kfGyroNoise"].empty())
			fs["kfGyroNoise"] >> kfGyroNoise;
		if (!fs["kfAccBiasNoise"].empty())
			fs["kfAccBiasNoise"] >> kfAccBiasNoise;
		if (!fs["kfGyroBiasNoise"].empty())
			fs["kfGyroBiasNoise"] >> kfGyroBiasNoise;
		if (!fs["kfPosCov"].empty())
			fs["kfPosCov"] >> kfPosCov;
		if (!fs["kfRotCov"].empty())
			fs["kfRotCov"] >> kfRotCov;

		// Load Camera Intrinsics
		if (!fs["fx"].empty())
			fs["fx"] >> fx;
		if (!fs["fy"].empty())
			fs["fy"] >> fy;
		if (!fs["cx"].empty())
			fs["cx"] >> cx;
		if (!fs["cy"].empty())
			fs["cy"] >> cy;
		if (!fs["k1"].empty())
			fs["k1"] >> k1;
		if (!fs["k2"].empty())
			fs["k2"] >> k2;
		if (!fs["p1"].empty())
			fs["p1"] >> p1;
		if (!fs["p2"].empty())
			fs["p2"] >> p2;

		// Load Extrinsics
		if (!fs["extrinsicRotation"].empty()) {
			fs["extrinsicRotation"] >> T_bc_rot;
		}
		if (!fs["extrinsicTranslation"].empty()) {
			fs["extrinsicTranslation"] >> T_bc_trans;
		}
	}

	void load(const std::string &filename) {
		std::string finalPath = filename;
		if (!std::filesystem::exists(finalPath)) {
			if (std::filesystem::exists("../" + filename)) {
				finalPath = "../" + filename;
			}
		}

		std::cout << "Loading config suite based on " << finalPath << std::endl;

		std::filesystem::path configDir =
			std::filesystem::path(finalPath).parent_path();
		std::vector<std::string> files = {finalPath,
			(configDir / "camera.yaml").string(),
			(configDir / "kalman.yaml").string()};

		bool anyLoaded = false;
		for (const auto &path : files) {
			if (std::filesystem::exists(path)) {
				cv::FileStorage fs(path, cv::FileStorage::READ);
				if (fs.isOpened()) {
					std::cout << "  - Loading " << path << "... ";
					parseFile(fs, path);
					std::cout << "OK" << std::endl;
					anyLoaded = true;
				}
			}
		}

		// Load Source Specific Config
		if (!dataSourceType.empty()) {
			std::filesystem::path sourceConfig =
				configDir / "sources" / (dataSourceType + ".yaml");
			if (std::filesystem::exists(sourceConfig)) {
				cv::FileStorage fs(sourceConfig.string(), cv::FileStorage::READ);
				if (fs.isOpened()) {
					std::cout << "  - Loading Source Config " << sourceConfig << "... ";
					parseFile(fs, sourceConfig.string());
					std::cout << "OK" << std::endl;
				}
			} else {
				std::cout << "Warning: Source config " << sourceConfig << " not found!"
					<< std::endl;
			}
		}

		if (!anyLoaded) {
			std::cout << "Warning: No config files found or opened. Using defaults."
				<< std::endl;
		} else {
			std::cout << "Configuration Summary:" << std::endl;
			std::cout << "  imagePath: " << imagePath << std::endl;
			std::cout << "  imuPath: " << imuPath << std::endl;
			std::cout << "  simulateRealtime: "
				<< (simulateRealtime ? "true" : "false") << std::endl;
			std::cout << "  DataSource: " << dataSourceType << std::endl;
			std::cout << "  averageThreshold: " << averageThreshold << std::endl;
			std::cout << "  minFeatures: " << minFeatures << std::endl;
			std::cout << "  IMU Bias Accel: [" << imuInitBiasAx << ", "
				<< imuInitBiasAy << ", " << imuInitBiasAz << "]" << std::endl;
			std::cout << "  IMU Bias Gyro: [" << imuInitBiasGx << ", "
				<< imuInitBiasGy << ", " << imuInitBiasGz << "]" << std::endl;
		}
	}

	void save(const std::string &filename) const {
		cv::FileStorage fs(filename, cv::FileStorage::WRITE);
		if (fs.isOpened()) {
			fs << "imagePath" << imagePath;
			fs << "imuPath" << imuPath;
			fs << "simulateRealtime" << simulateRealtime;
			fs << "showStream" << showStream;
			fs << "showFeatures" << showFeatures;
			fs << "saveFrames" << saveFrames;
			fs << "saveImu" << saveImu;
			fs << "dataSourceType" << dataSourceType;
			fs << "useShakeSync" << useShakeSync;
			fs << "shakeAccThreshold" << shakeAccThreshold;
			fs << "shakeSearchWindow" << shakeSearchWindow;
			fs << "shakeSettleTime" << shakeSettleTime;
			fs << "averageThreshold" << averageThreshold;

			fs << "minFeatures" << minFeatures;
			fs << "ransacThreshold" << ransacThreshold;
			fs << "ransacProb" << ransacProb;

			fs << "pnpMinInliers" << pnpMinInliers;
			fs << "pnpReprojectionError" << pnpReprojectionError;
			fs << "pnpConfidence" << pnpConfidence;

			fs << "vioMinInitBuffer" << vioMinInitBuffer;
			fs << "vioMinDt" << vioMinDt;
			fs << "vioScaleMin" << vioScaleMin;
			fs << "vioScaleMax" << vioScaleMax;
			fs << "vioGravityMin" << vioGravityMin;
			fs << "vioGravityMax" << vioGravityMax;

			fs << "min3DPoints" << min3DPoints;
			fs << "maxPointDepth" << maxPointDepth;
			fs << "minPointDepth" << minPointDepth;
			fs << "maxPointLateral" << maxPointLateral;
			fs << "maxTranslationJump" << maxTranslationJump;

			fs << "vioInitBufferSize" << vioInitBufferSize;
			fs << "vioMaxInitBufferSize" << vioMaxInitBufferSize;

			// Save IMU initial biases
			fs << "imuInitBiasAx" << imuInitBiasAx;
			fs << "imuInitBiasAy" << imuInitBiasAy;
			fs << "imuInitBiasAz" << imuInitBiasAz;
			fs << "imuInitBiasGx" << imuInitBiasGx;
			fs << "imuInitBiasGy" << imuInitBiasGy;
			fs << "imuInitBiasGz" << imuInitBiasGz;

			fs << "kfAccNoise" << kfAccNoise;
			fs << "kfGyroNoise" << kfGyroNoise;
			fs << "kfAccBiasNoise" << kfAccBiasNoise;
			fs << "kfGyroBiasNoise" << kfGyroBiasNoise;
			fs << "kfPosCov" << kfPosCov;
			fs << "kfRotCov" << kfRotCov;

			// Save Camera Intrinsics
			fs << "fx" << fx;
			fs << "fy" << fy;
			fs << "cx" << cx;
			fs << "cy" << cy;
			fs << "k1" << k1;
			fs << "k2" << k2;
			fs << "p1" << p1;
			fs << "p2" << p2;

			// Save Extrinsics
			fs << "extrinsicRotation" << T_bc_rot;
			fs << "extrinsicTranslation" << T_bc_trans;

			fs.release();
			std::cout << "Saved config to " << filename << std::endl;
		} else {
			std::cerr << "Failed to save config to " << filename << std::endl;
		}
	}
};
