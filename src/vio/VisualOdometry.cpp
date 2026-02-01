#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mvio/vio/vo.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <vector>

Camera::Camera() {}

void VisualOdometry::detectNewFeatures(const cv::Mat &img) {
	if (features.size() >= config.minFeatures)
		return;

	cv::Mat mask = cv::Mat::ones(img.size(), CV_8U);
	for (const auto &f : features) {
		cv::circle(mask, f.point, 10, cv::Scalar(0), -1);
	}

	std::vector<cv::KeyPoint> keyPts;
	detector->detect(img, keyPts, mask);
	for (const auto &kp : keyPts) {
		Feature f;
		f.id = nextFeatureId++;
		f.point = kp.pt;
		f.has3D = false;
		features.push_back(f);
	}
}

void VisualOdometry::init(const cv::Mat &initFrame, double timestamp) {
	frame = initFrame;

	detectNewFeatures(initFrame);

	t_f = cv::Mat::zeros(3, 1, CV_64F);
	R_f = cv::Mat::eye(3, 3, CV_64F);
	camera.xyzGlobalTrajectory.push_back(cv::Point3f(0, 0, 0));
	camera.timestamps.push_back(timestamp);

	frameCount = 0;
	if (config.saveFrames) {
		saveFrame(frame, timestamp);
	}
}

std::vector<cv::Point2f> VisualOdometry::track(const cv::Mat &nextFrame) {
	if (features.empty())
		return {};

	std::vector<cv::Point2f> currPoints;
	for (const auto &f : features) {
		currPoints.push_back(f.point);
	}

	std::vector<cv::Point2f> projectedPoints;
	std::vector<uchar> status;
	std::vector<float> err;

	cv::calcOpticalFlowPyrLK(frame, nextFrame, currPoints, projectedPoints,
			status, err);

	std::vector<Feature> goodFeatures;
	std::vector<cv::Point2f> goodProjectedPoints;

	for (size_t i = 0; i < status.size(); ++i) {
		if (status[i]) {
			goodFeatures.push_back(features[i]);
			goodProjectedPoints.push_back(projectedPoints[i]);
		}
	}

	features = goodFeatures;

	return goodProjectedPoints;
}

float VisualOdometry::getAverageParallax(const std::vector<cv::Point2f> &pts) {
	if (features.empty() || pts.empty())
		return 0.0f;

	double totalDisplacement = 0.0;
	size_t size = std::min(features.size(), pts.size());
	for (size_t i = 0; i < size; ++i) {
		totalDisplacement += cv::norm(features[i].point - pts[i]);
	}
	return static_cast<float>(totalDisplacement / size);
}

void VisualOdometry::updateFrame(const cv::Mat &newFrame,
		const std::vector<cv::Point2f> newPoints) {
	frame = newFrame;

	for (size_t i = 0; i < features.size(); ++i) {
		features[i].point = newPoints[i];
	}

	if (features.size() < config.minFeatures) {
		detectNewFeatures(frame);
	}
}

bool VisualOdometry::computeEssentialMatrixAndPose(
		const std::vector<cv::Point2f> &points1,
		const std::vector<cv::Point2f> &points2, cv::Mat &E, cv::Mat &R, cv::Mat &t,
		cv::Mat &mask) {
	cv::Mat pts1 = cv::Mat(points1).clone();
	cv::Mat pts2 = cv::Mat(points2).clone();

	if (pts1.rows < 5) {
		return false;
	}

	E = cv::findEssentialMat(pts1, pts2, camera.cameraMatrix, cv::RANSAC,
			config.ransacProb, config.ransacThreshold, mask);

	if (E.empty() || E.rows != 3 || E.cols != 3) {
		return false;
	}

	cv::recoverPose(E, pts1, pts2, camera.cameraMatrix, R, t, mask);
	return true;
}

void VisualOdometry::filterInliers(
		const std::vector<cv::Point2f> &projectedPoints, const cv::Mat &mask,
		std::vector<cv::Point2f> &inlierProjectedPoints) {
	std::vector<Feature> inlierFeatures;
	inlierProjectedPoints.clear();

	for (int i = 0; i < mask.rows; i++) {
		if (mask.at<uchar>(i)) {
			inlierFeatures.push_back(features[i]);
			inlierProjectedPoints.push_back(projectedPoints[i]);
		}
	}
	features = inlierFeatures;
}

void VisualOdometry::updatePoseAndTrajectory(const cv::Mat &R, const cv::Mat &t,
		double timestamp) {
	cv::Mat R_inv = R.t();
	R_f = R_f * R_inv;

	t_f = t_f - scale * (R_f * t);

	camera.xyzGlobalTrajectory.push_back(
			cv::Point3f(t_f.at<double>(0), t_f.at<double>(1), t_f.at<double>(2)));
	camera.timestamps.push_back(timestamp);
}

void VisualOdometry::update(const cv::Mat &nextFrame, double timestamp) {
	if (!frame.empty()) {
		std::vector<cv::Point2f> projectedPoints = track(nextFrame);

		float avgDisplacement = getAverageParallax(projectedPoints);

		if (avgDisplacement > config.averageThreshold) {
			cv::Mat E, R, t, mask;

			std::vector<cv::Point2f> currentPoints;
			for (const auto &f : features)
				currentPoints.push_back(f.point);

			if (!computeEssentialMatrixAndPose(currentPoints, projectedPoints, E, R,
						t, mask)) {
				updateFrame(nextFrame, projectedPoints);
				return;
			}

			std::vector<cv::Point2f> inlierProjectedPoints;
			filterInliers(projectedPoints, mask, inlierProjectedPoints);

			updatePoseAndTrajectory(R, t, timestamp);

			updateFrame(nextFrame, inlierProjectedPoints);
		}
	} else {
		init(nextFrame, timestamp);
	}

	frameCount++;
	if (config.saveFrames && !frame.empty()) {
		saveFrame(frame, timestamp);
	}
}

void VisualOdometry::saveTrajectory(const std::string &baseDirectory) {
	namespace fs = std::filesystem;

	// Create timestamp
	auto now = std::chrono::system_clock::now();
	auto in_time_t = std::chrono::system_clock::to_time_t(now);
	std::stringstream ss;
	ss << std::put_time(std::localtime(&in_time_t), "%Y%m%d_%H%M%S");
	std::string timestamp = ss.str();

	std::string folderName = "run_" + timestamp;
	fs::path outputDirPath;

	if (outputDirectory.empty()) {
		outputDirPath = fs::path(baseDirectory) / folderName;
	} else {
		outputDirPath = fs::path(outputDirectory);
	}

	if (!fs::exists(outputDirPath)) {
		fs::create_directories(outputDirPath);
	}

	std::cout << "Saving results to " << outputDirPath << std::endl;

	// Save Trajectory
	std::string csvPath = (outputDirPath / "trajectory.csv").string();
	std::ofstream file(csvPath);
	if (!file.is_open()) {
		std::cerr << "Error opening file: " << csvPath << std::endl;
		return;
	}

	file << "timestamp,x,y,z" << std::endl;
	for (size_t i = 0; i < camera.xyzGlobalTrajectory.size(); ++i) {
		const auto &point = camera.xyzGlobalTrajectory[i];
		double timestamp = camera.timestamps[i];
		file << std::fixed << std::setprecision(6) << timestamp << "," << point.x
			<< "," << point.y << "," << point.z << "\n";
	}
	file.close();

	// Save Config
	std::string configPath = (outputDirPath / "config.yaml").string();
	config.save(configPath);
}

void VisualOdometry::ensureOutputDirectoryExists() {
	if (!outputDirectory.empty())
		return;

	namespace fs = std::filesystem;

	// Create timestamp
	auto now = std::chrono::system_clock::now();
	auto in_time_t = std::chrono::system_clock::to_time_t(now);
	std::stringstream ss;
	ss << std::put_time(std::localtime(&in_time_t), "%Y%m%d_%H%M%S");
	std::string timestamp = ss.str();

	std::string folderName = "run_" + timestamp;
	fs::path outputDir = fs::path("results") / folderName;

	if (!fs::exists(outputDir)) {
		fs::create_directories(outputDir);
	}
	outputDirectory = outputDir.string();
}

void VisualOdometry::saveFrame(const cv::Mat &frame, double timestamp) {
	ensureOutputDirectoryExists();
	namespace fs = std::filesystem;
	fs::path camDir = fs::path(outputDirectory) / "cam";
	if (!fs::exists(camDir)) {
		fs::create_directories(camDir);
	}

	std::stringstream ss;
	ss << camDir.string() << "/" << std::fixed << std::setprecision(6)
		<< timestamp << ".png";
	cv::imwrite(ss.str(), frame);
}

void VisualOdometry::saveImuMeasurement(const ImuData &imu) {
	ensureOutputDirectoryExists();

	if (!imuCsvFile.is_open()) {
		namespace fs = std::filesystem;
		std::string csvPath = (fs::path(outputDirectory) / "imu.csv").string();
		imuCsvFile.open(csvPath);
		if (imuCsvFile.is_open()) {
			imuCsvFile << "timestamp,ax,ay,az,gx,gy,gz" << std::endl;
		} else {
			std::cerr << "Failed to open IMU CSV file: " << csvPath << std::endl;
			return;
		}
	}

	imuCsvFile << std::fixed << std::setprecision(9) << imu.timestamp << ","
		<< std::setprecision(6) << imu.ax << "," << imu.ay << "," << imu.az
		<< "," << imu.gx << "," << imu.gy << "," << imu.gz << std::endl;
}
