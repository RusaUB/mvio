#pragma once
#include <fstream>
#include <iostream>
#include <mvio/config.h>
#include <mvio/sensor/core.h>
#include <mvio/vio/imu.h>

#include <memory>
#include <mvio/estimators/estimator.h>
#include <opencv2/features2d.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

class Camera {
	public:
		cv::Mat cameraMatrix;

		std::vector<cv::Point3f> xyzGlobalTrajectory;
		std::vector<double> timestamps;

		Camera();
};

struct Feature {
	size_t id;
	cv::Point2f point;
	cv::Point3f point3D;
	bool has3D = false;
};

class VisualOdometry {
	public:
		Config config;
		double scale = 1.00;

		cv::Mat frame;
		Camera camera;

		cv::Mat R_f;
		cv::Mat t_f;

		cv::Ptr<cv::Feature2D> detector;
		std::vector<Feature> features;
		size_t nextFeatureId = 0;

		// Frame saving
		std::string outputDirectory;
		int frameCount = 0;
		void ensureOutputDirectoryExists();
		void saveFrame(const cv::Mat &frame, double timestamp);
		void saveImuMeasurement(const ImuData &imu);

		std::ofstream imuCsvFile;

		void init(const cv::Mat &initFrame, double timestamp = 0.0);
		virtual std::vector<cv::Point2f> track(const cv::Mat &nextFrame);
		float getAverageParallax(const std::vector<cv::Point2f> &pts);

		void detectNewFeatures(const cv::Mat &img);
		void updateFrame(const cv::Mat &newFrame,
				std::vector<cv::Point2f> newFeatures);
		virtual void update(const cv::Mat &nextFrame, double timestamp = 0.0);
		void saveTrajectory(const std::string &baseDirectory = "results");
		bool computeEssentialMatrixAndPose(const std::vector<cv::Point2f> &points1,
				const std::vector<cv::Point2f> &points2,
				cv::Mat &E, cv::Mat &R, cv::Mat &t,
				cv::Mat &mask);

		void filterInliers(const std::vector<cv::Point2f> &projectedPoints,
				const cv::Mat &mask,
				std::vector<cv::Point2f> &inlierProjectedPoints);

		void updatePoseAndTrajectory(const cv::Mat &R, const cv::Mat &t,
				double timestamp);

		VisualOdometry(const Config &_config,
				cv::Ptr<cv::Feature2D> _detector = cv::ORB::create())
			: config(_config), detector(_detector) {
				camera.cameraMatrix = (cv::Mat_<double>(3, 3) << config.fx, 0.0, config.cx,
						0.0, config.fy, config.cy, 0.0, 0.0, 1.0);
			}
};

class VisualOdometry3Dto2D : public VisualOdometry {
	public:
		struct InitData {
			double timestamp;
			cv::Mat R_f;
			cv::Mat t_f;
			ImuPreintegrator preint;
		};

		std::vector<InitData> initBuffer;
		bool isInitialized = false;

		Eigen::Vector3d estimatedGravity;
		ImuPreintegrator imu;
		std::unique_ptr<IEstimator> estimator;
		using VisualOdometry::VisualOdometry;
		void update(const cv::Mat &nextFrame, double timestamp = 0.0) override;
		void update(const cv::Mat &nextFrame, const std::vector<ImuData> &imuBuffer,
				double timestamp = 0.0);
		bool estimateGravityAndScale();

	protected:
		bool
			triangulateInitialFeatures(const cv::Mat &R, const cv::Mat &t,
					const std::vector<cv::Point2f> &pts1,
					const std::vector<cv::Point2f> &pts2,
					const std::vector<Feature> &currentFeatures,
					std::vector<Feature> &validFeatures,
					std::vector<cv::Point2f> &validProjectedPoints);

		bool estimatePosePnP(const std::vector<cv::Point2f> &projectedPoints,
				cv::Mat &R_out, cv::Mat &t_out,
				std::vector<int> &inliers, std::vector<int> &pnpIndices);

		void triangulateNewCandidates(
				const cv::Mat &R_prev, const cv::Mat &t_prev, const cv::Mat &R_curr,
				const cv::Mat &t_curr, const std::vector<Feature> &candidates,
				const std::vector<cv::Point2f> &candidateProjectedPoints,
				std::vector<Feature> &nextFeatures,
				std::vector<cv::Point2f> &nextProjectedPoints);

		bool validate3DPoint(const cv::Point3f &pt3D);

		static Eigen::Matrix3d cvMatToEigenMat(const cv::Mat &cvMat);
		static Eigen::Vector3d cvMatToEigenVec(const cv::Mat &cvVec);
};
