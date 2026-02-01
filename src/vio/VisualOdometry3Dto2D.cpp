#include <Eigen/Dense>
#include <iostream>
#include <mvio/vio/imu.h>
#include <mvio/vio/vo.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <vector>

Eigen::Matrix3d VisualOdometry3Dto2D::cvMatToEigenMat(const cv::Mat &cvMat) {
	Eigen::Matrix3d eigenMat;
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			eigenMat(i, j) = cvMat.at<double>(i, j);
	return eigenMat;
}

Eigen::Vector3d VisualOdometry3Dto2D::cvMatToEigenVec(const cv::Mat &cvVec) {
	Eigen::Vector3d eigenVec;
	eigenVec(0) = cvVec.at<double>(0);
	eigenVec(1) = cvVec.at<double>(1);
	eigenVec(2) = cvVec.at<double>(2);
	return eigenVec;
}

bool VisualOdometry3Dto2D::validate3DPoint(const cv::Point3f &pt3D) {
	return (pt3D.z > config.minPointDepth && pt3D.z < config.maxPointDepth &&
			std::abs(pt3D.x) < config.maxPointLateral &&
			std::abs(pt3D.y) < config.maxPointLateral);
}

bool VisualOdometry3Dto2D::triangulateInitialFeatures(
		const cv::Mat &R, const cv::Mat &t, const std::vector<cv::Point2f> &pts1,
		const std::vector<cv::Point2f> &pts2,
		const std::vector<Feature> &currentFeatures,
		std::vector<Feature> &validFeatures,
		std::vector<cv::Point2f> &validProjectedPoints) {

	cv::Mat P1 = camera.cameraMatrix * cv::Mat::eye(3, 4, CV_64F);
	cv::Mat Rt = (cv::Mat_<double>(3, 4) << R.at<double>(0, 0),
			R.at<double>(0, 1), R.at<double>(0, 2), t.at<double>(0),
			R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
			t.at<double>(1), R.at<double>(2, 0), R.at<double>(2, 1),
			R.at<double>(2, 2), t.at<double>(2));
	cv::Mat P2 = camera.cameraMatrix * Rt;

	P1.convertTo(P1, CV_32F);
	P2.convertTo(P2, CV_32F);

	cv::Mat pts1Mat(2, pts1.size(), CV_32F);
	cv::Mat pts2Mat(2, pts2.size(), CV_32F);

	for (size_t i = 0; i < pts1.size(); i++) {
		pts1Mat.at<float>(0, i) = pts1[i].x;
		pts1Mat.at<float>(1, i) = pts1[i].y;
		pts2Mat.at<float>(0, i) = pts2[i].x;
		pts2Mat.at<float>(1, i) = pts2[i].y;
	}

	cv::Mat points4D;
	cv::triangulatePoints(P1, P2, pts1Mat, pts2Mat, points4D);

	validFeatures.clear();
	validProjectedPoints.clear();

	for (int i = 0; i < points4D.cols; i++) {
		cv::Mat col = points4D.col(i);
		col /= col.at<float>(3);
		cv::Point3f pt3D(col.at<float>(0), col.at<float>(1), col.at<float>(2));

		if (validate3DPoint(pt3D)) {
			Feature f = currentFeatures[i];
			f.point3D = pt3D;
			f.has3D = true;

			validFeatures.push_back(f);
			validProjectedPoints.push_back(pts2[i]);
		}
	}

	return !validFeatures.empty();
}

bool VisualOdometry3Dto2D::estimatePosePnP(
		const std::vector<cv::Point2f> &projectedPoints, cv::Mat &R_wc_out,
		cv::Mat &t_wc_out, std::vector<int> &inliersIndices,
		std::vector<int> &pnpIndices) {

	std::vector<cv::Point3f> pnpObjPts;
	std::vector<cv::Point2f> pnpImgPts;
	pnpIndices.clear();

	for (size_t i = 0; i < features.size(); ++i) {
		if (features[i].has3D) {
			pnpObjPts.push_back(features[i].point3D);
			pnpImgPts.push_back(projectedPoints[i]);
			pnpIndices.push_back(i);
		}
	}

	if (pnpImgPts.size() < config.min3DPoints) {
		return false;
	}

	cv::Mat rvec, tvec, inliers;
	bool success = cv::solvePnPRansac(
			pnpObjPts, pnpImgPts, camera.cameraMatrix, cv::Mat(), rvec, tvec, false,
			100, config.pnpReprojectionError, config.pnpConfidence, inliers);

	if (!success || inliers.rows < config.pnpMinInliers) {
		return false;
	}

	inliersIndices.clear();
	for (int i = 0; i < inliers.rows; i++) {
		inliersIndices.push_back(inliers.at<int>(i));
	}

	cv::Mat R_cw;
	cv::Rodrigues(rvec, R_cw);
	cv::Mat t_cw = tvec;

	R_wc_out = R_cw.t();
	t_wc_out = -R_wc_out * t_cw;

	return true;
}

void VisualOdometry3Dto2D::triangulateNewCandidates(
		const cv::Mat &R_prev, const cv::Mat &t_prev, const cv::Mat &R_curr,
		const cv::Mat &t_curr, const std::vector<Feature> &candidates,
		const std::vector<cv::Point2f> &candidateProjectedPoints,
		std::vector<Feature> &nextFeatures,
		std::vector<cv::Point2f> &nextProjectedPoints) {

	if (candidates.empty())
		return;

	cv::Mat R_cw_prev = R_prev.t();
	cv::Mat t_cw_prev = -R_cw_prev * t_prev;

	cv::Mat R_cw_curr = R_curr.t();
	cv::Mat t_cw_curr = -R_cw_curr * t_curr;

	cv::Mat P1(3, 4, CV_64F);
	R_cw_prev.copyTo(P1.colRange(0, 3));
	t_cw_prev.copyTo(P1.col(3));
	P1 = camera.cameraMatrix * P1;

	cv::Mat P2(3, 4, CV_64F);
	R_cw_curr.copyTo(P2.colRange(0, 3));
	t_cw_curr.copyTo(P2.col(3));
	P2 = camera.cameraMatrix * P2;

	P1.convertTo(P1, CV_32F);
	P2.convertTo(P2, CV_32F);

	cv::Mat pts1Mat(2, candidates.size(), CV_32F);
	cv::Mat pts2Mat(2, candidateProjectedPoints.size(), CV_32F);

	for (size_t i = 0; i < candidates.size(); i++) {
		pts1Mat.at<float>(0, i) = candidates[i].point.x;
		pts1Mat.at<float>(1, i) = candidates[i].point.y;
		pts2Mat.at<float>(0, i) = candidateProjectedPoints[i].x;
		pts2Mat.at<float>(1, i) = candidateProjectedPoints[i].y;
	}

	cv::Mat points4D;
	cv::triangulatePoints(P1, P2, pts1Mat, pts2Mat, points4D);

	for (int i = 0; i < points4D.cols; i++) {
		cv::Mat col = points4D.col(i);
		col /= col.at<float>(3);
		cv::Point3f pt3D(col.at<float>(0), col.at<float>(1), col.at<float>(2));

		cv::Mat pt3Dmat = (cv::Mat_<double>(3, 1) << pt3D.x, pt3D.y, pt3D.z);
		cv::Mat ptCam = R_cw_curr * pt3Dmat + t_cw_curr;

		if (ptCam.at<double>(2) > config.minPointDepth &&
				ptCam.at<double>(2) < config.maxPointDepth &&
				std::abs(pt3D.x) < config.maxPointLateral &&
				std::abs(pt3D.y) < config.maxPointLateral) {

			Feature f = candidates[i];
			f.point3D = pt3D;
			f.has3D = true;
			nextFeatures.push_back(f);
			nextProjectedPoints.push_back(candidateProjectedPoints[i]);
		}
	}
}

void VisualOdometry3Dto2D::update(const cv::Mat &nextFrame, double timestamp) {
	if (!frame.empty()) {
		std::vector<cv::Point2f> projectedPoints = track(nextFrame);
		float avgDisplacement = getAverageParallax(projectedPoints);

		if (avgDisplacement > config.averageThreshold) {

			int count3D = 0;
			for (const auto &f : features)
				if (f.has3D)
					count3D++;

			if (count3D < config.min3DPoints) {
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

				if (features.empty()) {
					updateFrame(nextFrame, inlierProjectedPoints);
					return;
				}

				std::vector<Feature> inlierFeatures;
				for (size_t i = 0; i < mask.rows; ++i) {
					if (mask.at<unsigned char>(i)) {
						inlierFeatures.push_back(features[i]);
					}
				}

				std::vector<cv::Point2f> inlierOriginalPoints;
				for (const auto &f : inlierFeatures)
					inlierOriginalPoints.push_back(f.point);

				std::vector<Feature> validFeatures;
				std::vector<cv::Point2f> validProjectedPoints;

				triangulateInitialFeatures(R, t, inlierOriginalPoints,
						inlierProjectedPoints, inlierFeatures,
						validFeatures, validProjectedPoints);

				features = validFeatures;
				updatePoseAndTrajectory(R, t, timestamp);
				updateFrame(nextFrame, validProjectedPoints);

			} else {
				cv::Mat R_wc, t_wc;
				std::vector<int> inlierIndices;
				std::vector<int> pnpIndices;

				if (!estimatePosePnP(projectedPoints, R_wc, t_wc, inlierIndices,
							pnpIndices)) {
					updateFrame(nextFrame, projectedPoints);
					return;
				}

				double distance = cv::norm(t_wc - t_f);
				if (distance > config.maxTranslationJump) {
					updateFrame(nextFrame, projectedPoints);
					return;
				}

				cv::Mat R_wc_prev = R_f.clone();
				cv::Mat t_wc_prev = t_f.clone();

				R_f = R_wc;
				t_f = t_wc;
				camera.xyzGlobalTrajectory.push_back(cv::Point3f(
							t_f.at<double>(0), t_f.at<double>(1), t_f.at<double>(2)));
				camera.timestamps.push_back(timestamp);

				std::vector<Feature> nextFeatures;
				std::vector<cv::Point2f> nextProjectedPoints;
				std::vector<bool> isPnPInlier(features.size(), false);

				for (int idx : inlierIndices) {
					int originalIdx = pnpIndices[idx];
					isPnPInlier[originalIdx] = true;
					nextFeatures.push_back(features[originalIdx]);
					nextProjectedPoints.push_back(projectedPoints[originalIdx]);
				}

				std::vector<Feature> candidates;
				std::vector<cv::Point2f> candidateProjectedPoints;

				for (size_t i = 0; i < features.size(); ++i) {
					if (!isPnPInlier[i] && !features[i].has3D) {
						candidates.push_back(features[i]);
						candidateProjectedPoints.push_back(projectedPoints[i]);
					}
				}

				triangulateNewCandidates(R_wc_prev, t_wc_prev, R_f, t_f, candidates,
						candidateProjectedPoints, nextFeatures,
						nextProjectedPoints);

				features = nextFeatures;
				updateFrame(nextFrame, nextProjectedPoints);
			}
		}
	} else {
		init(nextFrame, timestamp);
	}
	if (config.saveFrames && !frame.empty()) {
		saveFrame(frame, timestamp);
	}
}
