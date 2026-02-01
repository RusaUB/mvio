#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/QR>
#include <iostream>
#include <mvio/estimators/kalman_filter.h>
#include <mvio/vio/imu.h>
#include <mvio/vio/vo.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <vector>

bool VisualOdometry3Dto2D::estimateGravityAndScale() {
  int N = initBuffer.size() - 1;
  if (N < config.vioMinInitBuffer)
    return false;

  int dim_eq = 6 * N;
  int dim_x = 3 * (N + 1) + 4;
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(dim_eq, dim_x);
  Eigen::VectorXd b = Eigen::VectorXd::Zero(dim_eq);

  for (int k = 0; k < N; ++k) {
    const auto &d0 = initBuffer[k];
    const auto &d1 = initBuffer[k + 1];
    const auto &preint = d1.preint;
    double dt = d1.timestamp - d0.timestamp;

    if (dt <= 1e-4)
      continue;

    Eigen::Matrix3d R_k_cam = VisualOdometry3Dto2D::cvMatToEigenMat(d0.R_f);
    Eigen::Vector3d P_k = VisualOdometry3Dto2D::cvMatToEigenVec(d0.t_f);
    Eigen::Vector3d P_k1 = VisualOdometry3Dto2D::cvMatToEigenVec(d1.t_f);

    Eigen::Matrix3d R_bc =
        VisualOdometry3Dto2D::cvMatToEigenMat(config.T_bc_rot);
    Eigen::Vector3d t_bc =
        VisualOdometry3Dto2D::cvMatToEigenVec(config.T_bc_trans);

    Eigen::Matrix3d R_wb_k = R_k_cam * R_bc;
    Eigen::Matrix3d R_k1_cam = VisualOdometry3Dto2D::cvMatToEigenMat(d1.R_f);
    Eigen::Matrix3d R_wb_k1 = R_k1_cam * R_bc;

    int idx_v_k = 3 * k;
    int idx_v_k1 = 3 * (k + 1);
    int idx_g = 3 * (N + 1);
    int idx_s = 3 * (N + 1) + 3;

    int row1 = 6 * k;
    A.block<3, 3>(row1, idx_v_k1) = Eigen::Matrix3d::Identity();
    A.block<3, 3>(row1, idx_v_k) = -Eigen::Matrix3d::Identity();
    A.block<3, 3>(row1, idx_g) = -dt * Eigen::Matrix3d::Identity();
    b.segment<3>(row1) = R_wb_k * preint.delta_v;

    int row2 = 6 * k + 3;
    A.block<3, 3>(row2, idx_v_k) = -dt * Eigen::Matrix3d::Identity();
    A.block<3, 3>(row2, idx_g) = -0.5 * dt * dt * Eigen::Matrix3d::Identity();
    A.block<3, 1>(row2, idx_s) = (P_k1 - P_k);

    Eigen::Vector3d lever_arm_correction = (R_k1_cam - R_k_cam) * t_bc;
    b.segment<3>(row2) = R_wb_k * preint.delta_p + lever_arm_correction;
  }

  Eigen::VectorXd x = A.colPivHouseholderQr().solve(b);

  Eigen::Vector3d g_est = x.segment<3>(3 * (N + 1));
  double s_est = x(3 * (N + 1) + 3);

  std::cout << "[VIO Init] Estimated Gravity: " << g_est.transpose()
            << " | Norm: " << g_est.norm() << std::endl;
  std::cout << "[VIO Init] Estimated Scale: " << s_est << std::endl;

  if (s_est > config.vioScaleMin && s_est < config.vioScaleMax &&
      g_est.norm() > config.vioGravityMin &&
      g_est.norm() < config.vioGravityMax) {
    this->scale = s_est;
    this->estimatedGravity = g_est;
    return true;
  } else {
    std::cout << "[VIO Init] Failed validation. Retrying..." << std::endl;
    return false;
  }
}

void VisualOdometry3Dto2D::update(const cv::Mat &nextFrame,
                                  const std::vector<ImuData> &imuBuffer,
                                  double timestamp) {
  if (!frame.empty()) {
    if (isInitialized) {
      estimator->predict(imuBuffer);
    } else {
      imu.propagate(imuBuffer);
    }

    std::vector<cv::Point2f> projectedPoints = track(nextFrame);

    if (projectedPoints.empty()) {
      updateFrame(nextFrame, projectedPoints);
      return;
    }

    float avgDisplacement = getAverageParallax(projectedPoints);

    std::cout << "[VIO] Tracked Features: " << projectedPoints.size()
              << std::endl;

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

        if (triangulateInitialFeatures(R, t, inlierOriginalPoints,
                                       inlierProjectedPoints, inlierFeatures,
                                       validFeatures, validProjectedPoints)) {

          for (auto &f : validFeatures) {
            cv::Mat pt3D_local_mat = (cv::Mat_<double>(3, 1) << f.point3D.x,
                                      f.point3D.y, f.point3D.z);
            cv::Mat pt3D_global_mat = R_f * pt3D_local_mat + t_f;
            f.point3D = cv::Point3f(pt3D_global_mat.at<double>(0),
                                    pt3D_global_mat.at<double>(1),
                                    pt3D_global_mat.at<double>(2));
          }
          features = validFeatures;
          updatePoseAndTrajectory(R, t, timestamp);

          if (isInitialized) {
            Eigen::Matrix3d R_meas = VisualOdometry3Dto2D::cvMatToEigenMat(R_f);
            Eigen::Vector3d t_meas = VisualOdometry3Dto2D::cvMatToEigenVec(t_f);

            Eigen::Matrix3d R_bc =
                VisualOdometry3Dto2D::cvMatToEigenMat(config.T_bc_rot);
            Eigen::Vector3d t_bc =
                VisualOdometry3Dto2D::cvMatToEigenVec(config.T_bc_trans);

            Eigen::Matrix3d R_wb = R_meas * R_bc.transpose();
            Eigen::Vector3d t_wb = t_meas - R_wb * t_bc;

            Eigen::Quaterniond q_meas(R_wb);

            Eigen::Matrix<double, 6, 6> R_cov =
                Eigen::Matrix<double, 6, 6>::Identity();
            R_cov.block<3, 3>(0, 0) *= config.kfPosCov;
            R_cov.block<3, 3>(3, 3) *= config.kfRotCov;

            estimator->update_pose(t_wb, q_meas, R_cov);

            Eigen::Vector3d t_filt_body = estimator->getPosition();
            Eigen::Quaterniond q_filt_body = estimator->getRotation();
            Eigen::Matrix3d R_filt_body = q_filt_body.toRotationMatrix();

            Eigen::Vector3d t_filt_cam = t_filt_body + R_filt_body * t_bc;

            if (!camera.xyzGlobalTrajectory.empty()) {
              camera.xyzGlobalTrajectory.back() =
                  cv::Point3f(t_filt_cam(0), t_filt_cam(1), t_filt_cam(2));
            }
          }

          updateFrame(nextFrame, validProjectedPoints);
        } else {
          updateFrame(nextFrame, projectedPoints);
          return;
        }

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

        if (isInitialized) {
          Eigen::Matrix3d R_meas = VisualOdometry3Dto2D::cvMatToEigenMat(R_wc);
          Eigen::Vector3d t_meas = VisualOdometry3Dto2D::cvMatToEigenVec(t_wc);

          Eigen::Matrix3d R_bc =
              VisualOdometry3Dto2D::cvMatToEigenMat(config.T_bc_rot);
          Eigen::Vector3d t_bc =
              VisualOdometry3Dto2D::cvMatToEigenVec(config.T_bc_trans);

          Eigen::Matrix3d R_wb = R_meas * R_bc.transpose();
          Eigen::Vector3d t_wb = t_meas - R_wb * t_bc;

          Eigen::Quaterniond q_meas(R_wb);

          Eigen::Matrix<double, 6, 6> R_cov =
              Eigen::Matrix<double, 6, 6>::Identity();
          R_cov.block<3, 3>(0, 0) *= config.kfPosCov;
          R_cov.block<3, 3>(3, 3) *= config.kfRotCov;

          estimator->update_pose(t_wb, q_meas, R_cov);

          Eigen::Vector3d t_filt_body = estimator->getPosition();
          Eigen::Quaterniond q_filt_body = estimator->getRotation();
          Eigen::Matrix3d R_filt_body = q_filt_body.toRotationMatrix();
          Eigen::Vector3d t_filt_cam = t_filt_body + R_filt_body * t_bc;
        }

        R_f = R_wc;
        t_f = t_wc;
        camera.xyzGlobalTrajectory.push_back(cv::Point3f(
            t_f.at<double>(0), t_f.at<double>(1), t_f.at<double>(2)));
        camera.timestamps.push_back(timestamp);

        if (isInitialized) {
          Eigen::Vector3d t_filt_body = estimator->getPosition();
          Eigen::Quaterniond q_filt_body = estimator->getRotation();
          Eigen::Matrix3d R_filt_body = q_filt_body.toRotationMatrix();

          Eigen::Matrix3d R_bc =
              VisualOdometry3Dto2D::cvMatToEigenMat(config.T_bc_rot);
          Eigen::Vector3d t_bc =
              VisualOdometry3Dto2D::cvMatToEigenVec(config.T_bc_trans);

          Eigen::Vector3d t_filt_cam = t_filt_body + R_filt_body * t_bc;
          camera.xyzGlobalTrajectory.back() =
              cv::Point3f(t_filt_cam(0), t_filt_cam(1), t_filt_cam(2));
        }

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

        if (!isInitialized) {
          if (imu.last_timestamp > 0) {
            InitData data;
            data.timestamp = imu.last_timestamp;
            data.R_f = R_f.clone();
            data.t_f = t_f.clone();
            data.preint = imu;

            initBuffer.push_back(data);

            if (initBuffer.size() > config.vioInitBufferSize) {
              if (estimateGravityAndScale()) {
                isInitialized = true;

                double s = this->scale;

                t_f *= s;

                for (auto &f : features) {
                  if (f.has3D) {
                    f.point3D.x *= s;
                    f.point3D.y *= s;
                    f.point3D.z *= s;
                  }
                }

                for (auto &p : camera.xyzGlobalTrajectory) {
                  p.x *= s;
                  p.y *= s;
                  p.z *= s;
                }

                initBuffer.clear();

                Eigen::Matrix3d R_curr =
                    VisualOdometry3Dto2D::cvMatToEigenMat(R_f);
                Eigen::Vector3d t_curr =
                    VisualOdometry3Dto2D::cvMatToEigenVec(t_f);

                Eigen::Matrix3d R_bc =
                    VisualOdometry3Dto2D::cvMatToEigenMat(config.T_bc_rot);
                Eigen::Vector3d t_bc =
                    VisualOdometry3Dto2D::cvMatToEigenVec(config.T_bc_trans);

                Eigen::Matrix3d R_wb = R_curr * R_bc.transpose();
                Eigen::Vector3d t_wb = t_curr - R_wb * t_bc;

                Eigen::Quaterniond q_curr_body(R_wb);

                estimator = std::make_unique<KalmanFilter>();
                estimator->init(t_wb, Eigen::Vector3d::Zero(), q_curr_body,
                                Eigen::Vector3d::Zero(),
                                Eigen::Vector3d::Zero(), config.kfAccNoise,
                                config.kfGyroNoise, config.kfAccBiasNoise,
                                config.kfGyroBiasNoise);
                estimator->setGravity(estimatedGravity);

                std::cout << "[VIO] Initialized Kalman Filter." << std::endl;
              } else {
                if (initBuffer.size() > config.vioMaxInitBufferSize) {
                  initBuffer.erase(initBuffer.begin());
                }
              }
            }
          }
          imu.reset();
        }
      }
    }
  } else {
    init(nextFrame, timestamp);
    imu.propagate(imuBuffer);
  }
  if (config.saveFrames && !frame.empty()) {
    saveFrame(frame, timestamp);
  }
}
