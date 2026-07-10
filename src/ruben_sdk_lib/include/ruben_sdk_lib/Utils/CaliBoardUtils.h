#pragma once
#include <RVC/RVC.h>

#include <opencv2/features2d.hpp>

/**
 * @brief Get the calibration board pose in camera coordinate
 *
 * @param pm[in]
 * @param image[in]
 * @param centerDistance[in]
 * @param calibBoardSize[in]
 * @param blobDetectorParam[in]
 * @param R[out]
 * @param T[out]
 * @param debugImage[out] detected information of calibration board if debugImage is not nullptr. default = nullptr.
 * @return true
 * @return false
 */
extern bool GetCaliBoardPose(RVC::PointMap &pm, const RVC::Image &image, const double centerDistance,
                             const int calibBoardSize[2], const cv::SimpleBlobDetector::Params &blobDetectorParam,
                             double R[9], double T[3], cv::Mat *debugImage = nullptr);

/**
 * @brief compute inverse pose of R T
 *
 * @param R[in]
 * @param T[in]
 * @param invR[out]
 * @param invT[out]
 */
extern void RigidTransformInv(const double R[9], const double T[3], double invR[9], double invT[3]);

/**
 * @brief transform pointmap
 *
 * @param pm[in,out]
 * @param R[in]
 * @param T[in]
 */
extern void RigidTransformPointMap(RVC::PointMap &pm, const double R[9], const double T[3]);

/**
 * @brief print homogeneous matrix
 *
 * @param R[in]
 * @param T[in]
 */
extern void PrintHomogeneousMatrix(const double R[9], const double T[3]);
