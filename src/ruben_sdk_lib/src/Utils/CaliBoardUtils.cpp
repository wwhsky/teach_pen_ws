#include "ruben_sdk_lib/Utils/CaliBoardUtils.h"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>
#include <iostream>
#include <iomanip>

static const cv::Mat ToCVGrayImage(const RVC::Image &image) {
    cv::Mat cv_image;
    RVC::ImageType::Enum image_type = image.GetType();
    const unsigned char *image_data = image.GetDataConstPtr();
    const RVC::Size image_size = image.GetSize();
    switch (image_type) {
    case RVC::ImageType::Mono8: {
        cv_image = cv::Mat(image_size.rows, image_size.cols, CV_8UC1, const_cast<unsigned char *>(image_data));
        break;
    }
    case RVC::ImageType::RGB8: {
        cv::Mat cv_image_rgb8(image_size.rows, image_size.cols, CV_8UC3, const_cast<unsigned char *>(image_data));
        cv::cvtColor(cv_image_rgb8, cv_image, cv::COLOR_RGB2GRAY);
        break;
    }
    case RVC::ImageType::BGR8: {
        cv::Mat cv_image_bgr8(image_size.rows, image_size.cols, CV_8UC3, const_cast<unsigned char *>(image_data));
        cv::cvtColor(cv_image_bgr8, cv_image, cv::COLOR_BGR2GRAY);
        break;
    }
    default:
        break;
    }
    return cv_image;
}

static bool DetectCVCircle(const RVC::Image &image, const cv::Ptr<cv::SimpleBlobDetector> &blobDetector,
                           const int calibBoardSize[2], std::vector<cv::Point2f> &centers, cv::Mat *debugImage) {
    const cv::Mat cv_image = ToCVGrayImage(image);
    if (cv_image.empty()) {
        printf("image is empty\n");
        return false;
    }
    bool found = cv::findCirclesGrid(cv_image, cv::Size(calibBoardSize[0], calibBoardSize[1]), centers,
                                     cv::CALIB_CB_ASYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING, blobDetector);
    if (debugImage) {
        cv::cvtColor(cv_image, *debugImage, cv::COLOR_GRAY2BGR);
        if (!found) {
            std::vector<cv::KeyPoint> key_point;
            blobDetector->detect(cv_image, key_point);
            cv::drawKeypoints(cv_image, key_point, *debugImage, cv::Scalar(0, 0, 255));
        } else {
            cv::drawChessboardCorners(*debugImage, cv::Size(calibBoardSize[0], calibBoardSize[1]), centers, found);
        }
    }
    return found;
}

static void ExtractCenterFromPCD(RVC::PointMap &pm, const std::vector<cv::Point2f> &centers,
                                 std::vector<double> &xyzs) {
    double nan = std::numeric_limits<double>::quiet_NaN();
    const int center_size = centers.size();
    const RVC::Size pm_size = pm.GetSize();
    const double *pm_data = pm.GetPointDataConstPtr();
    xyzs.resize(center_size * 3);
    double *xyz = xyzs.data();
    for (size_t i = 0; i < center_size; i++, xyz += 3) {
        const int id_x0 = centers[i].x, id_y0 = centers[i].y;
        if ((id_x0 < 0 || id_x0 >= pm_size.width - 1) || (id_y0 < 0 || id_y0 >= pm_size.height - 1)) {
            xyz[0] = nan;
            xyz[1] = nan;
            xyz[2] = nan;
        } else {
            const double *xyz0 = pm_data + (id_x0 + id_y0 * pm_size.width) * 3, *xyz1 = xyz0 + 3;
            const double *xyz2 = pm_data + (id_x0 + (id_y0 + 1) * pm_size.width) * 3, *xyz3 = xyz2 + 3;
            const double w_x = centers[i].x - id_x0, w_y = centers[i].y - id_y0;
            const double w_xyz0 = (1 - w_x) * (1 - w_y), w_xyz1 = w_x * (1 - w_y), w_xyz2 = (1 - w_x) * w_y,
                         w_xyz3 = w_x * w_y;
            for (size_t i = 0; i < 3; i++) {
                xyz[i] = xyz0[i] * w_xyz0 + xyz1[i] * w_xyz1 + xyz2[i] * w_xyz2 + xyz3[i] * w_xyz3;
            }
        }
    }
}

static void GenerateCalibBoardCenters(const double centerDistance, const int calibBoardSize[2],
                                      std::vector<double> &centers) {
    centers.resize(calibBoardSize[0] * calibBoardSize[1] * 3);
    double *xyz = centers.data();
    for (size_t r = 0; r < calibBoardSize[1]; r++) {
        for (size_t c = 0; c < calibBoardSize[0]; c++, xyz += 3) {
            xyz[1] = c * centerDistance + 0.5 * centerDistance * (r % 2);
            xyz[0] = r * centerDistance * 0.5;
            xyz[2] = 0;
        }
    }
}

static void DotProduct(const double *A, const int ma, const int na, const double *B, const int mb, const int nb,
                       double *AB) {
    if (na != mb) {
        throw std::invalid_argument("The size of Matrix cannot make dot product");
    }
    for (size_t ra = 0; ra < ma; ra++) {
        const double *A_row = A + ra * na, *B_col = B;
        for (size_t cb = 0; cb < nb; cb++, B_col++) {
            double *ab = AB + ra * nb + cb;
            *ab = 0;
            for (size_t rb = 0; rb < mb; rb++) {
                *ab += A_row[rb] * B_col[rb * nb];
            }
        }
    }
}

static bool ComputeRigidTransform(const double *srcPCD, const double *dstPCD, const int nPts, double R[9],
                                  double T[3]) {
    const double *src_xyz = srcPCD, *dst_xyz = dstPCD;
    double src_mean[3] = {0, 0, 0}, dst_mean[3] = {0, 0, 0};
    int count_valid = 0;
    for (size_t i = 0; i < nPts; i++, src_xyz += 3, dst_xyz += 3) {
        if (src_xyz[2] == src_xyz[2] && dst_xyz[2] == dst_xyz[2]) {
            for (size_t j = 0; j < 3; j++) {
                src_mean[j] += src_xyz[j];
                dst_mean[j] += dst_xyz[j];
            }
            count_valid++;
        }
    }
    if (count_valid < 3) {
        return false;
    }
    for (size_t i = 0; i < 3; i++) {
        src_mean[i] /= count_valid;
        dst_mean[i] /= count_valid;
    }
    double H[3][3] = {0};
    src_xyz = srcPCD;
    dst_xyz = dstPCD;
    double temp_src;
    for (size_t i = 0; i < nPts; i++, src_xyz += 3, dst_xyz += 3) {
        if (src_xyz[2] == src_xyz[2] && dst_xyz[2] == dst_xyz[2]) {
            for (size_t r = 0; r < 3; r++) {
                temp_src = src_xyz[r] - src_mean[r];
                for (size_t c = 0; c < 3; c++) {
                    H[r][c] += temp_src * (dst_xyz[c] - dst_mean[c]);
                }
            }
        }
    }
    cv::Mat S, U, VT, V, UT;
    cv::SVDecomp(cv::Mat(3, 3, CV_64F, H[0]), S, U, VT);
    cv::transpose(VT, V);
    cv::transpose(U, UT);
    DotProduct(V.ptr<double>(), V.rows, V.cols, UT.ptr<double>(), UT.rows, UT.cols, R);
    double det_R =
        R[0] * (R[4] * R[8] - R[5] * R[7]) + R[3] * (R[7] * R[2] - R[1] * R[8]) + R[6] * (R[1] * R[5] - R[2] * R[4]);
    if (det_R < 0) {
        double *UT_row = UT.ptr<double>() + (UT.rows - 1) * UT.cols;
        for (size_t c = 0; c < UT.cols; c++) {
            UT_row[c] *= -1;
        }
        DotProduct(V.ptr<double>(), V.rows, V.cols, UT.ptr<double>(), UT.rows, UT.cols, R);
    }
    DotProduct(R, 3, 3, src_mean, 3, 1, T);
    for (size_t i = 0; i < 3; i++) {
        T[i] = dst_mean[i] - T[i];
    }
    return true;
}

bool GetCaliBoardPose(RVC::PointMap &pm, const RVC::Image &image, const double centerDistance,
                      const int calibBoardSize[2], const cv::SimpleBlobDetector::Params &blobDetectorParam, double R[9],
                      double T[3], cv::Mat *debugImage /* = nullptr */) {
    cv::Ptr<cv::SimpleBlobDetector> cv_blob_detector = cv::SimpleBlobDetector::create(blobDetectorParam);
    std::vector<cv::Point2f> centers;
    bool found = DetectCVCircle(image, cv_blob_detector, calibBoardSize, centers, debugImage);
    if (!found) {
        return false;
    }
    std::vector<double> center_in_pcd;
    ExtractCenterFromPCD(pm, centers, center_in_pcd);
    std::vector<double> ref_centers;
    GenerateCalibBoardCenters(centerDistance, calibBoardSize, ref_centers);
    found = ComputeRigidTransform(ref_centers.data(), center_in_pcd.data(), ref_centers.size() / 3, R, T);
    return found;
}

void RigidTransformInv(const double R[9], const double T[3], double invR[9], double invT[3]) {
    for (size_t r = 0; r < 3; r++) {
        for (size_t c = 0; c < 3; c++) {
            invR[c * 3 + r] = R[r * 3 + c];
        }
    }
    const double *R_row = invR;
    for (size_t i = 0; i < 3; i++, R_row += 3) {
        invT[i] = -(R_row[0] * T[0] + R_row[1] * T[1] + R_row[2] * T[2]);
    }
}

void RigidTransformPointMap(RVC::PointMap &pm, const double R[9], const double T[3]) {
    const RVC::Size pm_size = pm.GetSize();
    double *pm_data = pm.GetPointDataPtr();
    const int length = pm_size.width * pm_size.height;
    double xyz[3];
    for (size_t i = 0; i < length; i++, pm_data += 3) {
        DotProduct(R, 3, 3, pm_data, 3, 1, xyz);
        for (size_t j = 0; j < 3; j++) {
            pm_data[j] = xyz[j] + T[j];
        }
    }
}


void PrintHomogeneousMatrix(const double R[9], const double T[3]) {
    cv::Mat homogeneousMatrix = cv::Mat::zeros(4, 4, CV_64F);

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            homogeneousMatrix.at<double>(i, j) = R[i * 3 + j];
        }
    }

    for (int i = 0; i < 3; ++i) {
        homogeneousMatrix.at<double>(i, 3) = T[i];
    }

    homogeneousMatrix.at<double>(3, 3) = 1.0;

    std::cout << "Transformation:" << std::endl;
    for (int i = 0; i < homogeneousMatrix.rows; ++i) {
        for (int j = 0; j < homogeneousMatrix.cols; ++j) {
            double value = homogeneousMatrix.at<double>(i, j);
            std::cout << std::setw(10) << std::fixed << std::setprecision(4) << value;
        }
        std::cout << std::endl;
    }
}