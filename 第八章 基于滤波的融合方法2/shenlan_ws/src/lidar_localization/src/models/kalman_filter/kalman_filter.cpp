/*
 * @Description: Kalman Filter utilities.
 * @Author: Ge Yao
 * @Date: 2020-11-12 15:14:07
 */

#include "lidar_localization/models/kalman_filter/kalman_filter.hpp"

// SVD for observability analysis:
#include <string>
#include <vector>

#include <Eigen/SVD>

#include "lidar_localization/tools/CSVWriter.hpp"

#include "lidar_localization/global_defination/global_defination.h"

#include "glog/logging.h"

namespace lidar_localization {

void KalmanFilter::AnalyzeQ(
    const int DIM_STATE,
    const double &time, const Eigen::MatrixXd &Q, const Eigen::VectorXd &Y,
    std::vector<std::vector<double>> &data
) {
    // perform SVD analysis, thin for rectangular matrix:
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(Q, Eigen::ComputeThinU | Eigen::ComputeThinV);

    // add record:
    std::vector<double> record;

    // a. record timestamp:
    record.push_back(time);

    // b. record singular values:
    for (int i = 0; i < DIM_STATE; ++i) {
        record.push_back(svd.singularValues()(i, 0));
    }

    // c. record degree of observability:
    Eigen::MatrixXd X = (
        svd.matrixV() * 
        svd.singularValues().asDiagonal().inverse()
    ) * (
        svd.matrixU().transpose() * Y
    ).asDiagonal();

    Eigen::VectorXd degree_of_observability = Eigen::VectorXd::Zero(DIM_STATE);
    for (int i = 0; i < DIM_STATE; ++i) {
        // find max. magnitude response in X:
        Eigen::MatrixXd::Index sv_index;
        X.col(i).cwiseAbs().maxCoeff(&sv_index);

        // associate with corresponding singular value:
        degree_of_observability(sv_index) = svd.singularValues()(i);
    }
    // normalize:
    degree_of_observability = 1.0 / svd.singularValues().maxCoeff() * degree_of_observability;

    for (int i = 0; i < DIM_STATE; ++i) {
        record.push_back(degree_of_observability(i));
    }

    // add to data:
    data.push_back(record);
}

void KalmanFilter::WriteAsCSV(
    const int DIM_STATE,
    const std::vector<std::vector<double>> &data,
    const std::string filename
) {
    // init:
    CSVWriter csv(",");
    csv.enableAutoNewRow(1 + 2*DIM_STATE);

    // a. write header:
    csv << "T";
    for (int i = 0; i < DIM_STATE; ++i) {
        csv << ("sv" + std::to_string(i + 1)); 
    }
    for (int i = 0; i < DIM_STATE; ++i) {
        csv << ("doo" + std::to_string(i + 1)); 
    }

    // b. write contents:
    for (const auto &record: data) {
        // cast timestamp to int:
        csv << static_cast<int>(record.at(0));

        for (size_t i = 1; i < record.size(); ++i) {
            csv << std::fabs(record.at(i));
        }    
    }

    // save to persistent storage:
    csv.writeToFile(filename);
}

} // namespace lidar_localization