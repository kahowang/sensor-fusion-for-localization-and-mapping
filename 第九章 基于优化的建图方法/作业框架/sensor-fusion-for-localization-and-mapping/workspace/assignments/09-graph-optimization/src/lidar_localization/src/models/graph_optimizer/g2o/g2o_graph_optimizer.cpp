/*
 * @Description: 
 * @Author: Ren Qian
 * @Date: 2020-03-01 18:07:42
 */
 
#include "lidar_localization/models/graph_optimizer/g2o/g2o_graph_optimizer.hpp"
#include "glog/logging.h"
#include "lidar_localization/tools/tic_toc.hpp"

#include <ostream>

namespace lidar_localization {

G2oGraphOptimizer::G2oGraphOptimizer(
    const std::string &solver_type
) {
    graph_ptr_.reset(new g2o::SparseOptimizer());

    g2o::OptimizationAlgorithmFactory *solver_factory = g2o::OptimizationAlgorithmFactory::instance();
    g2o::OptimizationAlgorithmProperty solver_property;
    g2o::OptimizationAlgorithm *solver = solver_factory->construct(solver_type, solver_property);
    graph_ptr_->setAlgorithm(solver);

    if (!graph_ptr_->solver()) {
        LOG(ERROR) << "Failed to create G2O optimizer!" << std::endl;
    }
    robust_kernel_factory_ = g2o::RobustKernelFactory::instance();
}

void G2oGraphOptimizer::SetEdgeRobustKernel(
    std::string robust_kernel_name,
    double robust_kernel_size
) {
    robust_kernel_name_ = robust_kernel_name;
    robust_kernel_size_ = robust_kernel_size;
    need_robust_kernel_ = true;
}

void G2oGraphOptimizer::AddRobustKernel(g2o::OptimizableGraph::Edge *edge, const std::string &kernel_type, double kernel_size) {
    if (kernel_type == "NONE") {
        return;
    }

    g2o::RobustKernel *kernel = robust_kernel_factory_->construct(kernel_type);
    if (kernel == nullptr) {
        std::cerr << "warning : invalid robust kernel type: " << kernel_type << std::endl;
        return;
    }

    kernel->setDelta(kernel_size);
    edge->setRobustKernel(kernel);
}

bool G2oGraphOptimizer::Optimize() {
    static int optimize_cnt = 0;

    if(graph_ptr_->edges().size() < 1) {
        return false;
    }

    TicToc optimize_time;

    graph_ptr_->initializeOptimization();
    graph_ptr_->computeInitialGuess();
    graph_ptr_->computeActiveErrors();
    graph_ptr_->setVerbose(false);

    double chi2 = graph_ptr_->chi2();

    int iterations = graph_ptr_->optimize(max_iterations_num_);

    LOG(INFO) << std::endl 
              << "------ Finish Iteration " << ++optimize_cnt << " of Backend Optimization -------" << std::endl
              << "\tNum. Vertices: " << graph_ptr_->vertices().size() << std::endl 
              << "\tNum. Edges: " << graph_ptr_->edges().size() << std::endl
              << "\tNum. Iterations: " << iterations << "/" << max_iterations_num_ << std::endl
              << "\tTime Consumption: " << optimize_time.toc() << std::endl
              << "\tCost Change: " << chi2 << "--->" << graph_ptr_->chi2()
              << std::endl << std::endl;

    return true;
}

int G2oGraphOptimizer::GetNodeNum() {
    return graph_ptr_->vertices().size();
}

bool G2oGraphOptimizer::GetOptimizedPose(std::deque<Eigen::Matrix4f>& optimized_pose) {
    optimized_pose.clear();
    int vertex_num = graph_ptr_->vertices().size();

    for (int i = 0; i < vertex_num; i++) {
        g2o::VertexSE3* v = dynamic_cast<g2o::VertexSE3*>(graph_ptr_->vertex(i));
        Eigen::Isometry3d pose = v->estimate();
        optimized_pose.push_back(pose.matrix().cast<float>());
    }
    return true;
}

void G2oGraphOptimizer::AddSe3Node(
    const Eigen::Isometry3d &pose, const bool need_fix
) {
    g2o::VertexSE3 *vertex(new g2o::VertexSE3());

    vertex->setId(graph_ptr_->vertices().size());
    vertex->setEstimate(pose);
    if (need_fix) {
        vertex->setFixed(true);
    }

    graph_ptr_->addVertex(vertex);
}

void G2oGraphOptimizer::AddSe3Edge(
    const int vertex_index1, const int vertex_index2,
    const Eigen::Isometry3d &relative_pose, const Eigen::VectorXd &noise
) {
    Eigen::MatrixXd information_matrix = CalculateSe3EdgeInformationMatrix(noise);

    g2o::VertexSE3* v1 = dynamic_cast<g2o::VertexSE3*>(graph_ptr_->vertex(vertex_index1));
    g2o::VertexSE3* v2 = dynamic_cast<g2o::VertexSE3*>(graph_ptr_->vertex(vertex_index2));
    
    g2o::EdgeSE3 *edge(new g2o::EdgeSE3());
    edge->setMeasurement(relative_pose);
    edge->setInformation(information_matrix);
    edge->vertices()[0] = v1;
    edge->vertices()[1] = v2;
    graph_ptr_->addEdge(edge);
    if (need_robust_kernel_) {
        AddRobustKernel(edge, robust_kernel_name_, robust_kernel_size_);
    }
}

void G2oGraphOptimizer::AddSe3PriorXYZEdge(
    const int se3_vertex_index,
    const Eigen::Vector3d &xyz, const Eigen::VectorXd &noise
) {
    Eigen::MatrixXd information_matrix = CalculateDiagMatrix(noise);
    g2o::VertexSE3 *v_se3 = dynamic_cast<g2o::VertexSE3 *>(graph_ptr_->vertex(se3_vertex_index));
    g2o::EdgeSE3PriorXYZ *edge(new g2o::EdgeSE3PriorXYZ());
    edge->setMeasurement(xyz);
    edge->setInformation(information_matrix);
    edge->vertices()[0] = v_se3;
    graph_ptr_->addEdge(edge);
}

void G2oGraphOptimizer::AddSe3PriorQuaternionEdge(
    const int se3_vertex_index,
    const Eigen::Quaterniond &quat, const Eigen::VectorXd &noise
) {
    Eigen::MatrixXd information_matrix = CalculateSe3PriorQuaternionEdgeInformationMatrix(noise);
    g2o::VertexSE3 *v_se3 = dynamic_cast<g2o::VertexSE3 *>(graph_ptr_->vertex(se3_vertex_index));
    g2o::EdgeSE3PriorQuat *edge(new g2o::EdgeSE3PriorQuat());
    edge->setMeasurement(quat);
    edge->setInformation(information_matrix);
    edge->vertices()[0] = v_se3;
    graph_ptr_->addEdge(edge);
}

/**
 * @brief  get optimized LIO key frame state estimation
 * @param  optimized_key_frames, output optimized LIO key frames
 * @return true if success false otherwise
 */
bool G2oGraphOptimizer::GetOptimizedKeyFrame(std::deque<KeyFrame> &optimized_key_frames) {
    optimized_key_frames.clear();

    const int N = graph_ptr_->vertices().size();

    for (int vertex_id = 0; vertex_id < N; vertex_id++) {
        g2o::VertexPRVAG* v = dynamic_cast<g2o::VertexPRVAG*>(graph_ptr_->vertex(vertex_id));

        const g2o::PRVAG &key_frame_state = v->estimate();

        optimized_key_frames.emplace_back( vertex_id, key_frame_state );
    }

    return true;
}

/**
 * @brief  add vertex for LIO key frame
 * @param  lio_key_frame, LIO key frame with (pos, ori, vel, b_a and b_g)
 * @param  need_fix, shall the vertex be fixed to eliminate trajectory estimation ambiguity
 * @return void
 */
void G2oGraphOptimizer::AddPRVAGNode(
    const KeyFrame &lio_key_frame, const bool need_fix
) {
    // init:
    g2o::VertexPRVAG *vertex(new g2o::VertexPRVAG());

    // a. set vertex ID:
    vertex->setId(graph_ptr_->vertices().size());
    // b. set vertex state:
    g2o::PRVAG measurement;

    measurement.time = lio_key_frame.time;

    measurement.pos = lio_key_frame.pose.block<3, 1>(0, 3).cast<double>();
    measurement.ori = Sophus::SO3d(
        Eigen::Quaterniond(lio_key_frame.pose.block<3, 3>(0, 0).cast<double>())
    );
    // transform linear velocity from body frame to navigation frame:
    measurement.vel = measurement.ori * lio_key_frame.vel.v.cast<double>();
    measurement.b_a = lio_key_frame.bias.accel.cast<double>();
    measurement.b_g = lio_key_frame.bias.gyro.cast<double>();
    
    vertex->setEstimate(measurement);

    // for first vertex:
    if (need_fix) {
        vertex->setFixed(true);
    }

    // add vertex:
    graph_ptr_->addVertex(vertex);
}

/**
 * @brief  add edge for relative pose constraint from lidar frontend / loop closure detection
 * @param  vertex_index_i, vertex ID of previous key frame
 * @param  vertex_index_j, vertex ID of current key frame
 * @param  relative_pose, relative pose measurement
 * @param  noise, relative pose measurement noise
 * @return void
 */
void G2oGraphOptimizer::AddPRVAGRelativePoseEdge(
    const int vertex_index_i, const int vertex_index_j,
    const Eigen::Matrix4d &relative_pose, const Eigen::VectorXd &noise
) {
    // init:
    g2o::EdgePRVAGRelativePose *edge(new g2o::EdgePRVAGRelativePose());

    // a. set nodes:
    g2o::VertexPRVAG* vertex_i = dynamic_cast<g2o::VertexPRVAG*>(graph_ptr_->vertex(vertex_index_i));
    g2o::VertexPRVAG* vertex_j = dynamic_cast<g2o::VertexPRVAG*>(graph_ptr_->vertex(vertex_index_j));
    edge->vertices()[0] = vertex_i;
    edge->vertices()[1] = vertex_j;

    // b. set measurement
    Vector6d measurement = Vector6d::Zero();
    // b.1. position:
    measurement.block<3, 1>(g2o::EdgePRVAGRelativePose::INDEX_P, 0) = relative_pose.block<3, 1>(0, 3);
    // b.2. orientation, so3:
    measurement.block<3, 1>(g2o::EdgePRVAGRelativePose::INDEX_R, 0) = Sophus::SO3d(
        Eigen::Quaterniond(relative_pose.block<3, 3>(0, 0).cast<double>())
    ).log();
    edge->setMeasurement(measurement);

    // c. set information matrix:
    Eigen::MatrixXd information_matrix = CalculateSe3EdgeInformationMatrix(noise);
    edge->setInformation(information_matrix);

    // d. set loss function:
    if (need_robust_kernel_) {
        AddRobustKernel(edge, robust_kernel_name_, robust_kernel_size_);
    }

    // add edge:
    graph_ptr_->addEdge(edge);
}

/**
 * @brief  add edge for prior position constraint from GNSS measurement
 * @param  vertex_index, vertex ID of current key frame
 * @param  pos, prior position measurement
 * @param  noise, prior position measurement noise
 * @return void
 */
void G2oGraphOptimizer::AddPRVAGPriorPosEdge(
    const int vertex_index,
    const Eigen::Vector3d &pos, const Eigen::Vector3d &noise
) {
    // init:
    g2o::EdgePRVAGPriorPos *edge(new g2o::EdgePRVAGPriorPos());

    // a. set node:
    g2o::VertexPRVAG *vertex = dynamic_cast<g2o::VertexPRVAG *>(graph_ptr_->vertex(vertex_index));
    edge->vertices()[0] = vertex;

    // b. set measurement:
    edge->setMeasurement(pos);

    // c. set information matrix:
    Eigen::MatrixXd information_matrix = CalculateDiagMatrix(noise);
    edge->setInformation(information_matrix);
    
    // d. set loss function:
    if (need_robust_kernel_) {
        AddRobustKernel(edge, robust_kernel_name_, robust_kernel_size_);
    }

    // add edge:
    graph_ptr_->addEdge(edge);
}

/**
 * @brief  add edge for IMU pre-integration constraint from IMU measurement
 * @param  vertex_index_i, vertex ID of previous key frame
 * @param  vertex_index_j, vertex ID of current key frame
 * @param  imu_pre_integration, IMU pre-integration measurement
 * @return void
 */
void G2oGraphOptimizer::AddPRVAGIMUPreIntegrationEdge(
    const int vertex_index_i, const int vertex_index_j,
    const IMUPreIntegrator::IMUPreIntegration &imu_pre_integration
) {
    // init:
    g2o::EdgePRVAGIMUPreIntegration *edge(new g2o::EdgePRVAGIMUPreIntegration());

    // a. set nodes:
    g2o::VertexPRVAG* vertex_i = dynamic_cast<g2o::VertexPRVAG*>(graph_ptr_->vertex(vertex_index_i));
    g2o::VertexPRVAG* vertex_j = dynamic_cast<g2o::VertexPRVAG*>(graph_ptr_->vertex(vertex_index_j));
    edge->vertices()[0] = vertex_i;
    edge->vertices()[1] = vertex_j;

    // b. set measurement
    edge->setT(
        imu_pre_integration.GetT()
    );
    edge->setGravitiy(
        imu_pre_integration.GetGravity()
    );
    edge->setJacobian(
        imu_pre_integration.GetJacobian()
    );
    edge->setMeasurement(
        imu_pre_integration.GetMeasurement()
    );

    // c. set information matrix:
    edge->setInformation(
        imu_pre_integration.GetInformation()
    );

    // d. set loss function:
    if (need_robust_kernel_) {
        AddRobustKernel(edge, robust_kernel_name_, robust_kernel_size_);
    }

    // add edge:
    graph_ptr_->addEdge(edge);
}

/**
 * @brief  add edge for odometer pre-integration constraint from odometer measurement
 * @param  vertex_index_i, vertex ID of previous key frame
 * @param  vertex_index_j, vertex ID of current key frame
 * @param  odo_pre_integration, odometer pre-integration measurement
 * @return void
 */
void G2oGraphOptimizer::AddPRVAGOdoPreIntegrationEdge(
    const int vertex_index_i, const int vertex_index_j,
    const OdoPreIntegrator::OdoPreIntegration &odo_pre_integration
) {
    // init:
    g2o::EdgePRVAGOdoPreIntegration::EdgePRVAGOdoPreIntegration *edge(
        new g2o::EdgePRVAGOdoPreIntegration::EdgePRVAGOdoPreIntegration()
    );

    // a. set nodes:
    g2o::VertexPRVAG* vertex_i = dynamic_cast<g2o::VertexPRVAG*>(graph_ptr_->vertex(vertex_index_i));
    g2o::VertexPRVAG* vertex_j = dynamic_cast<g2o::VertexPRVAG*>(graph_ptr_->vertex(vertex_index_j));
    edge->vertices()[0] = vertex_i;
    edge->vertices()[1] = vertex_j;

    // b. set measurement
    edge->setMeasurement(
        odo_pre_integration.GetMeasurement()
    );

    // c. set information matrix:
    edge->setInformation(
        odo_pre_integration.GetInformation()
    );

    // d. set loss function:
    if (need_robust_kernel_) {
        AddRobustKernel(edge, robust_kernel_name_, robust_kernel_size_);
    }

    // add edge:
    graph_ptr_->addEdge(edge);    
}

void G2oGraphOptimizer::ShowIMUPreIntegrationResidual(
    const int vertex_index_i, const int vertex_index_j,
    const IMUPreIntegrator::IMUPreIntegration &imu_pre_integration
) {
    const g2o::VertexPRVAG* vertex_i = dynamic_cast<g2o::VertexPRVAG*>(graph_ptr_->vertex(vertex_index_i));
    const g2o::VertexPRVAG* vertex_j = dynamic_cast<g2o::VertexPRVAG*>(graph_ptr_->vertex(vertex_index_j));

    const Eigen::Vector3d &pos_i = vertex_i->estimate().pos;
    const Sophus::SO3d    &ori_i = vertex_i->estimate().ori;
    const Eigen::Vector3d &vel_i = vertex_i->estimate().vel;

    const Eigen::Vector3d &pos_j = vertex_j->estimate().pos;
    const Sophus::SO3d    &ori_j = vertex_j->estimate().ori;
    const Eigen::Vector3d &vel_j = vertex_j->estimate().vel;

    const double &T = imu_pre_integration.T_;
    const Eigen::Vector3d &g = imu_pre_integration.g_;

    Eigen::Vector3d r_p = ori_i.inverse() * (pos_j - pos_i - (vel_i - 0.50 * g * T) * T) - imu_pre_integration.alpha_ij_;
    Eigen::Vector3d r_q = (imu_pre_integration.theta_ij_.inverse()*ori_i.inverse()*ori_j).log();
    Eigen::Vector3d r_v = ori_i.inverse() * (vel_j - vel_i + g * T) - imu_pre_integration.beta_ij_;

    LOG(INFO) << "IMU Pre-Integration Measurement: " << std::endl
                << "\tT: " << T << std::endl
                << "\talpha:" << std::endl
                << "\t\t" << r_p.x() << ", " << r_p.y() << ", " << r_p.z() << std::endl
                << "\ttheta:" << std::endl
                << "\t\t" << r_q.x() << ", " << r_q.y() << ", " << r_q.z() << std::endl
                << "\tbeta:" << std::endl
                << "\t\t" << r_v.x() << ", " << r_v.y() << ", " << r_v.z() << std::endl
                << "\tbias_accel:" 
                << imu_pre_integration.b_a_i_.x() << ", "
                << imu_pre_integration.b_a_i_.y() << ", " 
                << imu_pre_integration.b_a_i_.z()
                << std::endl
                << "\tbias_gyro:" 
                << imu_pre_integration.b_g_i_.x() << ", "
                << imu_pre_integration.b_g_i_.y() << ", " 
                << imu_pre_integration.b_g_i_.z()
                << std::endl
                << "\tcovariance:" << std::endl
                << "\t\talpha: "
                << imu_pre_integration.P_( 0,  0) << ", "
                << imu_pre_integration.P_( 1,  1) << ", " 
                << imu_pre_integration.P_( 2,  3)
                << std::endl
                << "\t\ttheta: "
                << imu_pre_integration.P_( 3,  3) << ", "
                << imu_pre_integration.P_( 4,  4) << ", " 
                << imu_pre_integration.P_( 5,  5)
                << std::endl
                << "\t\tbeta: "
                << imu_pre_integration.P_( 6,  6) << ", "
                << imu_pre_integration.P_( 7,  7) << ", " 
                << imu_pre_integration.P_( 8,  8)
                << std::endl
                << "\t\tbias_accel: "
                << imu_pre_integration.P_( 9,  9) << ", "
                << imu_pre_integration.P_(10, 10) << ", " 
                << imu_pre_integration.P_(11, 11)
                << std::endl
                << "\t\tbias_gyro: "
                << imu_pre_integration.P_(12, 12) << ", "
                << imu_pre_integration.P_(13, 13) << ", " 
                << imu_pre_integration.P_(14, 14)
                << std::endl
                /*
                << "\tJacobian:" << std::endl
                << "\t\td_alpha_d_b_a: " << std::endl
                << "\t\t\t" << imu_pre_integration.J_( 0,  9) << ", " << imu_pre_integration.J_( 0, 10) << ", " << imu_pre_integration.J_( 0, 11) << std::endl
                << "\t\t\t" << imu_pre_integration.J_( 1,  9) << ", " << imu_pre_integration.J_( 1, 10) << ", " << imu_pre_integration.J_( 1, 11) << std::endl
                << "\t\t\t" << imu_pre_integration.J_( 2,  9) << ", " << imu_pre_integration.J_( 2, 10) << ", " << imu_pre_integration.J_( 2, 11) << std::endl
                << "\t\td_alpha_d_b_g: " << std::endl
                << "\t\t\t" << imu_pre_integration.J_( 0, 12) << ", " << imu_pre_integration.J_( 0, 13) << ", " << imu_pre_integration.J_( 0, 14) << std::endl
                << "\t\t\t" << imu_pre_integration.J_( 1, 12) << ", " << imu_pre_integration.J_( 1, 13) << ", " << imu_pre_integration.J_( 1, 14) << std::endl
                << "\t\t\t" << imu_pre_integration.J_( 2, 12) << ", " << imu_pre_integration.J_( 2, 13) << ", " << imu_pre_integration.J_( 2, 14) << std::endl
                << "\t\td_theta_d_b_g: " << std::endl
                << "\t\t\t" << imu_pre_integration.J_( 3, 12) << ", " << imu_pre_integration.J_( 3, 13) << ", " << imu_pre_integration.J_( 3, 14) << std::endl
                << "\t\t\t" << imu_pre_integration.J_( 4, 12) << ", " << imu_pre_integration.J_( 4, 13) << ", " << imu_pre_integration.J_( 4, 14) << std::endl
                << "\t\t\t" << imu_pre_integration.J_( 5, 12) << ", " << imu_pre_integration.J_( 5, 13) << ", " << imu_pre_integration.J_( 5, 14) << std::endl
                << "\t\td_beta_d_b_a: " << std::endl
                << "\t\t\t" << imu_pre_integration.J_( 6,  9) << ", " << imu_pre_integration.J_( 6, 10) << ", " << imu_pre_integration.J_( 6, 11) << std::endl
                << "\t\t\t" << imu_pre_integration.J_( 7,  9) << ", " << imu_pre_integration.J_( 7, 10) << ", " << imu_pre_integration.J_( 7, 11) << std::endl
                << "\t\t\t" << imu_pre_integration.J_( 8,  9) << ", " << imu_pre_integration.J_( 8, 10) << ", " << imu_pre_integration.J_( 8, 11) << std::endl
                << "\t\td_beta_d_b_a: " << std::endl
                << "\t\t\t" << imu_pre_integration.J_( 6, 12) << ", " << imu_pre_integration.J_( 6, 13) << ", " << imu_pre_integration.J_( 6, 14) << std::endl
                << "\t\t\t" << imu_pre_integration.J_( 7, 12) << ", " << imu_pre_integration.J_( 7, 13) << ", " << imu_pre_integration.J_( 7, 14) << std::endl
                << "\t\t\t" << imu_pre_integration.J_( 8, 12) << ", " << imu_pre_integration.J_( 8, 13) << ", " << imu_pre_integration.J_( 8, 14) << std::endl
                */
                << std::endl;
}

Eigen::MatrixXd G2oGraphOptimizer::CalculateDiagMatrix(Eigen::VectorXd noise) {
    Eigen::MatrixXd information_matrix = Eigen::MatrixXd::Identity(noise.rows(), noise.rows());
    for (int i = 0; i < noise.rows(); i++) {
        information_matrix(i, i) /= noise(i);
    }
    return information_matrix;
}

Eigen::MatrixXd G2oGraphOptimizer::CalculateSe3EdgeInformationMatrix(Eigen::VectorXd noise) {
    Eigen::MatrixXd information_matrix = Eigen::MatrixXd::Identity(6, 6);
    information_matrix = CalculateDiagMatrix(noise);
    return information_matrix;
}

// TODO: 姿态观测的信息矩阵尚未添加
// 备注：各位使用时可只用位置观测，而不用姿态观测，影响不大
// 我自己在别的地方尝试过增加姿态观测，但效果反而变差，如果感兴趣，可自己编写此处的信息矩阵，并在后端优化中添加相应的边进行验证
Eigen::MatrixXd G2oGraphOptimizer::CalculateSe3PriorQuaternionEdgeInformationMatrix(Eigen::VectorXd noise) {
    Eigen::MatrixXd information_matrix;

    return information_matrix;
}


} // namespace graph_ptr_optimization
