/*
 * @Description: loop closure detection using scan context
 * @Author: Ge Yao
 * @Date: 2020-10-28 15:43:03
 */
#include <limits>

#include <math.h>
#include <ctime>
#include <cmath>
#include <cstdlib>
#include <memory>
#include <iostream>
#include <fstream>
#include <ostream>

#include "lidar_localization/models/scan_context_manager/scan_context_manager.hpp"

#include "lidar_localization/models/scan_context_manager/scan_contexts.pb.h"
#include "lidar_localization/models/scan_context_manager/ring_keys.pb.h"
#include "lidar_localization/models/scan_context_manager/key_frames.pb.h"

#include "glog/logging.h"

namespace lidar_localization {

ScanContextManager::ScanContextManager(const YAML::Node& node) {
    //
    // parse config:
    // 
    // a. ROI definition:
    MAX_RADIUS_ = node["max_radius"].as<float>();
    MAX_THETA_ = node["max_theta"].as<float>();
    // b. resolution:
    NUM_RINGS_ = node["num_rings"].as<int>();
    NUM_SECTORS_ = node["num_sectors"].as<int>();
    DEG_PER_SECTOR_ = MAX_THETA_ / NUM_SECTORS_;
    // c. ring key indexing interval:
    INDEXING_INTERVAL_ = node["indexing_interval"].as<int>();
    // d. min. key frame sequence distance:
    MIN_KEY_FRAME_SEQ_DISTANCE_ = node["min_key_frame_seq_distance"].as<int>();
    // e. num. of nearest-neighbor candidates to check:
    NUM_CANDIDATES_ = node["num_candidates"].as<int>();
    // f. sector key fast alignment search ratio:
    FAST_ALIGNMENT_SEARCH_RATIO_ = node["fast_alignment_search_ratio"].as<float>();
    // g. scan context distance threshold:
    SCAN_CONTEXT_DISTANCE_THRESH_ = node["scan_context_distance_thresh"].as<float>();

    // prompt:
    LOG(INFO) << "Scan Context params:" << std::endl
              << "\tmax. radius: " << MAX_RADIUS_ << std::endl
              << "\tmax. theta: " << MAX_THETA_ << std::endl
              << "\tnum. rings: " << NUM_RINGS_ << std::endl
              << "\tnum. sectors: " << NUM_SECTORS_  << std::endl
              << "\tre-indexing interval: " << INDEXING_INTERVAL_ << std::endl
              << "\tmin. key frame sequence distance: " << MIN_KEY_FRAME_SEQ_DISTANCE_ << std::endl
              << "\tnearest-neighbor candidates to check: " << NUM_CANDIDATES_ << std::endl
              << "\tfast alignment search ratio: " << FAST_ALIGNMENT_SEARCH_RATIO_ << std::endl
              << "\tloop-closure scan context distance thresh: " << SCAN_CONTEXT_DISTANCE_THRESH_ << std::endl
              << std::endl;
    
    // reset state:
    state_.scan_context_.clear();
    state_.ring_key_.clear();

    state_.index_.counter_ = 0;

    state_.index_.kd_tree_.reset();
    state_.index_.data_.ring_key_.clear();
    state_.index_.data_.key_frame_.clear();
}

void ScanContextManager::Update(
    const CloudData &scan,
    const KeyFrame &key_frame
) {
    // extract scan context and corresponding ring key:
    ScanContext scan_context = GetScanContext(scan);
    RingKey ring_key = GetRingKey(scan_context);

    // update buffer:
    state_.scan_context_.push_back(scan_context);
    state_.ring_key_.push_back(ring_key);
    state_.key_frame_.push_back(key_frame);
}

/**
 * @brief  detect loop closure for the latest key scan
 * @param  void
 * @return loop closure propsal as std::pair<int, float>
 */
std::pair<int, float> ScanContextManager::DetectLoopClosure(void) {
    // use latest key scan for query:
    const ScanContext &query_scan_context = state_.scan_context_.back();
    const RingKey &query_ring_key = state_.ring_key_.back();

    // update ring key index:
    ++state_.index_.counter_;
    if (
        0 == (state_.index_.counter_ % INDEXING_INTERVAL_)
    ) {
        UpdateIndex(MIN_KEY_FRAME_SEQ_DISTANCE_);
    }

    return GetLoopClosureMatch(query_scan_context, query_ring_key);
}

/**
 * @brief  get loop closure proposal using the given key scan
 * @param  scan, query key scan
 * @param  pose, matched pose
 * @return true for success match otherwise false
 */
bool ScanContextManager::DetectLoopClosure(
    const CloudData &scan,
    Eigen::Matrix4f &pose
) {
    // extract scan context and corresponding ring key:
    ScanContext query_scan_context = GetScanContext(scan);
    RingKey query_ring_key = GetRingKey(query_scan_context);

    // get proposal:
    std::pair<int, float> proposal = GetLoopClosureMatch(
        query_scan_context, query_ring_key
    );

    const int key_frame_id = proposal.first;
    const float yaw_change_in_rad = proposal.second;

    // check proposal validity:
    if (ScanContextManager::NONE == key_frame_id) {
        return false;
    }

    // set matched pose:
    pose = state_.index_.data_.key_frame_.at(key_frame_id).pose;
    // apply orientation change estimation:
    Eigen::AngleAxisf orientation_change(yaw_change_in_rad, Eigen::Vector3f::UnitZ());
    pose.block<3, 3>(0, 0) = pose.block<3, 3>(0, 0) * orientation_change.toRotationMatrix();

    // finally:
    return true;
}

/**
 * @brief  save scan context index & data to persistent storage
 * @param  output_path, scan context output path
 * @return true for success otherwise false
 */
bool ScanContextManager::Save(const std::string &output_path) {
     // get the latest scan context index:
    if (
        UpdateIndex(0)
    ) {
        // prompt:
        LOG(INFO) << std::endl
                  << "[Scan Context]: Index size " << state_.index_.kd_tree_->kdtree_get_point_count()
                  << std::endl;

        // a. save index:
        std::string index_output_path = output_path + "/index.bin";
        if (
            !SaveIndex(index_output_path)
        ) {
            LOG(ERROR) << "[Scan Context]: Failed to write index." << std::endl;
            return false;
        } else {
            LOG(INFO) << "\tSave index to: " << index_output_path << std::endl;
        }

        // b. save ring key data:

        // Verify that the version of the library that we linked against is
        // compatible with the version of the headers we compiled against.
        GOOGLE_PROTOBUF_VERIFY_VERSION;

        // 1. scan contexts:
        std::string scan_contexts_output_path = output_path + "/scan_contexts.proto";
        if (
            !SaveScanContexts(scan_contexts_output_path)
        ) {
            LOG(ERROR) << "[Scan Context]: Failed to write scan contexts." << std::endl;
            return false;
        } else {
            LOG(INFO) << "\tSave scan context of size " << state_.scan_context_.size()
                      << " to: " << scan_contexts_output_path
                      << std::endl;
        }

        // 2. ring keys:
        std::string ring_keys_output_path = output_path + "/ring_keys.proto";
        if (
            !SaveRingKeys(ring_keys_output_path)
        ) {
            LOG(ERROR) << "[Scan Context]: Failed to write ring keys." << std::endl;
            return false;
        } else {
            LOG(INFO) << "\tSave ring keys of size " << state_.index_.data_.ring_key_.size()
                      << " to: " << ring_keys_output_path
                      << std::endl;
        }

        // 3. key frames:
        std::string key_frames_output_path = output_path + "/key_frames.proto";
        if (
            !SaveKeyFrames(key_frames_output_path)
        ) {
            LOG(ERROR) << "[Scan Context]: Failed to write key frames." << std::endl;
            return false;
        } else {
            LOG(INFO) << "\tSave key frames of size " << state_.index_.data_.key_frame_.size()
                      << " to: " << key_frames_output_path
                      << std::endl;
        }

        google::protobuf::ShutdownProtobufLibrary();
    } else {
        LOG(ERROR) << std::endl
                    << "[Scan Context]: Skip empty index"
                    << std::endl << std::endl;
    }

    return true;
}

/**
 * @brief  load scan context index & data from persistent storage
 * @param  input_path, scan context input path
 * @return true for success otherwise false
 */
bool ScanContextManager::Load(const std::string &input_path) {
    // a. load ring key data:

    // Verify that the version of the library that we linked against is
    // compatible with the version of the headers we compiled against.
    GOOGLE_PROTOBUF_VERIFY_VERSION;

    LOG(INFO) << std::endl
              << "[Scan Context]: Load Scan Context Index."
              << std::endl;

    // 1. scan contexts:
    std::string scan_contexts_input_path = input_path + "/scan_contexts.proto";;
    if (
        !LoadScanContexts(scan_contexts_input_path)
    ) {
        LOG(ERROR) << "[Scan Context]: Failed to load scan contexts." << std::endl;
        return false;
    } else {
        LOG(INFO) << "\tNum. Scan Contexts: " << state_.scan_context_.size() << std::endl;
    }

    // 2. ring keys:
    std::string ring_keys_input_path = input_path + "/ring_keys.proto";
    if (
        !LoadRingKeys(ring_keys_input_path)
    ) {
        LOG(ERROR) << "[Scan Context]: Failed to load ring keys." << std::endl;
        return false;
    } else {
        LOG(INFO) << "\tNum. Ring Keys: " << state_.ring_key_.size() << std::endl;
    }

    // 3. key frames:
    std::string key_frames_input_path = input_path + "/key_frames.proto";
    if (
        !LoadKeyFrames(key_frames_input_path)
    ) {
        LOG(ERROR) << "[Scan Context]: Failed to load key frames." << std::endl;
        return false;
    } else {
        LOG(INFO) << "\tNum. Key Frames: " << state_.index_.data_.key_frame_.size() << std::endl;
    }
    
    // b. load scan context index:
    state_.index_.kd_tree_.reset(); 
    state_.index_.kd_tree_ = std::make_shared<RingKeyIndex>(
        NUM_RINGS_,  /* dim */
        state_.index_.data_.ring_key_,
        10           /* max leaf size */
    );

    LOG(INFO) << "\tIndex Size: " << state_.index_.kd_tree_->kdtree_get_point_count() 
              << std::endl;

    google::protobuf::ShutdownProtobufLibrary();

    return true;
}

/**
 * @brief  get scan context of given lidar scan
 * @param  scan, lidar scan of key frame
 * @return scan context as Eigen::MatrixXd
 */
ScanContextManager::ScanContext ScanContextManager::GetScanContext(const CloudData &scan) {
    // num. of point measurements in current scan:
    const size_t N = scan.cloud_ptr->points.size();
    
    // init scan context:
    const float UNKNOWN_HEIGHT = -1000.0f;
    ScanContext scan_context = UNKNOWN_HEIGHT * ScanContext::Ones(NUM_RINGS_, NUM_SECTORS_);

    // iterate through point measurements and create scan context:
    float x, y, z;
    float radius, theta;
    for (size_t i = 0; i < N; ++i) {
        // parse point measurement:
        x = scan.cloud_ptr->points.at(i).x;
        y = scan.cloud_ptr->points.at(i).y;
        z = scan.cloud_ptr->points.at(i).z + 2.0f;

        radius = hypot(x, y);
        theta = GetOrientation(x, y);

        // ROI check:
        if (radius > MAX_RADIUS_) {
            continue;
        }
        
        // get ring-sector index:
        int rid = GetIndex(radius, MAX_RADIUS_, NUM_RINGS_); 
        int sid = GetIndex(theta, MAX_THETA_, NUM_SECTORS_); 

        // update bin height:
        if (scan_context(rid, sid) < z) {
            scan_context(rid, sid) = z;
        }
    }

    // reset unknown height to 0.0 for later cosine distance calculation:
    for (int rid = 0; rid < scan_context.rows(); ++rid) {
        for (int sid = 0; sid < scan_context.cols(); ++sid) {
            if (UNKNOWN_HEIGHT == scan_context(rid, sid)) {
                scan_context(rid, sid) = 0.0;
            }
        }
    }

    return scan_context;
}

/**
 * @brief  get ring key of given scan context
 * @param  scan_context, scan context of key scan
 * @return ring key as RingKey
 */
ScanContextManager::RingKey ScanContextManager::GetRingKey(
    const ScanContext &scan_context
) {
    RingKey ring_key(scan_context.rows());

    for (int rid = 0; rid < scan_context.rows(); ++rid) {
        ring_key.at(rid) = scan_context.row(rid).mean();
    }

    return ring_key;
}

/**
 * @brief  generate random ring keys for indexing test
 * @return void
 */
void ScanContextManager::GenerateRandomRingKey(
    ScanContextManager::RingKeys &samples, 
    const int N, const int D, const float max_range
) {
	samples.clear();
    samples.resize(N);
	for (int i = 0; i < N; ++i)
	{
		samples.at(i).resize(D);
		for (int d = 0; d < D; ++d)
			samples.at(i).at(d) = max_range * (rand() % 1000) / (1000.0);
	}
}

/**
 * @brief  get sector key of given scan context
 * @param  scan_context, scan context of key scan
 * @return sector key as RingKey
 */
Eigen::MatrixXf ScanContextManager::GetSectorKey(
    const Eigen::MatrixXf &scan_context
) {
    Eigen::MatrixXf sector_key(1, scan_context.cols());

    for (int sid = 0; sid < scan_context.cols(); ++sid)
    {
        sector_key(0, sid) = scan_context.col(sid).mean();
    }

    return sector_key;
}

/**
 * @brief  get orientation of point measurement 
 * @param  x, x component of point measurement
 * @param  y, y component of point measurement
 * @return point measurement orientation, [0.0f, 360.0f)
 */
float ScanContextManager::GetOrientation(
    const float &x, 
    const float &y
) {
    float theta = 180.0f / M_PI * atan2(y, x);

    // make sure the orientation is consistent with scan context convension:
    if (theta < 0.0f) {
        theta += 360.0f;
    }

    return theta;
}

/**
 * @brief  convert floating point value to integer index 
 * @param  value, target floating point value 
 * @param  MAX_VALUE, max. floating point value
 * @param  RESOLUTION, resolution
 * @return integer index, {0, ..., RESOLUTION - 1}
 */
int ScanContextManager::GetIndex(
    const float &value, 
    const float &MAX_VALUE, 
    const int RESOLUTION
) {
    int index = std::floor(static_cast<int>(RESOLUTION*value/MAX_VALUE));

    // this ensures value at MAX_VALUE will be cast into last bin:
    index = std::min(index, RESOLUTION - 1);

    return index;
} 

/**
 * @brief  get candidate scan context indices 
 * @param  ring_key, query ring key 
 * @param  N, num. of nearest neighbor candidates
 * @param  indices, candidate indices
 * @param  distances, candidate distances
 * @return void
 */
void ScanContextManager::GetCandidateIndices(
    const RingKey &ring_key, const int N,
    std::vector<size_t> &indices,
	std::vector<float> &distances
) {    
    state_.index_.kd_tree_->query(
        &ring_key.at(0),
        N,
        &indices.at(0),
        &distances.at(0)
    );
}

/**
 * @brief  circular shift mat to right by shift
 * @param  mat, original matrix 
 * @param  shift, right shift amount  
 * @return shifted matrix
 */
Eigen::MatrixXf ScanContextManager::CircularShift(
    const Eigen::MatrixXf &mat, 
    int shift 
) {
    if(0 == shift)
    {
        Eigen::MatrixXf shifted_mat(mat);
        return shifted_mat; // Early return 
    }

    Eigen::MatrixXf shifted_mat = Eigen::MatrixXf::Zero( 
        mat.rows(), 
        mat.cols() 
    );
    for (int i = 0; i < mat.cols(); ++i) {
        int shifted_i = (i + shift) % mat.cols();
        shifted_mat.col(shifted_i) = mat.col(i);
    }

    return shifted_mat;
}

/**
 * @brief  get optimal shift estimation using sector key 
 * @param  target, target sector key 
 * @param  source, source sector key  
 * @return void
 */
int ScanContextManager::GetOptimalShiftUsingSectorKey(
    const Eigen::MatrixXf &target, 
    const Eigen::MatrixXf &source
) {
    int optimal_shift = 0;
    float optimal_dist = std::numeric_limits<float>::max();

    for (int curr_shift = 0; curr_shift < target.cols(); ++curr_shift) {
        Eigen::MatrixXf curr_target = CircularShift(
            target, curr_shift
        );

        float curr_dist = (curr_target - source).norm();

        if(curr_dist < optimal_dist)
        {
            optimal_dist = curr_dist;
            optimal_shift = curr_shift;
        }
    }

    return optimal_shift;
} 

/**
 * @brief  compute cosine distance between target and source scan context 
 * @param  target_scan_context, target scan context 
 * @param  source_scan_context, source scan context 
 * @return scan context cosine distance
 */
float ScanContextManager::GetCosineDistance(
    const ScanContext &target_scan_context, 
    const ScanContext &source_scan_context 
) {
    const int N = target_scan_context.cols();

    int num_effective_cols = 0;
    float sum_sector_similarity = 0.0f;
    for (int sid = 0; sid < N; ++sid)
    {
        const Eigen::VectorXf target_sector = target_scan_context.col(sid);
        const Eigen::VectorXf source_sector = source_scan_context.col(sid);
        
        float target_sector_norm = target_sector.norm();
        float source_sector_norm = source_sector.norm();

        if( 0.0f == target_sector_norm || 0.0f == source_sector_norm ) {
            continue;
        }
             
        float sector_similarity = target_sector.dot(source_sector) / (target_sector_norm * source_sector_norm);

        sum_sector_similarity += sector_similarity;
        ++num_effective_cols;
    }
    
    return (0 == num_effective_cols ? 1.0f : (1.0f - sum_sector_similarity / num_effective_cols));
}

/**
 * @brief  get scan context match result between target and source scan context 
 * @param  target_scan_context, target scan context 
 * @param  source_scan_context, source scan context 
 * @return scan context match result as std::pair<int, float>
 */
std::pair<int, float> ScanContextManager::GetScanContextMatch(
    const ScanContext &target_scan_context, 
    const ScanContext &source_scan_context
) {
    // first perform fast alignment using sector key:
    Eigen::MatrixXf target_sector_key = GetSectorKey(target_scan_context);
    Eigen::MatrixXf source_sector_key = GetSectorKey(source_scan_context);
    int sector_key_shift = GetOptimalShiftUsingSectorKey(
        target_sector_key, 
        source_sector_key 
    );

    // generate precise alignment proposals:
    const int N = target_scan_context.cols();
    const int SEARCH_RADIUS = round(
        0.5 * FAST_ALIGNMENT_SEARCH_RATIO_ * N
    );
    std::vector<int> candidate_shifts{ sector_key_shift };
    for (int r = 1; r < SEARCH_RADIUS + 1; ++r)
    {
        candidate_shifts.push_back(
            (sector_key_shift + r + N) % N 
        );
        candidate_shifts.push_back( 
            (sector_key_shift - r + N) % N 
        );
    }
    std::sort(candidate_shifts.begin(), candidate_shifts.end());

    // then continue to precise alignment using scan context cosine distance:
    int optimal_shift = 0;
    float optimal_dist = std::numeric_limits<float>::max();
    for (int curr_shift: candidate_shifts)
    {
        ScanContext target_scan_context_shifted = CircularShift(
            target_scan_context, 
            curr_shift
        );

        float curr_dist = GetCosineDistance(
            target_scan_context_shifted, 
            source_scan_context 
        );

        if(curr_dist < optimal_dist)
        {   
            optimal_shift = curr_shift;
            optimal_dist = curr_dist;
        }
    }

    return std::make_pair(optimal_shift, optimal_dist);
}

/**
 * @brief  update scan context index 
 * @param  MIN_KEY_FRAME_SEQ_DISTANCE, min. seq distance from current key frame 
 * @return true if success otherwise false
 */
bool ScanContextManager::UpdateIndex(const int MIN_KEY_FRAME_SEQ_DISTANCE) {
    if (
        state_.ring_key_.size() > static_cast<size_t>(MIN_KEY_FRAME_SEQ_DISTANCE)
    ) {
        // fetch to-be-indexed ring keys from buffer:
        state_.index_.data_.ring_key_.clear();
        state_.index_.data_.ring_key_.insert(
            state_.index_.data_.ring_key_.end(),
            state_.ring_key_.begin(), 
            // this ensures the min. key frame seq. distance
            state_.ring_key_.end() - MIN_KEY_FRAME_SEQ_DISTANCE
        );

        state_.index_.data_.key_frame_.clear();
        state_.index_.data_.key_frame_.insert(
            state_.index_.data_.key_frame_.end(),
            state_.key_frame_.begin(), 
            // this ensures the min. key frame seq. distance
            state_.key_frame_.end() - MIN_KEY_FRAME_SEQ_DISTANCE
        );

        // TODO: enable in-place update of KDTree
        state_.index_.kd_tree_.reset(); 
        state_.index_.kd_tree_ = std::make_shared<RingKeyIndex>(
            NUM_RINGS_,  /* dim */
            state_.index_.data_.ring_key_,
            10           /* max leaf size */
        );

        return true;
    }

    return false;
}

/**
 * @brief  get loop closure match result for given scan context and ring key 
 * @param  query_scan_context, query scan context 
 * @param  query_ring_key, query ring key
 * @return loop closure match result as std::pair<int, float>
 */
std::pair<int, float> ScanContextManager::GetLoopClosureMatch(
    const ScanContext &query_scan_context,
    const RingKey &query_ring_key
) {
    int match_id = NONE;

    //
    // step 1: loop closure detection criteria check -- only perform loop closure detection when
    //   a. current key scan is temporally far away from previous key scan:
    // 
    if(
        state_.ring_key_.size() <= (static_cast<size_t>(MIN_KEY_FRAME_SEQ_DISTANCE_))
    ) {
        std::pair<int, float> result {match_id, 0.0};
        return result; 
    }

    //
    // step 2: perform kNN search
    // 
    std::vector<size_t> candidate_indices(NUM_CANDIDATES_);
	std::vector<float> candidate_distances(NUM_CANDIDATES_);
    GetCandidateIndices(
        query_ring_key, NUM_CANDIDATES_,
        candidate_indices, candidate_distances
    );

    // 
    // step 3: find optimal match
    // 
    int optimal_index = 0;
    int optimal_shift = 0;
    float optimal_dist = std::numeric_limits<float>::max();
    for (int i = 0; i < NUM_CANDIDATES_; ++i)
    {   
        const ScanContext &candidate_scan_context = state_.scan_context_.at(
            candidate_indices.at(i)
        );

        std::pair<int, float> match_result = GetScanContextMatch(
            candidate_scan_context, query_scan_context
        ); 
        
        int candidate_shift = match_result.first;
        float candidate_dist = match_result.second;
        
        if(candidate_dist < optimal_dist)
        {
            optimal_dist = candidate_dist;
            optimal_shift = candidate_shift;
            optimal_index = candidate_indices.at(i);
        }
    }

    // 
    // step 4: loop closure threshold check:
    //
    float yaw_change_in_deg = optimal_shift * DEG_PER_SECTOR_;
    float yaw_change_in_rad = yaw_change_in_deg / 180.0f * M_PI;
    if(optimal_dist < SCAN_CONTEXT_DISTANCE_THRESH_)
    {
        match_id = optimal_index; 

        LOG(INFO) << std::endl
                  << "[Scan Context] Loop-Closure Detected " 
                  << state_.scan_context_.size() - 1 << "<-->" << optimal_index << std::endl 
                  << "\tDistance " << optimal_dist << std::endl 
                  << "\tHeading Change " << yaw_change_in_deg << " deg." << std::endl
                  << std::endl;
    }

    std::pair<int, float> result{match_id, yaw_change_in_rad};

    return result;
}

/**
 * @brief  save scan context index
 * @param  output_path, scan context index output path
 * @return true for success otherwise false
 */
bool ScanContextManager::SaveIndex(const std::string &output_path) {
    FILE *output_fptr = fopen(output_path.c_str(), "wb");
    if (!output_fptr) {
        return false;
    };
    state_.index_.kd_tree_->index->saveIndex(output_fptr);
    fclose(output_fptr);

    return true;
}

/**
 * @brief  save scan contexts
 * @param  output_path, scan contexts output path
 * @return true for success otherwise false
 */
bool ScanContextManager::SaveScanContexts(const std::string &output_path) {
    scan_context_io::ScanContexts scan_contexts;

    scan_contexts.set_num_rings(NUM_RINGS_);
    scan_contexts.set_num_sectors(NUM_SECTORS_);
    for (size_t i = 0; i < state_.scan_context_.size(); ++i) {
        const ScanContext &input_scan_context = state_.scan_context_.at(i);
        scan_context_io::ScanContext *output_scan_context = scan_contexts.add_data();

        for (int rid = 0; rid < NUM_RINGS_; ++rid) {
            for (int sid = 0; sid < NUM_SECTORS_; ++sid) {
                output_scan_context->add_data(
                    input_scan_context(rid, sid)
                );
            }
        }
    }

    std::fstream output(
        output_path, 
        std::ios::out | std::ios::trunc | std::ios::binary
    );
    if (!scan_contexts.SerializeToOstream(&output)) {
        return false;
    }

    return true;
}

/**
 * @brief  save ring keys
 * @param  output_path, ring keys output path
 * @return true for success otherwise false
 */
bool ScanContextManager::SaveRingKeys(const std::string &output_path) {
    scan_context_io::RingKeys ring_keys;
    for (size_t i = 0; i < state_.index_.data_.ring_key_.size(); ++i) {
        const RingKey &input_ring_key = state_.index_.data_.ring_key_.at(i);
        scan_context_io::RingKey *output_ring_key = ring_keys.add_data();

        for (size_t j = 0; j < input_ring_key.size(); ++j) {
            output_ring_key->add_data(input_ring_key.at(j));
        }
    }

    std::fstream output(
        output_path, 
        std::ios::out | std::ios::trunc | std::ios::binary
    );
    if (!ring_keys.SerializeToOstream(&output)) {
        return false;
    }

    return true;
}

/**
 * @brief  save key frames
 * @param  output_path, key frames output path
 * @return true for success otherwise false
 */
bool ScanContextManager::SaveKeyFrames(const std::string &output_path) {
    scan_context_io::KeyFrames key_frames;
    for (size_t i = 0; i < state_.index_.data_.key_frame_.size(); ++i) {
        const KeyFrame &input_key_frame = state_.index_.data_.key_frame_.at(i);
        scan_context_io::KeyFrame *output_key_frame = key_frames.add_data();

        // a. set orientation:
        const Eigen::Quaternionf input_q = input_key_frame.GetQuaternion();
        scan_context_io::Quat *output_q = new scan_context_io::Quat();

        output_q->set_w(input_q.w());
        output_q->set_x(input_q.x());
        output_q->set_y(input_q.y());
        output_q->set_z(input_q.z());

        // b. set translation:
        const Eigen::Vector3f input_t = input_key_frame.GetTranslation();
        scan_context_io::Trans *output_t = new scan_context_io::Trans();

        output_t->set_x(input_t.x());
        output_t->set_y(input_t.y());
        output_t->set_z(input_t.z());

        output_key_frame->set_allocated_q(output_q);
        output_key_frame->set_allocated_t(output_t);
    }

    std::fstream output(
        output_path, 
        std::ios::out | std::ios::trunc | std::ios::binary
    );
    if (!key_frames.SerializeToOstream(&output)) {
        return false;
    }

    return true;
}

/**
 * @brief  load scan contexts
 * @param  input_path, scan contexts input path
 * @return true for success otherwise false
 */
bool ScanContextManager::LoadScanContexts(const std::string &input_path) {
    scan_context_io::ScanContexts scan_contexts;

    std::fstream input(
        input_path, 
        std::ios::in | std::ios::binary
    );
    if (!scan_contexts.ParseFromIstream(&input)) {
        return false;
    }

    state_.scan_context_.clear();
    state_.scan_context_.resize(scan_contexts.data_size());
    for (int i = 0; i < scan_contexts.data_size(); ++i) {
        const scan_context_io::ScanContext &input_scan_context = scan_contexts.data(i);
        ScanContext output_scan_context = ScanContext::Zero(
            scan_contexts.num_rings(), scan_contexts.num_sectors()
        );

        for (int rid = 0; rid < scan_contexts.num_rings(); ++rid) {
            for (int sid = 0; sid < scan_contexts.num_sectors(); ++sid) {
                // get data id:
                int did = rid * scan_contexts.num_sectors() + sid;
                output_scan_context(rid, sid) = input_scan_context.data(did);
            }
        }

        state_.scan_context_.at(i) = output_scan_context;
    }

    return true;
}

/**
 * @brief  load ring keys
 * @param  input_path, ring keys input path
 * @return true for success otherwise false
 */
bool ScanContextManager::LoadRingKeys(const std::string &input_path) {
    scan_context_io::RingKeys ring_keys;

    std::fstream input(
        input_path, 
        std::ios::in | std::ios::binary
    );
    if (!ring_keys.ParseFromIstream(&input)) {
        return false;
    }

    state_.ring_key_.clear();
    state_.ring_key_.resize(ring_keys.data_size());
    state_.index_.data_.ring_key_.clear();
    state_.index_.data_.ring_key_.resize(ring_keys.data_size());
    for (int i = 0; i < ring_keys.data_size(); ++i) {
        const scan_context_io::RingKey &input_ring_key = ring_keys.data(i);
        RingKey output_ring_key;

        output_ring_key.clear();
        output_ring_key.resize(input_ring_key.data_size());
        for (int j = 0; j < input_ring_key.data_size(); ++j) {
            output_ring_key.at(j) = input_ring_key.data(j);
        }

        state_.ring_key_.at(i) = output_ring_key;
        state_.index_.data_.ring_key_.at(i) = output_ring_key;
    }

    return true;
}

/**
 * @brief  load key frames
 * @param  input_path, key frames input path
 * @return true for success otherwise false
 */
bool ScanContextManager::LoadKeyFrames(const std::string &input_path) {
    scan_context_io::KeyFrames key_frames;

    std::fstream input(
        input_path, 
        std::ios::in | std::ios::binary
    );
    if (!key_frames.ParseFromIstream(&input)) {
        return false;
    }
    
    state_.index_.data_.key_frame_.clear();
    state_.index_.data_.key_frame_.resize(key_frames.data_size());
    for (int i = 0; i < key_frames.data_size(); ++i) {
        const scan_context_io::KeyFrame &input_key_frame = key_frames.data(i);
        KeyFrame &output_key_frame = state_.index_.data_.key_frame_.at(i);

        output_key_frame.index = i;

        Eigen::Quaternionf input_q(
            input_key_frame.q().w(),
            input_key_frame.q().x(),
            input_key_frame.q().y(),
            input_key_frame.q().z()
        );
        Eigen::Vector3f input_t(
            input_key_frame.t().x(),
            input_key_frame.t().y(),
            input_key_frame.t().z()
        );

        output_key_frame.pose.block<3, 3>(0, 0) = input_q.toRotationMatrix();
        output_key_frame.pose.block<3, 1>(0, 3) = input_t;
    }

    return true;
}

} // namespace lidar_localization