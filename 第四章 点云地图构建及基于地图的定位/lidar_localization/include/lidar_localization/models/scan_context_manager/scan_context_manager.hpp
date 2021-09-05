/*
 * @Description: loop closure detection using scan context
 * @Author: Ge Yao
 * @Date: 2020-10-28 15:43:03
 */

#ifndef LIDAR_LOCALIZATION_MODELS_SCAN_CONTEXT_MANAGER_HPP_
#define LIDAR_LOCALIZATION_MODELS_SCAN_CONTEXT_MANAGER_HPP_

#include <yaml-cpp/yaml.h>

#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>

#include "lidar_localization/sensor_data/cloud_data.hpp"
#include "lidar_localization/sensor_data/key_frame.hpp"

#include "lidar_localization/models/scan_context_manager/kdtree_vector_of_vectors_adaptor.hpp"

namespace lidar_localization {

class ScanContextManager {
public:
    typedef Eigen::MatrixXf ScanContext;
    typedef std::vector<float> RingKey;
    typedef std::vector<RingKey> RingKeys;
    typedef KDTreeVectorOfVectorsAdaptor<RingKeys, float> RingKeyIndex;

    static const int NONE = -1;
    
    ScanContextManager(const YAML::Node& node);

    void Update(
        const CloudData &scan,
        const KeyFrame &key_frame
    );

    /**
     * @brief  get loop closure proposal using the latest key scan
     * @param  none
     * @return loop closure proposal (key_frame_id, scan_context_distance) as std::pair<int, float>
     */
    std::pair<int, float> DetectLoopClosure(void);
    /**
     * @brief  get loop closure proposal using the given key scan
     * @param  scan, query key scan
     * @param  pose, matched pose
     * @return true for success match otherwise false
     */
    bool DetectLoopClosure(const CloudData &scan,Eigen::Matrix4f &pose);

    /**
     * @brief  save scan context index & data to persistent storage
     * @param  output_path, scan context output path
     * @return true for success otherwise false
     */
    bool Save(const std::string &output_path);
    /**
     * @brief  load scan context index & data from persistent storage
     * @param  input_path, scan context input path
     * @return true for success otherwise false
     */
    bool Load(const std::string &input_path);

private:
    /**
     * @brief  get scan context of given lidar scan
     * @param  scan, lidar scan of key frame
     * @return scan context as Eigen::MatrixXd
     */
    ScanContext GetScanContext(const CloudData &scan);
    /**
     * @brief  get ring key of given scan context
     * @param  scan_context, scan context of key scan
     * @return ring key as RingKey
     */
    RingKey GetRingKey(
        const ScanContext &scan_context
    );
    /**
     * @brief  generate random ring keys for indexing test
     * @return void
     */
    void GenerateRandomRingKey(
        RingKeys &samples, 
        const int N, const int D, const float max_range
    );
    /**
     * @brief  get sector key of given scan context
     * @param  scan_context, scan context of key scan
     * @return sector key as RingKey
     */
    Eigen::MatrixXf GetSectorKey(
        const Eigen::MatrixXf &scan_context
    );

    /**
     * @brief  get orientation of point measurement 
     * @param  x, x component of point measurement
     * @param  y, y component of point measurement
     * @return point measurement orientation, [0.0f, 360.0f)
     */
    float GetOrientation(const float &x, const float &y);
    /**
     * @brief  convert floating point value to integer index 
     * @param  value, target floating point value 
     * @param  MAX_VALUE, max. floating point value
     * @param  RESOLUTION, resolution
     * @return integer index, {0, ..., RESOLUTION - 1}
     */
    int GetIndex(const float &value, const float &MAX_VALUE, const int RESOLUTION);
    /**
     * @brief  get candidate scan context indices 
     * @param  ring_key, query ring key 
     * @param  N, num. of nearest neighbor candidates
     * @param  indices, candidate indices
     * @param  distances, candidate distances
     * @return void
     */
    void GetCandidateIndices(
        const RingKey &ring_key, const int N,
        std::vector<size_t> &indices,
        std::vector<float> &distances
    );
    /**
     * @brief  circular shift mat to right by shift
     * @param  mat, original matrix 
     * @param  shift, right shift amount  
     * @return shifted matrix
     */
    Eigen::MatrixXf CircularShift(const Eigen::MatrixXf &mat, int shift);
    /**
     * @brief  get optimal shift estimation using sector key 
     * @param  target, target sector key 
     * @param  source, source sector key  
     * @return void
     */
    int GetOptimalShiftUsingSectorKey(
        const Eigen::MatrixXf &target, 
        const Eigen::MatrixXf &source
    );
    /**
     * @brief  compute cosine distance between target and source scan context 
     * @param  target_scan_context, target scan context 
     * @param  source_scan_context, source scan context 
     * @return scan context cosine distance
     */
    float GetCosineDistance(
        const ScanContext &target_scan_context, 
        const ScanContext &source_scan_context
    );
    /**
     * @brief  get scan context match result between target and source scan context 
     * @param  target_scan_context, target scan context 
     * @param  source_scan_context, source scan context 
     * @return scan context match result as std::pair<int, float>
     */
    std::pair<int, float> GetScanContextMatch(
        const ScanContext &target_scan_context, 
        const ScanContext &source_scan_context
    );

    /**
     * @brief  update scan context index 
     * @param  MIN_KEY_FRAME_SEQ_DISTANCE, min. seq distance from current key frame 
     * @return true if success otherwise false
     */
    bool UpdateIndex(const int MIN_KEY_FRAME_SEQ_DISTANCE);
    
    /**
     * @brief  get loop closure match result for given scan context and ring key 
     * @param  query_scan_context, query scan context 
     * @param  query_ring_key, query ring key
     * @return loop closure match result as std::pair<int, float>
     */
    std::pair<int, float> GetLoopClosureMatch(
        const ScanContext &query_scan_context,
        const RingKey &query_ring_key
    );

    /**
     * @brief  save scan context index
     * @param  output_path, scan context index output path
     * @return true for success otherwise false
     */
    bool SaveIndex(const std::string &output_path);
    /**
     * @brief  save scan contexts
     * @param  output_path, scan contexts output path
     * @return true for success otherwise false
     */
    bool SaveScanContexts(const std::string &output_path);
    /**
     * @brief  save ring keys
     * @param  output_path, ring keys output path
     * @return true for success otherwise false
     */
    bool SaveRingKeys(const std::string &output_path);
    /**
     * @brief  save key frames
     * @param  output_path, key frames output path
     * @return true for success otherwise false
     */
    bool SaveKeyFrames(const std::string &output_path);

    /**
     * @brief  load scan contexts
     * @param  input_path, scan contexts input path
     * @return true for success otherwise false
     */
    bool LoadScanContexts(const std::string &input_path);
    /**
     * @brief  load ring keys
     * @param  input_path, ring keys input path
     * @return true for success otherwise false
     */
    bool LoadRingKeys(const std::string &input_path);
    /**
     * @brief  load key frames
     * @param  input_path, key frames input path
     * @return true for success otherwise false
     */
    bool LoadKeyFrames(const std::string &input_path);

    // states:
    struct {
        // a. scan context buffer:
        std::vector<ScanContext> scan_context_;
        // b. ring-key buffer:
        RingKeys ring_key_;
        // c. key frame buffer:
        std::vector<KeyFrame> key_frame_;
        // d. ring key indexing counter:
        struct {
            // 1. indexing interval counter:
            int counter_ = 0;
            // 2. kd-tree:
            std::shared_ptr<RingKeyIndex> kd_tree_;
            // 3. data:
            struct {
                RingKeys ring_key_;
                std::vector<KeyFrame> key_frame_;
            } data_;
        } index_;
    } state_;

    // hyper-params:
    // a. ROI definition:
    float MAX_RADIUS_;
    float MAX_THETA_;
    // b. resolution:
    int NUM_RINGS_;
    int NUM_SECTORS_; 
    float DEG_PER_SECTOR_;
    // c. ring key indexing interval:
    int INDEXING_INTERVAL_;
    // d. min key frame sequence distance:
    int MIN_KEY_FRAME_SEQ_DISTANCE_;
    // e. num. of nearest-neighbor search candidates:
    int NUM_CANDIDATES_;
    // f. sector key fast alignment search ratio:
    float FAST_ALIGNMENT_SEARCH_RATIO_;
    // g. scan context distance threshold:
    float SCAN_CONTEXT_DISTANCE_THRESH_;
};

} // namespace lidar_localization

#endif