// Copyright (c) 2022，Horizon Robotics.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef EASY_DNN_FACE_HAND_DETECTION_OUTPUT_PARSER_H
#define EASY_DNN_FACE_HAND_DETECTION_OUTPUT_PARSER_H

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "dnn_node/dnn_node_data.h"

namespace hobot {
namespace dnn_node {
namespace parser_fasterrcnn {

// 解析人体关键点输出需要的参数，从加载的模型中查询
struct FasterRcnnKpsParserPara {
  int kps_points_number_ = 19;
  float kps_pos_distance_ = 0.1;
  float kps_anchor_param_ = -0.46875;
  int kps_feat_width_ = 16;
  int kps_feat_height_ = 16;
  std::vector<int> aligned_kps_dim;
  std::vector<uint32_t> kps_shifts_;
};

// 解析后的检测框数据类型
struct PerceptionRect {
 public:
  float left;
  float top;
  float right;
  float bottom;
  float conf;
  int type;
  int perception_type;
  int conf_scale;
  int expand;
  int branch;
  friend bool operator>(const PerceptionRect &lhs, const PerceptionRect &rhs) {
    return (lhs.conf > rhs.conf);
  }
};

// 检测框集合，Filter2DResult表示一类检测框
class Filter2DResult {
 public:
  std::vector<PerceptionRect> boxes;

  void Reset() { boxes.clear(); }
};

/**
 * \~Chinese @brief 2D坐标点
 */
template <typename Dtype>
struct Point_ {
  inline Point_() {}
  inline Point_(Dtype x_, Dtype y_, float score_ = 0.0)
      : x(x_), y(y_), score(score_) {}

  Dtype x = 0;
  Dtype y = 0;
  float score = 0.0;
};
typedef Point_<float> Point;
typedef std::vector<Point> Landmarks;

// 关键点集合，LandmarksResult表示一类关键点
class LandmarksResult {
 public:
  std::vector<Landmarks> values;

  void Reset() { values.clear(); }
};

// 算法输出解析方法
// - 参数
//   - [in] node_output dnn node输出，包含算法推理输出
//          解析时，如果不需要使用前处理参数，可以直接使用DnnNodeOutput中的
//          std::vector<std::shared_ptr<DNNTensor>>
//          output_tensors成员作为Parse的入口参数
//   - [in] parser_para 解析人体关键点输出需要的参数，从加载的模型中查询
//   - [in] box_outputs_index 人体、人头、人脸、人手等检测框输出索引集合
//   - [in] kps_output_index 人体关键点输出索引
//   - [in] body_box_output_index 人体检测框输出索引
//   - [in/out] outputs 解析后的检测框结构化数据
//   - [in/out] output_body_kps 解析后的人体关键点结构化数据
// - 返回值
//   - 0 成功
//   - -1 失败
int32_t Parse(
    const std::shared_ptr<hobot::dnn_node::DnnNodeOutput> &node_output,
    const std::shared_ptr<FasterRcnnKpsParserPara> &parser_para,
    const std::vector<int32_t> &box_outputs_index,
    int32_t kps_output_index,
    int32_t body_box_output_index,
    std::vector<std::shared_ptr<Filter2DResult>> &outputs,
    std::shared_ptr<LandmarksResult> &output_body_kps);

}  // namespace parser_fasterrcnn
}  // namespace dnn_node
}  // namespace hobot

#endif  // EASY_DNN_DETECTION_OUTPUT_PARSER_H
