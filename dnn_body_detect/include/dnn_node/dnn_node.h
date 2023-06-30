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

#ifndef DNN_NODE_H_
#define DNN_NODE_H_

#include <memory>
#include <string>
#include <vector>

#include "dnn_node/dnn_node_data.h"

namespace hobot {
namespace dnn_node {

class DnnNodeImpl;

//class DnnNode : public rclcpp::Node {
class DnnNode{
public:
  // node_name为创建的节点名，options为选项，用法和ROS Node相同
  //DnnNode(const std::string &node_name,
  //        const NodeOptions &options = NodeOptions());

  DnnNode(const std::string& node_name);
 
 // DnnNode(const std::string &node_name,
 //         const std::string &namespace_,
 //         const NodeOptions &options = NodeOptions());

  DnnNode(const std::string &node_name,
	  const std::string &namespace_);  

  virtual ~DnnNode();

  // 执行初始化流程，只做pipeline的串联，具体的每个初始化步骤由用户（子类中）实现。
  int Init();

  // 使用DNNInput类型数据进行推理，一般除了DDR模型之外，都使用此方式推理
  // 执行推理流程，只做pipeline的串联，具体的每个推理步骤由用户（子类中）实现。
  // 用户可以继承DnnNodeOutput来扩展输出数据智能指针output
  // 例如增加推理结果对应的图片数据、图片名、时间戳、ID等
  // 如果不需要扩展输出内容，可以不传入output
  // - 参数
  //   - [in] inputs 输入数据智能指针列表
  //   - [in] outputs 输出数据智能指针
  //   - [in] rois 抠图roi数据，只对ModelRoiInferType模型有效
  //   - [in] is_sync_mode 预测模式，true为同步模式，false为异步模式
  //   - [in] alloctask_timeout_ms 申请推理任务超时时间，单位毫秒
  //                               默认一直等待直到申请成功
  //   - [in] infer_timeout_ms 推理超时时间，单位毫秒，默认1000毫秒推理超时
  int Run(std::vector<std::shared_ptr<DNNInput>> &inputs,
          const std::shared_ptr<DnnNodeOutput> &output = nullptr,
          const std::shared_ptr<std::vector<hbDNNRoi>> rois = nullptr,
          const bool is_sync_mode = false,
          const int alloctask_timeout_ms = -1,
          const int infer_timeout_ms = 1000);

  // 使用DNNInput类型数据并指定输出描述进行推理
  // 执行推理流程，只做pipeline的串联，具体的每个推理步骤由用户（子类中）实现。
  // 用户可以继承DnnNodeOutput来扩展输出数据智能指针output
  // 例如增加推理结果对应的图片数据、图片名、时间戳、ID等
  // 如果不需要扩展输出内容，可以不传入output
  // - 参数
  //   - [in] inputs 输入数据智能指针列表
  //   - [in] output_descs 输出描述智能指针列表
  //   - [in] outputs 输出数据智能指针
  //   - [in] rois 抠图roi数据，只对ModelRoiInferType模型有效
  //   - [in] is_sync_mode 预测模式，true为同步模式，false为异步模式
  //   - [in] alloctask_timeout_ms 申请推理任务超时时间，单位毫秒
  //                               默认一直等待直到申请成功
  //   - [in] infer_timeout_ms 推理超时时间，单位毫秒，默认1000毫秒推理超时
  int Run(std::vector<std::shared_ptr<DNNInput>> &inputs,
          std::vector<std::shared_ptr<OutputDescription>> &output_descs,
          const std::shared_ptr<DnnNodeOutput> &output = nullptr,
          const std::shared_ptr<std::vector<hbDNNRoi>> rois = nullptr,
          const bool is_sync_mode = false,
          const int alloctask_timeout_ms = -1,
          const int infer_timeout_ms = 1000);

  // 使用DNNTensor类型数据并指定输出描述进行推理，一般DDR模型使用此方式推理
  // - 参数
  //   - [in] inputs 输入数据智能指针列表
  //   - [in] output_descs 输出描述智能指针列表
  //   - [in] outputs 输出数据智能指针
  //   - [in] is_sync_mode 预测模式，true为同步模式，false为异步模式
  //   - [in] alloctask_timeout_ms 申请推理任务超时时间，单位毫秒
  //                               默认一直等待直到申请成功
  //   - [in] infer_timeout_ms 推理超时时间，单位毫秒，默认1000毫秒推理超时
  int Run(std::vector<std::shared_ptr<DNNTensor>> &inputs,
          std::vector<std::shared_ptr<OutputDescription>> &output_descs,
          const std::shared_ptr<DnnNodeOutput> &output = nullptr,
          const bool is_sync_mode = false,
          const int alloctask_timeout_ms = -1,
          const int infer_timeout_ms = 1000);

 protected:
  // 设置DnnNodePara类型的dnn_node_para_ptr_
  virtual int SetNodePara() = 0;

  // 配置模型输出的解析方式
  // 如果子类没有override此接口，默认使用DnnNodePara中的output_parsers_进行配置
  virtual int SetOutputParser();

  // 处理解析后的模型输出数据，例如将输出封装成msg后发布
  // 如果子类没有override此接口，使用默认的处理方法
  // - 参数
  //   - [in] outputs 输出数据智能指针
  virtual int PostProcess(const std::shared_ptr<DnnNodeOutput> &output);

 protected:
  // 模型管理和推理参数，需要用户配置模型文件名和模型类型
  std::shared_ptr<DnnNodePara> dnn_node_para_ptr_ = nullptr;

  // 获取dnn node管理和推理使用的模型。
  Model *GetModel();

  // 获取模型的输入size
  // - 参数
  //   - [in] input_index 获取的输入索引，一个模型可能有多个输入。
  //   - [out] w 模型输入的宽度。
  //   - [out] h 模型输入的高度。
  int GetModelInputSize(int32_t input_index, int &w, int &h);

 private:
  // dnn node的实现类
  std::shared_ptr<DnnNodeImpl> dnn_node_impl_;
};

}  // namespace dnn_node
}  // namespace hobot
#endif  // DNN_NODE_H_
