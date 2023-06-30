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

#ifndef DNN_NODE_DATA_H_
#define DNN_NODE_DATA_H_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "dnn/hb_dnn.h"
#include "easy_dnn/data_structure.h"
#include "easy_dnn/input_process/crop.h"
#include "easy_dnn/input_processor.h"
#include "easy_dnn/model.h"
#include "easy_dnn/model_manager.h"
#include "easy_dnn/output_parser.h"
#include "easy_dnn/task.h"
#include "easy_dnn/task_manager.h"

namespace hobot {
namespace dnn_node {

using hobot::easy_dnn::CropDescription;
using hobot::easy_dnn::CropProcessor;
using hobot::easy_dnn::DNNInput;
using hobot::easy_dnn::DNNResult;
using hobot::easy_dnn::DNNTensor;
using hobot::easy_dnn::InputDescription;
using hobot::easy_dnn::InputProcessor;
using hobot::easy_dnn::Model;
using hobot::easy_dnn::ModelInferTask;
using hobot::easy_dnn::ModelManager;
using hobot::easy_dnn::ModelRoiInferTask;
using hobot::easy_dnn::ModelTask;
using hobot::easy_dnn::MultiModelTask;
using hobot::easy_dnn::NV12PyramidInput;
using hobot::easy_dnn::OutputParser;
using hobot::easy_dnn::Task;
using hobot::easy_dnn::TaskManager;
//using rclcpp::NodeOptions;

using hobot::easy_dnn::MultiBranchOutputParser;
using hobot::easy_dnn::OutputDescription;
using hobot::easy_dnn::SingleBranchOutputParser;

using TaskId = int;

// 任务实体类型，包括以下几种类型
// - ModelInferTask: Pyramid或DDR模型任务
// - ModelRoiInferTask: Resizer模型任务，
//   有且只有一个输入源为resizer (pyramid和roi)，剩余的为pyramid或DDR
enum class ModelTaskType {
  InvalidType = 0,
  ModelInferType = 1,
  ModelRoiInferType = 2
};

// 推理任务使用的BPU核，包括以下几种类型
// - BPU_CORE_ANY: 不指定BPU核，推理任务交替使用两个BPU核，实现负载均衡
// - BPU_CORE_0: 推理任务使用BPU 0
// - BPU_CORE_1: 推理任务使用BPU 1
enum class BPUCoreIDType { BPU_CORE_ANY = 0, BPU_CORE_0 = 1, BPU_CORE_1 = 2 };

struct DnnNodePara {
  // 模型文件名称
  std::string model_file;

  // 模型名，dnn node根据model_name解析出需要管理和推理使用的模型
  // 根据模型文件名称，在X3派上查询模型名的方法为：
  // hrt_model_exec model_info --model_file [model_file（模型文件名称）]
  // 查询输出包含模型名、模型输入和输出的信息，其中[model name]为模型名
  // 如果模型文件中只包含一个模型时，可以不指定模型名
  std::string model_name{""};

  // 模型的task类型，dnn node根据模型类型创建task
  // 当算法的输入包含roi（Region of
  // Interest，例如目标的检测框）时选择ModelRoiInferType类型，其他情况下选择ModelInferType类型。
  ModelTaskType model_task_type = ModelTaskType::ModelInferType;

  // 创建task超时时间，单位ms
  int timeout_ms = 100;

  // 创建的task数量，一个model支持由多个task执行
  // 如果算法推理耗时较长（推理输出帧率低于输入帧率），需要使用更多的task进行推理
  int task_num = 2;

  // 为task指定使用的BPU核，bpu_core_ids为空或者size等于task_num
  // 为空不指定，即使用BPU_CORE_ANY模式
  // 当bpu_core_ids size等于task_num时，分别为每个task指定对应的BPU核
  // 例如task_num为2，bpu_core_ids为{BPUCoreIDType::BPU_CORE_0,
  // BPUCoreIDType::BPU_CORE_1}时，两个task分别运行在BPU 0和BPU 1上
  std::vector<BPUCoreIDType> bpu_core_ids{};

  // 模型输出索引和对应的解析方式
  // 如果用户子类中没有override SetOutputParser接口，
  // dnn_node基类的SetOutputParser将使用output_parsers_设置模型解析方式
  std::vector<std::pair<int32_t, std::shared_ptr<OutputParser>>>
      output_parsers_;
};

// 运行时时间统计
struct DnnNodeRunTimeStat {
  // 推理统计，包括输入数据给模型、处理输入数据、模型推理、等待推理结束的过程
  int infer_time_ms = 0;
  struct timespec infer_timespec_start;
  struct timespec infer_timespec_end;

  // 解析模型输出统计
  int parse_time_ms = 0;
  struct timespec parse_timespec_start;
  struct timespec parse_timespec_end;

  // 推理输入帧率
  float input_fps;
  // 推理成功的输出帧率
  float output_fps;
  // 为true表示在当前帧刷新fps
  bool fps_updated = false;
};

// 用户可以继承DnnNodeOutput来扩展输出内容
// 例如增加推理结果对应的图片数据、图片名、时间戳、ID等
struct DnnNodeOutput {
  DnnNodeOutput() { outputs.clear(); }
  virtual ~DnnNodeOutput() {}

  // 模型输出的tensor数据指针列表，维度等于模型输出branch数
  // 推理完成后，用户可以直接解析output_tensors并使用解析后的结构化数据
  std::vector<std::shared_ptr<DNNTensor>> output_tensors;

  // 输出数据智能指针列表，即DNNTensor经过解析后的结构化数据，维度等于模型输出branch数
  // 当用户使用SetOutputParser接口配置模型输出的解析方式，推理完成后拿到output_tensors的同时还会拿到解析后的outputs数据
  std::vector<std::shared_ptr<DNNResult>> outputs;

  std::shared_ptr<DnnNodeRunTimeStat> rt_stat = nullptr;
};

}  // namespace dnn_node
}  // namespace hobot
#endif  // DNN_NODE_DATA_H_
