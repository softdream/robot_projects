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

#ifndef DNN_NODE_IMAGE_PROC_H
#define DNN_NODE_IMAGE_PROC_H

#include <memory>
#include <string>
#include <vector>

#include "dnn_node/dnn_node_data.h"

namespace hobot {
namespace dnn_node {

#define ALIGNED_2E(w, alignment) \
  ((static_cast<uint32_t>(w) + (alignment - 1U)) & (~(alignment - 1U)))
#define ALIGN_4(w) ALIGNED_2E(w, 4U)
#define ALIGN_8(w) ALIGNED_2E(w, 8U)
#define ALIGN_16(w) ALIGNED_2E(w, 16U)
#define ALIGN_64(w) ALIGNED_2E(w, 64U)

class ImageProc {
 public:
  // 使用nv12编码格式图片数据生成NV12PyramidInput
  // 如果输入图片size小于scale size（模型输入size）：将输入图片padding到左上区域
  // 如果输入图片size大于scale size（模型输入size）：crop输入图片左上区域
  // - 参数
  //   - [in] in_img_data 图片数据
  //   - [in] in_img_height 图片的高度
  //   - [in] in_img_width 图片的宽度
  //   - [in] scaled_img_height 模型输入的高度
  //   - [in] scaled_img_width 模型输入的宽度
  static std::shared_ptr<NV12PyramidInput> GetNV12PyramidFromNV12Img(
      const char* in_img_data,
      const int& in_img_height,
      const int& in_img_width,
      const int& scaled_img_height,
      const int& scaled_img_width);
      
  // 使用nv12编码格式图片数据生成NV12PyramidInput
  // 要求输入图片分辨率<=模型输入分辨率，将输入图片padding到中间，并设置padding参数
  // - 参数
  //   - [in] in_img_data 输入图片数据
  //   - [in] in_img_height 输入图片的高度
  //   - [in] in_img_width 输入图片的宽度
  //   - [in] scaled_img_height 模型输入的高度
  //   - [in] scaled_img_width 模型输入的宽度
  //   - [in/out] padding_l 左边padding的像素数
  //   - [in/out] padding_t 上方padding的像素数
  //   - [in/out] padding_r 右边padding的像素数
  //   - [in/out] padding_b 下边padding的像素数
  static std::shared_ptr<NV12PyramidInput> GetNV12PyramidFromNV12Img(
      const char* in_img_data,
      const int& in_img_height,
      const int& in_img_width,
      const int& scaled_img_height,
      const int& scaled_img_width,
      int& padding_l,
      int& padding_t,
      int& padding_r,
      int& padding_b);
};

}  // namespace dnn_node
}  // namespace hobot
#endif  // DNN_NODE_IMAGE_PROC_H
