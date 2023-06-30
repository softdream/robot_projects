#ifndef __BODY_DETECT_H
#define __BODY_DETECT_H

#include <iostream>
#include <vector>
#include <unordered_map>

#include "dnn_node/dnn_node.h"
#include "include/image_utils.h"
#include "dnn_node/util/output_parser/detection/fasterrcnn_output_parser.h"

#include <opencv2/opencv.hpp>

namespace body_detect
{

using hobot::dnn_node::DNNInput;
using hobot::dnn_node::DnnNode;
using hobot::dnn_node::DnnNodeOutput;
using hobot::dnn_node::DnnNodePara;
using hobot::dnn_node::DNNResult;
using hobot::dnn_node::ModelTaskType;
using hobot::dnn_node::NV12PyramidInput;
using hobot::dnn_node::TaskId;

using hobot::dnn_node::Model;
using hobot::dnn_node::ModelInferTask;
using hobot::dnn_node::ModelManager;
using hobot::dnn_node::ModelRoiInferTask;

using hobot::dnn_node::OutputParser;
using hobot::dnn_node::parser_fasterrcnn::FasterRcnnKpsParserPara;
using hobot::dnn_node::parser_fasterrcnn::LandmarksResult;

struct FasterRcnnOutput : public DnnNodeOutput {
  struct timespec timestamp;
};

class BodyDetection : public DnnNode
{
public:
	BodyDetection();
	~BodyDetection() override;

	bool imageProcess( const cv::Mat& image );
	void displayResults( cv::Mat& image );

protected:
	int SetNodePara() override;
	int PostProcess( const std::shared_ptr<DnnNodeOutput>&  output ) override;

private:
	std::string model_file_name_ = "/home/sunrise/workstation/sunrise_bpu/dnn_body_detect/model/multitask_body_head_face_hand_kps_960x544.hbm";
	std::string model_name_ = "multitask_body_head_face_hand_kps_960x544";
	ModelTaskType model_task_type_ = ModelTaskType::ModelInferType;

	int model_input_width_ = -1;
	int model_input_height_ = -1;

	const int32_t body_box_output_index_ = 1;
  	const int32_t head_box_output_index_ = 3;
  	const int32_t face_box_output_index_ = 5;
  	const int32_t hand_box_output_index_ = 7;
  	const std::vector<int32_t> box_outputs_index_ = {body_box_output_index_,
                                                   	 head_box_output_index_,
                                                   	 face_box_output_index_,
                                                   	 hand_box_output_index_};


	std::shared_ptr<FasterRcnnKpsParserPara> parser_para_ = nullptr;
	const int32_t kps_output_index_ = 8;
  	std::unordered_map<int32_t, std::string> box_outputs_index_type_ = {	{body_box_output_index_, "body"},
      										{head_box_output_index_, "head"},
      										{face_box_output_index_, "face"},
      										{hand_box_output_index_, "hand"} };

	

	// for displaying
	std::unordered_map<int, std::vector<cv::Rect>> rois_ret;
	std::vector<std::vector<cv::Point2f>> key_points_;
};


}

#endif
