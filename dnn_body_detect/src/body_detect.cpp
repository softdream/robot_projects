#include "body_detect.h"

namespace body_detect
{

BodyDetection::BodyDetection() : DnnNode( "dnn_node" )
{
	if( Init() != 0 ){
		std::cerr<<"body_detect: "<<"Init failed !"<<std::endl;

		return;
	}

	auto model_manage = GetModel();
  	if (!model_manage) {
		std::cerr<<"body_detect : "<<"Invalid Model !"<<std::endl;
	
		return;
	}

	parser_para_ = std::make_shared<FasterRcnnKpsParserPara>();
	
	hbDNNTensorProperties tensor_properties;
  	model_manage->GetOutputTensorProperties(tensor_properties, kps_output_index_);
	
	parser_para_->aligned_kps_dim.clear();
  	parser_para_->kps_shifts_.clear();

	for (int i = 0; i < tensor_properties.alignedShape.numDimensions; i++) {
    		parser_para_->aligned_kps_dim.push_back( tensor_properties.alignedShape.dimensionSize[i] );
  	}
	
	for (int i = 0; i < tensor_properties.shift.shiftLen; i++) {
    		parser_para_->kps_shifts_.push_back( static_cast<uint8_t>(tensor_properties.shift.shiftData[i]) );
  	}

	std::cout<<"aligned_kps_dim : "<<std::endl;
	for (const auto& val : parser_para_->aligned_kps_dim) {
		std::cout << " " << val;
    	}

	std::cout<<"\nkps_shifts: "<<std::endl;
	for (const auto& val : parser_para_->kps_shifts_) {
      		std::cout << " " << val;
    	}
	std::cout<<"\n";

	if (GetModelInputSize(0, model_input_width_, model_input_height_) < 0) {
		std::cerr<<"body_detect : "<<"Get Model input size failed !"<<std::endl;
		return;
	}
	else {
		std::cerr<<"body_detect : "<<"The Model input width is "<<model_input_width_<<", and height is "<<model_input_height_<<std::endl;
	}

}

BodyDetection::~BodyDetection()
{

}

int BodyDetection::SetNodePara()
{
	std::cout<<"body_detect : "<<"Set Node para ."<<std::endl;

	if (!dnn_node_para_ptr_) {
	    	return -1;
  	}
	
	dnn_node_para_ptr_->model_file = model_file_name_;
	dnn_node_para_ptr_->model_name = model_name_;
	dnn_node_para_ptr_->model_task_type = model_task_type_;
	dnn_node_para_ptr_->task_num = 2;

	return 0;
}

int BodyDetection::PostProcess( const std::shared_ptr<DnnNodeOutput>& output )
{
	std::cout<<"body_detect : "<<"outputs.size() = "<<output->outputs.size()<<std::endl;

	// results的维度等于检测出来的目标类别数
    	std::vector<std::shared_ptr<hobot::dnn_node::parser_fasterrcnn::Filter2DResult>> results;
    	std::shared_ptr<LandmarksResult> lmk_result = nullptr;

	// 使用hobot dnn内置的Parse解析方法，解析算法输出的DNNTensor类型数据
    	if (hobot::dnn_node::parser_fasterrcnn::Parse( output, 
						       parser_para_,
    						       box_outputs_index_, 
						       kps_output_index_, 
						       body_box_output_index_, 
						       results, 
						       lmk_result) < 0) {
		std::cerr<<"body_detect : "<<"Parse node_output failed !"<<std::endl;
	      	return -1;
    	}
	
	for (const auto& idx : box_outputs_index_) {
      		if (idx >= results.size()) {
       			std::cerr<<"body_detect : "<<"Output index: " <<idx<<" exceeds results size "<<results.size()<<std::endl;
			return -1;
      		}

      		auto filter2d_result = results.at(idx);
      		if (!filter2d_result) {
        		continue;
      		}

      		if (box_outputs_index_type_.find(idx) == box_outputs_index_type_.end()) {
			std::cerr<<"body_detect : "<<"Invalid output index : "<<idx<<std::endl;
        		return -1;
      		}
	
		// save the results for displaying 
		rois_ret[idx].resize(0);	
		
		std::string roi_type = box_outputs_index_type_[idx];
		std::cout<<"body_detect : "<<"Output box type : "<<roi_type<<", rect size : "<<filter2d_result->boxes.size()<<std::endl;

		for (auto& rect : filter2d_result->boxes) {
        		if (rect.left < 0) rect.left = 0;
        		if (rect.top < 0) rect.top = 0;
        		if (rect.right > model_input_width_) {
          			rect.right = model_input_width_;
        		}
       	 		if (rect.bottom > model_input_height_) {
          			rect.bottom = model_input_height_;
        		}
			std::cout <<"body_detect : "<< "rect: " << rect.left << " " << rect.top << " " << rect.right
           			  << " " << rect.bottom << ", " << rect.conf<<std::endl;
			// save the results for displaying 
			rois_ret[idx].emplace_back( cv::Rect( rect.left, rect.top, rect.right - rect.left, rect.bottom - rect.top ) );
      		}
	}	

	if (lmk_result) {
                for (const auto& value : lmk_result->values) {
			std::vector<cv::Point2f> key_vec;
                	std::cout<<"body_detect : " << "kps point: ";
                        for (const auto& lmk : value) {
	                        std::cout << "\n" << lmk.x << "," << lmk.y << "," << lmk.score;
				key_vec.push_back( cv::Point2f( lmk.x, lmk.y ) );
                        }
                        std::cout << "\n-------------------\n";
                
			key_points_.push_back( key_vec );
		}
	}

}


bool BodyDetection::imageProcess( const cv::Mat& image )
{
	// 1. 将图片处理成模型输入数据类型DNNInput	
	std::shared_ptr<hobot::easy_dnn::NV12PyramidInput> pyramid = nullptr;
	pyramid = ImageUtils::GetNV12Pyramid( image, model_input_height_, model_input_width_ );
	
	if (!pyramid) {
		std::cerr<<"body_detect : "<<"Get Nv12 pym failed !"<<std::endl;
		return false;
	}

	// 2. 使用pyramid创建DNNInput对象inputs
	auto inputs = std::vector<std::shared_ptr<DNNInput>>{ pyramid };
	

	// 3. 开始预测
	std::shared_ptr<DnnNodeOutput> dnn_output;
	int ret = Run( inputs, dnn_output, nullptr, true );

	// 4. 处理预测结果，如渲染到图片或者发布预测结果
	if( ret != 0 ){
		std::cerr<<"body_detect : "<<"Run Predict failed : "<< ret<<std::endl;
		return false;
	}

	return true;
}

void BodyDetection::displayResults( cv::Mat& image )
{
	for( auto& obs : rois_ret ){
		if( obs.first == body_box_output_index_ ){
			for( auto& rect : obs.second ){
				cv::putText( image, "body", cv::Point( rect.x, rect.y - 10 ), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 0, 0), 2 );
				cv::rectangle( image, rect, cv::Scalar(255, 0, 0), 2 );
			}
		}
		else if( obs.first == head_box_output_index_ ){
                        for( auto& rect : obs.second ){
                                cv::putText( image, "head", cv::Point( rect.x, rect.y - 10 ), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 0, 255), 2 );
                                cv::rectangle( image, rect, cv::Scalar(0, 0, 255), 2 );
                        }
                }
		else if( obs.first == face_box_output_index_ ){
                        for( auto& rect : obs.second ){
                                cv::putText( image, "face", cv::Point( rect.x, rect.y - 10 ), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 0), 2 );
                                cv::rectangle( image, rect, cv::Scalar(0, 255, 0), 2 );
                        }
                }
		else if( obs.first == hand_box_output_index_ ){
                        for( auto& rect : obs.second ){
                                cv::putText( image, "hand", cv::Point( rect.x, rect.y - 10 ), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 255), 2 );
                                cv::rectangle( image, rect, cv::Scalar(0, 255, 255), 2 );
                        }
                }

	}

	rois_ret.clear();

	for ( const auto& it : key_points_ ) {
		for ( int i = 0; i < it.size(); i ++ ) {
			cv::putText( image, std::to_string( i ), cv::Point( it[i].x, it[i].y - 10 ), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 255), 2 );
			cv::circle( image, it[i], 4, cv::Scalar( 255, 255, 0 ), -1 );
		}
		
		cv::line( image, it[5], it[6], cv::Scalar( 0, 0, 255 ), 2 );
		cv::line( image, it[6], it[12], cv::Scalar( 0, 0, 255 ), 2 );
		cv::line( image, it[12], it[11], cv::Scalar( 0, 0, 255 ), 2 );
		cv::line( image, it[11], it[5], cv::Scalar( 0, 0, 255 ), 2 );
		cv::line( image, it[6], it[8], cv::Scalar( 0, 0, 255 ), 2 );
		cv::line( image, it[8], it[10], cv::Scalar( 0, 0, 255 ), 2 );
		cv::line( image, it[5], it[7], cv::Scalar( 0, 0, 255 ), 2 );
		cv::line( image, it[7], it[9], cv::Scalar( 0, 0, 255 ), 2 );
		cv::line( image, it[12], it[14], cv::Scalar( 0, 0, 255 ), 2 );
		cv::line( image, it[14], it[16], cv::Scalar( 0, 0, 255 ), 2 );
		cv::line( image, it[11], it[13], cv::Scalar( 0, 0, 255 ), 2 );
		cv::line( image, it[13], it[15], cv::Scalar( 0, 0, 255 ), 2 );
	}

	key_points_.clear();
}

}
