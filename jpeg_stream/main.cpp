#include <iostream>
#include "image_utils.h"

#include "data_transport.h"

#include <opencv2/opencv.hpp>

extern "C" {
#include "sp_codec.h"
#include "sp_sys.h"
}

char stream_buffer[960 * 544 * 3];

transport::Sender imgs_sender( "192.168.3.27", 3333 );

int main() 
{

	std::cout<<"--------------- ENCODER TEST ------------------"<<std::endl;
	cv::VideoCapture cap(8);
        if ( !cap.isOpened() ) {
                std::cout<<"Camera Open Failed !"<<std::endl;
                return 0;
        }
        std::cout<<"Open the Camera !"<<std::endl;

	void *encoder = sp_init_encoder_module();
        if ( encoder == NULL ) {
        	std::cerr<<"init encoder failed !"<<std::endl;
                return 0;
        }

	int ret = sp_start_encode( encoder, 0, SP_ENCODER_MJPEG, 720, 480, 512 );
        if ( ret != 0 ) {
                std::cerr<<"Error to start encoder !"<<std::endl;
                return 0;
        }


	cv::Mat image;
	while ( 1 ) {
		cap >> image;
		
		cv::resize( image, image, cv::Size( 720, 480 ) );

		cv::imshow("image", image);

		if ( cv::waitKey(50) == 'q' ) {
			break;
		}

		cv::Mat nv12_img;
		int c_ret = ImageUtils::BGRToNv12( image, nv12_img );
		std::cout<<"convert ret = "<<c_ret<<std::endl;


		c_ret = sp_encoder_set_frame( encoder, (char *)nv12_img.data, 720 * 480 * 3 / 2 );
		if ( c_ret != 0 ) {
			std::cerr<<"Failed to encode the data !"<<std::endl;
		}

		int size = sp_encoder_get_stream( encoder, stream_buffer );
		std::cout<<"encoded size = "<<size<<std::endl;
	
		imgs_sender.send( stream_buffer, size );
	}

	sp_stop_encode( encoder );
	sp_release_encoder_module( encoder );

	return 0;
}
