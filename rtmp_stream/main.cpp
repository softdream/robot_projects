#include "librtmp_send264.h"
#include <iostream>

#include "image_utils.h"

#include <opencv2/opencv.hpp>

extern "C" {
#include "sp_codec.h"
#include "sp_sys.h"
}

// ---------------------- GLOBAL DATA --------------------- //
void *encoder = nullptr;
cv::Mat image;
cv::VideoCapture cap(8);


int readBuffer(unsigned char *buf, int buf_size )
{
	cv::Mat nv12_img;
	int c_ret = ImageUtils::BGRToNv12( image, nv12_img );
        std::cout<<"convert ret = "<<c_ret<<std::endl;

       	c_ret = sp_encoder_set_frame( encoder, (char *)nv12_img.data, 720 * 480 * 3 / 2 );
        if ( c_ret != 0 ) {
        	std::cerr<<"Failed to encode the data !"<<std::endl;
        }

        int size = sp_encoder_get_stream( encoder, (char *)buf );
        std::cout<<"encoded size = "<<size<<std::endl;


	return size;
}


int main()
{
	std::cout<<"----------------------- RTMP TEST -----------------------"<<std::endl;

        if ( !cap.isOpened() ) {
                std::cout<<"Camera Open Failed !"<<std::endl;
                return 0;
        }
        std::cout<<"Open the Camera !"<<std::endl;

	encoder = sp_init_encoder_module();
        if ( encoder == nullptr ) {
                std::cerr<<"init encoder failed !"<<std::endl;
                return 0;
        }

        int ret = sp_start_encode( encoder, 0, SP_ENCODER_H264, 720, 480, 1024 );
        if ( ret != 0 ) {
                std::cerr<<"Error to start encoder !"<<std::endl;
                return 0;
        }



	RTMP264_Connect("rtmp://42.192.222.24:1935/live/stream");

	
	while ( 1 ) { 
		cap >> image;

                cv::resize( image, image, cv::Size( 720, 480 ) );

                //cv::imshow("image", image);

		if ( cv::waitKey(50) == 'q' ) break;

		RTMP264_Send( readBuffer );

	}


	RTMP264_Close();

	sp_stop_encode( encoder );
        sp_release_encoder_module( encoder );

	return 0;
}
