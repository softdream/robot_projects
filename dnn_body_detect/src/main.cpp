#include "body_detect.h"
#include "data_transport.h"

transport::Sender images_sender( "192.168.3.27", 2400 );

int main()
{
	std::cout<<"---------------- BODY DETECT ---------------"<<std::endl;

	body_detect::BodyDetection detect;

	cv::VideoCapture capture(8);

        if( !capture.isOpened() ){
                std::cerr<<"Can not Open the Camera !"<<std::endl;
                return 0;
        }

        std::cout<<"Open the Camera !"<<std::endl;

	cv::Mat frame;
	while ( 1 ) {
		capture >> frame;

		if( cv::waitKey(40) == 'q' ){
                        break;
                }

		cv::resize( frame, frame, cv::Size( 960, 544 ) );
		cv::flip( frame, frame, 0 );

		detect.imageProcess( frame );
		detect.displayResults( frame );
	        //cv::imshow( "ret", frame );

		std::vector<unsigned char> vec;
                std::vector<int> params;
		
		params.push_back(cv::IMWRITE_JPEG_QUALITY);
		params.push_back( 50 );
		cv::imencode( ".jpg", frame, vec, params );
		
		std::cout<<"encoded size = "<<vec.size()<<std::endl;

		images_sender.send( vec );
	}


	return 0;
}
