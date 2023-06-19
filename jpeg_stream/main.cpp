#include <iostream>
#include "image_utils.h"

#include "image_stream.h"

#include <chrono>

extern "C" {
#include "sp_codec.h"
#include "sp_sys.h"
}

#define BUFFER_SIZE 50

// -------------------------- GLOBAL DATA -------------------------- //
cv::VideoCapture cap(8);

char buffer[WIDTH * HEIGHT];

void* encoder = nullptr;


        // server info
        int sock_fd = -1;
        struct sockaddr_in sock_server_addr;

        // remote client info
        struct sockaddr_in client_sock_addr;
        socklen_t client_sock_len ;

        // receive buffer
        char recv_buffer[BUFFER_SIZE];

bool initUdpServer( const int port = 3333 )
        {
                sock_fd = ::socket(AF_INET, SOCK_DGRAM, 0);
                if( sock_fd < 0 ){
                        std::cerr<<"Can not create socket FD : "<<sock_fd<<std::endl;
                        return false;
                }

                bzero( &sock_server_addr, sizeof( sock_server_addr ) );

                sock_server_addr.sin_family = AF_INET;
                sock_server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
                sock_server_addr.sin_port = htons( port );

                if( ::bind( sock_fd, (struct sockaddr*)&sock_server_addr, sizeof(sock_server_addr) ) < 0 ){
                        std::cerr<<"Failed to bind the socket server address !"<<std::endl;
                        return -1;
                }

                std::cout<<"Init Udp Socket : "<<sock_fd<<std::endl;
                return true;
        }


int main()
{
	std::cout<<"------------------------ IMAGES STREAM -----------------------"<<std::endl;

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
	
	int ret = sp_start_encode( encoder, 0, SP_ENCODER_MJPEG, WIDTH, HEIGHT, 256 );
        if ( ret != 0 ) {
                std::cerr<<"Error to start encoder !"<<std::endl;
                return 0;
        }

	initUdpServer();

	cv::Mat image;

	while ( 1 ) {
		memset( recv_buffer, 0, sizeof( recv_buffer ) );

                int ret = ::recvfrom(sock_fd, recv_buffer, sizeof( recv_buffer ), 0 , (struct sockaddr*)&client_sock_addr, &client_sock_len);

		if ( ret > 0 ) {
			std::string str = recv_buffer;
			if ( !str.compare( "image" ) ) {
				cap >> image;
		                if ( image.empty() ) break;
				

				auto beforeTime = std::chrono::steady_clock::now();

				cv::resize( image, image, cv::Size( WIDTH, HEIGHT ) );

				std::vector<unsigned char> vec;
				std::vector<int> params;
				params.push_back(cv::IMWRITE_JPEG_QUALITY);
				params.push_back( 20 );

				cv::imencode( ".jpg", image, vec, params );
				std::cout<<"encoded size = "<<vec.size()<<std::endl;

				if ( vec.size() > 6000 ) {
					std::cout<<"too large !"<<std::endl;
					continue;
				}

				int ret = ::sendto( sock_fd, (char *)vec.data(), vec.size(), 0, (struct sockaddr*)&client_sock_addr, client_sock_len );

		                if( ret <= 0 ){
                		        std::cerr<<"send failed : "<<ret<<std::endl;
                		}

/*				// cv::Mat to nv12
        			cv::Mat nv12_img;
			        int c_ret = ImageUtils::BGRToNv12( image, nv12_img );
			        std::cout<<"convert ret = "<<c_ret<<std::endl;

			        // encode
        			c_ret = sp_encoder_set_frame( encoder, (char *)nv12_img.data, WIDTH * HEIGHT * 3 / 2 );
        			if ( c_ret != 0 ) {
                			std::cerr<<"Failed to encode the data !"<<std::endl;
        			}

			        int size = sp_encoder_get_stream( encoder, buffer );
        			std::cout<<"encoded size = "<<size<<std::endl;
				int ret = ::sendto( sock_fd, buffer, size, 0, (struct sockaddr*)&client_sock_addr, client_sock_len );

                                if( ret <= 0 ){
                                        std::cerr<<"send failed : "<<ret<<std::endl;
                                }

*/					
		
				auto afterTime = std::chrono::steady_clock::now();
	                        double duration_millsecond = std::chrono::duration<double, std::milli>(afterTime - beforeTime).count();
        	                std::cout<<"duration : " << duration_millsecond << "ms" << std::endl;

			}
		}
		

	}

	return 0;
}
