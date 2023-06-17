#include "odometry.h"
#include <opencv2/opencv.hpp>
#include <chrono>

#include <iostream>

#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <dirent.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/un.h>
#include <sys/time.h>

#include <arpa/inet.h>

#include <vector>

#define WIDTH 480
#define HEIGHT 360

#define BUFFER_SIZE 30

// -------------------------------------- GLOBAL DATA ---------------------------------------- //
odom::Odometry<float> odometry;
cv::VideoCapture cap(8);

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
                return false;
        }

        std::cout<<"Init Udp Socket : "<<sock_fd<<std::endl;
        return true;
}


// ------------------------------------------------------------------------------------------- //

int main()
{
	std::cout<<"---------------------- DESKTOP ROBORT LOCALIZATION --------------------"<<std::endl;

	odometry.initDevice();

	if ( !cap.isOpened() ) {
                std::cout<<"Camera Open Failed !"<<std::endl;
                return 0;
        }
        std::cout<<"Open the Camera !"<<std::endl;
	
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

                                cv::resize( image, image, cv::Size( WIDTH, HEIGHT ) );

                                std::vector<unsigned char> vec;
                                std::vector<int> params;
                                params.push_back(cv::IMWRITE_JPEG_QUALITY);
                                params.push_back( 20 );

                                cv::imencode( ".jpg", image, vec, params );
                                std::cout<<"encoded size = "<<vec.size()<<std::endl;

                                int ret = ::sendto( sock_fd, (char *)vec.data(), vec.size(), 0, (struct sockaddr*)&client_sock_addr, client_sock_len );

                                if( ret <= 0 ){
                                        std::cerr<<"send failed : "<<ret<<std::endl;
                                }

			}
			else if ( !str.compare( "forward" ) ) {
				odometry.sendControlVector( 0.12, 0 );
			}
			else if ( !str.compare( "left" ) ) {
				odometry.sendControlVector( 0, -0.12 );
			}
			else if ( !str.compare( "right" ) ) {
				odometry.sendControlVector( 0, 0.12 );
			}
			else if ( !str.compare( "stop" ) ) {
				odometry.sendControlVector( 0, 0 );
			}
		}
	}



	return 0;
}
