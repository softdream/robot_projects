#ifndef __IMAGE_STREAM_H
#define __IMAGE_STREAM_H

#include "EpollEvent.h"
#include "data_transport.h"
#include <opencv2/opencv.hpp>

#include <functional>


#define WIDTH 720
#define HEIGHT 480

namespace stream
{

using namespace std::placeholders;

class ImageStream
{
public:
	using CallBackFunc = std::function<void( cv::Mat&, char*, int& )>;

	ImageStream()
	{
		udp_ = new transport::UdpServer<50>( 3333 );
		
		epoll::Event udp_event;
		udp_event.fd = udp_->getSocketFd();
		udp_event.event |= EPOLLIN;
		udp_event.event |= EPOLLERR;
		udp_event.event |= EPOLLET;
		udp_event.arg = NULL;

		epoll::FUNC recv_cb = std::bind( &ImageStream::udpRecvCallback, this, _1, _2 );
		udp_event.callback = recv_cb;

		event_base_.addEvent( udp_event );
	}

	~ImageStream()
	{
		delete udp_;
	}

	void registerCallBack( const CallBackFunc& cb ) 
	{
		cb_ = cb;
	}	

	void* udpRecvCallback( int fd, void* arg )
	{
		int ret = udp_->read(); // receive

		if ( ret > 0 ) {
			std::string str = udp_->getRecvBuffer();
			if ( !str.compare( "start" ) ) {
				std::cout<<"can start video transport !"<<std::endl;
				return nullptr;
			}
			else if ( !str.compare( "image" ) ) {
				memset( stream_buffer_, 0, sizeof( stream_buffer_ ) );
				size_ = -1;
				cb_( frame_, stream_buffer_, size_ );

				if ( size_ != -1 )
					udp_->write( stream_buffer_, size_ );
			}
		}

		return nullptr;
	}

	void spin()
	{
		while ( true ) {
			event_base_.dispatcher();
		}
	}

private:
	epoll::EpollEvent event_base_;

	transport::UdpServer<50> *udp_ = nullptr;

	CallBackFunc cb_;

	cv::Mat frame_;
	char stream_buffer_[WIDTH * HEIGHT];
	int size_ = -1;
};

}

#endif
