#ifndef __EPOLLEVENT_H_
#define __EPOLLEVENT_H_

#include <iostream>
#include <sys/epoll.h>
#include <map>
#include "IEvent.h"

class EpollEvent : public IEvent 
{
public:
	EpollEvent(): epollCreateSize(16) 
	{
		initEvent();	
	}
	EpollEvent( int createSize_ )
	{
		if( createSize_ < 16 ) createSize_ = 16;
		epollCreateSize = createSize_;
		initEvent();	
	}

	virtual int addEvent( const Event &event );
	virtual int delEvent( const Event &event );
        virtual int dispatcher();

private:
	int initEvent()
	{
		int epollFd = epoll_create( this->epollCreateSize );
		if( epollFd <= 0 ){
			std::cerr<<"epoll create error ..."<<std::endl;
			return epollFd;
		}
		this->epollFd = epollFd;
		return true;
	}
	int epollCreateSize;
	int epollFd;
	std::map<int, Event> events;
};


#endif 


















