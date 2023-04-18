#include "EpollEvent.h"

int EpollEvent::addEvent( const Event &event )
{
        struct epoll_event epollEvent;
        epollEvent.data.fd = event.fd;
        epollEvent.events = event.event;
	

        int ret = epoll_ctl( this->epollFd, EPOLL_CTL_ADD, event.fd, &epollEvent );
        if ( ret < 0 ) {
                std::cerr<<"epoll_ctl error ..."<<std::endl;
                return ret;
        }

        // add event to this->events
        this->events[event.fd] = event;
        return true;
}


int EpollEvent::delEvent( const Event &event )
{
        struct epoll_event epollEvent;
        epollEvent.data.fd = event.fd;
        epollEvent.events = event.event;
        int ret = epoll_ctl( this->epollFd, EPOLL_CTL_DEL, event.fd, &epollEvent );
        if ( ret < 0 ) {
                std::cerr<<"epoll_ctl EPOLL_CTL_DEL error ..."<<std::endl;
                return ret;
        }

        this->events.erase( event.fd );
        return true;
}


int EpollEvent::dispatcher()
{
        struct epoll_event epollEvents[32]; // 32 is the number of the  maximum events

        int nEvents = epoll_wait( epollFd, epollEvents, 32, -1 );

        if ( nEvents <= 0 ) {
                std::cerr<<"epoll wait error ..."<<std::endl;
                return nEvents;
        }

        // handle the events
        for ( int i = 0; i < nEvents; i ++ ) {
                int fd = epollEvents[i].data.fd;
                Event event = this->events[fd];

                if ( event.callback ) {
                        event.callback( fd, event.arg );
                }
        }
        return true;
}

