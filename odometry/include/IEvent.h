#ifndef __IEVENT_H_
#define __IEVENT_H_

#include <iostream>
#include <sys/epoll.h>

#include <functional>

typedef std::function<void*(int, void*)> FUNC;

typedef struct{
	int fd;// the fd want to monitor
	short event;// the event want to monitor
	//void *( *callback )( int fd, void *arg );// the callback function
	FUNC callback;
	void *arg;// the parameters of the callback function
}Event;


/* the interface of the event */
class IEvent{
public:
	virtual int addEvent( const Event &event ) = 0;
	virtual int delEvent( const Event &event ) = 0;
	virtual int dispatcher() = 0;

	virtual ~IEvent()
	{

	}
};




#endif
