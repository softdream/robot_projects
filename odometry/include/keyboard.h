#ifndef __KEYBOARD_H
#define __KEYBORAD_H

#include <type_traits>
#include <unordered_map>
#include <vector>

#include <unistd.h>
#include <iostream>
#include <sys/types.h>
#include <fcntl.h>
#include <linux/input.h>

#include <termios.h>

#include "EpollEvent.h"

#define ECHOFLAGS (ECHO | ECHOE | ECHOK | ECHONL)

#define KEYBOARD_DEVICE_PATH "/dev/input/event1"

namespace keyboard
{

using namespace std::placeholders;

typedef void(*CallBack)(void);

class Keyboard
{
public:
	Keyboard()
	{

	}

	~Keyboard()
	{

	}

	bool init()
	{
		setDispMode( STDIN_FILENO, 0 );

		key_fd = open( KEYBOARD_DEVICE_PATH, O_RDONLY);
		if( key_fd <= 0 ){
			std::cerr<<"Failed to open keyboard device !"<<std::endl;
			return false;
		}

		std::cout<<"Open the KeyBoard Device !"<<std::endl;

		Event key_event;
	        key_event.fd = key_fd;
        	key_event.event |= EPOLLIN;
	        key_event.event |= EPOLLET;
        	key_event.arg = NULL;
	        FUNC callback = std::bind( &Keyboard::keyCallBack, this, _1, _2 );
        	key_event.callback = callback;

	        event_base.addEvent( key_event );
	}

	void spin()
	{
		while(true){
        	        event_base.dispatcher();
 	       	}

	}

	void* keyCallBack( int fd, void* arg )
	{
		struct input_event t;
		if( read( key_fd, &t, sizeof( t ) ) == sizeof( t ) ){

			for( auto& it : pressed_fun_map ){
				if( t.type == EV_KEY && t.code == it.first  ){
					if( t.value == 1 && !pressed_flag ){
//						std::cout<<"key "<<it.first<<" is be pressed !"<<std::endl;

						// key event process
						it.second();

						pressed_flag = true;
					}
					else if( !t.value ){
//						std::cout<<"key "<<it.first<<" is be released !"<<std::endl;
	
						released_fun_map[it.first]();
						
						pressed_flag = false;
					}
				}
			}
		}


		
	}

	template<typename ...Args>
	void registCallBackFunc(Args... args)
	{
        	static_assert(!(sizeof...(args) % 3), "The number of arguments is too few !");
        	registCallBackFuncHelper( args... );
	}

private:
	void registCallBackFuncHelper()
	{

	}

	template<typename T, typename ...Args>
	void registCallBackFuncHelper( T value, Args... args )
	{
        	if constexpr (std::is_integral<T>::value) {

	               	pressed_fun_map[value] = fun_vec[0];
			released_fun_map[value] = fun_vec[1];
 			fun_vec.clear();
	       	}
        	else if constexpr (std::is_same<T, CallBack>::value) {
                	fun_vec.push_back( value );
        	}

        	registCallBackFuncHelper( args... );
	}


private:
	
	bool setDispMode(const int fd, const int option)
	{
   		struct termios term;
   		if(tcgetattr(fd,&term)==-1){
     			std::cerr<<"Cannot get the attribution of the terminal"<<std::endl;
     			return true;
   		}
   		if( option )
        		term.c_lflag |= ECHOFLAGS;
   		else
        		term.c_lflag &= ~ECHOFLAGS;
	
   		int err = tcsetattr(fd, TCSAFLUSH, &term);
   		if(err == -1 && err == EINTR){
        		std::cout << "Cannot set the attribution of the terminal"<<std::endl;
        		return true;
   		}
   		return false;
	}

private:
	int key_fd = -1;
	
	EpollEvent event_base;

	bool pressed_flag = false;

	std::vector<CallBack> fun_vec;
	std::unordered_map<int, CallBack> pressed_fun_map;
	std::unordered_map<int, CallBack> released_fun_map;

};

/*class KeyboardControl
{
public:
	static void keyWPressed()
	{

	}
	static void keyWReleased()
	{

	}

	static void keyAPressd()
	{

	}
	static void keyAReleased()
	{

	}

	static void keyDPressed()
	{

	}
	static void keyDReleased()
	{

	}
	
	static void start()
	{
		keyboard.init();
		keyboard.registCallBackFunc( keyWPressed, keyWReleased, 17, keyAPressd, keyAReleased, 30, keyDPressed, keyDReleased, 32);

	        keyboard.spin();

	}
private:
		
	static Keyboard keyboard;
};

Keyboard KeyboardControl::keyboard;
*/
}

#endif
