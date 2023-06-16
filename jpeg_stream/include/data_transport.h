#ifndef __DATA_TRANSPORT_H
#define __DATA_TRANSPORT_H

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

namespace transport
{

template<int BUFFER_SIZE>
class UdpServer
{
public:
        UdpServer()
        {
		initUdpServer();
		setNonBlock();
        }

        ~UdpServer()
        {

        }

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

        const int getSocketFd() const
        {
                return sock_fd;
        }

        const int read()
        {
                memset( recv_buffer, 0, sizeof( recv_buffer ) );

                int ret = ::recvfrom(sock_fd, recv_buffer, sizeof( recv_buffer ), 0 , (struct sockaddr*)&client_sock_addr, &client_sock_len);

                if( ret <= 0 ){
                        std::cerr<<"recv failed : "<<ret<<std::endl;

                }
		else {
			std::cout<<recv_buffer<<std::endl;
		}

                return ret;
        }

        const int write( const char* data, const int len )
        {
                int ret = ::sendto( sock_fd, data, len, 0, (struct sockaddr*)&client_sock_addr, client_sock_len );

                if( ret <= 0 ){
                        std::cerr<<"send failed : "<<ret<<std::endl;
                }

                return ret;
        }

	void setNonBlock()
	{
		int flag = fcntl(sock_fd, F_GETFL, 0);  
	    	if (flag < 0) {
        		std::cerr<<"fcntl F_GETFL fail"<<std::endl;
        		return;
    		}
    		if (fcntl(sock_fd, F_SETFL, flag | O_NONBLOCK) < 0) {  
        		std::cerr<<"fcntl F_SETFL fail"<<std::endl;

		}
	}

protected:
        // server info
        int sock_fd = -1;
        struct sockaddr_in sock_server_addr;

        // remote client info
        struct sockaddr_in client_sock_addr;
        socklen_t client_sock_len ;

        // receive buffer
        char recv_buffer[BUFFER_SIZE];
};

class UdpClient
{
public:
	UdpClient()
	{
		initUdpClient();
	}

	~UdpClient()
	{

	}

	bool initUdpClient()
	{
		sock_fd = socket( AF_INET, SOCK_DGRAM, 0 );
		if( sock_fd < 0 ){
			std::cerr<<"socket UDP client falied ..."<<std::endl;
	                return false;
		}
		std::cout<<"initialize the udp client successfull: "<< sock_fd <<std::endl;
	
		return true;
	}
	
	const int write( const char* data, const int len )
	{
		struct sockaddr_in dest_addr;
        	dest_addr.sin_family = AF_INET;
        	dest_addr.sin_addr.s_addr = inet_addr("127.0.0.1");
	        dest_addr.sin_port = htons( 3333 );

        	int ret = sendto( sock_fd, data, len, 0, (struct sockaddr*)&dest_addr, sizeof( dest_addr ) );
		if( ret <= 0 ){
                	std::cerr<<"send data falied ..."<<std::endl;
			return false;
        	}
        	else {
                	std::cerr<<"send data succussfully ..."<<ret<<std::endl;
        	}
		return ret;
	}
	
	const int write( const char* data, const int len, const int port )
        {
                struct sockaddr_in dest_addr;
                dest_addr.sin_family = AF_INET;
                dest_addr.sin_addr.s_addr = inet_addr("127.0.0.1");
                dest_addr.sin_port = htons( port );

                int ret = sendto( sock_fd, data, len, 0, (struct sockaddr*)&dest_addr, sizeof( dest_addr ) );
                if( ret <= 0 ){
                        std::cerr<<"send data falied ..."<<std::endl;
                        return false;
                }
                else {
                        std::cerr<<"send data succussfully ..."<<std::endl;
                }
                return ret;
        }

	const int write( const char* data, const int len, const int port, const std::string& ip )
        {
                struct sockaddr_in dest_addr;
                dest_addr.sin_family = AF_INET;
                dest_addr.sin_addr.s_addr = inet_addr( ip.c_str() );
                dest_addr.sin_port = htons( port );

                int ret = sendto( sock_fd, data, len, 0, (struct sockaddr*)&dest_addr, sizeof( dest_addr ) );
                if( ret <= 0 ){
                        std::cerr<<"send data falied ..."<<std::endl;
                        return false;
                }
                else {
                        std::cerr<<"send data succussfully ..."<<std::endl;
                }
                return ret;
        }

protected:
	int sock_fd = -1;

};


/*class Receiver : public UdpServer<50>
{
public:
	Receiver()
	{

	}

	~Receiver()
	{

	}

	template<typename T>
	void recive( T& data )
	{
		this->read(  );
	
		memcpy( &data, recv_buffer, sizeof( data ) );
	}

};
*/

template<typename T>
class Receiver : public UdpServer<50>
{
public:
        Receiver()
        {

        }

        ~Receiver()
        {

        }

        bool recive( T& data )
        {
                this->read(  );
        
		T curr_data;
                memcpy( &curr_data, recv_buffer, sizeof( curr_data ) );
	
		if( !curr_data.isSame( pre_data ) ){ // the value has been changed
			data = curr_data;
			pre_data = curr_data;
			return true;
		}
		else {
			return false;
		}
        }

private:
	T pre_data;
};


class Sender : public UdpClient
{
public:
	Sender()
	{

	}
	
	Sender( const std::string& dst_ip, const int dst_port ) : ip_( dst_ip ), port_( dst_port )
	{

	} 

	~Sender()
	{

	}

	void setDstIp( const std::string& dst_ip ) 
	{
		ip_ = dst_ip;
	} 

	void setDstPort( const int dst_port )
	{
		port_ = dst_port;
	} 

	template<typename T>
	int send( const std::vector<T>& vec )
	{
		return this->write( (char *)vec.data(), vec.size() * sizeof( T ), port_, ip_ );
	}

	// SFINAE
	template<typename T, typename = typename std::enable_if<!std::is_same<T, std::vector<T, std::allocator<T>>>::value>::type>
	int send( const T& data )
	{
		return this->write( (char *)&data, sizeof( data ), port_, ip_ );
	}

	template<typename T>
        int send( const int port, const std::string& ip, T&& data )
        {
                return this->write( (char *)&data, sizeof( data ), port, ip );
        }

	int send( char* buffer, int len, const std::string& ip, const int port )
	{
		return this->write( buffer, len, port, ip );
	}

	int send( char* buffer, int len )
	{
		return this->write( buffer, len, port_, ip_ );
	}

private:
	std::string ip_;
	int port_ = -1;	
};

}

#endif
