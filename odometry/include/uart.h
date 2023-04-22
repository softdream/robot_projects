#ifndef __UART_H
#define __UART_H

#include <stdio.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>

#include <iostream>


namespace uart
{

class Uart
{
public:
	Uart()
	{
		initUart( "/dev/ttyS3" );
	}

	Uart( const std::string& device_path )
	{
		initUart( device_path );
	}

	~Uart()
	{
	
	}

	int readData( char* buffer, int len )
	{
		return read( fd, buffer, len );
	}

	void closeDevice()
	{
		close( fd );
	}

	template<typename T>
	int writeData( T&& data )
	{
		return write( fd, &data, sizeof( data ) );
	}

	int getFd() const
        {
                return fd;
        }



private:
	bool initUart( const std::string& device_path )
	{
		//fd = open( device_path.c_str(), O_RDWR | O_NOCTTY | O_NDELAY );
		fd = open( device_path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK );
		if ( fd == -1 ) {
			std::cerr<<"Error in Opening TTY !"<<std::endl;
			return false;
		}
		else {
			std::cout<<"Open the TTY device !"<<std::endl;
		}

		struct termios SerialPortSettings;
		tcgetattr(fd, &SerialPortSettings); // Get the current attributes of the Serial port
		
		cfsetispeed(&SerialPortSettings, B230400); // Set Read Speed as 230400
		cfsetospeed(&SerialPortSettings, B230400); // Set Write Speed as 230400

		SerialPortSettings.c_cflag &= ~PARENB; // Disables the Parity Enable bit(PARENB),So No Parity
		SerialPortSettings.c_cflag &= ~CSTOPB; // CSTOPB = 2 Stop bits,here it is cleared so 1 Stop bit
		SerialPortSettings.c_cflag &= ~CSIZE; // Clears the mask for setting the data size
		SerialPortSettings.c_cflag |=  CS8; // Set the data bits = 8
		SerialPortSettings.c_cflag &= ~CRTSCTS; // No Hardware flow Control
		SerialPortSettings.c_cflag |= CREAD | CLOCAL; // Enable receiver,Ignore Modem Control lines
		SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY); // Disable XON/XOFF flow control both i/p and o/p
		SerialPortSettings.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); //  Non Cannonical mode
		SerialPortSettings.c_oflag &= ~OPOST; // No Output Processing   raw  format  output
		SerialPortSettings.c_cc[VMIN] = 0; // Read at least 10 characters
		SerialPortSettings.c_cc[VTIME] = 1; // Wait Indefinetly

		if ( (tcsetattr( fd, TCSANOW, &SerialPortSettings)) != 0 ) {
			std::cerr<<"ERROR ! in Setting attributes"<<std::endl;
			return false;
		}
		else {
			std::cout<<"BaudRate = 230400, StopBit = 1, Parity = none !"<<std::endl;
		}

		tcflush(fd, TCIFLUSH); // Discards old data in the rx buffer

		return true;
	}

private:
	int fd = -1;
};

}

#endif
