#include "librtmp_send264.h"
#include <iostream>
#include <stdio.h>

FILE* fp_send = NULL;

int readBuffer(unsigned char *buf, int buf_size ){
        if( !feof( fp_send ) ) {
                int true_size = fread( buf, 1, buf_size, fp_send );
                return true_size;
        }
	else {
                return -1;
        }
}


int main()
{
	std::cout<<"----------------------- RTMP TEST -----------------------"<<std::endl;

	fp_send = fopen( "/groupdata/share/ddf/Test/robot_test/robot_projects/rtmp_stream/test_data/cuc_ieschool.h264", "rb" );

	RTMP264_Connect("rtmp://42.192.222.24:1935/live/stream");

	RTMP264_Send( readBuffer );

	RTMP264_Close();


	return 0;
}
