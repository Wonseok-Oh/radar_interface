#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <netdb.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <iostream>

#include "SPxServerScanImgProcess.h"
#pragma comment(lib, "Ws2_32.lib")

#define SERVERIP "192.168.30.150"

#define MYPORT "4388" // the port client will be connecting to
#define MAXBUFLEN 262144 // max number of bytes we can get at once

int main(int argc, char **argv){
	// initialization of scan conversion module modified on our own
	CSPxScanImgProc ScanImgProc(argc, argv);
    std::cout << "heww" << std::endl;
/*	memset(&servAddr, 0, sizeof(servAddr)); // init
	servAddr.sin_family = AF_INET;
	servAddr.sin_addr.s_addr = inet_addr(SERVERIP);
	servAddr.sin_port = htons(PORT);*/

	//////////////////////////
	// PLEASE READ BELOW!!! //
	//////////////////////////
	// If you want to test this program in an offline mode by replaying the recorded radar source,
	// please comment below 
	/*if(connect(hSocket, (SOCKADDR*)&servAddr, sizeof(servAddr)) == SOCKET_ERROR){
		printf("connection error\n");
		exit(-1);
	}*/

	// For debug
	//namedWindow("Scanimg", WINDOW_AUTOSIZE);

	while(1){
		// For every 100ms, call the function to convert scan images into cv::Mat variables
		Mat img = ScanImgProc.radar2mat((unsigned char*)RadarImage->data);
		std::cout << "radar2mat passed " <<std::endl;
		unsigned char *img_data = img.data;
		// For debug
		//Mat img = imread("./dog.png");
		//imshow("Scanimg", img);
		
		//////////////////////////
		// PLEASE READ BELOW!!! //
		//////////////////////////
		// If you want to test this program in an offline mode by replaying the recorded radar source,
		// please comment below 
		//send(hSocket, (char *)img.data, BUFFER_LEN*4, 0);

		// More sophisticated timer is needed (We can't use MMTimer and others. Upgrading VSinterface is necessary)
		cv::waitKey(100);

		/* Don't busy loop. */
        //SPxTimeSleepMsecs(1000);
	}

	//closesocket(hSocket);

	/* Never gets here. */
    exit(EXIT_SUCCESS);

	return 0;
}
