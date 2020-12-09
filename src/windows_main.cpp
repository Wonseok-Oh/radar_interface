#include <winsock2.h>
#include <WS2tcpip.h>

#include "windows_SPxServerScanImgProcess.h"
#pragma comment(lib, "Ws2_32.lib")

#define BUFFER_LEN 262144
#define SERVERIP "192.168.30.150"
#define PORT 4388

int main(void){
	// initialization of scan conversion module modified on our own
	CSPxScanImgProc ScanImgProc;

	// Initializing Socketrelated variables
	WSADATA wsaData;
	SOCKET hSocket;
	SOCKADDR_IN servAddr;

	// Initialize socket library
	if(WSAStartup(MAKEWORD(2,2), &wsaData) != 0){
		printf("WSAStartup() error\n");
		exit(-1);
	}

	// Create UDP sockets
	hSocket = socket(AF_INET, SOCK_STREAM, 0); // socket construction
	if(hSocket == INVALID_SOCKET){
		printf("Socket error\n");
		exit(-1);
	}

	memset(&servAddr, 0, sizeof(servAddr)); // init   
	servAddr.sin_family = AF_INET;
	servAddr.sin_addr.s_addr = inet_addr(SERVERIP);
	servAddr.sin_port = htons(PORT);

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
		Mat img = ScanImgProc.hwnd2mat(ScanImgProc.m_spxBitmapWin->GetHBitmap(), ScanImgProc.m_spxBitmapWin->GetHDC());
		unsigned char *img_data = img.data;
		// For debug
		//Mat img = imread("./dog.png");
		imshow("Scanimg", img);
		
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

	closesocket(hSocket);
	WSACleanup();

	/* Never gets here. */
    exit(EXIT_SUCCESS);

	return 0;
}