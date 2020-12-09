#include <stdio.h>
#include <stdlib.h>
#include "SPx.h"
#include <windows.h>

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#define SCANIMG_WIDTH 512
#define SCANIMG_HEIGHT 512

using namespace cv;

static void bitmapCallback(SPxScDestBitmap *bitmap, UINT16 changes, UINT16 startAzi, UINT16 endAzi, DirtyBox box);

class CSPxScanImgProc
{
public:
	CSPxScanImgProc();
	~CSPxScanImgProc();

	Mat hwnd2mat(HBITMAP hBitmap, HDC hDC);

	SPxRemoteServer *m_svr; 
	SPxNetworkReceive *m_src;
	SPxNetworkReceiveP226 *m_src226;
	SPxRIB *m_spxRib;
	SPxPIM *m_spxPim;
	SPxBitmapWin *m_spxBitmapWin;
	SPxScDestBitmap *m_spxScDestBitmap;
	SPxRunProcess *m_spxScProcess;
	Mat scanImg;

};
