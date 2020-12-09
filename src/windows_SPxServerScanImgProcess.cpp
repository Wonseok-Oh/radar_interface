#include "windows_SPxServerScanImgProcess.h"

CSPxScanImgProc::CSPxScanImgProc(){
	/* SPxNetworkReceive -> SPxRiB -> SPxPIM -> SPxScSourceLocal -> SPxScDestBitmap */

	/* Initialise the SPx libraries. */
    if( SPxInit() != SPX_NO_ERROR ){
		printf("Failed to initialise SPx.\n");
		exit(-1);
    }

	/* Initialise the SPx run-time license. */
    if( SPxLicInit() != SPX_NO_ERROR ){
		printf("Failed to initialise SPx license.\n");
		exit(-1);
    }
	/* Create the bitmap pointer using a windows HBITMAP */
	m_spxBitmapWin = new SPxBitmapWin();
	m_spxBitmapWin->CreateLocal(SCANIMG_WIDTH, SCANIMG_HEIGHT);
	/* Create the bitmap, which is the destination for the scan converter */
	m_spxScDestBitmap = new SPxScDestBitmap();
	m_spxScDestBitmap->Create(SCANIMG_WIDTH, SCANIMG_HEIGHT, SPX_BITMAP_32BITS,(unsigned char *)m_spxBitmapWin->GetBufferPtr());
	/* Register the function that will be called when the bitmap is updated. */
    m_spxScDestBitmap->SetNotifyUpdate(bitmapCallback);

	/* Create the scan converter */
	SPxScSourceLocal *spxSc = new SPxScSourceLocal(m_spxScDestBitmap);
	spxSc->SetWinGeom(0,0,SCANIMG_WIDTH,SCANIMG_HEIGHT);

	/* Create the RIB. Size is 1Mb, with memory allocation done by class. */
    m_spxRib = new SPxRIB(1024*1024, NULL);

    /* Create the PIM to provide the polar storage allowing for a maximum of 2 bytes per sample to support 16-bit radar video. */
    m_spxPim = new SPxPIM(m_spxRib, 2048, 2048, SPX_PIM_RAN_SUBSAMPLE, SPX_PIM_AZI_SUBSAMPLE, SPX_PIM_OUTPUT(1));

	/* Create the network receiver to spxRIB (for general cases) */
	m_src = new SPxNetworkReceive(m_spxRib);
	m_src->Create("239.192.43.78", 4378);
	m_src->Enable(TRUE) ; // ��Ʈ��ũ���� ������ �ޱ� ����

	/* Create the network receiver from SIMRAD 4G radar (proj code 226) to spxRIB */
	//m_src226 = new SPxNetworkReceiveP226(m_spxRib);
	//m_src226->SetSerialNumber("1511403102");
	//m_src226->SetLicenseKey("BEC926FA00A043D41109E65BB457BC30CDC8CD310EC1FCBC71BD55BEDCFEF7FC8057236B61B89D5F1BB6172C362003D932A09272CACBD47CB40455721937BC46");
	//m_src226->Create();
	//m_src226->Enable(TRUE);

	/* Create a process that connects the scan converter to the PIM */
	m_spxScProcess = new SPxRunProcess(SPxProScanConv, NULL, m_spxPim, spxSc);

	scanImg.create(SCANIMG_HEIGHT, SCANIMG_WIDTH, CV_8UC4);

	/* Create a test generator input FOR TEST */
	//SPxTestGenerator *spxTest = new SPxTestGenerator(m_spxRib, 1024, 4.0, 200, 5, 0);
	//spxTest->Enable(1);	
}

CSPxScanImgProc::~CSPxScanImgProc(){
	SPxDeleteAllSystem();
}

void bitmapCallback(SPxScDestBitmap *bitmap, UINT16 changes, UINT16 startAzi, UINT16 endAzi, DirtyBox box){
	/* This function gets called as a notification that new data is available in the bitmap and/or the bitmap configuration has changed. */
	
	//printf("New data: pos = (%d,%d), sz = (%d,%d), for %.1f to %.1f deg\n", box.x, box.y, box.w, box.h,	((double)startAzi) * 360.0 / 65536.0, ((double)endAzi) * 360.0 / 65536.0); 
	
	/* Clear the dirty box. */
	//bitmap->ClearDirtyBox();

	// For every 20ms, check whether the received data has any change or not
}

Mat CSPxScanImgProc::hwnd2mat(HBITMAP hBitmap, HDC hDC){
	Mat src;
	HDC hwindowCompatibleDC;
	HBITMAP hbwindow;
    BITMAPINFOHEADER  bi;

    hwindowCompatibleDC=CreateCompatibleDC(hDC);
    SetStretchBltMode(hwindowCompatibleDC,COLORONCOLOR);

    src.create(SCANIMG_HEIGHT,SCANIMG_WIDTH,CV_8UC4);

    // create a bitmap
    hbwindow = CreateCompatibleBitmap( hDC, SCANIMG_WIDTH, SCANIMG_HEIGHT);
    bi.biSize = sizeof(BITMAPINFOHEADER);    //http://msdn.microsoft.com/en-us/library/windows/window/dd183402%28v=vs.85%29.aspx
    bi.biWidth = SCANIMG_WIDTH;    
    bi.biHeight = -SCANIMG_HEIGHT;  //this is the line that makes it draw upside down or not
    bi.biPlanes = 1;
    bi.biBitCount = 32;
    bi.biCompression = BI_RGB;
    bi.biSizeImage = 0;  
    bi.biXPelsPerMeter = 0;    
    bi.biYPelsPerMeter = 0;    
    bi.biClrUsed = 0;    
    bi.biClrImportant = 0;

    // use the previously created device context with the bitmap
    SelectObject(hwindowCompatibleDC, hbwindow);
    // copy from the window device context to the bitmap device context
    StretchBlt( hwindowCompatibleDC, 0,0, SCANIMG_WIDTH, SCANIMG_HEIGHT, hDC, 0, 0,SCANIMG_WIDTH,SCANIMG_HEIGHT, SRCCOPY); //change SRCCOPY to NOTSRCCOPY for wacky colors !
    GetDIBits(hwindowCompatibleDC,hbwindow,0,SCANIMG_HEIGHT,src.data,(BITMAPINFO *)&bi,DIB_RGB_COLORS);  //copy from hwindowCompatibleDC to hbwindow

	scanImg = src;

    // avoid memory leak
    DeleteObject (hbwindow); DeleteDC(hwindowCompatibleDC);

    return src;
}   