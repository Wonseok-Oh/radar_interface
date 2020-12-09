/* Standard headers. */
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <math.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include "SPx.h"

/* X headers. */
#include <X11/Xlib.h>
#include <X11/extensions/Xcomposite.h>
#include <X11/extensions/XShm.h>
#include <X11/extensions/Xrender.h>
#include <X11/Intrinsic.h>
#include <X11/StringDefs.h>
#include <X11/Core.h>
#include <X11/Xaw/Box.h>
#include <X11/Xaw/Command.h>
#include <X11/Xaw/Form.h>
#include <X11/Xaw/AsciiText.h>
#include <X11/Xaw/Toggle.h>
#include <X11/Xaw/Scrollbar.h>
#include <X11/Shell.h>

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

/* debug */
#include <iostream>

#define SCANIMG_WIDTH 512
#define SCANIMG_HEIGHT 512

using namespace cv;

static void bitmapCallback(SPxScDestBitmap *bitmap, UINT16 changes, UINT16 startAzi, UINT16 endAzi, DirtyBox box);

/*
 * Constants.
 */
#define	USAGE "spxppi1 [options]\n"					\
		"\nOptions:\n"						\
		"\t-a <azi>\tSet azimuth spoke to insert\n"		\
		"\t-l <rgb>\tSet sweep line colour\n"			\
		"\t-t <pattern>\tSet the test pattern to show\n"	\
		"\t-u <msecs>\tSet the window update interval\n"	\
		"\t-v\t\tIncrease verbosity\n"				\
		"\t-?\t\tPrint usage information.\n\n"


/* Initial view centre and scale. */
#define START_VIEW_W	200000		/* Metres */
#define START_VIEW_H	200000		/* Metres */
#define START_VIEW_X	0
#define START_VIEW_Y	0

/* Maximum window size supported by this demo. */
#define	MAX_WIN_WIDTH	1600
#define	MAX_WIN_HEIGHT	1200

/* Define the thumb size of a scroll bar as a percentage of the total size. */
#define SCROLLBAR_WIDTH_PERCENT 10
/*
 * Private function prototypes.
 */
/* Creation/setup and shutdown functions. */
static void sigIntHandler(int sig);
static void createUserInterface(int *argcPtr, char **argvPtr);
static void initialiseCompositing(void);
static XImage *CreateImage(void);
static void quitApplication(Widget w, XtPointer client, XtPointer call);

/* Graphics handling functions. */
static void exposureEvent(Widget w, XtPointer client, XEvent *ev, Boolean*);
static void DrawUnderlayPixmap(void);
static void DrawOverlayPixmap(void);
static void ChangeWindowView(double newViewX, double newViewY,
				double newViewW, double newViewH,
				int showWindow);
static void UpdateDisplay(XtPointer closure, XtIntervalId *id);

/* Scroll and zoom handlers. */
static void scrollRight(Widget w, XtPointer client, XtPointer call);
static void scrollLeft(Widget w, XtPointer client, XtPointer call);
static void scrollUp(Widget w, XtPointer client, XtPointer call);
static void scrollDown(Widget w, XtPointer client, XtPointer call);
static void recentre(Widget w, XtPointer client, XtPointer call);
static void zoomIn(Widget w, XtPointer client, XtPointer call);
static void zoomOut(Widget w, XtPointer client, XtPointer call);

/* Source selection handlers */
static void selectTestGenerator(Widget w, XtPointer client, XtPointer call);
static void selectNetworkSource(Widget w, XtPointer client, XtPointer call);

/* Fade control handlers.*/
static void selectSweepFade(Widget w, XtPointer client, XtPointer call);
static void selectRealTimeFade(Widget w, XtPointer client, XtPointer call);
static void selectNoFade(Widget w, XtPointer client, XtPointer call);
static void changeFadeTime(Widget w, XtPointer, XtPointer);
static void jumpFadeTime(Widget w, XtPointer, XtPointer);
static void SetNewFade(void);

/* Utility functions. */
static Boolean isButtonActive(Widget w);
static void setButtonActive(Widget w, Boolean state);

/*
 * Private variables.
 */
/* Verbosity flag.*/
static int Verbose = 0;

/* Current view centre and scale. */
static double Vx = START_VIEW_X;
static double Vy = START_VIEW_Y;
static double Vw = START_VIEW_W;
static double Vh = START_VIEW_H;

/* Size of window in pixels. */
static int WinW = 768, WinH = 768;

/* Handle of our scan converter object. */
static SPxScSourceLocal *ScanConverter = NULL;

/* The SPx Bitmap class used to receive video from the scan converter */
static SPxScDestBitmap *DestBitmap = NULL;

/* The X Image used to hold the client-side radar bitmap. */
static XImage *RadarImage;

/* The current fade length is maintained as a number from 1
 * to 100. This is then converted to a fade time, depending on the
 * fade mode.
 */
static int CurrentFadeLength = 25;

/* Holds the current radar fade mode. */
static int CurrentFade = SPX_RADAR_FADE_REAL_TIME;

/* High-level widgets etc. */
static Display *XDisplay;
static int XScreen;
static GC UnderlayGC=0, OverlayGC=0, RadarGC=0;
static Pixmap UnderlayPixmap, OverlayPixmap, CombinePixmap, RadarPixmap;
static Picture UnderlayPicture, OverlayPicture, CombinePicture, RadarPicture;
static Visual *ArgbVisual = 0;
static XShmSegmentInfo RadarImageSHMinfo;
static Widget Drawing;

/* Control widgets. */
static Widget upButton, downButton, leftButton, rightButton, quitButton;
static Widget zoomInButton, zoomOutButton;
static Widget recentreButton;
static Widget testButton, networkButton;
static Widget sweepFadeButton, realtimeFadeButton, noFadeButton;
static Widget fadeTimeControl, fadeTimeLabel;
static Widget rangeScaleLabel;

/* Display update period in ms */
static int UpdatePeriod = 40;		/* Milliseconds. */

/* X Windows Application Context */
static XtAppContext AppContext;

class CSPxScanImgProc
{
public:
	CSPxScanImgProc(int argc, char **argv);
	~CSPxScanImgProc();

	Mat radar2mat(unsigned char *radar_data);

	SPxRIB *m_spxRib;
	SPxPIM *m_spxPim;
	SPxScDestBitmap *m_spxScDestBitmap;
	SPxRunProcess *m_spxScProcess;
	Mat scanImg;

};
