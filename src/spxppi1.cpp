/*********************************************************************
*
* (c) Copyright 2008 - 2015, Cambridge Pixel Ltd.
*
* File: $RCSfile: spxppi1.cpp,v $
* ID: $Id: spxppi1.cpp,v 1.7 2015/02/20 14:06:12 rew Exp $
*
* Purpose:
*	SPx PPI example for X Windows.
*
*	This example shows how to use the SPxScSourceLocal and
*	SPxScDestBitmap objects to create a PPI window with
*	application-compositing of the radar and graphics.
*
*	It does not require any compositing window manager.  For best
*	performance it should be run without one.
*
*	The X Server must be running on the same machine as the one
*	running this application.  It must provide the SHM and RENDER
*	extensions.
*
*
* Revision Control:
*   20/02/15 v1.7    AGC	Fix clang warning.
*
* Previous Changes:
*   21/08/12 1.6    AGC	Fix clang warnings.
*   12/06/12 1.5    AGC	Fix 64-bit compilation error.
*   21/09/11 1.4    REW	Fix scale reading on GUI.
*   18/08/11 1.3    REW	Add -a for azimuth spoke.
*   07/12/09 1.2    REW	Add -l for sweep line.
*   02/09/08 1.1    DGJ	Initial Version.
**********************************************************************/

/* Standard headers. */
#include <iostream>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <math.h>
#include <sys/ipc.h>
#include <sys/shm.h>

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

/* SPx Library headers. */
#include "SPx.h"

/* ROS Library */
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
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
#define START_VIEW_W	512//1000		/* Metres */
#define START_VIEW_H	512//1000		/* Metres */
#define START_VIEW_X	0			
#define START_VIEW_Y	0

/* Maximum window size supported by this demo. */
#define	MAX_WIN_WIDTH	512//1600
#define	MAX_WIN_HEIGHT	512//1200

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

static void bitmapCallback(SPxScDestBitmap *bitmap, UINT16 changes,
                             UINT16 startAzi, UINT16 endAzi, DirtyBox box);

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
static int WinW = 512, WinH = 512;

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

/* SPx source objects */
static SPxTestGenerator *SrcTpg = NULL;
static SPxNetworkReceive *SrcNet = NULL;

static int counter = 0;
static ros::Publisher* imgPubPtr = NULL;

/*********************************************************************
*
*	Public functions
*
**********************************************************************/

/*====================================================================
*
* main
*	Main entry point for program.
*
* Params:
*	argc, argv	Standard C arguments.
*
* Returns:
*	Zero on success,
*	Error code otherwise.
*
* Notes:
*
*===================================================================*/
int main(int argc,char **argv)
{
	ros::init (argc, argv, "radar_interface_node");
	ros::NodeHandle n;
	imgPubPtr = new ros::Publisher;
	*imgPubPtr = n.advertise<sensor_msgs::Image>("radar_image", 1);

	int c;			/* For parsing command line options */
    int aziSpoke = -1;		/* Azimuth spoke to insert */
    int testPattern = 102;	/* Test pattern to show */
    UINT32 sweepLineRGB = 0;	/* Sweep line colour */

    /* Install our tidy-up function for Ctrl-C. */
    (void)signal(SIGINT, sigIntHandler);

    /*
     * Process any command line arguments.
     */
    opterr = 0;
    while( (c = getopt(argc, argv, "a:l:t:u:v?")) != -1 )
    {
	switch(c)
	{
	    case 'a':	aziSpoke = strtol(optarg, NULL, 0);break;
	    case 'l':	sweepLineRGB = strtoul(optarg, NULL, 0);break;
	    case 't':	testPattern = strtol(optarg, NULL, 0);	break;
	    case 'u':	UpdatePeriod = strtol(optarg, NULL, 0);	break;
	    case 'v':	Verbose++;				break;
	    case '?':	/* fall through */
	    default:
		fprintf(stderr, "\n%s", USAGE);
		return(-1);
	}
    } /* end of for each option */

    /*
     * Print a banner and set up verbose reporting.
     */
    printf("\n### Cambridge Pixel SPx PPI Demo (%s)\n", SPX_VERSION_STRING);
    printf("\nThis demo requires the following servers to be running:\n"
		"\tA local X Server with SHM and COMPOSITE extensions.\n"
		"\tNo compositing window manager is required.\n\n");

    /*
     * Initialise the SPx libraries.
     */
    if( SPxInit(Verbose) != SPX_NO_ERROR )
    {
	printf("Failed to initialise SPx.\n");
	exit(-1);
    }

    /*
     * Create the user-interface.
     */
    createUserInterface(&argc, argv);

    /* Save the X Display and Screen */
    XDisplay = XtDisplay(Drawing);
    XScreen  = XScreenNumberOfScreen(XtScreen(Drawing));
	
    /*
     * Initialise everything we need for combining radar with graphics.
     */
    initialiseCompositing();

    /*
     * Create the SPx objects etc.
     */ 
    /* We need a bitmap object mapping onto the RadarImage shared memory. */
    DestBitmap = new SPxScDestBitmap();
    DestBitmap->Create(MAX_WIN_WIDTH, MAX_WIN_HEIGHT, SPX_BITMAP_32BITS, 
			    (unsigned char*)RadarImage->data,
			    RadarImage->bytes_per_line);

    /* The DestBitmap class needs to know how often we'll update the display,
     * since it needs to calculate fades time.
     */
    DestBitmap->SetUpdateInterval(UpdatePeriod);
    DestBitmap->SetNotifyUpdate(bitmapCallback);

    /* Create the scan converter, giving it the bitmap destination. */
    ScanConverter = new SPxScSourceLocal(DestBitmap, SPX_SC_TYPE_PPI);

    /* Create the RIB, PIM, source, processes etc. */
    SPxRunProcess *spxProcess = NULL;
    int numSamples = 2048;
    int numAzimuths = 2048;
    SPxRIB *spxRib = new SPxRIB(numSamples * numAzimuths, NULL);
    SPxPIM *spxPim = new SPxPIM(spxRib, numSamples, numAzimuths,
					SPX_PIM_RAN_PEAK, SPX_PIM_AZI_PEAK,
					SPX_PIM_OUTPUT(1));
    if( aziSpoke >= 0 )
    {
	spxProcess = new SPxRunProcess(SPxProAziSpokes, spxProcess, spxPim);
	spxProcess->SetParamValueI("Azimuth", aziSpoke);
    }
    spxProcess = new SPxRunProcess(SPxProScanConv, spxProcess,
					spxPim, ScanConverter);
    SrcTpg = new SPxTestGenerator(spxRib, numSamples, 2.0, 1000,
					testPattern ? testPattern : 102, 0);

    /* Create the network receiver to spxRIB (for general cases) */
   	SrcNet = new SPxNetworkReceive(spxRib);
   	SrcNet->Create("239.192.43.78", 4378);

    /* Choose initial source. */
    if( testPattern > 0 )
    {
	SrcTpg->Enable(TRUE);
	setButtonActive(testButton, TRUE);
    }
    else
    {
	SrcNet->Enable(TRUE);
	setButtonActive(networkButton, TRUE);
    }

    /* Initialise the slider and scan-converter's fade 
     * to the default fade level.
     */
    SetNewFade();
    
    /* Initialise the scan converter geometry and the view in the window. */
    ScanConverter->SetWinGeom(0, 0, WinW, WinH);
    ChangeWindowView(Vx, Vy, Vw, Vh, 0);    
    ScanConverter->SetView(Vx,Vy,Vw,Vh);   

    /* Initialise the sweep line if requested. */
    if( sweepLineRGB != 0 )
    {
	ScanConverter->SetSweepLine(SPX_SC_SWEEPLINE_TYPE_AUTO,
			(SPxScSweepLineColMode_t)((sweepLineRGB >> 24) & 0xff),
			sweepLineRGB & 0xFFFFFF, 0, 0);
    }

    /* The screen updates are driven from a timer. Start this now. */
    XtAppAddTimeOut(AppContext, UpdatePeriod, UpdateDisplay, NULL);

    /* Start event loop. */
    printf("Starting main loop...\n");
    XtAppMainLoop(AppContext);
    return 0;
} /* main() */

/*********************************************************************
*
*	Application creation/setup/shutdown functions
*
**********************************************************************/

/*====================================================================
*   
* sigIntHandler
*	Handler function for SIGINT (i.e. Ctrl-C).
*
* Params:
*	sig	Signal we are being called for.
*
* Returns:
*	Nothing
*
* Notes:
*	Exit tidily.
*
*===================================================================*/
static void sigIntHandler(int sig)
{
    printf("\nSIGINT received - exiting application.\n");
    XShmDetach(XDisplay, &RadarImageSHMinfo);
    XDestroyImage(RadarImage);

    /* Detach the shared memory segment.  We don't need to do a
     * shmctl() here to delete it because we should have done that
     * when we first created and attached to it.  The delete takes
     * effect when the last process detaches from it, i.e. now.
     */
    shmdt(RadarImageSHMinfo.shmaddr);

    exit(EXIT_SUCCESS);
} /* sigIntHandler() */


/*====================================================================
*
* createUserInterface
*	Function to create the X application window, widgets and so on.
*
* Params:
*	argcPtr, argvPtr	Pointers to command line args.
*
* Returns:
*	Nothing (exits on failure).
*
* Notes:
*	Sets up various widgets.
*
*===================================================================*/
static void createUserInterface(int *argcPtr, char **argvPtr)
{
    Widget toplevel, box, controlArea;

    /* Initialise the X library. */
    toplevel = XtOpenApplication(&AppContext, "SPxPPI1", NULL, 0, 
					argcPtr, argvPtr, NULL,
					applicationShellWidgetClass, NULL, 0);


    /* Create the top level widgets. */
    box = XtVaCreateManagedWidget("form", formWidgetClass, toplevel,
    				XtNresizable, False,
    				XtNleft, XawChainLeft,
    				XtNright, XawChainLeft,
					NULL, (char*)0);
					
    controlArea =  XtVaCreateManagedWidget("box", formWidgetClass, box, 
					XtNbottom, XawChainTop, 
					XtNtop, XawChainTop,
					XtNleft, XawChainLeft,
					XtNright, XawChainLeft,
					XtNresizable, False,
					XtNwidth, 500,
					(char*)0);

    /* Create a Quit button. */
    quitButton = XtCreateManagedWidget("Quit", commandWidgetClass, controlArea,
					NULL, 0);
    XtAddCallback(quitButton, XtNcallback, quitApplication, NULL);

    /* Create an area for the view controls. */
    Widget viewArea =  XtVaCreateManagedWidget("box",
					formWidgetClass, controlArea, 
					XtNbottom, XawChainTop, 
					XtNtop, XawChainTop,
					XtNwidth, 500,
					XtNfromHoriz, quitButton,
					(char*)0);
    
    Widget viewLabel = XtVaCreateManagedWidget("View",
					labelWidgetClass, viewArea,
					XtNforeground, 0x5050b8,
					XtNborderWidth, 0,
					(char*)0);

    /* Create the scroll and zoom buttons. */
    leftButton = XtVaCreateManagedWidget("<", commandWidgetClass, viewArea, 
					XtNfromHoriz, viewLabel,
					XtNhorizDistance, 4,
					XtNfromVert, viewLabel,
					XtNvertDistance, 4,
					XtNwidth, 20,
					(char*)0);
					
    XtAddCallback(leftButton, XtNcallback, scrollLeft, NULL);
    
    upButton = XtVaCreateManagedWidget("^", commandWidgetClass, viewArea, 
					XtNfromHoriz, leftButton,
					XtNfromVert, NULL,
					XtNvertDistance, 4,
					XtNhorizDistance, 4,
					XtNwidth, 20,
					(char*)0);	
   XtAddCallback(upButton, XtNcallback, scrollUp, NULL);
    
   recentreButton = XtVaCreateManagedWidget("=", commandWidgetClass, viewArea, 
					XtNfromHoriz, leftButton,
					XtNhorizDistance, 4,
					XtNfromVert, viewLabel,
					XtNvertDistance, 4,
					XtNwidth, 20,
					(char*)0);
					
    XtAddCallback(recentreButton, XtNcallback, recentre, NULL);
    downButton = XtVaCreateManagedWidget("v", commandWidgetClass, viewArea, 
					XtNfromHoriz, leftButton,
					XtNhorizDistance, 4,
					XtNfromVert, recentreButton,
					XtNvertDistance, 4,
					XtNwidth, 20,
					(char*)0);
    XtAddCallback(downButton, XtNcallback, scrollDown, NULL);

    rightButton = XtVaCreateManagedWidget(">", commandWidgetClass, viewArea, 
					XtNfromHoriz, recentreButton,
					XtNhorizDistance, 4,
					XtNfromVert, viewLabel,
					XtNvertDistance, 4,
					XtNwidth, 20,
					(char*)0);
    XtAddCallback(rightButton, XtNcallback, scrollRight, NULL);

    zoomInButton = XtVaCreateManagedWidget("Zoom +",
					commandWidgetClass, viewArea, 
					XtNfromHoriz, rightButton,
					XtNfromVert, NULL,
					XtNvertDistance, 4, 
					(char*)0);
    XtAddCallback(zoomInButton, XtNcallback, zoomIn, NULL);

    zoomOutButton = XtVaCreateManagedWidget("Zoom -",
					commandWidgetClass, viewArea, 
					XtNfromVert, zoomInButton,
					XtNfromHoriz, rightButton,
					XtNvertDistance, 4, 
					(char*)0);
    XtAddCallback(zoomOutButton, XtNcallback, zoomOut, NULL);

 	
    rangeScaleLabel = XtVaCreateManagedWidget("SCALE",
					labelWidgetClass, viewArea,
					XtNfromVert, downButton,
					XtNfromHoriz, NULL,
					XtNborderWidth, 0,
					XtNwidth, 200,
					(char*)0);
								
    /* Create the source-selection controls. */
    Widget sourceSelect = XtVaCreateManagedWidget("sourceSelect",
					boxWidgetClass, controlArea, 
					XtNfromHoriz, viewArea,
					(char*)0);
					
    XtVaCreateManagedWidget("Source", labelWidgetClass, sourceSelect,
					XtNborderWidth, 0,
					XtNwidth, 120,
					XtNjustify, XtJustifyLeft,
					XtNforeground, 0x5050b8,
					(char*)0);

    testButton = XtVaCreateManagedWidget("SelectTest", toggleWidgetClass,
					sourceSelect, 
					XtNlabel, "Test",
					(char*)0);
    XtAddCallback(testButton, XtNcallback, selectTestGenerator, NULL);

    networkButton = XtVaCreateManagedWidget("SelectNetwork",
					toggleWidgetClass, sourceSelect, 
					XtNlabel, "Network", 
					XtNradioGroup, testButton,
					(char*)0);
    XtAddCallback(networkButton, XtNcallback, selectNetworkSource, NULL);

    /* Create the fade controls. */
    Widget fadeSelect = XtVaCreateManagedWidget("fadeSelect",
					formWidgetClass, controlArea, 
					XtNorientation, XtorientVertical,
					XtNfromHoriz, sourceSelect,
					(char*)0);
					
    Widget fadeLabel = XtVaCreateManagedWidget("Fading",
					labelWidgetClass, fadeSelect,
					XtNborderWidth, 0,
					XtNforeground, 0x5050b8,
					(char*)0);

    sweepFadeButton = XtVaCreateManagedWidget("FadeSweep",
					toggleWidgetClass, fadeSelect, 
					XtNfromVert, fadeLabel,
					XtNlabel, "Sweep",
					(char*)0);
					
    XtAddCallback(sweepFadeButton, XtNcallback, selectSweepFade, NULL);

    realtimeFadeButton = XtVaCreateManagedWidget("FadeRealTime",
					toggleWidgetClass, fadeSelect, 
					XtNlabel, "Real-time", 
					XtNfromVert, sweepFadeButton,
					XtNradioGroup, sweepFadeButton,
					(char*)0);
    XtAddCallback(realtimeFadeButton, XtNcallback, selectRealTimeFade, NULL);

    noFadeButton = XtVaCreateManagedWidget("FadeOverwrite",
					toggleWidgetClass, fadeSelect, 
					XtNlabel, "None", 
					XtNfromVert, realtimeFadeButton,
					XtNradioGroup, sweepFadeButton,
					(char*)0);
					
    XtAddCallback(noFadeButton, XtNcallback, selectNoFade, NULL);
				
    fadeTimeControl = XtVaCreateManagedWidget("FadeTime",
					scrollbarWidgetClass, fadeSelect,
					XtNorientation, XtorientVertical,
					XtNlength, 64,
					XtNheight, 64,
					XtNshown, 15, 
					XtNthickness, SCROLLBAR_WIDTH_PERCENT, 
					XtNfromHoriz, realtimeFadeButton,
					(char*)0);

    fadeTimeLabel = XtVaCreateManagedWidget("Unknown", 
    				labelWidgetClass, fadeSelect,
					XtNborderWidth, 0,
					XtNfromVert,  fadeTimeControl,
					XtNfromHoriz, fadeTimeControl,
					XtNhorizDistance, -15, 
					XtNjustify, XtJustifyLeft,
					(char*)0);
					
    XtAddCallback(fadeTimeControl, XtNscrollProc, changeFadeTime, NULL);
    XtAddCallback(fadeTimeControl, XtNjumpProc, jumpFadeTime, NULL);

    /* Create the drawing area. */
    Drawing = XtVaCreateManagedWidget("drawing",coreWidgetClass,box, 
					XtNheight, WinW,
					XtNwidth, WinH, 
					XtNfromVert, controlArea, 
					XtNleft, XawChainLeft, 
					XtNright, XawChainRight, 
					XtNbottom, XawChainBottom, 
					XtNtop, XawChainTop, 
					XtNresizable, True,
					(char*)0);

    /* Register an event handler for a window expose event. */
    XtAddEventHandler(Drawing, ExposureMask, FALSE, exposureEvent, NULL);
    
    /* Realize everything. */
    XtRealizeWidget(toplevel);

    /* And return to the main setup. */
    if( Verbose )
    {
	printf("User Interface construction complete.\n");
    }
    return;
} /* createUserInterface() */


/*====================================================================
*
* initialiseCompositing
*	Initialise everything we need for combining radar and graphics.
*
* Params:
*	None
*
* Returns:
*	Nothing (exits on error).
*
* Notes:
*
*===================================================================*/
void initialiseCompositing(void)
{
    /* Get an ARGB visual. */
    ArgbVisual = SPxFindArgbVisual(XDisplay, XScreen);

    /* Ensure we have the COMPOSITE extension. */
    int event_base, error_base;
    if( !XCompositeQueryExtension(XDisplay, &event_base, &error_base) )
    {
	printf("No support for compositing extension.\n");
	exit(-1);
    }

    /* Create the client-side image for a single scan conversion window. */
    RadarImage = CreateImage();

    /* Create pixmaps for the radar and the underlay and overlay graphics. */
    UnderlayPixmap = XCreatePixmap(XDisplay, XtWindow(Drawing),
					MAX_WIN_WIDTH, MAX_WIN_HEIGHT, 24);
    OverlayPixmap = XCreatePixmap(XDisplay, XtWindow(Drawing),
					MAX_WIN_WIDTH, MAX_WIN_HEIGHT, 32);
    RadarPixmap  = XCreatePixmap(XDisplay, XtWindow(Drawing),
					MAX_WIN_WIDTH, MAX_WIN_HEIGHT, 32);

    /* Create the server-side Picture structures. */
    XRenderPictureAttributes pa;
    pa.subwindow_mode = IncludeInferiors;
    UnderlayPicture = XRenderCreatePicture(XDisplay, UnderlayPixmap, 
				    XRenderFindVisualFormat(XDisplay,
					DefaultVisual(XDisplay,XScreen)),
				    CPSubwindowMode, &pa);
    OverlayPicture = XRenderCreatePicture(XDisplay, OverlayPixmap, 
				    XRenderFindStandardFormat(XDisplay,
					PictStandardARGB32),
				    CPSubwindowMode, &pa);
    RadarPicture = XRenderCreatePicture(XDisplay, RadarPixmap, 
				    XRenderFindStandardFormat(XDisplay,
					PictStandardARGB32), 
				    CPSubwindowMode, &pa);

    /* Create a pixmap and picture for the combination */
    CombinePixmap = XCreatePixmap(XDisplay, XtWindow(Drawing),
					MAX_WIN_WIDTH, MAX_WIN_HEIGHT, 24);
    CombinePicture = XRenderCreatePicture(XDisplay, CombinePixmap, 
				    XRenderFindVisualFormat(XDisplay,
					DefaultVisual(XDisplay,XScreen)), 
				    CPSubwindowMode, &pa);

    /* Create some relevant graphics contexts. */
    UnderlayGC   = XCreateGC(XDisplay, UnderlayPixmap, 0, NULL);
    OverlayGC = XCreateGC(XDisplay, OverlayPixmap, 0, NULL);
    RadarGC = XCreateGC(XDisplay, RadarPixmap, 0, NULL);  

    return;
} /* initialiseCompositing() */


/*====================================================================
*
* CreateImage
*	Function to create the shared memory XImage we put scan
*	converted radar video into.
*
* Params:
*	None
*
* Returns:
*	XImage.
*
* Notes:
*
*===================================================================*/
static XImage *CreateImage(void)
{
    // Create the X Image structure as a shared memory XImage.
    int XShmMajor, XShmMinor;
    Bool XShmPixmaps;
    XImage *xImage;
    
    // Test for the shared-memory extension */
    int shmAvailable = XShmQueryVersion(XDisplay, &XShmMajor, &XShmMinor,
						&XShmPixmaps);
    if( Verbose > 0 )
    {
	printf("SHM: available=%d, major=%d, minor=%d, shared pixmaps=%d.\n", 
			shmAvailable, XShmMajor, XShmMinor, XShmPixmaps);
    }
    
    // Now create the image.
    xImage = XShmCreateImage(XDisplay, ArgbVisual, 32, ZPixmap, NULL,
				&RadarImageSHMinfo,
				MAX_WIN_WIDTH, MAX_WIN_HEIGHT);

    // Create the shared-memory area.                           
    RadarImageSHMinfo.shmid = shmget(IPC_PRIVATE, 
				    xImage->bytes_per_line * xImage->height,
				    IPC_CREAT | 0777);
    if( RadarImageSHMinfo.shmid < 0 )
    {
        printf("Couldn't create the shared-memory segment.\n");
        exit(-1);
    }

    xImage->data = (char*)shmat(RadarImageSHMinfo.shmid, NULL, 0);
    RadarImageSHMinfo.shmaddr = xImage->data;
    if( RadarImageSHMinfo.shmaddr == (void *)(-1) )
    {
        printf("Couldn't attach to the shared memory area.\n");
        exit(-1);
    }
    if( Verbose > 0 )
    {
        printf("Shm attach ok, address = %p.\n", RadarImageSHMinfo.shmaddr);
    }
      
    // Now attach to Server
    int status = XShmAttach(XDisplay, &RadarImageSHMinfo);
    if( status == 0 )
    {
        printf("ShmAttach failed!.\n");
        exit(-1);
    }
    
    /* Now mark the shared-memory segment for deletion. This is safe
     * even though we are still using it because the actual deletion 
     * only occurs when the last process detatches (i.e. both us and the
     * X server).
     */
    shmctl(RadarImageSHMinfo.shmid, IPC_RMID, NULL);
    
    return xImage;
} /* CreateImage() */

/*====================================================================
*
* quitApplication
*	Handler function for the "quit" button.
*
* Params:
*	w		Widget emitting signal,
*	client, call	Standard X callback arguments.
*
* Returns:
*	Nothing
*
* Notes:
*	Exits the program.
*
*===================================================================*/
static void quitApplication(Widget w, XtPointer client, XtPointer call)
{
    // Quit button pressed
    exit(-1);
} /* quitApplication() */


/*********************************************************************
*
*	Graphics handling functions
*
**********************************************************************/

/*====================================================================
*
* exposureEvent
*	Handler function for the window exposure event.
*
* Params:
*	w		Widget emitting signal,
*	client, ev, cd	Standard X callback arguments.
*
* Returns:
*	Nothing
*
* Notes:
*	Causes a redraw of the graphics in the window.
*
*===================================================================*/
static void exposureEvent(Widget w, XtPointer client, XEvent *ev, Boolean *cd)
{
    // Expose event. Draw graphics and copy to screen.
    XExposeEvent *exposeEvent = (XExposeEvent*)ev;   
    if (exposeEvent->count == 0)
    {
	/* Draw underlay graphics first as that's the one that checks
	 * for size changes etc.
	 */
	DrawUnderlayPixmap();
	DrawOverlayPixmap();
    }
} /* exposureEvent() */


/*====================================================================
*
* DrawUnderlayPixmap
*	Draw the underlay graphics for the current view into UnderlayPixmap.
*
* Params:
*	None
*
* Returns:
*	Nothing
*
* Notes:
*	Also checks for window size changes etc.
*
*===================================================================*/
static void DrawUnderlayPixmap(void)
{
    Display *display;
    Window window = XtWindow(Drawing);
    if (window == 0) return;
    display = XtDisplay(Drawing);

    /* See if the window size has changed. */
    XWindowAttributes wa;
    XGetWindowAttributes(display, window, &wa);
    if( (wa.width != WinW) || (wa.height != WinH) )
    {
        /* Report the new size to the scan converter. */
        ScanConverter->SetWinGeom(0,0,wa.width, wa.height);
    }

    /* Record the window size, but remember that the radar window has
     * a maximum size that may be less than the current window size.
     */
    WinW = wa.width;
    WinH = wa.height;

    /* Clear the underlay to the background colour */
    XSetForeground(display, UnderlayGC, 0x202040);
    XFillRectangle(display, UnderlayPixmap, UnderlayGC, 0, 0, WinW, WinH);

    /*
     * Draw underlay graphics here...
     */

    /* Done. */
    return;
} /* DrawUnderlayPixmap() */

/*====================================================================
*
* DrawOverlayPixmap
*	Draw the overlay graphics for the current view into OverlayPixmap.
*
* Params:
*	None
*
* Returns:
*	Nothing
*
* Notes:
*
*===================================================================*/
static void DrawOverlayPixmap(void)
{
    Display *display;
    Window window = XtWindow(Drawing);
    if (window == 0) return;
    display = XtDisplay(Drawing);

    /* Clear overlay to transparent (not black). */
    XSetForeground(display, OverlayGC, 0x01000000);
    XFillRectangle(display, OverlayPixmap, OverlayGC, 0, 0, WinW, WinH);

    /* Set foreground colour to be overlay (alpha = 0xff). */
    XSetForeground(display, OverlayGC, 0xffc0c0c0);

    /*
     * Draw overlay graphics here...
     */

 

    /* Done. */
    return;
} /* DrawOverlayPixmap() */


/*====================================================================
*
* ChangeWindowView
*	Set the view in the window (i.e. the world area to show in
*	the window).
*
* Params:
*	newViewX, newViewY		New centre position,
*	newViewW, newViewH		New dimensions,
*	showWindow			Flag to say update the window
*					contents.
*
* Returns:
*	Nothing
*
* Notes:
*
*===================================================================*/
static void ChangeWindowView(double newViewX, double newViewY,
				double newViewW, double newViewH,
				int showWindow)
{
    /* Change the view in a window. */
    Vy = newViewY;
    Vx = newViewX;
    Vw = newViewW;
    Vh = newViewH;

    /* Update the range scale label. */
    char string[64];
    sprintf(string, "Scale: %-.0f km", Vw/1000);
    XtVaSetValues(rangeScaleLabel, XtNlabel, string, (void*)NULL);

    /* Redraw underlay graphics first as that's the one that checks
     * for size changes etc.
     */
    DrawUnderlayPixmap();
    DrawOverlayPixmap();
} /* ChangeWindowView() */


/*====================================================================
*
* UpdateDisplay
*	Function to update the display.
*
* Params:
*	closure, id		Ignored.
*
* Returns:
*	Nothing
*
* Notes:
*	This function is called under a timer to create the display
*	picture.  The radar video is being updated in the X Image structure
*	by a separate thread.  Every UpdatePeriod milliseconds we take the
*	current radar image and combine it with the current graphics to
*	construct the display.
*
*===================================================================*/
static void UpdateDisplay(XtPointer closure, XtIntervalId *id)
{
    /* Do we need to fade the bitmap? */
    if( DestBitmap->GetFadeType() == SPX_RADAR_FADE_REAL_TIME )
    {
	if( Verbose > 2 )
	{
	    UINT32 now = SPxTimeGetTickerMsecs();
	    fprintf(stdout, "%u.%03u: Fading bitmap.\n", now/1000, now%1000);
	}
	if( DestBitmap->FadeBitmap() != SPX_NO_ERROR )
	{
	    fprintf(stdout, "Failed to fade bitmap.\n");
	}
    }
 
    /* Copy the Image to the server pixmap over shared-memory. */
    if(RadarImage)
    {
	XShmPutImage(XDisplay, RadarPixmap, RadarGC, RadarImage, 0,0,0,0,
			MIN(WinW, MAX_WIN_WIDTH), MIN(WinH, MAX_WIN_HEIGHT),
			False);
    }

    /* Start the composited picture with the underlay. */
    XRenderComposite(XDisplay, PictOpSrc, UnderlayPicture, None,
			CombinePicture, 0,0,0,0,0,0, 
			MIN(WinW, MAX_WIN_WIDTH), MIN(WinH, MAX_WIN_HEIGHT));

    /* Alpha-blend the radar image onto the underlay (in the combined
     * picture), noting that this takes substantial time if the drivers
     * do not hardware-accelerate this.
     */
    XRenderComposite(XDisplay, PictOpOver, RadarPicture, None,
			CombinePicture, 0,0,0,0,0,0, 
			MIN(WinW, MAX_WIN_WIDTH), MIN(WinH, MAX_WIN_HEIGHT));

    /* Copy the overlay picture (hopefully largely transparent) on top. */
    XRenderComposite(XDisplay, PictOpOver, OverlayPicture, None,
			CombinePicture, 0,0,0,0,0,0, 
			MIN(WinW, MAX_WIN_WIDTH), MIN(WinH, MAX_WIN_HEIGHT));

    /* Copy the whole lot to the window. */
    XCopyArea(XDisplay, CombinePixmap, XtWindow(Drawing), UnderlayGC, 0,0, 
			MIN(WinW, MAX_WIN_WIDTH), MIN(WinH, MAX_WIN_HEIGHT),
			0, 0);

    /* Synchronise the X updates to ensure this update is actually done now */
    XSync(XDisplay,0);

    /* Schedule the next update. */
    XtAppAddTimeOut(AppContext, UpdatePeriod, UpdateDisplay, NULL);
} /* UpdateDisplay() */


/*********************************************************************
*
*	Other widget callback functions
*
**********************************************************************/


static void scrollUp(Widget w, XtPointer client, XtPointer call)
{
    // Scroll view upwards.
    ChangeWindowView(Vx, Vy+0.25*Vh, Vw, Vh, 1);
    ScanConverter->SetView(Vx,Vy,Vw,Vh);   
}

static void scrollDown(Widget w, XtPointer client, XtPointer call)
{
    // Scroll view down.
    ChangeWindowView(Vx, Vy-0.25*Vh, Vw, Vh, 1);
    ScanConverter->SetView(Vx,Vy,Vw,Vh);  
}

static void scrollLeft(Widget w, XtPointer client, XtPointer call)
{
    // Scroll left
    ChangeWindowView(Vx-0.25*Vw, Vy, Vw, Vh, 1);
    ScanConverter->SetView(Vx,Vy,Vw,Vh);  
}

static void scrollRight(Widget w, XtPointer client, XtPointer call)
{
    // Scroll right
    ChangeWindowView(Vx+0.25*Vw, Vy, Vw, Vh, 1);
    ScanConverter->SetView(Vx,Vy,Vw,Vh);  
}

static void zoomIn(Widget w, XtPointer client, XtPointer call)
{
    // Zoom in
    Vw *= 0.5;
    ChangeWindowView(Vx, Vy, Vw, Vh, 1);
    ScanConverter->SetView(Vx,Vy,Vw,Vh);  
}

static void zoomOut(Widget w, XtPointer client, XtPointer call)
{
    // Zoom out
    Vw *= 2.0;
    ChangeWindowView(Vx, Vy, Vw, Vh, 1);
    ScanConverter->SetView(Vx,Vy,Vw,Vh);  
}

static void recentre(Widget w, XtPointer client, XtPointer call)
{
    // Recentre.
    ChangeWindowView(START_VIEW_X,START_VIEW_Y,START_VIEW_W,START_VIEW_H,1);
    ScanConverter->SetView(Vx,Vy,Vw,Vh);
}

static void selectTestGenerator(Widget w, XtPointer client, XtPointer call)
{
    // Test Generator
    if( isButtonActive(w) && SrcTpg )
    {
	SrcTpg->Enable(1);
    }
}

static void selectNetworkSource(Widget w, XtPointer client, XtPointer call)
{
    // Network Source
    if( isButtonActive(w) && SrcNet )
    {
	SrcNet->Enable(1);
    }
}

static void selectSweepFade(Widget w, XtPointer client, XtPointer call)
{
    // Set Sweep fade
    if( isButtonActive(w) )
    {
    	CurrentFade = SPX_RADAR_FADE_SWEEP;
	SetNewFade();
    }
}

static void selectRealTimeFade(Widget w, XtPointer client, XtPointer call)
{
    // Set real-time fade
    if( isButtonActive(w) )
    {
    	CurrentFade = SPX_RADAR_FADE_REAL_TIME;
	SetNewFade();
    }
}

static void selectNoFade(Widget w, XtPointer client, XtPointer call)
{
    // Set no fade
    if( isButtonActive(w) )
    {
    	CurrentFade = SPX_RADAR_FADE_REPLACE;
	SetNewFade();
    }
}

static void changeFadeTime(Widget fadeScroll, XtPointer client, XtPointer pos)
{
    /* The fade scroll widget has incremented/decremented its value */
    int position = (int)(intptr_t)(pos);
    if (position < 0)
    {
	/* Increment the fade length and clip to 100. Step size is
	 * coarser if in sweep mode. */
	if (CurrentFade == SPX_RADAR_FADE_SWEEP)
		CurrentFadeLength += 5;
	else
		CurrentFadeLength++;
	if (CurrentFadeLength > 100) CurrentFadeLength = 100;
    }
    else
    {
	/* Decrement the fade length and clip to 0 */
	if (CurrentFade == SPX_RADAR_FADE_SWEEP)
		CurrentFadeLength -= 5;
	else
		CurrentFadeLength--;
	    if (CurrentFadeLength <= 0) CurrentFadeLength = 1;
    }
    SetNewFade();
}

static void jumpFadeTime(Widget fadeScroll, XtPointer client,
						XtPointer percentPtr)
{
    /* The fade scroll widget has jumped to a new location. */
    float percent = *(float*)(percentPtr);
    CurrentFadeLength = (int) (100.0 * percent);
    if (CurrentFadeLength <= 0 ) CurrentFadeLength = 1;
    if (CurrentFadeLength > 100) CurrentFadeLength = 100;
    SetNewFade();
}

static void SetNewFade(void)
{
    /* Use the value of CurrentFadeLength (0..100) to configure
     * a fade time that is appropriate for the current fade mode. 
     */
     XawScrollbarSetThumb(fadeTimeControl, 
	    (float)(1.0*CurrentFadeLength/100.0), 
	    (float)(SCROLLBAR_WIDTH_PERCENT/100.0));
     char fadeTimeString[64];
     switch (CurrentFade)
     {
	case SPX_RADAR_FADE_REAL_TIME:
	    {
	    /* Scaled 1 to 20 secs scross the slider */
	    UINT16 fadeRate10ms = CurrentFadeLength * 2000 / 100;
	    setButtonActive(realtimeFadeButton, True);
	    /* Restrict lower bound to 1 second (100 x 10 ms) */
	    if (fadeRate10ms < 100) fadeRate10ms = 100;
	    ScanConverter->SetFade(SPX_RADAR_FADE_REAL_TIME, fadeRate10ms);
	    sprintf(fadeTimeString,"%-.1fs", 
			    1.0 * fadeRate10ms * 10 / 1000.0);
	    XtVaSetValues(fadeTimeLabel, XtNlabel, fadeTimeString, (void*)NULL);
	    }
	    break;
		    
	case SPX_RADAR_FADE_SWEEP:
	    /* Scaled to to 20 scans across the slider */
	    {
	    setButtonActive(sweepFadeButton, True);
	    UINT16 fadeSweeps = CurrentFadeLength * 20 / 100;
	    if (fadeSweeps < 1) fadeSweeps = 1;
	    ScanConverter->SetFade(SPX_RADAR_FADE_SWEEP, fadeSweeps);
	    sprintf(fadeTimeString,"%d sps", fadeSweeps);
	    XtVaSetValues(fadeTimeLabel, XtNlabel, fadeTimeString, (void*)NULL);
	    }
	    break;
			    
	case SPX_RADAR_FADE_REPLACE:
	    setButtonActive(noFadeButton, True);
	    ScanConverter->SetFade(SPX_RADAR_FADE_REPLACE, 0);
	    sprintf(fadeTimeString,"None");
	    XtVaSetValues(fadeTimeLabel, XtNlabel, fadeTimeString, (void*)NULL);
	    break;
    }
}

static void setButtonActive(Widget w, Boolean state)
{
    /* Set the activity state of a widget button. */
    XtVaSetValues(w, XtNstate, state, (void*)NULL);
}

static Boolean isButtonActive(Widget w)
{
    /* See if the state is active or not. */
    Boolean state;
    XtVaGetValues(w, XtNstate, &state, (void*)NULL);
    return(state);
} /* isButtonActive() */


/*====================================================================
*
* bitmapCallback
*	Callback function to be invoked when the bitmap changes
*	contents or configuration.
*
* Params:
*	bitmap		Handle of bitmap that has changed,
*	changes		Bitwise combination of the following flags to
*			indicate that the configuration of the bitmap
*			has changed:
*				SPX_BITMAP_CHANGE_POSITION,
*				SPX_BITMAP_CHANGE_SIZE,
*				SPX_BITMAP_CHANGE_VISIBLE,
*				SPX_BITMAP_CHANGE_VIEW,
*	startAzi	First azimuth of changed data in the most recent
*			updates (in the range 0 - 65535 for 0 - 360 degrees),
*	endAzi		Last azimuth of changed data (same range as startAzi),
*	box		A structure giving the x, y, w and h of the
*			minimum enclosing area of the bitmap that has
*			recently been changed.
*
* Returns:
*	Nothing
*
* Notes:
*	This is the function that would do the "real work" in a program,
*	using the contents of the bitmap to update a display etc. etc.
*
*===================================================================*/
static void bitmapCallback(SPxScDestBitmap *bitmap, UINT16 changes,
                             UINT16 startAzi, UINT16 endAzi, DirtyBox box)
{
	//if (counter > 1) return;
	//printf("%d",bitmap->GetBitmap());
	//std::cout << RadarImage->data << std::endl;
	sensor_msgs::Image img;
	img.header.frame_id = "radar";
	img.header.stamp = ros::Time::now();
	img.height = bitmap->GetHeight();
	img.width = bitmap->GetWidth();
	img.encoding = sensor_msgs::image_encodings::BGRA8;
	img.is_bigendian = RadarImage->bitmap_bit_order;
	img.step = RadarImage->bytes_per_line;
	const char *data = reinterpret_cast<const char *>(bitmap->GetBitmap());
	img.data.assign(data, data+img.step*img.height);
	imgPubPtr->publish(img);
    /* This function gets called as a notification that new data is
     * available in the bitmap and/or the bitmap configuration has changed.
     *
     * The start and end azimuths of the sector that has changed are provided,
     * as are the coordinates and size of the box within the bitmap that has
     * changed.
     */
    if( changes )
    {
	/* The configuration has changed.
	 *
	 * We simply report the change, but a real program would act upon
	 * it, using the various Get() functions in the SPxScDestBitmap
	 * class to retrieve the new configuration.
	 */
	printf("Bitmap configuration changed, flags 0x%04x:\n", changes);

	/* Say what changed. */
	if( changes & SPX_BITMAP_CHANGE_POSITION )
	{
	    printf("\tPosition changed.\n");
	}
	if( changes & SPX_BITMAP_CHANGE_SIZE )
	{
	    printf("\tSize changed.\n");
	}
	if( changes & SPX_BITMAP_CHANGE_VISIBLE )
	{
	    printf("\tVisibility changed.\n");
	}
	if( changes & SPX_BITMAP_CHANGE_VIEW )
	{
	    printf("\tView changed.\n");
	}
    }
    if( (box.w > 0) && (box.h > 0) )
    {
	/* The contents of the given box have changed.
	 *
	 * We simply report the area, but a real program could use the
	 * new data.
	 */

	/* Report details of the whole area. */
	printf("New data: pos = (%d,%d), sz = (%d,%d), for %.1f to %.1f deg\n",
		box.x, box.y, box.w, box.h,
		((double)startAzi) * 360.0 / 65536.0,
		((double)endAzi) * 360.0 / 65536.0);
	fflush(stdout);

	/* Clear the dirty box. */
	bitmap->ClearDirtyBox();
    }
	counter += 1;
    return;
} /* bitmapCallback() */



/*********************************************************************
*
* End of file
*
**********************************************************************/
