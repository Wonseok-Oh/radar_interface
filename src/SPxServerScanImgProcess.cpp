#include "SPxServerScanImgProcess.h"
/* SPx source objects */
static SPxTestGenerator *SrcTpg = NULL;
static SPxNetworkReceive *SrcNet = NULL;

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



CSPxScanImgProc::CSPxScanImgProc(int argc, char **argv){
    int c;			/* For parsing command line options */
    int aziSpoke = -1;		/* Azimuth spoke to insert */
    int testPattern = 102;	/* Test pattern to show */
    UINT32 sweepLineRGB = 0;	/* Sweep line colour */

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
	}
    } /* end of for each option */


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


    /* Create the bitmap, which is the destination for the scan converter */
	m_spxScDestBitmap = new SPxScDestBitmap();
	m_spxScDestBitmap->Create(MAX_WIN_WIDTH, MAX_WIN_HEIGHT, SPX_BITMAP_32BITS,
		    (unsigned char*)RadarImage->data,
		    RadarImage->bytes_per_line);

    /* The m_spxScDestBitmap class needs to know how often we'll update the display,
     * since it needs to calculate fades time.
     */
    m_spxScDestBitmap->SetUpdateInterval(UpdatePeriod);

	/* Create the scan converter */
	ScanConverter = new SPxScSourceLocal(m_spxScDestBitmap, SPX_SC_TYPE_PPI);

	/* Create the RIB. Size is 1Mb, with memory allocation done by class. */
    m_spxScProcess = NULL;
	int numSamples = 2048;
    int numAzimuths = 2048;
    m_spxRib = new SPxRIB(numSamples*numAzimuths, NULL);

    /* Create the PIM to provide the polar storage allowing for a maximum of 2 bytes per sample to support 16-bit radar video. */
    m_spxPim = new SPxPIM(m_spxRib, numSamples, numAzimuths, SPX_PIM_RAN_PEAK, SPX_PIM_AZI_PEAK, SPX_PIM_OUTPUT(1));

    if( aziSpoke >= 0 ){
    	m_spxScProcess = new SPxRunProcess(SPxProAziSpokes, m_spxScProcess, m_spxPim);
    	m_spxScProcess->SetParamValueI("Azimuth", aziSpoke);
    }
    m_spxScProcess = new SPxRunProcess(SPxProScanConv, m_spxScProcess,
					m_spxPim, ScanConverter);
    SrcTpg = new SPxTestGenerator(m_spxRib, numSamples, 2.0, 1000,
					testPattern ? testPattern : 102, 0);

    m_spxScProcess = new SPxRunProcess(SPxProScanConv, NULL, m_spxPim, ScanConverter);

    std::cout << "spxScProcess created" << std::endl;

    scanImg.create(SCANIMG_HEIGHT, SCANIMG_WIDTH, CV_8UC4);

    std::cout << "scanImg created" << std::endl;


	/* Create the network receiver to spxRIB (for general cases) */
	SrcNet = new SPxNetworkReceive(m_spxRib);
	SrcNet->Create("239.192.43.78", 4378);
	SrcNet->Enable(TRUE) ;
	setButtonActive(networkButton, TRUE);

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

    std::cout << "end of class generator" << std::endl;

	/* Create the network receiver from SIMRAD 4G radar (proj code 226) to spxRIB */
	//m_src226 = new SPxNetworkReceiveP226(m_spxRib);
	//m_src226->SetSerialNumber("1511403102");
	//m_src226->SetLicenseKey("BEC926FA00A043D41109E65BB457BC30CDC8CD310EC1FCBC71BD55BEDCFEF7FC8057236B61B89D5F1BB6172C362003D932A09272CACBD47CB40455721937BC46");
	//m_src226->Create();
	//m_src226->Enable(TRUE);

	/* Create a process that connects the scan converter to the PIM */



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

Mat CSPxScanImgProc::radar2mat(unsigned char* radar_data){
	Mat src;

	scanImg = src;


    return src;
}   
