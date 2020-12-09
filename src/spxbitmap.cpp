/*********************************************************************
*
* (c) Copyright 2008, 2009, Cambridge Pixel Ltd.
*
* File: $RCSfile: spxbitmap.cpp,v $
* ID: $Id: spxbitmap.cpp,v 1.2 2009/03/24 20:07:17 rew Exp $
*
* Purpose:
*	SPx Display Bitmap Example.
*
*	This program uses the SPxScSourceLocal class to create a local
*	scan converter and request scan converted images.  It then uses
*	the SPxDestBitmap class to receive the bitmap data and make it
*	available.
*
*	The bitmapCallback() function is automatically called whenever the
*	bitmap has been updated with new images.  This is where a real
*	program could process the image, merge it with graphics, display
*	it to the user etc. etc.
*
*
* Revision Control:
*   24/03/09 v1.2    REW	Avoid compiler warning.
*
* Previous Changes:
*   11/10/08 1.1    DGJ	Initial Version
**********************************************************************/

/* Standard headers. */
#include <stdio.h>
#include <stdlib.h>

/* SPx headers. */
#include "SPx.h"

/* Callback for when the bitmap has been updated. */
static void bitmapCallback(SPxScDestBitmap *bitmap, UINT16 changes, 
				UINT16 startAzi, UINT16 endAzi, DirtyBox box);

/*
 * Private variables.
 */
static int MaxWinWidth = 512;
static int MaxWinHeight = 512;


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
int main(int argc, char **argv) 
{
    printf("\n### Cambridge Pixel SPx Bitmap Demo %s\n\n",SPX_VERSION_STRING);

    /*
     * Initialise the SPx libraries.
     */
    if( SPxInit() != SPX_NO_ERROR )
    {
	printf("Failed to initialise SPx.\n");
	exit(-1);
    }

    /* Create the bitmap, which is the destination for the scan converter */
    SPxScDestBitmap *spxBitmap = new SPxScDestBitmap();
    spxBitmap->Create(MaxWinWidth, MaxWinHeight);
    
    /* Register the function that will be called when the bitmap is
     * updated.
     */
    spxBitmap->SetNotifyUpdate(bitmapCallback);
    
    /* Create the scan converter */
    SPxScSourceLocal *spxSc = new SPxScSourceLocal(spxBitmap);	
    spxSc->SetWinGeom(0,0,MaxWinWidth, MaxWinHeight);
 	
    /* Create the RIB */
    SPxRIB * spxRib = new SPxRIB(1024*1024);
 	
    /* Create a PIM */
    SPxPIM * spxPIM = new SPxPIM(spxRib, 1024, 1024, 
					SPX_PIM_RAN_PEAK, SPX_PIM_AZI_PEAK,
					SPX_PIM_OUTPUT(1));
 	
    /* Create a process that connects the scan converter to the PIM */
    new SPxRunProcess(SPxProScanConv, NULL, spxPIM, spxSc);
 	
    /* Create a test generator input */
    SPxTestGenerator *spxTest = new SPxTestGenerator(spxRib,1024,4.0,200,5,0);
    spxTest->Enable(1);
 	
    /*
     * Run the main loop.  In this demo, we don't do anything, but a real
     * program might create graphics, provide a user interface etc. etc.
     */
    printf("Starting main loop...\n");

    int finish = 0;
    while(!finish)
    {
	/* Don't busy loop. */
        SPxTimeSleepMsecs(1000);
    }

    /* Never gets here. */
    exit(EXIT_SUCCESS);
} /* main() */


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
    return;   
} /* bitmapCallback() */


/*********************************************************************
*
* End of file
*
**********************************************************************/
