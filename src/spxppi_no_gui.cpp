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
#include <string>

/* SPx headers. */
#include "SPx.h"

/* ROS Library */
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>


/* Callback for when the bitmap has been updated. */
static void bitmapCallback(SPxScDestBitmap *bitmap, UINT16 changes,
				UINT16 startAzi, UINT16 endAzi, DirtyBox box);

/*
 * Private variables.
 */
//static int MaxWinWidth = 256;
//static int MaxWinHeight = 256;

static ros::Publisher* imgPubPtr = NULL;
static bool isInit = false;
static sensor_msgs::Image img;
/* The X Image used to hold the client-side radar bitmap. */
//static XImage *RadarImage;

/*********************************************************************
*
*	Public functions
*
**********************************************************************/
void publish_rostopic(const ros::TimerEvent& event){
	// if not initialized, do nothing
	if (!isInit) return;
	imgPubPtr->publish(img);
	return;
}
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
	ros::init(argc, argv, "radar_interface_node");
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");
	int max_win_width, max_win_height, port;
	std::string src_ip_addr;

	private_nh.param("max_win_width", max_win_width, 256);
	private_nh.param("max_win_height", max_win_height, 256);
	private_nh.param<std::string>("src_ip_addr", src_ip_addr, std::string("239.192.43.78"));
	private_nh.param("port", port, 4378);
	ros::Timer timer = nh.createTimer(ros::Duration(0.1), publish_rostopic);

	imgPubPtr = new ros::Publisher;
	*imgPubPtr = nh.advertise<sensor_msgs::Image>("radar_image", 1);

    printf("\n### Cambridge Pixel SPx Bitmap Demo %s\n\n",SPX_VERSION_STRING);

    /*
     * Initialise the SPx libraries.
     */
    if( SPxInit() != SPX_NO_ERROR )
    {
	printf("Failed to initialise SPx.\n");
	exit(-1);
    }

    /* Acquire license from hardware dongle */
    if (SPxLicInit() != SPX_NO_ERROR){
    	/* Failed to find valid dongle */
    	printf("WARNING: functionality will be restricted\n");
    }

    /* Create the bitmap, which is the destination for the scan converter */
    SPxScDestBitmap *spxBitmap = new SPxScDestBitmap();
    int bitmapCreated = spxBitmap->Create(max_win_width, max_win_height, SPX_BITMAP_32BITS);

    if( bitmapCreated != 0 ){
    	printf("Failed to create SPxBitmap.\n");
    	exit(-1);
    }

    /* Register the function that will be called when the bitmap is
     * updated.
     */
    spxBitmap->SetNotifyUpdate(bitmapCallback);

    /* Create the scan converter */
    SPxScSourceLocal *spxSc = new SPxScSourceLocal(spxBitmap);
    spxSc->SetWinGeom(0,0,max_win_width, max_win_height);

    /* Create the RIB */
    SPxRIB * spxRib = new SPxRIB(1024*1024);

    /* Create a PIM */
    SPxPIM * spxPIM = new SPxPIM(spxRib, 1024, 1024,
					SPX_PIM_RAN_PEAK, SPX_PIM_AZI_PEAK,
					SPX_PIM_OUTPUT(1));

    /* Create a process that connects the scan converter to the PIM */
    new SPxRunProcess(SPxProScanConv, NULL, spxPIM, spxSc);

    /* Create the network receiver to spxRIB (for general cases) */
   	SPxNetworkReceive *SrcNet = new SPxNetworkReceive(spxRib);
   	SrcNet->Create(src_ip_addr.c_str(), port);
	SrcNet->Enable(TRUE);

    /*
     * Run the main loop.  In this demo, we don't do anything, but a real
     * program might create graphics, provide a user interface etc. etc.
     */
    printf("Starting main loop...\n");
    isInit = true;
    ros::spin();
    while(ros::ok())
    {
	/* Don't busy loop. */
        SPxTimeSleepMsecs(1000);
    }

    /* Never gets here. */
    delete imgPubPtr, spxSc, spxRib, spxPIM;
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

	img.header.frame_id = "radar";
	img.header.stamp = ros::Time::now();
	img.height = bitmap->GetHeight();
	img.width = bitmap->GetWidth();
	img.encoding = sensor_msgs::image_encodings::BGRA8;
	img.is_bigendian = 0; // can this be changed? (from topic radar_image after run radar_monitor_node)
	img.step = img.height*4; // where can we get data from? (from topic radar_image after run radar_monitor_node)
	// MODIFIED, img.height*4(BGRA)
	const char *data = reinterpret_cast<const char *>(bitmap->GetBitmap());
	img.data.assign(data, data+img.step*img.height);

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
		/*printf("New data: pos = (%d,%d), sz = (%d,%d), for %.1f to %.1f deg\n",
			box.x, box.y, box.w, box.h,
			((double)startAzi) * 360.0 / 65536.0,
			((double)endAzi) * 360.0 / 65536.0);
		fflush(stdout);*/

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
