/*
 ISC License

 Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

 */
/*
    Hough Circle Finder

    Note:   this module takes a bit-map image and writes out the circles that are found in the image by OpenCV's HoughCricle Transform.
    Author: Thibaud Teil
    Date:   February 13, 2019
 
 */

/* modify the path to reflect the new module names */
#include "houghCircles.h"
#include "simulation/utilities/linearAlgebra.h"
#include <string.h>

/*! This method initializes the configData for this module.
 It checks to ensure that the inputs are sane and then creates the
 output message
 @return void
 @param configData The configuration data associated with this module
 */
void SelfInit_houghCircles(HoughCirclesConfig *configData, uint64_t moduleID)
{
    
    /*! Begin method steps */
    /*! - Create output message for module */
    configData->opnavCirclesOutMsgID = CreateNewMessage(configData->opnavCirclesOutMsgName,
                                               sizeof(CirclesOpnavMsg),
                                               "CirclesOpnavMsg",          /* add the output structure name */
                                               moduleID);
    
    configData->blurrSize = 5
    configData->cannyThresh1 = 200
    configData->cannyThresh2 = 20
    configData->houghMinDist = 50
    configData->houghMinRadius = 0
    configData->houghMaxRadius = 0 // Maximum circle radius. If <= 0, uses the maximum image dimension. If < 0, returns centers without finding the radius
}

/*! This method performs the second stage of initialization for this module.
 It's primary function is to link the input messages that were created elsewhere.
 @return void
 @param configData The configuration data associated with this module
 */
void CrossInit_houghCircles(HoughCirclesConfig *configData, uint64_t moduleID)
{
    /*! - Get the control data message ID*/
    configData->imageInMsgID = subscribeToMessage(configData->imageInMsgName,
                                                sizeof(ImageFswMsg),
                                                moduleID);
}

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param configData The configuration data associated with the module
 */
void Reset_houghCircles(HoughCirclesConfig *configData, uint64_t callTime, uint64_t moduleID)
{
    return;
}

/*! Add a description of what this main Update() routine does for this module
 @return void
 @param configData The configuration data associated with the module
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void Update_houghCircles(HoughCirclesConfig *configData, uint64_t callTime, uint64_t moduleID)
{
    ImageFswMsg imageBuffer;
    Mat src, canny, grey, blurred;
    /*! - Read in the image message*/
    imageBuffer = ReadMessage()
    
    src = imread( *imageBuffer.image, 1 );
    cvtColor( src, grey, CV_BGR2GRAY );
    blur( grey, blurred, Size(configData->blurrSize,configData->blurrSize) );
    
    vector<Vec4f> circles;
    // Apply the Hough Transform to find the circles
    HoughCircles( blurred, circles, CV_HOUGH_GRADIENT, 1, src.rows/2, configData->cannyThresh1,configData->cannyThresh2, configData->houghMinRadius, configData->houghMaxRadius );
    
    return;
}
