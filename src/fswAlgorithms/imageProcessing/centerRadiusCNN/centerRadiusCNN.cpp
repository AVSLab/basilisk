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
    Convolutional Neural Net Circle Finder

    Note:   This module takes an image and writes out the circles that are found in the image by OpenCV's AI library.
    Author: Thibaud Teil
    Date:   February 13, 2019
 
 */

/* modify the path to reflect the new module names */
#include <string.h>
#include "centerRadiusCNN.h"
#include <opencv2/dnn/dnn.hpp>


/*! The constructor for the CenterRadiusCNN module. It also sets some default values at its creation.  */
CenterRadiusCNN::CenterRadiusCNN()
{
    this->OutputBufferCount = 2;
    this->filename = "";
    this->saveImages = 0;
    this->imageSize[0] = 512;
    this->imageSize[1] = 512;
    for (int i=0; i<3; i++){
        this->pixelNoise[i] = 5;
    }
    this->pathToNetwork = "./position_net2_trained_11-14.onnx";
}

/*! Selfinit performs the first stage of initialization for this module.
 It's primary function is to create messages that will be written to.
 @return void
 */
void CenterRadiusCNN::SelfInit()
{
    /*! - Read in the CNN */
    std::ifstream test(this->pathToNetwork);
    if (!test)
    {
        std::cout << "The CNN file doesn't exist" << std::endl;
    }
    this->positionNet2 = cv::dnn::readNetFromONNX(this->pathToNetwork);
    this->positionNet2.setPreferableBackend(cv::dnn::DNN_BACKEND_DEFAULT);
    this->positionNet2.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    
    /*! - Create output message for module */
    this->opnavCirclesOutMsgID = SystemMessaging::GetInstance()->CreateNewMessage(this->opnavCirclesOutMsgName,sizeof(CirclesOpNavMsg),this->OutputBufferCount,"CirclesOpNavMsg",moduleID);
}


/*! CrossInit performs the second stage of initialization for this module.
 It's primary function is to link the input messages that were created elsewhere.
 @return void
 */
void CenterRadiusCNN::CrossInit()
{
    /*! - Get the image data message ID*/
    this->imageInMsgID = SystemMessaging::GetInstance()->subscribeToMessage(this->imageInMsgName,sizeof(CameraImageMsg), moduleID);
}

/*! This is the destructor */
CenterRadiusCNN::~CenterRadiusCNN()
{
    return;
}


/*! This method performs a complete reset of the module.  Local module variables that retain time varying states between function calls are reset to their default values.
 @return void
 @param this The configuration data associated with the module
 */
void CenterRadiusCNN::Reset(uint64_t CurrentSimNanos)
{
    return;
}

/*! This module reads an OpNav image and extracts circle information from its content using OpenCV's HoughCircle Transform. It performs a greyscale, a bur, and a threshold on the image to facilitate circle-finding. 
 @return void
 @param CurrentSimNanos The clock time at which the function was called (nanoseconds)
 */
void CenterRadiusCNN::UpdateState(uint64_t CurrentSimNanos)
{
    std::string filenamePre;
    CameraImageMsg imageBuffer;
    CirclesOpNavMsg circleBuffer;
    cv::Mat imageCV, blurred;
    filenamePre = "PreprocessedImage_" + std::to_string(CurrentSimNanos*1E-9) + ".jpg";

    /*! - Load in the trained CNN model*/

    /*! - Read in the bitmap*/
    SingleMessageHeader localHeader;
    memset(&imageBuffer, 0x0, sizeof(CameraImageMsg));
    memset(&circleBuffer, 0x0, sizeof(CirclesOpNavMsg));
    if(this->imageInMsgName != "")
    {
        SystemMessaging::GetInstance()->ReadMessage(this->imageInMsgID, &localHeader,
                                                    sizeof(CameraImageMsg), reinterpret_cast<uint8_t*>(&imageBuffer), this->moduleID);
        this->sensorTimeTag = localHeader.WriteClockNanos;
    }
    /* Added for debugging purposes*/
    if (!this->filename.empty()){
        imageCV = imread(this->filename, cv::IMREAD_COLOR);
    }
    else if(imageBuffer.valid == 1 && imageBuffer.timeTag >= CurrentSimNanos){
        /*! - Recast image pointer to CV type*/
        std::vector<unsigned char> vectorBuffer((char*)imageBuffer.imagePointer, (char*)imageBuffer.imagePointer + imageBuffer.imageBufferLength);
        imageCV = cv::imdecode(vectorBuffer, cv::IMREAD_COLOR);
        if (this->saveImages == 1){
            cv::imwrite(filenamePre, imageCV);
        }
    }
    else{
        /*! - If no image is present, write zeros in message */
        SystemMessaging::GetInstance()->WriteMessage(this->opnavCirclesOutMsgID, CurrentSimNanos, sizeof(CirclesOpNavMsg), reinterpret_cast<uint8_t *>(&circleBuffer), this->moduleID);
        return;
    }
    /*!-  evaluate CNN on image */
    cv::Mat img_blob = cv::dnn::blobFromImage(imageCV, 1.0/255.0, cv::Size(this->imageSize[0], this->imageSize[1]), cv::Scalar(0,0,0), true);

    positionNet2.setInput(img_blob);
    cv::Mat output = positionNet2.forward();
    float x_pred = output.at<float>(0,0);
    float y_pred = output.at<float>(0,1);
    float rad_pred = output.at<float>(0,2);
    
    /*!- If no circles are found do not validate the image as a measurement */
    if (x_pred != this->imageSize[0]/2 && y_pred != this->imageSize[1]/2 && rad_pred != this->imageSize[1]/4){
        circleBuffer.valid = 1;
        circleBuffer.planetIds[0] = 2;
        circleBuffer.circlesCenters[0] = x_pred;
        circleBuffer.circlesCenters[1] = y_pred;
        circleBuffer.circlesRadii[0] = rad_pred;
        for (int j=0; j<3; j++){
            circleBuffer.uncertainty[j] = this->pixelNoise[j];
        }
    }

    SystemMessaging::GetInstance()->WriteMessage(this->opnavCirclesOutMsgID, CurrentSimNanos, sizeof(CirclesOpNavMsg), reinterpret_cast<uint8_t *>(&circleBuffer), this->moduleID);
    
    return;
}

