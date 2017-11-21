/**
Copyright (c)
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: ?This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.?
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS ?AS IS? AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


**********************************************************************
* $Author:: spiesra $  $Date:: 2017-05-22 15:15:31#$ $Rev:: 63721   $
**********************************************************************/
#include "stdafx.h"
#include "cDepthProcessing.h"
#include <fstream>

#include <math.h>
 


#define HSV_UPPER_H_HIGH				"HSV upper::H High"
#define HSV_UPPER_H_LOW                                 "HSV upper::H Low"
#define HSV_UPPER_S_HIGH				"HSV upper::S High"
#define HSV_UPPER_S_LOW                                 "HSV upper::S Low"
#define HSV_UPPER_V_HIGH				"HSV upper::V High"
#define HSV_UPPER_V_LOW                                 "HSV upper::V Low"

#define HSV_MIDDLE_H_HIGH				"HSV middle::H High"
#define HSV_MIDDLE_H_LOW                                "HSV middle::H Low"
#define HSV_MIDDLE_S_HIGH				"HSV middle::S High"
#define HSV_MIDDLE_S_LOW                                "HSV middle::S Low"
#define HSV_MIDDLE_V_HIGH				"HSV middle::V High"
#define HSV_MIDDLE_V_LOW                                "HSV middle::V Low"

#define HSV_LOW_H_HIGH                                  "HSV low::H High"
#define HSV_LOW_H_LOW                                   "HSV low::H Low"
#define HSV_LOW_S_HIGH                                  "HSV low::S High"
#define HSV_LOW_S_LOW                                   "HSV low::S Low"
#define HSV_LOW_V_HIGH                                  "HSV low::V High"
#define HSV_LOW_V_LOW                                   "HSV low::V Low"



#define DISPLACEMENT_X1                                 "ROI::Displacement x1"
#define DISPLACEMENT_X2                                 "ROI::Displacement x2"
#define FACTOR_X1                                       "ROI::Factor x1"
#define FACTOR_X2                                       "ROI::Factor x2"



// define the ADTF property names to avoid errors
ADTF_FILTER_PLUGIN("Depth Processing", OID_ADTF_DepthProcessing, cDepthProcessing)




cDepthProcessing::cDepthProcessing(const tChar* __info) : cFilter(__info)
{
    // Pedestrian Detection
    SetPropertyBool("Pedestrian Detection::Enable saving data", tFalse);
    SetPropertyInt("Pedestrian Detection::Counter for Frames per Second", 30);

    // Get distance to object
    SetPropertyFloat("Get Distance to object::Factor for shifting RoI", 10);
    SetPropertyFloat("Get Distance to object::min Area", 300);


    // Get distance to object
    SetPropertyFloat("RESIZED_IMG::WIDTH", 30);
    SetPropertyFloat("RESIZED_IMG::HEIGHT", 30);


    // Displacement - RGB-Depth
    SetPropertyFloat(DISPLACEMENT_X1, 20);
    SetPropertyBool(DISPLACEMENT_X1 NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(DISPLACEMENT_X1 NSSUBPROP_DESCRIPTION, "Displacement x1");
    SetPropertyFloat(DISPLACEMENT_X2, 10);
    SetPropertyBool(DISPLACEMENT_X2 NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(DISPLACEMENT_X2 NSSUBPROP_DESCRIPTION, "Displacement x2");

    SetPropertyFloat(FACTOR_X1, 0.05);
    SetPropertyBool(FACTOR_X1 NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(FACTOR_X1 NSSUBPROP_DESCRIPTION, "Factor x1");
    SetPropertyFloat(FACTOR_X2, 0.05);
    SetPropertyBool(FACTOR_X2 NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(FACTOR_X2 NSSUBPROP_DESCRIPTION, "HFactor x2");


    // HSV properties - UPPER
    SetPropertyInt(HSV_UPPER_H_HIGH, 30);
    SetPropertyBool(HSV_UPPER_H_HIGH NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(HSV_UPPER_H_HIGH NSSUBPROP_DESCRIPTION, "HSV upper value H high");
    SetPropertyInt(HSV_UPPER_H_LOW, 0);
    SetPropertyBool(HSV_UPPER_H_LOW NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(HSV_UPPER_H_LOW NSSUBPROP_DESCRIPTION, "HSV upper value H low");

    SetPropertyInt(HSV_UPPER_S_HIGH, 75);
    SetPropertyBool(HSV_UPPER_S_HIGH NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(HSV_UPPER_S_HIGH NSSUBPROP_DESCRIPTION, "HSV upper value S high");
    SetPropertyInt(HSV_UPPER_S_LOW, 5);
    SetPropertyBool(HSV_UPPER_S_LOW NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(HSV_UPPER_S_LOW NSSUBPROP_DESCRIPTION, "HSV upper value S low");

    SetPropertyInt(HSV_UPPER_V_HIGH, 255);
    SetPropertyBool(HSV_UPPER_V_HIGH NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(HSV_UPPER_V_HIGH NSSUBPROP_DESCRIPTION, "HSV upper value V high");
    SetPropertyInt(HSV_UPPER_V_LOW, 100);
    SetPropertyBool(HSV_UPPER_V_LOW NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(HSV_UPPER_V_LOW NSSUBPROP_DESCRIPTION, "HSV upper value V low");



    // HSV properties - MIDDLE
    SetPropertyInt(HSV_MIDDLE_H_HIGH, 200);
    SetPropertyBool(HSV_MIDDLE_H_HIGH NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(HSV_MIDDLE_H_HIGH NSSUBPROP_DESCRIPTION, "HSV middle value H high");
    SetPropertyInt(HSV_MIDDLE_H_LOW, 0);
    SetPropertyBool(HSV_MIDDLE_H_LOW NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(HSV_MIDDLE_H_LOW NSSUBPROP_DESCRIPTION, "HSV middle value H low");

    SetPropertyInt(HSV_MIDDLE_S_HIGH, 200);
    SetPropertyBool(HSV_MIDDLE_S_HIGH NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(HSV_MIDDLE_S_HIGH NSSUBPROP_DESCRIPTION, "HSV middle value S high");
    SetPropertyInt(HSV_MIDDLE_S_LOW, 75);
    SetPropertyBool(HSV_MIDDLE_S_LOW NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(HSV_MIDDLE_S_LOW NSSUBPROP_DESCRIPTION, "HSV middle value S low");

    SetPropertyInt(HSV_MIDDLE_V_HIGH, 200);
    SetPropertyBool(HSV_MIDDLE_V_HIGH NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(HSV_MIDDLE_V_HIGH NSSUBPROP_DESCRIPTION, "HSV middle value V high");
    SetPropertyInt(HSV_MIDDLE_V_LOW, 100);
    SetPropertyBool(HSV_MIDDLE_V_LOW NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(HSV_MIDDLE_V_LOW NSSUBPROP_DESCRIPTION, "HSV middle value V low");


    // HSV properties - LOW
    SetPropertyInt(HSV_LOW_H_HIGH,50);
    SetPropertyBool(HSV_LOW_H_HIGH NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(HSV_LOW_H_HIGH NSSUBPROP_DESCRIPTION, "HSV low value H high");
    SetPropertyInt(HSV_LOW_H_LOW,0);
    SetPropertyBool(HSV_LOW_H_LOW NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(HSV_LOW_H_LOW NSSUBPROP_DESCRIPTION, "HSV low value H low");

    SetPropertyInt(HSV_LOW_S_HIGH,50);
    SetPropertyBool(HSV_LOW_S_HIGH NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(HSV_LOW_S_HIGH NSSUBPROP_DESCRIPTION, "HSV low value S high");
    SetPropertyInt(HSV_LOW_S_LOW,0);
    SetPropertyBool(HSV_LOW_S_LOW NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(HSV_LOW_S_LOW NSSUBPROP_DESCRIPTION, "HSV low value S low");

    SetPropertyInt(HSV_LOW_V_HIGH,50);
    SetPropertyBool(HSV_LOW_V_HIGH NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(HSV_LOW_V_HIGH NSSUBPROP_DESCRIPTION, "HSV low value V high");
    SetPropertyInt(HSV_LOW_V_LOW,0);
    SetPropertyBool(HSV_LOW_V_LOW NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(HSV_LOW_V_LOW NSSUBPROP_DESCRIPTION, "HSV low value V low");




}

cDepthProcessing::~cDepthProcessing()
{
}

tResult cDepthProcessing::Start(__exception)
{

    return cFilter::Start(__exception_ptr);
}

tResult cDepthProcessing::Stop(__exception)
{
    return cFilter::Stop(__exception_ptr);
}

tResult cDepthProcessing::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));

    if (eStage == StageFirst)
    {
        cObjectPtr<IMediaDescriptionManager> pDescManager;
        RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER, (tVoid**)&pDescManager, __exception_ptr));

        // Video Input - Depth
        RETURN_IF_FAILED(m_oVideoInputPin.Create("Video Input DEPTH", IPin::PD_Input, static_cast<IPinEventSink*>(this)));
        RETURN_IF_FAILED(RegisterPin(&m_oVideoInputPin));

        // Video Input - RGB
        RETURN_IF_FAILED(m_oVideoInputPinRGB.Create("Video Input RGB", IPin::PD_Input, static_cast<IPinEventSink*>(this)));
        RETURN_IF_FAILED(RegisterPin(&m_oVideoInputPinRGB));

        // Input - Steering Angle
        tChar const * strSteeringAngle = pDescManager->GetMediaDescription("tSignalValue");
        RETURN_IF_POINTER_NULL(strSteeringAngle);
        cObjectPtr<IMediaType> pTypeSteeringAngle = new cMediaType(0, 0, 0, "tSignalValue", strSteeringAngle, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(pTypeSteeringAngle->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionInputSteeringAngle));
        RETURN_IF_FAILED(m_oInputSteeringAngle.Create("Steering Angle", pTypeSteeringAngle, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oInputSteeringAngle));

        // Input - Check traffic
        tChar const * strInputCheckTraffic = pDescManager->GetMediaDescription("tBoolSignalValue");
        RETURN_IF_POINTER_NULL(strInputCheckTraffic);
        cObjectPtr<IMediaType> pTypeInputCheckTraffic = new cMediaType(0, 0, 0, "tBoolSignalValue", strInputCheckTraffic, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(pTypeInputCheckTraffic->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionInputCheckTraffic));
        RETURN_IF_FAILED(m_oInputCheckTraffic.Create("Check traffic start", pTypeInputCheckTraffic, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oInputCheckTraffic));


        // Output - Video RGB cut RoI
        RETURN_IF_FAILED(m_oVideoOutputPin.Create("Video_Output", IPin::PD_Output, static_cast<IPinEventSink*>(this)));
        RETURN_IF_FAILED(RegisterPin(&m_oVideoOutputPin));


        // Output - AEB activation
        tChar const * strOutputAEB = pDescManager->GetMediaDescription("tBoolSignalValue");
        RETURN_IF_POINTER_NULL(strOutputAEB);
        cObjectPtr<IMediaType> pTypeOutputAEB = new cMediaType(0, 0, 0, "tBoolSignalValue", strOutputAEB, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(pTypeOutputAEB->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionOutputAEB));
        RETURN_IF_FAILED(m_oOutputAEB.Create("AEB", pTypeOutputAEB, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oOutputAEB));


        // Output - ACC Distance
        tChar const * strOutputACCdistance = pDescManager->GetMediaDescription("tSignalValue");
        RETURN_IF_POINTER_NULL(strOutputACCdistance);
        cObjectPtr<IMediaType> pTypeOutputACCdistance = new cMediaType(0, 0, 0, "tSignalValue", strOutputACCdistance, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(pTypeOutputACCdistance->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionOutputACCdistance));
        RETURN_IF_FAILED(m_oOutputACCdistance.Create("ACC Distance", pTypeOutputACCdistance, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oOutputACCdistance));


        // Output - Check traffic feedback
        tChar const * strOutputTrafficFeedback = pDescManager->GetMediaDescription("tSignalValue");
        RETURN_IF_POINTER_NULL(strOutputTrafficFeedback);
        cObjectPtr<IMediaType> pTypeOutputTrafficFeedback = new cMediaType(0, 0, 0, "tSignalValue", strOutputTrafficFeedback, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(pTypeOutputTrafficFeedback->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionOutputTrafficFeedback));
        RETURN_IF_FAILED(m_oOutputTrafficFeedback.Create("Traffic Feedback", pTypeOutputTrafficFeedback, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oOutputTrafficFeedback));


        // Output - Vector and position
        tChar const * strOutput = pDescManager->GetMediaDescription("tVectorStruct");
        RETURN_IF_POINTER_NULL(strOutput);
        cObjectPtr<IMediaType> pTypeOutput = new cMediaType(0, 0, 0, "tVectorStruct", strOutput, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(pTypeOutput->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&pDescriptionOutputVector));
        RETURN_IF_FAILED(m_oOutputVector.Create("Vector and Position", pTypeOutput, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oOutputVector));
    }
    else if (eStage == StageNormal)
    {
        // Pedestrian Detection
        ct = 0;
        m_bEnableSaveData =  GetPropertyBool("Pedestrian Detection::Enable saving data");
        m_iCounterForFrames = GetPropertyInt("Pedestrian Detection::Counter for Frames per Second");

        // Get distance to object
        m_f32Factor4RoIShift = GetPropertyFloat("Get Distance to object::Factor for shifting RoI");
        m_f32AreaMin = GetPropertyFloat("Get Distance to object::min Area");




        m_iResizedImg_Height = GetPropertyFloat("RESIZED_IMG::WIDTH");
        m_iResizedImg_Width = GetPropertyFloat("RESIZED_IMG::HEIGHT");



        m_f32InputSteeringAngle = 0;
    }

    else if (eStage == StageGraphReady)
    {
        // get the image format of the input video pin
        cObjectPtr<IMediaType> pType;
        RETURN_IF_FAILED(m_oVideoInputPin.GetMediaType(&pType));

        cObjectPtr<IMediaTypeVideo> pTypeVideo;
        RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid**)&pTypeVideo));

        // set the image format of the input video pin
        // set the image format of the input video pin
        if (IS_FAILED(UpdateInputImageFormat(pTypeVideo->GetFormat())))
        {
            LOG_ERROR("Invalid Input Format for this filter");
        }
    }

    RETURN_NOERROR;
}

tResult cDepthProcessing::Shutdown(tInitStage eStage, ucom::IException** __exception_ptr)
{
    if (eStage == StageGraphReady)
    {
    }

    return cFilter::Shutdown(eStage, __exception_ptr);
}

tResult cDepthProcessing::PropertyChanged(const char* strProperty)
{
        ReadProperties(strProperty);
        RETURN_NOERROR;
}

tResult cDepthProcessing::ReadProperties(const tChar* strPropertyName)
{
//check properties for lanefollow front
        // DISPLACEMENT - RGB-Depth
        if (NULL == strPropertyName || cString::IsEqual(strPropertyName, DISPLACEMENT_X1))
        {
                m_f32Displacement_x1 = static_cast<tFloat32> (GetPropertyFloat(DISPLACEMENT_X1));
        }
        if (NULL == strPropertyName || cString::IsEqual(strPropertyName, DISPLACEMENT_X2))
        {
                m_f32Displacement_x2 = static_cast<tFloat32> (GetPropertyFloat(DISPLACEMENT_X2));
        }
        if (NULL == strPropertyName || cString::IsEqual(strPropertyName, FACTOR_X1))
        {
                m_f32Factor_x1 = static_cast<tFloat32> (GetPropertyFloat(FACTOR_X1));
        }
        if (NULL == strPropertyName || cString::IsEqual(strPropertyName, FACTOR_X2))
        {
                m_f32Factor_x2 = static_cast<tFloat32> (GetPropertyFloat(FACTOR_X2));
        }



        // UPPER - Properties
        if (NULL == strPropertyName || cString::IsEqual(strPropertyName, HSV_UPPER_H_HIGH))
        {
                m_iHighH_upper = static_cast<tInt> (GetPropertyInt(HSV_UPPER_H_HIGH));
        }
        if (NULL == strPropertyName || cString::IsEqual(strPropertyName, HSV_UPPER_H_LOW))
        {
                m_iLowH_upper = static_cast<tInt> (GetPropertyInt(HSV_UPPER_H_LOW));
        }
        if (NULL == strPropertyName || cString::IsEqual(strPropertyName, HSV_UPPER_S_HIGH))
        {
                m_iHighS_upper = static_cast<tInt> (GetPropertyInt(HSV_UPPER_S_HIGH));
        }
        if (NULL == strPropertyName || cString::IsEqual(strPropertyName, HSV_UPPER_S_LOW))
        {
                m_iLowS_upper = static_cast<tInt> (GetPropertyInt(HSV_UPPER_S_LOW));
        }
        if (NULL == strPropertyName || cString::IsEqual(strPropertyName, HSV_UPPER_V_HIGH))
        {
                m_iHighV_upper = static_cast<tInt> (GetPropertyInt(HSV_UPPER_V_HIGH));
        }
        if (NULL == strPropertyName || cString::IsEqual(strPropertyName, HSV_UPPER_V_LOW))
        {
                m_iLowV_upper = static_cast<tInt> (GetPropertyInt(HSV_UPPER_V_LOW));
        }

        // MIDDLE - Properties
        if (NULL == strPropertyName || cString::IsEqual(strPropertyName, HSV_MIDDLE_H_HIGH))
        {
                m_iHighH_middle = static_cast<tInt> (GetPropertyInt(HSV_MIDDLE_H_HIGH));
        }
        if (NULL == strPropertyName || cString::IsEqual(strPropertyName, HSV_MIDDLE_H_LOW))
        {
                m_iLowH_middle = static_cast<tInt> (GetPropertyInt(HSV_MIDDLE_H_LOW));
        }
        if (NULL == strPropertyName || cString::IsEqual(strPropertyName, HSV_MIDDLE_S_HIGH))
        {
                m_iHighS_middle = static_cast<tInt> (GetPropertyInt(HSV_MIDDLE_S_HIGH));
        }
        if (NULL == strPropertyName || cString::IsEqual(strPropertyName, HSV_MIDDLE_S_LOW))
        {
                m_iLowS_middle = static_cast<tInt> (GetPropertyInt(HSV_MIDDLE_S_LOW));
        }
        if (NULL == strPropertyName || cString::IsEqual(strPropertyName, HSV_MIDDLE_V_HIGH))
        {
                m_iHighV_middle = static_cast<tInt> (GetPropertyInt(HSV_MIDDLE_V_HIGH));
        }
        if (NULL == strPropertyName || cString::IsEqual(strPropertyName, HSV_MIDDLE_V_LOW))
        {
                m_iLowV_middle = static_cast<tInt> (GetPropertyInt(HSV_MIDDLE_V_LOW));
        }

        // LOW - Properties
        if (NULL == strPropertyName || cString::IsEqual(strPropertyName, HSV_LOW_H_HIGH))
        {
                m_iHighH_low = static_cast<tInt> (GetPropertyInt(HSV_LOW_H_HIGH));
        }
        if (NULL == strPropertyName || cString::IsEqual(strPropertyName, HSV_LOW_H_LOW))
        {
                m_iLowH_low = static_cast<tInt> (GetPropertyInt(HSV_LOW_H_LOW));
        }
        if (NULL == strPropertyName || cString::IsEqual(strPropertyName, HSV_LOW_S_HIGH))
        {
                m_iHighS_low = static_cast<tInt> (GetPropertyInt(HSV_LOW_S_HIGH));
        }
        if (NULL == strPropertyName || cString::IsEqual(strPropertyName, HSV_LOW_S_LOW))
        {
                m_iLowS_low = static_cast<tInt> (GetPropertyInt(HSV_LOW_S_LOW));
        }
        if (NULL == strPropertyName || cString::IsEqual(strPropertyName, HSV_LOW_V_HIGH))
        {
                m_iHighV_low = static_cast<tInt> (GetPropertyInt(HSV_LOW_V_HIGH));
        }
        if (NULL == strPropertyName || cString::IsEqual(strPropertyName, HSV_LOW_V_LOW))
        {
                m_iLowV_low = static_cast<tInt> (GetPropertyInt(HSV_LOW_V_LOW));
        }


        RETURN_NOERROR;
}

tResult cDepthProcessing::OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{

    RETURN_IF_POINTER_NULL(pMediaSample);
    RETURN_IF_POINTER_NULL(pSource);
    if (nEventCode == IPinEventSink::PE_MediaSampleReceived)
    {
        // Depth input received
        if (pSource == &m_oVideoInputPin)
        {
            if(m_iDepthImageCounter > m_iCounterForFrames)
            {
                //check if video format is still unkown
                if (m_sInputFormat.nPixelFormat == IImage::PF_UNKNOWN)
                {
                    RETURN_IF_FAILED(UpdateInputImageFormat(m_oVideoInputPin.GetFormat()));
                }
                ProcessVideo(pMediaSample);
                m_iDepthImageCounter = 0;
            }
            m_iDepthImageCounter++;
        }

        // RGB Input received
        else if (pSource == &m_oVideoInputPinRGB)
        {
            //check if video format is still unkown
            if (m_sInputFormat.nPixelFormat == IImage::PF_UNKNOWN)
            {
                RETURN_IF_FAILED(UpdateInputImageFormat(m_oVideoInputPinRGB.GetFormat()));
            }

            RETURN_IF_POINTER_NULL(pMediaSample);
            const tVoid* l_pSrcBufferRGB;
            IplImage* imgRGB;
            imgRGB = cvCreateImageHeader(cvSize(m_sInputFormat.nWidth, m_sInputFormat.nHeight), IPL_DEPTH_8U, 3);
            pMediaSample->Lock(&l_pSrcBufferRGB);
            // Get the image
            imgRGB->imageData = (char*)l_pSrcBufferRGB;
            // Set the image as a Mat
            cv::Mat matImageRGB;
            matImageRGB = cvarrToMat(imgRGB);
            pMediaSample->Unlock(&l_pSrcBufferRGB);
        //    imwrite("test/___Last_run/05_RGB.jpg",matImageRGB );
            m_matImageRGB = matImageRGB.clone();
        }

        // Check Traffic order received
        else if(pSource == &m_oInputCheckTraffic)
        {

        }

        // Current Steering angle received
        else if(pSource == &m_oInputSteeringAngle)
        {
            // Read out steering angle
            cObjectPtr<IMediaCoder> pCoderInput;
            RETURN_IF_FAILED(m_pDescriptionInputSteeringAngle->Lock(pMediaSample, &pCoderInput));
            pCoderInput->Get("f32Value", (tVoid*)&m_f32InputSteeringAngle);
            // pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timestamp);
            m_pDescriptionInputSteeringAngle->Unlock(pCoderInput);
        }

    }
    else if (nEventCode == IPinEventSink::PE_MediaTypeChanged)
    {
        if (pSource == &m_oVideoInputPin)
        {
            //the input format was changed, so the imageformat has to changed in this filter also
            RETURN_IF_FAILED(UpdateInputImageFormat(m_oVideoInputPin.GetFormat()));
        }
        else if (pSource == &m_oVideoInputPinRGB)
        {
            //the input format was changed, so the imageformat has to changed in this filter also
            RETURN_IF_FAILED(UpdateInputImageFormat(m_oVideoInputPinRGB.GetFormat()));
        }

        RETURN_IF_POINTER_NULL(pMediaSample);

    }
    RETURN_NOERROR;
}

tResult cDepthProcessing::ProcessVideo(IMediaSample* pSample)
{
    RETURN_IF_POINTER_NULL(pSample);

    /* Create buffer-pointer to set on data of input stream and create temporary header for referencing data */
    const tVoid* srcBuffer;
    IplImage* img = cvCreateImageHeader(cvSize(m_sInputFormat.nWidth, m_sInputFormat.nHeight), IPL_DEPTH_16U, 1);
    RETURN_IF_FAILED(pSample->Lock(&srcBuffer));
    img->imageData = (char*) srcBuffer;
    /* Set reference onto data to 'image' (method argument) */
    cv::Mat matImageDepth;
    matImageDepth = cvarrToMat(img);   // convert to Mat-type
    cvReleaseImage(&img);      // realese img to decrease reference-counter, expressing img-reference will not be needed anymore
    pSample->Unlock(srcBuffer); // Unlock buffer again

  //  imwrite("test/___Last_run/01_Depth_16bit.jpg", matImageDepth);


    // Scale the image from 16 to 8 bit
    cv::Mat matImageDepth8U;
    matImageDepth.convertTo(matImageDepth8U, CV_8U, 0.00390625);
   // imwrite("test/___Last_run/02_Depth_8bit.jpg", matImageDepth8U);

    // Cut the image
    Mat matImageDepthCUT;
    matImageDepthCUT = Mat(600, 256, CV_8UC1);
    matImageDepthCUT = matImageDepth8U(Range(140, 140+256),Range(1, 40+600)).clone();
  //  imwrite("test/___Last_run/03_Depth_Cut.jpg", matImageDepthCUT);

    // Morphological opening (removes small object from the foreground)
    erode(matImageDepthCUT,matImageDepthCUT,getStructuringElement(MORPH_ELLIPSE, Size(5,5)));
    dilate(matImageDepthCUT,matImageDepthCUT,getStructuringElement(MORPH_ELLIPSE, Size(5,5)));

    // Morphological closing (removes small holes from the foreground)
    dilate(matImageDepthCUT,matImageDepthCUT,getStructuringElement(MORPH_ELLIPSE, Size(5,5)));
    erode(matImageDepthCUT,matImageDepthCUT,getStructuringElement(MORPH_ELLIPSE, Size(5,5)));

    imwrite("test/___Last_run/04_Depth_RedNoise.jpg", matImageDepthCUT);



    PedestrianDetection(matImageDepthCUT);


    // GetDistanceToObject(matImageDepthCUT);


    /*
    // received a trigger for checking traffic
    if( )
    {
        CrossingFeedback(matImageDepthCUT);
    }

    */


    RETURN_NOERROR;
}



tResult cDepthProcessing::PedestrianDetection(cv::Mat matImageDepthCUT)
{
    Moments oMoments;
    oMoments = moments(matImageDepthCUT);

    tInt posY;
    tInt posX;

    tFloat32 f32DepthValue;
    tFloat32 f32Area;

    tFloat32 f32AreaMin;
    tFloat32 f32AreaMax;
    f32AreaMin = 0;
    f32AreaMax = 0;

    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;

    // Find contours
    cv::findContours(matImageDepthCUT, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

    // Get the 0
    std::vector<Moments> mu(contours.size() );
    for(int i = 0; i < contours.size(); i++)
    {
        mu[i] = moments(contours[i], false);
    }

    vector<vector<Point> > contours_poly( contours.size() );
    vector<Rect> boundRect( contours.size() );
    vector<Point2f>center( contours.size() );
    vector<float>radius( contours.size() );


    // Get the mass centers:
    std::vector<cv::Point2f> mc(contours.size());
    for(int i = 0; i < contours.size(); i++)
    {

        // apply bounding boxes to get the width and height
        approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
        boundRect[i] = boundingRect(contours[i]);
        // boundRect[i] = boundingRect( Mat(contours_poly[i]));

        tFloat32 boxWidth;
        boxWidth = boundRect[i].width;
        tFloat32 boxHeight;
        boxHeight = boundRect[i].height;


        f32Area = mu[i].m00;
        posY = mu[i].m01 / mu[i].m00 ;
        posX = mu[i].m10 / mu[i].m00 ;
        mc[i] = Point2f(posX , posY);

        if(posX < 6)
        {
            posX = 6;
        }
        else if(posX > 595)
        {
            posX = 595;
        }

        if(posY < 6){
            posY = 6;
        }
        else if(posY > 250){
            posY = 250;
        }

        f32DepthValue = 0;
        f32DepthValue = matImageDepthCUT.at<uchar>(posY, posX) +
                        matImageDepthCUT.at<uchar>(posY+5, posX+5) +
                        matImageDepthCUT.at<uchar>(posY+5, posX-5) +
                        matImageDepthCUT.at<uchar>(posY-5, posX-5) +
                        matImageDepthCUT.at<uchar>(posY-5, posX+5);
        f32DepthValue = f32DepthValue / 5;
        tBool bObjectDetected;
        bObjectDetected = tFalse;

        // Adapt the treshhold of the area to the distance

        // Distance::Close
        if(50 < f32DepthValue  && f32DepthValue < 90)
        {
            f32AreaMin =  1900;
            f32AreaMax = 25000;
            bObjectDetected = tTrue;
        }
        // Distance::LOW
        if(90 < f32DepthValue  && f32DepthValue < 120)
        {
            f32AreaMin =  1000;
            f32AreaMax = 10000;
            bObjectDetected = tTrue;
        }
        // Distance::Far
        if(120 < f32DepthValue  && f32DepthValue < 180)
        {
            f32AreaMin =   250;
            f32AreaMax =  9000;
            bObjectDetected = tTrue;
        }
        // Distance:: To far to detect a pedestrian
        if( /* f32DepthValue < 50 && */ 180 < f32DepthValue)
        {
            f32AreaMin =     0;
            f32AreaMax =     0;
            bObjectDetected = tFalse;
        }




        // Map Depth value into cm from the front of the vehicle
        f32DepthValue = (f32DepthValue + (10 + (f32DepthValue-60)/8) - 20);
      //  LOG_INFO(adtf_util::cString::Format("Object %i, Area %f, Depth %f, boxHeight %f, boxWidth %f",i+1 ,f32Area, f32DepthValue, boxHeight, boxWidth));


        // Check if the detected objects are in the exspected
        if(f32AreaMin < f32Area && f32Area < f32AreaMax && f32AreaMin != 0 && f32AreaMax != 0 && bObjectDetected)
        {

            // Maybe put this in a function     ->      cutROIfromRGB(posX , posY);
            cv::Mat matImageRGB;
            matImageRGB = Mat(640, 480, CV_8UC3);
            matImageRGB= m_matImageRGB.clone(); //(Range(1, 480),Range(1, 640))

            imwrite("test/___Last_run/05RGB_2.jpg", matImageRGB);
            cv::Mat matImageGREY;
            cvtColor(matImageRGB, matImageGREY, CV_BGR2GRAY);

        //    imwrite("test/___Last_run/05_GREY_2.jpg", matImageGREY);
            // Precut the RGB image to get the same format
            cv::Mat matImageRGBprecut = Mat(640, 256, CV_8UC3);
            matImageRGBprecut = matImageRGB(Range(140, 140+256),Range(1, 40+600)).clone();
        //    imwrite("test/___Last_run/06_RGB_PreCut.jpg", matImageRGBprecut);

            cv::Mat matImageDepthBoundingBox;
            matImageDepthBoundingBox = matImageDepthCUT(Range(boundRect[i].y, boundRect[i].y + boundRect[i].height), Range(boundRect[i].x, boundRect[i].x + boundRect[i].width)).clone();
            imwrite("test/06_Depth_BoundingBox.jpg", matImageDepthBoundingBox);



            tInt x1;
            tInt x2;
            tInt y1;
            tInt y2;
            // Height
            y1 = boundRect[i].y - 5;
            y2 = boundRect[i].y + boundRect[i].height +  5;
            // Width
            x1 = boundRect[i].x                             - m_f32Displacement_x1 - ((boundRect[i].x - 300) * - m_f32Factor_x1);
            x2 = boundRect[i].x + boundRect[i].width        - m_f32Displacement_x2 + ((boundRect[i].x - 300) * - m_f32Factor_x2);




            if(y1 < 1)
            {
                y1 = 1;
            }
            if(255 < y2)
            {
                y2 = 255;
            }

            if(x1 < 1)
            {
                x1 = 1;
            }
            if(639 < x2)
            {
                x2 = 639;
            }

            if(x1 > x2 || y1 > y2)
            {
                RETURN_NOERROR;
            }

            // LOG_INFO(adtf_util::cString::Format("matImageRGBprecut: height = %i, width = %i", matImageRGBprecut.size().height, matImageRGBprecut.size().width));

            // LOG_INFO(adtf_util::cString::Format(" boundRect[i].height = %i, boundRect[i].width20 = %i",boundRect[i].height, boundRect[i].width));
            // LOG_INFO(adtf_util::cString::Format("matImageRGB_CorrPrecut: y1 = %i, y2 = %i, x1 = %i, x2 = %i",y1, y2, x1, x2));

            // Cut the RGB image regarding to the bounding boxes, but cut a little bit more due to the shift
            cv::Mat matImageRGB_CorrPrecut;
            matImageRGB_CorrPrecut = matImageRGBprecut(Range(y1, y2), Range(x1, x2)).clone();
            stringstream ssname;
            // ssname<<("test/06_RGB_CorrPrecut_x_")<<(boundRect[i].x)<<(".jpg");
            ssname<<("test/06_RGB_CorrPrecut_x_")<<(".jpg");
            String name = ssname.str();
            imwrite(name, matImageRGB_CorrPrecut);

            // Convert the image into GREY
            cv::Mat matImageGREY_CorrPrecut;
            cvtColor(matImageRGB_CorrPrecut, matImageGREY_CorrPrecut, CV_BGR2GRAY);
   //         imwrite("test/07_GREY_CorrPrecut.jpg", matImageGREY_CorrPrecut);


            tBool bImage2send = tFalse;
            cv::Mat matImageRoICut;

            // Image LOW part - hsv low
            // ADULT
            if(boundRect[i].height > boundRect[i].width * 2 && boundRect[i].width > 10)
            {
                string typeJPG = ".jpg";
                // Cut the low part of the image
                cv::Mat matImageRoIlow;

                tInt y1PreCut = 0;
                if(y2 - y1 < 100)
                {
                    y1PreCut = (y2 - y1)*0.8;
                }
                else
                {
                    y1PreCut = 50;
                }
                matImageRoIlow = matImageRGB_CorrPrecut(Range(y1PreCut, y2 - y1), Range(0, x2 - x1)).clone();
                stringstream sslow;
                string low ="test/HSV_low/Data_";
                sslow<<low<<(ct+1)<<("_")<<(i+1)<<("_1matImageRoIlow_")<<typeJPG;
                string filelow = sslow.str();
             //   imwrite(filelow, matImageRoIlow);

                // convert the bgr-img in a hsv-img
                cv::Mat matImageHSV;
                cvtColor(matImageRoIlow, matImageHSV, COLOR_BGR2HSV);
                stringstream ssHSV;
                string HSV ="test/HSV_low/Data_";
                ssHSV<<HSV<<(ct+1)<<("_")<<(i+1)<<("_2HSV_low")<<typeJPG;
                string fileHSV = ssHSV.str();
            //    imwrite(fileHSV, matImageHSV);

                // apply the mask on the hsv image
                cv::Mat matImageHSVlow;
                inRange(matImageHSV, Scalar(m_iLowH_low, m_iLowS_low, m_iLowV_low), Scalar(m_iHighH_low, m_iHighS_low, m_iHighV_low), matImageHSVlow);
                stringstream ssHSVyellow;
                string HSVyellow ="test/HSV_low/Data_";
                ssHSVyellow<<HSVyellow<<(ct+1)<<("_")<<(i+1)<<("_3Filtered")<<typeJPG;
                string fileHSVyellow = ssHSVyellow.str();
            //    imwrite(fileHSVyellow, matImageHSVlow);

                // Get the 0
                Moments oMomentsLow;
                oMomentsLow = moments(matImageHSVlow);

                // Center of mass
                f32Area = oMomentsLow.m00;
                posY = oMomentsLow.m01 / oMomentsLow.m00 ;
                posX = oMomentsLow.m10 / oMomentsLow.m00 ;

                tInt iCutx1 = posX-20;
                tInt iCutx2 = posX+20;
                if(iCutx1 < 0)
                {
                    iCutx1 = 0;
                }
                if(x2 - x1 < iCutx2)
                {
                    iCutx2 = x2 - x1;
                }
                // LOG_INFO(adtf_util::cString::Format("Area %f, posX %i, posY %i",f32Area, posX, posY));
                // Cut the low part of the image

                if(iCutx2 > 10)
                {
                    //LOG_INFO(adtf_util::cString::Format("height = %i, width = %i", matImageRGB_CorrPrecut.size().height, matImageRGB_CorrPrecut.size().width));
                    //LOG_INFO(adtf_util::cString::Format("iCuty1 = %i, iCuty2 = %i, iCutx1 = %i, iCutx2 = %i", 0 ,y2 - y1, iCutx1, iCutx2));
                    matImageRoICut = matImageRGB_CorrPrecut(Range(0, y2-y1), Range(iCutx1, iCutx2)).clone();
                    // stringstream ssRGBCUT;
                    // string RGBCUT ="test/08_RGB_CUT_";
                    // ssRGBCUT<<RGBCUT<<(ct+1)<<typeJPG;
                    // string fileRGBCUT = ssRGBCUT.str();
              //      imwrite("test/08_RGB_CUT.jpg", matImageRoICut);

                    bImage2send = tTrue;

                }
            }
            //  Image MIDDLE part - hsv middle
            //  CHILD

            else
            {
                    string typeJPG = ".jpg";

                    // Cut the middle part of the image
                    cv::Mat matImageRoImiddle;

                    tInt iWidth2Cut = 0;
                    if(matImageRGB_CorrPrecut.size().width < boundRect[i].width)
                    {
                        iWidth2Cut = matImageRGB_CorrPrecut.size().width;
                    }
                    else
                    {
                        iWidth2Cut = boundRect[i].width;
                    }

                    tInt y1PreCut = 0;
                    if(y2 - y1 < 100)
                    {
                        y1PreCut = (y2 - y1)*0.8;
                    }
                    else
                    {
                        y1PreCut = 50;
                    }

                    matImageRoImiddle = matImageRGB_CorrPrecut(Range(y1PreCut, y2 - y1), Range(0, x2 - x1)).clone();
                    stringstream ssmiddle;
                    string middle ="test/HSV_middle/Data_";
                    ssmiddle<<middle<<(ct+1)<<("_")<<(i+1)<<("_1matImageRoImiddle")<<typeJPG;
                    string filemiddle = ssmiddle.str();
                    imwrite(filemiddle, matImageRoImiddle);

                    // convert the bgr-img in a hsv-img
                    cv::Mat matImageHSV;
                    cvtColor(matImageRoImiddle, matImageHSV, COLOR_BGR2HSV);
                    stringstream ssHSV;
                    string HSV ="test/HSV_middle/Data_";
                    ssHSV<<HSV<<(ct+1)<<("_")<<(i+1)<<("_2HSV_middle")<<typeJPG;
                    string fileHSV = ssHSV.str();
                    imwrite(fileHSV, matImageHSV);

                    // apply the mask on the hsv image
                    cv::Mat matImageHSVmiddle;
                    inRange(matImageHSV, Scalar(m_iLowH_middle, m_iLowS_middle, m_iLowV_middle), Scalar(m_iHighH_middle, m_iHighS_middle, m_iHighV_middle), matImageHSVmiddle);
                    stringstream ssHSVyellow;
                    string HSVyellow ="test/HSV_middle/Data_";
                    ssHSVyellow<<HSVyellow<<(ct+1)<<("_")<<(i+1)<<("_3Filtered")<<typeJPG;
                    string fileHSVyellow = ssHSVyellow.str();
                    imwrite(fileHSVyellow, matImageHSVmiddle);

                    // Get the 0
                    Moments oMomentsLow;
                    oMomentsLow = moments(matImageHSVmiddle);

                    // Center of mass
                    f32Area = oMomentsLow.m00;
                    posY = oMomentsLow.m01 / oMomentsLow.m00 ;
                    posX = oMomentsLow.m10 / oMomentsLow.m00 ;

                    tInt iCutx1 = posX-20;
                    tInt iCutx2 = posX+20;
                    if(iCutx1 < 0)
                    {
                        iCutx1 = 0;
                    }
                    if(x2 - x1 < iCutx2)
                    {
                        iCutx2 = x2 - x1;
                    }
                    //LOG_INFO(adtf_util::cString::Format("Area %f, posX %i, posY %i",f32Area, posX, posY));
                    // Cut the low part of the image

                    if(iCutx2 > 10)
                    {
                        //LOG_INFO(adtf_util::cString::Format("height = %i, width = %i", matImageRGB_CorrPrecut.size().height, matImageRGB_CorrPrecut.size().width));
                        //LOG_INFO(adtf_util::cString::Format("iCuty1 = %i, iCuty2 = %i, iCutx1 = %i, iCutx2 = %i", 0 ,y2-y1, iCutx1, iCutx2));
                        matImageRoICut = matImageRGB_CorrPrecut(Range(0, y2-y1), Range(iCutx1, iCutx2)).clone();
                        // stringstream ssRGBCUT;
                        // string RGBCUT ="test/08_RGB_CUT_";
                        // ssRGBCUT<<RGBCUT<<(ct+1)<<typeJPG;
                        // string fileRGBCUT = ssRGBCUT.str();
                        imwrite("test/08_RGB_CUT.jpg", matImageRoICut);

                        bImage2send = tTrue;
                    }
            }




/*
            // Displacement of RGB and Depth
            posX = posX - (640-200) * 0.15;
            if(posX < 200)
            {
                posX = posX  +40;
            }

            if(posX > 200 && posX < 400)
            {
                posX = posX +25;
            }
            if(posX > 400)
            {
                posX = posX + 20;
            }
            posY = posY + 10;


            // Ensure that posX and posY are not to close the the edge of the pic
            if(posX+50 > 550)
            {
                posX = 550;
            }
            else if(posX-50 < 1)
            {
                posX = 51;
            }

            if(posY+100 > 256)
            {
                posY = 156;
            }
            else if(posY-100 < 1)
            {
                posY = 101;
            }

            // Cut the RoI from the RGB-image
            cv::Mat matImageRGBRoIcut;
            matImageRGBRoIcut = Mat(100, 200, CV_8UC3);
            matImageRGBRoIcut = matImageRGBprecut(Range(posY-100, posY+100), Range(posX-50, posX+50)).clone();


            tInt cut_width;
            tInt x_start_cut;
            tInt x_end_cut;

            cut_width = 48;
            x_start_cut = 0;
            x_end_cut = cut_width;
*/




            // Rule based approach

/*
            // Split the image in upper - middle - low and search for specific colours
            for(tInt i = 0; i < 3; i++)
            {             
                tInt iNonZeroUpper;
                tInt iNonZeromiddle;
                tInt iNonZerolow;
                // Image upper part
                {
                    string typeJPG = ".jpg";

                    tInt iHeigth2Cut = matImageRoICut.size().width * 0.5;
                    // Cut the upper part of the image
                    cv::Mat matImageRoIUpper;
                    matImageRoIUpper = matImageRoICut(Range(0, iHeigth2Cut), Range(0, matImageRoICut.size().width)).clone();
                    stringstream ssUpper;
                    string upper ="test/HSV_upper/Data_";
                    ssUpper<<upper<<("_")<<(i+1)<<("_1matImageRoIUpper")<<typeJPG;
                    string fileUpper = ssUpper.str();
                    imwrite(fileUpper, matImageRoIUpper);

                    // convert the bgr-img in a hsv-img
                    cv::Mat matImageHSV;
                    cvtColor(matImageRoIUpper, matImageHSV, COLOR_BGR2HSV);
                    stringstream ssHSV;
                    string HSV ="test/HSV_upper/Data_";
                    ssHSV<<HSV<<("_")<<(i+1)<<("_2HSV_upper")<<typeJPG;
                    string fileHSV = ssHSV.str();
                    imwrite(fileHSV, matImageHSV);

                    // apply the mask on the hsv image
                    cv::Mat matImageHSVupper;
                    inRange(matImageHSV, Scalar(m_iLowH_upper, m_iLowS_upper, m_iLowV_upper), Scalar(m_iHighH_upper, m_iHighS_upper, m_iHighV_upper), matImageHSVupper);
                    stringstream ssHSVupper;
                    string HSVupper ="test/HSV_upper/Data_";
                    ssHSVupper<<HSVupper<<("_")<<(i+1)<<("_3Filtered")<<typeJPG;
                    string fileHSVupper = ssHSVupper.str();
                    imwrite(fileHSVupper, matImageHSVupper);

                    // Get the amount of non-zero pixels in the image
                    cv::Mat matNonZeroUpper;
                    findNonZero(matImageHSVupper, matNonZeroUpper);

                    iNonZeroUpper = matNonZeroUpper.total();
                    //LOG_INFO(adtf_util::cString::Format("Non zero upper %i",iNonZeroUpper));
                }


                // Image MIDDLE part - hsv middle
                {
                    string typeJPG = ".jpg";

                    // Cut the middle part of the image
                    cv::Mat matImageRoImiddle;
                    matImageRoImiddle = matImageRGBRoIcut(Range(60, 140), Range(x_start_cut,x_end_cut)).clone();
                    stringstream ssmiddle;
                    string middle ="test/HSV_middle/Data_";
                    ssmiddle<<middle<<(ct+1)<<("_")<<(i+1)<<("_1matImageRoImiddle")<<typeJPG;
                    string filemiddle = ssmiddle.str();
                    imwrite(filemiddle, matImageRoImiddle);

                    // convert the bgr-img in a hsv-img
                    cv::Mat matImageHSV;
                    cvtColor(matImageRoImiddle, matImageHSV, COLOR_BGR2HSV);
                    stringstream ssHSV;
                    string HSV ="test/HSV_middle/Data_";
                    ssHSV<<HSV<<(ct+1)<<("_")<<(i+1)<<("_2HSV_middle")<<typeJPG;
                    string fileHSV = ssHSV.str();
                    imwrite(fileHSV, matImageHSV);

                    // apply the mask on the hsv image
                    cv::Mat matImageHSVyellow;
                    inRange(matImageHSV, Scalar(m_iLowH_middle, m_iLowS_middle, m_iLowV_middle), Scalar(m_iHighH_middle, m_iHighS_middle, m_iHighV_middle), matImageHSVyellow);
                    stringstream ssHSVyellow;
                    string HSVyellow ="test/HSV_middle/Data_";
                    ssHSVyellow<<HSVyellow<<(ct+1)<<("_")<<(i+1)<<("_3Filtered")<<typeJPG;
                    string fileHSVyellow = ssHSVyellow.str();
                    imwrite(fileHSVyellow, matImageHSVyellow);

                    // Get the amount of non-zero pixels in the image
                    cv::Mat matNonZeromiddle;
                    findNonZero(matImageHSVyellow, matNonZeromiddle);
                    iNonZeromiddle = matNonZeromiddle.total();
                 //   LOG_INFO(adtf_util::cString::Format("Non zero middle %i",iNonZeromiddle));
                }

                // Image LOW part - hsv low
                {
                    string typeJPG = ".jpg";

                    // Cut the low part of the image
                    cv::Mat matImageRoIlow;
                    matImageRoIlow = matImageRGBRoIcut(Range(110, 200), Range(x_start_cut,x_end_cut)).clone();
                    stringstream sslow;
                    string low ="test/HSV_low/Data_";
                    sslow<<low<<(ct+1)<<("_")<<(i+1)<<("_1matImageRoIlow_")<<typeJPG;
                    string filelow = sslow.str();
                    imwrite(filelow, matImageRoIlow);

                    // convert the bgr-img in a hsv-img
                    cv::Mat matImageHSV;
                    cvtColor(matImageRoIlow, matImageHSV, COLOR_BGR2HSV);
                    stringstream ssHSV;
                    string HSV ="test/HSV_low/Data_";
                    ssHSV<<HSV<<(ct+1)<<("_")<<(i+1)<<("_2HSV_low")<<typeJPG;
                    string fileHSV = ssHSV.str();
                    imwrite(fileHSV, matImageHSV);

                    // apply the mask on the hsv image
                    cv::Mat matImageHSVyellow;
                    inRange(matImageHSV, Scalar(m_iLowH_low, m_iLowS_low, m_iLowV_low), Scalar(m_iHighH_low, m_iHighS_low, m_iHighV_low), matImageHSVyellow);
                    stringstream ssHSVyellow;
                    string HSVyellow ="test/HSV_low/Data_";
                    ssHSVyellow<<HSVyellow<<(ct+1)<<("_")<<(i+1)<<("_3Filtered")<<typeJPG;
                    string fileHSVyellow = ssHSVyellow.str();
                    imwrite(fileHSVyellow, matImageHSVyellow);

                    // Get the amount of non-zero pixels in the image
                    cv::Mat matNonZerolow;
                    findNonZero(matImageHSVyellow, matNonZerolow);
                    iNonZerolow = matNonZerolow.total();
                    //LOG_INFO(adtf_util::cString::Format("Non zero low %i",iNonZerolow));
                }


             //   LOG_INFO(adtf_util::cString::Format("ct = %i,  Col = %i     Box: boxHeight = %f, boxWidth = %f      Non zero: upper = %i, middle = %i, low = %i",ct+1, i+1,boxHeight, boxWidth ,iNonZeroUpper, iNonZeromiddle, iNonZerolow));

              //  x_start_cut = x_start_cut + cut_width / 2;
             //   x_end_cut = x_end_cut + cut_width / 2;
            }
*/



            if(bImage2send == tTrue)
            {
                // ### Resize image ###
                cv::Mat matImageRGB_resized;
                matImageRGB_resized = cv::Mat(m_iResizedImg_Width, m_iResizedImg_Height, CV_8UC3);
                resize(matImageRoICut, matImageRGB_resized, matImageRGB_resized.size());
       //         imwrite("test/08_RGB_resized.jpg", matImageRGB_resized);

                cv::Mat matImageGREY_resized;
                cvtColor(matImageRGB_resized, matImageGREY_resized, CV_BGR2GRAY);
      //          imwrite("test/09_GREY_resized.jpg", matImageGREY_resized);


                // Map the image data into a array for RF
                tInt iSizeOfFeatures;
                iSizeOfFeatures = matImageRGB_resized.size().width * matImageRGB_resized.size().height * 3;
                tUInt8 i8OutputArray [iSizeOfFeatures];
                tFloat64 f64OutputArray[iSizeOfFeatures];
                std::vector<tFloat64> vecRGBoutput;
                {
                        tInt32 index = 0;
                        // Go through cols
                        for(tInt j = 0; j < matImageRGB_resized.size().width; j++) {
                            // Gothrough rgb
                            for(tInt rgb = 0; rgb < 3; rgb++){
                                // Go through rows
                                for(tInt i = 0; i < matImageRGB_resized.size().height; i++){
                                   // index = rows + cols + rgb
                                   // index = (i * m_iWidth * 3)  + (j * 3) + rgb;
                                   i8OutputArray[index] = matImageRGB_resized.at<Vec3b>(i, j)[rgb];
                                   f64OutputArray[index] = i8OutputArray[index];
                                   vecRGBoutput.push_back(f64OutputArray[index]);
                                    // i8OutputArray[index] = matImageRGB_resized[i][j][rgb];
                                    //vecRGBoutput.at<tInt8>(index) = matImageRGB_resized.at<cv::Vec3b>(i, j)[rgb];
                                    // vecRGBoutput.push_back(matImageRGB_resized.at<cv::Vec3i>(i, j)[rgb]);
                                    index++;
                                }
                            }
                        }
                }

                if(m_bEnableSaveData){
                    // Save JPG
                    stringstream ssNameRGBCUT;
                    string typeJPG = ".jpg";
                    string nameRGBCUT ="test/___Last_run/Data_";
                    //ssNameRGBCUT<<nameRGBCUT<<(ct+1)<<("_Area_")<<(f32Area)<<("_Depth_")<<(f32DepthValue)<<("_posX_")<<(posX)<<("_posY_")<<(posY)<<typeJPG;
                    ssNameRGBCUT<<nameRGBCUT<<(ct+1)<<typeJPG;
                    string filenameRGBCUT = ssNameRGBCUT.str();
                    imwrite(filenameRGBCUT, matImageRGB_resized);
                    // LOG_INFO(adtf_util::cString::Format("Object %i, Area %f, posX %i, posY %i, Depth %f",i+1 ,f32Area, posX, posY,f32DepthValue));


                    // Save mat.csv
                    stringstream ssNameCSV;
                    string typeCSV = ".csv";
                    ssNameCSV<<nameRGBCUT<<(ct+1)<<("_MAT")<<typeCSV;
                    string filenameCSV = ssNameCSV.str();
                    writeCSV(filenameCSV, matImageRGB_resized);


                    // Save mat as vec.csv
                    stringstream ssNameRGBCUT_CSV_VEC;
                    ssNameRGBCUT_CSV_VEC<<nameRGBCUT<<(ct+1)<<("_VEC")<<typeCSV;
                    string filenameRGBCUTCSV_VEC = ssNameRGBCUT_CSV_VEC.str();
                    ofstream myfilew;
                    myfilew.open(filenameRGBCUTCSV_VEC.c_str());
                    myfilew<< cv::format(vecRGBoutput, cv::Formatter::FMT_CSV) << std::endl;
                    myfilew.close();
                }
                // Send output to Random Forest Filter

                tInt16 i16CenterBoundingBox = boundRect[i].x + boundRect[i].width/2;
                // Transmit the vector and the current position
                // Create a new MediaSmaple
                cObjectPtr<IMediaSample> pMediaSampleOutput;
                AllocMediaSample((tVoid**)&pMediaSampleOutput);
                // Send the Media Sample
                cObjectPtr<IMediaSerializer> pSerializerOutput;
                pDescriptionOutputVector->GetMediaSampleSerializer(&pSerializerOutput);
                tInt nSizeOutput = pSerializerOutput->GetDeserializedSize();
                pMediaSampleOutput->AllocBuffer(nSizeOutput);
                cObjectPtr<IMediaCoder> pCoderOutputOutput;
                pDescriptionOutputVector->WriteLock(pMediaSampleOutput, &pCoderOutputOutput);
                pCoderOutputOutput->Set("f64VecRGB", (tVoid*)&(f64OutputArray));                // Output Array
                            // pCoderOutputOutput->Set("fVector", (tVoid*)&(vecRGBoutput));     // Output Vector
                pCoderOutputOutput->Set("i16PositionX", (tVoid*)&(i16CenterBoundingBox));       // Position in x-Direction
                pCoderOutputOutput->Set("f32Distance", (tVoid*)&(f32DepthValue));               // Distance
                pCoderOutputOutput->Set("i16BoxWidth", (tVoid*)&(boundRect[i].width));          // Bounding Box Width
                pCoderOutputOutput->Set("i16BoxHeight", (tVoid*)&(boundRect[i].height));        // Bounding Box Height
                pDescriptionOutputVector->Unlock(pCoderOutputOutput);
                pMediaSampleOutput->SetTime(_clock->GetStreamTime());
                m_oOutputVector.Transmit(pMediaSampleOutput);



            }
            ct++;
        }

    }


/*
    cv::Mat outputImage;
    outputImage = matImageDepthCUT;

    if (!outputImage.empty())
    {
        UpdateOutputImageFormat(outputImage);

        //create a cImage from CV Matrix (not necessary, just for demonstration9
        cImage newImage;
        newImage.Create(m_sOutputFormat.nWidth, m_sOutputFormat.nHeight, m_sOutputFormat.nBitsPerPixel, m_sOutputFormat.nBytesPerLine, outputImage.data);

        //create the new media sample
        cObjectPtr<IMediaSample> pMediaSample;
        RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));
        //updating media sample
        RETURN_IF_FAILED(pMediaSample->Update(_clock->GetStreamTime(), newImage.GetBitmap(), newImage.GetSize(), IMediaSample::MSF_None));
        //transmitting
        RETURN_IF_FAILED(m_oVideoOutputPin.Transmit(pMediaSample));

        outputImage.release();
    }
*/
    RETURN_NOERROR;
}


tResult cDepthProcessing::GetDistanceToObject(cv::Mat matImageDepthCUT)
{
    // Cut depth image based on steering angle
    tInt iRoIcenter;
    iRoIcenter = 320 + (m_f32Factor4RoIShift * m_f32InputSteeringAngle);

    // Ensure that we dont cut to outside of the original pic
    if(iRoIcenter < 151)
    {
        iRoIcenter = 151;
    }
    else if(iRoIcenter > 489)
    {
        iRoIcenter = 489;
    }

    // Cut the image depending on the iRoIcenter -> new image width = 300, height = 255
    cv::Mat matImageDepthRoICUT;
    matImageDepthRoICUT = matImageDepthCUT(Range(1, 256),Range(iRoIcenter-150, iRoIcenter+150)).clone();


   // imwrite("test/___Last_run/RoI_DistanceCut.jpg", matImageDepthRoICUT);
         // Moments - function
            Moments oMoments;
            oMoments = moments(matImageDepthRoICUT);

            tInt posY;
            tInt posX;

            tFloat32 f32Area;
            f32Area = 0;
            tFloat32 f32DepthValue;
            tFloat32 f32DepthPlaceholder;
            f32DepthPlaceholder = 300;

            std::vector<std::vector<cv::Point> > contours;
            std::vector<cv::Vec4i> hierarchy;


            // Find contours
            cv::findContours(matImageDepthRoICUT, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

            // Get the 0
            std::vector<Moments> mu(contours.size() );
            for(int i = 0; i < contours.size(); i++)
            {
                mu[i] = moments(contours[i], false);
            }

            // Get the mass centers:
            std::vector<cv::Point2f> mc(contours.size());
            for(int i = 0; i < contours.size(); i++)
            {
                f32Area = mu[i].m00;
                posY = mu[i].m01 / mu[i].m00 ;
                posX = mu[i].m10 / mu[i].m00 ;
                mc[i] = Point2f(posX , posY);

                if(posX < 6)
                {
                    posX = 6;
                }
                else if(posX > 595)
                {
                    posX = 595;
                }

                if(posY < 6){
                    posY = 6;
                }
                else if(posY > 250)
                {
                    posY = 250;
                }

                f32DepthValue = 0;
                f32DepthValue = matImageDepthRoICUT.at<uchar>(posY, posX) +
                                matImageDepthRoICUT.at<uchar>(posY+5, posX+5) +
                                matImageDepthRoICUT.at<uchar>(posY+5, posX-5) +
                                matImageDepthRoICUT.at<uchar>(posY-5, posX-5) +
                                matImageDepthRoICUT.at<uchar>(posY-5, posX+5);
                f32DepthValue = f32DepthValue / 5;

                // Map Depth value into cm
                f32DepthValue = (f32DepthValue + (10 + (f32DepthValue-60)/8) - 20);

                // Check if the detected objects are in the exspected
                if(m_f32AreaMin < f32Area  && f32DepthValue < 300)
                {
                    // Object detected -> transmit distance to ACC and AEB
                    //LOG_INFO(adtf_util::cString::Format("Object %i DETECTED,            Area %f, posX %i, posY %i, Depth %f",i+1 ,f32Area, posX, posY,f32DepthValue));
                    // Get the smalled depth value
                    if(f32DepthValue < f32DepthPlaceholder)
                    {
                        f32DepthPlaceholder = f32DepthValue;
                    }

                }
                else
                {
                    // No proper Object detected -> transmit -1 to ACC and AEB
                    //LOG_INFO(adtf_util::cString::Format("Object %i is too small,        Area %f, posX %i, posY %i, Depth %f",i+1 ,f32Area, posX, posY,f32DepthValue));
                }



            }

            //LOG_INFO(adtf_util::cString::Format("Smallest depth detected is: %f",f32DepthPlaceholder));

            if(f32DepthPlaceholder == 300)
            {
                // No proper Object detected -> transmit -1 to ACC and AEB
                SendDistanceOutput(-1);
            }
            else
            {
                // Object detected -> transmit distance to ACC and AEB
                SendDistanceOutput(f32DepthPlaceholder);
            }
    RETURN_NOERROR;
}


tResult cDepthProcessing::CrossingFeedback(cv::Mat matImageDepthCUT)
{
    RETURN_NOERROR;
}


tResult cDepthProcessing::UpdateInputImageFormat(const tBitmapFormat* pFormat)
{
    if (pFormat != NULL)
    {
        //update member variable
        m_sInputFormat = (*pFormat);
        //LOG_INFO(adtf_util::cString::Format("Input: Size %d x %d ; BPL %d ; Size %d , PixelFormat; %d", m_sInputFormat.nWidth, m_sInputFormat.nHeight, m_sInputFormat.nBytesPerLine, m_sInputFormat.nSize, m_sInputFormat.nPixelFormat));
        //create the input matrix
        RETURN_IF_FAILED(BmpFormat2Mat(m_sInputFormat, m_inputImage));
    }

}


tResult cDepthProcessing::UpdateOutputImageFormat(const cv::Mat& outputImage)
{
    //check if pixelformat or size has changed
    if (tInt32(outputImage.total() * outputImage.elemSize()) != m_sOutputFormat.nSize)
    {
        Mat2BmpFormat(outputImage, m_sOutputFormat);

        //LOG_INFO(adtf_util::cString::Format("Output: Size %d x %d ; BPL %d ; Size %d , PixelFormat; %d", m_sOutputFormat.nWidth, m_sOutputFormat.nHeight, m_sOutputFormat.nBytesPerLine, m_sOutputFormat.nSize, m_sOutputFormat.nPixelFormat));
        //set output format for output pin
        m_oVideoOutputPin.SetFormat(&m_sOutputFormat, NULL);
    }
    RETURN_NOERROR;
}


tResult cDepthProcessing::SendDistanceOutput(tFloat32 f32Distance2Send)
{
    // Send MediaSample to ACC
    cObjectPtr<IMediaSample> pMediaSampleOutputACCDistance;
    AllocMediaSample((tVoid**)&pMediaSampleOutputACCDistance);

    cObjectPtr<IMediaSerializer> pSerializerOutputACCDistance;
    m_pDescriptionOutputACCdistance->GetMediaSampleSerializer(&pSerializerOutputACCDistance);
    tInt nSizeOutputACCDistance = pSerializerOutputACCDistance->GetDeserializedSize();
    pMediaSampleOutputACCDistance->AllocBuffer(nSizeOutputACCDistance);
    cObjectPtr<IMediaCoder> pCoderOutputACCDistance;
    m_pDescriptionOutputACCdistance->WriteLock(pMediaSampleOutputACCDistance, &pCoderOutputACCDistance);
    pCoderOutputACCDistance->Set("f32Value", (tVoid*)&(f32Distance2Send));
    m_pDescriptionOutputACCdistance->Unlock(pCoderOutputACCDistance);
    pMediaSampleOutputACCDistance->SetTime(_clock->GetStreamTime());
    m_oOutputACCdistance.Transmit(pMediaSampleOutputACCDistance);

    RETURN_NOERROR;
}


tResult cDepthProcessing::writeCSV(string filename, Mat m){
    ofstream myfile;
    myfile.open(filename.c_str());
    myfile<< cv::format(m, cv::Formatter::FMT_CSV) << std::endl;
    myfile.close();

    RETURN_NOERROR;
}

