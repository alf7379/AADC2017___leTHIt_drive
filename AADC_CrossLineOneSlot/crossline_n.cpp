/**
Copyright (c)
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


**********************************************************************
* $Author:: spiesra $  $Date:: 2017-05-22 15:15:31#$ $Rev:: 63721   $
**********************************************************************/
#include "stdafx.h"
#include "crossline_n.h"

using namespace std;
using namespace cv;

// define the ADTF property names to avoid errors
ADTF_FILTER_PLUGIN("crossline2slot", OID_ADTF_Stopline2oneslot, clanefollower)
clanefollower::clanefollower(const tChar* __info) : cFilter(__info)
{
    SetPropertyBool("Enable Debug Output", false);

    SetPropertyInt("KpsiRIGHT", 2735);
    SetPropertyBool("KpsiRIGHT" NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyInt("KpsiLEFT", 1600);
    SetPropertyBool("KpsiLEFT" NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyInt("Ky", 150);
    SetPropertyBool("Ky" NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyFloat("vMax", 0.5);
    SetPropertyBool("vMax" NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyFloat("vMin", 0.5);  // Ignore the variable
    SetPropertyBool("vMin" NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyFloat("v Road Sign", 0.5);  // Ignore the variable
    SetPropertyBool("v Road Sign" NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyInt("steermulti", 2000);
    SetPropertyBool("steermulti" NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyBool("imshow", tFalse);
    SetPropertyBool("imshow" NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyInt("houghlines", 28);
    SetPropertyBool("houghlines" NSSUBPROP_ISCHANGEABLE, tTrue);
    // SetPropertyInt("row1", 470);
    SetPropertyInt("row1", 520);
    SetPropertyBool("row1" NSSUBPROP_ISCHANGEABLE, tTrue);
    // SetPropertyInt("row2", 600);
    SetPropertyInt("row2", 600);
    SetPropertyBool("row2" NSSUBPROP_ISCHANGEABLE, tTrue);
    // SetPropertyInt("col1", 400);
    SetPropertyInt("col1", 491);
    SetPropertyBool("col1" NSSUBPROP_ISCHANGEABLE, tTrue);
    // SetPropertyInt("col2", 880);
    SetPropertyInt("col2", 763);
    SetPropertyBool("col2" NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyInt("v-max Steering angle", 1);
    SetPropertyBool("v-max Steering angle" NSSUBPROP_ISCHANGEABLE, tTrue);
    m_time=tFalse;

    writeZeroOutputs = tFalse;
    blind_count = 0;
	accel =0;
       	steeringAngle = 0;
      	steeringAngle_previous = 0;
        mean_theta_previous = 0;
        m_iCurrentTrafficSignID = -1;
        m_iPreviousTrafficSignID = -1;
        m_iTrafficSignID = -1;
	//m_szIdsOutputSpeedSet = tFalse;
}

clanefollower::~clanefollower()
{

}

tResult clanefollower::Start(__exception)
{

    return cFilter::Start(__exception_ptr);
}

tResult clanefollower::Stop(__exception)
{
    return cFilter::Stop(__exception_ptr);
}

tResult clanefollower::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));

    if (eStage == StageFirst)
    {
        cObjectPtr<IMediaDescriptionManager> pDescManager;
        RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER, (tVoid**)&pDescManager, __exception_ptr));

        // Video Input
        RETURN_IF_FAILED(m_oVideoInputPin.Create("Video_Input", IPin::PD_Input, static_cast<IPinEventSink*>(this)));
        RETURN_IF_FAILED(RegisterPin(&m_oVideoInputPin));

        // Input LaneFollowerStart
        tChar const * strDescSignalValueInputLaneFollowerStart = pDescManager->GetMediaDescription("tBoolSignalValue");
        RETURN_IF_POINTER_NULL(strDescSignalValueInputLaneFollowerStart);
        cObjectPtr<IMediaType> pTypeSignalValueInputLaneFollowerStart = new cMediaType(0, 0, 0, "tBoolSignalValue", 			 strDescSignalValueInputLaneFollowerStart, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(pTypeSignalValueInputLaneFollowerStart->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignalInputLaneFollowerStart));
        RETURN_IF_FAILED(m_bInputLaneFollowerStart.Create("Start", pTypeSignalValueInputLaneFollowerStart, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_bInputLaneFollowerStart));
        
        //stop line	
	tChar const * strDescSignaldistancestop = pDescManager->GetMediaDescription("tStoplineStruct");
	RETURN_IF_POINTER_NULL(strDescSignaldistancestop);
	cObjectPtr<IMediaType> pTypeSignaldistancestop = new cMediaType(0, 0, 0, "tStoplineStruct", strDescSignaldistancestop, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
	RETURN_IF_FAILED(pTypeSignaldistancestop->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDesStoplinedist));
	RETURN_IF_FAILED(m_Stoplinedist.Create("distance stop line", pTypeSignaldistancestop, static_cast<IPinEventSink*> (this)));
	RETURN_IF_FAILED(RegisterPin(&m_Stoplinedist));		
	

	// Media description for Signal Value
        tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");
        RETURN_IF_POINTER_NULL(strDescSignalValue);
        cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
         // Create Pins for selection of crossing edges
        //create pin for steering signal output
        RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDesEdgeselect));
        RETURN_IF_FAILED(m_edgeselect.Create("Crossing select", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_edgeselect));


	// Input pin for Speed selection based on sign detection	
        tChar const * strRoadSign = pDescManager->GetMediaDescription("tRoadSign");
        RETURN_IF_POINTER_NULL(strRoadSign);
        cObjectPtr<IMediaType> pTypeRoadSign = new cMediaType(0, 0, 0, "tRoadSign", strRoadSign, 			 IMediaDescription::MDF_DDL_DEFAULT_VERSION);
	//create pin for traffic sign input
	RETURN_IF_FAILED(pTypeRoadSign->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionInputTrafficSign));
        RETURN_IF_FAILED(m_oInputTrafficSign.Create("TrafficSign", pTypeRoadSign, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oInputTrafficSign));

        // acceleration output
        tChar const * strDescSignalValueOutputAccel = pDescManager->GetMediaDescription("tSignalValue");
        RETURN_IF_POINTER_NULL(strDescSignalValueOutputAccel);
        cObjectPtr<IMediaType> pTypeSignalValueOutputAccel = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValueOutputAccel, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(pTypeSignalValueOutputAccel->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignalOutputAccel));
        RETURN_IF_FAILED(m_oAccelerate.Create("acceleration", pTypeSignalValueOutputAccel, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oAccelerate));
	// Threshold input
        tChar const * strDescSignalValueOutputThres = pDescManager->GetMediaDescription("tSignalValue");
        RETURN_IF_POINTER_NULL(strDescSignalValueOutputThres);
        cObjectPtr<IMediaType> pTypeSignalValueOutputThre = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValueOutputThres, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(pTypeSignalValueOutputThre->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescThreshold));
        RETURN_IF_FAILED(m_iThreshold.Create("Threshold", pTypeSignalValueOutputThre, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_iThreshold));	


        // Video output
        RETURN_IF_FAILED(m_oVideoOutputPin.Create("Video_Output", IPin::PD_Output, static_cast<IPinEventSink*>(this)));
        RETURN_IF_FAILED(RegisterPin(&m_oVideoOutputPin));



    }
    else if (eStage == StageNormal)
    {
	firstFrame = tTrue;
        imagecount = 0;

        bDebugOutputEnabled = GetPropertyBool("Enable Debug Output");

        KpsiRIGHT = GetPropertyInt("KpsiRIGHT");
        KpsiLEFT = GetPropertyInt("KpsiLEFT");
        Ky = GetPropertyInt("Ky");
        vMax = GetPropertyFloat("vMax");
        vMin = GetPropertyFloat("vMin");
        vRoadSign = GetPropertyFloat("v Road Sign");
        steermulti = GetPropertyInt("steermulti");
        enable_imshow = GetPropertyBool("imshow");
        houghlinesvalue = GetPropertyInt("houghlines");
        row1 = GetPropertyInt("row1");
        row2 = GetPropertyInt("row2");
        col1 = GetPropertyInt("col1");
        col2 = GetPropertyInt("col2");

	
        iVmaxSteeringAngle =GetPropertyInt("v-max Steering angle");


         m_edgeselected = 0;

    }

    else if (eStage == StageGraphReady)
    {
        // get the image format of the input video pin
        cObjectPtr<IMediaType> pType;
        RETURN_IF_FAILED(m_oVideoInputPin.GetMediaType(&pType));

        cObjectPtr<IMediaTypeVideo> pTypeVideo;
        RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid**)&pTypeVideo));

		 // no ids were set so far
        m_bIDsRoadSignSet = tFalse;
	m_bLaneFollowerStart = tFalse;
	thresholdvalue=150;
	
          // set the image format of the input video pin
        // set the image format of the input video pin
        if (IS_FAILED(UpdateInputImageFormat(pTypeVideo->GetFormat())))
        {
            LOG_ERROR("Invalid Input Format for this filter");
        }
	if(!m_time)
	{
            starttime= _clock->GetStreamTime();
            m_time=tTrue;
	}    

        left =  tFalse;
        right = tTrue;
        straight = tFalse;
        stop_detect = tFalse;
    }

    RETURN_NOERROR;
}



tResult clanefollower::Shutdown(tInitStage eStage, ucom::IException** __exception_ptr)
{
    if (eStage == StageGraphReady)
    {
    }

    return cFilter::Shutdown(eStage, __exception_ptr);
}

tResult clanefollower::OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{

    RETURN_IF_POINTER_NULL(pMediaSample);
    RETURN_IF_POINTER_NULL(pSource);
    if (nEventCode == IPinEventSink::PE_MediaSampleReceived)
    {
   	if(pSource == &m_edgeselect)
        {
			cObjectPtr<IMediaCoder> pCoderInput;
			RETURN_IF_FAILED(m_pDesEdgeselect->Lock(pMediaSample, &pCoderInput));
			pCoderInput->Get("f32Value", (tVoid*)&m_edgeselected);
			//pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timestamp);
			m_pDesEdgeselect->Unlock(pCoderInput);

                        // LOG_INFO(adtf_util::cString::Format("received m_edgeselected %f", m_edgeselected));
        }
	else if(pSource == &m_iThreshold)
	 {
	 
            cObjectPtr<IMediaCoder> pCoderInput;
            RETURN_IF_FAILED(m_pCoderDescThreshold->Lock(pMediaSample, &pCoderInput));
            pCoderInput->Get("f32Value", (tVoid*)&thresholdvalue);
            m_pCoderDescThreshold->Unlock(pCoderInput);
	}	
        

        else if(pSource == &m_bInputLaneFollowerStart){
            cObjectPtr<IMediaCoder> pCoderInput;
            RETURN_IF_FAILED(m_pCoderDescSignalInputLaneFollowerStart->Lock(pMediaSample, &pCoderInput));
            pCoderInput->Get("bValue", (tVoid*)&m_bLaneFollowerStart);
            //pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timestamp);
            m_pCoderDescSignalInputLaneFollowerStart->Unlock(pCoderInput);
        }

        else if(pSource== &m_oInputTrafficSign)
        {
            RoadSignTime = 0;
            // Read out the media sample
            cObjectPtr<IMediaCoder> pCoderInput;
            RETURN_IF_FAILED(m_pDescriptionInputTrafficSign->Lock(pMediaSample, &pCoderInput));
            pCoderInput->Get("i16Identifier", (tVoid*)&m_iCurrentTrafficSignID);
            m_pDescriptionInputTrafficSign->Unlock(pCoderInput);

            RoadSignTime = _clock->GetStreamTime();

            // Check if the same Road sign as before was detected
            if(m_iPreviousTrafficSignID == m_iCurrentTrafficSignID){

                // Only set a road sign valid, when it is detected 3 times in a row
                if(m_iRoadSignDetectorCounter > 3){
                    m_iTrafficSignID = m_iCurrentTrafficSignID;
                    RoadSignTime = _clock->GetStreamTime();
                } else {
                    m_iRoadSignDetectorCounter++;
                }

            } else {
               m_iRoadSignDetectorCounter = 0;
               m_iTrafficSignID = -1;
            }
            m_iPreviousTrafficSignID = m_iCurrentTrafficSignID;

            // if(bDebugOutputEnabled) LOG_INFO(adtf_util::cString::Format("m_iTrafficSignID: %i", m_iTrafficSignID));

        }

        // Check if you have video input and the start-flag (m_bLaneFollowerStart) is true
        else if (pSource == &m_oVideoInputPin)
        {
            //check if video format is still unkown
            if (m_sInputFormat.nPixelFormat == IImage::PF_UNKNOWN)
            {
                RETURN_IF_FAILED(UpdateInputImageFormat(m_oVideoInputPin.GetFormat()));
           }
    	    tUInt32 timeStamp = 0;
            
	    if (firstFrame)
            {
                cObjectPtr<IMediaType> pType;
                RETURN_IF_FAILED(m_oVideoInputPin.GetMediaType(&pType));
                cObjectPtr<IMediaTypeVideo> pTypeVideo;
                RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid**)&pTypeVideo));
                const tBitmapFormat* pFormat = pTypeVideo->GetFormat();
                if (pFormat == NULL)
                {
                    LOG_ERROR("Spurerkennung: No Bitmap information found on pin \"input\"");
                    RETURN_ERROR(ERR_NOT_SUPPORTED);
                }
                m_sInputFormat.nPixelFormat = pFormat->nPixelFormat;
                m_sInputFormat.nWidth = pFormat->nWidth;
                m_sInputFormat.nHeight = pFormat->nHeight;
                m_sInputFormat.nBitsPerPixel = pFormat->nBitsPerPixel;
                m_sInputFormat.nBytesPerLine = pFormat->nBytesPerLine;
                m_sInputFormat.nSize = pFormat->nSize;
                m_sInputFormat.nPaletteSize = pFormat->nPaletteSize;
                firstFrame = tFalse;
            }
            
            steeringAngle = evaluateSteeringAngle(pMediaSample);
            if(blind_count > 50)
            {
                        // if(bDebugOutputEnabled) LOG_INFO(adtf_util::cString::Format("Lane Follower: Bilnd Count"));
            		accel = 0;
            }
            else
            {
                    // Check if a valid(not order than 10s) traffic sign is detected
                    if(m_iTrafficSignID != -1)
                    {
                            /*
                             * !  0 = UNMARKEDINTERSECTION
                             * !  1 = STOPANDGIVEWAY
                             * !  2 = PARKINAREA
                             * !  3 = HAVEWAY
                             * !  4 = AHEADONLY
                             * !  5 = GIVEWAY
                             * !  6 = PEDESTRIANCROSSING
                             *    7 = ROUNDABOUT
                             *    8 = NOOVERTAKING
                             *    9 = NOENTRYVEHICULARTRAFFIC
                             *   10 = TESTCOURSEA9
                             *   11 = ONEWAYSTREET
                             * ! 12 = ROADWORKS
                             * ! 13 = KMH50
                             * ! 14 = KMH100
                             *   99 = NOMATCH
                             */

                            // Check if the road sign is relevant and check if it is still valid
                            if((m_iTrafficSignID < 7 || m_iTrafficSignID == 12) && (RoadSignTime+6000000 > _clock->GetStreamTime()))
                            {
                                //if(bDebugOutputEnabled) LOG_INFO(adtf_util::cString::Format("Lane Follower: Relevant road sign"));
                                accel = vRoadSign;
                            }
                            // KMH50
                            else if(m_iTrafficSignID == 13)
                            {
                                //if(bDebugOutputEnabled) LOG_INFO(adtf_util::cString::Format("Lane Follower: speed limit 50"));
                                // Drive max 50
                            }
                            // KMH100
                            else if(m_iTrafficSignID == 14)
                            {
                                // Drive max 100
                                //if(bDebugOutputEnabled) LOG_INFO(adtf_util::cString::Format("Lane Follower: speed limit 100"));
                            }
                            // Traffic sign is not important
                            else
                            {
                                //if(bDebugOutputEnabled) LOG_INFO(adtf_util::cString::Format("Lane Follower: No important road sign"));
                                accel = vMax;
                            }
                    }
                    // No current traffic sign
                    else {
                            accel = vMax;
                            //if(bDebugOutputEnabled) LOG_INFO(adtf_util::cString::Format("Lane Follower: No road sign"));
                    }
                    //if(bDebugOutputEnabled) LOG_INFO(adtf_util::cString::Format("Output: Lakshman acceleration loop2"));

              }
            
            //timeStamp = _clock->GetStreamTime() / 1000;
       	   // writeOutputs(timeStamp);
        }
 
    }
    else if (nEventCode == IPinEventSink::PE_MediaTypeChanged)
    {
        if (pSource == &m_oVideoInputPin)
        {
            //the input format was changed, so the imageformat has to changed in this filter also
            RETURN_IF_FAILED(UpdateInputImageFormat(m_oVideoInputPin.GetFormat()));
        }
    }
    RETURN_NOERROR;
}


tFloat32 clanefollower::evaluateSteeringAngle(IMediaSample* pSample)
{
    // VideoInput
    RETURN_IF_POINTER_NULL(pSample);
	m_edgeselected=0;
    cObjectPtr<IMediaSample> pNewRGBSample;


    const tVoid* l_pSrcBuffer;

     /*
      * tInt timeStamp=0;
    tInt  iSelectedEdge = (tInt)  m_edgeselected;
    switch(iSelectedEdge)
	{
	case 0:
		{
			right=tTrue;
			left=tFalse;
			straight=tFalse;
			break;
		}
	case 1:
		{
			left=tTrue;
			right=tFalse;
			straight=tFalse;
			break;
		}
	case 2:
		{
			straight=tTrue;
			right=tFalse;
			left=tFalse;
			break;
		}
	default:
		{
		break;
		}

	}

        */
    if (IS_OK(pSample->Lock(&l_pSrcBuffer)))
    {
    // ### START Image Processing ###

        tInt timeStamp=0;
        IplImage* img = cvCreateImageHeader(cvSize(m_sInputFormat.nWidth, m_sInputFormat.nHeight), IPL_DEPTH_8U, 3);
        img->imageData = (char*)l_pSrcBuffer;
        //Übergang von OpenCV1 auf OpenCV2
        Mat image(cvarrToMat(img));
        cvReleaseImage(&img);
        pSample->Unlock(l_pSrcBuffer);
        //Zuschneiden des Bildes
	Mat imagecut,color_dst,imagecut2,imagecut3,imagecut4;
        // LOG_INFO(adtf_util::cString::Format("m_edgeselected %f", m_edgeselected));
        if(m_edgeselected == 0)
	{
                // RIGHT
                imagecut= image(cv::Range(520, 600), cv::Range(700, 950)).clone();
               // LOG_INFO(adtf_util::cString::Format("Image cut right"));
	}
        else if(m_edgeselected == 1)
	{
                // LEFT
		imagecut= image(cv::Range(520, 600), cv::Range(300, 400)).clone();
               // LOG_INFO(adtf_util::cString::Format("Image cut left"));
	}
        else if(m_edgeselected == 2)
	{
                // STRAIGHT
                imagecut= image(cv::Range(520, 600), cv::Range(300, 800)).clone();
              //  LOG_INFO(adtf_util::cString::Format("Image cut straight"));
	}
	else
	{
	imagecut= image(cv::Range(520, 600), cv::Range(600, 775)).clone();
	}

	// Crop the image
        //imagecut = image(Range(row1, row2), Range(col1, col2)).clone(); 
        
        //Erzeugen eines Graustufenbildes
	GaussianBlur(imagecut,imagecut2,Size(11,11),0,0, BORDER_DEFAULT); 
        cvtColor(imagecut2, grey, CV_BGR2GRAY);
	//equalizeHist(grey,imagecut3);
       	//GaussianBlur(imagecut3,imagecut4,Size(5,5),0,0, BORDER_DEFAULT);
        // adaptiveThreshold(imagecut2, greythresh, thresholdvalue, ADAPTIVE_THRESH_MEAN_C,THRESH_BINARY, 11, 12);
        //Treshold um Binärbild zu erhalten
        threshold(grey, greythresh, thresholdvalue, 500, THRESH_BINARY);
        //namedWindow("Grey Threshold");
        //imshow("Grey Threshold",greythresh);
        //waitKey(1);
        //Kantendedektion
        //Canny(greythresh, linecanny, 250, 350);
	//LOG_INFO(adtf_util::cString::Format("Image cut"));
  		
	Canny(greythresh, linecanny, 0, 2, 3, tFalse);
        //if(bDebugOutputEnabled)  LOG_INFO(adtf_util::cString::Format("Canny done"));
	cannysize = linecanny.size();
        vector<Vec2f> lines;
        // How long must be the line to be detected as roadlines -> HoughLines
        //HoughLines(linecanny, lines, 1, CV_PI / 180, houghlinesvalue, 0,0);
	HoughLines(linecanny, lines, 1, CV_PI / 180, houghlinesvalue, 0,0);
	//HoughLinesP(linecanny, lines, 2, CV_PI / 180, houghlinesvalue, 0, 0);
        cvtColor( linecanny, color_dst, CV_GRAY2BGR );
        // if(bDebugOutputEnabled) LOG_INFO(adtf_util::cString::Format("Output HoughLine: line size %d",lines.size()));

        tFloat32 thetaAll[1000];
        tFloat32 rhoAll[1000];
        for (tInt i = 0; i < (tInt)(lines.size()); i++)
        {
            thetaAll[i] = lines[i][1];
            rhoAll[i] = lines[i][0];
        }

        tFloat32 thetaNoRep[1000];
        tFloat32 rhoNoRep[1000];
        tInt sizeNoRep = 0;

        for (tInt i = 0; i < (tInt)(lines.size()); i++)
        {
            tInt rep = 0;
            for (tInt j = 0; j < i; j++)
            {
                if (abs(thetaAll[i] - thetaAll[j]) <= 0.2 && (abs(rhoAll[i] - rhoAll[j]) <= 30))
                {
                    rep = 1;		// One point of the Line is repeated, ignore the whole line
                }
            }	
            // Filter
            if ((rep == 0) && ((thetaAll[i]>CV_PI*0.4722) && (thetaAll[i]<CV_PI*0.5277)))
            {
                thetaNoRep[sizeNoRep] = thetaAll[i];
                rhoNoRep[sizeNoRep] = rhoAll[i];
                sizeNoRep += 1;
                //if(bDebugOutputEnabled) LOG_INFO(adtf_util::cString::Format("Output HoughLine: Lakshman_edited %d theta %f",sizeNoRep,thetaNoRep[i]));
                //if(bDebugOutputEnabled) LOG_INFO(adtf_util::cString::Format("Output HoughLine: size %d th0 %f rh0  %f",sizeNoRep,thetaNoRep[i],rhoNoRep[i]));
                //if(bDebugOutputEnabled) LOG_INFO(adtf_util::cString::Format("Output HoughLine: Lakshman_edited %d theta1 %f rho1  %d",sizeNoRep,thetaNoRep[i],rhoNoRep[i]));
            }

        }
	tFloat32 DistPack[sizeNoRep];
	// Print the detected lines on the screen
	for(tInt i = 0; i < sizeNoRep; i++ )
        {
            double a = cos(thetaNoRep[i]), b = sin(thetaNoRep[i]);
            double x0 = a*rhoNoRep[i], y0 = b*rhoNoRep[i];
            //if(bDebugOutputEnabled) LOG_INFO(adtf_util::cString::Format("StopLine Detection: X not %f Y not %f",x0,y0));
            
            Point pt1(cvRound(x0 + 1000*(-b)),
                      cvRound(y0 + 1000*(a)));
            Point pt2(cvRound(x0 - 1000*(-b)),
                      cvRound(y0 - 1000*(a)));
            line( color_dst, pt1, pt2, Scalar(0,0,255), 3, 8 );
	    tInt newv=80-y0;
            tFloat32 distanceCal = 0.0001*pow(newv,3)-0.0045*pow(newv,2)+0.8652*newv+28.8300;
            // if(bDebugOutputEnabled) LOG_INFO(adtf_util::cString::Format("Distance %f", distanceCal));
            DistPack[i] = distanceCal;
        }
	if (sizeNoRep==0)
	{
	distance = 0;
	Orientation = 0;
	stop_detect=tFalse;
	}
	
	if (sizeNoRep == 1)
	{
	distance = DistPack[0];
	Orientation = thetaNoRep[0];
	 tFloat32 difference_dist=abs(distance_prev-distance);
		if(difference_dist>=60)
		{
			stop_detect=tFalse;
			timeStamp = _clock->GetStreamTime() / 1000;
			writeOutputs1(0,timeStamp);
		}
		else
		{
			stop_detect=tTrue;
			timeStamp = _clock->GetStreamTime() / 1000;
			writeOutputs1(distance,timeStamp);
		}
		distance_prev=distance;
	}
	if (sizeNoRep == 2)
	{
        	if (DistPack[0]<DistPack[1])
        	{
		distance = DistPack[0];
		Orientation = thetaNoRep[0];
		tFloat32 difference_dist=abs(distance_prev-distance);
		if(difference_dist>=60)
		{
			stop_detect=tFalse;
			timeStamp = _clock->GetStreamTime() / 1000;
			writeOutputs1(0,timeStamp);
		}
		else
		{
			stop_detect=tTrue;
			timeStamp = _clock->GetStreamTime() / 1000;
			writeOutputs1(distance,timeStamp);
		}
		distance_prev=distance;	
		}

		else
		{
		distance = DistPack[1];
		Orientation = thetaNoRep[1];
		stop_detect=tFalse;		
		}	
        }
        
	Orientation = Orientation*180/CV_PI;
        // if(bDebugOutputEnabled) LOG_INFO(adtf_util::cString::Format("Orientation done"));
        timeStamp = _clock->GetStreamTime() / 1000;
	writeOutputs1(distance,timeStamp);

         if (!color_dst.empty())
   	 {
        UpdateOutputImageFormat(color_dst);

        //create a cImage from CV Matrix (not necessary, just for demonstration9
        cImage newImage;
        newImage.Create(m_sOutputFormat.nWidth, m_sOutputFormat.nHeight, m_sOutputFormat.nBitsPerPixel, m_sOutputFormat.nBytesPerLine, color_dst.data);

        //create the new media sample
        cObjectPtr<IMediaSample> pMediaSample;
        RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));
        //updating media sample
        RETURN_IF_FAILED(pMediaSample->Update(_clock->GetStreamTime(), newImage.GetBitmap(), newImage.GetSize(), IMediaSample::MSF_None));
        //transmitting
        RETURN_IF_FAILED(m_oVideoOutputPin.Transmit(pMediaSample));
        //if(bDebugOutputEnabled) LOG_INFO(adtf_util::cString::Format("Output done"));
        color_dst.release();
    	} 
        


 }
           RETURN_NOERROR;
}

tResult clanefollower::UpdateInputImageFormat(const tBitmapFormat* pFormat)
{
    if (pFormat != NULL)
    {
        //update member variable
        m_sInputFormat = (*pFormat);
        //if(bDebugOutputEnabled) LOG_INFO(adtf_util::cString::Format("Input: Size %d x %d ; BPL %d ; Size %d , PixelFormat; %d", m_sInputFormat.nWidth, m_sInputFormat.nHeight, m_sInputFormat.nBytesPerLine, m_sInputFormat.nSize, m_sInputFormat.nPixelFormat));
        //create the input matrix
        RETURN_IF_FAILED(BmpFormat2Mat(m_sInputFormat, m_inputImage));
    }
    RETURN_NOERROR;
}

tResult clanefollower::UpdateOutputImageFormat(const cv::Mat& outputImage)
{
    //check if pixelformat or size has changed
    if (tInt32(outputImage.total() * outputImage.elemSize()) != m_sOutputFormat.nSize)
    {
        Mat2BmpFormat(outputImage, m_sOutputFormat);

        // if(bDebugOutputEnabled) LOG_INFO(adtf_util::cString::Format("Output: Size %d x %d ; BPL %d ; Size %d , PixelFormat; %d", m_sOutputFormat.nWidth, m_sOutputFormat.nHeight, m_sOutputFormat.nBytesPerLine, m_sOutputFormat.nSize, m_sOutputFormat.nPixelFormat));
        //set output format for output pin
        m_oVideoOutputPin.SetFormat(&m_sOutputFormat, NULL);
    }
    RETURN_NOERROR;
}

tResult clanefollower::writeOutputs1(tFloat32 distance, tUInt32 timeStamp)
{
   
	//create new media sample
	cObjectPtr<IMediaSample> pMediaSampleaccelerate;
	
	AllocMediaSample((tVoid**)&pMediaSampleaccelerate);

	// acceleration
	cObjectPtr<IMediaSerializer> pSerializeraccelerate;
	m_pDesStoplinedist->GetMediaSampleSerializer(&pSerializeraccelerate);
	tInt nSizeaccelerate = pSerializeraccelerate->GetDeserializedSize();
	pMediaSampleaccelerate->AllocBuffer(nSizeaccelerate);
	cObjectPtr<IMediaCoder> pCoderOutputaccelerate;
	m_pDesStoplinedist->WriteLock(pMediaSampleaccelerate, &pCoderOutputaccelerate);
	pCoderOutputaccelerate->Set("bValue", (tVoid*)&(stop_detect));
	pCoderOutputaccelerate->Set("f32Distance", (tVoid*)&(distance));
	pCoderOutputaccelerate->Set("f32Orientation", (tVoid*)&(Orientation));
	pCoderOutputaccelerate->Set("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
	m_pDesStoplinedist->Unlock(pCoderOutputaccelerate);
	pMediaSampleaccelerate->SetTime(_clock->GetStreamTime());
	m_Stoplinedist.Transmit(pMediaSampleaccelerate);
RETURN_NOERROR;
}


