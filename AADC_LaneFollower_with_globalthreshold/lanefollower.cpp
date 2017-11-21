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
#include "lanefollower.h"

using namespace std;
using namespace cv;



tFloat32 SteeringAngleArray[1000];

// define the ADTF property names to avoid errors
ADTF_FILTER_PLUGIN("Lane_v_2", OID_ADTF_LaneFollower, clanefollower)
clanefollower::clanefollower(const tChar* __info) : cFilter(__info)
{
    SetPropertyBool("Enable Debug Output", false);

    SetPropertyInt("KpsiRIGHT", 2600);
    SetPropertyBool("KpsiRIGHT" NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyInt("KpsiLEFT", 2500);
    SetPropertyBool("KpsiLEFT" NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyInt("Ky", 200);
    SetPropertyBool("Ky" NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyFloat("vMax", 60);
    SetPropertyBool("vMax" NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyFloat("vMin", 60);  // Ignore the variable
    SetPropertyBool("vMin" NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyFloat("v Road Sign", 20);  // Ignore the variable
    SetPropertyBool("v Road Sign" NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyInt("steermulti", 2000);
    SetPropertyBool("steermulti" NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyInt("thresholdvalue", 170);
    SetPropertyBool("thresholdvalue" NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyBool("imshow", tFalse);
    SetPropertyBool("imshow" NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyInt("houghlines", 26);
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

    SetPropertyFloat("Distance for relecant Road Signs [m]",1);
    SetPropertyInt("v-max Steering angle", 1);
    SetPropertyBool("v-max Steering angle" NSSUBPROP_ISCHANGEABLE, tTrue);
    m_time=tFalse;

    writeZeroOutputs = tFalse;
    m_noline  = tFalse;
    m_oneline = tFalse;
    m_twoline = tFalse;
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
		// create description manager
        cObjectPtr<IMediaDescriptionManager> pDescManager;
        RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));
        tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");
        RETURN_IF_POINTER_NULL(strDescSignalValue);
        cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        tChar const * strDescBoolSignalValue = pDescManager->GetMediaDescription("tBoolSignalValue");
        RETURN_IF_POINTER_NULL(strDescBoolSignalValue);
        cObjectPtr<IMediaType> pTypeBoolSignalValue = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescBoolSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
		
        RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionFloat));
        RETURN_IF_FAILED(pTypeBoolSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionBool));

        // Video Input
        RETURN_IF_FAILED(m_oVideoInputPin.Create("Video_Input", IPin::PD_Input, static_cast<IPinEventSink*>(this)));
        RETURN_IF_FAILED(RegisterPin(&m_oVideoInputPin));

        // Input LaneFollowerStart
        tChar const * strDescSignalValueInputLaneFollowerStart = pDescManager->GetMediaDescription("tStartTrigger");
        RETURN_IF_POINTER_NULL(strDescSignalValueInputLaneFollowerStart);
        cObjectPtr<IMediaType> pTypeSignalValueInputLaneFollowerStart = new cMediaType(0, 0, 0, "tStartTrigger", strDescSignalValueInputLaneFollowerStart, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(pTypeSignalValueInputLaneFollowerStart->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignalInputLaneFollowerStart));
        RETURN_IF_FAILED(m_bInputLaneFollowerStart.Create("Start", pTypeSignalValueInputLaneFollowerStart, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_bInputLaneFollowerStart));

//create pin for start_test pin input
        tChar const * strDescSignalstart_test = pDescManager->GetMediaDescription("tBoolSignalValue");
        RETURN_IF_POINTER_NULL(strDescSignalstart_test);
        cObjectPtr<IMediaType> pTypeSignalstart_test = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescSignalstart_test, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(pTypeSignalstart_test->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescStart_test));
        RETURN_IF_FAILED(m_oStart_test.Create("Start_test1", pTypeSignalstart_test, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oStart_test));

	// Input pin for Speed selection based on sign detection	
        tChar const * strRoadSign = pDescManager->GetMediaDescription("tRoadSignExt");
        RETURN_IF_POINTER_NULL(strRoadSign);
        cObjectPtr<IMediaType> pTypeRoadSign = new cMediaType(0, 0, 0, "tRoadSignExt", strRoadSign, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

	//create pin for traffic sign input
	RETURN_IF_FAILED(pTypeRoadSign->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionInputTrafficSign));
        RETURN_IF_FAILED(m_oInputTrafficSign.Create("RoadSignExt", pTypeRoadSign, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oInputTrafficSign));

/*
        // Input pin for Speed selection based on sign detection
        tChar const * strRoadSignExt = pDescManager->GetMediaDescription("tRoadSignExt");
        RETURN_IF_POINTER_NULL(strRoadSignExt);
        cObjectPtr<IMediaType> pTypeRoadSignExt = new cMediaType(0, 0, 0, "tRoadSignExt", strRoadSignExt, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

        //create pin for traffic sign input
        RETURN_IF_FAILED(pTypeRoadSignExt->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionInputTrafficSignExt));
        RETURN_IF_FAILED(m_oInputTrafficSignExt.Create("RoadSignExt", pTypeRoadSignExt, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oInputTrafficSignExt));

*/
        // steering output
        tChar const * strDescSignalValueOutputSteer = pDescManager->GetMediaDescription("tSignalValue");
        RETURN_IF_POINTER_NULL(strDescSignalValueOutputSteer);
        cObjectPtr<IMediaType> pTypeSignalValueOutputSteer = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValueOutputSteer, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(pTypeSignalValueOutputSteer->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignalOutputSteer));
        RETURN_IF_FAILED(m_oSteer.Create("steering_angle", pTypeSignalValueOutputSteer, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oSteer));

        // AvgSteering output
        tChar const * strDescSignalValueOutputAvgSteering = pDescManager->GetMediaDescription("tSignalValue");
        RETURN_IF_POINTER_NULL(strDescSignalValueOutputAvgSteering);
        cObjectPtr<IMediaType> pTypeSignalValueOutputAvgSteering = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValueOutputAvgSteering, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(pTypeSignalValueOutputAvgSteering->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignalOutputAvgSteering));
        RETURN_IF_FAILED(m_oAvgSteering.Create("AvgSteering_angle", pTypeSignalValueOutputAvgSteering, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oAvgSteering));

        // acceleration output
        tChar const * strDescSignalValueOutputAccel = pDescManager->GetMediaDescription("tSignalValue");
        RETURN_IF_POINTER_NULL(strDescSignalValueOutputAccel);
        cObjectPtr<IMediaType> pTypeSignalValueOutputAccel = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValueOutputAccel, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(pTypeSignalValueOutputAccel->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignalOutputAccel));
        RETURN_IF_FAILED(m_oAccelerate.Create("acceleration", pTypeSignalValueOutputAccel, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oAccelerate));

	 // Threshold output
        tChar const * strDescSignalValueOutputThres = pDescManager->GetMediaDescription("tSignalValue");
        RETURN_IF_POINTER_NULL(strDescSignalValueOutputThres);
        cObjectPtr<IMediaType> pTypeSignalValueOutputThre = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValueOutputThres, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(pTypeSignalValueOutputThre->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescThreshold));
        RETURN_IF_FAILED(m_oThreshold.Create("Threshold", pTypeSignalValueOutputThre, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oThreshold));

	// Lanechange output
        tChar const * strDescSignalValueOutputlanechng = pDescManager->GetMediaDescription("tCheckTrafficForCrossing");
        RETURN_IF_POINTER_NULL(strDescSignalValueOutputlanechng);
        cObjectPtr<IMediaType> pTypeSignalValueOutputlanechng = new cMediaType(0, 0, 0, "tCheckTrafficForCrossing", strDescSignalValueOutputlanechng, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(pTypeSignalValueOutputlanechng->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDesclanechange));
        RETURN_IF_FAILED(m_olanechange.Create("Lane Change", pTypeSignalValueOutputlanechng, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_olanechange));


        // Video output main
        RETURN_IF_FAILED(m_oVideoOutputPin.Create("Video_Output", IPin::PD_Output, static_cast<IPinEventSink*>(this)));
        RETURN_IF_FAILED(RegisterPin(&m_oVideoOutputPin));

	 RETURN_IF_FAILED(m_oVideoOutputPin_raw.Create("Video_Output Raw", IPin::PD_Output, static_cast<IPinEventSink*>(this)));
        RETURN_IF_FAILED(RegisterPin(&m_oVideoOutputPin_raw));

	 RETURN_IF_FAILED(m_oVideoOutputPin_thres.Create("Video_Output Thre", IPin::PD_Output, static_cast<IPinEventSink*>(this)));
        RETURN_IF_FAILED(RegisterPin(&m_oVideoOutputPin_thres));



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
        thresholdvalue = GetPropertyInt("thresholdvalue");
        enable_imshow = GetPropertyBool("imshow");
        houghlinesvalue = GetPropertyInt("houghlines");
        row1 = GetPropertyInt("row1");
        row2 = GetPropertyInt("row2");
        col1 = GetPropertyInt("col1");
        col2 = GetPropertyInt("col2");
        iVmaxSteeringAngle =GetPropertyInt("v-max Steering angle");
        m_f32Distance4RelevantRoadSign = GetPropertyFloat("Distance for relecant Road Signs [m]");
        steeringAngle_old1=0;
        steeringAngle_old2=0;
        steeringAngle_old3=0;
        steeringAngle_old4=0;
        steeringAngle_old5=0;
        steeringAngle_old6=0;
        steeringAngle_old7=0;
        steeringAngle_old8=0;
        steeringAngle_old9=0;
        steeringAngle_old10=0;
        steeringAngle_old11=0;
        steeringAngle_old12=0;
        steeringAngle_old13=0;
        steeringAngle_old14=0;
        steeringAngle_old15=0;
        steeringAngle_Avg10=0;
	m_bAvg_Update=tTrue;
	thresholarray[0]=130;thresholarray[1]=140;thresholarray[2]=145;thresholarray[3]=150;thresholarray[4]=155;thresholarray[5]=160;
	



        m_bRelevantRoadSignDetected = false;
        m_iCounterRelevantRoadSign = 0;
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
	

        // Receive start or stop order


        if(pSource == &m_bInputLaneFollowerStart){
	    tBool m_onetimewrite=tFalse; 
            cObjectPtr<IMediaCoder> pCoderInput;
            RETURN_IF_FAILED(m_pCoderDescSignalInputLaneFollowerStart->Lock(pMediaSample, &pCoderInput));
            pCoderInput->Get("bValue", (tVoid*)&m_bLaneFollowerStart);
            //pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timestamp);
            m_pCoderDescSignalInputLaneFollowerStart->Unlock(pCoderInput);
		if(!m_bLaneFollowerStart)
		{
                    accel = 0;
                    steeringAngle =0;
                    timeStamp = _clock->GetStreamTime() / 1000;
		    if(!m_onetimewrite)
 		    {
 	       	    writeOutputs(timeStamp);
		    m_onetimewrite=tTrue;
                    m_bFirstupdate=tTrue;

		    }	

		}	

        }
	else if(pSource == &m_oThreshold)
	 {
	 
            cObjectPtr<IMediaCoder> pCoderInput;
            RETURN_IF_FAILED(m_pCoderDescThreshold->Lock(pMediaSample, &pCoderInput));
            pCoderInput->Get("f32Value", (tVoid*)&thresholdvalue);
            m_pCoderDescThreshold->Unlock(pCoderInput);
	}	
        else if (pSource == &m_oStart_test)
                {	
			tBool m_onetimewrite=tFalse; 
                    cObjectPtr<IMediaCoder> pCoderInput;
                    RETURN_IF_FAILED(m_pDescStart_test->Lock(pMediaSample, &pCoderInput));
                    pCoderInput->Get("bValue", (tVoid*)&m_bLaneFollowerStart);
                    m_pDescStart_test->Unlock(pCoderInput);
                    m_bFirstupdate=tTrue;
				if(!m_bLaneFollowerStart)
				{
				    accel = 0;
				    steeringAngle =0;
				    timeStamp = _clock->GetStreamTime() / 1000;
				    if(!m_onetimewrite)
				    {
			       	    writeOutputs(timeStamp);
				    m_onetimewrite=tTrue;

				    }	
                                }

	    	}
        else if(pSource== &m_oInputTrafficSign)
        {
            __adtf_sample_read_lock_mediadescription(m_pDescriptionInputTrafficSign,pMediaSample,pCoderInput);
            // During a Maneuver no road sign should be processed
            tInt8 i8CurrentRoadSignID;
            // get IDs
            tFloat32 f32ArrayRoadSignDistanceY[3];
            tFloat32 f32ArrayRoadSignDistanceX[3];
            f32ArrayRoadSignDistanceY [2] = 0;
            f32ArrayRoadSignDistanceX [2] = 0;

            if(!m_bIDsRoadSignSet)
            {
                        pCoderInput->GetID("i16Identifier",m_szIDRoadSignI16Identifier);
                        pCoderInput->GetID("af32TVec", m_szIDRoadSignTVec);
                        pCoderInput->GetID("af32RVec", m_szIDRoadSignRVec);
                        m_bIDsRoadSignSet = true;
            }
            pCoderInput->Get("i16Identifier", (tVoid*)&i8CurrentRoadSignID);
            pCoderInput->Get("af32TVec", (tVoid*)&f32ArrayRoadSignDistanceY);
            pCoderInput->Get("af32RVec", (tVoid*)&f32ArrayRoadSignDistanceX);

            tFloat32 f32RoadSignDistanceY = f32ArrayRoadSignDistanceY[2];
            tFloat32 f32RoadSignDistanceX = f32ArrayRoadSignDistanceX[2];
            // LOG_INFO(adtf_util::cString::Format("f32RoadSignDistanceY %f",f32RoadSignDistanceY));

            tFloat32 f32Distance2RoadSign;
            f32Distance2RoadSign = sqrt((f32RoadSignDistanceY * f32RoadSignDistanceY) + (f32RoadSignDistanceX * f32RoadSignDistanceX));

            // Check if the RoadSign is closer than 1.2 meter
            //if(bDebugOutputEnabled) LOG_WARNING(adtf_util::cString::Format("Lane Follower: f32Distance2RoadSign = %f", f32Distance2RoadSign));

            if(f32Distance2RoadSign < m_f32Distance4RelevantRoadSign)
            {
                // Check if it is a relevant Road Sign
                if(i8CurrentRoadSignID <7 || i8CurrentRoadSignID == 12)
                {
                      m_bRelevantRoadSignDetected = true;
                      m_iCounterRelevantRoadSign = 0;
                }
                else
                {
                      m_bRelevantRoadSignDetected = false;
                }
            }
            else
            {
                m_bRelevantRoadSignDetected = false;
            }

            // if(bDebugOutputEnabled) LOG_INFO(adtf_util::cString::Format("m_iTrafficSignID: %i", m_iTrafficSignID));

        }


        // Check if you have video input and the start-flag (m_bLaneFollowerStart) is true
        else if (pSource == &m_oVideoInputPin && m_bLaneFollowerStart)
        {
            //check if video format is still unkown
            if (m_sInputFormat.nPixelFormat == IImage::PF_UNKNOWN)
            {
                RETURN_IF_FAILED(UpdateInputImageFormat(m_oVideoInputPin.GetFormat()));
           }
    	    tUInt32 timeStamp = 0;
            
	    if (firstFrame)
            {
                m_bFirstupdate=tTrue;
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
            if(m_bFirstupdate)
            {
                m_bFirstupdate=tFalse;
		steeringAngle_row=steeringAngle;
                steeringAngle_old1=0;
                steeringAngle_old2=0;
                steeringAngle_old3=0;
                steeringAngle_old4=0;
                steeringAngle_old5=0;
                steeringAngle_old6=0;
                steeringAngle_old7=0;
                steeringAngle_old8=0;
                steeringAngle_old9=0;
                steeringAngle_old10=0;
                steeringAngle_old11=0;
                steeringAngle_old12=0;
                steeringAngle_old13=0;
                steeringAngle_old14=0;
                steeringAngle_old15=0;
		steeringAngle_old16=0;
                steeringAngle_old17=0;
                steeringAngle_old18=0;
                steeringAngle_old19=0;
                steeringAngle_old20=0;
                
                steeringAngle_Avg10=(steeringAngle_old1+steeringAngle_old2+steeringAngle_old3+steeringAngle_old4+steeringAngle_old5+steeringAngle_old6+steeringAngle_old7+steeringAngle_old8+steeringAngle_old9+steeringAngle_old10)/10;


            }
            if(blind_count > 60)
            {
                        // if(bDebugOutputEnabled) LOG_INFO(adtf_util::cString::Format("Lane Follower: Bilnd Count"));
			 m_twoline = tFalse;
	   		 m_noline  = tTrue;
           		 m_oneline = tFalse; 
                         accel = 0;
            }
            else
            {
                // Transmit acceleration

                    // Check if a valid(not order than 10s) traffic sign is detected
                   // if(bDebugOutputEnabled) LOG_WARNING(adtf_util::cString::Format("Lane Follower: m_bRelevantRoadSignDetected = %i, m_iCounterRelevantRoadSign = %i", m_bRelevantRoadSignDetected, m_iCounterRelevantRoadSign));
                    if(m_bRelevantRoadSignDetected == true && m_iCounterRelevantRoadSign < 10)
                    {
                          // Relevant road sign detected -> drive slow
                          accel = vRoadSign/100;
                         // if(bDebugOutputEnabled) LOG_WARNING(adtf_util::cString::Format("Lane Follower: RELEVANT road sign -> vRoadSign = %f", accel));

                    }
                    // No current traffic sign
                    else
                    {
                          accel = vMax/100;
                        //  if(bDebugOutputEnabled) LOG_WARNING(adtf_util::cString::Format("Lane Follower: No road sign -> vMax = %f", accel));
                    }
                    // Increase counter and reset bool for m_bRelevantRoadSignDetected to detect the passing of a Road Sign
                    m_iCounterRelevantRoadSign++;

                    if(50 < m_iCounterRelevantRoadSign)
                    {
                        m_iCounterRelevantRoadSign = 50;
                    }
                    m_bRelevantRoadSignDetected = false;
                    //if(bDebugOutputEnabled) LOG_INFO(adtf_util::cString::Format("Output: Lakshman acceleration loop2"));

              }
            
            timeStamp = _clock->GetStreamTime() / 1000;
            AduptiveSteeringAngle();
            TransmitFloatValue(&m_oAvgSteering, steeringAngle_Avg10, 0);
            //writeOutputs1(timeStamp);
            writeOutputs(timeStamp);
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

tResult clanefollower::AduptiveSteeringAngle()
{



    steeringAngle_Avg10=(steeringAngle_old1+ steeringAngle_old2+ steeringAngle_old3+ steeringAngle_old4+ steeringAngle_old5+ steeringAngle_old6+ steeringAngle_old7+ steeringAngle_old8+ steeringAngle_old9+ steeringAngle_old10 )/10;

/*
//---------------------------if one line is detected--------------------------------------------------------------------------------

    if(m_oneline)
{

    if(steeringAngle_Avg10 > 25) //we are on right turn
    {

           if((steeringAngle_Avg10)-steeringAngle_row > 55 )
           {

                if(steeringAngle_Avg10> 55)steeringAngle_row=steeringAngle_Avg10;
                else steeringAngle_row=1.2*steeringAngle_Avg10;

           if(bDebugOutputEnabled) LOG_INFO(adtf_util::cString::Format("Aduptive Steering on left turn %f",steeringAngle_row));
		m_bAvg_Update=tFalse;
           }
           else
           {
              //steeringAngle= steeringAngle_old10;
		m_bAvg_Update=tTrue;
           }

           //------------------------------------current angle processing----------------------------------
               if   ( steeringAngle_row > 70 )
               {
                   steeringAngle_row = ((steeringAngle_row/90)*12)+steeringAngle_row;
                   if(bDebugOutputEnabled) LOG_INFO(adtf_util::cString::Format("Aduptive Steering on turn %f",steeringAngle_row));

               }
                  else if   ( steeringAngle_row > 50 )
                  {
                      steeringAngle_row = ((steeringAngle_row/90)*11)+steeringAngle_row;
                      if(bDebugOutputEnabled) LOG_INFO(adtf_util::cString::Format("Aduptive Steering on turn %f",steeringAngle_row));

                  }
               else if   ( steeringAngle_row > 35 )
               {
                   steeringAngle_row = ((steeringAngle_row/90)*7)+steeringAngle_row;
                   if(bDebugOutputEnabled) LOG_INFO(adtf_util::cString::Format("Aduptive Steering on turn %f",steeringAngle_row));

               }
                  else
                  {
                   //steeringAngle_row = steeringAngle_row-((steeringAngle_row/90)*1);
                   steeringAngle_row = steeringAngle_row;
                  }
            accel=0.4 +  2*(steeringAngle_row/1000);
           //---------------------------------------------------------------------------------------------


    }
    else if(steeringAngle_Avg10 < -25) //we are on left turn
    {

        if( steeringAngle_row-(steeringAngle_Avg10) > 55)
        {
            //steeringAngle_row=steeringAngle_Avg10;
            if(steeringAngle_Avg10 < -55)steeringAngle_row=steeringAngle_Avg10;
            else steeringAngle_row=1.2*steeringAngle_Avg10;


                  // steeringAngle=steeringAngle_old10;
                if(bDebugOutputEnabled) LOG_INFO(adtf_util::cString::Format("Aduptive Steering on Right turn %f",steeringAngle_row));
		m_bAvg_Update=tFalse;

              }
        else
        {
           //steeringAngle= steeringAngle_old10;
	m_bAvg_Update=tTrue;

        }

        //------------------------------------current angle processing----------------------------------
            if   ( steeringAngle_row < -70 )
            {
                steeringAngle_row = ((steeringAngle_row/90)*12)+steeringAngle_row;
                if(bDebugOutputEnabled) LOG_INFO(adtf_util::cString::Format("Aduptive Steering on turn %f",steeringAngle_row));

            }
               else if   ( steeringAngle_row < -50 )
               {
                   steeringAngle_row = ((steeringAngle_row/90)*11)+steeringAngle_row;
                   if(bDebugOutputEnabled) LOG_INFO(adtf_util::cString::Format("Aduptive Steering on turn %f",steeringAngle_row));

               }
               else if (steeringAngle_row < -35)
               {


                   steeringAngle_row = ((steeringAngle_row/90)*7)+steeringAngle_row;
                 if(bDebugOutputEnabled) LOG_INFO(adtf_util::cString::Format("Aduptive Steering on turn %f",steeringAngle_row));
               }

               else
               {
                //steeringAngle_row = steeringAngle_row-((steeringAngle_row/90)*1);
                steeringAngle_row = steeringAngle_row;
               }
             accel=0.4 +  2*(-steeringAngle_row/1000);

        //---------------------------------------------------------------------------------------------

    }
        else // we are one stright road
	{
                //steeringAngle= steeringAngle_old10;
		m_bAvg_Update=tTrue;

	}

    //------------------------------------current angle processing----------------------------------
        if   ( steeringAngle_row > 70 || steeringAngle_row < -70 )
        {
            steeringAngle_row = ((steeringAngle_row/90)*13)+steeringAngle_row;
            if(bDebugOutputEnabled) LOG_INFO(adtf_util::cString::Format("Aduptive Steering on turn %f",steeringAngle_row));

        }
           else if   ( steeringAngle_row > 50 || steeringAngle_row < -50 )
           {
               steeringAngle_row = ((steeringAngle_row/90)*8)+steeringAngle_row;
               if(bDebugOutputEnabled) LOG_INFO(adtf_util::cString::Format("Aduptive Steering on turn %f",steeringAngle_row));

           }
           else if (steeringAngle_row >  35 || steeringAngle_row < -35)
           {


               steeringAngle_row = ((steeringAngle_row/90)*3)+steeringAngle_row;
             if(bDebugOutputEnabled) LOG_INFO(adtf_util::cString::Format("Aduptive Steering on turn %f",steeringAngle_row));
           }
           else
           {
            //steeringAngle_row = steeringAngle_row-((steeringAngle_row/90)*1);
            steeringAngle_row = steeringAngle_row;
           }
	

  //---------------------------------------------------------------------------------------------
}




//--------------------------------------------------------------------------------------------------------------------
//----------------------------two lines are detected-----------------------------------------------------------------
    else if(m_twoline)
    {
	


        m_bAvg_Update=tTrue;

        if(bDebugOutputEnabled) LOG_INFO(adtf_util::cString::Format("Avg Steering when both lines detected = %f",steeringAngle_Avg10));
        if(steeringAngle_Avg10<-55 || steeringAngle_Avg10>55)
        {

            steeringAngle_row=(steeringAngle_old1+steeringAngle_old2+steeringAngle_old3)/3;
            m_bAvg_Update=tFalse;
           if(bDebugOutputEnabled) LOG_INFO(adtf_util::cString::Format("Avg Steering when no lines detected on turn = %f",steeringAngle_Avg10));
            accel=vRoadSign/100;
        }

       else if(steeringAngle_Avg10<-25 || steeringAngle_Avg10>25)
        {

            steeringAngle_row=(steeringAngle_old1+steeringAngle_old2+steeringAngle_old3)/3; //BEFORE IT WAS DIRECT
            m_bAvg_Update=tFalse;
           if(bDebugOutputEnabled) LOG_INFO(adtf_util::cString::Format("Avg Steering when no lines detected on turn = %f",steeringAngle_Avg10));
            accel=vRoadSign/100;
        }

        else
         {

             steeringAngle_row=steeringAngle_row; //BEFORE IT WAS DIRECT
             m_bAvg_Update=tFalse;
            if(bDebugOutputEnabled) LOG_INFO(adtf_util::cString::Format("Avg Steering when no lines detected on turn = %f",steeringAngle_Avg10));

         }
        steeringAngle =steeringAngle_row;

    }*/
//-------------------------------------------------------------------------------------------------------------------
//----------------------No lines are detected-------------------------------------------------------------------------
    if(m_noline)
    {

        if(steeringAngle_Avg10<-50 || steeringAngle_Avg10>50)
        {
            steeringAngle_row=steeringAngle_Avg10;
            //steeringAngle=steeringAngle_old10;
            if(bDebugOutputEnabled) LOG_INFO(adtf_util::cString::Format("Avg Steering when no lines detected on turn = %f",steeringAngle_Avg10));
            //accel=vRoadSign/100+0.1;
	
	m_bAvg_Update=tFalse;
        }

       else if(steeringAngle_Avg10<-25 || steeringAngle_Avg10>25)
        {
            steeringAngle=(steeringAngle_old1+steeringAngle_old2+steeringAngle_old3+steeringAngle_old4 +steeringAngle_old5)/5; // bEFORE IT WAS WITHOUT GAIN
            //steeringAngle=steeringAngle_old10;
            if(bDebugOutputEnabled) LOG_INFO(adtf_util::cString::Format("Avg Steering when no lines detected on turn = %f",steeringAngle_Avg10));
           // accel=vRoadSign/100+0.1;

        m_bAvg_Update=tFalse;
        }
        else
        {
             steeringAngle =steeringAngle_Avg10;
              //if(bDebugOutputEnabled) LOG_INFO(adtf_util::cString::Format("Aduptive Steering when no lines detected on stright Road = %f",steeringAngle));
               // accel=vRoadSign/100;
        m_bAvg_Update=tTrue;
        }


    }
/*
//---------------------------------------------------------------------------------------------------------------------
	
        if(steeringAngle_Avg10 > 35) //we are on right turn
        {
			steeringAngle =  steeringAngle_old10; //aduptive steering
            accel= accel;
            if(m_oneline && steeringAngle_row > 60 ) // need extreem steering
            {
                steeringAngle_row =  ((steeringAngle_row/90)*8)+steeringAngle_row;
                //steeringAngle =  steeringAngle_old10; //aduptive steering
            }
            else if(m_oneline && (steeringAngle_Avg10-steeringAngle_row) >= (steeringAngle_Avg10-20) ) // false detection
            {
                steeringAngle_row = steeringAngle_Avg10 +10;
            }
            else if(m_oneline && (steeringAngle_row-steeringAngle_Avg10) >= (steeringAngle_Avg10-30) ) // false detection
            {

                steeringAngle_row = steeringAngle_Avg10-5;

            }
            else if(m_oneline && steeringAngle_row > 40 ) //need normalsteering on curve
            {
                steeringAngle_row =  steeringAngle_row;
                //steeringAngle =  steeringAngle_old10; //aduptive steering
            }

            else if(m_twoline && (steeringAngle_Avg10-steeringAngle_row) >= steeringAngle_Avg10-20 ) // on right turn and two line detected false other lane detection
            {
                //steeringAngle_Avg10 = steeringAngle_Avg10+steeringAngle_Avg10-40;
                steeringAngle_row = steeringAngle_Avg10+10; // realy on history

            }
            else if(m_twoline && (steeringAngle_row-steeringAngle_Avg10) >= steeringAngle_Avg10-30 ) // on right turn and two line detected false other lane detection
            {
                //steeringAngle_Avg10 = steeringAngle_Avg10+steeringAngle_Avg10-40;
                steeringAngle_row = steeringAngle_Avg10+10; // realy on history

            }
            else
            {
                steeringAngle_row = steeringAngle_Avg10-10 ; // no line detection
				accel= accel-0.1;
            }

        }
        else if(steeringAngle_Avg10 <-35 ) //we are on left turn
        {
            accel= accel;
			steeringAngle =  steeringAngle_old10; //aduptive steering
            if(m_oneline && steeringAngle_row < -60 ) // need extreem steering
            {
                steeringAngle_row = ((steeringAngle_row/90)*8)+steeringAngle_row; //aduptive steering
            }
            else if(m_oneline && ((steeringAngle_Avg10-steeringAngle_row) >= steeringAngle_Avg10+20)) // false detection
            {
                steeringAngle_row = steeringAngle_Avg10-10;
                //steeringAngle_row = steeringAngle_Avg10-(steeringAngle_Avg10+20);
            }
            else if(m_oneline && ((steeringAngle_row-steeringAngle_Avg10) <= steeringAngle_Avg10+30)) // false detection
            {
                steeringAngle_row = steeringAngle_Avg10+5;
                //steeringAngle_row = steeringAngle_Avg10-(steeringAngle_Avg10+20);
            }
            else if(m_oneline && steeringAngle_row < -40 ) //need normalsteering on curve
            {
                steeringAngle_row =steeringAngle_row;
            }
            else if(m_twoline &&  (steeringAngle_Avg10-steeringAngle_row) <= steeringAngle_Avg10+20  ) // on right turn and two line detected false other lane detection
            {
                steeringAngle_row = steeringAngle_Avg10-10;
            }

            else if(m_twoline &&  (steeringAngle_row-steeringAngle_Avg10) <= steeringAngle_Avg10+30  ) // on right turn and two line detected false other lane detection
            {
                steeringAngle_row = steeringAngle_Avg10+10;
            }


            else //no line detection
            {
                steeringAngle_row = steeringAngle_Avg10-10 ; // no line detewction
                accel =accel-0.1;
            }

        }
        else // we are on stright road
        {
			steeringAngle =  steeringAngle_old10; //aduptive steering
            if(m_oneline)
            {
               //steeringAngle_row = -steeringAngle_Avg10 ; //on stright rode but one line detection so go to history and give opposite steering to see two lines
               accel= accel;
               steeringAngle_row = 0.8*steeringAngle_row;

            }
            else if(m_twoline)
            {
                //steeringAngle_row=steeringAngle_Avg10 ; //on stright rode but one line detection so go to history and give opposite steering to see two lines
                accel= accel + 0.1 ;
            }
            else
            {

            }
        }*/

        if(steeringAngle>80)steeringAngle=80;
        else if(steeringAngle <-80)steeringAngle=-80;

        steeringAngle_old20=steeringAngle_old19;
        steeringAngle_old19=steeringAngle_old18;
        steeringAngle_old18=steeringAngle_old17;
        steeringAngle_old17=steeringAngle_old16;
        steeringAngle_old16=steeringAngle_old15;
        steeringAngle_old15=steeringAngle_old14;
        steeringAngle_old14=steeringAngle_old13;
        steeringAngle_old13=steeringAngle_old12;
        steeringAngle_old12=steeringAngle_old11;
        steeringAngle_old11=steeringAngle_old10;
        steeringAngle_old10=steeringAngle_old9;
        steeringAngle_old9=steeringAngle_old8;
        steeringAngle_old8=steeringAngle_old7;
        steeringAngle_old7=steeringAngle_old6;
        steeringAngle_old6=steeringAngle_old5;
        steeringAngle_old5=steeringAngle_old4;
        steeringAngle_old4=steeringAngle_old3;
        steeringAngle_old3=steeringAngle_old2;
        steeringAngle_old2=steeringAngle_old1;
        steeringAngle_old1=steeringAngle_row;
        steeringAngle_row=steeringAngle;
/*
        if(m_oneline ) //LOG_INFO(adtf_util::cString::Format("one line  final steering = %f prodcess streering=%f avgstreering=%f",steeringAngle,steeringAngle_row,steeringAngle_Avg10));
        else if(m_twoline ) // LOG_INFO(adtf_util::cString::Format("two line  final steering = %f prodcess streering=%f Avgstreering=%f",steeringAngle,steeringAngle_row,steeringAngle_Avg10));
        else  // LOG_INFO(adtf_util::cString::Format("no line  final steerong row= %f prodcess streering=%f Avgstreering=%f",steeringAngle,steeringAngle_row,steeringAngle_Avg10));
*/
accel=accel*((110-abs(steeringAngle))/80);
        
       //accel=accel+0.01;
 RETURN_NOERROR;
}

tFloat32 clanefollower::evaluateSteeringAngle(IMediaSample* pSample)
{
    // VideoInput
    RETURN_IF_POINTER_NULL(pSample);

    cObjectPtr<IMediaSample> pNewRGBSample;

    const tVoid* l_pSrcBuffer;
	
    if (IS_OK(pSample->Lock(&l_pSrcBuffer)))
    {
    // ### START Image Processing ###
        IplImage* img = cvCreateImageHeader(cvSize(m_sInputFormat.nWidth, m_sInputFormat.nHeight), IPL_DEPTH_8U, 3);
        img->imageData = (char*)l_pSrcBuffer;
        //Übergang von OpenCV1 auf OpenCV2
        Mat image(cvarrToMat(img));
        cvReleaseImage(&img);
        pSample->Unlock(l_pSrcBuffer);
        //Zuschneiden des Bildes
	Mat imagecut,color_dst,imagecut2,imagecut3,imagecut4,color_dst_check;
         tInt sizeNoRep_check = 0;
	// Crop the image
        imagecut = image(Range(row1, row2), Range(col1, col2)).clone(); 
        
        //Erzeugen eines Graustufenbildes
	GaussianBlur(imagecut,imagecut2,Size(11,11),0,0, BORDER_DEFAULT); 
        cvtColor(imagecut2, grey, CV_BGR2GRAY);	
	/*if( blind_count>60)
	{
	for(int i=0; i<6;i++)
	{
	thresholdvalue=thresholarray[i];
   	threshold(grey, greythresh_check, thresholdvalue, 500, THRESH_BINARY);
	Canny(greythresh_check, linecanny_check, 0, 2, 3, tFalse);  
	cannysize_check = linecanny_check.size();
  	vector<Vec2f> lines_check;
   	 // How long must be the line to be detected as roadlines_check -> Houghlines_check
	HoughLines(linecanny_check, lines_check, 1, CV_PI / 180, houghlinesvalue, 0,0);
    	cvtColor( linecanny_check, color_dst_check, CV_GRAY2BGR );
	tFloat32 thetaAll_check[1000];
  	tFloat32 rhoAll_check[1000];
   	 for (tInt i = 0; i < (tInt)(lines_check.size()); i++)
   	 {
        thetaAll_check[i] = lines_check[i][1];
        rhoAll_check[i] = lines_check[i][0];
    	}

        tFloat32 thetaNoRep[1000];
        tFloat32 rhoNoRep[1000];
       

   	for (tInt i = 0; i < (tInt)(lines_check.size()); i++)
  	{
        tInt rep = 0;
        for (tInt j = 0; j < i; j++)
        {
            if (abs(thetaAll_check[i] - thetaAll_check[j]) <= 0.2)
            {
            rep = 1;		// One point of the Line is repeated, ignore the whole line
            }
        }	
        // Filter
        if ((rep == 0) && ((thetaAll_check[i]<CV_PI*0.4) || (thetaAll_check[i]>CV_PI*0.6)))
        {
            thetaNoRep[sizeNoRep_check] = thetaAll_check[i];
            rhoNoRep[sizeNoRep_check] = rhoAll_check[i];
            sizeNoRep_check += 1;
	    blind_count=0;
            //if(bDebugOutputEnabled) LOG_INFO(adtf_util::cString::Format("Output HoughLine: Lakshman_edited %d theta %f",sizeNoRep,thetaNoRep[i]));
            //if(bDebugOutputEnabled) LOG_INFO(adtf_util::cString::Format("Output HoughLine: size %d th0 %f rh0  %f",sizeNoRep,thetaNoRep[i],rhoNoRep[i]));
            //if(bDebugOutputEnabled) LOG_INFO(adtf_util::cString::Format("Output HoughLine: Lakshman_edited %d theta1 %f rho1  %d",sizeNoRep,thetaNoRep[i],rhoNoRep[i]));
        }
  	}
	if((sizeNoRep_check==2)||(sizeNoRep_check==1))
		break;
	}
	}
	else	*/
        threshold(grey, greythresh, thresholdvalue, 500, THRESH_BINARY);
	Canny(greythresh, linecanny, 0, 2, 3, tFalse);
        
	cannysize = linecanny.size();
        vector<Vec2f> lines;
        // How long must be the line to be detected as roadlines -> HoughLines  
	HoughLines(linecanny, lines, 1, CV_PI / 180, houghlinesvalue, 0,0);
        cvtColor( linecanny, color_dst, CV_GRAY2BGR );
        if(bDebugOutputEnabled) LOG_INFO(adtf_util::cString::Format("Output HoughLine: line size %d",lines.size()));

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
                if (abs(thetaAll[i] - thetaAll[j]) <= 0.2)
                {
                    rep = 1;		// One point of the Line is repeated, ignore the whole line
                }
            }	
            // Filter
            if ((rep == 0) && (((thetaAll[i]>CV_PI*0.12)&&(thetaAll[i]<CV_PI*0.4)) || ((thetaAll[i]>CV_PI*0.6))))//&&(thetaAll[i]<CV_PI*0.944)
            {
                thetaNoRep[sizeNoRep] = thetaAll[i];
                rhoNoRep[sizeNoRep] = rhoAll[i];
                sizeNoRep += 1;
           
            }

        }
         //if(bDebugOutputEnabled) LOG_INFO(adtf_util::cString::Format("Output HoughLine: Lakshman_edited %d theta0 %f rho0  %f",sizeNoRep,thetaNoRep[0],rhoNoRep[0]));
         //if(bDebugOutputEnabled) LOG_INFO(adtf_util::cString::Format("Output HoughLine: Lakshman_edited %d theta1 %f rho1  %f",sizeNoRep,thetaNoRep[1],rhoNoRep[1]));
        
	// Print the detected lines on the screen
	for(tInt i = 0; i < sizeNoRep; i++ )
        {
            double a = cos(thetaNoRep[i]), b = sin(thetaNoRep[i]);
            double x0 = a*rhoNoRep[i], y0 = b*rhoNoRep[i];
            //if(bDebugOutputEnabled) LOG_INFO(adtf_util::cString::Format("Y not %f",y0));
            
            Point pt1(cvRound(x0 + 1000*(-b)),
                      cvRound(y0 + 1000*(a)));
            Point pt2(cvRound(x0 - 1000*(-b)),
                      cvRound(y0 - 1000*(a)));
            line( color_dst, pt1, pt2, Scalar(0,0,255), 3, 8 );
        }
        // ### END Image Processing ###



        // ### START Steering Angle ###

	// Controller 1
	tFloat32 mean_theta = 0;

	tFloat32 theta0 = 0;
	tFloat32 theta1 = 0;
        /*
	if(sizeNoRep >= 2)
        {
        if((thetaNoRep[0]&&thetaNoRep[1]<=CV_PI*0.5)||(thetaNoRep[0]&&thetaNoRep[1]>=CV_PI*0.5))
        	{
        		if(thetaNoRep[0]<thetaNoRep[1])
        		{
        			thetaNoRep[0] = thetaNoRep[0];
        		}
        		else{thetaNoRep[0] = thetaNoRep[1];}
        	}
        sizeNoRep = sizeNoRep-1;
        }
	if(sizeNoRep == 1)
        {
        if((thetaNoRep[0]>=CV_PI*0.69))
        	{
        		sizeNoRep=0;
			 m_twoline = tFalse;
	   		 m_noline  = tFalse;
           		 m_oneline = tFalse; 	
				
        	}
        }
        */
        // Two lines are detected
	if (sizeNoRep >=2) {
                // is the first line the left or right line
		if (thetaNoRep[0] >= CV_PI / 2){
                	theta0 = thetaNoRep[0];
			theta1 = thetaNoRep[1];
        	} else {
                	theta0 = thetaNoRep[1];
			theta1 = thetaNoRep[0];
		}
        	mean_theta = (CV_PI - theta0) - theta1;
                if(bDebugOutputEnabled) LOG_INFO(adtf_util::cString::Format("CV_PI - theta0 %f",CV_PI - theta0));
                if(bDebugOutputEnabled) LOG_INFO(adtf_util::cString::Format("theta1 %f",theta1));
        // one line is detected
	} else {
                // multiply the angle with -1 so the car steers to the side no line is detected
              	 if (thetaNoRep[0] >= CV_PI / 2){
                	theta0 = thetaNoRep[0];
			mean_theta = (theta0 - CV_PI);
        	} else {
                	theta0 = thetaNoRep[0];
			mean_theta = theta0;
		}
	
                if(bDebugOutputEnabled) LOG_INFO(adtf_util::cString::Format("mean_theta for only one line %f",mean_theta));
	}

       if(bDebugOutputEnabled) LOG_INFO(adtf_util::cString::Format("Final Mean theta is %f",mean_theta));
	
	// Controller 2
        tFloat32 x1, x2, deltaRL = 0;
        tFloat32 deltaR = 0;
        tFloat32 deltaL = 0;

	if (sizeNoRep >= 2)
	{
            x1 = rhoNoRep[0] * cos(thetaNoRep[0]) - (80 - rhoNoRep[0] * sin(thetaNoRep[0]))*tan(thetaNoRep[0]);
            x2 = rhoNoRep[1] * cos(thetaNoRep[1]) - (80 - rhoNoRep[1] * sin(thetaNoRep[1]))*tan(thetaNoRep[1]);		 
       
                if(bDebugOutputEnabled) LOG_INFO(adtf_util::cString::Format("Output: x1 %f : x2 %f",x1,x2));
            if (x1 > x2)
	{
                deltaR = x1 - 141;
                deltaL = 141 - x2;
			
            }
            else

	{	
                deltaR = x2 - 141;
                deltaL = 141 - x1;
            }
            deltaRL = deltaR - deltaL  ;
	    m_twoline = tTrue;
	    m_noline  = tFalse;
            m_oneline = tFalse;    
        }
	if (sizeNoRep == 1){
            x1 = rhoNoRep[0] * cos(thetaNoRep[0]) - (80 - rhoNoRep[0] * sin(thetaNoRep[0]))*tan(thetaNoRep[0]);
            if (x1 > 141){
                deltaRL = x1 - 252;
                // LOG_INFO(adtf_util::cString::Format("mean_theta for only one line"));
            }
            else{
                deltaRL = x1-30;
; // deltaRL = x1 - 0;
            }
	    m_twoline = tFalse;
	    m_noline  = tFalse;
            m_oneline = tTrue; 
        }
        if (sizeNoRep == 0)
        {
	 m_twoline = tFalse;
	   		 m_noline  = tFalse;
           		 m_oneline = tFalse; 	
        blind_count++;
        }
	
        //if(bDebugOutputEnabled) LOG_INFO(adtf_util::cString::Format("Output: Lakshman deltaL %f",deltaL));
        //if(bDebugOutputEnabled) LOG_INFO(adtf_util::cString::Format("Output: Lakshman deltaR %f",deltaR));
        //if(bDebugOutputEnabled) LOG_INFO(adtf_util::cString::Format("Output: Lakshman deltaRL %f",deltaRL));
     
        if (abs(deltaRL * Ky / 1000) >= 25)
            deltaRL = 0;


        // Combination of the results from the two controllers

        // Two lines are detected
        if ((sizeNoRep != 0) && (sizeNoRep != 1)){
            mean_theta /= sizeNoRep;
            //if(bDebugOutputEnabled) LOG_INFO(adtf_util::cString::Format("Mean theta final%f",mean_theta));
            // Check if we have to set a positive (right) or negative (left) steering angle
            if (mean_theta > 0)
                steeringAngle = KpsiRIGHT/100 * mean_theta + Ky / 1000 * deltaRL;
            else
                steeringAngle = KpsiLEFT/100 * mean_theta + Ky / 1000 * deltaRL;
        }

        // One line is detected
        else if (sizeNoRep == 1)
	{
        if (mean_theta > 0)
            {
            blind_count = 0;
            //if(deltaRL < 0){deltaRL = -1*deltaRL;}

            steeringAngle = KpsiRIGHT/100 * mean_theta/2+ Ky / 1000 * deltaRL;
           // LOG_INFO(adtf_util::cString::Format("check steering%f",steeringAngle));
            }
        else 
            {
            blind_count = 0;
           //if(deltaRL > 0){deltaRL = -1*deltaRL;}
            steeringAngle = KpsiLEFT/100 * mean_theta/2+ Ky / 1000 * deltaRL;
            // LOG_INFO(adtf_util::cString::Format("check steering%f",steeringAngle));
            }
	}
        else
            {
            steeringAngle = steeringAngle_previous;
            blind_count++;
            }

      	 steeringAngle_previous = steeringAngle;
         mean_theta_previous = mean_theta;

 

         // ### END Steering Angle ###


        // Show black-white detection of the lines
         if (!imagecut.empty())
         {
        UpdateOutputImageFormat(imagecut);

        //create a cImage from CV Matrix (not necessary, just for demonstration9
        cImage newImage;
        newImage.Create(m_sOutputFormat.nWidth, m_sOutputFormat.nHeight, m_sOutputFormat.nBitsPerPixel, m_sOutputFormat.nBytesPerLine, imagecut.data);

        //create the new media sample
        cObjectPtr<IMediaSample> pMediaSample;
        RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));
        //updating media sample
        RETURN_IF_FAILED(pMediaSample->Update(_clock->GetStreamTime(), newImage.GetBitmap(), newImage.GetSize(), IMediaSample::MSF_None));
        //transmitting
        RETURN_IF_FAILED(m_oVideoOutputPin_raw.Transmit(pMediaSample));

        imagecut.release();
        }


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

        color_dst.release();
    	} 




/*
	 if (!linecanny.empty())
   	 {
        UpdateOutputImageFormat(linecanny);

        //create a cImage from CV Matrix (not necessary, just for demonstration9
        cImage newImage;
        newImage.Create(m_sOutputFormat.nWidth, m_sOutputFormat.nHeight, m_sOutputFormat.nBitsPerPixel, m_sOutputFormat.nBytesPerLine, linecanny.data);

        //create the new media sample
        cObjectPtr<IMediaSample> pMediaSample;
        RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));
        //updating media sample
        RETURN_IF_FAILED(pMediaSample->Update(_clock->GetStreamTime(), newImage.GetBitmap(), newImage.GetSize(), IMediaSample::MSF_None));
        //transmitting
        RETURN_IF_FAILED(m_oVideoOutputPin_thres.Transmit(pMediaSample));

        linecanny.release();
    	} 
        
*/

 }
	
        // if(bDebugOutputEnabled) LOG_INFO(adtf_util::cString::Format("initial steeering angle: %f", steeringAngle));

	
   	this->steeringAng = 0;
	
      	if (steeringAngle != 0)
        	this->steeringAng = (steeringAngle*steermulti/1000);		// after 90 is -ve, then 0.856 refers to 60/70

        // Check if the calculated steering angle is bigger than allowed (have a buffer of 5)
   	if (this->steeringAng > 95)
  		this->steeringAng = 95;
   	if (this->steeringAng < -95)
    	   this->steeringAng = -95;
     //   if(bDebugOutputEnabled) LOG_INFO(adtf_util::cString::Format("Final Steering angle : %f", steeringAng));
    	return this->steeringAng;
	
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

      //  if(bDebugOutputEnabled) LOG_INFO(adtf_util::cString::Format("Output: Size %d x %d ; BPL %d ; Size %d , PixelFormat; %d", m_sOutputFormat.nWidth, m_sOutputFormat.nHeight, m_sOutputFormat.nBytesPerLine, m_sOutputFormat.nSize, m_sOutputFormat.nPixelFormat));
        //set output format for output pin
        m_oVideoOutputPin.SetFormat(&m_sOutputFormat, NULL);
    }
    RETURN_NOERROR;
}

tResult clanefollower::writeOutputs(tUInt32 timeStamp)
{

	/* Acceleration outputs */

    //create new media sample
    cObjectPtr<IMediaSample> pMediaSampleAccel;
    AllocMediaSample((tVoid**)&pMediaSampleAccel);
    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializerAccel;
    m_pCoderDescSignalOutputAccel->GetMediaSampleSerializer(&pSerializerAccel);
    tInt nSizeAccel = pSerializerAccel->GetDeserializedSize();
    pMediaSampleAccel->AllocBuffer(nSizeAccel);
    //write date to the media sample with the coder of the descriptor
    cObjectPtr<IMediaCoder> pCoderOutputAccel;
    m_pCoderDescSignalOutputAccel->WriteLock(pMediaSampleAccel, &pCoderOutputAccel);
    // ...
    pCoderOutputAccel->Set("f32Value", (tVoid*)&(accel));
    pCoderOutputAccel->Set("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
    m_pCoderDescSignalOutputAccel->Unlock(pCoderOutputAccel);
    //transmit media sample over output pin
    pMediaSampleAccel->SetTime(_clock->GetStreamTime());
    m_oAccelerate.Transmit(pMediaSampleAccel);
	
	/* Streeing outputs */

    //create new media sample
    cObjectPtr<IMediaSample> pMediaSampleSteer;
    AllocMediaSample((tVoid**)&pMediaSampleSteer);
    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializerSteer;
    m_pCoderDescSignalOutputSteer->GetMediaSampleSerializer(&pSerializerSteer);
    tInt nSizeSteer = pSerializerSteer->GetDeserializedSize();
    pMediaSampleSteer->AllocBuffer(nSizeSteer);
    //write date to the media sample with the coder of the descriptor
    cObjectPtr<IMediaCoder> pCoderOutputSteer;
    m_pCoderDescSignalOutputSteer->WriteLock(pMediaSampleSteer, &pCoderOutputSteer);
    // ...
    pCoderOutputSteer->Set("f32Value", (tVoid*)&(steeringAngle));
    pCoderOutputSteer->Set("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
    m_pCoderDescSignalOutputSteer->Unlock(pCoderOutputSteer);
    //transmit media sample over output pin
    pMediaSampleSteer->SetTime(_clock->GetStreamTime());
    m_oSteer.Transmit(pMediaSampleSteer);

	/* Lane Change Outputs */

  	//create new media sample
    cObjectPtr<IMediaSample> pMediaSampleLanechng;
    AllocMediaSample((tVoid**)&pMediaSampleLanechng);
    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializerLanechng;
    m_pCoderDesclanechange->GetMediaSampleSerializer(&pSerializerLanechng);
    tInt nSizeLanechng = pSerializerLanechng->GetDeserializedSize();
    pMediaSampleLanechng->AllocBuffer(nSizeLanechng);
    //write date to the media sample with the coder of the descriptor
    cObjectPtr<IMediaCoder> pCoderOutputlanechng;
    m_pCoderDesclanechange->WriteLock(pMediaSampleLanechng, &pCoderOutputlanechng);
    // ...
    pCoderOutputlanechng->Set("bLeft", (tVoid*)&(m_noline));  				// for no line being detected
    pCoderOutputlanechng->Set("bStraight", (tVoid*)&(m_oneline));			// for one line being detected
    pCoderOutputlanechng->Set("bRigth", (tVoid*)&(m_twoline));				// for two line being detected
    pCoderOutputlanechng->Set("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
    m_pCoderDesclanechange->Unlock(pCoderOutputlanechng);
    //transmit media sample over output pin
    pMediaSampleLanechng->SetTime(_clock->GetStreamTime());
    m_olanechange.Transmit(pMediaSampleLanechng);

    RETURN_NOERROR;

}
tResult clanefollower::writeOutputs1(tUInt32 timeStamp)
{

    //create new media sample
    cObjectPtr<IMediaSample> pMediaSampleThres;
    AllocMediaSample((tVoid**)&pMediaSampleThres);
    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializerThres;
    m_pCoderDescThreshold->GetMediaSampleSerializer(&pSerializerThres);
    tInt nSizeAccel = pSerializerThres->GetDeserializedSize();
    pMediaSampleThres->AllocBuffer(nSizeAccel);
    //write date to the media sample with the coder of the descriptor
    cObjectPtr<IMediaCoder> pCoderOutputThres;
    m_pCoderDescThreshold->WriteLock(pMediaSampleThres, &pCoderOutputThres);
    // ...
    pCoderOutputThres->Set("f32Value", (tVoid*)&(thresholdvalue));
    pCoderOutputThres->Set("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
    m_pCoderDescThreshold->Unlock(pCoderOutputThres);
    //transmit media sample over output pin
    pMediaSampleThres->SetTime(_clock->GetStreamTime());
    m_oThreshold.Transmit(pMediaSampleThres);
	RETURN_NOERROR;

}
tResult clanefollower::TransmitFloatValue(cOutputPin* oPin, tFloat32 value, tUInt32 timestamp)
{
        //use mutex
        __synchronized_obj(m_critSecTransmitControl);

        cObjectPtr<IMediaSample> pMediaSample;
        AllocMediaSample((tVoid**)&pMediaSample);

        cObjectPtr<IMediaSerializer> pSerializer;
        m_pDescriptionFloat->GetMediaSampleSerializer(&pSerializer);
        pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());

        static bool hasID = false;
        static tBufferID szIDValueOutput;
        static tBufferID szIDArduinoTimestampOutput;

        {
                __adtf_sample_write_lock_mediadescription(m_pDescriptionFloat, pMediaSample, pCoderOutput);

                if (!hasID)
                {
                        pCoderOutput->GetID("f32Value", szIDValueOutput);
                        pCoderOutput->GetID("ui32ArduinoTimestamp", szIDArduinoTimestampOutput);
                        hasID = tTrue;
                }

                pCoderOutput->Set(szIDValueOutput, (tVoid*)&value);
                pCoderOutput->Set(szIDArduinoTimestampOutput, (tVoid*)&timestamp);
        }

        pMediaSample->SetTime(_clock->GetStreamTime());

        oPin->Transmit(pMediaSample);

        RETURN_NOERROR;
}
tResult clanefollower::TransmitBoolValue(cOutputPin* oPin, bool value, tUInt32 timestamp)
{
        //use mutex
        __synchronized_obj(m_critSecTransmitBool);

        cObjectPtr<IMediaSample> pMediaSample;
        AllocMediaSample((tVoid**)&pMediaSample);

        cObjectPtr<IMediaSerializer> pSerializer;
        m_pDescriptionBool->GetMediaSampleSerializer(&pSerializer);
        pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());

        static bool hasID = false;
        static tBufferID szIDBoolValueOutput;
        static tBufferID szIDArduinoTimestampOutput;

        {
                __adtf_sample_write_lock_mediadescription(m_pDescriptionBool, pMediaSample, pCoderOutput);

                if (!hasID)
                {
                        pCoderOutput->GetID("bValue", szIDBoolValueOutput);
                        pCoderOutput->GetID("ui32ArduinoTimestamp", szIDArduinoTimestampOutput);
                        hasID = tTrue;
                }

                pCoderOutput->Set(szIDBoolValueOutput, (tVoid*)&value);
                pCoderOutput->Set(szIDArduinoTimestampOutput, (tVoid*)&timestamp);
        }

        pMediaSample->SetTime(_clock->GetStreamTime());

        oPin->Transmit(pMediaSample);

        RETURN_NOERROR;
}
