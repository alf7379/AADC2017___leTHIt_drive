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
#include "cthresh.h"

using namespace std;
using namespace cv;



tFloat32 SteeringAngleArray[1000];

// define the ADTF property names to avoid errors
ADTF_FILTER_PLUGIN("Global Thres", OID_ADTF_THres, cthresh)
cthresh::cthresh(const tChar* __info) : cFilter(__info)
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

cthresh::~cthresh()
{

}

tResult cthresh::Start(__exception)
{

    return cFilter::Start(__exception_ptr);
}

tResult cthresh::Stop(__exception)
{
    return cFilter::Stop(__exception_ptr);
}

tResult cthresh::Init(tInitStage eStage, __exception)
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

       




	 // Threshold output
        tChar const * strDescSignalValueOutputThres = pDescManager->GetMediaDescription("tSignalValue");
        RETURN_IF_POINTER_NULL(strDescSignalValueOutputThres);
        cObjectPtr<IMediaType> pTypeSignalValueOutputThre = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValueOutputThres, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(pTypeSignalValueOutputThre->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescThreshold));
        RETURN_IF_FAILED(m_oThreshold.Create("Threshold", pTypeSignalValueOutputThre, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oThreshold));

	


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
	thresholarray[0]=140;thresholarray[1]=150;thresholarray[2]=155;thresholarray[3]=160;thresholarray[4]=165;thresholarray[5]=170;
	



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



tResult cthresh::Shutdown(tInitStage eStage, ucom::IException** __exception_ptr)
{
    if (eStage == StageGraphReady)
    {
    }

    return cFilter::Shutdown(eStage, __exception_ptr);
}

tResult cthresh::OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{

    RETURN_IF_POINTER_NULL(pMediaSample);
    RETURN_IF_POINTER_NULL(pSource);
    if (nEventCode == IPinEventSink::PE_MediaSampleReceived)
    {
	

        // Receive start or stop order


       
      

        // Check if you have video input and the start-flag (m_bLaneFollowerStart) is true
     	if (pSource == &m_oVideoInputPin)
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
                    //LOG_ERROR("Spurerkennung: No Bitmap information found on pin \"input\"");
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
                 	//if(bDebugOutputEnabled) LOG_INFO(adtf_util::cString::Format("Lane Follower: Bilnd Count"));
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
            TransmitFloatValue(&m_oAvgSteering, steeringAngle_Avg10, 0);
            writeOutputs1(timeStamp);
          
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



tFloat32 cthresh::evaluateSteeringAngle(IMediaSample* pSample)
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
	if( blind_count>40)
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
	else	
        threshold(grey, greythresh, thresholdvalue, 500, THRESH_BINARY);
	Canny(greythresh, linecanny, 0, 2, 3, tFalse);
        
	cannysize = linecanny.size();
        vector<Vec2f> lines;
        // How long must be the line to be detected as roadlines -> HoughLines  
	HoughLines(linecanny, lines, 1, CV_PI / 180, houghlinesvalue, 0,0);
        cvtColor( linecanny, color_dst, CV_GRAY2BGR );
      //  if(bDebugOutputEnabled) LOG_INFO(adtf_util::cString::Format("Output HoughLine: line size %d",lines.size()));

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
            if ((rep == 0) && (((thetaAll[i]>CV_PI*0.12)&&(thetaAll[i]<CV_PI*0.4)) || ((thetaAll[i]>CV_PI*0.6)&&(thetaAll[i]<CV_PI*0.866))))
            {
                thetaNoRep[sizeNoRep] = thetaAll[i];
                rhoNoRep[sizeNoRep] = rhoAll[i];
                sizeNoRep += 1;
           
            }

        }
	//if(bDebugOutputEnabled) LOG_INFO(adtf_util::cString::Format("Output HoughLine: Lakshman_edited %d theta0 %f rho0  %f",sizeNoRep,thetaNoRep[0],rhoNoRep[0]));
     // if(bDebugOutputEnabled) LOG_INFO(adtf_util::cString::Format("Output HoughLine: Lakshman_edited %d theta1 %f rho1  %f",sizeNoRep,thetaNoRep[1],rhoNoRep[1]));
        
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
               // if(bDebugOutputEnabled) LOG_INFO(adtf_util::cString::Format("CV_PI - theta0 %f",CV_PI - theta0));
               // if(bDebugOutputEnabled) LOG_INFO(adtf_util::cString::Format("theta1 %f",theta1));
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
	
              //  if(bDebugOutputEnabled) LOG_INFO(adtf_util::cString::Format("mean_theta for only one line %f",mean_theta));
	}

        //if(bDebugOutputEnabled) LOG_INFO(adtf_util::cString::Format("Final Mean theta is %f",mean_theta));
	
	// Controller 2
        tFloat32 x1, x2, deltaRL = 0;
        tFloat32 deltaR = 0;
        tFloat32 deltaL = 0;

	if (sizeNoRep >= 2)
	{
            x1 = rhoNoRep[0] * cos(thetaNoRep[0]) - (80 - rhoNoRep[0] * sin(thetaNoRep[0]))*tan(thetaNoRep[0]);
            x2 = rhoNoRep[1] * cos(thetaNoRep[1]) - (80 - rhoNoRep[1] * sin(thetaNoRep[1]))*tan(thetaNoRep[1]);		 
       
               // if(bDebugOutputEnabled) LOG_INFO(adtf_util::cString::Format("Output: x1 %f : x2 %f",x1,x2));
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
                deltaRL = x1 - 282;
              // LOG_INFO(adtf_util::cString::Format("mean_theta for only one line"));
            }
            else{
                deltaRL = x1-0;
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
	
       // if(bDebugOutputEnabled) LOG_INFO(adtf_util::cString::Format("Output: Lakshman deltaL %f",deltaL));
       // if(bDebugOutputEnabled) LOG_INFO(adtf_util::cString::Format("Output: Lakshman deltaR %f",deltaR));
        //if(bDebugOutputEnabled) LOG_INFO(adtf_util::cString::Format("Output: Lakshman deltaRL %f",deltaRL));
     
        if (abs(deltaRL * Ky / 1000) >= 25)
            deltaRL = 0;


        // Combination of the results from the two controllers

        // Two lines are detected
        if ((sizeNoRep != 0) && (sizeNoRep != 1)){
            mean_theta /= sizeNoRep;
         //   if(bDebugOutputEnabled) LOG_INFO(adtf_util::cString::Format("Mean theta final%f",mean_theta));
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

tResult cthresh::UpdateInputImageFormat(const tBitmapFormat* pFormat)
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

tResult cthresh::UpdateOutputImageFormat(const cv::Mat& outputImage)
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


tResult cthresh::writeOutputs1(tUInt32 timeStamp)
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
tResult cthresh::TransmitFloatValue(cOutputPin* oPin, tFloat32 value, tUInt32 timestamp)
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
tResult cthresh::TransmitBoolValue(cOutputPin* oPin, bool value, tUInt32 timestamp)
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
