#ifndef _LaneFollower_FILTER_HEADER_
#define _LaneFollower_FILTER_HEADER_

#define OID_ADTF_THres  "adtf.aadc_Thres"

#include "stdafx.h"
#include "ADTF_OpenCV_helper.h"

class cthresh : public adtf::cFilter
{
	ADTF_DECLARE_FILTER_VERSION(OID_ADTF_THres, "Global Thres", adtf::OBJCAT_Tool, "AADC Global Thres", 1, 0, 0, "Beta Version");

protected:

	

	

    /*! input for rgb image */
    cVideoPin       m_oVideoInputPin;
    cInputPin       m_bInputLaneFollowerStart;
    cInputPin       m_oStart_test;
    cInputPin       m_oInputTrafficSign;
    cInputPin       m_oInputTrafficSignExt;
    tBufferID m_szIDRoadSignI16Identifier;
    tBufferID m_szIDRoadSignF32Imagesize;
    tBool m_bIDsRoadSignSet;

    /*! output for rgb image */
    cVideoPin       m_oVideoOutputPin;
    cOutputPin      m_oSteer;
cOutputPin      m_oAvgSteering;
    cOutputPin      m_oAccelerate;
    cOutputPin 	    m_oThreshold;
    cOutputPin	    m_olanechange;
    tBufferID	    m_szIdOutputSpeedControllerValue;           // Speed-Value
    tBufferID       m_szIdOutputSpeedControllerTs;		// Timestamp
    tBool           m_szIdsOutputSpeedSet;			// Bool first received
	
	// mediatype descriptions
	cObjectPtr<IMediaTypeDescription> m_pDescriptionInputTrafficSign;
        cObjectPtr<IMediaTypeDescription> m_pDescriptionInputTrafficSignExt;
	cObjectPtr<IMediaTypeDescription> m_pCoderDescSignalInputEnable;
	cObjectPtr<IMediaTypeDescription> m_pCoderDescThreshold;
	cObjectPtr<IMediaTypeDescription> m_pCoderDesclanechange;

	// critical section for current traffic sign
	cCriticalSection m_critSecCurrentTrafficSign;

	// member current traffic sign
	//tRoadSign m_oCurrentTrafficSign;

    cObjectPtr<IMediaTypeDescription> m_pCoderDescSignalOutputSteer;
   cObjectPtr<IMediaTypeDescription> m_pCoderDescSignalOutputAvgSteering;
    cObjectPtr<IMediaTypeDescription> m_pCoderDescSignalOutputAccel;
    cObjectPtr<IMediaTypeDescription> m_pCoderDescSignalInputLaneFollowerStart;
    cObjectPtr<IMediaTypeDescription> m_pDescStart_test;
        /*! The critical section transmit control */
        cCriticalSection m_critSecTransmitControl;

        /*! The critical section transmit bool */
        cCriticalSection m_critSecTransmitBool;

        /*! the media description for bool values */
        cObjectPtr<IMediaTypeDescription> m_pDescriptionBool;

        /*! the media description for float values */
        cObjectPtr<IMediaTypeDescription> m_pDescriptionFloat;

public:
    /*! default constructor for template class
        \param __info   [in] This is the name of the filter instance.
    */

   tUInt8       blind_count;
   tFloat32     steeringAng;

   tBufferID    m_szIDRoadSignTVec;
   tBufferID    m_szIDRoadSignRVec;
   tBool        m_bRelevantRoadSignDetected;
   tInt         m_iCounterRelevantRoadSign;
   
   tBool        m_bLaneFollowerStart;
   tBool        m_bAvg_Update;
   tFloat32     steeringAngle_previous,steermulti;
   tFloat32     mean_theta_previous;
   tFloat32     KpsiRIGHT, KpsiLEFT, Ky;
   tFloat32     vMax, vMin,vRoadSign;
   tBool        writeZeroOutputs;
   tBool        enable_imshow;
  tBool m_bFirstupdate;
   tInt         row1,row2,col1,col2;
   tInt         iVmaxSteeringAngle;
   tInt         cameraoffest,houghlinesvalue;
   tFloat32     accel,thresholdvalue;
   tFloat32	thresholarray[6];
   tFloat32     steeringAngle;
   tFloat32     steeringAngle_row;
   tFloat32     steeringAngle_old1;
   tFloat32     steeringAngle_old2;
   tFloat32     steeringAngle_old3;
   tFloat32     steeringAngle_old4;
   tFloat32     steeringAngle_old5;
   tFloat32     steeringAngle_old6;
   tFloat32     steeringAngle_old7;
   tFloat32     steeringAngle_old8;
   tFloat32     steeringAngle_old9;
   tFloat32     steeringAngle_old10;
   tFloat32     steeringAngle_old11;
   tFloat32     steeringAngle_old12;
   tFloat32     steeringAngle_old13;
   tFloat32     steeringAngle_old14;
   tFloat32     steeringAngle_old15;
   tFloat32     steeringAngle_old16;
   tFloat32     steeringAngle_old17;
   tFloat32     steeringAngle_old18;
   tFloat32     steeringAngle_old19;
   tFloat32     steeringAngle_old20;

   tFloat32     steeringAngle_Avg10;


   tFloat32     m_f32Distance4RelevantRoadSign;

   tUInt32      timeStamp;
   tBool        m_time;
   cv::Mat      Line;					/* matrix for the line*/
   cv::Mat      grey;					/* matrix for the gray image*/
   cv::Mat      greythresh;				/* matrix for the gray threshold*/
   cv::Size     cannysize;				/* size for the canny detector*/
   cv::Mat      linecanny;				/* size for the canny lines*/


   
   cv::Mat      grey_check;					/* matrix for the gray image*/
   cv::Mat      greythresh_check;				/* matrix for the gray threshold*/
   cv::Size     cannysize_check;				/* size for the canny detector*/
   cv::Mat      linecanny_check;				/* size for the canny lines*/
   cthresh(const tChar* __info);
    

    /*! default destructor */
    virtual ~cthresh();

    tResult Init(tInitStage eStage, ucom::IException** __exception_ptr);

    tResult Shutdown(tInitStage eStage, ucom::IException** __exception_ptr = NULL);
    tFloat32 evaluateSteeringAngle(IMediaSample* pSample);
    tResult OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample);

    tResult Start(ucom::IException** __exception_ptr = NULL);

    tResult Stop(ucom::IException** __exception_ptr = NULL);


private: // private methods

    tResult UpdateInputImageFormat(const tBitmapFormat* pFormat);

    tResult UpdateOutputImageFormat(const cv::Mat& outputImage);

    tResult ProcessVideo(IMediaSample* pSample);
    tResult TransmitSpeed(tFloat32 speed, tUInt32 timestamp);
    tResult writeOutputs(tUInt32 timeStamp);
    tResult writeOutputs1(tUInt32 timeStamp);
    tResult AduptiveSteeringAngle();
        tResult TransmitBoolValue(cOutputPin* pin, tBool value, tUInt32 timestamp);
        tResult TransmitFloatValue(cOutputPin* pin, tFloat32 value, tUInt32 timestamp);
    /*! bitmap format of input pin */
    tBitmapFormat m_sInputFormat;

    /*! bitmap format of output pin */
    tBitmapFormat m_sOutputFormat;
	// traffic sign
	//tFloat32 m_fTrafficSignImageSize;
    tInt16      m_iTrafficSignID;
    tInt16      m_iPreviousTrafficSignID;
    tInt16      m_iCurrentTrafficSignID;
    tInt16      m_iRoadSignDetectorCounter;
    /*! tha last received input image*/
    Mat         m_inputImage;
    tBool       firstFrame;				/* flag for the first frame*/
    tUInt8      imagecount;				/* counter for the images*/

    tTimeStamp  starttime;
    tTimeStamp  delaytime;
    tTimeStamp  RoadSignTime;

    tBool m_noline,m_oneline,m_twoline;			/* flags for lane change line detection inputs */
    tBool bDebugOutputEnabled;
};

/** @} */ // end of group

#endif  //_LaneFollower_FILTER_HEADER_
