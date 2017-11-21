#ifndef _LaneFollower_FILTER_HEADER_
#define _LaneFollower_FILTER_HEADER_

#define OID_ADTF_Stopline2oneslot  "adtf.aadc_Stopline2oneslot"

#include "stdafx.h"
#include "ADTF_OpenCV_helper.h"

class clanefollower : public adtf::cFilter
{
	ADTF_DECLARE_FILTER_VERSION(OID_ADTF_Stopline2oneslot, "Stopline2oneslot", adtf::OBJCAT_Tool, "AADC Stopline2oneslot", 1, 0, 0, "Beta Version");

protected:
    /*! input for rgb image */
    cVideoPin       m_oVideoInputPin;
    cInputPin       m_bInputLaneFollowerStart;
    cInputPin    m_oInputTrafficSign;
    tBufferID m_szIDRoadSignI16Identifier;
    tBufferID m_szIDRoadSignF32Imagesize;
    tBool m_bIDsRoadSignSet;

    /*! output for rgb image */
    cVideoPin       m_oVideoOutputPin;
    cOutputPin      m_oSteer;
    cOutputPin      m_oAccelerate;
    cOutputPin      m_Stoplinedist;
    tBufferID	    m_szIdOutputSpeedControllerValue;           // Speed-Value
    tBufferID       m_szIdOutputSpeedControllerTs;		// Timestamp
    tBool           m_szIdsOutputSpeedSet;			// Bool first received

	

	
	// mediatype descriptions
	cObjectPtr<IMediaTypeDescription> m_pDescriptionInputTrafficSign;
	cObjectPtr<IMediaTypeDescription> m_pCoderDescSignalInputEnable;

	// critical section for current traffic sign
	cCriticalSection m_critSecCurrentTrafficSign;

	// member current traffic sign
	//tRoadSign m_oCurrentTrafficSign;

    cObjectPtr<IMediaTypeDescription> m_pDesStoplinedist;
    cObjectPtr<IMediaTypeDescription> m_pCoderDescSignalOutputAccel;
    cObjectPtr<IMediaTypeDescription> m_pCoderDescSignalInputLaneFollowerStart;

public:
    /*! default constructor for template class
        \param __info   [in] This is the name of the filter instance.
    */

   tUInt8       blind_count;
   tFloat32     steeringAng;
   
   tBool        m_bLaneFollowerStart;
   tFloat32     steeringAngle_previous,steermulti;
   tFloat32     mean_theta_previous;
   tFloat32     KpsiRIGHT, KpsiLEFT, Ky;
   tFloat32     vMax, vMin,vRoadSign;
   tBool        writeZeroOutputs;
   tBool        enable_imshow;
   tInt         row1,row2,col1,col2;
   tInt         iVmaxSteeringAngle;
   tInt         cameraoffest,thresholdvalue,houghlinesvalue;
   tFloat32     accel;
   tFloat32     steeringAngle;
   tUInt32      timeStamp;
   tBool        m_time;
   cv::Mat      Line;					/* matrix for the line*/
   cv::Mat      grey;					/* matrix for the gray image*/
   cv::Mat      greythresh;				/* matrix for the gray threshold*/
   cv::Size     cannysize;				/* size for the canny detector*/
   cv::Mat      linecanny;				/* size for the canny lines*/
   clanefollower(const tChar* __info);
    

    /*! default destructor */
    virtual ~clanefollower();

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
    tResult writeOutputs2(tFloat32 accel, tUInt32 timeStamp);
    tResult writeOutputs1(tFloat32 steeringAngle, tUInt32 timeStamp);
    tResult TransmitSpeed(tFloat32 speed, tUInt32 timestamp);
    tResult writeOutputs1(tUInt32 timeStamp);
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
	tBool stop_detect;
     tFloat32 distance;
     tFloat32 Orientation;
     tFloat32 distance_prev;


    tBool bDebugOutputEnabled;
};

/** @} */ // end of group

#endif  //_LaneFollower_FILTER_HEADER_
