/**
Copyright (c)
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: “This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.”
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS “AS IS” AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


**********************************************************************
* $Author:: spiesra $  $Date:: 2017-05-12 09:34:53#$ $Rev:: 63109   $
**********************************************************************/
#ifndef _LOCALIZAION_FILTER_H_
#define _LOCALIZATION_FILTER_H_
#include "stdafx.h"
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define RAD2DEG static_cast<tFloat32>(180.0/M_PI)
#define DEG2RAD static_cast<tFloat32>(M_PI/180.0)

#define OID_ADTF_FILTER_DEF                "adtf.aadc.Cartrack"
#define ADTF_FILTER_DESC                   "AADC Car track"
#define ADTF_FILTER_VERSION_SUB_NAME       "Car Tracking"
#define ADTF_FILTER_VERSION_ACCEPT_LABEL   "AADC Car Track"
#define ADTF_FILTER_VERSION_STRING         "1.0.0"
#define ADTF_FILTER_VERSION_Major          1
#define ADTF_FILTER_VERSION_Minor          0
#define ADTF_FILTER_VERSION_Build          0
#define ADTF_FILTER_VERSION_LABEL          "Car Track Positioning Positioning for ADTF."
#define ADTF_CATEGORY OBJCAT_DataFilter


/*! Storage structure for the road sign data */
typedef struct _cartrack
    {
        

        /*! location */
        tFloat32 f32X;
        tFloat32 f32Y;


        /*! direction (heading) of the road sign */
        tFloat32 f32Orientation;

        tInt u16Cnt;

        tTimeStamp u32ticks;/*! measurement ticks*/

    } cartrack;


class cLocalization : public adtf::cFilter
{
   /*! This macro does all the plugin setup stuff
	* Warning: This macro opens a "protected" scope see UCOM_IMPLEMENT_OBJECT_INFO(...) in object.h
	*/
	ADTF_FILTER_VERSION(
		OID_ADTF_FILTER_DEF,
		ADTF_FILTER_DESC,
		ADTF_CATEGORY,
		ADTF_FILTER_VERSION_SUB_NAME,
		ADTF_FILTER_VERSION_Major,
		ADTF_FILTER_VERSION_Minor,
		ADTF_FILTER_VERSION_Build,
		ADTF_FILTER_VERSION_LABEL);


    public:
        cLocalization(const tChar* __info);
        virtual ~cLocalization();

public:
   /*! Input pin for the wheel speed data */
    cInputPin m_oInputSpeed;
    /*! Input pin for the inertial measurement data */
    cInputPin m_oInputInerMeasUnit;

     /*! media description for the input pin speed */
     cObjectPtr<IMediaTypeDescription> m_pDescMeasSpeed;
     /*! the id for the f32value of the media description for input pin for the speed */
     tBufferID m_buIDMeasSpeedF32Value;
     /*! the id for the arduino time stamp of the media description for input pin for thespeed */
     tBufferID m_buIDMeasSpeedArduinoTimestamp;
     /*! indicates of bufferIDs were set */
     tBool m_bIDsMeasWheelSpeedSet;

     /*! descriptor */
     cObjectPtr<IMediaTypeDescription> m_pDescriptionInerMeasUnitData;
     /*! the id for the f32x of the media description for output pin */
     tBufferID m_szIDInerMeasUnitF32G_x;
     tBufferID m_szIDInerMeasUnitF32G_y;
     tBufferID m_szIDInerMeasUnitF32G_z;

     tBufferID m_szIDInerMeasUnitF32A_x;
     tBufferID m_szIDInerMeasUnitF32A_y;
     tBufferID m_szIDInerMeasUnitF32A_z;

     tBufferID m_szIDInerMeasUnitF32M_x;
     tBufferID m_szIDInerMeasUnitF32M_y;
     tBufferID m_szIDInerMeasUnitF32M_z;
     tBufferID m_szIDInerMeasUnitArduinoTimestamp;
     /*! indicates if bufferIDs were set */
     tBool m_bIDsInerMeasUnitSet;
public:
    /*! default constructor for template class
           \param __info   [in] This is the name of the filter instance.
    */
   

    

protected:
    tResult Init(tInitStage eStage, ucom::IException** __exception_ptr);
    tResult Shutdown(tInitStage eStage, ucom::IException** __exception_ptr = NULL);
    tResult OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample);
private:
    tResult ProcessInerMeasUnitSample(IMediaSample* pMediaSampleIn);
    tResult CalculatePose();
    /*! support functions */
    tTimeStamp GetTime();

    tFloat32 mod(tFloat32 x, tFloat32 y);
    tFloat32 normalizeAngle(tFloat32 alpha, tFloat32 center);
    tFloat32 angleDiff(tFloat32 angle1, tFloat32 angle2);
    
     tTimeStamp m_ticks;

    /*! speed estimate */
    tFloat32 m_f32Speed;
    tFloat32 m_f32SpeedScale; /*! speed scalefactor */
    tUInt32 m_ui32ArduinoTimestamp;

    /*! camera offset parameters */
    tFloat32 m_f32CameraOffsetLat;
    tFloat32 m_f32CameraOffsetLon;
    tInt m_ui32Cnt;
    tBool m_isInitialized; /*! initialization state of the filter */
  
   

    
};

//*************************************************************************************************
#endif // _LOCALIZATION_FILTER_H_

/*!
*@}
*/
