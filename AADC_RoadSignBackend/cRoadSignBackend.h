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
#ifndef _ROADSIGNBACKEND_FILTER_H_
#define _ROADSIGNBACKEND_FILTER_H_

#include "stdafx.h"

#define RAD2DEG 180/M_PI
#define DEG2RAD M_PI/180
#define OID_ADTF_ROADSIGNBACKEND_FILTER "adtf.example.roadsignbackend"

/*! Storage structure for the road sign data */
typedef struct _roadSign
    {
        /*! road sign */
        tInt16 u16Id;

        /*! init sign */
        tBool bInit; 

        /*! location */
        tFloat32 f32X;
        tFloat32 f32Y;

        /*! sign search radius */
        tFloat32 f32Radius;

        /*! direction (heading) of the road sign */
        tFloat32 f32Direction;

        tInt u16Cnt;

        tTimeStamp u32ticks;/*! measurement ticks*/

    } roadSign;

class cRoadSignBackend : public adtf::cFilter
{
    /*! set the filter ID and the version */
    ADTF_FILTER(OID_ADTF_ROADSIGNBACKEND_FILTER, "User RoadSignBackend", adtf::OBJCAT_DataFilter);

protected:
    /*! the input pin */
    cInputPin    m_oRoadSignData;
    cInputPin    m_oCarPositionData;
    cOutputPin m_oOutputMarkerBool;
    cInputPin m_oOutputFirstHeading;
    cOutputPin   m_OutputTrafficSign;
    cOutputPin m_OutputTrafficSignxml;
    
    cObjectPtr<IMediaTypeDescription> m_pDescRoadSignData;
    cObjectPtr<IMediaTypeDescription> m_pDescCarPositionData;
    cObjectPtr<IMediaTypeDescription> m_pDescFirstHeading;
    cObjectPtr<IMediaTypeDescription> m_pMarkerPositionBool;
    cObjectPtr<IMediaTypeDescription> m_pDescriptionTrafficSign;
    cObjectPtr<IMediaTypeDescription> m_pDescriptionTrafficSignxml;

public:

    cRoadSignBackend(const tChar* __info);
    /*! default destructor */
    virtual ~cRoadSignBackend();
    cv::Mat m_Tvec; /*! translation vector */
    cv::Mat m_Rvec; /*! translation vector */

protected:
    tResult Init(tInitStage eStage, ucom::IException** __exception_ptr);
    tResult Shutdown(tInitStage eStage, ucom::IException** __exception_ptr = NULL);
    tResult OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample);

    /*! loads RoadSign configuration data
    */
    tResult LoadConfiguration();

        /*! support functions */
        tTimeStamp GetTime();
        tTimeStamp m_ticks;

    tResult ProcessRoadSignData(IMediaSample* pMediaSampleIn,tUInt32 timeStamp);
    tResult ProcessCarPositionData(IMediaSample* pMediaSampleIn,tUInt32 timeStamp);
    tResult sendPositionStruct(tUInt32 timeStamp);
    tResult sendPositionStructxml(tUInt32 timeStamp);
    tResult sendHeadingValue(tUInt32 timestamp);
    tResult AssociateRoadSign(tUInt32 timestamp);
    tResult SearchSignPrediction(tUInt32 timestamp);
    tResult MarkerBoolSending(tUInt32 timestamp);
    tFloat32 mod(tFloat32 x, tFloat32 y);
    tFloat32 normalizeAngle(tFloat32 alpha, tFloat32 center);
    tResult computepose(tFloat32 lateral,tFloat32 longitudinal,tFloat32 a0);

    //RoadSignExt input
    tBool m_bIDsRoadSignExtSet;
    tBufferID m_szIDRoadSignExtI16Identifier,m_szIDRoadSignExtF32Imagesize,m_szIDRoadSignExtAf32RVec,m_szIDRoadSignExtAf32TVec;

    //Position input
    tBool m_PosInputSet;
    tBufferID m_szF32X,m_szF32Y,m_szF32Radius,m_szF32Speed,m_szF32Heading;
    cCriticalSection m_oSendTrafficSign;
    cCriticalSection m_oSendTrafficSignxml;


    tBool m_bIDsTrafficPositionSet;
    tInt16 i16Identi;
    tFloat32 f32xTS;
    tFloat32 f32yTS;
    tFloat32 f32angleTS;
    tFloat32 lateral;
    tFloat32 longitudinal;
    tFloat32 f32Direction;

    tInt16 RoadsignID;
    tFloat32 RoadSignX;
    tFloat32 RoadSignY;
    tFloat32 RoadsignDirection;
    tBool MoreThanFirstSignDetected;
    tBool writeXML;

    tFloat32 Previousf32X;
    tFloat32 Previousf32Y;
    tFloat32 PreviousHeading;
    tFloat32 FirstHeading1;
    tFloat32 m_previousheadingangle; // transformation code
    tInt16 PreviousRoadsignID;
    tFloat32 PreviousLongitudinal;
    tBool FirstHeadingDetected;
    tBool FirstSignDetected;   
    tFloat32 PreviousRoadSignX;
    tFloat32 PreviousRoadSignY; 
    tBool m_bMarkerFlag;
    tBool ActivateLoop;

    tInt16 m_i16ID;
    tFloat32 Angle;
    tFloat32 a0;
    tInt16 m_ui32Cnt;
    tInt16 count;


  tFloat32 f32x;
  tFloat32 f32y;
  tFloat32 f32radius;
  tFloat32 f32speed;
  tFloat32 f32heading;
  tFloat32 m_fFirstHeadingAngle;
    /*! storage for the roadsign data */
    vector<roadSign> m_roadSigns;

};

//*************************************************************************************************
#endif // _ROADSIGNBACKEND_FILTER_H_

/*!
*@}
*/
