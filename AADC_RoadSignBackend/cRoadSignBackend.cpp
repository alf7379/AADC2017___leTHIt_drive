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

#include "stdafx.h"
#include "cRoadSignBackend.h"

/// Create filter shell
ADTF_FILTER_PLUGIN("User RoadSignBackend", OID_ADTF_ROADSIGNBACKEND_FILTER, cRoadSignBackend);


cRoadSignBackend::cRoadSignBackend(const tChar* __info):cFilter(__info)
{
    SetPropertyStr("Configuration","roadSigns_Finale.xml");
    SetPropertyBool("Configuration" NSSUBPROP_FILENAME, tTrue);
    SetPropertyStr("Configuration" NSSUBPROP_FILENAME NSSUBSUBPROP_EXTENSIONFILTER, "XML Files (*.xml)");
    SetPropertyStr("Configuration" NSSUBPROP_DESCRIPTION, "Configuration file for the roadsign coordinates");
}

cRoadSignBackend::~cRoadSignBackend()
{
}

tResult cRoadSignBackend::Init(tInitStage eStage, __exception)
{
    // never miss calling the parent implementation!!
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))

    // in StageFirst you can create and register your static pins.
    if (eStage == StageFirst)
    {
        //get the description manager for this filter
  	cObjectPtr<IMediaDescriptionManager> pDescManager;
  	RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));

        // create pin for receiving road sign data form tRoadSign
	tChar const * strDescRoadSignData = pDescManager->GetMediaDescription("tRoadSignExt");
	RETURN_IF_POINTER_NULL(strDescRoadSignData);
	cObjectPtr<IMediaType> pTypeSignalRoadSignData = new cMediaType(0, 0, 0, "tRoadSignExt", strDescRoadSignData, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
	RETURN_IF_FAILED(pTypeSignalRoadSignData->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescRoadSignData));
	RETURN_IF_FAILED(m_oRoadSignData.Create("tRoadSignExt", pTypeSignalRoadSignData, static_cast<IPinEventSink*> (this)));
	RETURN_IF_FAILED(RegisterPin(&m_oRoadSignData));

        // create pin for receiving position data form Marker Positioning
	tChar const * strDescCarPositionData = pDescManager->GetMediaDescription("tPosition");
	RETURN_IF_POINTER_NULL(strDescCarPositionData);
	cObjectPtr<IMediaType> pTypeSignalCarPositionData = new cMediaType(0, 0, 0, "tPosition", strDescCarPositionData, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
	RETURN_IF_FAILED(pTypeSignalCarPositionData->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescCarPositionData));
	RETURN_IF_FAILED(m_oCarPositionData.Create("Car Position Data", pTypeSignalCarPositionData, static_cast<IPinEventSink*> (this)));
	RETURN_IF_FAILED(RegisterPin(&m_oCarPositionData));

/*
	//create pin for starting marker positioning
	tChar const * strDescSignalMarkerPosition = pDescManager->GetMediaDescription("tBoolSignalValue");
	RETURN_IF_POINTER_NULL(strDescSignalMarkerPosition);
	cObjectPtr<IMediaType> pTypeSignalMarkerPosition = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescSignalMarkerPosition, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
	RETURN_IF_FAILED(pTypeSignalMarkerPosition->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pMarkerPositionBool));
	RETURN_IF_FAILED(m_oOutputMarkerBool.Create("MarkerPositionBool", pTypeSignalMarkerPosition, static_cast<IPinEventSink*> (this)));
	RETURN_IF_FAILED(RegisterPin(&m_oOutputMarkerBool));
*/
  	// create pin for receiving heading from other filter
  	tChar const * strDescHeadingFirst = pDescManager->GetMediaDescription("tSignalValue");
  	RETURN_IF_POINTER_NULL(strDescHeadingFirst);
  	cObjectPtr<IMediaType> pTypeHeadingFirst = new cMediaType(0, 0, 0, "tSignalValue", strDescHeadingFirst,      IMediaDescription::MDF_DDL_DEFAULT_VERSION);
  	RETURN_IF_FAILED(pTypeHeadingFirst->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescFirstHeading));
  	RETURN_IF_FAILED(m_oOutputFirstHeading.Create("FirstHeading", pTypeHeadingFirst, static_cast<IPinEventSink*> (this)));
  	RETURN_IF_FAILED(RegisterPin(&m_oOutputFirstHeading));

	// create output pin for sending traffic sign data to Backend
  	tChar const * strDescTrafficSign = pDescManager->GetMediaDescription("tTrafficSign");
  	RETURN_IF_POINTER_NULL(strDescTrafficSign);
  	cObjectPtr<IMediaType> pTypeTrafficSign = new cMediaType(0, 0, 0, "tTrafficSign", strDescTrafficSign, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
  	RETURN_IF_FAILED(pTypeTrafficSign->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionTrafficSign));
  	RETURN_IF_FAILED(m_OutputTrafficSign.Create("TrafficSign", pTypeTrafficSign, static_cast<IPinEventSink*> (this)));
  	RETURN_IF_FAILED(RegisterPin(&m_OutputTrafficSign));

        // create output pin for sending traffic sign data to xml
  	tChar const * strDescTrafficSignxml = pDescManager->GetMediaDescription("tTrafficSign");
  	RETURN_IF_POINTER_NULL(strDescTrafficSignxml);
  	cObjectPtr<IMediaType> pTypeTrafficSignxml = new cMediaType(0, 0, 0, "tTrafficSign", strDescTrafficSignxml, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
  	RETURN_IF_FAILED(pTypeTrafficSignxml->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionTrafficSignxml));
  	RETURN_IF_FAILED(m_OutputTrafficSignxml.Create("TrafficSignXML", pTypeTrafficSignxml, static_cast<IPinEventSink*> (this)));
  	RETURN_IF_FAILED(RegisterPin(&m_OutputTrafficSignxml));

	m_bIDsRoadSignExtSet =tFalse;        
	m_bIDsTrafficPositionSet = tFalse;
	m_PosInputSet = tFalse;
    }
    else if (eStage == StageNormal)
    {

        // load roadsign configuration
        // In this stage you would do further initialisation and/or create your dynamic pins.
        // Please take a look at the demo_dynamicpin example for further reference.
        // initialize translation and rotation vectors
        m_Tvec = Mat(3,1,CV_32F,Scalar::all(0));
        m_szIDRoadSignExtI16Identifier = 0;
        m_szIDRoadSignExtF32Imagesize = 0;
        m_szIDRoadSignExtAf32RVec = 0;
	m_szIDRoadSignExtAf32TVec = 0;

	f32x = 0;
	f32y = 0;
	f32radius = 0;
	f32speed = 0;
	f32heading = 0;

	m_szF32X = 0;
	m_szF32Y = 0;
	m_szF32Radius = 0;
	m_szF32Speed = 0;
	m_szF32Heading = 0;

	i16Identi = 0;
	f32xTS = 0;
	f32yTS = 0;
	f32angleTS = 0;

        
 	m_i16ID = 0;
        tFloat32 a0 = 0;

	RoadsignID = 0;
	RoadSignX = 0;
	RoadSignY = 0;
	RoadsignDirection = 0;
        f32Direction = 0;
        MoreThanFirstSignDetected = tFalse;
        m_fFirstHeadingAngle = 0;
        writeXML = tTrue;
        m_ui32Cnt = 0;
        count = 0;
        PreviousLongitudinal = 0;
        PreviousRoadsignID = 0;
        m_ticks = GetTime(); // init basetime

    }
    else if (eStage == StageGraphReady)
    {

        // All pin connections have been established in this stage so you can query your pins
        // about their media types and additional meta data.
        // Please take a look at the demo_imageproc example for further reference.
        Previousf32X = 0;
        Previousf32Y = 0;
        PreviousHeading = 0;
        FirstHeading1 = 0;

        FirstHeadingDetected = tFalse;
        FirstSignDetected = tFalse;
        PreviousRoadSignX = 0;
        PreviousRoadSignY = 0;
        ActivateLoop = tTrue;
        m_bMarkerFlag = tFalse;
    }

    RETURN_NOERROR;
}

tResult cRoadSignBackend::Shutdown(tInitStage eStage, __exception)
{
    // In each stage clean up everything that you initiaized in the corresponding stage during Init.
    // Pins are an exception:
    // - The base class takes care of static pins that are members of this class.
    // - Dynamic pins have to be cleaned up in the ReleasePins method, please see the demo_dynamicpin
    //   example for further reference.

    if (eStage == StageGraphReady)
    {
    }
    else if (eStage == StageNormal)
    {
    }
    else if (eStage == StageFirst)
    {
    }

    // call the base class implementation
    return cFilter::Shutdown(eStage, __exception_ptr);
}

tResult cRoadSignBackend::OnPinEvent(IPin* pSource,
                                    tInt nEventCode,
                                    tInt nParam1,
                                    tInt nParam2,
                                    IMediaSample* pMediaSample)
{
    // first check what kind of event it is
    if (nEventCode == IPinEventSink::PE_MediaSampleReceived)
    {
        // so we received a media sample, so this pointer better be valid.
        RETURN_IF_POINTER_NULL(pMediaSample);

        if (pSource == &m_oRoadSignData)
        {
         tUInt32 timestamp = 0;
         timestamp = _clock->GetStreamTime() / 1000;
         //Process Sample
         RETURN_IF_FAILED(ProcessRoadSignData(pMediaSample,timestamp)); 
    	}

        else if(pSource == &m_oCarPositionData)	
        {
	 tUInt32 timestamp = 0;
         timestamp = _clock->GetStreamTime() / 1000;
         //Process Sample
         RETURN_IF_FAILED(ProcessCarPositionData(pMediaSample,timestamp));
        }
        else if(pSource == &m_oOutputFirstHeading)
        {
        cObjectPtr<IMediaCoder> pCoderInput;
        RETURN_IF_FAILED(m_pDescFirstHeading->Lock(pMediaSample, &pCoderInput));
        pCoderInput->Get("f32Value", (tVoid*)&m_fFirstHeadingAngle);
        m_pDescFirstHeading->Unlock(pCoderInput);
        }
    }
    RETURN_NOERROR;
}

tResult cRoadSignBackend::ProcessRoadSignData(IMediaSample* pMediaSampleIn,tUInt32 timestamp)
{
   tInt16 m_f32MarkerSize = 0;
   {   // focus for sample read lock
        // read-out the incoming Media Sample
        __adtf_sample_read_lock_mediadescription(m_pDescRoadSignData,pMediaSampleIn,pCoderInput);
        // get IDs
        if(!m_bIDsRoadSignExtSet)
        {
            pCoderInput->GetID("i16Identifier",m_szIDRoadSignExtI16Identifier);
            pCoderInput->GetID("f32Imagesize", m_szIDRoadSignExtF32Imagesize);
            pCoderInput->GetID("af32RVec[0]", m_szIDRoadSignExtAf32RVec);
            pCoderInput->GetID("af32TVec[0]", m_szIDRoadSignExtAf32TVec);
            m_bIDsRoadSignExtSet = tTrue;
        }

        pCoderInput->Get(m_szIDRoadSignExtI16Identifier, (tVoid*)&m_i16ID);
        pCoderInput->Get(m_szIDRoadSignExtF32Imagesize, (tVoid*)&m_f32MarkerSize);
        pCoderInput->Get("af32TVec", (tVoid*)m_Tvec.data);
        pCoderInput->Get("af32RVec", (tVoid*)m_Rvec.data);

    }
    timestamp = _clock->GetStreamTime() / 1000;
    // ignore initial noisy markers
    if (m_ui32Cnt<50)
    {
        m_ui32Cnt++;
        RETURN_NOERROR;
    }

    // calculate translation
    lateral = m_Tvec.at<float>(0);
    longitudinal = m_Tvec.at<float>(2);
 
    tFloat32 m_f32CameraOffsetLat = 0;
    tFloat32 m_f32CameraOffsetLon = 0;
      
    // add camera offset
    lateral += m_f32CameraOffsetLat;
    longitudinal += m_f32CameraOffsetLon;
   // LOG_INFO(cString::Format("lateral %f longitudinal %f",lateral,longitudinal));
    tFloat32 d0 = sqrt(lateral*lateral+longitudinal*longitudinal);

    tFloat32 a0 = atan2(lateral, longitudinal);
    a0 = (tFloat32)normalizeAngle(a0,0.0f)*RAD2DEG; // normalize angle -pi:pi
    a0 *= -1.0; // and change direction
    
    //Data required for searchRadiusSignPrediction
    RoadsignID = m_i16ID;
    tFloat32 X_distance, Y_distance;
    tFloat32 a1 = m_fFirstHeadingAngle - PreviousHeading;
    f32Direction = PreviousHeading*RAD2DEG;
    X_distance = longitudinal*cos(a1)-lateral*sin(a1);
    Y_distance = lateral*cos(a1)+longitudinal*sin(a1);
    PreviousRoadSignX = X_distance + Previousf32X;
    PreviousRoadSignY = Y_distance + Previousf32Y;

    if(m_f32MarkerSize > 2000)
    {
    computepose(lateral,longitudinal,a1);
    }
    PreviousLongitudinal = longitudinal;
    PreviousRoadsignID = m_i16ID;
    //LOG_INFO(cString::Format("ID %d RoadSign First X %f RoadSign first  Y %f LOngitudinal %f",RoadsignID,RoadSignX,RoadSignY,PreviousLongitudinal));
   
    RETURN_NOERROR;
}

tResult cRoadSignBackend::ProcessCarPositionData(IMediaSample* pMediaSampleIn,tUInt32 timestamp)
{
  {   __adtf_sample_read_lock_mediadescription(m_pDescCarPositionData,pMediaSampleIn,pCoderInput);
    // get IDs
    if (!m_PosInputSet)
    {
      pCoderInput->GetID("f32x", m_szF32X);
      pCoderInput->GetID("f32y", m_szF32Y);
      pCoderInput->GetID("f32radius", m_szF32Radius);
      pCoderInput->GetID("f32speed", m_szF32Speed);
      pCoderInput->GetID("f32heading", m_szF32Heading);
      m_PosInputSet=tTrue;
    }

    pCoderInput->Get(m_szF32X, (tVoid*)&f32x);
    pCoderInput->Get(m_szF32Y, (tVoid*)&f32y);
    pCoderInput->Get(m_szF32Radius, (tVoid*)&f32radius);
    pCoderInput->Get(m_szF32Speed, (tVoid*)&f32speed);
    pCoderInput->Get(m_szF32Heading, (tVoid*)&f32heading);
    
    /*
    if(FirstHeadingDetected==tFalse)
    {
    FirstHeading1 = f32heading;
    sendHeadingValue(timestamp);
    FirstHeadingDetected = tTrue;
    }
    */
    
    Previousf32X = f32x;
    Previousf32Y = f32y;
    PreviousHeading = f32heading;
  }
  timestamp = _clock->GetStreamTime() / 1000;
  SearchSignPrediction(timestamp);
  //LOG_INFO(cString::Format("car position is X is %f Y is %f",f32x,f32y));
  RETURN_NOERROR;
}

/*! calculates normalized angle */
tFloat32 cRoadSignBackend::normalizeAngle(tFloat32 alpha, tFloat32 center)
{
    return mod(alpha-center+ static_cast<tFloat32>(M_PI), 2.0*static_cast<tFloat32>(M_PI)) + center- static_cast<tFloat32>(M_PI);
}

/*! calculates modulus after division */
tFloat32 cRoadSignBackend::mod(tFloat32 x, tFloat32 y)
{
    tFloat32 r;
    tFloat32 b_x;
    if (y == floor(y))
    {
        return x - floor(x / y) * y;
    }
    else
    {
        r = x / y;
        if (r < 0.0f)
        {
            b_x = ceil(r - 0.5f);
        }
        else
        {
            b_x = floor(r + 0.5f);
        }
        if (fabs(r - b_x) <= 2.2204460492503131E-16f * fabs(r))
        {
            return 0.0f;
        }
        else
        {
            return (r - floor(r)) * y;
        }
    }
}


tResult cRoadSignBackend::AssociateRoadSign(tUInt32 timestamp)
{
  LoadConfiguration();
  timestamp = _clock->GetStreamTime() / 1000;
  for (unsigned int i=0;i<m_roadSigns.size();i++)
    {
    if((abs((m_roadSigns[i].f32X)-RoadSignX)<0.50f)&& (abs((m_roadSigns[i].f32Y)-RoadSignY)<0.50f))
      {
      if(m_roadSigns[i].u16Id != m_i16ID)
        {
        LOG_INFO(cString::Format("ID is changed w.r.t old RoadSign Id is %d new RoadSign ID is %d",m_roadSigns[i].u16Id,m_i16ID));
        i16Identi = m_i16ID;
        f32angleTS = f32Direction;  //Angle found, Identifier available
        f32xTS = RoadSignX;
        f32yTS = RoadSignY;
        sendPositionStruct(timestamp);
        //sendPositionStructxml(timestamp);
        }
      }
    }
    RETURN_NOERROR;
}

tResult cRoadSignBackend::SearchSignPrediction(tUInt32 timestamp)
{
    LoadConfiguration();
    timestamp = _clock->GetStreamTime() / 1000;
    for (unsigned int i=0;i<m_roadSigns.size();i++)
    {
    if((abs(Previousf32X-m_roadSigns[i].f32X)<0.75f)&&(abs(Previousf32Y-m_roadSigns[i].f32Y)<0.75f))
    {
    LOG_INFO(cString::Format("The Near RoadSign is %d",m_roadSigns[i].u16Id));
    LOG_INFO(cString::Format("The PreviousRoadSignX %f m_roadSigns[i].f32X %f  PreviousRoadSignY%f m_roadSigns[i].f32Y %f m_roadSigns[i].u16Id %d PreviousRoadsignID %d",PreviousRoadSignX,m_roadSigns[i].f32X, PreviousRoadSignY,m_roadSigns[i].f32Y,m_roadSigns[i].u16Id,PreviousRoadsignID));
    if(((abs(PreviousRoadSignX-m_roadSigns[i].f32X)>0.50f)&&(abs(PreviousRoadSignY-m_roadSigns[i].f32Y)>0.50f))&&(m_roadSigns[i].u16Id != PreviousRoadsignID))
    {
    LOG_INFO(cString::Format("The RoadSign has been removed which is %d",m_roadSigns[i].u16Id));
    i16Identi = -1;
    f32angleTS = m_roadSigns[i].f32Direction;  //Angle found, Identifier available
    f32xTS = m_roadSigns[i].f32X;
    f32yTS = m_roadSigns[i].f32Y;
    sendPositionStruct(timestamp);
    }
    }
    }
    RETURN_NOERROR;
}

tResult cRoadSignBackend::LoadConfiguration()
{
    cFilename fileConfig = GetPropertyStr("Configuration");

    // create absolute path for marker configuration file
    ADTF_GET_CONFIG_FILENAME(fileConfig);
    fileConfig = fileConfig.CreateAbsolutePath(".");

    if (fileConfig.IsEmpty())
    {
        RETURN_ERROR(ERR_INVALID_FILE);
    }

    tInt i = 0;
    if (cFileSystem::Exists(fileConfig))
    {
        cDOM oDOM;
        oDOM.Load(fileConfig);
        cDOMElementRefList oElems;

        if(IS_OK(oDOM.FindNodes("configuration/roadSign", oElems)))
        {
            for (cDOMElementRefList::iterator itElem = oElems.begin(); itElem != oElems.end(); ++itElem)
            {
                roadSign item;
                item.u16Id = tUInt16((*itElem)->GetAttribute("id","0").AsInt32());
                item.f32X = tFloat32((*itElem)->GetAttribute("x","0").AsFloat64());
                item.f32Y = tFloat32((*itElem)->GetAttribute("y","0").AsFloat64());
                item.f32Radius = tFloat32((*itElem)->GetAttribute("radius","0").AsFloat64());
                item.f32Direction = tFloat32((*itElem)->GetAttribute("direction","0").AsFloat64());

		item.bInit = tBool((*itElem)->GetAttribute("init","0").AsInt32());
				
                item.u16Cnt = 0;
                item.u32ticks = GetTime();

                item.f32Direction *= DEG2RAD; // convert to radians

                m_roadSigns.push_back(item);

                i++;

            }
        }
     }
    else
    {
        //LOG_ERROR("Configuration file does not exist");
        RETURN_ERROR(ERR_INVALID_FILE);
    }


    RETURN_NOERROR;
}



tResult cRoadSignBackend::computepose(tFloat32 lateral, tFloat32 longitudinal, tFloat32 a1)
{
        tUInt32 timestamp = 0;
        timestamp = _clock->GetStreamTime() / 1000;
        
        tFloat32 X_distance, Y_distance;
        X_distance = longitudinal*cos(a1)-lateral*sin(a1);
        Y_distance = lateral*cos(a1)+longitudinal*sin(a1);
        RoadSignX = X_distance + Previousf32X;
        RoadSignY = Y_distance + Previousf32Y;
        writeXML = tTrue;
        AssociateRoadSign(timestamp);
        //sendPositionStructxml(timestamp);
        //LOG_INFO(cString::Format("Roadsign X %f Roadsign Y %f",RoadSignX,RoadSignY));
        RETURN_NOERROR;
}

/*! support function for getting time */
tTimeStamp cRoadSignBackend::GetTime()
{
    return adtf_util::cHighResTimer::GetTime();
}

tResult cRoadSignBackend::sendPositionStructxml(tUInt32 timestamp)
{

    __synchronized_obj(m_oSendTrafficSignxml);

    // create new media sample
    cObjectPtr<IMediaSample> pMediaSamplexml;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSamplexml));

    // get the serializer
    cObjectPtr<IMediaSerializer> pSerializerxml;
    m_pDescriptionTrafficSignxml->GetMediaSampleSerializer(&pSerializerxml);
    tInt nSizexml = pSerializerxml->GetDeserializedSize();

    // alloc the buffer memory
    RETURN_IF_FAILED(pMediaSamplexml->AllocBuffer(nSizexml));

        cObjectPtr<IMediaCoder> pCoderOutputTrafficSignxml;
	m_pDescriptionTrafficSignxml->WriteLock(pMediaSamplexml, &pCoderOutputTrafficSignxml);
	pCoderOutputTrafficSignxml->Set("i16Identifier", (tVoid*)&RoadsignID);
	pCoderOutputTrafficSignxml->Set("f32x", (tVoid*)&RoadSignX);
	pCoderOutputTrafficSignxml->Set("f32y", (tVoid*)&RoadSignY);
	pCoderOutputTrafficSignxml->Set("f32angle", (tVoid*)&f32Direction);
	m_pDescriptionTrafficSignxml->Unlock(pCoderOutputTrafficSignxml);
        pMediaSamplexml->SetTime(_clock->GetStreamTime());

    //  doing the transmit
    RETURN_IF_FAILED(m_OutputTrafficSignxml.Transmit(pMediaSamplexml));

    RETURN_NOERROR;
}

/*! sending Roadsign position data out */
tResult cRoadSignBackend::sendPositionStruct(tUInt32 timestamp)
{

    __synchronized_obj(m_oSendTrafficSign);

    // create new media sample
    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));

    // get the serializer
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pDescriptionTrafficSign->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();

    // alloc the buffer memory
    RETURN_IF_FAILED(pMediaSample->AllocBuffer(nSize));

        cObjectPtr<IMediaCoder> pCoderOutputTrafficSign;
	m_pDescriptionTrafficSign->WriteLock(pMediaSample, &pCoderOutputTrafficSign);
	pCoderOutputTrafficSign->Set("i16Identifier", (tVoid*)&i16Identi);
	pCoderOutputTrafficSign->Set("f32x", (tVoid*)&f32xTS);
	pCoderOutputTrafficSign->Set("f32y", (tVoid*)&f32yTS);
	pCoderOutputTrafficSign->Set("f32angle", (tVoid*)&f32angleTS);
	m_pDescriptionTrafficSign->Unlock(pCoderOutputTrafficSign);
        pMediaSample->SetTime(_clock->GetStreamTime());

    //  doing the transmit
    RETURN_IF_FAILED(m_OutputTrafficSign.Transmit(pMediaSample));

    RETURN_NOERROR;
}

/*! sending Roadsign position data out */
/*
tResult cRoadSignBackend::sendHeadingValue(tUInt32 timestamp)
{
    // create new media sample
    cObjectPtr<IMediaSample> pMediaSampleHeading;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSampleHeading));

    // get the serializer
    cObjectPtr<IMediaSerializer> pSerializerHeading;
    m_pDescFirstHeading->GetMediaSampleSerializer(&pSerializerHeading);
    tInt nSize = pSerializerHeading->GetDeserializedSize();

    // alloc the buffer memory
    RETURN_IF_FAILED(pMediaSampleHeading->AllocBuffer(nSize));

        cObjectPtr<IMediaCoder> pCoderOutputFirstHeading;
	m_pDescFirstHeading->WriteLock(pMediaSampleHeading, &pCoderOutputFirstHeading);
	pCoderOutputFirstHeading->Set("ui32ArduinoTimestamp", (tVoid*)&timestamp);
	pCoderOutputFirstHeading->Set("f32Value", (tVoid*)&FirstHeading1);
	m_pDescFirstHeading->Unlock(pCoderOutputFirstHeading);
        pMediaSampleHeading->SetTime(_clock->GetStreamTime());

    //  doing the transmit
    RETURN_IF_FAILED(m_oOutputFirstHeading.Transmit(pMediaSampleHeading));

    RETURN_NOERROR;
}
*/

