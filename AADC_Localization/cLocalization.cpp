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
#include <math.h>
#include "stdafx.h"
#include "cLocalization.h"


/// Create filter shell
ADTF_FILTER_PLUGIN(ADTF_FILTER_DESC, OID_ADTF_FILTER_DEF, cLocalization);
#define MP_PROP_CAMERA_OFFSET_LAT "Camera Offset::Lateral"
#define MP_PROP_CAMERA_OFFSET_LON "Camera Offset::Longitudinal"

#define MP_PROP_SPEED_SCALE "Speed Scale"


cLocalization::cLocalization(const tChar* __info):cFilter(__info)
{

    SetPropertyFloat(MP_PROP_CAMERA_OFFSET_LAT, 0.05);
    SetPropertyBool(MP_PROP_CAMERA_OFFSET_LAT NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(MP_PROP_CAMERA_OFFSET_LAT NSSUBPROP_DESCRIPTION, "Camera offset in lateral direction");

    SetPropertyFloat(MP_PROP_CAMERA_OFFSET_LON, 0.0);
    SetPropertyBool(MP_PROP_CAMERA_OFFSET_LON NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(MP_PROP_CAMERA_OFFSET_LON NSSUBPROP_DESCRIPTION, "Camera offset in longitudinal direction");

    SetPropertyFloat(MP_PROP_SPEED_SCALE, 1.0);
    SetPropertyBool(MP_PROP_SPEED_SCALE NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(MP_PROP_SPEED_SCALE NSSUBPROP_DESCRIPTION, "Initial scale value for the speed measurement");

}

cLocalization::~cLocalization()
{

}

tResult cLocalization::Init(tInitStage eStage, __exception)
{
    // never miss calling the parent implementation!!
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))

    // in StageFirst you can create and register your static pins.
    if (eStage == StageFirst)
    {
	// get a media type for the input pin
	cObjectPtr<IMediaDescriptionManager> pDescManager;
	RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER, (tVoid**)&pDescManager,__exception_ptr));
	//get description for sensor data pins
	tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");
	RETURN_IF_POINTER_NULL(strDescSignalValue);

	//get mediatype for data pins
	cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

	// set the description for the speed pin
	RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescMeasSpeed));

	// create the speed measurement Input
	RETURN_IF_FAILED(m_oInputSpeed.Create("Speed", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
	RETURN_IF_FAILED(RegisterPin(&m_oInputSpeed));
       
        //get description for inertial measurement sensor data pin
        tChar const * strDescInerMeasUnit = pDescManager->GetMediaDescription("tInerMeasUnitData");
        RETURN_IF_POINTER_NULL(strDescInerMeasUnit);

        //get mediatype for Inertial sensor data pins
        cObjectPtr<IMediaType> pTypeInerMeasUnit = new cMediaType(0, 0, 0, "tInerMeasUnitData", strDescInerMeasUnit, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

        //get mediatype description for inertial measurement unit sensor data type
        RETURN_IF_FAILED(pTypeInerMeasUnit->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionInerMeasUnitData));

        //create pin for inertial measurement unit data
        RETURN_IF_FAILED(m_oInputInerMeasUnit.Create("InerMeasUnit_Struct", pTypeInerMeasUnit, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oInputInerMeasUnit));

    }
    else if (eStage == StageNormal)
    {
        // In this stage you would do further initialisation and/or create your dynamic pins.
        // Please take a look at the demo_dynamicpin example for further reference.
	// no ids were set so far

        // input ids
        m_bIDsMeasWheelSpeedSet = tFalse;
        m_bIDsInerMeasUnitSet   = tFalse;
	// initialize other variables
        m_f32Speed     = 0;
        m_ui32ArduinoTimestamp = 0;

        m_ui32Cnt = 0;

        m_ticks = GetTime(); // init basetime

       

    }
    else if (eStage == StageGraphReady)
    {
        // All pin connections have been established in this stage so you can query your pins
        // about their media types and additional meta data.
        // Please take a look at the demo_imageproc example for further reference.
    }

    RETURN_NOERROR;
}

tResult cLocalization::Shutdown(tInitStage eStage, __exception)
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

tResult cLocalization::OnPinEvent(IPin* pSource,
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

        // by comparing it to our member pin variable we can find out which pin received
        // the sample
	if (pSource == &m_oInputSpeed)
        {

            // write values with zero
            tFloat32 f32Value = 0;
            tUInt32 Ui32TimeStamp = 0;
            {
                // focus for sample write lock
                // read data from the media sample with the coder of the descriptor
                __adtf_sample_read_lock_mediadescription(m_pDescMeasSpeed,pMediaSample,pCoderInput);

                if(!m_bIDsMeasWheelSpeedSet)
                {
                    pCoderInput->GetID("f32Value", m_buIDMeasSpeedF32Value);
                    pCoderInput->GetID("ui32ArduinoTimestamp", m_buIDMeasSpeedArduinoTimestamp);
                    m_bIDsMeasWheelSpeedSet = tTrue;
                }
                // get values from media sample
                pCoderInput->Get(m_buIDMeasSpeedF32Value, (tVoid*)&f32Value);
                pCoderInput->Get(m_buIDMeasSpeedArduinoTimestamp, (tVoid*)&Ui32TimeStamp);
            }


            m_f32Speed = f32Value;


        }
      // process InertialMeasurementUnit sample
        else if (pSource == &m_oInputInerMeasUnit)
        {
            RETURN_IF_FAILED(ProcessInerMeasUnitSample(pMediaSample));
        }
    }

    RETURN_NOERROR;
}

tTimeStamp cLocalization::GetTime()
{
    return adtf_util::cHighResTimer::GetTime();
}
/*! processes inertial measurement data sample, and runs EKF prediction
 *  based on heading rate and speed measurements */
tResult cLocalization::ProcessInerMeasUnitSample(IMediaSample* pMediaSampleIn)
{
    // write values with zero
    tFloat32 f32G_x = 0;
    tFloat32 f32G_y = 0;
    tFloat32 f32G_z = 0;
    tFloat32 f32A_x = 0;
    tFloat32 f32A_y = 0;
    tFloat32 f32A_z = 0;
    tFloat32 f32M_x = 0;
    tFloat32 f32M_y = 0;
    tFloat32 f32M_z = 0;
    tUInt32 ui32ArduinoTimestamp = 0;

    {   // focus for sample read lock
        // read-out the incoming Media Sample
        __adtf_sample_read_lock_mediadescription(m_pDescriptionInerMeasUnitData,pMediaSampleIn,pCoderInput);

        // get the IDs for the items in the media sample
        if(!m_bIDsInerMeasUnitSet)
        {
            pCoderInput->GetID("f32G_x", m_szIDInerMeasUnitF32G_x);
            pCoderInput->GetID("f32G_y", m_szIDInerMeasUnitF32G_y);
            pCoderInput->GetID("f32G_z", m_szIDInerMeasUnitF32G_z);
            pCoderInput->GetID("f32A_x", m_szIDInerMeasUnitF32A_x);
            pCoderInput->GetID("f32A_y", m_szIDInerMeasUnitF32A_y);
            pCoderInput->GetID("f32A_z", m_szIDInerMeasUnitF32A_z);
            pCoderInput->GetID("f32M_x", m_szIDInerMeasUnitF32M_x);
            pCoderInput->GetID("f32M_y", m_szIDInerMeasUnitF32M_y);
            pCoderInput->GetID("f32M_z", m_szIDInerMeasUnitF32M_z);
            pCoderInput->GetID("ui32ArduinoTimestamp", m_szIDInerMeasUnitArduinoTimestamp);
            m_bIDsInerMeasUnitSet = tTrue;
        }

        //write date to the media sample with the coder of the descriptor
        pCoderInput->Get(m_szIDInerMeasUnitF32G_x, (tVoid*)&f32G_x);
        pCoderInput->Get(m_szIDInerMeasUnitF32G_y, (tVoid*)&f32G_y);
        pCoderInput->Get(m_szIDInerMeasUnitF32G_z, (tVoid*)&f32G_z);

        pCoderInput->Get(m_szIDInerMeasUnitF32A_x, (tVoid*)&f32A_x);
        pCoderInput->Get(m_szIDInerMeasUnitF32A_y, (tVoid*)&f32A_y);
        pCoderInput->Get(m_szIDInerMeasUnitF32A_z, (tVoid*)&f32A_z);

        pCoderInput->Get(m_szIDInerMeasUnitF32M_x, (tVoid*)&f32M_x);
        pCoderInput->Get(m_szIDInerMeasUnitF32M_y, (tVoid*)&f32M_y);
        pCoderInput->Get(m_szIDInerMeasUnitF32M_z, (tVoid*)&f32M_z);
        pCoderInput->Get(m_szIDInerMeasUnitArduinoTimestamp, (tVoid*)&ui32ArduinoTimestamp);

    }

    tFloat32 dt = static_cast<tFloat32>((ui32ArduinoTimestamp-m_ui32ArduinoTimestamp)*1e-6f);
    m_ui32ArduinoTimestamp = ui32ArduinoTimestamp;

    // filter not initialized
    if (m_isInitialized==tFalse)
    {
        RETURN_NOERROR;
    }
	/*
    // update heading
    tFloat32 hk = static_cast<tFloat32>(m_state.at<double>(2)) + (f32G_z*DEG2RAD + static_cast<tFloat32>(m_state.at<double>(3)))*dt;

    // normalize heading -pi:pi
    hk = normalizeAngle(hk,0);

    tFloat32 sc = m_f32SpeedScale;

    // update speed and scale
    tFloat32 ak = static_cast<tFloat32>(m_state.at<double>(5));
    tFloat32 vk = static_cast<tFloat32>(m_f32Speed*(sc-ak));*/
    RETURN_NOERROR;
}

/*! calculates normalized angle difference */
tFloat32 cLocalization::angleDiff(tFloat32 angle1, tFloat32 angle2)
{
    // normalization
    angle1 = normalizeAngle(angle1, static_cast<tFloat32>(M_PI));
    angle2 = normalizeAngle(angle2, static_cast<tFloat32>(M_PI));

    // compute difference and normalize in [-pi pi]
    return normalizeAngle(angle2 - angle1, 0);
}


/*! calculates normalized angle */
tFloat32 cLocalization::normalizeAngle(tFloat32 alpha, tFloat32 center)
{
    return mod(alpha-center+ static_cast<tFloat32>(M_PI), 2.0*static_cast<tFloat32>(M_PI)) + center- static_cast<tFloat32>(M_PI);
}
/*! calculates modulus after division */
tFloat32 cLocalization::mod(tFloat32 x, tFloat32 y)
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

tResult cLocalization::CalculatePose()
{	
        // calculate pose based on IMU yaw angle and wheel speed
/*
	tFloat32 car_speed_y = car_speed;
	tFloat32 car_speed_x = 0.0;
        tfloat32 timestamp;
	tFloat32 last_yaw = ( yawindegree / 180.0) * ( 3.1415926 ) * m_f32YawCorrection;
	tFloat32 dt = ( timestamp - previoustimestamp ) / 1E6;

	// calculate relative distances (TODO remove speed_y)
    	tFloat32 X_travel = ( car_speed_x * cos(last_yaw) - car_speed_y * sin(last_yaw) ) * dt;
    	tFloat32 Y_travel = ( car_speed_x * sin(last_yaw) + car_speed_y * cos(last_yaw) ) * dt;

    	X_travel += X_travel;
    	Y_travel += Y_travel;
        previoustimestamp = timestamp;
    	LOG_WARNING(cString::Format("data value of car pose x, y: %f, %f",X_travel, Y_travel));
*/
	RETURN_NOERROR;
}
