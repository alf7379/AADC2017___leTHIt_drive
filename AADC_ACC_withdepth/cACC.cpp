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
#include "cACC.h"
/// Create filter shell


#define DIST_OVERTAKE		"cACC::dist_overtake"
#define DIST_REDUCE_SPEED	"cACC::dist_reduce_speed"
#define WIGHT_SIDE_FRONT	"cACC::wight_side_front"
#define Critical_steering_angle	"cACC::m_f32CriticalSteeringAngle"

ADTF_FILTER_PLUGIN("ACC", OID_ADTF_ACC_FILTER, cACC);

using namespace SensorDefinition;

cACC::cACC(const tChar* __info):cFilter(__info)
{
        SetPropertyBool("Debug Output to Console",true);

        SetPropertyFloat(DIST_OVERTAKE,0.4);
    SetPropertyBool(DIST_OVERTAKE NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(DIST_OVERTAKE NSSUBPROP_DESCRIPTION, "dist to start overtake");

        SetPropertyFloat(DIST_REDUCE_SPEED,1.5);
    SetPropertyBool(DIST_REDUCE_SPEED NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(DIST_REDUCE_SPEED NSSUBPROP_DESCRIPTION, "reduce speed at this distance");

        SetPropertyFloat(WIGHT_SIDE_FRONT,1.5);
    SetPropertyBool(WIGHT_SIDE_FRONT NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(WIGHT_SIDE_FRONT NSSUBPROP_DESCRIPTION, "weighting of sensors according to turn");

        SetPropertyFloat(Critical_steering_angle,20);
    SetPropertyBool(Critical_steering_angle NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(Critical_steering_angle NSSUBPROP_DESCRIPTION, "weighting of sensors according to turn");

}

cACC::~cACC()
{

}

tResult cACC::Init(tInitStage eStage, __exception)
{
    // never miss calling the parent implementation!!
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))

    // in StageFirst you can create and register your static pins.
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
                // creeate pin for start signal input
                tChar const * strDescSignalstart = pDescManager->GetMediaDescription("tStartTrigger");
                RETURN_IF_POINTER_NULL(strDescSignalstart);
                cObjectPtr<IMediaType> pTypeSignalstart = new cMediaType(0, 0, 0, "tStartTrigger", strDescSignalstart, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
                RETURN_IF_FAILED(pTypeSignalstart->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescStart));
                RETURN_IF_FAILED(m_oStart.Create("Start", pTypeSignalstart, static_cast<IPinEventSink*> (this)));
                RETURN_IF_FAILED(RegisterPin(&m_oStart));

                //create pin for start_test pin input
                        tChar const * strDescSignalstart_test = pDescManager->GetMediaDescription("tBoolSignalValue");
                        RETURN_IF_POINTER_NULL(strDescSignalstart_test);
                        cObjectPtr<IMediaType> pTypeSignalstart_test = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescSignalstart_test, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
                        RETURN_IF_FAILED(pTypeSignalstart_test->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescStart_test));
                        RETURN_IF_FAILED(m_oStart_test.Create("Start_test1", pTypeSignalstart_test, static_cast<IPinEventSink*> (this)));
                        RETURN_IF_FAILED(RegisterPin(&m_oStart_test));

                // create pin for speed controller
                tChar const * strDescSignalSpeed = pDescManager->GetMediaDescription("tSignalValue");
                RETURN_IF_POINTER_NULL(strDescSignalSpeed);
                cObjectPtr<IMediaType> pTypeSignalSpeed = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalSpeed, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
                RETURN_IF_FAILED(pTypeSignalSpeed->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescSpeed));
                RETURN_IF_FAILED(m_oInputSpeedController.Create("Speed Controller", pTypeSignalSpeed, static_cast<IPinEventSink*> (this)));
                RETURN_IF_FAILED(RegisterPin(&m_oInputSpeedController));

                // create pin for current speed
                tChar const * strDescCurrentSpeed = pDescManager->GetMediaDescription("tSignalValue");
                RETURN_IF_POINTER_NULL(strDescCurrentSpeed);
                cObjectPtr<IMediaType> pTypeCurrentSpeed = new cMediaType(0, 0, 0, "tSignalValue", strDescCurrentSpeed, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
                RETURN_IF_FAILED(pTypeCurrentSpeed->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescCurrentSpeed));
                RETURN_IF_FAILED(m_oInputCurrentSpeed.Create("Current Speed", pTypeSignalSpeed, static_cast<IPinEventSink*> (this)));
                RETURN_IF_FAILED(RegisterPin(&m_oInputCurrentSpeed));

                // create pin for steering input
                tChar const * strDescSignalSteering = pDescManager->GetMediaDescription("tSignalValue");
                RETURN_IF_POINTER_NULL(strDescSignalSteering);
                cObjectPtr<IMediaType> pTypeSignalSteering = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalSteering, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
                RETURN_IF_FAILED(pTypeSignalSteering->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescSteering));
                RETURN_IF_FAILED(m_oInputSteering.Create("Steering", pTypeSignalSteering, static_cast<IPinEventSink*> (this)));
                RETURN_IF_FAILED(RegisterPin(&m_oInputSteering));

                // create pin for ultrasonic struct input
                tChar const * strUltrasonicStruct = pDescManager->GetMediaDescription("tUltrasonicStruct");
                RETURN_IF_POINTER_NULL(strUltrasonicStruct);
                cObjectPtr<IMediaType> pTypeUsStruct = new cMediaType(0, 0, 0, "tUltrasonicStruct", strUltrasonicStruct, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
                RETURN_IF_FAILED(pTypeUsStruct->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionUsStruct));
                RETURN_IF_FAILED(m_oInputUsStruct.Create("UsStruct", pTypeUsStruct, static_cast<IPinEventSink*> (this)));
                RETURN_IF_FAILED(RegisterPin(&m_oInputUsStruct));


                // create pin for Depth distance
                tChar const * strDescSignalDepth = pDescManager->GetMediaDescription("tSignalValue");
                RETURN_IF_POINTER_NULL(strDescSignalDepth);
                cObjectPtr<IMediaType> pTypeSignalDepth = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalDepth, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
                RETURN_IF_FAILED(pTypeSignalDepth->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescDistance));
                RETURN_IF_FAILED(m_oInputdistance.Create("Depth Distance", pTypeSignalDepth, static_cast<IPinEventSink*> (this)));
                RETURN_IF_FAILED(RegisterPin(&m_oInputdistance));

                //create pin for steering signal output

                RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionOutputSteering));
        RETURN_IF_FAILED(m_oOutputSteering.Create("SteeringOutput", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oOutputSteering));

        // create pin for steering input
        tChar const * strDescSignalAvgSteering = pDescManager->GetMediaDescription("tSignalValue");
        RETURN_IF_POINTER_NULL(strDescSignalAvgSteering);
        cObjectPtr<IMediaType> pTypeSignalAvgSteering = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalAvgSteering, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(pTypeSignalAvgSteering->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionInputAvgSteering));
        RETURN_IF_FAILED(m_oInputAvgSteering.Create("AvgSteering In", pTypeSignalAvgSteering, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oInputAvgSteering));

                //create pin for acceleration signal output
                tChar const * strDescSignalaccelerate = pDescManager->GetMediaDescription("tSignalValue");
                RETURN_IF_POINTER_NULL(strDescSignalaccelerate);
                cObjectPtr<IMediaType> pTypeSignalaccelerate = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalaccelerate, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
                RETURN_IF_FAILED(pTypeSignalaccelerate->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionOutputAcceleration));
                RETURN_IF_FAILED(m_oOutputAcceleration.Create("Acceleration", pTypeSignalaccelerate, static_cast<IPinEventSink*> (this)));
                RETURN_IF_FAILED(RegisterPin(&m_oOutputAcceleration));

                //create pin for turnSignalLeftEnabled output
                tChar const * strDescSignalTurnSignalLeftEnabled = pDescManager->GetMediaDescription("tBoolSignalValue"); //tBoolSignalValue
                RETURN_IF_POINTER_NULL(strDescSignalTurnSignalLeftEnabled);
                cObjectPtr<IMediaType> pTypeSignalTurnSignalLeftEnabled = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescSignalTurnSignalLeftEnabled, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
                RETURN_IF_FAILED(pTypeSignalTurnSignalLeftEnabled->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionOutputTurnSignalLeftEnabled));
                RETURN_IF_FAILED(m_oOutputTurnSignalLeftEnabled.Create("turnSignalLeftEnabled", pTypeSignalTurnSignalLeftEnabled, static_cast<IPinEventSink*> (this)));
                RETURN_IF_FAILED(RegisterPin(&m_oOutputTurnSignalLeftEnabled));

                //create pin for turnSignalRightEnabled output
                tChar const * strDescSignalTurnSignalRightEnabled = pDescManager->GetMediaDescription("tBoolSignalValue"); //tBoolSignalValue
                RETURN_IF_POINTER_NULL(strDescSignalTurnSignalRightEnabled);
                cObjectPtr<IMediaType> pTypeSignalTurnSignalRightEnabled = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescSignalTurnSignalRightEnabled, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
                RETURN_IF_FAILED(pTypeSignalTurnSignalRightEnabled->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionOutputTurnSignalRightEnabled));
                RETURN_IF_FAILED(m_oOutputTurnSignalRightEnabled.Create("turnSignalRightEnabled", pTypeSignalTurnSignalRightEnabled, static_cast<IPinEventSink*> (this)));
                RETURN_IF_FAILED(RegisterPin(&m_oOutputTurnSignalRightEnabled));

                //create pin for Overtake output
                tChar const * strDescSignalOvertake = pDescManager->GetMediaDescription("tBoolSignalValue"); //tBoolSignalValue
                RETURN_IF_POINTER_NULL(strDescSignalOvertake);
                cObjectPtr<IMediaType> pTypeSignalOvertake = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescSignalOvertake, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
                RETURN_IF_FAILED(pTypeSignalOvertake->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionOvertake));
                RETURN_IF_FAILED(m_oOutputOvertake.Create("Overtake", pTypeSignalOvertake, static_cast<IPinEventSink*> (this)));
                RETURN_IF_FAILED(RegisterPin(&m_oOutputOvertake));
// Input pin for position input ADD IN Scanning
                tChar const * strDescPos = pDescManager->GetMediaDescription("tPosition");
                RETURN_IF_POINTER_NULL(strDescPos);
                cObjectPtr<IMediaType> pTypePos = new cMediaType(0, 0, 0, "tPosition", strDescPos, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
                RETURN_IF_FAILED(pTypePos->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionPos));
                RETURN_IF_FAILED(m_InputPostion.Create("Position", pTypePos, static_cast<IPinEventSink*> (this)));
                RETURN_IF_FAILED(RegisterPin(&m_InputPostion));
// create pin for heading angle
                tChar const * strDescSignalHeadingAngle = pDescManager->GetMediaDescription("tSignalValue");
                RETURN_IF_POINTER_NULL(strDescSignalHeadingAngle);
                cObjectPtr<IMediaType> pTypeSignalHeadingAngle = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalHeadingAngle, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
                RETURN_IF_FAILED(pTypeSignalHeadingAngle->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescHeadingAngle));
                RETURN_IF_FAILED(m_oInputHeadingAngle.Create("Heading Angle", pTypeSignalHeadingAngle, static_cast<IPinEventSink*> (this)));
                RETURN_IF_FAILED(RegisterPin(&m_oInputHeadingAngle));
//output pin for parking space
                tChar const * strDescPosition = pDescManager->GetMediaDescription("tPosition");
                RETURN_IF_POINTER_NULL(strDescPosition);
                cObjectPtr<IMediaType> pTypePosition = new cMediaType(0, 0, 0, "tPosition", strDescPosition, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
                RETURN_IF_FAILED(pTypePosition->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionPosition));
                RETURN_IF_FAILED(m_oPostion.Create("Position TO XML", pTypePosition, static_cast<IPinEventSink*> (this)));
                RETURN_IF_FAILED(RegisterPin(&m_oPostion));
//output pin for parking space update
                tChar const * strDescObstacle = pDescManager->GetMediaDescription("tObstacle");
                RETURN_IF_POINTER_NULL(strDescObstacle);
                cObjectPtr<IMediaType> pTypeObstacle = new cMediaType(0, 0, 0, "tObstacle", strDescObstacle, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
                RETURN_IF_FAILED(pTypeObstacle->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionObstacle));
                RETURN_IF_FAILED(m_InputObstacle.Create("Obstacle", pTypeObstacle, static_cast<IPinEventSink*> (this)));
                RETURN_IF_FAILED(RegisterPin(&m_InputObstacle));


    }
    else if (eStage == StageNormal)
    {
        // In this stage you would do further initialisation and/or create your dynamic pins.
        // Please take a look at the demo_dynamicpin example for further reference.

                m_bDebugModeEnabled = GetPropertyBool("Debug Output to Console");
    }
    else if (eStage == StageGraphReady)
    {
        // init all member variables
                isInitialized_=tFalse;
                init_time = _clock->GetStreamTime();
                m_bStart=tFalse;
                m_szIdsUsStructSet=tFalse;
                m_fCurrentSpeedInput=0;

                m_bFlagTimeOvertake=tFalse;
                m_fDepth_row=0;
                m_fDepth_old1=0;
                m_fDepth_old2=0;
                m_fDepth_Final=0;
                m_szF32X = 0;
                m_szF32Y = 0;
                m_fposx = 0;
                m_fposy = 0;
                Obstaclef32X = 0;
                Obstaclef32Y = 0;
                Obstaclef32XPose = 0;
                Obstaclef32YPose = 0;
                m_fObstacleStatus=0;
                m_fFirstHeadingAngle = 0;
                m_szF32Heading =0;
                m_fAvgSteeringInput =0;
                for(int i=0;i<=10;i++)
                {
                        m_aUSSensors[i]=400;
                         m_depth[i]=-1;
                }

                // init output
                m_bOvertake=tFalse;
                m_fAccelerationOutput=0;
m_bTurnSignalRightEnabled = tFalse;
m_bTurnSignalLeftEnabled =tFalse;
    }

    RETURN_NOERROR;
}

tResult cACC::Shutdown(tInitStage eStage, __exception)
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


tResult cACC::PropertyChanged(const char* strProperty)
{
        ReadProperties(strProperty);
        RETURN_NOERROR;
}

tResult cACC::ReadProperties(const tChar* strPropertyName)
{
        if (NULL == strPropertyName || cString::IsEqual(strPropertyName, DIST_OVERTAKE))
        {
                m_fDistOvertake = static_cast<tFloat32> (GetPropertyFloat(DIST_OVERTAKE));
        }
                if (NULL == strPropertyName || cString::IsEqual(strPropertyName, DIST_REDUCE_SPEED))
        {
                m_fDistReduceSpeed = static_cast<tFloat32> (GetPropertyFloat(DIST_REDUCE_SPEED));
        }
                if (NULL == strPropertyName || cString::IsEqual(strPropertyName, WIGHT_SIDE_FRONT))
        {
                wight_side_front = static_cast<tFloat32> (GetPropertyFloat(WIGHT_SIDE_FRONT));
        }
                if (NULL == strPropertyName || cString::IsEqual(strPropertyName, Critical_steering_angle))
        {
                m_f32CriticalSteeringAngle = static_cast<tFloat32> (GetPropertyFloat(Critical_steering_angle));
        }
        RETURN_NOERROR;
}

tResult cACC::OnPinEvent(IPin* pSource,
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

                // Input signal at Start
                if (pSource == &m_oStart)
                {
                        cObjectPtr<IMediaCoder> pCoderInput;
                        RETURN_IF_FAILED(m_pDescStart->Lock(pMediaSample, &pCoderInput));
                        pCoderInput->Get("bValue", (tVoid*)&m_bStart);
                        m_pDescStart->Unlock(pCoderInput);


                }
                if (pSource == &m_oStart_test)
                        {

                            cObjectPtr<IMediaCoder> pCoderInput;
                            RETURN_IF_FAILED(m_pDescStart_test->Lock(pMediaSample, &pCoderInput));
                            pCoderInput->Get("bValue", (tVoid*)&m_bStart);
                            m_pDescStart_test->Unlock(pCoderInput);

                                        if(!m_bStart)
                                        {
                                            m_fSteeringOutput=0;
                                            m_fAccelerationOutput=0;
                                            TransmitOutput();
                                            m_bOvertake=tFalse;
                                            TransmitOutputOvertake();

                                        }
                                        else
                                        {
                                                        m_fObstacleStatus=0;
                                                        m_fDepth_row=m_fDepthDistance;
                                                        m_fDepth_old1=m_fDepthDistance;
                                                        m_fDepth_old2=m_fDepthDistance;
                                                        m_fDepth_old3=m_fDepthDistance;
                                        }
                m_bTurnSignalRightEnabled = tFalse;
                m_bTurnSignalLeftEnabled =tFalse;
                        }
                if(pSource == &m_oInputUsStruct)
                {
                ProcessInputUS(pMediaSample);
                }
                                if (pSource == &m_oInputHeadingAngle)
                                {
                                        cObjectPtr<IMediaCoder> pCoderInput;
                                        RETURN_IF_FAILED(m_pDescHeadingAngle->Lock(pMediaSample, &pCoderInput));
                                        pCoderInput->Get("f32Value", (tVoid*)&m_fFirstHeadingAngle);
                                        m_pDescHeadingAngle->Unlock(pCoderInput);
                                }
                                if (pSource == &m_InputPostion)
                                {
                                        tTimeStamp tsInputTime;
                                        tsInputTime = pMediaSample->GetTime();
                                        //Process Sample
                                        RETURN_IF_FAILED(ProcessInputPosition(pMediaSample, tsInputTime));
                                }
                if(pSource == &m_oInputSpeedController)
                {
                        cObjectPtr<IMediaCoder> pCoderInput;
                        RETURN_IF_FAILED(m_pDescSpeed->Lock(pMediaSample, &pCoderInput));
                        pCoderInput->Get("f32Value", (tVoid*)&m_fSpeedControllerInput);
                        pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timestamp);
                        m_pDescSpeed->Unlock(pCoderInput);
                }

                if(pSource == &m_oInputdistance)
                {
                        cObjectPtr<IMediaCoder> pCoderInput;
                        RETURN_IF_FAILED(m_pDescDistance->Lock(pMediaSample, &pCoderInput));
                        pCoderInput->Get("f32Value", (tVoid*)&m_fDepthDistance);
                        pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timestamp);
                        m_pDescDistance->Unlock(pCoderInput);

                }
                if(pSource == &m_oInputCurrentSpeed)
                {
                        cObjectPtr<IMediaCoder> pCoderInput;
                        RETURN_IF_FAILED(m_pDescCurrentSpeed->Lock(pMediaSample, &pCoderInput));
                        pCoderInput->Get("f32Value", (tVoid*)&m_fCurrentSpeedInput);
                        pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timestamp);
                        m_pDescCurrentSpeed->Unlock(pCoderInput);
                }
                if (pSource == &m_oInputSteering)
                {
                        cObjectPtr<IMediaCoder> pCoderInput;
                        RETURN_IF_FAILED(m_pDescSteering->Lock(pMediaSample, &pCoderInput));
                        pCoderInput->Get("f32Value", (tVoid*)&m_fSteeringInput);
                        pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timestamp);
                        m_pDescSteering->Unlock(pCoderInput);

                        if (pSource == &m_oInputAvgSteering)
                       {
                               cObjectPtr<IMediaCoder> pCoderInput;
                               RETURN_IF_FAILED(m_pDescriptionInputAvgSteering->Lock(pMediaSample, &pCoderInput));
                               pCoderInput->Get("f32Value", (tVoid*)&m_fAvgSteeringInput);
                               pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timestamp);
                               m_pDescriptionInputAvgSteering->Unlock(pCoderInput);

                       } // stop signal
                        if(!m_bStart)
                        {
                                m_fSteeringOutput=0;
                                m_fAccelerationOutput=0;
                                m_bOvertake=tFalse;

                        }
                        else
                        {
                                m_fSteeringOutput=m_fSteeringInput;
                                Depth_confidance();
                                CalculateSpeed();

                        }

                }

                // only send output ACC is active
                if(m_bStart)
                {
                        TransmitOutput();
                }

    }

    RETURN_NOERROR;
}


tResult cACC::ProcessInputUS(IMediaSample* pMediaSample)
{
        // use mutex to access min US value
        __synchronized_obj(m_critSecMinimumUsValue);

    //read out incoming Media Samples
    __adtf_sample_read_lock_mediadescription(m_pDescriptionUsStruct, pMediaSample, pCoderInput);

    if(!m_szIdsUsStructSet)
    {
        tBufferID idValue, idTimestamp;
        m_szIdUsStructValues.clear();
        m_szIdUsStructTss.clear();

                pCoderInput->GetID("tFrontLeft.f32Value", idValue);
                pCoderInput->GetID("tFrontLeft.ui32ArduinoTimestamp", idTimestamp);
                m_szIdUsStructValues.push_back(idValue);
                m_szIdUsStructTss.push_back(idTimestamp);

                pCoderInput->GetID("tFrontCenterLeft.f32Value", idValue);
                pCoderInput->GetID("tFrontCenterLeft.ui32ArduinoTimestamp", idTimestamp);
                m_szIdUsStructValues.push_back(idValue);
                m_szIdUsStructTss.push_back(idTimestamp);

                pCoderInput->GetID("tFrontCenter.f32Value", idValue);
                pCoderInput->GetID("tFrontCenter.ui32ArduinoTimestamp", idTimestamp);
                m_szIdUsStructValues.push_back(idValue);
                m_szIdUsStructTss.push_back(idTimestamp);

        pCoderInput->GetID("tFrontCenterRight.f32Value", idValue);
        pCoderInput->GetID("tFrontCenterRight.ui32ArduinoTimestamp", idTimestamp);
        m_szIdUsStructValues.push_back(idValue);
        m_szIdUsStructTss.push_back(idTimestamp);

        pCoderInput->GetID("tFrontRight.f32Value", idValue);
        pCoderInput->GetID("tFrontRight.ui32ArduinoTimestamp", idTimestamp);
        m_szIdUsStructValues.push_back(idValue);
        m_szIdUsStructTss.push_back(idTimestamp);

        pCoderInput->GetID("tSideLeft.f32Value", idValue);
        pCoderInput->GetID("tSideLeft.ui32ArduinoTimestamp", idTimestamp);
        m_szIdUsStructValues.push_back(idValue);
        m_szIdUsStructTss.push_back(idTimestamp);

        pCoderInput->GetID("tSideRight.f32Value", idValue);
        pCoderInput->GetID("tSideRight.ui32ArduinoTimestamp", idTimestamp);
        m_szIdUsStructValues.push_back(idValue);
        m_szIdUsStructTss.push_back(idTimestamp);

        pCoderInput->GetID("tRearLeft.f32Value", idValue);
        pCoderInput->GetID("tRearLeft.ui32ArduinoTimestamp", idTimestamp);
        m_szIdUsStructValues.push_back(idValue);
        m_szIdUsStructTss.push_back(idTimestamp);

        pCoderInput->GetID("tRearCenter.f32Value", idValue);
        pCoderInput->GetID("tRearCenter.ui32ArduinoTimestamp", idTimestamp);
        m_szIdUsStructValues.push_back(idValue);
        m_szIdUsStructTss.push_back(idTimestamp);

        pCoderInput->GetID("tRearRight.f32Value", idValue);
        pCoderInput->GetID("tRearRight.ui32ArduinoTimestamp", idTimestamp);
        m_szIdUsStructValues.push_back(idValue);
        m_szIdUsStructTss.push_back(idTimestamp);

                // m_szIdsUsStructSet = tTrue;
                m_szIdsUsStructSet = tTrue;

        }

        // iterate through all values and save them in m_aUSSensors
        for (int i=0;i<10;++i)
        {
                tFloat32 buffer=-1;
                pCoderInput->Get(m_szIdUsStructValues[i],(tVoid*)&buffer);
                // Wrong Us-values are  0 or -1
                if (buffer>0)
                {
                        pCoderInput->Get(m_szIdUsStructValues[i],(tVoid*)&buffer);
                        m_aUSSensors[i]=buffer;
                        //LOG_INFO(cString::Format("front left %f", buffer));
                }
        }

        //LOG_INFO(cString::Format("front left %f, frontcenterleft %f", m_oUSFrontLeft, m_oUSFrontCenterLeft));
        //LOG_INFO(cString::Format("front right %f, frontcenterright %f", m_oUSFrontRight, m_oUSFrontCenterRight));
        //LOG_INFO(cString::Format("ARRAY center %f   frontcenterleft %f  frontcenterright %f", m_aUSSensors[US_FRONTCENTER], m_aUSSensors[US_FRONTCENTERLEFT], m_aUSSensors[US_FRONTCENTERRIGHT]));

        RETURN_NOERROR;
}


tResult cACC::CalculateSpeed()
{
        // use mutex to access
        __synchronized_obj(m_critSecMinimumUsValue);

        tFloat32 fAverageDist;
                // Calculate min-Distance depending on Steering angle

        // TODO Testing
        // Steering left
        if(m_fAvgSteeringInput < -m_f32CriticalSteeringAngle){
         //   if(m_bDebugModeEnabled) LOG_INFO(adtf_util::cString::Format("AEB: Steering LEFT"));
        fAverageDist=min(m_aUSSensors[US_FRONTCENTERLEFT],m_aUSSensors[US_FRONTCENTER]);
        }
        // Steering straight
        else if((m_fAvgSteeringInput > -m_f32CriticalSteeringAngle) && (m_fAvgSteeringInput < m_f32CriticalSteeringAngle)){
          //  if(m_bDebugModeEnabled) LOG_INFO(adtf_util::cString::Format("AEB: Steering STRAIGHT"));
        // min dist of three US sensors
        //tFloat32 fMinSideDist=min(m_aUSSensors[US_FRONTCENTERLEFT],m_aUSSensors[US_FRONTCENTERRIGHT]);
       // fAverageDist=min(fMinSideDist, m_aUSSensors[US_FRONTCENTER]);
        fAverageDist=m_aUSSensors[US_FRONTCENTER];
        }
        // Steering right
        else if(m_fAvgSteeringInput > m_f32CriticalSteeringAngle){
           // if(m_bDebugModeEnabled) LOG_INFO(adtf_util::cString::Format("AEB: Steering RIGHT"));
        fAverageDist=min(m_aUSSensors[US_FRONTCENTERRIGHT],m_aUSSensors[US_FRONTCENTER]);
        }


        // fAverageDist from [cm] to [m]
        fAverageDist=fAverageDist/100;



      if(fAverageDist <= m_fDistOvertake /*|| ((m_fDepth_Final/10)<=1 && (m_fDepth_Final/10)>0.7)*/) //In stop region
                {
                       // LOG_INFO(cString::Format("in stop m_fDepth_Final= %f fAverageDist=%f", m_fDepth_Final,fAverageDist));
                        m_fAccelerationOutput=0;
                        m_fSteeringOutput=0;
                        TransmitOutput();

                        // save time stamp for overtaking
                        if(!m_bFlagTimeOvertake)
                        {
                                timestampOvertake=_clock->GetStreamTime();
                                m_bFlagTimeOvertake=tTrue;
                        }
                        else
                        {
                                // if speed is too slow for a too long time --> wish to overtake
                                if(((_clock->GetStreamTime()-timestampOvertake) > 5e6) && m_fCurrentSpeedInput<=0.1)
                                {
                                        m_bStart=tFalse;
                                        m_bFlagTimeOvertake=tFalse;
                                        m_bOvertake=tTrue;
                                        m_fObstacleStatus=1;

                                        m_fposx = min(fAverageDist,m_fDepth_Final/10);
                                        m_fposy = 0;
                                        computepose();
                                        Obstaclef32X = m_szF32X + Obstaclef32XPose;
                                        Obstaclef32Y = m_szF32Y + Obstaclef32YPose;
                                        SendObstacleData_to_XML();
                                        SendObstacleData();
                                        timestampOvertake = _clock->GetStreamTime();

                                        TransmitOutput();
                                        TransmitOutputOvertake();
                                      //  LOG_INFO(cString::Format("Overtake by Ultrasonic Dteecttion  %f   by Depth  %f   threshold for ultra 0.6  & for Depth 0.7", fAverageDist, m_fDepth_Final/10));

                                }
                                else
                                {

                                }
                        }
                }

                else if(((m_fDepth_Final/10)<1.5 && (m_fDepth_Final/10)>1 )|| fAverageDist < m_fDistReduceSpeed) //in acc region
                {
                    //    LOG_INFO(cString::Format("in ACC m_fDepth_Final= %f fAverageDist=%f", m_fDepth_Final,fAverageDist));
                        // reduce speed linear
                        if((m_fDepth_Final/10)<=0.7)fAverageDist=fAverageDist;
                        else fAverageDist=min(fAverageDist,(m_fDepth_Final/10));
                        m_fAccelerationOutput=(fAverageDist/(m_fDistReduceSpeed+0.2)) * m_fSpeedControllerInput;
                        m_fSteeringOutput=m_fSteeringInput;
                        m_bFlagTimeOvertake = tFalse;
                }
                
                else
                {
                        m_fSteeringOutput=m_fSteeringInput;
                        m_fAccelerationOutput=m_fSpeedControllerInput;
                        m_bFlagTimeOvertake = tFalse;
                }

                //LOG_INFO(cString::Format("ACC dist %f, controller %f, speed_out %f,  property_stop %f    property_reduce %f", fAverageDist, m_fSpeedControllerInput, m_fAccelerationOutput, m_fDistOvertake, m_fDistReduceSpeed));

                // output
                TransmitOutput();


        RETURN_NOERROR;
}

//for map
tResult cACC::ProcessInputPosition(IMediaSample* pMediaSampleIn, tTimeStamp tsInputTime)
{

        cObjectPtr<IMediaCoder> pCoderInput;
        RETURN_IF_FAILED(m_pDescriptionPos->Lock(pMediaSampleIn, &pCoderInput));
        pCoderInput->Get("f32x", (tVoid*)&m_szF32X);
        pCoderInput->Get("f32y", (tVoid*)&m_szF32Y);
        pCoderInput->Get("f32radius", (tVoid*)&m_szF32Radius);
        pCoderInput->Get("f32speed", (tVoid*)&m_szF32Speed);
        pCoderInput->Get("f32heading", (tVoid*)&m_szF32Heading);
        m_pDescriptionPos->Unlock(pCoderInput);

        RETURN_NOERROR;
}

//computing the variables
tResult cACC::computepose()
{

        tFloat32 m_angle_change = m_fFirstHeadingAngle - m_szF32Heading;
        Obstaclef32XPose = m_fposx*std::cos(m_angle_change) - m_fposy*std::sin(m_angle_change);    //
        Obstaclef32YPose = m_fposy*std::cos(m_angle_change) + m_fposx*std::sin(m_angle_change);    //

        //if(m_bDebugModeEnabled)LOG_INFO(cString::Format("Park X %f Park Y %f Angle change %f", m_parkingF32X, m_parkingF32Y, m_angle_change));
        RETURN_NOERROR;
}

tResult cACC::Depth_confidance()
{
        if(m_fDepthDistance<0)m_fDepthDistance=0;
        m_fDepth_old3=m_fDepth_old3;
        m_fDepth_old2=m_fDepth_old1;
        m_fDepth_old1=m_fDepth_row;
        m_fDepth_row=m_fDepthDistance;
        if(m_fDepth_old1<=m_fDepth_old2 && m_fDepth_row <= m_fDepth_old1) // if depth is continuesly decresing
                {
                        m_fDepth_Final=m_fDepth_old1;
                }
        else if(m_fDepth_old1>=m_fDepth_old2 && m_fDepth_row >= m_fDepth_old1) //if depth is continuesly incresing
                {
                        m_fDepth_Final=m_fDepth_old1;
                }
        else if ((m_fDepth_old1<m_fDepth_old2 && m_fDepth_old1<m_fDepth_row && (m_fDepth_old2<m_fDepth_old3)) ||(m_fDepth_old1 >m_fDepth_old2 && m_fDepth_old1>m_fDepth_row && (m_fDepth_old2<m_fDepth_old3))) //if false detection between depth contineusly decresing
                {
                        m_fDepth_old1=(m_fDepth_row+m_fDepth_old2+m_fDepth_old3)/3; //check history of deecrising depth
                        m_fDepth_Final=m_fDepth_old1;

                }
        else if ((m_fDepth_old1>m_fDepth_old2 && m_fDepth_old1<m_fDepth_row && (m_fDepth_old2>m_fDepth_old3)) ||(m_fDepth_old1<m_fDepth_old2 && m_fDepth_old1<m_fDepth_row&& (m_fDepth_old2>m_fDepth_old3)) ) //if false detection between depth contineusly incrising
                {
                        m_fDepth_old1=(m_fDepth_row+m_fDepth_old2+m_fDepth_old3)/3; //check history of deecrising depth
                        m_fDepth_Final=m_fDepth_old1;
                }
        else
                {
                        m_fDepth_Final=m_fDepth_old1;
                }
                //LOG_INFO(adtf_util::cString::Format("Final depth =  %f",m_fDepth_Final));
        RETURN_NOERROR;
}
tResult cACC::TransmitOutput()
{


      TransmitFloatValue(&m_oOutputSteering, m_fSteeringOutput, 0);
      TransmitFloatValue(&m_oOutputAcceleration, m_fAccelerationOutput, 0);
      TransmitBoolValue(&m_oOutputTurnSignalRightEnabled, m_bTurnSignalRightEnabled, 0);
      TransmitBoolValue(&m_oOutputTurnSignalLeftEnabled, m_bTurnSignalLeftEnabled, 0);

        RETURN_NOERROR;
}


tResult cACC::TransmitOutputOvertake()
{

        TransmitBoolValue(&m_oOutputOvertake, m_bOvertake, 0);
        RETURN_NOERROR;
}
tResult cACC::TransmitFloatValue(cOutputPin* oPin, tFloat32 value, tUInt32 timestamp)
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
tResult cACC::TransmitBoolValue(cOutputPin* oPin, bool value, tUInt32 timestamp)
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
tResult cACC::SendObstacleData() // add this in Scanning Code

{
        cObjectPtr<IMediaSample> pMediaSampleObstacle;
        AllocMediaSample((tVoid**)&pMediaSampleObstacle);
        // Parking space data
        cObjectPtr<IMediaSerializer> pSerializerObstacle;
        m_pDescriptionObstacle->GetMediaSampleSerializer(&pSerializerObstacle);
        tInt nSizesteer = pSerializerObstacle->GetDeserializedSize();
        pMediaSampleObstacle->AllocBuffer(nSizesteer);
        cObjectPtr<IMediaCoder> pCoderOutputObstacle;
        m_pDescriptionObstacle->WriteLock(pMediaSampleObstacle, &pCoderOutputObstacle);
        pCoderOutputObstacle->Set("f32x", (tVoid*)&(Obstaclef32X));
        pCoderOutputObstacle->Set("f32y", (tVoid*)&(Obstaclef32Y));
        m_pDescriptionObstacle->Unlock(pCoderOutputObstacle);
        pMediaSampleObstacle->SetTime(_clock->GetStreamTime());
        m_InputObstacle.Transmit(pMediaSampleObstacle);

        RETURN_NOERROR;
}
tResult cACC::SendObstacleData_to_XML() // add this in Scanning Code
{
        cObjectPtr<IMediaSample> pMediaSampleObstacle_XML;
        AllocMediaSample((tVoid**)&pMediaSampleObstacle_XML);
        // Parking space data
        cObjectPtr<IMediaSerializer> pSerializerObstacle_XML;
        m_pDescriptionPosition->GetMediaSampleSerializer(&pSerializerObstacle_XML);
        tInt nSizesteer = pSerializerObstacle_XML->GetDeserializedSize();
        pMediaSampleObstacle_XML->AllocBuffer(nSizesteer);
        cObjectPtr<IMediaCoder> pCoderOutputObstacle_XML;
        tFloat32 dummy=0;
        m_pDescriptionPosition->WriteLock(pMediaSampleObstacle_XML, &pCoderOutputObstacle_XML);
        pCoderOutputObstacle_XML->Set("f32radius", (tVoid*)&(dummy));
        pCoderOutputObstacle_XML->Set("f32x", (tVoid*)&(Obstaclef32X));
        pCoderOutputObstacle_XML->Set("f32y", (tVoid*)&(Obstaclef32Y));
        pCoderOutputObstacle_XML->Set("f32speed", (tVoid*)&(m_fObstacleStatus));
        pCoderOutputObstacle_XML->Set("f32heading", (tVoid*)&(m_szF32Heading));
        m_pDescriptionPosition->Unlock(pCoderOutputObstacle_XML);
        pMediaSampleObstacle_XML->SetTime(_clock->GetStreamTime());
        m_oPostion.Transmit(pMediaSampleObstacle_XML);

        RETURN_NOERROR;
}
