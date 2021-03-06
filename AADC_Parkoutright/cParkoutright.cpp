/**
Copyright (c)
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: �This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.�
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS �AS IS� AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


**********************************************************************
* $Author:: spiesra $  $Date:: 2017-05-12 09:34:53#$ $Rev:: 63109   $
**********************************************************************/

#include "stdafx.h"
#include "cParkoutright.h"

#define DISTANCE_PARKOUT_RIGHT 			"cParkoutright::DIST_PARKOUTRIGHT"
#define SPEED_PARKOUT_RIGHT 			"cParkoutright::SPEED_PARKOUTRIGHT"

#define DISTANCE_RIGHT_turn			"cParkoutright::DIST_RIGHTturn"
#define STEER_RIGHT_turn        		"cParkoutright::STEER_RIGHTturn"
#define SPEED_RIGHT_turn        		"cParkoutright::SPEED_RIGHTturn"
#define SPEED_RIGHT_turn_back        		"cParkoutright::SPEED_RIGHTturn_back"
#define DELTA_YAW				"cParkoutright::YAW_Diff"
#define KPSTEERING				"cParkoutright::kpsteering"
/// Create filter shell
ADTF_FILTER_PLUGIN("Parkoutright", OID_ADTF_PARKOUTRIGHT, cParkoutright);

using namespace roadsignIDs;

cParkoutright::cParkoutright(const tChar* __info):cFilter(__info)
{
// set properties for Dibug
    // m_bDebugModeEnabled = tTrue;
    SetPropertyBool("Debug Output to Console",false);
// set properties for STRIGHT front
        SetPropertyFloat(DISTANCE_PARKOUT_RIGHT,0.05);
    SetPropertyBool(DISTANCE_PARKOUT_RIGHT NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(DISTANCE_PARKOUT_RIGHT NSSUBPROP_DESCRIPTION, "the distance for the PARKOUT_RIGHT");

        SetPropertyFloat(SPEED_PARKOUT_RIGHT,0.5);
    SetPropertyBool(SPEED_PARKOUT_RIGHT NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(SPEED_PARKOUT_RIGHT NSSUBPROP_DESCRIPTION, "the speed for the PARKOUT_RIGHT");

// set properties for RIGHT TURN	
        SetPropertyFloat(DISTANCE_RIGHT_turn,1.5);
    SetPropertyBool(DISTANCE_RIGHT_turn NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(DISTANCE_RIGHT_turn NSSUBPROP_DESCRIPTION, "the distance for the steer RIGHT_turn");

        SetPropertyFloat(STEER_RIGHT_turn,90);
    SetPropertyBool(STEER_RIGHT_turn NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(STEER_RIGHT_turn NSSUBPROP_DESCRIPTION, "the steer for RIGHT_turn");

        SetPropertyFloat(SPEED_RIGHT_turn,0.5);
    SetPropertyBool(SPEED_RIGHT_turn NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(SPEED_RIGHT_turn NSSUBPROP_DESCRIPTION, "the speed for the steer RIGHT_turn");
    SetPropertyFloat(SPEED_RIGHT_turn_back,-0.3);
SetPropertyBool(SPEED_RIGHT_turn_back NSSUBPROP_ISCHANGEABLE,tTrue);
SetPropertyStr(SPEED_RIGHT_turn_back NSSUBPROP_DESCRIPTION, "the speed for the steer RIGHT_turn_back");

	SetPropertyFloat(DELTA_YAW,12);
    SetPropertyBool(DELTA_YAW NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(DELTA_YAW NSSUBPROP_DESCRIPTION, "yaw diffrence for turn");

    SetPropertyFloat(KPSTEERING,20);
SetPropertyBool(KPSTEERING NSSUBPROP_ISCHANGEABLE,tTrue);
SetPropertyStr(KPSTEERING NSSUBPROP_DESCRIPTION, "Kp value for steering control until stop line");

}

cParkoutright::~cParkoutright()
{

}

tResult cParkoutright::Init(tInitStage eStage, __exception)
{
    // never miss calling the parent implementation!!
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))

    // in StageFirst you can create and register your static pins.
    if (eStage == StageFirst)
    {
        // get a media type for the input pin
        //cObjectPtr<IMediaType> pInputType;
        //RETURN_IF_FAILED(AllocMediaType(&pInputType, MEDIA_TYPE_TEMPLATE, MEDIA_SUBTYPE_TEMPLATE, __exception_ptr));

        // create and register the input pin
        //RETURN_IF_FAILED(m_oInputPin.Create("input_template", pInputType, this));
        //RETURN_IF_FAILED(RegisterPin(&m_oInputPin));

        // get a media type for the output pin
        //cObjectPtr<IMediaType> pOutputType;
        //RETURN_IF_FAILED(AllocMediaType(&pOutputType, MEDIA_TYPE_TEMPLATE, MEDIA_SUBTYPE_TEMPLATE, __exception_ptr));

        // create and register the output pin
        //RETURN_IF_FAILED(m_oOutputPin.Create("output_template", pOutputType, this));
        //RETURN_IF_FAILED(RegisterPin(&m_oOutputPin));
        
	//get description for wheel sensors data pins
//        tChar const * strDescWheelData = pDescManager->GetMediaDescription("tWheelData");
//        RETURN_IF_POINTER_NULL(strDescWheelData);
  //      cObjectPtr<IMediaDescriptionManager> pDescManager;
    cObjectPtr<IMediaDescriptionManager> pDescManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));
    tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");
    RETURN_IF_POINTER_NULL(strDescSignalValue);
    cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    tChar const * strDescBoolSignalValue = pDescManager->GetMediaDescription("tBoolSignalValue");
    RETURN_IF_POINTER_NULL(strDescSignalValue);
    cObjectPtr<IMediaType> pTypeBoolSignalValue = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescBoolSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);


    RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionFloat));
    RETURN_IF_FAILED(pTypeBoolSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionBool));

        tChar const * strRoadSign = pDescManager->GetMediaDescription("tRoadSign");
        RETURN_IF_POINTER_NULL(strRoadSign);
        cObjectPtr<IMediaType> pTypeRoadSign = new cMediaType(0, 0, 0, "tRoadSign", strRoadSign, IMediaDescription::MDF_DDL_DEFAULT_VERSION);



//create pin for start pin		
				tChar const * strDescSignalstart = pDescManager->GetMediaDescription("tBoolSignalValue");
				RETURN_IF_POINTER_NULL(strDescSignalstart);
				cObjectPtr<IMediaType> pTypeSignalstart = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescSignalstart, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
				RETURN_IF_FAILED(pTypeSignalstart->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescStart));
				RETURN_IF_FAILED(m_oStart.Create("Start", pTypeSignalstart, static_cast<IPinEventSink*> (this)));
				RETURN_IF_FAILED(RegisterPin(&m_oStart));

//create pin for Obstacle pin		
				tChar const * strDescSignalObstacle = pDescManager->GetMediaDescription("tCheckTrafficForCrossing");
				RETURN_IF_POINTER_NULL(strDescSignalObstacle);
				cObjectPtr<IMediaType> pTypeSignalObstacle = new cMediaType(0, 0, 0, "tCheckTrafficForCrossing", strDescSignalObstacle, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
				RETURN_IF_FAILED(pTypeSignalObstacle->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescObstacle));
                                RETURN_IF_FAILED(m_oObstacle.Create("Obstacle", pTypeSignalObstacle, static_cast<IPinEventSink*> (this)));
				RETURN_IF_FAILED(RegisterPin(&m_oObstacle));
 //create pin for Steering Input of LaneFollower
                tChar const * strDescSignalSteeringOfLaneFollower = pDescManager->GetMediaDescription("tSignalValue");
                RETURN_IF_POINTER_NULL(strDescSignalSteeringOfLaneFollower);
                cObjectPtr<IMediaType> pTypeSignalSteeringOfLaneFollower = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalSteeringOfLaneFollower, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
                RETURN_IF_FAILED(pTypeSignalSteeringOfLaneFollower->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescSteeringOfLaneFollower));
                RETURN_IF_FAILED(m_oSteeringOfLaneFollower.Create("Steering Input", pTypeSignalSteeringOfLaneFollower, static_cast<IPinEventSink*> (this)));
                RETURN_IF_FAILED(RegisterPin(&m_oSteeringOfLaneFollower));
// Lane_Information
                tChar const * strDescSignallanechange = pDescManager->GetMediaDescription("tCheckTrafficForCrossing");
                RETURN_IF_POINTER_NULL(strDescSignallanechange);
                cObjectPtr<IMediaType> pTypeSignallanechange = new cMediaType(0, 0, 0, "tCheckTrafficForCrossing", strDescSignallanechange, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
                RETURN_IF_FAILED(pTypeSignallanechange->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDesclanechange));
                RETURN_IF_FAILED(m_olanechange.Create("Lane_Information", pTypeSignallanechange, static_cast<IPinEventSink*> (this)));
                RETURN_IF_FAILED(RegisterPin(&m_olanechange));
//create pin for Yaw input
                tChar const * strDescSignalYaw = pDescManager->GetMediaDescription("tSignalValue");
                RETURN_IF_POINTER_NULL(strDescSignalYaw);
                cObjectPtr<IMediaType> pTypeSignalYaw = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalYaw, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
                RETURN_IF_FAILED(pTypeSignalYaw->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescYaw));
                RETURN_IF_FAILED(m_oYaw.Create("Yaw", pTypeSignalYaw, static_cast<IPinEventSink*> (this)));
                RETURN_IF_FAILED(RegisterPin(&m_oYaw));
//create pin for US Rightside input
                tChar const * strDescSignalUSRightside = pDescManager->GetMediaDescription("tSignalValue");
                RETURN_IF_POINTER_NULL(strDescSignalUSRightside);
                cObjectPtr<IMediaType> pTypeSignalUSRightside = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalUSRightside, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
                RETURN_IF_FAILED(pTypeSignalUSRightside->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescUSRightside));
                RETURN_IF_FAILED(m_oUSRightside.Create("US_Rightside", pTypeSignalUSRightside, static_cast<IPinEventSink*> (this)));
                RETURN_IF_FAILED(RegisterPin(&m_oUSRightside));


//create pin for Distance over all input
		tChar const * strDescSignaldistanceoverall = pDescManager->GetMediaDescription("tSignalValue");
		RETURN_IF_POINTER_NULL(strDescSignaldistanceoverall);
		cObjectPtr<IMediaType> pTypeSignaldistanceoverall = new cMediaType(0, 0, 0, "tSignalValue", strDescSignaldistanceoverall, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
		RETURN_IF_FAILED(pTypeSignaldistanceoverall->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescdistanceoverall));
		RETURN_IF_FAILED(m_oDistanceOverall.Create("Distance_Overall", pTypeSignaldistanceoverall, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oDistanceOverall));

// create pin for Infos about Stop Line
		tChar const * strDescStopLine = pDescManager->GetMediaDescription("tStoplineStruct");
		RETURN_IF_POINTER_NULL(strDescStopLine);
		cObjectPtr<IMediaType> pTypeSignalStopLine = new cMediaType(0, 0, 0, "tStoplineStruct", strDescStopLine, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
		RETURN_IF_FAILED(pTypeSignalStopLine->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescStopLine));
		RETURN_IF_FAILED(m_oStopLine.Create("StopLine", pTypeSignalStopLine, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oStopLine));

// Output - Parkoutright_Finish
                tChar const * strDescSignalParkoutright_Finish = pDescManager->GetMediaDescription("tBoolSignalValue"); //tBoolSignalValue
                RETURN_IF_POINTER_NULL(strDescSignalParkoutright_Finish);
                cObjectPtr<IMediaType> pTypeSignalParkoutright_Finish = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescSignalParkoutright_Finish, 	IMediaDescription::MDF_DDL_DEFAULT_VERSION);

                RETURN_IF_FAILED(pTypeSignalParkoutright_Finish->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionOutputParkoutright_Finish));
                RETURN_IF_FAILED(m_oOutputParkoutright_Finish.Create("Parkoutright Finish", pTypeSignalParkoutright_Finish, static_cast<IPinEventSink*> (this)));
                RETURN_IF_FAILED(RegisterPin(&m_oOutputParkoutright_Finish));

// Output - Lanefollow_Start_test
                tChar const * strDescSignalLanefollow_Start_test = pDescManager->GetMediaDescription("tBoolSignalValue"); //tBoolSignalValue
                RETURN_IF_POINTER_NULL(strDescSignalLanefollow_Start_test);
                cObjectPtr<IMediaType> pTypeSignalLanefollow_Start_test = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescSignalLanefollow_Start_test, 	IMediaDescription::MDF_DDL_DEFAULT_VERSION);
                RETURN_IF_FAILED(pTypeSignalLanefollow_Start_test->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionOutputLanefollow_Start_test));
                RETURN_IF_FAILED(m_oOutputLanefollow_Start_test.Create("Lanefollow Start test", pTypeSignalLanefollow_Start_test, static_cast<IPinEventSink*> (this)));
                RETURN_IF_FAILED(RegisterPin(&m_oOutputLanefollow_Start_test));

//create pin for steering signal output
                tChar const * strDescSignalSteering = pDescManager->GetMediaDescription("tSignalValue");
                RETURN_IF_POINTER_NULL(strDescSignalSteering);
                cObjectPtr<IMediaType> pTypeSignalSteering = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalSteering, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
                RETURN_IF_FAILED(pTypeSignalSteering->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionOutputSteering));
                RETURN_IF_FAILED(m_oOutputSteering.Create("Steering", pTypeSignalSteering, static_cast<IPinEventSink*> (this)));
                RETURN_IF_FAILED(RegisterPin(&m_oOutputSteering));

//create pin for steering signal output
		tChar const * strDescSignalaccelerate = pDescManager->GetMediaDescription("tSignalValue");
		RETURN_IF_POINTER_NULL(strDescSignalaccelerate);
		cObjectPtr<IMediaType> pTypeSignalaccelerate = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalaccelerate, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
		RETURN_IF_FAILED(pTypeSignalaccelerate->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionOutputAcceleration));
		RETURN_IF_FAILED(m_oOutputAcceleration.Create("accelerate", pTypeSignalaccelerate, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oOutputAcceleration));

// Output - Hazard_Light
                tChar const * strDescSignalHazard_Light = pDescManager->GetMediaDescription("tBoolSignalValue"); //tBoolSignalValue
                RETURN_IF_POINTER_NULL(strDescSignalHazard_Light);
                cObjectPtr<IMediaType> pTypeSignalHazard_Light = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescSignalHazard_Light, 	IMediaDescription::MDF_DDL_DEFAULT_VERSION);
                RETURN_IF_FAILED(pTypeSignalHazard_Light->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionOutputHazard_Light));
                RETURN_IF_FAILED(m_oOutputHazard_Light.Create("Hazard Light", pTypeSignalHazard_Light, static_cast<IPinEventSink*> (this)));
                RETURN_IF_FAILED(RegisterPin(&m_oOutputHazard_Light));

// Output - Back_Light
                tChar const * strDescSignalBack_Light = pDescManager->GetMediaDescription("tBoolSignalValue"); //tBoolSignalValue
                RETURN_IF_POINTER_NULL(strDescSignalBack_Light);
                cObjectPtr<IMediaType> pTypeSignalBack_Light = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescSignalBack_Light, 	IMediaDescription::MDF_DDL_DEFAULT_VERSION);
                RETURN_IF_FAILED(pTypeSignalBack_Light->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionOutputBack_Light));
                RETURN_IF_FAILED(m_oOutputBack_Light.Create("Back Light", pTypeSignalBack_Light, static_cast<IPinEventSink*> (this)));
                RETURN_IF_FAILED(RegisterPin(&m_oOutputBack_Light));

// Output - Head_Light
                tChar const * strDescSignalHead_Light = pDescManager->GetMediaDescription("tBoolSignalValue"); //tBoolSignalValue
                RETURN_IF_POINTER_NULL(strDescSignalHead_Light);
                cObjectPtr<IMediaType> pTypeSignalHead_Light = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescSignalHead_Light, 	IMediaDescription::MDF_DDL_DEFAULT_VERSION);
                RETURN_IF_FAILED(pTypeSignalHead_Light->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionOutputHead_Light));
                RETURN_IF_FAILED(m_oOutputHead_Light.Create("Head Light", pTypeSignalHead_Light, static_cast<IPinEventSink*> (this)));
                RETURN_IF_FAILED(RegisterPin(&m_oOutputHead_Light));

// Output - TurnLeft_Light
                tChar const * strDescSignalTurnLeft_Light = pDescManager->GetMediaDescription("tBoolSignalValue"); //tBoolSignalValue
                RETURN_IF_POINTER_NULL(strDescSignalTurnLeft_Light);
                cObjectPtr<IMediaType> pTypeSignalTurnLeft_Light = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescSignalTurnLeft_Light, 	IMediaDescription::MDF_DDL_DEFAULT_VERSION);
                RETURN_IF_FAILED(pTypeSignalTurnLeft_Light->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionOutputTurnLeft_Light));
                RETURN_IF_FAILED(m_oOutputTurnLeft_Light.Create("TurnLeft Light", pTypeSignalTurnLeft_Light, static_cast<IPinEventSink*> (this)));
                RETURN_IF_FAILED(RegisterPin(&m_oOutputTurnLeft_Light));

// Output - TurnRight_Light
                tChar const * strDescSignalTurnRight_Light = pDescManager->GetMediaDescription("tBoolSignalValue"); //tBoolSignalValue
                RETURN_IF_POINTER_NULL(strDescSignalTurnRight_Light);
                cObjectPtr<IMediaType> pTypeSignalTurnRight_Light = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescSignalTurnRight_Light, 	IMediaDescription::MDF_DDL_DEFAULT_VERSION);
                RETURN_IF_FAILED(pTypeSignalTurnRight_Light->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionOutputTurnRight_Light));
                RETURN_IF_FAILED(m_oOutputTurnRight_Light.Create("TurnRight Light", pTypeSignalTurnRight_Light, static_cast<IPinEventSink*> (this)));
                RETURN_IF_FAILED(RegisterPin(&m_oOutputTurnRight_Light));

// Output - Break_Light
                tChar const * strDescSignalBreak_Light = pDescManager->GetMediaDescription("tBoolSignalValue"); //tBoolSignalValue
                RETURN_IF_POINTER_NULL(strDescSignalBreak_Light);
                cObjectPtr<IMediaType> pTypeSignalBreak_Light = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescSignalBreak_Light, 	IMediaDescription::MDF_DDL_DEFAULT_VERSION);
                RETURN_IF_FAILED(pTypeSignalBreak_Light->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionOutputBreak_Light));
                RETURN_IF_FAILED(m_oOutputBreak_Light.Create("Break Light", pTypeSignalBreak_Light, static_cast<IPinEventSink*> (this)));
                RETURN_IF_FAILED(RegisterPin(&m_oOutputBreak_Light));

    }
    else if (eStage == StageNormal)
    {
        // In this stage you would do further initialisation and/or create your dynamic pins.
        // Please take a look at the demo_dynamicpin example for further reference.
        m_bDebugModeEnabled = GetPropertyBool("Debug Output to Console");
    }
    else if (eStage == StageGraphReady)
    {
        // All pin connections have been established in this stage so you can query your pins
		m_fAccelerationOutput=0;
		m_fSteeringOutput=0;
		m_bStart=tFalse;
		m_iStateOfParkoutright=SOP_NOSTART;
                m_bDebugModeEnabled = GetPropertyBool("Debug Output to Console");
		// init process values
		m_bTransmitstop=tFalse;
                m_fYaw_Start=0;
                m_fYaw=0;
                m_bHazard_Light= tFalse;
                m_bBack_Light= tFalse;
                m_bHead_Light= tFalse;
                m_bTurnLeft_Light= tFalse;
                m_bTurnRight_Light= tFalse;
                m_bBreak_Light= tFalse;
		m_bFinished=tFalse;
		m_bTurnSignalLeftEnabled=tFalse;
		m_bTurnSignalRightEnabled=tFalse;	
		m_bLine_detection = tFalse;
		m_bObstacle = tTrue;
		m_fLine_distance = 0;
		m_fcover_dist = 0;
        // about their media types and additional meta data.
        // Please take a look at the demo_imageproc example for further reference.
    }

    RETURN_NOERROR;
}

tResult cParkoutright::Shutdown(tInitStage eStage, __exception)
{
    // In each stage clean up everything that you initiaized in the corresponding stage during Init.
    // Pins are an exception:
    // - The base class takes care of static pins that are members of this class.
    // - Dynamic pins have to be cleaned up in the ReleasePins method, please see the demo_dynamicpin
    //   example for further reference.

    if (eStage == StageGraphReady)
    {

// initial values for steering and acceleration
		m_fAccelerationOutput=0;
		m_fSteeringOutput=0;
		m_bStart=tFalse;
		m_iStateOfParkoutright=SOP_NOSTART;
		m_bTransmitstop=tFalse;
		m_bHazard_Light= tFalse;
        m_bBack_Light= tFalse;
        m_bHead_Light= tFalse;
        m_bTurnLeft_Light= tFalse;
        m_bTurnRight_Light= tFalse;
        m_bBreak_Light= tFalse;
		// init process values
		m_bFinished=tFalse;
		m_bTurnSignalLeftEnabled=tFalse;
		m_bTurnSignalRightEnabled=tFalse;
		m_bLine_detection = tFalse;
		m_fLine_distance = 0;
		m_fcover_dist = 0; 
		m_bObstacle = tTrue;
    }
    else if (eStage == StageNormal)
    {
                m_bDebugModeEnabled = GetPropertyBool("Debug Output to Console");
    }
    else if (eStage == StageFirst)
    {
    }

    // call the base class implementation
    return cFilter::Shutdown(eStage, __exception_ptr);
}

tResult cParkoutright::PropertyChanged(const char* strProperty)
{
	ReadProperties(strProperty);
	RETURN_NOERROR;
}

tResult cParkoutright::ReadProperties(const tChar* strPropertyName)
{
//check properties for lanefollow front
	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, DISTANCE_PARKOUT_RIGHT))
	{
		DIST_PARKOUTRIGHT = static_cast<tFloat32> (GetPropertyFloat(DISTANCE_PARKOUT_RIGHT));
	}
	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, SPEED_PARKOUT_RIGHT))
	{
		SPEED_PARKOUTRIGHT = static_cast<tFloat32> (GetPropertyFloat(SPEED_PARKOUT_RIGHT));
	}
//check properties for steer out front
	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, DISTANCE_RIGHT_turn))
	{
		DIST_RIGHTturn = static_cast<tFloat32> (GetPropertyFloat(DISTANCE_RIGHT_turn));
	}
	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, SPEED_RIGHT_turn))
	{
		SPEED_RIGHTturn = static_cast<tFloat32> (GetPropertyFloat(SPEED_RIGHT_turn));
	}
        if (NULL == strPropertyName || cString::IsEqual(strPropertyName, SPEED_RIGHT_turn_back))
        {
                SPEED_RIGHTturn_back = static_cast<tFloat32> (GetPropertyFloat(SPEED_RIGHT_turn_back));
        }
	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, STEER_RIGHT_turn))
	{
		STEER_RIGHTturn = static_cast<tFloat32> (GetPropertyFloat(STEER_RIGHT_turn));
	}
	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, DELTA_YAW))
	{
		YAW_Diff = static_cast<tFloat32> (GetPropertyFloat(DELTA_YAW));
	}
        if (NULL == strPropertyName || cString::IsEqual(strPropertyName, KPSTEERING))
        {
                propKpSteering = static_cast<tFloat32> (GetPropertyFloat(KPSTEERING));
        }
	RETURN_NOERROR;
}

tResult cParkoutright::OnPinEvent(IPin* pSource,
                                    tInt nEventCode,
                                    tInt nParam1,
                                    tInt nParam2,
                                    IMediaSample* pMediaSample)
{
    // first check what kind of event it is
    if (nEventCode == IPinEventSink::PE_MediaSampleReceived)
    {
        // so we received a media sample, so this pointer better be valid.
        if (pMediaSample != NULL)
	{
		// Input signal at Start
		if (pSource == &m_oStart)
		{
			cObjectPtr<IMediaCoder> pCoderInput;
			RETURN_IF_FAILED(m_pDescStart->Lock(pMediaSample, &pCoderInput));
			pCoderInput->Get("bValue", (tVoid*)&m_bStart);
			pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timestamp);
			m_pDescStart->Unlock(pCoderInput);
			 if(!m_bStart)
          		{
               	stop_time = _clock->GetStreamTime();
				m_bTransmitstop = tTrue;
				if (m_bTransmitstop)
                    {
                        m_bHazard_Light= tFalse;
                        m_bBack_Light= tFalse;
                        m_bHead_Light= tFalse;
                        m_bTurnLeft_Light= tFalse;
                        m_bTurnRight_Light= tFalse;
                        m_bBreak_Light= tFalse;
                        
                    }
				}
                         TransmitBoolValue(&m_oOutputLanefollow_Start_test, m_bStart, 0);

		}
                // Input signal at Onelinedetection
                if (pSource == &m_olanechange)
                {
                        cObjectPtr<IMediaCoder> pCoderInput;
                        RETURN_IF_FAILED(m_pCoderDesclanechange->Lock(pMediaSample, &pCoderInput));
                        pCoderInput->Get("bLeft", (tVoid*)&m_bNolinedetection);
                        pCoderInput->Get("bStraight", (tVoid*)&m_bOnelinedetection);
                        pCoderInput->Get("bRigth", (tVoid*)&m_bTwolinedetection);
                        pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timestamp);
                        m_pCoderDesclanechange->Unlock(pCoderInput);

                }
                // Input signal at SteeringIn of LaneFollower
                if (pSource == &m_oSteeringOfLaneFollower)
                {
                        cObjectPtr<IMediaCoder> pCoderInput;
                        RETURN_IF_FAILED(m_pDescSteeringOfLaneFollower->Lock(pMediaSample, &pCoderInput));
                        pCoderInput->Get("f32Value", (tVoid*)&m_fInputSteeringOfLaneFollower);
                        pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timestamp);
                        m_pDescSteeringOfLaneFollower->Unlock(pCoderInput);
                }
		// Input signal at Obstacle
				if (pSource == &m_oObstacle)
				{
					cObjectPtr<IMediaCoder> pCoderInput;
					RETURN_IF_FAILED(m_pDescObstacle->Lock(pMediaSample, &pCoderInput));
                    pCoderInput->Get("bRigth", (tVoid*)&m_bTrafficOnRight);
                    pCoderInput->Get("bStraight", (tVoid*)&m_bTrafficOnStraight);
                    pCoderInput->Get("bLeft", (tVoid*)&m_bTrafficOnLeft);
					pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timestamp);
					m_pDescObstacle->Unlock(pCoderInput);
					if (m_bTrafficOnRight || m_bTrafficOnStraight || m_bTrafficOnLeft)
					{
						m_bObstacle = tTrue;
					}
					else
					{
						m_bObstacle = tFalse;
					}

				}

                // Input signal at YaW
                else if (pSource == &m_oYaw)
                {
                        cObjectPtr<IMediaCoder> pCoderInput;
                        RETURN_IF_FAILED(m_pDescYaw->Lock(pMediaSample, &pCoderInput));
                        pCoderInput->Get("f32Value", (tVoid*)&m_fYaw);
                        pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timestamp);
                        m_pDescYaw->Unlock(pCoderInput);
                }
                else if (pSource == &m_oUSRightside)
                {
                    cObjectPtr<IMediaCoder> pCoderInput;
                    RETURN_IF_FAILED(m_pDescUSRightside->Lock(pMediaSample, &pCoderInput));
                    pCoderInput->Get("f32Value", (tVoid*)&m_fUSRightside_row);
                    pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timestamp);
                    m_pDescUSRightside->Unlock(pCoderInput);
					if(m_fUSRightside_row<1)
					{
						m_fUSRightside=400;
					}
					else
					{
						m_fUSRightside=m_fUSRightside_row;
					}
                }

		// Input signal at Distance Overall
		else if (pSource == &m_oDistanceOverall)
		{
			//if(m_bDebugModeEnabled) LOG_INFO("Vinoth distance info");
			cObjectPtr<IMediaCoder> pCoderInput;
			RETURN_IF_FAILED(m_pDescdistanceoverall->Lock(pMediaSample, &pCoderInput));
			pCoderInput->Get("f32Value", (tVoid*)&m_fDistanceOverall);
			pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timestamp);
			m_pDescdistanceoverall->Unlock(pCoderInput);
		}
		// Input signal at linestop
			else if (pSource == &m_oStopLine)
			{
				cObjectPtr<IMediaCoder> pCoderInput;
				RETURN_IF_FAILED(m_pDescStopLine->Lock(pMediaSample, &pCoderInput));
				pCoderInput->Get("bValue", (tVoid*)&m_bLine_detection);
				pCoderInput->Get("f32Distance", (tVoid*)&m_fLine_distance);
				pCoderInput->Get("f32Orientation", (tVoid*)&m_fOrientation2StopLine);
				m_pDescStopLine->Unlock(pCoderInput);

			}


// stop signal
	if(!m_bStart)
	{
		//if(m_bDebugModeEnabled) LOG_INFO(cString::Format("Start bool false"));
		m_fAccelerationOutput=0;
		m_fSteeringOutput=0;
        m_iStateOfParkoutright=SOP_NOSTART;

			if(m_bTransmitstop)
			{
                            m_fSteeringOutput=0;
                            m_fAccelerationOutput=0;
				//if(m_bDebugModeEnabled) LOG_INFO(cString::Format("stop signal"));
				//TransmitOutput(m_fSteeringOutput,m_fAccelerationOutput);
				m_bHazard_Light= tFalse;
				m_bBack_Light= tFalse;
				m_bHead_Light= tFalse;
				m_bTurnLeft_Light= tFalse;
				m_bTurnRight_Light= tFalse;
				m_bBreak_Light= tFalse;
				TransmitFloatValue(&m_oOutputSteering, m_fSteeringOutput, 0);
				TransmitFloatValue(&m_oOutputAcceleration, m_fAccelerationOutput, 0);
				TransmitBoolValue(&m_oOutputHazard_Light, m_bHazard_Light, 0);
				TransmitBoolValue(&m_oOutputHead_Light, m_bHead_Light, 0);
				TransmitBoolValue(&m_oOutputTurnRight_Light, m_bTurnRight_Light, 0);
				TransmitBoolValue(&m_oOutputBreak_Light, m_bBreak_Light, 0);
				m_bTransmitstop= tFalse;
		        }

	}

	else
	{
                //if(m_bDebugModeEnabled) LOG_INFO(cString::Format("Start bool true"));
		stage_parkoutright();
		//TransmitLight();
		//TransmitOutput(m_fSteeringOutput,m_fAccelerationOutput);
                        TransmitFloatValue(&m_oOutputSteering, m_fSteeringOutput, 0);
                        TransmitFloatValue(&m_oOutputAcceleration, m_fAccelerationOutput, 0);


                        TransmitBoolValue(&m_oOutputBreak_Light, m_bBreak_Light, 0);
	}
    }

}
	    RETURN_NOERROR;
}


tResult cParkoutright::stage_parkoutright()
{
     //   if(m_bDebugModeEnabled) LOG_INFO(cString::Format("PARKING OUT Right"));
     //   if(m_bDebugModeEnabled) LOG_INFO(cString::Format("m_iStateOfParkoutright %i, m_bStart %i", m_iStateOfParkoutright, m_bStart));
	switch(m_iStateOfParkoutright)
        {
        if(m_bDebugModeEnabled) LOG_INFO(cString::Format("PARKoutright"));
        if(m_bDebugModeEnabled)LOG_INFO(cString::Format("Parkoutright case: %d",m_iStateOfParkoutright));
        case SOP_NOSTART:
                {

             //           if(m_bDebugModeEnabled) LOG_INFO(cString::Format("Step 0:no start"));
                        m_fSteeringOutput=0;
                        m_fAccelerationOutput=0;
                        //reset finish flag
                        m_bFinished=tFalse;
                        m_fcover_dist = DIST_PARKOUTRIGHT;
						m_bBreak_Light = tTrue;
                                                TransmitBoolValue(&m_oOutputBreak_Light, m_bBreak_Light, 0);
                                                stop_time = _clock->GetStreamTime(); //timer reset
                        // change state of turn when StartSignal is true
                        if(m_bStart)
                                {
										
                            m_bHead_Light = tTrue;
                            TransmitBoolValue(&m_oOutputHead_Light, m_bHead_Light, 0);
                                        m_iStateOfParkoutright = SOP_Obstacle;
					m_bHazard_Light= tTrue;
					TransmitBoolValue(&m_oOutputHazard_Light, m_bHazard_Light, 0);
                                        //save the first distance and get started
                                        m_fDistanceOverall_Start=m_fDistanceOverall;
                                        if (m_bLine_detection)
                                        {
                                                m_bLine_detection = tFalse;

                                                if (m_bLine_detection && m_fLine_distance >80 && m_fLine_distance < 120)
                                                {
                                                        m_fcover_dist = (m_fLine_distance / 100) - 0.95; //-70
                                                        m_bLine_detection = tFalse;
                                                }
                                                else if (m_bLine_detection && m_fLine_distance <70 && m_fLine_distance > 40)
                                                {
                                                        m_fcover_dist = (m_fLine_distance / 100) - 0.60; //-30
                                                        m_bLine_detection = tFalse;
                                                }
                                        }
                                }
                        break;
                }
		case SOP_Obstacle:
		{
                                                         if ((_clock->GetStreamTime() - stop_time) / 1000000 < 1) //time for wait
							 {
                                        //			 if (m_bDebugModeEnabled) LOG_INFO(cString::Format("Step 1:wait"));
                                       //                          if (m_bDebugModeEnabled) LOG_INFO(cString::Format("Step 1: still waiting stoptime %i   ", _clock->GetStreamTime(), stop_time ));
								 m_fSteeringOutput = 0;		//accuator output for backside_lanefollow
								 m_fAccelerationOutput = 0;

                                                                 //if (m_bObstacle)
                                                                 //{
                                                                //	 stop_time = _clock->GetStreamTime(); //timer reset
                                                                  //       m_bBreak_Light = tTrue;
                                                                 //}

                                                           }
                                                         //else if(!m_bObstacle &&(_clock->GetStreamTime() - stop_time) / 1000000 > 1)
                                                         else
                                                            {



                                                                 m_iStateOfParkoutright = SOP_Start_step1;
                                                                 m_bBreak_Light = tFalse;
                                                                 TransmitBoolValue(&m_oOutputBreak_Light, m_bBreak_Light, 0);
                                                                 m_fDistanceOverall_Start=m_fDistanceOverall;
                                                                 m_bTurnRight_Light = tTrue;
                                                                 TransmitBoolValue(&m_oOutputTurnRight_Light, m_bTurnRight_Light, 0);
                                                                 m_bHazard_Light= tFalse;
                                                                 TransmitBoolValue(&m_oOutputHazard_Light, m_bHazard_Light, 0);

                                                            }

							 break;
		}
		case SOP_Start_step1:
						{

                                                //		if (m_bDebugModeEnabled) LOG_INFO(cString::Format("Step 1:desired distance"));
                                                //                LOG_INFO(cString::Format("MOHAMMAD m_fDistanceOverall %f, m_fDistanceOverall_Start %f , diff %f m_fcover_dist %f",m_fDistanceOverall, m_fDistanceOverall_Start, (m_fDistanceOverall - m_fDistanceOverall_Start), m_fcover_dist));
								if (m_fDistanceOverall - m_fDistanceOverall_Start < m_fcover_dist) // distance for first_lanefollow
								{
                                        //				if (m_bDebugModeEnabled)LOG_INFO(cString::Format("Step 1:desired distance"));
                                                                        if (m_bLine_detection)
                                                                        {
                                                                            m_fSteeringOutput = propKpSteering*(m_fOrientation2StopLine - 90);	//accuator output
                                                                        }
                                                                        else
                                                                        {
                                                                            m_fSteeringOutput = 0;
                                                                        }
									m_fAccelerationOutput = SPEED_PARKOUTRIGHT;

								}
								else
								{

                                                                                m_iStateOfParkoutright = SOP_Rightsteer_front;  // change parking manuver
										m_fDistanceOverall_Start = m_fDistanceOverall;

										m_fYaw_Start = m_fYaw;
                                                                                 m_fSteeringOutput = 0;		//accuator output for backside_lanefollow
                                                                                 m_fAccelerationOutput = 0;
                                                                                 if (m_fcover_dist>=0)m_fSteerGain = 240 + (m_fcover_dist*30);
                                                                                 else m_fSteerGain= 240 ;
										
									
								}
							
                        break;
                }
        case SOP_Rightsteer_front:
                {
                       //  if(m_bDebugModeEnabled) LOG_INFO(cString::Format("Step 2:rightsteer frontside"));

                         if((m_fDistanceOverall-m_fDistanceOverall_Start) < DIST_RIGHTturn ) // distance for rightsteer_front
                                {
                                        stop_time = _clock->GetStreamTime();
                                        if(((m_fDistanceOverall-m_fDistanceOverall_Start) < DIST_RIGHTturn-0.70))
                                        {
                                            m_fSteeringCalc=(((m_fDistanceOverall-m_fDistanceOverall_Start)/(DIST_RIGHTturn-0.70))*m_fSteerGain);
                                            if(m_fSteeringCalc<70)m_fSteeringOutput=m_fSteeringCalc;
                                            else m_fSteeringOutput=70;
                                            m_fAccelerationOutput=SPEED_RIGHTturn + (m_fSteeringOutput/1000);

                               //             LOG_INFO(cString::Format("OpenLoop distance %f, steering %f, acc %f",m_fDistanceOverall-m_fDistanceOverall_Start ,m_fSteeringOutput, m_fAccelerationOutput ));
                                        }
                                        else if (((m_fDistanceOverall-m_fDistanceOverall_Start) >= DIST_RIGHTturn-0.70) &&((m_fDistanceOverall-m_fDistanceOverall_Start) < DIST_RIGHTturn))
                                        {
                                            if(m_bOnelinedetection)
                                            {
                                                m_fDistanceOverall_Start=m_fDistanceOverall;
                                                m_iStateOfParkoutright=SOP_Leftsteer_back;
                                                m_fAccelerationOutput=0 ;

                                                m_bBreak_Light= tTrue;
                                                TransmitBoolValue(&m_oOutputBreak_Light, m_bBreak_Light, 0);
                                                m_fSteeringOutput=0;

                                      //           LOG_INFO(cString::Format("ClosedLoop distance %f, steering %f",m_fDistanceOverall-m_fDistanceOverall_Start ,m_fSteeringOutput ));
                                            }

                                            else if(m_bTwolinedetection)
                                            {
                                                m_fSteeringOutput=m_fInputSteeringOfLaneFollower;

                                                m_fDistanceOverall_Start=m_fDistanceOverall;
                                                m_iStateOfParkoutright=SOP_Leftsteer_back;
                                                m_fAccelerationOutput=0 ;
                                                m_fSteeringOutput=0;

                                                m_bBreak_Light= tTrue;
                                                TransmitBoolValue(&m_oOutputBreak_Light, m_bBreak_Light, 0);
                                   //         LOG_INFO(cString::Format("ClosedLoop transfer from two line detection distance %f, steering %f",m_fDistanceOverall-m_fDistanceOverall_Start ,m_fSteeringOutput ));


                                            }
                                            else
                                            {
                                                m_fSteeringCalc=((m_fDistanceOverall-m_fDistanceOverall_Start)/(DIST_RIGHTturn))*m_fSteerGain;
                                                if(m_fSteeringCalc<70)m_fSteeringOutput=m_fSteeringCalc;
                                                else m_fSteeringOutput=70;
                                                m_fAccelerationOutput=SPEED_RIGHTturn+(m_fSteeringOutput/1000);

                             //               LOG_INFO(cString::Format("ClosedLoop distance %f, steering %f",m_fDistanceOverall-m_fDistanceOverall_Start ,m_fSteeringOutput ));
                                        }
                                        }
                                        else
                                            {
                                            if(m_bOnelinedetection)
                                            {
                                                m_fSteeringOutput=1.2*m_fInputSteeringOfLaneFollower;
                                                m_fAccelerationOutput=0 ;
                                                m_iStateOfParkoutright=SOP_Leftsteer_back;
                                                m_fDistanceOverall_Start=m_fDistanceOverall;

                                                m_bBreak_Light= tTrue;
                                                TransmitBoolValue(&m_oOutputBreak_Light, m_bBreak_Light, 0);
                                     //           LOG_INFO(cString::Format("ClosedLoop transfer from one line detection distance %f, steering %f",m_fDistanceOverall-m_fDistanceOverall_Start ,m_fSteeringOutput ));
                                            }

                                            else if(m_bTwolinedetection)
                                            {
                                                m_fSteeringOutput=m_fInputSteeringOfLaneFollower;

                                                m_fDistanceOverall_Start=m_fDistanceOverall;
                                                m_iStateOfParkoutright=SOP_Leftsteer_back;
                                                m_fAccelerationOutput = 0;

                                                m_bBreak_Light= tTrue;
                                                TransmitBoolValue(&m_oOutputBreak_Light, m_bBreak_Light, 0);



                                            }
                                            else
                                            {
                                                m_fSteeringCalc=(((m_fDistanceOverall-m_fDistanceOverall_Start)/(DIST_RIGHTturn))*m_fSteerGain);
                                                if(m_fSteeringCalc<90)m_fSteeringOutput=m_fSteeringCalc;
                                                else m_fSteeringOutput=90;
                                                m_fAccelerationOutput=SPEED_RIGHTturn+(m_fSteeringOutput/1000);

                                   //         LOG_INFO(cString::Format("ClosedLoop distance %f, steering %f",m_fDistanceOverall-m_fDistanceOverall_Start ,m_fSteeringOutput  ));
                                            }
                                            }

                                }
                         else
                                {

                             m_fSteeringOutput = -70;		//accuator output for backside_lanefollow
                             m_fAccelerationOutput = 0;
                                                        m_iStateOfParkoutright=SOP_Leftsteer_back;  // change parking manuver
                                                        m_fDistanceOverall_Start=m_fDistanceOverall;
                                                        m_fYaw_Start=m_fYaw;
                                                                m_bBreak_Light = tTrue;
                                                            TransmitBoolValue(&m_oOutputBreak_Light, m_bBreak_Light, 0);

                                                
                                }
                         break;
                }
        case SOP_Leftsteer_back:
                {
                     //    if(m_bDebugModeEnabled) LOG_INFO(cString::Format("Step 2:rightsteer frontside"));

                         if((m_fDistanceOverall-m_fDistanceOverall_Start) < DIST_RIGHTturn+0.10 ) // distance for rightsteer_front
                                {
                             stop_time = _clock->GetStreamTime();
                             if(((m_fDistanceOverall-m_fDistanceOverall_Start) < DIST_RIGHTturn-0.9))
                             {
                                 m_fSteeringCalc=-20+ (((m_fDistanceOverall-m_fDistanceOverall_Start)/(DIST_RIGHTturn-0.9))*-m_fSteerGain);
                                 if(m_fSteeringCalc<=-75)m_fSteeringOutput=m_fSteeringCalc;
                                 else m_fSteeringOutput=-75;
                                 m_fAccelerationOutput=SPEED_RIGHTturn_back +1.5*(m_fSteeringOutput/1000);;

                    //             LOG_INFO(cString::Format("OpenLoop distance %f, steering %f",m_fDistanceOverall-m_fDistanceOverall_Start ,m_fSteeringOutput  ));
                             }
                             else if (((m_fDistanceOverall-m_fDistanceOverall_Start) >= DIST_RIGHTturn-0.9) &&((m_fDistanceOverall-m_fDistanceOverall_Start) < DIST_RIGHTturn))
                             {
                                 if(m_bOnelinedetection)
                                 {
                                     m_fSteeringOutput=-m_fInputSteeringOfLaneFollower;
                                     m_fAccelerationOutput=SPEED_RIGHTturn_back ;
                                 }

                                 else if(m_bTwolinedetection)
                                 {
                                     m_fSteeringOutput=m_fInputSteeringOfLaneFollower;

                                     m_fDistanceOverall_Start=m_fDistanceOverall;
                                     m_iStateOfParkoutright=SOP_Finished;
                                     m_fAccelerationOutput=0 ;
                                     m_bTurnRight_Light = tFalse;
                                     TransmitBoolValue(&m_oOutputTurnRight_Light, m_bTurnRight_Light, 0);





                                 }
                                 else
                                 {
                                     m_fSteeringCalc=-20 + (((m_fDistanceOverall-m_fDistanceOverall_Start)/(DIST_RIGHTturn))*-m_fSteerGain);
                                     if(m_fSteeringCalc<=-75)m_fSteeringOutput=m_fSteeringCalc;
                                     else m_fSteeringOutput=-75;
                                     m_fAccelerationOutput=SPEED_RIGHTturn_back + 1.5*(m_fSteeringOutput/1000);

                     //            LOG_INFO(cString::Format("ClosedLoop distance %f, steering %f",m_fDistanceOverall-m_fDistanceOverall_Start ,m_fSteeringOutput ));
                             }
                             }
                             else
                                 {
                                 if(m_bOnelinedetection)
                                 {
                                     m_fSteeringOutput=0;
                                     m_fAccelerationOutput=0 ;
                                     m_iStateOfParkoutright=SOP_Finished;
                                     m_fDistanceOverall_Start=m_fDistanceOverall;
                                     m_bTurnRight_Light = tFalse;


                                 }

                                 else if(m_bTwolinedetection)
                                 {
                                     m_fSteeringOutput=m_fInputSteeringOfLaneFollower;

                                     m_fDistanceOverall_Start=m_fDistanceOverall;
                                     m_iStateOfParkoutright=SOP_Finished;
                                     m_fAccelerationOutput = 0;
                                     m_bTurnRight_Light = tFalse;
                                     m_bBreak_Light= tTrue;




                                 }
                                 else
                                 {
                                     m_fSteeringCalc=-20+(((m_fDistanceOverall-m_fDistanceOverall_Start)/(DIST_RIGHTturn))*-m_fSteerGain);
                                     if(m_fSteeringCalc<=-75)m_fSteeringOutput=m_fSteeringCalc;
                                     else m_fSteeringOutput=-75;
                                     m_fAccelerationOutput=SPEED_RIGHTturn + 1.5*(m_fSteeringOutput/1000);;

                         //        LOG_INFO(cString::Format("ClosedLoop distance %f, steering %f",m_fDistanceOverall-m_fDistanceOverall_Start ,m_fSteeringOutput  ));
                                 }
                                 }
                                }
                         else
                                {

                             m_fSteeringOutput = 0;		//accuator output for backside_lanefollow
                             m_fAccelerationOutput = 0;
                                                        m_iStateOfParkoutright=SOP_Finished;  // change parking manuver
                                                        m_fDistanceOverall_Start=m_fDistanceOverall;
                                                        m_fYaw_Start=m_fYaw;
                                                        m_bBreak_Light = tFalse;


                                }
                         break;
                }
        case SOP_Finished:
                {
                 //       if(m_bDebugModeEnabled) LOG_INFO(cString::Format("Step 5 : finished"));
                                //stop
                        m_bFinished=tTrue;
						m_bHazard_Light = tFalse;
						TransmitBoolValue(&m_oOutputHazard_Light, m_bHazard_Light, 0);
						m_bBack_Light = tFalse;
						m_bHead_Light = tFalse;
                                                m_bHead_Light = tTrue;
                                                TransmitBoolValue(&m_oOutputHead_Light, m_bHead_Light, 0);
						m_bTurnLeft_Light = tFalse;
						m_bTurnRight_Light = tFalse;
                                                TransmitBoolValue(&m_oOutputTurnRight_Light, m_bTurnRight_Light, 0);
						m_bBreak_Light = tFalse;
                        m_bStart=tFalse;
                        stop_time = _clock->GetStreamTime();
                        TransmitFinish();
                        TransmitBoolValue(&m_oOutputLanefollow_Start_test, m_bStart, 0);
                 //       if(m_bDebugModeEnabled) LOG_INFO(cString::Format("Step 5 : sent transmit finish for parking= %d ",m_bFinished));
                        m_iStateOfParkoutright=SOP_NOSTART;

                        break;
                }
        default :
                {
                        m_iStateOfParkoutright=SOP_NOSTART;
                        m_fDistanceOverall_Start=m_fDistanceOverall;
              //          if(m_bDebugModeEnabled) LOG_INFO(cString::Format("PARKING default case"));
                }
	}
RETURN_NOERROR;

}
tResult cParkoutright::TransmitFloatValue(cOutputPin* oPin, tFloat32 value, tUInt32 timestamp)
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
tResult cParkoutright::TransmitBoolValue(cOutputPin* oPin, bool value, tUInt32 timestamp)
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
/*
tResult cParkoutright::TransmitLight()
{
        // Create a new MediaSmaple
        cObjectPtr<IMediaSample> pMediaSampleHead_Light;
        cObjectPtr<IMediaSample> pMediaSampleBack_Light;
        cObjectPtr<IMediaSample> pMediaSampleHazard_Light;
        cObjectPtr<IMediaSample> pMediaSampleTurnLeft_Light;
        cObjectPtr<IMediaSample> pMediaSampleTurnRight_Light;
        cObjectPtr<IMediaSample> pMediaSampleBreak_Light;

        AllocMediaSample((tVoid**)&pMediaSampleHead_Light);
        AllocMediaSample((tVoid**)&pMediaSampleBack_Light);
        AllocMediaSample((tVoid**)&pMediaSampleHazard_Light);
        AllocMediaSample((tVoid**)&pMediaSampleTurnLeft_Light);
        AllocMediaSample((tVoid**)&pMediaSampleTurnRight_Light);
        AllocMediaSample((tVoid**)&pMediaSampleBreak_Light);





        // Send the Media Sample Head_Light
        cObjectPtr<IMediaSerializer> pSerializerHead_Light;
        m_pDescriptionOutputHead_Light->GetMediaSampleSerializer(&pSerializerHead_Light);
        tInt nSizeHead_Light = pSerializerHead_Light->GetDeserializedSize();
        pMediaSampleHead_Light->AllocBuffer(nSizeHead_Light);
        cObjectPtr<IMediaCoder> pCoderOutputHead_Light;
        m_pDescriptionOutputHead_Light->WriteLock(pMediaSampleHead_Light, &pCoderOutputHead_Light);
        pCoderOutputHead_Light->Set("bValue", (tVoid*)&(m_bHead_Light));
        pCoderOutputHead_Light->Set("ui32ArduinoTimestamp", (tVoid*)&timestamp);
        m_pDescriptionOutputHead_Light->Unlock(pCoderOutputHead_Light);
        pMediaSampleHead_Light->SetTime(_clock->GetStreamTime());
        m_oOutputHead_Light.Transmit(pMediaSampleHead_Light);


        // Send the Media Sample Back_Light
        cObjectPtr<IMediaSerializer> pSerializerBack_Light;
        m_pDescriptionOutputBack_Light->GetMediaSampleSerializer(&pSerializerBack_Light);
        tInt nSizeBack_Light = pSerializerBack_Light->GetDeserializedSize();
        pMediaSampleBack_Light->AllocBuffer(nSizeBack_Light);
        cObjectPtr<IMediaCoder> pCoderOutputBack_Light;
        m_pDescriptionOutputBack_Light->WriteLock(pMediaSampleBack_Light, &pCoderOutputBack_Light);
        pCoderOutputBack_Light->Set("bValue", (tVoid*)&(m_bBack_Light));
        pCoderOutputBack_Light->Set("ui32ArduinoTimestamp", (tVoid*)&timestamp);
        m_pDescriptionOutputBack_Light->Unlock(pCoderOutputBack_Light);
        pMediaSampleBack_Light->SetTime(_clock->GetStreamTime());
        m_oOutputBack_Light.Transmit(pMediaSampleBack_Light);


        // Send the Media Sample Hazard_Light
        cObjectPtr<IMediaSerializer> pSerializerHazard_Light;
        m_pDescriptionOutputHazard_Light->GetMediaSampleSerializer(&pSerializerHazard_Light);
        tInt nSizeHazard_Light = pSerializerHazard_Light->GetDeserializedSize();
        pMediaSampleHazard_Light->AllocBuffer(nSizeHazard_Light);
        cObjectPtr<IMediaCoder> pCoderOutputHazard_Light;
        m_pDescriptionOutputHazard_Light->WriteLock(pMediaSampleHazard_Light, &pCoderOutputHazard_Light);
        pCoderOutputHazard_Light->Set("bValue", (tVoid*)&(m_bHazard_Light));
        pCoderOutputHazard_Light->Set("ui32ArduinoTimestamp", (tVoid*)&timestamp);
        m_pDescriptionOutputHazard_Light->Unlock(pCoderOutputHazard_Light);
        pMediaSampleHazard_Light->SetTime(_clock->GetStreamTime());
        m_oOutputHazard_Light.Transmit(pMediaSampleHazard_Light);

        // Send the Media Sample TurnLeft_Light
        cObjectPtr<IMediaSerializer> pSerializerTurnLeft_Light;
        m_pDescriptionOutputTurnLeft_Light->GetMediaSampleSerializer(&pSerializerTurnLeft_Light);
        tInt nSizeTurnLeft_Light = pSerializerTurnLeft_Light->GetDeserializedSize();
        pMediaSampleTurnLeft_Light->AllocBuffer(nSizeTurnLeft_Light);
        cObjectPtr<IMediaCoder> pCoderOutputTurnLeft_Light;
        m_pDescriptionOutputTurnLeft_Light->WriteLock(pMediaSampleTurnLeft_Light, &pCoderOutputTurnLeft_Light);
        pCoderOutputTurnLeft_Light->Set("bValue", (tVoid*)&(m_bTurnLeft_Light));
        pCoderOutputTurnLeft_Light->Set("ui32ArduinoTimestamp", (tVoid*)&timestamp);
        m_pDescriptionOutputTurnLeft_Light->Unlock(pCoderOutputTurnLeft_Light);
        pMediaSampleTurnLeft_Light->SetTime(_clock->GetStreamTime());
        m_oOutputTurnLeft_Light.Transmit(pMediaSampleTurnLeft_Light);


        // Send the Media Sample TurnRight_Light
        cObjectPtr<IMediaSerializer> pSerializerTurnRight_Light;
        m_pDescriptionOutputTurnRight_Light->GetMediaSampleSerializer(&pSerializerTurnRight_Light);
        tInt nSizeTurnRight_Light = pSerializerTurnRight_Light->GetDeserializedSize();
        pMediaSampleTurnRight_Light->AllocBuffer(nSizeTurnRight_Light);
        cObjectPtr<IMediaCoder> pCoderOutputTurnRight_Light;
        m_pDescriptionOutputTurnRight_Light->WriteLock(pMediaSampleTurnRight_Light, &pCoderOutputTurnRight_Light);
        pCoderOutputTurnRight_Light->Set("bValue", (tVoid*)&(m_bTurnRight_Light));
        pCoderOutputTurnRight_Light->Set("ui32ArduinoTimestamp", (tVoid*)&timestamp);
        m_pDescriptionOutputTurnRight_Light->Unlock(pCoderOutputTurnRight_Light);
        pMediaSampleTurnRight_Light->SetTime(_clock->GetStreamTime());
        m_oOutputTurnRight_Light.Transmit(pMediaSampleTurnRight_Light);


        // Send the Media Sample Break_Light
        cObjectPtr<IMediaSerializer> pSerializerBreak_Light;
        m_pDescriptionOutputBreak_Light->GetMediaSampleSerializer(&pSerializerBreak_Light);
        tInt nSizeBreak_Light = pSerializerBreak_Light->GetDeserializedSize();
        pMediaSampleBreak_Light->AllocBuffer(nSizeBreak_Light);
        cObjectPtr<IMediaCoder> pCoderOutputBreak_Light;
        m_pDescriptionOutputBreak_Light->WriteLock(pMediaSampleBreak_Light, &pCoderOutputBreak_Light);
        pCoderOutputBreak_Light->Set("bValue", (tVoid*)&(m_bBreak_Light));
        pCoderOutputBreak_Light->Set("ui32ArduinoTimestamp", (tVoid*)&timestamp);
        m_pDescriptionOutputBreak_Light->Unlock(pCoderOutputBreak_Light);
        pMediaSampleBreak_Light->SetTime(_clock->GetStreamTime());
        m_oOutputBreak_Light.Transmit(pMediaSampleBreak_Light);

        RETURN_NOERROR;
}
*/
tResult cParkoutright::TransmitFinish()
{
	// Create a new MediaSmaple
        cObjectPtr<IMediaSample> pMediaSampleParkoutright_Finish;
        AllocMediaSample((tVoid**)&pMediaSampleParkoutright_Finish);

        // Send the Media Sample
        cObjectPtr<IMediaSerializer> pSerializerParkoutright_Finish;
        m_pDescriptionOutputParkoutright_Finish->GetMediaSampleSerializer(&pSerializerParkoutright_Finish);
        tInt nSizeParkoutright_Finish = pSerializerParkoutright_Finish->GetDeserializedSize();
        pMediaSampleParkoutright_Finish->AllocBuffer(nSizeParkoutright_Finish);
        cObjectPtr<IMediaCoder> pCoderOutputParkoutright_Finish;
        m_pDescriptionOutputParkoutright_Finish->WriteLock(pMediaSampleParkoutright_Finish, &pCoderOutputParkoutright_Finish);
        pCoderOutputParkoutright_Finish->Set("bValue", (tVoid*)&(m_bFinished));
        pCoderOutputParkoutright_Finish->Set("ui32ArduinoTimestamp", (tVoid*)&timestamp);
        m_pDescriptionOutputParkoutright_Finish->Unlock(pCoderOutputParkoutright_Finish);
        pMediaSampleParkoutright_Finish->SetTime(_clock->GetStreamTime());
        m_oOutputParkoutright_Finish.Transmit(pMediaSampleParkoutright_Finish);
	RETURN_NOERROR;
}


/*
tResult cParkoutright::TransmitObstacle_detection()
{
	// Create a new MediaSmaple
	cObjectPtr<IMediaSample> pMediaSampleObstacle_detection;
	AllocMediaSample((tVoid**)&pMediaSampleObstacle_detection);

	// Send the Media Sample
	cObjectPtr<IMediaSerializer> pSerializerObstacle_detection;
	m_pDescriptionOutputObstacle_detection->GetMediaSampleSerializer(&pSerializerObstacle_detection);
	tInt nSizeObstacle_detection = pSerializerObstacle_detection->GetDeserializedSize();
	pMediaSampleObstacle_detection->AllocBuffer(nSizeObstacle_detection);
	cObjectPtr<IMediaCoder> pCoderOutputObstacle_detection;
	m_pDescriptionOutputObstacle_detection->WriteLock(pMediaSampleObstacle_detection, &pCoderOutputObstacle_detection);
	pCoderOutputObstacle_detection->Set("bValue", (tVoid*)&(m_bObstacle_detection));
	pCoderOutputObstacle_detection->Set("ui32ArduinoTimestamp", (tVoid*)&timestamp);
	m_pDescriptionOutputObstacle_detection->Unlock(pCoderOutputObstacle_detection);
	pMediaSampleObstacle_detection->SetTime(_clock->GetStreamTime());
	m_oOutputObstacle_detection.Transmit(pMediaSampleObstacle_detection);
	RETURN_NOERROR;
}


tResult cParkoutright::TransmitOutput(tFloat32 speed,tFloat32 steering)
{

	//create new media sample
	cObjectPtr<IMediaSample> pMediaSampleaccelerate;
	cObjectPtr<IMediaSample> pMediaSamplesteer;
	cObjectPtr<IMediaSample> pMediaSampleTurnSignalLeftEnabled;
	cObjectPtr<IMediaSample> pMediaSampleTurnSignalRightEnabled;

	AllocMediaSample((tVoid**)&pMediaSampleaccelerate);
	AllocMediaSample((tVoid**)&pMediaSamplesteer);
	AllocMediaSample((tVoid**)&pMediaSampleTurnSignalLeftEnabled);
	AllocMediaSample((tVoid**)&pMediaSampleTurnSignalRightEnabled);

	// acceleration
	cObjectPtr<IMediaSerializer> pSerializeraccelerate;
	m_pDescriptionOutputAcceleration->GetMediaSampleSerializer(&pSerializeraccelerate);
	tInt nSizeaccelerate = pSerializeraccelerate->GetDeserializedSize();
	pMediaSampleaccelerate->AllocBuffer(nSizeaccelerate);
	cObjectPtr<IMediaCoder> pCoderOutputaccelerate;
	m_pDescriptionOutputAcceleration->WriteLock(pMediaSampleaccelerate, &pCoderOutputaccelerate);
	pCoderOutputaccelerate->Set("f32Value", (tVoid*)&(m_fAccelerationOutput));
	pCoderOutputaccelerate->Set("ui32ArduinoTimestamp", (tVoid*)&timestamp);
	m_pDescriptionOutputAcceleration->Unlock(pCoderOutputaccelerate);
	pMediaSampleaccelerate->SetTime(_clock->GetStreamTime());
	m_oOutputAcceleration.Transmit(pMediaSampleaccelerate);

	// steering
	cObjectPtr<IMediaSerializer> pSerializersteer;
    	m_pDescriptionOutputSteering->GetMediaSampleSerializer(&pSerializersteer);
    	tInt nSizesteer = pSerializersteer->GetDeserializedSize();
    	pMediaSamplesteer->AllocBuffer(nSizesteer);
	cObjectPtr<IMediaCoder> pCoderOutputsteer;
	m_pDescriptionOutputSteering->WriteLock(pMediaSamplesteer, &pCoderOutputsteer);
	pCoderOutputsteer->Set("f32Value", (tVoid*)&(m_fSteeringOutput));
	pCoderOutputsteer->Set("ui32ArduinoTimestamp", (tVoid*)&timestamp);
	m_pDescriptionOutputSteering->Unlock(pCoderOutputsteer);
	pMediaSamplesteer->SetTime(_clock->GetStreamTime());
	m_oOutputSteering.Transmit(pMediaSamplesteer);
	
	// TurnSignalLeftEnabled
	cObjectPtr<IMediaSerializer> pSerializerTurnSignalLeftEnabled;
	m_pDescriptionOutputTurnSignalLeftEnabled->GetMediaSampleSerializer(&pSerializerTurnSignalLeftEnabled);
	tInt nSizeTurnSignalLeftEnabled = pSerializerTurnSignalLeftEnabled->GetDeserializedSize();
	pMediaSampleTurnSignalLeftEnabled->AllocBuffer(nSizeTurnSignalLeftEnabled);
	cObjectPtr<IMediaCoder> pCoderOutputTurnSignalLeftEnabled;
	m_pDescriptionOutputTurnSignalLeftEnabled->WriteLock(pMediaSampleTurnSignalLeftEnabled, &pCoderOutputTurnSignalLeftEnabled);
	pCoderOutputTurnSignalLeftEnabled->Set("bValue", (tVoid*)&(m_bTurnSignalLeftEnabled));
	pCoderOutputTurnSignalLeftEnabled->Set("ui32ArduinoTimestamp", (tVoid*)&timestamp);
	m_pDescriptionOutputTurnSignalLeftEnabled->Unlock(pCoderOutputTurnSignalLeftEnabled);
	pMediaSampleTurnSignalLeftEnabled->SetTime(_clock->GetStreamTime());
	m_oOutputAcceleration.Transmit(pMediaSampleTurnSignalLeftEnabled);

	// TurnSignalRightEnabled
	cObjectPtr<IMediaSerializer> pSerializerTurnSignalRightEnabled;
	m_pDescriptionOutputTurnSignalRightEnabled->GetMediaSampleSerializer(&pSerializerTurnSignalRightEnabled);
	tInt nSizeTurnSignalRightEnabled = pSerializerTurnSignalRightEnabled->GetDeserializedSize();
	pMediaSampleTurnSignalRightEnabled->AllocBuffer(nSizeTurnSignalRightEnabled);
	cObjectPtr<IMediaCoder> pCoderOutputTurnSignalRightEnabled;
	m_pDescriptionOutputTurnSignalRightEnabled->WriteLock(pMediaSampleTurnSignalRightEnabled, &pCoderOutputTurnSignalRightEnabled);
	pCoderOutputTurnSignalRightEnabled->Set("bValue", (tVoid*)&(m_bTurnSignalRightEnabled));
	pCoderOutputTurnSignalRightEnabled->Set("ui32ArduinoTimestamp", (tVoid*)&timestamp);
	m_pDescriptionOutputTurnSignalRightEnabled->Unlock(pCoderOutputTurnSignalRightEnabled);
	pMediaSampleTurnSignalRightEnabled->SetTime(_clock->GetStreamTime());
	m_oOutputAcceleration.Transmit(pMediaSampleTurnSignalRightEnabled);

	RETURN_NOERROR;
}
*/


