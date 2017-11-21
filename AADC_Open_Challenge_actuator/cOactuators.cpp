/**
Copyright (c)
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: ?This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.?
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS ?AS IS? AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


**********************************************************************
* $Author:: spiesra $  $Date:: 2017-05-12 09:34:53#$ $Rev:: 63109   $
**********************************************************************/

#include "stdafx.h"
#include "cOactuators.h"
#define DISTANCE_SCAN_SLOT_LENGTH               "cOactuators::DIST_SCAN_SLOTLENGHT"
#define GESTURE_ID               "cOactuators::GESTUREID"
#define DICE_ID               "cOactuators::DiceID"
#define DRIFT_START_SPEED    "cOactuators::DRIFT_SPEED"
#define DRIFT_START_STEER    "cOactuators::DRIFT_STEER"
/// Create filter shell
ADTF_FILTER_PLUGIN("Oactuators", OID_ADTF_OPEN_ACTUATORS_FILTER, cOactuators);


cOactuators::cOactuators(const tChar* __info):cFilter(__info)
{
    SetPropertyBool("Debug Output to Console",false);
    SetPropertyBool("Odd --> True / Even --> False",true);
    SetPropertyFloat(DISTANCE_SCAN_SLOT_LENGTH,0.75);
    SetPropertyBool(DISTANCE_SCAN_SLOT_LENGTH NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(DISTANCE_SCAN_SLOT_LENGTH NSSUBPROP_DESCRIPTION, "slot length");
    SetPropertyFloat(GESTURE_ID,0);
    SetPropertyBool(GESTURE_ID NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(GESTURE_ID NSSUBPROP_DESCRIPTION, "GESTURE ID");
    SetPropertyFloat(DICE_ID,0);
    SetPropertyBool(DICE_ID NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(DICE_ID NSSUBPROP_DESCRIPTION, "dice ID");
    SetPropertyFloat(DRIFT_START_SPEED,-70);
    SetPropertyBool(DRIFT_START_SPEED NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(DRIFT_START_SPEED NSSUBPROP_DESCRIPTION, "DRIFT SPEED");
    SetPropertyFloat(DRIFT_START_STEER,80);
    SetPropertyBool(DRIFT_START_STEER NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(DRIFT_START_STEER NSSUBPROP_DESCRIPTION, "DRIFT STEER");
}

cOactuators::~cOactuators()
{

}

tResult cOactuators::Init(tInitStage eStage, __exception)
{
    // never miss calling the parent implementation!!
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))

    // in StageFirst you can create and register your static pins.
    if (eStage == StageFirst)
    {
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



                        //create pin for start_test pin input
                tChar const * strDescSignalstart_test = pDescManager->GetMediaDescription("tBoolSignalValue");
                RETURN_IF_POINTER_NULL(strDescSignalstart_test);
                cObjectPtr<IMediaType> pTypeSignalstart_test = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescSignalstart_test, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
                RETURN_IF_FAILED(pTypeSignalstart_test->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescStart_test));
                RETURN_IF_FAILED(m_iStart_test.Create("start_test", pTypeSignalstart_test, static_cast<IPinEventSink*> (this)));
                RETURN_IF_FAILED(RegisterPin(&m_iStart_test));
        //Input pin for Dice Input of Classifier
                tChar const * strDescSignalDice = pDescManager->GetMediaDescription("tSignalValue");
                RETURN_IF_POINTER_NULL(strDescSignalDice);
                cObjectPtr<IMediaType> pTypeSignalDice = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalDice, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
                RETURN_IF_FAILED(pTypeSignalDice->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignalInputDice));
                RETURN_IF_FAILED(m_iDice.Create("Dice Input", pTypeSignalDice, static_cast<IPinEventSink*> (this)));
                RETURN_IF_FAILED(RegisterPin(&m_iDice));
                //Input pin for Opp_Slot Input of Classifier
                        tChar const * strDescSignalOpp_Slot = pDescManager->GetMediaDescription("tSignalValue");
                        RETURN_IF_POINTER_NULL(strDescSignalOpp_Slot);
                        cObjectPtr<IMediaType> pTypeSignalOpp_Slot = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalOpp_Slot, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
                        RETURN_IF_FAILED(pTypeSignalOpp_Slot->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignalInputOpp_Slot));
                        RETURN_IF_FAILED(m_iOpp_Slot.Create("Opp_Slot Input", pTypeSignalOpp_Slot, static_cast<IPinEventSink*> (this)));
                        RETURN_IF_FAILED(RegisterPin(&m_iOpp_Slot));


                        //create pin for GESTURE_start_test pin input
                tChar const * strDescSignalGESTURE_start_test = pDescManager->GetMediaDescription("tBoolSignalValue");
                RETURN_IF_POINTER_NULL(strDescSignalGESTURE_start_test);
                cObjectPtr<IMediaType> pTypeSignalGESTURE_start_test = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescSignalGESTURE_start_test, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
                RETURN_IF_FAILED(pTypeSignalGESTURE_start_test->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescGESTURE_Start_test));
                RETURN_IF_FAILED(m_iGESTURE_Start_test.Create("GESTURE_start_test", pTypeSignalGESTURE_start_test, static_cast<IPinEventSink*> (this)));
                RETURN_IF_FAILED(RegisterPin(&m_iGESTURE_Start_test));

                //Input pin for GESTURE_ID Input of Classifier
                        tChar const * strDescSignalGESTURE_ID = pDescManager->GetMediaDescription("tGestureIDStruct");
                        RETURN_IF_POINTER_NULL(strDescSignalGESTURE_ID);
                        cObjectPtr<IMediaType> pTypeSignalGESTURE_ID = new cMediaType(0, 0, 0, "tGestureIDStruct", strDescSignalGESTURE_ID, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
                        RETURN_IF_FAILED(pTypeSignalGESTURE_ID->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignalInputGESTURE_ID));
                        RETURN_IF_FAILED(m_iGESTURE_ID.Create("GESTURE_ID", pTypeSignalGESTURE_ID, static_cast<IPinEventSink*> (this)));
                        RETURN_IF_FAILED(RegisterPin(&m_iGESTURE_ID));
                /*
        //Input pin for GESTURE_ID Input of Classifier
                tChar const * strDescSignalGESTURE_ID = pDescManager->GetMediaDescription("tSignalValue");
                RETURN_IF_POINTER_NULL(strDescSignalGESTURE_ID);
                cObjectPtr<IMediaType> pTypeSignalGESTURE_ID = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalGESTURE_ID, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
                RETURN_IF_FAILED(pTypeSignalGESTURE_ID->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignalInputGESTURE_ID));
                RETURN_IF_FAILED(m_iGESTURE_ID.Create("GESTURE_ID", pTypeSignalGESTURE_ID, static_cast<IPinEventSink*> (this)));
                RETURN_IF_FAILED(RegisterPin(&m_iGESTURE_ID));
*/
                //create pin forCar_Speed input
                        tChar const * strDescSignalCar_Speed = pDescManager->GetMediaDescription("tSignalValue");
                        RETURN_IF_POINTER_NULL(strDescSignalCar_Speed);
                        cObjectPtr<IMediaType> pTypeSignalCar_Speed = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalCar_Speed, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
                        RETURN_IF_FAILED(pTypeSignalCar_Speed->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescCar_Speed));
                        RETURN_IF_FAILED(m_iCar_Speed.Create("Car_Speed", pTypeSignalCar_Speed, static_cast<IPinEventSink*> (this)));
                        RETURN_IF_FAILED(RegisterPin(&m_iCar_Speed));

        //create pin for Distance over all input
                tChar const * strDescSignaldistanceoverall = pDescManager->GetMediaDescription("tSignalValue");
                RETURN_IF_POINTER_NULL(strDescSignaldistanceoverall);
                cObjectPtr<IMediaType> pTypeSignaldistanceoverall = new cMediaType(0, 0, 0, "tSignalValue", strDescSignaldistanceoverall, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
                RETURN_IF_FAILED(pTypeSignaldistanceoverall->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescdistanceoverall));
                RETURN_IF_FAILED(m_iDistanceOverall.Create("Distance_Overall", pTypeSignaldistanceoverall, static_cast<IPinEventSink*> (this)));
                RETURN_IF_FAILED(RegisterPin(&m_iDistanceOverall));
                // Lane_Information
                tChar const * strDescSignallanechange = pDescManager->GetMediaDescription("tCheckTrafficForCrossing");
                RETURN_IF_POINTER_NULL(strDescSignallanechange);
                cObjectPtr<IMediaType> pTypeSignallanechange = new cMediaType(0, 0, 0, "tCheckTrafficForCrossing", strDescSignallanechange, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
                RETURN_IF_FAILED(pTypeSignallanechange->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDesclanechange));
                RETURN_IF_FAILED(m_olanechange.Create("Lane_Information", pTypeSignallanechange, static_cast<IPinEventSink*> (this)));
                RETURN_IF_FAILED(RegisterPin(&m_olanechange));


                // Input - RoadSign
                tChar const * strRoadSign = pDescManager->GetMediaDescription("tRoadSignExt");
                RETURN_IF_POINTER_NULL(strRoadSign);
                cObjectPtr<IMediaType> pTypeRoadSign = new cMediaType(0, 0, 0, "tRoadSignExt", strRoadSign, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
                RETURN_IF_FAILED(pTypeRoadSign->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionInputRoadSign));
                RETURN_IF_FAILED(m_oInputRoadSign.Create("RoadSign_Ext", pTypeRoadSign, static_cast<IPinEventSink*> (this)));
                RETURN_IF_FAILED(RegisterPin(&m_oInputRoadSign));

        //create pin for lanefollower steering
                tChar const * strDescSignalLane_steer = pDescManager->GetMediaDescription("tSignalValue");
                RETURN_IF_POINTER_NULL(strDescSignalLane_steer);
                cObjectPtr<IMediaType> pTypeSignalLane_steer = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalLane_steer, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
                RETURN_IF_FAILED(pTypeSignalLane_steer->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescLane_steer));
                RETURN_IF_FAILED(m_iLane_steer.Create("Lane_steer", pTypeSignalLane_steer, static_cast<IPinEventSink*> (this)));
                RETURN_IF_FAILED(RegisterPin(&m_iLane_steer));
        // create pin for Infos about Stop Line
                tChar const * strDescStopLine = pDescManager->GetMediaDescription("tStoplineStruct");
                RETURN_IF_POINTER_NULL(strDescStopLine);
                cObjectPtr<IMediaType> pTypeSignalStopLine = new cMediaType(0, 0, 0, "tStoplineStruct", strDescStopLine, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
                RETURN_IF_FAILED(pTypeSignalStopLine->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescStopLine));
                RETURN_IF_FAILED(m_iStopLine.Create("StopLine", pTypeSignalStopLine, static_cast<IPinEventSink*> (this)));
                RETURN_IF_FAILED(RegisterPin(&m_iStopLine));
    //Output pin for slot number
            tChar const * strDescSignalSlot = pDescManager->GetMediaDescription("tSignalValue");
            RETURN_IF_POINTER_NULL(strDescSignalSlot);
            cObjectPtr<IMediaType> pTypeSignalSlot = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalSlot, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
            RETURN_IF_FAILED(pTypeSignalSlot->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignalOutputSlot));
            RETURN_IF_FAILED(m_oSlot.Create("Slot Number", pTypeSignalSlot, static_cast<IPinEventSink*> (this)));
            RETURN_IF_FAILED(RegisterPin(&m_oSlot));
            // Output - Lanefollow_Start_test
                            tChar const * strDescSignalLanefollow_Start_test = pDescManager->GetMediaDescription("tBoolSignalValue"); //tBoolSignalValue
                            RETURN_IF_POINTER_NULL(strDescSignalLanefollow_Start_test);
                            cObjectPtr<IMediaType> pTypeSignalLanefollow_Start_test = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescSignalLanefollow_Start_test, 	IMediaDescription::MDF_DDL_DEFAULT_VERSION);
                            RETURN_IF_FAILED(pTypeSignalLanefollow_Start_test->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionOutputLanefollow_Start_test));
                            RETURN_IF_FAILED(m_oOutputLanefollow_Start_test.Create("Lanefollow Start test", pTypeSignalLanefollow_Start_test, static_cast<IPinEventSink*> (this)));
                            RETURN_IF_FAILED(RegisterPin(&m_oOutputLanefollow_Start_test));

                            //create pin for steering signal output
                                            tChar const * strDescSignalSteering_Drift = pDescManager->GetMediaDescription("tSignalValue");
                                            RETURN_IF_POINTER_NULL(strDescSignalSteering_Drift);
                                            cObjectPtr<IMediaType> pTypeSignalSteering_Drift = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalSteering_Drift, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
                                            RETURN_IF_FAILED(pTypeSignalSteering_Drift->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionOutputSteering_Drift));
                                            RETURN_IF_FAILED(m_oOutputSteering_Drift.Create("Steering_drift_direct", pTypeSignalSteering_Drift, static_cast<IPinEventSink*> (this)));
                                            RETURN_IF_FAILED(RegisterPin(&m_oOutputSteering_Drift));
                            //create pin for speed signal output
                                            tChar const * strDescSignalaccelerate_Drift = pDescManager->GetMediaDescription("tSignalValue");
                                            RETURN_IF_POINTER_NULL(strDescSignalaccelerate_Drift);
                                            cObjectPtr<IMediaType> pTypeSignalaccelerate_Drift = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalaccelerate_Drift, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
                                            RETURN_IF_FAILED(pTypeSignalaccelerate_Drift->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionOutputAcceleration_Drift));
                                            RETURN_IF_FAILED(m_oOutputAcceleration_Drift.Create("accelerate_drift_direct", pTypeSignalaccelerate_Drift, static_cast<IPinEventSink*> (this)));
                                            RETURN_IF_FAILED(RegisterPin(&m_oOutputAcceleration_Drift));
//create pin for steering signal output
                tChar const * strDescSignalSteering = pDescManager->GetMediaDescription("tSignalValue");
                RETURN_IF_POINTER_NULL(strDescSignalSteering);
                cObjectPtr<IMediaType> pTypeSignalSteering = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalSteering, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
                RETURN_IF_FAILED(pTypeSignalSteering->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionOutputSteering));
                RETURN_IF_FAILED(m_oOutputSteering.Create("Steering", pTypeSignalSteering, static_cast<IPinEventSink*> (this)));
                RETURN_IF_FAILED(RegisterPin(&m_oOutputSteering));
//create pin for speed signal output
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

// Output - Break_Light
                tChar const * strDescSignalBreak_Light = pDescManager->GetMediaDescription("tBoolSignalValue"); //tBoolSignalValue
                RETURN_IF_POINTER_NULL(strDescSignalBreak_Light);
                cObjectPtr<IMediaType> pTypeSignalBreak_Light = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescSignalBreak_Light, 	IMediaDescription::MDF_DDL_DEFAULT_VERSION);
                RETURN_IF_FAILED(pTypeSignalBreak_Light->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionOutputBreak_Light));
                RETURN_IF_FAILED(m_oOutputBreak_Light.Create("Break Light", pTypeSignalBreak_Light, static_cast<IPinEventSink*> (this)));
                RETURN_IF_FAILED(RegisterPin(&m_oOutputBreak_Light));
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

    // Output - Finish
                    tChar const * strDescSignalFinish = pDescManager->GetMediaDescription("tBoolSignalValue"); //tBoolSignalValue
                    RETURN_IF_POINTER_NULL(strDescSignalFinish);
                    cObjectPtr<IMediaType> pTypeSignalFinish = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescSignalFinish, 	IMediaDescription::MDF_DDL_DEFAULT_VERSION);
                    RETURN_IF_FAILED(pTypeSignalFinish->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionOutputFinish));
                    RETURN_IF_FAILED(m_oOutputFinish.Create("Finish", pTypeSignalFinish, static_cast<IPinEventSink*> (this)));
                    RETURN_IF_FAILED(RegisterPin(&m_oOutputFinish));



    }
    else if (eStage == StageNormal)
    {
        m_bDebugModeEnabled = GetPropertyBool("Debug Output to Console");
        m_bOdd_start = GetPropertyBool("Odd --> True / Even --> False");
        m_fDiceID=DiceID;
        m_fGESTURE_ID=GESTUREID;
        m_bStart=tFalse;
        m_b_GESTURE_Start=tFalse;
        m_f32RoadSignDistanceX = 0;
        m_f32RoadSignDistanceY = 0;
        m_f32ArrayRoadSignDistanceY[0] = 0;
        m_f32ArrayRoadSignDistanceY[1] = 0;
        m_f32ArrayRoadSignDistanceY[2] = 0;
        m_f32ArrayRoadSignDistanceX[0] = 0;
        m_f32ArrayRoadSignDistanceX[1] = 0;
        m_f32ArrayRoadSignDistanceX[2] = 0;
        m_iDestinationTrafficSignID =0;
        m_bRoadSignDetected = tFalse;
        m_iCurrentTrafficSignID =0;
    }
    else if (eStage == StageGraphReady)
    {
       m_bDebugModeEnabled = GetPropertyBool("Debug Output to Console");
       m_bOdd_start = GetPropertyBool("Odd --> True / Even --> False");
       m_fDiceID=DiceID;
       m_fGESTURE_ID=GESTUREID;
    }

    RETURN_NOERROR;
}

tResult cOactuators::Shutdown(tInitStage eStage, __exception)
{

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
tResult cOactuators::PropertyChanged(const char* strProperty)
{
        ReadProperties(strProperty);
        RETURN_NOERROR;
}

tResult cOactuators::ReadProperties(const tChar* strPropertyName)
{

//check property
                if (NULL == strPropertyName || cString::IsEqual(strPropertyName, DISTANCE_SCAN_SLOT_LENGTH))
        {
                DIST_SCAN_SLOTLENGHT = static_cast<tFloat32> (GetPropertyFloat(DISTANCE_SCAN_SLOT_LENGTH));
        }
                if (NULL == strPropertyName || cString::IsEqual(strPropertyName, DICE_ID))
        {
                DiceID = static_cast<tFloat32> (GetPropertyFloat(DICE_ID));
        }
                if (NULL == strPropertyName || cString::IsEqual(strPropertyName, GESTURE_ID))
        {
                GESTUREID = static_cast<tFloat32> (GetPropertyFloat(GESTURE_ID));
        }
                if (NULL == strPropertyName || cString::IsEqual(strPropertyName, DRIFT_START_SPEED))
        {
                DRIFT_SPEED = static_cast<tFloat32> (GetPropertyFloat(DRIFT_START_SPEED));
        }
                if (NULL == strPropertyName || cString::IsEqual(strPropertyName, DRIFT_START_STEER))
        {
                DRIFT_STEER = static_cast<tFloat32> (GetPropertyFloat(DRIFT_START_STEER));
        }

        RETURN_NOERROR;
}
tResult cOactuators::OnPinEvent(IPin* pSource,
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
        if (pSource == &m_iStart_test)
                {
                    //LOG_INFO(cString::Format("Start_test bool input"));
                    cObjectPtr<IMediaCoder> pCoderInput;
                    RETURN_IF_FAILED(m_pDescStart_test->Lock(pMediaSample, &pCoderInput));
                    pCoderInput->Get("bValue", (tVoid*)&m_bStart);
                    m_pDescStart_test->Unlock(pCoderInput);

                    if(m_bDebugModeEnabled) LOG_INFO(cString::Format("Start_test bool input %d", m_bStart));
                    m_bWake_Up=m_bStart;

                    if(!m_bStart)
                    {
                        m_b_GESTURE_Start=tFalse;
                        stop_time = _clock->GetStreamTime();
                                        m_bTransmitstop=tTrue;
                                        if (m_bTransmitstop)
                                        {
                                            m_fAccelerationOutput=0;
                                            m_fSteeringOutput=0;
                                            TransmitFloatValue(&m_oOutputSteering, m_fSteeringOutput, 0);
                                            TransmitFloatValue(&m_oOutputAcceleration, m_fAccelerationOutput, 0);
                                             m_iStateOfGame=SOG_NOSTART;
                                            m_bHazard_Light= tFalse;
                                            m_bBack_Light= tFalse;
                                            m_bHead_Light= tFalse;
                                            m_bBreak_Light= tFalse;
                                            m_bTurnRight_Light=tFalse;
                                            m_bTurnLeft_Light=tFalse;
                                            TransmitBoolValue(&m_oOutputTurnLeft_Light, m_bTurnLeft_Light, 0);
                                            TransmitBoolValue(&m_oOutputTurnRight_Light, m_bTurnRight_Light, 0);
                                        TransmitBoolValue(&m_oOutputHazard_Light, m_bHazard_Light, 0);
                                        TransmitBoolValue(&m_oOutputBack_Light, m_bBack_Light, 0);
                                        TransmitBoolValue(&m_oOutputHead_Light, m_bHead_Light, 0);
                                        TransmitBoolValue(&m_oOutputBreak_Light, m_bBreak_Light, 0);
                                        }

                    }
                TransmitBoolValue(&m_oOutputLanefollow_Start_test, m_bStart, 0); // send start signal to lanefollower
                }
                else if (pSource == &m_iGESTURE_Start_test)
                {
                    //LOG_INFO(cString::Format("GESTURE_Start_test bool input"));
                    cObjectPtr<IMediaCoder> pCoderInput;
                    RETURN_IF_FAILED(m_pDescGESTURE_Start_test->Lock(pMediaSample, &pCoderInput));
                    pCoderInput->Get("bValue", (tVoid*)&m_b_GESTURE_Start);
                    m_pDescGESTURE_Start_test->Unlock(pCoderInput);
                    if(!m_b_GESTURE_Start)
                    {
                        m_bStart=tFalse;
                        stop_time = _clock->GetStreamTime();
                        m_bTransmitstop=tTrue;
                        if (m_bTransmitstop)
                        {
                            m_fAccelerationOutput=0;
                            m_fSteeringOutput=0;
                            TransmitFloatValue(&m_oOutputSteering, m_fSteeringOutput, 0);
                            TransmitFloatValue(&m_oOutputAcceleration, m_fAccelerationOutput, 0);
                             m_iStateOfGame=SOG_GESTURE_NO_START;
                            m_bHazard_Light= tFalse;
                            m_bBack_Light= tFalse;
                            m_bHead_Light= tFalse;
                            m_bBreak_Light= tFalse;

                            m_bTurnRight_Light=tFalse;
                            m_bTurnLeft_Light=tFalse;
                            TransmitBoolValue(&m_oOutputTurnLeft_Light, m_bTurnLeft_Light, 0);
                            TransmitBoolValue(&m_oOutputTurnRight_Light, m_bTurnRight_Light, 0);
                        TransmitBoolValue(&m_oOutputHazard_Light, m_bHazard_Light, 0);
                        TransmitBoolValue(&m_oOutputBack_Light, m_bBack_Light, 0);
                        TransmitBoolValue(&m_oOutputHead_Light, m_bHead_Light, 0);
                        TransmitBoolValue(&m_oOutputBreak_Light, m_bBreak_Light, 0);

                        }

                    }

                }
                // Input signal at DiceID
                 if (pSource == &m_iDice)
                {
                        cObjectPtr<IMediaCoder> pCoderInput;
                        RETURN_IF_FAILED(m_pCoderDescSignalInputDice->Lock(pMediaSample, &pCoderInput));
                        pCoderInput->Get("f32Value", (tVoid*)&m_fDiceID);
                        pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timestamp);
                        m_pCoderDescSignalInputDice->Unlock(pCoderInput);

                }
                 // Input signal at DiceID
                  if (pSource == &m_iOpp_Slot)
                 {
                         cObjectPtr<IMediaCoder> pCoderInput;
                         RETURN_IF_FAILED(m_pCoderDescSignalInputOpp_Slot->Lock(pMediaSample, &pCoderInput));
                         pCoderInput->Get("f32Value", (tVoid*)&m_fOpponent_SLOT_ID);
                         pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timestamp);
                         m_pCoderDescSignalInputOpp_Slot->Unlock(pCoderInput);

                 }
                // Input signal at GESTURE_ID
                 if (pSource == &m_iGESTURE_ID)
                {
                        tInt32 i32InputArray[8];

                        cObjectPtr<IMediaCoder> pCoderInput;
                        RETURN_IF_FAILED(m_pCoderDescSignalInputGESTURE_ID->Lock(pMediaSample, &pCoderInput));
                        pCoderInput->Get("i32ArrayGestureID", (tVoid*)&i32InputArray);
                        m_pCoderDescSignalInputGESTURE_ID->Unlock(pCoderInput);

                     /*
                        cObjectPtr<IMediaCoder> pCoderInput;
                        RETURN_IF_FAILED(m_pCoderDescSignalInputGESTURE_ID->Lock(pMediaSample, &pCoderInput));
                        pCoderInput->Get("f32Value", (tVoid*)&m_fGESTURE_ID);
                        pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timestamp);
                        m_pCoderDescSignalInputGESTURE_ID->Unlock(pCoderInput);
                                                //m_fGESTURE_ID = m_fGESTURE_ID-48;

                                                */
                }
                 // Input signal at Distance Overall
                  if (pSource == &m_iCar_Speed)
                 {
                         cObjectPtr<IMediaCoder> pCoderInput;
                         RETURN_IF_FAILED(m_pDescCar_Speed->Lock(pMediaSample, &pCoderInput));
                         pCoderInput->Get("f32Value", (tVoid*)&m_fCar_Speed);
                         pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timestamp);
                         m_pDescCar_Speed->Unlock(pCoderInput);

                 }
                // Input signal at Distance Overall
                 if (pSource == &m_iDistanceOverall)
                {
                        cObjectPtr<IMediaCoder> pCoderInput;
                        RETURN_IF_FAILED(m_pDescdistanceoverall->Lock(pMediaSample, &pCoderInput));
                        pCoderInput->Get("f32Value", (tVoid*)&m_fDistanceOverall);
                        pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timestamp);
                        m_pDescdistanceoverall->Unlock(pCoderInput);

                }

                 if (pSource == &m_oInputRoadSign)
                 {
                     //__adtf_sample_read_lock_mediadescription(m_pDescriptionInputTrafficSign,pMediaSample,pCoderInput);
			 cObjectPtr<IMediaCoder> pCoderInput;
                         RETURN_IF_FAILED(m_pDescriptionInputRoadSign->Lock(pMediaSample, &pCoderInput));
                         pCoderInput->Get("i16Identifier", (tVoid*)&m_iCurrentTrafficSignID);
                         pCoderInput->Get("f32Imagesize", (tVoid*)&m_fTrafficSignImageSize);
                         pCoderInput->Get("af32TVec", (tVoid*)&m_f32ArrayRoadSignDistanceY);
                         pCoderInput->Get("af32RVec", (tVoid*)&m_f32ArrayRoadSignDistanceX);
                         m_pDescriptionInputRoadSign->Unlock(pCoderInput);

                         m_f32RoadSignDistanceY = m_f32ArrayRoadSignDistanceY[2];
                         m_f32RoadSignDistanceX = m_f32ArrayRoadSignDistanceX[2];
                         // LOG_INFO(adtf_util::cString::Format("m_f32RoadSignDistanceY %f",m_f32RoadSignDistanceY));


                         f32Distance2RoadSign = sqrt((m_f32RoadSignDistanceY*m_f32RoadSignDistanceY) + (m_f32RoadSignDistanceX*m_f32RoadSignDistanceX));
                         if(f32Distance2RoadSign < 1 && f32Distance2RoadSign > 0 )
                         {
                             m_bRoadSignDetected = tTrue;
                             m_iCurrentTrafficSignID =m_iCurrentTrafficSignID;
                           }

                         else
                         {
                            m_bRoadSignDetected = tFalse;
                            m_iCurrentTrafficSignID =0;
                         }
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

                 if (pSource == &m_iLane_steer)
                {
                        cObjectPtr<IMediaCoder> pCoderInput;
                        RETURN_IF_FAILED(m_pDescLane_steer->Lock(pMediaSample, &pCoderInput));
                        pCoderInput->Get("f32Value", (tVoid*)&m_fLane_steer);
                        pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timestamp);
                        m_pDescLane_steer->Unlock(pCoderInput);

                }
                 if (pSource == &m_iStopLine)
                {
                        cObjectPtr<IMediaCoder> pCoderInput;
                        RETURN_IF_FAILED(m_pDescStopLine->Lock(pMediaSample, &pCoderInput));
                        pCoderInput->Get("bValue", (tVoid*)&m_bLine_detection);
                        pCoderInput->Get("f32Distance", (tVoid*)&m_fLine_distance);
                        pCoderInput->Get("f32Orientation", (tVoid*)&m_fOrientation2StopLine);
                        m_pDescStopLine->Unlock(pCoderInput);

                }

          // stop signal
                 if (m_bStart || m_b_GESTURE_Start)
                             {


                                         ProcessManeuver();
                                         if(m_iStateOfGame==SOG_GESTURE_DRIFT_LEFT || m_iStateOfGame==SOG_GESTURE_DRIFT_LEFT_INF || m_iStateOfGame==SOG_GESTURE_DRIFT_RIGHT || m_iStateOfGame==SOG_GESTURE_DRIFT_RIGHT_INF )
                                         {
                                             TransmitFloatValue(&m_oOutputSteering_Drift, m_fSteeringOutput, 0);
                                             TransmitFloatValue(&m_oOutputAcceleration_Drift, m_fAccelerationOutput, 0);
                                         }
                                         else
                                         {
                                         TransmitFloatValue(&m_oOutputSteering, m_fSteeringOutput, 0);
                                         TransmitFloatValue(&m_oOutputAcceleration, m_fAccelerationOutput, 0);
                                         }


                                         TransmitBoolValue(&m_oOutputHead_Light, m_bHead_Light, 0);

                                         TransmitBoolValue(&m_oOutputBreak_Light, m_bBreak_Light, 0);
                                        // TransmitOutput(m_fSteeringOutput, m_fAccelerationOutput);
                             }





            else
        {

                if(m_bTransmitstop)
                {
         // LOG_INFO(cString::Format("Start bool false"));
                m_fAccelerationOutput=0;
                m_fSteeringOutput=0;
                if(!m_bStart) m_iStateOfGame=SOG_NOSTART;
                else if(!m_b_GESTURE_Start) m_iStateOfGame=SOG_GESTURE_NO_START;
               // TransmitOutput(m_fSteeringOutput,m_fAccelerationOutput);
                m_bHazard_Light = tFalse;
                m_bBack_Light = tFalse;
                m_bHead_Light = tFalse;
                m_bBreak_Light = tFalse;
                TransmitFloatValue(&m_oOutputSteering, m_fSteeringOutput, 0);
                TransmitFloatValue(&m_oOutputAcceleration, m_fAccelerationOutput, 0);
                TransmitBoolValue(&m_oOutputHazard_Light, m_bHazard_Light, 0);
                TransmitBoolValue(&m_oOutputBack_Light, m_bBack_Light, 0);
                TransmitBoolValue(&m_oOutputHead_Light, m_bHead_Light, 0);
                TransmitBoolValue(&m_oOutputBreak_Light, m_bBreak_Light, 0);

                m_bTransmitstop= tFalse;
                }
        }

    }
}
    RETURN_NOERROR;
}
tResult cOactuators::ProcessManeuver()
{
switch(m_iStateOfGame)
                {

                LOG_INFO(cString::Format("Gaming State : %d",m_iStateOfGame));
                case SOG_NOSTART:
                        {

                                LOG_INFO(cString::Format("Step 0:SOG_NOSTART"));

                                //reset finish flag
                                m_bFinished=tFalse;
                                m_iGameslot_Counter=0;
                                m_iDiceroll_Counter=0;
                                m_fOpponent_SLOT_ID=0;
                                m_iDestinationTrafficSignID =0;
                                m_iCurrentTrafficSignID =0;
                                m_fDistanceOverall_Start=m_fDistanceOverall;
                                m_fDiceID=DiceID; // to change

                                // change state of turn when StartSignal is true
                                if(m_bStart)
                                        {
                                                if (m_bOdd_start)m_iModeOfGame=MOG_ODD_START;
                                                else if (!m_bOdd_start)m_iModeOfGame= MOG_EVEN_START;
                                                else m_iModeOfGame= MOG_NOT_SET;
                                                m_fDistanceOverall_Start=m_fDistanceOverall;
                                                m_iStateOfGame=SOG_DECISION;
                                                m_iDiceroll_Counter++;
                                                LOG_INFO(cString::Format("m_iDiceroll_Counter = %d",m_iDiceroll_Counter));
                                                m_iModulo_2 = m_iDiceroll_Counter%2;
                                                LOG_INFO(cString::Format("GO to SOG_DECISION"));
                                                 m_fcoverdist=DIST_SCAN_SLOTLENGHT;
                                                m_bImage_used=tFalse;


                                        }
                                else if(m_b_GESTURE_Start)
                                {
                                    //Hazard light off
                                    m_fGESTURE_ID=m_fGESTURE_ID;
                                    m_iStateOfGame=SOG_GESTURE_NO_START;

                                }
                                break;
                        }
                case SOG_DECISION:
                        {

                                LOG_INFO(cString::Format("1:SOG_DECISION"));
                                stop_time = _clock->GetStreamTime();
                                if(m_iModeOfGame==MOG_NOT_SET)
                                        {
                                                LOG_INFO(cString::Format("m_iModeOfGame=MOG_NOT_SET"));
                                                stop_time = _clock->GetStreamTime(); //store time data for waitning in next case
                                                m_iStateOfGame=SOG_MANUVER_FINISH;

                                        }
                                else if (m_iModeOfGame== MOG_ODD_START && m_iModulo_2==1)
                                        {
                                                stop_time = _clock->GetStreamTime(); //store time data for waitning in next case

                                                if(m_fDiceID==1) //one dot on dice
                                                {
                                                m_iStateOfGame=SOG_GOTO_SLOT;
                                                m_fDistanceOverall_Start=m_fDistanceOverall;
                                                LOG_INFO(cString::Format("m_iStateOfGame=SOG_GOTO_SLOT, m_iModeOfGame=MOG_ODD_START && m_iModulo_2=%d && m_fDiceID =%f",m_iModulo_2,m_fDiceID));
                                                m_iDestinationTrafficSignID =m_iGameslot_Counter+1;
                                                m_fOpponent_SLOT_ID =m_fOpponent_SLOT_ID;
                                                 LOG_INFO(cString::Format("m_iDestinationTrafficSignID=%f",m_iDestinationTrafficSignID));
                                                 LOG_INFO(cString::Format("m_iCurrentTrafficSignID=%f",m_iCurrentTrafficSignID));

                                                }
                                                else if(m_fDiceID==2) //two dot on dice
                                                {
                                                m_iStateOfGame=SOG_GOTO_SLOT;
                                                m_fDistanceOverall_Start=m_fDistanceOverall;
                                                m_fOpponent_SLOT_ID=m_fOpponent_SLOT_ID;
                                                LOG_INFO(cString::Format("m_iStateOfGame=SOG_GOTO_SLOT, m_iModeOfGame=MOG_ODD_START && m_iModulo_2=%d && m_fDiceID =%f",m_iModulo_2,m_fDiceID));
                                                 m_iDestinationTrafficSignID =m_iGameslot_Counter+2;
                                                  LOG_INFO(cString::Format("m_iDestinationTrafficSignID=%f",m_iDestinationTrafficSignID));
                                                  LOG_INFO(cString::Format("m_iCurrentTrafficSignID=%f",m_iCurrentTrafficSignID));
                                                }
                                                else if(m_fDiceID==3) //Red on Dice: yourself going back
                                                {
                                                m_iStateOfGame=SOG_BACKWORD;
                                                m_fDistanceOverall_Start=m_fDistanceOverall;
                                                m_fOpponent_SLOT_ID=m_fOpponent_SLOT_ID;
                                                if(m_iGameslot_Counter>0) m_iDestinationTrafficSignID =m_iGameslot_Counter-1;
                                                LOG_INFO(cString::Format("m_iStateOfGame=SOG_BACKWORD, m_iModeOfGame=MOG_ODD_START && m_iModulo_2=%d && m_fDiceID =%f",m_iModulo_2,m_fDiceID));

                                                 LOG_INFO(cString::Format("m_iDestinationTrafficSignID=%f",m_iDestinationTrafficSignID));
                                                 LOG_INFO(cString::Format("m_iCurrentTrafficSignID=%f",m_iCurrentTrafficSignID));
                                                }
                                                else if(m_fDiceID==4) //Green on Dice : Opponent going back, Self: stay
                                                {
                                                m_iStateOfGame=SOG_MANUVER_FINISH;
                                                m_fDistanceOverall_Start=m_fDistanceOverall;
                                                LOG_INFO(cString::Format("m_iStateOfGame=SOG_MANUVER_FINISH, m_iModeOfGame=MOG_ODD_START && m_iModulo_2=%d && m_fDiceID =%f",m_iModulo_2,m_fDiceID));
                                                if(m_fOpponent_SLOT_ID>0)m_fOpponent_SLOT_ID=m_fOpponent_SLOT_ID-1;
                                                else m_fOpponent_SLOT_ID =0;

                                                //Go to make fun
                                                }
                                                else if(m_fDiceID==5) // Blue on Dice : you have one more chance
                                                {
                                                m_fDistanceOverall_Start=m_fDistanceOverall;
                                                m_fOpponent_SLOT_ID=m_fOpponent_SLOT_ID;
                                                m_iStateOfGame=SOG_MANUVER_FINISH;
                                                m_iModeOfGame=MOG_EVEN_START; // change mode from even to odd
                                                LOG_INFO(cString::Format("m_iStateOfGame=SOG_MANUVER_FINISH, m_iModeOfGame=MOG_ODD_START change to MOG_EVEN_START && m_iModulo_2=%d && m_fDiceID =%f",m_iModulo_2,m_fDiceID));

                                                }
                                                else if(m_fDiceID==6) //Orange on Dice : Opponent going forword
                                                {
                                                m_fDistanceOverall_Start=m_fDistanceOverall;
                                                m_iStateOfGame=SOG_MANUVER_FINISH;
                                                m_fOpponent_SLOT_ID=m_fOpponent_SLOT_ID+1;
                                                LOG_INFO(cString::Format("m_iStateOfGame=SOG_MANUVER_FINISH, m_iModeOfGame=MOG_ODD_START && m_iModulo_2=%d && m_fDiceID =%f",m_iModulo_2,m_fDiceID));
                                                //Go to congratulate opponent
                                                }
                                                else if(m_fDiceID==0)
                                                {
                                                    LOG_INFO(cString::Format("m_iModeOfGame=MOG_ODD_START && m_iModulo_2=%d && m_fDiceID =%f",m_iModulo_2,m_fDiceID));

                                                }

                                        }
                                else if (m_iModeOfGame== MOG_ODD_START && m_iModulo_2==0)
                                        {
                                                stop_time = _clock->GetStreamTime(); //store time data for waitning in next case

                                                if(m_fDiceID==1) //one dot on dice : but not your turn
                                                {
                                                m_iStateOfGame=SOG_MANUVER_FINISH;
                                                m_fDistanceOverall_Start=m_fDistanceOverall;
                                                m_fOpponent_SLOT_ID=m_fOpponent_SLOT_ID+1;

                                                LOG_INFO(cString::Format("m_iStateOfGame=SOG_MANUVER_FINISH, m_iModeOfGame=MOG_ODD_START && m_iModulo_2=%d && m_fDiceID =%f",m_iModulo_2,m_fDiceID));
                                                }
                                                else if(m_fDiceID==2) //two dot on dice :but not your turn
                                                {
                                                m_iStateOfGame=SOG_MANUVER_FINISH;
                                                m_fDistanceOverall_Start=m_fDistanceOverall;
                                                m_fOpponent_SLOT_ID=m_fOpponent_SLOT_ID+2;

                                                LOG_INFO(cString::Format("m_iStateOfGame=SOG_MANUVER_FINISH, m_iModeOfGame=MOG_ODD_START && m_iModulo_2=%d && m_fDiceID =%f",m_iModulo_2,m_fDiceID));
                                                }
                                                else if(m_fDiceID==3) //Red on Dice
                                                {
                                                m_iStateOfGame=SOG_MANUVER_FINISH;
                                                m_fDistanceOverall_Start=m_fDistanceOverall;
                                                if(m_fOpponent_SLOT_ID>0)m_fOpponent_SLOT_ID=m_fOpponent_SLOT_ID-1;

                                                LOG_INFO(cString::Format("m_iStateOfGame=SOG_MANUVER_FINISH, m_iModeOfGame=MOG_ODD_START && m_iModulo_2=%d && m_fDiceID =%f",m_iModulo_2,m_fDiceID));
                                                }
                                                else if(m_fDiceID==4) //Green on Dice : yourself going back
                                                {
                                                m_iStateOfGame=SOG_BACKWORD;
                                                m_fDistanceOverall_Start=m_fDistanceOverall;
                                                m_fOpponent_SLOT_ID=m_fOpponent_SLOT_ID;
                                                if(m_iGameslot_Counter>0) m_iDestinationTrafficSignID =m_iGameslot_Counter-1;
                                                LOG_INFO(cString::Format("m_iStateOfGame=SOG_BACKWORD, m_iModeOfGame=MOG_ODD_START && m_iModulo_2=%d && m_fDiceID =%f",m_iModulo_2,m_fDiceID));
                                                 LOG_INFO(cString::Format("m_iDestinationTrafficSignID=%f",m_iDestinationTrafficSignID));
                                                 LOG_INFO(cString::Format("m_iCurrentTrafficSignID=%f",m_iCurrentTrafficSignID));

                                                }
                                                else if(m_fDiceID==5) // Blue on Dice
                                                {
                                                m_iStateOfGame=SOG_MANUVER_FINISH;
                                                m_fDistanceOverall_Start=m_fDistanceOverall;
                                                m_iModeOfGame=MOG_EVEN_START; // change mode from even to odd
                                                LOG_INFO(cString::Format("m_iStateOfGame=SOG_MANUVER_FINISH m_iModeOfGame=MOG_ODD_START change to MOG_EVEN_START && m_iModulo_2=%d && m_fDiceID =%f",m_iModulo_2,m_fDiceID));
                                                m_fOpponent_SLOT_ID=m_fOpponent_SLOT_ID;
                                                }
                                                else if(m_fDiceID==6) //Orange on Dice :
                                                {
                                                m_iStateOfGame=SOG_GOTO_SLOT;
                                                m_fOpponent_SLOT_ID=m_fOpponent_SLOT_ID;
                                                m_fDistanceOverall_Start=m_fDistanceOverall;
                                               m_iDestinationTrafficSignID =m_iGameslot_Counter+1;
                                                LOG_INFO(cString::Format("m_iStateOfGame=SOG_GOTO_SLOT, m_iModeOfGame=MOG_ODD_START && m_iModulo_2=%d && m_fDiceID =%f",m_iModulo_2,m_fDiceID));
                                                LOG_INFO(cString::Format("m_iDestinationTrafficSignID=%f",m_iDestinationTrafficSignID));
                                                LOG_INFO(cString::Format("m_iCurrentTrafficSignID=%f",m_iCurrentTrafficSignID));


                                                }
                                                else if(m_fDiceID==0)
                                                {
                                                    LOG_INFO(cString::Format("m_iModeOfGame=MOG_ODD_START && m_iModulo_2=%d && m_fDiceID =%f",m_iModulo_2,m_fDiceID));
                                                }

                                        }
                                else if (m_iModeOfGame== MOG_EVEN_START && m_iModulo_2==1)
                                        {
                                         stop_time = _clock->GetStreamTime(); //store time data for waitning in next case

                                                if(m_fDiceID==1) //one dot on dice : but not your turn
                                                {
                                                m_iStateOfGame=SOG_MANUVER_FINISH;
                                                m_fDistanceOverall_Start=m_fDistanceOverall;
                                                m_fOpponent_SLOT_ID=m_fOpponent_SLOT_ID+1;
                                                LOG_INFO(cString::Format("m_iStateOfGame=SOG_MANUVER_FINISH, m_iModeOfGame=MOG_EVEN_START && m_iModulo_2=%d && m_fDiceID =%f",m_iModulo_2,m_fDiceID));
                                                }
                                                else if(m_fDiceID==2) //two dot on dice :but not your turn
                                                {
                                                m_iStateOfGame=SOG_MANUVER_FINISH;
                                                m_fDistanceOverall_Start=m_fDistanceOverall;
                                                m_fOpponent_SLOT_ID=m_fOpponent_SLOT_ID+2;
                                                LOG_INFO(cString::Format("m_iStateOfGame=SOG_MANUVER_FINISH, m_iModeOfGame=MOG_EVEN_START && m_iModulo_2=%d && m_fDiceID =%f",m_iModulo_2,m_fDiceID));
                                                }
                                                else if(m_fDiceID==3) //Red on Dice: yourself going back
                                                {
                                                m_iStateOfGame=SOG_MANUVER_FINISH;
                                                m_fDistanceOverall_Start=m_fDistanceOverall;
                                                if(m_fOpponent_SLOT_ID>0)m_fOpponent_SLOT_ID=m_fOpponent_SLOT_ID-1;
                                                LOG_INFO(cString::Format("m_iStateOfGame=SOG_MANUVER_FINISH, m_iModeOfGame=MOG_EVEN_START && m_iModulo_2=%d && m_fDiceID =%f",m_iModulo_2,m_fDiceID));
                                                LOG_INFO(cString::Format("m_iDestinationTrafficSignID=%f",m_iDestinationTrafficSignID));
                                                LOG_INFO(cString::Format("m_iCurrentTrafficSignID=%f",m_iCurrentTrafficSignID));
                                                }
                                                else if(m_fDiceID==4) //Green on Dice :
                                                {
                                                m_iStateOfGame=SOG_BACKWORD;
                                                m_fOpponent_SLOT_ID=m_fOpponent_SLOT_ID;
                                                m_fDistanceOverall_Start=m_fDistanceOverall;
                                                if(m_iGameslot_Counter>0) m_iDestinationTrafficSignID =m_iGameslot_Counter-1;
                                                LOG_INFO(cString::Format("m_iStateOfGame=SOG_BACKWORD, m_iModeOfGame=MOG_EVEN_START && m_iModulo_2=%d && m_fDiceID =%f",m_iModulo_2,m_fDiceID));
                                                LOG_INFO(cString::Format("m_iDestinationTrafficSignID=%f",m_iDestinationTrafficSignID));
                                                LOG_INFO(cString::Format("m_iCurrentTrafficSignID=%f",m_iCurrentTrafficSignID));


                                                }
                                                else if(m_fDiceID==5) // Blue on Dice : you have one more chance
                                                {
                                                m_iStateOfGame=SOG_MANUVER_FINISH;
                                                m_fOpponent_SLOT_ID=m_fOpponent_SLOT_ID;
                                                  m_iModeOfGame=MOG_ODD_START; // change mode from even to odd
                                                m_fDistanceOverall_Start=m_fDistanceOverall;

                                                LOG_INFO(cString::Format("m_iStateOfGame=SOG_MANUVER_FINISH, m_iModeOfGame=MOG_EVEN_START change to MOG_ODD_START && m_iModulo_2=%d && m_fDiceID =%f",m_iModulo_2,m_fDiceID));
                                                }
                                                else if(m_fDiceID==6) //Orange on Dice
                                                {
                                                    m_iStateOfGame=SOG_GOTO_SLOT;
                                                    m_fOpponent_SLOT_ID=m_fOpponent_SLOT_ID;
                                                    m_fDistanceOverall_Start=m_fDistanceOverall;
                                                   m_iDestinationTrafficSignID =m_iGameslot_Counter+1;

                                                LOG_INFO(cString::Format("m_iStateOfGame=SOG_GOTO_SLOT, m_iModeOfGame=MOG_EVEN_START && m_iModulo_2=%d && m_fDiceID =%f",m_iModulo_2,m_fDiceID));
                                                LOG_INFO(cString::Format("m_iDestinationTrafficSignID=%f",m_iDestinationTrafficSignID));
                                                LOG_INFO(cString::Format("m_iCurrentTrafficSignID=%f",m_iCurrentTrafficSignID));
                                                }
                                                else if(m_fDiceID==0)
                                                {
                                                    LOG_INFO(cString::Format("m_iModeOfGame=MOG_EVEN_START && m_iModulo_2=%d && m_fDiceID =%f",m_iModulo_2,m_fDiceID));

                                                }

                                        }
                                else if (m_iModeOfGame== MOG_EVEN_START && m_iModulo_2==0)
                                        {
                                         stop_time = _clock->GetStreamTime(); //store time data for waitning in next case
                                                LOG_INFO(cString::Format("m_iModeOfGame=MOG_EVEN_START && m_iModulo_2=%d",m_iModulo_2));
                                                if(m_fDiceID==1) //one dot on dice
                                                {
                                                m_iStateOfGame=SOG_GOTO_SLOT;
                                                 m_fOpponent_SLOT_ID=m_fOpponent_SLOT_ID;
                                                m_iDestinationTrafficSignID =m_iGameslot_Counter+1;
                                                LOG_INFO(cString::Format("m_iStateOfGame=SOG_GOTO_SLOT, m_iModeOfGame=MOG_EVEN_START && m_iModulo_2=%d && m_fDiceID =%f",m_iModulo_2,m_fDiceID));
                                                m_fDistanceOverall_Start=m_fDistanceOverall;
                                                LOG_INFO(cString::Format("m_iDestinationTrafficSignID=%f",m_iDestinationTrafficSignID));
                                                LOG_INFO(cString::Format("m_iCurrentTrafficSignID=%f",m_iCurrentTrafficSignID));
                                                }
                                                else if(m_fDiceID==2) //two dot on dice
                                                {
                                                m_iStateOfGame=SOG_GOTO_SLOT;
                                                 m_fOpponent_SLOT_ID=m_fOpponent_SLOT_ID;
                                                m_iDestinationTrafficSignID =m_iGameslot_Counter+2;
                                                m_fDistanceOverall_Start=m_fDistanceOverall;
                                                LOG_INFO(cString::Format("m_iStateOfGame=SOG_GOTO_SLOT, m_iModeOfGame=MOG_EVEN_START && m_iModulo_2=%d && m_fDiceID =%f",m_iModulo_2,m_fDiceID));
                                                LOG_INFO(cString::Format("m_iDestinationTrafficSignID=%f",m_iDestinationTrafficSignID));
                                                LOG_INFO(cString::Format("m_iCurrentTrafficSignID=%f",m_iCurrentTrafficSignID));
                                                }
                                                else if(m_fDiceID==3) //Red on Dice: yourself going back
                                                {
                                                m_iStateOfGame=SOG_BACKWORD;
                                                 m_fOpponent_SLOT_ID=m_fOpponent_SLOT_ID;
                                                m_fDistanceOverall_Start=m_fDistanceOverall;
                                                if(m_iGameslot_Counter>0) m_iDestinationTrafficSignID =m_iGameslot_Counter-1;

                                                LOG_INFO(cString::Format("m_iStateOfGame=SOG_BACKWORD, m_iModeOfGame=MOG_EVEN_START && m_iModulo_2=%d && m_fDiceID =%f",m_iModulo_2,m_fDiceID));
                                                LOG_INFO(cString::Format("m_iDestinationTrafficSignID=%f",m_iDestinationTrafficSignID));
                                                LOG_INFO(cString::Format("m_iCurrentTrafficSignID=%f",m_iCurrentTrafficSignID));

                                                }
                                                else if(m_fDiceID==4) //Green on Dice : Opponent going back, Self: stay
                                                {
                                                m_iStateOfGame=SOG_MANUVER_FINISH;
                                                m_fDistanceOverall_Start=m_fDistanceOverall;
                                                if( m_fOpponent_SLOT_ID>0) m_fOpponent_SLOT_ID=m_fOpponent_SLOT_ID-1;
                                                LOG_INFO(cString::Format("m_iStateOfGame=SOG_MANUVER_FINISH, m_iModeOfGame=MOG_EVEN_START && m_iModulo_2=%d && m_fDiceID =%f",m_iModulo_2,m_fDiceID));
                                                }
                                                else if(m_fDiceID==5) // Blue on Dice : you have one more chance
                                                {
                                                m_fDistanceOverall_Start=m_fDistanceOverall;
                                                 m_fOpponent_SLOT_ID=m_fOpponent_SLOT_ID;
                                                m_iStateOfGame=SOG_MANUVER_FINISH;
                                                m_iModeOfGame=MOG_ODD_START; // change mode from even to odd
                                                LOG_INFO(cString::Format("m_iStateOfGame=SOG_MANUVER_FINISH, m_iModeOfGame=MOG_EVEN_START change to MOG_ODD_START  && m_iModulo_2=%d && m_fDiceID =%f",m_iModulo_2,m_fDiceID));

                                                }
                                                else if(m_fDiceID==6) //Orange on Dice : Opponent going forword
                                                {
                                                m_fDistanceOverall_Start=m_fDistanceOverall;
                                                 m_fOpponent_SLOT_ID=m_fOpponent_SLOT_ID+1;
                                                m_iStateOfGame=SOG_MANUVER_FINISH;
                                                LOG_INFO(cString::Format("m_iStateOfGame=SOG_MANUVER_FINISH, m_iModeOfGame=MOG_EVEN_START && m_iModulo_2=%d && m_fDiceID =%f",m_iModulo_2,m_fDiceID));
                                                }
                                                else if(m_fDiceID==0)
                                                {
                                                    LOG_INFO(cString::Format("m_iModeOfGame=MOG_EVEN_START && m_iModulo_2=%d && m_fDiceID =%f",m_iModulo_2,m_fDiceID));
                                                }


                                        }
                                break;
                        }
                case SOG_GOTO_SLOT:
                        {
                            //LOG_INFO(cString::Format("4:SOG_BACKWORD"));
                            if((_clock->GetStreamTime() - stop_time)/1000000 < 0.5) //time for wait
                            {
                                    m_fSteeringOutput=0;		//accuator output
                                    m_fAccelerationOutput= 0;
                                    // Back light on
                                    //park light off
                            }
                            //--------------------%%%% if Road sign is in 0.5 meter %%%%%_________
                            else if(m_bRoadSignDetected && f32Distance2RoadSign<0.5  )
                            {
                                    //--------------**** if any slot detected *****---------------------
                               if(m_iCurrentTrafficSignID==m_iDestinationTrafficSignID)
                               {
                                    //--------------if it is destination slot ------------------
                                   if(m_fDistanceOverall-m_fDistanceOverall_Start <= m_fcoverdist)
                                   {

                                           m_fAccelerationOutput= 0.5;
                                           m_fSteeringOutput =m_fLane_steer;
                                           if(m_bLine_detection && m_fLine_distance<=70 && m_fLine_distance>=50 && !m_bImage_used)
                                           {

                                                   m_fDistanceOverall_Start=m_fDistanceOverall;
                                                   // Stop line detection
                                                   m_fcoverdist=((m_fLine_distance)/100)-0.25; // coverdistance update with line detection
                                                   m_bImage_used=tTrue;
                                                   m_iStateOfGame=SOG_GOTO_STOPLINE;
                                                   stop_time = _clock->GetStreamTime();

                                           }


                                   }
                                   else
                                   {
                                           //front lignt off

                                           m_iStateOfGame=SOG_GOTO_STOPLINE;
                                           stop_time = _clock->GetStreamTime();



                                   }
                                   //------------------------------------------------------------
                               }
                               else
                               {
                                   m_fAccelerationOutput= 0.5;
                                   m_fSteeringOutput =m_fLane_steer;
                                   m_fDistanceOverall_Start= m_fDistanceOverall;
                                   m_iStateOfGame=SOG_GOTO_SLOT;
                               }
                               //--------------------*********----*****----------------------------------
                            }
                               else
                               {
                                   m_fAccelerationOutput= 0.5;
                                   m_fSteeringOutput =m_fLane_steer;
                                    m_fDistanceOverall_Start= m_fDistanceOverall;
                                    m_iStateOfGame=SOG_GOTO_SLOT;
                               }

                              //-------------%%%%%-----%%%%%------------------------------------

                                break;
                        }
                case SOG_GOTO_STOPLINE:
                        {


                    if(m_fDistanceOverall-m_fDistanceOverall_Start <= m_fcoverdist)
                    {

                            m_fAccelerationOutput= 0.4;
                            m_fSteeringOutput =m_fLane_steer;
                            if(m_bLine_detection && m_fLine_distance<=55 && m_fLine_distance>=40)
                            {

                                    if (!m_bImage_used)
                                    {
                                    m_fDistanceOverall_Start=m_fDistanceOverall;
                                    LOG_INFO(cString::Format("2:SOG_FORWORD1 close loop not whhile line detection" ));
                                    m_fcoverdist=((m_fLine_distance)/100)-0.25; // coverdistance update with line detection
                                    m_bImage_used=tTrue;
                                    }
                            }


                    }
                    else
                    {
                            //front lignt off

                            m_iStateOfGame=SOG_MANUVER_FINISH;
                            stop_time = _clock->GetStreamTime();
                            m_iGameslot_Counter=m_iDestinationTrafficSignID;
                            m_bImage_used=tFalse;

                    }




                                break;
                        }
                case SOG_BACKWORD:
                        {


                                //LOG_INFO(cString::Format("4:SOG_BACKWORD"));
                                if((_clock->GetStreamTime() - stop_time)/1000000 < 0.5) //time for wait
                                {
                                        m_fSteeringOutput=0;		//accuator output
                                        m_fAccelerationOutput= 0;
                                        // Back light on
                                        //park light off
                                }
                                else if(m_fDistanceOverall-m_fDistanceOverall_Start <= m_fcoverdist+0.5 && m_iGameslot_Counter>0)
                                {

                                        m_fAccelerationOutput= -0.5;
                                        if(m_bTwolinedetection)m_fSteeringOutput =-m_fLane_steer;
                                        else m_fSteeringOutput =0.8*m_fLane_steer;

                                        if(m_fDistanceOverall-m_fDistanceOverall_Start >= m_fcoverdist && m_bLine_detection && m_fLine_distance<=55 && m_fLine_distance>=40)
                                        {
                                                //m_fSteeringOutput=15*(m_fOrientation2StopLine-90);		//accuator output
                                            if (!m_bImage_used)
                                            {
                                                m_fDistanceOverall_Start=m_fDistanceOverall;
                                                m_fcoverdist=m_fSlot_Length-((m_fLine_distance)/100)+0.5; // coverdistance update with line detection
                                            m_bImage_used=tTrue;
                                            LOG_INFO(cString::Format(" close loop 4:SOG_BACKWORD"));
                                            }


                                        }


                                }
                                else
                                {
                                        //front lignt off

                                        m_iStateOfGame=SOG_MANUVER_FINISH;
                                        if(m_iGameslot_Counter>0)m_iGameslot_Counter--;
                                        stop_time = _clock->GetStreamTime();
                                        m_fDistanceOverall_Start=m_fDistanceOverall;
                                }


                                break;
                        }
                case SOG_STAY:
                        {
                                m_fSteeringOutput=0;		//accuator output
                                m_fAccelerationOutput= 0;
                                if(m_bWake_Up)
                                {

                                    m_fDistanceOverall_Start=m_fDistanceOverall;
                                    m_iStateOfGame=SOG_DECISION;
                                    m_iDiceroll_Counter++;
                                    LOG_INFO(cString::Format("m_iDiceroll_Counter = %d",m_iDiceroll_Counter));
                                    m_iModulo_2 = m_iDiceroll_Counter%2;
                                    LOG_INFO(cString::Format("GO to SOG_DECISION"));
                                    m_fcoverdist=DIST_SCAN_SLOTLENGHT;
                                    m_fDiceID=DiceID; // to change
                                    m_bRoadSignDetected = tFalse;

                                }


                                break;
                        }
                case SOG_MANUVER_FINISH:
                        {
                                //park light on
                                LOG_INFO(cString::Format("6:SOG_MANUVER_FINISH"));
                                m_fCurrent_SLot=m_iGameslot_Counter;
                                LOG_INFO(cString::Format("Current slot = %d",m_iGameslot_Counter));
                                 LOG_INFO(cString::Format("Opponent slot = %d",m_fOpponent_SLOT_ID));
                                TransmitFloatValue(&m_oSlot, m_fCurrent_SLot, 0);
                                if(m_iGameslot_Counter<5)
                                {
                                    m_iStateOfGame=SOG_STAY;
                                    TransmitBoolValue(&m_oOutputFinish, m_bFinished, 0);
                                    m_fDistanceOverall_Start=m_fDistanceOverall;
                                    stop_time = _clock->GetStreamTime();
                                    m_bWake_Up = tFalse;
                                    m_bImage_used = tFalse;
                                    m_bRoadSignDetected = tFalse;
                                }
                                else
                                {
                                    m_iStateOfGame=SOG_GAME_WIN_FINISH;
                                m_fDistanceOverall_Start=m_fDistanceOverall;
                                stop_time = _clock->GetStreamTime();
                                m_bWake_Up = tFalse;
                                m_bImage_used = tFalse;

                                }
                                break;
                        }
                case SOG_GAME_WIN_FINISH:
                        {

                                LOG_INFO(cString::Format("7:SOG_GAME_WIN_FINISH"));
                                m_bFinished = tTrue;
                                m_bStart = tFalse;
                                m_b_GESTURE_Start= tFalse;
                                TransmitBoolValue(&m_oOutputFinish, m_bFinished, 0);

                                m_iStateOfGame=SOG_GESTURE_NO_START;
                                m_fGESTURE_ID=0;
                                m_fGESTURE_ID_OLD=0;
                                break;
                        }
                case SOG_GESTURE_NO_START:
                        {
                                LOG_INFO(cString::Format("6:SOG_GESTURE_NO_START"));
                                m_bGESTURE_Finished= tFalse;
                                // change state of turn when StartSignal is true
                                m_fGESTURE_ID=GESTUREID;

                                if(m_b_GESTURE_Start)
                                {

                                    m_fGESTURE_ID=m_fGESTURE_ID;
                                    m_iStateOfGame=SOG_GESTURE_DECISION;

                                }
                                else if(m_bStart)
                                {
                                        if (m_bOdd_start)m_iModeOfGame=MOG_ODD_START;
                                        else if (!m_bOdd_start)m_iModeOfGame= MOG_EVEN_START;
                                        else m_iModeOfGame= MOG_NOT_SET;
                                        m_fDistanceOverall_Start=m_fDistanceOverall;
                                        m_iStateOfGame=SOG_DECISION;
                                        m_iDiceroll_Counter++;
                                        LOG_INFO(cString::Format("m_iDiceroll_Counter = %d",m_iDiceroll_Counter));
                                        m_iModulo_2 = m_iDiceroll_Counter%2;
                                        LOG_INFO(cString::Format("GO to SOG_DECISION"));
                                         m_fcoverdist=DIST_SCAN_SLOTLENGHT;
                                        m_bImage_used=tFalse;


                                }

                                break;
                        }
                case SOG_GESTURE_DECISION:
                        {

                    m_bHazard_Light= tFalse;
                    m_bBack_Light= tFalse;
                    m_bHead_Light= tFalse;
                    m_bBreak_Light= tFalse;
                    m_bHazard_Light=tFalse;
                    m_bTurnRight_Light=tFalse;
                    m_bTurnLeft_Light=tFalse;
                    TransmitBoolValue(&m_oOutputTurnLeft_Light, m_bTurnLeft_Light, 0);
                    TransmitBoolValue(&m_oOutputTurnRight_Light, m_bTurnRight_Light, 0);
                    TransmitBoolValue(&m_oOutputHazard_Light, m_bHazard_Light, 0);
                    TransmitBoolValue(&m_oOutputBack_Light, m_bBack_Light, 0);
                    TransmitBoolValue(&m_oOutputHead_Light, m_bHead_Light, 0);
                    TransmitBoolValue(&m_oOutputBreak_Light, m_bBreak_Light, 0);
                    TransmitBoolValue(&m_oOutputHazard_Light, m_bHazard_Light, 0);
                    LOG_INFO(cString::Format("6:SOG_GESTURE_DECISION"));
                    m_fAccelerationOutput=0;
                    m_fSteeringOutput=0;
                    TransmitFloatValue(&m_oOutputSteering, m_fSteeringOutput, 0);
                    TransmitFloatValue(&m_oOutputAcceleration, m_fAccelerationOutput, 0);
                    m_fGESTURE_ID_OLD=m_fGESTURE_ID;
                    m_fGESTURE_ID=GESTUREID;
                                if(m_fGESTURE_ID==0)
                                {
                                    m_bHazard_Light=tTrue;
                                    TransmitBoolValue(&m_oOutputHazard_Light, m_bHazard_Light, 0);
                                    m_iStateOfGame=SOG_GESTURE_STAY;
                                     LOG_INFO(cString::Format("SOG_GESTURE_STAY"));
                                    //hazard lights on
                                }
                                else if(m_fGESTURE_ID==1)
                                {
                                    m_bHead_Light=tTrue;
                                    TransmitBoolValue(&m_oOutputHead_Light, m_bHead_Light, 0);
                                    m_iStateOfGame=SOG_GESTURE_FORWORD;
                                    LOG_INFO(cString::Format("SOG_GESTURE_FORWORD"));



                                    //head lights on
                                }
                                else if(m_fGESTURE_ID==2)
                                {
                                    m_bTurnLeft_Light=tTrue;
                                    TransmitBoolValue(&m_oOutputTurnLeft_Light, m_bTurnLeft_Light, 0);
                                    m_iStateOfGame=SOG_GESTURE_LEFT;
                                    LOG_INFO(cString::Format("SOG_GESTURE_LEFT"));


                                    //turn left lights on
                                }
                                else if(m_fGESTURE_ID==3)
                                {
                                    m_bTurnRight_Light=tTrue;
                                    TransmitBoolValue(&m_oOutputTurnRight_Light, m_bTurnRight_Light, 0);
                                    m_iStateOfGame=SOG_GESTURE_RIGHT;
                                      LOG_INFO(cString::Format("SOG_GESTURE_RIGHT"));

                                    //turn right lights on
                                }
                                else if(m_fGESTURE_ID==4)
                                {
                                    m_bBack_Light=tTrue;
                                    TransmitBoolValue(&m_oOutputBack_Light, m_bBack_Light, 0);
                                    m_iStateOfGame=SOG_GESTURE_BACKWORD;

                                    LOG_INFO(cString::Format("SOG_GESTURE_BACKWORD"));
                                    //back lights on
                                }
                                else if(m_fGESTURE_ID==5)
                                {
                                    m_bHead_Light=tTrue;
                                    TransmitBoolValue(&m_oOutputHead_Light, m_bHead_Light, 0);

                                    m_iStateOfGame=SOG_GESTURE_DRIFT_LEFT;
                                    m_fDistanceOverall_Start=m_fDistanceOverall;
                                    LOG_INFO(cString::Format("SOG_GESTURE_DRIFT_LEFT"));


                                    //break lights on
                                }
                                else if(m_fGESTURE_ID==6)
                                {
                                    m_bHead_Light=tTrue;
                                    TransmitBoolValue(&m_oOutputHead_Light, m_bHead_Light, 0);
                                    m_iStateOfGame=SOG_GESTURE_DRIFT_RIGHT;
                                    m_fDistanceOverall_Start=m_fDistanceOverall;
                                    LOG_INFO(cString::Format("SOG_GESTURE_DRIFT_RIGHT"));

                                    //break lights on
                                }
                                else if(m_fGESTURE_ID==7)
                                {
                                    m_iStateOfGame=SOG_GESTURE_FINISH;
                                    LOG_INFO(cString::Format("SOG_GESTURE_FINISH"));
                                    //all light off
                                }


                                break;
                        }
                case SOG_GESTURE_FORWORD:
                        {

                                m_fSteeringOutput=0;		//accuator output
                                m_fAccelerationOutput= 0.3;
                                m_fGESTURE_ID=GESTUREID;
                                if(m_fGESTURE_ID!=m_fGESTURE_ID_OLD)
                                {
                                    m_iStateOfGame=SOG_GESTURE_DECISION;
                                    m_fSteeringOutput=0;		//accuator output
                                    m_fAccelerationOutput= 0;
                                }

                                break;
                        }
                case SOG_GESTURE_LEFT:
                        {

                                m_fSteeringOutput=-80;		//accuator output
                                m_fAccelerationOutput= 0.4;
                                m_fGESTURE_ID=GESTUREID;
                                if(m_fGESTURE_ID!=m_fGESTURE_ID_OLD)
                                {
                                    m_iStateOfGame=SOG_GESTURE_DECISION;
                                    m_fSteeringOutput=0;		//accuator output
                                    m_fAccelerationOutput= 0;
                                }

                                break;
                        }
                case SOG_GESTURE_RIGHT:
                        {

                                m_fSteeringOutput=80;		//accuator output
                                m_fAccelerationOutput= 0.4;
                                m_fGESTURE_ID=GESTUREID;
                                if(m_fGESTURE_ID!=m_fGESTURE_ID_OLD)
                                {
                                    m_iStateOfGame=SOG_GESTURE_DECISION;
                                    m_fSteeringOutput=0;		//accuator output
                                    m_fAccelerationOutput= 0;
                                }

                                break;
                        }
                case SOG_GESTURE_BACKWORD:
                        {

                                m_fSteeringOutput=0;		//accuator output
                                m_fAccelerationOutput= -0.3;
                                m_fGESTURE_ID=GESTUREID;

                                if(m_fGESTURE_ID!=m_fGESTURE_ID_OLD)
                                {
                                    m_iStateOfGame=SOG_GESTURE_DECISION;
                                    m_fSteeringOutput=0;		//accuator output
                                    m_fAccelerationOutput= 0;
                                }


                                break;
                        }
                case SOG_GESTURE_DRIFT_LEFT:
                        {

                    m_fGESTURE_ID=GESTUREID;

                    if(m_fDistanceOverall-m_fDistanceOverall_Start< 1 )
                    {
                        m_fSteeringOutput=0;		//accuator output
                        m_fAccelerationOutput=m_fAccelerationOutput-1;
                        LOG_INFO(cString::Format("SOG_GESTURE_DRIFT_RIGHT Distance =%f & m_fCar_Speed =%f ",m_fDistanceOverall-m_fDistanceOverall_Start,m_fCar_Speed));
                    }
                    else
                    {
                        m_iStateOfGame=SOG_GESTURE_DRIFT_LEFT_INF;
                        m_bTurnLeft_Light=tTrue;
                        TransmitBoolValue(&m_oOutputTurnLeft_Light, m_bTurnLeft_Light, 0);
                        m_bHead_Light= tFalse;
                        TransmitBoolValue(&m_oOutputHead_Light, m_bHead_Light, 0);
                         m_bBreak_Light= tTrue;
                        TransmitBoolValue(&m_oOutputBreak_Light, m_bBreak_Light, 0);
                        LOG_INFO(cString::Format("SOG_GESTURE_DRIFT_LEFT_INF  "));

                    }
                    if(!m_b_GESTURE_Start)
                    {
                        m_bStart=tFalse;
                        stop_time = _clock->GetStreamTime();
                        m_bTransmitstop=tTrue;
                        if (m_bTransmitstop)
                        {
                            m_fAccelerationOutput=0;
                            m_fSteeringOutput=0;
                            TransmitFloatValue(&m_oOutputSteering, m_fSteeringOutput, 0);
                            TransmitFloatValue(&m_oOutputAcceleration, m_fAccelerationOutput, 0);
                            m_bHazard_Light= tFalse;
                            m_bBack_Light= tFalse;
                            m_bHead_Light= tFalse;
                            m_bBreak_Light= tFalse;
                            m_bTurnRight_Light=tFalse;
                            m_bTurnLeft_Light=tFalse;
                            TransmitBoolValue(&m_oOutputTurnLeft_Light, m_bTurnLeft_Light, 0);
                            TransmitBoolValue(&m_oOutputTurnRight_Light, m_bTurnRight_Light, 0);
                        TransmitBoolValue(&m_oOutputHazard_Light, m_bHazard_Light, 0);
                        TransmitBoolValue(&m_oOutputBack_Light, m_bBack_Light, 0);
                        TransmitBoolValue(&m_oOutputHead_Light, m_bHead_Light, 0);
                        TransmitBoolValue(&m_oOutputBreak_Light, m_bBreak_Light, 0);
                        TransmitFloatValue(&m_oOutputSteering_Drift, m_fSteeringOutput, 0);
                        TransmitFloatValue(&m_oOutputAcceleration_Drift, m_fAccelerationOutput, 0);
                        }

                    }







                                break;
                        }
                case SOG_GESTURE_DRIFT_RIGHT:
                        {

                            m_fGESTURE_ID=GESTUREID;

                            if(m_fDistanceOverall-m_fDistanceOverall_Start< 1 )
                            {
                                m_fSteeringOutput=0;		//accuator output
                                m_fAccelerationOutput=m_fAccelerationOutput-1;
                                LOG_INFO(cString::Format("SOG_GESTURE_DRIFT_RIGHT Distance =%f & m_fCar_Speed =%f ",m_fDistanceOverall-m_fDistanceOverall_Start,m_fCar_Speed));
                            }
                            else
                            {
                                m_iStateOfGame=SOG_GESTURE_DRIFT_RIGHT_INF;
                                     LOG_INFO(cString::Format("SOG_GESTURE_DRIFT_RIGHT_INF  "));
                                     m_bTurnRight_Light=tTrue;
                                     TransmitBoolValue(&m_oOutputTurnRight_Light, m_bTurnRight_Light, 0);
                                     m_bHead_Light= tFalse;
                                     TransmitBoolValue(&m_oOutputHead_Light, m_bHead_Light, 0);
                                      m_bBreak_Light= tTrue;
                                     TransmitBoolValue(&m_oOutputBreak_Light, m_bBreak_Light, 0);
                            }
                            if(!m_b_GESTURE_Start)
                            {
                                m_bStart=tFalse;
                                stop_time = _clock->GetStreamTime();
                                m_bTransmitstop=tTrue;
                                if (m_bTransmitstop)
                                {
                                    m_fAccelerationOutput=0;
                                    m_fSteeringOutput=0;
                                    TransmitFloatValue(&m_oOutputSteering, m_fSteeringOutput, 0);
                                    TransmitFloatValue(&m_oOutputAcceleration, m_fAccelerationOutput, 0);
                                    m_bHazard_Light= tFalse;
                                    m_bBack_Light= tFalse;
                                    m_bHead_Light= tFalse;
                                    m_bBreak_Light= tFalse;
                                    m_bTurnRight_Light=tFalse;
                                    m_bTurnLeft_Light=tFalse;
                                    TransmitBoolValue(&m_oOutputTurnLeft_Light, m_bTurnLeft_Light, 0);
                                    TransmitBoolValue(&m_oOutputTurnRight_Light, m_bTurnRight_Light, 0);

                                TransmitBoolValue(&m_oOutputHazard_Light, m_bHazard_Light, 0);
                                TransmitBoolValue(&m_oOutputBack_Light, m_bBack_Light, 0);
                                TransmitBoolValue(&m_oOutputHead_Light, m_bHead_Light, 0);
                                TransmitBoolValue(&m_oOutputBreak_Light, m_bBreak_Light, 0);
                                TransmitFloatValue(&m_oOutputSteering_Drift, m_fSteeringOutput, 0);
                                TransmitFloatValue(&m_oOutputAcceleration_Drift, m_fAccelerationOutput, 0);
                                }

                            }






                                break;
                        }
                case SOG_GESTURE_DRIFT_LEFT_INF:
                        {
                            m_fGESTURE_ID=GESTUREID;
                            m_fSteeringOutput=-DRIFT_STEER;		//accuator output
                            m_fAccelerationOutput=DRIFT_SPEED;
                            //if(m_fCar_Speed > DRIFT_SPEED ) m_fAccelerationOutput=m_fAccelerationOutput-0.01;
                           // else if(m_fCar_Speed < DRIFT_SPEED ) m_fAccelerationOutput= m_fAccelerationOutput+0.01;
                            //else  m_fAccelerationOutput=m_fAccelerationOutput;
                            if(!m_b_GESTURE_Start)
                            {
                                m_bStart=tFalse;
                                stop_time = _clock->GetStreamTime();
                                m_bTransmitstop=tTrue;
                                if (m_bTransmitstop)
                                {
                                    m_fAccelerationOutput=0;
                                    m_fSteeringOutput=0;
                                    TransmitFloatValue(&m_oOutputSteering, m_fSteeringOutput, 0);
                                    TransmitFloatValue(&m_oOutputAcceleration, m_fAccelerationOutput, 0);
                                    m_bHazard_Light= tFalse;
                                    m_bBack_Light= tFalse;
                                    m_bHead_Light= tFalse;
                                    m_bBreak_Light= tFalse;
                                    m_bTurnRight_Light=tFalse;
                                    m_bTurnLeft_Light=tFalse;
                                    TransmitBoolValue(&m_oOutputTurnLeft_Light, m_bTurnLeft_Light, 0);
                                    TransmitBoolValue(&m_oOutputTurnRight_Light, m_bTurnRight_Light, 0);

                                TransmitBoolValue(&m_oOutputHazard_Light, m_bHazard_Light, 0);
                                TransmitBoolValue(&m_oOutputBack_Light, m_bBack_Light, 0);
                                TransmitBoolValue(&m_oOutputHead_Light, m_bHead_Light, 0);
                                TransmitBoolValue(&m_oOutputBreak_Light, m_bBreak_Light, 0);
                                TransmitFloatValue(&m_oOutputSteering_Drift, m_fSteeringOutput, 0);
                                TransmitFloatValue(&m_oOutputAcceleration_Drift, m_fAccelerationOutput, 0);
                                }

                            }
                            if(m_fGESTURE_ID!=m_fGESTURE_ID_OLD)
                            {
                                m_iStateOfGame=SOG_GESTURE_DECISION;
                                m_fSteeringOutput=0;		//accuator output
                                m_fAccelerationOutput= 0;
                                m_fDistanceOverall_Start=m_fDistanceOverall;
                            }
                            else
                            {
                                m_iStateOfGame=SOG_GESTURE_DRIFT_LEFT_INF;
                            }

                                break;
                        }
                case SOG_GESTURE_DRIFT_RIGHT_INF:
                        {
                            m_fGESTURE_ID=GESTUREID;
                            m_fSteeringOutput=DRIFT_STEER;		//accuator output
                            m_fAccelerationOutput=DRIFT_SPEED;
                            //if(m_fCar_Speed > DRIFT_SPEED ) m_fAccelerationOutput=m_fAccelerationOutput-0.01;
                           // else if(m_fCar_Speed < DRIFT_SPEED ) m_fAccelerationOutput= m_fAccelerationOutput+0.01;
                            //else  m_fAccelerationOutput=m_fAccelerationOutput;
                            if(!m_b_GESTURE_Start)
                            {
                                m_bStart=tFalse;
                                stop_time = _clock->GetStreamTime();
                                m_bTransmitstop=tTrue;
                                if (m_bTransmitstop)
                                {
                                    m_fAccelerationOutput=0;
                                    m_fSteeringOutput=0;
                                    TransmitFloatValue(&m_oOutputSteering, m_fSteeringOutput, 0);
                                    TransmitFloatValue(&m_oOutputAcceleration, m_fAccelerationOutput, 0);
                                    m_bHazard_Light= tFalse;
                                    m_bBack_Light= tFalse;
                                    m_bHead_Light= tFalse;
                                    m_bBreak_Light= tFalse;
                                    m_bTurnRight_Light=tFalse;
                                    m_bTurnLeft_Light=tFalse;
                                    TransmitBoolValue(&m_oOutputTurnLeft_Light, m_bTurnLeft_Light, 0);
                                    TransmitBoolValue(&m_oOutputTurnRight_Light, m_bTurnRight_Light, 0);

                                TransmitBoolValue(&m_oOutputHazard_Light, m_bHazard_Light, 0);
                                TransmitBoolValue(&m_oOutputBack_Light, m_bBack_Light, 0);
                                TransmitBoolValue(&m_oOutputHead_Light, m_bHead_Light, 0);
                                TransmitBoolValue(&m_oOutputBreak_Light, m_bBreak_Light, 0);
                                TransmitFloatValue(&m_oOutputSteering_Drift, m_fSteeringOutput, 0);
                                TransmitFloatValue(&m_oOutputAcceleration_Drift, m_fAccelerationOutput, 0);
                                }

                            }
                            if(m_fGESTURE_ID!=m_fGESTURE_ID_OLD)
                            {
                                m_iStateOfGame=SOG_GESTURE_DECISION;
                                m_fSteeringOutput=0;		//accuator output
                                m_fAccelerationOutput= 0;
                                m_fDistanceOverall_Start=m_fDistanceOverall;
                            }
                            else
                            {
                                m_iStateOfGame=SOG_GESTURE_DRIFT_RIGHT_INF;
                            }

                                break;
                        }
                case SOG_GESTURE_STAY:
                        {

                                m_fSteeringOutput=0;		//accuator output
                                m_fAccelerationOutput= 0;
                                m_fGESTURE_ID=GESTUREID;
                                if(m_fGESTURE_ID>0)
                                {

                                    m_iStateOfGame=SOG_GESTURE_DECISION;
                                    m_fSteeringOutput=0;		//accuator output
                                    m_fAccelerationOutput= 0;

                                }
                                break;
                        }
                case SOG_GESTURE_FINISH:
                        {

                                m_fGESTURE_ID=GESTUREID;
                                m_b_GESTURE_Start= tFalse;
                                m_bStart= tFalse;
                                m_bGESTURE_Finished =tTrue;
                                TransmitBoolValue(&m_oOutputFinish, m_bGESTURE_Finished, 0);
                                m_iStateOfGame=SOG_GESTURE_NO_START;
                                break;
                        }
                }
 RETURN_NOERROR;
}


tResult cOactuators::TransmitFloatValue(cOutputPin* oPin, tFloat32 value, tUInt32 timestamp)
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
tResult cOactuators::TransmitBoolValue(cOutputPin* oPin, bool value, tUInt32 timestamp)
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
