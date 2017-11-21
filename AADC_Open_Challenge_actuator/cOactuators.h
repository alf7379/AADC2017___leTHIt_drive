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
#ifndef _ACTUATOR_FILTER_H_
#define _ACTUATOR_FILTER_H_

#define OID_ADTF_OPEN_ACTUATORS_FILTER "adtf.aadc.Oactuators"
//#define OID_ADTF_OPEN_ACTUATORS_FILTER "adtf.example.Oactuators"


class cOactuators : public adtf::cFilter
{
    /*! set the filter ID and the version */
    ADTF_FILTER(OID_ADTF_OPEN_ACTUATORS_FILTER, "Oactuators", adtf::OBJCAT_DataFilter);

protected:
    /*! the input pin for actuator data */
        // Start_test
        cInputPin m_iStart_test;
        cObjectPtr<IMediaTypeDescription> m_pDescStart_test;

        //Dice
        cInputPin	 m_iDice;
        cObjectPtr<IMediaTypeDescription> m_pCoderDescSignalInputDice;

        //Opp_Slot
        cInputPin	 m_iOpp_Slot;
        cObjectPtr<IMediaTypeDescription> m_pCoderDescSignalInputOpp_Slot;
                cInputPin m_iGESTURE_Start_test;
        cObjectPtr<IMediaTypeDescription> m_pDescGESTURE_Start_test;

        //GESTURE_ID
        cInputPin	 m_iGESTURE_ID;
        cObjectPtr<IMediaTypeDescription> m_pCoderDescSignalInputGESTURE_ID;
        // Distance over all
        cInputPin m_iCar_Speed;
        cObjectPtr<IMediaTypeDescription> m_pDescCar_Speed;
        // Distance over all
        cInputPin m_iDistanceOverall;
        cObjectPtr<IMediaTypeDescription> m_pDescdistanceoverall;

        // Input TrafficSign
        cInputPin    m_oInputRoadSign;
        cObjectPtr<IMediaTypeDescription> m_pDescriptionInputRoadSign;

        // Lane INFORMATION
        cInputPin m_olanechange;
        cObjectPtr<IMediaTypeDescription> m_pCoderDesclanechange;
        // Lane steer
        cInputPin m_iLane_steer;
        cObjectPtr<IMediaTypeDescription> m_pDescLane_steer;
        // Infos about Stop Line
        cInputPin m_iStopLine;
        cObjectPtr<IMediaTypeDescription> m_pDescStopLine;

    /*! output pins */
        //Slot
        cOutputPin	 m_oSlot;
        cObjectPtr<IMediaTypeDescription> m_pCoderDescSignalOutputSlot;
        // output Lanefollow_Start_test
        cOutputPin m_oOutputLanefollow_Start_test;
        cObjectPtr<IMediaTypeDescription> m_pDescriptionOutputLanefollow_Start_test;
        // output steering
        cOutputPin    m_oOutputSteering;
        cObjectPtr<IMediaTypeDescription> m_pDescriptionOutputSteering;

        // output acceleration
        cOutputPin m_oOutputAcceleration;
        cObjectPtr<IMediaTypeDescription> m_pDescriptionOutputAcceleration;

        //Output Steering__Drift
        cOutputPin    m_oOutputSteering_Drift;
        cObjectPtr<IMediaTypeDescription> m_pDescriptionOutputSteering_Drift;

        // output acceleration for drift
        cOutputPin m_oOutputAcceleration_Drift;
        cObjectPtr<IMediaTypeDescription> m_pDescriptionOutputAcceleration_Drift;

        // output finish
        cOutputPin m_oOutputFinish;
        cObjectPtr<IMediaTypeDescription> m_pDescriptionOutputFinish;



        // output Back_Light
        cOutputPin m_oOutputBack_Light;
        cObjectPtr<IMediaTypeDescription> m_pDescriptionOutputBack_Light;
        // output Hazard_Light
        cOutputPin m_oOutputHazard_Light;
        cObjectPtr<IMediaTypeDescription> m_pDescriptionOutputHazard_Light;
        // output Head_Light
        cOutputPin m_oOutputHead_Light;
        cObjectPtr<IMediaTypeDescription> m_pDescriptionOutputHead_Light;

        // output Break_Light
        cOutputPin m_oOutputBreak_Light;
        cObjectPtr<IMediaTypeDescription> m_pDescriptionOutputBreak_Light;
        // output TurnLeft_Light
        cOutputPin m_oOutputTurnLeft_Light;
        cObjectPtr<IMediaTypeDescription> m_pDescriptionOutputTurnLeft_Light;
        // output TurnRight_Light
        cOutputPin m_oOutputTurnRight_Light;
        cObjectPtr<IMediaTypeDescription> m_pDescriptionOutputTurnRight_Light;

    /*! The critical section transmit control */
    cCriticalSection m_critSecTransmitControl;

    /*! The critical section transmit bool */
    cCriticalSection m_critSecTransmitBool;

    /*! the media description for bool values */
    cObjectPtr<IMediaTypeDescription> m_pDescriptionBool;

    /*! the media description for float values */
    cObjectPtr<IMediaTypeDescription> m_pDescriptionFloat;


        tBool m_bStart;
        tBool m_bWake_Up;
       tBool m_bRoadSignDetected;
        tFloat32 m_fLane_steer;
        tTimeStamp stop_time;
        tBool m_bFinished;
        tBool m_bGESTURE_Finished;
        tBool m_bTransmitout;
        tBool m_bTransmitstop;
        tBool m_bImage_used;
        tBool m_bOdd_start;
        tFloat32 m_fSlot_Length;
        tFloat32 m_fCurrent_SLot;
        tInt16 m_iGameslot_Counter;
        tInt16 m_iDiceroll_Counter;
        tInt16 m_iModulo_2;
        tFloat32  m_fDiceID;
        tFloat32 m_fOpponent_SLOT_ID;
                tFloat32  m_fGESTURE_ID;
                tFloat32  m_fGESTURE_ID_OLD;
        tInt16 m_iStateOfGame;
        tInt16 m_iModeOfGame;
        tInt16 m_iCurrentTrafficSignID;
         tInt16 m_iDestinationTrafficSignID;
        tFloat32 m_fTrafficSignImageSize;

        tFloat32 m_f32RoadSignDistanceY;
        tFloat32 m_f32RoadSignDistanceX;
        tFloat32 m_fback_coverdist;
        tFloat32 m_fcoverdist;
        tFloat32 m_fOrientation2StopLine;
        tFloat32 m_fLine_distance;
        tBool m_bLine_detection;
        tBool m_bOnelinedetection;
        tBool m_bTwolinedetection;
        tBool m_b_GESTURE_Start;
        tFloat32 m_fCar_Speed;
                tBool m_bNolinedetection;

tFloat32 f32Distance2RoadSign;
                tFloat32 m_f32ArrayRoadSignDistanceY[3];
                tFloat32 m_f32ArrayRoadSignDistanceX[3];
        enum StateOfGameEnums
        {
        SOG_NOSTART = 0,
        SOG_DECISION,

          SOG_GOTO_SLOT,
             SOG_GOTO_STOPLINE,
        SOG_BACKWORD,
        SOG_STAY,
        SOG_MANUVER_FINISH,
        SOG_GAME_WIN_FINISH,
                SOG_GESTURE_NO_START,
                SOG_GESTURE_DECISION,
                SOG_GESTURE_FORWORD,
                SOG_GESTURE_LEFT,
        SOG_GESTURE_RIGHT,
                SOG_GESTURE_BACKWORD,
                SOG_GESTURE_DRIFT_LEFT,
                SOG_GESTURE_DRIFT_RIGHT,
            SOG_GESTURE_DRIFT_LEFT_INF,
            SOG_GESTURE_DRIFT_RIGHT_INF,
        SOG_GESTURE_STAY,
                SOG_GESTURE_FINISH,
        }   ;
        enum 	ModeOfGameEnums
        {
        MOG_NOT_SET = 0,
        MOG_ODD_START ,
        MOG_EVEN_START
        }   ;
        // time stamp
        tUInt32 timestamp;

        // debug mode
        tBool m_bDebugModeEnabled;


        // distance over all
        tFloat32 m_fDistanceOverall;
        tFloat32 m_fDistanceOverall_Start;
        /* MEMBER VARIABLES OUTPUT*/
        tFloat32 m_fSteeringOutput;
        tFloat32 m_fAccelerationOutput;

        tBool m_bHazard_Light;
        tBool m_bBack_Light;
        tBool m_bHead_Light;

        tBool m_bBreak_Light;
        tBool m_bTurnLeft_Light;
        tBool m_bTurnRight_Light;


public:
    /*! default constructor for actuator class
           \param __info   [in] This is the name of the filter instance.
    */
    cOactuators(const tChar* __info);

    /*! default destructor */
    virtual ~cOactuators();
 tResult PropertyChanged(const char* strProperty);
protected:
                                tResult TransmitBoolValue(cOutputPin* pin, tBool value, tUInt32 timestamp);
                                tResult TransmitFloatValue(cOutputPin* pin, tFloat32 value, tUInt32 timestamp);
                                tResult ProcessManeuver();
                                 tResult ReadProperties(const tChar* strPropertyName);
    tResult Init(tInitStage eStage, ucom::IException** __exception_ptr);
    tResult Shutdown(tInitStage eStage, ucom::IException** __exception_ptr = NULL);
    tResult OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample);
    private:
        tFloat32    DIST_SCAN_SLOTLENGHT,DiceID,GESTUREID,DRIFT_SPEED,DRIFT_STEER ;

        };

//*************************************************************************************************
#endif // _ACTUATOR_FILTER_H_

/*!
*@}
*/
