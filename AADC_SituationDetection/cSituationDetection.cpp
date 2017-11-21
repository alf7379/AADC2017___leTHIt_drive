
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
#include "cSituationDetection.h" 
#include <string>

//#include "template_data.h"
/// Create filter shell
ADTF_FILTER_PLUGIN("SituationDetection", OID_ADTF_SITUATION_DETECTION, cSituationDetection);

using namespace roadsignIDs;


// Detecting what instance of the filter is running currently
cSituationDetection::cSituationDetection(const tChar* __info):cFilter(__info)
{

 
    SetPropertyFloat ("Trigger Distance to Edge 1",40);
    //SetPropertyFloat ("Trigger Distance to Edge 2",80);
    SetPropertyFloat ("Trigger Distance to Stopline",35);
    SetPropertyFloat("Trigger Distance to Edgeline",40);
    SetPropertyFloat("Trigger Distance to RoadSign",0.4);

    SetPropertyBool ("Enable Stopline for triggering Crossing", true);
    SetPropertyBool ("Enable Edge1 for triggering Crossing", true);
    //SetPropertyBool ("Enable Edge2 for triggering Crossing", true);
    SetPropertyBool ("Enable Edgeline for triggering Crossing", true);
    SetPropertyBool ("Enable RoadSign for triggering Crossing", true);

    // m_bDebugModeEnabled = true;
    SetPropertyBool("Debug Output to Console",true);
    // m_iRoadSignCounterMax = 5;
    SetPropertyInt ("Road Sign Counter max",3);
    // Properties for testing Crossing
    SetPropertyBool("Test Crossing enabled", false);
    SetPropertyInt("Test Crossing Maneuver ID",1);
    // Properties for testing PullOut
    SetPropertyBool("Test Pull Out enabled",false);
    SetPropertyInt("Test Pull Out Direction ID",0);

    // Properties for testing Parking
    SetPropertyBool("Test Parking enabled",false);
    SetPropertyBool("Enable the ManeuverInProcess Falg", true);
    SetPropertyBool ("Enable Stopline for triggering Crossing",false);

}

cSituationDetection::~cSituationDetection()
{

}

tResult cSituationDetection::Init(tInitStage eStage, __exception)
{
    // never miss calling the parent implementation!!
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))

    // in StageFirst you can create and register your static pins.
    if (eStage == StageFirst)
    {
        // Registration of the input and output pins
        /*
        // get a media type for the input pin
        cObjectPtr<IMediaDescriptionManager> pDescManager;
        RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER, (tVoid**)&pDescManager, __exception_ptr));
        //
        // create and register the video input pin
        RETURN_IF_FAILED(m_oVideoInputPin.Create("Video_Input", IPin::PD_Input, static_cast<IPinEventSink*>(this)));
        RETURN_IF_FAILED(RegisterPin(&m_oVideoInputPin));
        */

        /*
        // get a media type for the input pin
        cObjectPtr<IMediaType> pInputType;
        RETURN_IF_FAILED(AllocMediaType(&pInputType, MEDIA_TYPE_TEMPLATE, MEDIA_SUBTYPE_TEMPLATE, __exception_ptr));

        // create and register the input pin
        RETURN_IF_FAILED(m_oInputPin.Create("input_template", pInputType, this));
        RETURN_IF_FAILED(RegisterPin(&m_oInputPin));

        // get a media type for the output pin
        cObjectPtr<IMediaType> pOutputType;
        RETURN_IF_FAILED(AllocMediaType(&pOutputType, MEDIA_TYPE_TEMPLATE, MEDIA_SUBTYPE_TEMPLATE, __exception_ptr));

        // create and register the output pin
        RETURN_IF_FAILED(m_oOutputPin.Create("output_template", pOutputType, this));
        RETURN_IF_FAILED(RegisterPin(&m_oOutputPin));
        */

        cObjectPtr<IMediaDescriptionManager> pDescManager;
        RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));

        // Input - JuryStruct
        tChar const * strInputJuryStruct = pDescManager->GetMediaDescription("tJuryStruct");
        RETURN_IF_POINTER_NULL(strInputJuryStruct);
        cObjectPtr<IMediaType> pTypeInputJuryStruct = new cMediaType(0, 0, 0, "tJuryStruct", strInputJuryStruct, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

        RETURN_IF_FAILED(pTypeInputJuryStruct->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionInputJuryStruct));
        RETURN_IF_FAILED(m_oInputJuryStruct.Create("JuryStruct", pTypeInputJuryStruct, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oInputJuryStruct));

        // Input - ManeuverList
        tChar const * strInputManeuverList = pDescManager->GetMediaDescription("tManeuverList");
        RETURN_IF_POINTER_NULL(strInputManeuverList);
        cObjectPtr<IMediaType> pTypeInputManeuverList = new cMediaType(0, 0, 0, "tManeuverList", strInputManeuverList, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

        RETURN_IF_FAILED(pTypeInputManeuverList->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionInputManeuverList));
        RETURN_IF_FAILED(m_oInputManeuverList.Create("Maneuver_List", pTypeInputManeuverList, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oInputManeuverList));

        // Input - RoadSign
        tChar const * strRoadSign = pDescManager->GetMediaDescription("tRoadSignExt");
        RETURN_IF_POINTER_NULL(strRoadSign);
        cObjectPtr<IMediaType> pTypeRoadSign = new cMediaType(0, 0, 0, "tRoadSignExt", strRoadSign, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

        RETURN_IF_FAILED(pTypeRoadSign->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionInputTrafficSign));
        RETURN_IF_FAILED(m_oInputTrafficSign.Create("RoadSignExt", pTypeRoadSign, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oInputTrafficSign));


        // Input - Edgeline Detection
        tChar const * strEdgelineDetection = pDescManager->GetMediaDescription("tStoplineStruct");
        RETURN_IF_POINTER_NULL(strEdgelineDetection);
        cObjectPtr<IMediaType> pTypeEdgelineDetection = new cMediaType(0, 0, 0, "tStoplineStruct", strEdgelineDetection, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

        RETURN_IF_FAILED(pTypeEdgelineDetection->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionInputEdgelineDetection));
        RETURN_IF_FAILED(m_oInputEdgelineDetection.Create("Edgeline Detection", pTypeEdgelineDetection, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oInputEdgelineDetection));


        // Input - Stopline Detection
        tChar const * strStoplineDetection = pDescManager->GetMediaDescription("tStoplineStruct");
        RETURN_IF_POINTER_NULL(strStoplineDetection);
        cObjectPtr<IMediaType> pTypeStoplineDetection = new cMediaType(0, 0, 0, "tStoplineStruct", strStoplineDetection, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

        RETURN_IF_FAILED(pTypeStoplineDetection->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionInputStoplineDetection));
        RETURN_IF_FAILED(m_oInputStoplineDetection.Create("Stopline Detection", pTypeStoplineDetection, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oInputStoplineDetection));


        // Input - EdgeDetection
        tChar const * strEdgeDetection = pDescManager->GetMediaDescription("tEdgeStruct");
        RETURN_IF_POINTER_NULL(strEdgeDetection);
        cObjectPtr<IMediaType> pTypeSignalEdgeDetection = new cMediaType(0, 0, 0, "tEdgeStruct", strEdgeDetection, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

        RETURN_IF_FAILED(pTypeSignalEdgeDetection->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionInputEdgeDetection));
        RETURN_IF_FAILED(m_oInputEdgeDetection.Create("Edge Detection", pTypeSignalEdgeDetection, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oInputEdgeDetection));

        // Input - ObjectDetection
        tChar const * strObjectDetection = pDescManager->GetMediaDescription("tBoolSignalValue");
        RETURN_IF_POINTER_NULL(strObjectDetection);
        cObjectPtr<IMediaType> pTypeSignalObjectDetection = new cMediaType(0, 0, 0, "tBoolSignalValue", strObjectDetection, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

        RETURN_IF_FAILED(pTypeSignalObjectDetection->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionInputObjectDetection));
        RETURN_IF_FAILED(m_oInputObjectDetection.Create("Object Detection", pTypeSignalObjectDetection, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oInputObjectDetection));


        // Input - ACC
        tChar const * strACC = pDescManager->GetMediaDescription("tBoolSignalValue");
        RETURN_IF_POINTER_NULL(strACC);
        cObjectPtr<IMediaType> pTypeSignalACC = new cMediaType(0, 0, 0, "tBoolSignalValue", strACC, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

        RETURN_IF_FAILED(pTypeSignalACC->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionInputACC));
        RETURN_IF_FAILED(m_oInputACC.Create("ACC Feedback", pTypeSignalACC, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oInputACC));


        // Input - InputLaneChangeFinished
        tChar const * strInputLaneChangeFinished = pDescManager->GetMediaDescription("tBoolSignalValue");
        RETURN_IF_POINTER_NULL(strInputLaneChangeFinished);
        cObjectPtr<IMediaType> pTypeSignalInputLaneChangeFinished = new cMediaType(0, 0, 0, "tBoolSignalValue", strInputLaneChangeFinished, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

        RETURN_IF_FAILED(pTypeSignalInputLaneChangeFinished->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionInputLaneChangeFinished));
        RETURN_IF_FAILED(m_oInputLaneChangeFinished.Create("LaneChange Feedback", pTypeSignalACC, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oInputLaneChangeFinished));


        // Input - ScanningFinished
        tChar const * strScanningFinished = pDescManager->GetMediaDescription("tBoolSignalValue");
        RETURN_IF_POINTER_NULL(strScanningFinished);
        cObjectPtr<IMediaType> pTypeSignalScanningFinished = new cMediaType(0, 0, 0, "tBoolSignalValue", strScanningFinished, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

        RETURN_IF_FAILED(pTypeSignalScanningFinished->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionInputScanningFinished));
        RETURN_IF_FAILED(m_oInputScanningFinished.Create("Scanning Finished", pTypeSignalScanningFinished, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oInputScanningFinished));


        // Input - ParkingFeedback
        tChar const * strParkingFinished = pDescManager->GetMediaDescription("tBoolSignalValue");
        RETURN_IF_POINTER_NULL(strParkingFinished);
        cObjectPtr<IMediaType> pTypeSignalParkingFinished = new cMediaType(0, 0, 0, "tBoolSignalValue", strParkingFinished, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

        RETURN_IF_FAILED(pTypeSignalParkingFinished->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionInputParkingFinished));
        RETURN_IF_FAILED(m_oInputParkingFinished.Create("Parking Finished", pTypeSignalParkingFinished, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oInputParkingFinished));

        // Input - PullOutLeftFeedback
        tChar const * strPullOutLeftFinished = pDescManager->GetMediaDescription("tBoolSignalValue");
        RETURN_IF_POINTER_NULL(strPullOutLeftFinished);
        cObjectPtr<IMediaType> pTypeSignalPullOutLeftFinished = new cMediaType(0, 0, 0, "tBoolSignalValue", strPullOutLeftFinished, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

        RETURN_IF_FAILED(pTypeSignalPullOutLeftFinished->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionInputPullOutLeftFinished));
        RETURN_IF_FAILED(m_oInputPullOutLeftFinished.Create("PullOutLeft Finished", pTypeSignalPullOutLeftFinished, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oInputPullOutLeftFinished));

        // Input - PullOutRightFeedback
        tChar const * strPullOutRightFinished = pDescManager->GetMediaDescription("tBoolSignalValue");
        RETURN_IF_POINTER_NULL(strPullOutRightFinished);
        cObjectPtr<IMediaType> pTypeSignalPullOutRightFinished = new cMediaType(0, 0, 0, "tBoolSignalValue", strPullOutRightFinished, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

        RETURN_IF_FAILED(pTypeSignalPullOutRightFinished->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionInputPullOutRightFinished));
        RETURN_IF_FAILED(m_oInputPullOutRightFinished.Create("PullOutRight Finished", pTypeSignalPullOutRightFinished, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oInputPullOutRightFinished));

        // Input - CrossingFeedback
        tChar const * strCrossingFinished = pDescManager->GetMediaDescription("tBoolSignalValue");
        RETURN_IF_POINTER_NULL(strCrossingFinished);
        cObjectPtr<IMediaType> pTypeSignalCrossingFinished = new cMediaType(0, 0, 0, "tBoolSignalValue", strCrossingFinished, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

        RETURN_IF_FAILED(pTypeSignalCrossingFinished->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionInputCrossingFinished));
        RETURN_IF_FAILED(m_oInputCrossingFinished.Create("Crossing Finished", pTypeSignalCrossingFinished, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oInputCrossingFinished));

        // Input - ZebracrossingFeedback
        tChar const * strZebracrossingFinished = pDescManager->GetMediaDescription("tBoolSignalValue");
        RETURN_IF_POINTER_NULL(strZebracrossingFinished);
        cObjectPtr<IMediaType> pTypeSignalZebracrossingFinished = new cMediaType(0, 0, 0, "tBoolSignalValue", strZebracrossingFinished, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

        RETURN_IF_FAILED(pTypeSignalZebracrossingFinished->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionInputZebracrossingFinished));
        RETURN_IF_FAILED(m_oInputZebracrossingFinished.Create("Zebracrossing Finished", pTypeSignalZebracrossingFinished, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oInputZebracrossingFinished));



        // Output - DriverStruct
        tChar const * strOutputDriverStruct = pDescManager->GetMediaDescription("tDriverStruct");
        RETURN_IF_POINTER_NULL(strOutputDriverStruct);
        cObjectPtr<IMediaType> pTypeOutputDriverStruct = new cMediaType(0, 0, 0, "tDriverStruct", strOutputDriverStruct, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

        RETURN_IF_FAILED(pTypeOutputDriverStruct->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionOutputDriverStruct));
        RETURN_IF_FAILED(m_oOutputDriverStruct.Create("Drive Struct", pTypeOutputDriverStruct, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oOutputDriverStruct));

        // Output - RoI
        tChar const * strOutputRoI = pDescManager->GetMediaDescription("tSignalValue");
        RETURN_IF_POINTER_NULL(strOutputRoI);
        cObjectPtr<IMediaType> pTypeOutputRoI = new cMediaType(0, 0, 0, "tSignalValue", strOutputRoI, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

        RETURN_IF_FAILED(pTypeOutputRoI->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionOutputRoI));
        RETURN_IF_FAILED(m_oOutputRoI.Create("RoI", pTypeOutputRoI, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oOutputRoI));

        // Output - LaneFollowerStart
        tChar const * strDescSignalLaneFollowerStart = pDescManager->GetMediaDescription("tStartTrigger");
        RETURN_IF_POINTER_NULL(strDescSignalLaneFollowerStart);
        cObjectPtr<IMediaType> pTypeSignalLaneFollowerStart = new cMediaType(0, 0, 0, "tStartTrigger", strDescSignalLaneFollowerStart, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

        RETURN_IF_FAILED(pTypeSignalLaneFollowerStart->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionOutputLaneFollowerStart));
        RETURN_IF_FAILED(m_oOutputLaneFollowerStart.Create("LaneFollower Start", pTypeSignalLaneFollowerStart, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oOutputLaneFollowerStart));

        // Output - LaneChangeStart
        tChar const * strOutputLaneChangeStart = pDescManager->GetMediaDescription("tBoolSignalValue");
        RETURN_IF_POINTER_NULL(strOutputLaneChangeStart);
        cObjectPtr<IMediaType> pTypeOutputLaneChangeStart = new cMediaType(0, 0, 0, "tBoolSignalValue", strOutputLaneChangeStart, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

        RETURN_IF_FAILED(pTypeOutputLaneChangeStart->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionOutputLaneChangeStart));
        RETURN_IF_FAILED(m_oOutputLaneChangeStart.Create("Lanechange Start", pTypeOutputLaneChangeStart, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oOutputLaneChangeStart));


        // Output - ScanningStart
        tChar const * strDescSignalScanningStart = pDescManager->GetMediaDescription("tBoolSignalValue"); //tBoolSignalValue
        RETURN_IF_POINTER_NULL(strDescSignalScanningStart);
        cObjectPtr<IMediaType> pTypeSignalScanningStart = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescSignalScanningStart, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

        RETURN_IF_FAILED(pTypeSignalScanningStart->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionOutputScanningStart));
        RETURN_IF_FAILED(m_oOutputScanningStart.Create("Scanning Start", pTypeSignalScanningStart, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oOutputScanningStart));


        // Output - ParkingStart
        tChar const * strDescSignalParkingStart = pDescManager->GetMediaDescription("tStartTrigger"); //tBoolSignalValue
        RETURN_IF_POINTER_NULL(strDescSignalParkingStart);
        cObjectPtr<IMediaType> pTypeSignalParkingStart = new cMediaType(0, 0, 0, "tStartTrigger", strDescSignalParkingStart, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

        RETURN_IF_FAILED(pTypeSignalParkingStart->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionOutputParkingStart));
        RETURN_IF_FAILED(m_oOutputParkingStart.Create("Parking Start", pTypeSignalParkingStart, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oOutputParkingStart));


        // Output - PullOutLeft
        tChar const * strDescSignalPullOutLeftStart = pDescManager->GetMediaDescription("tBoolSignalValue"); //tBoolSignalValue
        RETURN_IF_POINTER_NULL(strDescSignalPullOutLeftStart);
        cObjectPtr<IMediaType> pTypeSignalPullOutLeftStart = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescSignalPullOutLeftStart, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

        RETURN_IF_FAILED(pTypeSignalPullOutLeftStart->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionOutputPullOutLeftStart));
        RETURN_IF_FAILED(m_oOutputPullOutLeftStart.Create("PullOutLeft Start", pTypeSignalPullOutLeftStart, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oOutputPullOutLeftStart));

        // Output - PullOutRight
        tChar const * strDescSignalPullOutRightStart = pDescManager->GetMediaDescription("tBoolSignalValue"); //tBoolSignalValue
        RETURN_IF_POINTER_NULL(strDescSignalPullOutRightStart);
        cObjectPtr<IMediaType> pTypeSignalPullOutRightStart = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescSignalPullOutRightStart, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

        RETURN_IF_FAILED(pTypeSignalPullOutRightStart->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionOutputPullOutRightStart));
        RETURN_IF_FAILED(m_oOutputPullOutRightStart.Create("PullOutRight Start", pTypeSignalPullOutRightStart, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oOutputPullOutRightStart));

        // Output - CrossingStart
        tChar const * strDescSignalCrossingStart = pDescManager->GetMediaDescription("tCrossingStruct");
        RETURN_IF_POINTER_NULL(strDescSignalCrossingStart);
        cObjectPtr<IMediaType> pTypeSignalCrossingStart = new cMediaType(0, 0, 0, "tCrossingStruct", strDescSignalCrossingStart, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

        RETURN_IF_FAILED(pTypeSignalCrossingStart->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionOutputCrossingStart));
        RETURN_IF_FAILED(m_oOutputCrossingStart.Create("Crossing Start", pTypeSignalCrossingStart, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oOutputCrossingStart));

        // Output - Zebracrossing
        tChar const * strDescSignalZebracrossingStart = pDescManager->GetMediaDescription("tBoolSignalValue");
        RETURN_IF_POINTER_NULL(strDescSignalZebracrossingStart);
        cObjectPtr<IMediaType> pTypeSignalZebracrossingStart = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescSignalZebracrossingStart, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

        RETURN_IF_FAILED(pTypeSignalZebracrossingStart->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionOutputZebracrossingStart));
        RETURN_IF_FAILED(m_oOutputZebracrossingStart.Create("Zebracrossing Start", pTypeSignalZebracrossingStart, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oOutputZebracrossingStart));

    }
    else if (eStage == StageNormal)
    {
        // In this stage you would do further initialisation and/or create your dynamic pins.
        // Please take a look at the demo_dynamicpin example for further reference.

        timestamp = 0;

        // Inputs
        m_bInputACCLaneChangeRequested = false;
        m_bInputParkingFinished = false;
        m_bInputPullOutLeftFinished = false;
        m_bInputPullOutRightFinished =false;
        m_bInputCrossingFinished =false;

        // Outputs
        m_bOutputParkingStart = false;
        m_bOutputPullOutLeftStart = false;
        m_bOutputPullOutRightStart = false;
        m_bOutputLaneFollowerStart = false;


        // Road sign
        m_fTrafficSignImageSize = 0;
        m_iCurrentTrafficSignID = -1;
        m_iPreviousTrafficSignID = -1;
        m_szIDRoadSignI16Identifier = 0;
        m_szIDRoadSignF32Imagesize = 0;
        m_bIDsRoadSignSet = false;
        m_iRoadSignDetectorCounter = 0;
        m_bManeuverInProcess = false;
        m_iRoadSignCounterMax = GetPropertyInt ("Road Sign Counter max");
        m_f32RoadSignDistanceX = 0;
        m_f32RoadSignDistanceY = 0;
        m_f32ArrayRoadSignDistanceY[0] = 0;
        m_f32ArrayRoadSignDistanceY[1] = 0;
        m_f32ArrayRoadSignDistanceY[2] = 0;
        m_f32ArrayRoadSignDistanceX[0] = 0;
        m_f32ArrayRoadSignDistanceX[1] = 0;
        m_f32ArrayRoadSignDistanceX[2] = 0;
        m_bParkingSignDetected = false;
        m_bParkingSpotAhead = false;



        m_iStartCrossingCase = 0;
        m_bRoadSignEvaluated = false;

        // Jury Struct
        m_szIDJuryStructI8ActionID = 0;
        m_szIDJuryStructI16ManeuverEntry = 0;
        m_bIDsJuryStructSet = false;
        // Driver Struct
        m_szIDDriverStructI8StateID = 0;
        m_szIDDriverStructI16ManeuverEntry = 0;
        m_bIDsDriverStructSet = false;


        // Edge Detection
        m_bInputEdge1Detected = false;
        m_fInputEdge1Distance = 0;
        m_bInputEdge2Detected = false;
        m_fInputEdge2Distance = 0;

        // Edgline Detection
        m_bInputEdgeLineDetected = false;
        m_fInputEdgeLineDistance = 0;

        // Stopline Detection
        m_bInputStoplineDetected = false;
        m_fInputStoplineDistance = 0;

        // ManeuverVariables
        m_strManeuverFileString = "";
/*
        for(tInt i=0; i<m_sectorList.size();i++){
          m_sectorList[i].id = 0;
          for(tInt j=0; j<m_sectorList[i].maneuverList[j].size(); j++){
              m_sectorList[i].maneuverList[j].action = 0;
              m_sectorList[i].maneuverList[j].action = 0;
          }
        }
*/
        for(tInt i=0;i<100;i++){
            m_iArrayNumberOfManeuversInSerctor[i]=0;
        }
        m_iAmountOfSectors=0;
        m_iSectorIDCounter=0;
        m_iManeuverIDOverallCounter=0;
        m_iManeuverIDSectorCounter=0;
        m_bManeuverListProcessed = false;

        m_fTriggerDistance2Edge1 = GetPropertyFloat("Trigger Distance to Edge 1");
        // m_fTriggerDistance2Edge2 = GetPropertyFloat("Trigger Distance to Edge 2");
        m_fTriggerDistance2Stopline = GetPropertyFloat("Trigger Distance to Stopline");
        m_fTriggerDistance2Edgeline = GetPropertyFloat("Trigger Distance to Edgeline");
        m_fTriggerDistance2RoadSign = GetPropertyFloat("Trigger Distance to RoadSign");


        m_bEnableStopline4TriggeringCrossing = GetPropertyBool ("Enable Stopline for triggering Crossing");
        m_bEnableEdge14TriggeringCrossing = GetPropertyBool ("Enable Edge1 for triggering Crossing");
        // m_bEnableEdge24TriggeringCrossing = GetPropertyBool ("Enable Edge2 for triggering Crossing");
        m_bEnableEdgeline4TriggeringCrossing = GetPropertyBool ("Enable Edgeline for triggering Crossing");
        m_bEnableRoadSign4TriggeringCrossing = GetPropertyBool ("Enable RoadSign for triggering Crossing");

        // Testing properties
        {
            // Property for enable the flag ManeuverInProcess
            m_bEnableManeuverInProcessFlag = GetPropertyBool("Enable the ManeuverInProcess Falg");

            m_bDebugModeEnabled = GetPropertyBool("Debug Output to Console");

            // Properties for testing Crossing
            m_bTestCrossingEnabled = GetPropertyBool("Test Crossing enabled");
            m_iTestCrossingManeuverID = GetPropertyInt("Test Crossing Maneuver ID");

            // Properties for testing Pull Out
            m_bTestPullOutEnabled = GetPropertyBool("Test Pull Out enabled");
            m_iTestPullOutDirectionID = GetPropertyInt("Test Pull Out Direction ID");

            // Properties for testing Parking
            m_bTestParkingEnabled = GetPropertyBool("Test Parking enabled");

            if(m_bTestPullOutEnabled == true || m_bTestCrossingEnabled == true || m_bTestParkingEnabled == true)
            {
                m_bCarStateRunning = true;
                SendRoI2StopLineDetection();
            }
            else
            {
                m_bCarStateRunning = false;
            }
        }

        //m_bIDsDriverStructSet = false;
        //m_bIDsJuryStructSet = false;
    }
    else if (eStage == StageGraphReady)
    {
        // All pin connections have been established in this stage so you can query your pins
        // about their media types and additional meta data.
        // Please take a look at the demo_imageproc example for further reference.
        // disable maneuver group box until receiving maneuver list
        //m_pWidget->EnableManeuverGroupBox(false);
        // no ids were set so far
        m_bIDsJuryStructSet = false;
        m_bIDsDriverStructSet = false;
    }
    RETURN_NOERROR;
}

tResult cSituationDetection::Shutdown(tInitStage eStage, __exception)
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

tResult cSituationDetection::OnPinEvent(IPin* pSource,
                                    tInt nEventCode,
                                    tInt nParam1,
                                    tInt nParam2,
                                    IMediaSample* pMediaSample)
{

    if (nEventCode == IPinEventSink::PE_MediaSampleReceived)
    {
        // so we received a media sample, so this pointer better be valid.
        RETURN_IF_POINTER_NULL(pMediaSample);

        // by comparing it to our member pin variable we can find out which pin received
        // the sample


        // Jury Struct received
        if(pSource == &m_oInputJuryStruct && m_pDescriptionInputJuryStruct != NULL){

            if(m_bDebugModeEnabled) LOG_INFO(cString::Format("JuryStruct received"));
            tInt8 i8ActionID = -2;
            tInt16 i16entry = -1;
            // Read out the JuryStruct
            {
                // focus for sample read lock
                __adtf_sample_read_lock_mediadescription(m_pDescriptionInputJuryStruct,pMediaSample,pCoder);
                // get the IDs for the items in the media sample
                //if(m_bDebugModeEnabled)  LOG_INFO(cString::Format("m_bIDsJuryStructSet =%b",m_bIDsJuryStructSet));
                if(!m_bIDsJuryStructSet)
                {
                    pCoder->GetID("i8ActionID", m_szIDJuryStructI8ActionID);
                    pCoder->GetID("i16ManeuverEntry", m_szIDJuryStructI16ManeuverEntry);
                    m_bIDsJuryStructSet = true;
                }
                // Read the actionID and the ManeuverEntry out of the DriverStruct
                pCoder->Get(m_szIDJuryStructI8ActionID, (tVoid*)&i8ActionID);
                pCoder->Get(m_szIDJuryStructI16ManeuverEntry, (tVoid*)&i16entry);
            }


            if(m_bManeuverListProcessed){
                // ManeuverList is processed
                cString strNextManeuver;
                switch (juryActions(i8ActionID))
                {
                case action_GETREADY:   // = 0
                    if(m_bDebugModeEnabled)
                    SendDriverStruct(stateCar_READY, i16entry);                    
                    m_bCarStateRunning = false;
                    StopVehicle();
                    break;

                case action_START:      // = 1
                    if(m_bDebugModeEnabled)  LOG_INFO(cString::Format("Situation Detection: Received Run with maneuver ID %d",i16entry));
                    // Send the JuryModel the state running with the ManeuverID 0
                    tBool bCurrentSectorFound;
                    bCurrentSectorFound= false;
                    // Get current sectorID in the case of a restart not at sector 0
                    for(tInt i=0; i<m_iAmountOfSectors;i++)
                    {
                        if(i16entry == m_sectorList[i].maneuverList[0].id)
                        {
                            m_iSectorIDCounter = i;
                            bCurrentSectorFound = true;
                        }
                    }
                    if(bCurrentSectorFound == false){
                        LOG_ERROR(cString::Format("Situation Detection: The current sector ID was not found"));
                    }

                    m_iManeuverIDOverallCounter = i16entry; //m_sectorList[m_iSectorIDCounter].maneuverList[0].id;



                    SendDriverStruct(stateCar_RUNNING, m_iManeuverIDOverallCounter);
                    // Check whats the first order in the ManeuverList
                    // cString strNextManeuver;
                    strNextManeuver = m_sectorList[m_iSectorIDCounter].maneuverList[0].action;
                    tTimeStamp tsTime1;
                    tsTime1 = _clock->GetStreamTime();
                    while((tsTime1+200000) > _clock->GetStreamTime())
                    {
                        // Wait 0.2s
                    }
                    m_bCarStateRunning = true;

                    if(strNextManeuver.IsEqual("pull_out_left") || strNextManeuver.IsEqual("pull_out_right"))
                    {
                        StartPullOut();
                    } 
                    else if(strNextManeuver.IsEqual("left") || strNextManeuver.IsEqual("straight") || strNextManeuver.IsEqual("right") || (strNextManeuver.Find("cross_parking") != -1) )
                    {
                        SendRoI2StopLineDetection();
                        tBool bLaneFollowerEnabled = true;
                       // if(m_bDebugModeEnabled) LOG_INFO(adtf_util::cString::Format("Start Lane Folloer, after receiving StartFlag from the Jury: bLaneFollowerEnabled %i",bLaneFollowerEnabled));

                        if(!m_bManeuverInProcess)
                        {
                           SendOutputs(true, false, -1, -1);
                        }

/*
                        // Send start signal to LaneFollower-filter
                        // Create a new MediaSmaple
                        cObjectPtr<IMediaSample> pMediaSampleLaneFollowerStart;
                        AllocMediaSample((tVoid**)&pMediaSampleLaneFollowerStart);
                        // Send the Media Sample
                        cObjectPtr<IMediaSerializer> pSerializerLaneFollowerStart;
                        m_pDescriptionOutputLaneFollowerStart->GetMediaSampleSerializer(&pSerializerLaneFollowerStart);
                        tInt nSizeLaneFollowerStart = pSerializerLaneFollowerStart->GetDeserializedSize();
                        pMediaSampleLaneFollowerStart->AllocBuffer(nSizeLaneFollowerStart);
                        cObjectPtr<IMediaCoder> pCoderOutputLaneFollowerStart;
                        m_pDescriptionOutputLaneFollowerStart->WriteLock(pMediaSampleLaneFollowerStart, &pCoderOutputLaneFollowerStart);
                        pCoderOutputLaneFollowerStart->Set("bValue", (tVoid*)&(bLaneFollowerEnabled));
                        pCoderOutputLaneFollowerStart->Set("ui32ArduinoTimestamp", (tVoid*)&timestamp);
                        m_pDescriptionOutputLaneFollowerStart->Unlock(pCoderOutputLaneFollowerStart);
                        pMediaSampleLaneFollowerStart->SetTime(_clock->GetStreamTime());
                        m_oOutputLaneFollowerStart.Transmit(pMediaSampleLaneFollowerStart);
*/
                    }
                    break;

                case action_STOP:       // = -1
                //    if(m_bDebugModeEnabled)  LOG_INFO(cString::Format("Situation Detection: Received Stop with maneuver ID %d",i16entry));

                    SendDriverStruct(stateCar_STARTUP, i16entry);
                    // Reset the ID to the latest section
                    m_iManeuverIDSectorCounter = 0;
                    m_iManeuverIDOverallCounter = m_sectorList[m_iSectorIDCounter].maneuverList[m_iManeuverIDSectorCounter].id;
                    StopVehicle();
                    m_bCarStateRunning = false;
                    break;

				/*
                case -2:
                    if(m_bDebugModeEnabled)  LOG_INFO(cString::Format("Situation Detection: Received no i8ActionID from JuryStruct, maneuver ID %d",i16entry));
                    SendDriverStruct(stateCar_ERROR, i16entry);
                    break;
                    // TODO Case Emergency Brake? -> will they do a STOP, START or REQUEST action after triggering the EB
                    // TODO Stop the running LaneFollower or function -> send false value to all functions
				*/
                }

            }
            else
            {
                // ManeuverList is not processed yet
                SendDriverStruct(stateCar_STARTUP, 0);
                LOG_ERROR(adtf_util::cString::Format("No ManeuverList received"));
            }

        }

        // ManeuverList received
        else if(pSource == &m_oInputManeuverList && m_pDescriptionInputManeuverList != NULL)
		{
            // if(m_bDebugModeEnabled) LOG_INFO(cString::Format("Situation Detection: ManeuverList received"));
            {
                StopVehicle();

                // focus for sample read lock
                __adtf_sample_read_lock_mediadescription(m_pDescriptionInputManeuverList,pMediaSample,pCoder);
                std::vector<tSize> vecDynamicIDs;
                // retrieve number of elements by providing NULL as first paramter
                tSize szBufferSize = 0;
                if(IS_OK(pCoder->GetDynamicBufferIDs(NULL, szBufferSize)))
                {
                    // create a buffer depending on the size element
                    tChar* pcBuffer = new tChar[szBufferSize];
                    vecDynamicIDs.resize(szBufferSize);
                    // get the dynamic ids (we already got the first "static" size element)
                    if (IS_OK(pCoder->GetDynamicBufferIDs(&(vecDynamicIDs.front()), szBufferSize)))
                    {
                        // iterate over all elements
                        for (tUInt32 nIdx = 0; nIdx < vecDynamicIDs.size(); ++nIdx)
                        {
                            // get the value and put it into the buffer
                            pCoder->Get(vecDynamicIDs[nIdx], (tVoid*)&pcBuffer[nIdx]);
                        }
                        // set the resulting char buffer to the string object
                        m_strManeuverFileString = (const tChar*) pcBuffer;
                        //if(m_bDebugModeEnabled) LOG_INFO(cString::Format("Situation Detection: ManeuverList Sring %s", m_strManeuverFileString.c_str()));
                    }
                    // cleanup the buffer
                    delete pcBuffer;
                }
            }
            LoadManeuverList();
            // Send "Ready to Start" and send the very first Maneuver ID to the Jury
            m_iManeuverIDOverallCounter = m_sectorList[0].maneuverList[0].id;
            SendDriverStruct(stateCar_READY, m_iManeuverIDOverallCounter);
            m_bManeuverListProcessed = true;
        }

        if(m_bCarStateRunning){
            // Road sign detected
            if (pSource == &m_oInputTrafficSign)
            {
                __adtf_sample_read_lock_mediadescription(m_pDescriptionInputTrafficSign,pMediaSample,pCoderInput);
                // During a Maneuver no road sign should be processed
                if(!m_bManeuverInProcess){
                    tInt8 i8CurrentRoadSignID;
                    // get IDs
                    if(!m_bIDsRoadSignSet)
                    {
                            pCoderInput->GetID("i16Identifier",m_szIDRoadSignI16Identifier);
                            pCoderInput->GetID("f32Imagesize", m_szIDRoadSignF32Imagesize);
                            pCoderInput->GetID("af32TVec", m_szIDRoadSignTVec);
                            pCoderInput->GetID("af32RVec", m_szIDRoadSignRVec);
                            m_bIDsRoadSignSet = true;
                    }
                    pCoderInput->Get("i16Identifier", (tVoid*)&i8CurrentRoadSignID);
                    pCoderInput->Get("f32Imagesize", (tVoid*)&m_fTrafficSignImageSize);
                    pCoderInput->Get("af32TVec", (tVoid*)&m_f32ArrayRoadSignDistanceY);
                    pCoderInput->Get("af32RVec", (tVoid*)&m_f32ArrayRoadSignDistanceX);

                    /*
                    cObjectPtr<IMediaCoder> pCoderInput;
                    RETURN_IF_FAILED(m_pDescriptionInputTrafficSign->Lock(pMediaSample, &pCoderInput));
                    pCoderInput->Get("i16Identifier", (tVoid*)&i8CurrentRoadSignID);
                    pCoderInput->Get("f32Imagesize", (tVoid*)&m_fTrafficSignImageSize);
                    pCoderInput->Get("af32TVec", (tVoid*)&m_f32PositionArray);
                    m_pDescriptionInputTrafficSign->Unlock(pCoderInput);
                    */

                    // Only process a road sign if the same Road sign was detected 3 times in a row
                    //if(m_bDebugModeEnabled) LOG_INFO(adtf_util::cString::Format("current Road sign ID %i",i8CurrentRoadSignID));
                    //if(m_bDebugModeEnabled) LOG_INFO(adtf_util::cString::Format("Previous Road sign ID %i",m_iPreviousTrafficSignID));

                    m_f32RoadSignDistanceY = m_f32ArrayRoadSignDistanceY[2];
                    m_f32RoadSignDistanceX = m_f32ArrayRoadSignDistanceX[2];
                    // LOG_INFO(adtf_util::cString::Format("m_f32RoadSignDistanceY %f",m_f32RoadSignDistanceY));

                    tFloat32 f32Distance2RoadSign;
                    f32Distance2RoadSign = sqrt((m_f32RoadSignDistanceY*m_f32RoadSignDistanceY) + (m_f32RoadSignDistanceX*m_f32RoadSignDistanceX));

                    // ZEBRA-CROSSING - SIGN
                    if(i8CurrentRoadSignID == 6)
                    {
                        if(f32Distance2RoadSign < 1.7)
                        {
                            EvaluateRoadSign(i8CurrentRoadSignID);
                        }
                        m_bParkingSignDetected = false;
                    }
                    // PARKING - SIGN
                    else if(i8CurrentRoadSignID == 2)
                    {
                        if(f32Distance2RoadSign < 1.2)
                        {
                            // Set boolean on true
                            m_bParkingSignDetected = true;
                            m_bParkingSpotAhead = true;
                            // Reset counter for triggering
                            m_iParkingSignTriggerCounter = 0;
                            EvaluateRoadSign(2);
                        }


                    }
                    // ALL OTHER ROAD SIGNS - CROSSING
                    else
                    {
                        // Only Road sign closer than 1.2 meters are evaluated for maneuvers
                        if(f32Distance2RoadSign < 1.2)
                        {
                            if(i8CurrentRoadSignID == m_iPreviousTrafficSignID)
                            {
                                EvaluateRoadSign(i8CurrentRoadSignID);
                            }
                            m_iPreviousTrafficSignID = i8CurrentRoadSignID; // m_iCurrentTrafficSignID;
                        }
                        m_bParkingSignDetected = false;
                    }

                }
            }

            // Received Information about StopLine
            else if(pSource == &m_oInputStoplineDetection)
            {

                m_bInputStoplineDetected = false;
                m_fInputStoplineDistance = 0;

                cObjectPtr<IMediaCoder> pCoderInput;
                RETURN_IF_FAILED(m_pDescriptionInputStoplineDetection->Lock(pMediaSample, &pCoderInput));
                pCoderInput->Get("bValue", (tVoid*)&m_bInputStoplineDetected);
                pCoderInput->Get("f32Distance", (tVoid*)&m_fInputStoplineDistance);
                m_pDescriptionInputStoplineDetection->Unlock(pCoderInput);

                if(m_bInputStoplineDetected){
                   //if(m_bDebugModeEnabled) LOG_INFO(adtf_util::cString::Format("Situation Detection: Positive Stopline received: %i",m_bInputStoplineDetected));
                   //if(m_bDebugModeEnabled) LOG_INFO(adtf_util::cString::Format("Situation Detection: Distance to StopLine %f", m_fInputStoplineDistance));
                }
                if(m_fInputStoplineDistance == 0){
                    m_bInputStoplineDetected = false;
                }
                // StartCrossing(false, 0);


                // ### THIS IS FOR TRIGGERING PARKING AND SCANNING ###

                // Check if a Parking Sign was detected
                if(m_bParkingSpotAhead)
                {
                    // Check the car passed the sign
                    if(m_bParkingSignDetected == false)
                    {
                        // Check if the sign is passed for a certain amount of calls
                        if(m_iParkingSignTriggerCounter > 10)
                        {
                            // Trigger Parking or scanning by evaluating the Road Sign
                            EvaluateRoadSign(2);
                            m_bParkingSpotAhead = false;
                           //  if(m_bDebugModeEnabled) LOG_INFO(adtf_util::cString::Format("Situation Detection: Trigger EvaluateRoadSign(2): m_bParkingSignDetected = %i, m_iParkingSignTriggerCounter = %i",m_bParkingSignDetected, m_iParkingSignTriggerCounter));
                        }
                        m_iParkingSignTriggerCounter++;
                    }
                    // Reset the boolean
                    m_bParkingSignDetected = false;
                }


                // ### END ###





                /*
                 * ### Crossing without ROAD SIGN
                 *
                // if no traffic sign is detected
                if(tsLastTimeRoadSignDetected + 5000000 < _clock->GetStreamTime()){
                    // but a stopline and a edgeline are detected
                    // TODO and edges
                    if(m_bInputStoplineDetected && m_bInputEdge1Detected ){
                        StartCrossing(true, 0);
                    }
                }
                *
                *
                */

            }

            // Received Information about Edgeline
            else if(pSource == &m_oInputEdgelineDetection){
                m_bInputEdgeLineDetected = false;
                m_fInputEdgeLineDistance = 0;

                cObjectPtr<IMediaCoder> pCoderInput;
                RETURN_IF_FAILED(m_pDescriptionInputEdgelineDetection->Lock(pMediaSample, &pCoderInput));
                pCoderInput->Get("bValue", (tVoid*)&m_bInputEdgeLineDetected);
                pCoderInput->Get("f32Distance", (tVoid*)&m_fInputEdgeLineDistance);
                m_pDescriptionInputEdgelineDetection->Unlock(pCoderInput);

                if(m_fInputEdgeLineDistance == 0){
                    m_bInputEdgeLineDetected = false;
                }

                if(m_bInputEdgeLineDetected){
                   //if(m_bDebugModeEnabled) LOG_INFO(adtf_util::cString::Format("Situation Detection: Positive Edgeline received: %i",m_bInputEdgeLineDetected));
                   //if(m_bDebugModeEnabled) LOG_INFO(adtf_util::cString::Format("Situation Detection: Distance to EdgeLine %f", m_fInputEdgeLineDistance));
                }
                // StartCrossing(false);
            }

            // Received Information about Edges
            else if(pSource == &m_oInputEdgeDetection){
                // Reset the values
                m_bInputEdge1Detected = false;
                m_fInputEdge1Distance = 0;
                //m_bInputEdge2Detected = false;
                //m_fInputEdge2Distance = 0;

                cObjectPtr<IMediaCoder> pCoderInput;
                RETURN_IF_FAILED(m_pDescriptionInputEdgeDetection->Lock(pMediaSample, &pCoderInput));
                pCoderInput->Get("bValue1", (tVoid*)&m_bInputEdge1Detected);
                pCoderInput->Get("f32Distance1", (tVoid*)&m_fInputEdge1Distance);
               // pCoderInput->Get("bValue2", (tVoid*)&m_bInputEdge2Detected);
               // pCoderInput->Get("f32Distance2", (tVoid*)&m_fInputEdge2Distance);
                m_pDescriptionInputEdgeDetection->Unlock(pCoderInput);

               /*
                if(m_fInputEdge2Distance <= 0){
                    m_bInputEdge2Detected = false;
                }
                if(m_fInputEdge2Distance){
                    if(m_bDebugModeEnabled) LOG_INFO(adtf_util::cString::Format("Situation Detection: Positive Edge 2 received"));
                    if(m_bDebugModeEnabled) LOG_INFO(adtf_util::cString::Format("Situation Detection: Distance to Edge 2 = %f", m_fInputEdge2Distance));
                }
                */

            }

            // Received Feedback from ACC - Request for Lane Change
            else if(pSource == &m_oInputACC)
            {
               // if(m_bDebugModeEnabled) LOG_INFO(adtf_util::cString::Format("ACC Requst received"));
                tBool bInputACCLaneChangeRequested = false;
                // Check what is the value
                cObjectPtr<IMediaCoder> pCoderInput;
                RETURN_IF_FAILED(m_pDescriptionInputACC->Lock(pMediaSample, &pCoderInput));
                pCoderInput->Get("bValue", (tVoid*)&bInputACCLaneChangeRequested);
                m_pDescriptionInputACC->Unlock(pCoderInput);

                // positive Feedback = Lane Change is finished
                if(bInputACCLaneChangeRequested){
                    if(m_bDebugModeEnabled) LOG_INFO(adtf_util::cString::Format("ACC Requst is positive -> call EvaluateLaneChange()"));
                    // call evaluate Lane Change Request function
                    EvaluateLaneChange();
                }
            }

            // Received Feedback from LaneChange - LaneChange Finished
            else if(pSource == &m_oInputLaneChangeFinished)
            {
              //  if(m_bDebugModeEnabled) LOG_INFO(adtf_util::cString::Format("Received Feedback from LaneChange"));
                tBool bInputFeedbackLaneChange = false;
                // Check what is the value
                cObjectPtr<IMediaCoder> pCoderInput;
                RETURN_IF_FAILED(m_pDescriptionInputLaneChangeFinished->Lock(pMediaSample, &pCoderInput));
                pCoderInput->Get("bValue", (tVoid*)&bInputFeedbackLaneChange);
                m_pDescriptionInputLaneChangeFinished->Unlock(pCoderInput);

                // Lane change completed -> get control back to lane followewr
                if(bInputFeedbackLaneChange)
                {
               //     if(m_bDebugModeEnabled) LOG_INFO(adtf_util::cString::Format("LANE CHANGE FINISHED"));
                    m_bManeuverInProcess = false;

                    // Send bool with false to LaneChange
                    tBool bStartLaneChange = false;
          //          if(m_bDebugModeEnabled) LOG_INFO(adtf_util::cString::Format("Send order to LaneChange"));
                    // Create a new MediaSmaple
                    cObjectPtr<IMediaSample> pMediaSampleLaneChangeStart;
                    AllocMediaSample((tVoid**)&pMediaSampleLaneChangeStart);
                    // Send the Media Sample
                    cObjectPtr<IMediaSerializer> pSerializerLaneChangeStart;
                    m_pDescriptionOutputLaneChangeStart->GetMediaSampleSerializer(&pSerializerLaneChangeStart);
                    tInt nSizeLaneChangeStart = pSerializerLaneChangeStart->GetDeserializedSize();
                    pMediaSampleLaneChangeStart->AllocBuffer(nSizeLaneChangeStart);
                    cObjectPtr<IMediaCoder> pCoderOutputLaneChangeStart;
                    m_pDescriptionOutputLaneChangeStart->WriteLock(pMediaSampleLaneChangeStart, &pCoderOutputLaneChangeStart);
                    pCoderOutputLaneChangeStart->Set("bValue", (tVoid*)&(bStartLaneChange));
                    pCoderOutputLaneChangeStart->Set("ui32ArduinoTimestamp", (tVoid*)&timestamp);
                    m_pDescriptionOutputLaneChangeStart->Unlock(pCoderOutputLaneChangeStart);
                    pMediaSampleLaneChangeStart->SetTime(_clock->GetStreamTime());
                    m_oOutputLaneChangeStart.Transmit(pMediaSampleLaneChangeStart);

                    // Send start to lane follower
                    tInt iStop = -1;
                    SendOutputs(true, false, iStop, iStop);
                }
            }


            // Received Feedpack from Scanning filter
            else if(pSource == &m_oInputScanningFinished){
                tBool bInputScanningFinished;
                // Check what is the value
                cObjectPtr<IMediaCoder> pCoderInput;
                RETURN_IF_FAILED(m_pDescriptionInputScanningFinished->Lock(pMediaSample, &pCoderInput));
                pCoderInput->Get("bValue", (tVoid*)&bInputScanningFinished);
                m_pDescriptionInputScanningFinished->Unlock(pCoderInput);

                // positive Feedback = parking is finished
                if(bInputScanningFinished){
                  //  if(m_bDebugModeEnabled) LOG_INFO(adtf_util::cString::Format("SCANNING FINISHED"));
                    m_bManeuverInProcess = false;

                    Command2LaneFollower(true);

                    // Send start signal to scanning-filter
                    tBool bStartSignal = false;
                    // Create a new MediaSmaple
                    cObjectPtr<IMediaSample> pMediaSampleScanningStart;
                    AllocMediaSample((tVoid**)&pMediaSampleScanningStart);
                    // Send the Media Sample
                    cObjectPtr<IMediaSerializer> pSerializerScanningStart;
                    m_pDescriptionOutputScanningStart->GetMediaSampleSerializer(&pSerializerScanningStart);
                    tInt nSizeScanningStart = pSerializerScanningStart->GetDeserializedSize();
                    pMediaSampleScanningStart->AllocBuffer(nSizeScanningStart);
                    cObjectPtr<IMediaCoder> pCoderOutputScanningStart;
                    m_pDescriptionOutputScanningStart->WriteLock(pMediaSampleScanningStart, &pCoderOutputScanningStart);
                    pCoderOutputScanningStart->Set("bValue", (tVoid*)&(bStartSignal));
                    m_pDescriptionOutputScanningStart->Unlock(pCoderOutputScanningStart);
                    pMediaSampleScanningStart->SetTime(_clock->GetStreamTime());
                    m_oOutputScanningStart.Transmit(pMediaSampleScanningStart);

                }
            }


            // Received Feedpack from Parking filter
            else if(pSource == &m_oInputParkingFinished)
            {
                tBool bInputParkingFinished = false;
                // Check what is the value
                cObjectPtr<IMediaCoder> pCoderInput;
                RETURN_IF_FAILED(m_pDescriptionInputParkingFinished->Lock(pMediaSample, &pCoderInput));
                pCoderInput->Get("bValue", (tVoid*)&bInputParkingFinished);
                m_pDescriptionInputParkingFinished->Unlock(pCoderInput);

                // positive Feedback = parking is finished
                if(bInputParkingFinished){
               //     if(m_bDebugModeEnabled) LOG_INFO(adtf_util::cString::Format("PARKING FINISHED"));
                    m_bManeuverInProcess = false;
                    // TODO Wait some time here
                    FinishManeuver("cross_parking");
                }
            }


            // Received Feedpack from PullOutLeft filter
            else if(pSource == &m_oInputPullOutLeftFinished)
            {
                tBool InputPullOutLeftFinished = false;
                // Check what is the value
            //    if(m_bDebugModeEnabled) LOG_INFO(adtf_util::cString::Format("PULLOULEFT FINISHED answered"));
                cObjectPtr<IMediaCoder> pCoderInput;
                RETURN_IF_FAILED(m_pDescriptionInputPullOutLeftFinished->Lock(pMediaSample, &pCoderInput));
                pCoderInput->Get("bValue", (tVoid*)&InputPullOutLeftFinished);
                m_pDescriptionInputPullOutLeftFinished->Unlock(pCoderInput);
                // positive Feedback = Crossing is finished
                if(InputPullOutLeftFinished){
                    if(m_bDebugModeEnabled) LOG_INFO(adtf_util::cString::Format("PULLOUTLEFT FINISHED"));
                    m_bManeuverInProcess = false;
                    // Send start signal to LaneFollower-filter
                    Command2LaneFollower(true);
                    FinishManeuver("pull_out_left");
                }
            }

            // Received Feedpack from PullOutRight filter
            else if(pSource == &m_oInputPullOutRightFinished)
            {
                tBool InputPullOutRightFinished = false;
                // Check what is the value
           //     if(m_bDebugModeEnabled) LOG_INFO(adtf_util::cString::Format("PULLOURIGHT FINISHED answered"));
                cObjectPtr<IMediaCoder> pCoderInput;
                RETURN_IF_FAILED(m_pDescriptionInputPullOutRightFinished->Lock(pMediaSample, &pCoderInput));
                pCoderInput->Get("bValue", (tVoid*)&InputPullOutRightFinished);
                m_pDescriptionInputPullOutRightFinished->Unlock(pCoderInput);
                // positive Feedback = Crossing is finished
                if(InputPullOutRightFinished)
                {
//                    if(m_bDebugModeEnabled) LOG_INFO(adtf_util::cString::Format("PULLOURIGHT FINISHED"));
                    m_bManeuverInProcess = false;
                    // Send start signal to LaneFollower-filter
                    Command2LaneFollower(true);
                    FinishManeuver("pull_out_right");
                }
            }

            // Received Feedpack from Crossing filter
            else if(pSource == &m_oInputCrossingFinished)
            {
                tBool bInputCrossingFinished = false;
                // Check what is the value
                cObjectPtr<IMediaCoder> pCoderInput;
                RETURN_IF_FAILED(m_pDescriptionInputCrossingFinished->Lock(pMediaSample, &pCoderInput));
                pCoderInput->Get("bValue", (tVoid*)&bInputCrossingFinished);
                m_pDescriptionInputCrossingFinished->Unlock(pCoderInput);

                // positive Feedback = Crossing is finished
                if(bInputCrossingFinished)
                {
              //      if(m_bDebugModeEnabled) LOG_INFO(adtf_util::cString::Format("CROSSING FINISHED"));
                    m_bManeuverInProcess = false;

                    tInt iStop = -1;

                    SendOutputs(true, false, iStop, iStop);

                    /*
                    // Send a stop sign to the crossing filter
                    // Create a new MediaSmaple
                    cObjectPtr<IMediaSample> pMediaSampleCrossingStart;
                    AllocMediaSample((tVoid**)&pMediaSampleCrossingStart);
                    // Send the Media Sample
                    cObjectPtr<IMediaSerializer> pSerializerCrossingStart;
                    m_pDescriptionOutputCrossingStart->GetMediaSampleSerializer(&pSerializerCrossingStart);
                    tInt nSizeCrossingStart = pSerializerCrossingStart->GetDeserializedSize();
                    pMediaSampleCrossingStart->AllocBuffer(nSizeCrossingStart);
                    cObjectPtr<IMediaCoder> pCoderOutputCrossingStart;
                    m_pDescriptionOutputCrossingStart->WriteLock(pMediaSampleCrossingStart, &pCoderOutputCrossingStart);
                    pCoderOutputCrossingStart->Set("bValue", (tVoid*)&(bOutputCrossingStart));
                    pCoderOutputCrossingStart->Set("i8RoadSignID", (tVoid*)&(iStop));
                    pCoderOutputCrossingStart->Set("i8CrossingManeuverID", (tVoid*)&(iStop));
                    pCoderOutputCrossingStart->Set("ui32ArduinoTimestamp", (tVoid*)&timestamp);
                    m_pDescriptionOutputCrossingStart->Unlock(pCoderOutputCrossingStart);
                    pMediaSampleCrossingStart->SetTime(_clock->GetStreamTime());
                    m_oOutputCrossingStart.Transmit(pMediaSampleCrossingStart);

                    // Send start signal to LaneFollower-filter
                    tBool bLaneFollowerEnabled = true;
                    // Send start signal to LaneFollower-filter
                    // Create a new MediaSmaple
                    cObjectPtr<IMediaSample> pMediaSampleLaneFollowerStart;
                    AllocMediaSample((tVoid**)&pMediaSampleLaneFollowerStart);
                    // Send the Media Sample
                    cObjectPtr<IMediaSerializer> pSerializerLaneFollowerStart;
                    m_pDescriptionOutputLaneFollowerStart->GetMediaSampleSerializer(&pSerializerLaneFollowerStart);
                    tInt nSizeLaneFollowerStart = pSerializerLaneFollowerStart->GetDeserializedSize();
                    pMediaSampleLaneFollowerStart->AllocBuffer(nSizeLaneFollowerStart);
                    cObjectPtr<IMediaCoder> pCoderOutputLaneFollowerStart;
                    m_pDescriptionOutputLaneFollowerStart->WriteLock(pMediaSampleLaneFollowerStart, &pCoderOutputLaneFollowerStart);
                    pCoderOutputLaneFollowerStart->Set("bValue", (tVoid*)&(bLaneFollowerEnabled));
                    pCoderOutputLaneFollowerStart->Set("ui32ArduinoTimestamp", (tVoid*)&timestamp);
                    m_pDescriptionOutputLaneFollowerStart->Unlock(pCoderOutputLaneFollowerStart);
                    pMediaSampleLaneFollowerStart->SetTime(_clock->GetStreamTime());
                    m_oOutputLaneFollowerStart.Transmit(pMediaSampleLaneFollowerStart);

                    */

                   // if(m_bDebugModeEnabled) LOG_INFO(adtf_util::cString::Format("Disable Crossing, after receiving FinishFlag"));
                   // if(m_bDebugModeEnabled) LOG_INFO(adtf_util::cString::Format("Start Lane Folloer, after finishing crossing"));

                    FinishManeuver("crossing");
                }
            }

            // Receive Feedback from Zebracrossing Filter
            else if(pSource == &m_oInputZebracrossingFinished)
            {
                tBool bZebraCrossingFinished = false;
                // Read out the Mediasample
                cObjectPtr<IMediaCoder> pCoderInput;
                RETURN_IF_FAILED(m_pDescriptionInputZebracrossingFinished->Lock(pMediaSample, &pCoderInput));
                pCoderInput->Get("bValue", (tVoid*)&bZebraCrossingFinished);
                m_pDescriptionInputZebracrossingFinished->Unlock(pCoderInput);

                if(bZebraCrossingFinished)
                {
                    m_bManeuverInProcess = false;
                }
            }
        }
    }
    RETURN_NOERROR;
}



tResult cSituationDetection::EvaluateRoadSign(tInt8 i8RoadSignID) {
    if(!m_bCarStateRunning)
    {
        StopVehicle();
    }
    else
    {
        //if(m_bDebugModeEnabled) LOG_INFO(adtf_util::cString::Format("Traffic Sign# %i",i8RoadSignID));

        switch(i8RoadSignID)
        {
            // MARKER_ID_UNMARKEDINTERSECTION
            case 0 : // if(m_bDebugModeEnabled) LOG_INFO(adtf_util::cString::Format("UNMARKEDINTERSECTION"));
                m_bRoadSignEvaluated = true;
                StartCrossing(false, i8RoadSignID);
            break;

            // MARKER_ID_STOPANDGIVEWAY
            case 1 : //if(m_bDebugModeEnabled) LOG_INFO(adtf_util::cString::Format("STOPANDGIVEWAY"));
                m_bRoadSignEvaluated = true;
                StartCrossing(false, i8RoadSignID);
            break;

            // MARKER_ID_PARKINGAREA
            case 2 :// if(m_bDebugModeEnabled) LOG_INFO(adtf_util::cString::Format("PARKINAREA"));  // --> Signal to Parking or Scanning

                tFloat32 f32Distance2RoadSign;
                f32Distance2RoadSign = sqrt((m_f32RoadSignDistanceY*m_f32RoadSignDistanceY) + (m_f32RoadSignDistanceX*m_f32RoadSignDistanceX));

                // Trigger Parking only after passing the actual RoadSign
                if((m_bParkingSignDetected == false && m_iParkingSignTriggerCounter > 10) || f32Distance2RoadSign  < 0.5)
                {
                    // Reset the counter
                    m_iParkingSignTriggerCounter = 0;
                    // Get the next maneuver as string
                    cString strNextManeuver = m_sectorList[m_iSectorIDCounter].maneuverList[m_iManeuverIDSectorCounter].action;

                    // Is the next maneuver in the list "cross_parking" + ID
                    if(strNextManeuver.Find("cross_parking") != -1)
                    {
                                tInt8 i8ParkingID;
                                // Get the last character of the string strNextManeuver
                                i8ParkingID = atoi(strNextManeuver.Right(1));
                                //if(m_bDebugModeEnabled) LOG_INFO(adtf_util::cString::Format("Current parking ID: %i",i8ParkingID));

                                if(i8ParkingID < 1 || 8 < i8ParkingID){
                                    // if(m_bDebugModeEnabled) LOG_ERROR(adtf_util::cString::Format("Wrong parking ID: %i",i8ParkingID));
                                }
                                m_bSignalParkingSign = false;


                                // Map the parking ID from 1-8 to 8-1
                                i8ParkingID = 9 - i8ParkingID;


                                Command2LaneFollower(false);
                                StartParking(i8ParkingID);
                    }
                    // Start scanning for SLAM
                    else
                    {
                                // if(m_bDebugModeEnabled) LOG_INFO(adtf_util::cString::Format("Parking is not the next order in the maneuver List, start scanning for SLAM"));
                                Command2LaneFollower(false);
                                StartScanning();
                    }

                }

                // No sign detected -> m_iParkingSignTriggerCounter++
                if(m_bParkingSignDetected == false)
                {
                    m_iParkingSignTriggerCounter++;
                }
                // Sign detected again -> reset counter
                else
                {
                    m_iParkingSignTriggerCounter = 0;
                    m_bParkingSignDetected = false;
                }


            break;

            // MARKER_ID_HAVEWAY
            case 3 : // if(m_bDebugModeEnabled) LOG_INFO(adtf_util::cString::Format("HAVEWAY"));
                m_bRoadSignEvaluated = true;
                StartCrossing(false, i8RoadSignID);

            break;

            // MARKER_ID_AHEADONLY
            case 4 : //if(m_bDebugModeEnabled) LOG_INFO(adtf_util::cString::Format("AHEADONLY"));
                 m_bRoadSignEvaluated = true;
                 StartCrossing(false, i8RoadSignID);
            break;

            // MARKER_ID_GIVEWAY
            case 5 : //if(m_bDebugModeEnabled) LOG_INFO(adtf_util::cString::Format("GIVEWAY"));
                m_bRoadSignEvaluated = true;
                StartCrossing(false, i8RoadSignID);
            break;

            // MARKER_ID_PEDESTRIANCROSSING
            case 6 : // if(m_bDebugModeEnabled) LOG_INFO(adtf_util::cString::Format("PEDESTRIANCROSSING"));
                StartZebraCrossing();
            break;

            // MARKER_ID_ROUNDABOUT
            case 7 :  // if(m_bDebugModeEnabled) LOG_INFO(adtf_util::cString::Format("ROUNDABOUT"));
                // DO NOTHING
            break;

            // MARKER_ID_NOOVERTAKING
            case 8 : //  if(m_bDebugModeEnabled) LOG_INFO(adtf_util::cString::Format("NOOVERTAKING"));
                // DO NOTHING
            break;

            // MARKER_ID_NOENTRYVEHICULARTRAFFIC
            case 9 :  // if(m_bDebugModeEnabled) LOG_INFO(adtf_util::cString::Format("NOENTRYVEHICULARTRAFFIC"));
                // DO NOTHING
            break;

            // MARKER_ID_TESTCOURSEA9
            case 10 : // if(m_bDebugModeEnabled) LOG_INFO(adtf_util::cString::Format("TESTCOURSEA9"));
                // DO NOTHING
            break;

            // MARKER_ID_ONEWAYSTREET
            case 11 :  // if(m_bDebugModeEnabled) LOG_INFO(adtf_util::cString::Format("ONEWAYSTREET"));
                // DO NOTHING
            break;

            // MARKER_ID_ROADWORKS
            case 12 :   // if(m_bDebugModeEnabled) LOG_INFO(adtf_util::cString::Format("ROADWORKS"));
                // drive slow
            break;

            // MARKER_ID_KMH50
            case 13 : // if(m_bDebugModeEnabled) LOG_INFO(adtf_util::cString::Format("KMH50"));
                // TODO Set max valocity at 50 km/h
            break;

            // MARKER_ID_KMH100
            case 14 : // if(m_bDebugModeEnabled) LOG_INFO(adtf_util::cString::Format("KMH100"));
                // TODO Set max valocity at 100 km/h
            break;

            // MARKER_ID_NOMATCH
            case 99 : // if(m_bDebugModeEnabled) LOG_INFO(adtf_util::cString::Format("NOMATCH"));
                // Do nothing
            break;
            RETURN_NOERROR;
        }
    }
    RETURN_NOERROR;
}


tResult cSituationDetection::EvaluateLaneChange(){
    if(!m_bCarStateRunning)
    {
        StopVehicle();
    }
    if(m_bManeuverInProcess)
    {
        // Do nothing
        // if(m_bDebugModeEnabled) LOG_INFO(adtf_util::cString::Format("Situation Detection: A maneuver is already in process, starting a lanechange is not allowed"));
    }
    else
    {
        tBool bStartLaneChange = true;
        m_bManeuverInProcess = true;
        // Send order to LaneChange
        // Disable LaneFollower
        Command2LaneFollower(false);

      //  if(m_bDebugModeEnabled) LOG_INFO(adtf_util::cString::Format("Send order to LaneChange"));
        // Create a new MediaSmaple
        cObjectPtr<IMediaSample> pMediaSampleLaneChangeStart;
        AllocMediaSample((tVoid**)&pMediaSampleLaneChangeStart);
        // Send the Media Sample
        cObjectPtr<IMediaSerializer> pSerializerLaneChangeStart;
        m_pDescriptionOutputLaneChangeStart->GetMediaSampleSerializer(&pSerializerLaneChangeStart);
        tInt nSizeLaneChangeStart = pSerializerLaneChangeStart->GetDeserializedSize();
        pMediaSampleLaneChangeStart->AllocBuffer(nSizeLaneChangeStart);
        cObjectPtr<IMediaCoder> pCoderOutputLaneChangeStart;
        m_pDescriptionOutputLaneChangeStart->WriteLock(pMediaSampleLaneChangeStart, &pCoderOutputLaneChangeStart);
        pCoderOutputLaneChangeStart->Set("bValue", (tVoid*)&(bStartLaneChange));
        pCoderOutputLaneChangeStart->Set("ui32ArduinoTimestamp", (tVoid*)&timestamp);
        m_pDescriptionOutputLaneChangeStart->Unlock(pCoderOutputLaneChangeStart);
        pMediaSampleLaneChangeStart->SetTime(_clock->GetStreamTime());
        m_oOutputLaneChangeStart.Transmit(pMediaSampleLaneChangeStart);
    }
    RETURN_NOERROR;
}


tResult cSituationDetection::Command2LaneFollower(tBool bLaneFollowerEnabled){
    if(!m_bCarStateRunning)
    {
        StopVehicle();
    }
    else
    {
     //   if(m_bDebugModeEnabled) LOG_INFO(adtf_util::cString::Format("Command2LaneFollower: bLaneFollowerEnabled %i",bLaneFollowerEnabled));

        // Send start signal to LaneFollower-filter
        // Create a new MediaSmaple
        cObjectPtr<IMediaSample> pMediaSampleLaneFollowerStart;
        AllocMediaSample((tVoid**)&pMediaSampleLaneFollowerStart);
        // Send the Media Sample
        cObjectPtr<IMediaSerializer> pSerializerLaneFollowerStart;
        m_pDescriptionOutputLaneFollowerStart->GetMediaSampleSerializer(&pSerializerLaneFollowerStart);
        tInt nSizeLaneFollowerStart = pSerializerLaneFollowerStart->GetDeserializedSize();
        pMediaSampleLaneFollowerStart->AllocBuffer(nSizeLaneFollowerStart);
        cObjectPtr<IMediaCoder> pCoderOutputLaneFollowerStart;
        m_pDescriptionOutputLaneFollowerStart->WriteLock(pMediaSampleLaneFollowerStart, &pCoderOutputLaneFollowerStart);
        pCoderOutputLaneFollowerStart->Set("bValue", (tVoid*)&(bLaneFollowerEnabled));
        pCoderOutputLaneFollowerStart->Set("ui32ArduinoTimestamp", (tVoid*)&timestamp);
        m_pDescriptionOutputLaneFollowerStart->Unlock(pCoderOutputLaneFollowerStart);
        pMediaSampleLaneFollowerStart->SetTime(_clock->GetStreamTime());
        m_oOutputLaneFollowerStart.Transmit(pMediaSampleLaneFollowerStart);
    }
    RETURN_NOERROR;
}


tResult cSituationDetection::StartParking(tInt8 i_i8ParkingID){
    if(!m_bCarStateRunning)
    {
        StopVehicle();
    }
    else
    {
        // Set this true so no other road sign can be processed until the current maneuver
        if(m_bEnableManeuverInProcessFlag) m_bManeuverInProcess = true;

        // Send start signal to parking-filter

        tBool bOutputParkingStart = true;

        // Create a new MediaSmaple
        cObjectPtr<IMediaSample> pMediaSampleParkingStart;
        AllocMediaSample((tVoid**)&pMediaSampleParkingStart);

        // Send the Media Sample
        cObjectPtr<IMediaSerializer> pSerializerParkingStart;
        m_pDescriptionOutputParkingStart->GetMediaSampleSerializer(&pSerializerParkingStart);
        tInt nSizeParkingStart = pSerializerParkingStart->GetDeserializedSize();
        pMediaSampleParkingStart->AllocBuffer(nSizeParkingStart);
        cObjectPtr<IMediaCoder> pCoderOutputParkingStart;
        m_pDescriptionOutputParkingStart->WriteLock(pMediaSampleParkingStart, &pCoderOutputParkingStart);
        pCoderOutputParkingStart->Set("bValue", (tVoid*)&(bOutputParkingStart));
        pCoderOutputParkingStart->Set("i8ParkingID", (tVoid*)&i_i8ParkingID);
        m_pDescriptionOutputParkingStart->Unlock(pCoderOutputParkingStart);
        pMediaSampleParkingStart->SetTime(_clock->GetStreamTime());
        m_oOutputParkingStart.Transmit(pMediaSampleParkingStart);
    }
    RETURN_NOERROR;
}


tResult cSituationDetection::StartScanning(){

    if(!m_bCarStateRunning)
    {
        StopVehicle();
    }
    else
    {
        // Set this true so no other road sign can be processed until the current maneuver
        if(m_bEnableManeuverInProcessFlag) m_bManeuverInProcess = true;

        // Send start signal to scanning-filter
        tBool bStartSignal = true;
        // Create a new MediaSmaple
        cObjectPtr<IMediaSample> pMediaSampleScanningStart;
        AllocMediaSample((tVoid**)&pMediaSampleScanningStart);
        // Send the Media Sample
        cObjectPtr<IMediaSerializer> pSerializerScanningStart;
        m_pDescriptionOutputScanningStart->GetMediaSampleSerializer(&pSerializerScanningStart);
        tInt nSizeScanningStart = pSerializerScanningStart->GetDeserializedSize();
        pMediaSampleScanningStart->AllocBuffer(nSizeScanningStart);
        cObjectPtr<IMediaCoder> pCoderOutputScanningStart;
        m_pDescriptionOutputScanningStart->WriteLock(pMediaSampleScanningStart, &pCoderOutputScanningStart);
        pCoderOutputScanningStart->Set("bValue", (tVoid*)&(bStartSignal));
        m_pDescriptionOutputScanningStart->Unlock(pCoderOutputScanningStart);
        pMediaSampleScanningStart->SetTime(_clock->GetStreamTime());
        m_oOutputScanningStart.Transmit(pMediaSampleScanningStart);
    }
    RETURN_NOERROR;
}


tResult cSituationDetection::StartPullOut(){

    if(!m_bCarStateRunning){
         StopVehicle();
    } else {
        tInt16 iPullOutDirectionID = -1;
        // Order by ManeuverList
        if(!m_bTestPullOutEnabled && !m_bTestParkingEnabled){
            // Get the direction of pulling out from the maneuver list
            if(m_sectorList[m_iSectorIDCounter].maneuverList[m_iManeuverIDSectorCounter].action.IsEqual("pull_out_left")){
                iPullOutDirectionID = 0;
            } else if (m_sectorList[m_iSectorIDCounter].maneuverList[m_iManeuverIDSectorCounter].action.IsEqual("pull_out_right")){
                iPullOutDirectionID = 1;
            } else {
                // TODO When we do get an invalid order in iPullOutDirectionID --> send an ERROR flag to the Jury model (propery for this)
                iPullOutDirectionID = -1;
            }
        }
        // Order by Test Mode
        else {
            // Trigger is Bool-Generator for PullOut-Filter
            if(m_iTestPullOutDirectionID < -1 || 1 < m_iTestPullOutDirectionID){
                // Error
                LOG_ERROR(adtf_util::cString::Format("Error"));
                LOG_ERROR(adtf_util::cString::Format("An invalid value for the Property m_iTestPullOutDirectionID was set"));
                LOG_ERROR(adtf_util::cString::Format("Valid values are: -1 (Error), 0 (left), 1 (right)"));
                m_iTestPullOutDirectionID = -1;
            } else {
                iPullOutDirectionID = m_iTestPullOutDirectionID;
            }
        }


        // PullOutLeft
        if(iPullOutDirectionID == 0){
            // Send start signal to PullOutLeft-filter
            m_bOutputPullOutLeftStart = true;
            if(m_bEnableManeuverInProcessFlag) m_bManeuverInProcess = true;
            // Create a new MediaSmaple
            cObjectPtr<IMediaSample> pMediaSamplePullOutLeftStart;
            AllocMediaSample((tVoid**)&pMediaSamplePullOutLeftStart);
            // Send the Media Sample
            cObjectPtr<IMediaSerializer> pSerializerPullOutLeftStart;
            m_pDescriptionOutputPullOutLeftStart->GetMediaSampleSerializer(&pSerializerPullOutLeftStart);
            tInt nSizePullOutLeftStart = pSerializerPullOutLeftStart->GetDeserializedSize();
            pMediaSamplePullOutLeftStart->AllocBuffer(nSizePullOutLeftStart);
            cObjectPtr<IMediaCoder> pCoderOutputPullOutLeftStart;
            m_pDescriptionOutputPullOutLeftStart->WriteLock(pMediaSamplePullOutLeftStart, &pCoderOutputPullOutLeftStart);
            pCoderOutputPullOutLeftStart->Set("bValue", (tVoid*)&(m_bOutputPullOutLeftStart));
            pCoderOutputPullOutLeftStart->Set("ui32ArduinoTimestamp", (tVoid*)&timestamp);
            m_pDescriptionOutputPullOutLeftStart->Unlock(pCoderOutputPullOutLeftStart);
            pMediaSamplePullOutLeftStart->SetTime(_clock->GetStreamTime());
            m_oOutputPullOutLeftStart.Transmit(pMediaSamplePullOutLeftStart);
        }
        // PullOutRight
        else if(iPullOutDirectionID == 1){
            // Send start signal to PullOutRight-filter
            m_bOutputPullOutRightStart = true;
            if(m_bEnableManeuverInProcessFlag) m_bManeuverInProcess = true;
            // Create a new MediaSmaple
            cObjectPtr<IMediaSample> pMediaSamplePullOutRightStart;
            AllocMediaSample((tVoid**)&pMediaSamplePullOutRightStart);
            // Send the Media Sample
            cObjectPtr<IMediaSerializer> pSerializerPullOutRightStart;
            m_pDescriptionOutputPullOutRightStart->GetMediaSampleSerializer(&pSerializerPullOutRightStart);
            tInt nSizePullOutRightStart = pSerializerPullOutRightStart->GetDeserializedSize();
            pMediaSamplePullOutRightStart->AllocBuffer(nSizePullOutRightStart);
            cObjectPtr<IMediaCoder> pCoderOutputPullOutRightStart;
            m_pDescriptionOutputPullOutRightStart->WriteLock(pMediaSamplePullOutRightStart, &pCoderOutputPullOutRightStart);
            pCoderOutputPullOutRightStart->Set("bValue", (tVoid*)&(m_bOutputPullOutRightStart));
            pCoderOutputPullOutRightStart->Set("ui32ArduinoTimestamp", (tVoid*)&timestamp);
            m_pDescriptionOutputPullOutRightStart->Unlock(pCoderOutputPullOutRightStart);
            pMediaSamplePullOutRightStart->SetTime(_clock->GetStreamTime());
            m_oOutputPullOutRightStart.Transmit(pMediaSamplePullOutRightStart);
        } else {
           // TODO When we do get an invalid order in iPullOutDirectionID --> send an ERROR flag to the Jury model (propery for this)
            StopVehicle();
            LOG_ERROR(adtf_util::cString::Format("Pull out is triggered, but Pull out (left or right) is not the next task in the ManeuverList", iPullOutDirectionID));

        }
        m_bOutputPullOutLeftStart = false;
        m_bOutputPullOutRightStart = false;
    }
    RETURN_NOERROR;
}


tResult cSituationDetection::StartCrossing(tBool bIntersectionWithoutRoadSign, tInt8 i8RoadSignID){

    // if(m_bDebugModeEnabled) LOG_INFO(adtf_util::cString::Format("StartCrossing - entry function"));

    if(!m_bCarStateRunning)
    {
        StopVehicle();
      //  if(m_bDebugModeEnabled) LOG_INFO(adtf_util::cString::Format("StartCrossing - CarStateRunning = false"));
    }
    else if(m_bManeuverInProcess)
    {
     //   if(m_bDebugModeEnabled) LOG_INFO(adtf_util::cString::Format("StartCrossing - Maneuver in Process do nothing"));
        m_bRoadSignEvaluated = false;
        // Do nothing
    }
    else
    {
                switch(m_iStartCrossingCase) {
                   //
                   case 0:
                                              if(m_bRoadSignEvaluated)
                                              {
                                                  m_iStartCrossingCase = 1;
                                                  m_bRoadSignEvaluated = false;
                                                 //  if(m_bDebugModeEnabled) LOG_INFO(adtf_util::cString::Format("switch to case 1 and check triggers"));
                                              }

                                              // if(m_bDebugModeEnabled) LOG_INFO(adtf_util::cString::Format("StartCrossing - Case 0"));
                   break;

                   // Wait for some trigger given
                   case 1:
                                             // if(m_bDebugModeEnabled) LOG_INFO(adtf_util::cString::Format("StartCrossing - Case 1 check triggers"));

                                              // Check if a stopline, edge or edgeline is detected -> then trigger crossing
                                              tBool bTriggerCrossing;
                                              bTriggerCrossing = false;
                                              /*
                                              // Stopline as trigger
                                              if((m_fTriggerDistance2Stopline > m_fInputStoplineDistance) && m_bInputStoplineDetected && m_bEnableStopline4TriggeringCrossing)
                                              {
                                                  bTriggerCrossing = true;
                                                  if(m_bDebugModeEnabled) LOG_WARNING(adtf_util::cString::Format("Stopline Trigger = true"));
                                                  if(m_bDebugModeEnabled) LOG_WARNING(adtf_util::cString::Format("m_fInputStoplineDistance %f",m_fInputStoplineDistance));
                                              }
                                              // Edge1 as trigger
                                              else if(m_fTriggerDistance2Edge1 > m_fInputEdge1Distance && m_bInputEdge1Detected && m_bEnableEdge14TriggeringCrossing)
                                              {
                                                  bTriggerCrossing = true;
                                                  if(m_bDebugModeEnabled) LOG_WARNING(adtf_util::cString::Format("Edge1 Trigger = true"));
                                                  if(m_bDebugModeEnabled) LOG_WARNING(adtf_util::cString::Format("m_fInputEdge1Distance %f",m_fInputEdge1Distance));
                                              }
                                              */
                                              /*
                                              // Edge2 as trigger
                                              else if(m_fTriggerDistance2Edge2 > m_fInputEdge2Distance && m_bInputEdge2Detected && m_bEnableEdge24TriggeringCrossing){
                                                  bTriggerCrossing = true;
                                                  if(m_bDebugModeEnabled) LOG_WARNING(adtf_util::cString::Format("Edge2 Trigger = true"));
                                                  if(m_bDebugModeEnabled) LOG_WARNING(adtf_util::cString::Format("m_fInputEdge2Distance %f",m_fInputEdge2Distance));
                                              }
                                              */

                                              /*
                                              // Edgeline as trigger
                                              else if(m_fTriggerDistance2Edgeline > m_fInputEdgeLineDistance && m_bInputEdgeLineDetected && m_bEnableEdgeline4TriggeringCrossing)
                                              {
                                                  bTriggerCrossing = true;
                                                  if(m_bDebugModeEnabled) LOG_WARNING(adtf_util::cString::Format("Edgeline Trigger = true"));
                                                  if(m_bDebugModeEnabled) LOG_WARNING(adtf_util::cString::Format("m_fInputEdgeLineDistance %f",m_fInputEdgeLineDistance));
                                              }
                                              */

                                              // RoadSign as trigger
                                              /*else*/ if(m_fTriggerDistance2RoadSign > m_f32RoadSignDistanceY && m_bEnableRoadSign4TriggeringCrossing)
                                              {
                                                  bTriggerCrossing = true;
                                                  if(m_bDebugModeEnabled) LOG_WARNING(adtf_util::cString::Format("RoadSign Trigger = true"));
                                                  if(m_bDebugModeEnabled) LOG_WARNING(adtf_util::cString::Format("m_f32RoadSignDistanceY %f",m_f32RoadSignDistanceY));
                                              }

                                              // No trigger
                                              else
                                              {
                                                  bTriggerCrossing = false;
                                                //  if(m_bDebugModeEnabled) LOG_INFO(adtf_util::cString::Format("Crossing Trigger = false"));
                                                //  if(m_bDebugModeEnabled) LOG_INFO(adtf_util::cString::Format("m_fInputStoplineDistance %f",m_fInputStoplineDistance));
                                                //  if(m_bDebugModeEnabled) LOG_INFO(adtf_util::cString::Format("m_fInputEdge1Distance %f",m_fInputEdge1Distance));
                                                //  if(m_bDebugModeEnabled) LOG_INFO(adtf_util::cString::Format("m_fInputEdge2Distance %f",m_fInputEdge2Distance));
                                                //  if(m_bDebugModeEnabled) LOG_INFO(adtf_util::cString::Format("m_fInputEdgeLineDistance %f",m_fInputEdgeLineDistance));
                                                //  if(m_bDebugModeEnabled) LOG_INFO(adtf_util::cString::Format("m_f32RoadSignDistanceY %f",m_f32RoadSignDistanceY));
                                              }


                                              if(bTriggerCrossing)
                                              {
                                                    m_iStartCrossingCase = 2;
                                                   // if(m_bDebugModeEnabled) LOG_INFO(adtf_util::cString::Format("switch to case 2 and send the media sample"));
                                              }
                   break;

                   // Send MediaSamples
                   case 2:
                     //     LOG_INFO(adtf_util::cString::Format("StartCrossing - Case 2 Send Media Sample for Stop LaneFollower and start Crossing"));

                          tInt16 iCrossingManeuverID = -1;
                          // Order by ManeuverList
                          // if(m_bDebugModeEnabled) LOG_INFO(adtf_util::cString::Format("m_bTestCrossingEnabled %i",m_bTestCrossingEnabled));


                                // Check if maneuverList is empty
                                if(m_sectorList.size() != 0 && m_sectorList[m_iSectorIDCounter].maneuverList.size() != 0)
                                {
                                    // What maneuver is next in the ManeuverList
                                    if(m_sectorList[m_iSectorIDCounter].maneuverList[m_iManeuverIDSectorCounter].action.IsEqual("left"))
                                    {
                                      //  if(m_bDebugModeEnabled) LOG_INFO(adtf_util::cString::Format("Turn left"));
                                        iCrossingManeuverID = 0;
                                    }
                                    else if(m_sectorList[m_iSectorIDCounter].maneuverList[m_iManeuverIDSectorCounter].action.IsEqual("straight"))
                                    {
                                      //  if(m_bDebugModeEnabled) LOG_INFO(adtf_util::cString::Format("Straight"));
                                        iCrossingManeuverID = 1;
                                    }
                                    else if(m_sectorList[m_iSectorIDCounter].maneuverList[m_iManeuverIDSectorCounter].action.IsEqual("right"))
                                    {
                                      //  if(m_bDebugModeEnabled) LOG_INFO(adtf_util::cString::Format("Turn right"));
                                        iCrossingManeuverID = 2;
                                    }
                                    else
                                    {
                                       // if(m_bDebugModeEnabled) LOG_INFO(adtf_util::cString::Format("Crossing is not the next order in the maneuver list"));
                                        iCrossingManeuverID = -1;
                                    }
                                }
                                // ManeuverList is empty
                                else
                                {
                                     // Debug Info
                                     LOG_ERROR(adtf_util::cString::Format("Amount of sectors in this ManeuverList: %i",m_sectorList.size() ));
                                     if(m_sectorList.size() != 0)
                                     {
                                        LOG_ERROR(adtf_util::cString::Format("Amount of maneuvers in the secotor %i is:",m_iSectorIDCounter, m_sectorList[m_iSectorIDCounter].maneuverList.size() ));
                                     }
                                     SendDriverStruct(stateCar_ERROR, m_iManeuverIDOverallCounter);
                                 }

                          /*
                          // Testing - Mode
                          else
                          {
                                if (m_iTestCrossingManeuverID < -1 || 2 < m_iTestCrossingManeuverID)
                                {
                                // Error
                                LOG_ERROR(adtf_util::cString::Format("Error"));
                                LOG_ERROR(adtf_util::cString::Format("An invalid value for the Property m_iTestCrossingManeuverID was set"));
                                LOG_ERROR(adtf_util::cString::Format("Valid values are: -1 (Error), 0 (left), 1 (straight), 2 (right)"));
                                iCrossingManeuverID = -1;
                                }
                                else
                                {
                                    iCrossingManeuverID = m_iTestCrossingManeuverID;
                                }
                                LOG_INFO(adtf_util::cString::Format("iCrossingManeuverID %i, m_iTestCrossingManeuverID %i",iCrossingManeuverID, m_iTestCrossingManeuverID));
                          }
                          */
                           tInt16 iTrafficSignID;
                           iTrafficSignID = i8RoadSignID;
                           SendOutputs(false, true, iCrossingManeuverID, iTrafficSignID);
                           if (m_bEnableManeuverInProcessFlag) m_bManeuverInProcess = true;
                       //    if(m_bDebugModeEnabled) LOG_INFO(adtf_util::cString::Format("Disable LaneFollower, after receiving CrossingTrigger"));
                       //    if(m_bDebugModeEnabled) LOG_INFO(adtf_util::cString::Format("Start Crossing:"));

                                              /*
                                              tBool bOutputCrossingStart;
                                              bOutputCrossingStart= true;
                                              // Set this true so no other road sign can be processed until the current maneuver


                                              // Disable Lane Follower

                                                  tBool bLaneFollowerEnabled = false;
                                                  // Send start signal to LaneFollower-filter
                                                  // Create a new MediaSmaple
                                                  cObjectPtr<IMediaSample> pMediaSampleLaneFollowerStart;
                                                  AllocMediaSample((tVoid**)&pMediaSampleLaneFollowerStart);
                                                  // Send the Media Sample
                                                  cObjectPtr<IMediaSerializer> pSerializerLaneFollowerStart;
                                                  m_pDescriptionOutputLaneFollowerStart->GetMediaSampleSerializer(&pSerializerLaneFollowerStart);
                                                  tInt nSizeLaneFollowerStart = pSerializerLaneFollowerStart->GetDeserializedSize();
                                                  pMediaSampleLaneFollowerStart->AllocBuffer(nSizeLaneFollowerStart);
                                                  cObjectPtr<IMediaCoder> pCoderOutputLaneFollowerStart;
                                                  m_pDescriptionOutputLaneFollowerStart->WriteLock(pMediaSampleLaneFollowerStart, &pCoderOutputLaneFollowerStart);
                                                  pCoderOutputLaneFollowerStart->Set("bValue", (tVoid*)&(bLaneFollowerEnabled));
                                                  pCoderOutputLaneFollowerStart->Set("ui32ArduinoTimestamp", (tVoid*)&timestamp);
                                                  m_pDescriptionOutputLaneFollowerStart->Unlock(pCoderOutputLaneFollowerStart);
                                                  pMediaSampleLaneFollowerStart->SetTime(_clock->GetStreamTime());
                                                  m_oOutputLaneFollowerStart.Transmit(pMediaSampleLaneFollowerStart);


                                                 // Send crossing start true value

                                                  // Create a new MediaSmaple
                                                  cObjectPtr<IMediaSample> pMediaSampleCrossingStart;
                                                  AllocMediaSample((tVoid**)&pMediaSampleCrossingStart);

                                                  // Send the Media Sample
                                                  cObjectPtr<IMediaSerializer> pSerializerCrossingStart;
                                                  m_pDescriptionOutputCrossingStart->GetMediaSampleSerializer(&pSerializerCrossingStart);
                                                  tInt nSizeCrossingStart = pSerializerCrossingStart->GetDeserializedSize();
                                                  pMediaSampleCrossingStart->AllocBuffer(nSizeCrossingStart);
                                                  cObjectPtr<IMediaCoder> pCoderOutputCrossingStart;
                                                  m_pDescriptionOutputCrossingStart->WriteLock(pMediaSampleCrossingStart, &pCoderOutputCrossingStart);
                                                  pCoderOutputCrossingStart->Set("ui32ArduinoTimestamp", (tVoid*)&timestamp);
                                                  pCoderOutputCrossingStart->Set("bValue", (tVoid*)&(bOutputCrossingStart));
                                                  pCoderOutputCrossingStart->Set("i8CrossingManeuverID", (tVoid*)&(iCrossingManeuverID));
                                                  pCoderOutputCrossingStart->Set("i8RoadSignID", (tVoid*)&(iTrafficSignID));
                                                  pCoderOutputCrossingStart->Set("f32Imagesize", (tVoid*)&m_fTrafficSignImageSize);
                                                  m_pDescriptionOutputCrossingStart->Unlock(pCoderOutputCrossingStart);
                                                  pMediaSampleCrossingStart->SetTime(_clock->GetStreamTime());
                                                  m_oOutputCrossingStart.Transmit(pMediaSampleCrossingStart);


                                                  if(m_bDebugModeEnabled) LOG_INFO(adtf_util::cString::Format("Disable LaneFollower, after receiving CrossingTrigger: bLaneFollowerEnabled %i",bLaneFollowerEnabled));
                                                  if(m_bDebugModeEnabled) LOG_INFO(adtf_util::cString::Format("Start Crossing: bOutputCrossingStart %i",bOutputCrossingStart));

 */



                         // Reset state
                         m_iStartCrossingCase = 0;
                         m_bRoadSignEvaluated = false;
                   break;
                }
    }

    RETURN_NOERROR;
}


tResult cSituationDetection::StartZebraCrossing(){

    if(!m_bCarStateRunning)
    {
        StopVehicle();
    }
    else
    {
      //  if(m_bDebugModeEnabled) LOG_INFO(adtf_util::cString::Format("Send order to Zebracrossing"));

        if(m_bEnableManeuverInProcessFlag) m_bManeuverInProcess = true;
        tBool bZebraCrossingStart = true;
        // Create a new MediaSmaple
        cObjectPtr<IMediaSample> pMediaSampleZebracrossingStart;
        AllocMediaSample((tVoid**)&pMediaSampleZebracrossingStart);
        // Send the Media Sample
        cObjectPtr<IMediaSerializer> pSerializerZebracrossingStart;
        m_pDescriptionOutputZebracrossingStart->GetMediaSampleSerializer(&pSerializerZebracrossingStart);
        tInt nSizeZebracrossingStart = pSerializerZebracrossingStart->GetDeserializedSize();
        pMediaSampleZebracrossingStart->AllocBuffer(nSizeZebracrossingStart);
        cObjectPtr<IMediaCoder> pCoderOutputZebracrossingStart;
        m_pDescriptionOutputZebracrossingStart->WriteLock(pMediaSampleZebracrossingStart, &pCoderOutputZebracrossingStart);
        pCoderOutputZebracrossingStart->Set("bValue", (tVoid*)&(bZebraCrossingStart));
        // pCoderOutputZebracrossingStart->Set("ui32ArduinoTimestamp", (tVoid*)&timestamp);
        m_pDescriptionOutputZebracrossingStart->Unlock(pCoderOutputZebracrossingStart);
        pMediaSampleZebracrossingStart->SetTime(_clock->GetStreamTime());
        m_oOutputZebracrossingStart.Transmit(pMediaSampleZebracrossingStart);
    }
    RETURN_NOERROR;
}


tResult cSituationDetection::SendRoI2StopLineDetection(){

    tFloat32 i16RoI;
    i16RoI = -1;
    // Do not acces it when there is No maneuverList available
    if(!m_bTestCrossingEnabled && !m_bTestParkingEnabled && !m_bTestPullOutEnabled){

            // Set the RoI depending on the next maneuver
            cString strNextManeuver = m_sectorList[m_iSectorIDCounter].maneuverList[m_iManeuverIDSectorCounter].action;
            if(strNextManeuver == "left"){
                 i16RoI = 1;
            } else if(strNextManeuver == "straight"){
                i16RoI = 2;
            } else if(strNextManeuver == "right"){
                i16RoI = 0;
            } else if(strNextManeuver.Find("cross_parking") != -1){
               i16RoI = 3;
            }

    } else {
        if(m_bTestCrossingEnabled){
            //Set the test value from the Crossing Test Property
            i16RoI = m_iTestCrossingManeuverID;
        } else if(m_bTestParkingEnabled){
            // RoI is always Parking
            i16RoI = 3;
        }
    }

    // LOG_INFO(adtf_util::cString::Format("Situation Detection: RoI sent: %f",i16RoI));

    // Create a new MediaSmaple
    cObjectPtr<IMediaSample> pMediaSampleSendRoI;
    AllocMediaSample((tVoid**)&pMediaSampleSendRoI);

    // Send the Media Sample
    cObjectPtr<IMediaSerializer> pSerializerSendRoI;
    m_pDescriptionOutputRoI->GetMediaSampleSerializer(&pSerializerSendRoI);
    tInt nSizeSendRoI = pSerializerSendRoI->GetDeserializedSize();
    pMediaSampleSendRoI->AllocBuffer(nSizeSendRoI);
    cObjectPtr<IMediaCoder> pCoderOutputSendRoI;
    tTimeStamp timestamp;
    timestamp = _clock->GetStreamTime();
    m_pDescriptionOutputRoI->WriteLock(pMediaSampleSendRoI, &pCoderOutputSendRoI);
    pCoderOutputSendRoI->Set("ui32ArduinoTimestamp", (tVoid*)&timestamp);
    pCoderOutputSendRoI->Set("f32Value", (tVoid*)&i16RoI);
    m_pDescriptionOutputRoI->Unlock(pCoderOutputSendRoI);
    pMediaSampleSendRoI->SetTime(_clock->GetStreamTime());
    m_oOutputRoI.Transmit(pMediaSampleSendRoI);
    RETURN_NOERROR;
}


tResult cSituationDetection::LoadManeuverList(){

    m_sectorList.clear();
    // create dom from string received from pin
    cDOM oDOM;
    oDOM.FromString(m_strManeuverFileString);
    cDOMElementRefList oSectorElems;
    cDOMElementRefList oManeuverElems;

    //read first Sector Elem
    if(IS_OK(oDOM.FindNodes("AADC-Maneuver-List/AADC-Sector", oSectorElems)))
    {
        //iterate through sectors
        for (cDOMElementRefList::iterator itSectorElem = oSectorElems.begin(); itSectorElem != oSectorElems.end(); ++itSectorElem)
        {
            //if sector found
            tSector sector;
            sector.id = (*itSectorElem)->GetAttributeUInt32("id");

            if(IS_OK((*itSectorElem)->FindNodes("AADC-Maneuver", oManeuverElems)))
            {
                //iterate through maneuvers
                for(cDOMElementRefList::iterator itManeuverElem = oManeuverElems.begin(); itManeuverElem != oManeuverElems.end(); ++itManeuverElem)
                {
                    tAADC_Maneuver man;
                    man.id = (*itManeuverElem)->GetAttributeUInt32("id");
                    man.action = (*itManeuverElem)->GetAttribute("action");
                    sector.maneuverList.push_back(man);
                }
            }

           m_sectorList.push_back(sector);
        }
    }
    if (oSectorElems.size() > 0)
    {
     //   LOG_INFO("DriverFilter: Loaded Maneuver file successfully.");
        // m_pWidget->EnableManeuverGroupBox(true);
    }
    else
    {
        LOG_ERROR("DriverFilter: no valid Maneuver Data found!");
        // m_pWidget->EnableManeuverGroupBox(false);
        RETURN_ERROR(ERR_INVALID_FILE);
    }

    // update the ui
    // m_pWidget->SetManeuverList(m_sectorList);
    // m_pWidget->ShowManeuverList();
    // m_pWidget->FillComboBox();


    // Read the data from the m_sectorList
    {
       // LOG_INFO(cString::Format("Display complete Maneuver List"));
        tInt32 ManeuverID = 0;
        cString ManeuverAction = "";



        m_iAmountOfSectors = m_sectorList.size();
        // Check that the maneuverList contains some sectors
        if(m_iAmountOfSectors==0){
            LOG_ERROR(cString::Format("Situation Detection: Amount of sectors from the ManeuverList: %i", m_iAmountOfSectors));
            SendDriverStruct(stateCar_ERROR,0);
        }else{
           // if(m_bDebugModeEnabled) LOG_INFO(cString::Format("Situation Detection: Amount of sectors from the ManeuverList: %i", m_iAmountOfSectors));
        }

        for(tInt i = 0; i<m_iAmountOfSectors; i ++){
            tInt32 sectorID = m_sectorList[i].id;
           // if(m_bDebugModeEnabled) LOG_INFO(cString::Format("Situation Detection: ID of current sector: %i", sectorID));
            tInt32 ManeuverListSize = m_sectorList[i].maneuverList.size();
            // Check if the current sector contains maneuvers
            if(m_iAmountOfSectors==0){
                LOG_ERROR(cString::Format("Situation Detection: Amount of maneuvers in sector %i is: %i",sectorID, ManeuverListSize));
                SendDriverStruct(stateCar_ERROR,0);
            }else{
             //   if(m_bDebugModeEnabled) LOG_INFO(cString::Format("Situation Detection: Amount of maneuvers in sector %i is: %i",i, ManeuverListSize));
            }

            // if(m_bDebugModeEnabled) LOG_INFO(cString::Format("Situation Detection: Size of the ManeuverList from the first sector: %i", ManeuverListSize));

            m_iArrayNumberOfManeuversInSerctor[i] = ManeuverListSize;

            ManeuverID =0;
            for (tInt j =0; j<ManeuverListSize;j++){
               ManeuverID = m_sectorList[i].maneuverList[j].id;
               ManeuverAction = m_sectorList[i].maneuverList[j].action;
            //   if(m_bDebugModeEnabled) LOG_INFO(cString::Format("Situation Detection: Maneuver ID: %i with action:  ", ManeuverID));
             //  if(m_bDebugModeEnabled) LOG_INFO(ManeuverAction);

               // TODO maybe check for invalid maneuver-actions or IDs?
            }

        }
        for(tInt i = 0; i<m_iAmountOfSectors; i ++){
//            if(m_bDebugModeEnabled) LOG_INFO(cString::Format("Situation Detection: Array of the size of all ManouverList from sector %d  ", m_iArrayNumberOfManeuversInSerctor[i]));
        }
    }
    if(m_bDebugModeEnabled) LOG_INFO(cString::Format("Situation Detection: ManeuverList loaded successfully"));

    RETURN_NOERROR;
}


tResult cSituationDetection::SendDriverStruct(stateCar stateID, tInt16 i16ManeuverEntry){
    /*
     *  parameter: stateCar stateID
     *
     *  enum stateCar{
     *      stateCar_ERROR    = -1,
     *      stateCar_READY    =  0,
     *      stateCar_RUNNING  =  1,
     *      stateCar_COMPLETE =  2,
     *      stateCar_STARTUP  = -2
     *  };
    */
    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));

    cObjectPtr<IMediaSerializer> pSerializer;
    m_pDescriptionOutputDriverStruct->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    tInt8 value = tInt8(stateID);

    RETURN_IF_FAILED(pMediaSample->AllocBuffer(nSize));
    {
        // focus for sample write lock
        __adtf_sample_write_lock_mediadescription(m_pDescriptionOutputDriverStruct,pMediaSample,pCoder);
        // get the IDs for the items in the media sample
        if(!m_bIDsDriverStructSet)
        {
            pCoder->GetID("i8StateID", m_szIDDriverStructI8StateID);
            pCoder->GetID("i16ManeuverEntry", m_szIDDriverStructI16ManeuverEntry);
            m_bIDsDriverStructSet = true;
        }
        // Write the state and current maneuverID in the struct
        pCoder->Set(m_szIDDriverStructI8StateID, (tVoid*)&value);
        pCoder->Set(m_szIDDriverStructI16ManeuverEntry, (tVoid*)&i16ManeuverEntry);
    }
    pMediaSample->SetTime(_clock->GetStreamTime());
    m_oOutputDriverStruct.Transmit(pMediaSample);

    if(m_bDebugModeEnabled)
    {
        switch (stateID)
        {
        case stateCar_READY:
     //       LOG_INFO(cString::Format("Situation Detection SendDriverStruct:     Send state: READY,      Maneuver ID %d",i16ManeuverEntry));
            break;
        case stateCar_RUNNING:
     //       LOG_INFO(cString::Format("Situation Detection SendDriverStruct:     Send state: RUNNING,    Maneuver ID %d",i16ManeuverEntry));
            break;
        case stateCar_COMPLETE:
     //       LOG_INFO(cString::Format("Situation Detection SendDriverStruct:     Send state: COMPLETE,   Maneuver ID %d",i16ManeuverEntry));
            break;
        case stateCar_ERROR:
     //       LOG_INFO(cString::Format("Situation Detection SendDriverStruct:     Send state: ERROR,      Maneuver ID %d",i16ManeuverEntry));
            break;
        case stateCar_STARTUP:
     //       LOG_INFO(cString::Format("Situation Detection SendDriverStruct:     Send state: STARTUP,    Maneuver ID %d",i16ManeuverEntry));
            break;
        }
    }
    RETURN_NOERROR;

}


tResult cSituationDetection::FinishManeuver(cString strFinishedManeuver){
    cString strCrossing;
    cString strNextManeuver = m_sectorList[m_iSectorIDCounter].maneuverList[m_iManeuverIDSectorCounter].action;

    // Check if test mode is enabled
    if(!m_bTestCrossingEnabled && !m_bTestParkingEnabled && !m_bTestPullOutEnabled){
        if(strNextManeuver == "left" ||
           strNextManeuver == "straight" ||
           strNextManeuver == "right")
        {
            strCrossing = "crossing";
            //LOG_INFO(m_sectorList[m_iSectorIDCounter].maneuverList[m_iManeuverIDSectorCounter].action);
            //LOG_INFO(cString::Format("Situation Detection: Crossing detected."));
        }
        else
        {
            strCrossing = "NOTcrossing";
            //LOG_INFO(m_sectorList[m_iSectorIDCounter].maneuverList[m_iManeuverIDSectorCounter].action);
            //LOG_INFO(cString::Format("Situation Detection: Crossing NOT detected."));
        }

        // Check if this function was triggered by the correct ManeuverFinished-function
        if((strNextManeuver.Find(strFinishedManeuver) != -1) || (strFinishedManeuver == strCrossing))
        {

           // if(m_bDebugModeEnabled) LOG_INFO(cString::Format("Situation Detection: Previous maneuver was correct, FinishManeuver was triggered by the correct function"));
          //  if(m_bDebugModeEnabled) LOG_INFO(strFinishedManeuver);

            // When an maneuver is finished it is no longer in process
            m_bOutputPullOutLeftStart = false;

            if(m_bDebugModeEnabled)
            {
             //   LOG_INFO(cString::Format("Situation Detection: Previous maneuver = "));
             //   LOG_INFO(strNextManeuver);
            }

            tBool bNotLaneFollower = false;
            if(strFinishedManeuver == "cross_parking")
            {
	    		bNotLaneFollower = true;
                   //     if(m_bDebugModeEnabled) LOG_INFO(cString::Format("bNotLaneFollower = true"));
            }

            // Increase Counter
            m_iManeuverIDOverallCounter++;
            m_iManeuverIDSectorCounter++;

            if(m_iManeuverIDSectorCounter < m_iArrayNumberOfManeuversInSerctor[m_iSectorIDCounter])
			{
                // There are still some maneuvers left in this sector -> continue
            } 
			else 
			{
                // All maneuvers of this sector are completed
                // Get into the next sector and reset the m_iManeuverIDSectorCounter
                m_iManeuverIDSectorCounter = 0;
                m_iSectorIDCounter++;
            }


            if(m_iSectorIDCounter < m_iAmountOfSectors)
            {
                // Print the maneuver information out to the console
                if(m_bDebugModeEnabled)
				{
                //    LOG_INFO(cString::Format("Situation Detection: m_iAmountOfSectors = %i", m_iAmountOfSectors));
               //     LOG_INFO(cString::Format("Situation Detection: m_iManeuverIDOverallCounter = %i", m_iManeuverIDOverallCounter));
               //     LOG_INFO(cString::Format("Situation Detection: m_iSectorIDCounter = %i", m_iSectorIDCounter));
                 //   LOG_INFO(cString::Format("Situation Detection: m_iManeuverIDSectorCounter = %i", m_iManeuverIDSectorCounter));
                 //   LOG_INFO(cString::Format("Situation Detection: m_iManeuverIDSectorCounter = %i", m_sectorList[m_iSectorIDCounter].maneuverList[m_iManeuverIDSectorCounter].id));
                 //   LOG_INFO(cString::Format("Situation Detection: Next maneuver = "));
                 //   LOG_INFO(m_sectorList[m_iSectorIDCounter].maneuverList[m_iManeuverIDSectorCounter].action);
                }
                // There are still some sectors left in the ManeuverList -> continue
                // Send the running state and the current OverallManeuverID
                SendDriverStruct(stateCar_RUNNING,m_iManeuverIDOverallCounter);
                SendRoI2StopLineDetection();
    		if(bNotLaneFollower)
                {
                    StartPullOut();
                }
            } 
            else
            {
                // The last sector is finished and the ManouverList is completed
                // Send the completed state
                SendDriverStruct(stateCar_COMPLETE, 0);
                m_bCarStateRunning = false;
                // Increase Counter
                m_iManeuverIDOverallCounter--;
                m_iManeuverIDSectorCounter--;
                StopVehicle();
            }
        } 
        else
        {
           // if(m_bDebugModeEnabled) LOG_INFO(cString::Format("Situation Detection: Incorrect triggering of the maneuver FinishManeuver by:"));
           //  if(m_bDebugModeEnabled) LOG_INFO(strFinishedManeuver);
            //if(m_bDebugModeEnabled) LOG_INFO(strCrossing);
        }

        m_bManeuverInProcess = false;
    }
    else
    {
       SendRoI2StopLineDetection();
       m_bManeuverInProcess = false;
       m_bCarStateRunning = true;
    }
    RETURN_NOERROR;
}


tResult cSituationDetection::SendOutputs(tBool bLaneFollowerStart, tBool bCrossingStart, tInt iCrossingManeuverID, tInt iTrafficSignID)
{

    // Send bool with false to LaneFollower
    cObjectPtr<IMediaSample> pMediaSampleLaneFollowerStart;
    AllocMediaSample((tVoid**)&pMediaSampleLaneFollowerStart);
    // Send the Media Sample
    cObjectPtr<IMediaSerializer> pSerializerLaneFollowerStart;
    m_pDescriptionOutputLaneFollowerStart->GetMediaSampleSerializer(&pSerializerLaneFollowerStart);
    tInt nSizeLaneFollowerStart = pSerializerLaneFollowerStart->GetDeserializedSize();
    pMediaSampleLaneFollowerStart->AllocBuffer(nSizeLaneFollowerStart);
    cObjectPtr<IMediaCoder> pCoderOutputLaneFollowerStart;
    m_pDescriptionOutputLaneFollowerStart->WriteLock(pMediaSampleLaneFollowerStart, &pCoderOutputLaneFollowerStart);
    pCoderOutputLaneFollowerStart->Set("bValue", (tVoid*)&(bLaneFollowerStart));
    pCoderOutputLaneFollowerStart->Set("ui32ArduinoTimestamp", (tVoid*)&timestamp);
    m_pDescriptionOutputLaneFollowerStart->Unlock(pCoderOutputLaneFollowerStart);
    pMediaSampleLaneFollowerStart->SetTime(_clock->GetStreamTime());
    m_oOutputLaneFollowerStart.Transmit(pMediaSampleLaneFollowerStart);

    // Send bool with false to Crossing
    cObjectPtr<IMediaSample> pMediaSampleCrossingStart;
    AllocMediaSample((tVoid**)&pMediaSampleCrossingStart);
    // Create a new MediaSmaple
    cObjectPtr<IMediaSerializer> pSerializerCrossingStart;
    m_pDescriptionOutputCrossingStart->GetMediaSampleSerializer(&pSerializerCrossingStart);
    tInt nSizeCrossingStart = pSerializerCrossingStart->GetDeserializedSize();
    pMediaSampleCrossingStart->AllocBuffer(nSizeCrossingStart);
    cObjectPtr<IMediaCoder> pCoderOutputCrossingStart;
    m_pDescriptionOutputCrossingStart->WriteLock(pMediaSampleCrossingStart, &pCoderOutputCrossingStart);
    pCoderOutputCrossingStart->Set("ui32ArduinoTimestamp", (tVoid*)&timestamp);
    pCoderOutputCrossingStart->Set("bValue", (tVoid*)&(bCrossingStart));
    pCoderOutputCrossingStart->Set("i8CrossingManeuverID", (tVoid*)&(iCrossingManeuverID));
    pCoderOutputCrossingStart->Set("i8RoadSignID", (tVoid*)&(iTrafficSignID));
    pCoderOutputCrossingStart->Set("f32Imagesize", (tVoid*)&m_fTrafficSignImageSize);
    m_pDescriptionOutputCrossingStart->Unlock(pCoderOutputCrossingStart);
    pMediaSampleCrossingStart->SetTime(_clock->GetStreamTime());
    m_oOutputCrossingStart.Transmit(pMediaSampleCrossingStart);


    RETURN_NOERROR;
}


tResult cSituationDetection::StopVehicle(){
    // Send all functions bool values with false
    if(m_bDebugModeEnabled) LOG_INFO(cString::Format("Stop-order received, send false-values to all the functions"));

    m_bManeuverInProcess =false;

    // Send bool with false to LaneFollower
    tBool bLaneFollowerEnabled = false;
    cObjectPtr<IMediaSample> pMediaSampleLaneFollowerStart;
    AllocMediaSample((tVoid**)&pMediaSampleLaneFollowerStart);
    // Send the Media Sample
    cObjectPtr<IMediaSerializer> pSerializerLaneFollowerStart;
    m_pDescriptionOutputLaneFollowerStart->GetMediaSampleSerializer(&pSerializerLaneFollowerStart);
    tInt nSizeLaneFollowerStart = pSerializerLaneFollowerStart->GetDeserializedSize();
    pMediaSampleLaneFollowerStart->AllocBuffer(nSizeLaneFollowerStart);
    cObjectPtr<IMediaCoder> pCoderOutputLaneFollowerStart;
    m_pDescriptionOutputLaneFollowerStart->WriteLock(pMediaSampleLaneFollowerStart, &pCoderOutputLaneFollowerStart);
    pCoderOutputLaneFollowerStart->Set("bValue", (tVoid*)&(bLaneFollowerEnabled));
    pCoderOutputLaneFollowerStart->Set("ui32ArduinoTimestamp", (tVoid*)&timestamp);
    m_pDescriptionOutputLaneFollowerStart->Unlock(pCoderOutputLaneFollowerStart);
    pMediaSampleLaneFollowerStart->SetTime(_clock->GetStreamTime());
    m_oOutputLaneFollowerStart.Transmit(pMediaSampleLaneFollowerStart);


    // Send bool with false to Crossing
    cObjectPtr<IMediaSample> pMediaSampleCrossingStart;
    AllocMediaSample((tVoid**)&pMediaSampleCrossingStart);
    tBool bOutputCrossingStart;
    bOutputCrossingStart = false;
    m_iCurrentTrafficSignID = -1;
    tInt16 iCrossingManeuverID = -1;
    m_fTrafficSignImageSize = -1;
    // Create a new MediaSmaple
    cObjectPtr<IMediaSerializer> pSerializerCrossingStart;
    m_pDescriptionOutputCrossingStart->GetMediaSampleSerializer(&pSerializerCrossingStart);
    tInt nSizeCrossingStart = pSerializerCrossingStart->GetDeserializedSize();
    pMediaSampleCrossingStart->AllocBuffer(nSizeCrossingStart);
    cObjectPtr<IMediaCoder> pCoderOutputCrossingStart;
    m_pDescriptionOutputCrossingStart->WriteLock(pMediaSampleCrossingStart, &pCoderOutputCrossingStart);
    pCoderOutputCrossingStart->Set("ui32ArduinoTimestamp", (tVoid*)&timestamp);
    pCoderOutputCrossingStart->Set("bValue", (tVoid*)&(bOutputCrossingStart));
    pCoderOutputCrossingStart->Set("i8CrossingManeuverID", (tVoid*)&(iCrossingManeuverID));
    pCoderOutputCrossingStart->Set("i8RoadSignID", (tVoid*)&(m_iCurrentTrafficSignID));
    pCoderOutputCrossingStart->Set("f32Imagesize", (tVoid*)&m_fTrafficSignImageSize);
    m_pDescriptionOutputCrossingStart->Unlock(pCoderOutputCrossingStart);
    pMediaSampleCrossingStart->SetTime(_clock->GetStreamTime());
    m_oOutputCrossingStart.Transmit(pMediaSampleCrossingStart);


    // Send bool with false to LaneChange
    tBool bStartLaneChange = false;
    // Create a new MediaSmaple
    cObjectPtr<IMediaSample> pMediaSampleLaneChangeStart;
    AllocMediaSample((tVoid**)&pMediaSampleLaneChangeStart);
    // Send the Media Sample
    cObjectPtr<IMediaSerializer> pSerializerLaneChangeStart;
    m_pDescriptionOutputLaneChangeStart->GetMediaSampleSerializer(&pSerializerLaneChangeStart);
    tInt nSizeLaneChangeStart = pSerializerLaneChangeStart->GetDeserializedSize();
    pMediaSampleLaneChangeStart->AllocBuffer(nSizeLaneChangeStart);
    cObjectPtr<IMediaCoder> pCoderOutputLaneChangeStart;
    m_pDescriptionOutputLaneChangeStart->WriteLock(pMediaSampleLaneChangeStart, &pCoderOutputLaneChangeStart);
    pCoderOutputLaneChangeStart->Set("bValue", (tVoid*)&(bStartLaneChange));
    pCoderOutputLaneChangeStart->Set("ui32ArduinoTimestamp", (tVoid*)&timestamp);
    m_pDescriptionOutputLaneChangeStart->Unlock(pCoderOutputLaneChangeStart);
    pMediaSampleLaneChangeStart->SetTime(_clock->GetStreamTime());
    m_oOutputLaneChangeStart.Transmit(pMediaSampleLaneChangeStart);


    // Send start signal to scanning-filter
    tBool bStartSignal = false;
    // Create a new MediaSmaple
    cObjectPtr<IMediaSample> pMediaSampleScanningStart;
    AllocMediaSample((tVoid**)&pMediaSampleScanningStart);
    // Send the Media Sample
    cObjectPtr<IMediaSerializer> pSerializerScanningStart;
    m_pDescriptionOutputScanningStart->GetMediaSampleSerializer(&pSerializerScanningStart);
    tInt nSizeScanningStart = pSerializerScanningStart->GetDeserializedSize();
    pMediaSampleScanningStart->AllocBuffer(nSizeScanningStart);
    cObjectPtr<IMediaCoder> pCoderOutputScanningStart;
    m_pDescriptionOutputScanningStart->WriteLock(pMediaSampleScanningStart, &pCoderOutputScanningStart);
    pCoderOutputScanningStart->Set("bValue", (tVoid*)&(bStartSignal));
    m_pDescriptionOutputScanningStart->Unlock(pCoderOutputScanningStart);
    pMediaSampleScanningStart->SetTime(_clock->GetStreamTime());
    m_oOutputScanningStart.Transmit(pMediaSampleScanningStart);


    // Send bool with false to Parking
    m_bOutputParkingStart = false;
    // Create a new MediaSmaple
    cObjectPtr<IMediaSample> pMediaSampleParkingStart;
    AllocMediaSample((tVoid**)&pMediaSampleParkingStart);
    // Send the Media Sample
    cObjectPtr<IMediaSerializer> pSerializerParkingStart;
    m_pDescriptionOutputParkingStart->GetMediaSampleSerializer(&pSerializerParkingStart);
    tInt nSizeParkingStart = pSerializerParkingStart->GetDeserializedSize();
    pMediaSampleParkingStart->AllocBuffer(nSizeParkingStart);
    cObjectPtr<IMediaCoder> pCoderOutputParkingStart;
    m_pDescriptionOutputParkingStart->WriteLock(pMediaSampleParkingStart, &pCoderOutputParkingStart);
    pCoderOutputParkingStart->Set("bValue", (tVoid*)&(m_bOutputParkingStart));
    pCoderOutputParkingStart->Set("ui32ArduinoTimestamp", (tVoid*)&timestamp);
    m_pDescriptionOutputParkingStart->Unlock(pCoderOutputParkingStart);
    pMediaSampleParkingStart->SetTime(_clock->GetStreamTime());
    m_oOutputParkingStart.Transmit(pMediaSampleParkingStart);


    // Send bool with false to Pull_out_left
    m_bOutputPullOutLeftStart = false;
    // Create a new MediaSmaple
    cObjectPtr<IMediaSample> pMediaSamplePullOutLeftStart;
    AllocMediaSample((tVoid**)&pMediaSamplePullOutLeftStart);
    // Send the Media Sample
    cObjectPtr<IMediaSerializer> pSerializerPullOutLeftStart;
    m_pDescriptionOutputPullOutLeftStart->GetMediaSampleSerializer(&pSerializerPullOutLeftStart);
    tInt nSizePullOutLeftStart = pSerializerPullOutLeftStart->GetDeserializedSize();
    pMediaSamplePullOutLeftStart->AllocBuffer(nSizePullOutLeftStart);
    cObjectPtr<IMediaCoder> pCoderOutputPullOutLeftStart;
    m_pDescriptionOutputPullOutLeftStart->WriteLock(pMediaSamplePullOutLeftStart, &pCoderOutputPullOutLeftStart);
    pCoderOutputPullOutLeftStart->Set("bValue", (tVoid*)&(m_bOutputPullOutLeftStart));
    pCoderOutputPullOutLeftStart->Set("ui32ArduinoTimestamp", (tVoid*)&timestamp);
    m_pDescriptionOutputPullOutLeftStart->Unlock(pCoderOutputPullOutLeftStart);
    pMediaSamplePullOutLeftStart->SetTime(_clock->GetStreamTime());
    m_oOutputPullOutLeftStart.Transmit(pMediaSamplePullOutLeftStart);


    // Send bool with false to Pull_out_right
    m_bOutputPullOutRightStart = false;
    // Create a new MediaSmaple
    cObjectPtr<IMediaSample> pMediaSamplePullOutRightStart;
    AllocMediaSample((tVoid**)&pMediaSamplePullOutRightStart);
    // Send the Media Sample
    cObjectPtr<IMediaSerializer> pSerializerPullOutRightStart;
    m_pDescriptionOutputPullOutRightStart->GetMediaSampleSerializer(&pSerializerPullOutRightStart);
    tInt nSizePullOutRightStart = pSerializerPullOutRightStart->GetDeserializedSize();
    pMediaSamplePullOutRightStart->AllocBuffer(nSizePullOutRightStart);
    cObjectPtr<IMediaCoder> pCoderOutputPullOutRightStart;
    m_pDescriptionOutputPullOutRightStart->WriteLock(pMediaSamplePullOutRightStart, &pCoderOutputPullOutRightStart);
    pCoderOutputPullOutRightStart->Set("bValue", (tVoid*)&(m_bOutputPullOutRightStart));
    pCoderOutputPullOutRightStart->Set("ui32ArduinoTimestamp", (tVoid*)&timestamp);
    m_pDescriptionOutputPullOutRightStart->Unlock(pCoderOutputPullOutRightStart);
    pMediaSamplePullOutRightStart->SetTime(_clock->GetStreamTime());
    m_oOutputPullOutRightStart.Transmit(pMediaSamplePullOutRightStart);


    // Send bool with false to Zebracrossing
    tBool bZebraCrossingStart = false;
    // Create a new MediaSmaple
    cObjectPtr<IMediaSample> pMediaSampleZebracrossingStart;
    AllocMediaSample((tVoid**)&pMediaSampleZebracrossingStart);
    // Send the Media Sample
    cObjectPtr<IMediaSerializer> pSerializerZebracrossingStart;
    m_pDescriptionOutputZebracrossingStart->GetMediaSampleSerializer(&pSerializerZebracrossingStart);
    tInt nSizeZebracrossingStart = pSerializerZebracrossingStart->GetDeserializedSize();
    pMediaSampleZebracrossingStart->AllocBuffer(nSizeZebracrossingStart);
    cObjectPtr<IMediaCoder> pCoderOutputZebracrossingStart;
    m_pDescriptionOutputZebracrossingStart->WriteLock(pMediaSampleZebracrossingStart, &pCoderOutputZebracrossingStart);
    pCoderOutputZebracrossingStart->Set("bValue", (tVoid*)&(bZebraCrossingStart));
    // pCoderOutputZebracrossingStart->Set("ui32ArduinoTimestamp", (tVoid*)&timestamp);
    m_pDescriptionOutputZebracrossingStart->Unlock(pCoderOutputZebracrossingStart);
    pMediaSampleZebracrossingStart->SetTime(_clock->GetStreamTime());
    m_oOutputZebracrossingStart.Transmit(pMediaSampleZebracrossingStart);

    RETURN_NOERROR;
}


tResult cSituationDetection::PropertyChanged(const char* strProperty){

    if(NULL== strProperty || cString::IsEqual(strProperty,"Test Crossing enabled" ))
    {
        m_bTestCrossingEnabled = static_cast<tBool> (GetPropertyBool("Test Crossing enabled"));
    }
    if(NULL== strProperty || cString::IsEqual(strProperty,"Test Pull Out enabled" ))
    {
        m_bTestPullOutEnabled = static_cast<tBool> (GetPropertyBool("Test Pull Out enabled"));
    }
    if(NULL== strProperty || cString::IsEqual(strProperty,"Test Parking enabled" ))
    {
        m_bTestParkingEnabled = static_cast<tBool> (GetPropertyBool("Test Parking enabled"));
    }
    if(NULL== strProperty || cString::IsEqual(strProperty,"Trigger Distance to Edge 1" ))
    {
        m_fTriggerDistance2Edge1 = static_cast<tBool> (GetPropertyBool("Trigger Distance to Edge 1"));
    }
    /*
    if(NULL== strProperty || cString::IsEqual(strProperty,"Trigger Distance to Edge 2" ))
    {
        m_fTriggerDistance2Edge2 = static_cast<tBool> (GetPropertyBool("Trigger Distance to Edge 2"));
    }
    */
    if(NULL== strProperty || cString::IsEqual(strProperty,"Trigger Distance to Stopline" ))
    {
        m_fTriggerDistance2Stopline = static_cast<tBool> (GetPropertyBool("Trigger Distance to Stopline"));
    }
    if(NULL== strProperty || cString::IsEqual(strProperty,"Trigger Distance to Edgeline" ))
    {
        m_fTriggerDistance2Edgeline = static_cast<tBool> (GetPropertyBool("Trigger Distance to Edgeline"));
    }
	if(NULL== strProperty || cString::IsEqual(strProperty,"Trigger Distance to RoadSign" ))
    {
        m_fTriggerDistance2RoadSign = static_cast<tBool> (GetPropertyBool("Trigger Distance to RoadSign"));
    }
    if(NULL== strProperty || cString::IsEqual(strProperty,"Enable Stopline for triggering Crossing" ))
    {
        m_bEnableStopline4TriggeringCrossing = static_cast<tBool> (GetPropertyBool("Enable Stopline for triggering Crossing"));
    }
    if(NULL== strProperty || cString::IsEqual(strProperty,"Enable Edge1 for triggering Crossing" ))
    {
        m_bEnableEdge14TriggeringCrossing = static_cast<tBool> (GetPropertyBool("Enable Edge1 for triggering Crossing"));
    }
	if(NULL== strProperty || cString::IsEqual(strProperty,"Enable RoadSign for triggering Crossing" ))
    {
        m_bEnableRoadSign4TriggeringCrossing = static_cast<tBool> (GetPropertyBool("Enable RoadSign for triggering Crossing"));
    }


    /*
    if(NULL== strProperty || cString::IsEqual(strProperty,"Enable Edge2 for triggering Crossing" ))
    {
        m_bEnableEdge24TriggeringCrossing = static_cast<tBool> (GetPropertyBool("Enable Edge2 for triggering Crossing"));
    }
    */
    if(NULL== strProperty || cString::IsEqual(strProperty,"Enable Edgeline for triggering Crossing" ))
    {
        m_bEnableEdgeline4TriggeringCrossing = static_cast<tBool> (GetPropertyBool("Enable Edgeline for triggering Crossing"));
    }


    if(NULL== strProperty || cString::IsEqual(strProperty,"Debug Output to Console" ))
    {
        m_bDebugModeEnabled = static_cast<tBool> (GetPropertyBool("Debug Output to Console"));
    }
    if(NULL== strProperty || cString::IsEqual(strProperty,"Road Sign Counter max" ))
    {
        m_iRoadSignCounterMax = static_cast<tBool> (GetPropertyBool("Road Sign Counter max"));
    }
    if(NULL== strProperty || cString::IsEqual(strProperty,"Test Crossing Maneuver ID" ))
    {
        m_iTestCrossingManeuverID = static_cast<tBool> (GetPropertyBool("Test Crossing Maneuver ID"));
    }
    if(NULL== strProperty || cString::IsEqual(strProperty,"Test Pull Out Direction ID" ))
    {
        m_iTestPullOutDirectionID = static_cast<tBool> (GetPropertyBool("Test Pull Out Direction ID"));
    }

    RETURN_NOERROR;
}


