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
#include "cPedestrianDetection.h"  
#include "aadc_classification_structs.h"
//#include "template_data.h"


#define NORMAL              1
#define ZEBRA               2
#define CHILD               3

#define DRIVING_2_ZEBRA     4
#define WAITING_AT_ZEBRA    5
#define PASS_ZEBRA          6

#define EV_NORMAL           7
#define EV_ZEBRA            8
#define EV_WAITING          9

/// Create filter shell 
ADTF_FILTER_PLUGIN("PedestrianDetection", OID_ADTF_PEDESTRIAN_DETECTION, cPedestrianDetection);


cPedestrianDetection::cPedestrianDetection(const tChar* __info):cFilter(__info)
{
    SetPropertyFloat("Zebracrossing::Velocity [m/s]", 0.35);
    SetPropertyFloat("Zebracrossing::Distance for waiting [cm]", 70);
    SetPropertyFloat("Zebracrossing::Max. time to wait before Zebracrossing [s]", 7);

    SetPropertyBool("Test Pedestrian Detected",true);

    SetPropertyBool("Debug output to console enabled",true);

}

cPedestrianDetection::~cPedestrianDetection()
{

}

tResult cPedestrianDetection::Init(tInitStage eStage, __exception)
{
    // never miss calling the parent implementation!!
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))

    // in StageFirst you can create and register your static pins.
    if (eStage == StageFirst)
    {
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
        RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER, (tVoid**)&pDescManager,__exception_ptr));


        // Input - Classification Result
        tChar const * strClassificationResult = pDescManager->GetMediaDescription("tClassificationResult");
        RETURN_IF_POINTER_NULL(strClassificationResult);
        cObjectPtr<IMediaType> pTypeClassificationResult = new cMediaType(0, 0, 0, "tClassificationResult", strClassificationResult, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(pTypeClassificationResult->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionInputClassificationResult));
        RETURN_IF_FAILED(m_oInputClassificationResult.Create("Classification Result", pTypeClassificationResult, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oInputClassificationResult));


        // Input - Zebracrossing Start
        tChar const * strZebracrossingStart = pDescManager->GetMediaDescription("tBoolSignalValue");
        RETURN_IF_POINTER_NULL(strZebracrossingStart);
        cObjectPtr<IMediaType> pTypeZebracrossingStart = new cMediaType(0, 0, 0, "tBoolSignalValue", strZebracrossingStart, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(pTypeZebracrossingStart->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionInputZebracrossingStart));
        RETURN_IF_FAILED(m_oInputZebracrossingStart.Create("Zebracrossing Start", pTypeZebracrossingStart, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oInputZebracrossingStart));


        // Input - Speed Controller
        tChar const * strInputRoadSignExt = pDescManager->GetMediaDescription("tRoadSignExt");
        RETURN_IF_POINTER_NULL(strInputRoadSignExt);
        cObjectPtr<IMediaType> pTypeInputRoadSignExt = new cMediaType(0, 0, 0, "tRoadSignExt", strInputRoadSignExt, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(pTypeInputRoadSignExt->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionInputRoadSignExt));
        RETURN_IF_FAILED(m_oInputRoadSignExt.Create("Road Sign Ext", pTypeInputRoadSignExt, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oInputRoadSignExt));


        // Input - Speed Controller
        tChar const * strInputSpeedController = pDescManager->GetMediaDescription("tSignalValue");
        RETURN_IF_POINTER_NULL(strInputSpeedController);
        cObjectPtr<IMediaType> pTypeInputSpeedController = new cMediaType(0, 0, 0, "tSignalValue", strInputSpeedController, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(pTypeInputSpeedController->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionInputSpeedController));
        RETURN_IF_FAILED(m_oInputSpeedController.Create("Speed Controller In", pTypeInputSpeedController, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oInputSpeedController));

        // Input - Steering Angle
        tChar const * strSteeringAngle = pDescManager->GetMediaDescription("tSignalValue");
        RETURN_IF_POINTER_NULL(strSteeringAngle);
        cObjectPtr<IMediaType> pTypeSteeringAngle = new cMediaType(0, 0, 0, "tSignalValue", strSteeringAngle, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(pTypeSteeringAngle->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionInputSteeringAngle));
        RETURN_IF_FAILED(m_oInputSteeringAngle.Create("Steering Angle", pTypeSteeringAngle, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oInputSteeringAngle));


        // Input - StoplineDetection
        tChar const * strStoplineDetection = pDescManager->GetMediaDescription("tStoplineStruct");
        RETURN_IF_POINTER_NULL(strStoplineDetection);
        cObjectPtr<IMediaType> pTypeStoplineDetection = new cMediaType(0, 0, 0, "tStoplineStruct", strStoplineDetection, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(pTypeStoplineDetection->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionInputStoplineDetection));
        RETURN_IF_FAILED(m_oInputStoplineDetection.Create("Sropline Detection", pTypeStoplineDetection, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oInputStoplineDetection));


        // Input - Distance Overall
        tChar const * strDescSignalInputDistanceoverall = pDescManager->GetMediaDescription("tSignalValue");
        RETURN_IF_POINTER_NULL(strDescSignalInputDistanceoverall);
        cObjectPtr<IMediaType> pTypeSignalInputDistanceoverall = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalInputDistanceoverall, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(pTypeSignalInputDistanceoverall->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionInputDistanceoverall));
        RETURN_IF_FAILED(m_oInputDistanceoverall.Create("Distance_Overall", pTypeSignalInputDistanceoverall, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oInputDistanceoverall));



        // Output - Brake lights
        tChar const * strOutputBrakeLights = pDescManager->GetMediaDescription("tBoolSignalValue");
        RETURN_IF_POINTER_NULL(strOutputBrakeLights);
        cObjectPtr<IMediaType> pTypeOutputBrakeLights = new cMediaType(0, 0, 0, "tBoolSignalValue", strOutputBrakeLights, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

        RETURN_IF_FAILED(pTypeOutputBrakeLights->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionOutputBrakeLights));
        RETURN_IF_FAILED(m_oOutputBrakeLights.Create("Brakelights", pTypeOutputBrakeLights, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oOutputBrakeLights));

        // Output - Speed Controller
        tChar const * strOutputSpeedController = pDescManager->GetMediaDescription("tSignalValue");
        RETURN_IF_POINTER_NULL(strOutputSpeedController);
        cObjectPtr<IMediaType> pTypeOutputSpeedController = new cMediaType(0, 0, 0, "tSignalValue", strOutputSpeedController, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

        RETURN_IF_FAILED(pTypeOutputSpeedController->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionOutputSpeedController));
        RETURN_IF_FAILED(m_oOutputSpeedController.Create("Speed Controller Out", pTypeOutputSpeedController, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oOutputSpeedController));

        // Output - Steering Angle
        tChar const * strOutputSteeringAngle = pDescManager->GetMediaDescription("tSignalValue");
        RETURN_IF_POINTER_NULL(strOutputSteeringAngle);
        cObjectPtr<IMediaType> pTypeOutputSteeringAngle = new cMediaType(0, 0, 0, "tSignalValue", strOutputSteeringAngle, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

        RETURN_IF_FAILED(pTypeOutputSteeringAngle->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionOutputSteeringAngle));
        RETURN_IF_FAILED(m_oOutputSteeringAngle.Create("Steering Angle Out", pTypeOutputSteeringAngle, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oOutputSteeringAngle));

        // Input - Zebracrossing Finished
        tChar const * strZebracrossingFinished = pDescManager->GetMediaDescription("tBoolSignalValue");
        RETURN_IF_POINTER_NULL(strZebracrossingFinished);
        cObjectPtr<IMediaType> pTypeZebracrossingFinished = new cMediaType(0, 0, 0, "tBoolSignalValue", strZebracrossingFinished, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(pTypeZebracrossingFinished->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionInputZebracrossingFinished));
        RETURN_IF_FAILED(m_oInputZebracrossingFinished.Create("Zebracrossing Finished", pTypeZebracrossingFinished, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oInputZebracrossingFinished));

    }
    else if (eStage == StageNormal)
    {
        // In this stage you would do further initialisation and/or create your dynamic pins.
        // Please take a look at the demo_dynamicpin example for further reference.



        m_iClassificationEvaluation_State = 0;
        m_iProcessSituation_State = 0;
        m_iZebracrossing_State = 0;




        m_bPedestrianMovmentDetected = false;
        m_bPedestrianPassed = false;
        m_bBrakeLightsOn = false;
        m_bStateFirstAccess = true;


        // Inputs
        m_f32InputDistanceOverall = 0;

        m_f32InputSteeringAngle = 0;
        m_f32InputVelocity = 0;
        m_bZebracrossingSign = false;
        m_bPedestrianDetected = false;
        m_i8CrossingCounter = 0;
        m_iZebraPedestrianStage = 1;

        m_f32StartDistanceOverall = 0;
        m_f32StoplineDistanceOverall = 0;

        m_bBrakeLightsOn = false;
        m_tsBrakeLights = 0;

        m_bEnteredLeft = false;
        m_bEnteredStreetLeft = false;
        m_bEnteredStreetCenter = false;
        m_bEnteredStreetRight = false;
        m_bEnteredRight = false;

        m_bZebraSignDetected = false;
        m_i16ZebraCrossingSignCounter = 0;




        m_f32VelocityZebracrossing =     GetPropertyFloat("Zebracrossing::Velocity [m/s]");
        m_f32ZebracrossingStopDistance = GetPropertyFloat("Zebracrossing::Distance for waiting [cm]");
        m_f32MaxTime2Wait =              GetPropertyFloat("Zebracrossing::Max. time to wait before Zebracrossing [s]");


        m_bTestPedestrianDetected = GetPropertyBool("Test Pedestrian Detected");
        m_bDebugOutputEnabled = GetPropertyBool("Debug output to console enabled");


    }
    else if (eStage == StageGraphReady)
    {
        // All pin connections have been established in this stage so you can query your pins
        // about their media types and additional meta data.
        // Please take a look at the demo_imageproc example for further reference.
    }

    RETURN_NOERROR;
}

tResult cPedestrianDetection::Shutdown(tInitStage eStage, __exception)
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

tResult cPedestrianDetection::OnPinEvent(IPin* pSource,
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

        // Received MediaSample Velocity
        if (pSource == &m_oInputSpeedController)
        {
            // Read out the media sample
            cObjectPtr<IMediaCoder> pCoderInput;
            RETURN_IF_FAILED(m_pDescriptionInputSpeedController->Lock(pMediaSample, &pCoderInput));
            pCoderInput->Get("f32Value", (tVoid*)&m_f32InputVelocity);
            m_pDescriptionInputSpeedController->Unlock(pCoderInput);

          //  evaluateSituation();
        }

        // Received Media Sample Steering
        else if (pSource == &m_oInputSteeringAngle)
        {
            // Read out the media sample
            cObjectPtr<IMediaCoder> pCoderInput;
            RETURN_IF_FAILED(m_pDescriptionInputSteeringAngle->Lock(pMediaSample, &pCoderInput));
            pCoderInput->Get("f32Value", (tVoid*)&m_f32InputSteeringAngle);
            m_pDescriptionInputSteeringAngle->Unlock(pCoderInput);

            ProcessSituation();
        }


        // Received Media Sample for RoadSignExt
        else if(pSource == &m_oInputRoadSignExt)
        {
            tFloat32 f32ArrDistanceX[3];
            tFloat32 f32ArrDistanceY[3];

            tInt16 i16RoadSignID = 0;



            // Read out the media sample
            cObjectPtr<IMediaCoder> pCoderInput;
            RETURN_IF_FAILED(m_pDescriptionInputRoadSignExt->Lock(pMediaSample, &pCoderInput));
            pCoderInput->Get("af32RVec", (tVoid*)&f32ArrDistanceX);
            pCoderInput->Get("af32TVec", (tVoid*)&f32ArrDistanceY);
            pCoderInput->Get("i16Identifier", (tVoid*)&i16RoadSignID);
            m_pDescriptionInputRoadSignExt->Unlock(pCoderInput);

            tFloat32 f32DistanceX = 0;
            f32DistanceX = f32ArrDistanceX[2];
            tFloat32 f32DistanceY = 0;
            f32DistanceY= f32ArrDistanceY[2];


            // If ID is zebracrossing
            if(i16RoadSignID == 6)
            {
                // m_f32Distance2RoadSign = sqrt((f32DistanceX*f32DistanceX) + (f32DistanceY*f32DistanceY));
                m_f32Distance2RoadSign = f32DistanceY;
                m_i16ZebraCrossingSignCounter = 0;
                m_bZebraSignDetected = true;
            }
        }


        // Received Media Sample for starting Zebracrossing
        else if (pSource == &m_oInputZebracrossingStart)
        {
            // Read out the media sample
            cObjectPtr<IMediaCoder> pCoderInput;
            RETURN_IF_FAILED(m_pDescriptionInputZebracrossingStart->Lock(pMediaSample, &pCoderInput));
            pCoderInput->Get("bValue", (tVoid*)&m_bZebracrossingSign);
            m_pDescriptionInputZebracrossingStart->Unlock(pCoderInput);

           // if(m_bDebugOutputEnabled) LOG_INFO(cString::Format("Received Zebracrossisng Start"));
            // Get the current distance at the start of the zebracrossing maneuver
            if(m_bZebracrossingSign){
                m_f32StartDistanceOverall = m_f32InputDistanceOverall;
            }
        }

        // Received Media Sample for Stopline Detection
        else if (pSource == &m_oInputStoplineDetection)
        {
            // Read out the media sample
            cObjectPtr<IMediaCoder> pCoderInput;
            RETURN_IF_FAILED(m_pDescriptionInputStoplineDetection->Lock(pMediaSample, &pCoderInput));
            pCoderInput->Get("bValue", (tVoid*)&m_bStoplineDetected);
            pCoderInput->Get("f32Distance", (tVoid*)&m_f32StoplineDistance);
            m_pDescriptionInputStoplineDetection->Unlock(pCoderInput);
        }     

        // Received Media Sample for Distance overall
        else if (pSource == &m_oInputDistanceoverall)
        {
            // Read out the media sample
            cObjectPtr<IMediaCoder> pCoderInput;
            RETURN_IF_FAILED(m_pDescriptionInputDistanceoverall->Lock(pMediaSample, &pCoderInput));
            pCoderInput->Get("f32Value", (tVoid*)&m_f32InputDistanceOverall);
            m_pDescriptionInputDistanceoverall->Unlock(pCoderInput);
        }

        // Received Media Sample for Classification
        else if(pSource == &m_oInputClassificationResult)
        {
            // Thrift connection
            /*
            std::vector<classificationResult> classificationResults;
            classificationResults.resize(pMediaSample->GetSize() / sizeof(classificationResult));
            //get the date from the media sample
            tVoid* pIncomingData;
            if (IS_OK(pMediaSample->Lock((const tVoid**)&pIncomingData)))
            {
                //make copy
                memcpy(classificationResults.data(), pIncomingData, pMediaSample->GetSize());
                pMediaSample->Unlock(pIncomingData);
            }
            // Get the data out of the vector
            tFloat64 f32ClassificationPropability[9];
            cString strClassificationName[9];
            for(tInt i =0;i<9;i++){
                f32ClassificationPropability[i]=0;
                strClassificationName[i] = "";
            }
            if (!classificationResults.empty())
            {
                tInt i = 0;
                // emit resetClassificationResults();
                for (std::vector<classificationResult>::iterator it = classificationResults.begin(); it != classificationResults.end(); it++)
                {
                    //emit sendClassificationResult(it->classificationDesc, it->probability);
                    f32ClassificationPropability[i] = it->probability;
                    strClassificationName[i] = it->classificationDesc;
                    i++;
                }
            }

            // Find the value with the biggest propability
            tFloat32 value = 0;
            tInt16 iteration = 0;
            for(tInt i=0; i<9;i++){
                if(value < f32ClassificationPropability[i]){
                    value = f32ClassificationPropability[i];
                    iteration = i;
                }
            }

            // What kind of pedestrian is detected
            if(strClassificationName[iteration]== "nothing"){
                 m_bPedestrianDetected = false;
            }
            else if(strClassificationName[iteration]== "child"){
                 m_bPedestrianDetected = true;
            }
            else if(strClassificationName[iteration]== "adult"){
                 m_bPedestrianDetected = true;
            }
            else if(strClassificationName[iteration]== "adult_right"){
                 m_bPedestrianDetected = true;
            }
            else if(strClassificationName[iteration]== "adult_left"){
                 m_bPedestrianDetected = true;
            }
            else{
                // Same like nothing
                 m_bPedestrianDetected = false;
            }
            */


            tInt16 i16PositionX;
            tFloat32 f32Distance;
            tInt16 i16BoxWidth;
            tInt16 i16BoxHeight;
            tInt16 i16ClassificationResult;

            // Read out the media sample
            cObjectPtr<IMediaCoder> pCoderInput;
            RETURN_IF_FAILED(m_pDescriptionInputClassificationResult->Lock(pMediaSample, &pCoderInput));
            pCoderInput->Get("i16PositionX", (tVoid*)&i16PositionX);
            pCoderInput->Get("f32Distance", (tVoid*)&f32Distance);
            pCoderInput->Get("i16BoxWidth", (tVoid*)&i16BoxWidth);
            pCoderInput->Get("i16BoxHeight", (tVoid*)&i16BoxHeight);
            pCoderInput->Get("i16ClassificationResult", (tVoid*)&i16ClassificationResult);
            m_pDescriptionInputClassificationResult->Unlock(pCoderInput);


            EvaluateClassification(i16PositionX, f32Distance, i16ClassificationResult);
        }
    }
    RETURN_NOERROR;
}



tResult cPedestrianDetection::EvaluateClassification(tInt16 i16PositionX, tFloat32 f32Distance, tInt16 i16ClassificationResult){


    switch(m_iClassificationEvaluation_State)
    {

        // New information received -> delete the oldest element in the array
        for(tInt i = 49; i > 0; i--)
        {
            for(tInt j = 0; j<5; j++)
            {
                m_f32ClassificationInformation[i][j] = m_f32ClassificationInformation[i-1][j];
            }
        }

        // Normal evaluation mode during driving
        case EV_NORMAL:
        {
            /*
                if(child is detected)
                {
                    m_iProcessSituation_State = CHILD;
                }
            */
        }
        break;


        // Evaluate Classification before driving to Zebracrossing
        case EV_ZEBRA:
        {


        }
        break;


        // Try to detect a movement of the pedestrian
        case EV_WAITING:
        {

        }
        break;


        default:
        {
            m_iClassificationEvaluation_State = EV_NORMAL;
        }
        break;

    }

    RETURN_NOERROR;
}



tResult cPedestrianDetection::ProcessSituation()
{
    tFloat32    f32OutputVelocity;
    tFloat32    f32OutputSteering;

    switch(m_iProcessSituation_State)
    {
        // No pedestrian detected - passing the velocity and steering angle
        case NORMAL:
        {
           //  if(m_bDebugOutputEnabled) LOG_INFO(cString::Format("STATE: NORMAL %i", NORMAL));
            f32OutputVelocity = m_f32InputVelocity;
            f32OutputSteering = m_f32InputSteeringAngle;

            if(m_bZebracrossingSign)
            {
                // Switch to ZEBRA-state for maneuvers
                m_iProcessSituation_State   = ZEBRA;
                m_iZebracrossing_State      = DRIVING_2_ZEBRA;
                // Switch to ZEBRA-state for evaluation
                m_iClassificationEvaluation_State = EV_ZEBRA;

                // Reset the bool
                m_bZebracrossingSign = false;
            }
        }
        break;


        // Zebracrossing is incoming - slow down and evaluate
        case ZEBRA:
        {
                switch(m_iZebracrossing_State)
                {

                    // Drive to the zebracrossing
                    case DRIVING_2_ZEBRA:
                    {
                        if(m_bStateFirstAccess == true)
                        {
                    //        if(m_bDebugOutputEnabled) LOG_INFO(cString::Format("STATE: DRIVING_2_ZEBRA %i", DRIVING_2_ZEBRA));
                            m_bStateFirstAccess = false;
                            m_f32Distance_EnteredState = m_f32InputDistanceOverall;
                        }

                        // Drive with reduces velocity
                        f32OutputVelocity = m_f32VelocityZebracrossing;
                        f32OutputSteering = m_f32InputSteeringAngle;

                        // Stop 70 cm ahead of the zebracrossing itself
                        //if(m_bDebugOutputEnabled) LOG_INFO(cString::Format("STATE: DRIVING_2_ZEBRA::    m_f32StoplineDistance = %f, m_f32ZebracrossingStopDistance = %f, m_bStoplineDetected = %i ", m_f32StoplineDistance, m_f32ZebracrossingStopDistance, m_bStoplineDetected));

                        // Driving to Zebracrossing by Stopline
                        // if(m_f32StoplineDistance < m_f32ZebracrossingStopDistance  &&  m_bStoplineDetected)

                        // Driving to Zebracrossing by TrafficSign
                   //     if(m_bDebugOutputEnabled) LOG_INFO(cString::Format("Counter for loosing Zebrasign: m_bZebraSignDetected = %i, m_i16ZebraCrossingSignCounter = %i ", m_bZebraSignDetected, m_i16ZebraCrossingSignCounter));

                        if(m_bZebraSignDetected == false && m_i16ZebraCrossingSignCounter > 20)
                        {


                            // Check if a pedestrian is detected
                            if(true)
                            {
                       //         if(m_bDebugOutputEnabled) LOG_INFO(cString::Format("STATE: DRIVING_2_ZEBRA:: Pedestrian detected -> WAITING"));
                                // Switch to EV_WAITING-state for maneuvers
                                m_iZebracrossing_State = WAITING_AT_ZEBRA;
                                // Switch to EV_WAITING-state for evaluation
                                m_iClassificationEvaluation_State = EV_WAITING;

                                m_bStateFirstAccess = true;
                            }

                            else
                            {
                                // No Pedestrian Detected -> continue driving without stopping
                        //        if(m_bDebugOutputEnabled) LOG_INFO(cString::Format("STATE: DRIVING_2_ZEBRA:: NO Pedestrian detected -> PASS"));

                                // Switch to PASS_ZEBRA-state for maneuvers
                                m_iZebracrossing_State = PASS_ZEBRA;

                                m_bStateFirstAccess = true;
                            }


                        }
                        m_i16ZebraCrossingSignCounter++;
                        m_bZebraSignDetected = false;

                        /*
                        // Safety-exit: when no stopline is ever detected, end the maneuver after 4 meters
                        if(m_f32InputDistanceOverall - m_f32Distance_EnteredState > 1.5 && m_bStateFirstAccess == false)
                        {
                            // Switch to NORMAL-state for maneuvers
                            m_iProcessSituation_State = NORMAL;
                            // Switch to EV_NORMAL-state for evaluation
                            m_iClassificationEvaluation_State = EV_NORMAL;

                            sendFinishFlag();
                            m_bStateFirstAccess = true;
                        }
                        */

                    }
                    break;


                    // wait at the zebracrossing
                    case WAITING_AT_ZEBRA:
                    {
                        tBool   bContinueDriving = false;

                        if(m_bStateFirstAccess == true)
                        {
                       //     if(m_bDebugOutputEnabled) LOG_INFO(cString::Format("STATE: WAITING_AT_ZEBRA %i", WAITING_AT_ZEBRA));
                            m_bStateFirstAccess = false;
                            m_tsStateFirstAccess = _clock->GetStreamTime();


                            // Turn Brake Lights ON
                            if(m_bBrakeLightsOn == false)
                            {
                                toggleBrakeLights();
                            }
                        }

                        // Stop
                        f32OutputVelocity = 0;
                        f32OutputSteering = 0;



                        // When you get th confirmation that the pedestrian passed the zebracrossing
                        if(m_bPedestrianPassed == true && m_bPedestrianMovmentDetected  == false)
                        {
                      //      if(m_bDebugOutputEnabled) LOG_INFO(cString::Format("STATE: WAITING_AT_ZEBRA: detected crossing of the pedestrian"));
                            bContinueDriving = true;
                        }


                        // Safety exit before deadlock - when there was a detection earlier but now nothing is detected anymore
                        if((m_tsStateFirstAccess + (7 * 1000000) < _clock->GetStreamTime()) && m_bStateFirstAccess == false)
                        {
                 //           if(m_bDebugOutputEnabled) LOG_INFO(cString::Format("STATE: WAITING_AT_ZEBRA: waited for the max time = %f [s]", m_f32MaxTime2Wait));
                            bContinueDriving = true;
                        }


                        if(bContinueDriving == true)
                        {
                            // Switch to PASS_ZEBRA-state for maneuvers
                            m_iZebracrossing_State = PASS_ZEBRA;
                            m_bStateFirstAccess = true;

                            // Turn Brake Lights OFF
                            if(m_bBrakeLightsOn == true)
                            {
                                toggleBrakeLights();
                            }
                        }

                    }
                    break;


                    // continue driving
                    case PASS_ZEBRA:
                    {
                        if(m_bStateFirstAccess == true)
                        {
                  //          if(m_bDebugOutputEnabled) LOG_INFO(cString::Format("STATE: PASS_ZEBRA %i", PASS_ZEBRA));
                            m_bStateFirstAccess = false;
                            m_f32Distance_EnteredState = m_f32InputDistanceOverall;
                        }


                        // Drive with reduces velocity
                        f32OutputVelocity = m_f32VelocityZebracrossing;
                        f32OutputSteering = m_f32InputSteeringAngle;


                        // Leave this state after 1m and start driving normal again
                        if(m_f32InputDistanceOverall - m_f32Distance_EnteredState > 1 && m_bStateFirstAccess == false)
                        {
                            // Switch to NORMAL-state for maneuvers
                            m_iProcessSituation_State = NORMAL;
                            // Switch to EV_NORMAL-state for evaluation
                            m_iClassificationEvaluation_State = EV_NORMAL;

                            m_bStateFirstAccess = true;

                            // Send finish-flag to Situation Detection
                            sendFinishFlag();
                        }
                    }
                    break;

                    default:
                    {
                        m_iZebracrossing_State = PASS_ZEBRA;
                    }
                }
        }
        break;


        // Child on a road is detected - slow down and enable brake lights
        case  CHILD :
        {
            if(m_bStateFirstAccess == true)
            {
          //      if(m_bDebugOutputEnabled) LOG_INFO(cString::Format("STATE: CHILD %i", CHILD));
                m_bStateFirstAccess = false;
                m_f32Distance_EnteredState = m_f32InputDistanceOverall;

                // Turn Brake Lights ON
                if(m_bBrakeLightsOn == false)
                {
                    toggleBrakeLights();
                }
            }

            // Drive with reduces velocity
            f32OutputVelocity = m_f32InputVelocity * 0.7;
            f32OutputSteering = m_f32InputSteeringAngle;

            // TODO make sure that not a already reduced velocity is multiplied by 0.7


            // Leave this state after 1.5m
            if(m_f32InputDistanceOverall - m_f32Distance_EnteredState > 1.5 && m_bStateFirstAccess == false)
            {
                m_bStateFirstAccess = true;
                // Switch to NORMAL-state for maneuvers
                m_iProcessSituation_State = NORMAL;

                // Turn Brake Lights OFF
                if(m_bBrakeLightsOn)
                {
                    toggleBrakeLights();
                }
            }



            if(m_bZebracrossingSign)
            {
                // Switch to ZEBRA-state for maneuvers
                m_iProcessSituation_State   = ZEBRA;
                m_iZebracrossing_State      = DRIVING_2_ZEBRA;
                // Switch to ZEBRA-state for evaluation
                m_iClassificationEvaluation_State = EV_ZEBRA;

                // Reset the bool
                m_bZebracrossingSign = false;
            }

        }
        break;


        default:
        {
            m_iProcessSituation_State = NORMAL;
        }
        break;

    }


    // Sending the outputs
    SendingOutput(f32OutputVelocity, f32OutputSteering);


    RETURN_NOERROR;
}



tResult cPedestrianDetection::SendingOutput(tFloat32 f32OutputVelocity, tFloat32 f32OutputSteeringAngle){

    // Send MediaSample for SpeedController
    tTimeStamp stTimestamp = _clock->GetStreamTime();
    cObjectPtr<IMediaSample> pMediaSampleOutputSpeedController;
    AllocMediaSample((tVoid**)&pMediaSampleOutputSpeedController);

    cObjectPtr<IMediaSerializer> pSerializerOutputSpeedController;
    m_pDescriptionOutputSpeedController->GetMediaSampleSerializer(&pSerializerOutputSpeedController);
    tInt nSizeOutputSpeedController = pSerializerOutputSpeedController->GetDeserializedSize();
    pMediaSampleOutputSpeedController->AllocBuffer(nSizeOutputSpeedController);
    cObjectPtr<IMediaCoder> pCoderOutputSpeedController;
    m_pDescriptionOutputSpeedController->WriteLock(pMediaSampleOutputSpeedController, &pCoderOutputSpeedController);
    pCoderOutputSpeedController->Set("ui32ArduinoTimestamp", (tVoid*)&stTimestamp);
    pCoderOutputSpeedController->Set("f32Value", (tVoid*)&(f32OutputVelocity));
    m_pDescriptionOutputSpeedController->Unlock(pCoderOutputSpeedController);
    pMediaSampleOutputSpeedController->SetTime(_clock->GetStreamTime());
    m_oOutputSpeedController.Transmit(pMediaSampleOutputSpeedController);


    // Send MediaSample for Steering Controller
    cObjectPtr<IMediaSample> pMediaSampleOutputSteeringAngle;
    AllocMediaSample((tVoid**)&pMediaSampleOutputSteeringAngle);

    cObjectPtr<IMediaSerializer> pSerializerOutputSteeringAngle;
    m_pDescriptionOutputSteeringAngle->GetMediaSampleSerializer(&pSerializerOutputSteeringAngle);
    tInt nSizeOutputSteeringAngle = pSerializerOutputSteeringAngle->GetDeserializedSize();
    pMediaSampleOutputSteeringAngle->AllocBuffer(nSizeOutputSteeringAngle);
    cObjectPtr<IMediaCoder> pCoderOutputSteeringAngle;
    m_pDescriptionOutputSteeringAngle->WriteLock(pMediaSampleOutputSteeringAngle, &pCoderOutputSteeringAngle);
    pCoderOutputSteeringAngle->Set("ui32ArduinoTimestamp", (tVoid*)&stTimestamp);
    pCoderOutputSteeringAngle->Set("f32Value", (tVoid*)&(f32OutputSteeringAngle));
    m_pDescriptionOutputSteeringAngle->Unlock(pCoderOutputSteeringAngle);
    pMediaSampleOutputSteeringAngle->SetTime(_clock->GetStreamTime());
    m_oOutputSteeringAngle.Transmit(pMediaSampleOutputSteeringAngle);

    RETURN_NOERROR;
}


tResult cPedestrianDetection::toggleBrakeLights(){
    m_bBrakeLightsOn = !m_bBrakeLightsOn;
    // Send MediaSample for BrakeLights
    cObjectPtr<IMediaSample> pMediaSampleOutputBrakeLights;
    AllocMediaSample((tVoid**)&pMediaSampleOutputBrakeLights);
    cObjectPtr<IMediaSerializer> pSerializerOutputBrakeLights;
    m_pDescriptionOutputBrakeLights->GetMediaSampleSerializer(&pSerializerOutputBrakeLights);
    tInt nSizeOutputBrakeLights = pSerializerOutputBrakeLights->GetDeserializedSize();
    pMediaSampleOutputBrakeLights->AllocBuffer(nSizeOutputBrakeLights);
    cObjectPtr<IMediaCoder> pCoderOutputBrakeLights;
    m_pDescriptionOutputBrakeLights->WriteLock(pMediaSampleOutputBrakeLights, &pCoderOutputBrakeLights);
    pCoderOutputBrakeLights->Set("bValue", (tVoid*)&(m_bBrakeLightsOn));
    m_pDescriptionOutputBrakeLights->Unlock(pCoderOutputBrakeLights);
    pMediaSampleOutputBrakeLights->SetTime(_clock->GetStreamTime());
    m_oOutputBrakeLights.Transmit(pMediaSampleOutputBrakeLights);

    RETURN_NOERROR;
}


tResult cPedestrianDetection::sendFinishFlag(){
    tBool bFinished = true;
    // Create a new MediaSmaple
    cObjectPtr<IMediaSample> pMediaSampleZebracrossingFinished;
    AllocMediaSample((tVoid**)&pMediaSampleZebracrossingFinished);
    // Send the Media Sample
    cObjectPtr<IMediaSerializer> pSerializerZebracrossingFinished;
    m_pDescriptionInputZebracrossingFinished->GetMediaSampleSerializer(&pSerializerZebracrossingFinished);
    tInt nSizeZebracrossingFinished = pSerializerZebracrossingFinished->GetDeserializedSize();
    pMediaSampleZebracrossingFinished->AllocBuffer(nSizeZebracrossingFinished);
    cObjectPtr<IMediaCoder> pCoderOutputZebracrossingFinished;
    m_pDescriptionInputZebracrossingFinished->WriteLock(pMediaSampleZebracrossingFinished, &pCoderOutputZebracrossingFinished);
    pCoderOutputZebracrossingFinished->Set("bValue", (tVoid*)&(bFinished));
    //pCoderOutputZebracrossingFinished->Set("ui32ArduinoTimestamp", (tVoid*)_clock->GetStreamTime());
    m_pDescriptionInputZebracrossingFinished->Unlock(pCoderOutputZebracrossingFinished);
    pMediaSampleZebracrossingFinished->SetTime(_clock->GetStreamTime());
    m_oInputZebracrossingFinished.Transmit(pMediaSampleZebracrossingFinished);

    RETURN_NOERROR;
}
