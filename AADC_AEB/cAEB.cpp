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
#include "cAEB.h" 
/// Create filter shell
ADTF_FILTER_PLUGIN("AEB", OID_ADTF_TEMPLATE_FILTER, cAEB);

// CONSTRUCTOR get called at first
// EDIT
cAEB::cAEB(const tChar* __info):cFilter(__info)
{
        m_szIdsUsStructSet = tFalse;
        m_szIdsInputSpeedSet = tFalse;
        m_szIdsOutputSpeedSet = tFalse;
        SetPropertyFloat("FRONT Emergency Brake Distance [cm]", 50.0f);
        SetPropertyFloat("FRONT weight on US left and right",20.0f);

        SetPropertyFloat("BACK Emergency Brake Distance [cm]", 10.0f);
        SetPropertyFloat("BACK weight on US left and right",5.0f);

        SetPropertyFloat("Critical Steering Angle", 20.0f);

        SetPropertyBool("Debug Mode enabled", true);

        SetPropertyFloat("Set the min. distance for US to detect", 3.0f);
}

// DECONSTRUCTOR
cAEB::~cAEB()
{

}


// INIT is called after CONSTRUCTOR
// EDIT
tResult cAEB::Init(tInitStage eStage, __exception)
{
    // never miss calling the parent implementation!!
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))

    // in StageFirst you can create and register your static pins.
    if (eStage == StageFirst)
    {

            cObjectPtr<IMediaDescriptionManager> pDescManager;
            RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER, (tVoid**)&pDescManager,__exception_ptr));


            // media type for Signal Values
            tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");
            RETURN_IF_POINTER_NULL(strDescSignalValue);
            cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

            //get mediatype description for Signal data type
            RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionSignalValue));

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


            // Input - US Struct
            tChar const * strUSStruct = pDescManager->GetMediaDescription("tUltrasonicStruct");
            RETURN_IF_POINTER_NULL(strUSStruct);
            cObjectPtr<IMediaType> pTypeUSStruct = new cMediaType(0, 0, 0, "tUltrasonicStruct", strUSStruct, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
            RETURN_IF_FAILED(pTypeUSStruct->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionInputUSStruct));
            RETURN_IF_FAILED(m_oInputUSStruct.Create("US Struct", pTypeUSStruct, static_cast<IPinEventSink*> (this)));
            RETURN_IF_FAILED(RegisterPin(&m_oInputUSStruct));


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


            tsEBactivated = _clock->GetStreamTime();
            tsEBactivatedFirst = _clock->GetStreamTime();
            tsLastSpeedInput = _clock->GetStreamTime();

            m_bEBactivateBack = false;
            m_bEBactivateFront = false;
            m_bBrakeLightsOn = false;


    }
    else if (eStage == StageNormal)
    {
        // In this stage you would do further initialisation and/or create your dynamic pins.
        // Please take a look at the demo_dynamicpin example for further reference.


        // Properties
        m_f32EmergencyBrakeDistanceFront = GetPropertyFloat("FRONT Emergency Brake Distance [cm]");
        m_f32EmergencyBrakeDistanceBack = GetPropertyFloat("BACK Emergency Brake Distance [cm]");

        m_f32Weight_sideFront = GetPropertyFloat("FRONT weight on US left and right");
        m_f32Weight_sideBack = GetPropertyFloat("BACK weight on US left and right");

        m_f32CriticalSteeringAngle = GetPropertyFloat("Critical Steering Angle");


        m_bDebugModeEnabled = GetPropertyBool("Debug Mode enabled");


        m_f32MinDistance4US = GetPropertyFloat("Set the min. distance for US to detect");



        m_f32SpeedInput = 0;


    }
    else if (eStage == StageGraphReady)
    {
        // All pin connections have been established in this stage so you can query your pins
        // about their media types and additional meta data.
        // Please take a look at the demo_imageproc example for further reference.
    }

    RETURN_NOERROR;
}


// Shutdown is the reverse function of the Init-Fct
// EDIT
tResult cAEB::Shutdown(tInitStage eStage, __exception)
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

    RETURN_NOERROR;
}



// This fuction will be called if anything happens with the input-pins (speedcontroller and UsSruct)
tResult cAEB::OnPinEvent(IPin* pSource,
                                    tInt nEventCode,
                                    tInt nParam1,
                                    tInt nParam2,
                                    IMediaSample* pMediaSample)
{
    // first check what kind of event it is
    if (nEventCode == IPinEventSink::PE_MediaSampleReceived)
    {
        // Is the event a MeidaSampleReceived-event go here

        // so we received a media sample, so this pointer better be valid.
        RETURN_IF_POINTER_NULL(pMediaSample);

        // Received a signal from the speed controller
        if(pSource == &m_oInputSpeedController)
        {
            // Read out steering angle
            cObjectPtr<IMediaCoder> pCoderInput;
            RETURN_IF_FAILED(m_pDescriptionInputSpeedController->Lock(pMediaSample, &pCoderInput));
            pCoderInput->Get("f32Value", (tVoid*)&m_f32SpeedInput);
            // pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timestamp);
            m_pDescriptionInputSpeedController->Unlock(pCoderInput);

            tsLastSpeedInput =  _clock->GetStreamTime();
        }
        // Received a signal from the ultra sound sensors
        else if(pSource == &m_oInputUSStruct)
        {
            tFloat32    f32USCenterLeft = 0;
            tFloat32    f32USCenterCenter = 0;
            tFloat32    f32USCenterRight = 0;

            tFloat32    f32USRearLeft = 0;
            tFloat32    f32USRearCenter = 0;
            tFloat32    f32USRearRight = 0;

            // Read out steering angle
            cObjectPtr<IMediaCoder> pCoderInput;
            RETURN_IF_FAILED(m_pDescriptionInputUSStruct->Lock(pMediaSample, &pCoderInput));
            pCoderInput->Get("tFrontCenterLeft.f32Value", (tVoid*)&f32USCenterLeft);
            // CoderInput->Get("tFrontCenterLeft.ui32ArduinoTimestamp", (tVoid*)&timestamp);
            pCoderInput->Get("tFrontCenter.f32Value", (tVoid*)&f32USCenterCenter);
            // pCoderInput->Get("tFrontCenter.ui32ArduinoTimestamp", (tVoid*)&timestamp);
            pCoderInput->Get("tFrontCenterRight.f32Value", (tVoid*)&f32USCenterRight);
            // pCoderInput->Get("tFrontCenterRight.ui32ArduinoTimestamp", (tVoid*)&timestamp);

            pCoderInput->Get("tSideRight.f32Value", (tVoid*)&f32USRearLeft);
            pCoderInput->Get("tRearCenter.f32Value", (tVoid*)&f32USRearCenter);
            pCoderInput->Get("tRearRight.f32Value", (tVoid*)&f32USRearRight);

            m_pDescriptionInputUSStruct->Unlock(pCoderInput);

            //if(m_bDebugModeEnabled) LOG_INFO(adtf_util::cString::Format("AEB: US left %f, US center %f, US right %f", f32USCenterLeft, f32USCenterCenter, f32USCenterRight));



            ProcessMinimumValueUs(f32USCenterLeft, f32USCenterCenter, f32USCenterRight,f32USRearLeft,f32USRearCenter,f32USRearRight);
        }
        else if(pSource == &m_oInputSteeringAngle){
            // Read out steering angle
            cObjectPtr<IMediaCoder> pCoderInput;
            RETURN_IF_FAILED(m_pDescriptionInputSteeringAngle->Lock(pMediaSample, &pCoderInput));
            pCoderInput->Get("f32Value", (tVoid*)&m_f32SteeringAngle);
            // pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timestamp);
            m_pDescriptionInputSteeringAngle->Unlock(pCoderInput);

            if(m_bDebugModeEnabled) LOG_INFO(adtf_util::cString::Format("AEB: Steering angle received: %f", m_f32SteeringAngle));
        }
    }
    RETURN_NOERROR;
}


tResult cAEB::ProcessMinimumValueUs(tFloat32 f32USCenterLeft, tFloat32 f32USCenterCenter, tFloat32 f32USCenterRight, tFloat32 f32USRearLeft, tFloat32 f32USRearCenter, tFloat32 f32USRearRight)
{
    // driving forward
    if(m_f32SpeedInput > 0)
    {
	m_bEBactivateBack = false;

        if(m_bDebugModeEnabled) LOG_INFO(adtf_util::cString::Format("Front"));
        // Check for unvalid US-values
        if(f32USCenterLeft == 0 || f32USCenterLeft == -1){
            f32USCenterLeft = 400;
        }
        if(f32USCenterCenter == 0 || f32USCenterCenter == -1){
            f32USCenterCenter = 400;
        }
        if(f32USCenterRight == 0 || f32USCenterRight == -1){
            f32USCenterRight = 400;
        }

        // Calculate min-Distance depending on Steering angle
        tFloat32 weight_left   = 0;
        tFloat32 weight_center = 0;
        tFloat32 weight_right  = 0;

        // TODO Testing
        // Steering left
        if(m_f32SteeringAngle < -m_f32CriticalSteeringAngle){
            if(m_bDebugModeEnabled) LOG_INFO(adtf_util::cString::Format("AEB: Steering LEFT"));
             weight_left   = 0;
             weight_center = m_f32Weight_sideFront;
             weight_right  = m_f32Weight_sideFront*1.5;
        }
        // Steering straight
        else if((m_f32SteeringAngle > -m_f32CriticalSteeringAngle) && (m_f32SteeringAngle < m_f32CriticalSteeringAngle)){
            if(m_bDebugModeEnabled) LOG_INFO(adtf_util::cString::Format("AEB: Steering STRAIGHT"));
             weight_left   = m_f32Weight_sideFront;
             weight_center = 0;
             weight_right  = m_f32Weight_sideFront;
        }
        // Steering right
        else if(m_f32SteeringAngle > m_f32CriticalSteeringAngle){
            if(m_bDebugModeEnabled) LOG_INFO(adtf_util::cString::Format("AEB: Steering RIGHT"));
             weight_left   = m_f32Weight_sideFront*1.5;
             weight_center = m_f32Weight_sideFront;
             weight_right  = 0;
        }


        // Is one of the current US-values smaller than the Min-Distance to trigger AEB (and is bigger than the)?
        if((f32USCenterLeft  < m_f32EmergencyBrakeDistanceFront - weight_left)     && m_f32MinDistance4US > f32USCenterLeft ||
           (f32USCenterCenter < m_f32EmergencyBrakeDistanceFront - weight_center)  && m_f32MinDistance4US > f32USCenterLeft ||
           (f32USCenterRight < m_f32EmergencyBrakeDistanceFront - weight_right)    && m_f32MinDistance4US > f32USCenterLeft)
         {
             // do emergency brake
             m_bEBactivateFront = true;
             tsEBactivated = _clock->GetStreamTime();
             if(m_bDebugModeEnabled) LOG_INFO(adtf_util::cString::Format("AEB: EB-Front triggered"));
         }
         else
         {
             // Wait for 1,5 seconds (=1500000) after the emergency brake was activated
             if(tsEBactivated+1500000 < _clock->GetStreamTime()){
                 // pass through the speed
                 m_bEBactivateFront = false;
             } else {
                 if(m_bDebugModeEnabled) LOG_INFO(adtf_util::cString::Format("Wait 2 seconds after EB-Front"));
             }
         }

    }
    // driving backwards 
    else if (m_f32SpeedInput < 0)
    {
	m_bEBactivateFront = false;

        if(m_bDebugModeEnabled) LOG_INFO(adtf_util::cString::Format("Back"));
        // Check for unvalid US-values
        if(f32USRearLeft == 0 || f32USRearLeft == -1){
            f32USRearLeft = 400;
        }
        if(f32USRearCenter == 0 || f32USRearCenter == -1){
            f32USRearCenter = 400;
        }
        if(f32USRearRight == 0 || f32USRearRight == -1){
            f32USRearRight = 400;
        }

        // Is one of the current US-values smaller than the Min-Distance?
        if(f32USRearLeft  < m_f32EmergencyBrakeDistanceBack - m_f32Weight_sideBack||
           f32USRearCenter< m_f32EmergencyBrakeDistanceBack ||
           f32USRearRight < m_f32EmergencyBrakeDistanceBack - m_f32Weight_sideBack)
         {
             // do emergency brake
             m_bEBactivateBack = true;
             tsEBactivated = _clock->GetStreamTime();
             if(m_bDebugModeEnabled) LOG_INFO(adtf_util::cString::Format("AEB: EB-Back triggered"));
         }
         else
         {
             // Wait for 1,5 seconds (=1500000) after the emergency brake was activated
             if(tsEBactivated+1500000 < _clock->GetStreamTime()){
                 // pass through the speed
                 m_bEBactivateBack = false;
             } else {
                 if(m_bDebugModeEnabled) LOG_INFO(adtf_util::cString::Format("Wait 2 seconds after EB-Back"));
             }
         }
    }

    ProcessSpeedController();

    RETURN_NOERROR;
}



tResult cAEB::ProcessSpeedController()
{
    tFloat32 f32Speed;
    tFloat32 f32SteeringAngle;
    // Check if the other function detected a US-value that is smaller than the min-distance
    // AEB Front
    if(m_bEBactivateFront){
        // Stop
        // Make sure the engine is not making a strange noise
        if(tsEBactivatedFirst + 50000 > _clock->GetStreamTime()){
            f32Speed = -0.25;
        } else {
            f32Speed = 0; 
        }
        f32SteeringAngle = 0;
        // Brake lights on
        if(!m_bBrakeLightsOn){
            toggleBrakeLights();
            tsEBactivatedFirst = _clock->GetStreamTime();
        }
        if(m_bDebugModeEnabled) LOG_INFO(adtf_util::cString::Format("AEB: EB-Front set"));
    }
    // AEB Back
    else if(m_bEBactivateBack)
    {
        // Stop
        f32Speed = 0;
        f32SteeringAngle = 0;
        // Brake lights on
        if(!m_bBrakeLightsOn){
            toggleBrakeLights();
            tsEBactivatedFirst = _clock->GetStreamTime();
        }
        if(m_bDebugModeEnabled) LOG_INFO(adtf_util::cString::Format("AEB: EB-Back set"));
    }
    // Pass speed
    else
    {
        // Check if the input has stopped
        if(tsLastSpeedInput+700000 < _clock->GetStreamTime())
        {
            f32Speed = 0;
            f32SteeringAngle = 0;
            if(m_bDebugModeEnabled) LOG_INFO(adtf_util::cString::Format("AEB: No speed input"));
        }
        else
        {
            // Process current speed
            f32Speed = m_f32SpeedInput;
            f32SteeringAngle = m_f32SteeringAngle;
            //DisableBrakeLights(false);
            if(m_bBrakeLightsOn){
                toggleBrakeLights();
            }
        }


    }
     //if(m_bDebugModeEnabled) LOG_INFO(adtf_util::cString::Format("AEB: f32Speed %f", f32Speed));

     // Send MediaSample for SpeedController
     cObjectPtr<IMediaSample> pMediaSampleOutputSpeedController;
     AllocMediaSample((tVoid**)&pMediaSampleOutputSpeedController);

     cObjectPtr<IMediaSerializer> pSerializerOutputSpeedController;
     m_pDescriptionOutputSpeedController->GetMediaSampleSerializer(&pSerializerOutputSpeedController);
     tInt nSizeOutputSpeedController = pSerializerOutputSpeedController->GetDeserializedSize();
     pMediaSampleOutputSpeedController->AllocBuffer(nSizeOutputSpeedController);
     cObjectPtr<IMediaCoder> pCoderOutputSpeedController;
     m_pDescriptionOutputSpeedController->WriteLock(pMediaSampleOutputSpeedController, &pCoderOutputSpeedController);
     tTimeStamp timestamp;
     timestamp = _clock->GetStreamTime();
     pCoderOutputSpeedController->Set("ui32ArduinoTimestamp", (tVoid*)&timestamp);
     pCoderOutputSpeedController->Set("f32Value", (tVoid*)&(f32Speed));
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
     pCoderOutputSteeringAngle->Set("ui32ArduinoTimestamp", (tVoid*)&timestamp);
     pCoderOutputSteeringAngle->Set("f32Value", (tVoid*)&(f32SteeringAngle));
     m_pDescriptionOutputSteeringAngle->Unlock(pCoderOutputSteeringAngle);
     pMediaSampleOutputSteeringAngle->SetTime(_clock->GetStreamTime());
     m_oOutputSteeringAngle.Transmit(pMediaSampleOutputSteeringAngle);

     RETURN_NOERROR;
}


tResult cAEB::toggleBrakeLights(){
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






