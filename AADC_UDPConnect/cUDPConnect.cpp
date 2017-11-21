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
#include "cUDPConnect.h"
#include <stdlib.h>



/// Create filter shell
ADTF_FILTER_PLUGIN("UDP V2", OID_ADTF_UDP, cUDPConnect);


cUDPConnect::cUDPConnect(const tChar* __info):cFilter(__info)
{

}

cUDPConnect::~cUDPConnect()
{

}

tResult cUDPConnect::Init(tInitStage eStage, __exception)
{
    // never miss calling the parent implementation!!
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))

            // in StageFirst you can create and register your static pins.
            if (eStage == StageFirst)
    {


        /* Desc Manager Declaration */

        cObjectPtr<IMediaDescriptionManager> pDescManager;
        RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));
        /* Input Pins */
        // Pin for Image receiving
        IMediaType* pIMediaTypeIn = new cMediaType(MEDIA_TYPE_NETWORK_DATA, MEDIA_SUBTYPE_NETWORK_DATA_IP);
        RETURN_IF_FAILED(m_oInputPin.Create("image", pIMediaTypeIn, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oInputPin));

        // Pin for Gesture output
        IMediaType* pIMediaTypeInG = new cMediaType(MEDIA_TYPE_NETWORK_DATA, MEDIA_SUBTYPE_NETWORK_DATA_IP);
        RETURN_IF_FAILED(m_oGesture.Create("Gesture", pIMediaTypeInG, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oGesture));

        /* Output Pins */
        //  pin for GESTURE_ID Output of Classifier
        tChar const * strDescSignalGESTURE_ID = pDescManager->GetMediaDescription("tSignalValue");
        RETURN_IF_POINTER_NULL(strDescSignalGESTURE_ID);
        cObjectPtr<IMediaType> pTypeSignalGESTURE_ID = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalGESTURE_ID, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(pTypeSignalGESTURE_ID->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignalInputGESTURE_ID));
        RETURN_IF_FAILED(m_oGESTURE_ID.Create("GESTURE_ID", pTypeSignalGESTURE_ID, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oGESTURE_ID));



        // Pin - Vector and position
        tChar const * strOutput = pDescManager->GetMediaDescription("tArrayStruct");
        RETURN_IF_POINTER_NULL(strOutput);
        cObjectPtr<IMediaType> pTypeOutput = new cMediaType(0, 0, 0, "tArrayStruct", strOutput, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(pTypeOutput->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignalIMAGE_ARRAY));
        RETURN_IF_FAILED(m_oIMAGE_ARRAY.Create("Vector", pTypeOutput, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oIMAGE_ARRAY));

        /* Trasmit Int Desc */
        tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");
        RETURN_IF_POINTER_NULL(strDescSignalValue);
        cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionFloat));

        tChar const * strDescBoolSignalValue = pDescManager->GetMediaDescription("tBoolSignalValue");
        RETURN_IF_POINTER_NULL(strDescSignalValue);
        cObjectPtr<IMediaType> pTypeBoolSignalValue = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescBoolSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

        RETURN_IF_FAILED(pTypeBoolSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionBool));





    }
    else if (eStage == StageNormal)
    {
        // In this stage you would do further initialisation and/or create your dynamic pins.
        // Please take a look at the demo_dynamicpin example for further reference.
    }
    else if (eStage == StageGraphReady)
    {
        // All pin connections have been established in this stage so you can query your pins
        // about their media types and additional meta data.
        // Please take a look at the demo_imageproc example for further reference.
        m_nRXBufferSize=0;
        m_pBuffer = new tChar[MAX_PACKET_SIZE];
        start_gesture=tFalse;
        m_nGesture=0;

        for(int i=0;i<256;i++)
        {
            char temp=i;
            values[i]=temp;
            //LOG_INFO(cString::Format("loop %d:%c",i,values[i]));
        }

        for(int j=0;j<784;j++)
        {
            Pixel[j]=0;
        }

    }

    RETURN_NOERROR;
}

tResult cUDPConnect::Shutdown(tInitStage eStage, __exception)
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

tResult cUDPConnect::OnPinEvent(IPin* pSource,
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
        if (pSource == &m_oInputPin)
        {
            Process(pSource, pMediaSample);
        }

        else if(pSource == &m_oGesture)
        {
            Process_Gesture(pSource,pMediaSample);
            //TransmitGestures();
        }
    }

    RETURN_NOERROR;
}

tResult cUDPConnect::Process(IPin* pSource, IMediaSample* pSample)
{
    RETURN_IF_POINTER_NULL(pSample);
    const tVoid* pSrcData;
    m_nRXBufferSize = UDP_DEFAULT_RXBUFFER_SIZE;

    if(IS_OK(pSample->Lock(&pSrcData)))
    {
        m_oRXBuffer.Alloc(m_nRXBufferSize);

        memcpy(m_oRXBuffer.GetPtr(), pSrcData, m_nRXBufferSize);

        char **sp =static_cast<char**>(m_oRXBuffer.GetPtr());

        for(int j=0;j<784;j++)
        {

            char* pass1=(char*) sp;
            cString c=cString::Format("%c",pass1[j]);

            for(int i=0;i<256;i++)
            {
                if(c==values[i])
                {
                    Pixel[j]=i;

                }
            }
        }
        pSample->Unlock(pSrcData);
        TransmitVectors(0);

    }

    RETURN_NOERROR;
}


tResult cUDPConnect::Process_Gesture(IPin* pSource, IMediaSample* pSample)
{
    RETURN_IF_POINTER_NULL(pSample);
    const tVoid* pSrcData;
    m_nBuffergest=UDP_DEFAULT_RXBUFFER_SIZE;
    if(IS_OK(pSample->Lock(&pSrcData)))
    {
        m_oRXGesture.Alloc(m_nBuffergest);
        memcpy(m_oRXGesture.GetPtr(), pSrcData, m_nBuffergest);
        char **sp =static_cast<char**>(m_oRXGesture.GetPtr());
        char* pass1=(char*) sp[0];
        cString c=cString::Format("%c",pass1);
        for(int i=0;i<256;i++)
        {
            if(c==values[i])
            {
                start_gesture=tTrue;
                m_nGesture=i;
                //LOG_INFO(cString::Format("Gestures value %d ",m_nGesture));
                TransmitIntValue(&m_oGESTURE_ID, m_nGesture, 0);
                TransmitBoolValue(&m_oGESTURE_ID, start_gesture, 0);

            }
        }
        pSample->Unlock(pSrcData);
    }

    RETURN_NOERROR;

}

tResult cUDPConnect::TransmitVectors(tUInt32 timestamp)
{
    // Transmit the vector and the current position
    // Create a new MediaSmaple


    // Map the array from int into float64
    tFloat64 f64Array[784];
    for(int i = 0; i<784; i++)
    {
        f64Array[i] = Pixel[i];//* 1.0;
        //LOG_INFO(cString::Format("Pixel value %d ",f64Array[i]));
    }

    //create new media sample
    cObjectPtr<IMediaSample> pMediaSampleArray;
    AllocMediaSample((tVoid**)&pMediaSampleArray);
    // Array out
    cObjectPtr<IMediaSerializer> pSerializerArray;
    m_pCoderDescSignalIMAGE_ARRAY->GetMediaSampleSerializer(&pSerializerArray);
    tInt nSizeArray = pSerializerArray->GetDeserializedSize();
    pMediaSampleArray->AllocBuffer(nSizeArray);
    cObjectPtr<IMediaCoder> pCoderOutputArray;
    m_pCoderDescSignalIMAGE_ARRAY->WriteLock(pMediaSampleArray, &pCoderOutputArray);
    pCoderOutputArray->Set("f64VecRGB", (tVoid*)&(f64Array));
    m_pCoderDescSignalIMAGE_ARRAY->Unlock(pCoderOutputArray);
    pMediaSampleArray->SetTime(_clock->GetStreamTime());
    m_oIMAGE_ARRAY.Transmit(pMediaSampleArray);

}
tResult cUDPConnect::TransmitIntValue(cOutputPin* oPin, tUInt32 value, tUInt32 timestamp)
{
    //use mutex
    __synchronized_obj(m_critSecTransmitControl);

    cObjectPtr<IMediaSample> pMediaSample;
    AllocMediaSample((tVoid**)&pMediaSample);

    cObjectPtr<IMediaSerializer> pSerializer;
    m_pCoderDescSignalInputGESTURE_ID->GetMediaSampleSerializer(&pSerializer);
    pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());

    static bool hasID = false;
    static tBufferID szIDValueOutput;
    static tBufferID szIDArduinoTimestampOutput;

    {
        __adtf_sample_write_lock_mediadescription(m_pCoderDescSignalInputGESTURE_ID, pMediaSample, pCoderOutput);

        if (!hasID)
        {
            pCoderOutput->GetID("ui32ArduinoTimestamp", szIDValueOutput);
            //pCoderOutput->GetID("ui32ArduinoTimestamp", szIDArduinoTimestampOutput);
            hasID = tTrue;
        }

        pCoderOutput->Set(szIDValueOutput, (tVoid*)&value);
        //pCoderOutput->Set(szIDArduinoTimestampOutput, (tVoid*)&timestamp);
    }

    pMediaSample->SetTime(_clock->GetStreamTime());

    oPin->Transmit(pMediaSample);

    RETURN_NOERROR;
}


tResult cUDPConnect::TransmitBoolValue(cOutputPin* oPin, bool value, tUInt32 timestamp)
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
    // LOG_INFO(cString::Format("LANE FOLLOW START %d", value));
    RETURN_NOERROR;
}


