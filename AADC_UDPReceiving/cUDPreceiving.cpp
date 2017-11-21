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
#include "cUDPreceiving.h"
#include "template_data.h"


/// Create filter shell
ADTF_FILTER_PLUGIN("User UDPreceiving", OID_ADTF_UDPRECEIVING, cUDPreceiving);


cUDPreceiving::cUDPreceiving(const tChar* __info):cFilter(__info)
{

}

cUDPreceiving::~cUDPreceiving()
{

}

tResult cUDPreceiving::Init(tInitStage eStage, __exception)
{
    // never miss calling the parent implementation!!
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))

    // in StageFirst you can create and register your static pins.
    if (eStage == StageFirst)
    {
        //cObjectPtr<IMediaDescriptionManager> pDescManager;
        //RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER, (tVoid**)&pDescManager, __exception_ptr));
        // get a media type for the input pin
        cObjectPtr<IMediaType> pInputType;
        RETURN_IF_FAILED(AllocMediaType(&pInputType, MEDIA_TYPE_TEMPLATE, MEDIA_SUBTYPE_TEMPLATE, __exception_ptr));

        // create and register the input pin
        RETURN_IF_FAILED(m_oInputPin.Create("input_template", pInputType, this));
        RETURN_IF_FAILED(RegisterPin(&m_oInputPin));

        /*
        //media to read the udp input
        tChar const * strInputUDPlist = pDescManager->GetMediaDescription("tManeuverList");
        RETURN_IF_POINTER_NULL(strInputUDPlist);
        cObjectPtr<IMediaType> pTypeInputUDPlist = new cMediaType(0, 0, 0, "tManeuverList", strInputUDPlist, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

        RETURN_IF_FAILED(pTypeInputUDPlist->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionInputUDPlist));
        RETURN_IF_FAILED(m_oInputUDPlist.Create("UDP_List", pTypeInputUDPlist, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oInputUDPlist));
        */
       IMediaType* pIMediaTypeIn = new cMediaType(MEDIA_TYPE_NETWORK_DATA, MEDIA_SUBTYPE_NETWORK_DATA_IP);
       RETURN_IF_FAILED(m_oUDPInputPin.Create("UDP_Input", pIMediaTypeIn, static_cast<IPinEventSink*> (this)));
       RETURN_IF_FAILED(RegisterPin(&m_oUDPInputPin));
      

        // get a media type for the output pin
        cObjectPtr<IMediaType> pOutputType;
        RETURN_IF_FAILED(AllocMediaType(&pOutputType, MEDIA_TYPE_TEMPLATE, MEDIA_SUBTYPE_TEMPLATE, __exception_ptr));

        // create and register the output pin
        RETURN_IF_FAILED(m_oOutputPin.Create("output_template", pOutputType, this));
        RETURN_IF_FAILED(RegisterPin(&m_oOutputPin));
    }
    else if (eStage == StageNormal)
    {
        // In this stage you would do further initialisation and/or create your dynamic pins.
        // Please take a look at the demo_dynamicpin example for further reference.

        m_strUDPreceivingstring = "";
    }
    else if (eStage == StageGraphReady)
    {
        // All pin connections have been established in this stage so you can query your pins
        // about their media types and additional meta data.
        // Please take a look at the demo_imageproc example for further reference.
    }

    RETURN_NOERROR;
}

tResult cUDPreceiving::Shutdown(tInitStage eStage, __exception)
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

tResult cUDPreceiving::OnPinEvent(IPin* pSource,
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
            // this will store the value for our new sample
            tTemplateData fNewValue;

            // now lets access the data in the sample,
            // the Lock method gives you access to the buffer of the sample.
            // we use a scoped sample lock to ensure that the lock is released in all circumstances.

            {
                // this will aquire the read lock on the sample and declare and initialize a pointer to the data
                __sample_read_lock(pMediaSample, tTemplateData, pData);
                // now we can access the sample data through the pointer
                fNewValue = *pData + 1.0;
                // the read lock on the sample will be released when leaving this scope
            }

            // now we need a new media sample to forward the data.
            cObjectPtr<IMediaSample> pNewSample;
            if (IS_OK(AllocMediaSample(&pNewSample)))
            {
                // now set its data
                // we reuse the timestamp from the incoming media sample. Please see the api documentation
                // (ADTF Extreme Programmers -> The ADTF Streamtime) for further reference on how sample times are handled in ADTF
                pNewSample->Update(pMediaSample->GetTime(), &fNewValue, sizeof(tTemplateData), 0);

                // and now we can transmit it
                m_oOutputPin.Transmit(pNewSample);
            }
        }
        
        else if(pSource == &m_oUDPInputPin)
        {
        /*
        // focus for sample read lock
                __adtf_sample_read_lock_mediadescription(m_pDescriptionInputUDPlist,pMediaSample,pCoder);
                std::vector<tSize> getUDPs;
                // retrieve number of elements by providing NULL as first paramter
                tSize szBufferSize = 0;
                if(IS_OK(pCoder->GetDynamicBufferIDs(NULL, szBufferSize)))
                {
                    // create a buffer depending on the size element
                    tChar* pcBuffer = new tChar[szBufferSize];
                    getUDPs.resize(szBufferSize);
                    // get the dynamic ids (we already got the first "static" size element)
                    if (IS_OK(pCoder->GetDynamicBufferIDs(&(getUDPs.front()), szBufferSize)))
                    {
                        // iterate over all elements
                        for (tUInt32 nIdx = 0; nIdx < getUDPs.size(); ++nIdx)
                        {
                            // get the value and put it into the buffer
                            pCoder->Get(getUDPs[nIdx], (tVoid*)&pcBuffer[nIdx]);
                        }
                        // set the resulting char buffer to the string object
                        m_strUDPreceivingstring = (const tChar*)pcBuffer;    //(const tChar*)
                        LOG_INFO(cString::Format("UDP receiving signal is %s", m_strUDPreceivingstring[0])); //.c_str()
                    }
                    delete pcBuffer;
                }
        */
        Process(pSource, pMediaSample);
        }
    }

    RETURN_NOERROR;
}

tResult cUDPreceiving::Process(IPin* pSource, IMediaSample* pMediaSample)
{

    if (this->GetState() != State_Running)
    {
        RETURN_ERROR(ERR_UNEXPECTED);
    }
    tInt nRXBufferRead = 0;
    tInt m_nRXBufferRead     = 0;
    const tVoid* pSrcData;
    RETURN_IF_FAILED(pMediaSample->Lock(&pSrcData));

    tDeviceConfiguration m_sEthernetDeviceConfig;
    tDeviceConfiguration*   m_psDeviceConfig;

    m_psDeviceConfig    = new tDeviceConfiguration;
    tTimeStamp m_tmReceiveDataTime = cHighResTimer::GetTime();
    *((tTimeStamp*) pvData) = m_tmReceiveDataTime;
    memset(m_psDeviceConfig, 0, sizeof(tDeviceConfiguration));
    memcpy(m_psDeviceConfig, pvData, sizeof(tDeviceConfiguration));
    m_nRXBufferSize = m_psDeviceConfig->ui32RxBufferSize;
    if (m_nRXBufferSize == 0)
    {
        m_nRXBufferSize = UDP_DEFAULT_RXBUFFER_SIZE;
    }
    m_oRXBuffer.Alloc(m_nRXBufferSize);
    m_oDataSocket.SetOption(SOL_SOCKET, SO_RCVBUF, &m_nRXBufferSize, sizeof(m_nRXBufferSize));
    if (m_psDeviceConfig->bRawMode)
            {
                if (IS_FAILED(m_oDataSocket.Read(m_oRXBuffer.GetPtr(), (tInt)m_oRXBuffer.GetSize(),
                                                 &m_nRXBufferRead)))
                {
                    m_nRXBufferRead = 0;
                    return 0;
                }
                else
                {
                    m_tmReceiveDataTime = cHighResTimer::GetTime();
                }

                return m_nRXBufferRead;
            }
    
    if (&m_oDataSocket != NULL)
    {
        // Raw mode - PacketSize is constant
        if (m_psDeviceConfig->bRawMode)
        {
            tInt nRXBufferRead = m_nRXBufferRead;
            if (nRXBufferRead <= nBufferSize)
            {
                memcpy(pSrcData, m_oRXBuffer.GetPtr(), nRXBufferRead);
                m_tmReceiveDataTime = cHighResTimer::GetTime();
                m_nRXBufferRead = 0;
                LOG_INFO(cString::Format("UDP receiving signal is %s", nRXBufferRead));
            }
            else
            {
                return -1;
            }
        }
    }
    
    pMediaSample->Unlock(pSrcData);
    RETURN_NOERROR;
}
