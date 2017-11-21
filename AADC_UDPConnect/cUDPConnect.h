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
#ifndef _TEMPLATE_FILTER_H_
#define _TEMPLATE_FILTER_H_


#define OID_ADTF_UDP "adtf.aadc.udp"

const tUInt32 UDP_DEFAULT_RXBUFFER_SIZE = 0xFFFF;
const tUInt32 check=0x30;
 static const tInt           MAX_PACKET_SIZE = 8129;


/*! @defgroup TemplateFilter
*  @{
*
*  \image html User_Template.PNG "Plugin Template Filter"
*
* This is a small template which can be used by the AADC teams for their own filter implementations.
* \b Dependencies \n
* This plugin needs the following libraries:
*
*
* <b> Filter Properties</b>
* <table>
* <tr><th>Property<th>Description<th>Default
* </table>
*
* <b> Output Pins</b>
* <table>
* <tr><th>Pin<th>Description<th>MajorType<th>SubType
* <tr><td>output_template<td>An example output pin<td>MEDIA_TYPE_TEMPLATE<td>MEDIA_TYPE_TEMPLATE
*</table>
*
* <b> Input Pins</b>
* <table>
* <tr><th>Pin<th>Description<th>MajorType<th>SubType
* <tr><td>input_template<td>An example input pin<td>MEDIA_TYPE_TEMPLATE<td>MEDIA_TYPE_TEMPLATE
* </table>
*
* <b>Plugin Details</b>
* <table>
* <tr><td>Path<td>src/aadcUser/AADC_TemplateFilter
* <tr><td>Filename<td>user_templateFilter.plb
* <tr><td>Version<td>1.0.0
* </table>
*
*
*/

//!  Template filter for AADC Teams
/*!
* This is a example filter for the AADC
*/
class cUDPConnect : public adtf::cFilter
{
    /*! set the filter ID and the version */
    ADTF_FILTER(OID_ADTF_UDP, "UDP V2", adtf::OBJCAT_DataFilter);

protected:
    /*! the input pin for template data */
    cInputPin    m_oInputPin;

    cInputPin	m_oGesture;
	
    /*! the output pin for template data */
    cOutputPin m_oGESTURE_ID;
    cObjectPtr<IMediaTypeDescription>    m_pCoderDescSignalInputGESTURE_ID;

    cOutputPin          m_oIMAGE_ARRAY;
    cObjectPtr<IMediaTypeDescription> m_pCoderDescSignalIMAGE_ARRAY;
   
public:
    /*! default constructor for template class
           \param __info   [in] This is the name of the filter instance.
    */
    cUDPConnect(const tChar* __info);
   /* Memory block for buffer */
    cMemoryBlock            m_oRXBuffer,m_oRXGesture;
    tUInt32                 m_nRXBufferSize; 
    tUInt32 		    m_nBuffergest;	
    tChar*                  m_pBuffer;
   /*Array for the ASCII Character value comparison */
    char values[256]; 
   
   /* Variables for image data storing*/
    int Pixel[784];
    int index;	

   /* Variables for gesture storing and sending */
   tUInt32	m_nGesture;
   tBool 	start_gesture;

   /*! The critical section transmit control */
    cCriticalSection m_critSecTransmitControl;

   /*! the media description for float values */
    cObjectPtr<IMediaTypeDescription> m_pDescriptionFloat;


	/*! The critical section transmit bool */
        cCriticalSection m_critSecTransmitBool;

        /*! the media description for bool values */
        cObjectPtr<IMediaTypeDescription> m_pDescriptionBool;
   

cString cStrTest;

    /*! default destructor */
    virtual ~cUDPConnect();

protected:
  
    tResult Init(tInitStage eStage, ucom::IException** __exception_ptr);

    tResult Shutdown(tInitStage eStage, ucom::IException** __exception_ptr = NULL);

    tResult OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample);
    
    tResult Process(IPin* pSource, IMediaSample* pMediaSample);
 
    tResult Process_Gesture(IPin* pSource, IMediaSample* pMediaSample);

    tResult TransmitVectors(tUInt32 timestamp);
    tResult TransmitIntValue(cOutputPin* oPin, tUInt32 value, tUInt32 timestamp);
    tResult TransmitBoolValue(cOutputPin* oPin, bool value, tUInt32 timestamp);
 
};

//*************************************************************************************************
#endif // _TEMPLATE_FILTER_H_

/*!
*@}
*/
