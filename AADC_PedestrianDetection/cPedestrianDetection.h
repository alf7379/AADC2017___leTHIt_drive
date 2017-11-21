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
#ifndef _PEDESTRIANDETECTION_HEADER_
#define _PEDESTRIANDETECTION_HEADER_

#define OID_ADTF_PEDESTRIAN_DETECTION "adtf.example.PedestrianDetection"


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
class cPedestrianDetection : public adtf::cFilter
{
    /*! set the filter ID and the version */
    ADTF_FILTER(OID_ADTF_PEDESTRIAN_DETECTION, "Pedestrian Detection", adtf::OBJCAT_DataFilter);

protected:
    /*! the input pin for template data */
    cInputPin    m_oInputSteeringAngle;
    cInputPin    m_oInputSpeedController;
    cInputPin    m_oInputRoadSignExt;
    cInputPin    m_oInputZebracrossingStart;
    cInputPin    m_oInputStoplineDetection;
    cInputPin    m_oInputClassificationResult;
    cInputPin    m_oInputDistanceoverall;
    /*! the output pin for template data */
    cOutputPin   m_oOutputSpeedController;
    cOutputPin   m_oOutputSteeringAngle;
    cOutputPin   m_oOutputBrakeLights;
    cOutputPin   m_oInputZebracrossingFinished;



    cObjectPtr<IMediaTypeDescription> m_pDescriptionInputZebracrossingStart;
    cObjectPtr<IMediaTypeDescription> m_pDescriptionInputSteeringAngle;
    cObjectPtr<IMediaTypeDescription> m_pDescriptionInputRoadSignExt;
    cObjectPtr<IMediaTypeDescription> m_pDescriptionInputSpeedController;
    cObjectPtr<IMediaTypeDescription> m_pDescriptionInputStoplineDetection;
    cObjectPtr<IMediaTypeDescription> m_pDescriptionInputClassificationResult;
    cObjectPtr<IMediaTypeDescription> m_pDescriptionInputDistanceoverall;


    cObjectPtr<IMediaTypeDescription> m_pDescriptionOutputSpeedController;
    cObjectPtr<IMediaTypeDescription> m_pDescriptionOutputSteeringAngle;
    cObjectPtr<IMediaTypeDescription> m_pDescriptionOutputBrakeLights;
    cObjectPtr<IMediaTypeDescription> m_pDescriptionInputZebracrossingFinished;


    // Memeber variables

    // States variables
    tInt 	m_iClassificationEvaluation_State;
    tInt 	m_iProcessSituation_State;
    tInt	m_iZebracrossing_State;

    tBool       m_bStateFirstAccess;
    tBool       m_f32Distance_EnteredState;

    tFloat32    m_f32Distance2RoadSign;
    tInt16      m_i16ZebraCrossingSignCounter;
    tBool       m_bZebraSignDetected;
    // NORMAL

    // ZEBRACROSSING

        // DRIVING_2_ZEBRA
        tBool       m_bDRIVING_2_ZEBRA_enter;
        tFloat32    m_f32Distance_Zebra_start;

        tFloat32    m_f32ZebracrossingStopDistance;

        // WAITING_AT_ZEBRA
        tTimeStamp  m_tsStateFirstAccess;
        tFloat32    m_f32MaxTime2Wait;

        // PASS_ZEBRA

    // CHILD
    tBool       m_bCHILD_enter;
    tFloat32    m_f32Distance_Child_Detected;



    tFloat32    m_f32ClassificationInformation [50][5];
    // EV_NORMAL

    // EV_ZEBRA

    // EV_WAITING
    tBool       m_bPedestrianMovmentDetected;
    tBool       m_bPedestrianPassed;



    tFloat32    m_f32InputSteeringAngle;
    tFloat32    m_f32InputVelocity;
    tBool       m_bZebracrossingSign;
    tBool       m_bStoplineDetected;
    tFloat32    m_f32StoplineDistance;
    tInt        m_iZebraPedestrianStage;
    tFloat32    m_f32InputDistanceOverall;


    tFloat32    m_f32StartDistanceOverall;
    tFloat32    m_f32StoplineDistanceOverall;

    tBool       m_bBrakeLightsOn;
    tTimeStamp  m_tsBrakeLights;

    tBool       m_bPedestrianDetected;

    tBool       m_bPedestrianMoving;
    tTimeStamp  m_tsPedestrianMovingTrue;
    tInt16      m_iArrayCurrentPosition[3][9];
    tInt16      m_iArrayPreviousPosition[3][9];
    tInt8       m_i8CrossingCounter;

    tBool       m_bEnteredLeft;
    tBool       m_bEnteredStreetLeft;
    tBool       m_bEnteredStreetCenter;
    tBool       m_bEnteredStreetRight;
    tBool       m_bEnteredRight;

    // Property
    tFloat32    m_f32VelocityZebracrossing;

    // Test Property
    tBool       m_bTestPedestrianDetected;
    tBool       m_bDebugOutputEnabled;

public:
    /*! default constructor for template class
           \param __info   [in] This is the name of the filter instance.
    */
    cPedestrianDetection(const tChar* __info);

    /*! default destructor */
    virtual ~cPedestrianDetection();

protected:
    /*! Implements the default cFilter state machine call. It will be
    *	    called automatically by changing the filters state and needs
    *	    to be overwritten by the special filter.
    *    Please see page_filter_life_cycle for further information on when the state of a filter changes.
    *
    *    \param [in,out] __exception_ptr   An Exception pointer where exceptions will be put when failed.
    *        If not using the cException smart pointer, the interface has to
    *        be released by calling Unref().
    *    \param  [in] eStage The Init function will be called when the filter state changes as follows:\n
    *    \return Standard Result Code.
    */
    tResult Init(tInitStage eStage, ucom::IException** __exception_ptr);

    /*!
    *   Implements the default cFilter state machine call. It will be
    *   called automatically by changing the filters state and needs
    *   to be overwritten by the special filter.
    *   Please see page_filter_life_cycle for further information on when the state of a filter changes.
    *
    *   \param [in,out] __exception_ptr   An Exception pointer where exceptions will be put when failed.
    *                                   If not using the cException smart pointer, the interface has to
    *                                   be released by calling Unref().
    *   \param  [in] eStage The Init function will be called when the filter state changes as follows:\n   *
    *   \return Returns a standard result code.
    *
    */
    tResult Shutdown(tInitStage eStage, ucom::IException** __exception_ptr = NULL);

    /*! This Function will be called by all pins the filter is registered to.
    *   \param [in] pSource Pointer to the sending pin's IPin interface.
    *   \param [in] nEventCode Event code. For allowed values see IPinEventSink::tPinEventCode
    *   \param [in] nParam1 Optional integer parameter.
    *   \param [in] nParam2 Optional integer parameter.
    *   \param [in] pMediaSample Address of an IMediaSample interface pointers.
    *   \return   Returns a standard result code.
    *   \warning This function will not implement a thread-safe synchronization between the calls from different sources.
    *   You need to synchronize this call by your own. Have a look to adtf_util::__synchronized , adtf_util::__synchronized_obj .
    */
    tResult OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample);

    tResult EvaluateClassification(tInt16 i16PositionX, tFloat32 f32Distance, tInt16 i16ClassificationResult);

    tResult ProcessSituation();

    tResult evaluateSituation();
    tResult SendingOutput(tFloat32 f32OutputVelocity, tFloat32 f32OutputSteeringAngle);
    tResult toggleBrakeLights();
    tResult evaluatePedestrian();
    tResult sendFinishFlag();
};

//*************************************************************************************************
#endif // _PEDESTRIANDETECTION_HEADER_

/*!
*@}
*/
