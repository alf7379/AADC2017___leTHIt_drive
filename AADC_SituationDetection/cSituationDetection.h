/**
Copyright (c)
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: ?This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.?
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS ?AS IS? AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. s


**********************************************************************
* $Author:: spiesra $  $Date:: 2017-05-12 09:34:53#$ $Rev:: 63109   $
**********************************************************************/
#ifndef _SITUATION_DETECTION_H_
#define _SITUATION_DETECTION_H_ 

#define OID_ADTF_SITUATION_DETECTION "adtf.aadc.SituationDetection"

#include "stdafx.h"
  
#include "aadc_juryEnums.h"

// #include "displaywidget.h"




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
class cSituationDetection : public adtf::cFilter
{
    /*! set the filter ID and the version */
    ADTF_FILTER(OID_ADTF_SITUATION_DETECTION, "SituationDetection", adtf::OBJCAT_DataFilter);

protected:
    /*! the INPUT pin for template data */
    // Input JuryStruct
    cInputPin   m_oInputJuryStruct;
    cObjectPtr<IMediaTypeDescription> m_pDescriptionInputJuryStruct;

    // Inpu ManeuverList
    cInputPin   m_oInputManeuverList;
    cObjectPtr<IMediaTypeDescription> m_pDescriptionInputManeuverList;

    // Input TrafficSign
    cInputPin    m_oInputTrafficSign;
    cObjectPtr<IMediaTypeDescription> m_pDescriptionInputTrafficSign;

    // Input StopLine Detection
    cInputPin    m_oInputStoplineDetection;
    cObjectPtr<IMediaTypeDescription> m_pDescriptionInputStoplineDetection;

    // Input Edge Detection
    cInputPin    m_oInputEdgelineDetection;
    cObjectPtr<IMediaTypeDescription> m_pDescriptionInputEdgelineDetection;

    // Input Edge Detection
    cInputPin    m_oInputEdgeDetection;
    cObjectPtr<IMediaTypeDescription> m_pDescriptionInputEdgeDetection;

    // Input ObjectDetection
    cInputPin   m_oInputObjectDetection;
    cObjectPtr<IMediaTypeDescription> m_pDescriptionInputObjectDetection;

    // Input ACC
    cInputPin   m_oInputACC;
    cObjectPtr<IMediaTypeDescription> m_pDescriptionInputACC;

    // Input LaneChange
    cInputPin   m_oInputLaneChangeFinished;
    cObjectPtr<IMediaTypeDescription> m_pDescriptionInputLaneChangeFinished;

    // Input ScanningFinshed
    cInputPin  	 m_oInputScanningFinished;
    cObjectPtr<IMediaTypeDescription> m_pDescriptionInputScanningFinished;

    // Input ParkingFinshed
    cInputPin  	 m_oInputParkingFinished;
    cObjectPtr<IMediaTypeDescription> m_pDescriptionInputParkingFinished;

    // Input PullOutLeftFinished
    cInputPin  	 m_oInputPullOutLeftFinished;
    cObjectPtr<IMediaTypeDescription> m_pDescriptionInputPullOutLeftFinished;

    // Input PullOutRightFinished
    cInputPin  	 m_oInputPullOutRightFinished;
    cObjectPtr<IMediaTypeDescription> m_pDescriptionInputPullOutRightFinished;

    // Input CrossingFinshed
    cInputPin  	 m_oInputCrossingFinished;
    cObjectPtr<IMediaTypeDescription> m_pDescriptionInputCrossingFinished;

    // Input ZebracrossingFinished
    cInputPin  	 m_oInputZebracrossingFinished;
    cObjectPtr<IMediaTypeDescription> m_pDescriptionInputZebracrossingFinished;

    /*! the OUTPUT pin for template data */

    // Output Driver Struct
    cOutputPin   m_oOutputDriverStruct;
    cObjectPtr<IMediaTypeDescription> m_pDescriptionOutputDriverStruct;

    // Output RoI
    cOutputPin   m_oOutputRoI;
    cObjectPtr<IMediaTypeDescription> m_pDescriptionOutputRoI;

    // Output LaneFollowerStart
    cOutputPin   m_oOutputLaneFollowerStart;
    cObjectPtr<IMediaTypeDescription> m_pDescriptionOutputLaneFollowerStart;

    // Output LaneFollowerStart
    cOutputPin   m_oOutputRoadSignDetected;
    cObjectPtr<IMediaTypeDescription> m_pDescriptionOutputRoadSignDetected;

    // Output LaneChangeStart
    cOutputPin   m_oOutputLaneChangeStart;
    cObjectPtr<IMediaTypeDescription> m_pDescriptionOutputLaneChangeStart;

    // Output ScanningStart
    cOutputPin    m_oOutputScanningStart;
    cObjectPtr<IMediaTypeDescription> m_pDescriptionOutputScanningStart;

    // Output ParkingStart
    cOutputPin    m_oOutputParkingStart;
    cObjectPtr<IMediaTypeDescription> m_pDescriptionOutputParkingStart;

    // Output PullOutLeft
    cOutputPin    m_oOutputPullOutLeftStart;
    cObjectPtr<IMediaTypeDescription> m_pDescriptionOutputPullOutLeftStart;

    // Output PullOutRight
    cOutputPin    m_oOutputPullOutRightStart;
    cObjectPtr<IMediaTypeDescription> m_pDescriptionOutputPullOutRightStart;

    // Output Crossing
    cOutputPin    m_oOutputCrossingStart;
    cObjectPtr<IMediaTypeDescription> m_pDescriptionOutputCrossingStart;

    // Output Crossing
    cOutputPin    m_oOutputZebracrossingStart;
    cObjectPtr<IMediaTypeDescription> m_pDescriptionOutputZebracrossingStart;


    // Variables

    // jury Struct
    /*! the id for the i8StateID of the media description */
    tBufferID m_szIDJuryStructI8ActionID;
    /*! the id for the i16ManeuverEntry of the media description data */
    tBufferID m_szIDJuryStructI16ManeuverEntry;
    /*! indicates if bufferIDs were set */
    tBool m_bIDsJuryStructSet;


    // ManeuverList
    /*! The maneuver file string */
    cString     m_strManeuverFileString;
    /*! this is the filename of the maneuver list*/
    //cFilename   m_maneuverListFile;
    /*! this is the list with all the loaded sections and single maneuvers from the maneuver list*/
    std::vector<tSector> m_sectorList;

    tInt32 m_iArrayNumberOfManeuversInSerctor[99];
    tInt32 m_iAmountOfSectors;
    tInt32 m_iSectorIDCounter;
    tInt32 m_iManeuverIDOverallCounter;
    tInt32 m_iManeuverIDSectorCounter;
    tBool m_bManeuverListProcessed;



    // traffic sign
    tFloat32  m_fTrafficSignImageSize;
    tInt16    m_iCurrentTrafficSignID;
    tInt16    m_iPreviousTrafficSignID;
    tBufferID m_szIDRoadSignI16Identifier;
    tBufferID m_szIDRoadSignF32Imagesize;
    tBool     m_bIDsRoadSignSet;
    tInt16    m_iRoadSignDetectorCounter;
    tBool     m_bManeuverInProcess;
    tBool     m_bParkingSignDetected;
    tBool     m_bParkingSpotAhead;
    tInt      m_iParkingSignTriggerCounter;
    tBool     m_bSignalParkingSign;


    // Input from StoplineDetection
    tBool       m_bInputStoplineDetected;
    tFloat32    m_fInputStoplineDistance;

    // Input from Edgeline Detection
    tBool       m_bInputEdgeLineDetected;
    tFloat32    m_fInputEdgeLineDistance;

    // Input from EdgeDetection
    tBool       m_bInputEdge1Detected;
    tFloat32    m_fInputEdge1Distance;
    tBool       m_bInputEdge2Detected;
    tFloat32    m_fInputEdge2Distance;

    // Processing Edge and Stopline
    tFloat32 m_fTriggerDistance2Edge1;
    tFloat32 m_fTriggerDistance2Edge2;
    tFloat32 m_fTriggerDistance2Stopline;
    tFloat32 m_fTriggerDistance2Edgeline;
    tFloat32 m_fTriggerDistance2RoadSign;

    tBool    m_bEnableStopline4TriggeringCrossing;
    tBool    m_bEnableEdge14TriggeringCrossing;
    tBool    m_bEnableEdge24TriggeringCrossing;
    tBool    m_bEnableEdgeline4TriggeringCrossing;
    tBool    m_bEnableRoadSign4TriggeringCrossing;

    // Input from ACC
    tBool m_bInputACCLaneChangeRequested;
    // Input_Feedback from Parking
    tBool m_bInputParkingFinished;
    tTimeStamp timestamp;
    tTimeStamp tsCrossingFinished;
    tTimeStamp tsLastTimeRoadSignDetected;

    // Input_Feedback from PullOutLeftFinished
    tBool m_bInputPullOutLeftFinished;

    // Input_Feedback from PullOutRightFinished
    tBool m_bInputPullOutRightFinished;

    // Input_Feedback from Crossing
    tBool m_bInputCrossingFinished;


    // Driver Struct

    tBufferID m_szIDRoadSignTVec;
    tBufferID m_szIDRoadSignRVec;
    /*! the id for the i8StateID of the media description */
    tBufferID m_szIDDriverStructI8StateID;
    /*! the id for the i16ManeuverEntry of the media description data */
    tBufferID m_szIDDriverStructI16ManeuverEntry;
    /*! indicates if bufferIDs were set */
    tBool m_bIDsDriverStructSet;

    tBool m_bCarStateRunning;
 
    tFloat32 m_f32RoadSignDistanceY;
    tFloat32 m_f32RoadSignDistanceX;
    tFloat32 m_f32ArrayRoadSignDistanceY[3];
    tFloat32 m_f32ArrayRoadSignDistanceX[3];


    tInt  m_iStartCrossingCase;
    tBool m_bRoadSignEvaluated;

    // Output - order for Lane Follower
    tBool m_bOutputLaneFollowerStart;

    // Output - order for Parking
    tBool m_bOutputParkingStart;

    // Output - order for Pull Out Left
    tBool m_bOutputPullOutLeftStart;

    // Output - order for Pull Out Right
    tBool m_bOutputPullOutRightStart;

    // Output order for Crossing
    tBool m_bOutputCrossingStart;

    /*! Property - whether output to console is enabled or not*/
    tBool     m_bDebugModeEnabled;

    /*! Property - Number of road signs that need to be detected before it is processed*/
    tInt16    m_iRoadSignCounterMax;


    // Testing Properties for Crossing
    tBool m_bTestCrossingEnabled;
    tInt16 m_iTestCrossingManeuverID;

    // Testing Properties for Pull Out
    tBool m_bTestPullOutEnabled;
    tInt16 m_iTestPullOutDirectionID;


    // Testing Properties for Parking
    tBool m_bTestParkingEnabled;

    // Testing Property for enable bool ManeuverInProcess Flag
    tBool m_bEnableManeuverInProcessFlag;


public:
    /*! default constructor for template class
           \param __info   [in] This is the name of the filter instance.
    */
    cSituationDetection(const tChar* __info);

    /*! default destructor */
    virtual ~cSituationDetection();

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
    tResult EvaluateRoadSign(tInt8 i8RoadSignID);
    tResult Command2LaneFollower(tBool bLaneFollowerEnabled);
    tResult StartCrossing(tBool bIntersectionWithoutRoadSign, tInt8 i8RoadSignID);
    tResult SendRoI2StopLineDetection();
    tResult SendDriverStruct(stateCar stateID, tInt16 i16ManeuverEntry);
    tResult LoadManeuverList();
    tResult FinishManeuver(cString strFinishedManeuver);
    tResult StartPullOut();
    tResult StartParking(tInt8 i8ParkingID);
    tResult StartScanning();
    tResult StopVehicle();
    tResult EvaluateLaneChange();
    tResult StartZebraCrossing();
    tResult RoadSignDetected(tBool bRoadSignDetected);
    tResult PropertyChanged(const char* strProperty);
    tResult SendOutputs(tBool bLaneFollowerStart, tBool bCrossingStart, tInt iCrossingManeuverID, tInt iTrafficSignID);
};

//*************************************************************************************************
#endif // _SITUATION_DETECTION_H_

/*!
*@}
*/
