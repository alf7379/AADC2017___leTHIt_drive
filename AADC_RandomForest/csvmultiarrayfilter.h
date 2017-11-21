/**
* Filter to predict the RF model output
* Author: Rakesh B Timmapur
*/

#ifndef _RANDOM_FOREST_PREDICTION_H
#define _RANDOM_FOREST_PREDICTION_H

#define MEDIA_TYPE_RF 0x101
#define MEDIA_SUBTYPE_RF 0x102
#define OID_ADTF_RF_PREDICTION "adtf.example.randomforestprediction"

int TREES;
int FEATURES;
int nSAMPLES;

#include "stdafx.h"
//#include "ADTF_OpenCV_helper.h"

class cDemoCSVMultiArrayFilter : public adtf::cFilter
{
	ADTF_DECLARE_FILTER_VERSION(OID_ADTF_RF_PREDICTION, "Random Forest Prediction Filter", OBJCAT_DataFilter,
		"RF_Prediction", 1, 0, 0, "")
protected: 

		cInputPin		m_oEnable;
        	cInputPin		m_oInputVector;
        	cOutputPin		m_oOutput;
		cOutputPin		m_oClassificationOutput;

   	        cVideoPin           m_oVideoOutputPin;

        cObjectPtr<IMediaTypeDescription> m_pCoderDescInputVector;
		cObjectPtr<IMediaTypeDescription> m_pCoderDescSignalInputEnable;
		cObjectPtr<IMediaTypeDescription>m_pCoderDescSignalOutputPrediction;
		cObjectPtr<IMediaTypeDescription>m_pCoderDescSignalClassificationOutput;
public: // overrides cFilter


	tBool enable;
	tBool enabled;
	tInt iTree;	


	tBool 		m_bTreesLoaded;

	tInt ct;

	tBool 		m_bIDsVectortSet;


	tBufferID 	m_szIDVector;
	tBufferID 	m_szIDPosition;


	cDemoCSVMultiArrayFilter(const tChar* __info);
	~cDemoCSVMultiArrayFilter();
	tResult Init(tInitStage eStage, __exception = NULL);
	tResult Shutdown(tInitStage eStage, __exception = NULL);
	tResult OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample);
	/**
	 * Function to read the .csv files
	  *
	   * @param strFile - input csv file
	    * @param oData - output 2D vector
		 *
		  * @return tResult
		   */
        tResult Read_CSV_File(const A_UTILS_NS::cString& strFile, std::vector<std::vector<tFloat64> >& oData);

	/**
	 * Function to predict the output value for a regression tree
	  *
	   * @param i_test_input - input test sample
	    * @param tree_matrix - input regression tree model
		 *
		  * @return double - predicted output estimate of one tree
		   */
        tFloat64 ProcessOneTree(std::vector<tFloat64>& i_test_input, std::vector<std::vector<tFloat64> >& tree_matrix);

	/**
	 * Function to get regression tree model
	  *
	   * @param i_idx_tree - input tree number
	    * @param o_Tree_Model - output regression tree model
		 *
		  * @return tResult
		   */
	tResult GetTreeModel(int& i_idx_tree, std::vector<std::vector<tFloat64> >& o_Tree_Model);

	/**
	 * Main function.
	  *
	   *
	    * @return tResult
		 */
	tResult ProcessInput(IMediaSample* pMediaSample);

	tResult PropertyChanged(const char* strProperty);
	tResult UpdateOutputImageFormat(const cv::Mat& outputImage);
protected:

	std::vector<std::vector<tFloat64> > Tree_Model;///< regression tree model is stored
	std::vector<std::vector<tFloat64> > Test_Samples;///<test samples are stored
	std::vector<tFloat64> Tree_Output;///<contains the predictions of all regression trees of one test sample
	std::vector<tFloat64> RF_Output;///<contains the RF model predictions for all test samples
};

#endif // _RANDOM_FOREST_PREDICTION_H
