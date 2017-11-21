/**
* Filter to predict the RF model output
* Author: Rakesh B Timmapur
*/ 

#include "stdafx.h"
#include "csvmultiarrayfilter.h"
#include <fstream>

ADTF_FILTER_PLUGIN("Random Forest Prediction Filter", OID_ADTF_RF_PREDICTION, cDemoCSVMultiArrayFilter)
 
	cDemoCSVMultiArrayFilter::cDemoCSVMultiArrayFilter(const tChar* __info) : cFilter(__info) 
        {
	// Select the trained RF Model
        SetPropertyStr("RF_Model_Input", "");   //Initialized with empty field
	SetPropertyStr("RF_Model_Input" NSSUBPROP_DESCRIPTION, "CSV input file");
	SetPropertyBool("RF_Model_Input" NSSUBPROP_FILENAME, tTrue); 

	// Select the test sample file
	SetPropertyStr("Test_Sample_Input", "");//Initialized with empty field
	SetPropertyStr("Test_Sample_Input" NSSUBPROP_DESCRIPTION, "CSV input file");
	SetPropertyBool("Test_Sample_Input" NSSUBPROP_FILENAME, tTrue);

	// Select the number of trees
	SetPropertyInt("TREES", 100);
	SetPropertyBool("TREES" NSSUBPROP_ISCHANGEABLE, tTrue);

	// Select the number of features
	SetPropertyInt("FEATURES", 10); 
	SetPropertyBool("FEATURES" NSSUBPROP_ISCHANGEABLE, tTrue);
 
	// Select the number of test samples
	SetPropertyInt("nSAMPLES", 100);
	SetPropertyBool("nSAMPLES" NSSUBPROP_ISCHANGEABLE, tTrue);

	enable = tFalse;
	enabled = tFalse;

}

cDemoCSVMultiArrayFilter::~cDemoCSVMultiArrayFilter()
{
}

tResult cDemoCSVMultiArrayFilter::Init(tInitStage eStage, __exception)
{
	RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));

	if (eStage == StageFirst)
	{
		cObjectPtr<IMediaDescriptionManager> pDescManager;
		RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER, (tVoid**)&pDescManager, __exception_ptr));

				// Input - Vector and position
				tChar const * strInput = pDescManager->GetMediaDescription("tVectorStruct");
				RETURN_IF_POINTER_NULL(strInput);
				cObjectPtr<IMediaType> pTypeInput = new cMediaType(0, 0, 0, "tVectorStruct", strInput, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
				RETURN_IF_FAILED(pTypeInput->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescInputVector));
				RETURN_IF_FAILED(m_oInputVector.Create("Vector and Position", pTypeInput, static_cast<IPinEventSink*> (this)));
				RETURN_IF_FAILED(RegisterPin(&m_oInputVector));		

				/*
                                //Enable Input
                                tChar const * strDescInputVector = pDescManager->GetMediaDescription("tBoolSignalValue");
                                RETURN_IF_POINTER_NULL(strDescInputVector);
                                cObjectPtr<IMediaType> pTypeInputVector = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescInputVector, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
                                RETURN_IF_FAILED(pTypeInputVector->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescInputVector));
                                RETURN_IF_FAILED(m_oInputVector.Create("Input Vector", pTypeInputVector, static_cast<IPinEventSink*> (this)));
                                RETURN_IF_FAILED(RegisterPin(&m_oInputVector));
				*/

				//Output - Classification and position
                                tChar const * strOutput = pDescManager->GetMediaDescription("tClassificationResult");
				RETURN_IF_POINTER_NULL(strOutput);
                                cObjectPtr<IMediaType> pTypeOutput = new cMediaType(0, 0, 0, "tClassificationResult", strOutput, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
				RETURN_IF_FAILED(pTypeOutput->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignalClassificationOutput));
				RETURN_IF_FAILED(m_oClassificationOutput.Create("Classification and Position", pTypeOutput, static_cast<IPinEventSink*> (this)));
				RETURN_IF_FAILED(RegisterPin(&m_oClassificationOutput));	
				
				/*
				RETURN_IF_FAILED(m_oOutput.Create("RF Output", cObjectPtr<IMediaType>(new cMediaType(MEDIA_TYPE_RF, MEDIA_SUBTYPE_RF)),
				static_cast<IPinEventSink*>(this)));
				RETURN_IF_FAILED(RegisterPin(&m_oOutput));
				*/

                                // Video Output
                                RETURN_IF_FAILED(m_oVideoOutputPin.Create("Video_Output", IPin::PD_Output, static_cast<IPinEventSink*>(this)));
                                RETURN_IF_FAILED(RegisterPin(&m_oVideoOutputPin));


	}
	else if (eStage == StageNormal)
	{
                m_bIDsVectortSet = false;
                ct = 0;
                iTree = 0;

		TREES = GetPropertyInt("TREES");
		FEATURES = GetPropertyInt("FEATURES");
		nSAMPLES = GetPropertyInt("nSAMPLES");
		cFilename strCSV_test(GetPropertyStr("Test_Sample_Input"));
		ADTF_GET_CONFIG_FILENAME(strCSV_test);
                Read_CSV_File(strCSV_test, Test_Samples);
	}
	RETURN_NOERROR;
}

tResult cDemoCSVMultiArrayFilter::PropertyChanged(const char* strProperty)
{
	TREES=GetPropertyInt("TREES");
	FEATURES=GetPropertyInt("FEATURES");
	nSAMPLES=GetPropertyInt("nSAMPLES");

	RETURN_NOERROR;
}


tResult cDemoCSVMultiArrayFilter::Shutdown(tInitStage eStage, __exception)
{
	if (eStage == StageFirst)
	{
	}
	else if (eStage == StageNormal)
	{
	}
	else if (eStage == StageGraphReady)
	{
	}

	return cFilter::Shutdown(eStage, __exception_ptr);
}

tResult cDemoCSVMultiArrayFilter::OnPinEvent(IPin* pSource,
	tInt nEventCode,
	tInt nParam1,
	tInt nParam2,
	IMediaSample* pMediaSample)
{
	if (nEventCode == IPinEventSink::PE_MediaSampleReceived)
	{
                    //int timeStamp = 0;
            if (pSource == &m_oInputVector)
            {
                ProcessInput(pMediaSample);
            }
            RETURN_IF_POINTER_NULL(pMediaSample);
	}

	RETURN_NOERROR;
}

tResult cDemoCSVMultiArrayFilter::ProcessInput(IMediaSample* pMediaSample)
{


    // cv::Mat matRGBInput;

    // 92160 elements
    //Mat test;
    // test = Mat(1,92160,CV_8UC1);
   // test = Mat(92160,1,CV_8UI);


    // read-out the incoming Media Sample
    //get values from media sample
/*
    __adtf_sample_read_lock_mediadescription(m_pCoderDescInputVector,pMediaSample,pCoderInput);
    if(!m_bIDsVectortSet)
    {
           pCoderInput->GetID("i8VecRGB",m_szIDVector);
           pCoderInput->GetID("i8Position", m_szIDPosition);
           m_bIDsVectortSet = tTrue;
    }
    pCoderInput->Get("i8VecRGB", (tVoid*)&test.data);
    pCoderInput->Get("i8Position", (tVoid*)&i8PositionCol);

*/

    // tUInt8 i8InputAraryRGB[92160];
    tFloat64 f64InputArrayRGB[1800];
    std::vector<tFloat64> test_input;

    tInt16      i16PositionX = 0;
    tFloat32    f32Distance  = 0;
    tInt16      i16BoxWidth  = 0;
    tInt16      i16BoxHeight = 0;

    cObjectPtr<IMediaCoder> pCoderInput;
    RETURN_IF_FAILED(m_pCoderDescInputVector->Lock(pMediaSample, &pCoderInput));
    pCoderInput->Get("f64VecRGB", (tVoid*)&f64InputArrayRGB);
    pCoderInput->Get("i16PositionX", (tVoid*)&i16PositionX);
    pCoderInput->Get("f32Distance", (tVoid*)&f32Distance);
    pCoderInput->Get("i16BoxWidth", (tVoid*)&i16BoxWidth);
    pCoderInput->Get("i16BoxHeight", (tVoid*)&i16BoxHeight);
    m_pCoderDescInputVector->Unlock(pCoderInput);

    //LOG_INFO(cString::Format("i8InputAraryRGB: %i",i8InputAraryRGB[0]));


        // Revisualize the image to show the vector transmition is properly done
        for(tInt i =0; i<1800; i++){
           // f64InputArrayRGB[i] = i8InputAraryRGB[i];
            test_input.push_back(f64InputArrayRGB[i]);
        }

        /*
        string typeCSV = ".csv";
        string nameRGBCUT_VEC ="RecordingImages/RGB/image_RGB_vec_RF_";
        stringstream ssNameRGBCUT_CSV_VEC;
        ssNameRGBCUT_CSV_VEC<<nameRGBCUT_VEC<<("_")<<(ct+1)<<typeCSV;
        string filenameRGBCUTCSV_VEC = ssNameRGBCUT_CSV_VEC.str();


        ofstream myfile;
        myfile.open(filenameRGBCUTCSV_VEC.c_str());
        myfile<< cv::format(test_input, cv::Formatter::FMT_CSV) << std::endl;
        myfile.close();

        */


        /*
        // Remap the inut vector to an RGB image
            tInt index =0;
            cv::Mat test;
            test = Mat(120, 256,CV_8UC3);
            for(tInt i = 0;i<256;i++){
                for(tInt j=0; j<120; j++){
                    for(tInt rgb=0; rgb <3; rgb++){
                        test.at<cv::Vec3b>(i, j) [rgb] = i8InputAraryRGB[index];
                        LOG_INFO(cString::Format("test_input: %i",i8InputAraryRGB[index]));
                        index++;
                    }
                }
            }

            string typeJPG = ".jpg";
            stringstream ssNameRGBCUT;
                            string nameRGBCUT ="RecordingImages/RGB/image_RGB_";
                            ssNameRGBCUT<<nameRGBCUT<<("_")<<("_")<<typeJPG;
                            string filenameRGBCUT = ssNameRGBCUT.str();
                            imwrite(filenameRGBCUT, test);
        */


        // LOG_INFO(cString::Format("test_input: %f",test_input[5]));


        // nSAMPLES will be always 1 --> one image at a time
        // for (int idx_t=0; idx_t<nSAMPLES; ++idx_t)
            for (int idx_t=0; idx_t<1; ++idx_t)
            {
                    // We do not need this innput

                    //copy the test sample
           //         for (int col=0; col<FEATURES; ++col)
           //         {
           //                test_input.push_back(Test_Samples[idx_t][col]);
           //                vecRGBinput.push_back(Test_Samples[1][col]);
           //       }

                    // for the test_input calculate the output
                    iTree=0;
                    tFloat64 f64TreeOutput;
                    tInt iCounterChild = 0;
                    tInt iCounterAdult = 0;
                    tInt iCounterRotW = 0;
                    tFloat32 f32Case = 99;

                    //Tree_Output.clear();
                    for (int idx_tree=1; idx_tree<=TREES; ++idx_tree)
                    {
                            iTree = idx_tree;
                            GetTreeModel(idx_tree, Tree_Model);
                            f64TreeOutput = ProcessOneTree(test_input, Tree_Model);

                            //LOG_INFO(cString::Format("Tree number %i", idx_tree));

                            if((0.5 < f64TreeOutput) && (f64TreeOutput <= 1.5))
                            {
                                iCounterAdult++;
                                //LOG_INFO(cString::Format("Counter Child %i", iCounterChild));
                            }
                            else if((1.5 < f64TreeOutput) && (f64TreeOutput <= 2.5))
                            {
                                iCounterRotW++;
                                //LOG_INFO(cString::Format("Counter Adult %i", iCounterAdult));
                            }
                            /*
                            else if((2.5< f64TreeOutput) && (f64TreeOutput <= 3.5))
                            {
                                iCounterRotW++;
                                //LOG_INFO(cString::Format("Counter RotW %i", iCounterRotW));
                            }
                            */
                            else
                            {
                                //LOG_INFO(cString::Format("No counter"));
                            }

                            //Tree_Output.push_back(f64TreeOutput);
                    }

                    // Check what counter-value is the biggest
                    if((iCounterChild > iCounterAdult) && (iCounterChild > iCounterRotW))
                    {
                        f32Case = 1;
                        LOG_WARNING(cString::Format("Case Child: iCounterChild = %i, iCounterAdult = %i, iCounterRotW = %i", iCounterChild,iCounterAdult,iCounterRotW));
                    }
                    else if((iCounterAdult > iCounterChild) && (iCounterAdult > iCounterRotW))
                    {
                        f32Case = 2;
                        LOG_WARNING(cString::Format("Case Adult: iCounterChild = %i, iCounterAdult = %i, iCounterRotW = %i", iCounterChild,iCounterAdult,iCounterRotW));
                    }
                    else
                    {
                        f32Case = 3;
                        LOG_WARNING(cString::Format("Case RotW: iCounterChild = %i, iCounterAdult = %i, iCounterRotW = %i", iCounterChild,iCounterAdult,iCounterRotW));
                    }
                    LOG_INFO(cString::Format("i16PositionX = %i, f32Distance = %f, i16BoxWidth = %i, i16BoxWidth = %i", i16PositionX, f32Distance, i16BoxWidth, i16BoxWidth));

    /*
                    tFloat64 sum = 0;
                    int i1=0;



                    // double max = *max_element(Tree_Output.begin(),Tree_Output.end());
                    // double min = *min_element(Tree_Output.begin(),Tree_Output.end());
                    //cout<<"Max value: "<<max<<endl;
                    for (int idx = 0; idx<100; ++idx)
                    {
                            sum = sum + Tree_Output[i1];
                            i1++;
                            // LOG_INFO(cString::Format("+++ The output is: %f",Tree_Output[i1]));
                    }
                    // The auto type is not working, so I`ve created the upper for-loop
         //           for (auto idx = (Tree_Output.begin()); idx<Tree_Output.end(); ++idx)
         //           {
         //                   sum=sum+Tree_Output[i1++];
         //                   //LOG_INFO(cString::Format("The output is: %f",Tree_Output[i1]));
         //           }
                    tFloat64 output;
                    output=sum/TREES;
                    // LOG_INFO(cString::Format("The output of the Random Forest is: %f",output));


                    // This part is not required, because we only have one image
          //          RF_Output.push_back((sum/TREES));
                    Tree_Output.clear();
    */

                    test_input.clear();



                    // Transmit the output and the position
                    // Transmit the vector and the current position
                    // Create a new MediaSmaple
                    //tFloat32 output = 123;
                    cObjectPtr<IMediaSample> pMediaSampleOutput;
                    AllocMediaSample((tVoid**)&pMediaSampleOutput);
                    // Send the Media Sample
                    cObjectPtr<IMediaSerializer> pSerializerOutput;
                    m_pCoderDescSignalClassificationOutput->GetMediaSampleSerializer(&pSerializerOutput);
                    tInt nSizeOutput = pSerializerOutput->GetDeserializedSize();
                    pMediaSampleOutput->AllocBuffer(nSizeOutput);
                    cObjectPtr<IMediaCoder> pCoderOutputOutput;
                    m_pCoderDescSignalClassificationOutput->WriteLock(pMediaSampleOutput, &pCoderOutputOutput);
                    pCoderOutputOutput->Set("i16PositionX", (tVoid*)&(i16PositionX));
                    pCoderOutputOutput->Set("f32Distance", (tVoid*)&(f32Distance));
                    pCoderOutputOutput->Set("i16BoxWidth", (tVoid*)&(i16BoxWidth));
                    pCoderOutputOutput->Set("i16BoxHeight", (tVoid*)&(i16BoxHeight));
                    pCoderOutputOutput->Set("i16ClassificationResult", (tVoid*)&(f32Case));
                    m_pCoderDescSignalClassificationOutput->Unlock(pCoderOutputOutput);
                    pMediaSampleOutput->SetTime(_clock->GetStreamTime());
                    m_oClassificationOutput.Transmit(pMediaSampleOutput);

            }


   




        //transmission
        /*
        cObjectPtr<IMediaSample> mediaSample;

        if (IS_OK(AllocMediaSample(reinterpret_cast<tVoid**>(&mediaSample), OID_ADTF_MEDIA_SAMPLE)))
        {
                if (IS_OK(mediaSample->Update(_clock->GetStreamTime(),
                        &RF_Output,
                        sizeof(RF_Output),
                        IMediaSample::MSF_None)))
                {
                        m_oOutput.Transmit(mediaSample);
                }
        }
        */
	RETURN_NOERROR;
}

tResult cDemoCSVMultiArrayFilter::GetTreeModel(int& i_idx_tree, std::vector<std::vector<tFloat64> >& o_Tree_Model)
{
	cFilename strCSV(GetPropertyStr("RF_Model_Input"));
	ADTF_GET_CONFIG_FILENAME(strCSV);
	int pos = strCSV.GetLength()-15;
	int nlength_1 = 3; 
        // New part

	strCSV.Delete(pos, nlength_1);
	strCSV.Insert(cString::Format("%.3d",i_idx_tree), pos, nlength_1);

        // LOG_INFO(strCSV);
        Read_CSV_File(strCSV, o_Tree_Model);

        //LOG_INFO(cString::Format("o_Tree_Model %f",o_Tree_Model[0][0]));
	RETURN_NOERROR;
}

tResult cDemoCSVMultiArrayFilter::Read_CSV_File(const cString& strFile, std::vector<std::vector<tFloat64> >& oData)
{
	// open the input file
	cFile oFile;
	oData.clear();

	if (!oFile.Open(cFilename(strFile), cFile::OM_Read))
	{
		return ERR_INVALID_ARG;
	}


	// read every line
	while (!oFile.IsEof())
	{
		cString strLine;
		oFile.ReadLine(strLine);
		strLine.Trim();
		cStringList oElements;
		oElements.Clear();

		// split into columns

		strLine.Split(oElements, ','); //comma separated
		tInt nCount = oElements.GetItemCount();

		if (nCount < 1)
		{
			return ERR_INVALID_ARG;
		}

		if (oElements.GetItemCount() != nCount)
		{
			return ERR_INVALID_ARG;
		}

		// store line elements as floats into a vector ...
		std::vector<tFloat64> oLineElements;

		for (tInt i = 0; i < nCount; ++i)
		{
                        oLineElements.push_back(oElements.Get(i).AsFloat64());
		}

		// ... and put that vector into the two dimensional vector
                oData.push_back(oLineElements);
		oLineElements.clear();
	}

	return ERR_NOERROR;
}

tFloat64 cDemoCSVMultiArrayFilter::ProcessOneTree(std::vector<tFloat64>& i_test_input,
        std::vector<std::vector<tFloat64> >& tree_matrix)
{
	tFloat64 output = 0;
	int current_node, current_node_cutdim;
	current_node=1;
        // tInt index=0;

        // LOG_INFO(cString::Format("i_test_input[0]: %f",i_test_input[0]));
        // LOG_INFO(cString::Format("tree_matrix[0][4]: %f",tree_matrix[current_node-1][4]));

 /*
        if(iTree == 1)
        {


            string typeCSV = ".csv";
            string nameRGBCUT_VEC_while ="RecordingImages/RGB/image_RGB_vec_RF_while";
            stringstream ssNameRGBCUT_CSV_VEC_while;
            ssNameRGBCUT_CSV_VEC_while<<nameRGBCUT_VEC_while<<("_")<<(ct+1)<<("_")<<(iTree)<<typeCSV;
            string filenameRGBCUTCSV_VEC_while = ssNameRGBCUT_CSV_VEC_while.str();


            iTree++;
            ofstream myfile;
            myfile.open(filenameRGBCUTCSV_VEC_while.c_str());
            myfile<< cv::format(i_test_input, cv::Formatter::FMT_CSV) << std::endl;
            myfile.close();
        }
*/
	while (1)
	{
		current_node_cutdim = tree_matrix[current_node-1][4]; //5th column will alwas be feature index

                if (i_test_input[current_node_cutdim-1] < tree_matrix[current_node-1][5]) //threshold column
		{
                        current_node = tree_matrix[current_node-1][6]; //left child node
		}
		else
		{
                        current_node = tree_matrix[current_node-1][7]; //right child node
		}

		if (tree_matrix[current_node-1][0] == 1) //tree type, 1 is leaf node
		{
			output = tree_matrix[current_node-1][3]; //tree output
                        // LOG_WARNING(cString::Format("BREAK %f",output));
			break;
		}

	} //End of While loop

	return output;
}

/*

tResult cDemoCSVMultiArrayFilter::UpdateOutputImageFormat(const cv::Mat& outputImage)
{
    //check if pixelformat or size has changed
    if (tInt32(outputImage.total() * outputImage.elemSize()) != m_sOutputFormat.nSize)
    {
        Mat2BmpFormat(outputImage, m_sOutputFormat);

        LOG_INFO(adtf_util::cString::Format("Output: Size %d x %d ; BPL %d ; Size %d , PixelFormat; %d", m_sOutputFormat.nWidth, m_sOutputFormat.nHeight, m_sOutputFormat.nBytesPerLine, m_sOutputFormat.nSize, m_sOutputFormat.nPixelFormat));
        //set output format for output pin
        m_oVideoOutputPin.SetFormat(&m_sOutputFormat, NULL);
    }
    RETURN_NOERROR;
}
*/
