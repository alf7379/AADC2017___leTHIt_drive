cmake_minimum_required(VERSION 2.8.4 FATAL_ERROR)

cmake_policy(SET CMP0002 NEW)
cmake_policy(SET CMP0003 NEW)
cmake_policy(SET CMP0011 NEW)

# The project and cmake vars must only be defined in case that only the base
# filter should be build.
if(NOT AADC_BUILD_ALL_IN_ONE)
    # define a project name
    project(aadc_user)

    # CMAKE_BUILD_TYPE is empty on Unix systems, but needed by ADTF. Otherwise the ADTF libs
    # can't be found and linked.
    if(UNIX AND NOT CMAKE_BUILD_TYPE)
        set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING
                "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
                FORCE)
    endif(UNIX AND NOT CMAKE_BUILD_TYPE)
    

endif(NOT AADC_BUILD_ALL_IN_ONE)

find_package(ADTF REQUIRED)
#-------General------------

set(AADC_FOUND FALSE)
set(AADC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../)
find_package(AADC REQUIRED)
if (AADC_FOUND)
   message("-- AADC found")
else(AADC_FOUND)
   message(FATAL_ERROR "-- AADC not found")
endif (AADC_FOUND)

#--------------add private macros-----------------
include(${AADC_DIR}/AADC_PRIVATE.cmake)
include(${AADC_DIR}/AADCConfigVersion.cmake)

#--------------sources----------------------------
include_directories(${AADC_DIR}/include)

#add_subdirectory(AADC_Slam)
add_subdirectory(AADC_AEB)
add_subdirectory(AADC_Crossing)
add_subdirectory(AADC_LaneFollower_with_globalthreshold)
add_subdirectory(AADC_Parking)
add_subdirectory(AADC_Parkoutleft)
add_subdirectory(AADC_Parkoutright)
add_subdirectory(AADC_CrossLineOneSlot)
add_subdirectory(AADC_StopLineFinal)
add_subdirectory(AADC_Scanning)
add_subdirectory(AADC_RecordData)
add_subdirectory(AADC_SituationDetection)
add_subdirectory(AADC_Parking_detector)
add_subdirectory(AADC_WheelSpeedController)
add_subdirectory(AADC_Parkline)
add_subdirectory(AADC_TrafficSigndetect)
add_subdirectory(AADC_PedestrianDetection)
add_subdirectory(AADC_Globalthresh)
add_subdirectory(AADC_ACC_withdepth)
add_subdirectory(AADC_LaneChange)
add_subdirectory(AADC_Localization)
add_subdirectory(AADC_UDPConnect)
add_subdirectory(AADC_RoadSignBackend)
add_subdirectory(AADC_XMLWriter)
add_subdirectory(AADC_Trafficdetection)
add_subdirectory(AADC_DepthProcessing)
add_subdirectory(AADC_UDPReceiving)
add_subdirectory(demo_udp)

