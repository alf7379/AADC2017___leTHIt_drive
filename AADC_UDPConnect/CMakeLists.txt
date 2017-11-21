# add the include directory of the filter
include_directories(../include)
# declare our filter
set(FILTER_NAME UDP_V2)

adtf_add_filter(${FILTER_NAME}
    stdafx.h
    cUDPConnect.h
    cUDPConnect.cpp
)

adtf_set_folder(${FILTER_NAME} template) 

# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${CMAKE_INSTALL_BINARY})
