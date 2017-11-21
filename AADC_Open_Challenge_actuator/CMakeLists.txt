# add the include directory of the filter
include_directories(../include)
# declare our filter
set(FILTER_NAME user_Oactuators)

adtf_add_filter(${FILTER_NAME}
    stdafx.h
    cOactuators.h
    cOactuators.cpp
)

adtf_set_folder(${FILTER_NAME} Oactuators) 

# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${CMAKE_INSTALL_BINARY})
#adtf_install_plugin(${FILTER_NAME} ${AADC_INSTALL_DIR})
