# add the include directory of the filter
include_directories(../include)
# declare our filter
set(FILTER_NAME user_UDPreceiving)

adtf_add_filter(${FILTER_NAME}
    stdafx.h
    cUDPreceiving.h
    cUDPreceiving.cpp
    template_data.h
)

adtf_set_folder(${FILTER_NAME} user_UDPreceiving) 

# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${CMAKE_INSTALL_BINARY})
