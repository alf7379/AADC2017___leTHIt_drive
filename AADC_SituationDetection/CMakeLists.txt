# add the include directory of the filter
#include_directories(../include)
# declare our filter
set(FILTER_NAME user_SituationDetection)

adtf_add_filter(${FILTER_NAME}
    stdafx.h
    cSituationDetection.h
    cSituationDetection.cpp
)
# Folder for the storage with using Visual Studio
adtf_set_folder(${FILTER_NAME} SituationDetection) 

# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${CMAKE_INSTALL_BINARY})




#set(FILTER_NAME user_SituationDetection)

#adtf_use_qt(QtCore QtGui)

##we build this example only if qt was found
#if(QT_FOUND)

#    set(HEADER_MOC
#        displaywidget.h
#        cSituationDetection.h
#        )
#    
#    adtf_add_moc(${FILTER_NAME}
#                ${HEADER_MOC}
#    )
#    
#    adtf_add_filter(${FILTER_NAME}
#                    stdafx.h
#                    displaywidget.cpp
#                    cSituationDetection.cpp
#                    ${HEADER_MOC}
#    )
#
#    
#    # set folder (only needed and available for MSVC)
#    adtf_set_folder(${FILTER_NAME} SituationDetection) 
#    
#    adtf_install_plugin(${FILTER_NAME} ${CMAKE_INSTALL_BINARY})
#    
#endif(QT_FOUND)
