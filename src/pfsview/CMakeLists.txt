include_directories ("${PROJECT_BINARY_DIR}/" "${PROJECT_SOURCE_DIR}/src/pfs" ${Qt5Widgets_INCLUDE_DIRS})
if( NOT HAS_GETOPT )
	include_directories ("${GETOPT_INCLUDE}")
endif( NOT HAS_GETOPT )

#include_directories ("${PROJECT_BINARY_DIR}/" "${PROJECT_SOURCE_DIR}/src/pfs" ${Qt5Widgets_INCLUDE_DIRS})
link_directories("${PROJECT_SOURCE_DIR}/src/pfs")

SET(pfsview_SOURCES main.cpp pfsview_widget.cpp luminancerange_widget.cpp histogram.cpp resources.cpp)
SET(pfsview_HEADERS main.h pfsview_widget.h luminancerange_widget.h)

if (OPENMP_FOUND)
#    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

QT5_WRAP_CPP(pfsview_HEADERS_MOC ${pfsview_HEADERS})

QT5_WRAP_CPP(pfsview_HEADERS_MOC ${pfsview_HEADERS})

# Replace the tag with the path to bash
file(READ ${CMAKE_CURRENT_SOURCE_DIR}/pfsv.in file_content)
string(REGEX REPLACE "@BASH_PATH@" "${BASH_EXECUTABLE}" file_content "${file_content}")
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/pfsv" "${file_content}")	

install (FILES "${CMAKE_CURRENT_BINARY_DIR}/pfsv" 
         PERMISSIONS OWNER_EXECUTE GROUP_EXECUTE WORLD_EXECUTE OWNER_WRITE WORLD_READ GROUP_READ OWNER_READ
         DESTINATION bin)


ADD_EXECUTABLE(pfsview ${pfsview_SOURCES} 
    ${pfsview_HEADERS_MOC} "${GETOPT_OBJECT}")
TARGET_LINK_LIBRARIES(pfsview pfs Qt5::Widgets)

install (TARGETS pfsview DESTINATION bin)
install (FILES pfsview.1 DESTINATION ${MAN_DIR})
