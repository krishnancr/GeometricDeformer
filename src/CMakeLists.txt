set(SOURCE_FILES
    "pluginMain.cpp"
	"geometricDeformer.h"
	"geometricDeformer.cpp"

)

find_package(Maya REQUIRED)

include_directories(${MAYA_INCLUDE_DIR})

link_directories(${MAYA_LIBRARY_DIR})
# I know this is terrible. I'll fix this soon
link_directories("D:/Autodesk/Maya2017/lib")

add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES})

target_link_libraries(${PROJECT_NAME} ${MAYA_LIBRARIES})

MAYA_PLUGIN(${PROJECT_NAME})

install( TARGETS ${PROJECT_NAME} ${MAYA_TARGET_TYPE} DESTINATION plugins)