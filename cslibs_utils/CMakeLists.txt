cmake_minimum_required(VERSION 2.8.3)
project(cslibs_utils)

add_definitions(-W -Wall -Wno-unused-parameter -fno-strict-aliasing -Wno-unused-function -Wno-deprecated-register)

find_package(catkin REQUIRED COMPONENTS tf)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    DEPENDS Eigen3 OpenCV
)

find_package(Boost COMPONENTS system filesystem REQUIRED)
include_directories(SYSTEM ${Boost_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)

find_package(OpenCV REQUIRED)



include_directories(include)
include_directories(SYSTEM
    ${EIGEN3_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
)

add_library (${PROJECT_NAME} SHARED
    src/CircleEstimator.cpp
    src/ConfigFileReader.cpp
    src/EggTimer.cpp
    src/EigenTools.cpp
    src/Graphics.cpp
    src/LogCollector.cpp
    src/LowPassFilter.cpp
    src/MathHelper.cpp
    src/Misc.cpp
    src/Stopwatch.cpp
    src/Strings.cpp
    src/RamaxxException.cpp
    src/CvMathHelper.cpp
    src/CvContourManipulator.cpp
    src/LaserTo3DTransformation.cpp
    src/Path.cpp
    src/runningstat.cpp
    src/Line2d.cpp
    src/Line2dSegment.cpp
)

target_link_libraries(${PROJECT_NAME}
    ${OpenCV_LIBS}
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
