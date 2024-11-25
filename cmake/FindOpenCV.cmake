# FindOpenCV.cmake
# Adapted for Android projects

# Assume the OpenCV Android SDK is installed at the specified location
# Find the OpenCV library
set(OpenCV_INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/Thirdparty/opencv/opencv480/android/sdk/native/jni/include)
set(OpenCV_LIB_DIR ${CMAKE_SOURCE_DIR}/Thirdparty/opencv/opencv480/android/sdk/native/libs/${ANDROID_ABI})
find_library(OpenCV_LIBRARY NAMES opencv_java4 PATHS ${OpenCV_LIB_DIR} NO_CMAKE_FIND_ROOT_PATH)
if(NOT OpenCV_LIBRARY)
    message(FATAL_ERROR "OpenCV library not found ${OpenCV_LIB_DIR} ${OpenCV_LIBRARY}")
else()
    message(STATUS "OpenCV library found at ${OpenCV_LIBRARY}")
    include_directories(${OpenCV_INCLUDE_DIRS})
    link_libraries(${OpenCV_LIBRARY})
endif()

# Handle the QUIETLY and REQUIRED arguments, and set OpenCV_FOUND to TRUE
# if all listed components are found.
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(OpenCV
    REQUIRED_VARS OpenCV_INCLUDE_DIRS OpenCV_LIBRARY
)