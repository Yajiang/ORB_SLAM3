# FindOpenCV.cmake
# Adapted for Android projects

# Assume the OpenCV Android SDK is installed at the specified location
# Find the OpenCV library
set(BOOST_INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/Thirdparty/boost_lib/boostLib")
set(BOOST_LIB_DIR "${CMAKE_SOURCE_DIR}/Thirdparty/boost_lib/${ANDROID_ABI}")
find_library(BOOST_SERIALIZATION NAMES boost_serialization PATHS ${BOOST_LIB_DIR} NO_CMAKE_FIND_ROOT_PATH)
include_directories(${BOOST_INCLUDE_DIRS})
link_libraries(${BOOST_SERIALIZATION})

# Handle the QUIETLY and REQUIRED arguments, and set OpenCV_FOUND to TRUE
# if all listed components are found.
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Boost
    REQUIRED_VARS BOOST_INCLUDE_DIRS BOOST_LIB_DIR
)