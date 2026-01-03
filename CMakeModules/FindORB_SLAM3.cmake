# FindORB_SLAM3.cmake

set(ORB_SLAM3_ROOT_DIR "$ENV{ORB_SLAM3_ROOT_PATH}/ORB-SLAM3")

# Find ORB_SLAM3 library
find_library(ORB_SLAM3_LIBRARY
    NAMES ORB_SLAM3
    PATHS ${ORB_SLAM3_ROOT_DIR}/lib
    NO_DEFAULT_PATH
)

# Find ORB_SLAM3 include directory
find_path(ORB_SLAM3_INCLUDE_DIR
    NAMES System.h
    PATHS ${ORB_SLAM3_ROOT_DIR}/include
    NO_DEFAULT_PATH
)

# Find DBoW2 library
find_library(DBoW2_LIBRARY
    NAMES DBoW2
    PATHS ${ORB_SLAM3_ROOT_DIR}/Thirdparty/DBoW2/lib
    NO_DEFAULT_PATH
)

# DBoW2 headers are in the source directory (no separate include dir)
set(DBoW2_INCLUDE_DIR ${ORB_SLAM3_ROOT_DIR}/Thirdparty/DBoW2)

# Find g2o library
find_library(g2o_LIBRARY
    NAMES g2o
    PATHS ${ORB_SLAM3_ROOT_DIR}/Thirdparty/g2o/lib
    NO_DEFAULT_PATH
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(ORB_SLAM3
    REQUIRED_VARS 
        ORB_SLAM3_LIBRARY 
        ORB_SLAM3_INCLUDE_DIR 
        DBoW2_INCLUDE_DIR 
        DBoW2_LIBRARY 
        g2o_LIBRARY
)

if(ORB_SLAM3_FOUND)
    set(ORB_SLAM3_INCLUDE_DIRS 
        ${ORB_SLAM3_INCLUDE_DIR}
        ${ORB_SLAM3_ROOT_DIR}/Thirdparty/DBoW2
        ${ORB_SLAM3_ROOT_DIR}/Thirdparty/g2o
    )
    set(ORB_SLAM3_LIBRARIES 
        ${ORB_SLAM3_LIBRARY}
        ${DBoW2_LIBRARY}
        ${g2o_LIBRARY}
    )
endif()

mark_as_advanced(
    ORB_SLAM3_INCLUDE_DIR
    ORB_SLAM3_LIBRARY
    DBoW2_INCLUDE_DIR
    DBoW2_LIBRARY
    g2o_LIBRARY
)
