



ADD_LIBRARY(SlamBase SlamBase.cpp)
TARGET_LINK_LIBRARIES( SlamBase
    ${OpenCV_LIBS} 
    ${PCL_LIBRARIES}
)


