




	
#设定头文件目录


   

ADD_EXECUTABLE(VSlam VSlam.cpp)
TARGET_LINK_LIBRARIES( VSlam
    SlamBase
${OpenCV_LIBS} 
    ${PCL_LIBRARIES}
${catkin_LIBRARIES})
add_dependencies(VSlam ${catkin_EXPORTED_TARGETS})




