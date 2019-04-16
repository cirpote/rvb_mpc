find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
include_directories( ${OPENGL_INCLUDE_DIRS}  ${GLUT_INCLUDE_DIRS} )

add_library(mav_imgui_library SHARED
  mav_imgui.h mav_imgui.cpp
  dynObstacle.h dynObstacle.cpp
)
target_link_libraries(mav_imgui_library
  dear_imgui
  yaml-cpp
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
  AprilTags_lib
  ${OPENGL_LIBRARIES}
  ${GLUT_LIBRARY}
  ${ASSIMP_LIBRARIES}
  ${GLM_LIBRARIES}
  )


add_subdirectory(imgui)
