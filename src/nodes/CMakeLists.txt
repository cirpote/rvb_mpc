add_library(object_spawner_lib randomObjectSpawner.cpp)
target_link_libraries(object_spawner_lib short_term_solver_lib  )


add_executable(IBVS_controller_node IBVS_controller_node.cpp )
add_dependencies(IBVS_controller_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(IBVS_controller_node short_term_solver_lib mav_imgui_library  )

add_executable(IBVS_random_controller_node IBVS_random_controller_node.cpp )
add_dependencies(IBVS_random_controller_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(IBVS_random_controller_node object_spawner_lib mav_imgui_library  )

add_executable(assimp_node assimp_node.cpp )
target_link_libraries(assimp_node dear_imgui mav_imgui_library ${ASSIMP_LIBRARIES} ${GLM_LIBRARIES} ${OpenCV_LIBS})
