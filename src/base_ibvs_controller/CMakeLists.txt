add_library(IBVS_base_controller_lib base_IBVS_controller.cpp)
target_link_libraries(IBVS_base_controller_lib ${ACADO_SHARED_LIBRARIES} ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
