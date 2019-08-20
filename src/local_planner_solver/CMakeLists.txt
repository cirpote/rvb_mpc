add_executable(local_planner_solver_ACADO_SHERPA_exec local_planner_solver_ACADO_SHERPA.cpp)
target_link_libraries(local_planner_solver_ACADO_SHERPA_exec ${ACADO_SHARED_LIBRARIES})
set_target_properties(local_planner_solver_ACADO_SHERPA_exec PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/src/local_planner_solver/")

add_custom_target(local_planner_solver_ACADO_SHERPA_run
     COMMAND local_planner_solver_ACADO_SHERPA_exec
     WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
     COMMENT "run generated local_planner_solver_ACADO_SHERPA in ${CMAKE_CURRENT_SOURCE_DIR}"
     VERBATIM
     SOURCES local_planner_solver_ACADO_SHERPA.cpp
)

add_library(local_planner_solver_lib 
        SHERPA_local_planner.cpp
		qpoases/SRC/Bounds.cpp
		qpoases/SRC/Constraints.cpp
		qpoases/SRC/CyclingManager.cpp
		qpoases/SRC/Indexlist.cpp
		qpoases/SRC/MessageHandling.cpp
		qpoases/SRC/QProblem.cpp
		qpoases/SRC/QProblemB.cpp
		qpoases/SRC/Utils.cpp
        qpoases/SRC/SubjectTo.cpp
		qpoases/SRC/EXTRAS/SolutionAnalysis.cpp
		core/acado_qpoases_interface.cpp
		core/acado_integrator.c
		core/acado_solver.c
		core/acado_auxiliary_functions.c
)

target_link_libraries(local_planner_solver_lib
    IBVS_base_controller_lib    
)

target_include_directories(local_planner_solver_lib PUBLIC
	core
 	qpoases
 	qpoases/INCLUDE
 	qpoases/SRC
)

if(GENERATE_ACADO_CPP)
    add_dependencies(local_planner_solver_lib local_planner_solver_ACADO_SHERPA_run)
endif(GENERATE_ACADO)

add_executable(local_planner_solver_ACADO_SHERPA_TEST_exec local_planner_solver_ACADO_SHERPA_TEST.cpp)
target_link_libraries(local_planner_solver_ACADO_SHERPA_TEST_exec ${ACADO_SHARED_LIBRARIES})
set_target_properties(local_planner_solver_ACADO_SHERPA_TEST_exec PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/src/local_planner_solver/")

add_executable(local_planner_solver_ACADO_SHERPA_LINEARIZED_TEST local_planner_solver_ACADO_SHERPA_LINEARIZED_TEST.cpp)
target_link_libraries(local_planner_solver_ACADO_SHERPA_LINEARIZED_TEST ${ACADO_SHARED_LIBRARIES})
set_target_properties(local_planner_solver_ACADO_SHERPA_LINEARIZED_TEST PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/src/local_planner_solver/")