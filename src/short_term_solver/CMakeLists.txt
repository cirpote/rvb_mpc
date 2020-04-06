add_executable(short_term_solver_ACADO short_term_solver_ACADO.cpp)
target_link_libraries(short_term_solver_ACADO ${ACADO_SHARED_LIBRARIES})
set_target_properties(short_term_solver_ACADO PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/src/short_term_solver/")

add_custom_target(short_term_solver_ACADO_run
     COMMAND short_term_solver_ACADO
     WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
     COMMENT "run generated short_term_solver_ACADO in ${CMAKE_CURRENT_SOURCE_DIR}"
     VERBATIM
     SOURCES short_term_solver_ACADO.cpp
)

add_library(short_term_solver_lib 
        STIVS_controller.cpp
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

target_link_libraries(short_term_solver_lib
    IBVS_base_controller_lib    
)

target_include_directories(short_term_solver_lib PUBLIC
	core
 	qpoases
 	qpoases/INCLUDE
 	qpoases/SRC
)

if(GENERATE_ACADO_CPP)
    add_dependencies(short_term_solver_lib short_term_solver_ACADO_run)
endif(GENERATE_ACADO)

add_executable(ACADO_test short_term_solver_ACADO_test.cpp)
target_link_libraries(ACADO_test ${ACADO_SHARED_LIBRARIES})
set_target_properties(ACADO_test PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/src/short_term_solver/")