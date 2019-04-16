#!/bin/bash  

sed -i 's/acadoVariables/shortTermAcadoVariables/g' core/acado_auxiliary_functions.c
sed -i 's/acadoVariables/shortTermAcadoVariables/g' core/acado_auxiliary_functions.h
sed -i 's/acadoVariables/shortTermAcadoVariables/g' core/acado_common.h
sed -i 's/acadoVariables/shortTermAcadoVariables/g' core/acado_integrator.c
sed -i 's/acadoVariables/shortTermAcadoVariables/g' core/acado_qpoases_interface.cpp
sed -i 's/acadoVariables/shortTermAcadoVariables/g' core/acado_qpoases_interface.hpp
sed -i 's/acadoVariables/shortTermAcadoVariables/g' core/acado_solver.c
sed -i 's/acadoVariables/shortTermAcadoVariables/g' core/test.c

sed -i 's/acadoWorkspace/shortTermAcadoWorkspace/g' core/acado_auxiliary_functions.c
sed -i 's/acadoWorkspace/shortTermAcadoWorkspace/g' core/acado_auxiliary_functions.h
sed -i 's/acadoWorkspace/shortTermAcadoWorkspace/g' core/acado_common.h
sed -i 's/acadoWorkspace/shortTermAcadoWorkspace/g' core/acado_integrator.c
sed -i 's/acadoWorkspace/shortTermAcadoWorkspace/g' core/acado_qpoases_interface.cpp
sed -i 's/acadoWorkspace/shortTermAcadoWorkspace/g' core/acado_qpoases_interface.hpp
sed -i 's/acadoWorkspace/shortTermAcadoWorkspace/g' core/acado_solver.c
sed -i 's/acadoWorkspace/shortTermAcadoWorkspace/g' core/test.c
