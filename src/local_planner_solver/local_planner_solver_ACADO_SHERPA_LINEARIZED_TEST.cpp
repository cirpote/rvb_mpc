#include <acado_code_generation.hpp>
#include <acado/matrix_vector/vector.hpp>
#include <acado_toolkit.hpp>
#include <acado_gnuplot.hpp>
#include <acado/variables_grid/matrix_variable.hpp>
#include <iostream>
#include <fstream>

using namespace std;

int main( )
{   
      USING_NAMESPACE_ACADO

      const bool CODE_GEN = true;

      // System variables
      DifferentialState     eps1, eps2, eps3, eps4, eps5, eps6;
      Control               vi1, vi2;
      DifferentialEquation  f;
      Function              h, hN;

      // Parameters with exemplary values. These are set/overwritten at runtime.
      const double t_start = 0.0;     // Initial time [s]
      const double t_end = 10.0;       // Time horizon [s]
      const double dt = 1;          // Discretization time [s]
      const int N = round(t_end/dt);  // Number of nodes
      const double phi_max = 1.5;     // Maximal yaw rate [rad/s]
      const double v_max = 1.5;      // Maximal pitch and roll rate [rad/s]

      const double xObst = 0.5;
      const double yObst = 0.01;

      IntermediateState obstDist = ( eps1 - xObst ) * ( eps1 - xObst ) + ( eps4 - yObst ) * ( eps4 - yObst );
      IntermediateState theta = atan( eps5/eps2 );

      // System Dynamics
      f << dot(eps1) ==  eps2;
      f << dot(eps2) ==  eps3;
      f << dot(eps3) ==  vi1;
      f << dot(eps4) ==  eps5;
      f << dot(eps5) ==  eps6;
      f << dot(eps6) ==  vi2;
      // f << dot(eps7) == 2*( eps1 - xObst )*eps1 + 2*( eps4 - yObst )*eps4;



      // Cost: Sum(i=0, ..., N-1){h_i' * Q * h_i} + h_N' * Q_N * h_N
      // Running cost vector consists of all states and inputs.
      h << eps1 << eps2 << eps3 << eps4 << eps5 << eps6 << vi1 << vi2; // << theta;

      // End cost vector consists of all states (no inputs at last state).
      hN << eps1 << eps2 << eps4 << eps5;

      // Running cost weight matrix
      DMatrix Q(h.getDim(), h.getDim());
      Q.setIdentity();
      Q(0,0) = 100;   // x
      Q(1,1) = 100;   // y
      Q(2,2) = 100;   // theta
      Q(3,3) = 100;
      Q(4,4) = 100;
      Q(5,5) = 100;
      Q(6,6) = 100;
      Q(7,7) = 100;
      //Q(8,8) = 100;

      // End cost weight matrix
      DMatrix QN(hN.getDim(), hN.getDim());
      QN.setIdentity();
      QN(0,0) = 100;   // x
      QN(1,1) = 100;   // y
      QN(2,2) = 100;   // x
      QN(3,3) = 100;   // y

      // Set a reference for the analysis (if CODE_GEN is false).
      // Reference is at x = 2.0m in hover (qw = 1).
      DVector r(h.getDim());    // Running cost reference
      r.setZero();
      r(0) = 0;
      r(1) = 0;
      r(2) = 0;
      r(3) = 0;
      r(4) = -1;
      r(5) = 0;
      r(6) = 0;
      r(7) = 0;
      //r(8) = 1.57;

      DVector rN(hN.getDim());   // End cost reference
      rN.setZero();
      rN(0) = 0;
      rN(1) = 0;
      rN(2) = 0;
      rN(3) = -1;

      // DEFINE AN OPTIMAL CONTROL PROBLEM:
      // ----------------------------------
      OCP ocp( t_start, t_end, N );
      // For analysis, set references.
      ocp.minimizeLSQ( Q, h, r );
      ocp.minimizeLSQEndTerm( QN, hN, rN );


      // Add system dynamics
      ocp.subjectTo( f );

      // Add constraints on inputs
      ocp.subjectTo(-1 <= vi1 <= 1);
      ocp.subjectTo(-1 <= vi2 <= 1);

      // Add constraints on state variables
      ocp.subjectTo(-1 <= eps2 <= 1);
      ocp.subjectTo(-.5 <= eps3 <= .5);
      ocp.subjectTo(-1 <= eps5 <= 1);
      ocp.subjectTo(-.5 <= eps6 <= .5);
      //ocp.subjectTo(.25 <= obstDist <= 10000);

      // Set initial state
      ocp.subjectTo( AT_START, eps1 ==  0.0 );
      ocp.subjectTo( AT_START, eps2 ==  0.1 );
      ocp.subjectTo( AT_START, eps3 ==  0.0 );
      ocp.subjectTo( AT_START, eps4 ==  0.0 );
      ocp.subjectTo( AT_START, eps5 ==  0.0 );
      ocp.subjectTo( AT_START, eps6 ==  0.0 );

      // Setup some visualization
      GnuplotWindow window( PLOT_AT_EACH_ITERATION );
      window.addSubplot( eps1,"position x" );
      window.addSubplot( eps4,"position y" );
      //window.addSubplot( eps7,"distance_sum");
      window.addSubplot( vi1,"input1" );
      window.addSubplot( vi2,"input2");
      window.addSubplot( obstDist,"obstacle distance");
      window.addSubplot( theta + 3, "heading + 3 rads");

      GnuplotWindow window2( PLOT_AT_EACH_ITERATION );

      // Define an algorithm to solve it.
      OptimizationAlgorithm algorithm(ocp);
      algorithm.set( KKT_TOLERANCE, 1e-8 );
      algorithm.set( MAX_NUM_ITERATIONS, 10);
      algorithm.set( HESSIAN_APPROXIMATION, GAUSS_NEWTON);
      algorithm.set( LINEAR_ALGEBRA_SOLVER, GAUSS_LU);
      algorithm.set( LEVENBERG_MARQUARDT, 1e-8);
      algorithm.set( DISCRETIZATION_TYPE, MULTIPLE_SHOOTING);
      algorithm.set( INTEGRATOR_TYPE, INT_RK45);
      algorithm.set( INFEASIBLE_QP_HANDLING, IQH_RELAX_L1);
      algorithm << window;
      algorithm.solve();

      VariablesGrid x;
      algorithm.getDifferentialStates(x);

      ofstream logFile;
      logFile.open ("/home/ciro/RenzoLogFile.csv");
      for(unsigned int iter = 0; iter < N + 1; ++iter)
            logFile << x(iter,0) << ", " << x(iter,1) << ", " << x(iter,2) << ", " 
                    << x(iter,3) << ", " << x(iter,4) << ", " << x(iter,5) << "\n";
      logFile.close();

      return EXIT_SUCCESS;

}
