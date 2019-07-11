#include <acado_code_generation.hpp>
#include <acado/matrix_vector/vector.hpp>
#include <acado_toolkit.hpp>
#include <acado_gnuplot.hpp>
#include <acado/variables_grid/matrix_variable.hpp>

using namespace std;

int main( )
{   
      USING_NAMESPACE_ACADO

      const bool CODE_GEN = true;

      // System variables
      DifferentialState     p_x, p_y, theta;
      Control               v, omega;
      DifferentialEquation  f;
      Function              h, hN;

      // Parameters with exemplary values. These are set/overwritten at runtime.
      const double t_start = 0.0;     // Initial time [s]
      const double t_end = 4.0;       // Time horizon [s]
      const double dt = 0.1;          // Discretization time [s]
      const int N = round(t_end/dt);  // Number of nodes
      const double omega_max = 1.5;     // Maximal yaw rate [rad/s]
      const double v_max = 1;      // Maximal pitch and roll rate [rad/s]

      // System Dynamics
      f << dot(p_x) ==  v * cos(theta);
      f << dot(p_y) ==  v * sin(theta);
      f << dot(theta) ==  omega;

      // Cost: Sum(i=0, ..., N-1){h_i' * Q * h_i} + h_N' * Q_N * h_N
      // Running cost vector consists of all states and inputs.
      h << p_x << p_y << theta << v << omega;

      // End cost vector consists of all states (no inputs at last state).
      hN << p_x << p_y << theta;

      // Running cost weight matrix
      DMatrix Q(h.getDim(), h.getDim());
      Q.setIdentity();
      Q(0,0) = 2000;   // x
      Q(1,1) = 2000;   // y
      Q(2,2) = 500;   // theta
      Q(3,3) = 10;
      Q(4,4) = 10;

      // End cost weight matrix
      DMatrix QN(hN.getDim(), hN.getDim());
      QN.setIdentity();
      QN(0,0) = Q(0,0);   // x
      QN(1,1) = Q(1,1);   // y
      QN(2,2) = Q(2,2);   // theta

      // Set a reference for the analysis (if CODE_GEN is false).
      // Reference is at x = 2.0m in hover (qw = 1).
      DVector r(h.getDim());    // Running cost reference
      r.setZero();
      r(0) = 1.5;
      r(1) = 1.5;
      r(2) = 1;

      DVector rN(hN.getDim());   // End cost reference
      rN.setZero();
      rN(0) = r(0);
      rN(1) = r(1);
      rN(2) = r(2);

      // DEFINE AN OPTIMAL CONTROL PROBLEM:
      // ----------------------------------
      OCP ocp( t_start, t_end, N );
      // For analysis, set references.
      ocp.minimizeLSQ( Q, h, r );
      ocp.minimizeLSQEndTerm( QN, hN, rN );


      // Add system dynamics
      ocp.subjectTo( f );

      // Add constraints
      ocp.subjectTo(-omega_max <= v <= omega_max);
      ocp.subjectTo(-v_max <= omega <= v_max);
      // ocp.setNOD(10);

      // Set initial state
      ocp.subjectTo( AT_START, p_x ==  0.0 );
      ocp.subjectTo( AT_START, p_y ==  0.0 );
      ocp.subjectTo( AT_START, theta ==  0.0 );

      // Setup some visualization
      GnuplotWindow window1( PLOT_AT_EACH_ITERATION );
      window1.addSubplot( p_x,"position x" );
      window1.addSubplot( p_y,"position y" );
      window1.addSubplot( theta,"theta" );

      GnuplotWindow window2( PLOT_AT_EACH_ITERATION );
      window2.addSubplot( v,"velocity x" );
      window2.addSubplot( omega,"omega" );

      // Define an algorithm to solve it.
      OptimizationAlgorithm algorithm(ocp);
      algorithm.set( INTEGRATOR_TOLERANCE, 1e-6 );
      algorithm.set( KKT_TOLERANCE, 1e-3 );
      algorithm.set( MAX_NUM_ITERATIONS, 50);
      algorithm.set( HESSIAN_APPROXIMATION, GAUSS_NEWTON);
      algorithm.set( LINEAR_ALGEBRA_SOLVER, GAUSS_LU);
      algorithm.set( LEVENBERG_MARQUARDT, 1e-8);
      algorithm << window1;
      algorithm << window2;
      algorithm.solve();

      return EXIT_SUCCESS;
}
