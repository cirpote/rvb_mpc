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
      DifferentialState     p_x, p_y, theta;//, phi, d;
      Control               v, phi;//phi_dot;
      DifferentialEquation  f;
      Function              h, hN;

      // Parameters with exemplary values. These are set/overwritten at runtime.
      const double t_start = 0.0;     // Initial time [s]
      const double t_end = 12.0;       // Time horizon [s]
      const double dt = 0.5;          // Discretization time [s]
      const int N = round(t_end/dt);  // Number of nodes
      const double phi_max = 1;     // Maximal yaw rate [rad/s]
      const double v_max = 0.8;      // Maximal pitch and roll rate [rad/s]
      const double l = 1.33;

      const double xObst1 = 2.f;
      const double yObst1 = 0.01;

      const double xObst2 = 2.f;
      const double yObst2 = 3.0;

      const double xObst3 = 2.f;
      const double yObst3 = 6.0;

      const double xObst4 = 5.f;
      const double yObst4 = 0.01;

      const double xObst5 = 5.f;
      const double yObst5 = 3.0;

      const double xObst6 = 8.f;
      const double yObst6 = 6.0;

      IntermediateState obstDist1 = ( p_x - xObst1 ) * ( p_x - xObst1 ) + ( p_y - yObst1 ) * ( p_y - yObst1 );
      IntermediateState obstDist2 = ( p_x - xObst2 ) * ( p_x - xObst2 ) + ( p_y - yObst2 ) * ( p_y - yObst2 );
      IntermediateState obstDist3 = ( p_x - xObst3 ) * ( p_x - xObst3 ) + ( p_y - yObst3 ) * ( p_y - yObst3 );
      IntermediateState obstDist4 = ( p_x - xObst4 ) * ( p_x - xObst4 ) + ( p_y - yObst4 ) * ( p_y - yObst4 );
      IntermediateState obstDist5 = ( p_x - xObst5 ) * ( p_x - xObst5 ) + ( p_y - yObst5 ) * ( p_y - yObst5 );
      IntermediateState obstDist6 = ( p_x - xObst6 ) * ( p_x - xObst6 ) + ( p_y - yObst6 ) * ( p_y - yObst6 );

      // System Dynamics
      f << dot(p_x) ==  v * cos(theta);
      f << dot(p_y) ==  v * sin(theta);
      f << dot(theta) ==  (v / l) * tan(phi);

      // Cost: Sum(i=0, ..., N-1){h_i' * Q * h_i} + h_N' * Q_N * h_N
      // Running cost vector consists of all states and inputs.
      h << p_x << p_y << theta << 1 / obstDist1 << 1 / obstDist2
        << 1 / obstDist3 << 1 / obstDist4 << 1 / obstDist5 << 1 / obstDist6;

      // End cost vector consists of all states (no inputs at last state).
      hN << p_x << p_y << theta;

      // Running cost weight matrix
      DMatrix Q(h.getDim(), h.getDim());
      Q.setIdentity();
      Q(0,0) = 300;   // x
      Q(1,1) = 300;   // y
      Q(2,2) = 300;   // theta
      Q(3,3) = 100;   // Obst1
      Q(4,4) = 100;   // Obst2
      Q(5,5) = 100;   // Obst3
      Q(6,6) = 100;   // Obst4
      Q(7,7) = 100;   // Obst5
      Q(8,8) = 100;   // Obst6

      // End cost weight matrix
      DMatrix QN(hN.getDim(), hN.getDim());
      QN.setIdentity();
      QN(0,0) = Q(0,0)*10;   // x
      QN(1,1) = Q(1,1)*10;   // y
      QN(2,2) = Q(2,2)*10;   // theta

      // Set a reference for the analysis (if CODE_GEN is false).
      // Referes at x = 2.0m in hover (qw = 1).
      DVector r(h.getDim());
      r.setZero();
      r(0) = 3;
      r(1) = 5;
      r(2) = 1.57;

      DVector rN(hN.getDim());   // End cost reference
      rN.setZero();
      rN(0) = r(0);
      rN(1) = r(1);
      rN(2) = r(2);

      // DEFINE AN OPTIMAL CONTROL PROBLEM:
      // ----------------------------------
      OCP ocp( 0, 0.25*40, 40 );
      // For analysis, set references.
      ocp.minimizeLSQ( Q, h, r );
      ocp.minimizeLSQEndTerm( QN, hN, rN );


      // Add system dynamics
      ocp.subjectTo( f );

      // Add constraints
      ocp.subjectTo(-phi_max <= phi <= phi_max);
      ocp.subjectTo(-v_max <= v <= v_max);
      ocp.subjectTo(1 <= obstDist1 <= 10000);
      ocp.subjectTo(1 <= obstDist2 <= 10000);
      ocp.subjectTo(1 <= obstDist3 <= 10000);
      ocp.subjectTo(1 <= obstDist4 <= 10000);
      ocp.subjectTo(1 <= obstDist5 <= 10000);
      ocp.subjectTo(1 <= obstDist6 <= 10000);

      // Set initial state
      ocp.subjectTo( AT_START, p_x ==  0.0 );
      ocp.subjectTo( AT_START, p_y ==  0.0 );
      ocp.subjectTo( AT_START, theta ==  0.0 );

      // ocp.subjectTo( AT_END, p_x ==  1.0 );
      // ocp.subjectTo( AT_END, p_y ==  1.0 );
      // ocp.subjectTo( AT_END, theta ==  1.0 );

      // Setup some visualization
      GnuplotWindow window( PLOT_AT_EACH_ITERATION );
      window.addSubplot( p_x,"position x" );
      window.addSubplot( p_y,"position y" );
      window.addSubplot( theta,"theta" );
      //window.addSubplot( v,"velocity");
      //window.addSubplot( phi,"steering angle");
      window.addSubplot( obstDist1, "obstacle 1 distance");
      window.addSubplot( obstDist2, "obstacle 2 distance");
      window.addSubplot( obstDist3, "obstacle 3 distance");
      window.addSubplot( obstDist4, "obstacle 4 distance");
      window.addSubplot( obstDist5, "obstacle 5 distance");
      window.addSubplot( obstDist6, "obstacle 6 distance");

      // Define an algorithm to solve it.
      OptimizationAlgorithm algorithm(ocp);
      //algorithm.set( INTEGRATOR_TOLERANCE, 1e-3 );
      algorithm.set( KKT_TOLERANCE, 1e-8 );
      algorithm.set( MAX_NUM_ITERATIONS, 30);
      algorithm.set( HESSIAN_APPROXIMATION, GAUSS_NEWTON);
      algorithm.set( LINEAR_ALGEBRA_SOLVER, GAUSS_LU);
      algorithm.set( LEVENBERG_MARQUARDT, 1e-8);
      algorithm.set( DISCRETIZATION_TYPE, MULTIPLE_SHOOTING);
      algorithm.set( INTEGRATOR_TYPE, INT_RK45);
      algorithm.set( ABSOLUTE_TOLERANCE, 1e-2 ); 
      algorithm.set( INTEGRATOR_TOLERANCE, 1e-2 );
      // algorithm.set( INFEASIBLE_QP_HANDLING, IQH_RELAX_L1);

      algorithm << window;
      algorithm.solve();
      
      VariablesGrid states, parameters, controls;
    
      algorithm.getDifferentialStates(states    );
      algorithm.getParameters        (parameters);
      algorithm.getControls          (controls  );
      
      states.print();
      parameters.print();
      controls.print();

      return EXIT_SUCCESS;
}
