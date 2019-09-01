#include <acado_code_generation.hpp>
#include <acado/matrix_vector/vector.hpp>
#include <acado_toolkit.hpp>
#include <acado_gnuplot.hpp>
#include <acado/variables_grid/matrix_variable.hpp>

using namespace std;

int main( )
{   

      /*  LINEARIZED CONTROLLER  */
      /*USING_NAMESPACE_ACADO

      DifferentialState eps1;
      DifferentialState eps2;
      DifferentialState eps3;
      DifferentialState eps4;
      DifferentialState dump;

      Control vi1;
      Control vi2;

      OnlineData xObst1;
      OnlineData yObst1;
      OnlineData xObst2;
      OnlineData yObst2;
      OnlineData xObst3;
      OnlineData yObst3;
      OnlineData xObst4;
      OnlineData yObst4;
      OnlineData xObst5;
      OnlineData yObst5;
      OnlineData xObst6;
      OnlineData yObst6;

      const double  Ts = 0.2;
      const int N = 40;

      DifferentialEquation f;
      f << dot(eps1) == eps2;
      f << dot(eps2) == vi1;
      f << dot(eps3) == eps4;
      f << dot(eps4) == vi2;
      f << dot(dump) == xObst1 + yObst1 + xObst2 + yObst2 + xObst3 + yObst3 + xObst4 + yObst4 + xObst5 + yObst5 + xObst6 + yObst6;

      IntermediateState obstDist1 = (xObst1 - eps1) * (xObst1 - eps1) + (yObst1 - eps3) * (yObst1 - eps3);
      IntermediateState obstDist2 = (xObst2 - eps1) * (xObst2 - eps1) + (yObst2 - eps3) * (yObst2 - eps3);
      IntermediateState obstDist3 = (xObst3 - eps1) * (xObst3 - eps1) + (yObst3 - eps3) * (yObst3 - eps3);
      IntermediateState obstDist4 = (xObst4 - eps1) * (xObst4 - eps1) + (yObst4 - eps3) * (yObst4 - eps3);
      IntermediateState obstDist5 = (xObst5 - eps1) * (xObst5 - eps1) + (yObst5 - eps3) * (yObst5 - eps3);
      IntermediateState obstDist6 = (xObst6 - eps1) * (xObst6 - eps1) + (yObst6 - eps3) * (yObst6 - eps3);

      Function h, hN;
      h <<  eps1 << eps3 << 
            1 / obstDist1 << 1 / obstDist2 << 1 / obstDist3 <<
            1 / obstDist4 << 1 / obstDist5 << 1 / obstDist6;
      hN << eps1 << eps2 << eps3 << eps4;

      BMatrix W = eye<bool>( h.getDim() );
      BMatrix WN = eye<bool>( hN.getDim() );

      OCP ocp(0.0, N*Ts, N);
      ocp.minimizeLSQ( W, h); 
      ocp.minimizeLSQEndTerm( WN, hN);

      ocp.subjectTo(1.25 <= obstDist1 <= 10000);
      ocp.subjectTo(1.25 <= obstDist2 <= 10000);
      ocp.subjectTo(1.25 <= obstDist3 <= 10000);
      ocp.subjectTo(1.25 <= obstDist4 <= 10000);
      ocp.subjectTo(1.25 <= obstDist5 <= 10000);
      ocp.subjectTo(1.25 <= obstDist6 <= 10000);

      ocp.subjectTo(-.5 <= vi1 <= .5);
      ocp.subjectTo(-.5 <= vi2 <= .5);

      ocp.setNOD(12);

      ocp.setModel(f);
      OCPexport mpc(ocp);

      mpc.set( HESSIAN_APPROXIMATION, GAUSS_NEWTON);
      mpc.set( DISCRETIZATION_TYPE, MULTIPLE_SHOOTING);
      mpc.set( SPARSE_QP_SOLUTION, FULL_CONDENSING_N2);
      mpc.set( IMPLICIT_INTEGRATOR_NUM_ITS, 2);
      mpc.set( LINEAR_ALGEBRA_SOLVER, GAUSS_LU);
      mpc.set( LEVENBERG_MARQUARDT, 1e-8);
      mpc.set( INTEGRATOR_TYPE, INT_RK78);
      mpc.set( NUM_INTEGRATOR_STEPS, N );
      mpc.set( QP_SOLVER, QP_QPOASES);
      mpc.set( CG_HARDCODE_CONSTRAINT_VALUES, NO);
      mpc.set( HOTSTART_QP, NO);
      mpc.set( FIX_INITIAL_STATE, BT_TRUE);

      mpc.printOptionsList();

      if (mpc.exportCode( "core" ) != SUCCESSFUL_RETURN)
            exit( EXIT_FAILURE );

      mpc.printDimensionsQP( );
    
      return EXIT_SUCCESS;*/





      /*  STANDARD NON LINEAR CONTROLLER  */
      // USING_NAMESPACE_ACADO

      USING_NAMESPACE_ACADO

      const bool CODE_GEN = true;

      // System variables
      DifferentialState     p_x, p_y, theta, dump;
      Control               v, phi;
      OnlineData xObst1, yObst1, xObst2, yObst2, xObst3, yObst3, xObst4, yObst4, xObst5, yObst5, xObst6, yObst6, l;
      DifferentialEquation  f;
      Function              h, hN;

      // Parameters with exemplary values. These are set/overwritten at runtime.
      const double t_start = 0.0;     // Initial time [s]
      const double t_end = 10.0;       // Time horizon [s]
      const double dt = 0.5;          // Discretization time [s]
      const int N = round(t_end/dt);  // Number of nodes

      // System Dynamics
      f << dot(p_x) ==  v * cos(theta);
      f << dot(p_y) ==  v * sin(theta);
      f << dot(theta) ==  (v / l) * tan(phi);
      f << dot(dump) == xObst1 + yObst1 + xObst2 + yObst2 + xObst3 + yObst3 +
                        xObst4 + yObst4 + xObst5 + yObst5 + xObst6 + yObst6;

      IntermediateState obstDist1 = ( p_x - xObst1 ) * ( p_x - xObst1 ) + ( p_y - yObst1 ) * ( p_y - yObst1 );
      IntermediateState obstDist2 = ( p_x - xObst2 ) * ( p_x - xObst2 ) + ( p_y - yObst2 ) * ( p_y - yObst2 );
      IntermediateState obstDist3 = ( p_x - xObst3 ) * ( p_x - xObst3 ) + ( p_y - yObst3 ) * ( p_y - yObst3 );
      IntermediateState obstDist4 = ( p_x - xObst4 ) * ( p_x - xObst4 ) + ( p_y - yObst4 ) * ( p_y - yObst4 );
      IntermediateState obstDist5 = ( p_x - xObst5 ) * ( p_x - xObst5 ) + ( p_y - yObst5 ) * ( p_y - yObst5 );
      IntermediateState obstDist6 = ( p_x - xObst6 ) * ( p_x - xObst6 ) + ( p_y - yObst6 ) * ( p_y - yObst6 );


      h << p_x << p_y << theta << 1 / obstDist1 << 1 / obstDist2
        << 1 / obstDist3 << 1 / obstDist4 << 1 / obstDist5 << 1 / obstDist6;

      // End cost vector consists of all states (no inputs at last state).
      hN << p_x << p_y << theta;

      BMatrix W = eye<bool>( h.getDim() );
      BMatrix WN = eye<bool>( hN.getDim() );

      OCP ocp(0, 15, 60);
      ocp.minimizeLSQ( W, h); 
      ocp.minimizeLSQEndTerm( WN, hN);
      ocp.subjectTo( f );

      // Add constraints
      ocp.subjectTo(-1.1 <=    phi    <= 1.1);
      ocp.subjectTo(-0.5 <=     v     <= 0.5);
      /*ocp.subjectTo(1    <= obstDist1 <= 10000);
      ocp.subjectTo(1    <= obstDist2 <= 10000);
      ocp.subjectTo(1    <= obstDist3 <= 10000);
      ocp.subjectTo(1    <= obstDist4 <= 10000);
      ocp.subjectTo(1    <= obstDist5 <= 10000);
      ocp.subjectTo(1    <= obstDist6 <= 10000);*/

      ocp.setNOD(13);
      OCPexport mpc(ocp);

      mpc.set( HESSIAN_APPROXIMATION, GAUSS_NEWTON);
      mpc.set( DISCRETIZATION_TYPE, MULTIPLE_SHOOTING);
      mpc.set( LINEAR_ALGEBRA_SOLVER, GAUSS_LU);
      mpc.set( LEVENBERG_MARQUARDT, 1e-8);
      mpc.set( INTEGRATOR_TYPE, INT_RK78);
      mpc.set( QP_SOLVER, QP_QPOASES);
      mpc.set( CG_HARDCODE_CONSTRAINT_VALUES, NO);
      mpc.set( HOTSTART_QP, NO);
      mpc.set( FIX_INITIAL_STATE, BT_TRUE);
      mpc.set( ABSOLUTE_TOLERANCE, 1e-2 ); 
      mpc.set( INTEGRATOR_TOLERANCE, 1e-2 );
      //mpc.set( INFEASIBLE_QP_HANDLING, IQH_RELAX_L2);

      mpc.printOptionsList();

      if (mpc.exportCode( "core" ) != SUCCESSFUL_RETURN)
            exit( EXIT_FAILURE );

      mpc.printDimensionsQP( );
    
      return EXIT_SUCCESS;
}
