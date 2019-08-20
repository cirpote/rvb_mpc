#include <acado_code_generation.hpp>
#include <acado/matrix_vector/vector.hpp>
#include <acado_toolkit.hpp>
#include <acado_gnuplot.hpp>
#include <acado/variables_grid/matrix_variable.hpp>

using namespace std;

int main( )
{   

      /*  LINEARIZED CONTROLLER  */
      // USING_NAMESPACE_ACADO

      // DifferentialState eps1;
      // DifferentialState eps2;
      // DifferentialState eps3;
      // DifferentialState eps4;
      // DifferentialState eps5;
      // DifferentialState eps6;
      // Control vi1;
      // Control vi2;
      // OnlineData xObst;
      // OnlineData yObst;

      // const double  Ts = 0.2;
      // const int N = 40;

      // DifferentialEquation f;
      // f << dot(eps1) == eps2;
      // f << dot(eps2) == eps3;
      // f << dot(eps3) == vi1;
      // f << dot(eps4) == eps5;
      // f << dot(eps5) == eps6;
      // f << dot(eps6) == vi2;

      // Function h, hN;
      // h <<  eps1 << eps2 << eps3 << eps4 << eps5 << eps6 << vi1 << vi2;
      // hN << eps1 << eps4;

      // BMatrix W = eye<bool>( h.getDim() );
      // BMatrix WN = eye<bool>( hN.getDim() );

      // OCP ocp(0.0, N*Ts, N);
      // ocp.minimizeLSQ( W, h); 
      // ocp.minimizeLSQEndTerm( WN, hN);

      // // ocp.subjectTo(-.5 <= eps2 <= .5);
      // // ocp.subjectTo(-.5 <= eps3 <= .5);
      // // ocp.subjectTo(-1 <= eps5 <= 1);
      // // ocp.subjectTo(-1 <= eps6 <= 1);

      // ocp.subjectTo(-.5 <= vi1 <= .5);
      // ocp.subjectTo(-.5 <= vi2 <= .5);

      // ocp.setNOD(2);

      // ocp.setModel(f);
      // OCPexport mpc(ocp);

      // mpc.set( HESSIAN_APPROXIMATION, GAUSS_NEWTON);
      // mpc.set( DISCRETIZATION_TYPE, MULTIPLE_SHOOTING);
      // mpc.set( SPARSE_QP_SOLUTION, FULL_CONDENSING_N2);
      // mpc.set( IMPLICIT_INTEGRATOR_NUM_ITS, 2);
      // mpc.set( LINEAR_ALGEBRA_SOLVER, GAUSS_LU);
      // mpc.set( LEVENBERG_MARQUARDT, 1e-8);
      // mpc.set( INTEGRATOR_TYPE, INT_RK78);
      // mpc.set( NUM_INTEGRATOR_STEPS, N );
      // mpc.set( QP_SOLVER, QP_QPOASES);
      // mpc.set( CG_HARDCODE_CONSTRAINT_VALUES, NO);
      // mpc.set( HOTSTART_QP, NO);
      // mpc.set( FIX_INITIAL_STATE, BT_TRUE);

      // mpc.printOptionsList();

      // if (mpc.exportCode( "core" ) != SUCCESSFUL_RETURN)
      //       exit( EXIT_FAILURE );

      // mpc.printDimensionsQP( );
    
      // return EXIT_SUCCESS;





      /*  STANDARD NON LINEAR CONTROLLER  */
      USING_NAMESPACE_ACADO

      DifferentialState px;
      DifferentialState py;
      DifferentialState theta;
      DifferentialState d;



      Control vel;
      Control phi_cmd;

      OnlineData l;
      OnlineData xObst;
      OnlineData yObst;

      const double g = 9.8066;
      double PI = 3.1415926535897932;
      const double Ts = 0.2;
      const int N = 40;

      // System Dynamics
      DifferentialEquation f;
      f << dot(px) ==  vel * cos(theta);
      f << dot(py) ==  vel * sin(theta);
      f << dot(theta) == (vel / l) * tan(phi_cmd);
      f << dot(d) ==  2*( px - xObst )*px + 2*( py - yObst )*py;

      IntermediateState obstDist = ( px - xObst ) * ( px - xObst ) + ( py - yObst ) * ( py - yObst );

      // Cost: Sum(i=0, ..., N-1){h_i' * Q * h_i} + h_N' * Q_N * h_N
      Function h, hN;
      // Running cost vector consists of all states and inputs.
      h << px << py << theta << phi_cmd << vel << 1/obstDist;
      // End cost vector consists of all states (no inputs at last state).
      hN << px << py << theta;

      BMatrix W = eye<bool>( h.getDim() );
      BMatrix WN = eye<bool>( hN.getDim() );
    
      OCP ocp(0.0, N*Ts, N);

      ocp.minimizeLSQ( W, h); 
      ocp.minimizeLSQEndTerm( WN, hN);

      // Add constraints
      ocp.subjectTo(-.5 <= phi_cmd <= .5);
      ocp.subjectTo(-1.5 <= vel <= 1.5);

      //ocp.subjectTo(-1 <= phi_cmd <= 1);
      ocp.subjectTo(1 <= obstDist <= 10000);
      // ocp.setNOD(10);

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
    
      return EXIT_SUCCESS;
}
