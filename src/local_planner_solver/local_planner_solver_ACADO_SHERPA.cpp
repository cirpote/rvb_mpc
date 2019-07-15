#include <acado_code_generation.hpp>
#include <acado/matrix_vector/vector.hpp>
#include <acado_toolkit.hpp>
#include <acado_gnuplot.hpp>
#include <acado/variables_grid/matrix_variable.hpp>

using namespace std;

int main( )
{   
      USING_NAMESPACE_ACADO

      DifferentialState px;
      DifferentialState py;
      DifferentialState theta;
      DifferentialState phi;
      DifferentialState dump;

      Control vel;
      Control phi_cmd;

      OnlineData l;
      OnlineData xObst;
      OnlineData yObst;

      const double g = 9.8066;
      double PI = 3.1415926535897932;
      const double Ts = 0.2;
      const int N = 30;

      // System Dynamics
      DifferentialEquation f;
      f << dot(px) ==  vel * cos(theta);
      f << dot(py) ==  vel * sin(theta);
      f << dot(theta) == (vel / l) * tan(phi);
      f << dot(phi) == phi_cmd;
      f << dot(dump) == xObst + yObst;

      IntermediateState obstDist = ( px - xObst ) * ( px - xObst ) + ( py - yObst ) * ( py - yObst );

      // Cost: Sum(i=0, ..., N-1){h_i' * Q * h_i} + h_N' * Q_N * h_N
      Function h, hN;
      // Running cost vector consists of all states and inputs.
      h << px << py << theta << phi << phi_cmd << vel;
      // End cost vector consists of all states (no inputs at last state).
      hN << px << py << theta << phi;

      BMatrix W = eye<bool>( h.getDim() );
      BMatrix WN = eye<bool>( hN.getDim() );
    
      OCP ocp(0.0, N*Ts, N);

      ocp.minimizeLSQ( W, h); 
      ocp.minimizeLSQEndTerm( WN, hN);

      // Add constraints
      ocp.subjectTo(-.5 <= phi_cmd <= .5);
      ocp.subjectTo(-1.5 <= vel <= 1.5);
      ocp.subjectTo(1 <= obstDist <= 10000);
      ocp.setNOD(10);

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
