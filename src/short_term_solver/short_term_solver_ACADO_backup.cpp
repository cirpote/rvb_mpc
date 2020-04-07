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
    DifferentialState pz;
    DifferentialState vx;
    DifferentialState vy;
    DifferentialState vz;
    DifferentialState roll;
    DifferentialState pitch;
    DifferentialState yaw;
    DifferentialState trash;

    Control roll_ref;
    Control pitch_ref;
    Control thrust;
    Control yaw_rate;

    OnlineData roll_tau;
    OnlineData roll_gain;
    OnlineData pitch_tau;
    OnlineData pitch_gain;
    OnlineData roll_c;
    OnlineData pitch_c;
    OnlineData yaw_c;
    OnlineData tx_c;
    OnlineData ty_c;
    OnlineData tz_c;
    OnlineData tx;
    OnlineData ty;
    OnlineData tz;
    OnlineData fx;
    OnlineData fy;

    OnlineData xObst;
    OnlineData yObst;
    OnlineData Obstwx;
    OnlineData Obstwy;

    OnlineData xObst1;
    OnlineData yObst1;
    OnlineData Obst1wx;
    OnlineData Obst1wy;

    OnlineData xObst2;
    OnlineData zObst2;
    OnlineData Obst2wx;
    OnlineData Obst2wz;

    OnlineData xDynObst1;
    OnlineData yDynObst1;
    OnlineData zDynObst1; 
    OnlineData DynObstwx;
    OnlineData DynObstwy;
    OnlineData DynObstwz;

    OnlineData xObst3;
    OnlineData zObst3;
    OnlineData Obst3wx;
    OnlineData Obst3wz;

    const double g = 9.8066;
    double PI = 3.1415926535897932;
    const double Ts = 0.2;
    const int N = 10;

    DifferentialEquation f;
    f << dot(px) == vx;
    f << dot(py) == vy;
    f << dot(pz) == vz;
    f << dot(vx) == (cos(yaw)*sin(pitch)*cos(roll)+sin(yaw)*sin(roll))*thrust; 
    f << dot(vy) == (sin(yaw)*sin(pitch)*cos(roll)-cos(yaw)*sin(roll))*thrust; 
    f << dot(vz) == (cos(pitch)*cos(roll))*thrust - g; 
    f << dot(roll) == (1/roll_tau)*(roll_gain*roll_ref - roll); 
    f << dot(pitch) == (1/pitch_tau)*(pitch_gain*pitch_ref - pitch); 
    f << dot(yaw) == yaw_rate; 
    f << dot(trash) == xObst2 + zObst2 + xDynObst1 + yDynObst1 + zDynObst1 + roll_c +
                       pitch_c + yaw_c + tx_c + ty_c + tz_c + tx + ty + tz + roll_tau +
                       roll_gain + pitch_tau + pitch_gain + fx + fy + xObst + yObst + xObst1 +
                       yObst1 + Obstwx + Obstwy + Obst1wx + Obst1wy + Obst2wx + Obst2wz +
                       DynObstwx + DynObstwy + DynObstwz + xObst3 + zObst3 + Obst3wx + Obst3wz;

    IntermediateState Xt_b_x = (sin(pitch_c)*(sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch)) + cos(pitch_c)*sin(yaw_c)*(cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll)) - cos(pitch)*cos(pitch_c)*cos(yaw)*cos(yaw_c))*(px - tx - ty_c*(cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll)) + tz_c*(sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch)) + tx_c*cos(pitch)*cos(yaw)) + (cos(pitch)*cos(roll)*sin(pitch_c) + cos(pitch_c)*cos(yaw_c)*sin(pitch) - cos(pitch)*cos(pitch_c)*sin(roll)*sin(yaw_c))*(pz - tz - tx_c*sin(pitch) + tz_c*cos(pitch)*cos(roll) + ty_c*cos(pitch)*sin(roll)) - (sin(pitch_c)*(cos(yaw)*sin(roll) - cos(roll)*sin(pitch)*sin(yaw)) + cos(pitch_c)*sin(yaw_c)*(cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw)) + cos(pitch)*cos(pitch_c)*cos(yaw_c)*sin(yaw))*(py - ty + ty_c*(cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw)) - tz_c*(cos(yaw)*sin(roll) - cos(roll)*sin(pitch)*sin(yaw)) + tx_c*cos(pitch)*sin(yaw));
    IntermediateState Xt_b_y = ((cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll))*(cos(roll_c)*cos(yaw_c) + sin(pitch_c)*sin(roll_c)*sin(yaw_c)) + cos(pitch)*cos(yaw)*(cos(roll_c)*sin(yaw_c) - cos(yaw_c)*sin(pitch_c)*sin(roll_c)) - cos(pitch_c)*sin(roll_c)*(sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch)))*(px - tx - ty_c*(cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll)) + tz_c*(sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch)) + tx_c*cos(pitch)*cos(yaw)) - (sin(pitch)*(cos(roll_c)*sin(yaw_c) - cos(yaw_c)*sin(pitch_c)*sin(roll_c)) + cos(pitch)*sin(roll)*(cos(roll_c)*cos(yaw_c) + sin(pitch_c)*sin(roll_c)*sin(yaw_c)) + cos(pitch)*cos(pitch_c)*cos(roll)*sin(roll_c))*(pz - tz - tx_c*sin(pitch) + tz_c*cos(pitch)*cos(roll) + ty_c*cos(pitch)*sin(roll)) + (cos(pitch_c)*sin(roll_c)*(cos(yaw)*sin(roll) - cos(roll)*sin(pitch)*sin(yaw)) - (cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw))*(cos(roll_c)*cos(yaw_c) + sin(pitch_c)*sin(roll_c)*sin(yaw_c)) + cos(pitch)*sin(yaw)*(cos(roll_c)*sin(yaw_c) - cos(yaw_c)*sin(pitch_c)*sin(roll_c)))*(py - ty + ty_c*(cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw)) - tz_c*(cos(yaw)*sin(roll) - cos(roll)*sin(pitch)*sin(yaw)) + tx_c*cos(pitch)*sin(yaw));
    IntermediateState Xt_b_z = (sin(pitch)*(sin(roll_c)*sin(yaw_c) + cos(roll_c)*cos(yaw_c)*sin(pitch_c)) + cos(pitch)*sin(roll)*(cos(yaw_c)*sin(roll_c) - cos(roll_c)*sin(pitch_c)*sin(yaw_c)) - cos(pitch)*cos(pitch_c)*cos(roll)*cos(roll_c))*(pz - tz - tx_c*sin(pitch) + tz_c*cos(pitch)*cos(roll) + ty_c*cos(pitch)*sin(roll)) - ((cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll))*(cos(yaw_c)*sin(roll_c) - cos(roll_c)*sin(pitch_c)*sin(yaw_c)) + cos(pitch_c)*cos(roll_c)*(sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch)) + cos(pitch)*cos(yaw)*(sin(roll_c)*sin(yaw_c) + cos(roll_c)*cos(yaw_c)*sin(pitch_c)))*(px - tx - ty_c*(cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll)) + tz_c*(sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch)) + tx_c*cos(pitch)*cos(yaw)) + ((cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw))*(cos(yaw_c)*sin(roll_c) - cos(roll_c)*sin(pitch_c)*sin(yaw_c)) + cos(pitch_c)*cos(roll_c)*(cos(yaw)*sin(roll) - cos(roll)*sin(pitch)*sin(yaw)) - cos(pitch)*sin(yaw)*(sin(roll_c)*sin(yaw_c) + cos(roll_c)*cos(yaw_c)*sin(pitch_c)))*(py - ty + ty_c*(cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw)) - tz_c*(cos(yaw)*sin(roll) - cos(roll)*sin(pitch)*sin(yaw)) + tx_c*cos(pitch)*sin(yaw));

    IntermediateState ObstDist = sqrt(Obstwx*(px - xObst)*(px - xObst) + Obstwy*(py - yObst)*(py - yObst));
    IntermediateState ObstDist1 = sqrt(Obst1wx*(px - xObst1)*(px - xObst1) + Obst1wy*(py - yObst1)*(py - yObst1));
    IntermediateState ObstDist2 = sqrt(Obst2wx*(px - xObst2)*(px - xObst2) + Obst2wz*(pz - zObst2)*(pz - zObst2));
    IntermediateState ObstDist3 = sqrt(Obst3wx*(px - xObst3)*(px - xObst3) + Obst3wz*(pz - zObst3)*(pz - zObst3));
    IntermediateState DynObstDist1 = sqrt(DynObstwx*(px - xDynObst1)*(px - xDynObst1) + DynObstwy*(py - yDynObst1)*(py - yDynObst1) + DynObstwz*(pz - zDynObst1)*(pz - zDynObst1));

    Function h, hN;
    
    float alpha = 6;
    float beta = 8;

    h << px << py << pz << 
         vx << vy << vz << 
         roll << pitch << yaw << 
         roll_ref << pitch_ref << yaw_rate << 
         - Xt_b_y / Xt_b_x << - Xt_b_z / Xt_b_x 
         << exp( alpha - beta * ObstDist) 
         << exp( alpha - beta * ObstDist1) 
         << exp( alpha - beta * ObstDist2) 
         << exp( alpha - beta * DynObstDist1) 
         << exp( alpha - beta * ObstDist3);

    hN << px << py << pz << 
          vx << vy << vz << 
          roll << pitch << yaw << 
          - Xt_b_y / Xt_b_x << - Xt_b_z / Xt_b_x;
    
    BMatrix W = eye<bool>( h.getDim() );
    BMatrix WN = eye<bool>( hN.getDim() );
    
    OCP ocp(0.0, N*Ts, N);
    
    ocp.minimizeLSQ( W, h); 
    ocp.minimizeLSQEndTerm( WN, hN);
    
    ocp.subjectTo(-35*PI/180 <= roll_ref  <= 35*PI/180);
    ocp.subjectTo(-35*PI/180 <= pitch_ref <= 35*PI/180);
    ocp.subjectTo(g/2.0 <= thrust <= g*1.5);
    ocp.subjectTo(-1 <= yaw_rate <= 1);

    //ocp.subjectTo( -40 <= - fx * Xt_b_y / Xt_b_x <= 40);
    //ocp.subjectTo( -40 <= - fy * Xt_b_z / Xt_b_x <= 40);
    //ocp.subjectTo( 1.25 <= ObstDist <= 100000);
    //ocp.subjectTo( 1.25 <= ObstDist1 <= 100000);
    //ocp.subjectTo( 1.25 <= ObstDist2 <= 100000);
    //ocp.subjectTo( 1.25 <= DynObstDist1 <= 100000);
    //ocp.subjectTo( 1.25 <= ObstDist3 <= 100000);

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
