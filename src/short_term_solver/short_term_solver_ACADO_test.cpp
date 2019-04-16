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



    float roll_gain = 1.1;
    float roll_tau = 0.15;
    float pitch_gain = 1.1;
    float pitch_tau = 0.15;
    float fx = 448.1;
    float fy = 448.1;   
    float roll_c = 0.f;
    float pitch_c = 0.21;
    float yaw_c = 0.f;
    float tx_c = 0.1;
    float ty_c = 0.055;
    float tz_c = -0.05;
    float tx = 10.f;
    float ty = 0.f;
    float tz = 1.f;

    float xObst = 1.f;
    float yObst = 0.45;

    float xObst1 = 1.f;
    float yObst1 = -0.45;

    float xObst2 = 1.f;
    float zObst2 = 1.9;

    float xObst3 = 1.f;
    float zObst3 = 1.f;

  


    const double g = 9.8066;
    double PI = 3.1415926535897932;
    const double Ts = 0.1;
    const int N = 30;

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
    f << dot(trash) == xObst + yObst + xObst1 + yObst1 + xObst2 + zObst2 + xObst3 + zObst3 + roll_c + pitch_c + yaw_c + tx_c + ty_c + tz_c + tx + ty + tz + roll_tau + roll_gain + pitch_tau + pitch_gain + fx + fy;

    IntermediateState Xt_b_x = (sin(pitch_c)*(sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch)) + cos(pitch_c)*sin(yaw_c)*(cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll)) - cos(pitch)*cos(pitch_c)*cos(yaw)*cos(yaw_c))*(px - tx - ty_c*(cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll)) + tz_c*(sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch)) + tx_c*cos(pitch)*cos(yaw)) + (cos(pitch)*cos(roll)*sin(pitch_c) + cos(pitch_c)*cos(yaw_c)*sin(pitch) - cos(pitch)*cos(pitch_c)*sin(roll)*sin(yaw_c))*(pz - tz - tx_c*sin(pitch) + tz_c*cos(pitch)*cos(roll) + ty_c*cos(pitch)*sin(roll)) - (sin(pitch_c)*(cos(yaw)*sin(roll) - cos(roll)*sin(pitch)*sin(yaw)) + cos(pitch_c)*sin(yaw_c)*(cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw)) + cos(pitch)*cos(pitch_c)*cos(yaw_c)*sin(yaw))*(py - ty + ty_c*(cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw)) - tz_c*(cos(yaw)*sin(roll) - cos(roll)*sin(pitch)*sin(yaw)) + tx_c*cos(pitch)*sin(yaw));
    IntermediateState Xt_b_y = ((cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll))*(cos(roll_c)*cos(yaw_c) + sin(pitch_c)*sin(roll_c)*sin(yaw_c)) + cos(pitch)*cos(yaw)*(cos(roll_c)*sin(yaw_c) - cos(yaw_c)*sin(pitch_c)*sin(roll_c)) - cos(pitch_c)*sin(roll_c)*(sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch)))*(px - tx - ty_c*(cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll)) + tz_c*(sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch)) + tx_c*cos(pitch)*cos(yaw)) - (sin(pitch)*(cos(roll_c)*sin(yaw_c) - cos(yaw_c)*sin(pitch_c)*sin(roll_c)) + cos(pitch)*sin(roll)*(cos(roll_c)*cos(yaw_c) + sin(pitch_c)*sin(roll_c)*sin(yaw_c)) + cos(pitch)*cos(pitch_c)*cos(roll)*sin(roll_c))*(pz - tz - tx_c*sin(pitch) + tz_c*cos(pitch)*cos(roll) + ty_c*cos(pitch)*sin(roll)) + (cos(pitch_c)*sin(roll_c)*(cos(yaw)*sin(roll) - cos(roll)*sin(pitch)*sin(yaw)) - (cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw))*(cos(roll_c)*cos(yaw_c) + sin(pitch_c)*sin(roll_c)*sin(yaw_c)) + cos(pitch)*sin(yaw)*(cos(roll_c)*sin(yaw_c) - cos(yaw_c)*sin(pitch_c)*sin(roll_c)))*(py - ty + ty_c*(cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw)) - tz_c*(cos(yaw)*sin(roll) - cos(roll)*sin(pitch)*sin(yaw)) + tx_c*cos(pitch)*sin(yaw));
    IntermediateState Xt_b_z = (sin(pitch)*(sin(roll_c)*sin(yaw_c) + cos(roll_c)*cos(yaw_c)*sin(pitch_c)) + cos(pitch)*sin(roll)*(cos(yaw_c)*sin(roll_c) - cos(roll_c)*sin(pitch_c)*sin(yaw_c)) - cos(pitch)*cos(pitch_c)*cos(roll)*cos(roll_c))*(pz - tz - tx_c*sin(pitch) + tz_c*cos(pitch)*cos(roll) + ty_c*cos(pitch)*sin(roll)) - ((cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll))*(cos(yaw_c)*sin(roll_c) - cos(roll_c)*sin(pitch_c)*sin(yaw_c)) + cos(pitch_c)*cos(roll_c)*(sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch)) + cos(pitch)*cos(yaw)*(sin(roll_c)*sin(yaw_c) + cos(roll_c)*cos(yaw_c)*sin(pitch_c)))*(px - tx - ty_c*(cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll)) + tz_c*(sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch)) + tx_c*cos(pitch)*cos(yaw)) + ((cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw))*(cos(yaw_c)*sin(roll_c) - cos(roll_c)*sin(pitch_c)*sin(yaw_c)) + cos(pitch_c)*cos(roll_c)*(cos(yaw)*sin(roll) - cos(roll)*sin(pitch)*sin(yaw)) - cos(pitch)*sin(yaw)*(sin(roll_c)*sin(yaw_c) + cos(roll_c)*cos(yaw_c)*sin(pitch_c)))*(py - ty + ty_c*(cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw)) - tz_c*(cos(yaw)*sin(roll) - cos(roll)*sin(pitch)*sin(yaw)) + tx_c*cos(pitch)*sin(yaw));

    IntermediateState ObstDist  = (px - xObst)*(px - xObst) + (py - yObst)*(py - yObst);
    IntermediateState ObstDist1 = (px - xObst1)*(px - xObst1) + (py - yObst1)*(py - yObst1);
    IntermediateState ObstDist2 = (px - xObst2)*(px - xObst2) + (pz - zObst2)*(pz - zObst2);
    IntermediateState ObstDist3 = (px - xObst3)*(px - xObst3) + (pz - zObst3)*(pz - zObst3);

    Function h, hN;
    
    h << px << py << pz << 
         vx << vy << vz << 
         roll << pitch << yaw << 
         roll_ref << pitch_ref << yaw_rate << 1 / ObstDist <<
         1 / ObstDist1 << 1 / ObstDist2 << 1 / ObstDist3;
         //- Xt_b_y / Xt_b_x << - Xt_b_z / Xt_b_x  
         //1 / ObstDist << 1 / ObstDist1 << 1 / ObstDist2;  
    hN << px << py << pz << 
          vx << vy << vz << 
          roll << pitch << yaw;// << 
          //- Xt_b_y / Xt_b_x << - Xt_b_z / Xt_b_x; 
    
    DMatrix W(h.getDim(), h.getDim()); 
    DMatrix WN( hN.getDim(), hN.getDim());  
    DVector cons( h.getDim() );
    DVector cons_N( hN.getDim() );

    W.setIdentity();
    WN.setIdentity();
    cons.setAll( 0.0 );
    cons_N.setAll( 0.0 );

    /*WN(0,0) = 1000;
    WN(1,1) = 1000;
    WN(2,2) = 1000;
    cons_N(0) = 4;*/

    OCP ocp(0.0, N*Ts, N);
    
    ocp.minimizeLSQ( W, h, cons); 
    ocp.minimizeLSQEndTerm( WN, hN, cons_N);
    
    ocp.subjectTo(-35*PI/180 <= roll_ref  <= 35*PI/180);
    ocp.subjectTo(-35*PI/180 <= pitch_ref <= 35*PI/180);
    ocp.subjectTo(g/2.0 <= thrust <= g*1.5);
    ocp.subjectTo(-1 <= yaw_rate <= 1);

    //ocp.subjectTo( -40 <= - fx * Xt_b_y / Xt_b_x <= 40);
    //ocp.subjectTo( -40 <= - fy * Xt_b_z / Xt_b_x <= 40);
    ocp.subjectTo( 0.5 <= ObstDist <= 100000);
    ocp.subjectTo( 0.5 <= ObstDist1 <= 100000);
    ocp.subjectTo( 0.5 <= ObstDist2 <= 100000);
    ocp.subjectTo( 0.5 <= ObstDist3 <= 100000);

    ocp.setModel(f);

    ocp.subjectTo( AT_START, px ==  0.0 );
    ocp.subjectTo( AT_START, py ==  0.0 );
    ocp.subjectTo( AT_START, pz ==  1.5 );
    ocp.subjectTo( AT_START, roll ==  0.0 );
    ocp.subjectTo( AT_START, pitch ==  0.0 );
    ocp.subjectTo( AT_START, yaw ==  0.0 );
    ocp.subjectTo( AT_START, vx ==  0.0 );
    ocp.subjectTo( AT_START, vy ==  0.0 );
    ocp.subjectTo( AT_START, vz ==  0.0 );
    ocp.subjectTo( AT_START, trash ==  0.0 );

    ocp.subjectTo( AT_END, px ==  4.0 );
    ocp.subjectTo( AT_END, py ==  0.0 );
    ocp.subjectTo( AT_END, pz ==  1.5 );
    ocp.subjectTo( AT_END, roll ==  0.0 );
    ocp.subjectTo( AT_END, pitch ==  0.0 );
    ocp.subjectTo( AT_END, yaw ==  0.0 );
    ocp.subjectTo( AT_END, vx ==  0.0 );
    ocp.subjectTo( AT_END, vy ==  0.0 );
    ocp.subjectTo( AT_END, vz ==  0.0 );
    ocp.subjectTo( AT_END, trash ==  0.0 );

      GnuplotWindow window;
      window.addSubplot(px, "position along x axis");
      window.addSubplot(py, "position along y axis");
      window.addSubplot(pz, "position along z axis");
      window.addSubplot(ObstDist, "...");
      window.addSubplot(ObstDist1, "...");
      window.addSubplot(ObstDist2, "...");
      window.addSubplot(ObstDist3, "...");

    OptimizationAlgorithm algorithm(ocp);

    algorithm.set( MAX_NUM_ITERATIONS, 10);
    algorithm.set( HESSIAN_APPROXIMATION, GAUSS_NEWTON);
    algorithm.set( LINEAR_ALGEBRA_SOLVER, GAUSS_LU);
    algorithm.set( LEVENBERG_MARQUARDT, 1e-8);

    algorithm << window;
    algorithm.solve();
    
    return EXIT_SUCCESS;

}
