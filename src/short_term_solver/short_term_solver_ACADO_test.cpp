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
      DifferentialState     px, py, pz, vx, vy, vz, roll, pitch, yaw, pitch_c, yaw_c, trash;//, phi, d;
      Control               roll_ref, pitch_ref, thrust, yaw_rate, p_ref, y_ref;//phi_dot;
      DifferentialEquation  f;
      Function              h, hN;

      const double mass = 1.5435;
      const double roll_tau = 0.15;
      const double roll_gain = 1.1;
      const double pitch_tau = 0.15;
      const double pitch_gain = 1.1;

      const double p_tau = 0.3;
      const double p_gain = 1.f;
      const double y_tau = 0.45;
      const double y_gain = 1.f;

      const double roll_c = 0;
      const double tx_c = 0.1;
      const double ty_c = 0;
      const double tz_c = -0.06;
      const double tx = 10;
      const double ty = 1;
      const double tz = 1;
      const double fx = 448.1;
      const double fy = 448.1;

      const double xObst1 = 1;
      const double yObst1 = 0.45;
      const double Obst1wx = 1;
      const double Obst1wy = 1;

      const double xObst2 = 2;
      const double yObst2 = -0.45;
      const double Obst2wx = 1;
      const double Obst2wy = 1;

      const double xObst3 = 1.5;
      const double zObst3 = 2.6;
      const double Obst3wx = 1;
      const double Obst3wz = 1;

      const double xObst4 = 1.5;
      const double zObst4 = 3.7;
      const double Obst4wx = 1;
      const double Obst4wz = 1;

      const double xDynObst = -10;
      const double yDynObst = -10;
      const double zDynObst = -10; 
      const double DynObstwx = 1;
      const double DynObstwy = 1;
      const double DynObstwz = 1;


      const double g = 9.8066;
      double PI = 3.1415926535897932;
      const double t_start = 0;
      const double t_end = 5;
      const double Ts = 0.2;
      const int N_steps = t_end / Ts;

      f << dot(px) == vx;
      f << dot(py) == vy;
      f << dot(pz) == vz;
      f << dot(vx) == (cos(yaw)*sin(pitch)*cos(roll)+sin(yaw)*sin(roll))*thrust; 
      f << dot(vy) == (sin(yaw)*sin(pitch)*cos(roll)-cos(yaw)*sin(roll))*thrust; 
      f << dot(vz) == (cos(pitch)*cos(roll))*thrust - g; 
      f << dot(roll) == (1/roll_tau)*(roll_gain*roll_ref - roll); 
      f << dot(pitch) == (1/pitch_tau)*(pitch_gain*pitch_ref - pitch); 
      f << dot(yaw) == yaw_rate; 
      f << dot(pitch_c) == (1/p_tau)*(p_gain*p_ref - pitch_c);
      f << dot(yaw_c) == (1/y_tau)*(y_gain*y_ref - yaw_c); 
      f << dot(trash) == Obst1wx + Obst1wy + xObst1 + yObst1 + Obst2wx + Obst2wy + xObst2 + yObst2 +
                         Obst3wx + Obst3wz + xObst3 + zObst3 + Obst4wx + Obst4wz + xObst4 + zObst4 +
                         roll_c + pitch_c + yaw_c + tx_c + ty_c + tz_c + tx + ty + tz + roll_tau +
                         roll_gain + pitch_tau + pitch_gain + fx + fy;


      IntermediateState Xt_b_x = (sin(pitch_c)*(sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch)) + cos(pitch_c)*sin(yaw_c)*(cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll)) - cos(pitch)*cos(pitch_c)*cos(yaw)*cos(yaw_c))*(px - tx - ty_c*(cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll)) + tz_c*(sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch)) + tx_c*cos(pitch)*cos(yaw)) + (cos(pitch)*cos(roll)*sin(pitch_c) + cos(pitch_c)*cos(yaw_c)*sin(pitch) - cos(pitch)*cos(pitch_c)*sin(roll)*sin(yaw_c))*(pz - tz - tx_c*sin(pitch) + tz_c*cos(pitch)*cos(roll) + ty_c*cos(pitch)*sin(roll)) - (sin(pitch_c)*(cos(yaw)*sin(roll) - cos(roll)*sin(pitch)*sin(yaw)) + cos(pitch_c)*sin(yaw_c)*(cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw)) + cos(pitch)*cos(pitch_c)*cos(yaw_c)*sin(yaw))*(py - ty + ty_c*(cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw)) - tz_c*(cos(yaw)*sin(roll) - cos(roll)*sin(pitch)*sin(yaw)) + tx_c*cos(pitch)*sin(yaw));
      IntermediateState Xt_b_y = ((cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll))*(cos(roll_c)*cos(yaw_c) + sin(pitch_c)*sin(roll_c)*sin(yaw_c)) + cos(pitch)*cos(yaw)*(cos(roll_c)*sin(yaw_c) - cos(yaw_c)*sin(pitch_c)*sin(roll_c)) - cos(pitch_c)*sin(roll_c)*(sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch)))*(px - tx - ty_c*(cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll)) + tz_c*(sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch)) + tx_c*cos(pitch)*cos(yaw)) - (sin(pitch)*(cos(roll_c)*sin(yaw_c) - cos(yaw_c)*sin(pitch_c)*sin(roll_c)) + cos(pitch)*sin(roll)*(cos(roll_c)*cos(yaw_c) + sin(pitch_c)*sin(roll_c)*sin(yaw_c)) + cos(pitch)*cos(pitch_c)*cos(roll)*sin(roll_c))*(pz - tz - tx_c*sin(pitch) + tz_c*cos(pitch)*cos(roll) + ty_c*cos(pitch)*sin(roll)) + (cos(pitch_c)*sin(roll_c)*(cos(yaw)*sin(roll) - cos(roll)*sin(pitch)*sin(yaw)) - (cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw))*(cos(roll_c)*cos(yaw_c) + sin(pitch_c)*sin(roll_c)*sin(yaw_c)) + cos(pitch)*sin(yaw)*(cos(roll_c)*sin(yaw_c) - cos(yaw_c)*sin(pitch_c)*sin(roll_c)))*(py - ty + ty_c*(cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw)) - tz_c*(cos(yaw)*sin(roll) - cos(roll)*sin(pitch)*sin(yaw)) + tx_c*cos(pitch)*sin(yaw));
      IntermediateState Xt_b_z = (sin(pitch)*(sin(roll_c)*sin(yaw_c) + cos(roll_c)*cos(yaw_c)*sin(pitch_c)) + cos(pitch)*sin(roll)*(cos(yaw_c)*sin(roll_c) - cos(roll_c)*sin(pitch_c)*sin(yaw_c)) - cos(pitch)*cos(pitch_c)*cos(roll)*cos(roll_c))*(pz - tz - tx_c*sin(pitch) + tz_c*cos(pitch)*cos(roll) + ty_c*cos(pitch)*sin(roll)) - ((cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll))*(cos(yaw_c)*sin(roll_c) - cos(roll_c)*sin(pitch_c)*sin(yaw_c)) + cos(pitch_c)*cos(roll_c)*(sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch)) + cos(pitch)*cos(yaw)*(sin(roll_c)*sin(yaw_c) + cos(roll_c)*cos(yaw_c)*sin(pitch_c)))*(px - tx - ty_c*(cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll)) + tz_c*(sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch)) + tx_c*cos(pitch)*cos(yaw)) + ((cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw))*(cos(yaw_c)*sin(roll_c) - cos(roll_c)*sin(pitch_c)*sin(yaw_c)) + cos(pitch_c)*cos(roll_c)*(cos(yaw)*sin(roll) - cos(roll)*sin(pitch)*sin(yaw)) - cos(pitch)*sin(yaw)*(sin(roll_c)*sin(yaw_c) + cos(roll_c)*cos(yaw_c)*sin(pitch_c)))*(py - ty + ty_c*(cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw)) - tz_c*(cos(yaw)*sin(roll) - cos(roll)*sin(pitch)*sin(yaw)) + tx_c*cos(pitch)*sin(yaw));

      IntermediateState ObstDist = sqrt(Obst1wx*(px - xObst1)*(px - xObst1) + Obst1wy*(py - yObst1)*(py - yObst1));
      IntermediateState ObstDist1 = sqrt(Obst2wx*(px - xObst2)*(px - xObst2) + Obst2wy*(py - yObst2)*(py - yObst2));
      IntermediateState ObstDist2 = sqrt(Obst3wx*(px - xObst3)*(px - xObst3) + Obst3wz*(pz - zObst3)*(pz - zObst3));
      IntermediateState ObstDist3 = sqrt(Obst4wx*(px - xObst4)*(px - xObst4) + Obst4wz*(pz - zObst4)*(pz - zObst4));
      IntermediateState DynObstDist1 = sqrt(DynObstwx*(px - xDynObst)*(px - xDynObst) + DynObstwy*(py - yDynObst)*(py - yDynObst) + DynObstwz*(pz - zDynObst)*(pz - zDynObst));

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

      // Running cost weight matrix
      DMatrix Q(h.getDim(), h.getDim());
      Q.setIdentity();
      Q(0,0) = 100; Q(1,1) = 100; Q(2,2) = 100;   // x, y, z
      Q(3,3) = 100; Q(4,4) = 100; Q(5,5) = 100;   // vx, vy, vz
      Q(6,6) = 100; Q(7,7) = 100; Q(8,8) = 100;   //roll, pitch, yaw
      Q(9,9) = 100; Q(10,10) = 100; Q(11,11) = 100;   // roll_ref, pitch_ref, yaw_rate
      Q(12,12) = 100; Q(13,13) = 100; // u, v
      Q(14,14) = 1; Q(15,15) = 1; Q(16,16) = 1; Q(17,17) = 1; Q(18,18) = 1; // obst1, obst2, obst3, dyn obst, obst4

      // End cost weight matrix
      DMatrix QN(hN.getDim(), hN.getDim());
      QN.setIdentity();
      QN(0,0) = 100; QN(1,1) = 100; QN(2,2) = 100;   // x, y, z
      QN(3,3) = 100; QN(4,4) = 100; QN(5,5) = 100;   // vx, vy, vz
      QN(6,6) = 100; QN(7,7) = 100; QN(8,8) = 100;   // roll, pitch, yaw
      QN(9,9) = 100; QN(10,10) = 100;   // u, v


      // Set a reference for the analysis (if CODE_GEN is false).
      // Referes at x = 2.0m in hover (qw = 1).
      DVector r(h.getDim());
      r.setZero();
      r(0) = 5; r(1) = 0; r(2) = 2.07;      // x, y, z
      r(3) = 0; r(4) = 0; r(5) = 0;      // vx, vy, vz
      r(6) = 0; r(7) = 0; r(8) = 0;      // roll, pitch, yaw
      r(9) = 0; r(10) = 0; r(11) = 0;    // roll_ref, pitch_ref, yaw_rate
      r(12) = 0; r(13) = 0;    // u, v
      r(14) = 0; r(15) = 0; r(16) = 0; r(17) = 0; r(18) = 0;  // obst1, obst2, obst3, dyn obst, obst4

      // End cost reference
      DVector rN(hN.getDim());   
      rN.setZero();
      rN(0) = 5; rN(1) = 0; rN(2) = 2.07;   // x, y, z
      rN(3) = 0; rN(4) = 0; rN(5) = 0;   // vx, vy, vz
      rN(6) = 0; rN(7) = 0; rN(8) = 0;   // roll, pitch, yaw
      rN(9) = 0; rN(10) = 0;   // u, v

      // DEFINE AN OPTIMAL CONTROL PROBLEM:
      // ----------------------------------
      OCP ocp( t_start, t_end, N_steps );

      // For analysis, set references.
      ocp.minimizeLSQ( Q, h, r );
      ocp.minimizeLSQEndTerm( QN, hN, rN );

      // Add system dynamics
      ocp.subjectTo( f );

      ocp.subjectTo(-35*PI/180 <= roll_ref  <= 35*PI/180);
      ocp.subjectTo(-35*PI/180 <= pitch_ref <= 35*PI/180);
      ocp.subjectTo(g/2.0 <= thrust <= g*1.5);
      ocp.subjectTo(-1 <= yaw_rate <= 1);
      ocp.subjectTo(-1 <= p_ref <= 1);
      ocp.subjectTo(-1 <= y_ref <= 1);

      // Set initial state
      ocp.subjectTo( AT_START, px ==  0.0 );
      ocp.subjectTo( AT_START, py ==  0.0 );
      ocp.subjectTo( AT_START, pz ==  2.0 );
      ocp.subjectTo( AT_START, vx ==  0.0 );
      ocp.subjectTo( AT_START, vy ==  0.0 );
      ocp.subjectTo( AT_START, vz ==  0.0 );
      ocp.subjectTo( AT_START, roll ==  0.0 );
      ocp.subjectTo( AT_START, pitch ==  0.0 );
      ocp.subjectTo( AT_START, yaw ==  0.0 );
      ocp.subjectTo( AT_START, pitch_c ==  0.21 );
      ocp.subjectTo( AT_START, yaw_c ==  0.0 );

      // Setup some visualization
      GnuplotWindow window( PLOT_AT_EACH_ITERATION );
      window.addSubplot( px, "px" );
      window.addSubplot( py, "py" );
      window.addSubplot( pz, "pz" );
      window.addSubplot( yaw, "yaw" );
      window.addSubplot( - Xt_b_y / Xt_b_x * fx + 320, "u error" );
      window.addSubplot( - Xt_b_z / Xt_b_x * fx + 240, "v error" );
      window.addSubplot( pitch_c, "pitch_c" );
      window.addSubplot( yaw_c, "yaw_c" );

      //window.addSubplot( v,"velocity");
      //window.addSubplot( phi,"steering angle");
      // window.addSubplot( obstDist1, "obstacle 1 distance");
      // window.addSubplot( obstDist2, "obstacle 2 distance");
      // window.addSubplot( obstDist3, "obstacle 3 distance");
      // window.addSubplot( obstDist4, "obstacle 4 distance");
      // window.addSubplot( obstDist5, "obstacle 5 distance");
      // window.addSubplot( obstDist6, "obstacle 6 distance");

      // Define an algorithm to solve it.
      OptimizationAlgorithm algorithm(ocp);
      algorithm.set( KKT_TOLERANCE, 1e-4 );
      algorithm.set( MAX_NUM_ITERATIONS, 10);
      algorithm.set( LINEAR_ALGEBRA_SOLVER, GAUSS_LU);
      algorithm.set( LEVENBERG_MARQUARDT, 1e-8);
      algorithm.set( DISCRETIZATION_TYPE, MULTIPLE_SHOOTING);
      algorithm.set( INTEGRATOR_TYPE, INT_RK78);
      algorithm.set( ABSOLUTE_TOLERANCE, 1e-3 ); 
      algorithm.set( INTEGRATOR_TOLERANCE, 1e-3 );

      

      algorithm << window;
      algorithm.solve();
      
      return EXIT_SUCCESS;
}