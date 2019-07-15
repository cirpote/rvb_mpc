/*
 *    This file was auto-generated using the ACADO Toolkit.
 *    
 *    While ACADO Toolkit is free software released under the terms of
 *    the GNU Lesser General Public License (LGPL), the generated code
 *    as such remains the property of the user who used ACADO Toolkit
 *    to generate this code. In particular, user dependent data of the code
 *    do not inherit the GNU LGPL license. On the other hand, parts of the
 *    generated code that are a direct copy of source code from the
 *    ACADO Toolkit or the software tools it is based on, remain, as derived
 *    work, automatically covered by the LGPL license.
 *    
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *    
 */


#include "acado_common.h"




/******************************************************************************/
/*                                                                            */
/* ACADO code generation                                                      */
/*                                                                            */
/******************************************************************************/


int acado_modelSimulation(  )
{
int ret;

int lRun1;
ret = 0;
for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
acadoWorkspace.state[0] = acadoVariables.x[lRun1 * 5];
acadoWorkspace.state[1] = acadoVariables.x[lRun1 * 5 + 1];
acadoWorkspace.state[2] = acadoVariables.x[lRun1 * 5 + 2];
acadoWorkspace.state[3] = acadoVariables.x[lRun1 * 5 + 3];
acadoWorkspace.state[4] = acadoVariables.x[lRun1 * 5 + 4];

acadoWorkspace.state[40] = acadoVariables.u[lRun1 * 2];
acadoWorkspace.state[41] = acadoVariables.u[lRun1 * 2 + 1];
acadoWorkspace.state[42] = acadoVariables.od[lRun1 * 10];
acadoWorkspace.state[43] = acadoVariables.od[lRun1 * 10 + 1];
acadoWorkspace.state[44] = acadoVariables.od[lRun1 * 10 + 2];
acadoWorkspace.state[45] = acadoVariables.od[lRun1 * 10 + 3];
acadoWorkspace.state[46] = acadoVariables.od[lRun1 * 10 + 4];
acadoWorkspace.state[47] = acadoVariables.od[lRun1 * 10 + 5];
acadoWorkspace.state[48] = acadoVariables.od[lRun1 * 10 + 6];
acadoWorkspace.state[49] = acadoVariables.od[lRun1 * 10 + 7];
acadoWorkspace.state[50] = acadoVariables.od[lRun1 * 10 + 8];
acadoWorkspace.state[51] = acadoVariables.od[lRun1 * 10 + 9];

ret = acado_integrate(acadoWorkspace.state, 1);

acadoWorkspace.d[lRun1 * 5] = acadoWorkspace.state[0] - acadoVariables.x[lRun1 * 5 + 5];
acadoWorkspace.d[lRun1 * 5 + 1] = acadoWorkspace.state[1] - acadoVariables.x[lRun1 * 5 + 6];
acadoWorkspace.d[lRun1 * 5 + 2] = acadoWorkspace.state[2] - acadoVariables.x[lRun1 * 5 + 7];
acadoWorkspace.d[lRun1 * 5 + 3] = acadoWorkspace.state[3] - acadoVariables.x[lRun1 * 5 + 8];
acadoWorkspace.d[lRun1 * 5 + 4] = acadoWorkspace.state[4] - acadoVariables.x[lRun1 * 5 + 9];

acadoWorkspace.evGx[lRun1 * 25] = acadoWorkspace.state[5];
acadoWorkspace.evGx[lRun1 * 25 + 1] = acadoWorkspace.state[6];
acadoWorkspace.evGx[lRun1 * 25 + 2] = acadoWorkspace.state[7];
acadoWorkspace.evGx[lRun1 * 25 + 3] = acadoWorkspace.state[8];
acadoWorkspace.evGx[lRun1 * 25 + 4] = acadoWorkspace.state[9];
acadoWorkspace.evGx[lRun1 * 25 + 5] = acadoWorkspace.state[10];
acadoWorkspace.evGx[lRun1 * 25 + 6] = acadoWorkspace.state[11];
acadoWorkspace.evGx[lRun1 * 25 + 7] = acadoWorkspace.state[12];
acadoWorkspace.evGx[lRun1 * 25 + 8] = acadoWorkspace.state[13];
acadoWorkspace.evGx[lRun1 * 25 + 9] = acadoWorkspace.state[14];
acadoWorkspace.evGx[lRun1 * 25 + 10] = acadoWorkspace.state[15];
acadoWorkspace.evGx[lRun1 * 25 + 11] = acadoWorkspace.state[16];
acadoWorkspace.evGx[lRun1 * 25 + 12] = acadoWorkspace.state[17];
acadoWorkspace.evGx[lRun1 * 25 + 13] = acadoWorkspace.state[18];
acadoWorkspace.evGx[lRun1 * 25 + 14] = acadoWorkspace.state[19];
acadoWorkspace.evGx[lRun1 * 25 + 15] = acadoWorkspace.state[20];
acadoWorkspace.evGx[lRun1 * 25 + 16] = acadoWorkspace.state[21];
acadoWorkspace.evGx[lRun1 * 25 + 17] = acadoWorkspace.state[22];
acadoWorkspace.evGx[lRun1 * 25 + 18] = acadoWorkspace.state[23];
acadoWorkspace.evGx[lRun1 * 25 + 19] = acadoWorkspace.state[24];
acadoWorkspace.evGx[lRun1 * 25 + 20] = acadoWorkspace.state[25];
acadoWorkspace.evGx[lRun1 * 25 + 21] = acadoWorkspace.state[26];
acadoWorkspace.evGx[lRun1 * 25 + 22] = acadoWorkspace.state[27];
acadoWorkspace.evGx[lRun1 * 25 + 23] = acadoWorkspace.state[28];
acadoWorkspace.evGx[lRun1 * 25 + 24] = acadoWorkspace.state[29];

acadoWorkspace.evGu[lRun1 * 10] = acadoWorkspace.state[30];
acadoWorkspace.evGu[lRun1 * 10 + 1] = acadoWorkspace.state[31];
acadoWorkspace.evGu[lRun1 * 10 + 2] = acadoWorkspace.state[32];
acadoWorkspace.evGu[lRun1 * 10 + 3] = acadoWorkspace.state[33];
acadoWorkspace.evGu[lRun1 * 10 + 4] = acadoWorkspace.state[34];
acadoWorkspace.evGu[lRun1 * 10 + 5] = acadoWorkspace.state[35];
acadoWorkspace.evGu[lRun1 * 10 + 6] = acadoWorkspace.state[36];
acadoWorkspace.evGu[lRun1 * 10 + 7] = acadoWorkspace.state[37];
acadoWorkspace.evGu[lRun1 * 10 + 8] = acadoWorkspace.state[38];
acadoWorkspace.evGu[lRun1 * 10 + 9] = acadoWorkspace.state[39];
}
return ret;
}

void acado_evaluateLSQ(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 5;

/* Compute outputs: */
out[0] = xd[0];
out[1] = xd[1];
out[2] = xd[2];
out[3] = xd[3];
out[4] = u[1];
out[5] = u[0];
}

void acado_evaluateLSQEndTerm(const real_t* in, real_t* out)
{
const real_t* xd = in;

/* Compute outputs: */
out[0] = xd[0];
out[1] = xd[1];
out[2] = xd[2];
out[3] = xd[3];
}

void acado_evaluatePathConstraints(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* od = in + 7;
/* Vector of auxiliary variables; number of elements: 12. */
real_t* a = acadoWorkspace.conAuxVar;

/* Compute intermediate quantities: */
a[0] = (((xd[0]-od[1])*(xd[0]-od[1]))+((xd[1]-od[2])*(xd[1]-od[2])));
a[1] = (xd[0]-od[1]);
a[2] = (xd[0]-od[1]);
a[3] = (a[1]+a[2]);
a[4] = (xd[1]-od[2]);
a[5] = (xd[1]-od[2]);
a[6] = (a[4]+a[5]);
a[7] = (real_t)(0.0000000000000000e+00);
a[8] = (real_t)(0.0000000000000000e+00);
a[9] = (real_t)(0.0000000000000000e+00);
a[10] = (real_t)(0.0000000000000000e+00);
a[11] = (real_t)(0.0000000000000000e+00);

/* Compute outputs: */
out[0] = a[0];
out[1] = a[3];
out[2] = a[6];
out[3] = a[7];
out[4] = a[8];
out[5] = a[9];
out[6] = a[10];
out[7] = a[11];
}

void acado_setObjQ1Q2( real_t* const tmpObjS, real_t* const tmpQ1, real_t* const tmpQ2 )
{
tmpQ2[0] = +tmpObjS[0];
tmpQ2[1] = +tmpObjS[1];
tmpQ2[2] = +tmpObjS[2];
tmpQ2[3] = +tmpObjS[3];
tmpQ2[4] = +tmpObjS[4];
tmpQ2[5] = +tmpObjS[5];
tmpQ2[6] = +tmpObjS[6];
tmpQ2[7] = +tmpObjS[7];
tmpQ2[8] = +tmpObjS[8];
tmpQ2[9] = +tmpObjS[9];
tmpQ2[10] = +tmpObjS[10];
tmpQ2[11] = +tmpObjS[11];
tmpQ2[12] = +tmpObjS[12];
tmpQ2[13] = +tmpObjS[13];
tmpQ2[14] = +tmpObjS[14];
tmpQ2[15] = +tmpObjS[15];
tmpQ2[16] = +tmpObjS[16];
tmpQ2[17] = +tmpObjS[17];
tmpQ2[18] = +tmpObjS[18];
tmpQ2[19] = +tmpObjS[19];
tmpQ2[20] = +tmpObjS[20];
tmpQ2[21] = +tmpObjS[21];
tmpQ2[22] = +tmpObjS[22];
tmpQ2[23] = +tmpObjS[23];
tmpQ2[24] = 0.0;
;
tmpQ2[25] = 0.0;
;
tmpQ2[26] = 0.0;
;
tmpQ2[27] = 0.0;
;
tmpQ2[28] = 0.0;
;
tmpQ2[29] = 0.0;
;
tmpQ1[0] = + tmpQ2[0];
tmpQ1[1] = + tmpQ2[1];
tmpQ1[2] = + tmpQ2[2];
tmpQ1[3] = + tmpQ2[3];
tmpQ1[4] = 0.0;
;
tmpQ1[5] = + tmpQ2[6];
tmpQ1[6] = + tmpQ2[7];
tmpQ1[7] = + tmpQ2[8];
tmpQ1[8] = + tmpQ2[9];
tmpQ1[9] = 0.0;
;
tmpQ1[10] = + tmpQ2[12];
tmpQ1[11] = + tmpQ2[13];
tmpQ1[12] = + tmpQ2[14];
tmpQ1[13] = + tmpQ2[15];
tmpQ1[14] = 0.0;
;
tmpQ1[15] = + tmpQ2[18];
tmpQ1[16] = + tmpQ2[19];
tmpQ1[17] = + tmpQ2[20];
tmpQ1[18] = + tmpQ2[21];
tmpQ1[19] = 0.0;
;
tmpQ1[20] = + tmpQ2[24];
tmpQ1[21] = + tmpQ2[25];
tmpQ1[22] = + tmpQ2[26];
tmpQ1[23] = + tmpQ2[27];
tmpQ1[24] = 0.0;
;
}

void acado_setObjR1R2( real_t* const tmpObjS, real_t* const tmpR1, real_t* const tmpR2 )
{
tmpR2[0] = +tmpObjS[30];
tmpR2[1] = +tmpObjS[31];
tmpR2[2] = +tmpObjS[32];
tmpR2[3] = +tmpObjS[33];
tmpR2[4] = +tmpObjS[34];
tmpR2[5] = +tmpObjS[35];
tmpR2[6] = +tmpObjS[24];
tmpR2[7] = +tmpObjS[25];
tmpR2[8] = +tmpObjS[26];
tmpR2[9] = +tmpObjS[27];
tmpR2[10] = +tmpObjS[28];
tmpR2[11] = +tmpObjS[29];
tmpR1[0] = + tmpR2[5];
tmpR1[1] = + tmpR2[4];
tmpR1[2] = + tmpR2[11];
tmpR1[3] = + tmpR2[10];
}

void acado_setObjQN1QN2( real_t* const tmpObjSEndTerm, real_t* const tmpQN1, real_t* const tmpQN2 )
{
tmpQN2[0] = +tmpObjSEndTerm[0];
tmpQN2[1] = +tmpObjSEndTerm[1];
tmpQN2[2] = +tmpObjSEndTerm[2];
tmpQN2[3] = +tmpObjSEndTerm[3];
tmpQN2[4] = +tmpObjSEndTerm[4];
tmpQN2[5] = +tmpObjSEndTerm[5];
tmpQN2[6] = +tmpObjSEndTerm[6];
tmpQN2[7] = +tmpObjSEndTerm[7];
tmpQN2[8] = +tmpObjSEndTerm[8];
tmpQN2[9] = +tmpObjSEndTerm[9];
tmpQN2[10] = +tmpObjSEndTerm[10];
tmpQN2[11] = +tmpObjSEndTerm[11];
tmpQN2[12] = +tmpObjSEndTerm[12];
tmpQN2[13] = +tmpObjSEndTerm[13];
tmpQN2[14] = +tmpObjSEndTerm[14];
tmpQN2[15] = +tmpObjSEndTerm[15];
tmpQN2[16] = 0.0;
;
tmpQN2[17] = 0.0;
;
tmpQN2[18] = 0.0;
;
tmpQN2[19] = 0.0;
;
tmpQN1[0] = + tmpQN2[0];
tmpQN1[1] = + tmpQN2[1];
tmpQN1[2] = + tmpQN2[2];
tmpQN1[3] = + tmpQN2[3];
tmpQN1[4] = 0.0;
;
tmpQN1[5] = + tmpQN2[4];
tmpQN1[6] = + tmpQN2[5];
tmpQN1[7] = + tmpQN2[6];
tmpQN1[8] = + tmpQN2[7];
tmpQN1[9] = 0.0;
;
tmpQN1[10] = + tmpQN2[8];
tmpQN1[11] = + tmpQN2[9];
tmpQN1[12] = + tmpQN2[10];
tmpQN1[13] = + tmpQN2[11];
tmpQN1[14] = 0.0;
;
tmpQN1[15] = + tmpQN2[12];
tmpQN1[16] = + tmpQN2[13];
tmpQN1[17] = + tmpQN2[14];
tmpQN1[18] = + tmpQN2[15];
tmpQN1[19] = 0.0;
;
tmpQN1[20] = + tmpQN2[16];
tmpQN1[21] = + tmpQN2[17];
tmpQN1[22] = + tmpQN2[18];
tmpQN1[23] = + tmpQN2[19];
tmpQN1[24] = 0.0;
;
}

void acado_evaluateObjective(  )
{
int runObj;
for (runObj = 0; runObj < 30; ++runObj)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[runObj * 5];
acadoWorkspace.objValueIn[1] = acadoVariables.x[runObj * 5 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[runObj * 5 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.x[runObj * 5 + 3];
acadoWorkspace.objValueIn[4] = acadoVariables.x[runObj * 5 + 4];
acadoWorkspace.objValueIn[5] = acadoVariables.u[runObj * 2];
acadoWorkspace.objValueIn[6] = acadoVariables.u[runObj * 2 + 1];
acadoWorkspace.objValueIn[7] = acadoVariables.od[runObj * 10];
acadoWorkspace.objValueIn[8] = acadoVariables.od[runObj * 10 + 1];
acadoWorkspace.objValueIn[9] = acadoVariables.od[runObj * 10 + 2];
acadoWorkspace.objValueIn[10] = acadoVariables.od[runObj * 10 + 3];
acadoWorkspace.objValueIn[11] = acadoVariables.od[runObj * 10 + 4];
acadoWorkspace.objValueIn[12] = acadoVariables.od[runObj * 10 + 5];
acadoWorkspace.objValueIn[13] = acadoVariables.od[runObj * 10 + 6];
acadoWorkspace.objValueIn[14] = acadoVariables.od[runObj * 10 + 7];
acadoWorkspace.objValueIn[15] = acadoVariables.od[runObj * 10 + 8];
acadoWorkspace.objValueIn[16] = acadoVariables.od[runObj * 10 + 9];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[runObj * 6] = acadoWorkspace.objValueOut[0];
acadoWorkspace.Dy[runObj * 6 + 1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.Dy[runObj * 6 + 2] = acadoWorkspace.objValueOut[2];
acadoWorkspace.Dy[runObj * 6 + 3] = acadoWorkspace.objValueOut[3];
acadoWorkspace.Dy[runObj * 6 + 4] = acadoWorkspace.objValueOut[4];
acadoWorkspace.Dy[runObj * 6 + 5] = acadoWorkspace.objValueOut[5];

acado_setObjQ1Q2( acadoVariables.W, &(acadoWorkspace.Q1[ runObj * 25 ]), &(acadoWorkspace.Q2[ runObj * 30 ]) );

acado_setObjR1R2( acadoVariables.W, &(acadoWorkspace.R1[ runObj * 4 ]), &(acadoWorkspace.R2[ runObj * 12 ]) );

}
acadoWorkspace.objValueIn[0] = acadoVariables.x[150];
acadoWorkspace.objValueIn[1] = acadoVariables.x[151];
acadoWorkspace.objValueIn[2] = acadoVariables.x[152];
acadoWorkspace.objValueIn[3] = acadoVariables.x[153];
acadoWorkspace.objValueIn[4] = acadoVariables.x[154];
acadoWorkspace.objValueIn[5] = acadoVariables.od[300];
acadoWorkspace.objValueIn[6] = acadoVariables.od[301];
acadoWorkspace.objValueIn[7] = acadoVariables.od[302];
acadoWorkspace.objValueIn[8] = acadoVariables.od[303];
acadoWorkspace.objValueIn[9] = acadoVariables.od[304];
acadoWorkspace.objValueIn[10] = acadoVariables.od[305];
acadoWorkspace.objValueIn[11] = acadoVariables.od[306];
acadoWorkspace.objValueIn[12] = acadoVariables.od[307];
acadoWorkspace.objValueIn[13] = acadoVariables.od[308];
acadoWorkspace.objValueIn[14] = acadoVariables.od[309];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );

acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.DyN[2] = acadoWorkspace.objValueOut[2];
acadoWorkspace.DyN[3] = acadoWorkspace.objValueOut[3];

acado_setObjQN1QN2( acadoVariables.WN, acadoWorkspace.QN1, acadoWorkspace.QN2 );

}

void acado_moveGxT( real_t* const Gx1, real_t* const Gx2 )
{
Gx2[0] = Gx1[0];
Gx2[1] = Gx1[1];
Gx2[2] = Gx1[2];
Gx2[3] = Gx1[3];
Gx2[4] = Gx1[4];
Gx2[5] = Gx1[5];
Gx2[6] = Gx1[6];
Gx2[7] = Gx1[7];
Gx2[8] = Gx1[8];
Gx2[9] = Gx1[9];
Gx2[10] = Gx1[10];
Gx2[11] = Gx1[11];
Gx2[12] = Gx1[12];
Gx2[13] = Gx1[13];
Gx2[14] = Gx1[14];
Gx2[15] = Gx1[15];
Gx2[16] = Gx1[16];
Gx2[17] = Gx1[17];
Gx2[18] = Gx1[18];
Gx2[19] = Gx1[19];
Gx2[20] = Gx1[20];
Gx2[21] = Gx1[21];
Gx2[22] = Gx1[22];
Gx2[23] = Gx1[23];
Gx2[24] = Gx1[24];
}

void acado_multGxGx( real_t* const Gx1, real_t* const Gx2, real_t* const Gx3 )
{
Gx3[0] = + Gx1[0]*Gx2[0] + Gx1[1]*Gx2[5] + Gx1[2]*Gx2[10] + Gx1[3]*Gx2[15] + Gx1[4]*Gx2[20];
Gx3[1] = + Gx1[0]*Gx2[1] + Gx1[1]*Gx2[6] + Gx1[2]*Gx2[11] + Gx1[3]*Gx2[16] + Gx1[4]*Gx2[21];
Gx3[2] = + Gx1[0]*Gx2[2] + Gx1[1]*Gx2[7] + Gx1[2]*Gx2[12] + Gx1[3]*Gx2[17] + Gx1[4]*Gx2[22];
Gx3[3] = + Gx1[0]*Gx2[3] + Gx1[1]*Gx2[8] + Gx1[2]*Gx2[13] + Gx1[3]*Gx2[18] + Gx1[4]*Gx2[23];
Gx3[4] = + Gx1[0]*Gx2[4] + Gx1[1]*Gx2[9] + Gx1[2]*Gx2[14] + Gx1[3]*Gx2[19] + Gx1[4]*Gx2[24];
Gx3[5] = + Gx1[5]*Gx2[0] + Gx1[6]*Gx2[5] + Gx1[7]*Gx2[10] + Gx1[8]*Gx2[15] + Gx1[9]*Gx2[20];
Gx3[6] = + Gx1[5]*Gx2[1] + Gx1[6]*Gx2[6] + Gx1[7]*Gx2[11] + Gx1[8]*Gx2[16] + Gx1[9]*Gx2[21];
Gx3[7] = + Gx1[5]*Gx2[2] + Gx1[6]*Gx2[7] + Gx1[7]*Gx2[12] + Gx1[8]*Gx2[17] + Gx1[9]*Gx2[22];
Gx3[8] = + Gx1[5]*Gx2[3] + Gx1[6]*Gx2[8] + Gx1[7]*Gx2[13] + Gx1[8]*Gx2[18] + Gx1[9]*Gx2[23];
Gx3[9] = + Gx1[5]*Gx2[4] + Gx1[6]*Gx2[9] + Gx1[7]*Gx2[14] + Gx1[8]*Gx2[19] + Gx1[9]*Gx2[24];
Gx3[10] = + Gx1[10]*Gx2[0] + Gx1[11]*Gx2[5] + Gx1[12]*Gx2[10] + Gx1[13]*Gx2[15] + Gx1[14]*Gx2[20];
Gx3[11] = + Gx1[10]*Gx2[1] + Gx1[11]*Gx2[6] + Gx1[12]*Gx2[11] + Gx1[13]*Gx2[16] + Gx1[14]*Gx2[21];
Gx3[12] = + Gx1[10]*Gx2[2] + Gx1[11]*Gx2[7] + Gx1[12]*Gx2[12] + Gx1[13]*Gx2[17] + Gx1[14]*Gx2[22];
Gx3[13] = + Gx1[10]*Gx2[3] + Gx1[11]*Gx2[8] + Gx1[12]*Gx2[13] + Gx1[13]*Gx2[18] + Gx1[14]*Gx2[23];
Gx3[14] = + Gx1[10]*Gx2[4] + Gx1[11]*Gx2[9] + Gx1[12]*Gx2[14] + Gx1[13]*Gx2[19] + Gx1[14]*Gx2[24];
Gx3[15] = + Gx1[15]*Gx2[0] + Gx1[16]*Gx2[5] + Gx1[17]*Gx2[10] + Gx1[18]*Gx2[15] + Gx1[19]*Gx2[20];
Gx3[16] = + Gx1[15]*Gx2[1] + Gx1[16]*Gx2[6] + Gx1[17]*Gx2[11] + Gx1[18]*Gx2[16] + Gx1[19]*Gx2[21];
Gx3[17] = + Gx1[15]*Gx2[2] + Gx1[16]*Gx2[7] + Gx1[17]*Gx2[12] + Gx1[18]*Gx2[17] + Gx1[19]*Gx2[22];
Gx3[18] = + Gx1[15]*Gx2[3] + Gx1[16]*Gx2[8] + Gx1[17]*Gx2[13] + Gx1[18]*Gx2[18] + Gx1[19]*Gx2[23];
Gx3[19] = + Gx1[15]*Gx2[4] + Gx1[16]*Gx2[9] + Gx1[17]*Gx2[14] + Gx1[18]*Gx2[19] + Gx1[19]*Gx2[24];
Gx3[20] = + Gx1[20]*Gx2[0] + Gx1[21]*Gx2[5] + Gx1[22]*Gx2[10] + Gx1[23]*Gx2[15] + Gx1[24]*Gx2[20];
Gx3[21] = + Gx1[20]*Gx2[1] + Gx1[21]*Gx2[6] + Gx1[22]*Gx2[11] + Gx1[23]*Gx2[16] + Gx1[24]*Gx2[21];
Gx3[22] = + Gx1[20]*Gx2[2] + Gx1[21]*Gx2[7] + Gx1[22]*Gx2[12] + Gx1[23]*Gx2[17] + Gx1[24]*Gx2[22];
Gx3[23] = + Gx1[20]*Gx2[3] + Gx1[21]*Gx2[8] + Gx1[22]*Gx2[13] + Gx1[23]*Gx2[18] + Gx1[24]*Gx2[23];
Gx3[24] = + Gx1[20]*Gx2[4] + Gx1[21]*Gx2[9] + Gx1[22]*Gx2[14] + Gx1[23]*Gx2[19] + Gx1[24]*Gx2[24];
}

void acado_multGxGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0] + Gx1[1]*Gu1[2] + Gx1[2]*Gu1[4] + Gx1[3]*Gu1[6] + Gx1[4]*Gu1[8];
Gu2[1] = + Gx1[0]*Gu1[1] + Gx1[1]*Gu1[3] + Gx1[2]*Gu1[5] + Gx1[3]*Gu1[7] + Gx1[4]*Gu1[9];
Gu2[2] = + Gx1[5]*Gu1[0] + Gx1[6]*Gu1[2] + Gx1[7]*Gu1[4] + Gx1[8]*Gu1[6] + Gx1[9]*Gu1[8];
Gu2[3] = + Gx1[5]*Gu1[1] + Gx1[6]*Gu1[3] + Gx1[7]*Gu1[5] + Gx1[8]*Gu1[7] + Gx1[9]*Gu1[9];
Gu2[4] = + Gx1[10]*Gu1[0] + Gx1[11]*Gu1[2] + Gx1[12]*Gu1[4] + Gx1[13]*Gu1[6] + Gx1[14]*Gu1[8];
Gu2[5] = + Gx1[10]*Gu1[1] + Gx1[11]*Gu1[3] + Gx1[12]*Gu1[5] + Gx1[13]*Gu1[7] + Gx1[14]*Gu1[9];
Gu2[6] = + Gx1[15]*Gu1[0] + Gx1[16]*Gu1[2] + Gx1[17]*Gu1[4] + Gx1[18]*Gu1[6] + Gx1[19]*Gu1[8];
Gu2[7] = + Gx1[15]*Gu1[1] + Gx1[16]*Gu1[3] + Gx1[17]*Gu1[5] + Gx1[18]*Gu1[7] + Gx1[19]*Gu1[9];
Gu2[8] = + Gx1[20]*Gu1[0] + Gx1[21]*Gu1[2] + Gx1[22]*Gu1[4] + Gx1[23]*Gu1[6] + Gx1[24]*Gu1[8];
Gu2[9] = + Gx1[20]*Gu1[1] + Gx1[21]*Gu1[3] + Gx1[22]*Gu1[5] + Gx1[23]*Gu1[7] + Gx1[24]*Gu1[9];
}

void acado_moveGuE( real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = Gu1[0];
Gu2[1] = Gu1[1];
Gu2[2] = Gu1[2];
Gu2[3] = Gu1[3];
Gu2[4] = Gu1[4];
Gu2[5] = Gu1[5];
Gu2[6] = Gu1[6];
Gu2[7] = Gu1[7];
Gu2[8] = Gu1[8];
Gu2[9] = Gu1[9];
}

void acado_multBTW1( real_t* const Gu1, real_t* const Gu2, int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 120) + (iCol * 2)] = + Gu1[0]*Gu2[0] + Gu1[2]*Gu2[2] + Gu1[4]*Gu2[4] + Gu1[6]*Gu2[6] + Gu1[8]*Gu2[8];
acadoWorkspace.H[(iRow * 120) + (iCol * 2 + 1)] = + Gu1[0]*Gu2[1] + Gu1[2]*Gu2[3] + Gu1[4]*Gu2[5] + Gu1[6]*Gu2[7] + Gu1[8]*Gu2[9];
acadoWorkspace.H[(iRow * 120 + 60) + (iCol * 2)] = + Gu1[1]*Gu2[0] + Gu1[3]*Gu2[2] + Gu1[5]*Gu2[4] + Gu1[7]*Gu2[6] + Gu1[9]*Gu2[8];
acadoWorkspace.H[(iRow * 120 + 60) + (iCol * 2 + 1)] = + Gu1[1]*Gu2[1] + Gu1[3]*Gu2[3] + Gu1[5]*Gu2[5] + Gu1[7]*Gu2[7] + Gu1[9]*Gu2[9];
}

void acado_multBTW1_R1( real_t* const R11, real_t* const Gu1, real_t* const Gu2, int iRow )
{
acadoWorkspace.H[iRow * 122] = + Gu1[0]*Gu2[0] + Gu1[2]*Gu2[2] + Gu1[4]*Gu2[4] + Gu1[6]*Gu2[6] + Gu1[8]*Gu2[8] + R11[0];
acadoWorkspace.H[iRow * 122 + 1] = + Gu1[0]*Gu2[1] + Gu1[2]*Gu2[3] + Gu1[4]*Gu2[5] + Gu1[6]*Gu2[7] + Gu1[8]*Gu2[9] + R11[1];
acadoWorkspace.H[iRow * 122 + 60] = + Gu1[1]*Gu2[0] + Gu1[3]*Gu2[2] + Gu1[5]*Gu2[4] + Gu1[7]*Gu2[6] + Gu1[9]*Gu2[8] + R11[2];
acadoWorkspace.H[iRow * 122 + 61] = + Gu1[1]*Gu2[1] + Gu1[3]*Gu2[3] + Gu1[5]*Gu2[5] + Gu1[7]*Gu2[7] + Gu1[9]*Gu2[9] + R11[3];
acadoWorkspace.H[iRow * 122] += 1.0000000000000000e-08;
acadoWorkspace.H[iRow * 122 + 61] += 1.0000000000000000e-08;
}

void acado_multGxTGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0] + Gx1[5]*Gu1[2] + Gx1[10]*Gu1[4] + Gx1[15]*Gu1[6] + Gx1[20]*Gu1[8];
Gu2[1] = + Gx1[0]*Gu1[1] + Gx1[5]*Gu1[3] + Gx1[10]*Gu1[5] + Gx1[15]*Gu1[7] + Gx1[20]*Gu1[9];
Gu2[2] = + Gx1[1]*Gu1[0] + Gx1[6]*Gu1[2] + Gx1[11]*Gu1[4] + Gx1[16]*Gu1[6] + Gx1[21]*Gu1[8];
Gu2[3] = + Gx1[1]*Gu1[1] + Gx1[6]*Gu1[3] + Gx1[11]*Gu1[5] + Gx1[16]*Gu1[7] + Gx1[21]*Gu1[9];
Gu2[4] = + Gx1[2]*Gu1[0] + Gx1[7]*Gu1[2] + Gx1[12]*Gu1[4] + Gx1[17]*Gu1[6] + Gx1[22]*Gu1[8];
Gu2[5] = + Gx1[2]*Gu1[1] + Gx1[7]*Gu1[3] + Gx1[12]*Gu1[5] + Gx1[17]*Gu1[7] + Gx1[22]*Gu1[9];
Gu2[6] = + Gx1[3]*Gu1[0] + Gx1[8]*Gu1[2] + Gx1[13]*Gu1[4] + Gx1[18]*Gu1[6] + Gx1[23]*Gu1[8];
Gu2[7] = + Gx1[3]*Gu1[1] + Gx1[8]*Gu1[3] + Gx1[13]*Gu1[5] + Gx1[18]*Gu1[7] + Gx1[23]*Gu1[9];
Gu2[8] = + Gx1[4]*Gu1[0] + Gx1[9]*Gu1[2] + Gx1[14]*Gu1[4] + Gx1[19]*Gu1[6] + Gx1[24]*Gu1[8];
Gu2[9] = + Gx1[4]*Gu1[1] + Gx1[9]*Gu1[3] + Gx1[14]*Gu1[5] + Gx1[19]*Gu1[7] + Gx1[24]*Gu1[9];
}

void acado_multQEW2( real_t* const Q11, real_t* const Gu1, real_t* const Gu2, real_t* const Gu3 )
{
Gu3[0] = + Q11[0]*Gu1[0] + Q11[1]*Gu1[2] + Q11[2]*Gu1[4] + Q11[3]*Gu1[6] + Q11[4]*Gu1[8] + Gu2[0];
Gu3[1] = + Q11[0]*Gu1[1] + Q11[1]*Gu1[3] + Q11[2]*Gu1[5] + Q11[3]*Gu1[7] + Q11[4]*Gu1[9] + Gu2[1];
Gu3[2] = + Q11[5]*Gu1[0] + Q11[6]*Gu1[2] + Q11[7]*Gu1[4] + Q11[8]*Gu1[6] + Q11[9]*Gu1[8] + Gu2[2];
Gu3[3] = + Q11[5]*Gu1[1] + Q11[6]*Gu1[3] + Q11[7]*Gu1[5] + Q11[8]*Gu1[7] + Q11[9]*Gu1[9] + Gu2[3];
Gu3[4] = + Q11[10]*Gu1[0] + Q11[11]*Gu1[2] + Q11[12]*Gu1[4] + Q11[13]*Gu1[6] + Q11[14]*Gu1[8] + Gu2[4];
Gu3[5] = + Q11[10]*Gu1[1] + Q11[11]*Gu1[3] + Q11[12]*Gu1[5] + Q11[13]*Gu1[7] + Q11[14]*Gu1[9] + Gu2[5];
Gu3[6] = + Q11[15]*Gu1[0] + Q11[16]*Gu1[2] + Q11[17]*Gu1[4] + Q11[18]*Gu1[6] + Q11[19]*Gu1[8] + Gu2[6];
Gu3[7] = + Q11[15]*Gu1[1] + Q11[16]*Gu1[3] + Q11[17]*Gu1[5] + Q11[18]*Gu1[7] + Q11[19]*Gu1[9] + Gu2[7];
Gu3[8] = + Q11[20]*Gu1[0] + Q11[21]*Gu1[2] + Q11[22]*Gu1[4] + Q11[23]*Gu1[6] + Q11[24]*Gu1[8] + Gu2[8];
Gu3[9] = + Q11[20]*Gu1[1] + Q11[21]*Gu1[3] + Q11[22]*Gu1[5] + Q11[23]*Gu1[7] + Q11[24]*Gu1[9] + Gu2[9];
}

void acado_macATw1QDy( real_t* const Gx1, real_t* const w11, real_t* const w12, real_t* const w13 )
{
w13[0] = + Gx1[0]*w11[0] + Gx1[5]*w11[1] + Gx1[10]*w11[2] + Gx1[15]*w11[3] + Gx1[20]*w11[4] + w12[0];
w13[1] = + Gx1[1]*w11[0] + Gx1[6]*w11[1] + Gx1[11]*w11[2] + Gx1[16]*w11[3] + Gx1[21]*w11[4] + w12[1];
w13[2] = + Gx1[2]*w11[0] + Gx1[7]*w11[1] + Gx1[12]*w11[2] + Gx1[17]*w11[3] + Gx1[22]*w11[4] + w12[2];
w13[3] = + Gx1[3]*w11[0] + Gx1[8]*w11[1] + Gx1[13]*w11[2] + Gx1[18]*w11[3] + Gx1[23]*w11[4] + w12[3];
w13[4] = + Gx1[4]*w11[0] + Gx1[9]*w11[1] + Gx1[14]*w11[2] + Gx1[19]*w11[3] + Gx1[24]*w11[4] + w12[4];
}

void acado_macBTw1( real_t* const Gu1, real_t* const w11, real_t* const U1 )
{
U1[0] += + Gu1[0]*w11[0] + Gu1[2]*w11[1] + Gu1[4]*w11[2] + Gu1[6]*w11[3] + Gu1[8]*w11[4];
U1[1] += + Gu1[1]*w11[0] + Gu1[3]*w11[1] + Gu1[5]*w11[2] + Gu1[7]*w11[3] + Gu1[9]*w11[4];
}

void acado_macQSbarW2( real_t* const Q11, real_t* const w11, real_t* const w12, real_t* const w13 )
{
w13[0] = + Q11[0]*w11[0] + Q11[1]*w11[1] + Q11[2]*w11[2] + Q11[3]*w11[3] + Q11[4]*w11[4] + w12[0];
w13[1] = + Q11[5]*w11[0] + Q11[6]*w11[1] + Q11[7]*w11[2] + Q11[8]*w11[3] + Q11[9]*w11[4] + w12[1];
w13[2] = + Q11[10]*w11[0] + Q11[11]*w11[1] + Q11[12]*w11[2] + Q11[13]*w11[3] + Q11[14]*w11[4] + w12[2];
w13[3] = + Q11[15]*w11[0] + Q11[16]*w11[1] + Q11[17]*w11[2] + Q11[18]*w11[3] + Q11[19]*w11[4] + w12[3];
w13[4] = + Q11[20]*w11[0] + Q11[21]*w11[1] + Q11[22]*w11[2] + Q11[23]*w11[3] + Q11[24]*w11[4] + w12[4];
}

void acado_macASbar( real_t* const Gx1, real_t* const w11, real_t* const w12 )
{
w12[0] += + Gx1[0]*w11[0] + Gx1[1]*w11[1] + Gx1[2]*w11[2] + Gx1[3]*w11[3] + Gx1[4]*w11[4];
w12[1] += + Gx1[5]*w11[0] + Gx1[6]*w11[1] + Gx1[7]*w11[2] + Gx1[8]*w11[3] + Gx1[9]*w11[4];
w12[2] += + Gx1[10]*w11[0] + Gx1[11]*w11[1] + Gx1[12]*w11[2] + Gx1[13]*w11[3] + Gx1[14]*w11[4];
w12[3] += + Gx1[15]*w11[0] + Gx1[16]*w11[1] + Gx1[17]*w11[2] + Gx1[18]*w11[3] + Gx1[19]*w11[4];
w12[4] += + Gx1[20]*w11[0] + Gx1[21]*w11[1] + Gx1[22]*w11[2] + Gx1[23]*w11[3] + Gx1[24]*w11[4];
}

void acado_expansionStep( real_t* const Gx1, real_t* const Gu1, real_t* const U1, real_t* const w11, real_t* const w12 )
{
w12[0] += + Gx1[0]*w11[0] + Gx1[1]*w11[1] + Gx1[2]*w11[2] + Gx1[3]*w11[3] + Gx1[4]*w11[4];
w12[1] += + Gx1[5]*w11[0] + Gx1[6]*w11[1] + Gx1[7]*w11[2] + Gx1[8]*w11[3] + Gx1[9]*w11[4];
w12[2] += + Gx1[10]*w11[0] + Gx1[11]*w11[1] + Gx1[12]*w11[2] + Gx1[13]*w11[3] + Gx1[14]*w11[4];
w12[3] += + Gx1[15]*w11[0] + Gx1[16]*w11[1] + Gx1[17]*w11[2] + Gx1[18]*w11[3] + Gx1[19]*w11[4];
w12[4] += + Gx1[20]*w11[0] + Gx1[21]*w11[1] + Gx1[22]*w11[2] + Gx1[23]*w11[3] + Gx1[24]*w11[4];
w12[0] += + Gu1[0]*U1[0] + Gu1[1]*U1[1];
w12[1] += + Gu1[2]*U1[0] + Gu1[3]*U1[1];
w12[2] += + Gu1[4]*U1[0] + Gu1[5]*U1[1];
w12[3] += + Gu1[6]*U1[0] + Gu1[7]*U1[1];
w12[4] += + Gu1[8]*U1[0] + Gu1[9]*U1[1];
}

void acado_copyHTH( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 120) + (iCol * 2)] = acadoWorkspace.H[(iCol * 120) + (iRow * 2)];
acadoWorkspace.H[(iRow * 120) + (iCol * 2 + 1)] = acadoWorkspace.H[(iCol * 120 + 60) + (iRow * 2)];
acadoWorkspace.H[(iRow * 120 + 60) + (iCol * 2)] = acadoWorkspace.H[(iCol * 120) + (iRow * 2 + 1)];
acadoWorkspace.H[(iRow * 120 + 60) + (iCol * 2 + 1)] = acadoWorkspace.H[(iCol * 120 + 60) + (iRow * 2 + 1)];
}

void acado_multRDy( real_t* const R2, real_t* const Dy1, real_t* const RDy1 )
{
RDy1[0] = + R2[0]*Dy1[0] + R2[1]*Dy1[1] + R2[2]*Dy1[2] + R2[3]*Dy1[3] + R2[4]*Dy1[4] + R2[5]*Dy1[5];
RDy1[1] = + R2[6]*Dy1[0] + R2[7]*Dy1[1] + R2[8]*Dy1[2] + R2[9]*Dy1[3] + R2[10]*Dy1[4] + R2[11]*Dy1[5];
}

void acado_multQDy( real_t* const Q2, real_t* const Dy1, real_t* const QDy1 )
{
QDy1[0] = + Q2[0]*Dy1[0] + Q2[1]*Dy1[1] + Q2[2]*Dy1[2] + Q2[3]*Dy1[3] + Q2[4]*Dy1[4] + Q2[5]*Dy1[5];
QDy1[1] = + Q2[6]*Dy1[0] + Q2[7]*Dy1[1] + Q2[8]*Dy1[2] + Q2[9]*Dy1[3] + Q2[10]*Dy1[4] + Q2[11]*Dy1[5];
QDy1[2] = + Q2[12]*Dy1[0] + Q2[13]*Dy1[1] + Q2[14]*Dy1[2] + Q2[15]*Dy1[3] + Q2[16]*Dy1[4] + Q2[17]*Dy1[5];
QDy1[3] = + Q2[18]*Dy1[0] + Q2[19]*Dy1[1] + Q2[20]*Dy1[2] + Q2[21]*Dy1[3] + Q2[22]*Dy1[4] + Q2[23]*Dy1[5];
QDy1[4] = + Q2[24]*Dy1[0] + Q2[25]*Dy1[1] + Q2[26]*Dy1[2] + Q2[27]*Dy1[3] + Q2[28]*Dy1[4] + Q2[29]*Dy1[5];
}

void acado_multHxE( real_t* const Hx, real_t* const E, int row, int col )
{
acadoWorkspace.A[(row * 60) + (col * 2)] = + Hx[0]*E[0] + Hx[1]*E[2] + Hx[2]*E[4] + Hx[3]*E[6] + Hx[4]*E[8];
acadoWorkspace.A[(row * 60) + (col * 2 + 1)] = + Hx[0]*E[1] + Hx[1]*E[3] + Hx[2]*E[5] + Hx[3]*E[7] + Hx[4]*E[9];
}

void acado_macHxd( real_t* const Hx, real_t* const tmpd, real_t* const lbA, real_t* const ubA )
{
acadoWorkspace.evHxd[0] = + Hx[0]*tmpd[0] + Hx[1]*tmpd[1] + Hx[2]*tmpd[2] + Hx[3]*tmpd[3] + Hx[4]*tmpd[4];
lbA[0] -= acadoWorkspace.evHxd[0];
ubA[0] -= acadoWorkspace.evHxd[0];
}

void acado_condensePrep(  )
{
int lRun1;
int lRun2;
int lRun3;
int lRun4;
int lRun5;
acado_moveGxT( acadoWorkspace.evGx, acadoWorkspace.C );
acado_multGxGx( &(acadoWorkspace.evGx[ 25 ]), acadoWorkspace.C, &(acadoWorkspace.C[ 25 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 50 ]), &(acadoWorkspace.C[ 25 ]), &(acadoWorkspace.C[ 50 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 75 ]), &(acadoWorkspace.C[ 50 ]), &(acadoWorkspace.C[ 75 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 100 ]), &(acadoWorkspace.C[ 75 ]), &(acadoWorkspace.C[ 100 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 125 ]), &(acadoWorkspace.C[ 100 ]), &(acadoWorkspace.C[ 125 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 150 ]), &(acadoWorkspace.C[ 125 ]), &(acadoWorkspace.C[ 150 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 175 ]), &(acadoWorkspace.C[ 150 ]), &(acadoWorkspace.C[ 175 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 200 ]), &(acadoWorkspace.C[ 175 ]), &(acadoWorkspace.C[ 200 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 225 ]), &(acadoWorkspace.C[ 200 ]), &(acadoWorkspace.C[ 225 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 250 ]), &(acadoWorkspace.C[ 225 ]), &(acadoWorkspace.C[ 250 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 275 ]), &(acadoWorkspace.C[ 250 ]), &(acadoWorkspace.C[ 275 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 300 ]), &(acadoWorkspace.C[ 275 ]), &(acadoWorkspace.C[ 300 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 325 ]), &(acadoWorkspace.C[ 300 ]), &(acadoWorkspace.C[ 325 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 350 ]), &(acadoWorkspace.C[ 325 ]), &(acadoWorkspace.C[ 350 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 375 ]), &(acadoWorkspace.C[ 350 ]), &(acadoWorkspace.C[ 375 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 400 ]), &(acadoWorkspace.C[ 375 ]), &(acadoWorkspace.C[ 400 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 425 ]), &(acadoWorkspace.C[ 400 ]), &(acadoWorkspace.C[ 425 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 450 ]), &(acadoWorkspace.C[ 425 ]), &(acadoWorkspace.C[ 450 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 475 ]), &(acadoWorkspace.C[ 450 ]), &(acadoWorkspace.C[ 475 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 500 ]), &(acadoWorkspace.C[ 475 ]), &(acadoWorkspace.C[ 500 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 525 ]), &(acadoWorkspace.C[ 500 ]), &(acadoWorkspace.C[ 525 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 550 ]), &(acadoWorkspace.C[ 525 ]), &(acadoWorkspace.C[ 550 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 575 ]), &(acadoWorkspace.C[ 550 ]), &(acadoWorkspace.C[ 575 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 600 ]), &(acadoWorkspace.C[ 575 ]), &(acadoWorkspace.C[ 600 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 625 ]), &(acadoWorkspace.C[ 600 ]), &(acadoWorkspace.C[ 625 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 650 ]), &(acadoWorkspace.C[ 625 ]), &(acadoWorkspace.C[ 650 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 675 ]), &(acadoWorkspace.C[ 650 ]), &(acadoWorkspace.C[ 675 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 700 ]), &(acadoWorkspace.C[ 675 ]), &(acadoWorkspace.C[ 700 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 725 ]), &(acadoWorkspace.C[ 700 ]), &(acadoWorkspace.C[ 725 ]) );
for (lRun2 = 0; lRun2 < 30; ++lRun2)
{
lRun3 = ((lRun2) * (lRun2 * -1 + 61)) / (2);
acado_moveGuE( &(acadoWorkspace.evGu[ lRun2 * 10 ]), &(acadoWorkspace.E[ lRun3 * 10 ]) );
for (lRun1 = 1; lRun1 < lRun2 * -1 + 30; ++lRun1)
{
acado_multGxGu( &(acadoWorkspace.evGx[ ((((lRun2) + (lRun1)) * (5)) * (5)) + (0) ]), &(acadoWorkspace.E[ (((((lRun3) + (lRun1)) - (1)) * (5)) * (2)) + (0) ]), &(acadoWorkspace.E[ ((((lRun3) + (lRun1)) * (5)) * (2)) + (0) ]) );
}

acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ ((((((lRun3) - (lRun2)) + (30)) - (1)) * (5)) * (2)) + (0) ]), acadoWorkspace.W1 );
for (lRun1 = 29; lRun2 < lRun1; --lRun1)
{
acado_multBTW1( &(acadoWorkspace.evGu[ lRun1 * 10 ]), acadoWorkspace.W1, lRun1, lRun2 );
acado_multGxTGu( &(acadoWorkspace.evGx[ lRun1 * 25 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ lRun1 * 25 ]), &(acadoWorkspace.E[ ((((((lRun3) + (lRun1)) - (lRun2)) - (1)) * (5)) * (2)) + (0) ]), acadoWorkspace.W2, acadoWorkspace.W1 );
}
acado_multBTW1_R1( &(acadoWorkspace.R1[ lRun2 * 4 ]), &(acadoWorkspace.evGu[ lRun2 * 10 ]), acadoWorkspace.W1, lRun2 );
}

for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
for (lRun2 = 0; lRun2 < lRun1; ++lRun2)
{
acado_copyHTH( lRun2, lRun1 );
}
}

for (lRun2 = 0; lRun2 < 150; ++lRun2)
acadoWorkspace.sbar[lRun2 + 5] = acadoWorkspace.d[lRun2];


for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
acadoWorkspace.conValueIn[0] = acadoVariables.x[lRun1 * 5];
acadoWorkspace.conValueIn[1] = acadoVariables.x[lRun1 * 5 + 1];
acadoWorkspace.conValueIn[2] = acadoVariables.x[lRun1 * 5 + 2];
acadoWorkspace.conValueIn[3] = acadoVariables.x[lRun1 * 5 + 3];
acadoWorkspace.conValueIn[4] = acadoVariables.x[lRun1 * 5 + 4];
acadoWorkspace.conValueIn[5] = acadoVariables.u[lRun1 * 2];
acadoWorkspace.conValueIn[6] = acadoVariables.u[lRun1 * 2 + 1];
acadoWorkspace.conValueIn[7] = acadoVariables.od[lRun1 * 10];
acadoWorkspace.conValueIn[8] = acadoVariables.od[lRun1 * 10 + 1];
acadoWorkspace.conValueIn[9] = acadoVariables.od[lRun1 * 10 + 2];
acadoWorkspace.conValueIn[10] = acadoVariables.od[lRun1 * 10 + 3];
acadoWorkspace.conValueIn[11] = acadoVariables.od[lRun1 * 10 + 4];
acadoWorkspace.conValueIn[12] = acadoVariables.od[lRun1 * 10 + 5];
acadoWorkspace.conValueIn[13] = acadoVariables.od[lRun1 * 10 + 6];
acadoWorkspace.conValueIn[14] = acadoVariables.od[lRun1 * 10 + 7];
acadoWorkspace.conValueIn[15] = acadoVariables.od[lRun1 * 10 + 8];
acadoWorkspace.conValueIn[16] = acadoVariables.od[lRun1 * 10 + 9];
acado_evaluatePathConstraints( acadoWorkspace.conValueIn, acadoWorkspace.conValueOut );
acadoWorkspace.evH[lRun1] = acadoWorkspace.conValueOut[0];

acadoWorkspace.evHx[lRun1 * 5] = acadoWorkspace.conValueOut[1];
acadoWorkspace.evHx[lRun1 * 5 + 1] = acadoWorkspace.conValueOut[2];
acadoWorkspace.evHx[lRun1 * 5 + 2] = acadoWorkspace.conValueOut[3];
acadoWorkspace.evHx[lRun1 * 5 + 3] = acadoWorkspace.conValueOut[4];
acadoWorkspace.evHx[lRun1 * 5 + 4] = acadoWorkspace.conValueOut[5];
acadoWorkspace.evHu[lRun1 * 2] = acadoWorkspace.conValueOut[6];
acadoWorkspace.evHu[lRun1 * 2 + 1] = acadoWorkspace.conValueOut[7];
}



for (lRun2 = 0; lRun2 < 29; ++lRun2)
{
for (lRun3 = 0; lRun3 < lRun2 + 1; ++lRun3)
{
lRun4 = (((lRun3) * (lRun3 * -1 + 59)) / (2)) + (lRun2);
lRun5 = lRun2 + 1;
acado_multHxE( &(acadoWorkspace.evHx[ lRun2 * 5 + 5 ]), &(acadoWorkspace.E[ lRun4 * 10 ]), lRun5, lRun3 );
}
}

acadoWorkspace.A[0] = acadoWorkspace.evHu[0];
acadoWorkspace.A[1] = acadoWorkspace.evHu[1];
acadoWorkspace.A[62] = acadoWorkspace.evHu[2];
acadoWorkspace.A[63] = acadoWorkspace.evHu[3];
acadoWorkspace.A[124] = acadoWorkspace.evHu[4];
acadoWorkspace.A[125] = acadoWorkspace.evHu[5];
acadoWorkspace.A[186] = acadoWorkspace.evHu[6];
acadoWorkspace.A[187] = acadoWorkspace.evHu[7];
acadoWorkspace.A[248] = acadoWorkspace.evHu[8];
acadoWorkspace.A[249] = acadoWorkspace.evHu[9];
acadoWorkspace.A[310] = acadoWorkspace.evHu[10];
acadoWorkspace.A[311] = acadoWorkspace.evHu[11];
acadoWorkspace.A[372] = acadoWorkspace.evHu[12];
acadoWorkspace.A[373] = acadoWorkspace.evHu[13];
acadoWorkspace.A[434] = acadoWorkspace.evHu[14];
acadoWorkspace.A[435] = acadoWorkspace.evHu[15];
acadoWorkspace.A[496] = acadoWorkspace.evHu[16];
acadoWorkspace.A[497] = acadoWorkspace.evHu[17];
acadoWorkspace.A[558] = acadoWorkspace.evHu[18];
acadoWorkspace.A[559] = acadoWorkspace.evHu[19];
acadoWorkspace.A[620] = acadoWorkspace.evHu[20];
acadoWorkspace.A[621] = acadoWorkspace.evHu[21];
acadoWorkspace.A[682] = acadoWorkspace.evHu[22];
acadoWorkspace.A[683] = acadoWorkspace.evHu[23];
acadoWorkspace.A[744] = acadoWorkspace.evHu[24];
acadoWorkspace.A[745] = acadoWorkspace.evHu[25];
acadoWorkspace.A[806] = acadoWorkspace.evHu[26];
acadoWorkspace.A[807] = acadoWorkspace.evHu[27];
acadoWorkspace.A[868] = acadoWorkspace.evHu[28];
acadoWorkspace.A[869] = acadoWorkspace.evHu[29];
acadoWorkspace.A[930] = acadoWorkspace.evHu[30];
acadoWorkspace.A[931] = acadoWorkspace.evHu[31];
acadoWorkspace.A[992] = acadoWorkspace.evHu[32];
acadoWorkspace.A[993] = acadoWorkspace.evHu[33];
acadoWorkspace.A[1054] = acadoWorkspace.evHu[34];
acadoWorkspace.A[1055] = acadoWorkspace.evHu[35];
acadoWorkspace.A[1116] = acadoWorkspace.evHu[36];
acadoWorkspace.A[1117] = acadoWorkspace.evHu[37];
acadoWorkspace.A[1178] = acadoWorkspace.evHu[38];
acadoWorkspace.A[1179] = acadoWorkspace.evHu[39];
acadoWorkspace.A[1240] = acadoWorkspace.evHu[40];
acadoWorkspace.A[1241] = acadoWorkspace.evHu[41];
acadoWorkspace.A[1302] = acadoWorkspace.evHu[42];
acadoWorkspace.A[1303] = acadoWorkspace.evHu[43];
acadoWorkspace.A[1364] = acadoWorkspace.evHu[44];
acadoWorkspace.A[1365] = acadoWorkspace.evHu[45];
acadoWorkspace.A[1426] = acadoWorkspace.evHu[46];
acadoWorkspace.A[1427] = acadoWorkspace.evHu[47];
acadoWorkspace.A[1488] = acadoWorkspace.evHu[48];
acadoWorkspace.A[1489] = acadoWorkspace.evHu[49];
acadoWorkspace.A[1550] = acadoWorkspace.evHu[50];
acadoWorkspace.A[1551] = acadoWorkspace.evHu[51];
acadoWorkspace.A[1612] = acadoWorkspace.evHu[52];
acadoWorkspace.A[1613] = acadoWorkspace.evHu[53];
acadoWorkspace.A[1674] = acadoWorkspace.evHu[54];
acadoWorkspace.A[1675] = acadoWorkspace.evHu[55];
acadoWorkspace.A[1736] = acadoWorkspace.evHu[56];
acadoWorkspace.A[1737] = acadoWorkspace.evHu[57];
acadoWorkspace.A[1798] = acadoWorkspace.evHu[58];
acadoWorkspace.A[1799] = acadoWorkspace.evHu[59];
acadoWorkspace.lbA[0] = acadoVariables.lbAValues[0] - acadoWorkspace.evH[0];
acadoWorkspace.lbA[1] = acadoVariables.lbAValues[1] - acadoWorkspace.evH[1];
acadoWorkspace.lbA[2] = acadoVariables.lbAValues[2] - acadoWorkspace.evH[2];
acadoWorkspace.lbA[3] = acadoVariables.lbAValues[3] - acadoWorkspace.evH[3];
acadoWorkspace.lbA[4] = acadoVariables.lbAValues[4] - acadoWorkspace.evH[4];
acadoWorkspace.lbA[5] = acadoVariables.lbAValues[5] - acadoWorkspace.evH[5];
acadoWorkspace.lbA[6] = acadoVariables.lbAValues[6] - acadoWorkspace.evH[6];
acadoWorkspace.lbA[7] = acadoVariables.lbAValues[7] - acadoWorkspace.evH[7];
acadoWorkspace.lbA[8] = acadoVariables.lbAValues[8] - acadoWorkspace.evH[8];
acadoWorkspace.lbA[9] = acadoVariables.lbAValues[9] - acadoWorkspace.evH[9];
acadoWorkspace.lbA[10] = acadoVariables.lbAValues[10] - acadoWorkspace.evH[10];
acadoWorkspace.lbA[11] = acadoVariables.lbAValues[11] - acadoWorkspace.evH[11];
acadoWorkspace.lbA[12] = acadoVariables.lbAValues[12] - acadoWorkspace.evH[12];
acadoWorkspace.lbA[13] = acadoVariables.lbAValues[13] - acadoWorkspace.evH[13];
acadoWorkspace.lbA[14] = acadoVariables.lbAValues[14] - acadoWorkspace.evH[14];
acadoWorkspace.lbA[15] = acadoVariables.lbAValues[15] - acadoWorkspace.evH[15];
acadoWorkspace.lbA[16] = acadoVariables.lbAValues[16] - acadoWorkspace.evH[16];
acadoWorkspace.lbA[17] = acadoVariables.lbAValues[17] - acadoWorkspace.evH[17];
acadoWorkspace.lbA[18] = acadoVariables.lbAValues[18] - acadoWorkspace.evH[18];
acadoWorkspace.lbA[19] = acadoVariables.lbAValues[19] - acadoWorkspace.evH[19];
acadoWorkspace.lbA[20] = acadoVariables.lbAValues[20] - acadoWorkspace.evH[20];
acadoWorkspace.lbA[21] = acadoVariables.lbAValues[21] - acadoWorkspace.evH[21];
acadoWorkspace.lbA[22] = acadoVariables.lbAValues[22] - acadoWorkspace.evH[22];
acadoWorkspace.lbA[23] = acadoVariables.lbAValues[23] - acadoWorkspace.evH[23];
acadoWorkspace.lbA[24] = acadoVariables.lbAValues[24] - acadoWorkspace.evH[24];
acadoWorkspace.lbA[25] = acadoVariables.lbAValues[25] - acadoWorkspace.evH[25];
acadoWorkspace.lbA[26] = acadoVariables.lbAValues[26] - acadoWorkspace.evH[26];
acadoWorkspace.lbA[27] = acadoVariables.lbAValues[27] - acadoWorkspace.evH[27];
acadoWorkspace.lbA[28] = acadoVariables.lbAValues[28] - acadoWorkspace.evH[28];
acadoWorkspace.lbA[29] = acadoVariables.lbAValues[29] - acadoWorkspace.evH[29];

acadoWorkspace.ubA[0] = acadoVariables.ubAValues[0] - acadoWorkspace.evH[0];
acadoWorkspace.ubA[1] = acadoVariables.ubAValues[1] - acadoWorkspace.evH[1];
acadoWorkspace.ubA[2] = acadoVariables.ubAValues[2] - acadoWorkspace.evH[2];
acadoWorkspace.ubA[3] = acadoVariables.ubAValues[3] - acadoWorkspace.evH[3];
acadoWorkspace.ubA[4] = acadoVariables.ubAValues[4] - acadoWorkspace.evH[4];
acadoWorkspace.ubA[5] = acadoVariables.ubAValues[5] - acadoWorkspace.evH[5];
acadoWorkspace.ubA[6] = acadoVariables.ubAValues[6] - acadoWorkspace.evH[6];
acadoWorkspace.ubA[7] = acadoVariables.ubAValues[7] - acadoWorkspace.evH[7];
acadoWorkspace.ubA[8] = acadoVariables.ubAValues[8] - acadoWorkspace.evH[8];
acadoWorkspace.ubA[9] = acadoVariables.ubAValues[9] - acadoWorkspace.evH[9];
acadoWorkspace.ubA[10] = acadoVariables.ubAValues[10] - acadoWorkspace.evH[10];
acadoWorkspace.ubA[11] = acadoVariables.ubAValues[11] - acadoWorkspace.evH[11];
acadoWorkspace.ubA[12] = acadoVariables.ubAValues[12] - acadoWorkspace.evH[12];
acadoWorkspace.ubA[13] = acadoVariables.ubAValues[13] - acadoWorkspace.evH[13];
acadoWorkspace.ubA[14] = acadoVariables.ubAValues[14] - acadoWorkspace.evH[14];
acadoWorkspace.ubA[15] = acadoVariables.ubAValues[15] - acadoWorkspace.evH[15];
acadoWorkspace.ubA[16] = acadoVariables.ubAValues[16] - acadoWorkspace.evH[16];
acadoWorkspace.ubA[17] = acadoVariables.ubAValues[17] - acadoWorkspace.evH[17];
acadoWorkspace.ubA[18] = acadoVariables.ubAValues[18] - acadoWorkspace.evH[18];
acadoWorkspace.ubA[19] = acadoVariables.ubAValues[19] - acadoWorkspace.evH[19];
acadoWorkspace.ubA[20] = acadoVariables.ubAValues[20] - acadoWorkspace.evH[20];
acadoWorkspace.ubA[21] = acadoVariables.ubAValues[21] - acadoWorkspace.evH[21];
acadoWorkspace.ubA[22] = acadoVariables.ubAValues[22] - acadoWorkspace.evH[22];
acadoWorkspace.ubA[23] = acadoVariables.ubAValues[23] - acadoWorkspace.evH[23];
acadoWorkspace.ubA[24] = acadoVariables.ubAValues[24] - acadoWorkspace.evH[24];
acadoWorkspace.ubA[25] = acadoVariables.ubAValues[25] - acadoWorkspace.evH[25];
acadoWorkspace.ubA[26] = acadoVariables.ubAValues[26] - acadoWorkspace.evH[26];
acadoWorkspace.ubA[27] = acadoVariables.ubAValues[27] - acadoWorkspace.evH[27];
acadoWorkspace.ubA[28] = acadoVariables.ubAValues[28] - acadoWorkspace.evH[28];
acadoWorkspace.ubA[29] = acadoVariables.ubAValues[29] - acadoWorkspace.evH[29];

}

void acado_condenseFdb(  )
{
int lRun1;
acadoWorkspace.Dx0[0] = acadoVariables.x0[0] - acadoVariables.x[0];
acadoWorkspace.Dx0[1] = acadoVariables.x0[1] - acadoVariables.x[1];
acadoWorkspace.Dx0[2] = acadoVariables.x0[2] - acadoVariables.x[2];
acadoWorkspace.Dx0[3] = acadoVariables.x0[3] - acadoVariables.x[3];
acadoWorkspace.Dx0[4] = acadoVariables.x0[4] - acadoVariables.x[4];
for (lRun1 = 0; lRun1 < 180; ++lRun1)
acadoWorkspace.Dy[lRun1] -= acadoVariables.y[lRun1];

acadoWorkspace.DyN[0] -= acadoVariables.yN[0];
acadoWorkspace.DyN[1] -= acadoVariables.yN[1];
acadoWorkspace.DyN[2] -= acadoVariables.yN[2];
acadoWorkspace.DyN[3] -= acadoVariables.yN[3];

acado_multRDy( acadoWorkspace.R2, acadoWorkspace.Dy, acadoWorkspace.g );
acado_multRDy( &(acadoWorkspace.R2[ 12 ]), &(acadoWorkspace.Dy[ 6 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 24 ]), &(acadoWorkspace.Dy[ 12 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 36 ]), &(acadoWorkspace.Dy[ 18 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 48 ]), &(acadoWorkspace.Dy[ 24 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 60 ]), &(acadoWorkspace.Dy[ 30 ]), &(acadoWorkspace.g[ 10 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 72 ]), &(acadoWorkspace.Dy[ 36 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 84 ]), &(acadoWorkspace.Dy[ 42 ]), &(acadoWorkspace.g[ 14 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 96 ]), &(acadoWorkspace.Dy[ 48 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 108 ]), &(acadoWorkspace.Dy[ 54 ]), &(acadoWorkspace.g[ 18 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 120 ]), &(acadoWorkspace.Dy[ 60 ]), &(acadoWorkspace.g[ 20 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 132 ]), &(acadoWorkspace.Dy[ 66 ]), &(acadoWorkspace.g[ 22 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 144 ]), &(acadoWorkspace.Dy[ 72 ]), &(acadoWorkspace.g[ 24 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 156 ]), &(acadoWorkspace.Dy[ 78 ]), &(acadoWorkspace.g[ 26 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 168 ]), &(acadoWorkspace.Dy[ 84 ]), &(acadoWorkspace.g[ 28 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 180 ]), &(acadoWorkspace.Dy[ 90 ]), &(acadoWorkspace.g[ 30 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 192 ]), &(acadoWorkspace.Dy[ 96 ]), &(acadoWorkspace.g[ 32 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 204 ]), &(acadoWorkspace.Dy[ 102 ]), &(acadoWorkspace.g[ 34 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 216 ]), &(acadoWorkspace.Dy[ 108 ]), &(acadoWorkspace.g[ 36 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 228 ]), &(acadoWorkspace.Dy[ 114 ]), &(acadoWorkspace.g[ 38 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 240 ]), &(acadoWorkspace.Dy[ 120 ]), &(acadoWorkspace.g[ 40 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 252 ]), &(acadoWorkspace.Dy[ 126 ]), &(acadoWorkspace.g[ 42 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 264 ]), &(acadoWorkspace.Dy[ 132 ]), &(acadoWorkspace.g[ 44 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 276 ]), &(acadoWorkspace.Dy[ 138 ]), &(acadoWorkspace.g[ 46 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 288 ]), &(acadoWorkspace.Dy[ 144 ]), &(acadoWorkspace.g[ 48 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 300 ]), &(acadoWorkspace.Dy[ 150 ]), &(acadoWorkspace.g[ 50 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 312 ]), &(acadoWorkspace.Dy[ 156 ]), &(acadoWorkspace.g[ 52 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 324 ]), &(acadoWorkspace.Dy[ 162 ]), &(acadoWorkspace.g[ 54 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 336 ]), &(acadoWorkspace.Dy[ 168 ]), &(acadoWorkspace.g[ 56 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 348 ]), &(acadoWorkspace.Dy[ 174 ]), &(acadoWorkspace.g[ 58 ]) );

acado_multQDy( acadoWorkspace.Q2, acadoWorkspace.Dy, acadoWorkspace.QDy );
acado_multQDy( &(acadoWorkspace.Q2[ 30 ]), &(acadoWorkspace.Dy[ 6 ]), &(acadoWorkspace.QDy[ 5 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 60 ]), &(acadoWorkspace.Dy[ 12 ]), &(acadoWorkspace.QDy[ 10 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 90 ]), &(acadoWorkspace.Dy[ 18 ]), &(acadoWorkspace.QDy[ 15 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 120 ]), &(acadoWorkspace.Dy[ 24 ]), &(acadoWorkspace.QDy[ 20 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 150 ]), &(acadoWorkspace.Dy[ 30 ]), &(acadoWorkspace.QDy[ 25 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 180 ]), &(acadoWorkspace.Dy[ 36 ]), &(acadoWorkspace.QDy[ 30 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 210 ]), &(acadoWorkspace.Dy[ 42 ]), &(acadoWorkspace.QDy[ 35 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 240 ]), &(acadoWorkspace.Dy[ 48 ]), &(acadoWorkspace.QDy[ 40 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 270 ]), &(acadoWorkspace.Dy[ 54 ]), &(acadoWorkspace.QDy[ 45 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 300 ]), &(acadoWorkspace.Dy[ 60 ]), &(acadoWorkspace.QDy[ 50 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 330 ]), &(acadoWorkspace.Dy[ 66 ]), &(acadoWorkspace.QDy[ 55 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 360 ]), &(acadoWorkspace.Dy[ 72 ]), &(acadoWorkspace.QDy[ 60 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 390 ]), &(acadoWorkspace.Dy[ 78 ]), &(acadoWorkspace.QDy[ 65 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 420 ]), &(acadoWorkspace.Dy[ 84 ]), &(acadoWorkspace.QDy[ 70 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 450 ]), &(acadoWorkspace.Dy[ 90 ]), &(acadoWorkspace.QDy[ 75 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 480 ]), &(acadoWorkspace.Dy[ 96 ]), &(acadoWorkspace.QDy[ 80 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 510 ]), &(acadoWorkspace.Dy[ 102 ]), &(acadoWorkspace.QDy[ 85 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 540 ]), &(acadoWorkspace.Dy[ 108 ]), &(acadoWorkspace.QDy[ 90 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 570 ]), &(acadoWorkspace.Dy[ 114 ]), &(acadoWorkspace.QDy[ 95 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 600 ]), &(acadoWorkspace.Dy[ 120 ]), &(acadoWorkspace.QDy[ 100 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 630 ]), &(acadoWorkspace.Dy[ 126 ]), &(acadoWorkspace.QDy[ 105 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 660 ]), &(acadoWorkspace.Dy[ 132 ]), &(acadoWorkspace.QDy[ 110 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 690 ]), &(acadoWorkspace.Dy[ 138 ]), &(acadoWorkspace.QDy[ 115 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 720 ]), &(acadoWorkspace.Dy[ 144 ]), &(acadoWorkspace.QDy[ 120 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 750 ]), &(acadoWorkspace.Dy[ 150 ]), &(acadoWorkspace.QDy[ 125 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 780 ]), &(acadoWorkspace.Dy[ 156 ]), &(acadoWorkspace.QDy[ 130 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 810 ]), &(acadoWorkspace.Dy[ 162 ]), &(acadoWorkspace.QDy[ 135 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 840 ]), &(acadoWorkspace.Dy[ 168 ]), &(acadoWorkspace.QDy[ 140 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 870 ]), &(acadoWorkspace.Dy[ 174 ]), &(acadoWorkspace.QDy[ 145 ]) );

acadoWorkspace.QDy[150] = + acadoWorkspace.QN2[0]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[1]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[2]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[3]*acadoWorkspace.DyN[3];
acadoWorkspace.QDy[151] = + acadoWorkspace.QN2[4]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[5]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[6]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[7]*acadoWorkspace.DyN[3];
acadoWorkspace.QDy[152] = + acadoWorkspace.QN2[8]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[9]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[10]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[11]*acadoWorkspace.DyN[3];
acadoWorkspace.QDy[153] = + acadoWorkspace.QN2[12]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[13]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[14]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[15]*acadoWorkspace.DyN[3];
acadoWorkspace.QDy[154] = + acadoWorkspace.QN2[16]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[17]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[18]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[19]*acadoWorkspace.DyN[3];

acadoWorkspace.sbar[0] = acadoWorkspace.Dx0[0];
acadoWorkspace.sbar[1] = acadoWorkspace.Dx0[1];
acadoWorkspace.sbar[2] = acadoWorkspace.Dx0[2];
acadoWorkspace.sbar[3] = acadoWorkspace.Dx0[3];
acadoWorkspace.sbar[4] = acadoWorkspace.Dx0[4];
acado_macASbar( acadoWorkspace.evGx, acadoWorkspace.sbar, &(acadoWorkspace.sbar[ 5 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 25 ]), &(acadoWorkspace.sbar[ 5 ]), &(acadoWorkspace.sbar[ 10 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 50 ]), &(acadoWorkspace.sbar[ 10 ]), &(acadoWorkspace.sbar[ 15 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 75 ]), &(acadoWorkspace.sbar[ 15 ]), &(acadoWorkspace.sbar[ 20 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 100 ]), &(acadoWorkspace.sbar[ 20 ]), &(acadoWorkspace.sbar[ 25 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 125 ]), &(acadoWorkspace.sbar[ 25 ]), &(acadoWorkspace.sbar[ 30 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 150 ]), &(acadoWorkspace.sbar[ 30 ]), &(acadoWorkspace.sbar[ 35 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 175 ]), &(acadoWorkspace.sbar[ 35 ]), &(acadoWorkspace.sbar[ 40 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 200 ]), &(acadoWorkspace.sbar[ 40 ]), &(acadoWorkspace.sbar[ 45 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 225 ]), &(acadoWorkspace.sbar[ 45 ]), &(acadoWorkspace.sbar[ 50 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 250 ]), &(acadoWorkspace.sbar[ 50 ]), &(acadoWorkspace.sbar[ 55 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 275 ]), &(acadoWorkspace.sbar[ 55 ]), &(acadoWorkspace.sbar[ 60 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 300 ]), &(acadoWorkspace.sbar[ 60 ]), &(acadoWorkspace.sbar[ 65 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 325 ]), &(acadoWorkspace.sbar[ 65 ]), &(acadoWorkspace.sbar[ 70 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 350 ]), &(acadoWorkspace.sbar[ 70 ]), &(acadoWorkspace.sbar[ 75 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 375 ]), &(acadoWorkspace.sbar[ 75 ]), &(acadoWorkspace.sbar[ 80 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 400 ]), &(acadoWorkspace.sbar[ 80 ]), &(acadoWorkspace.sbar[ 85 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 425 ]), &(acadoWorkspace.sbar[ 85 ]), &(acadoWorkspace.sbar[ 90 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 450 ]), &(acadoWorkspace.sbar[ 90 ]), &(acadoWorkspace.sbar[ 95 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 475 ]), &(acadoWorkspace.sbar[ 95 ]), &(acadoWorkspace.sbar[ 100 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 500 ]), &(acadoWorkspace.sbar[ 100 ]), &(acadoWorkspace.sbar[ 105 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 525 ]), &(acadoWorkspace.sbar[ 105 ]), &(acadoWorkspace.sbar[ 110 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 550 ]), &(acadoWorkspace.sbar[ 110 ]), &(acadoWorkspace.sbar[ 115 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 575 ]), &(acadoWorkspace.sbar[ 115 ]), &(acadoWorkspace.sbar[ 120 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 600 ]), &(acadoWorkspace.sbar[ 120 ]), &(acadoWorkspace.sbar[ 125 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 625 ]), &(acadoWorkspace.sbar[ 125 ]), &(acadoWorkspace.sbar[ 130 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 650 ]), &(acadoWorkspace.sbar[ 130 ]), &(acadoWorkspace.sbar[ 135 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 675 ]), &(acadoWorkspace.sbar[ 135 ]), &(acadoWorkspace.sbar[ 140 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 700 ]), &(acadoWorkspace.sbar[ 140 ]), &(acadoWorkspace.sbar[ 145 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 725 ]), &(acadoWorkspace.sbar[ 145 ]), &(acadoWorkspace.sbar[ 150 ]) );

acadoWorkspace.w1[0] = + acadoWorkspace.QN1[0]*acadoWorkspace.sbar[150] + acadoWorkspace.QN1[1]*acadoWorkspace.sbar[151] + acadoWorkspace.QN1[2]*acadoWorkspace.sbar[152] + acadoWorkspace.QN1[3]*acadoWorkspace.sbar[153] + acadoWorkspace.QN1[4]*acadoWorkspace.sbar[154] + acadoWorkspace.QDy[150];
acadoWorkspace.w1[1] = + acadoWorkspace.QN1[5]*acadoWorkspace.sbar[150] + acadoWorkspace.QN1[6]*acadoWorkspace.sbar[151] + acadoWorkspace.QN1[7]*acadoWorkspace.sbar[152] + acadoWorkspace.QN1[8]*acadoWorkspace.sbar[153] + acadoWorkspace.QN1[9]*acadoWorkspace.sbar[154] + acadoWorkspace.QDy[151];
acadoWorkspace.w1[2] = + acadoWorkspace.QN1[10]*acadoWorkspace.sbar[150] + acadoWorkspace.QN1[11]*acadoWorkspace.sbar[151] + acadoWorkspace.QN1[12]*acadoWorkspace.sbar[152] + acadoWorkspace.QN1[13]*acadoWorkspace.sbar[153] + acadoWorkspace.QN1[14]*acadoWorkspace.sbar[154] + acadoWorkspace.QDy[152];
acadoWorkspace.w1[3] = + acadoWorkspace.QN1[15]*acadoWorkspace.sbar[150] + acadoWorkspace.QN1[16]*acadoWorkspace.sbar[151] + acadoWorkspace.QN1[17]*acadoWorkspace.sbar[152] + acadoWorkspace.QN1[18]*acadoWorkspace.sbar[153] + acadoWorkspace.QN1[19]*acadoWorkspace.sbar[154] + acadoWorkspace.QDy[153];
acadoWorkspace.w1[4] = + acadoWorkspace.QN1[20]*acadoWorkspace.sbar[150] + acadoWorkspace.QN1[21]*acadoWorkspace.sbar[151] + acadoWorkspace.QN1[22]*acadoWorkspace.sbar[152] + acadoWorkspace.QN1[23]*acadoWorkspace.sbar[153] + acadoWorkspace.QN1[24]*acadoWorkspace.sbar[154] + acadoWorkspace.QDy[154];
acado_macBTw1( &(acadoWorkspace.evGu[ 290 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 58 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 725 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 145 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 725 ]), &(acadoWorkspace.sbar[ 145 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 280 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 56 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 700 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 140 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 700 ]), &(acadoWorkspace.sbar[ 140 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 270 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 54 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 675 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 135 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 675 ]), &(acadoWorkspace.sbar[ 135 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 260 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 52 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 650 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 130 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 650 ]), &(acadoWorkspace.sbar[ 130 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 250 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 50 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 625 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 125 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 625 ]), &(acadoWorkspace.sbar[ 125 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 240 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 48 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 600 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 120 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 600 ]), &(acadoWorkspace.sbar[ 120 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 230 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 46 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 575 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 115 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 575 ]), &(acadoWorkspace.sbar[ 115 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 220 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 44 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 550 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 110 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 550 ]), &(acadoWorkspace.sbar[ 110 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 210 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 42 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 525 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 105 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 525 ]), &(acadoWorkspace.sbar[ 105 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 200 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 40 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 500 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 100 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 500 ]), &(acadoWorkspace.sbar[ 100 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 190 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 38 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 475 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 95 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 475 ]), &(acadoWorkspace.sbar[ 95 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 180 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 36 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 450 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 90 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 450 ]), &(acadoWorkspace.sbar[ 90 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 170 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 34 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 425 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 85 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 425 ]), &(acadoWorkspace.sbar[ 85 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 160 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 32 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 400 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 80 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 400 ]), &(acadoWorkspace.sbar[ 80 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 150 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 30 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 375 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 75 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 375 ]), &(acadoWorkspace.sbar[ 75 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 140 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 28 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 350 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 70 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 350 ]), &(acadoWorkspace.sbar[ 70 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 130 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 26 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 325 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 65 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 325 ]), &(acadoWorkspace.sbar[ 65 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 120 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 24 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 300 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 60 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 300 ]), &(acadoWorkspace.sbar[ 60 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 110 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 22 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 275 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 55 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 275 ]), &(acadoWorkspace.sbar[ 55 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 100 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 20 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 250 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 50 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 250 ]), &(acadoWorkspace.sbar[ 50 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 90 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 18 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 225 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 45 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 225 ]), &(acadoWorkspace.sbar[ 45 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 80 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 16 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 200 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 40 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 200 ]), &(acadoWorkspace.sbar[ 40 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 70 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 14 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 175 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 35 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 175 ]), &(acadoWorkspace.sbar[ 35 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 60 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 12 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 150 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 30 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 150 ]), &(acadoWorkspace.sbar[ 30 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 50 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 10 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 125 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 25 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 125 ]), &(acadoWorkspace.sbar[ 25 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 40 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 8 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 100 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 20 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 100 ]), &(acadoWorkspace.sbar[ 20 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 30 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 6 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 75 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 15 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 75 ]), &(acadoWorkspace.sbar[ 15 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 20 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 4 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 50 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 10 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 50 ]), &(acadoWorkspace.sbar[ 10 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 10 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 2 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 25 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 5 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 25 ]), &(acadoWorkspace.sbar[ 5 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( acadoWorkspace.evGu, acadoWorkspace.w1, acadoWorkspace.g );

acadoWorkspace.lb[0] = acadoVariables.lbValues[0] - acadoVariables.u[0];
acadoWorkspace.lb[1] = acadoVariables.lbValues[1] - acadoVariables.u[1];
acadoWorkspace.lb[2] = acadoVariables.lbValues[2] - acadoVariables.u[2];
acadoWorkspace.lb[3] = acadoVariables.lbValues[3] - acadoVariables.u[3];
acadoWorkspace.lb[4] = acadoVariables.lbValues[4] - acadoVariables.u[4];
acadoWorkspace.lb[5] = acadoVariables.lbValues[5] - acadoVariables.u[5];
acadoWorkspace.lb[6] = acadoVariables.lbValues[6] - acadoVariables.u[6];
acadoWorkspace.lb[7] = acadoVariables.lbValues[7] - acadoVariables.u[7];
acadoWorkspace.lb[8] = acadoVariables.lbValues[8] - acadoVariables.u[8];
acadoWorkspace.lb[9] = acadoVariables.lbValues[9] - acadoVariables.u[9];
acadoWorkspace.lb[10] = acadoVariables.lbValues[10] - acadoVariables.u[10];
acadoWorkspace.lb[11] = acadoVariables.lbValues[11] - acadoVariables.u[11];
acadoWorkspace.lb[12] = acadoVariables.lbValues[12] - acadoVariables.u[12];
acadoWorkspace.lb[13] = acadoVariables.lbValues[13] - acadoVariables.u[13];
acadoWorkspace.lb[14] = acadoVariables.lbValues[14] - acadoVariables.u[14];
acadoWorkspace.lb[15] = acadoVariables.lbValues[15] - acadoVariables.u[15];
acadoWorkspace.lb[16] = acadoVariables.lbValues[16] - acadoVariables.u[16];
acadoWorkspace.lb[17] = acadoVariables.lbValues[17] - acadoVariables.u[17];
acadoWorkspace.lb[18] = acadoVariables.lbValues[18] - acadoVariables.u[18];
acadoWorkspace.lb[19] = acadoVariables.lbValues[19] - acadoVariables.u[19];
acadoWorkspace.lb[20] = acadoVariables.lbValues[20] - acadoVariables.u[20];
acadoWorkspace.lb[21] = acadoVariables.lbValues[21] - acadoVariables.u[21];
acadoWorkspace.lb[22] = acadoVariables.lbValues[22] - acadoVariables.u[22];
acadoWorkspace.lb[23] = acadoVariables.lbValues[23] - acadoVariables.u[23];
acadoWorkspace.lb[24] = acadoVariables.lbValues[24] - acadoVariables.u[24];
acadoWorkspace.lb[25] = acadoVariables.lbValues[25] - acadoVariables.u[25];
acadoWorkspace.lb[26] = acadoVariables.lbValues[26] - acadoVariables.u[26];
acadoWorkspace.lb[27] = acadoVariables.lbValues[27] - acadoVariables.u[27];
acadoWorkspace.lb[28] = acadoVariables.lbValues[28] - acadoVariables.u[28];
acadoWorkspace.lb[29] = acadoVariables.lbValues[29] - acadoVariables.u[29];
acadoWorkspace.lb[30] = acadoVariables.lbValues[30] - acadoVariables.u[30];
acadoWorkspace.lb[31] = acadoVariables.lbValues[31] - acadoVariables.u[31];
acadoWorkspace.lb[32] = acadoVariables.lbValues[32] - acadoVariables.u[32];
acadoWorkspace.lb[33] = acadoVariables.lbValues[33] - acadoVariables.u[33];
acadoWorkspace.lb[34] = acadoVariables.lbValues[34] - acadoVariables.u[34];
acadoWorkspace.lb[35] = acadoVariables.lbValues[35] - acadoVariables.u[35];
acadoWorkspace.lb[36] = acadoVariables.lbValues[36] - acadoVariables.u[36];
acadoWorkspace.lb[37] = acadoVariables.lbValues[37] - acadoVariables.u[37];
acadoWorkspace.lb[38] = acadoVariables.lbValues[38] - acadoVariables.u[38];
acadoWorkspace.lb[39] = acadoVariables.lbValues[39] - acadoVariables.u[39];
acadoWorkspace.lb[40] = acadoVariables.lbValues[40] - acadoVariables.u[40];
acadoWorkspace.lb[41] = acadoVariables.lbValues[41] - acadoVariables.u[41];
acadoWorkspace.lb[42] = acadoVariables.lbValues[42] - acadoVariables.u[42];
acadoWorkspace.lb[43] = acadoVariables.lbValues[43] - acadoVariables.u[43];
acadoWorkspace.lb[44] = acadoVariables.lbValues[44] - acadoVariables.u[44];
acadoWorkspace.lb[45] = acadoVariables.lbValues[45] - acadoVariables.u[45];
acadoWorkspace.lb[46] = acadoVariables.lbValues[46] - acadoVariables.u[46];
acadoWorkspace.lb[47] = acadoVariables.lbValues[47] - acadoVariables.u[47];
acadoWorkspace.lb[48] = acadoVariables.lbValues[48] - acadoVariables.u[48];
acadoWorkspace.lb[49] = acadoVariables.lbValues[49] - acadoVariables.u[49];
acadoWorkspace.lb[50] = acadoVariables.lbValues[50] - acadoVariables.u[50];
acadoWorkspace.lb[51] = acadoVariables.lbValues[51] - acadoVariables.u[51];
acadoWorkspace.lb[52] = acadoVariables.lbValues[52] - acadoVariables.u[52];
acadoWorkspace.lb[53] = acadoVariables.lbValues[53] - acadoVariables.u[53];
acadoWorkspace.lb[54] = acadoVariables.lbValues[54] - acadoVariables.u[54];
acadoWorkspace.lb[55] = acadoVariables.lbValues[55] - acadoVariables.u[55];
acadoWorkspace.lb[56] = acadoVariables.lbValues[56] - acadoVariables.u[56];
acadoWorkspace.lb[57] = acadoVariables.lbValues[57] - acadoVariables.u[57];
acadoWorkspace.lb[58] = acadoVariables.lbValues[58] - acadoVariables.u[58];
acadoWorkspace.lb[59] = acadoVariables.lbValues[59] - acadoVariables.u[59];
acadoWorkspace.ub[0] = acadoVariables.ubValues[0] - acadoVariables.u[0];
acadoWorkspace.ub[1] = acadoVariables.ubValues[1] - acadoVariables.u[1];
acadoWorkspace.ub[2] = acadoVariables.ubValues[2] - acadoVariables.u[2];
acadoWorkspace.ub[3] = acadoVariables.ubValues[3] - acadoVariables.u[3];
acadoWorkspace.ub[4] = acadoVariables.ubValues[4] - acadoVariables.u[4];
acadoWorkspace.ub[5] = acadoVariables.ubValues[5] - acadoVariables.u[5];
acadoWorkspace.ub[6] = acadoVariables.ubValues[6] - acadoVariables.u[6];
acadoWorkspace.ub[7] = acadoVariables.ubValues[7] - acadoVariables.u[7];
acadoWorkspace.ub[8] = acadoVariables.ubValues[8] - acadoVariables.u[8];
acadoWorkspace.ub[9] = acadoVariables.ubValues[9] - acadoVariables.u[9];
acadoWorkspace.ub[10] = acadoVariables.ubValues[10] - acadoVariables.u[10];
acadoWorkspace.ub[11] = acadoVariables.ubValues[11] - acadoVariables.u[11];
acadoWorkspace.ub[12] = acadoVariables.ubValues[12] - acadoVariables.u[12];
acadoWorkspace.ub[13] = acadoVariables.ubValues[13] - acadoVariables.u[13];
acadoWorkspace.ub[14] = acadoVariables.ubValues[14] - acadoVariables.u[14];
acadoWorkspace.ub[15] = acadoVariables.ubValues[15] - acadoVariables.u[15];
acadoWorkspace.ub[16] = acadoVariables.ubValues[16] - acadoVariables.u[16];
acadoWorkspace.ub[17] = acadoVariables.ubValues[17] - acadoVariables.u[17];
acadoWorkspace.ub[18] = acadoVariables.ubValues[18] - acadoVariables.u[18];
acadoWorkspace.ub[19] = acadoVariables.ubValues[19] - acadoVariables.u[19];
acadoWorkspace.ub[20] = acadoVariables.ubValues[20] - acadoVariables.u[20];
acadoWorkspace.ub[21] = acadoVariables.ubValues[21] - acadoVariables.u[21];
acadoWorkspace.ub[22] = acadoVariables.ubValues[22] - acadoVariables.u[22];
acadoWorkspace.ub[23] = acadoVariables.ubValues[23] - acadoVariables.u[23];
acadoWorkspace.ub[24] = acadoVariables.ubValues[24] - acadoVariables.u[24];
acadoWorkspace.ub[25] = acadoVariables.ubValues[25] - acadoVariables.u[25];
acadoWorkspace.ub[26] = acadoVariables.ubValues[26] - acadoVariables.u[26];
acadoWorkspace.ub[27] = acadoVariables.ubValues[27] - acadoVariables.u[27];
acadoWorkspace.ub[28] = acadoVariables.ubValues[28] - acadoVariables.u[28];
acadoWorkspace.ub[29] = acadoVariables.ubValues[29] - acadoVariables.u[29];
acadoWorkspace.ub[30] = acadoVariables.ubValues[30] - acadoVariables.u[30];
acadoWorkspace.ub[31] = acadoVariables.ubValues[31] - acadoVariables.u[31];
acadoWorkspace.ub[32] = acadoVariables.ubValues[32] - acadoVariables.u[32];
acadoWorkspace.ub[33] = acadoVariables.ubValues[33] - acadoVariables.u[33];
acadoWorkspace.ub[34] = acadoVariables.ubValues[34] - acadoVariables.u[34];
acadoWorkspace.ub[35] = acadoVariables.ubValues[35] - acadoVariables.u[35];
acadoWorkspace.ub[36] = acadoVariables.ubValues[36] - acadoVariables.u[36];
acadoWorkspace.ub[37] = acadoVariables.ubValues[37] - acadoVariables.u[37];
acadoWorkspace.ub[38] = acadoVariables.ubValues[38] - acadoVariables.u[38];
acadoWorkspace.ub[39] = acadoVariables.ubValues[39] - acadoVariables.u[39];
acadoWorkspace.ub[40] = acadoVariables.ubValues[40] - acadoVariables.u[40];
acadoWorkspace.ub[41] = acadoVariables.ubValues[41] - acadoVariables.u[41];
acadoWorkspace.ub[42] = acadoVariables.ubValues[42] - acadoVariables.u[42];
acadoWorkspace.ub[43] = acadoVariables.ubValues[43] - acadoVariables.u[43];
acadoWorkspace.ub[44] = acadoVariables.ubValues[44] - acadoVariables.u[44];
acadoWorkspace.ub[45] = acadoVariables.ubValues[45] - acadoVariables.u[45];
acadoWorkspace.ub[46] = acadoVariables.ubValues[46] - acadoVariables.u[46];
acadoWorkspace.ub[47] = acadoVariables.ubValues[47] - acadoVariables.u[47];
acadoWorkspace.ub[48] = acadoVariables.ubValues[48] - acadoVariables.u[48];
acadoWorkspace.ub[49] = acadoVariables.ubValues[49] - acadoVariables.u[49];
acadoWorkspace.ub[50] = acadoVariables.ubValues[50] - acadoVariables.u[50];
acadoWorkspace.ub[51] = acadoVariables.ubValues[51] - acadoVariables.u[51];
acadoWorkspace.ub[52] = acadoVariables.ubValues[52] - acadoVariables.u[52];
acadoWorkspace.ub[53] = acadoVariables.ubValues[53] - acadoVariables.u[53];
acadoWorkspace.ub[54] = acadoVariables.ubValues[54] - acadoVariables.u[54];
acadoWorkspace.ub[55] = acadoVariables.ubValues[55] - acadoVariables.u[55];
acadoWorkspace.ub[56] = acadoVariables.ubValues[56] - acadoVariables.u[56];
acadoWorkspace.ub[57] = acadoVariables.ubValues[57] - acadoVariables.u[57];
acadoWorkspace.ub[58] = acadoVariables.ubValues[58] - acadoVariables.u[58];
acadoWorkspace.ub[59] = acadoVariables.ubValues[59] - acadoVariables.u[59];


acado_macHxd( acadoWorkspace.evHx, acadoWorkspace.sbar, acadoWorkspace.lbA, acadoWorkspace.ubA );
acado_macHxd( &(acadoWorkspace.evHx[ 5 ]), &(acadoWorkspace.sbar[ 5 ]), &(acadoWorkspace.lbA[ 1 ]), &(acadoWorkspace.ubA[ 1 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 10 ]), &(acadoWorkspace.sbar[ 10 ]), &(acadoWorkspace.lbA[ 2 ]), &(acadoWorkspace.ubA[ 2 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 15 ]), &(acadoWorkspace.sbar[ 15 ]), &(acadoWorkspace.lbA[ 3 ]), &(acadoWorkspace.ubA[ 3 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 20 ]), &(acadoWorkspace.sbar[ 20 ]), &(acadoWorkspace.lbA[ 4 ]), &(acadoWorkspace.ubA[ 4 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 25 ]), &(acadoWorkspace.sbar[ 25 ]), &(acadoWorkspace.lbA[ 5 ]), &(acadoWorkspace.ubA[ 5 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 30 ]), &(acadoWorkspace.sbar[ 30 ]), &(acadoWorkspace.lbA[ 6 ]), &(acadoWorkspace.ubA[ 6 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 35 ]), &(acadoWorkspace.sbar[ 35 ]), &(acadoWorkspace.lbA[ 7 ]), &(acadoWorkspace.ubA[ 7 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 40 ]), &(acadoWorkspace.sbar[ 40 ]), &(acadoWorkspace.lbA[ 8 ]), &(acadoWorkspace.ubA[ 8 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 45 ]), &(acadoWorkspace.sbar[ 45 ]), &(acadoWorkspace.lbA[ 9 ]), &(acadoWorkspace.ubA[ 9 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 50 ]), &(acadoWorkspace.sbar[ 50 ]), &(acadoWorkspace.lbA[ 10 ]), &(acadoWorkspace.ubA[ 10 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 55 ]), &(acadoWorkspace.sbar[ 55 ]), &(acadoWorkspace.lbA[ 11 ]), &(acadoWorkspace.ubA[ 11 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 60 ]), &(acadoWorkspace.sbar[ 60 ]), &(acadoWorkspace.lbA[ 12 ]), &(acadoWorkspace.ubA[ 12 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 65 ]), &(acadoWorkspace.sbar[ 65 ]), &(acadoWorkspace.lbA[ 13 ]), &(acadoWorkspace.ubA[ 13 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 70 ]), &(acadoWorkspace.sbar[ 70 ]), &(acadoWorkspace.lbA[ 14 ]), &(acadoWorkspace.ubA[ 14 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 75 ]), &(acadoWorkspace.sbar[ 75 ]), &(acadoWorkspace.lbA[ 15 ]), &(acadoWorkspace.ubA[ 15 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 80 ]), &(acadoWorkspace.sbar[ 80 ]), &(acadoWorkspace.lbA[ 16 ]), &(acadoWorkspace.ubA[ 16 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 85 ]), &(acadoWorkspace.sbar[ 85 ]), &(acadoWorkspace.lbA[ 17 ]), &(acadoWorkspace.ubA[ 17 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 90 ]), &(acadoWorkspace.sbar[ 90 ]), &(acadoWorkspace.lbA[ 18 ]), &(acadoWorkspace.ubA[ 18 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 95 ]), &(acadoWorkspace.sbar[ 95 ]), &(acadoWorkspace.lbA[ 19 ]), &(acadoWorkspace.ubA[ 19 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 100 ]), &(acadoWorkspace.sbar[ 100 ]), &(acadoWorkspace.lbA[ 20 ]), &(acadoWorkspace.ubA[ 20 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 105 ]), &(acadoWorkspace.sbar[ 105 ]), &(acadoWorkspace.lbA[ 21 ]), &(acadoWorkspace.ubA[ 21 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 110 ]), &(acadoWorkspace.sbar[ 110 ]), &(acadoWorkspace.lbA[ 22 ]), &(acadoWorkspace.ubA[ 22 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 115 ]), &(acadoWorkspace.sbar[ 115 ]), &(acadoWorkspace.lbA[ 23 ]), &(acadoWorkspace.ubA[ 23 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 120 ]), &(acadoWorkspace.sbar[ 120 ]), &(acadoWorkspace.lbA[ 24 ]), &(acadoWorkspace.ubA[ 24 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 125 ]), &(acadoWorkspace.sbar[ 125 ]), &(acadoWorkspace.lbA[ 25 ]), &(acadoWorkspace.ubA[ 25 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 130 ]), &(acadoWorkspace.sbar[ 130 ]), &(acadoWorkspace.lbA[ 26 ]), &(acadoWorkspace.ubA[ 26 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 135 ]), &(acadoWorkspace.sbar[ 135 ]), &(acadoWorkspace.lbA[ 27 ]), &(acadoWorkspace.ubA[ 27 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 140 ]), &(acadoWorkspace.sbar[ 140 ]), &(acadoWorkspace.lbA[ 28 ]), &(acadoWorkspace.ubA[ 28 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 145 ]), &(acadoWorkspace.sbar[ 145 ]), &(acadoWorkspace.lbA[ 29 ]), &(acadoWorkspace.ubA[ 29 ]) );

}

void acado_expand(  )
{
int lRun1;
acadoVariables.u[0] += acadoWorkspace.x[0];
acadoVariables.u[1] += acadoWorkspace.x[1];
acadoVariables.u[2] += acadoWorkspace.x[2];
acadoVariables.u[3] += acadoWorkspace.x[3];
acadoVariables.u[4] += acadoWorkspace.x[4];
acadoVariables.u[5] += acadoWorkspace.x[5];
acadoVariables.u[6] += acadoWorkspace.x[6];
acadoVariables.u[7] += acadoWorkspace.x[7];
acadoVariables.u[8] += acadoWorkspace.x[8];
acadoVariables.u[9] += acadoWorkspace.x[9];
acadoVariables.u[10] += acadoWorkspace.x[10];
acadoVariables.u[11] += acadoWorkspace.x[11];
acadoVariables.u[12] += acadoWorkspace.x[12];
acadoVariables.u[13] += acadoWorkspace.x[13];
acadoVariables.u[14] += acadoWorkspace.x[14];
acadoVariables.u[15] += acadoWorkspace.x[15];
acadoVariables.u[16] += acadoWorkspace.x[16];
acadoVariables.u[17] += acadoWorkspace.x[17];
acadoVariables.u[18] += acadoWorkspace.x[18];
acadoVariables.u[19] += acadoWorkspace.x[19];
acadoVariables.u[20] += acadoWorkspace.x[20];
acadoVariables.u[21] += acadoWorkspace.x[21];
acadoVariables.u[22] += acadoWorkspace.x[22];
acadoVariables.u[23] += acadoWorkspace.x[23];
acadoVariables.u[24] += acadoWorkspace.x[24];
acadoVariables.u[25] += acadoWorkspace.x[25];
acadoVariables.u[26] += acadoWorkspace.x[26];
acadoVariables.u[27] += acadoWorkspace.x[27];
acadoVariables.u[28] += acadoWorkspace.x[28];
acadoVariables.u[29] += acadoWorkspace.x[29];
acadoVariables.u[30] += acadoWorkspace.x[30];
acadoVariables.u[31] += acadoWorkspace.x[31];
acadoVariables.u[32] += acadoWorkspace.x[32];
acadoVariables.u[33] += acadoWorkspace.x[33];
acadoVariables.u[34] += acadoWorkspace.x[34];
acadoVariables.u[35] += acadoWorkspace.x[35];
acadoVariables.u[36] += acadoWorkspace.x[36];
acadoVariables.u[37] += acadoWorkspace.x[37];
acadoVariables.u[38] += acadoWorkspace.x[38];
acadoVariables.u[39] += acadoWorkspace.x[39];
acadoVariables.u[40] += acadoWorkspace.x[40];
acadoVariables.u[41] += acadoWorkspace.x[41];
acadoVariables.u[42] += acadoWorkspace.x[42];
acadoVariables.u[43] += acadoWorkspace.x[43];
acadoVariables.u[44] += acadoWorkspace.x[44];
acadoVariables.u[45] += acadoWorkspace.x[45];
acadoVariables.u[46] += acadoWorkspace.x[46];
acadoVariables.u[47] += acadoWorkspace.x[47];
acadoVariables.u[48] += acadoWorkspace.x[48];
acadoVariables.u[49] += acadoWorkspace.x[49];
acadoVariables.u[50] += acadoWorkspace.x[50];
acadoVariables.u[51] += acadoWorkspace.x[51];
acadoVariables.u[52] += acadoWorkspace.x[52];
acadoVariables.u[53] += acadoWorkspace.x[53];
acadoVariables.u[54] += acadoWorkspace.x[54];
acadoVariables.u[55] += acadoWorkspace.x[55];
acadoVariables.u[56] += acadoWorkspace.x[56];
acadoVariables.u[57] += acadoWorkspace.x[57];
acadoVariables.u[58] += acadoWorkspace.x[58];
acadoVariables.u[59] += acadoWorkspace.x[59];
acadoWorkspace.sbar[0] = acadoWorkspace.Dx0[0];
acadoWorkspace.sbar[1] = acadoWorkspace.Dx0[1];
acadoWorkspace.sbar[2] = acadoWorkspace.Dx0[2];
acadoWorkspace.sbar[3] = acadoWorkspace.Dx0[3];
acadoWorkspace.sbar[4] = acadoWorkspace.Dx0[4];
for (lRun1 = 0; lRun1 < 150; ++lRun1)
acadoWorkspace.sbar[lRun1 + 5] = acadoWorkspace.d[lRun1];

acado_expansionStep( acadoWorkspace.evGx, acadoWorkspace.evGu, acadoWorkspace.x, acadoWorkspace.sbar, &(acadoWorkspace.sbar[ 5 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 25 ]), &(acadoWorkspace.evGu[ 10 ]), &(acadoWorkspace.x[ 2 ]), &(acadoWorkspace.sbar[ 5 ]), &(acadoWorkspace.sbar[ 10 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 50 ]), &(acadoWorkspace.evGu[ 20 ]), &(acadoWorkspace.x[ 4 ]), &(acadoWorkspace.sbar[ 10 ]), &(acadoWorkspace.sbar[ 15 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 75 ]), &(acadoWorkspace.evGu[ 30 ]), &(acadoWorkspace.x[ 6 ]), &(acadoWorkspace.sbar[ 15 ]), &(acadoWorkspace.sbar[ 20 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 100 ]), &(acadoWorkspace.evGu[ 40 ]), &(acadoWorkspace.x[ 8 ]), &(acadoWorkspace.sbar[ 20 ]), &(acadoWorkspace.sbar[ 25 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 125 ]), &(acadoWorkspace.evGu[ 50 ]), &(acadoWorkspace.x[ 10 ]), &(acadoWorkspace.sbar[ 25 ]), &(acadoWorkspace.sbar[ 30 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 150 ]), &(acadoWorkspace.evGu[ 60 ]), &(acadoWorkspace.x[ 12 ]), &(acadoWorkspace.sbar[ 30 ]), &(acadoWorkspace.sbar[ 35 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 175 ]), &(acadoWorkspace.evGu[ 70 ]), &(acadoWorkspace.x[ 14 ]), &(acadoWorkspace.sbar[ 35 ]), &(acadoWorkspace.sbar[ 40 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 200 ]), &(acadoWorkspace.evGu[ 80 ]), &(acadoWorkspace.x[ 16 ]), &(acadoWorkspace.sbar[ 40 ]), &(acadoWorkspace.sbar[ 45 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 225 ]), &(acadoWorkspace.evGu[ 90 ]), &(acadoWorkspace.x[ 18 ]), &(acadoWorkspace.sbar[ 45 ]), &(acadoWorkspace.sbar[ 50 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 250 ]), &(acadoWorkspace.evGu[ 100 ]), &(acadoWorkspace.x[ 20 ]), &(acadoWorkspace.sbar[ 50 ]), &(acadoWorkspace.sbar[ 55 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 275 ]), &(acadoWorkspace.evGu[ 110 ]), &(acadoWorkspace.x[ 22 ]), &(acadoWorkspace.sbar[ 55 ]), &(acadoWorkspace.sbar[ 60 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 300 ]), &(acadoWorkspace.evGu[ 120 ]), &(acadoWorkspace.x[ 24 ]), &(acadoWorkspace.sbar[ 60 ]), &(acadoWorkspace.sbar[ 65 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 325 ]), &(acadoWorkspace.evGu[ 130 ]), &(acadoWorkspace.x[ 26 ]), &(acadoWorkspace.sbar[ 65 ]), &(acadoWorkspace.sbar[ 70 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 350 ]), &(acadoWorkspace.evGu[ 140 ]), &(acadoWorkspace.x[ 28 ]), &(acadoWorkspace.sbar[ 70 ]), &(acadoWorkspace.sbar[ 75 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 375 ]), &(acadoWorkspace.evGu[ 150 ]), &(acadoWorkspace.x[ 30 ]), &(acadoWorkspace.sbar[ 75 ]), &(acadoWorkspace.sbar[ 80 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 400 ]), &(acadoWorkspace.evGu[ 160 ]), &(acadoWorkspace.x[ 32 ]), &(acadoWorkspace.sbar[ 80 ]), &(acadoWorkspace.sbar[ 85 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 425 ]), &(acadoWorkspace.evGu[ 170 ]), &(acadoWorkspace.x[ 34 ]), &(acadoWorkspace.sbar[ 85 ]), &(acadoWorkspace.sbar[ 90 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 450 ]), &(acadoWorkspace.evGu[ 180 ]), &(acadoWorkspace.x[ 36 ]), &(acadoWorkspace.sbar[ 90 ]), &(acadoWorkspace.sbar[ 95 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 475 ]), &(acadoWorkspace.evGu[ 190 ]), &(acadoWorkspace.x[ 38 ]), &(acadoWorkspace.sbar[ 95 ]), &(acadoWorkspace.sbar[ 100 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 500 ]), &(acadoWorkspace.evGu[ 200 ]), &(acadoWorkspace.x[ 40 ]), &(acadoWorkspace.sbar[ 100 ]), &(acadoWorkspace.sbar[ 105 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 525 ]), &(acadoWorkspace.evGu[ 210 ]), &(acadoWorkspace.x[ 42 ]), &(acadoWorkspace.sbar[ 105 ]), &(acadoWorkspace.sbar[ 110 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 550 ]), &(acadoWorkspace.evGu[ 220 ]), &(acadoWorkspace.x[ 44 ]), &(acadoWorkspace.sbar[ 110 ]), &(acadoWorkspace.sbar[ 115 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 575 ]), &(acadoWorkspace.evGu[ 230 ]), &(acadoWorkspace.x[ 46 ]), &(acadoWorkspace.sbar[ 115 ]), &(acadoWorkspace.sbar[ 120 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 600 ]), &(acadoWorkspace.evGu[ 240 ]), &(acadoWorkspace.x[ 48 ]), &(acadoWorkspace.sbar[ 120 ]), &(acadoWorkspace.sbar[ 125 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 625 ]), &(acadoWorkspace.evGu[ 250 ]), &(acadoWorkspace.x[ 50 ]), &(acadoWorkspace.sbar[ 125 ]), &(acadoWorkspace.sbar[ 130 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 650 ]), &(acadoWorkspace.evGu[ 260 ]), &(acadoWorkspace.x[ 52 ]), &(acadoWorkspace.sbar[ 130 ]), &(acadoWorkspace.sbar[ 135 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 675 ]), &(acadoWorkspace.evGu[ 270 ]), &(acadoWorkspace.x[ 54 ]), &(acadoWorkspace.sbar[ 135 ]), &(acadoWorkspace.sbar[ 140 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 700 ]), &(acadoWorkspace.evGu[ 280 ]), &(acadoWorkspace.x[ 56 ]), &(acadoWorkspace.sbar[ 140 ]), &(acadoWorkspace.sbar[ 145 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 725 ]), &(acadoWorkspace.evGu[ 290 ]), &(acadoWorkspace.x[ 58 ]), &(acadoWorkspace.sbar[ 145 ]), &(acadoWorkspace.sbar[ 150 ]) );
for (lRun1 = 0; lRun1 < 155; ++lRun1)
acadoVariables.x[lRun1] += acadoWorkspace.sbar[lRun1];

}

int acado_preparationStep(  )
{
int ret;

ret = acado_modelSimulation();
acado_evaluateObjective(  );
acado_condensePrep(  );
return ret;
}

int acado_feedbackStep(  )
{
int tmp;

acado_condenseFdb(  );

tmp = acado_solve( );

acado_expand(  );
return tmp;
}

int acado_initializeSolver(  )
{
int ret;

/* This is a function which must be called once before any other function call! */


ret = 0;

memset(&acadoWorkspace, 0, sizeof( acadoWorkspace ));
acadoVariables.lbValues[0] = -1.5000000000000000e+00;
acadoVariables.lbValues[1] = -5.0000000000000000e-01;
acadoVariables.lbValues[2] = -1.5000000000000000e+00;
acadoVariables.lbValues[3] = -5.0000000000000000e-01;
acadoVariables.lbValues[4] = -1.5000000000000000e+00;
acadoVariables.lbValues[5] = -5.0000000000000000e-01;
acadoVariables.lbValues[6] = -1.5000000000000000e+00;
acadoVariables.lbValues[7] = -5.0000000000000000e-01;
acadoVariables.lbValues[8] = -1.5000000000000000e+00;
acadoVariables.lbValues[9] = -5.0000000000000000e-01;
acadoVariables.lbValues[10] = -1.5000000000000000e+00;
acadoVariables.lbValues[11] = -5.0000000000000000e-01;
acadoVariables.lbValues[12] = -1.5000000000000000e+00;
acadoVariables.lbValues[13] = -5.0000000000000000e-01;
acadoVariables.lbValues[14] = -1.5000000000000000e+00;
acadoVariables.lbValues[15] = -5.0000000000000000e-01;
acadoVariables.lbValues[16] = -1.5000000000000000e+00;
acadoVariables.lbValues[17] = -5.0000000000000000e-01;
acadoVariables.lbValues[18] = -1.5000000000000000e+00;
acadoVariables.lbValues[19] = -5.0000000000000000e-01;
acadoVariables.lbValues[20] = -1.5000000000000000e+00;
acadoVariables.lbValues[21] = -5.0000000000000000e-01;
acadoVariables.lbValues[22] = -1.5000000000000000e+00;
acadoVariables.lbValues[23] = -5.0000000000000000e-01;
acadoVariables.lbValues[24] = -1.5000000000000000e+00;
acadoVariables.lbValues[25] = -5.0000000000000000e-01;
acadoVariables.lbValues[26] = -1.5000000000000000e+00;
acadoVariables.lbValues[27] = -5.0000000000000000e-01;
acadoVariables.lbValues[28] = -1.5000000000000000e+00;
acadoVariables.lbValues[29] = -5.0000000000000000e-01;
acadoVariables.lbValues[30] = -1.5000000000000000e+00;
acadoVariables.lbValues[31] = -5.0000000000000000e-01;
acadoVariables.lbValues[32] = -1.5000000000000000e+00;
acadoVariables.lbValues[33] = -5.0000000000000000e-01;
acadoVariables.lbValues[34] = -1.5000000000000000e+00;
acadoVariables.lbValues[35] = -5.0000000000000000e-01;
acadoVariables.lbValues[36] = -1.5000000000000000e+00;
acadoVariables.lbValues[37] = -5.0000000000000000e-01;
acadoVariables.lbValues[38] = -1.5000000000000000e+00;
acadoVariables.lbValues[39] = -5.0000000000000000e-01;
acadoVariables.lbValues[40] = -1.5000000000000000e+00;
acadoVariables.lbValues[41] = -5.0000000000000000e-01;
acadoVariables.lbValues[42] = -1.5000000000000000e+00;
acadoVariables.lbValues[43] = -5.0000000000000000e-01;
acadoVariables.lbValues[44] = -1.5000000000000000e+00;
acadoVariables.lbValues[45] = -5.0000000000000000e-01;
acadoVariables.lbValues[46] = -1.5000000000000000e+00;
acadoVariables.lbValues[47] = -5.0000000000000000e-01;
acadoVariables.lbValues[48] = -1.5000000000000000e+00;
acadoVariables.lbValues[49] = -5.0000000000000000e-01;
acadoVariables.lbValues[50] = -1.5000000000000000e+00;
acadoVariables.lbValues[51] = -5.0000000000000000e-01;
acadoVariables.lbValues[52] = -1.5000000000000000e+00;
acadoVariables.lbValues[53] = -5.0000000000000000e-01;
acadoVariables.lbValues[54] = -1.5000000000000000e+00;
acadoVariables.lbValues[55] = -5.0000000000000000e-01;
acadoVariables.lbValues[56] = -1.5000000000000000e+00;
acadoVariables.lbValues[57] = -5.0000000000000000e-01;
acadoVariables.lbValues[58] = -1.5000000000000000e+00;
acadoVariables.lbValues[59] = -5.0000000000000000e-01;
acadoVariables.ubValues[0] = 1.5000000000000000e+00;
acadoVariables.ubValues[1] = 5.0000000000000000e-01;
acadoVariables.ubValues[2] = 1.5000000000000000e+00;
acadoVariables.ubValues[3] = 5.0000000000000000e-01;
acadoVariables.ubValues[4] = 1.5000000000000000e+00;
acadoVariables.ubValues[5] = 5.0000000000000000e-01;
acadoVariables.ubValues[6] = 1.5000000000000000e+00;
acadoVariables.ubValues[7] = 5.0000000000000000e-01;
acadoVariables.ubValues[8] = 1.5000000000000000e+00;
acadoVariables.ubValues[9] = 5.0000000000000000e-01;
acadoVariables.ubValues[10] = 1.5000000000000000e+00;
acadoVariables.ubValues[11] = 5.0000000000000000e-01;
acadoVariables.ubValues[12] = 1.5000000000000000e+00;
acadoVariables.ubValues[13] = 5.0000000000000000e-01;
acadoVariables.ubValues[14] = 1.5000000000000000e+00;
acadoVariables.ubValues[15] = 5.0000000000000000e-01;
acadoVariables.ubValues[16] = 1.5000000000000000e+00;
acadoVariables.ubValues[17] = 5.0000000000000000e-01;
acadoVariables.ubValues[18] = 1.5000000000000000e+00;
acadoVariables.ubValues[19] = 5.0000000000000000e-01;
acadoVariables.ubValues[20] = 1.5000000000000000e+00;
acadoVariables.ubValues[21] = 5.0000000000000000e-01;
acadoVariables.ubValues[22] = 1.5000000000000000e+00;
acadoVariables.ubValues[23] = 5.0000000000000000e-01;
acadoVariables.ubValues[24] = 1.5000000000000000e+00;
acadoVariables.ubValues[25] = 5.0000000000000000e-01;
acadoVariables.ubValues[26] = 1.5000000000000000e+00;
acadoVariables.ubValues[27] = 5.0000000000000000e-01;
acadoVariables.ubValues[28] = 1.5000000000000000e+00;
acadoVariables.ubValues[29] = 5.0000000000000000e-01;
acadoVariables.ubValues[30] = 1.5000000000000000e+00;
acadoVariables.ubValues[31] = 5.0000000000000000e-01;
acadoVariables.ubValues[32] = 1.5000000000000000e+00;
acadoVariables.ubValues[33] = 5.0000000000000000e-01;
acadoVariables.ubValues[34] = 1.5000000000000000e+00;
acadoVariables.ubValues[35] = 5.0000000000000000e-01;
acadoVariables.ubValues[36] = 1.5000000000000000e+00;
acadoVariables.ubValues[37] = 5.0000000000000000e-01;
acadoVariables.ubValues[38] = 1.5000000000000000e+00;
acadoVariables.ubValues[39] = 5.0000000000000000e-01;
acadoVariables.ubValues[40] = 1.5000000000000000e+00;
acadoVariables.ubValues[41] = 5.0000000000000000e-01;
acadoVariables.ubValues[42] = 1.5000000000000000e+00;
acadoVariables.ubValues[43] = 5.0000000000000000e-01;
acadoVariables.ubValues[44] = 1.5000000000000000e+00;
acadoVariables.ubValues[45] = 5.0000000000000000e-01;
acadoVariables.ubValues[46] = 1.5000000000000000e+00;
acadoVariables.ubValues[47] = 5.0000000000000000e-01;
acadoVariables.ubValues[48] = 1.5000000000000000e+00;
acadoVariables.ubValues[49] = 5.0000000000000000e-01;
acadoVariables.ubValues[50] = 1.5000000000000000e+00;
acadoVariables.ubValues[51] = 5.0000000000000000e-01;
acadoVariables.ubValues[52] = 1.5000000000000000e+00;
acadoVariables.ubValues[53] = 5.0000000000000000e-01;
acadoVariables.ubValues[54] = 1.5000000000000000e+00;
acadoVariables.ubValues[55] = 5.0000000000000000e-01;
acadoVariables.ubValues[56] = 1.5000000000000000e+00;
acadoVariables.ubValues[57] = 5.0000000000000000e-01;
acadoVariables.ubValues[58] = 1.5000000000000000e+00;
acadoVariables.ubValues[59] = 5.0000000000000000e-01;
acadoVariables.lbAValues[0] = 1.0000000000000000e+00;
acadoVariables.lbAValues[1] = 1.0000000000000000e+00;
acadoVariables.lbAValues[2] = 1.0000000000000000e+00;
acadoVariables.lbAValues[3] = 1.0000000000000000e+00;
acadoVariables.lbAValues[4] = 1.0000000000000000e+00;
acadoVariables.lbAValues[5] = 1.0000000000000000e+00;
acadoVariables.lbAValues[6] = 1.0000000000000000e+00;
acadoVariables.lbAValues[7] = 1.0000000000000000e+00;
acadoVariables.lbAValues[8] = 1.0000000000000000e+00;
acadoVariables.lbAValues[9] = 1.0000000000000000e+00;
acadoVariables.lbAValues[10] = 1.0000000000000000e+00;
acadoVariables.lbAValues[11] = 1.0000000000000000e+00;
acadoVariables.lbAValues[12] = 1.0000000000000000e+00;
acadoVariables.lbAValues[13] = 1.0000000000000000e+00;
acadoVariables.lbAValues[14] = 1.0000000000000000e+00;
acadoVariables.lbAValues[15] = 1.0000000000000000e+00;
acadoVariables.lbAValues[16] = 1.0000000000000000e+00;
acadoVariables.lbAValues[17] = 1.0000000000000000e+00;
acadoVariables.lbAValues[18] = 1.0000000000000000e+00;
acadoVariables.lbAValues[19] = 1.0000000000000000e+00;
acadoVariables.lbAValues[20] = 1.0000000000000000e+00;
acadoVariables.lbAValues[21] = 1.0000000000000000e+00;
acadoVariables.lbAValues[22] = 1.0000000000000000e+00;
acadoVariables.lbAValues[23] = 1.0000000000000000e+00;
acadoVariables.lbAValues[24] = 1.0000000000000000e+00;
acadoVariables.lbAValues[25] = 1.0000000000000000e+00;
acadoVariables.lbAValues[26] = 1.0000000000000000e+00;
acadoVariables.lbAValues[27] = 1.0000000000000000e+00;
acadoVariables.lbAValues[28] = 1.0000000000000000e+00;
acadoVariables.lbAValues[29] = 1.0000000000000000e+00;
acadoVariables.ubAValues[0] = 1.0000000000000000e+04;
acadoVariables.ubAValues[1] = 1.0000000000000000e+04;
acadoVariables.ubAValues[2] = 1.0000000000000000e+04;
acadoVariables.ubAValues[3] = 1.0000000000000000e+04;
acadoVariables.ubAValues[4] = 1.0000000000000000e+04;
acadoVariables.ubAValues[5] = 1.0000000000000000e+04;
acadoVariables.ubAValues[6] = 1.0000000000000000e+04;
acadoVariables.ubAValues[7] = 1.0000000000000000e+04;
acadoVariables.ubAValues[8] = 1.0000000000000000e+04;
acadoVariables.ubAValues[9] = 1.0000000000000000e+04;
acadoVariables.ubAValues[10] = 1.0000000000000000e+04;
acadoVariables.ubAValues[11] = 1.0000000000000000e+04;
acadoVariables.ubAValues[12] = 1.0000000000000000e+04;
acadoVariables.ubAValues[13] = 1.0000000000000000e+04;
acadoVariables.ubAValues[14] = 1.0000000000000000e+04;
acadoVariables.ubAValues[15] = 1.0000000000000000e+04;
acadoVariables.ubAValues[16] = 1.0000000000000000e+04;
acadoVariables.ubAValues[17] = 1.0000000000000000e+04;
acadoVariables.ubAValues[18] = 1.0000000000000000e+04;
acadoVariables.ubAValues[19] = 1.0000000000000000e+04;
acadoVariables.ubAValues[20] = 1.0000000000000000e+04;
acadoVariables.ubAValues[21] = 1.0000000000000000e+04;
acadoVariables.ubAValues[22] = 1.0000000000000000e+04;
acadoVariables.ubAValues[23] = 1.0000000000000000e+04;
acadoVariables.ubAValues[24] = 1.0000000000000000e+04;
acadoVariables.ubAValues[25] = 1.0000000000000000e+04;
acadoVariables.ubAValues[26] = 1.0000000000000000e+04;
acadoVariables.ubAValues[27] = 1.0000000000000000e+04;
acadoVariables.ubAValues[28] = 1.0000000000000000e+04;
acadoVariables.ubAValues[29] = 1.0000000000000000e+04;
return ret;
}

void acado_initializeNodesByForwardSimulation(  )
{
int index;
for (index = 0; index < 30; ++index)
{
acadoWorkspace.state[0] = acadoVariables.x[index * 5];
acadoWorkspace.state[1] = acadoVariables.x[index * 5 + 1];
acadoWorkspace.state[2] = acadoVariables.x[index * 5 + 2];
acadoWorkspace.state[3] = acadoVariables.x[index * 5 + 3];
acadoWorkspace.state[4] = acadoVariables.x[index * 5 + 4];
acadoWorkspace.state[40] = acadoVariables.u[index * 2];
acadoWorkspace.state[41] = acadoVariables.u[index * 2 + 1];
acadoWorkspace.state[42] = acadoVariables.od[index * 10];
acadoWorkspace.state[43] = acadoVariables.od[index * 10 + 1];
acadoWorkspace.state[44] = acadoVariables.od[index * 10 + 2];
acadoWorkspace.state[45] = acadoVariables.od[index * 10 + 3];
acadoWorkspace.state[46] = acadoVariables.od[index * 10 + 4];
acadoWorkspace.state[47] = acadoVariables.od[index * 10 + 5];
acadoWorkspace.state[48] = acadoVariables.od[index * 10 + 6];
acadoWorkspace.state[49] = acadoVariables.od[index * 10 + 7];
acadoWorkspace.state[50] = acadoVariables.od[index * 10 + 8];
acadoWorkspace.state[51] = acadoVariables.od[index * 10 + 9];

acado_integrate(acadoWorkspace.state, index == 0);

acadoVariables.x[index * 5 + 5] = acadoWorkspace.state[0];
acadoVariables.x[index * 5 + 6] = acadoWorkspace.state[1];
acadoVariables.x[index * 5 + 7] = acadoWorkspace.state[2];
acadoVariables.x[index * 5 + 8] = acadoWorkspace.state[3];
acadoVariables.x[index * 5 + 9] = acadoWorkspace.state[4];
}
}

void acado_shiftStates( int strategy, real_t* const xEnd, real_t* const uEnd )
{
int index;
for (index = 0; index < 30; ++index)
{
acadoVariables.x[index * 5] = acadoVariables.x[index * 5 + 5];
acadoVariables.x[index * 5 + 1] = acadoVariables.x[index * 5 + 6];
acadoVariables.x[index * 5 + 2] = acadoVariables.x[index * 5 + 7];
acadoVariables.x[index * 5 + 3] = acadoVariables.x[index * 5 + 8];
acadoVariables.x[index * 5 + 4] = acadoVariables.x[index * 5 + 9];
}

if (strategy == 1 && xEnd != 0)
{
acadoVariables.x[150] = xEnd[0];
acadoVariables.x[151] = xEnd[1];
acadoVariables.x[152] = xEnd[2];
acadoVariables.x[153] = xEnd[3];
acadoVariables.x[154] = xEnd[4];
}
else if (strategy == 2) 
{
acadoWorkspace.state[0] = acadoVariables.x[150];
acadoWorkspace.state[1] = acadoVariables.x[151];
acadoWorkspace.state[2] = acadoVariables.x[152];
acadoWorkspace.state[3] = acadoVariables.x[153];
acadoWorkspace.state[4] = acadoVariables.x[154];
if (uEnd != 0)
{
acadoWorkspace.state[40] = uEnd[0];
acadoWorkspace.state[41] = uEnd[1];
}
else
{
acadoWorkspace.state[40] = acadoVariables.u[58];
acadoWorkspace.state[41] = acadoVariables.u[59];
}
acadoWorkspace.state[42] = acadoVariables.od[300];
acadoWorkspace.state[43] = acadoVariables.od[301];
acadoWorkspace.state[44] = acadoVariables.od[302];
acadoWorkspace.state[45] = acadoVariables.od[303];
acadoWorkspace.state[46] = acadoVariables.od[304];
acadoWorkspace.state[47] = acadoVariables.od[305];
acadoWorkspace.state[48] = acadoVariables.od[306];
acadoWorkspace.state[49] = acadoVariables.od[307];
acadoWorkspace.state[50] = acadoVariables.od[308];
acadoWorkspace.state[51] = acadoVariables.od[309];

acado_integrate(acadoWorkspace.state, 1);

acadoVariables.x[150] = acadoWorkspace.state[0];
acadoVariables.x[151] = acadoWorkspace.state[1];
acadoVariables.x[152] = acadoWorkspace.state[2];
acadoVariables.x[153] = acadoWorkspace.state[3];
acadoVariables.x[154] = acadoWorkspace.state[4];
}
}

void acado_shiftControls( real_t* const uEnd )
{
int index;
for (index = 0; index < 29; ++index)
{
acadoVariables.u[index * 2] = acadoVariables.u[index * 2 + 2];
acadoVariables.u[index * 2 + 1] = acadoVariables.u[index * 2 + 3];
}

if (uEnd != 0)
{
acadoVariables.u[58] = uEnd[0];
acadoVariables.u[59] = uEnd[1];
}
}

real_t acado_getKKT(  )
{
real_t kkt;

int index;
real_t prd;

kkt = + acadoWorkspace.g[0]*acadoWorkspace.x[0] + acadoWorkspace.g[1]*acadoWorkspace.x[1] + acadoWorkspace.g[2]*acadoWorkspace.x[2] + acadoWorkspace.g[3]*acadoWorkspace.x[3] + acadoWorkspace.g[4]*acadoWorkspace.x[4] + acadoWorkspace.g[5]*acadoWorkspace.x[5] + acadoWorkspace.g[6]*acadoWorkspace.x[6] + acadoWorkspace.g[7]*acadoWorkspace.x[7] + acadoWorkspace.g[8]*acadoWorkspace.x[8] + acadoWorkspace.g[9]*acadoWorkspace.x[9] + acadoWorkspace.g[10]*acadoWorkspace.x[10] + acadoWorkspace.g[11]*acadoWorkspace.x[11] + acadoWorkspace.g[12]*acadoWorkspace.x[12] + acadoWorkspace.g[13]*acadoWorkspace.x[13] + acadoWorkspace.g[14]*acadoWorkspace.x[14] + acadoWorkspace.g[15]*acadoWorkspace.x[15] + acadoWorkspace.g[16]*acadoWorkspace.x[16] + acadoWorkspace.g[17]*acadoWorkspace.x[17] + acadoWorkspace.g[18]*acadoWorkspace.x[18] + acadoWorkspace.g[19]*acadoWorkspace.x[19] + acadoWorkspace.g[20]*acadoWorkspace.x[20] + acadoWorkspace.g[21]*acadoWorkspace.x[21] + acadoWorkspace.g[22]*acadoWorkspace.x[22] + acadoWorkspace.g[23]*acadoWorkspace.x[23] + acadoWorkspace.g[24]*acadoWorkspace.x[24] + acadoWorkspace.g[25]*acadoWorkspace.x[25] + acadoWorkspace.g[26]*acadoWorkspace.x[26] + acadoWorkspace.g[27]*acadoWorkspace.x[27] + acadoWorkspace.g[28]*acadoWorkspace.x[28] + acadoWorkspace.g[29]*acadoWorkspace.x[29] + acadoWorkspace.g[30]*acadoWorkspace.x[30] + acadoWorkspace.g[31]*acadoWorkspace.x[31] + acadoWorkspace.g[32]*acadoWorkspace.x[32] + acadoWorkspace.g[33]*acadoWorkspace.x[33] + acadoWorkspace.g[34]*acadoWorkspace.x[34] + acadoWorkspace.g[35]*acadoWorkspace.x[35] + acadoWorkspace.g[36]*acadoWorkspace.x[36] + acadoWorkspace.g[37]*acadoWorkspace.x[37] + acadoWorkspace.g[38]*acadoWorkspace.x[38] + acadoWorkspace.g[39]*acadoWorkspace.x[39] + acadoWorkspace.g[40]*acadoWorkspace.x[40] + acadoWorkspace.g[41]*acadoWorkspace.x[41] + acadoWorkspace.g[42]*acadoWorkspace.x[42] + acadoWorkspace.g[43]*acadoWorkspace.x[43] + acadoWorkspace.g[44]*acadoWorkspace.x[44] + acadoWorkspace.g[45]*acadoWorkspace.x[45] + acadoWorkspace.g[46]*acadoWorkspace.x[46] + acadoWorkspace.g[47]*acadoWorkspace.x[47] + acadoWorkspace.g[48]*acadoWorkspace.x[48] + acadoWorkspace.g[49]*acadoWorkspace.x[49] + acadoWorkspace.g[50]*acadoWorkspace.x[50] + acadoWorkspace.g[51]*acadoWorkspace.x[51] + acadoWorkspace.g[52]*acadoWorkspace.x[52] + acadoWorkspace.g[53]*acadoWorkspace.x[53] + acadoWorkspace.g[54]*acadoWorkspace.x[54] + acadoWorkspace.g[55]*acadoWorkspace.x[55] + acadoWorkspace.g[56]*acadoWorkspace.x[56] + acadoWorkspace.g[57]*acadoWorkspace.x[57] + acadoWorkspace.g[58]*acadoWorkspace.x[58] + acadoWorkspace.g[59]*acadoWorkspace.x[59];
kkt = fabs( kkt );
for (index = 0; index < 60; ++index)
{
prd = acadoWorkspace.y[index];
if (prd > 1e-12)
kkt += fabs(acadoWorkspace.lb[index] * prd);
else if (prd < -1e-12)
kkt += fabs(acadoWorkspace.ub[index] * prd);
}
for (index = 0; index < 30; ++index)
{
prd = acadoWorkspace.y[index + 60];
if (prd > 1e-12)
kkt += fabs(acadoWorkspace.lbA[index] * prd);
else if (prd < -1e-12)
kkt += fabs(acadoWorkspace.ubA[index] * prd);
}
return kkt;
}

real_t acado_getObjective(  )
{
real_t objVal;

int lRun1;
/** Row vector of size: 6 */
real_t tmpDy[ 6 ];

/** Row vector of size: 4 */
real_t tmpDyN[ 4 ];

for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[lRun1 * 5];
acadoWorkspace.objValueIn[1] = acadoVariables.x[lRun1 * 5 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[lRun1 * 5 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.x[lRun1 * 5 + 3];
acadoWorkspace.objValueIn[4] = acadoVariables.x[lRun1 * 5 + 4];
acadoWorkspace.objValueIn[5] = acadoVariables.u[lRun1 * 2];
acadoWorkspace.objValueIn[6] = acadoVariables.u[lRun1 * 2 + 1];
acadoWorkspace.objValueIn[7] = acadoVariables.od[lRun1 * 10];
acadoWorkspace.objValueIn[8] = acadoVariables.od[lRun1 * 10 + 1];
acadoWorkspace.objValueIn[9] = acadoVariables.od[lRun1 * 10 + 2];
acadoWorkspace.objValueIn[10] = acadoVariables.od[lRun1 * 10 + 3];
acadoWorkspace.objValueIn[11] = acadoVariables.od[lRun1 * 10 + 4];
acadoWorkspace.objValueIn[12] = acadoVariables.od[lRun1 * 10 + 5];
acadoWorkspace.objValueIn[13] = acadoVariables.od[lRun1 * 10 + 6];
acadoWorkspace.objValueIn[14] = acadoVariables.od[lRun1 * 10 + 7];
acadoWorkspace.objValueIn[15] = acadoVariables.od[lRun1 * 10 + 8];
acadoWorkspace.objValueIn[16] = acadoVariables.od[lRun1 * 10 + 9];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[lRun1 * 6] = acadoWorkspace.objValueOut[0] - acadoVariables.y[lRun1 * 6];
acadoWorkspace.Dy[lRun1 * 6 + 1] = acadoWorkspace.objValueOut[1] - acadoVariables.y[lRun1 * 6 + 1];
acadoWorkspace.Dy[lRun1 * 6 + 2] = acadoWorkspace.objValueOut[2] - acadoVariables.y[lRun1 * 6 + 2];
acadoWorkspace.Dy[lRun1 * 6 + 3] = acadoWorkspace.objValueOut[3] - acadoVariables.y[lRun1 * 6 + 3];
acadoWorkspace.Dy[lRun1 * 6 + 4] = acadoWorkspace.objValueOut[4] - acadoVariables.y[lRun1 * 6 + 4];
acadoWorkspace.Dy[lRun1 * 6 + 5] = acadoWorkspace.objValueOut[5] - acadoVariables.y[lRun1 * 6 + 5];
}
acadoWorkspace.objValueIn[0] = acadoVariables.x[150];
acadoWorkspace.objValueIn[1] = acadoVariables.x[151];
acadoWorkspace.objValueIn[2] = acadoVariables.x[152];
acadoWorkspace.objValueIn[3] = acadoVariables.x[153];
acadoWorkspace.objValueIn[4] = acadoVariables.x[154];
acadoWorkspace.objValueIn[5] = acadoVariables.od[300];
acadoWorkspace.objValueIn[6] = acadoVariables.od[301];
acadoWorkspace.objValueIn[7] = acadoVariables.od[302];
acadoWorkspace.objValueIn[8] = acadoVariables.od[303];
acadoWorkspace.objValueIn[9] = acadoVariables.od[304];
acadoWorkspace.objValueIn[10] = acadoVariables.od[305];
acadoWorkspace.objValueIn[11] = acadoVariables.od[306];
acadoWorkspace.objValueIn[12] = acadoVariables.od[307];
acadoWorkspace.objValueIn[13] = acadoVariables.od[308];
acadoWorkspace.objValueIn[14] = acadoVariables.od[309];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0] - acadoVariables.yN[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1] - acadoVariables.yN[1];
acadoWorkspace.DyN[2] = acadoWorkspace.objValueOut[2] - acadoVariables.yN[2];
acadoWorkspace.DyN[3] = acadoWorkspace.objValueOut[3] - acadoVariables.yN[3];
objVal = 0.0000000000000000e+00;
for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
tmpDy[0] = + acadoWorkspace.Dy[lRun1 * 6]*acadoVariables.W[0];
tmpDy[1] = + acadoWorkspace.Dy[lRun1 * 6 + 1]*acadoVariables.W[7];
tmpDy[2] = + acadoWorkspace.Dy[lRun1 * 6 + 2]*acadoVariables.W[14];
tmpDy[3] = + acadoWorkspace.Dy[lRun1 * 6 + 3]*acadoVariables.W[21];
tmpDy[4] = + acadoWorkspace.Dy[lRun1 * 6 + 4]*acadoVariables.W[28];
tmpDy[5] = + acadoWorkspace.Dy[lRun1 * 6 + 5]*acadoVariables.W[35];
objVal += + acadoWorkspace.Dy[lRun1 * 6]*tmpDy[0] + acadoWorkspace.Dy[lRun1 * 6 + 1]*tmpDy[1] + acadoWorkspace.Dy[lRun1 * 6 + 2]*tmpDy[2] + acadoWorkspace.Dy[lRun1 * 6 + 3]*tmpDy[3] + acadoWorkspace.Dy[lRun1 * 6 + 4]*tmpDy[4] + acadoWorkspace.Dy[lRun1 * 6 + 5]*tmpDy[5];
}

tmpDyN[0] = + acadoWorkspace.DyN[0]*acadoVariables.WN[0];
tmpDyN[1] = + acadoWorkspace.DyN[1]*acadoVariables.WN[5];
tmpDyN[2] = + acadoWorkspace.DyN[2]*acadoVariables.WN[10];
tmpDyN[3] = + acadoWorkspace.DyN[3]*acadoVariables.WN[15];
objVal += + acadoWorkspace.DyN[0]*tmpDyN[0] + acadoWorkspace.DyN[1]*tmpDyN[1] + acadoWorkspace.DyN[2]*tmpDyN[2] + acadoWorkspace.DyN[3]*tmpDyN[3];

objVal *= 0.5;
return objVal;
}

