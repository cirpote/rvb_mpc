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
for (lRun1 = 0; lRun1 < 60; ++lRun1)
{
acadoWorkspace.state[0] = acadoVariables.x[lRun1 * 4];
acadoWorkspace.state[1] = acadoVariables.x[lRun1 * 4 + 1];
acadoWorkspace.state[2] = acadoVariables.x[lRun1 * 4 + 2];
acadoWorkspace.state[3] = acadoVariables.x[lRun1 * 4 + 3];

acadoWorkspace.state[28] = acadoVariables.u[lRun1 * 2];
acadoWorkspace.state[29] = acadoVariables.u[lRun1 * 2 + 1];
acadoWorkspace.state[30] = acadoVariables.od[lRun1 * 13];
acadoWorkspace.state[31] = acadoVariables.od[lRun1 * 13 + 1];
acadoWorkspace.state[32] = acadoVariables.od[lRun1 * 13 + 2];
acadoWorkspace.state[33] = acadoVariables.od[lRun1 * 13 + 3];
acadoWorkspace.state[34] = acadoVariables.od[lRun1 * 13 + 4];
acadoWorkspace.state[35] = acadoVariables.od[lRun1 * 13 + 5];
acadoWorkspace.state[36] = acadoVariables.od[lRun1 * 13 + 6];
acadoWorkspace.state[37] = acadoVariables.od[lRun1 * 13 + 7];
acadoWorkspace.state[38] = acadoVariables.od[lRun1 * 13 + 8];
acadoWorkspace.state[39] = acadoVariables.od[lRun1 * 13 + 9];
acadoWorkspace.state[40] = acadoVariables.od[lRun1 * 13 + 10];
acadoWorkspace.state[41] = acadoVariables.od[lRun1 * 13 + 11];
acadoWorkspace.state[42] = acadoVariables.od[lRun1 * 13 + 12];

ret = acado_integrate(acadoWorkspace.state, 1);

acadoWorkspace.d[lRun1 * 4] = acadoWorkspace.state[0] - acadoVariables.x[lRun1 * 4 + 4];
acadoWorkspace.d[lRun1 * 4 + 1] = acadoWorkspace.state[1] - acadoVariables.x[lRun1 * 4 + 5];
acadoWorkspace.d[lRun1 * 4 + 2] = acadoWorkspace.state[2] - acadoVariables.x[lRun1 * 4 + 6];
acadoWorkspace.d[lRun1 * 4 + 3] = acadoWorkspace.state[3] - acadoVariables.x[lRun1 * 4 + 7];

acadoWorkspace.evGx[lRun1 * 16] = acadoWorkspace.state[4];
acadoWorkspace.evGx[lRun1 * 16 + 1] = acadoWorkspace.state[5];
acadoWorkspace.evGx[lRun1 * 16 + 2] = acadoWorkspace.state[6];
acadoWorkspace.evGx[lRun1 * 16 + 3] = acadoWorkspace.state[7];
acadoWorkspace.evGx[lRun1 * 16 + 4] = acadoWorkspace.state[8];
acadoWorkspace.evGx[lRun1 * 16 + 5] = acadoWorkspace.state[9];
acadoWorkspace.evGx[lRun1 * 16 + 6] = acadoWorkspace.state[10];
acadoWorkspace.evGx[lRun1 * 16 + 7] = acadoWorkspace.state[11];
acadoWorkspace.evGx[lRun1 * 16 + 8] = acadoWorkspace.state[12];
acadoWorkspace.evGx[lRun1 * 16 + 9] = acadoWorkspace.state[13];
acadoWorkspace.evGx[lRun1 * 16 + 10] = acadoWorkspace.state[14];
acadoWorkspace.evGx[lRun1 * 16 + 11] = acadoWorkspace.state[15];
acadoWorkspace.evGx[lRun1 * 16 + 12] = acadoWorkspace.state[16];
acadoWorkspace.evGx[lRun1 * 16 + 13] = acadoWorkspace.state[17];
acadoWorkspace.evGx[lRun1 * 16 + 14] = acadoWorkspace.state[18];
acadoWorkspace.evGx[lRun1 * 16 + 15] = acadoWorkspace.state[19];

acadoWorkspace.evGu[lRun1 * 8] = acadoWorkspace.state[20];
acadoWorkspace.evGu[lRun1 * 8 + 1] = acadoWorkspace.state[21];
acadoWorkspace.evGu[lRun1 * 8 + 2] = acadoWorkspace.state[22];
acadoWorkspace.evGu[lRun1 * 8 + 3] = acadoWorkspace.state[23];
acadoWorkspace.evGu[lRun1 * 8 + 4] = acadoWorkspace.state[24];
acadoWorkspace.evGu[lRun1 * 8 + 5] = acadoWorkspace.state[25];
acadoWorkspace.evGu[lRun1 * 8 + 6] = acadoWorkspace.state[26];
acadoWorkspace.evGu[lRun1 * 8 + 7] = acadoWorkspace.state[27];
}
return ret;
}

void acado_evaluateLSQ(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* od = in + 6;
/* Vector of auxiliary variables; number of elements: 30. */
real_t* a = acadoWorkspace.objAuxVar;

/* Compute intermediate quantities: */
a[0] = (((xd[0]-od[0])*(xd[0]-od[0]))+((xd[1]-od[1])*(xd[1]-od[1])));
a[1] = (((xd[0]-od[2])*(xd[0]-od[2]))+((xd[1]-od[3])*(xd[1]-od[3])));
a[2] = (((xd[0]-od[4])*(xd[0]-od[4]))+((xd[1]-od[5])*(xd[1]-od[5])));
a[3] = (((xd[0]-od[6])*(xd[0]-od[6]))+((xd[1]-od[7])*(xd[1]-od[7])));
a[4] = (((xd[0]-od[8])*(xd[0]-od[8]))+((xd[1]-od[9])*(xd[1]-od[9])));
a[5] = (((xd[0]-od[10])*(xd[0]-od[10]))+((xd[1]-od[11])*(xd[1]-od[11])));
a[6] = ((xd[0]-od[0])+(xd[0]-od[0]));
a[7] = ((real_t)(1.0000000000000000e+00)/a[0]);
a[8] = (a[7]*a[7]);
a[9] = ((xd[1]-od[1])+(xd[1]-od[1]));
a[10] = ((xd[0]-od[2])+(xd[0]-od[2]));
a[11] = ((real_t)(1.0000000000000000e+00)/a[1]);
a[12] = (a[11]*a[11]);
a[13] = ((xd[1]-od[3])+(xd[1]-od[3]));
a[14] = ((xd[0]-od[4])+(xd[0]-od[4]));
a[15] = ((real_t)(1.0000000000000000e+00)/a[2]);
a[16] = (a[15]*a[15]);
a[17] = ((xd[1]-od[5])+(xd[1]-od[5]));
a[18] = ((xd[0]-od[6])+(xd[0]-od[6]));
a[19] = ((real_t)(1.0000000000000000e+00)/a[3]);
a[20] = (a[19]*a[19]);
a[21] = ((xd[1]-od[7])+(xd[1]-od[7]));
a[22] = ((xd[0]-od[8])+(xd[0]-od[8]));
a[23] = ((real_t)(1.0000000000000000e+00)/a[4]);
a[24] = (a[23]*a[23]);
a[25] = ((xd[1]-od[9])+(xd[1]-od[9]));
a[26] = ((xd[0]-od[10])+(xd[0]-od[10]));
a[27] = ((real_t)(1.0000000000000000e+00)/a[5]);
a[28] = (a[27]*a[27]);
a[29] = ((xd[1]-od[11])+(xd[1]-od[11]));

/* Compute outputs: */
out[0] = xd[0];
out[1] = xd[1];
out[2] = xd[2];
out[3] = ((real_t)(1.0000000000000000e+00)/a[0]);
out[4] = ((real_t)(1.0000000000000000e+00)/a[1]);
out[5] = ((real_t)(1.0000000000000000e+00)/a[2]);
out[6] = ((real_t)(1.0000000000000000e+00)/a[3]);
out[7] = ((real_t)(1.0000000000000000e+00)/a[4]);
out[8] = ((real_t)(1.0000000000000000e+00)/a[5]);
out[9] = (real_t)(1.0000000000000000e+00);
out[10] = (real_t)(0.0000000000000000e+00);
out[11] = (real_t)(0.0000000000000000e+00);
out[12] = (real_t)(0.0000000000000000e+00);
out[13] = (real_t)(0.0000000000000000e+00);
out[14] = (real_t)(1.0000000000000000e+00);
out[15] = (real_t)(0.0000000000000000e+00);
out[16] = (real_t)(0.0000000000000000e+00);
out[17] = (real_t)(0.0000000000000000e+00);
out[18] = (real_t)(0.0000000000000000e+00);
out[19] = (real_t)(1.0000000000000000e+00);
out[20] = (real_t)(0.0000000000000000e+00);
out[21] = ((real_t)(0.0000000000000000e+00)-(a[6]*a[8]));
out[22] = ((real_t)(0.0000000000000000e+00)-(a[9]*a[8]));
out[23] = (real_t)(0.0000000000000000e+00);
out[24] = (real_t)(0.0000000000000000e+00);
out[25] = ((real_t)(0.0000000000000000e+00)-(a[10]*a[12]));
out[26] = ((real_t)(0.0000000000000000e+00)-(a[13]*a[12]));
out[27] = (real_t)(0.0000000000000000e+00);
out[28] = (real_t)(0.0000000000000000e+00);
out[29] = ((real_t)(0.0000000000000000e+00)-(a[14]*a[16]));
out[30] = ((real_t)(0.0000000000000000e+00)-(a[17]*a[16]));
out[31] = (real_t)(0.0000000000000000e+00);
out[32] = (real_t)(0.0000000000000000e+00);
out[33] = ((real_t)(0.0000000000000000e+00)-(a[18]*a[20]));
out[34] = ((real_t)(0.0000000000000000e+00)-(a[21]*a[20]));
out[35] = (real_t)(0.0000000000000000e+00);
out[36] = (real_t)(0.0000000000000000e+00);
out[37] = ((real_t)(0.0000000000000000e+00)-(a[22]*a[24]));
out[38] = ((real_t)(0.0000000000000000e+00)-(a[25]*a[24]));
out[39] = (real_t)(0.0000000000000000e+00);
out[40] = (real_t)(0.0000000000000000e+00);
out[41] = ((real_t)(0.0000000000000000e+00)-(a[26]*a[28]));
out[42] = ((real_t)(0.0000000000000000e+00)-(a[29]*a[28]));
out[43] = (real_t)(0.0000000000000000e+00);
out[44] = (real_t)(0.0000000000000000e+00);
}

void acado_evaluateLSQEndTerm(const real_t* in, real_t* out)
{
const real_t* xd = in;

/* Compute outputs: */
out[0] = xd[0];
out[1] = xd[1];
out[2] = xd[2];
}

void acado_setObjQ1Q2( real_t* const tmpFx, real_t* const tmpObjS, real_t* const tmpQ1, real_t* const tmpQ2 )
{
tmpQ2[0] = + tmpFx[0]*tmpObjS[0] + tmpFx[4]*tmpObjS[9] + tmpFx[8]*tmpObjS[18] + tmpFx[12]*tmpObjS[27] + tmpFx[16]*tmpObjS[36] + tmpFx[20]*tmpObjS[45] + tmpFx[24]*tmpObjS[54] + tmpFx[28]*tmpObjS[63] + tmpFx[32]*tmpObjS[72];
tmpQ2[1] = + tmpFx[0]*tmpObjS[1] + tmpFx[4]*tmpObjS[10] + tmpFx[8]*tmpObjS[19] + tmpFx[12]*tmpObjS[28] + tmpFx[16]*tmpObjS[37] + tmpFx[20]*tmpObjS[46] + tmpFx[24]*tmpObjS[55] + tmpFx[28]*tmpObjS[64] + tmpFx[32]*tmpObjS[73];
tmpQ2[2] = + tmpFx[0]*tmpObjS[2] + tmpFx[4]*tmpObjS[11] + tmpFx[8]*tmpObjS[20] + tmpFx[12]*tmpObjS[29] + tmpFx[16]*tmpObjS[38] + tmpFx[20]*tmpObjS[47] + tmpFx[24]*tmpObjS[56] + tmpFx[28]*tmpObjS[65] + tmpFx[32]*tmpObjS[74];
tmpQ2[3] = + tmpFx[0]*tmpObjS[3] + tmpFx[4]*tmpObjS[12] + tmpFx[8]*tmpObjS[21] + tmpFx[12]*tmpObjS[30] + tmpFx[16]*tmpObjS[39] + tmpFx[20]*tmpObjS[48] + tmpFx[24]*tmpObjS[57] + tmpFx[28]*tmpObjS[66] + tmpFx[32]*tmpObjS[75];
tmpQ2[4] = + tmpFx[0]*tmpObjS[4] + tmpFx[4]*tmpObjS[13] + tmpFx[8]*tmpObjS[22] + tmpFx[12]*tmpObjS[31] + tmpFx[16]*tmpObjS[40] + tmpFx[20]*tmpObjS[49] + tmpFx[24]*tmpObjS[58] + tmpFx[28]*tmpObjS[67] + tmpFx[32]*tmpObjS[76];
tmpQ2[5] = + tmpFx[0]*tmpObjS[5] + tmpFx[4]*tmpObjS[14] + tmpFx[8]*tmpObjS[23] + tmpFx[12]*tmpObjS[32] + tmpFx[16]*tmpObjS[41] + tmpFx[20]*tmpObjS[50] + tmpFx[24]*tmpObjS[59] + tmpFx[28]*tmpObjS[68] + tmpFx[32]*tmpObjS[77];
tmpQ2[6] = + tmpFx[0]*tmpObjS[6] + tmpFx[4]*tmpObjS[15] + tmpFx[8]*tmpObjS[24] + tmpFx[12]*tmpObjS[33] + tmpFx[16]*tmpObjS[42] + tmpFx[20]*tmpObjS[51] + tmpFx[24]*tmpObjS[60] + tmpFx[28]*tmpObjS[69] + tmpFx[32]*tmpObjS[78];
tmpQ2[7] = + tmpFx[0]*tmpObjS[7] + tmpFx[4]*tmpObjS[16] + tmpFx[8]*tmpObjS[25] + tmpFx[12]*tmpObjS[34] + tmpFx[16]*tmpObjS[43] + tmpFx[20]*tmpObjS[52] + tmpFx[24]*tmpObjS[61] + tmpFx[28]*tmpObjS[70] + tmpFx[32]*tmpObjS[79];
tmpQ2[8] = + tmpFx[0]*tmpObjS[8] + tmpFx[4]*tmpObjS[17] + tmpFx[8]*tmpObjS[26] + tmpFx[12]*tmpObjS[35] + tmpFx[16]*tmpObjS[44] + tmpFx[20]*tmpObjS[53] + tmpFx[24]*tmpObjS[62] + tmpFx[28]*tmpObjS[71] + tmpFx[32]*tmpObjS[80];
tmpQ2[9] = + tmpFx[1]*tmpObjS[0] + tmpFx[5]*tmpObjS[9] + tmpFx[9]*tmpObjS[18] + tmpFx[13]*tmpObjS[27] + tmpFx[17]*tmpObjS[36] + tmpFx[21]*tmpObjS[45] + tmpFx[25]*tmpObjS[54] + tmpFx[29]*tmpObjS[63] + tmpFx[33]*tmpObjS[72];
tmpQ2[10] = + tmpFx[1]*tmpObjS[1] + tmpFx[5]*tmpObjS[10] + tmpFx[9]*tmpObjS[19] + tmpFx[13]*tmpObjS[28] + tmpFx[17]*tmpObjS[37] + tmpFx[21]*tmpObjS[46] + tmpFx[25]*tmpObjS[55] + tmpFx[29]*tmpObjS[64] + tmpFx[33]*tmpObjS[73];
tmpQ2[11] = + tmpFx[1]*tmpObjS[2] + tmpFx[5]*tmpObjS[11] + tmpFx[9]*tmpObjS[20] + tmpFx[13]*tmpObjS[29] + tmpFx[17]*tmpObjS[38] + tmpFx[21]*tmpObjS[47] + tmpFx[25]*tmpObjS[56] + tmpFx[29]*tmpObjS[65] + tmpFx[33]*tmpObjS[74];
tmpQ2[12] = + tmpFx[1]*tmpObjS[3] + tmpFx[5]*tmpObjS[12] + tmpFx[9]*tmpObjS[21] + tmpFx[13]*tmpObjS[30] + tmpFx[17]*tmpObjS[39] + tmpFx[21]*tmpObjS[48] + tmpFx[25]*tmpObjS[57] + tmpFx[29]*tmpObjS[66] + tmpFx[33]*tmpObjS[75];
tmpQ2[13] = + tmpFx[1]*tmpObjS[4] + tmpFx[5]*tmpObjS[13] + tmpFx[9]*tmpObjS[22] + tmpFx[13]*tmpObjS[31] + tmpFx[17]*tmpObjS[40] + tmpFx[21]*tmpObjS[49] + tmpFx[25]*tmpObjS[58] + tmpFx[29]*tmpObjS[67] + tmpFx[33]*tmpObjS[76];
tmpQ2[14] = + tmpFx[1]*tmpObjS[5] + tmpFx[5]*tmpObjS[14] + tmpFx[9]*tmpObjS[23] + tmpFx[13]*tmpObjS[32] + tmpFx[17]*tmpObjS[41] + tmpFx[21]*tmpObjS[50] + tmpFx[25]*tmpObjS[59] + tmpFx[29]*tmpObjS[68] + tmpFx[33]*tmpObjS[77];
tmpQ2[15] = + tmpFx[1]*tmpObjS[6] + tmpFx[5]*tmpObjS[15] + tmpFx[9]*tmpObjS[24] + tmpFx[13]*tmpObjS[33] + tmpFx[17]*tmpObjS[42] + tmpFx[21]*tmpObjS[51] + tmpFx[25]*tmpObjS[60] + tmpFx[29]*tmpObjS[69] + tmpFx[33]*tmpObjS[78];
tmpQ2[16] = + tmpFx[1]*tmpObjS[7] + tmpFx[5]*tmpObjS[16] + tmpFx[9]*tmpObjS[25] + tmpFx[13]*tmpObjS[34] + tmpFx[17]*tmpObjS[43] + tmpFx[21]*tmpObjS[52] + tmpFx[25]*tmpObjS[61] + tmpFx[29]*tmpObjS[70] + tmpFx[33]*tmpObjS[79];
tmpQ2[17] = + tmpFx[1]*tmpObjS[8] + tmpFx[5]*tmpObjS[17] + tmpFx[9]*tmpObjS[26] + tmpFx[13]*tmpObjS[35] + tmpFx[17]*tmpObjS[44] + tmpFx[21]*tmpObjS[53] + tmpFx[25]*tmpObjS[62] + tmpFx[29]*tmpObjS[71] + tmpFx[33]*tmpObjS[80];
tmpQ2[18] = + tmpFx[2]*tmpObjS[0] + tmpFx[6]*tmpObjS[9] + tmpFx[10]*tmpObjS[18] + tmpFx[14]*tmpObjS[27] + tmpFx[18]*tmpObjS[36] + tmpFx[22]*tmpObjS[45] + tmpFx[26]*tmpObjS[54] + tmpFx[30]*tmpObjS[63] + tmpFx[34]*tmpObjS[72];
tmpQ2[19] = + tmpFx[2]*tmpObjS[1] + tmpFx[6]*tmpObjS[10] + tmpFx[10]*tmpObjS[19] + tmpFx[14]*tmpObjS[28] + tmpFx[18]*tmpObjS[37] + tmpFx[22]*tmpObjS[46] + tmpFx[26]*tmpObjS[55] + tmpFx[30]*tmpObjS[64] + tmpFx[34]*tmpObjS[73];
tmpQ2[20] = + tmpFx[2]*tmpObjS[2] + tmpFx[6]*tmpObjS[11] + tmpFx[10]*tmpObjS[20] + tmpFx[14]*tmpObjS[29] + tmpFx[18]*tmpObjS[38] + tmpFx[22]*tmpObjS[47] + tmpFx[26]*tmpObjS[56] + tmpFx[30]*tmpObjS[65] + tmpFx[34]*tmpObjS[74];
tmpQ2[21] = + tmpFx[2]*tmpObjS[3] + tmpFx[6]*tmpObjS[12] + tmpFx[10]*tmpObjS[21] + tmpFx[14]*tmpObjS[30] + tmpFx[18]*tmpObjS[39] + tmpFx[22]*tmpObjS[48] + tmpFx[26]*tmpObjS[57] + tmpFx[30]*tmpObjS[66] + tmpFx[34]*tmpObjS[75];
tmpQ2[22] = + tmpFx[2]*tmpObjS[4] + tmpFx[6]*tmpObjS[13] + tmpFx[10]*tmpObjS[22] + tmpFx[14]*tmpObjS[31] + tmpFx[18]*tmpObjS[40] + tmpFx[22]*tmpObjS[49] + tmpFx[26]*tmpObjS[58] + tmpFx[30]*tmpObjS[67] + tmpFx[34]*tmpObjS[76];
tmpQ2[23] = + tmpFx[2]*tmpObjS[5] + tmpFx[6]*tmpObjS[14] + tmpFx[10]*tmpObjS[23] + tmpFx[14]*tmpObjS[32] + tmpFx[18]*tmpObjS[41] + tmpFx[22]*tmpObjS[50] + tmpFx[26]*tmpObjS[59] + tmpFx[30]*tmpObjS[68] + tmpFx[34]*tmpObjS[77];
tmpQ2[24] = + tmpFx[2]*tmpObjS[6] + tmpFx[6]*tmpObjS[15] + tmpFx[10]*tmpObjS[24] + tmpFx[14]*tmpObjS[33] + tmpFx[18]*tmpObjS[42] + tmpFx[22]*tmpObjS[51] + tmpFx[26]*tmpObjS[60] + tmpFx[30]*tmpObjS[69] + tmpFx[34]*tmpObjS[78];
tmpQ2[25] = + tmpFx[2]*tmpObjS[7] + tmpFx[6]*tmpObjS[16] + tmpFx[10]*tmpObjS[25] + tmpFx[14]*tmpObjS[34] + tmpFx[18]*tmpObjS[43] + tmpFx[22]*tmpObjS[52] + tmpFx[26]*tmpObjS[61] + tmpFx[30]*tmpObjS[70] + tmpFx[34]*tmpObjS[79];
tmpQ2[26] = + tmpFx[2]*tmpObjS[8] + tmpFx[6]*tmpObjS[17] + tmpFx[10]*tmpObjS[26] + tmpFx[14]*tmpObjS[35] + tmpFx[18]*tmpObjS[44] + tmpFx[22]*tmpObjS[53] + tmpFx[26]*tmpObjS[62] + tmpFx[30]*tmpObjS[71] + tmpFx[34]*tmpObjS[80];
tmpQ2[27] = + tmpFx[3]*tmpObjS[0] + tmpFx[7]*tmpObjS[9] + tmpFx[11]*tmpObjS[18] + tmpFx[15]*tmpObjS[27] + tmpFx[19]*tmpObjS[36] + tmpFx[23]*tmpObjS[45] + tmpFx[27]*tmpObjS[54] + tmpFx[31]*tmpObjS[63] + tmpFx[35]*tmpObjS[72];
tmpQ2[28] = + tmpFx[3]*tmpObjS[1] + tmpFx[7]*tmpObjS[10] + tmpFx[11]*tmpObjS[19] + tmpFx[15]*tmpObjS[28] + tmpFx[19]*tmpObjS[37] + tmpFx[23]*tmpObjS[46] + tmpFx[27]*tmpObjS[55] + tmpFx[31]*tmpObjS[64] + tmpFx[35]*tmpObjS[73];
tmpQ2[29] = + tmpFx[3]*tmpObjS[2] + tmpFx[7]*tmpObjS[11] + tmpFx[11]*tmpObjS[20] + tmpFx[15]*tmpObjS[29] + tmpFx[19]*tmpObjS[38] + tmpFx[23]*tmpObjS[47] + tmpFx[27]*tmpObjS[56] + tmpFx[31]*tmpObjS[65] + tmpFx[35]*tmpObjS[74];
tmpQ2[30] = + tmpFx[3]*tmpObjS[3] + tmpFx[7]*tmpObjS[12] + tmpFx[11]*tmpObjS[21] + tmpFx[15]*tmpObjS[30] + tmpFx[19]*tmpObjS[39] + tmpFx[23]*tmpObjS[48] + tmpFx[27]*tmpObjS[57] + tmpFx[31]*tmpObjS[66] + tmpFx[35]*tmpObjS[75];
tmpQ2[31] = + tmpFx[3]*tmpObjS[4] + tmpFx[7]*tmpObjS[13] + tmpFx[11]*tmpObjS[22] + tmpFx[15]*tmpObjS[31] + tmpFx[19]*tmpObjS[40] + tmpFx[23]*tmpObjS[49] + tmpFx[27]*tmpObjS[58] + tmpFx[31]*tmpObjS[67] + tmpFx[35]*tmpObjS[76];
tmpQ2[32] = + tmpFx[3]*tmpObjS[5] + tmpFx[7]*tmpObjS[14] + tmpFx[11]*tmpObjS[23] + tmpFx[15]*tmpObjS[32] + tmpFx[19]*tmpObjS[41] + tmpFx[23]*tmpObjS[50] + tmpFx[27]*tmpObjS[59] + tmpFx[31]*tmpObjS[68] + tmpFx[35]*tmpObjS[77];
tmpQ2[33] = + tmpFx[3]*tmpObjS[6] + tmpFx[7]*tmpObjS[15] + tmpFx[11]*tmpObjS[24] + tmpFx[15]*tmpObjS[33] + tmpFx[19]*tmpObjS[42] + tmpFx[23]*tmpObjS[51] + tmpFx[27]*tmpObjS[60] + tmpFx[31]*tmpObjS[69] + tmpFx[35]*tmpObjS[78];
tmpQ2[34] = + tmpFx[3]*tmpObjS[7] + tmpFx[7]*tmpObjS[16] + tmpFx[11]*tmpObjS[25] + tmpFx[15]*tmpObjS[34] + tmpFx[19]*tmpObjS[43] + tmpFx[23]*tmpObjS[52] + tmpFx[27]*tmpObjS[61] + tmpFx[31]*tmpObjS[70] + tmpFx[35]*tmpObjS[79];
tmpQ2[35] = + tmpFx[3]*tmpObjS[8] + tmpFx[7]*tmpObjS[17] + tmpFx[11]*tmpObjS[26] + tmpFx[15]*tmpObjS[35] + tmpFx[19]*tmpObjS[44] + tmpFx[23]*tmpObjS[53] + tmpFx[27]*tmpObjS[62] + tmpFx[31]*tmpObjS[71] + tmpFx[35]*tmpObjS[80];
tmpQ1[0] = + tmpQ2[0]*tmpFx[0] + tmpQ2[1]*tmpFx[4] + tmpQ2[2]*tmpFx[8] + tmpQ2[3]*tmpFx[12] + tmpQ2[4]*tmpFx[16] + tmpQ2[5]*tmpFx[20] + tmpQ2[6]*tmpFx[24] + tmpQ2[7]*tmpFx[28] + tmpQ2[8]*tmpFx[32];
tmpQ1[1] = + tmpQ2[0]*tmpFx[1] + tmpQ2[1]*tmpFx[5] + tmpQ2[2]*tmpFx[9] + tmpQ2[3]*tmpFx[13] + tmpQ2[4]*tmpFx[17] + tmpQ2[5]*tmpFx[21] + tmpQ2[6]*tmpFx[25] + tmpQ2[7]*tmpFx[29] + tmpQ2[8]*tmpFx[33];
tmpQ1[2] = + tmpQ2[0]*tmpFx[2] + tmpQ2[1]*tmpFx[6] + tmpQ2[2]*tmpFx[10] + tmpQ2[3]*tmpFx[14] + tmpQ2[4]*tmpFx[18] + tmpQ2[5]*tmpFx[22] + tmpQ2[6]*tmpFx[26] + tmpQ2[7]*tmpFx[30] + tmpQ2[8]*tmpFx[34];
tmpQ1[3] = + tmpQ2[0]*tmpFx[3] + tmpQ2[1]*tmpFx[7] + tmpQ2[2]*tmpFx[11] + tmpQ2[3]*tmpFx[15] + tmpQ2[4]*tmpFx[19] + tmpQ2[5]*tmpFx[23] + tmpQ2[6]*tmpFx[27] + tmpQ2[7]*tmpFx[31] + tmpQ2[8]*tmpFx[35];
tmpQ1[4] = + tmpQ2[9]*tmpFx[0] + tmpQ2[10]*tmpFx[4] + tmpQ2[11]*tmpFx[8] + tmpQ2[12]*tmpFx[12] + tmpQ2[13]*tmpFx[16] + tmpQ2[14]*tmpFx[20] + tmpQ2[15]*tmpFx[24] + tmpQ2[16]*tmpFx[28] + tmpQ2[17]*tmpFx[32];
tmpQ1[5] = + tmpQ2[9]*tmpFx[1] + tmpQ2[10]*tmpFx[5] + tmpQ2[11]*tmpFx[9] + tmpQ2[12]*tmpFx[13] + tmpQ2[13]*tmpFx[17] + tmpQ2[14]*tmpFx[21] + tmpQ2[15]*tmpFx[25] + tmpQ2[16]*tmpFx[29] + tmpQ2[17]*tmpFx[33];
tmpQ1[6] = + tmpQ2[9]*tmpFx[2] + tmpQ2[10]*tmpFx[6] + tmpQ2[11]*tmpFx[10] + tmpQ2[12]*tmpFx[14] + tmpQ2[13]*tmpFx[18] + tmpQ2[14]*tmpFx[22] + tmpQ2[15]*tmpFx[26] + tmpQ2[16]*tmpFx[30] + tmpQ2[17]*tmpFx[34];
tmpQ1[7] = + tmpQ2[9]*tmpFx[3] + tmpQ2[10]*tmpFx[7] + tmpQ2[11]*tmpFx[11] + tmpQ2[12]*tmpFx[15] + tmpQ2[13]*tmpFx[19] + tmpQ2[14]*tmpFx[23] + tmpQ2[15]*tmpFx[27] + tmpQ2[16]*tmpFx[31] + tmpQ2[17]*tmpFx[35];
tmpQ1[8] = + tmpQ2[18]*tmpFx[0] + tmpQ2[19]*tmpFx[4] + tmpQ2[20]*tmpFx[8] + tmpQ2[21]*tmpFx[12] + tmpQ2[22]*tmpFx[16] + tmpQ2[23]*tmpFx[20] + tmpQ2[24]*tmpFx[24] + tmpQ2[25]*tmpFx[28] + tmpQ2[26]*tmpFx[32];
tmpQ1[9] = + tmpQ2[18]*tmpFx[1] + tmpQ2[19]*tmpFx[5] + tmpQ2[20]*tmpFx[9] + tmpQ2[21]*tmpFx[13] + tmpQ2[22]*tmpFx[17] + tmpQ2[23]*tmpFx[21] + tmpQ2[24]*tmpFx[25] + tmpQ2[25]*tmpFx[29] + tmpQ2[26]*tmpFx[33];
tmpQ1[10] = + tmpQ2[18]*tmpFx[2] + tmpQ2[19]*tmpFx[6] + tmpQ2[20]*tmpFx[10] + tmpQ2[21]*tmpFx[14] + tmpQ2[22]*tmpFx[18] + tmpQ2[23]*tmpFx[22] + tmpQ2[24]*tmpFx[26] + tmpQ2[25]*tmpFx[30] + tmpQ2[26]*tmpFx[34];
tmpQ1[11] = + tmpQ2[18]*tmpFx[3] + tmpQ2[19]*tmpFx[7] + tmpQ2[20]*tmpFx[11] + tmpQ2[21]*tmpFx[15] + tmpQ2[22]*tmpFx[19] + tmpQ2[23]*tmpFx[23] + tmpQ2[24]*tmpFx[27] + tmpQ2[25]*tmpFx[31] + tmpQ2[26]*tmpFx[35];
tmpQ1[12] = + tmpQ2[27]*tmpFx[0] + tmpQ2[28]*tmpFx[4] + tmpQ2[29]*tmpFx[8] + tmpQ2[30]*tmpFx[12] + tmpQ2[31]*tmpFx[16] + tmpQ2[32]*tmpFx[20] + tmpQ2[33]*tmpFx[24] + tmpQ2[34]*tmpFx[28] + tmpQ2[35]*tmpFx[32];
tmpQ1[13] = + tmpQ2[27]*tmpFx[1] + tmpQ2[28]*tmpFx[5] + tmpQ2[29]*tmpFx[9] + tmpQ2[30]*tmpFx[13] + tmpQ2[31]*tmpFx[17] + tmpQ2[32]*tmpFx[21] + tmpQ2[33]*tmpFx[25] + tmpQ2[34]*tmpFx[29] + tmpQ2[35]*tmpFx[33];
tmpQ1[14] = + tmpQ2[27]*tmpFx[2] + tmpQ2[28]*tmpFx[6] + tmpQ2[29]*tmpFx[10] + tmpQ2[30]*tmpFx[14] + tmpQ2[31]*tmpFx[18] + tmpQ2[32]*tmpFx[22] + tmpQ2[33]*tmpFx[26] + tmpQ2[34]*tmpFx[30] + tmpQ2[35]*tmpFx[34];
tmpQ1[15] = + tmpQ2[27]*tmpFx[3] + tmpQ2[28]*tmpFx[7] + tmpQ2[29]*tmpFx[11] + tmpQ2[30]*tmpFx[15] + tmpQ2[31]*tmpFx[19] + tmpQ2[32]*tmpFx[23] + tmpQ2[33]*tmpFx[27] + tmpQ2[34]*tmpFx[31] + tmpQ2[35]*tmpFx[35];
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
tmpQN2[9] = 0.0;
;
tmpQN2[10] = 0.0;
;
tmpQN2[11] = 0.0;
;
tmpQN1[0] = + tmpQN2[0];
tmpQN1[1] = + tmpQN2[1];
tmpQN1[2] = + tmpQN2[2];
tmpQN1[3] = 0.0;
;
tmpQN1[4] = + tmpQN2[3];
tmpQN1[5] = + tmpQN2[4];
tmpQN1[6] = + tmpQN2[5];
tmpQN1[7] = 0.0;
;
tmpQN1[8] = + tmpQN2[6];
tmpQN1[9] = + tmpQN2[7];
tmpQN1[10] = + tmpQN2[8];
tmpQN1[11] = 0.0;
;
tmpQN1[12] = + tmpQN2[9];
tmpQN1[13] = + tmpQN2[10];
tmpQN1[14] = + tmpQN2[11];
tmpQN1[15] = 0.0;
;
}

void acado_evaluateObjective(  )
{
int runObj;
for (runObj = 0; runObj < 60; ++runObj)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[runObj * 4];
acadoWorkspace.objValueIn[1] = acadoVariables.x[runObj * 4 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[runObj * 4 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.x[runObj * 4 + 3];
acadoWorkspace.objValueIn[4] = acadoVariables.u[runObj * 2];
acadoWorkspace.objValueIn[5] = acadoVariables.u[runObj * 2 + 1];
acadoWorkspace.objValueIn[6] = acadoVariables.od[runObj * 13];
acadoWorkspace.objValueIn[7] = acadoVariables.od[runObj * 13 + 1];
acadoWorkspace.objValueIn[8] = acadoVariables.od[runObj * 13 + 2];
acadoWorkspace.objValueIn[9] = acadoVariables.od[runObj * 13 + 3];
acadoWorkspace.objValueIn[10] = acadoVariables.od[runObj * 13 + 4];
acadoWorkspace.objValueIn[11] = acadoVariables.od[runObj * 13 + 5];
acadoWorkspace.objValueIn[12] = acadoVariables.od[runObj * 13 + 6];
acadoWorkspace.objValueIn[13] = acadoVariables.od[runObj * 13 + 7];
acadoWorkspace.objValueIn[14] = acadoVariables.od[runObj * 13 + 8];
acadoWorkspace.objValueIn[15] = acadoVariables.od[runObj * 13 + 9];
acadoWorkspace.objValueIn[16] = acadoVariables.od[runObj * 13 + 10];
acadoWorkspace.objValueIn[17] = acadoVariables.od[runObj * 13 + 11];
acadoWorkspace.objValueIn[18] = acadoVariables.od[runObj * 13 + 12];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[runObj * 9] = acadoWorkspace.objValueOut[0];
acadoWorkspace.Dy[runObj * 9 + 1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.Dy[runObj * 9 + 2] = acadoWorkspace.objValueOut[2];
acadoWorkspace.Dy[runObj * 9 + 3] = acadoWorkspace.objValueOut[3];
acadoWorkspace.Dy[runObj * 9 + 4] = acadoWorkspace.objValueOut[4];
acadoWorkspace.Dy[runObj * 9 + 5] = acadoWorkspace.objValueOut[5];
acadoWorkspace.Dy[runObj * 9 + 6] = acadoWorkspace.objValueOut[6];
acadoWorkspace.Dy[runObj * 9 + 7] = acadoWorkspace.objValueOut[7];
acadoWorkspace.Dy[runObj * 9 + 8] = acadoWorkspace.objValueOut[8];

acado_setObjQ1Q2( &(acadoWorkspace.objValueOut[ 9 ]), acadoVariables.W, &(acadoWorkspace.Q1[ runObj * 16 ]), &(acadoWorkspace.Q2[ runObj * 36 ]) );

}
acadoWorkspace.objValueIn[0] = acadoVariables.x[240];
acadoWorkspace.objValueIn[1] = acadoVariables.x[241];
acadoWorkspace.objValueIn[2] = acadoVariables.x[242];
acadoWorkspace.objValueIn[3] = acadoVariables.x[243];
acadoWorkspace.objValueIn[4] = acadoVariables.od[780];
acadoWorkspace.objValueIn[5] = acadoVariables.od[781];
acadoWorkspace.objValueIn[6] = acadoVariables.od[782];
acadoWorkspace.objValueIn[7] = acadoVariables.od[783];
acadoWorkspace.objValueIn[8] = acadoVariables.od[784];
acadoWorkspace.objValueIn[9] = acadoVariables.od[785];
acadoWorkspace.objValueIn[10] = acadoVariables.od[786];
acadoWorkspace.objValueIn[11] = acadoVariables.od[787];
acadoWorkspace.objValueIn[12] = acadoVariables.od[788];
acadoWorkspace.objValueIn[13] = acadoVariables.od[789];
acadoWorkspace.objValueIn[14] = acadoVariables.od[790];
acadoWorkspace.objValueIn[15] = acadoVariables.od[791];
acadoWorkspace.objValueIn[16] = acadoVariables.od[792];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );

acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.DyN[2] = acadoWorkspace.objValueOut[2];

acado_setObjQN1QN2( acadoVariables.WN, acadoWorkspace.QN1, acadoWorkspace.QN2 );

}

void acado_multGxd( real_t* const dOld, real_t* const Gx1, real_t* const dNew )
{
dNew[0] += + Gx1[0]*dOld[0] + Gx1[1]*dOld[1] + Gx1[2]*dOld[2] + Gx1[3]*dOld[3];
dNew[1] += + Gx1[4]*dOld[0] + Gx1[5]*dOld[1] + Gx1[6]*dOld[2] + Gx1[7]*dOld[3];
dNew[2] += + Gx1[8]*dOld[0] + Gx1[9]*dOld[1] + Gx1[10]*dOld[2] + Gx1[11]*dOld[3];
dNew[3] += + Gx1[12]*dOld[0] + Gx1[13]*dOld[1] + Gx1[14]*dOld[2] + Gx1[15]*dOld[3];
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
}

void acado_multGxGx( real_t* const Gx1, real_t* const Gx2, real_t* const Gx3 )
{
Gx3[0] = + Gx1[0]*Gx2[0] + Gx1[1]*Gx2[4] + Gx1[2]*Gx2[8] + Gx1[3]*Gx2[12];
Gx3[1] = + Gx1[0]*Gx2[1] + Gx1[1]*Gx2[5] + Gx1[2]*Gx2[9] + Gx1[3]*Gx2[13];
Gx3[2] = + Gx1[0]*Gx2[2] + Gx1[1]*Gx2[6] + Gx1[2]*Gx2[10] + Gx1[3]*Gx2[14];
Gx3[3] = + Gx1[0]*Gx2[3] + Gx1[1]*Gx2[7] + Gx1[2]*Gx2[11] + Gx1[3]*Gx2[15];
Gx3[4] = + Gx1[4]*Gx2[0] + Gx1[5]*Gx2[4] + Gx1[6]*Gx2[8] + Gx1[7]*Gx2[12];
Gx3[5] = + Gx1[4]*Gx2[1] + Gx1[5]*Gx2[5] + Gx1[6]*Gx2[9] + Gx1[7]*Gx2[13];
Gx3[6] = + Gx1[4]*Gx2[2] + Gx1[5]*Gx2[6] + Gx1[6]*Gx2[10] + Gx1[7]*Gx2[14];
Gx3[7] = + Gx1[4]*Gx2[3] + Gx1[5]*Gx2[7] + Gx1[6]*Gx2[11] + Gx1[7]*Gx2[15];
Gx3[8] = + Gx1[8]*Gx2[0] + Gx1[9]*Gx2[4] + Gx1[10]*Gx2[8] + Gx1[11]*Gx2[12];
Gx3[9] = + Gx1[8]*Gx2[1] + Gx1[9]*Gx2[5] + Gx1[10]*Gx2[9] + Gx1[11]*Gx2[13];
Gx3[10] = + Gx1[8]*Gx2[2] + Gx1[9]*Gx2[6] + Gx1[10]*Gx2[10] + Gx1[11]*Gx2[14];
Gx3[11] = + Gx1[8]*Gx2[3] + Gx1[9]*Gx2[7] + Gx1[10]*Gx2[11] + Gx1[11]*Gx2[15];
Gx3[12] = + Gx1[12]*Gx2[0] + Gx1[13]*Gx2[4] + Gx1[14]*Gx2[8] + Gx1[15]*Gx2[12];
Gx3[13] = + Gx1[12]*Gx2[1] + Gx1[13]*Gx2[5] + Gx1[14]*Gx2[9] + Gx1[15]*Gx2[13];
Gx3[14] = + Gx1[12]*Gx2[2] + Gx1[13]*Gx2[6] + Gx1[14]*Gx2[10] + Gx1[15]*Gx2[14];
Gx3[15] = + Gx1[12]*Gx2[3] + Gx1[13]*Gx2[7] + Gx1[14]*Gx2[11] + Gx1[15]*Gx2[15];
}

void acado_multGxGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0] + Gx1[1]*Gu1[2] + Gx1[2]*Gu1[4] + Gx1[3]*Gu1[6];
Gu2[1] = + Gx1[0]*Gu1[1] + Gx1[1]*Gu1[3] + Gx1[2]*Gu1[5] + Gx1[3]*Gu1[7];
Gu2[2] = + Gx1[4]*Gu1[0] + Gx1[5]*Gu1[2] + Gx1[6]*Gu1[4] + Gx1[7]*Gu1[6];
Gu2[3] = + Gx1[4]*Gu1[1] + Gx1[5]*Gu1[3] + Gx1[6]*Gu1[5] + Gx1[7]*Gu1[7];
Gu2[4] = + Gx1[8]*Gu1[0] + Gx1[9]*Gu1[2] + Gx1[10]*Gu1[4] + Gx1[11]*Gu1[6];
Gu2[5] = + Gx1[8]*Gu1[1] + Gx1[9]*Gu1[3] + Gx1[10]*Gu1[5] + Gx1[11]*Gu1[7];
Gu2[6] = + Gx1[12]*Gu1[0] + Gx1[13]*Gu1[2] + Gx1[14]*Gu1[4] + Gx1[15]*Gu1[6];
Gu2[7] = + Gx1[12]*Gu1[1] + Gx1[13]*Gu1[3] + Gx1[14]*Gu1[5] + Gx1[15]*Gu1[7];
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
}

void acado_setBlockH11( int iRow, int iCol, real_t* const Gu1, real_t* const Gu2 )
{
acadoWorkspace.H[(iRow * 240) + (iCol * 2)] += + Gu1[0]*Gu2[0] + Gu1[2]*Gu2[2] + Gu1[4]*Gu2[4] + Gu1[6]*Gu2[6];
acadoWorkspace.H[(iRow * 240) + (iCol * 2 + 1)] += + Gu1[0]*Gu2[1] + Gu1[2]*Gu2[3] + Gu1[4]*Gu2[5] + Gu1[6]*Gu2[7];
acadoWorkspace.H[(iRow * 240 + 120) + (iCol * 2)] += + Gu1[1]*Gu2[0] + Gu1[3]*Gu2[2] + Gu1[5]*Gu2[4] + Gu1[7]*Gu2[6];
acadoWorkspace.H[(iRow * 240 + 120) + (iCol * 2 + 1)] += + Gu1[1]*Gu2[1] + Gu1[3]*Gu2[3] + Gu1[5]*Gu2[5] + Gu1[7]*Gu2[7];
}

void acado_setBlockH11_R1( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 240) + (iCol * 2)] = + (real_t)1.0000000000000000e-08;
acadoWorkspace.H[(iRow * 240) + (iCol * 2 + 1)] = 0.0;
acadoWorkspace.H[(iRow * 240 + 120) + (iCol * 2)] = 0.0;
acadoWorkspace.H[(iRow * 240 + 120) + (iCol * 2 + 1)] = + (real_t)1.0000000000000000e-08;
}

void acado_zeroBlockH11( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 240) + (iCol * 2)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 240) + (iCol * 2 + 1)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 240 + 120) + (iCol * 2)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 240 + 120) + (iCol * 2 + 1)] = 0.0000000000000000e+00;
}

void acado_copyHTH( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 240) + (iCol * 2)] = acadoWorkspace.H[(iCol * 240) + (iRow * 2)];
acadoWorkspace.H[(iRow * 240) + (iCol * 2 + 1)] = acadoWorkspace.H[(iCol * 240 + 120) + (iRow * 2)];
acadoWorkspace.H[(iRow * 240 + 120) + (iCol * 2)] = acadoWorkspace.H[(iCol * 240) + (iRow * 2 + 1)];
acadoWorkspace.H[(iRow * 240 + 120) + (iCol * 2 + 1)] = acadoWorkspace.H[(iCol * 240 + 120) + (iRow * 2 + 1)];
}

void acado_multQ1d( real_t* const Gx1, real_t* const dOld, real_t* const dNew )
{
dNew[0] = + Gx1[0]*dOld[0] + Gx1[1]*dOld[1] + Gx1[2]*dOld[2] + Gx1[3]*dOld[3];
dNew[1] = + Gx1[4]*dOld[0] + Gx1[5]*dOld[1] + Gx1[6]*dOld[2] + Gx1[7]*dOld[3];
dNew[2] = + Gx1[8]*dOld[0] + Gx1[9]*dOld[1] + Gx1[10]*dOld[2] + Gx1[11]*dOld[3];
dNew[3] = + Gx1[12]*dOld[0] + Gx1[13]*dOld[1] + Gx1[14]*dOld[2] + Gx1[15]*dOld[3];
}

void acado_multQN1d( real_t* const QN1, real_t* const dOld, real_t* const dNew )
{
dNew[0] = + acadoWorkspace.QN1[0]*dOld[0] + acadoWorkspace.QN1[1]*dOld[1] + acadoWorkspace.QN1[2]*dOld[2] + acadoWorkspace.QN1[3]*dOld[3];
dNew[1] = + acadoWorkspace.QN1[4]*dOld[0] + acadoWorkspace.QN1[5]*dOld[1] + acadoWorkspace.QN1[6]*dOld[2] + acadoWorkspace.QN1[7]*dOld[3];
dNew[2] = + acadoWorkspace.QN1[8]*dOld[0] + acadoWorkspace.QN1[9]*dOld[1] + acadoWorkspace.QN1[10]*dOld[2] + acadoWorkspace.QN1[11]*dOld[3];
dNew[3] = + acadoWorkspace.QN1[12]*dOld[0] + acadoWorkspace.QN1[13]*dOld[1] + acadoWorkspace.QN1[14]*dOld[2] + acadoWorkspace.QN1[15]*dOld[3];
}

void acado_multRDy( real_t* const Dy1, real_t* const RDy1 )
{
RDy1[0] = 0.0;
;
RDy1[1] = 0.0;
;
}

void acado_multQDy( real_t* const Q2, real_t* const Dy1, real_t* const QDy1 )
{
QDy1[0] = + Q2[0]*Dy1[0] + Q2[1]*Dy1[1] + Q2[2]*Dy1[2] + Q2[3]*Dy1[3] + Q2[4]*Dy1[4] + Q2[5]*Dy1[5] + Q2[6]*Dy1[6] + Q2[7]*Dy1[7] + Q2[8]*Dy1[8];
QDy1[1] = + Q2[9]*Dy1[0] + Q2[10]*Dy1[1] + Q2[11]*Dy1[2] + Q2[12]*Dy1[3] + Q2[13]*Dy1[4] + Q2[14]*Dy1[5] + Q2[15]*Dy1[6] + Q2[16]*Dy1[7] + Q2[17]*Dy1[8];
QDy1[2] = + Q2[18]*Dy1[0] + Q2[19]*Dy1[1] + Q2[20]*Dy1[2] + Q2[21]*Dy1[3] + Q2[22]*Dy1[4] + Q2[23]*Dy1[5] + Q2[24]*Dy1[6] + Q2[25]*Dy1[7] + Q2[26]*Dy1[8];
QDy1[3] = + Q2[27]*Dy1[0] + Q2[28]*Dy1[1] + Q2[29]*Dy1[2] + Q2[30]*Dy1[3] + Q2[31]*Dy1[4] + Q2[32]*Dy1[5] + Q2[33]*Dy1[6] + Q2[34]*Dy1[7] + Q2[35]*Dy1[8];
}

void acado_multEQDy( real_t* const E1, real_t* const QDy1, real_t* const U1 )
{
U1[0] += + E1[0]*QDy1[0] + E1[2]*QDy1[1] + E1[4]*QDy1[2] + E1[6]*QDy1[3];
U1[1] += + E1[1]*QDy1[0] + E1[3]*QDy1[1] + E1[5]*QDy1[2] + E1[7]*QDy1[3];
}

void acado_multQETGx( real_t* const E1, real_t* const Gx1, real_t* const H101 )
{
H101[0] += + E1[0]*Gx1[0] + E1[2]*Gx1[4] + E1[4]*Gx1[8] + E1[6]*Gx1[12];
H101[1] += + E1[0]*Gx1[1] + E1[2]*Gx1[5] + E1[4]*Gx1[9] + E1[6]*Gx1[13];
H101[2] += + E1[0]*Gx1[2] + E1[2]*Gx1[6] + E1[4]*Gx1[10] + E1[6]*Gx1[14];
H101[3] += + E1[0]*Gx1[3] + E1[2]*Gx1[7] + E1[4]*Gx1[11] + E1[6]*Gx1[15];
H101[4] += + E1[1]*Gx1[0] + E1[3]*Gx1[4] + E1[5]*Gx1[8] + E1[7]*Gx1[12];
H101[5] += + E1[1]*Gx1[1] + E1[3]*Gx1[5] + E1[5]*Gx1[9] + E1[7]*Gx1[13];
H101[6] += + E1[1]*Gx1[2] + E1[3]*Gx1[6] + E1[5]*Gx1[10] + E1[7]*Gx1[14];
H101[7] += + E1[1]*Gx1[3] + E1[3]*Gx1[7] + E1[5]*Gx1[11] + E1[7]*Gx1[15];
}

void acado_zeroBlockH10( real_t* const H101 )
{
{ int lCopy; for (lCopy = 0; lCopy < 8; lCopy++) H101[ lCopy ] = 0; }
}

void acado_multEDu( real_t* const E1, real_t* const U1, real_t* const dNew )
{
dNew[0] += + E1[0]*U1[0] + E1[1]*U1[1];
dNew[1] += + E1[2]*U1[0] + E1[3]*U1[1];
dNew[2] += + E1[4]*U1[0] + E1[5]*U1[1];
dNew[3] += + E1[6]*U1[0] + E1[7]*U1[1];
}

void acado_macETSlu( real_t* const E0, real_t* const g1 )
{
g1[0] += 0.0;
;
g1[1] += 0.0;
;
}

void acado_condensePrep(  )
{
int lRun1;
int lRun2;
int lRun3;
int lRun4;
int lRun5;
acado_moveGuE( acadoWorkspace.evGu, acadoWorkspace.E );
for (lRun1 = 1; lRun1 < 60; ++lRun1)
{
acado_moveGxT( &(acadoWorkspace.evGx[ lRun1 * 16 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ lRun1 * 4-4 ]), &(acadoWorkspace.evGx[ lRun1 * 16 ]), &(acadoWorkspace.d[ lRun1 * 4 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ lRun1 * 16-16 ]), &(acadoWorkspace.evGx[ lRun1 * 16 ]) );
for (lRun2 = 0; lRun2 < lRun1; ++lRun2)
{
lRun4 = (((lRun1) * (lRun1-1)) / (2)) + (lRun2);
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ lRun4 * 8 ]), &(acadoWorkspace.E[ lRun3 * 8 ]) );
}
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
acado_moveGuE( &(acadoWorkspace.evGu[ lRun1 * 8 ]), &(acadoWorkspace.E[ lRun3 * 8 ]) );
}

for (lRun1 = 0; lRun1 < 59; ++lRun1)
{
for (lRun2 = 0; lRun2 < lRun1 + 1; ++lRun2)
{
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
acado_multGxGu( &(acadoWorkspace.Q1[ lRun1 * 16 + 16 ]), &(acadoWorkspace.E[ lRun3 * 8 ]), &(acadoWorkspace.QE[ lRun3 * 8 ]) );
}
}

for (lRun2 = 0; lRun2 < lRun1 + 1; ++lRun2)
{
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ lRun3 * 8 ]), &(acadoWorkspace.QE[ lRun3 * 8 ]) );
}

for (lRun1 = 0; lRun1 < 60; ++lRun1)
{
acado_zeroBlockH10( &(acadoWorkspace.H10[ lRun1 * 8 ]) );
for (lRun2 = lRun1; lRun2 < 60; ++lRun2)
{
lRun3 = (((lRun2 + 1) * (lRun2)) / (2)) + (lRun1);
acado_multQETGx( &(acadoWorkspace.QE[ lRun3 * 8 ]), &(acadoWorkspace.evGx[ lRun2 * 16 ]), &(acadoWorkspace.H10[ lRun1 * 8 ]) );
}
}

for (lRun1 = 0; lRun1 < 60; ++lRun1)
{
acado_setBlockH11_R1( lRun1, lRun1 );
lRun2 = lRun1;
for (lRun3 = lRun1; lRun3 < 60; ++lRun3)
{
lRun4 = (((lRun3 + 1) * (lRun3)) / (2)) + (lRun1);
lRun5 = (((lRun3 + 1) * (lRun3)) / (2)) + (lRun2);
acado_setBlockH11( lRun1, lRun2, &(acadoWorkspace.E[ lRun4 * 8 ]), &(acadoWorkspace.QE[ lRun5 * 8 ]) );
}
for (lRun2 = lRun1 + 1; lRun2 < 60; ++lRun2)
{
acado_zeroBlockH11( lRun1, lRun2 );
for (lRun3 = lRun2; lRun3 < 60; ++lRun3)
{
lRun4 = (((lRun3 + 1) * (lRun3)) / (2)) + (lRun1);
lRun5 = (((lRun3 + 1) * (lRun3)) / (2)) + (lRun2);
acado_setBlockH11( lRun1, lRun2, &(acadoWorkspace.E[ lRun4 * 8 ]), &(acadoWorkspace.QE[ lRun5 * 8 ]) );
}
}
}

for (lRun1 = 0; lRun1 < 60; ++lRun1)
{
for (lRun2 = 0; lRun2 < lRun1; ++lRun2)
{
acado_copyHTH( lRun1, lRun2 );
}
}

acado_multQ1d( &(acadoWorkspace.Q1[ 16 ]), acadoWorkspace.d, acadoWorkspace.Qd );
acado_multQ1d( &(acadoWorkspace.Q1[ 32 ]), &(acadoWorkspace.d[ 4 ]), &(acadoWorkspace.Qd[ 4 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 48 ]), &(acadoWorkspace.d[ 8 ]), &(acadoWorkspace.Qd[ 8 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 64 ]), &(acadoWorkspace.d[ 12 ]), &(acadoWorkspace.Qd[ 12 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 80 ]), &(acadoWorkspace.d[ 16 ]), &(acadoWorkspace.Qd[ 16 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 96 ]), &(acadoWorkspace.d[ 20 ]), &(acadoWorkspace.Qd[ 20 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 112 ]), &(acadoWorkspace.d[ 24 ]), &(acadoWorkspace.Qd[ 24 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 128 ]), &(acadoWorkspace.d[ 28 ]), &(acadoWorkspace.Qd[ 28 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 144 ]), &(acadoWorkspace.d[ 32 ]), &(acadoWorkspace.Qd[ 32 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 160 ]), &(acadoWorkspace.d[ 36 ]), &(acadoWorkspace.Qd[ 36 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 176 ]), &(acadoWorkspace.d[ 40 ]), &(acadoWorkspace.Qd[ 40 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 192 ]), &(acadoWorkspace.d[ 44 ]), &(acadoWorkspace.Qd[ 44 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 208 ]), &(acadoWorkspace.d[ 48 ]), &(acadoWorkspace.Qd[ 48 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 224 ]), &(acadoWorkspace.d[ 52 ]), &(acadoWorkspace.Qd[ 52 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 240 ]), &(acadoWorkspace.d[ 56 ]), &(acadoWorkspace.Qd[ 56 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 256 ]), &(acadoWorkspace.d[ 60 ]), &(acadoWorkspace.Qd[ 60 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 272 ]), &(acadoWorkspace.d[ 64 ]), &(acadoWorkspace.Qd[ 64 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 288 ]), &(acadoWorkspace.d[ 68 ]), &(acadoWorkspace.Qd[ 68 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 304 ]), &(acadoWorkspace.d[ 72 ]), &(acadoWorkspace.Qd[ 72 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 320 ]), &(acadoWorkspace.d[ 76 ]), &(acadoWorkspace.Qd[ 76 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 336 ]), &(acadoWorkspace.d[ 80 ]), &(acadoWorkspace.Qd[ 80 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 352 ]), &(acadoWorkspace.d[ 84 ]), &(acadoWorkspace.Qd[ 84 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 368 ]), &(acadoWorkspace.d[ 88 ]), &(acadoWorkspace.Qd[ 88 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 384 ]), &(acadoWorkspace.d[ 92 ]), &(acadoWorkspace.Qd[ 92 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 400 ]), &(acadoWorkspace.d[ 96 ]), &(acadoWorkspace.Qd[ 96 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 416 ]), &(acadoWorkspace.d[ 100 ]), &(acadoWorkspace.Qd[ 100 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 432 ]), &(acadoWorkspace.d[ 104 ]), &(acadoWorkspace.Qd[ 104 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 448 ]), &(acadoWorkspace.d[ 108 ]), &(acadoWorkspace.Qd[ 108 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 464 ]), &(acadoWorkspace.d[ 112 ]), &(acadoWorkspace.Qd[ 112 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 480 ]), &(acadoWorkspace.d[ 116 ]), &(acadoWorkspace.Qd[ 116 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 496 ]), &(acadoWorkspace.d[ 120 ]), &(acadoWorkspace.Qd[ 120 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 512 ]), &(acadoWorkspace.d[ 124 ]), &(acadoWorkspace.Qd[ 124 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 528 ]), &(acadoWorkspace.d[ 128 ]), &(acadoWorkspace.Qd[ 128 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 544 ]), &(acadoWorkspace.d[ 132 ]), &(acadoWorkspace.Qd[ 132 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 560 ]), &(acadoWorkspace.d[ 136 ]), &(acadoWorkspace.Qd[ 136 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 576 ]), &(acadoWorkspace.d[ 140 ]), &(acadoWorkspace.Qd[ 140 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 592 ]), &(acadoWorkspace.d[ 144 ]), &(acadoWorkspace.Qd[ 144 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 608 ]), &(acadoWorkspace.d[ 148 ]), &(acadoWorkspace.Qd[ 148 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 624 ]), &(acadoWorkspace.d[ 152 ]), &(acadoWorkspace.Qd[ 152 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 640 ]), &(acadoWorkspace.d[ 156 ]), &(acadoWorkspace.Qd[ 156 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 656 ]), &(acadoWorkspace.d[ 160 ]), &(acadoWorkspace.Qd[ 160 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 672 ]), &(acadoWorkspace.d[ 164 ]), &(acadoWorkspace.Qd[ 164 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 688 ]), &(acadoWorkspace.d[ 168 ]), &(acadoWorkspace.Qd[ 168 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 704 ]), &(acadoWorkspace.d[ 172 ]), &(acadoWorkspace.Qd[ 172 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 720 ]), &(acadoWorkspace.d[ 176 ]), &(acadoWorkspace.Qd[ 176 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 736 ]), &(acadoWorkspace.d[ 180 ]), &(acadoWorkspace.Qd[ 180 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 752 ]), &(acadoWorkspace.d[ 184 ]), &(acadoWorkspace.Qd[ 184 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 768 ]), &(acadoWorkspace.d[ 188 ]), &(acadoWorkspace.Qd[ 188 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 784 ]), &(acadoWorkspace.d[ 192 ]), &(acadoWorkspace.Qd[ 192 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 800 ]), &(acadoWorkspace.d[ 196 ]), &(acadoWorkspace.Qd[ 196 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 816 ]), &(acadoWorkspace.d[ 200 ]), &(acadoWorkspace.Qd[ 200 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 832 ]), &(acadoWorkspace.d[ 204 ]), &(acadoWorkspace.Qd[ 204 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 848 ]), &(acadoWorkspace.d[ 208 ]), &(acadoWorkspace.Qd[ 208 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 864 ]), &(acadoWorkspace.d[ 212 ]), &(acadoWorkspace.Qd[ 212 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 880 ]), &(acadoWorkspace.d[ 216 ]), &(acadoWorkspace.Qd[ 216 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 896 ]), &(acadoWorkspace.d[ 220 ]), &(acadoWorkspace.Qd[ 220 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 912 ]), &(acadoWorkspace.d[ 224 ]), &(acadoWorkspace.Qd[ 224 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 928 ]), &(acadoWorkspace.d[ 228 ]), &(acadoWorkspace.Qd[ 228 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 944 ]), &(acadoWorkspace.d[ 232 ]), &(acadoWorkspace.Qd[ 232 ]) );
acado_multQN1d( acadoWorkspace.QN1, &(acadoWorkspace.d[ 236 ]), &(acadoWorkspace.Qd[ 236 ]) );

for (lRun1 = 0; lRun1 < 60; ++lRun1)
{
for (lRun2 = lRun1; lRun2 < 60; ++lRun2)
{
lRun3 = (((lRun2 + 1) * (lRun2)) / (2)) + (lRun1);
acado_macETSlu( &(acadoWorkspace.QE[ lRun3 * 8 ]), &(acadoWorkspace.g[ lRun1 * 2 ]) );
}
}
}

void acado_condenseFdb(  )
{
int lRun1;
int lRun2;
int lRun3;
acadoWorkspace.Dx0[0] = acadoVariables.x0[0] - acadoVariables.x[0];
acadoWorkspace.Dx0[1] = acadoVariables.x0[1] - acadoVariables.x[1];
acadoWorkspace.Dx0[2] = acadoVariables.x0[2] - acadoVariables.x[2];
acadoWorkspace.Dx0[3] = acadoVariables.x0[3] - acadoVariables.x[3];

for (lRun2 = 0; lRun2 < 540; ++lRun2)
acadoWorkspace.Dy[lRun2] -= acadoVariables.y[lRun2];

acadoWorkspace.DyN[0] -= acadoVariables.yN[0];
acadoWorkspace.DyN[1] -= acadoVariables.yN[1];
acadoWorkspace.DyN[2] -= acadoVariables.yN[2];

acado_multRDy( acadoWorkspace.Dy, acadoWorkspace.g );
acado_multRDy( &(acadoWorkspace.Dy[ 9 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 18 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 27 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 36 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 45 ]), &(acadoWorkspace.g[ 10 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 54 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 63 ]), &(acadoWorkspace.g[ 14 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 72 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 81 ]), &(acadoWorkspace.g[ 18 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 90 ]), &(acadoWorkspace.g[ 20 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 99 ]), &(acadoWorkspace.g[ 22 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 108 ]), &(acadoWorkspace.g[ 24 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 117 ]), &(acadoWorkspace.g[ 26 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 126 ]), &(acadoWorkspace.g[ 28 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 135 ]), &(acadoWorkspace.g[ 30 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 144 ]), &(acadoWorkspace.g[ 32 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 153 ]), &(acadoWorkspace.g[ 34 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 162 ]), &(acadoWorkspace.g[ 36 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 171 ]), &(acadoWorkspace.g[ 38 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 180 ]), &(acadoWorkspace.g[ 40 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 189 ]), &(acadoWorkspace.g[ 42 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 198 ]), &(acadoWorkspace.g[ 44 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 207 ]), &(acadoWorkspace.g[ 46 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 216 ]), &(acadoWorkspace.g[ 48 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 225 ]), &(acadoWorkspace.g[ 50 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 234 ]), &(acadoWorkspace.g[ 52 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 243 ]), &(acadoWorkspace.g[ 54 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 252 ]), &(acadoWorkspace.g[ 56 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 261 ]), &(acadoWorkspace.g[ 58 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 270 ]), &(acadoWorkspace.g[ 60 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 279 ]), &(acadoWorkspace.g[ 62 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 288 ]), &(acadoWorkspace.g[ 64 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 297 ]), &(acadoWorkspace.g[ 66 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 306 ]), &(acadoWorkspace.g[ 68 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 315 ]), &(acadoWorkspace.g[ 70 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 324 ]), &(acadoWorkspace.g[ 72 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 333 ]), &(acadoWorkspace.g[ 74 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 342 ]), &(acadoWorkspace.g[ 76 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 351 ]), &(acadoWorkspace.g[ 78 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 360 ]), &(acadoWorkspace.g[ 80 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 369 ]), &(acadoWorkspace.g[ 82 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 378 ]), &(acadoWorkspace.g[ 84 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 387 ]), &(acadoWorkspace.g[ 86 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 396 ]), &(acadoWorkspace.g[ 88 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 405 ]), &(acadoWorkspace.g[ 90 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 414 ]), &(acadoWorkspace.g[ 92 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 423 ]), &(acadoWorkspace.g[ 94 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 432 ]), &(acadoWorkspace.g[ 96 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 441 ]), &(acadoWorkspace.g[ 98 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 450 ]), &(acadoWorkspace.g[ 100 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 459 ]), &(acadoWorkspace.g[ 102 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 468 ]), &(acadoWorkspace.g[ 104 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 477 ]), &(acadoWorkspace.g[ 106 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 486 ]), &(acadoWorkspace.g[ 108 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 495 ]), &(acadoWorkspace.g[ 110 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 504 ]), &(acadoWorkspace.g[ 112 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 513 ]), &(acadoWorkspace.g[ 114 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 522 ]), &(acadoWorkspace.g[ 116 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 531 ]), &(acadoWorkspace.g[ 118 ]) );

acado_multQDy( acadoWorkspace.Q2, acadoWorkspace.Dy, acadoWorkspace.QDy );
acado_multQDy( &(acadoWorkspace.Q2[ 36 ]), &(acadoWorkspace.Dy[ 9 ]), &(acadoWorkspace.QDy[ 4 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 72 ]), &(acadoWorkspace.Dy[ 18 ]), &(acadoWorkspace.QDy[ 8 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 108 ]), &(acadoWorkspace.Dy[ 27 ]), &(acadoWorkspace.QDy[ 12 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 144 ]), &(acadoWorkspace.Dy[ 36 ]), &(acadoWorkspace.QDy[ 16 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 180 ]), &(acadoWorkspace.Dy[ 45 ]), &(acadoWorkspace.QDy[ 20 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 216 ]), &(acadoWorkspace.Dy[ 54 ]), &(acadoWorkspace.QDy[ 24 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 252 ]), &(acadoWorkspace.Dy[ 63 ]), &(acadoWorkspace.QDy[ 28 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 288 ]), &(acadoWorkspace.Dy[ 72 ]), &(acadoWorkspace.QDy[ 32 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 324 ]), &(acadoWorkspace.Dy[ 81 ]), &(acadoWorkspace.QDy[ 36 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 360 ]), &(acadoWorkspace.Dy[ 90 ]), &(acadoWorkspace.QDy[ 40 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 396 ]), &(acadoWorkspace.Dy[ 99 ]), &(acadoWorkspace.QDy[ 44 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 432 ]), &(acadoWorkspace.Dy[ 108 ]), &(acadoWorkspace.QDy[ 48 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 468 ]), &(acadoWorkspace.Dy[ 117 ]), &(acadoWorkspace.QDy[ 52 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 504 ]), &(acadoWorkspace.Dy[ 126 ]), &(acadoWorkspace.QDy[ 56 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 540 ]), &(acadoWorkspace.Dy[ 135 ]), &(acadoWorkspace.QDy[ 60 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 576 ]), &(acadoWorkspace.Dy[ 144 ]), &(acadoWorkspace.QDy[ 64 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 612 ]), &(acadoWorkspace.Dy[ 153 ]), &(acadoWorkspace.QDy[ 68 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 648 ]), &(acadoWorkspace.Dy[ 162 ]), &(acadoWorkspace.QDy[ 72 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 684 ]), &(acadoWorkspace.Dy[ 171 ]), &(acadoWorkspace.QDy[ 76 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 720 ]), &(acadoWorkspace.Dy[ 180 ]), &(acadoWorkspace.QDy[ 80 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 756 ]), &(acadoWorkspace.Dy[ 189 ]), &(acadoWorkspace.QDy[ 84 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 792 ]), &(acadoWorkspace.Dy[ 198 ]), &(acadoWorkspace.QDy[ 88 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 828 ]), &(acadoWorkspace.Dy[ 207 ]), &(acadoWorkspace.QDy[ 92 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 864 ]), &(acadoWorkspace.Dy[ 216 ]), &(acadoWorkspace.QDy[ 96 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 900 ]), &(acadoWorkspace.Dy[ 225 ]), &(acadoWorkspace.QDy[ 100 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 936 ]), &(acadoWorkspace.Dy[ 234 ]), &(acadoWorkspace.QDy[ 104 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 972 ]), &(acadoWorkspace.Dy[ 243 ]), &(acadoWorkspace.QDy[ 108 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1008 ]), &(acadoWorkspace.Dy[ 252 ]), &(acadoWorkspace.QDy[ 112 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1044 ]), &(acadoWorkspace.Dy[ 261 ]), &(acadoWorkspace.QDy[ 116 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1080 ]), &(acadoWorkspace.Dy[ 270 ]), &(acadoWorkspace.QDy[ 120 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1116 ]), &(acadoWorkspace.Dy[ 279 ]), &(acadoWorkspace.QDy[ 124 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1152 ]), &(acadoWorkspace.Dy[ 288 ]), &(acadoWorkspace.QDy[ 128 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1188 ]), &(acadoWorkspace.Dy[ 297 ]), &(acadoWorkspace.QDy[ 132 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1224 ]), &(acadoWorkspace.Dy[ 306 ]), &(acadoWorkspace.QDy[ 136 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1260 ]), &(acadoWorkspace.Dy[ 315 ]), &(acadoWorkspace.QDy[ 140 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1296 ]), &(acadoWorkspace.Dy[ 324 ]), &(acadoWorkspace.QDy[ 144 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1332 ]), &(acadoWorkspace.Dy[ 333 ]), &(acadoWorkspace.QDy[ 148 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1368 ]), &(acadoWorkspace.Dy[ 342 ]), &(acadoWorkspace.QDy[ 152 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1404 ]), &(acadoWorkspace.Dy[ 351 ]), &(acadoWorkspace.QDy[ 156 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1440 ]), &(acadoWorkspace.Dy[ 360 ]), &(acadoWorkspace.QDy[ 160 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1476 ]), &(acadoWorkspace.Dy[ 369 ]), &(acadoWorkspace.QDy[ 164 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1512 ]), &(acadoWorkspace.Dy[ 378 ]), &(acadoWorkspace.QDy[ 168 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1548 ]), &(acadoWorkspace.Dy[ 387 ]), &(acadoWorkspace.QDy[ 172 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1584 ]), &(acadoWorkspace.Dy[ 396 ]), &(acadoWorkspace.QDy[ 176 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1620 ]), &(acadoWorkspace.Dy[ 405 ]), &(acadoWorkspace.QDy[ 180 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1656 ]), &(acadoWorkspace.Dy[ 414 ]), &(acadoWorkspace.QDy[ 184 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1692 ]), &(acadoWorkspace.Dy[ 423 ]), &(acadoWorkspace.QDy[ 188 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1728 ]), &(acadoWorkspace.Dy[ 432 ]), &(acadoWorkspace.QDy[ 192 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1764 ]), &(acadoWorkspace.Dy[ 441 ]), &(acadoWorkspace.QDy[ 196 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1800 ]), &(acadoWorkspace.Dy[ 450 ]), &(acadoWorkspace.QDy[ 200 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1836 ]), &(acadoWorkspace.Dy[ 459 ]), &(acadoWorkspace.QDy[ 204 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1872 ]), &(acadoWorkspace.Dy[ 468 ]), &(acadoWorkspace.QDy[ 208 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1908 ]), &(acadoWorkspace.Dy[ 477 ]), &(acadoWorkspace.QDy[ 212 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1944 ]), &(acadoWorkspace.Dy[ 486 ]), &(acadoWorkspace.QDy[ 216 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1980 ]), &(acadoWorkspace.Dy[ 495 ]), &(acadoWorkspace.QDy[ 220 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 2016 ]), &(acadoWorkspace.Dy[ 504 ]), &(acadoWorkspace.QDy[ 224 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 2052 ]), &(acadoWorkspace.Dy[ 513 ]), &(acadoWorkspace.QDy[ 228 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 2088 ]), &(acadoWorkspace.Dy[ 522 ]), &(acadoWorkspace.QDy[ 232 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 2124 ]), &(acadoWorkspace.Dy[ 531 ]), &(acadoWorkspace.QDy[ 236 ]) );

acadoWorkspace.QDy[240] = + acadoWorkspace.QN2[0]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[1]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[2]*acadoWorkspace.DyN[2];
acadoWorkspace.QDy[241] = + acadoWorkspace.QN2[3]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[4]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[5]*acadoWorkspace.DyN[2];
acadoWorkspace.QDy[242] = + acadoWorkspace.QN2[6]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[7]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[8]*acadoWorkspace.DyN[2];
acadoWorkspace.QDy[243] = + acadoWorkspace.QN2[9]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[10]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[11]*acadoWorkspace.DyN[2];

for (lRun2 = 0; lRun2 < 240; ++lRun2)
acadoWorkspace.QDy[lRun2 + 4] += acadoWorkspace.Qd[lRun2];


for (lRun1 = 0; lRun1 < 60; ++lRun1)
{
for (lRun2 = lRun1; lRun2 < 60; ++lRun2)
{
lRun3 = (((lRun2 + 1) * (lRun2)) / (2)) + (lRun1);
acado_multEQDy( &(acadoWorkspace.E[ lRun3 * 8 ]), &(acadoWorkspace.QDy[ lRun2 * 4 + 4 ]), &(acadoWorkspace.g[ lRun1 * 2 ]) );
}
}

acadoWorkspace.g[0] += + acadoWorkspace.H10[0]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[2]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[3]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[1] += + acadoWorkspace.H10[4]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[5]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[6]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[7]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[2] += + acadoWorkspace.H10[8]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[9]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[10]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[11]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[3] += + acadoWorkspace.H10[12]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[13]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[14]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[15]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[4] += + acadoWorkspace.H10[16]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[17]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[18]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[19]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[5] += + acadoWorkspace.H10[20]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[21]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[22]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[23]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[6] += + acadoWorkspace.H10[24]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[25]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[26]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[27]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[7] += + acadoWorkspace.H10[28]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[29]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[30]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[31]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[8] += + acadoWorkspace.H10[32]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[33]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[34]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[35]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[9] += + acadoWorkspace.H10[36]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[37]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[38]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[39]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[10] += + acadoWorkspace.H10[40]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[41]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[42]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[43]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[11] += + acadoWorkspace.H10[44]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[45]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[46]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[47]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[12] += + acadoWorkspace.H10[48]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[49]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[50]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[51]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[13] += + acadoWorkspace.H10[52]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[53]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[54]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[55]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[14] += + acadoWorkspace.H10[56]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[57]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[58]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[59]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[15] += + acadoWorkspace.H10[60]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[61]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[62]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[63]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[16] += + acadoWorkspace.H10[64]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[65]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[66]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[67]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[17] += + acadoWorkspace.H10[68]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[69]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[70]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[71]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[18] += + acadoWorkspace.H10[72]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[73]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[74]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[75]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[19] += + acadoWorkspace.H10[76]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[77]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[78]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[79]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[20] += + acadoWorkspace.H10[80]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[81]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[82]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[83]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[21] += + acadoWorkspace.H10[84]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[85]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[86]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[87]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[22] += + acadoWorkspace.H10[88]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[89]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[90]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[91]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[23] += + acadoWorkspace.H10[92]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[93]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[94]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[95]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[24] += + acadoWorkspace.H10[96]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[97]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[98]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[99]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[25] += + acadoWorkspace.H10[100]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[101]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[102]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[103]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[26] += + acadoWorkspace.H10[104]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[105]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[106]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[107]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[27] += + acadoWorkspace.H10[108]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[109]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[110]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[111]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[28] += + acadoWorkspace.H10[112]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[113]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[114]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[115]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[29] += + acadoWorkspace.H10[116]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[117]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[118]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[119]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[30] += + acadoWorkspace.H10[120]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[121]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[122]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[123]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[31] += + acadoWorkspace.H10[124]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[125]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[126]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[127]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[32] += + acadoWorkspace.H10[128]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[129]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[130]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[131]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[33] += + acadoWorkspace.H10[132]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[133]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[134]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[135]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[34] += + acadoWorkspace.H10[136]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[137]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[138]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[139]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[35] += + acadoWorkspace.H10[140]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[141]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[142]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[143]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[36] += + acadoWorkspace.H10[144]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[145]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[146]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[147]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[37] += + acadoWorkspace.H10[148]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[149]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[150]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[151]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[38] += + acadoWorkspace.H10[152]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[153]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[154]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[155]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[39] += + acadoWorkspace.H10[156]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[157]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[158]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[159]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[40] += + acadoWorkspace.H10[160]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[161]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[162]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[163]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[41] += + acadoWorkspace.H10[164]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[165]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[166]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[167]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[42] += + acadoWorkspace.H10[168]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[169]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[170]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[171]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[43] += + acadoWorkspace.H10[172]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[173]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[174]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[175]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[44] += + acadoWorkspace.H10[176]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[177]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[178]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[179]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[45] += + acadoWorkspace.H10[180]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[181]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[182]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[183]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[46] += + acadoWorkspace.H10[184]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[185]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[186]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[187]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[47] += + acadoWorkspace.H10[188]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[189]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[190]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[191]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[48] += + acadoWorkspace.H10[192]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[193]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[194]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[195]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[49] += + acadoWorkspace.H10[196]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[197]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[198]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[199]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[50] += + acadoWorkspace.H10[200]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[201]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[202]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[203]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[51] += + acadoWorkspace.H10[204]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[205]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[206]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[207]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[52] += + acadoWorkspace.H10[208]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[209]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[210]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[211]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[53] += + acadoWorkspace.H10[212]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[213]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[214]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[215]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[54] += + acadoWorkspace.H10[216]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[217]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[218]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[219]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[55] += + acadoWorkspace.H10[220]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[221]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[222]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[223]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[56] += + acadoWorkspace.H10[224]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[225]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[226]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[227]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[57] += + acadoWorkspace.H10[228]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[229]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[230]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[231]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[58] += + acadoWorkspace.H10[232]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[233]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[234]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[235]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[59] += + acadoWorkspace.H10[236]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[237]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[238]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[239]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[60] += + acadoWorkspace.H10[240]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[241]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[242]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[243]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[61] += + acadoWorkspace.H10[244]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[245]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[246]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[247]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[62] += + acadoWorkspace.H10[248]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[249]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[250]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[251]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[63] += + acadoWorkspace.H10[252]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[253]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[254]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[255]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[64] += + acadoWorkspace.H10[256]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[257]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[258]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[259]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[65] += + acadoWorkspace.H10[260]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[261]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[262]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[263]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[66] += + acadoWorkspace.H10[264]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[265]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[266]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[267]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[67] += + acadoWorkspace.H10[268]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[269]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[270]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[271]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[68] += + acadoWorkspace.H10[272]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[273]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[274]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[275]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[69] += + acadoWorkspace.H10[276]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[277]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[278]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[279]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[70] += + acadoWorkspace.H10[280]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[281]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[282]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[283]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[71] += + acadoWorkspace.H10[284]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[285]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[286]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[287]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[72] += + acadoWorkspace.H10[288]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[289]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[290]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[291]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[73] += + acadoWorkspace.H10[292]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[293]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[294]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[295]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[74] += + acadoWorkspace.H10[296]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[297]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[298]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[299]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[75] += + acadoWorkspace.H10[300]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[301]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[302]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[303]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[76] += + acadoWorkspace.H10[304]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[305]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[306]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[307]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[77] += + acadoWorkspace.H10[308]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[309]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[310]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[311]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[78] += + acadoWorkspace.H10[312]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[313]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[314]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[315]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[79] += + acadoWorkspace.H10[316]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[317]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[318]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[319]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[80] += + acadoWorkspace.H10[320]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[321]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[322]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[323]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[81] += + acadoWorkspace.H10[324]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[325]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[326]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[327]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[82] += + acadoWorkspace.H10[328]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[329]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[330]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[331]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[83] += + acadoWorkspace.H10[332]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[333]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[334]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[335]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[84] += + acadoWorkspace.H10[336]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[337]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[338]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[339]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[85] += + acadoWorkspace.H10[340]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[341]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[342]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[343]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[86] += + acadoWorkspace.H10[344]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[345]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[346]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[347]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[87] += + acadoWorkspace.H10[348]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[349]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[350]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[351]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[88] += + acadoWorkspace.H10[352]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[353]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[354]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[355]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[89] += + acadoWorkspace.H10[356]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[357]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[358]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[359]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[90] += + acadoWorkspace.H10[360]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[361]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[362]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[363]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[91] += + acadoWorkspace.H10[364]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[365]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[366]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[367]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[92] += + acadoWorkspace.H10[368]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[369]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[370]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[371]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[93] += + acadoWorkspace.H10[372]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[373]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[374]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[375]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[94] += + acadoWorkspace.H10[376]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[377]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[378]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[379]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[95] += + acadoWorkspace.H10[380]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[381]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[382]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[383]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[96] += + acadoWorkspace.H10[384]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[385]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[386]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[387]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[97] += + acadoWorkspace.H10[388]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[389]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[390]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[391]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[98] += + acadoWorkspace.H10[392]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[393]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[394]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[395]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[99] += + acadoWorkspace.H10[396]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[397]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[398]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[399]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[100] += + acadoWorkspace.H10[400]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[401]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[402]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[403]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[101] += + acadoWorkspace.H10[404]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[405]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[406]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[407]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[102] += + acadoWorkspace.H10[408]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[409]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[410]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[411]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[103] += + acadoWorkspace.H10[412]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[413]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[414]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[415]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[104] += + acadoWorkspace.H10[416]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[417]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[418]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[419]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[105] += + acadoWorkspace.H10[420]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[421]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[422]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[423]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[106] += + acadoWorkspace.H10[424]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[425]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[426]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[427]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[107] += + acadoWorkspace.H10[428]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[429]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[430]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[431]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[108] += + acadoWorkspace.H10[432]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[433]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[434]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[435]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[109] += + acadoWorkspace.H10[436]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[437]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[438]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[439]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[110] += + acadoWorkspace.H10[440]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[441]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[442]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[443]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[111] += + acadoWorkspace.H10[444]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[445]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[446]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[447]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[112] += + acadoWorkspace.H10[448]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[449]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[450]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[451]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[113] += + acadoWorkspace.H10[452]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[453]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[454]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[455]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[114] += + acadoWorkspace.H10[456]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[457]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[458]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[459]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[115] += + acadoWorkspace.H10[460]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[461]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[462]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[463]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[116] += + acadoWorkspace.H10[464]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[465]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[466]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[467]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[117] += + acadoWorkspace.H10[468]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[469]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[470]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[471]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[118] += + acadoWorkspace.H10[472]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[473]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[474]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[475]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[119] += + acadoWorkspace.H10[476]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[477]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[478]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[479]*acadoWorkspace.Dx0[3];

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
acadoWorkspace.lb[60] = acadoVariables.lbValues[60] - acadoVariables.u[60];
acadoWorkspace.lb[61] = acadoVariables.lbValues[61] - acadoVariables.u[61];
acadoWorkspace.lb[62] = acadoVariables.lbValues[62] - acadoVariables.u[62];
acadoWorkspace.lb[63] = acadoVariables.lbValues[63] - acadoVariables.u[63];
acadoWorkspace.lb[64] = acadoVariables.lbValues[64] - acadoVariables.u[64];
acadoWorkspace.lb[65] = acadoVariables.lbValues[65] - acadoVariables.u[65];
acadoWorkspace.lb[66] = acadoVariables.lbValues[66] - acadoVariables.u[66];
acadoWorkspace.lb[67] = acadoVariables.lbValues[67] - acadoVariables.u[67];
acadoWorkspace.lb[68] = acadoVariables.lbValues[68] - acadoVariables.u[68];
acadoWorkspace.lb[69] = acadoVariables.lbValues[69] - acadoVariables.u[69];
acadoWorkspace.lb[70] = acadoVariables.lbValues[70] - acadoVariables.u[70];
acadoWorkspace.lb[71] = acadoVariables.lbValues[71] - acadoVariables.u[71];
acadoWorkspace.lb[72] = acadoVariables.lbValues[72] - acadoVariables.u[72];
acadoWorkspace.lb[73] = acadoVariables.lbValues[73] - acadoVariables.u[73];
acadoWorkspace.lb[74] = acadoVariables.lbValues[74] - acadoVariables.u[74];
acadoWorkspace.lb[75] = acadoVariables.lbValues[75] - acadoVariables.u[75];
acadoWorkspace.lb[76] = acadoVariables.lbValues[76] - acadoVariables.u[76];
acadoWorkspace.lb[77] = acadoVariables.lbValues[77] - acadoVariables.u[77];
acadoWorkspace.lb[78] = acadoVariables.lbValues[78] - acadoVariables.u[78];
acadoWorkspace.lb[79] = acadoVariables.lbValues[79] - acadoVariables.u[79];
acadoWorkspace.lb[80] = acadoVariables.lbValues[80] - acadoVariables.u[80];
acadoWorkspace.lb[81] = acadoVariables.lbValues[81] - acadoVariables.u[81];
acadoWorkspace.lb[82] = acadoVariables.lbValues[82] - acadoVariables.u[82];
acadoWorkspace.lb[83] = acadoVariables.lbValues[83] - acadoVariables.u[83];
acadoWorkspace.lb[84] = acadoVariables.lbValues[84] - acadoVariables.u[84];
acadoWorkspace.lb[85] = acadoVariables.lbValues[85] - acadoVariables.u[85];
acadoWorkspace.lb[86] = acadoVariables.lbValues[86] - acadoVariables.u[86];
acadoWorkspace.lb[87] = acadoVariables.lbValues[87] - acadoVariables.u[87];
acadoWorkspace.lb[88] = acadoVariables.lbValues[88] - acadoVariables.u[88];
acadoWorkspace.lb[89] = acadoVariables.lbValues[89] - acadoVariables.u[89];
acadoWorkspace.lb[90] = acadoVariables.lbValues[90] - acadoVariables.u[90];
acadoWorkspace.lb[91] = acadoVariables.lbValues[91] - acadoVariables.u[91];
acadoWorkspace.lb[92] = acadoVariables.lbValues[92] - acadoVariables.u[92];
acadoWorkspace.lb[93] = acadoVariables.lbValues[93] - acadoVariables.u[93];
acadoWorkspace.lb[94] = acadoVariables.lbValues[94] - acadoVariables.u[94];
acadoWorkspace.lb[95] = acadoVariables.lbValues[95] - acadoVariables.u[95];
acadoWorkspace.lb[96] = acadoVariables.lbValues[96] - acadoVariables.u[96];
acadoWorkspace.lb[97] = acadoVariables.lbValues[97] - acadoVariables.u[97];
acadoWorkspace.lb[98] = acadoVariables.lbValues[98] - acadoVariables.u[98];
acadoWorkspace.lb[99] = acadoVariables.lbValues[99] - acadoVariables.u[99];
acadoWorkspace.lb[100] = acadoVariables.lbValues[100] - acadoVariables.u[100];
acadoWorkspace.lb[101] = acadoVariables.lbValues[101] - acadoVariables.u[101];
acadoWorkspace.lb[102] = acadoVariables.lbValues[102] - acadoVariables.u[102];
acadoWorkspace.lb[103] = acadoVariables.lbValues[103] - acadoVariables.u[103];
acadoWorkspace.lb[104] = acadoVariables.lbValues[104] - acadoVariables.u[104];
acadoWorkspace.lb[105] = acadoVariables.lbValues[105] - acadoVariables.u[105];
acadoWorkspace.lb[106] = acadoVariables.lbValues[106] - acadoVariables.u[106];
acadoWorkspace.lb[107] = acadoVariables.lbValues[107] - acadoVariables.u[107];
acadoWorkspace.lb[108] = acadoVariables.lbValues[108] - acadoVariables.u[108];
acadoWorkspace.lb[109] = acadoVariables.lbValues[109] - acadoVariables.u[109];
acadoWorkspace.lb[110] = acadoVariables.lbValues[110] - acadoVariables.u[110];
acadoWorkspace.lb[111] = acadoVariables.lbValues[111] - acadoVariables.u[111];
acadoWorkspace.lb[112] = acadoVariables.lbValues[112] - acadoVariables.u[112];
acadoWorkspace.lb[113] = acadoVariables.lbValues[113] - acadoVariables.u[113];
acadoWorkspace.lb[114] = acadoVariables.lbValues[114] - acadoVariables.u[114];
acadoWorkspace.lb[115] = acadoVariables.lbValues[115] - acadoVariables.u[115];
acadoWorkspace.lb[116] = acadoVariables.lbValues[116] - acadoVariables.u[116];
acadoWorkspace.lb[117] = acadoVariables.lbValues[117] - acadoVariables.u[117];
acadoWorkspace.lb[118] = acadoVariables.lbValues[118] - acadoVariables.u[118];
acadoWorkspace.lb[119] = acadoVariables.lbValues[119] - acadoVariables.u[119];
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
acadoWorkspace.ub[60] = acadoVariables.ubValues[60] - acadoVariables.u[60];
acadoWorkspace.ub[61] = acadoVariables.ubValues[61] - acadoVariables.u[61];
acadoWorkspace.ub[62] = acadoVariables.ubValues[62] - acadoVariables.u[62];
acadoWorkspace.ub[63] = acadoVariables.ubValues[63] - acadoVariables.u[63];
acadoWorkspace.ub[64] = acadoVariables.ubValues[64] - acadoVariables.u[64];
acadoWorkspace.ub[65] = acadoVariables.ubValues[65] - acadoVariables.u[65];
acadoWorkspace.ub[66] = acadoVariables.ubValues[66] - acadoVariables.u[66];
acadoWorkspace.ub[67] = acadoVariables.ubValues[67] - acadoVariables.u[67];
acadoWorkspace.ub[68] = acadoVariables.ubValues[68] - acadoVariables.u[68];
acadoWorkspace.ub[69] = acadoVariables.ubValues[69] - acadoVariables.u[69];
acadoWorkspace.ub[70] = acadoVariables.ubValues[70] - acadoVariables.u[70];
acadoWorkspace.ub[71] = acadoVariables.ubValues[71] - acadoVariables.u[71];
acadoWorkspace.ub[72] = acadoVariables.ubValues[72] - acadoVariables.u[72];
acadoWorkspace.ub[73] = acadoVariables.ubValues[73] - acadoVariables.u[73];
acadoWorkspace.ub[74] = acadoVariables.ubValues[74] - acadoVariables.u[74];
acadoWorkspace.ub[75] = acadoVariables.ubValues[75] - acadoVariables.u[75];
acadoWorkspace.ub[76] = acadoVariables.ubValues[76] - acadoVariables.u[76];
acadoWorkspace.ub[77] = acadoVariables.ubValues[77] - acadoVariables.u[77];
acadoWorkspace.ub[78] = acadoVariables.ubValues[78] - acadoVariables.u[78];
acadoWorkspace.ub[79] = acadoVariables.ubValues[79] - acadoVariables.u[79];
acadoWorkspace.ub[80] = acadoVariables.ubValues[80] - acadoVariables.u[80];
acadoWorkspace.ub[81] = acadoVariables.ubValues[81] - acadoVariables.u[81];
acadoWorkspace.ub[82] = acadoVariables.ubValues[82] - acadoVariables.u[82];
acadoWorkspace.ub[83] = acadoVariables.ubValues[83] - acadoVariables.u[83];
acadoWorkspace.ub[84] = acadoVariables.ubValues[84] - acadoVariables.u[84];
acadoWorkspace.ub[85] = acadoVariables.ubValues[85] - acadoVariables.u[85];
acadoWorkspace.ub[86] = acadoVariables.ubValues[86] - acadoVariables.u[86];
acadoWorkspace.ub[87] = acadoVariables.ubValues[87] - acadoVariables.u[87];
acadoWorkspace.ub[88] = acadoVariables.ubValues[88] - acadoVariables.u[88];
acadoWorkspace.ub[89] = acadoVariables.ubValues[89] - acadoVariables.u[89];
acadoWorkspace.ub[90] = acadoVariables.ubValues[90] - acadoVariables.u[90];
acadoWorkspace.ub[91] = acadoVariables.ubValues[91] - acadoVariables.u[91];
acadoWorkspace.ub[92] = acadoVariables.ubValues[92] - acadoVariables.u[92];
acadoWorkspace.ub[93] = acadoVariables.ubValues[93] - acadoVariables.u[93];
acadoWorkspace.ub[94] = acadoVariables.ubValues[94] - acadoVariables.u[94];
acadoWorkspace.ub[95] = acadoVariables.ubValues[95] - acadoVariables.u[95];
acadoWorkspace.ub[96] = acadoVariables.ubValues[96] - acadoVariables.u[96];
acadoWorkspace.ub[97] = acadoVariables.ubValues[97] - acadoVariables.u[97];
acadoWorkspace.ub[98] = acadoVariables.ubValues[98] - acadoVariables.u[98];
acadoWorkspace.ub[99] = acadoVariables.ubValues[99] - acadoVariables.u[99];
acadoWorkspace.ub[100] = acadoVariables.ubValues[100] - acadoVariables.u[100];
acadoWorkspace.ub[101] = acadoVariables.ubValues[101] - acadoVariables.u[101];
acadoWorkspace.ub[102] = acadoVariables.ubValues[102] - acadoVariables.u[102];
acadoWorkspace.ub[103] = acadoVariables.ubValues[103] - acadoVariables.u[103];
acadoWorkspace.ub[104] = acadoVariables.ubValues[104] - acadoVariables.u[104];
acadoWorkspace.ub[105] = acadoVariables.ubValues[105] - acadoVariables.u[105];
acadoWorkspace.ub[106] = acadoVariables.ubValues[106] - acadoVariables.u[106];
acadoWorkspace.ub[107] = acadoVariables.ubValues[107] - acadoVariables.u[107];
acadoWorkspace.ub[108] = acadoVariables.ubValues[108] - acadoVariables.u[108];
acadoWorkspace.ub[109] = acadoVariables.ubValues[109] - acadoVariables.u[109];
acadoWorkspace.ub[110] = acadoVariables.ubValues[110] - acadoVariables.u[110];
acadoWorkspace.ub[111] = acadoVariables.ubValues[111] - acadoVariables.u[111];
acadoWorkspace.ub[112] = acadoVariables.ubValues[112] - acadoVariables.u[112];
acadoWorkspace.ub[113] = acadoVariables.ubValues[113] - acadoVariables.u[113];
acadoWorkspace.ub[114] = acadoVariables.ubValues[114] - acadoVariables.u[114];
acadoWorkspace.ub[115] = acadoVariables.ubValues[115] - acadoVariables.u[115];
acadoWorkspace.ub[116] = acadoVariables.ubValues[116] - acadoVariables.u[116];
acadoWorkspace.ub[117] = acadoVariables.ubValues[117] - acadoVariables.u[117];
acadoWorkspace.ub[118] = acadoVariables.ubValues[118] - acadoVariables.u[118];
acadoWorkspace.ub[119] = acadoVariables.ubValues[119] - acadoVariables.u[119];

}

void acado_expand(  )
{
int lRun1;
int lRun2;
int lRun3;
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
acadoVariables.u[60] += acadoWorkspace.x[60];
acadoVariables.u[61] += acadoWorkspace.x[61];
acadoVariables.u[62] += acadoWorkspace.x[62];
acadoVariables.u[63] += acadoWorkspace.x[63];
acadoVariables.u[64] += acadoWorkspace.x[64];
acadoVariables.u[65] += acadoWorkspace.x[65];
acadoVariables.u[66] += acadoWorkspace.x[66];
acadoVariables.u[67] += acadoWorkspace.x[67];
acadoVariables.u[68] += acadoWorkspace.x[68];
acadoVariables.u[69] += acadoWorkspace.x[69];
acadoVariables.u[70] += acadoWorkspace.x[70];
acadoVariables.u[71] += acadoWorkspace.x[71];
acadoVariables.u[72] += acadoWorkspace.x[72];
acadoVariables.u[73] += acadoWorkspace.x[73];
acadoVariables.u[74] += acadoWorkspace.x[74];
acadoVariables.u[75] += acadoWorkspace.x[75];
acadoVariables.u[76] += acadoWorkspace.x[76];
acadoVariables.u[77] += acadoWorkspace.x[77];
acadoVariables.u[78] += acadoWorkspace.x[78];
acadoVariables.u[79] += acadoWorkspace.x[79];
acadoVariables.u[80] += acadoWorkspace.x[80];
acadoVariables.u[81] += acadoWorkspace.x[81];
acadoVariables.u[82] += acadoWorkspace.x[82];
acadoVariables.u[83] += acadoWorkspace.x[83];
acadoVariables.u[84] += acadoWorkspace.x[84];
acadoVariables.u[85] += acadoWorkspace.x[85];
acadoVariables.u[86] += acadoWorkspace.x[86];
acadoVariables.u[87] += acadoWorkspace.x[87];
acadoVariables.u[88] += acadoWorkspace.x[88];
acadoVariables.u[89] += acadoWorkspace.x[89];
acadoVariables.u[90] += acadoWorkspace.x[90];
acadoVariables.u[91] += acadoWorkspace.x[91];
acadoVariables.u[92] += acadoWorkspace.x[92];
acadoVariables.u[93] += acadoWorkspace.x[93];
acadoVariables.u[94] += acadoWorkspace.x[94];
acadoVariables.u[95] += acadoWorkspace.x[95];
acadoVariables.u[96] += acadoWorkspace.x[96];
acadoVariables.u[97] += acadoWorkspace.x[97];
acadoVariables.u[98] += acadoWorkspace.x[98];
acadoVariables.u[99] += acadoWorkspace.x[99];
acadoVariables.u[100] += acadoWorkspace.x[100];
acadoVariables.u[101] += acadoWorkspace.x[101];
acadoVariables.u[102] += acadoWorkspace.x[102];
acadoVariables.u[103] += acadoWorkspace.x[103];
acadoVariables.u[104] += acadoWorkspace.x[104];
acadoVariables.u[105] += acadoWorkspace.x[105];
acadoVariables.u[106] += acadoWorkspace.x[106];
acadoVariables.u[107] += acadoWorkspace.x[107];
acadoVariables.u[108] += acadoWorkspace.x[108];
acadoVariables.u[109] += acadoWorkspace.x[109];
acadoVariables.u[110] += acadoWorkspace.x[110];
acadoVariables.u[111] += acadoWorkspace.x[111];
acadoVariables.u[112] += acadoWorkspace.x[112];
acadoVariables.u[113] += acadoWorkspace.x[113];
acadoVariables.u[114] += acadoWorkspace.x[114];
acadoVariables.u[115] += acadoWorkspace.x[115];
acadoVariables.u[116] += acadoWorkspace.x[116];
acadoVariables.u[117] += acadoWorkspace.x[117];
acadoVariables.u[118] += acadoWorkspace.x[118];
acadoVariables.u[119] += acadoWorkspace.x[119];

acadoVariables.x[0] += acadoWorkspace.Dx0[0];
acadoVariables.x[1] += acadoWorkspace.Dx0[1];
acadoVariables.x[2] += acadoWorkspace.Dx0[2];
acadoVariables.x[3] += acadoWorkspace.Dx0[3];

acadoVariables.x[4] += + acadoWorkspace.evGx[0]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[2]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[3]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[0];
acadoVariables.x[5] += + acadoWorkspace.evGx[4]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[5]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[6]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[7]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[1];
acadoVariables.x[6] += + acadoWorkspace.evGx[8]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[9]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[10]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[11]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[2];
acadoVariables.x[7] += + acadoWorkspace.evGx[12]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[13]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[14]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[15]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[3];
acadoVariables.x[8] += + acadoWorkspace.evGx[16]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[17]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[18]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[19]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[4];
acadoVariables.x[9] += + acadoWorkspace.evGx[20]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[21]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[22]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[23]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[5];
acadoVariables.x[10] += + acadoWorkspace.evGx[24]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[25]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[26]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[27]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[6];
acadoVariables.x[11] += + acadoWorkspace.evGx[28]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[29]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[30]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[31]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[7];
acadoVariables.x[12] += + acadoWorkspace.evGx[32]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[33]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[34]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[35]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[8];
acadoVariables.x[13] += + acadoWorkspace.evGx[36]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[37]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[38]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[39]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[9];
acadoVariables.x[14] += + acadoWorkspace.evGx[40]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[41]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[42]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[43]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[10];
acadoVariables.x[15] += + acadoWorkspace.evGx[44]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[45]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[46]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[47]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[11];
acadoVariables.x[16] += + acadoWorkspace.evGx[48]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[49]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[50]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[51]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[12];
acadoVariables.x[17] += + acadoWorkspace.evGx[52]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[53]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[54]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[55]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[13];
acadoVariables.x[18] += + acadoWorkspace.evGx[56]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[57]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[58]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[59]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[14];
acadoVariables.x[19] += + acadoWorkspace.evGx[60]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[61]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[62]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[63]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[15];
acadoVariables.x[20] += + acadoWorkspace.evGx[64]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[65]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[66]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[67]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[16];
acadoVariables.x[21] += + acadoWorkspace.evGx[68]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[69]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[70]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[71]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[17];
acadoVariables.x[22] += + acadoWorkspace.evGx[72]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[73]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[74]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[75]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[18];
acadoVariables.x[23] += + acadoWorkspace.evGx[76]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[77]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[78]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[79]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[19];
acadoVariables.x[24] += + acadoWorkspace.evGx[80]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[81]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[82]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[83]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[20];
acadoVariables.x[25] += + acadoWorkspace.evGx[84]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[85]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[86]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[87]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[21];
acadoVariables.x[26] += + acadoWorkspace.evGx[88]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[89]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[90]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[91]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[22];
acadoVariables.x[27] += + acadoWorkspace.evGx[92]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[93]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[94]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[95]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[23];
acadoVariables.x[28] += + acadoWorkspace.evGx[96]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[97]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[98]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[99]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[24];
acadoVariables.x[29] += + acadoWorkspace.evGx[100]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[101]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[102]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[103]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[25];
acadoVariables.x[30] += + acadoWorkspace.evGx[104]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[105]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[106]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[107]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[26];
acadoVariables.x[31] += + acadoWorkspace.evGx[108]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[109]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[110]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[111]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[27];
acadoVariables.x[32] += + acadoWorkspace.evGx[112]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[113]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[114]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[115]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[28];
acadoVariables.x[33] += + acadoWorkspace.evGx[116]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[117]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[118]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[119]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[29];
acadoVariables.x[34] += + acadoWorkspace.evGx[120]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[121]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[122]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[123]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[30];
acadoVariables.x[35] += + acadoWorkspace.evGx[124]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[125]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[126]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[127]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[31];
acadoVariables.x[36] += + acadoWorkspace.evGx[128]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[129]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[130]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[131]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[32];
acadoVariables.x[37] += + acadoWorkspace.evGx[132]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[133]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[134]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[135]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[33];
acadoVariables.x[38] += + acadoWorkspace.evGx[136]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[137]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[138]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[139]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[34];
acadoVariables.x[39] += + acadoWorkspace.evGx[140]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[141]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[142]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[143]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[35];
acadoVariables.x[40] += + acadoWorkspace.evGx[144]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[145]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[146]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[147]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[36];
acadoVariables.x[41] += + acadoWorkspace.evGx[148]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[149]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[150]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[151]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[37];
acadoVariables.x[42] += + acadoWorkspace.evGx[152]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[153]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[154]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[155]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[38];
acadoVariables.x[43] += + acadoWorkspace.evGx[156]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[157]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[158]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[159]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[39];
acadoVariables.x[44] += + acadoWorkspace.evGx[160]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[161]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[162]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[163]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[40];
acadoVariables.x[45] += + acadoWorkspace.evGx[164]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[165]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[166]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[167]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[41];
acadoVariables.x[46] += + acadoWorkspace.evGx[168]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[169]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[170]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[171]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[42];
acadoVariables.x[47] += + acadoWorkspace.evGx[172]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[173]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[174]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[175]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[43];
acadoVariables.x[48] += + acadoWorkspace.evGx[176]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[177]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[178]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[179]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[44];
acadoVariables.x[49] += + acadoWorkspace.evGx[180]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[181]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[182]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[183]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[45];
acadoVariables.x[50] += + acadoWorkspace.evGx[184]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[185]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[186]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[187]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[46];
acadoVariables.x[51] += + acadoWorkspace.evGx[188]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[189]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[190]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[191]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[47];
acadoVariables.x[52] += + acadoWorkspace.evGx[192]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[193]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[194]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[195]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[48];
acadoVariables.x[53] += + acadoWorkspace.evGx[196]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[197]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[198]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[199]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[49];
acadoVariables.x[54] += + acadoWorkspace.evGx[200]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[201]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[202]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[203]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[50];
acadoVariables.x[55] += + acadoWorkspace.evGx[204]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[205]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[206]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[207]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[51];
acadoVariables.x[56] += + acadoWorkspace.evGx[208]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[209]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[210]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[211]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[52];
acadoVariables.x[57] += + acadoWorkspace.evGx[212]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[213]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[214]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[215]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[53];
acadoVariables.x[58] += + acadoWorkspace.evGx[216]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[217]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[218]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[219]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[54];
acadoVariables.x[59] += + acadoWorkspace.evGx[220]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[221]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[222]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[223]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[55];
acadoVariables.x[60] += + acadoWorkspace.evGx[224]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[225]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[226]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[227]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[56];
acadoVariables.x[61] += + acadoWorkspace.evGx[228]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[229]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[230]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[231]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[57];
acadoVariables.x[62] += + acadoWorkspace.evGx[232]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[233]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[234]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[235]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[58];
acadoVariables.x[63] += + acadoWorkspace.evGx[236]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[237]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[238]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[239]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[59];
acadoVariables.x[64] += + acadoWorkspace.evGx[240]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[241]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[242]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[243]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[60];
acadoVariables.x[65] += + acadoWorkspace.evGx[244]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[245]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[246]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[247]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[61];
acadoVariables.x[66] += + acadoWorkspace.evGx[248]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[249]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[250]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[251]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[62];
acadoVariables.x[67] += + acadoWorkspace.evGx[252]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[253]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[254]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[255]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[63];
acadoVariables.x[68] += + acadoWorkspace.evGx[256]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[257]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[258]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[259]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[64];
acadoVariables.x[69] += + acadoWorkspace.evGx[260]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[261]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[262]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[263]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[65];
acadoVariables.x[70] += + acadoWorkspace.evGx[264]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[265]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[266]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[267]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[66];
acadoVariables.x[71] += + acadoWorkspace.evGx[268]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[269]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[270]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[271]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[67];
acadoVariables.x[72] += + acadoWorkspace.evGx[272]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[273]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[274]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[275]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[68];
acadoVariables.x[73] += + acadoWorkspace.evGx[276]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[277]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[278]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[279]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[69];
acadoVariables.x[74] += + acadoWorkspace.evGx[280]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[281]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[282]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[283]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[70];
acadoVariables.x[75] += + acadoWorkspace.evGx[284]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[285]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[286]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[287]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[71];
acadoVariables.x[76] += + acadoWorkspace.evGx[288]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[289]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[290]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[291]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[72];
acadoVariables.x[77] += + acadoWorkspace.evGx[292]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[293]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[294]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[295]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[73];
acadoVariables.x[78] += + acadoWorkspace.evGx[296]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[297]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[298]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[299]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[74];
acadoVariables.x[79] += + acadoWorkspace.evGx[300]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[301]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[302]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[303]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[75];
acadoVariables.x[80] += + acadoWorkspace.evGx[304]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[305]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[306]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[307]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[76];
acadoVariables.x[81] += + acadoWorkspace.evGx[308]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[309]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[310]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[311]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[77];
acadoVariables.x[82] += + acadoWorkspace.evGx[312]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[313]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[314]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[315]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[78];
acadoVariables.x[83] += + acadoWorkspace.evGx[316]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[317]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[318]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[319]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[79];
acadoVariables.x[84] += + acadoWorkspace.evGx[320]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[321]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[322]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[323]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[80];
acadoVariables.x[85] += + acadoWorkspace.evGx[324]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[325]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[326]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[327]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[81];
acadoVariables.x[86] += + acadoWorkspace.evGx[328]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[329]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[330]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[331]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[82];
acadoVariables.x[87] += + acadoWorkspace.evGx[332]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[333]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[334]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[335]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[83];
acadoVariables.x[88] += + acadoWorkspace.evGx[336]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[337]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[338]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[339]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[84];
acadoVariables.x[89] += + acadoWorkspace.evGx[340]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[341]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[342]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[343]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[85];
acadoVariables.x[90] += + acadoWorkspace.evGx[344]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[345]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[346]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[347]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[86];
acadoVariables.x[91] += + acadoWorkspace.evGx[348]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[349]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[350]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[351]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[87];
acadoVariables.x[92] += + acadoWorkspace.evGx[352]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[353]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[354]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[355]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[88];
acadoVariables.x[93] += + acadoWorkspace.evGx[356]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[357]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[358]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[359]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[89];
acadoVariables.x[94] += + acadoWorkspace.evGx[360]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[361]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[362]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[363]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[90];
acadoVariables.x[95] += + acadoWorkspace.evGx[364]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[365]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[366]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[367]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[91];
acadoVariables.x[96] += + acadoWorkspace.evGx[368]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[369]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[370]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[371]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[92];
acadoVariables.x[97] += + acadoWorkspace.evGx[372]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[373]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[374]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[375]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[93];
acadoVariables.x[98] += + acadoWorkspace.evGx[376]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[377]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[378]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[379]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[94];
acadoVariables.x[99] += + acadoWorkspace.evGx[380]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[381]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[382]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[383]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[95];
acadoVariables.x[100] += + acadoWorkspace.evGx[384]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[385]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[386]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[387]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[96];
acadoVariables.x[101] += + acadoWorkspace.evGx[388]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[389]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[390]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[391]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[97];
acadoVariables.x[102] += + acadoWorkspace.evGx[392]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[393]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[394]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[395]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[98];
acadoVariables.x[103] += + acadoWorkspace.evGx[396]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[397]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[398]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[399]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[99];
acadoVariables.x[104] += + acadoWorkspace.evGx[400]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[401]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[402]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[403]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[100];
acadoVariables.x[105] += + acadoWorkspace.evGx[404]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[405]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[406]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[407]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[101];
acadoVariables.x[106] += + acadoWorkspace.evGx[408]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[409]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[410]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[411]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[102];
acadoVariables.x[107] += + acadoWorkspace.evGx[412]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[413]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[414]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[415]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[103];
acadoVariables.x[108] += + acadoWorkspace.evGx[416]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[417]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[418]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[419]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[104];
acadoVariables.x[109] += + acadoWorkspace.evGx[420]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[421]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[422]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[423]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[105];
acadoVariables.x[110] += + acadoWorkspace.evGx[424]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[425]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[426]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[427]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[106];
acadoVariables.x[111] += + acadoWorkspace.evGx[428]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[429]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[430]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[431]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[107];
acadoVariables.x[112] += + acadoWorkspace.evGx[432]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[433]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[434]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[435]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[108];
acadoVariables.x[113] += + acadoWorkspace.evGx[436]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[437]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[438]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[439]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[109];
acadoVariables.x[114] += + acadoWorkspace.evGx[440]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[441]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[442]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[443]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[110];
acadoVariables.x[115] += + acadoWorkspace.evGx[444]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[445]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[446]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[447]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[111];
acadoVariables.x[116] += + acadoWorkspace.evGx[448]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[449]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[450]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[451]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[112];
acadoVariables.x[117] += + acadoWorkspace.evGx[452]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[453]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[454]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[455]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[113];
acadoVariables.x[118] += + acadoWorkspace.evGx[456]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[457]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[458]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[459]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[114];
acadoVariables.x[119] += + acadoWorkspace.evGx[460]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[461]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[462]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[463]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[115];
acadoVariables.x[120] += + acadoWorkspace.evGx[464]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[465]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[466]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[467]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[116];
acadoVariables.x[121] += + acadoWorkspace.evGx[468]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[469]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[470]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[471]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[117];
acadoVariables.x[122] += + acadoWorkspace.evGx[472]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[473]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[474]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[475]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[118];
acadoVariables.x[123] += + acadoWorkspace.evGx[476]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[477]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[478]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[479]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[119];
acadoVariables.x[124] += + acadoWorkspace.evGx[480]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[481]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[482]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[483]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[120];
acadoVariables.x[125] += + acadoWorkspace.evGx[484]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[485]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[486]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[487]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[121];
acadoVariables.x[126] += + acadoWorkspace.evGx[488]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[489]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[490]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[491]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[122];
acadoVariables.x[127] += + acadoWorkspace.evGx[492]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[493]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[494]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[495]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[123];
acadoVariables.x[128] += + acadoWorkspace.evGx[496]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[497]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[498]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[499]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[124];
acadoVariables.x[129] += + acadoWorkspace.evGx[500]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[501]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[502]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[503]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[125];
acadoVariables.x[130] += + acadoWorkspace.evGx[504]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[505]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[506]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[507]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[126];
acadoVariables.x[131] += + acadoWorkspace.evGx[508]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[509]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[510]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[511]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[127];
acadoVariables.x[132] += + acadoWorkspace.evGx[512]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[513]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[514]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[515]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[128];
acadoVariables.x[133] += + acadoWorkspace.evGx[516]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[517]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[518]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[519]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[129];
acadoVariables.x[134] += + acadoWorkspace.evGx[520]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[521]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[522]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[523]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[130];
acadoVariables.x[135] += + acadoWorkspace.evGx[524]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[525]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[526]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[527]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[131];
acadoVariables.x[136] += + acadoWorkspace.evGx[528]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[529]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[530]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[531]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[132];
acadoVariables.x[137] += + acadoWorkspace.evGx[532]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[533]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[534]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[535]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[133];
acadoVariables.x[138] += + acadoWorkspace.evGx[536]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[537]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[538]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[539]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[134];
acadoVariables.x[139] += + acadoWorkspace.evGx[540]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[541]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[542]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[543]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[135];
acadoVariables.x[140] += + acadoWorkspace.evGx[544]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[545]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[546]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[547]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[136];
acadoVariables.x[141] += + acadoWorkspace.evGx[548]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[549]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[550]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[551]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[137];
acadoVariables.x[142] += + acadoWorkspace.evGx[552]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[553]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[554]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[555]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[138];
acadoVariables.x[143] += + acadoWorkspace.evGx[556]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[557]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[558]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[559]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[139];
acadoVariables.x[144] += + acadoWorkspace.evGx[560]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[561]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[562]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[563]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[140];
acadoVariables.x[145] += + acadoWorkspace.evGx[564]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[565]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[566]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[567]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[141];
acadoVariables.x[146] += + acadoWorkspace.evGx[568]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[569]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[570]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[571]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[142];
acadoVariables.x[147] += + acadoWorkspace.evGx[572]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[573]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[574]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[575]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[143];
acadoVariables.x[148] += + acadoWorkspace.evGx[576]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[577]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[578]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[579]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[144];
acadoVariables.x[149] += + acadoWorkspace.evGx[580]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[581]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[582]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[583]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[145];
acadoVariables.x[150] += + acadoWorkspace.evGx[584]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[585]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[586]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[587]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[146];
acadoVariables.x[151] += + acadoWorkspace.evGx[588]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[589]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[590]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[591]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[147];
acadoVariables.x[152] += + acadoWorkspace.evGx[592]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[593]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[594]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[595]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[148];
acadoVariables.x[153] += + acadoWorkspace.evGx[596]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[597]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[598]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[599]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[149];
acadoVariables.x[154] += + acadoWorkspace.evGx[600]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[601]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[602]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[603]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[150];
acadoVariables.x[155] += + acadoWorkspace.evGx[604]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[605]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[606]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[607]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[151];
acadoVariables.x[156] += + acadoWorkspace.evGx[608]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[609]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[610]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[611]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[152];
acadoVariables.x[157] += + acadoWorkspace.evGx[612]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[613]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[614]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[615]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[153];
acadoVariables.x[158] += + acadoWorkspace.evGx[616]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[617]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[618]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[619]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[154];
acadoVariables.x[159] += + acadoWorkspace.evGx[620]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[621]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[622]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[623]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[155];
acadoVariables.x[160] += + acadoWorkspace.evGx[624]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[625]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[626]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[627]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[156];
acadoVariables.x[161] += + acadoWorkspace.evGx[628]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[629]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[630]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[631]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[157];
acadoVariables.x[162] += + acadoWorkspace.evGx[632]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[633]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[634]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[635]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[158];
acadoVariables.x[163] += + acadoWorkspace.evGx[636]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[637]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[638]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[639]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[159];
acadoVariables.x[164] += + acadoWorkspace.evGx[640]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[641]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[642]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[643]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[160];
acadoVariables.x[165] += + acadoWorkspace.evGx[644]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[645]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[646]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[647]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[161];
acadoVariables.x[166] += + acadoWorkspace.evGx[648]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[649]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[650]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[651]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[162];
acadoVariables.x[167] += + acadoWorkspace.evGx[652]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[653]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[654]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[655]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[163];
acadoVariables.x[168] += + acadoWorkspace.evGx[656]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[657]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[658]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[659]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[164];
acadoVariables.x[169] += + acadoWorkspace.evGx[660]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[661]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[662]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[663]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[165];
acadoVariables.x[170] += + acadoWorkspace.evGx[664]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[665]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[666]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[667]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[166];
acadoVariables.x[171] += + acadoWorkspace.evGx[668]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[669]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[670]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[671]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[167];
acadoVariables.x[172] += + acadoWorkspace.evGx[672]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[673]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[674]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[675]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[168];
acadoVariables.x[173] += + acadoWorkspace.evGx[676]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[677]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[678]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[679]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[169];
acadoVariables.x[174] += + acadoWorkspace.evGx[680]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[681]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[682]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[683]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[170];
acadoVariables.x[175] += + acadoWorkspace.evGx[684]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[685]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[686]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[687]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[171];
acadoVariables.x[176] += + acadoWorkspace.evGx[688]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[689]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[690]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[691]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[172];
acadoVariables.x[177] += + acadoWorkspace.evGx[692]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[693]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[694]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[695]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[173];
acadoVariables.x[178] += + acadoWorkspace.evGx[696]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[697]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[698]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[699]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[174];
acadoVariables.x[179] += + acadoWorkspace.evGx[700]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[701]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[702]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[703]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[175];
acadoVariables.x[180] += + acadoWorkspace.evGx[704]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[705]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[706]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[707]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[176];
acadoVariables.x[181] += + acadoWorkspace.evGx[708]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[709]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[710]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[711]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[177];
acadoVariables.x[182] += + acadoWorkspace.evGx[712]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[713]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[714]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[715]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[178];
acadoVariables.x[183] += + acadoWorkspace.evGx[716]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[717]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[718]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[719]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[179];
acadoVariables.x[184] += + acadoWorkspace.evGx[720]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[721]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[722]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[723]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[180];
acadoVariables.x[185] += + acadoWorkspace.evGx[724]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[725]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[726]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[727]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[181];
acadoVariables.x[186] += + acadoWorkspace.evGx[728]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[729]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[730]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[731]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[182];
acadoVariables.x[187] += + acadoWorkspace.evGx[732]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[733]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[734]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[735]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[183];
acadoVariables.x[188] += + acadoWorkspace.evGx[736]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[737]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[738]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[739]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[184];
acadoVariables.x[189] += + acadoWorkspace.evGx[740]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[741]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[742]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[743]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[185];
acadoVariables.x[190] += + acadoWorkspace.evGx[744]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[745]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[746]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[747]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[186];
acadoVariables.x[191] += + acadoWorkspace.evGx[748]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[749]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[750]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[751]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[187];
acadoVariables.x[192] += + acadoWorkspace.evGx[752]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[753]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[754]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[755]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[188];
acadoVariables.x[193] += + acadoWorkspace.evGx[756]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[757]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[758]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[759]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[189];
acadoVariables.x[194] += + acadoWorkspace.evGx[760]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[761]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[762]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[763]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[190];
acadoVariables.x[195] += + acadoWorkspace.evGx[764]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[765]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[766]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[767]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[191];
acadoVariables.x[196] += + acadoWorkspace.evGx[768]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[769]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[770]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[771]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[192];
acadoVariables.x[197] += + acadoWorkspace.evGx[772]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[773]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[774]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[775]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[193];
acadoVariables.x[198] += + acadoWorkspace.evGx[776]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[777]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[778]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[779]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[194];
acadoVariables.x[199] += + acadoWorkspace.evGx[780]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[781]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[782]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[783]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[195];
acadoVariables.x[200] += + acadoWorkspace.evGx[784]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[785]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[786]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[787]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[196];
acadoVariables.x[201] += + acadoWorkspace.evGx[788]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[789]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[790]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[791]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[197];
acadoVariables.x[202] += + acadoWorkspace.evGx[792]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[793]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[794]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[795]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[198];
acadoVariables.x[203] += + acadoWorkspace.evGx[796]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[797]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[798]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[799]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[199];
acadoVariables.x[204] += + acadoWorkspace.evGx[800]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[801]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[802]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[803]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[200];
acadoVariables.x[205] += + acadoWorkspace.evGx[804]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[805]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[806]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[807]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[201];
acadoVariables.x[206] += + acadoWorkspace.evGx[808]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[809]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[810]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[811]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[202];
acadoVariables.x[207] += + acadoWorkspace.evGx[812]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[813]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[814]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[815]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[203];
acadoVariables.x[208] += + acadoWorkspace.evGx[816]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[817]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[818]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[819]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[204];
acadoVariables.x[209] += + acadoWorkspace.evGx[820]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[821]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[822]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[823]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[205];
acadoVariables.x[210] += + acadoWorkspace.evGx[824]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[825]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[826]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[827]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[206];
acadoVariables.x[211] += + acadoWorkspace.evGx[828]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[829]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[830]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[831]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[207];
acadoVariables.x[212] += + acadoWorkspace.evGx[832]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[833]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[834]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[835]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[208];
acadoVariables.x[213] += + acadoWorkspace.evGx[836]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[837]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[838]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[839]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[209];
acadoVariables.x[214] += + acadoWorkspace.evGx[840]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[841]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[842]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[843]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[210];
acadoVariables.x[215] += + acadoWorkspace.evGx[844]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[845]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[846]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[847]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[211];
acadoVariables.x[216] += + acadoWorkspace.evGx[848]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[849]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[850]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[851]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[212];
acadoVariables.x[217] += + acadoWorkspace.evGx[852]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[853]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[854]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[855]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[213];
acadoVariables.x[218] += + acadoWorkspace.evGx[856]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[857]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[858]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[859]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[214];
acadoVariables.x[219] += + acadoWorkspace.evGx[860]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[861]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[862]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[863]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[215];
acadoVariables.x[220] += + acadoWorkspace.evGx[864]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[865]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[866]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[867]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[216];
acadoVariables.x[221] += + acadoWorkspace.evGx[868]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[869]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[870]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[871]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[217];
acadoVariables.x[222] += + acadoWorkspace.evGx[872]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[873]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[874]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[875]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[218];
acadoVariables.x[223] += + acadoWorkspace.evGx[876]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[877]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[878]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[879]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[219];
acadoVariables.x[224] += + acadoWorkspace.evGx[880]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[881]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[882]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[883]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[220];
acadoVariables.x[225] += + acadoWorkspace.evGx[884]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[885]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[886]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[887]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[221];
acadoVariables.x[226] += + acadoWorkspace.evGx[888]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[889]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[890]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[891]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[222];
acadoVariables.x[227] += + acadoWorkspace.evGx[892]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[893]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[894]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[895]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[223];
acadoVariables.x[228] += + acadoWorkspace.evGx[896]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[897]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[898]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[899]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[224];
acadoVariables.x[229] += + acadoWorkspace.evGx[900]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[901]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[902]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[903]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[225];
acadoVariables.x[230] += + acadoWorkspace.evGx[904]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[905]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[906]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[907]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[226];
acadoVariables.x[231] += + acadoWorkspace.evGx[908]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[909]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[910]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[911]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[227];
acadoVariables.x[232] += + acadoWorkspace.evGx[912]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[913]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[914]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[915]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[228];
acadoVariables.x[233] += + acadoWorkspace.evGx[916]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[917]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[918]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[919]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[229];
acadoVariables.x[234] += + acadoWorkspace.evGx[920]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[921]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[922]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[923]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[230];
acadoVariables.x[235] += + acadoWorkspace.evGx[924]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[925]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[926]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[927]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[231];
acadoVariables.x[236] += + acadoWorkspace.evGx[928]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[929]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[930]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[931]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[232];
acadoVariables.x[237] += + acadoWorkspace.evGx[932]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[933]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[934]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[935]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[233];
acadoVariables.x[238] += + acadoWorkspace.evGx[936]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[937]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[938]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[939]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[234];
acadoVariables.x[239] += + acadoWorkspace.evGx[940]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[941]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[942]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[943]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[235];
acadoVariables.x[240] += + acadoWorkspace.evGx[944]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[945]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[946]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[947]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[236];
acadoVariables.x[241] += + acadoWorkspace.evGx[948]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[949]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[950]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[951]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[237];
acadoVariables.x[242] += + acadoWorkspace.evGx[952]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[953]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[954]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[955]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[238];
acadoVariables.x[243] += + acadoWorkspace.evGx[956]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[957]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[958]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[959]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[239];

for (lRun1 = 0; lRun1 < 60; ++lRun1)
{
for (lRun2 = 0; lRun2 < lRun1 + 1; ++lRun2)
{
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
acado_multEDu( &(acadoWorkspace.E[ lRun3 * 8 ]), &(acadoWorkspace.x[ lRun2 * 2 ]), &(acadoVariables.x[ lRun1 * 4 + 4 ]) );
}
}
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
acadoVariables.lbValues[0] = -5.0000000000000000e-01;
acadoVariables.lbValues[1] = -1.1000000000000001e+00;
acadoVariables.lbValues[2] = -5.0000000000000000e-01;
acadoVariables.lbValues[3] = -1.1000000000000001e+00;
acadoVariables.lbValues[4] = -5.0000000000000000e-01;
acadoVariables.lbValues[5] = -1.1000000000000001e+00;
acadoVariables.lbValues[6] = -5.0000000000000000e-01;
acadoVariables.lbValues[7] = -1.1000000000000001e+00;
acadoVariables.lbValues[8] = -5.0000000000000000e-01;
acadoVariables.lbValues[9] = -1.1000000000000001e+00;
acadoVariables.lbValues[10] = -5.0000000000000000e-01;
acadoVariables.lbValues[11] = -1.1000000000000001e+00;
acadoVariables.lbValues[12] = -5.0000000000000000e-01;
acadoVariables.lbValues[13] = -1.1000000000000001e+00;
acadoVariables.lbValues[14] = -5.0000000000000000e-01;
acadoVariables.lbValues[15] = -1.1000000000000001e+00;
acadoVariables.lbValues[16] = -5.0000000000000000e-01;
acadoVariables.lbValues[17] = -1.1000000000000001e+00;
acadoVariables.lbValues[18] = -5.0000000000000000e-01;
acadoVariables.lbValues[19] = -1.1000000000000001e+00;
acadoVariables.lbValues[20] = -5.0000000000000000e-01;
acadoVariables.lbValues[21] = -1.1000000000000001e+00;
acadoVariables.lbValues[22] = -5.0000000000000000e-01;
acadoVariables.lbValues[23] = -1.1000000000000001e+00;
acadoVariables.lbValues[24] = -5.0000000000000000e-01;
acadoVariables.lbValues[25] = -1.1000000000000001e+00;
acadoVariables.lbValues[26] = -5.0000000000000000e-01;
acadoVariables.lbValues[27] = -1.1000000000000001e+00;
acadoVariables.lbValues[28] = -5.0000000000000000e-01;
acadoVariables.lbValues[29] = -1.1000000000000001e+00;
acadoVariables.lbValues[30] = -5.0000000000000000e-01;
acadoVariables.lbValues[31] = -1.1000000000000001e+00;
acadoVariables.lbValues[32] = -5.0000000000000000e-01;
acadoVariables.lbValues[33] = -1.1000000000000001e+00;
acadoVariables.lbValues[34] = -5.0000000000000000e-01;
acadoVariables.lbValues[35] = -1.1000000000000001e+00;
acadoVariables.lbValues[36] = -5.0000000000000000e-01;
acadoVariables.lbValues[37] = -1.1000000000000001e+00;
acadoVariables.lbValues[38] = -5.0000000000000000e-01;
acadoVariables.lbValues[39] = -1.1000000000000001e+00;
acadoVariables.lbValues[40] = -5.0000000000000000e-01;
acadoVariables.lbValues[41] = -1.1000000000000001e+00;
acadoVariables.lbValues[42] = -5.0000000000000000e-01;
acadoVariables.lbValues[43] = -1.1000000000000001e+00;
acadoVariables.lbValues[44] = -5.0000000000000000e-01;
acadoVariables.lbValues[45] = -1.1000000000000001e+00;
acadoVariables.lbValues[46] = -5.0000000000000000e-01;
acadoVariables.lbValues[47] = -1.1000000000000001e+00;
acadoVariables.lbValues[48] = -5.0000000000000000e-01;
acadoVariables.lbValues[49] = -1.1000000000000001e+00;
acadoVariables.lbValues[50] = -5.0000000000000000e-01;
acadoVariables.lbValues[51] = -1.1000000000000001e+00;
acadoVariables.lbValues[52] = -5.0000000000000000e-01;
acadoVariables.lbValues[53] = -1.1000000000000001e+00;
acadoVariables.lbValues[54] = -5.0000000000000000e-01;
acadoVariables.lbValues[55] = -1.1000000000000001e+00;
acadoVariables.lbValues[56] = -5.0000000000000000e-01;
acadoVariables.lbValues[57] = -1.1000000000000001e+00;
acadoVariables.lbValues[58] = -5.0000000000000000e-01;
acadoVariables.lbValues[59] = -1.1000000000000001e+00;
acadoVariables.lbValues[60] = -5.0000000000000000e-01;
acadoVariables.lbValues[61] = -1.1000000000000001e+00;
acadoVariables.lbValues[62] = -5.0000000000000000e-01;
acadoVariables.lbValues[63] = -1.1000000000000001e+00;
acadoVariables.lbValues[64] = -5.0000000000000000e-01;
acadoVariables.lbValues[65] = -1.1000000000000001e+00;
acadoVariables.lbValues[66] = -5.0000000000000000e-01;
acadoVariables.lbValues[67] = -1.1000000000000001e+00;
acadoVariables.lbValues[68] = -5.0000000000000000e-01;
acadoVariables.lbValues[69] = -1.1000000000000001e+00;
acadoVariables.lbValues[70] = -5.0000000000000000e-01;
acadoVariables.lbValues[71] = -1.1000000000000001e+00;
acadoVariables.lbValues[72] = -5.0000000000000000e-01;
acadoVariables.lbValues[73] = -1.1000000000000001e+00;
acadoVariables.lbValues[74] = -5.0000000000000000e-01;
acadoVariables.lbValues[75] = -1.1000000000000001e+00;
acadoVariables.lbValues[76] = -5.0000000000000000e-01;
acadoVariables.lbValues[77] = -1.1000000000000001e+00;
acadoVariables.lbValues[78] = -5.0000000000000000e-01;
acadoVariables.lbValues[79] = -1.1000000000000001e+00;
acadoVariables.lbValues[80] = -5.0000000000000000e-01;
acadoVariables.lbValues[81] = -1.1000000000000001e+00;
acadoVariables.lbValues[82] = -5.0000000000000000e-01;
acadoVariables.lbValues[83] = -1.1000000000000001e+00;
acadoVariables.lbValues[84] = -5.0000000000000000e-01;
acadoVariables.lbValues[85] = -1.1000000000000001e+00;
acadoVariables.lbValues[86] = -5.0000000000000000e-01;
acadoVariables.lbValues[87] = -1.1000000000000001e+00;
acadoVariables.lbValues[88] = -5.0000000000000000e-01;
acadoVariables.lbValues[89] = -1.1000000000000001e+00;
acadoVariables.lbValues[90] = -5.0000000000000000e-01;
acadoVariables.lbValues[91] = -1.1000000000000001e+00;
acadoVariables.lbValues[92] = -5.0000000000000000e-01;
acadoVariables.lbValues[93] = -1.1000000000000001e+00;
acadoVariables.lbValues[94] = -5.0000000000000000e-01;
acadoVariables.lbValues[95] = -1.1000000000000001e+00;
acadoVariables.lbValues[96] = -5.0000000000000000e-01;
acadoVariables.lbValues[97] = -1.1000000000000001e+00;
acadoVariables.lbValues[98] = -5.0000000000000000e-01;
acadoVariables.lbValues[99] = -1.1000000000000001e+00;
acadoVariables.lbValues[100] = -5.0000000000000000e-01;
acadoVariables.lbValues[101] = -1.1000000000000001e+00;
acadoVariables.lbValues[102] = -5.0000000000000000e-01;
acadoVariables.lbValues[103] = -1.1000000000000001e+00;
acadoVariables.lbValues[104] = -5.0000000000000000e-01;
acadoVariables.lbValues[105] = -1.1000000000000001e+00;
acadoVariables.lbValues[106] = -5.0000000000000000e-01;
acadoVariables.lbValues[107] = -1.1000000000000001e+00;
acadoVariables.lbValues[108] = -5.0000000000000000e-01;
acadoVariables.lbValues[109] = -1.1000000000000001e+00;
acadoVariables.lbValues[110] = -5.0000000000000000e-01;
acadoVariables.lbValues[111] = -1.1000000000000001e+00;
acadoVariables.lbValues[112] = -5.0000000000000000e-01;
acadoVariables.lbValues[113] = -1.1000000000000001e+00;
acadoVariables.lbValues[114] = -5.0000000000000000e-01;
acadoVariables.lbValues[115] = -1.1000000000000001e+00;
acadoVariables.lbValues[116] = -5.0000000000000000e-01;
acadoVariables.lbValues[117] = -1.1000000000000001e+00;
acadoVariables.lbValues[118] = -5.0000000000000000e-01;
acadoVariables.lbValues[119] = -1.1000000000000001e+00;
acadoVariables.ubValues[0] = 5.0000000000000000e-01;
acadoVariables.ubValues[1] = 1.1000000000000001e+00;
acadoVariables.ubValues[2] = 5.0000000000000000e-01;
acadoVariables.ubValues[3] = 1.1000000000000001e+00;
acadoVariables.ubValues[4] = 5.0000000000000000e-01;
acadoVariables.ubValues[5] = 1.1000000000000001e+00;
acadoVariables.ubValues[6] = 5.0000000000000000e-01;
acadoVariables.ubValues[7] = 1.1000000000000001e+00;
acadoVariables.ubValues[8] = 5.0000000000000000e-01;
acadoVariables.ubValues[9] = 1.1000000000000001e+00;
acadoVariables.ubValues[10] = 5.0000000000000000e-01;
acadoVariables.ubValues[11] = 1.1000000000000001e+00;
acadoVariables.ubValues[12] = 5.0000000000000000e-01;
acadoVariables.ubValues[13] = 1.1000000000000001e+00;
acadoVariables.ubValues[14] = 5.0000000000000000e-01;
acadoVariables.ubValues[15] = 1.1000000000000001e+00;
acadoVariables.ubValues[16] = 5.0000000000000000e-01;
acadoVariables.ubValues[17] = 1.1000000000000001e+00;
acadoVariables.ubValues[18] = 5.0000000000000000e-01;
acadoVariables.ubValues[19] = 1.1000000000000001e+00;
acadoVariables.ubValues[20] = 5.0000000000000000e-01;
acadoVariables.ubValues[21] = 1.1000000000000001e+00;
acadoVariables.ubValues[22] = 5.0000000000000000e-01;
acadoVariables.ubValues[23] = 1.1000000000000001e+00;
acadoVariables.ubValues[24] = 5.0000000000000000e-01;
acadoVariables.ubValues[25] = 1.1000000000000001e+00;
acadoVariables.ubValues[26] = 5.0000000000000000e-01;
acadoVariables.ubValues[27] = 1.1000000000000001e+00;
acadoVariables.ubValues[28] = 5.0000000000000000e-01;
acadoVariables.ubValues[29] = 1.1000000000000001e+00;
acadoVariables.ubValues[30] = 5.0000000000000000e-01;
acadoVariables.ubValues[31] = 1.1000000000000001e+00;
acadoVariables.ubValues[32] = 5.0000000000000000e-01;
acadoVariables.ubValues[33] = 1.1000000000000001e+00;
acadoVariables.ubValues[34] = 5.0000000000000000e-01;
acadoVariables.ubValues[35] = 1.1000000000000001e+00;
acadoVariables.ubValues[36] = 5.0000000000000000e-01;
acadoVariables.ubValues[37] = 1.1000000000000001e+00;
acadoVariables.ubValues[38] = 5.0000000000000000e-01;
acadoVariables.ubValues[39] = 1.1000000000000001e+00;
acadoVariables.ubValues[40] = 5.0000000000000000e-01;
acadoVariables.ubValues[41] = 1.1000000000000001e+00;
acadoVariables.ubValues[42] = 5.0000000000000000e-01;
acadoVariables.ubValues[43] = 1.1000000000000001e+00;
acadoVariables.ubValues[44] = 5.0000000000000000e-01;
acadoVariables.ubValues[45] = 1.1000000000000001e+00;
acadoVariables.ubValues[46] = 5.0000000000000000e-01;
acadoVariables.ubValues[47] = 1.1000000000000001e+00;
acadoVariables.ubValues[48] = 5.0000000000000000e-01;
acadoVariables.ubValues[49] = 1.1000000000000001e+00;
acadoVariables.ubValues[50] = 5.0000000000000000e-01;
acadoVariables.ubValues[51] = 1.1000000000000001e+00;
acadoVariables.ubValues[52] = 5.0000000000000000e-01;
acadoVariables.ubValues[53] = 1.1000000000000001e+00;
acadoVariables.ubValues[54] = 5.0000000000000000e-01;
acadoVariables.ubValues[55] = 1.1000000000000001e+00;
acadoVariables.ubValues[56] = 5.0000000000000000e-01;
acadoVariables.ubValues[57] = 1.1000000000000001e+00;
acadoVariables.ubValues[58] = 5.0000000000000000e-01;
acadoVariables.ubValues[59] = 1.1000000000000001e+00;
acadoVariables.ubValues[60] = 5.0000000000000000e-01;
acadoVariables.ubValues[61] = 1.1000000000000001e+00;
acadoVariables.ubValues[62] = 5.0000000000000000e-01;
acadoVariables.ubValues[63] = 1.1000000000000001e+00;
acadoVariables.ubValues[64] = 5.0000000000000000e-01;
acadoVariables.ubValues[65] = 1.1000000000000001e+00;
acadoVariables.ubValues[66] = 5.0000000000000000e-01;
acadoVariables.ubValues[67] = 1.1000000000000001e+00;
acadoVariables.ubValues[68] = 5.0000000000000000e-01;
acadoVariables.ubValues[69] = 1.1000000000000001e+00;
acadoVariables.ubValues[70] = 5.0000000000000000e-01;
acadoVariables.ubValues[71] = 1.1000000000000001e+00;
acadoVariables.ubValues[72] = 5.0000000000000000e-01;
acadoVariables.ubValues[73] = 1.1000000000000001e+00;
acadoVariables.ubValues[74] = 5.0000000000000000e-01;
acadoVariables.ubValues[75] = 1.1000000000000001e+00;
acadoVariables.ubValues[76] = 5.0000000000000000e-01;
acadoVariables.ubValues[77] = 1.1000000000000001e+00;
acadoVariables.ubValues[78] = 5.0000000000000000e-01;
acadoVariables.ubValues[79] = 1.1000000000000001e+00;
acadoVariables.ubValues[80] = 5.0000000000000000e-01;
acadoVariables.ubValues[81] = 1.1000000000000001e+00;
acadoVariables.ubValues[82] = 5.0000000000000000e-01;
acadoVariables.ubValues[83] = 1.1000000000000001e+00;
acadoVariables.ubValues[84] = 5.0000000000000000e-01;
acadoVariables.ubValues[85] = 1.1000000000000001e+00;
acadoVariables.ubValues[86] = 5.0000000000000000e-01;
acadoVariables.ubValues[87] = 1.1000000000000001e+00;
acadoVariables.ubValues[88] = 5.0000000000000000e-01;
acadoVariables.ubValues[89] = 1.1000000000000001e+00;
acadoVariables.ubValues[90] = 5.0000000000000000e-01;
acadoVariables.ubValues[91] = 1.1000000000000001e+00;
acadoVariables.ubValues[92] = 5.0000000000000000e-01;
acadoVariables.ubValues[93] = 1.1000000000000001e+00;
acadoVariables.ubValues[94] = 5.0000000000000000e-01;
acadoVariables.ubValues[95] = 1.1000000000000001e+00;
acadoVariables.ubValues[96] = 5.0000000000000000e-01;
acadoVariables.ubValues[97] = 1.1000000000000001e+00;
acadoVariables.ubValues[98] = 5.0000000000000000e-01;
acadoVariables.ubValues[99] = 1.1000000000000001e+00;
acadoVariables.ubValues[100] = 5.0000000000000000e-01;
acadoVariables.ubValues[101] = 1.1000000000000001e+00;
acadoVariables.ubValues[102] = 5.0000000000000000e-01;
acadoVariables.ubValues[103] = 1.1000000000000001e+00;
acadoVariables.ubValues[104] = 5.0000000000000000e-01;
acadoVariables.ubValues[105] = 1.1000000000000001e+00;
acadoVariables.ubValues[106] = 5.0000000000000000e-01;
acadoVariables.ubValues[107] = 1.1000000000000001e+00;
acadoVariables.ubValues[108] = 5.0000000000000000e-01;
acadoVariables.ubValues[109] = 1.1000000000000001e+00;
acadoVariables.ubValues[110] = 5.0000000000000000e-01;
acadoVariables.ubValues[111] = 1.1000000000000001e+00;
acadoVariables.ubValues[112] = 5.0000000000000000e-01;
acadoVariables.ubValues[113] = 1.1000000000000001e+00;
acadoVariables.ubValues[114] = 5.0000000000000000e-01;
acadoVariables.ubValues[115] = 1.1000000000000001e+00;
acadoVariables.ubValues[116] = 5.0000000000000000e-01;
acadoVariables.ubValues[117] = 1.1000000000000001e+00;
acadoVariables.ubValues[118] = 5.0000000000000000e-01;
acadoVariables.ubValues[119] = 1.1000000000000001e+00;
return ret;
}

void acado_initializeNodesByForwardSimulation(  )
{
int index;
for (index = 0; index < 60; ++index)
{
acadoWorkspace.state[0] = acadoVariables.x[index * 4];
acadoWorkspace.state[1] = acadoVariables.x[index * 4 + 1];
acadoWorkspace.state[2] = acadoVariables.x[index * 4 + 2];
acadoWorkspace.state[3] = acadoVariables.x[index * 4 + 3];
acadoWorkspace.state[28] = acadoVariables.u[index * 2];
acadoWorkspace.state[29] = acadoVariables.u[index * 2 + 1];
acadoWorkspace.state[30] = acadoVariables.od[index * 13];
acadoWorkspace.state[31] = acadoVariables.od[index * 13 + 1];
acadoWorkspace.state[32] = acadoVariables.od[index * 13 + 2];
acadoWorkspace.state[33] = acadoVariables.od[index * 13 + 3];
acadoWorkspace.state[34] = acadoVariables.od[index * 13 + 4];
acadoWorkspace.state[35] = acadoVariables.od[index * 13 + 5];
acadoWorkspace.state[36] = acadoVariables.od[index * 13 + 6];
acadoWorkspace.state[37] = acadoVariables.od[index * 13 + 7];
acadoWorkspace.state[38] = acadoVariables.od[index * 13 + 8];
acadoWorkspace.state[39] = acadoVariables.od[index * 13 + 9];
acadoWorkspace.state[40] = acadoVariables.od[index * 13 + 10];
acadoWorkspace.state[41] = acadoVariables.od[index * 13 + 11];
acadoWorkspace.state[42] = acadoVariables.od[index * 13 + 12];

acado_integrate(acadoWorkspace.state, index == 0);

acadoVariables.x[index * 4 + 4] = acadoWorkspace.state[0];
acadoVariables.x[index * 4 + 5] = acadoWorkspace.state[1];
acadoVariables.x[index * 4 + 6] = acadoWorkspace.state[2];
acadoVariables.x[index * 4 + 7] = acadoWorkspace.state[3];
}
}

void acado_shiftStates( int strategy, real_t* const xEnd, real_t* const uEnd )
{
int index;
for (index = 0; index < 60; ++index)
{
acadoVariables.x[index * 4] = acadoVariables.x[index * 4 + 4];
acadoVariables.x[index * 4 + 1] = acadoVariables.x[index * 4 + 5];
acadoVariables.x[index * 4 + 2] = acadoVariables.x[index * 4 + 6];
acadoVariables.x[index * 4 + 3] = acadoVariables.x[index * 4 + 7];
}

if (strategy == 1 && xEnd != 0)
{
acadoVariables.x[240] = xEnd[0];
acadoVariables.x[241] = xEnd[1];
acadoVariables.x[242] = xEnd[2];
acadoVariables.x[243] = xEnd[3];
}
else if (strategy == 2) 
{
acadoWorkspace.state[0] = acadoVariables.x[240];
acadoWorkspace.state[1] = acadoVariables.x[241];
acadoWorkspace.state[2] = acadoVariables.x[242];
acadoWorkspace.state[3] = acadoVariables.x[243];
if (uEnd != 0)
{
acadoWorkspace.state[28] = uEnd[0];
acadoWorkspace.state[29] = uEnd[1];
}
else
{
acadoWorkspace.state[28] = acadoVariables.u[118];
acadoWorkspace.state[29] = acadoVariables.u[119];
}
acadoWorkspace.state[30] = acadoVariables.od[780];
acadoWorkspace.state[31] = acadoVariables.od[781];
acadoWorkspace.state[32] = acadoVariables.od[782];
acadoWorkspace.state[33] = acadoVariables.od[783];
acadoWorkspace.state[34] = acadoVariables.od[784];
acadoWorkspace.state[35] = acadoVariables.od[785];
acadoWorkspace.state[36] = acadoVariables.od[786];
acadoWorkspace.state[37] = acadoVariables.od[787];
acadoWorkspace.state[38] = acadoVariables.od[788];
acadoWorkspace.state[39] = acadoVariables.od[789];
acadoWorkspace.state[40] = acadoVariables.od[790];
acadoWorkspace.state[41] = acadoVariables.od[791];
acadoWorkspace.state[42] = acadoVariables.od[792];

acado_integrate(acadoWorkspace.state, 1);

acadoVariables.x[240] = acadoWorkspace.state[0];
acadoVariables.x[241] = acadoWorkspace.state[1];
acadoVariables.x[242] = acadoWorkspace.state[2];
acadoVariables.x[243] = acadoWorkspace.state[3];
}
}

void acado_shiftControls( real_t* const uEnd )
{
int index;
for (index = 0; index < 59; ++index)
{
acadoVariables.u[index * 2] = acadoVariables.u[index * 2 + 2];
acadoVariables.u[index * 2 + 1] = acadoVariables.u[index * 2 + 3];
}

if (uEnd != 0)
{
acadoVariables.u[118] = uEnd[0];
acadoVariables.u[119] = uEnd[1];
}
}

real_t acado_getKKT(  )
{
real_t kkt;

int index;
real_t prd;

kkt = + acadoWorkspace.g[0]*acadoWorkspace.x[0] + acadoWorkspace.g[1]*acadoWorkspace.x[1] + acadoWorkspace.g[2]*acadoWorkspace.x[2] + acadoWorkspace.g[3]*acadoWorkspace.x[3] + acadoWorkspace.g[4]*acadoWorkspace.x[4] + acadoWorkspace.g[5]*acadoWorkspace.x[5] + acadoWorkspace.g[6]*acadoWorkspace.x[6] + acadoWorkspace.g[7]*acadoWorkspace.x[7] + acadoWorkspace.g[8]*acadoWorkspace.x[8] + acadoWorkspace.g[9]*acadoWorkspace.x[9] + acadoWorkspace.g[10]*acadoWorkspace.x[10] + acadoWorkspace.g[11]*acadoWorkspace.x[11] + acadoWorkspace.g[12]*acadoWorkspace.x[12] + acadoWorkspace.g[13]*acadoWorkspace.x[13] + acadoWorkspace.g[14]*acadoWorkspace.x[14] + acadoWorkspace.g[15]*acadoWorkspace.x[15] + acadoWorkspace.g[16]*acadoWorkspace.x[16] + acadoWorkspace.g[17]*acadoWorkspace.x[17] + acadoWorkspace.g[18]*acadoWorkspace.x[18] + acadoWorkspace.g[19]*acadoWorkspace.x[19] + acadoWorkspace.g[20]*acadoWorkspace.x[20] + acadoWorkspace.g[21]*acadoWorkspace.x[21] + acadoWorkspace.g[22]*acadoWorkspace.x[22] + acadoWorkspace.g[23]*acadoWorkspace.x[23] + acadoWorkspace.g[24]*acadoWorkspace.x[24] + acadoWorkspace.g[25]*acadoWorkspace.x[25] + acadoWorkspace.g[26]*acadoWorkspace.x[26] + acadoWorkspace.g[27]*acadoWorkspace.x[27] + acadoWorkspace.g[28]*acadoWorkspace.x[28] + acadoWorkspace.g[29]*acadoWorkspace.x[29] + acadoWorkspace.g[30]*acadoWorkspace.x[30] + acadoWorkspace.g[31]*acadoWorkspace.x[31] + acadoWorkspace.g[32]*acadoWorkspace.x[32] + acadoWorkspace.g[33]*acadoWorkspace.x[33] + acadoWorkspace.g[34]*acadoWorkspace.x[34] + acadoWorkspace.g[35]*acadoWorkspace.x[35] + acadoWorkspace.g[36]*acadoWorkspace.x[36] + acadoWorkspace.g[37]*acadoWorkspace.x[37] + acadoWorkspace.g[38]*acadoWorkspace.x[38] + acadoWorkspace.g[39]*acadoWorkspace.x[39] + acadoWorkspace.g[40]*acadoWorkspace.x[40] + acadoWorkspace.g[41]*acadoWorkspace.x[41] + acadoWorkspace.g[42]*acadoWorkspace.x[42] + acadoWorkspace.g[43]*acadoWorkspace.x[43] + acadoWorkspace.g[44]*acadoWorkspace.x[44] + acadoWorkspace.g[45]*acadoWorkspace.x[45] + acadoWorkspace.g[46]*acadoWorkspace.x[46] + acadoWorkspace.g[47]*acadoWorkspace.x[47] + acadoWorkspace.g[48]*acadoWorkspace.x[48] + acadoWorkspace.g[49]*acadoWorkspace.x[49] + acadoWorkspace.g[50]*acadoWorkspace.x[50] + acadoWorkspace.g[51]*acadoWorkspace.x[51] + acadoWorkspace.g[52]*acadoWorkspace.x[52] + acadoWorkspace.g[53]*acadoWorkspace.x[53] + acadoWorkspace.g[54]*acadoWorkspace.x[54] + acadoWorkspace.g[55]*acadoWorkspace.x[55] + acadoWorkspace.g[56]*acadoWorkspace.x[56] + acadoWorkspace.g[57]*acadoWorkspace.x[57] + acadoWorkspace.g[58]*acadoWorkspace.x[58] + acadoWorkspace.g[59]*acadoWorkspace.x[59] + acadoWorkspace.g[60]*acadoWorkspace.x[60] + acadoWorkspace.g[61]*acadoWorkspace.x[61] + acadoWorkspace.g[62]*acadoWorkspace.x[62] + acadoWorkspace.g[63]*acadoWorkspace.x[63] + acadoWorkspace.g[64]*acadoWorkspace.x[64] + acadoWorkspace.g[65]*acadoWorkspace.x[65] + acadoWorkspace.g[66]*acadoWorkspace.x[66] + acadoWorkspace.g[67]*acadoWorkspace.x[67] + acadoWorkspace.g[68]*acadoWorkspace.x[68] + acadoWorkspace.g[69]*acadoWorkspace.x[69] + acadoWorkspace.g[70]*acadoWorkspace.x[70] + acadoWorkspace.g[71]*acadoWorkspace.x[71] + acadoWorkspace.g[72]*acadoWorkspace.x[72] + acadoWorkspace.g[73]*acadoWorkspace.x[73] + acadoWorkspace.g[74]*acadoWorkspace.x[74] + acadoWorkspace.g[75]*acadoWorkspace.x[75] + acadoWorkspace.g[76]*acadoWorkspace.x[76] + acadoWorkspace.g[77]*acadoWorkspace.x[77] + acadoWorkspace.g[78]*acadoWorkspace.x[78] + acadoWorkspace.g[79]*acadoWorkspace.x[79] + acadoWorkspace.g[80]*acadoWorkspace.x[80] + acadoWorkspace.g[81]*acadoWorkspace.x[81] + acadoWorkspace.g[82]*acadoWorkspace.x[82] + acadoWorkspace.g[83]*acadoWorkspace.x[83] + acadoWorkspace.g[84]*acadoWorkspace.x[84] + acadoWorkspace.g[85]*acadoWorkspace.x[85] + acadoWorkspace.g[86]*acadoWorkspace.x[86] + acadoWorkspace.g[87]*acadoWorkspace.x[87] + acadoWorkspace.g[88]*acadoWorkspace.x[88] + acadoWorkspace.g[89]*acadoWorkspace.x[89] + acadoWorkspace.g[90]*acadoWorkspace.x[90] + acadoWorkspace.g[91]*acadoWorkspace.x[91] + acadoWorkspace.g[92]*acadoWorkspace.x[92] + acadoWorkspace.g[93]*acadoWorkspace.x[93] + acadoWorkspace.g[94]*acadoWorkspace.x[94] + acadoWorkspace.g[95]*acadoWorkspace.x[95] + acadoWorkspace.g[96]*acadoWorkspace.x[96] + acadoWorkspace.g[97]*acadoWorkspace.x[97] + acadoWorkspace.g[98]*acadoWorkspace.x[98] + acadoWorkspace.g[99]*acadoWorkspace.x[99] + acadoWorkspace.g[100]*acadoWorkspace.x[100] + acadoWorkspace.g[101]*acadoWorkspace.x[101] + acadoWorkspace.g[102]*acadoWorkspace.x[102] + acadoWorkspace.g[103]*acadoWorkspace.x[103] + acadoWorkspace.g[104]*acadoWorkspace.x[104] + acadoWorkspace.g[105]*acadoWorkspace.x[105] + acadoWorkspace.g[106]*acadoWorkspace.x[106] + acadoWorkspace.g[107]*acadoWorkspace.x[107] + acadoWorkspace.g[108]*acadoWorkspace.x[108] + acadoWorkspace.g[109]*acadoWorkspace.x[109] + acadoWorkspace.g[110]*acadoWorkspace.x[110] + acadoWorkspace.g[111]*acadoWorkspace.x[111] + acadoWorkspace.g[112]*acadoWorkspace.x[112] + acadoWorkspace.g[113]*acadoWorkspace.x[113] + acadoWorkspace.g[114]*acadoWorkspace.x[114] + acadoWorkspace.g[115]*acadoWorkspace.x[115] + acadoWorkspace.g[116]*acadoWorkspace.x[116] + acadoWorkspace.g[117]*acadoWorkspace.x[117] + acadoWorkspace.g[118]*acadoWorkspace.x[118] + acadoWorkspace.g[119]*acadoWorkspace.x[119];
kkt = fabs( kkt );
for (index = 0; index < 120; ++index)
{
prd = acadoWorkspace.y[index];
if (prd > 1e-12)
kkt += fabs(acadoWorkspace.lb[index] * prd);
else if (prd < -1e-12)
kkt += fabs(acadoWorkspace.ub[index] * prd);
}
return kkt;
}

real_t acado_getObjective(  )
{
real_t objVal;

int lRun1;
/** Row vector of size: 9 */
real_t tmpDy[ 9 ];

/** Row vector of size: 3 */
real_t tmpDyN[ 3 ];

for (lRun1 = 0; lRun1 < 60; ++lRun1)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[lRun1 * 4];
acadoWorkspace.objValueIn[1] = acadoVariables.x[lRun1 * 4 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[lRun1 * 4 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.x[lRun1 * 4 + 3];
acadoWorkspace.objValueIn[4] = acadoVariables.u[lRun1 * 2];
acadoWorkspace.objValueIn[5] = acadoVariables.u[lRun1 * 2 + 1];
acadoWorkspace.objValueIn[6] = acadoVariables.od[lRun1 * 13];
acadoWorkspace.objValueIn[7] = acadoVariables.od[lRun1 * 13 + 1];
acadoWorkspace.objValueIn[8] = acadoVariables.od[lRun1 * 13 + 2];
acadoWorkspace.objValueIn[9] = acadoVariables.od[lRun1 * 13 + 3];
acadoWorkspace.objValueIn[10] = acadoVariables.od[lRun1 * 13 + 4];
acadoWorkspace.objValueIn[11] = acadoVariables.od[lRun1 * 13 + 5];
acadoWorkspace.objValueIn[12] = acadoVariables.od[lRun1 * 13 + 6];
acadoWorkspace.objValueIn[13] = acadoVariables.od[lRun1 * 13 + 7];
acadoWorkspace.objValueIn[14] = acadoVariables.od[lRun1 * 13 + 8];
acadoWorkspace.objValueIn[15] = acadoVariables.od[lRun1 * 13 + 9];
acadoWorkspace.objValueIn[16] = acadoVariables.od[lRun1 * 13 + 10];
acadoWorkspace.objValueIn[17] = acadoVariables.od[lRun1 * 13 + 11];
acadoWorkspace.objValueIn[18] = acadoVariables.od[lRun1 * 13 + 12];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[lRun1 * 9] = acadoWorkspace.objValueOut[0] - acadoVariables.y[lRun1 * 9];
acadoWorkspace.Dy[lRun1 * 9 + 1] = acadoWorkspace.objValueOut[1] - acadoVariables.y[lRun1 * 9 + 1];
acadoWorkspace.Dy[lRun1 * 9 + 2] = acadoWorkspace.objValueOut[2] - acadoVariables.y[lRun1 * 9 + 2];
acadoWorkspace.Dy[lRun1 * 9 + 3] = acadoWorkspace.objValueOut[3] - acadoVariables.y[lRun1 * 9 + 3];
acadoWorkspace.Dy[lRun1 * 9 + 4] = acadoWorkspace.objValueOut[4] - acadoVariables.y[lRun1 * 9 + 4];
acadoWorkspace.Dy[lRun1 * 9 + 5] = acadoWorkspace.objValueOut[5] - acadoVariables.y[lRun1 * 9 + 5];
acadoWorkspace.Dy[lRun1 * 9 + 6] = acadoWorkspace.objValueOut[6] - acadoVariables.y[lRun1 * 9 + 6];
acadoWorkspace.Dy[lRun1 * 9 + 7] = acadoWorkspace.objValueOut[7] - acadoVariables.y[lRun1 * 9 + 7];
acadoWorkspace.Dy[lRun1 * 9 + 8] = acadoWorkspace.objValueOut[8] - acadoVariables.y[lRun1 * 9 + 8];
}
acadoWorkspace.objValueIn[0] = acadoVariables.x[240];
acadoWorkspace.objValueIn[1] = acadoVariables.x[241];
acadoWorkspace.objValueIn[2] = acadoVariables.x[242];
acadoWorkspace.objValueIn[3] = acadoVariables.x[243];
acadoWorkspace.objValueIn[4] = acadoVariables.od[780];
acadoWorkspace.objValueIn[5] = acadoVariables.od[781];
acadoWorkspace.objValueIn[6] = acadoVariables.od[782];
acadoWorkspace.objValueIn[7] = acadoVariables.od[783];
acadoWorkspace.objValueIn[8] = acadoVariables.od[784];
acadoWorkspace.objValueIn[9] = acadoVariables.od[785];
acadoWorkspace.objValueIn[10] = acadoVariables.od[786];
acadoWorkspace.objValueIn[11] = acadoVariables.od[787];
acadoWorkspace.objValueIn[12] = acadoVariables.od[788];
acadoWorkspace.objValueIn[13] = acadoVariables.od[789];
acadoWorkspace.objValueIn[14] = acadoVariables.od[790];
acadoWorkspace.objValueIn[15] = acadoVariables.od[791];
acadoWorkspace.objValueIn[16] = acadoVariables.od[792];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0] - acadoVariables.yN[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1] - acadoVariables.yN[1];
acadoWorkspace.DyN[2] = acadoWorkspace.objValueOut[2] - acadoVariables.yN[2];
objVal = 0.0000000000000000e+00;
for (lRun1 = 0; lRun1 < 60; ++lRun1)
{
tmpDy[0] = + acadoWorkspace.Dy[lRun1 * 9]*acadoVariables.W[0];
tmpDy[1] = + acadoWorkspace.Dy[lRun1 * 9 + 1]*acadoVariables.W[10];
tmpDy[2] = + acadoWorkspace.Dy[lRun1 * 9 + 2]*acadoVariables.W[20];
tmpDy[3] = + acadoWorkspace.Dy[lRun1 * 9 + 3]*acadoVariables.W[30];
tmpDy[4] = + acadoWorkspace.Dy[lRun1 * 9 + 4]*acadoVariables.W[40];
tmpDy[5] = + acadoWorkspace.Dy[lRun1 * 9 + 5]*acadoVariables.W[50];
tmpDy[6] = + acadoWorkspace.Dy[lRun1 * 9 + 6]*acadoVariables.W[60];
tmpDy[7] = + acadoWorkspace.Dy[lRun1 * 9 + 7]*acadoVariables.W[70];
tmpDy[8] = + acadoWorkspace.Dy[lRun1 * 9 + 8]*acadoVariables.W[80];
objVal += + acadoWorkspace.Dy[lRun1 * 9]*tmpDy[0] + acadoWorkspace.Dy[lRun1 * 9 + 1]*tmpDy[1] + acadoWorkspace.Dy[lRun1 * 9 + 2]*tmpDy[2] + acadoWorkspace.Dy[lRun1 * 9 + 3]*tmpDy[3] + acadoWorkspace.Dy[lRun1 * 9 + 4]*tmpDy[4] + acadoWorkspace.Dy[lRun1 * 9 + 5]*tmpDy[5] + acadoWorkspace.Dy[lRun1 * 9 + 6]*tmpDy[6] + acadoWorkspace.Dy[lRun1 * 9 + 7]*tmpDy[7] + acadoWorkspace.Dy[lRun1 * 9 + 8]*tmpDy[8];
}

tmpDyN[0] = + acadoWorkspace.DyN[0]*acadoVariables.WN[0];
tmpDyN[1] = + acadoWorkspace.DyN[1]*acadoVariables.WN[4];
tmpDyN[2] = + acadoWorkspace.DyN[2]*acadoVariables.WN[8];
objVal += + acadoWorkspace.DyN[0]*tmpDyN[0] + acadoWorkspace.DyN[1]*tmpDyN[1] + acadoWorkspace.DyN[2]*tmpDyN[2];

objVal *= 0.5;
return objVal;
}

