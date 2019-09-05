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
acadoWorkspace.state[0] = acadoVariables.x[lRun1 * 4];
acadoWorkspace.state[1] = acadoVariables.x[lRun1 * 4 + 1];
acadoWorkspace.state[2] = acadoVariables.x[lRun1 * 4 + 2];
acadoWorkspace.state[3] = acadoVariables.x[lRun1 * 4 + 3];

acadoWorkspace.state[28] = acadoVariables.u[lRun1 * 2];
acadoWorkspace.state[29] = acadoVariables.u[lRun1 * 2 + 1];
acadoWorkspace.state[30] = acadoVariables.od[lRun1 * 15];
acadoWorkspace.state[31] = acadoVariables.od[lRun1 * 15 + 1];
acadoWorkspace.state[32] = acadoVariables.od[lRun1 * 15 + 2];
acadoWorkspace.state[33] = acadoVariables.od[lRun1 * 15 + 3];
acadoWorkspace.state[34] = acadoVariables.od[lRun1 * 15 + 4];
acadoWorkspace.state[35] = acadoVariables.od[lRun1 * 15 + 5];
acadoWorkspace.state[36] = acadoVariables.od[lRun1 * 15 + 6];
acadoWorkspace.state[37] = acadoVariables.od[lRun1 * 15 + 7];
acadoWorkspace.state[38] = acadoVariables.od[lRun1 * 15 + 8];
acadoWorkspace.state[39] = acadoVariables.od[lRun1 * 15 + 9];
acadoWorkspace.state[40] = acadoVariables.od[lRun1 * 15 + 10];
acadoWorkspace.state[41] = acadoVariables.od[lRun1 * 15 + 11];
acadoWorkspace.state[42] = acadoVariables.od[lRun1 * 15 + 12];
acadoWorkspace.state[43] = acadoVariables.od[lRun1 * 15 + 13];
acadoWorkspace.state[44] = acadoVariables.od[lRun1 * 15 + 14];

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
const real_t* u = in + 4;
const real_t* od = in + 6;
/* Vector of auxiliary variables; number of elements: 49. */
real_t* a = acadoWorkspace.objAuxVar;

/* Compute intermediate quantities: */
a[0] = (sqrt((((xd[0]-od[0])*(xd[0]-od[0]))+((xd[1]-od[1])*(xd[1]-od[1])))));
a[1] = (sqrt((((xd[0]-od[2])*(xd[0]-od[2]))+((xd[1]-od[3])*(xd[1]-od[3])))));
a[2] = (sqrt((((xd[0]-od[4])*(xd[0]-od[4]))+((xd[1]-od[5])*(xd[1]-od[5])))));
a[3] = (sqrt((((xd[0]-od[6])*(xd[0]-od[6]))+((xd[1]-od[7])*(xd[1]-od[7])))));
a[4] = (sqrt((((xd[0]-od[8])*(xd[0]-od[8]))+((xd[1]-od[9])*(xd[1]-od[9])))));
a[5] = (sqrt((((xd[0]-od[10])*(xd[0]-od[10]))+((xd[1]-od[11])*(xd[1]-od[11])))));
a[6] = (sqrt((((xd[0]-od[12])*(xd[0]-od[12]))+((xd[1]-od[13])*(xd[1]-od[13])))));
a[7] = (1.0/sqrt((((xd[0]-od[0])*(xd[0]-od[0]))+((xd[1]-od[1])*(xd[1]-od[1])))));
a[8] = (a[7]*(real_t)(5.0000000000000000e-01));
a[9] = (((xd[0]-od[0])+(xd[0]-od[0]))*a[8]);
a[10] = ((real_t)(1.0000000000000000e+00)/a[0]);
a[11] = (a[10]*a[10]);
a[12] = (((xd[1]-od[1])+(xd[1]-od[1]))*a[8]);
a[13] = (1.0/sqrt((((xd[0]-od[2])*(xd[0]-od[2]))+((xd[1]-od[3])*(xd[1]-od[3])))));
a[14] = (a[13]*(real_t)(5.0000000000000000e-01));
a[15] = (((xd[0]-od[2])+(xd[0]-od[2]))*a[14]);
a[16] = ((real_t)(1.0000000000000000e+00)/a[1]);
a[17] = (a[16]*a[16]);
a[18] = (((xd[1]-od[3])+(xd[1]-od[3]))*a[14]);
a[19] = (1.0/sqrt((((xd[0]-od[4])*(xd[0]-od[4]))+((xd[1]-od[5])*(xd[1]-od[5])))));
a[20] = (a[19]*(real_t)(5.0000000000000000e-01));
a[21] = (((xd[0]-od[4])+(xd[0]-od[4]))*a[20]);
a[22] = ((real_t)(1.0000000000000000e+00)/a[2]);
a[23] = (a[22]*a[22]);
a[24] = (((xd[1]-od[5])+(xd[1]-od[5]))*a[20]);
a[25] = (1.0/sqrt((((xd[0]-od[6])*(xd[0]-od[6]))+((xd[1]-od[7])*(xd[1]-od[7])))));
a[26] = (a[25]*(real_t)(5.0000000000000000e-01));
a[27] = (((xd[0]-od[6])+(xd[0]-od[6]))*a[26]);
a[28] = ((real_t)(1.0000000000000000e+00)/a[3]);
a[29] = (a[28]*a[28]);
a[30] = (((xd[1]-od[7])+(xd[1]-od[7]))*a[26]);
a[31] = (1.0/sqrt((((xd[0]-od[8])*(xd[0]-od[8]))+((xd[1]-od[9])*(xd[1]-od[9])))));
a[32] = (a[31]*(real_t)(5.0000000000000000e-01));
a[33] = (((xd[0]-od[8])+(xd[0]-od[8]))*a[32]);
a[34] = ((real_t)(1.0000000000000000e+00)/a[4]);
a[35] = (a[34]*a[34]);
a[36] = (((xd[1]-od[9])+(xd[1]-od[9]))*a[32]);
a[37] = (1.0/sqrt((((xd[0]-od[10])*(xd[0]-od[10]))+((xd[1]-od[11])*(xd[1]-od[11])))));
a[38] = (a[37]*(real_t)(5.0000000000000000e-01));
a[39] = (((xd[0]-od[10])+(xd[0]-od[10]))*a[38]);
a[40] = ((real_t)(1.0000000000000000e+00)/a[5]);
a[41] = (a[40]*a[40]);
a[42] = (((xd[1]-od[11])+(xd[1]-od[11]))*a[38]);
a[43] = (1.0/sqrt((((xd[0]-od[12])*(xd[0]-od[12]))+((xd[1]-od[13])*(xd[1]-od[13])))));
a[44] = (a[43]*(real_t)(5.0000000000000000e-01));
a[45] = (((xd[0]-od[12])+(xd[0]-od[12]))*a[44]);
a[46] = ((real_t)(1.0000000000000000e+00)/a[6]);
a[47] = (a[46]*a[46]);
a[48] = (((xd[1]-od[13])+(xd[1]-od[13]))*a[44]);

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
out[9] = ((real_t)(1.0000000000000000e+00)/a[6]);
out[10] = u[1];
out[11] = u[0];
out[12] = (real_t)(1.0000000000000000e+00);
out[13] = (real_t)(0.0000000000000000e+00);
out[14] = (real_t)(0.0000000000000000e+00);
out[15] = (real_t)(0.0000000000000000e+00);
out[16] = (real_t)(0.0000000000000000e+00);
out[17] = (real_t)(1.0000000000000000e+00);
out[18] = (real_t)(0.0000000000000000e+00);
out[19] = (real_t)(0.0000000000000000e+00);
out[20] = (real_t)(0.0000000000000000e+00);
out[21] = (real_t)(0.0000000000000000e+00);
out[22] = (real_t)(1.0000000000000000e+00);
out[23] = (real_t)(0.0000000000000000e+00);
out[24] = ((real_t)(0.0000000000000000e+00)-(a[9]*a[11]));
out[25] = ((real_t)(0.0000000000000000e+00)-(a[12]*a[11]));
out[26] = (real_t)(0.0000000000000000e+00);
out[27] = (real_t)(0.0000000000000000e+00);
out[28] = ((real_t)(0.0000000000000000e+00)-(a[15]*a[17]));
out[29] = ((real_t)(0.0000000000000000e+00)-(a[18]*a[17]));
out[30] = (real_t)(0.0000000000000000e+00);
out[31] = (real_t)(0.0000000000000000e+00);
out[32] = ((real_t)(0.0000000000000000e+00)-(a[21]*a[23]));
out[33] = ((real_t)(0.0000000000000000e+00)-(a[24]*a[23]));
out[34] = (real_t)(0.0000000000000000e+00);
out[35] = (real_t)(0.0000000000000000e+00);
out[36] = ((real_t)(0.0000000000000000e+00)-(a[27]*a[29]));
out[37] = ((real_t)(0.0000000000000000e+00)-(a[30]*a[29]));
out[38] = (real_t)(0.0000000000000000e+00);
out[39] = (real_t)(0.0000000000000000e+00);
out[40] = ((real_t)(0.0000000000000000e+00)-(a[33]*a[35]));
out[41] = ((real_t)(0.0000000000000000e+00)-(a[36]*a[35]));
out[42] = (real_t)(0.0000000000000000e+00);
out[43] = (real_t)(0.0000000000000000e+00);
out[44] = ((real_t)(0.0000000000000000e+00)-(a[39]*a[41]));
out[45] = ((real_t)(0.0000000000000000e+00)-(a[42]*a[41]));
out[46] = (real_t)(0.0000000000000000e+00);
out[47] = (real_t)(0.0000000000000000e+00);
out[48] = ((real_t)(0.0000000000000000e+00)-(a[45]*a[47]));
out[49] = ((real_t)(0.0000000000000000e+00)-(a[48]*a[47]));
out[50] = (real_t)(0.0000000000000000e+00);
out[51] = (real_t)(0.0000000000000000e+00);
out[52] = (real_t)(0.0000000000000000e+00);
out[53] = (real_t)(0.0000000000000000e+00);
out[54] = (real_t)(0.0000000000000000e+00);
out[55] = (real_t)(0.0000000000000000e+00);
out[56] = (real_t)(0.0000000000000000e+00);
out[57] = (real_t)(0.0000000000000000e+00);
out[58] = (real_t)(0.0000000000000000e+00);
out[59] = (real_t)(0.0000000000000000e+00);
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
tmpQ2[0] = + tmpFx[0]*tmpObjS[0] + tmpFx[4]*tmpObjS[12] + tmpFx[8]*tmpObjS[24] + tmpFx[12]*tmpObjS[36] + tmpFx[16]*tmpObjS[48] + tmpFx[20]*tmpObjS[60] + tmpFx[24]*tmpObjS[72] + tmpFx[28]*tmpObjS[84] + tmpFx[32]*tmpObjS[96] + tmpFx[36]*tmpObjS[108] + tmpFx[40]*tmpObjS[120] + tmpFx[44]*tmpObjS[132];
tmpQ2[1] = + tmpFx[0]*tmpObjS[1] + tmpFx[4]*tmpObjS[13] + tmpFx[8]*tmpObjS[25] + tmpFx[12]*tmpObjS[37] + tmpFx[16]*tmpObjS[49] + tmpFx[20]*tmpObjS[61] + tmpFx[24]*tmpObjS[73] + tmpFx[28]*tmpObjS[85] + tmpFx[32]*tmpObjS[97] + tmpFx[36]*tmpObjS[109] + tmpFx[40]*tmpObjS[121] + tmpFx[44]*tmpObjS[133];
tmpQ2[2] = + tmpFx[0]*tmpObjS[2] + tmpFx[4]*tmpObjS[14] + tmpFx[8]*tmpObjS[26] + tmpFx[12]*tmpObjS[38] + tmpFx[16]*tmpObjS[50] + tmpFx[20]*tmpObjS[62] + tmpFx[24]*tmpObjS[74] + tmpFx[28]*tmpObjS[86] + tmpFx[32]*tmpObjS[98] + tmpFx[36]*tmpObjS[110] + tmpFx[40]*tmpObjS[122] + tmpFx[44]*tmpObjS[134];
tmpQ2[3] = + tmpFx[0]*tmpObjS[3] + tmpFx[4]*tmpObjS[15] + tmpFx[8]*tmpObjS[27] + tmpFx[12]*tmpObjS[39] + tmpFx[16]*tmpObjS[51] + tmpFx[20]*tmpObjS[63] + tmpFx[24]*tmpObjS[75] + tmpFx[28]*tmpObjS[87] + tmpFx[32]*tmpObjS[99] + tmpFx[36]*tmpObjS[111] + tmpFx[40]*tmpObjS[123] + tmpFx[44]*tmpObjS[135];
tmpQ2[4] = + tmpFx[0]*tmpObjS[4] + tmpFx[4]*tmpObjS[16] + tmpFx[8]*tmpObjS[28] + tmpFx[12]*tmpObjS[40] + tmpFx[16]*tmpObjS[52] + tmpFx[20]*tmpObjS[64] + tmpFx[24]*tmpObjS[76] + tmpFx[28]*tmpObjS[88] + tmpFx[32]*tmpObjS[100] + tmpFx[36]*tmpObjS[112] + tmpFx[40]*tmpObjS[124] + tmpFx[44]*tmpObjS[136];
tmpQ2[5] = + tmpFx[0]*tmpObjS[5] + tmpFx[4]*tmpObjS[17] + tmpFx[8]*tmpObjS[29] + tmpFx[12]*tmpObjS[41] + tmpFx[16]*tmpObjS[53] + tmpFx[20]*tmpObjS[65] + tmpFx[24]*tmpObjS[77] + tmpFx[28]*tmpObjS[89] + tmpFx[32]*tmpObjS[101] + tmpFx[36]*tmpObjS[113] + tmpFx[40]*tmpObjS[125] + tmpFx[44]*tmpObjS[137];
tmpQ2[6] = + tmpFx[0]*tmpObjS[6] + tmpFx[4]*tmpObjS[18] + tmpFx[8]*tmpObjS[30] + tmpFx[12]*tmpObjS[42] + tmpFx[16]*tmpObjS[54] + tmpFx[20]*tmpObjS[66] + tmpFx[24]*tmpObjS[78] + tmpFx[28]*tmpObjS[90] + tmpFx[32]*tmpObjS[102] + tmpFx[36]*tmpObjS[114] + tmpFx[40]*tmpObjS[126] + tmpFx[44]*tmpObjS[138];
tmpQ2[7] = + tmpFx[0]*tmpObjS[7] + tmpFx[4]*tmpObjS[19] + tmpFx[8]*tmpObjS[31] + tmpFx[12]*tmpObjS[43] + tmpFx[16]*tmpObjS[55] + tmpFx[20]*tmpObjS[67] + tmpFx[24]*tmpObjS[79] + tmpFx[28]*tmpObjS[91] + tmpFx[32]*tmpObjS[103] + tmpFx[36]*tmpObjS[115] + tmpFx[40]*tmpObjS[127] + tmpFx[44]*tmpObjS[139];
tmpQ2[8] = + tmpFx[0]*tmpObjS[8] + tmpFx[4]*tmpObjS[20] + tmpFx[8]*tmpObjS[32] + tmpFx[12]*tmpObjS[44] + tmpFx[16]*tmpObjS[56] + tmpFx[20]*tmpObjS[68] + tmpFx[24]*tmpObjS[80] + tmpFx[28]*tmpObjS[92] + tmpFx[32]*tmpObjS[104] + tmpFx[36]*tmpObjS[116] + tmpFx[40]*tmpObjS[128] + tmpFx[44]*tmpObjS[140];
tmpQ2[9] = + tmpFx[0]*tmpObjS[9] + tmpFx[4]*tmpObjS[21] + tmpFx[8]*tmpObjS[33] + tmpFx[12]*tmpObjS[45] + tmpFx[16]*tmpObjS[57] + tmpFx[20]*tmpObjS[69] + tmpFx[24]*tmpObjS[81] + tmpFx[28]*tmpObjS[93] + tmpFx[32]*tmpObjS[105] + tmpFx[36]*tmpObjS[117] + tmpFx[40]*tmpObjS[129] + tmpFx[44]*tmpObjS[141];
tmpQ2[10] = + tmpFx[0]*tmpObjS[10] + tmpFx[4]*tmpObjS[22] + tmpFx[8]*tmpObjS[34] + tmpFx[12]*tmpObjS[46] + tmpFx[16]*tmpObjS[58] + tmpFx[20]*tmpObjS[70] + tmpFx[24]*tmpObjS[82] + tmpFx[28]*tmpObjS[94] + tmpFx[32]*tmpObjS[106] + tmpFx[36]*tmpObjS[118] + tmpFx[40]*tmpObjS[130] + tmpFx[44]*tmpObjS[142];
tmpQ2[11] = + tmpFx[0]*tmpObjS[11] + tmpFx[4]*tmpObjS[23] + tmpFx[8]*tmpObjS[35] + tmpFx[12]*tmpObjS[47] + tmpFx[16]*tmpObjS[59] + tmpFx[20]*tmpObjS[71] + tmpFx[24]*tmpObjS[83] + tmpFx[28]*tmpObjS[95] + tmpFx[32]*tmpObjS[107] + tmpFx[36]*tmpObjS[119] + tmpFx[40]*tmpObjS[131] + tmpFx[44]*tmpObjS[143];
tmpQ2[12] = + tmpFx[1]*tmpObjS[0] + tmpFx[5]*tmpObjS[12] + tmpFx[9]*tmpObjS[24] + tmpFx[13]*tmpObjS[36] + tmpFx[17]*tmpObjS[48] + tmpFx[21]*tmpObjS[60] + tmpFx[25]*tmpObjS[72] + tmpFx[29]*tmpObjS[84] + tmpFx[33]*tmpObjS[96] + tmpFx[37]*tmpObjS[108] + tmpFx[41]*tmpObjS[120] + tmpFx[45]*tmpObjS[132];
tmpQ2[13] = + tmpFx[1]*tmpObjS[1] + tmpFx[5]*tmpObjS[13] + tmpFx[9]*tmpObjS[25] + tmpFx[13]*tmpObjS[37] + tmpFx[17]*tmpObjS[49] + tmpFx[21]*tmpObjS[61] + tmpFx[25]*tmpObjS[73] + tmpFx[29]*tmpObjS[85] + tmpFx[33]*tmpObjS[97] + tmpFx[37]*tmpObjS[109] + tmpFx[41]*tmpObjS[121] + tmpFx[45]*tmpObjS[133];
tmpQ2[14] = + tmpFx[1]*tmpObjS[2] + tmpFx[5]*tmpObjS[14] + tmpFx[9]*tmpObjS[26] + tmpFx[13]*tmpObjS[38] + tmpFx[17]*tmpObjS[50] + tmpFx[21]*tmpObjS[62] + tmpFx[25]*tmpObjS[74] + tmpFx[29]*tmpObjS[86] + tmpFx[33]*tmpObjS[98] + tmpFx[37]*tmpObjS[110] + tmpFx[41]*tmpObjS[122] + tmpFx[45]*tmpObjS[134];
tmpQ2[15] = + tmpFx[1]*tmpObjS[3] + tmpFx[5]*tmpObjS[15] + tmpFx[9]*tmpObjS[27] + tmpFx[13]*tmpObjS[39] + tmpFx[17]*tmpObjS[51] + tmpFx[21]*tmpObjS[63] + tmpFx[25]*tmpObjS[75] + tmpFx[29]*tmpObjS[87] + tmpFx[33]*tmpObjS[99] + tmpFx[37]*tmpObjS[111] + tmpFx[41]*tmpObjS[123] + tmpFx[45]*tmpObjS[135];
tmpQ2[16] = + tmpFx[1]*tmpObjS[4] + tmpFx[5]*tmpObjS[16] + tmpFx[9]*tmpObjS[28] + tmpFx[13]*tmpObjS[40] + tmpFx[17]*tmpObjS[52] + tmpFx[21]*tmpObjS[64] + tmpFx[25]*tmpObjS[76] + tmpFx[29]*tmpObjS[88] + tmpFx[33]*tmpObjS[100] + tmpFx[37]*tmpObjS[112] + tmpFx[41]*tmpObjS[124] + tmpFx[45]*tmpObjS[136];
tmpQ2[17] = + tmpFx[1]*tmpObjS[5] + tmpFx[5]*tmpObjS[17] + tmpFx[9]*tmpObjS[29] + tmpFx[13]*tmpObjS[41] + tmpFx[17]*tmpObjS[53] + tmpFx[21]*tmpObjS[65] + tmpFx[25]*tmpObjS[77] + tmpFx[29]*tmpObjS[89] + tmpFx[33]*tmpObjS[101] + tmpFx[37]*tmpObjS[113] + tmpFx[41]*tmpObjS[125] + tmpFx[45]*tmpObjS[137];
tmpQ2[18] = + tmpFx[1]*tmpObjS[6] + tmpFx[5]*tmpObjS[18] + tmpFx[9]*tmpObjS[30] + tmpFx[13]*tmpObjS[42] + tmpFx[17]*tmpObjS[54] + tmpFx[21]*tmpObjS[66] + tmpFx[25]*tmpObjS[78] + tmpFx[29]*tmpObjS[90] + tmpFx[33]*tmpObjS[102] + tmpFx[37]*tmpObjS[114] + tmpFx[41]*tmpObjS[126] + tmpFx[45]*tmpObjS[138];
tmpQ2[19] = + tmpFx[1]*tmpObjS[7] + tmpFx[5]*tmpObjS[19] + tmpFx[9]*tmpObjS[31] + tmpFx[13]*tmpObjS[43] + tmpFx[17]*tmpObjS[55] + tmpFx[21]*tmpObjS[67] + tmpFx[25]*tmpObjS[79] + tmpFx[29]*tmpObjS[91] + tmpFx[33]*tmpObjS[103] + tmpFx[37]*tmpObjS[115] + tmpFx[41]*tmpObjS[127] + tmpFx[45]*tmpObjS[139];
tmpQ2[20] = + tmpFx[1]*tmpObjS[8] + tmpFx[5]*tmpObjS[20] + tmpFx[9]*tmpObjS[32] + tmpFx[13]*tmpObjS[44] + tmpFx[17]*tmpObjS[56] + tmpFx[21]*tmpObjS[68] + tmpFx[25]*tmpObjS[80] + tmpFx[29]*tmpObjS[92] + tmpFx[33]*tmpObjS[104] + tmpFx[37]*tmpObjS[116] + tmpFx[41]*tmpObjS[128] + tmpFx[45]*tmpObjS[140];
tmpQ2[21] = + tmpFx[1]*tmpObjS[9] + tmpFx[5]*tmpObjS[21] + tmpFx[9]*tmpObjS[33] + tmpFx[13]*tmpObjS[45] + tmpFx[17]*tmpObjS[57] + tmpFx[21]*tmpObjS[69] + tmpFx[25]*tmpObjS[81] + tmpFx[29]*tmpObjS[93] + tmpFx[33]*tmpObjS[105] + tmpFx[37]*tmpObjS[117] + tmpFx[41]*tmpObjS[129] + tmpFx[45]*tmpObjS[141];
tmpQ2[22] = + tmpFx[1]*tmpObjS[10] + tmpFx[5]*tmpObjS[22] + tmpFx[9]*tmpObjS[34] + tmpFx[13]*tmpObjS[46] + tmpFx[17]*tmpObjS[58] + tmpFx[21]*tmpObjS[70] + tmpFx[25]*tmpObjS[82] + tmpFx[29]*tmpObjS[94] + tmpFx[33]*tmpObjS[106] + tmpFx[37]*tmpObjS[118] + tmpFx[41]*tmpObjS[130] + tmpFx[45]*tmpObjS[142];
tmpQ2[23] = + tmpFx[1]*tmpObjS[11] + tmpFx[5]*tmpObjS[23] + tmpFx[9]*tmpObjS[35] + tmpFx[13]*tmpObjS[47] + tmpFx[17]*tmpObjS[59] + tmpFx[21]*tmpObjS[71] + tmpFx[25]*tmpObjS[83] + tmpFx[29]*tmpObjS[95] + tmpFx[33]*tmpObjS[107] + tmpFx[37]*tmpObjS[119] + tmpFx[41]*tmpObjS[131] + tmpFx[45]*tmpObjS[143];
tmpQ2[24] = + tmpFx[2]*tmpObjS[0] + tmpFx[6]*tmpObjS[12] + tmpFx[10]*tmpObjS[24] + tmpFx[14]*tmpObjS[36] + tmpFx[18]*tmpObjS[48] + tmpFx[22]*tmpObjS[60] + tmpFx[26]*tmpObjS[72] + tmpFx[30]*tmpObjS[84] + tmpFx[34]*tmpObjS[96] + tmpFx[38]*tmpObjS[108] + tmpFx[42]*tmpObjS[120] + tmpFx[46]*tmpObjS[132];
tmpQ2[25] = + tmpFx[2]*tmpObjS[1] + tmpFx[6]*tmpObjS[13] + tmpFx[10]*tmpObjS[25] + tmpFx[14]*tmpObjS[37] + tmpFx[18]*tmpObjS[49] + tmpFx[22]*tmpObjS[61] + tmpFx[26]*tmpObjS[73] + tmpFx[30]*tmpObjS[85] + tmpFx[34]*tmpObjS[97] + tmpFx[38]*tmpObjS[109] + tmpFx[42]*tmpObjS[121] + tmpFx[46]*tmpObjS[133];
tmpQ2[26] = + tmpFx[2]*tmpObjS[2] + tmpFx[6]*tmpObjS[14] + tmpFx[10]*tmpObjS[26] + tmpFx[14]*tmpObjS[38] + tmpFx[18]*tmpObjS[50] + tmpFx[22]*tmpObjS[62] + tmpFx[26]*tmpObjS[74] + tmpFx[30]*tmpObjS[86] + tmpFx[34]*tmpObjS[98] + tmpFx[38]*tmpObjS[110] + tmpFx[42]*tmpObjS[122] + tmpFx[46]*tmpObjS[134];
tmpQ2[27] = + tmpFx[2]*tmpObjS[3] + tmpFx[6]*tmpObjS[15] + tmpFx[10]*tmpObjS[27] + tmpFx[14]*tmpObjS[39] + tmpFx[18]*tmpObjS[51] + tmpFx[22]*tmpObjS[63] + tmpFx[26]*tmpObjS[75] + tmpFx[30]*tmpObjS[87] + tmpFx[34]*tmpObjS[99] + tmpFx[38]*tmpObjS[111] + tmpFx[42]*tmpObjS[123] + tmpFx[46]*tmpObjS[135];
tmpQ2[28] = + tmpFx[2]*tmpObjS[4] + tmpFx[6]*tmpObjS[16] + tmpFx[10]*tmpObjS[28] + tmpFx[14]*tmpObjS[40] + tmpFx[18]*tmpObjS[52] + tmpFx[22]*tmpObjS[64] + tmpFx[26]*tmpObjS[76] + tmpFx[30]*tmpObjS[88] + tmpFx[34]*tmpObjS[100] + tmpFx[38]*tmpObjS[112] + tmpFx[42]*tmpObjS[124] + tmpFx[46]*tmpObjS[136];
tmpQ2[29] = + tmpFx[2]*tmpObjS[5] + tmpFx[6]*tmpObjS[17] + tmpFx[10]*tmpObjS[29] + tmpFx[14]*tmpObjS[41] + tmpFx[18]*tmpObjS[53] + tmpFx[22]*tmpObjS[65] + tmpFx[26]*tmpObjS[77] + tmpFx[30]*tmpObjS[89] + tmpFx[34]*tmpObjS[101] + tmpFx[38]*tmpObjS[113] + tmpFx[42]*tmpObjS[125] + tmpFx[46]*tmpObjS[137];
tmpQ2[30] = + tmpFx[2]*tmpObjS[6] + tmpFx[6]*tmpObjS[18] + tmpFx[10]*tmpObjS[30] + tmpFx[14]*tmpObjS[42] + tmpFx[18]*tmpObjS[54] + tmpFx[22]*tmpObjS[66] + tmpFx[26]*tmpObjS[78] + tmpFx[30]*tmpObjS[90] + tmpFx[34]*tmpObjS[102] + tmpFx[38]*tmpObjS[114] + tmpFx[42]*tmpObjS[126] + tmpFx[46]*tmpObjS[138];
tmpQ2[31] = + tmpFx[2]*tmpObjS[7] + tmpFx[6]*tmpObjS[19] + tmpFx[10]*tmpObjS[31] + tmpFx[14]*tmpObjS[43] + tmpFx[18]*tmpObjS[55] + tmpFx[22]*tmpObjS[67] + tmpFx[26]*tmpObjS[79] + tmpFx[30]*tmpObjS[91] + tmpFx[34]*tmpObjS[103] + tmpFx[38]*tmpObjS[115] + tmpFx[42]*tmpObjS[127] + tmpFx[46]*tmpObjS[139];
tmpQ2[32] = + tmpFx[2]*tmpObjS[8] + tmpFx[6]*tmpObjS[20] + tmpFx[10]*tmpObjS[32] + tmpFx[14]*tmpObjS[44] + tmpFx[18]*tmpObjS[56] + tmpFx[22]*tmpObjS[68] + tmpFx[26]*tmpObjS[80] + tmpFx[30]*tmpObjS[92] + tmpFx[34]*tmpObjS[104] + tmpFx[38]*tmpObjS[116] + tmpFx[42]*tmpObjS[128] + tmpFx[46]*tmpObjS[140];
tmpQ2[33] = + tmpFx[2]*tmpObjS[9] + tmpFx[6]*tmpObjS[21] + tmpFx[10]*tmpObjS[33] + tmpFx[14]*tmpObjS[45] + tmpFx[18]*tmpObjS[57] + tmpFx[22]*tmpObjS[69] + tmpFx[26]*tmpObjS[81] + tmpFx[30]*tmpObjS[93] + tmpFx[34]*tmpObjS[105] + tmpFx[38]*tmpObjS[117] + tmpFx[42]*tmpObjS[129] + tmpFx[46]*tmpObjS[141];
tmpQ2[34] = + tmpFx[2]*tmpObjS[10] + tmpFx[6]*tmpObjS[22] + tmpFx[10]*tmpObjS[34] + tmpFx[14]*tmpObjS[46] + tmpFx[18]*tmpObjS[58] + tmpFx[22]*tmpObjS[70] + tmpFx[26]*tmpObjS[82] + tmpFx[30]*tmpObjS[94] + tmpFx[34]*tmpObjS[106] + tmpFx[38]*tmpObjS[118] + tmpFx[42]*tmpObjS[130] + tmpFx[46]*tmpObjS[142];
tmpQ2[35] = + tmpFx[2]*tmpObjS[11] + tmpFx[6]*tmpObjS[23] + tmpFx[10]*tmpObjS[35] + tmpFx[14]*tmpObjS[47] + tmpFx[18]*tmpObjS[59] + tmpFx[22]*tmpObjS[71] + tmpFx[26]*tmpObjS[83] + tmpFx[30]*tmpObjS[95] + tmpFx[34]*tmpObjS[107] + tmpFx[38]*tmpObjS[119] + tmpFx[42]*tmpObjS[131] + tmpFx[46]*tmpObjS[143];
tmpQ2[36] = + tmpFx[3]*tmpObjS[0] + tmpFx[7]*tmpObjS[12] + tmpFx[11]*tmpObjS[24] + tmpFx[15]*tmpObjS[36] + tmpFx[19]*tmpObjS[48] + tmpFx[23]*tmpObjS[60] + tmpFx[27]*tmpObjS[72] + tmpFx[31]*tmpObjS[84] + tmpFx[35]*tmpObjS[96] + tmpFx[39]*tmpObjS[108] + tmpFx[43]*tmpObjS[120] + tmpFx[47]*tmpObjS[132];
tmpQ2[37] = + tmpFx[3]*tmpObjS[1] + tmpFx[7]*tmpObjS[13] + tmpFx[11]*tmpObjS[25] + tmpFx[15]*tmpObjS[37] + tmpFx[19]*tmpObjS[49] + tmpFx[23]*tmpObjS[61] + tmpFx[27]*tmpObjS[73] + tmpFx[31]*tmpObjS[85] + tmpFx[35]*tmpObjS[97] + tmpFx[39]*tmpObjS[109] + tmpFx[43]*tmpObjS[121] + tmpFx[47]*tmpObjS[133];
tmpQ2[38] = + tmpFx[3]*tmpObjS[2] + tmpFx[7]*tmpObjS[14] + tmpFx[11]*tmpObjS[26] + tmpFx[15]*tmpObjS[38] + tmpFx[19]*tmpObjS[50] + tmpFx[23]*tmpObjS[62] + tmpFx[27]*tmpObjS[74] + tmpFx[31]*tmpObjS[86] + tmpFx[35]*tmpObjS[98] + tmpFx[39]*tmpObjS[110] + tmpFx[43]*tmpObjS[122] + tmpFx[47]*tmpObjS[134];
tmpQ2[39] = + tmpFx[3]*tmpObjS[3] + tmpFx[7]*tmpObjS[15] + tmpFx[11]*tmpObjS[27] + tmpFx[15]*tmpObjS[39] + tmpFx[19]*tmpObjS[51] + tmpFx[23]*tmpObjS[63] + tmpFx[27]*tmpObjS[75] + tmpFx[31]*tmpObjS[87] + tmpFx[35]*tmpObjS[99] + tmpFx[39]*tmpObjS[111] + tmpFx[43]*tmpObjS[123] + tmpFx[47]*tmpObjS[135];
tmpQ2[40] = + tmpFx[3]*tmpObjS[4] + tmpFx[7]*tmpObjS[16] + tmpFx[11]*tmpObjS[28] + tmpFx[15]*tmpObjS[40] + tmpFx[19]*tmpObjS[52] + tmpFx[23]*tmpObjS[64] + tmpFx[27]*tmpObjS[76] + tmpFx[31]*tmpObjS[88] + tmpFx[35]*tmpObjS[100] + tmpFx[39]*tmpObjS[112] + tmpFx[43]*tmpObjS[124] + tmpFx[47]*tmpObjS[136];
tmpQ2[41] = + tmpFx[3]*tmpObjS[5] + tmpFx[7]*tmpObjS[17] + tmpFx[11]*tmpObjS[29] + tmpFx[15]*tmpObjS[41] + tmpFx[19]*tmpObjS[53] + tmpFx[23]*tmpObjS[65] + tmpFx[27]*tmpObjS[77] + tmpFx[31]*tmpObjS[89] + tmpFx[35]*tmpObjS[101] + tmpFx[39]*tmpObjS[113] + tmpFx[43]*tmpObjS[125] + tmpFx[47]*tmpObjS[137];
tmpQ2[42] = + tmpFx[3]*tmpObjS[6] + tmpFx[7]*tmpObjS[18] + tmpFx[11]*tmpObjS[30] + tmpFx[15]*tmpObjS[42] + tmpFx[19]*tmpObjS[54] + tmpFx[23]*tmpObjS[66] + tmpFx[27]*tmpObjS[78] + tmpFx[31]*tmpObjS[90] + tmpFx[35]*tmpObjS[102] + tmpFx[39]*tmpObjS[114] + tmpFx[43]*tmpObjS[126] + tmpFx[47]*tmpObjS[138];
tmpQ2[43] = + tmpFx[3]*tmpObjS[7] + tmpFx[7]*tmpObjS[19] + tmpFx[11]*tmpObjS[31] + tmpFx[15]*tmpObjS[43] + tmpFx[19]*tmpObjS[55] + tmpFx[23]*tmpObjS[67] + tmpFx[27]*tmpObjS[79] + tmpFx[31]*tmpObjS[91] + tmpFx[35]*tmpObjS[103] + tmpFx[39]*tmpObjS[115] + tmpFx[43]*tmpObjS[127] + tmpFx[47]*tmpObjS[139];
tmpQ2[44] = + tmpFx[3]*tmpObjS[8] + tmpFx[7]*tmpObjS[20] + tmpFx[11]*tmpObjS[32] + tmpFx[15]*tmpObjS[44] + tmpFx[19]*tmpObjS[56] + tmpFx[23]*tmpObjS[68] + tmpFx[27]*tmpObjS[80] + tmpFx[31]*tmpObjS[92] + tmpFx[35]*tmpObjS[104] + tmpFx[39]*tmpObjS[116] + tmpFx[43]*tmpObjS[128] + tmpFx[47]*tmpObjS[140];
tmpQ2[45] = + tmpFx[3]*tmpObjS[9] + tmpFx[7]*tmpObjS[21] + tmpFx[11]*tmpObjS[33] + tmpFx[15]*tmpObjS[45] + tmpFx[19]*tmpObjS[57] + tmpFx[23]*tmpObjS[69] + tmpFx[27]*tmpObjS[81] + tmpFx[31]*tmpObjS[93] + tmpFx[35]*tmpObjS[105] + tmpFx[39]*tmpObjS[117] + tmpFx[43]*tmpObjS[129] + tmpFx[47]*tmpObjS[141];
tmpQ2[46] = + tmpFx[3]*tmpObjS[10] + tmpFx[7]*tmpObjS[22] + tmpFx[11]*tmpObjS[34] + tmpFx[15]*tmpObjS[46] + tmpFx[19]*tmpObjS[58] + tmpFx[23]*tmpObjS[70] + tmpFx[27]*tmpObjS[82] + tmpFx[31]*tmpObjS[94] + tmpFx[35]*tmpObjS[106] + tmpFx[39]*tmpObjS[118] + tmpFx[43]*tmpObjS[130] + tmpFx[47]*tmpObjS[142];
tmpQ2[47] = + tmpFx[3]*tmpObjS[11] + tmpFx[7]*tmpObjS[23] + tmpFx[11]*tmpObjS[35] + tmpFx[15]*tmpObjS[47] + tmpFx[19]*tmpObjS[59] + tmpFx[23]*tmpObjS[71] + tmpFx[27]*tmpObjS[83] + tmpFx[31]*tmpObjS[95] + tmpFx[35]*tmpObjS[107] + tmpFx[39]*tmpObjS[119] + tmpFx[43]*tmpObjS[131] + tmpFx[47]*tmpObjS[143];
tmpQ1[0] = + tmpQ2[0]*tmpFx[0] + tmpQ2[1]*tmpFx[4] + tmpQ2[2]*tmpFx[8] + tmpQ2[3]*tmpFx[12] + tmpQ2[4]*tmpFx[16] + tmpQ2[5]*tmpFx[20] + tmpQ2[6]*tmpFx[24] + tmpQ2[7]*tmpFx[28] + tmpQ2[8]*tmpFx[32] + tmpQ2[9]*tmpFx[36] + tmpQ2[10]*tmpFx[40] + tmpQ2[11]*tmpFx[44];
tmpQ1[1] = + tmpQ2[0]*tmpFx[1] + tmpQ2[1]*tmpFx[5] + tmpQ2[2]*tmpFx[9] + tmpQ2[3]*tmpFx[13] + tmpQ2[4]*tmpFx[17] + tmpQ2[5]*tmpFx[21] + tmpQ2[6]*tmpFx[25] + tmpQ2[7]*tmpFx[29] + tmpQ2[8]*tmpFx[33] + tmpQ2[9]*tmpFx[37] + tmpQ2[10]*tmpFx[41] + tmpQ2[11]*tmpFx[45];
tmpQ1[2] = + tmpQ2[0]*tmpFx[2] + tmpQ2[1]*tmpFx[6] + tmpQ2[2]*tmpFx[10] + tmpQ2[3]*tmpFx[14] + tmpQ2[4]*tmpFx[18] + tmpQ2[5]*tmpFx[22] + tmpQ2[6]*tmpFx[26] + tmpQ2[7]*tmpFx[30] + tmpQ2[8]*tmpFx[34] + tmpQ2[9]*tmpFx[38] + tmpQ2[10]*tmpFx[42] + tmpQ2[11]*tmpFx[46];
tmpQ1[3] = + tmpQ2[0]*tmpFx[3] + tmpQ2[1]*tmpFx[7] + tmpQ2[2]*tmpFx[11] + tmpQ2[3]*tmpFx[15] + tmpQ2[4]*tmpFx[19] + tmpQ2[5]*tmpFx[23] + tmpQ2[6]*tmpFx[27] + tmpQ2[7]*tmpFx[31] + tmpQ2[8]*tmpFx[35] + tmpQ2[9]*tmpFx[39] + tmpQ2[10]*tmpFx[43] + tmpQ2[11]*tmpFx[47];
tmpQ1[4] = + tmpQ2[12]*tmpFx[0] + tmpQ2[13]*tmpFx[4] + tmpQ2[14]*tmpFx[8] + tmpQ2[15]*tmpFx[12] + tmpQ2[16]*tmpFx[16] + tmpQ2[17]*tmpFx[20] + tmpQ2[18]*tmpFx[24] + tmpQ2[19]*tmpFx[28] + tmpQ2[20]*tmpFx[32] + tmpQ2[21]*tmpFx[36] + tmpQ2[22]*tmpFx[40] + tmpQ2[23]*tmpFx[44];
tmpQ1[5] = + tmpQ2[12]*tmpFx[1] + tmpQ2[13]*tmpFx[5] + tmpQ2[14]*tmpFx[9] + tmpQ2[15]*tmpFx[13] + tmpQ2[16]*tmpFx[17] + tmpQ2[17]*tmpFx[21] + tmpQ2[18]*tmpFx[25] + tmpQ2[19]*tmpFx[29] + tmpQ2[20]*tmpFx[33] + tmpQ2[21]*tmpFx[37] + tmpQ2[22]*tmpFx[41] + tmpQ2[23]*tmpFx[45];
tmpQ1[6] = + tmpQ2[12]*tmpFx[2] + tmpQ2[13]*tmpFx[6] + tmpQ2[14]*tmpFx[10] + tmpQ2[15]*tmpFx[14] + tmpQ2[16]*tmpFx[18] + tmpQ2[17]*tmpFx[22] + tmpQ2[18]*tmpFx[26] + tmpQ2[19]*tmpFx[30] + tmpQ2[20]*tmpFx[34] + tmpQ2[21]*tmpFx[38] + tmpQ2[22]*tmpFx[42] + tmpQ2[23]*tmpFx[46];
tmpQ1[7] = + tmpQ2[12]*tmpFx[3] + tmpQ2[13]*tmpFx[7] + tmpQ2[14]*tmpFx[11] + tmpQ2[15]*tmpFx[15] + tmpQ2[16]*tmpFx[19] + tmpQ2[17]*tmpFx[23] + tmpQ2[18]*tmpFx[27] + tmpQ2[19]*tmpFx[31] + tmpQ2[20]*tmpFx[35] + tmpQ2[21]*tmpFx[39] + tmpQ2[22]*tmpFx[43] + tmpQ2[23]*tmpFx[47];
tmpQ1[8] = + tmpQ2[24]*tmpFx[0] + tmpQ2[25]*tmpFx[4] + tmpQ2[26]*tmpFx[8] + tmpQ2[27]*tmpFx[12] + tmpQ2[28]*tmpFx[16] + tmpQ2[29]*tmpFx[20] + tmpQ2[30]*tmpFx[24] + tmpQ2[31]*tmpFx[28] + tmpQ2[32]*tmpFx[32] + tmpQ2[33]*tmpFx[36] + tmpQ2[34]*tmpFx[40] + tmpQ2[35]*tmpFx[44];
tmpQ1[9] = + tmpQ2[24]*tmpFx[1] + tmpQ2[25]*tmpFx[5] + tmpQ2[26]*tmpFx[9] + tmpQ2[27]*tmpFx[13] + tmpQ2[28]*tmpFx[17] + tmpQ2[29]*tmpFx[21] + tmpQ2[30]*tmpFx[25] + tmpQ2[31]*tmpFx[29] + tmpQ2[32]*tmpFx[33] + tmpQ2[33]*tmpFx[37] + tmpQ2[34]*tmpFx[41] + tmpQ2[35]*tmpFx[45];
tmpQ1[10] = + tmpQ2[24]*tmpFx[2] + tmpQ2[25]*tmpFx[6] + tmpQ2[26]*tmpFx[10] + tmpQ2[27]*tmpFx[14] + tmpQ2[28]*tmpFx[18] + tmpQ2[29]*tmpFx[22] + tmpQ2[30]*tmpFx[26] + tmpQ2[31]*tmpFx[30] + tmpQ2[32]*tmpFx[34] + tmpQ2[33]*tmpFx[38] + tmpQ2[34]*tmpFx[42] + tmpQ2[35]*tmpFx[46];
tmpQ1[11] = + tmpQ2[24]*tmpFx[3] + tmpQ2[25]*tmpFx[7] + tmpQ2[26]*tmpFx[11] + tmpQ2[27]*tmpFx[15] + tmpQ2[28]*tmpFx[19] + tmpQ2[29]*tmpFx[23] + tmpQ2[30]*tmpFx[27] + tmpQ2[31]*tmpFx[31] + tmpQ2[32]*tmpFx[35] + tmpQ2[33]*tmpFx[39] + tmpQ2[34]*tmpFx[43] + tmpQ2[35]*tmpFx[47];
tmpQ1[12] = + tmpQ2[36]*tmpFx[0] + tmpQ2[37]*tmpFx[4] + tmpQ2[38]*tmpFx[8] + tmpQ2[39]*tmpFx[12] + tmpQ2[40]*tmpFx[16] + tmpQ2[41]*tmpFx[20] + tmpQ2[42]*tmpFx[24] + tmpQ2[43]*tmpFx[28] + tmpQ2[44]*tmpFx[32] + tmpQ2[45]*tmpFx[36] + tmpQ2[46]*tmpFx[40] + tmpQ2[47]*tmpFx[44];
tmpQ1[13] = + tmpQ2[36]*tmpFx[1] + tmpQ2[37]*tmpFx[5] + tmpQ2[38]*tmpFx[9] + tmpQ2[39]*tmpFx[13] + tmpQ2[40]*tmpFx[17] + tmpQ2[41]*tmpFx[21] + tmpQ2[42]*tmpFx[25] + tmpQ2[43]*tmpFx[29] + tmpQ2[44]*tmpFx[33] + tmpQ2[45]*tmpFx[37] + tmpQ2[46]*tmpFx[41] + tmpQ2[47]*tmpFx[45];
tmpQ1[14] = + tmpQ2[36]*tmpFx[2] + tmpQ2[37]*tmpFx[6] + tmpQ2[38]*tmpFx[10] + tmpQ2[39]*tmpFx[14] + tmpQ2[40]*tmpFx[18] + tmpQ2[41]*tmpFx[22] + tmpQ2[42]*tmpFx[26] + tmpQ2[43]*tmpFx[30] + tmpQ2[44]*tmpFx[34] + tmpQ2[45]*tmpFx[38] + tmpQ2[46]*tmpFx[42] + tmpQ2[47]*tmpFx[46];
tmpQ1[15] = + tmpQ2[36]*tmpFx[3] + tmpQ2[37]*tmpFx[7] + tmpQ2[38]*tmpFx[11] + tmpQ2[39]*tmpFx[15] + tmpQ2[40]*tmpFx[19] + tmpQ2[41]*tmpFx[23] + tmpQ2[42]*tmpFx[27] + tmpQ2[43]*tmpFx[31] + tmpQ2[44]*tmpFx[35] + tmpQ2[45]*tmpFx[39] + tmpQ2[46]*tmpFx[43] + tmpQ2[47]*tmpFx[47];
}

void acado_setObjR1R2( real_t* const tmpObjS, real_t* const tmpR1, real_t* const tmpR2 )
{
tmpR2[0] = +tmpObjS[132];
tmpR2[1] = +tmpObjS[133];
tmpR2[2] = +tmpObjS[134];
tmpR2[3] = +tmpObjS[135];
tmpR2[4] = +tmpObjS[136];
tmpR2[5] = +tmpObjS[137];
tmpR2[6] = +tmpObjS[138];
tmpR2[7] = +tmpObjS[139];
tmpR2[8] = +tmpObjS[140];
tmpR2[9] = +tmpObjS[141];
tmpR2[10] = +tmpObjS[142];
tmpR2[11] = +tmpObjS[143];
tmpR2[12] = +tmpObjS[120];
tmpR2[13] = +tmpObjS[121];
tmpR2[14] = +tmpObjS[122];
tmpR2[15] = +tmpObjS[123];
tmpR2[16] = +tmpObjS[124];
tmpR2[17] = +tmpObjS[125];
tmpR2[18] = +tmpObjS[126];
tmpR2[19] = +tmpObjS[127];
tmpR2[20] = +tmpObjS[128];
tmpR2[21] = +tmpObjS[129];
tmpR2[22] = +tmpObjS[130];
tmpR2[23] = +tmpObjS[131];
tmpR1[0] = + tmpR2[11];
tmpR1[1] = + tmpR2[10];
tmpR1[2] = + tmpR2[23];
tmpR1[3] = + tmpR2[22];
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
for (runObj = 0; runObj < 30; ++runObj)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[runObj * 4];
acadoWorkspace.objValueIn[1] = acadoVariables.x[runObj * 4 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[runObj * 4 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.x[runObj * 4 + 3];
acadoWorkspace.objValueIn[4] = acadoVariables.u[runObj * 2];
acadoWorkspace.objValueIn[5] = acadoVariables.u[runObj * 2 + 1];
acadoWorkspace.objValueIn[6] = acadoVariables.od[runObj * 15];
acadoWorkspace.objValueIn[7] = acadoVariables.od[runObj * 15 + 1];
acadoWorkspace.objValueIn[8] = acadoVariables.od[runObj * 15 + 2];
acadoWorkspace.objValueIn[9] = acadoVariables.od[runObj * 15 + 3];
acadoWorkspace.objValueIn[10] = acadoVariables.od[runObj * 15 + 4];
acadoWorkspace.objValueIn[11] = acadoVariables.od[runObj * 15 + 5];
acadoWorkspace.objValueIn[12] = acadoVariables.od[runObj * 15 + 6];
acadoWorkspace.objValueIn[13] = acadoVariables.od[runObj * 15 + 7];
acadoWorkspace.objValueIn[14] = acadoVariables.od[runObj * 15 + 8];
acadoWorkspace.objValueIn[15] = acadoVariables.od[runObj * 15 + 9];
acadoWorkspace.objValueIn[16] = acadoVariables.od[runObj * 15 + 10];
acadoWorkspace.objValueIn[17] = acadoVariables.od[runObj * 15 + 11];
acadoWorkspace.objValueIn[18] = acadoVariables.od[runObj * 15 + 12];
acadoWorkspace.objValueIn[19] = acadoVariables.od[runObj * 15 + 13];
acadoWorkspace.objValueIn[20] = acadoVariables.od[runObj * 15 + 14];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[runObj * 12] = acadoWorkspace.objValueOut[0];
acadoWorkspace.Dy[runObj * 12 + 1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.Dy[runObj * 12 + 2] = acadoWorkspace.objValueOut[2];
acadoWorkspace.Dy[runObj * 12 + 3] = acadoWorkspace.objValueOut[3];
acadoWorkspace.Dy[runObj * 12 + 4] = acadoWorkspace.objValueOut[4];
acadoWorkspace.Dy[runObj * 12 + 5] = acadoWorkspace.objValueOut[5];
acadoWorkspace.Dy[runObj * 12 + 6] = acadoWorkspace.objValueOut[6];
acadoWorkspace.Dy[runObj * 12 + 7] = acadoWorkspace.objValueOut[7];
acadoWorkspace.Dy[runObj * 12 + 8] = acadoWorkspace.objValueOut[8];
acadoWorkspace.Dy[runObj * 12 + 9] = acadoWorkspace.objValueOut[9];
acadoWorkspace.Dy[runObj * 12 + 10] = acadoWorkspace.objValueOut[10];
acadoWorkspace.Dy[runObj * 12 + 11] = acadoWorkspace.objValueOut[11];

acado_setObjQ1Q2( &(acadoWorkspace.objValueOut[ 12 ]), acadoVariables.W, &(acadoWorkspace.Q1[ runObj * 16 ]), &(acadoWorkspace.Q2[ runObj * 48 ]) );

acado_setObjR1R2( acadoVariables.W, &(acadoWorkspace.R1[ runObj * 4 ]), &(acadoWorkspace.R2[ runObj * 24 ]) );

}
acadoWorkspace.objValueIn[0] = acadoVariables.x[120];
acadoWorkspace.objValueIn[1] = acadoVariables.x[121];
acadoWorkspace.objValueIn[2] = acadoVariables.x[122];
acadoWorkspace.objValueIn[3] = acadoVariables.x[123];
acadoWorkspace.objValueIn[4] = acadoVariables.od[450];
acadoWorkspace.objValueIn[5] = acadoVariables.od[451];
acadoWorkspace.objValueIn[6] = acadoVariables.od[452];
acadoWorkspace.objValueIn[7] = acadoVariables.od[453];
acadoWorkspace.objValueIn[8] = acadoVariables.od[454];
acadoWorkspace.objValueIn[9] = acadoVariables.od[455];
acadoWorkspace.objValueIn[10] = acadoVariables.od[456];
acadoWorkspace.objValueIn[11] = acadoVariables.od[457];
acadoWorkspace.objValueIn[12] = acadoVariables.od[458];
acadoWorkspace.objValueIn[13] = acadoVariables.od[459];
acadoWorkspace.objValueIn[14] = acadoVariables.od[460];
acadoWorkspace.objValueIn[15] = acadoVariables.od[461];
acadoWorkspace.objValueIn[16] = acadoVariables.od[462];
acadoWorkspace.objValueIn[17] = acadoVariables.od[463];
acadoWorkspace.objValueIn[18] = acadoVariables.od[464];
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
acadoWorkspace.H[(iRow * 120) + (iCol * 2)] += + Gu1[0]*Gu2[0] + Gu1[2]*Gu2[2] + Gu1[4]*Gu2[4] + Gu1[6]*Gu2[6];
acadoWorkspace.H[(iRow * 120) + (iCol * 2 + 1)] += + Gu1[0]*Gu2[1] + Gu1[2]*Gu2[3] + Gu1[4]*Gu2[5] + Gu1[6]*Gu2[7];
acadoWorkspace.H[(iRow * 120 + 60) + (iCol * 2)] += + Gu1[1]*Gu2[0] + Gu1[3]*Gu2[2] + Gu1[5]*Gu2[4] + Gu1[7]*Gu2[6];
acadoWorkspace.H[(iRow * 120 + 60) + (iCol * 2 + 1)] += + Gu1[1]*Gu2[1] + Gu1[3]*Gu2[3] + Gu1[5]*Gu2[5] + Gu1[7]*Gu2[7];
}

void acado_setBlockH11_R1( int iRow, int iCol, real_t* const R11 )
{
acadoWorkspace.H[(iRow * 120) + (iCol * 2)] = R11[0] + (real_t)1.0000000000000000e-08;
acadoWorkspace.H[(iRow * 120) + (iCol * 2 + 1)] = R11[1];
acadoWorkspace.H[(iRow * 120 + 60) + (iCol * 2)] = R11[2];
acadoWorkspace.H[(iRow * 120 + 60) + (iCol * 2 + 1)] = R11[3] + (real_t)1.0000000000000000e-08;
}

void acado_zeroBlockH11( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 120) + (iCol * 2)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 120) + (iCol * 2 + 1)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 120 + 60) + (iCol * 2)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 120 + 60) + (iCol * 2 + 1)] = 0.0000000000000000e+00;
}

void acado_copyHTH( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 120) + (iCol * 2)] = acadoWorkspace.H[(iCol * 120) + (iRow * 2)];
acadoWorkspace.H[(iRow * 120) + (iCol * 2 + 1)] = acadoWorkspace.H[(iCol * 120 + 60) + (iRow * 2)];
acadoWorkspace.H[(iRow * 120 + 60) + (iCol * 2)] = acadoWorkspace.H[(iCol * 120) + (iRow * 2 + 1)];
acadoWorkspace.H[(iRow * 120 + 60) + (iCol * 2 + 1)] = acadoWorkspace.H[(iCol * 120 + 60) + (iRow * 2 + 1)];
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

void acado_multRDy( real_t* const R2, real_t* const Dy1, real_t* const RDy1 )
{
RDy1[0] = + R2[0]*Dy1[0] + R2[1]*Dy1[1] + R2[2]*Dy1[2] + R2[3]*Dy1[3] + R2[4]*Dy1[4] + R2[5]*Dy1[5] + R2[6]*Dy1[6] + R2[7]*Dy1[7] + R2[8]*Dy1[8] + R2[9]*Dy1[9] + R2[10]*Dy1[10] + R2[11]*Dy1[11];
RDy1[1] = + R2[12]*Dy1[0] + R2[13]*Dy1[1] + R2[14]*Dy1[2] + R2[15]*Dy1[3] + R2[16]*Dy1[4] + R2[17]*Dy1[5] + R2[18]*Dy1[6] + R2[19]*Dy1[7] + R2[20]*Dy1[8] + R2[21]*Dy1[9] + R2[22]*Dy1[10] + R2[23]*Dy1[11];
}

void acado_multQDy( real_t* const Q2, real_t* const Dy1, real_t* const QDy1 )
{
QDy1[0] = + Q2[0]*Dy1[0] + Q2[1]*Dy1[1] + Q2[2]*Dy1[2] + Q2[3]*Dy1[3] + Q2[4]*Dy1[4] + Q2[5]*Dy1[5] + Q2[6]*Dy1[6] + Q2[7]*Dy1[7] + Q2[8]*Dy1[8] + Q2[9]*Dy1[9] + Q2[10]*Dy1[10] + Q2[11]*Dy1[11];
QDy1[1] = + Q2[12]*Dy1[0] + Q2[13]*Dy1[1] + Q2[14]*Dy1[2] + Q2[15]*Dy1[3] + Q2[16]*Dy1[4] + Q2[17]*Dy1[5] + Q2[18]*Dy1[6] + Q2[19]*Dy1[7] + Q2[20]*Dy1[8] + Q2[21]*Dy1[9] + Q2[22]*Dy1[10] + Q2[23]*Dy1[11];
QDy1[2] = + Q2[24]*Dy1[0] + Q2[25]*Dy1[1] + Q2[26]*Dy1[2] + Q2[27]*Dy1[3] + Q2[28]*Dy1[4] + Q2[29]*Dy1[5] + Q2[30]*Dy1[6] + Q2[31]*Dy1[7] + Q2[32]*Dy1[8] + Q2[33]*Dy1[9] + Q2[34]*Dy1[10] + Q2[35]*Dy1[11];
QDy1[3] = + Q2[36]*Dy1[0] + Q2[37]*Dy1[1] + Q2[38]*Dy1[2] + Q2[39]*Dy1[3] + Q2[40]*Dy1[4] + Q2[41]*Dy1[5] + Q2[42]*Dy1[6] + Q2[43]*Dy1[7] + Q2[44]*Dy1[8] + Q2[45]*Dy1[9] + Q2[46]*Dy1[10] + Q2[47]*Dy1[11];
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
for (lRun1 = 1; lRun1 < 30; ++lRun1)
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

for (lRun1 = 0; lRun1 < 29; ++lRun1)
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

for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
acado_zeroBlockH10( &(acadoWorkspace.H10[ lRun1 * 8 ]) );
for (lRun2 = lRun1; lRun2 < 30; ++lRun2)
{
lRun3 = (((lRun2 + 1) * (lRun2)) / (2)) + (lRun1);
acado_multQETGx( &(acadoWorkspace.QE[ lRun3 * 8 ]), &(acadoWorkspace.evGx[ lRun2 * 16 ]), &(acadoWorkspace.H10[ lRun1 * 8 ]) );
}
}

for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
acado_setBlockH11_R1( lRun1, lRun1, &(acadoWorkspace.R1[ lRun1 * 4 ]) );
lRun2 = lRun1;
for (lRun3 = lRun1; lRun3 < 30; ++lRun3)
{
lRun4 = (((lRun3 + 1) * (lRun3)) / (2)) + (lRun1);
lRun5 = (((lRun3 + 1) * (lRun3)) / (2)) + (lRun2);
acado_setBlockH11( lRun1, lRun2, &(acadoWorkspace.E[ lRun4 * 8 ]), &(acadoWorkspace.QE[ lRun5 * 8 ]) );
}
for (lRun2 = lRun1 + 1; lRun2 < 30; ++lRun2)
{
acado_zeroBlockH11( lRun1, lRun2 );
for (lRun3 = lRun2; lRun3 < 30; ++lRun3)
{
lRun4 = (((lRun3 + 1) * (lRun3)) / (2)) + (lRun1);
lRun5 = (((lRun3 + 1) * (lRun3)) / (2)) + (lRun2);
acado_setBlockH11( lRun1, lRun2, &(acadoWorkspace.E[ lRun4 * 8 ]), &(acadoWorkspace.QE[ lRun5 * 8 ]) );
}
}
}

for (lRun1 = 0; lRun1 < 30; ++lRun1)
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
acado_multQN1d( acadoWorkspace.QN1, &(acadoWorkspace.d[ 116 ]), &(acadoWorkspace.Qd[ 116 ]) );

for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
for (lRun2 = lRun1; lRun2 < 30; ++lRun2)
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

for (lRun2 = 0; lRun2 < 360; ++lRun2)
acadoWorkspace.Dy[lRun2] -= acadoVariables.y[lRun2];

acadoWorkspace.DyN[0] -= acadoVariables.yN[0];
acadoWorkspace.DyN[1] -= acadoVariables.yN[1];
acadoWorkspace.DyN[2] -= acadoVariables.yN[2];

acado_multRDy( acadoWorkspace.R2, acadoWorkspace.Dy, acadoWorkspace.g );
acado_multRDy( &(acadoWorkspace.R2[ 24 ]), &(acadoWorkspace.Dy[ 12 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 48 ]), &(acadoWorkspace.Dy[ 24 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 72 ]), &(acadoWorkspace.Dy[ 36 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 96 ]), &(acadoWorkspace.Dy[ 48 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 120 ]), &(acadoWorkspace.Dy[ 60 ]), &(acadoWorkspace.g[ 10 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 144 ]), &(acadoWorkspace.Dy[ 72 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 168 ]), &(acadoWorkspace.Dy[ 84 ]), &(acadoWorkspace.g[ 14 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 192 ]), &(acadoWorkspace.Dy[ 96 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 216 ]), &(acadoWorkspace.Dy[ 108 ]), &(acadoWorkspace.g[ 18 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 240 ]), &(acadoWorkspace.Dy[ 120 ]), &(acadoWorkspace.g[ 20 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 264 ]), &(acadoWorkspace.Dy[ 132 ]), &(acadoWorkspace.g[ 22 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 288 ]), &(acadoWorkspace.Dy[ 144 ]), &(acadoWorkspace.g[ 24 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 312 ]), &(acadoWorkspace.Dy[ 156 ]), &(acadoWorkspace.g[ 26 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 336 ]), &(acadoWorkspace.Dy[ 168 ]), &(acadoWorkspace.g[ 28 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 360 ]), &(acadoWorkspace.Dy[ 180 ]), &(acadoWorkspace.g[ 30 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 384 ]), &(acadoWorkspace.Dy[ 192 ]), &(acadoWorkspace.g[ 32 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 408 ]), &(acadoWorkspace.Dy[ 204 ]), &(acadoWorkspace.g[ 34 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 432 ]), &(acadoWorkspace.Dy[ 216 ]), &(acadoWorkspace.g[ 36 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 456 ]), &(acadoWorkspace.Dy[ 228 ]), &(acadoWorkspace.g[ 38 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 480 ]), &(acadoWorkspace.Dy[ 240 ]), &(acadoWorkspace.g[ 40 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 504 ]), &(acadoWorkspace.Dy[ 252 ]), &(acadoWorkspace.g[ 42 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 528 ]), &(acadoWorkspace.Dy[ 264 ]), &(acadoWorkspace.g[ 44 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 552 ]), &(acadoWorkspace.Dy[ 276 ]), &(acadoWorkspace.g[ 46 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 576 ]), &(acadoWorkspace.Dy[ 288 ]), &(acadoWorkspace.g[ 48 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 600 ]), &(acadoWorkspace.Dy[ 300 ]), &(acadoWorkspace.g[ 50 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 624 ]), &(acadoWorkspace.Dy[ 312 ]), &(acadoWorkspace.g[ 52 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 648 ]), &(acadoWorkspace.Dy[ 324 ]), &(acadoWorkspace.g[ 54 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 672 ]), &(acadoWorkspace.Dy[ 336 ]), &(acadoWorkspace.g[ 56 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 696 ]), &(acadoWorkspace.Dy[ 348 ]), &(acadoWorkspace.g[ 58 ]) );

acado_multQDy( acadoWorkspace.Q2, acadoWorkspace.Dy, acadoWorkspace.QDy );
acado_multQDy( &(acadoWorkspace.Q2[ 48 ]), &(acadoWorkspace.Dy[ 12 ]), &(acadoWorkspace.QDy[ 4 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 96 ]), &(acadoWorkspace.Dy[ 24 ]), &(acadoWorkspace.QDy[ 8 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 144 ]), &(acadoWorkspace.Dy[ 36 ]), &(acadoWorkspace.QDy[ 12 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 192 ]), &(acadoWorkspace.Dy[ 48 ]), &(acadoWorkspace.QDy[ 16 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 240 ]), &(acadoWorkspace.Dy[ 60 ]), &(acadoWorkspace.QDy[ 20 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 288 ]), &(acadoWorkspace.Dy[ 72 ]), &(acadoWorkspace.QDy[ 24 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 336 ]), &(acadoWorkspace.Dy[ 84 ]), &(acadoWorkspace.QDy[ 28 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 384 ]), &(acadoWorkspace.Dy[ 96 ]), &(acadoWorkspace.QDy[ 32 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 432 ]), &(acadoWorkspace.Dy[ 108 ]), &(acadoWorkspace.QDy[ 36 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 480 ]), &(acadoWorkspace.Dy[ 120 ]), &(acadoWorkspace.QDy[ 40 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 528 ]), &(acadoWorkspace.Dy[ 132 ]), &(acadoWorkspace.QDy[ 44 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 576 ]), &(acadoWorkspace.Dy[ 144 ]), &(acadoWorkspace.QDy[ 48 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 624 ]), &(acadoWorkspace.Dy[ 156 ]), &(acadoWorkspace.QDy[ 52 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 672 ]), &(acadoWorkspace.Dy[ 168 ]), &(acadoWorkspace.QDy[ 56 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 720 ]), &(acadoWorkspace.Dy[ 180 ]), &(acadoWorkspace.QDy[ 60 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 768 ]), &(acadoWorkspace.Dy[ 192 ]), &(acadoWorkspace.QDy[ 64 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 816 ]), &(acadoWorkspace.Dy[ 204 ]), &(acadoWorkspace.QDy[ 68 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 864 ]), &(acadoWorkspace.Dy[ 216 ]), &(acadoWorkspace.QDy[ 72 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 912 ]), &(acadoWorkspace.Dy[ 228 ]), &(acadoWorkspace.QDy[ 76 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 960 ]), &(acadoWorkspace.Dy[ 240 ]), &(acadoWorkspace.QDy[ 80 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1008 ]), &(acadoWorkspace.Dy[ 252 ]), &(acadoWorkspace.QDy[ 84 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1056 ]), &(acadoWorkspace.Dy[ 264 ]), &(acadoWorkspace.QDy[ 88 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1104 ]), &(acadoWorkspace.Dy[ 276 ]), &(acadoWorkspace.QDy[ 92 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1152 ]), &(acadoWorkspace.Dy[ 288 ]), &(acadoWorkspace.QDy[ 96 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1200 ]), &(acadoWorkspace.Dy[ 300 ]), &(acadoWorkspace.QDy[ 100 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1248 ]), &(acadoWorkspace.Dy[ 312 ]), &(acadoWorkspace.QDy[ 104 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1296 ]), &(acadoWorkspace.Dy[ 324 ]), &(acadoWorkspace.QDy[ 108 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1344 ]), &(acadoWorkspace.Dy[ 336 ]), &(acadoWorkspace.QDy[ 112 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1392 ]), &(acadoWorkspace.Dy[ 348 ]), &(acadoWorkspace.QDy[ 116 ]) );

acadoWorkspace.QDy[120] = + acadoWorkspace.QN2[0]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[1]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[2]*acadoWorkspace.DyN[2];
acadoWorkspace.QDy[121] = + acadoWorkspace.QN2[3]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[4]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[5]*acadoWorkspace.DyN[2];
acadoWorkspace.QDy[122] = + acadoWorkspace.QN2[6]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[7]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[8]*acadoWorkspace.DyN[2];
acadoWorkspace.QDy[123] = + acadoWorkspace.QN2[9]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[10]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[11]*acadoWorkspace.DyN[2];

acadoWorkspace.QDy[4] += acadoWorkspace.Qd[0];
acadoWorkspace.QDy[5] += acadoWorkspace.Qd[1];
acadoWorkspace.QDy[6] += acadoWorkspace.Qd[2];
acadoWorkspace.QDy[7] += acadoWorkspace.Qd[3];
acadoWorkspace.QDy[8] += acadoWorkspace.Qd[4];
acadoWorkspace.QDy[9] += acadoWorkspace.Qd[5];
acadoWorkspace.QDy[10] += acadoWorkspace.Qd[6];
acadoWorkspace.QDy[11] += acadoWorkspace.Qd[7];
acadoWorkspace.QDy[12] += acadoWorkspace.Qd[8];
acadoWorkspace.QDy[13] += acadoWorkspace.Qd[9];
acadoWorkspace.QDy[14] += acadoWorkspace.Qd[10];
acadoWorkspace.QDy[15] += acadoWorkspace.Qd[11];
acadoWorkspace.QDy[16] += acadoWorkspace.Qd[12];
acadoWorkspace.QDy[17] += acadoWorkspace.Qd[13];
acadoWorkspace.QDy[18] += acadoWorkspace.Qd[14];
acadoWorkspace.QDy[19] += acadoWorkspace.Qd[15];
acadoWorkspace.QDy[20] += acadoWorkspace.Qd[16];
acadoWorkspace.QDy[21] += acadoWorkspace.Qd[17];
acadoWorkspace.QDy[22] += acadoWorkspace.Qd[18];
acadoWorkspace.QDy[23] += acadoWorkspace.Qd[19];
acadoWorkspace.QDy[24] += acadoWorkspace.Qd[20];
acadoWorkspace.QDy[25] += acadoWorkspace.Qd[21];
acadoWorkspace.QDy[26] += acadoWorkspace.Qd[22];
acadoWorkspace.QDy[27] += acadoWorkspace.Qd[23];
acadoWorkspace.QDy[28] += acadoWorkspace.Qd[24];
acadoWorkspace.QDy[29] += acadoWorkspace.Qd[25];
acadoWorkspace.QDy[30] += acadoWorkspace.Qd[26];
acadoWorkspace.QDy[31] += acadoWorkspace.Qd[27];
acadoWorkspace.QDy[32] += acadoWorkspace.Qd[28];
acadoWorkspace.QDy[33] += acadoWorkspace.Qd[29];
acadoWorkspace.QDy[34] += acadoWorkspace.Qd[30];
acadoWorkspace.QDy[35] += acadoWorkspace.Qd[31];
acadoWorkspace.QDy[36] += acadoWorkspace.Qd[32];
acadoWorkspace.QDy[37] += acadoWorkspace.Qd[33];
acadoWorkspace.QDy[38] += acadoWorkspace.Qd[34];
acadoWorkspace.QDy[39] += acadoWorkspace.Qd[35];
acadoWorkspace.QDy[40] += acadoWorkspace.Qd[36];
acadoWorkspace.QDy[41] += acadoWorkspace.Qd[37];
acadoWorkspace.QDy[42] += acadoWorkspace.Qd[38];
acadoWorkspace.QDy[43] += acadoWorkspace.Qd[39];
acadoWorkspace.QDy[44] += acadoWorkspace.Qd[40];
acadoWorkspace.QDy[45] += acadoWorkspace.Qd[41];
acadoWorkspace.QDy[46] += acadoWorkspace.Qd[42];
acadoWorkspace.QDy[47] += acadoWorkspace.Qd[43];
acadoWorkspace.QDy[48] += acadoWorkspace.Qd[44];
acadoWorkspace.QDy[49] += acadoWorkspace.Qd[45];
acadoWorkspace.QDy[50] += acadoWorkspace.Qd[46];
acadoWorkspace.QDy[51] += acadoWorkspace.Qd[47];
acadoWorkspace.QDy[52] += acadoWorkspace.Qd[48];
acadoWorkspace.QDy[53] += acadoWorkspace.Qd[49];
acadoWorkspace.QDy[54] += acadoWorkspace.Qd[50];
acadoWorkspace.QDy[55] += acadoWorkspace.Qd[51];
acadoWorkspace.QDy[56] += acadoWorkspace.Qd[52];
acadoWorkspace.QDy[57] += acadoWorkspace.Qd[53];
acadoWorkspace.QDy[58] += acadoWorkspace.Qd[54];
acadoWorkspace.QDy[59] += acadoWorkspace.Qd[55];
acadoWorkspace.QDy[60] += acadoWorkspace.Qd[56];
acadoWorkspace.QDy[61] += acadoWorkspace.Qd[57];
acadoWorkspace.QDy[62] += acadoWorkspace.Qd[58];
acadoWorkspace.QDy[63] += acadoWorkspace.Qd[59];
acadoWorkspace.QDy[64] += acadoWorkspace.Qd[60];
acadoWorkspace.QDy[65] += acadoWorkspace.Qd[61];
acadoWorkspace.QDy[66] += acadoWorkspace.Qd[62];
acadoWorkspace.QDy[67] += acadoWorkspace.Qd[63];
acadoWorkspace.QDy[68] += acadoWorkspace.Qd[64];
acadoWorkspace.QDy[69] += acadoWorkspace.Qd[65];
acadoWorkspace.QDy[70] += acadoWorkspace.Qd[66];
acadoWorkspace.QDy[71] += acadoWorkspace.Qd[67];
acadoWorkspace.QDy[72] += acadoWorkspace.Qd[68];
acadoWorkspace.QDy[73] += acadoWorkspace.Qd[69];
acadoWorkspace.QDy[74] += acadoWorkspace.Qd[70];
acadoWorkspace.QDy[75] += acadoWorkspace.Qd[71];
acadoWorkspace.QDy[76] += acadoWorkspace.Qd[72];
acadoWorkspace.QDy[77] += acadoWorkspace.Qd[73];
acadoWorkspace.QDy[78] += acadoWorkspace.Qd[74];
acadoWorkspace.QDy[79] += acadoWorkspace.Qd[75];
acadoWorkspace.QDy[80] += acadoWorkspace.Qd[76];
acadoWorkspace.QDy[81] += acadoWorkspace.Qd[77];
acadoWorkspace.QDy[82] += acadoWorkspace.Qd[78];
acadoWorkspace.QDy[83] += acadoWorkspace.Qd[79];
acadoWorkspace.QDy[84] += acadoWorkspace.Qd[80];
acadoWorkspace.QDy[85] += acadoWorkspace.Qd[81];
acadoWorkspace.QDy[86] += acadoWorkspace.Qd[82];
acadoWorkspace.QDy[87] += acadoWorkspace.Qd[83];
acadoWorkspace.QDy[88] += acadoWorkspace.Qd[84];
acadoWorkspace.QDy[89] += acadoWorkspace.Qd[85];
acadoWorkspace.QDy[90] += acadoWorkspace.Qd[86];
acadoWorkspace.QDy[91] += acadoWorkspace.Qd[87];
acadoWorkspace.QDy[92] += acadoWorkspace.Qd[88];
acadoWorkspace.QDy[93] += acadoWorkspace.Qd[89];
acadoWorkspace.QDy[94] += acadoWorkspace.Qd[90];
acadoWorkspace.QDy[95] += acadoWorkspace.Qd[91];
acadoWorkspace.QDy[96] += acadoWorkspace.Qd[92];
acadoWorkspace.QDy[97] += acadoWorkspace.Qd[93];
acadoWorkspace.QDy[98] += acadoWorkspace.Qd[94];
acadoWorkspace.QDy[99] += acadoWorkspace.Qd[95];
acadoWorkspace.QDy[100] += acadoWorkspace.Qd[96];
acadoWorkspace.QDy[101] += acadoWorkspace.Qd[97];
acadoWorkspace.QDy[102] += acadoWorkspace.Qd[98];
acadoWorkspace.QDy[103] += acadoWorkspace.Qd[99];
acadoWorkspace.QDy[104] += acadoWorkspace.Qd[100];
acadoWorkspace.QDy[105] += acadoWorkspace.Qd[101];
acadoWorkspace.QDy[106] += acadoWorkspace.Qd[102];
acadoWorkspace.QDy[107] += acadoWorkspace.Qd[103];
acadoWorkspace.QDy[108] += acadoWorkspace.Qd[104];
acadoWorkspace.QDy[109] += acadoWorkspace.Qd[105];
acadoWorkspace.QDy[110] += acadoWorkspace.Qd[106];
acadoWorkspace.QDy[111] += acadoWorkspace.Qd[107];
acadoWorkspace.QDy[112] += acadoWorkspace.Qd[108];
acadoWorkspace.QDy[113] += acadoWorkspace.Qd[109];
acadoWorkspace.QDy[114] += acadoWorkspace.Qd[110];
acadoWorkspace.QDy[115] += acadoWorkspace.Qd[111];
acadoWorkspace.QDy[116] += acadoWorkspace.Qd[112];
acadoWorkspace.QDy[117] += acadoWorkspace.Qd[113];
acadoWorkspace.QDy[118] += acadoWorkspace.Qd[114];
acadoWorkspace.QDy[119] += acadoWorkspace.Qd[115];
acadoWorkspace.QDy[120] += acadoWorkspace.Qd[116];
acadoWorkspace.QDy[121] += acadoWorkspace.Qd[117];
acadoWorkspace.QDy[122] += acadoWorkspace.Qd[118];
acadoWorkspace.QDy[123] += acadoWorkspace.Qd[119];

for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
for (lRun2 = lRun1; lRun2 < 30; ++lRun2)
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

for (lRun1 = 0; lRun1 < 30; ++lRun1)
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
acadoVariables.lbValues[1] = -5.9999999999999998e-01;
acadoVariables.lbValues[2] = -5.0000000000000000e-01;
acadoVariables.lbValues[3] = -5.9999999999999998e-01;
acadoVariables.lbValues[4] = -5.0000000000000000e-01;
acadoVariables.lbValues[5] = -5.9999999999999998e-01;
acadoVariables.lbValues[6] = -5.0000000000000000e-01;
acadoVariables.lbValues[7] = -5.9999999999999998e-01;
acadoVariables.lbValues[8] = -5.0000000000000000e-01;
acadoVariables.lbValues[9] = -5.9999999999999998e-01;
acadoVariables.lbValues[10] = -5.0000000000000000e-01;
acadoVariables.lbValues[11] = -5.9999999999999998e-01;
acadoVariables.lbValues[12] = -5.0000000000000000e-01;
acadoVariables.lbValues[13] = -5.9999999999999998e-01;
acadoVariables.lbValues[14] = -5.0000000000000000e-01;
acadoVariables.lbValues[15] = -5.9999999999999998e-01;
acadoVariables.lbValues[16] = -5.0000000000000000e-01;
acadoVariables.lbValues[17] = -5.9999999999999998e-01;
acadoVariables.lbValues[18] = -5.0000000000000000e-01;
acadoVariables.lbValues[19] = -5.9999999999999998e-01;
acadoVariables.lbValues[20] = -5.0000000000000000e-01;
acadoVariables.lbValues[21] = -5.9999999999999998e-01;
acadoVariables.lbValues[22] = -5.0000000000000000e-01;
acadoVariables.lbValues[23] = -5.9999999999999998e-01;
acadoVariables.lbValues[24] = -5.0000000000000000e-01;
acadoVariables.lbValues[25] = -5.9999999999999998e-01;
acadoVariables.lbValues[26] = -5.0000000000000000e-01;
acadoVariables.lbValues[27] = -5.9999999999999998e-01;
acadoVariables.lbValues[28] = -5.0000000000000000e-01;
acadoVariables.lbValues[29] = -5.9999999999999998e-01;
acadoVariables.lbValues[30] = -5.0000000000000000e-01;
acadoVariables.lbValues[31] = -5.9999999999999998e-01;
acadoVariables.lbValues[32] = -5.0000000000000000e-01;
acadoVariables.lbValues[33] = -5.9999999999999998e-01;
acadoVariables.lbValues[34] = -5.0000000000000000e-01;
acadoVariables.lbValues[35] = -5.9999999999999998e-01;
acadoVariables.lbValues[36] = -5.0000000000000000e-01;
acadoVariables.lbValues[37] = -5.9999999999999998e-01;
acadoVariables.lbValues[38] = -5.0000000000000000e-01;
acadoVariables.lbValues[39] = -5.9999999999999998e-01;
acadoVariables.lbValues[40] = -5.0000000000000000e-01;
acadoVariables.lbValues[41] = -5.9999999999999998e-01;
acadoVariables.lbValues[42] = -5.0000000000000000e-01;
acadoVariables.lbValues[43] = -5.9999999999999998e-01;
acadoVariables.lbValues[44] = -5.0000000000000000e-01;
acadoVariables.lbValues[45] = -5.9999999999999998e-01;
acadoVariables.lbValues[46] = -5.0000000000000000e-01;
acadoVariables.lbValues[47] = -5.9999999999999998e-01;
acadoVariables.lbValues[48] = -5.0000000000000000e-01;
acadoVariables.lbValues[49] = -5.9999999999999998e-01;
acadoVariables.lbValues[50] = -5.0000000000000000e-01;
acadoVariables.lbValues[51] = -5.9999999999999998e-01;
acadoVariables.lbValues[52] = -5.0000000000000000e-01;
acadoVariables.lbValues[53] = -5.9999999999999998e-01;
acadoVariables.lbValues[54] = -5.0000000000000000e-01;
acadoVariables.lbValues[55] = -5.9999999999999998e-01;
acadoVariables.lbValues[56] = -5.0000000000000000e-01;
acadoVariables.lbValues[57] = -5.9999999999999998e-01;
acadoVariables.lbValues[58] = -5.0000000000000000e-01;
acadoVariables.lbValues[59] = -5.9999999999999998e-01;
acadoVariables.ubValues[0] = 5.0000000000000000e-01;
acadoVariables.ubValues[1] = 5.9999999999999998e-01;
acadoVariables.ubValues[2] = 5.0000000000000000e-01;
acadoVariables.ubValues[3] = 5.9999999999999998e-01;
acadoVariables.ubValues[4] = 5.0000000000000000e-01;
acadoVariables.ubValues[5] = 5.9999999999999998e-01;
acadoVariables.ubValues[6] = 5.0000000000000000e-01;
acadoVariables.ubValues[7] = 5.9999999999999998e-01;
acadoVariables.ubValues[8] = 5.0000000000000000e-01;
acadoVariables.ubValues[9] = 5.9999999999999998e-01;
acadoVariables.ubValues[10] = 5.0000000000000000e-01;
acadoVariables.ubValues[11] = 5.9999999999999998e-01;
acadoVariables.ubValues[12] = 5.0000000000000000e-01;
acadoVariables.ubValues[13] = 5.9999999999999998e-01;
acadoVariables.ubValues[14] = 5.0000000000000000e-01;
acadoVariables.ubValues[15] = 5.9999999999999998e-01;
acadoVariables.ubValues[16] = 5.0000000000000000e-01;
acadoVariables.ubValues[17] = 5.9999999999999998e-01;
acadoVariables.ubValues[18] = 5.0000000000000000e-01;
acadoVariables.ubValues[19] = 5.9999999999999998e-01;
acadoVariables.ubValues[20] = 5.0000000000000000e-01;
acadoVariables.ubValues[21] = 5.9999999999999998e-01;
acadoVariables.ubValues[22] = 5.0000000000000000e-01;
acadoVariables.ubValues[23] = 5.9999999999999998e-01;
acadoVariables.ubValues[24] = 5.0000000000000000e-01;
acadoVariables.ubValues[25] = 5.9999999999999998e-01;
acadoVariables.ubValues[26] = 5.0000000000000000e-01;
acadoVariables.ubValues[27] = 5.9999999999999998e-01;
acadoVariables.ubValues[28] = 5.0000000000000000e-01;
acadoVariables.ubValues[29] = 5.9999999999999998e-01;
acadoVariables.ubValues[30] = 5.0000000000000000e-01;
acadoVariables.ubValues[31] = 5.9999999999999998e-01;
acadoVariables.ubValues[32] = 5.0000000000000000e-01;
acadoVariables.ubValues[33] = 5.9999999999999998e-01;
acadoVariables.ubValues[34] = 5.0000000000000000e-01;
acadoVariables.ubValues[35] = 5.9999999999999998e-01;
acadoVariables.ubValues[36] = 5.0000000000000000e-01;
acadoVariables.ubValues[37] = 5.9999999999999998e-01;
acadoVariables.ubValues[38] = 5.0000000000000000e-01;
acadoVariables.ubValues[39] = 5.9999999999999998e-01;
acadoVariables.ubValues[40] = 5.0000000000000000e-01;
acadoVariables.ubValues[41] = 5.9999999999999998e-01;
acadoVariables.ubValues[42] = 5.0000000000000000e-01;
acadoVariables.ubValues[43] = 5.9999999999999998e-01;
acadoVariables.ubValues[44] = 5.0000000000000000e-01;
acadoVariables.ubValues[45] = 5.9999999999999998e-01;
acadoVariables.ubValues[46] = 5.0000000000000000e-01;
acadoVariables.ubValues[47] = 5.9999999999999998e-01;
acadoVariables.ubValues[48] = 5.0000000000000000e-01;
acadoVariables.ubValues[49] = 5.9999999999999998e-01;
acadoVariables.ubValues[50] = 5.0000000000000000e-01;
acadoVariables.ubValues[51] = 5.9999999999999998e-01;
acadoVariables.ubValues[52] = 5.0000000000000000e-01;
acadoVariables.ubValues[53] = 5.9999999999999998e-01;
acadoVariables.ubValues[54] = 5.0000000000000000e-01;
acadoVariables.ubValues[55] = 5.9999999999999998e-01;
acadoVariables.ubValues[56] = 5.0000000000000000e-01;
acadoVariables.ubValues[57] = 5.9999999999999998e-01;
acadoVariables.ubValues[58] = 5.0000000000000000e-01;
acadoVariables.ubValues[59] = 5.9999999999999998e-01;
return ret;
}

void acado_initializeNodesByForwardSimulation(  )
{
int index;
for (index = 0; index < 30; ++index)
{
acadoWorkspace.state[0] = acadoVariables.x[index * 4];
acadoWorkspace.state[1] = acadoVariables.x[index * 4 + 1];
acadoWorkspace.state[2] = acadoVariables.x[index * 4 + 2];
acadoWorkspace.state[3] = acadoVariables.x[index * 4 + 3];
acadoWorkspace.state[28] = acadoVariables.u[index * 2];
acadoWorkspace.state[29] = acadoVariables.u[index * 2 + 1];
acadoWorkspace.state[30] = acadoVariables.od[index * 15];
acadoWorkspace.state[31] = acadoVariables.od[index * 15 + 1];
acadoWorkspace.state[32] = acadoVariables.od[index * 15 + 2];
acadoWorkspace.state[33] = acadoVariables.od[index * 15 + 3];
acadoWorkspace.state[34] = acadoVariables.od[index * 15 + 4];
acadoWorkspace.state[35] = acadoVariables.od[index * 15 + 5];
acadoWorkspace.state[36] = acadoVariables.od[index * 15 + 6];
acadoWorkspace.state[37] = acadoVariables.od[index * 15 + 7];
acadoWorkspace.state[38] = acadoVariables.od[index * 15 + 8];
acadoWorkspace.state[39] = acadoVariables.od[index * 15 + 9];
acadoWorkspace.state[40] = acadoVariables.od[index * 15 + 10];
acadoWorkspace.state[41] = acadoVariables.od[index * 15 + 11];
acadoWorkspace.state[42] = acadoVariables.od[index * 15 + 12];
acadoWorkspace.state[43] = acadoVariables.od[index * 15 + 13];
acadoWorkspace.state[44] = acadoVariables.od[index * 15 + 14];

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
for (index = 0; index < 30; ++index)
{
acadoVariables.x[index * 4] = acadoVariables.x[index * 4 + 4];
acadoVariables.x[index * 4 + 1] = acadoVariables.x[index * 4 + 5];
acadoVariables.x[index * 4 + 2] = acadoVariables.x[index * 4 + 6];
acadoVariables.x[index * 4 + 3] = acadoVariables.x[index * 4 + 7];
}

if (strategy == 1 && xEnd != 0)
{
acadoVariables.x[120] = xEnd[0];
acadoVariables.x[121] = xEnd[1];
acadoVariables.x[122] = xEnd[2];
acadoVariables.x[123] = xEnd[3];
}
else if (strategy == 2) 
{
acadoWorkspace.state[0] = acadoVariables.x[120];
acadoWorkspace.state[1] = acadoVariables.x[121];
acadoWorkspace.state[2] = acadoVariables.x[122];
acadoWorkspace.state[3] = acadoVariables.x[123];
if (uEnd != 0)
{
acadoWorkspace.state[28] = uEnd[0];
acadoWorkspace.state[29] = uEnd[1];
}
else
{
acadoWorkspace.state[28] = acadoVariables.u[58];
acadoWorkspace.state[29] = acadoVariables.u[59];
}
acadoWorkspace.state[30] = acadoVariables.od[450];
acadoWorkspace.state[31] = acadoVariables.od[451];
acadoWorkspace.state[32] = acadoVariables.od[452];
acadoWorkspace.state[33] = acadoVariables.od[453];
acadoWorkspace.state[34] = acadoVariables.od[454];
acadoWorkspace.state[35] = acadoVariables.od[455];
acadoWorkspace.state[36] = acadoVariables.od[456];
acadoWorkspace.state[37] = acadoVariables.od[457];
acadoWorkspace.state[38] = acadoVariables.od[458];
acadoWorkspace.state[39] = acadoVariables.od[459];
acadoWorkspace.state[40] = acadoVariables.od[460];
acadoWorkspace.state[41] = acadoVariables.od[461];
acadoWorkspace.state[42] = acadoVariables.od[462];
acadoWorkspace.state[43] = acadoVariables.od[463];
acadoWorkspace.state[44] = acadoVariables.od[464];

acado_integrate(acadoWorkspace.state, 1);

acadoVariables.x[120] = acadoWorkspace.state[0];
acadoVariables.x[121] = acadoWorkspace.state[1];
acadoVariables.x[122] = acadoWorkspace.state[2];
acadoVariables.x[123] = acadoWorkspace.state[3];
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
return kkt;
}

real_t acado_getObjective(  )
{
real_t objVal;

int lRun1;
/** Row vector of size: 12 */
real_t tmpDy[ 12 ];

/** Row vector of size: 3 */
real_t tmpDyN[ 3 ];

for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[lRun1 * 4];
acadoWorkspace.objValueIn[1] = acadoVariables.x[lRun1 * 4 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[lRun1 * 4 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.x[lRun1 * 4 + 3];
acadoWorkspace.objValueIn[4] = acadoVariables.u[lRun1 * 2];
acadoWorkspace.objValueIn[5] = acadoVariables.u[lRun1 * 2 + 1];
acadoWorkspace.objValueIn[6] = acadoVariables.od[lRun1 * 15];
acadoWorkspace.objValueIn[7] = acadoVariables.od[lRun1 * 15 + 1];
acadoWorkspace.objValueIn[8] = acadoVariables.od[lRun1 * 15 + 2];
acadoWorkspace.objValueIn[9] = acadoVariables.od[lRun1 * 15 + 3];
acadoWorkspace.objValueIn[10] = acadoVariables.od[lRun1 * 15 + 4];
acadoWorkspace.objValueIn[11] = acadoVariables.od[lRun1 * 15 + 5];
acadoWorkspace.objValueIn[12] = acadoVariables.od[lRun1 * 15 + 6];
acadoWorkspace.objValueIn[13] = acadoVariables.od[lRun1 * 15 + 7];
acadoWorkspace.objValueIn[14] = acadoVariables.od[lRun1 * 15 + 8];
acadoWorkspace.objValueIn[15] = acadoVariables.od[lRun1 * 15 + 9];
acadoWorkspace.objValueIn[16] = acadoVariables.od[lRun1 * 15 + 10];
acadoWorkspace.objValueIn[17] = acadoVariables.od[lRun1 * 15 + 11];
acadoWorkspace.objValueIn[18] = acadoVariables.od[lRun1 * 15 + 12];
acadoWorkspace.objValueIn[19] = acadoVariables.od[lRun1 * 15 + 13];
acadoWorkspace.objValueIn[20] = acadoVariables.od[lRun1 * 15 + 14];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[lRun1 * 12] = acadoWorkspace.objValueOut[0] - acadoVariables.y[lRun1 * 12];
acadoWorkspace.Dy[lRun1 * 12 + 1] = acadoWorkspace.objValueOut[1] - acadoVariables.y[lRun1 * 12 + 1];
acadoWorkspace.Dy[lRun1 * 12 + 2] = acadoWorkspace.objValueOut[2] - acadoVariables.y[lRun1 * 12 + 2];
acadoWorkspace.Dy[lRun1 * 12 + 3] = acadoWorkspace.objValueOut[3] - acadoVariables.y[lRun1 * 12 + 3];
acadoWorkspace.Dy[lRun1 * 12 + 4] = acadoWorkspace.objValueOut[4] - acadoVariables.y[lRun1 * 12 + 4];
acadoWorkspace.Dy[lRun1 * 12 + 5] = acadoWorkspace.objValueOut[5] - acadoVariables.y[lRun1 * 12 + 5];
acadoWorkspace.Dy[lRun1 * 12 + 6] = acadoWorkspace.objValueOut[6] - acadoVariables.y[lRun1 * 12 + 6];
acadoWorkspace.Dy[lRun1 * 12 + 7] = acadoWorkspace.objValueOut[7] - acadoVariables.y[lRun1 * 12 + 7];
acadoWorkspace.Dy[lRun1 * 12 + 8] = acadoWorkspace.objValueOut[8] - acadoVariables.y[lRun1 * 12 + 8];
acadoWorkspace.Dy[lRun1 * 12 + 9] = acadoWorkspace.objValueOut[9] - acadoVariables.y[lRun1 * 12 + 9];
acadoWorkspace.Dy[lRun1 * 12 + 10] = acadoWorkspace.objValueOut[10] - acadoVariables.y[lRun1 * 12 + 10];
acadoWorkspace.Dy[lRun1 * 12 + 11] = acadoWorkspace.objValueOut[11] - acadoVariables.y[lRun1 * 12 + 11];
}
acadoWorkspace.objValueIn[0] = acadoVariables.x[120];
acadoWorkspace.objValueIn[1] = acadoVariables.x[121];
acadoWorkspace.objValueIn[2] = acadoVariables.x[122];
acadoWorkspace.objValueIn[3] = acadoVariables.x[123];
acadoWorkspace.objValueIn[4] = acadoVariables.od[450];
acadoWorkspace.objValueIn[5] = acadoVariables.od[451];
acadoWorkspace.objValueIn[6] = acadoVariables.od[452];
acadoWorkspace.objValueIn[7] = acadoVariables.od[453];
acadoWorkspace.objValueIn[8] = acadoVariables.od[454];
acadoWorkspace.objValueIn[9] = acadoVariables.od[455];
acadoWorkspace.objValueIn[10] = acadoVariables.od[456];
acadoWorkspace.objValueIn[11] = acadoVariables.od[457];
acadoWorkspace.objValueIn[12] = acadoVariables.od[458];
acadoWorkspace.objValueIn[13] = acadoVariables.od[459];
acadoWorkspace.objValueIn[14] = acadoVariables.od[460];
acadoWorkspace.objValueIn[15] = acadoVariables.od[461];
acadoWorkspace.objValueIn[16] = acadoVariables.od[462];
acadoWorkspace.objValueIn[17] = acadoVariables.od[463];
acadoWorkspace.objValueIn[18] = acadoVariables.od[464];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0] - acadoVariables.yN[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1] - acadoVariables.yN[1];
acadoWorkspace.DyN[2] = acadoWorkspace.objValueOut[2] - acadoVariables.yN[2];
objVal = 0.0000000000000000e+00;
for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
tmpDy[0] = + acadoWorkspace.Dy[lRun1 * 12]*acadoVariables.W[0];
tmpDy[1] = + acadoWorkspace.Dy[lRun1 * 12 + 1]*acadoVariables.W[13];
tmpDy[2] = + acadoWorkspace.Dy[lRun1 * 12 + 2]*acadoVariables.W[26];
tmpDy[3] = + acadoWorkspace.Dy[lRun1 * 12 + 3]*acadoVariables.W[39];
tmpDy[4] = + acadoWorkspace.Dy[lRun1 * 12 + 4]*acadoVariables.W[52];
tmpDy[5] = + acadoWorkspace.Dy[lRun1 * 12 + 5]*acadoVariables.W[65];
tmpDy[6] = + acadoWorkspace.Dy[lRun1 * 12 + 6]*acadoVariables.W[78];
tmpDy[7] = + acadoWorkspace.Dy[lRun1 * 12 + 7]*acadoVariables.W[91];
tmpDy[8] = + acadoWorkspace.Dy[lRun1 * 12 + 8]*acadoVariables.W[104];
tmpDy[9] = + acadoWorkspace.Dy[lRun1 * 12 + 9]*acadoVariables.W[117];
tmpDy[10] = + acadoWorkspace.Dy[lRun1 * 12 + 10]*acadoVariables.W[130];
tmpDy[11] = + acadoWorkspace.Dy[lRun1 * 12 + 11]*acadoVariables.W[143];
objVal += + acadoWorkspace.Dy[lRun1 * 12]*tmpDy[0] + acadoWorkspace.Dy[lRun1 * 12 + 1]*tmpDy[1] + acadoWorkspace.Dy[lRun1 * 12 + 2]*tmpDy[2] + acadoWorkspace.Dy[lRun1 * 12 + 3]*tmpDy[3] + acadoWorkspace.Dy[lRun1 * 12 + 4]*tmpDy[4] + acadoWorkspace.Dy[lRun1 * 12 + 5]*tmpDy[5] + acadoWorkspace.Dy[lRun1 * 12 + 6]*tmpDy[6] + acadoWorkspace.Dy[lRun1 * 12 + 7]*tmpDy[7] + acadoWorkspace.Dy[lRun1 * 12 + 8]*tmpDy[8] + acadoWorkspace.Dy[lRun1 * 12 + 9]*tmpDy[9] + acadoWorkspace.Dy[lRun1 * 12 + 10]*tmpDy[10] + acadoWorkspace.Dy[lRun1 * 12 + 11]*tmpDy[11];
}

tmpDyN[0] = + acadoWorkspace.DyN[0]*acadoVariables.WN[0];
tmpDyN[1] = + acadoWorkspace.DyN[1]*acadoVariables.WN[4];
tmpDyN[2] = + acadoWorkspace.DyN[2]*acadoVariables.WN[8];
objVal += + acadoWorkspace.DyN[0]*tmpDyN[0] + acadoWorkspace.DyN[1]*tmpDyN[1] + acadoWorkspace.DyN[2]*tmpDyN[2];

objVal *= 0.5;
return objVal;
}

