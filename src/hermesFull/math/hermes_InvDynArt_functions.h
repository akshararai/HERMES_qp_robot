
void
hermes_InvDynArtfunc1(void)
     {
/* rotation matrices */
S00[1][1]=-1 + 2*Power(baseo[0].q[1],2) + 2*Power(baseo[0].q[2],2);
S00[1][2]=2*(baseo[0].q[2]*baseo[0].q[3] + baseo[0].q[1]*baseo[0].q[4]);
S00[1][3]=2*(-(baseo[0].q[1]*baseo[0].q[3]) + baseo[0].q[2]*baseo[0].q[4]);

S00[2][1]=2*(baseo[0].q[2]*baseo[0].q[3] - baseo[0].q[1]*baseo[0].q[4]);
S00[2][2]=-1 + 2*Power(baseo[0].q[1],2) + 2*Power(baseo[0].q[3],2);
S00[2][3]=2*(baseo[0].q[1]*baseo[0].q[2] + baseo[0].q[3]*baseo[0].q[4]);

S00[3][1]=2*(baseo[0].q[1]*baseo[0].q[3] + baseo[0].q[2]*baseo[0].q[4]);
S00[3][2]=2*(-(baseo[0].q[1]*baseo[0].q[2]) + baseo[0].q[3]*baseo[0].q[4]);
S00[3][3]=-1 + 2*Power(baseo[0].q[1],2) + 2*Power(baseo[0].q[4],2);


S10[1][1]=cstate29th;
S10[1][2]=-sstate29th;

S10[2][1]=-sstate29th;
S10[2][2]=-cstate29th;


S21[1][1]=-sstate30th;
S21[1][3]=cstate30th;

S21[2][1]=-cstate30th;
S21[2][3]=-sstate30th;


S32[1][1]=cstate31th;
S32[1][3]=-sstate31th;

S32[2][1]=-sstate31th;
S32[2][3]=-cstate31th;


S43[1][1]=0.7071067811865475*cstate1th;
S43[1][2]=-sstate1th;
S43[1][3]=0.7071067811865475*cstate1th;

S43[2][1]=-0.7071067811865475*sstate1th;
S43[2][2]=-cstate1th;
S43[2][3]=-0.7071067811865475*sstate1th;


S54[1][1]=-0.7071067811865475*cstate2th - 0.7071067811865475*sstate2th;
S54[1][3]=0.7071067811865475*cstate2th - 0.7071067811865475*sstate2th;

S54[2][1]=-0.7071067811865475*cstate2th + 0.7071067811865475*sstate2th;
S54[2][3]=-0.7071067811865475*cstate2th - 0.7071067811865475*sstate2th;


S65[1][1]=cstate3th;
S65[1][3]=-sstate3th;

S65[2][1]=-sstate3th;
S65[2][3]=-cstate3th;


S76[1][2]=cstate4th;
S76[1][3]=sstate4th;

S76[2][2]=-sstate4th;
S76[2][3]=cstate4th;


S87[1][1]=cstate5th;
S87[1][3]=-sstate5th;

S87[2][1]=-sstate5th;
S87[2][3]=-cstate5th;


S98[1][2]=sstate6th;
S98[1][3]=-cstate6th;

S98[2][2]=cstate6th;
S98[2][3]=sstate6th;


S109[1][1]=cstate7th;
S109[1][3]=sstate7th;

S109[2][1]=-sstate7th;
S109[2][3]=cstate7th;


S1110[1][1]=rceff2a2*rceff2a3;
S1110[1][2]=rceff2a3*rseff2a1*rseff2a2 + rceff2a1*rseff2a3;
S1110[1][3]=-(rceff2a1*rceff2a3*rseff2a2) + rseff2a1*rseff2a3;

S1110[2][1]=-(rceff2a2*rseff2a3);
S1110[2][2]=rceff2a1*rceff2a3 - rseff2a1*rseff2a2*rseff2a3;
S1110[2][3]=rceff2a3*rseff2a1 + rceff2a1*rseff2a2*rseff2a3;

S1110[3][1]=rseff2a2;
S1110[3][2]=-(rceff2a2*rseff2a1);
S1110[3][3]=rceff2a1*rceff2a2;


S1210[1][1]=rcTHUMBZROT;
S1210[1][2]=rcHANDXROT*rsTHUMBZROT;
S1210[1][3]=rsHANDXROT*rsTHUMBZROT;

S1210[2][1]=-(cstate39th*rsTHUMBZROT);
S1210[2][2]=cstate39th*rcHANDXROT*rcTHUMBZROT - rsHANDXROT*sstate39th;
S1210[2][3]=cstate39th*rcTHUMBZROT*rsHANDXROT + rcHANDXROT*sstate39th;

S1210[3][1]=rsTHUMBZROT*sstate39th;
S1210[3][2]=-(cstate39th*rsHANDXROT) - rcHANDXROT*rcTHUMBZROT*sstate39th;
S1210[3][3]=cstate39th*rcHANDXROT - rcTHUMBZROT*rsHANDXROT*sstate39th;


S1312[1][1]=cstate40th*rcTHUMBANGLEOFF + rsTHUMBANGLEOFF*sstate40th;
S1312[1][2]=cstate40th*rsTHUMBANGLEOFF - rcTHUMBANGLEOFF*sstate40th;

S1312[2][1]=-(cstate40th*rsTHUMBANGLEOFF) + rcTHUMBANGLEOFF*sstate40th;
S1312[2][2]=cstate40th*rcTHUMBANGLEOFF + rsTHUMBANGLEOFF*sstate40th;


S1413[1][1]=rcMinusstate40th;
S1413[1][2]=rsMinusstate40th;

S1413[2][1]=-rsMinusstate40th;
S1413[2][2]=rcMinusstate40th;



S1610[1][1]=cstate41th;
S1610[1][2]=rcHANDXROT*sstate41th;
S1610[1][3]=rsHANDXROT*sstate41th;

S1610[2][1]=-sstate41th;
S1610[2][2]=cstate41th*rcHANDXROT;
S1610[2][3]=cstate41th*rsHANDXROT;

S1610[3][2]=-rsHANDXROT;
S1610[3][3]=rcHANDXROT;


S1716[1][1]=rcstate41th;
S1716[1][2]=rsstate41th;

S1716[2][1]=-rsstate41th;
S1716[2][2]=rcstate41th;


S1817[1][1]=rcstate41th;
S1817[1][2]=rsstate41th;

S1817[2][1]=-rsstate41th;
S1817[2][2]=rcstate41th;



S2010[1][1]=cstate42th;
S2010[1][2]=rcHANDXROT*sstate42th;
S2010[1][3]=rsHANDXROT*sstate42th;

S2010[2][1]=-sstate42th;
S2010[2][2]=cstate42th*rcHANDXROT;
S2010[2][3]=cstate42th*rsHANDXROT;

S2010[3][2]=-rsHANDXROT;
S2010[3][3]=rcHANDXROT;


S2120[1][1]=rcstate42th;
S2120[1][2]=rsstate42th;

S2120[2][1]=-rsstate42th;
S2120[2][2]=rcstate42th;


S2221[1][1]=rcstate42th;
S2221[1][2]=rsstate42th;

S2221[2][1]=-rsstate42th;
S2221[2][2]=rcstate42th;



S2410[1][1]=cstate43th;
S2410[1][2]=rcHANDXROT*sstate43th;
S2410[1][3]=rsHANDXROT*sstate43th;

S2410[2][1]=-sstate43th;
S2410[2][2]=cstate43th*rcHANDXROT;
S2410[2][3]=cstate43th*rsHANDXROT;

S2410[3][2]=-rsHANDXROT;
S2410[3][3]=rcHANDXROT;


S2524[1][1]=rcstate43th;
S2524[1][2]=rsstate43th;

S2524[2][1]=-rsstate43th;
S2524[2][2]=rcstate43th;


S2625[1][1]=rcstate43th;
S2625[1][2]=rsstate43th;

S2625[2][1]=-rsstate43th;
S2625[2][2]=rcstate43th;



S2810[1][1]=cstate44th;
S2810[1][2]=rcHANDXROT*sstate44th;
S2810[1][3]=rsHANDXROT*sstate44th;

S2810[2][1]=-sstate44th;
S2810[2][2]=cstate44th*rcHANDXROT;
S2810[2][3]=cstate44th*rsHANDXROT;

S2810[3][2]=-rsHANDXROT;
S2810[3][3]=rcHANDXROT;


S2928[1][1]=rcstate44th;
S2928[1][2]=rsstate44th;

S2928[2][1]=-rsstate44th;
S2928[2][2]=rcstate44th;


S3029[1][1]=rcstate44th;
S3029[1][2]=rsstate44th;

S3029[2][1]=-rsstate44th;
S3029[2][2]=rcstate44th;



S323[1][1]=0.7071067811865475*cstate8th;
S323[1][2]=-sstate8th;
S323[1][3]=-0.7071067811865475*cstate8th;

S323[2][1]=-0.7071067811865475*sstate8th;
S323[2][2]=-cstate8th;
S323[2][3]=0.7071067811865475*sstate8th;


S3332[1][1]=-0.7071067811865475*cstate9th - 0.7071067811865475*sstate9th;
S3332[1][3]=-0.7071067811865475*cstate9th + 0.7071067811865475*sstate9th;

S3332[2][1]=-0.7071067811865475*cstate9th + 0.7071067811865475*sstate9th;
S3332[2][3]=0.7071067811865475*cstate9th + 0.7071067811865475*sstate9th;


S3433[1][1]=cstate10th;
S3433[1][3]=sstate10th;

S3433[2][1]=-sstate10th;
S3433[2][3]=cstate10th;


S3534[1][2]=cstate11th;
S3534[1][3]=-sstate11th;

S3534[2][2]=-sstate11th;
S3534[2][3]=-cstate11th;


S3635[1][1]=cstate12th;
S3635[1][3]=sstate12th;

S3635[2][1]=-sstate12th;
S3635[2][3]=cstate12th;


S3736[1][2]=sstate13th;
S3736[1][3]=cstate13th;

S3736[2][2]=cstate13th;
S3736[2][3]=-sstate13th;


S3837[1][1]=cstate14th;
S3837[1][3]=-sstate14th;

S3837[2][1]=-sstate14th;
S3837[2][3]=-cstate14th;


S3938[1][1]=rceff1a2*rceff1a3;
S3938[1][2]=rceff1a3*rseff1a1*rseff1a2 + rceff1a1*rseff1a3;
S3938[1][3]=-(rceff1a1*rceff1a3*rseff1a2) + rseff1a1*rseff1a3;

S3938[2][1]=-(rceff1a2*rseff1a3);
S3938[2][2]=rceff1a1*rceff1a3 - rseff1a1*rseff1a2*rseff1a3;
S3938[2][3]=rceff1a3*rseff1a1 + rceff1a1*rseff1a2*rseff1a3;

S3938[3][1]=rseff1a2;
S3938[3][2]=-(rceff1a2*rseff1a1);
S3938[3][3]=rceff1a1*rceff1a2;


S4038[1][1]=rcTHUMBZROT;
S4038[1][2]=rcMinusHANDXROT*rsTHUMBZROT;
S4038[1][3]=rsMinusHANDXROT*rsTHUMBZROT;

S4038[2][1]=-(cstate45th*rsTHUMBZROT);
S4038[2][2]=cstate45th*rcMinusHANDXROT*rcTHUMBZROT - rsMinusHANDXROT*sstate45th;
S4038[2][3]=cstate45th*rcTHUMBZROT*rsMinusHANDXROT + rcMinusHANDXROT*sstate45th;

S4038[3][1]=rsTHUMBZROT*sstate45th;
S4038[3][2]=-(cstate45th*rsMinusHANDXROT) - rcMinusHANDXROT*rcTHUMBZROT*sstate45th;
S4038[3][3]=cstate45th*rcMinusHANDXROT - rcTHUMBZROT*rsMinusHANDXROT*sstate45th;


S4140[1][1]=cstate46th*rcTHUMBANGLEOFF + rsTHUMBANGLEOFF*sstate46th;
S4140[1][2]=cstate46th*rsTHUMBANGLEOFF - rcTHUMBANGLEOFF*sstate46th;

S4140[2][1]=-(cstate46th*rsTHUMBANGLEOFF) + rcTHUMBANGLEOFF*sstate46th;
S4140[2][2]=cstate46th*rcTHUMBANGLEOFF + rsTHUMBANGLEOFF*sstate46th;


S4241[1][1]=rcMinusstate46th;
S4241[1][2]=rsMinusstate46th;

S4241[2][1]=-rsMinusstate46th;
S4241[2][2]=rcMinusstate46th;



S4438[1][1]=cstate47th;
S4438[1][2]=rcMinusHANDXROT*sstate47th;
S4438[1][3]=rsMinusHANDXROT*sstate47th;

S4438[2][1]=-sstate47th;
S4438[2][2]=cstate47th*rcMinusHANDXROT;
S4438[2][3]=cstate47th*rsMinusHANDXROT;

S4438[3][2]=-rsMinusHANDXROT;
S4438[3][3]=rcMinusHANDXROT;


S4544[1][1]=rcstate47th;
S4544[1][2]=rsstate47th;

S4544[2][1]=-rsstate47th;
S4544[2][2]=rcstate47th;


S4645[1][1]=rcstate47th;
S4645[1][2]=rsstate47th;

S4645[2][1]=-rsstate47th;
S4645[2][2]=rcstate47th;



S4838[1][1]=cstate48th;
S4838[1][2]=rcMinusHANDXROT*sstate48th;
S4838[1][3]=rsMinusHANDXROT*sstate48th;

S4838[2][1]=-sstate48th;
S4838[2][2]=cstate48th*rcMinusHANDXROT;
S4838[2][3]=cstate48th*rsMinusHANDXROT;

S4838[3][2]=-rsMinusHANDXROT;
S4838[3][3]=rcMinusHANDXROT;


S4948[1][1]=rcstate48th;
S4948[1][2]=rsstate48th;

S4948[2][1]=-rsstate48th;
S4948[2][2]=rcstate48th;


S5049[1][1]=rcstate48th;
S5049[1][2]=rsstate48th;

S5049[2][1]=-rsstate48th;
S5049[2][2]=rcstate48th;



S5238[1][1]=cstate49th;
S5238[1][2]=rcMinusHANDXROT*sstate49th;
S5238[1][3]=rsMinusHANDXROT*sstate49th;

S5238[2][1]=-sstate49th;
S5238[2][2]=cstate49th*rcMinusHANDXROT;
S5238[2][3]=cstate49th*rsMinusHANDXROT;

S5238[3][2]=-rsMinusHANDXROT;
S5238[3][3]=rcMinusHANDXROT;


S5352[1][1]=rcstate49th;
S5352[1][2]=rsstate49th;

S5352[2][1]=-rsstate49th;
S5352[2][2]=rcstate49th;


S5453[1][1]=rcstate49th;
S5453[1][2]=rsstate49th;

S5453[2][1]=-rsstate49th;
S5453[2][2]=rcstate49th;



S5638[1][1]=cstate50th;
S5638[1][2]=rcMinusHANDXROT*sstate50th;
S5638[1][3]=rsMinusHANDXROT*sstate50th;

S5638[2][1]=-sstate50th;
S5638[2][2]=cstate50th*rcMinusHANDXROT;
S5638[2][3]=cstate50th*rsMinusHANDXROT;

S5638[3][2]=-rsMinusHANDXROT;
S5638[3][3]=rcMinusHANDXROT;


S5756[1][1]=rcstate50th;
S5756[1][2]=rsstate50th;

S5756[2][1]=-rsstate50th;
S5756[2][2]=rcstate50th;


S5857[1][1]=rcstate50th;
S5857[1][2]=rsstate50th;

S5857[2][1]=-rsstate50th;
S5857[2][2]=rcstate50th;



S603[1][1]=sstate32th;
S603[1][2]=-cstate32th;

S603[2][1]=cstate32th;
S603[2][2]=sstate32th;


S6160[1][2]=sstate33th;
S6160[1][3]=-cstate33th;

S6160[2][2]=cstate33th;
S6160[2][3]=sstate33th;


S6261[1][1]=cstate34th;
S6261[1][3]=-sstate34th;

S6261[2][1]=-sstate34th;
S6261[2][3]=-cstate34th;


S6362[1][1]=cstate35th;
S6362[1][2]=sstate35th;

S6362[2][1]=-sstate35th;
S6362[2][2]=cstate35th;


S6463[1][2]=sstate36th;
S6463[1][3]=cstate36th;

S6463[2][2]=cstate36th;
S6463[2][3]=-sstate36th;



S6662[1][1]=cstate37th;
S6662[1][2]=sstate37th;

S6662[2][1]=-sstate37th;
S6662[2][2]=cstate37th;


S6766[1][2]=sstate38th;
S6766[1][3]=cstate38th;

S6766[2][2]=cstate38th;
S6766[2][3]=-sstate38th;




S700[1][1]=-sstate23th;
S700[1][3]=-cstate23th;

S700[2][1]=-cstate23th;
S700[2][3]=sstate23th;


S7170[1][1]=sstate22th;
S7170[1][3]=-cstate22th;

S7170[2][1]=cstate22th;
S7170[2][3]=sstate22th;


S7271[1][1]=cstate24th*rcMinusTHIGHANGLEOFF;
S7271[1][2]=-(cstate24th*rsMinusTHIGHANGLEOFF);
S7271[1][3]=-sstate24th;

S7271[2][1]=-(rcMinusTHIGHANGLEOFF*sstate24th);
S7271[2][2]=rsMinusTHIGHANGLEOFF*sstate24th;
S7271[2][3]=-cstate24th;

S7271[3][1]=rsMinusTHIGHANGLEOFF;
S7271[3][2]=rcMinusTHIGHANGLEOFF;


S7372[1][1]=cstate25th*rcTHIGHANGLEOFF - rsTHIGHANGLEOFF*sstate25th;
S7372[1][3]=-(cstate25th*rsTHIGHANGLEOFF) - rcTHIGHANGLEOFF*sstate25th;

S7372[2][1]=-(cstate25th*rsTHIGHANGLEOFF) - rcTHIGHANGLEOFF*sstate25th;
S7372[2][3]=-(cstate25th*rcTHIGHANGLEOFF) + rsTHIGHANGLEOFF*sstate25th;


S7473[1][1]=cstate26th;
S7473[1][3]=sstate26th;

S7473[2][1]=-sstate26th;
S7473[2][3]=cstate26th;


S7574[1][1]=-sstate27th;
S7574[1][3]=cstate27th;

S7574[2][1]=-cstate27th;
S7574[2][3]=-sstate27th;


S7675[1][1]=cstate28th;
S7675[1][3]=-sstate28th;

S7675[2][1]=-sstate28th;
S7675[2][3]=-cstate28th;








S8376[1][1]=rceff3a2*rceff3a3;
S8376[1][2]=rceff3a3*rseff3a1*rseff3a2 + rceff3a1*rseff3a3;
S8376[1][3]=-(rceff3a1*rceff3a3*rseff3a2) + rseff3a1*rseff3a3;

S8376[2][1]=-(rceff3a2*rseff3a3);
S8376[2][2]=rceff3a1*rceff3a3 - rseff3a1*rseff3a2*rseff3a3;
S8376[2][3]=rceff3a3*rseff3a1 + rceff3a1*rseff3a2*rseff3a3;

S8376[3][1]=rseff3a2;
S8376[3][2]=-(rceff3a2*rseff3a1);
S8376[3][3]=rceff3a1*rceff3a2;


S840[1][1]=sstate16th;
S840[1][3]=-cstate16th;

S840[2][1]=cstate16th;
S840[2][3]=sstate16th;


S8584[1][1]=sstate15th;
S8584[1][3]=cstate15th;

S8584[2][1]=cstate15th;
S8584[2][3]=-sstate15th;


S8685[1][1]=cstate17th*rcTHIGHANGLEOFF;
S8685[1][2]=cstate17th*rsTHIGHANGLEOFF;
S8685[1][3]=sstate17th;

S8685[2][1]=-(rcTHIGHANGLEOFF*sstate17th);
S8685[2][2]=-(rsTHIGHANGLEOFF*sstate17th);
S8685[2][3]=cstate17th;

S8685[3][1]=rsTHIGHANGLEOFF;
S8685[3][2]=-rcTHIGHANGLEOFF;


S8786[1][1]=cstate18th*rcTHIGHANGLEOFF - rsTHIGHANGLEOFF*sstate18th;
S8786[1][3]=cstate18th*rsTHIGHANGLEOFF + rcTHIGHANGLEOFF*sstate18th;

S8786[2][1]=-(cstate18th*rsTHIGHANGLEOFF) - rcTHIGHANGLEOFF*sstate18th;
S8786[2][3]=cstate18th*rcTHIGHANGLEOFF - rsTHIGHANGLEOFF*sstate18th;


S8887[1][1]=cstate19th;
S8887[1][3]=-sstate19th;

S8887[2][1]=-sstate19th;
S8887[2][3]=-cstate19th;


S8988[1][1]=-sstate20th;
S8988[1][3]=-cstate20th;

S8988[2][1]=-cstate20th;
S8988[2][3]=sstate20th;


S9089[1][1]=cstate21th;
S9089[1][3]=sstate21th;

S9089[2][1]=-sstate21th;
S9089[2][3]=cstate21th;








S9790[1][1]=rceff4a2*rceff4a3;
S9790[1][2]=rceff4a3*rseff4a1*rseff4a2 + rceff4a1*rseff4a3;
S9790[1][3]=-(rceff4a1*rceff4a3*rseff4a2) + rseff4a1*rseff4a3;

S9790[2][1]=-(rceff4a2*rseff4a3);
S9790[2][2]=rceff4a1*rceff4a3 - rseff4a1*rseff4a2*rseff4a3;
S9790[2][3]=rceff4a3*rseff4a1 + rceff4a1*rseff4a2*rseff4a3;

S9790[3][1]=rseff4a2;
S9790[3][2]=-(rceff4a2*rseff4a1);
S9790[3][3]=rceff4a1*rceff4a2;




}


void
hermes_InvDynArtfunc2(void)
     {
/* inverse rotation matrices */
Si00[1][1]=-1 + 2*Power(baseo[0].q[1],2) + 2*Power(baseo[0].q[2],2);
Si00[1][2]=2*(baseo[0].q[2]*baseo[0].q[3] - baseo[0].q[1]*baseo[0].q[4]);
Si00[1][3]=2*(baseo[0].q[1]*baseo[0].q[3] + baseo[0].q[2]*baseo[0].q[4]);

Si00[2][1]=2*(baseo[0].q[2]*baseo[0].q[3] + baseo[0].q[1]*baseo[0].q[4]);
Si00[2][2]=-1 + 2*Power(baseo[0].q[1],2) + 2*Power(baseo[0].q[3],2);
Si00[2][3]=2*(-(baseo[0].q[1]*baseo[0].q[2]) + baseo[0].q[3]*baseo[0].q[4]);

Si00[3][1]=2*(-(baseo[0].q[1]*baseo[0].q[3]) + baseo[0].q[2]*baseo[0].q[4]);
Si00[3][2]=2*(baseo[0].q[1]*baseo[0].q[2] + baseo[0].q[3]*baseo[0].q[4]);
Si00[3][3]=-1 + 2*Power(baseo[0].q[1],2) + 2*Power(baseo[0].q[4],2);


Si01[1][1]=cstate29th;
Si01[1][2]=-sstate29th;

Si01[2][1]=-sstate29th;
Si01[2][2]=-cstate29th;


Si12[1][1]=-sstate30th;
Si12[1][2]=-cstate30th;

Si12[3][1]=cstate30th;
Si12[3][2]=-sstate30th;


Si23[1][1]=cstate31th;
Si23[1][2]=-sstate31th;

Si23[3][1]=-sstate31th;
Si23[3][2]=-cstate31th;


Si34[1][1]=0.7071067811865475*cstate1th;
Si34[1][2]=-0.7071067811865475*sstate1th;

Si34[2][1]=-sstate1th;
Si34[2][2]=-cstate1th;

Si34[3][1]=0.7071067811865475*cstate1th;
Si34[3][2]=-0.7071067811865475*sstate1th;


Si45[1][1]=-0.7071067811865475*cstate2th - 0.7071067811865475*sstate2th;
Si45[1][2]=-0.7071067811865475*cstate2th + 0.7071067811865475*sstate2th;

Si45[3][1]=0.7071067811865475*cstate2th - 0.7071067811865475*sstate2th;
Si45[3][2]=-0.7071067811865475*cstate2th - 0.7071067811865475*sstate2th;


Si56[1][1]=cstate3th;
Si56[1][2]=-sstate3th;

Si56[3][1]=-sstate3th;
Si56[3][2]=-cstate3th;


Si67[2][1]=cstate4th;
Si67[2][2]=-sstate4th;

Si67[3][1]=sstate4th;
Si67[3][2]=cstate4th;


Si78[1][1]=cstate5th;
Si78[1][2]=-sstate5th;

Si78[3][1]=-sstate5th;
Si78[3][2]=-cstate5th;


Si89[2][1]=sstate6th;
Si89[2][2]=cstate6th;

Si89[3][1]=-cstate6th;
Si89[3][2]=sstate6th;


Si910[1][1]=cstate7th;
Si910[1][2]=-sstate7th;

Si910[3][1]=sstate7th;
Si910[3][2]=cstate7th;


Si1011[1][1]=rceff2a2*rceff2a3;
Si1011[1][2]=-(rceff2a2*rseff2a3);
Si1011[1][3]=rseff2a2;

Si1011[2][1]=rceff2a3*rseff2a1*rseff2a2 + rceff2a1*rseff2a3;
Si1011[2][2]=rceff2a1*rceff2a3 - rseff2a1*rseff2a2*rseff2a3;
Si1011[2][3]=-(rceff2a2*rseff2a1);

Si1011[3][1]=-(rceff2a1*rceff2a3*rseff2a2) + rseff2a1*rseff2a3;
Si1011[3][2]=rceff2a3*rseff2a1 + rceff2a1*rseff2a2*rseff2a3;
Si1011[3][3]=rceff2a1*rceff2a2;


Si1012[1][1]=rcTHUMBZROT;
Si1012[1][2]=-(cstate39th*rsTHUMBZROT);
Si1012[1][3]=rsTHUMBZROT*sstate39th;

Si1012[2][1]=rcHANDXROT*rsTHUMBZROT;
Si1012[2][2]=cstate39th*rcHANDXROT*rcTHUMBZROT - rsHANDXROT*sstate39th;
Si1012[2][3]=-(cstate39th*rsHANDXROT) - rcHANDXROT*rcTHUMBZROT*sstate39th;

Si1012[3][1]=rsHANDXROT*rsTHUMBZROT;
Si1012[3][2]=cstate39th*rcTHUMBZROT*rsHANDXROT + rcHANDXROT*sstate39th;
Si1012[3][3]=cstate39th*rcHANDXROT - rcTHUMBZROT*rsHANDXROT*sstate39th;


Si1213[1][1]=cstate40th*rcTHUMBANGLEOFF + rsTHUMBANGLEOFF*sstate40th;
Si1213[1][2]=-(cstate40th*rsTHUMBANGLEOFF) + rcTHUMBANGLEOFF*sstate40th;

Si1213[2][1]=cstate40th*rsTHUMBANGLEOFF - rcTHUMBANGLEOFF*sstate40th;
Si1213[2][2]=cstate40th*rcTHUMBANGLEOFF + rsTHUMBANGLEOFF*sstate40th;


Si1314[1][1]=rcMinusstate40th;
Si1314[1][2]=-rsMinusstate40th;

Si1314[2][1]=rsMinusstate40th;
Si1314[2][2]=rcMinusstate40th;



Si1016[1][1]=cstate41th;
Si1016[1][2]=-sstate41th;

Si1016[2][1]=rcHANDXROT*sstate41th;
Si1016[2][2]=cstate41th*rcHANDXROT;
Si1016[2][3]=-rsHANDXROT;

Si1016[3][1]=rsHANDXROT*sstate41th;
Si1016[3][2]=cstate41th*rsHANDXROT;
Si1016[3][3]=rcHANDXROT;


Si1617[1][1]=rcstate41th;
Si1617[1][2]=-rsstate41th;

Si1617[2][1]=rsstate41th;
Si1617[2][2]=rcstate41th;


Si1718[1][1]=rcstate41th;
Si1718[1][2]=-rsstate41th;

Si1718[2][1]=rsstate41th;
Si1718[2][2]=rcstate41th;



Si1020[1][1]=cstate42th;
Si1020[1][2]=-sstate42th;

Si1020[2][1]=rcHANDXROT*sstate42th;
Si1020[2][2]=cstate42th*rcHANDXROT;
Si1020[2][3]=-rsHANDXROT;

Si1020[3][1]=rsHANDXROT*sstate42th;
Si1020[3][2]=cstate42th*rsHANDXROT;
Si1020[3][3]=rcHANDXROT;


Si2021[1][1]=rcstate42th;
Si2021[1][2]=-rsstate42th;

Si2021[2][1]=rsstate42th;
Si2021[2][2]=rcstate42th;


Si2122[1][1]=rcstate42th;
Si2122[1][2]=-rsstate42th;

Si2122[2][1]=rsstate42th;
Si2122[2][2]=rcstate42th;



Si1024[1][1]=cstate43th;
Si1024[1][2]=-sstate43th;

Si1024[2][1]=rcHANDXROT*sstate43th;
Si1024[2][2]=cstate43th*rcHANDXROT;
Si1024[2][3]=-rsHANDXROT;

Si1024[3][1]=rsHANDXROT*sstate43th;
Si1024[3][2]=cstate43th*rsHANDXROT;
Si1024[3][3]=rcHANDXROT;


Si2425[1][1]=rcstate43th;
Si2425[1][2]=-rsstate43th;

Si2425[2][1]=rsstate43th;
Si2425[2][2]=rcstate43th;


Si2526[1][1]=rcstate43th;
Si2526[1][2]=-rsstate43th;

Si2526[2][1]=rsstate43th;
Si2526[2][2]=rcstate43th;



Si1028[1][1]=cstate44th;
Si1028[1][2]=-sstate44th;

Si1028[2][1]=rcHANDXROT*sstate44th;
Si1028[2][2]=cstate44th*rcHANDXROT;
Si1028[2][3]=-rsHANDXROT;

Si1028[3][1]=rsHANDXROT*sstate44th;
Si1028[3][2]=cstate44th*rsHANDXROT;
Si1028[3][3]=rcHANDXROT;


Si2829[1][1]=rcstate44th;
Si2829[1][2]=-rsstate44th;

Si2829[2][1]=rsstate44th;
Si2829[2][2]=rcstate44th;


Si2930[1][1]=rcstate44th;
Si2930[1][2]=-rsstate44th;

Si2930[2][1]=rsstate44th;
Si2930[2][2]=rcstate44th;



Si332[1][1]=0.7071067811865475*cstate8th;
Si332[1][2]=-0.7071067811865475*sstate8th;

Si332[2][1]=-sstate8th;
Si332[2][2]=-cstate8th;

Si332[3][1]=-0.7071067811865475*cstate8th;
Si332[3][2]=0.7071067811865475*sstate8th;


Si3233[1][1]=-0.7071067811865475*cstate9th - 0.7071067811865475*sstate9th;
Si3233[1][2]=-0.7071067811865475*cstate9th + 0.7071067811865475*sstate9th;

Si3233[3][1]=-0.7071067811865475*cstate9th + 0.7071067811865475*sstate9th;
Si3233[3][2]=0.7071067811865475*cstate9th + 0.7071067811865475*sstate9th;


Si3334[1][1]=cstate10th;
Si3334[1][2]=-sstate10th;

Si3334[3][1]=sstate10th;
Si3334[3][2]=cstate10th;


Si3435[2][1]=cstate11th;
Si3435[2][2]=-sstate11th;

Si3435[3][1]=-sstate11th;
Si3435[3][2]=-cstate11th;


Si3536[1][1]=cstate12th;
Si3536[1][2]=-sstate12th;

Si3536[3][1]=sstate12th;
Si3536[3][2]=cstate12th;


Si3637[2][1]=sstate13th;
Si3637[2][2]=cstate13th;

Si3637[3][1]=cstate13th;
Si3637[3][2]=-sstate13th;


Si3738[1][1]=cstate14th;
Si3738[1][2]=-sstate14th;

Si3738[3][1]=-sstate14th;
Si3738[3][2]=-cstate14th;


Si3839[1][1]=rceff1a2*rceff1a3;
Si3839[1][2]=-(rceff1a2*rseff1a3);
Si3839[1][3]=rseff1a2;

Si3839[2][1]=rceff1a3*rseff1a1*rseff1a2 + rceff1a1*rseff1a3;
Si3839[2][2]=rceff1a1*rceff1a3 - rseff1a1*rseff1a2*rseff1a3;
Si3839[2][3]=-(rceff1a2*rseff1a1);

Si3839[3][1]=-(rceff1a1*rceff1a3*rseff1a2) + rseff1a1*rseff1a3;
Si3839[3][2]=rceff1a3*rseff1a1 + rceff1a1*rseff1a2*rseff1a3;
Si3839[3][3]=rceff1a1*rceff1a2;


Si3840[1][1]=rcTHUMBZROT;
Si3840[1][2]=-(cstate45th*rsTHUMBZROT);
Si3840[1][3]=rsTHUMBZROT*sstate45th;

Si3840[2][1]=rcMinusHANDXROT*rsTHUMBZROT;
Si3840[2][2]=cstate45th*rcMinusHANDXROT*rcTHUMBZROT - rsMinusHANDXROT*sstate45th;
Si3840[2][3]=-(cstate45th*rsMinusHANDXROT) - rcMinusHANDXROT*rcTHUMBZROT*sstate45th;

Si3840[3][1]=rsMinusHANDXROT*rsTHUMBZROT;
Si3840[3][2]=cstate45th*rcTHUMBZROT*rsMinusHANDXROT + rcMinusHANDXROT*sstate45th;
Si3840[3][3]=cstate45th*rcMinusHANDXROT - rcTHUMBZROT*rsMinusHANDXROT*sstate45th;


Si4041[1][1]=cstate46th*rcTHUMBANGLEOFF + rsTHUMBANGLEOFF*sstate46th;
Si4041[1][2]=-(cstate46th*rsTHUMBANGLEOFF) + rcTHUMBANGLEOFF*sstate46th;

Si4041[2][1]=cstate46th*rsTHUMBANGLEOFF - rcTHUMBANGLEOFF*sstate46th;
Si4041[2][2]=cstate46th*rcTHUMBANGLEOFF + rsTHUMBANGLEOFF*sstate46th;


Si4142[1][1]=rcMinusstate46th;
Si4142[1][2]=-rsMinusstate46th;

Si4142[2][1]=rsMinusstate46th;
Si4142[2][2]=rcMinusstate46th;



Si3844[1][1]=cstate47th;
Si3844[1][2]=-sstate47th;

Si3844[2][1]=rcMinusHANDXROT*sstate47th;
Si3844[2][2]=cstate47th*rcMinusHANDXROT;
Si3844[2][3]=-rsMinusHANDXROT;

Si3844[3][1]=rsMinusHANDXROT*sstate47th;
Si3844[3][2]=cstate47th*rsMinusHANDXROT;
Si3844[3][3]=rcMinusHANDXROT;


Si4445[1][1]=rcstate47th;
Si4445[1][2]=-rsstate47th;

Si4445[2][1]=rsstate47th;
Si4445[2][2]=rcstate47th;


Si4546[1][1]=rcstate47th;
Si4546[1][2]=-rsstate47th;

Si4546[2][1]=rsstate47th;
Si4546[2][2]=rcstate47th;



Si3848[1][1]=cstate48th;
Si3848[1][2]=-sstate48th;

Si3848[2][1]=rcMinusHANDXROT*sstate48th;
Si3848[2][2]=cstate48th*rcMinusHANDXROT;
Si3848[2][3]=-rsMinusHANDXROT;

Si3848[3][1]=rsMinusHANDXROT*sstate48th;
Si3848[3][2]=cstate48th*rsMinusHANDXROT;
Si3848[3][3]=rcMinusHANDXROT;


Si4849[1][1]=rcstate48th;
Si4849[1][2]=-rsstate48th;

Si4849[2][1]=rsstate48th;
Si4849[2][2]=rcstate48th;


Si4950[1][1]=rcstate48th;
Si4950[1][2]=-rsstate48th;

Si4950[2][1]=rsstate48th;
Si4950[2][2]=rcstate48th;



Si3852[1][1]=cstate49th;
Si3852[1][2]=-sstate49th;

Si3852[2][1]=rcMinusHANDXROT*sstate49th;
Si3852[2][2]=cstate49th*rcMinusHANDXROT;
Si3852[2][3]=-rsMinusHANDXROT;

Si3852[3][1]=rsMinusHANDXROT*sstate49th;
Si3852[3][2]=cstate49th*rsMinusHANDXROT;
Si3852[3][3]=rcMinusHANDXROT;


Si5253[1][1]=rcstate49th;
Si5253[1][2]=-rsstate49th;

Si5253[2][1]=rsstate49th;
Si5253[2][2]=rcstate49th;


Si5354[1][1]=rcstate49th;
Si5354[1][2]=-rsstate49th;

Si5354[2][1]=rsstate49th;
Si5354[2][2]=rcstate49th;



Si3856[1][1]=cstate50th;
Si3856[1][2]=-sstate50th;

Si3856[2][1]=rcMinusHANDXROT*sstate50th;
Si3856[2][2]=cstate50th*rcMinusHANDXROT;
Si3856[2][3]=-rsMinusHANDXROT;

Si3856[3][1]=rsMinusHANDXROT*sstate50th;
Si3856[3][2]=cstate50th*rsMinusHANDXROT;
Si3856[3][3]=rcMinusHANDXROT;


Si5657[1][1]=rcstate50th;
Si5657[1][2]=-rsstate50th;

Si5657[2][1]=rsstate50th;
Si5657[2][2]=rcstate50th;


Si5758[1][1]=rcstate50th;
Si5758[1][2]=-rsstate50th;

Si5758[2][1]=rsstate50th;
Si5758[2][2]=rcstate50th;



Si360[1][1]=sstate32th;
Si360[1][2]=cstate32th;

Si360[2][1]=-cstate32th;
Si360[2][2]=sstate32th;


Si6061[2][1]=sstate33th;
Si6061[2][2]=cstate33th;

Si6061[3][1]=-cstate33th;
Si6061[3][2]=sstate33th;


Si6162[1][1]=cstate34th;
Si6162[1][2]=-sstate34th;

Si6162[3][1]=-sstate34th;
Si6162[3][2]=-cstate34th;


Si6263[1][1]=cstate35th;
Si6263[1][2]=-sstate35th;

Si6263[2][1]=sstate35th;
Si6263[2][2]=cstate35th;


Si6364[2][1]=sstate36th;
Si6364[2][2]=cstate36th;

Si6364[3][1]=cstate36th;
Si6364[3][2]=-sstate36th;



Si6266[1][1]=cstate37th;
Si6266[1][2]=-sstate37th;

Si6266[2][1]=sstate37th;
Si6266[2][2]=cstate37th;


Si6667[2][1]=sstate38th;
Si6667[2][2]=cstate38th;

Si6667[3][1]=cstate38th;
Si6667[3][2]=-sstate38th;




Si070[1][1]=-sstate23th;
Si070[1][2]=-cstate23th;

Si070[3][1]=-cstate23th;
Si070[3][2]=sstate23th;


Si7071[1][1]=sstate22th;
Si7071[1][2]=cstate22th;

Si7071[3][1]=-cstate22th;
Si7071[3][2]=sstate22th;


Si7172[1][1]=cstate24th*rcMinusTHIGHANGLEOFF;
Si7172[1][2]=-(rcMinusTHIGHANGLEOFF*sstate24th);
Si7172[1][3]=rsMinusTHIGHANGLEOFF;

Si7172[2][1]=-(cstate24th*rsMinusTHIGHANGLEOFF);
Si7172[2][2]=rsMinusTHIGHANGLEOFF*sstate24th;
Si7172[2][3]=rcMinusTHIGHANGLEOFF;

Si7172[3][1]=-sstate24th;
Si7172[3][2]=-cstate24th;


Si7273[1][1]=cstate25th*rcTHIGHANGLEOFF - rsTHIGHANGLEOFF*sstate25th;
Si7273[1][2]=-(cstate25th*rsTHIGHANGLEOFF) - rcTHIGHANGLEOFF*sstate25th;

Si7273[3][1]=-(cstate25th*rsTHIGHANGLEOFF) - rcTHIGHANGLEOFF*sstate25th;
Si7273[3][2]=-(cstate25th*rcTHIGHANGLEOFF) + rsTHIGHANGLEOFF*sstate25th;


Si7374[1][1]=cstate26th;
Si7374[1][2]=-sstate26th;

Si7374[3][1]=sstate26th;
Si7374[3][2]=cstate26th;


Si7475[1][1]=-sstate27th;
Si7475[1][2]=-cstate27th;

Si7475[3][1]=cstate27th;
Si7475[3][2]=-sstate27th;


Si7576[1][1]=cstate28th;
Si7576[1][2]=-sstate28th;

Si7576[3][1]=-sstate28th;
Si7576[3][2]=-cstate28th;








Si7683[1][1]=rceff3a2*rceff3a3;
Si7683[1][2]=-(rceff3a2*rseff3a3);
Si7683[1][3]=rseff3a2;

Si7683[2][1]=rceff3a3*rseff3a1*rseff3a2 + rceff3a1*rseff3a3;
Si7683[2][2]=rceff3a1*rceff3a3 - rseff3a1*rseff3a2*rseff3a3;
Si7683[2][3]=-(rceff3a2*rseff3a1);

Si7683[3][1]=-(rceff3a1*rceff3a3*rseff3a2) + rseff3a1*rseff3a3;
Si7683[3][2]=rceff3a3*rseff3a1 + rceff3a1*rseff3a2*rseff3a3;
Si7683[3][3]=rceff3a1*rceff3a2;


Si084[1][1]=sstate16th;
Si084[1][2]=cstate16th;

Si084[3][1]=-cstate16th;
Si084[3][2]=sstate16th;


Si8485[1][1]=sstate15th;
Si8485[1][2]=cstate15th;

Si8485[3][1]=cstate15th;
Si8485[3][2]=-sstate15th;


Si8586[1][1]=cstate17th*rcTHIGHANGLEOFF;
Si8586[1][2]=-(rcTHIGHANGLEOFF*sstate17th);
Si8586[1][3]=rsTHIGHANGLEOFF;

Si8586[2][1]=cstate17th*rsTHIGHANGLEOFF;
Si8586[2][2]=-(rsTHIGHANGLEOFF*sstate17th);
Si8586[2][3]=-rcTHIGHANGLEOFF;

Si8586[3][1]=sstate17th;
Si8586[3][2]=cstate17th;


Si8687[1][1]=cstate18th*rcTHIGHANGLEOFF - rsTHIGHANGLEOFF*sstate18th;
Si8687[1][2]=-(cstate18th*rsTHIGHANGLEOFF) - rcTHIGHANGLEOFF*sstate18th;

Si8687[3][1]=cstate18th*rsTHIGHANGLEOFF + rcTHIGHANGLEOFF*sstate18th;
Si8687[3][2]=cstate18th*rcTHIGHANGLEOFF - rsTHIGHANGLEOFF*sstate18th;


Si8788[1][1]=cstate19th;
Si8788[1][2]=-sstate19th;

Si8788[3][1]=-sstate19th;
Si8788[3][2]=-cstate19th;


Si8889[1][1]=-sstate20th;
Si8889[1][2]=-cstate20th;

Si8889[3][1]=-cstate20th;
Si8889[3][2]=sstate20th;


Si8990[1][1]=cstate21th;
Si8990[1][2]=-sstate21th;

Si8990[3][1]=sstate21th;
Si8990[3][2]=cstate21th;








Si9097[1][1]=rceff4a2*rceff4a3;
Si9097[1][2]=-(rceff4a2*rseff4a3);
Si9097[1][3]=rseff4a2;

Si9097[2][1]=rceff4a3*rseff4a1*rseff4a2 + rceff4a1*rseff4a3;
Si9097[2][2]=rceff4a1*rceff4a3 - rseff4a1*rseff4a2*rseff4a3;
Si9097[2][3]=-(rceff4a2*rseff4a1);

Si9097[3][1]=-(rceff4a1*rceff4a3*rseff4a2) + rseff4a1*rseff4a3;
Si9097[3][2]=rceff4a3*rseff4a1 + rceff4a1*rseff4a2*rseff4a3;
Si9097[3][3]=rceff4a1*rceff4a2;




}


void
hermes_InvDynArtfunc3(void)
     {
/* rotation matrices from global to link coordinates */
SG10[1][1]=S00[1][1]*S10[1][1] + S00[2][1]*S10[1][2];
SG10[1][2]=S00[1][2]*S10[1][1] + S00[2][2]*S10[1][2];
SG10[1][3]=S00[1][3]*S10[1][1] + S00[2][3]*S10[1][2];

SG10[2][1]=S00[1][1]*S10[2][1] + S00[2][1]*S10[2][2];
SG10[2][2]=S00[1][2]*S10[2][1] + S00[2][2]*S10[2][2];
SG10[2][3]=S00[1][3]*S10[2][1] + S00[2][3]*S10[2][2];

SG10[3][1]=-S00[3][1];
SG10[3][2]=-S00[3][2];
SG10[3][3]=-S00[3][3];


SG20[1][1]=S21[1][1]*SG10[1][1] + S21[1][3]*SG10[3][1];
SG20[1][2]=S21[1][1]*SG10[1][2] + S21[1][3]*SG10[3][2];
SG20[1][3]=S21[1][1]*SG10[1][3] + S21[1][3]*SG10[3][3];

SG20[2][1]=S21[2][1]*SG10[1][1] + S21[2][3]*SG10[3][1];
SG20[2][2]=S21[2][1]*SG10[1][2] + S21[2][3]*SG10[3][2];
SG20[2][3]=S21[2][1]*SG10[1][3] + S21[2][3]*SG10[3][3];

SG20[3][1]=-SG10[2][1];
SG20[3][2]=-SG10[2][2];
SG20[3][3]=-SG10[2][3];


SG30[1][1]=S32[1][1]*SG20[1][1] + S32[1][3]*SG20[3][1];
SG30[1][2]=S32[1][1]*SG20[1][2] + S32[1][3]*SG20[3][2];
SG30[1][3]=S32[1][1]*SG20[1][3] + S32[1][3]*SG20[3][3];

SG30[2][1]=S32[2][1]*SG20[1][1] + S32[2][3]*SG20[3][1];
SG30[2][2]=S32[2][1]*SG20[1][2] + S32[2][3]*SG20[3][2];
SG30[2][3]=S32[2][1]*SG20[1][3] + S32[2][3]*SG20[3][3];

SG30[3][1]=SG20[2][1];
SG30[3][2]=SG20[2][2];
SG30[3][3]=SG20[2][3];


SG40[1][1]=S43[1][1]*SG30[1][1] + S43[1][2]*SG30[2][1] + S43[1][3]*SG30[3][1];
SG40[1][2]=S43[1][1]*SG30[1][2] + S43[1][2]*SG30[2][2] + S43[1][3]*SG30[3][2];
SG40[1][3]=S43[1][1]*SG30[1][3] + S43[1][2]*SG30[2][3] + S43[1][3]*SG30[3][3];

SG40[2][1]=S43[2][1]*SG30[1][1] + S43[2][2]*SG30[2][1] + S43[2][3]*SG30[3][1];
SG40[2][2]=S43[2][1]*SG30[1][2] + S43[2][2]*SG30[2][2] + S43[2][3]*SG30[3][2];
SG40[2][3]=S43[2][1]*SG30[1][3] + S43[2][2]*SG30[2][3] + S43[2][3]*SG30[3][3];

SG40[3][1]=0.7071067811865475*SG30[1][1] - 0.7071067811865475*SG30[3][1];
SG40[3][2]=0.7071067811865475*SG30[1][2] - 0.7071067811865475*SG30[3][2];
SG40[3][3]=0.7071067811865475*SG30[1][3] - 0.7071067811865475*SG30[3][3];


SG50[1][1]=S54[1][1]*SG40[1][1] + S54[1][3]*SG40[3][1];
SG50[1][2]=S54[1][1]*SG40[1][2] + S54[1][3]*SG40[3][2];
SG50[1][3]=S54[1][1]*SG40[1][3] + S54[1][3]*SG40[3][3];

SG50[2][1]=S54[2][1]*SG40[1][1] + S54[2][3]*SG40[3][1];
SG50[2][2]=S54[2][1]*SG40[1][2] + S54[2][3]*SG40[3][2];
SG50[2][3]=S54[2][1]*SG40[1][3] + S54[2][3]*SG40[3][3];

SG50[3][1]=-SG40[2][1];
SG50[3][2]=-SG40[2][2];
SG50[3][3]=-SG40[2][3];


SG60[1][1]=S65[1][1]*SG50[1][1] + S65[1][3]*SG50[3][1];
SG60[1][2]=S65[1][1]*SG50[1][2] + S65[1][3]*SG50[3][2];
SG60[1][3]=S65[1][1]*SG50[1][3] + S65[1][3]*SG50[3][3];

SG60[2][1]=S65[2][1]*SG50[1][1] + S65[2][3]*SG50[3][1];
SG60[2][2]=S65[2][1]*SG50[1][2] + S65[2][3]*SG50[3][2];
SG60[2][3]=S65[2][1]*SG50[1][3] + S65[2][3]*SG50[3][3];

SG60[3][1]=SG50[2][1];
SG60[3][2]=SG50[2][2];
SG60[3][3]=SG50[2][3];


SG70[1][1]=S76[1][2]*SG60[2][1] + S76[1][3]*SG60[3][1];
SG70[1][2]=S76[1][2]*SG60[2][2] + S76[1][3]*SG60[3][2];
SG70[1][3]=S76[1][2]*SG60[2][3] + S76[1][3]*SG60[3][3];

SG70[2][1]=S76[2][2]*SG60[2][1] + S76[2][3]*SG60[3][1];
SG70[2][2]=S76[2][2]*SG60[2][2] + S76[2][3]*SG60[3][2];
SG70[2][3]=S76[2][2]*SG60[2][3] + S76[2][3]*SG60[3][3];

SG70[3][1]=SG60[1][1];
SG70[3][2]=SG60[1][2];
SG70[3][3]=SG60[1][3];


SG80[1][1]=S87[1][1]*SG70[1][1] + S87[1][3]*SG70[3][1];
SG80[1][2]=S87[1][1]*SG70[1][2] + S87[1][3]*SG70[3][2];
SG80[1][3]=S87[1][1]*SG70[1][3] + S87[1][3]*SG70[3][3];

SG80[2][1]=S87[2][1]*SG70[1][1] + S87[2][3]*SG70[3][1];
SG80[2][2]=S87[2][1]*SG70[1][2] + S87[2][3]*SG70[3][2];
SG80[2][3]=S87[2][1]*SG70[1][3] + S87[2][3]*SG70[3][3];

SG80[3][1]=SG70[2][1];
SG80[3][2]=SG70[2][2];
SG80[3][3]=SG70[2][3];


SG90[1][1]=S98[1][2]*SG80[2][1] + S98[1][3]*SG80[3][1];
SG90[1][2]=S98[1][2]*SG80[2][2] + S98[1][3]*SG80[3][2];
SG90[1][3]=S98[1][2]*SG80[2][3] + S98[1][3]*SG80[3][3];

SG90[2][1]=S98[2][2]*SG80[2][1] + S98[2][3]*SG80[3][1];
SG90[2][2]=S98[2][2]*SG80[2][2] + S98[2][3]*SG80[3][2];
SG90[2][3]=S98[2][2]*SG80[2][3] + S98[2][3]*SG80[3][3];

SG90[3][1]=SG80[1][1];
SG90[3][2]=SG80[1][2];
SG90[3][3]=SG80[1][3];


SG100[1][1]=S109[1][1]*SG90[1][1] + S109[1][3]*SG90[3][1];
SG100[1][2]=S109[1][1]*SG90[1][2] + S109[1][3]*SG90[3][2];
SG100[1][3]=S109[1][1]*SG90[1][3] + S109[1][3]*SG90[3][3];

SG100[2][1]=S109[2][1]*SG90[1][1] + S109[2][3]*SG90[3][1];
SG100[2][2]=S109[2][1]*SG90[1][2] + S109[2][3]*SG90[3][2];
SG100[2][3]=S109[2][1]*SG90[1][3] + S109[2][3]*SG90[3][3];

SG100[3][1]=-SG90[2][1];
SG100[3][2]=-SG90[2][2];
SG100[3][3]=-SG90[2][3];


SG110[1][1]=S1110[1][1]*SG100[1][1] + S1110[1][2]*SG100[2][1] + S1110[1][3]*SG100[3][1];
SG110[1][2]=S1110[1][1]*SG100[1][2] + S1110[1][2]*SG100[2][2] + S1110[1][3]*SG100[3][2];
SG110[1][3]=S1110[1][1]*SG100[1][3] + S1110[1][2]*SG100[2][3] + S1110[1][3]*SG100[3][3];

SG110[2][1]=S1110[2][1]*SG100[1][1] + S1110[2][2]*SG100[2][1] + S1110[2][3]*SG100[3][1];
SG110[2][2]=S1110[2][1]*SG100[1][2] + S1110[2][2]*SG100[2][2] + S1110[2][3]*SG100[3][2];
SG110[2][3]=S1110[2][1]*SG100[1][3] + S1110[2][2]*SG100[2][3] + S1110[2][3]*SG100[3][3];

SG110[3][1]=S1110[3][1]*SG100[1][1] + S1110[3][2]*SG100[2][1] + S1110[3][3]*SG100[3][1];
SG110[3][2]=S1110[3][1]*SG100[1][2] + S1110[3][2]*SG100[2][2] + S1110[3][3]*SG100[3][2];
SG110[3][3]=S1110[3][1]*SG100[1][3] + S1110[3][2]*SG100[2][3] + S1110[3][3]*SG100[3][3];


SG120[1][1]=S1210[1][1]*SG100[1][1] + S1210[1][2]*SG100[2][1] + S1210[1][3]*SG100[3][1];
SG120[1][2]=S1210[1][1]*SG100[1][2] + S1210[1][2]*SG100[2][2] + S1210[1][3]*SG100[3][2];
SG120[1][3]=S1210[1][1]*SG100[1][3] + S1210[1][2]*SG100[2][3] + S1210[1][3]*SG100[3][3];

SG120[2][1]=S1210[2][1]*SG100[1][1] + S1210[2][2]*SG100[2][1] + S1210[2][3]*SG100[3][1];
SG120[2][2]=S1210[2][1]*SG100[1][2] + S1210[2][2]*SG100[2][2] + S1210[2][3]*SG100[3][2];
SG120[2][3]=S1210[2][1]*SG100[1][3] + S1210[2][2]*SG100[2][3] + S1210[2][3]*SG100[3][3];

SG120[3][1]=S1210[3][1]*SG100[1][1] + S1210[3][2]*SG100[2][1] + S1210[3][3]*SG100[3][1];
SG120[3][2]=S1210[3][1]*SG100[1][2] + S1210[3][2]*SG100[2][2] + S1210[3][3]*SG100[3][2];
SG120[3][3]=S1210[3][1]*SG100[1][3] + S1210[3][2]*SG100[2][3] + S1210[3][3]*SG100[3][3];


SG130[1][1]=S1312[1][1]*SG120[1][1] + S1312[1][2]*SG120[2][1];
SG130[1][2]=S1312[1][1]*SG120[1][2] + S1312[1][2]*SG120[2][2];
SG130[1][3]=S1312[1][1]*SG120[1][3] + S1312[1][2]*SG120[2][3];

SG130[2][1]=S1312[2][1]*SG120[1][1] + S1312[2][2]*SG120[2][1];
SG130[2][2]=S1312[2][1]*SG120[1][2] + S1312[2][2]*SG120[2][2];
SG130[2][3]=S1312[2][1]*SG120[1][3] + S1312[2][2]*SG120[2][3];

SG130[3][1]=SG120[3][1];
SG130[3][2]=SG120[3][2];
SG130[3][3]=SG120[3][3];


SG140[1][1]=S1413[1][1]*SG130[1][1] + S1413[1][2]*SG130[2][1];
SG140[1][2]=S1413[1][1]*SG130[1][2] + S1413[1][2]*SG130[2][2];
SG140[1][3]=S1413[1][1]*SG130[1][3] + S1413[1][2]*SG130[2][3];

SG140[2][1]=S1413[2][1]*SG130[1][1] + S1413[2][2]*SG130[2][1];
SG140[2][2]=S1413[2][1]*SG130[1][2] + S1413[2][2]*SG130[2][2];
SG140[2][3]=S1413[2][1]*SG130[1][3] + S1413[2][2]*SG130[2][3];

SG140[3][1]=SG130[3][1];
SG140[3][2]=SG130[3][2];
SG140[3][3]=SG130[3][3];


SG150[1][1]=SG140[1][1];
SG150[1][2]=SG140[1][2];
SG150[1][3]=SG140[1][3];

SG150[2][1]=SG140[2][1];
SG150[2][2]=SG140[2][2];
SG150[2][3]=SG140[2][3];

SG150[3][1]=SG140[3][1];
SG150[3][2]=SG140[3][2];
SG150[3][3]=SG140[3][3];


SG160[1][1]=S1610[1][1]*SG100[1][1] + S1610[1][2]*SG100[2][1] + S1610[1][3]*SG100[3][1];
SG160[1][2]=S1610[1][1]*SG100[1][2] + S1610[1][2]*SG100[2][2] + S1610[1][3]*SG100[3][2];
SG160[1][3]=S1610[1][1]*SG100[1][3] + S1610[1][2]*SG100[2][3] + S1610[1][3]*SG100[3][3];

SG160[2][1]=S1610[2][1]*SG100[1][1] + S1610[2][2]*SG100[2][1] + S1610[2][3]*SG100[3][1];
SG160[2][2]=S1610[2][1]*SG100[1][2] + S1610[2][2]*SG100[2][2] + S1610[2][3]*SG100[3][2];
SG160[2][3]=S1610[2][1]*SG100[1][3] + S1610[2][2]*SG100[2][3] + S1610[2][3]*SG100[3][3];

SG160[3][1]=S1610[3][2]*SG100[2][1] + S1610[3][3]*SG100[3][1];
SG160[3][2]=S1610[3][2]*SG100[2][2] + S1610[3][3]*SG100[3][2];
SG160[3][3]=S1610[3][2]*SG100[2][3] + S1610[3][3]*SG100[3][3];


SG170[1][1]=S1716[1][1]*SG160[1][1] + S1716[1][2]*SG160[2][1];
SG170[1][2]=S1716[1][1]*SG160[1][2] + S1716[1][2]*SG160[2][2];
SG170[1][3]=S1716[1][1]*SG160[1][3] + S1716[1][2]*SG160[2][3];

SG170[2][1]=S1716[2][1]*SG160[1][1] + S1716[2][2]*SG160[2][1];
SG170[2][2]=S1716[2][1]*SG160[1][2] + S1716[2][2]*SG160[2][2];
SG170[2][3]=S1716[2][1]*SG160[1][3] + S1716[2][2]*SG160[2][3];

SG170[3][1]=SG160[3][1];
SG170[3][2]=SG160[3][2];
SG170[3][3]=SG160[3][3];


SG180[1][1]=S1817[1][1]*SG170[1][1] + S1817[1][2]*SG170[2][1];
SG180[1][2]=S1817[1][1]*SG170[1][2] + S1817[1][2]*SG170[2][2];
SG180[1][3]=S1817[1][1]*SG170[1][3] + S1817[1][2]*SG170[2][3];

SG180[2][1]=S1817[2][1]*SG170[1][1] + S1817[2][2]*SG170[2][1];
SG180[2][2]=S1817[2][1]*SG170[1][2] + S1817[2][2]*SG170[2][2];
SG180[2][3]=S1817[2][1]*SG170[1][3] + S1817[2][2]*SG170[2][3];

SG180[3][1]=SG170[3][1];
SG180[3][2]=SG170[3][2];
SG180[3][3]=SG170[3][3];


SG190[1][1]=SG180[1][1];
SG190[1][2]=SG180[1][2];
SG190[1][3]=SG180[1][3];

SG190[2][1]=SG180[2][1];
SG190[2][2]=SG180[2][2];
SG190[2][3]=SG180[2][3];

SG190[3][1]=SG180[3][1];
SG190[3][2]=SG180[3][2];
SG190[3][3]=SG180[3][3];


SG200[1][1]=S2010[1][1]*SG100[1][1] + S2010[1][2]*SG100[2][1] + S2010[1][3]*SG100[3][1];
SG200[1][2]=S2010[1][1]*SG100[1][2] + S2010[1][2]*SG100[2][2] + S2010[1][3]*SG100[3][2];
SG200[1][3]=S2010[1][1]*SG100[1][3] + S2010[1][2]*SG100[2][3] + S2010[1][3]*SG100[3][3];

SG200[2][1]=S2010[2][1]*SG100[1][1] + S2010[2][2]*SG100[2][1] + S2010[2][3]*SG100[3][1];
SG200[2][2]=S2010[2][1]*SG100[1][2] + S2010[2][2]*SG100[2][2] + S2010[2][3]*SG100[3][2];
SG200[2][3]=S2010[2][1]*SG100[1][3] + S2010[2][2]*SG100[2][3] + S2010[2][3]*SG100[3][3];

SG200[3][1]=S2010[3][2]*SG100[2][1] + S2010[3][3]*SG100[3][1];
SG200[3][2]=S2010[3][2]*SG100[2][2] + S2010[3][3]*SG100[3][2];
SG200[3][3]=S2010[3][2]*SG100[2][3] + S2010[3][3]*SG100[3][3];


SG210[1][1]=S2120[1][1]*SG200[1][1] + S2120[1][2]*SG200[2][1];
SG210[1][2]=S2120[1][1]*SG200[1][2] + S2120[1][2]*SG200[2][2];
SG210[1][3]=S2120[1][1]*SG200[1][3] + S2120[1][2]*SG200[2][3];

SG210[2][1]=S2120[2][1]*SG200[1][1] + S2120[2][2]*SG200[2][1];
SG210[2][2]=S2120[2][1]*SG200[1][2] + S2120[2][2]*SG200[2][2];
SG210[2][3]=S2120[2][1]*SG200[1][3] + S2120[2][2]*SG200[2][3];

SG210[3][1]=SG200[3][1];
SG210[3][2]=SG200[3][2];
SG210[3][3]=SG200[3][3];


SG220[1][1]=S2221[1][1]*SG210[1][1] + S2221[1][2]*SG210[2][1];
SG220[1][2]=S2221[1][1]*SG210[1][2] + S2221[1][2]*SG210[2][2];
SG220[1][3]=S2221[1][1]*SG210[1][3] + S2221[1][2]*SG210[2][3];

SG220[2][1]=S2221[2][1]*SG210[1][1] + S2221[2][2]*SG210[2][1];
SG220[2][2]=S2221[2][1]*SG210[1][2] + S2221[2][2]*SG210[2][2];
SG220[2][3]=S2221[2][1]*SG210[1][3] + S2221[2][2]*SG210[2][3];

SG220[3][1]=SG210[3][1];
SG220[3][2]=SG210[3][2];
SG220[3][3]=SG210[3][3];


SG230[1][1]=SG220[1][1];
SG230[1][2]=SG220[1][2];
SG230[1][3]=SG220[1][3];

SG230[2][1]=SG220[2][1];
SG230[2][2]=SG220[2][2];
SG230[2][3]=SG220[2][3];

SG230[3][1]=SG220[3][1];
SG230[3][2]=SG220[3][2];
SG230[3][3]=SG220[3][3];


SG240[1][1]=S2410[1][1]*SG100[1][1] + S2410[1][2]*SG100[2][1] + S2410[1][3]*SG100[3][1];
SG240[1][2]=S2410[1][1]*SG100[1][2] + S2410[1][2]*SG100[2][2] + S2410[1][3]*SG100[3][2];
SG240[1][3]=S2410[1][1]*SG100[1][3] + S2410[1][2]*SG100[2][3] + S2410[1][3]*SG100[3][3];

SG240[2][1]=S2410[2][1]*SG100[1][1] + S2410[2][2]*SG100[2][1] + S2410[2][3]*SG100[3][1];
SG240[2][2]=S2410[2][1]*SG100[1][2] + S2410[2][2]*SG100[2][2] + S2410[2][3]*SG100[3][2];
SG240[2][3]=S2410[2][1]*SG100[1][3] + S2410[2][2]*SG100[2][3] + S2410[2][3]*SG100[3][3];

SG240[3][1]=S2410[3][2]*SG100[2][1] + S2410[3][3]*SG100[3][1];
SG240[3][2]=S2410[3][2]*SG100[2][2] + S2410[3][3]*SG100[3][2];
SG240[3][3]=S2410[3][2]*SG100[2][3] + S2410[3][3]*SG100[3][3];


SG250[1][1]=S2524[1][1]*SG240[1][1] + S2524[1][2]*SG240[2][1];
SG250[1][2]=S2524[1][1]*SG240[1][2] + S2524[1][2]*SG240[2][2];
SG250[1][3]=S2524[1][1]*SG240[1][3] + S2524[1][2]*SG240[2][3];

SG250[2][1]=S2524[2][1]*SG240[1][1] + S2524[2][2]*SG240[2][1];
SG250[2][2]=S2524[2][1]*SG240[1][2] + S2524[2][2]*SG240[2][2];
SG250[2][3]=S2524[2][1]*SG240[1][3] + S2524[2][2]*SG240[2][3];

SG250[3][1]=SG240[3][1];
SG250[3][2]=SG240[3][2];
SG250[3][3]=SG240[3][3];


SG260[1][1]=S2625[1][1]*SG250[1][1] + S2625[1][2]*SG250[2][1];
SG260[1][2]=S2625[1][1]*SG250[1][2] + S2625[1][2]*SG250[2][2];
SG260[1][3]=S2625[1][1]*SG250[1][3] + S2625[1][2]*SG250[2][3];

SG260[2][1]=S2625[2][1]*SG250[1][1] + S2625[2][2]*SG250[2][1];
SG260[2][2]=S2625[2][1]*SG250[1][2] + S2625[2][2]*SG250[2][2];
SG260[2][3]=S2625[2][1]*SG250[1][3] + S2625[2][2]*SG250[2][3];

SG260[3][1]=SG250[3][1];
SG260[3][2]=SG250[3][2];
SG260[3][3]=SG250[3][3];


SG270[1][1]=SG260[1][1];
SG270[1][2]=SG260[1][2];
SG270[1][3]=SG260[1][3];

SG270[2][1]=SG260[2][1];
SG270[2][2]=SG260[2][2];
SG270[2][3]=SG260[2][3];

SG270[3][1]=SG260[3][1];
SG270[3][2]=SG260[3][2];
SG270[3][3]=SG260[3][3];


SG280[1][1]=S2810[1][1]*SG100[1][1] + S2810[1][2]*SG100[2][1] + S2810[1][3]*SG100[3][1];
SG280[1][2]=S2810[1][1]*SG100[1][2] + S2810[1][2]*SG100[2][2] + S2810[1][3]*SG100[3][2];
SG280[1][3]=S2810[1][1]*SG100[1][3] + S2810[1][2]*SG100[2][3] + S2810[1][3]*SG100[3][3];

SG280[2][1]=S2810[2][1]*SG100[1][1] + S2810[2][2]*SG100[2][1] + S2810[2][3]*SG100[3][1];
SG280[2][2]=S2810[2][1]*SG100[1][2] + S2810[2][2]*SG100[2][2] + S2810[2][3]*SG100[3][2];
SG280[2][3]=S2810[2][1]*SG100[1][3] + S2810[2][2]*SG100[2][3] + S2810[2][3]*SG100[3][3];

SG280[3][1]=S2810[3][2]*SG100[2][1] + S2810[3][3]*SG100[3][1];
SG280[3][2]=S2810[3][2]*SG100[2][2] + S2810[3][3]*SG100[3][2];
SG280[3][3]=S2810[3][2]*SG100[2][3] + S2810[3][3]*SG100[3][3];


SG290[1][1]=S2928[1][1]*SG280[1][1] + S2928[1][2]*SG280[2][1];
SG290[1][2]=S2928[1][1]*SG280[1][2] + S2928[1][2]*SG280[2][2];
SG290[1][3]=S2928[1][1]*SG280[1][3] + S2928[1][2]*SG280[2][3];

SG290[2][1]=S2928[2][1]*SG280[1][1] + S2928[2][2]*SG280[2][1];
SG290[2][2]=S2928[2][1]*SG280[1][2] + S2928[2][2]*SG280[2][2];
SG290[2][3]=S2928[2][1]*SG280[1][3] + S2928[2][2]*SG280[2][3];

SG290[3][1]=SG280[3][1];
SG290[3][2]=SG280[3][2];
SG290[3][3]=SG280[3][3];


SG300[1][1]=S3029[1][1]*SG290[1][1] + S3029[1][2]*SG290[2][1];
SG300[1][2]=S3029[1][1]*SG290[1][2] + S3029[1][2]*SG290[2][2];
SG300[1][3]=S3029[1][1]*SG290[1][3] + S3029[1][2]*SG290[2][3];

SG300[2][1]=S3029[2][1]*SG290[1][1] + S3029[2][2]*SG290[2][1];
SG300[2][2]=S3029[2][1]*SG290[1][2] + S3029[2][2]*SG290[2][2];
SG300[2][3]=S3029[2][1]*SG290[1][3] + S3029[2][2]*SG290[2][3];

SG300[3][1]=SG290[3][1];
SG300[3][2]=SG290[3][2];
SG300[3][3]=SG290[3][3];


SG310[1][1]=SG300[1][1];
SG310[1][2]=SG300[1][2];
SG310[1][3]=SG300[1][3];

SG310[2][1]=SG300[2][1];
SG310[2][2]=SG300[2][2];
SG310[2][3]=SG300[2][3];

SG310[3][1]=SG300[3][1];
SG310[3][2]=SG300[3][2];
SG310[3][3]=SG300[3][3];


SG320[1][1]=S323[1][1]*SG30[1][1] + S323[1][2]*SG30[2][1] + S323[1][3]*SG30[3][1];
SG320[1][2]=S323[1][1]*SG30[1][2] + S323[1][2]*SG30[2][2] + S323[1][3]*SG30[3][2];
SG320[1][3]=S323[1][1]*SG30[1][3] + S323[1][2]*SG30[2][3] + S323[1][3]*SG30[3][3];

SG320[2][1]=S323[2][1]*SG30[1][1] + S323[2][2]*SG30[2][1] + S323[2][3]*SG30[3][1];
SG320[2][2]=S323[2][1]*SG30[1][2] + S323[2][2]*SG30[2][2] + S323[2][3]*SG30[3][2];
SG320[2][3]=S323[2][1]*SG30[1][3] + S323[2][2]*SG30[2][3] + S323[2][3]*SG30[3][3];

SG320[3][1]=-0.7071067811865475*SG30[1][1] - 0.7071067811865475*SG30[3][1];
SG320[3][2]=-0.7071067811865475*SG30[1][2] - 0.7071067811865475*SG30[3][2];
SG320[3][3]=-0.7071067811865475*SG30[1][3] - 0.7071067811865475*SG30[3][3];


SG330[1][1]=S3332[1][1]*SG320[1][1] + S3332[1][3]*SG320[3][1];
SG330[1][2]=S3332[1][1]*SG320[1][2] + S3332[1][3]*SG320[3][2];
SG330[1][3]=S3332[1][1]*SG320[1][3] + S3332[1][3]*SG320[3][3];

SG330[2][1]=S3332[2][1]*SG320[1][1] + S3332[2][3]*SG320[3][1];
SG330[2][2]=S3332[2][1]*SG320[1][2] + S3332[2][3]*SG320[3][2];
SG330[2][3]=S3332[2][1]*SG320[1][3] + S3332[2][3]*SG320[3][3];

SG330[3][1]=SG320[2][1];
SG330[3][2]=SG320[2][2];
SG330[3][3]=SG320[2][3];


SG340[1][1]=S3433[1][1]*SG330[1][1] + S3433[1][3]*SG330[3][1];
SG340[1][2]=S3433[1][1]*SG330[1][2] + S3433[1][3]*SG330[3][2];
SG340[1][3]=S3433[1][1]*SG330[1][3] + S3433[1][3]*SG330[3][3];

SG340[2][1]=S3433[2][1]*SG330[1][1] + S3433[2][3]*SG330[3][1];
SG340[2][2]=S3433[2][1]*SG330[1][2] + S3433[2][3]*SG330[3][2];
SG340[2][3]=S3433[2][1]*SG330[1][3] + S3433[2][3]*SG330[3][3];

SG340[3][1]=-SG330[2][1];
SG340[3][2]=-SG330[2][2];
SG340[3][3]=-SG330[2][3];


SG350[1][1]=S3534[1][2]*SG340[2][1] + S3534[1][3]*SG340[3][1];
SG350[1][2]=S3534[1][2]*SG340[2][2] + S3534[1][3]*SG340[3][2];
SG350[1][3]=S3534[1][2]*SG340[2][3] + S3534[1][3]*SG340[3][3];

SG350[2][1]=S3534[2][2]*SG340[2][1] + S3534[2][3]*SG340[3][1];
SG350[2][2]=S3534[2][2]*SG340[2][2] + S3534[2][3]*SG340[3][2];
SG350[2][3]=S3534[2][2]*SG340[2][3] + S3534[2][3]*SG340[3][3];

SG350[3][1]=-SG340[1][1];
SG350[3][2]=-SG340[1][2];
SG350[3][3]=-SG340[1][3];


SG360[1][1]=S3635[1][1]*SG350[1][1] + S3635[1][3]*SG350[3][1];
SG360[1][2]=S3635[1][1]*SG350[1][2] + S3635[1][3]*SG350[3][2];
SG360[1][3]=S3635[1][1]*SG350[1][3] + S3635[1][3]*SG350[3][3];

SG360[2][1]=S3635[2][1]*SG350[1][1] + S3635[2][3]*SG350[3][1];
SG360[2][2]=S3635[2][1]*SG350[1][2] + S3635[2][3]*SG350[3][2];
SG360[2][3]=S3635[2][1]*SG350[1][3] + S3635[2][3]*SG350[3][3];

SG360[3][1]=-SG350[2][1];
SG360[3][2]=-SG350[2][2];
SG360[3][3]=-SG350[2][3];


SG370[1][1]=S3736[1][2]*SG360[2][1] + S3736[1][3]*SG360[3][1];
SG370[1][2]=S3736[1][2]*SG360[2][2] + S3736[1][3]*SG360[3][2];
SG370[1][3]=S3736[1][2]*SG360[2][3] + S3736[1][3]*SG360[3][3];

SG370[2][1]=S3736[2][2]*SG360[2][1] + S3736[2][3]*SG360[3][1];
SG370[2][2]=S3736[2][2]*SG360[2][2] + S3736[2][3]*SG360[3][2];
SG370[2][3]=S3736[2][2]*SG360[2][3] + S3736[2][3]*SG360[3][3];

SG370[3][1]=-SG360[1][1];
SG370[3][2]=-SG360[1][2];
SG370[3][3]=-SG360[1][3];


SG380[1][1]=S3837[1][1]*SG370[1][1] + S3837[1][3]*SG370[3][1];
SG380[1][2]=S3837[1][1]*SG370[1][2] + S3837[1][3]*SG370[3][2];
SG380[1][3]=S3837[1][1]*SG370[1][3] + S3837[1][3]*SG370[3][3];

SG380[2][1]=S3837[2][1]*SG370[1][1] + S3837[2][3]*SG370[3][1];
SG380[2][2]=S3837[2][1]*SG370[1][2] + S3837[2][3]*SG370[3][2];
SG380[2][3]=S3837[2][1]*SG370[1][3] + S3837[2][3]*SG370[3][3];

SG380[3][1]=SG370[2][1];
SG380[3][2]=SG370[2][2];
SG380[3][3]=SG370[2][3];


SG390[1][1]=S3938[1][1]*SG380[1][1] + S3938[1][2]*SG380[2][1] + S3938[1][3]*SG380[3][1];
SG390[1][2]=S3938[1][1]*SG380[1][2] + S3938[1][2]*SG380[2][2] + S3938[1][3]*SG380[3][2];
SG390[1][3]=S3938[1][1]*SG380[1][3] + S3938[1][2]*SG380[2][3] + S3938[1][3]*SG380[3][3];

SG390[2][1]=S3938[2][1]*SG380[1][1] + S3938[2][2]*SG380[2][1] + S3938[2][3]*SG380[3][1];
SG390[2][2]=S3938[2][1]*SG380[1][2] + S3938[2][2]*SG380[2][2] + S3938[2][3]*SG380[3][2];
SG390[2][3]=S3938[2][1]*SG380[1][3] + S3938[2][2]*SG380[2][3] + S3938[2][3]*SG380[3][3];

SG390[3][1]=S3938[3][1]*SG380[1][1] + S3938[3][2]*SG380[2][1] + S3938[3][3]*SG380[3][1];
SG390[3][2]=S3938[3][1]*SG380[1][2] + S3938[3][2]*SG380[2][2] + S3938[3][3]*SG380[3][2];
SG390[3][3]=S3938[3][1]*SG380[1][3] + S3938[3][2]*SG380[2][3] + S3938[3][3]*SG380[3][3];


SG400[1][1]=S4038[1][1]*SG380[1][1] + S4038[1][2]*SG380[2][1] + S4038[1][3]*SG380[3][1];
SG400[1][2]=S4038[1][1]*SG380[1][2] + S4038[1][2]*SG380[2][2] + S4038[1][3]*SG380[3][2];
SG400[1][3]=S4038[1][1]*SG380[1][3] + S4038[1][2]*SG380[2][3] + S4038[1][3]*SG380[3][3];

SG400[2][1]=S4038[2][1]*SG380[1][1] + S4038[2][2]*SG380[2][1] + S4038[2][3]*SG380[3][1];
SG400[2][2]=S4038[2][1]*SG380[1][2] + S4038[2][2]*SG380[2][2] + S4038[2][3]*SG380[3][2];
SG400[2][3]=S4038[2][1]*SG380[1][3] + S4038[2][2]*SG380[2][3] + S4038[2][3]*SG380[3][3];

SG400[3][1]=S4038[3][1]*SG380[1][1] + S4038[3][2]*SG380[2][1] + S4038[3][3]*SG380[3][1];
SG400[3][2]=S4038[3][1]*SG380[1][2] + S4038[3][2]*SG380[2][2] + S4038[3][3]*SG380[3][2];
SG400[3][3]=S4038[3][1]*SG380[1][3] + S4038[3][2]*SG380[2][3] + S4038[3][3]*SG380[3][3];


SG410[1][1]=S4140[1][1]*SG400[1][1] + S4140[1][2]*SG400[2][1];
SG410[1][2]=S4140[1][1]*SG400[1][2] + S4140[1][2]*SG400[2][2];
SG410[1][3]=S4140[1][1]*SG400[1][3] + S4140[1][2]*SG400[2][3];

SG410[2][1]=S4140[2][1]*SG400[1][1] + S4140[2][2]*SG400[2][1];
SG410[2][2]=S4140[2][1]*SG400[1][2] + S4140[2][2]*SG400[2][2];
SG410[2][3]=S4140[2][1]*SG400[1][3] + S4140[2][2]*SG400[2][3];

SG410[3][1]=SG400[3][1];
SG410[3][2]=SG400[3][2];
SG410[3][3]=SG400[3][3];


SG420[1][1]=S4241[1][1]*SG410[1][1] + S4241[1][2]*SG410[2][1];
SG420[1][2]=S4241[1][1]*SG410[1][2] + S4241[1][2]*SG410[2][2];
SG420[1][3]=S4241[1][1]*SG410[1][3] + S4241[1][2]*SG410[2][3];

SG420[2][1]=S4241[2][1]*SG410[1][1] + S4241[2][2]*SG410[2][1];
SG420[2][2]=S4241[2][1]*SG410[1][2] + S4241[2][2]*SG410[2][2];
SG420[2][3]=S4241[2][1]*SG410[1][3] + S4241[2][2]*SG410[2][3];

SG420[3][1]=SG410[3][1];
SG420[3][2]=SG410[3][2];
SG420[3][3]=SG410[3][3];


SG430[1][1]=SG420[1][1];
SG430[1][2]=SG420[1][2];
SG430[1][3]=SG420[1][3];

SG430[2][1]=SG420[2][1];
SG430[2][2]=SG420[2][2];
SG430[2][3]=SG420[2][3];

SG430[3][1]=SG420[3][1];
SG430[3][2]=SG420[3][2];
SG430[3][3]=SG420[3][3];


SG440[1][1]=S4438[1][1]*SG380[1][1] + S4438[1][2]*SG380[2][1] + S4438[1][3]*SG380[3][1];
SG440[1][2]=S4438[1][1]*SG380[1][2] + S4438[1][2]*SG380[2][2] + S4438[1][3]*SG380[3][2];
SG440[1][3]=S4438[1][1]*SG380[1][3] + S4438[1][2]*SG380[2][3] + S4438[1][3]*SG380[3][3];

SG440[2][1]=S4438[2][1]*SG380[1][1] + S4438[2][2]*SG380[2][1] + S4438[2][3]*SG380[3][1];
SG440[2][2]=S4438[2][1]*SG380[1][2] + S4438[2][2]*SG380[2][2] + S4438[2][3]*SG380[3][2];
SG440[2][3]=S4438[2][1]*SG380[1][3] + S4438[2][2]*SG380[2][3] + S4438[2][3]*SG380[3][3];

SG440[3][1]=S4438[3][2]*SG380[2][1] + S4438[3][3]*SG380[3][1];
SG440[3][2]=S4438[3][2]*SG380[2][2] + S4438[3][3]*SG380[3][2];
SG440[3][3]=S4438[3][2]*SG380[2][3] + S4438[3][3]*SG380[3][3];


SG450[1][1]=S4544[1][1]*SG440[1][1] + S4544[1][2]*SG440[2][1];
SG450[1][2]=S4544[1][1]*SG440[1][2] + S4544[1][2]*SG440[2][2];
SG450[1][3]=S4544[1][1]*SG440[1][3] + S4544[1][2]*SG440[2][3];

SG450[2][1]=S4544[2][1]*SG440[1][1] + S4544[2][2]*SG440[2][1];
SG450[2][2]=S4544[2][1]*SG440[1][2] + S4544[2][2]*SG440[2][2];
SG450[2][3]=S4544[2][1]*SG440[1][3] + S4544[2][2]*SG440[2][3];

SG450[3][1]=SG440[3][1];
SG450[3][2]=SG440[3][2];
SG450[3][3]=SG440[3][3];


SG460[1][1]=S4645[1][1]*SG450[1][1] + S4645[1][2]*SG450[2][1];
SG460[1][2]=S4645[1][1]*SG450[1][2] + S4645[1][2]*SG450[2][2];
SG460[1][3]=S4645[1][1]*SG450[1][3] + S4645[1][2]*SG450[2][3];

SG460[2][1]=S4645[2][1]*SG450[1][1] + S4645[2][2]*SG450[2][1];
SG460[2][2]=S4645[2][1]*SG450[1][2] + S4645[2][2]*SG450[2][2];
SG460[2][3]=S4645[2][1]*SG450[1][3] + S4645[2][2]*SG450[2][3];

SG460[3][1]=SG450[3][1];
SG460[3][2]=SG450[3][2];
SG460[3][3]=SG450[3][3];


SG470[1][1]=SG460[1][1];
SG470[1][2]=SG460[1][2];
SG470[1][3]=SG460[1][3];

SG470[2][1]=SG460[2][1];
SG470[2][2]=SG460[2][2];
SG470[2][3]=SG460[2][3];

SG470[3][1]=SG460[3][1];
SG470[3][2]=SG460[3][2];
SG470[3][3]=SG460[3][3];


SG480[1][1]=S4838[1][1]*SG380[1][1] + S4838[1][2]*SG380[2][1] + S4838[1][3]*SG380[3][1];
SG480[1][2]=S4838[1][1]*SG380[1][2] + S4838[1][2]*SG380[2][2] + S4838[1][3]*SG380[3][2];
SG480[1][3]=S4838[1][1]*SG380[1][3] + S4838[1][2]*SG380[2][3] + S4838[1][3]*SG380[3][3];

SG480[2][1]=S4838[2][1]*SG380[1][1] + S4838[2][2]*SG380[2][1] + S4838[2][3]*SG380[3][1];
SG480[2][2]=S4838[2][1]*SG380[1][2] + S4838[2][2]*SG380[2][2] + S4838[2][3]*SG380[3][2];
SG480[2][3]=S4838[2][1]*SG380[1][3] + S4838[2][2]*SG380[2][3] + S4838[2][3]*SG380[3][3];

SG480[3][1]=S4838[3][2]*SG380[2][1] + S4838[3][3]*SG380[3][1];
SG480[3][2]=S4838[3][2]*SG380[2][2] + S4838[3][3]*SG380[3][2];
SG480[3][3]=S4838[3][2]*SG380[2][3] + S4838[3][3]*SG380[3][3];


SG490[1][1]=S4948[1][1]*SG480[1][1] + S4948[1][2]*SG480[2][1];
SG490[1][2]=S4948[1][1]*SG480[1][2] + S4948[1][2]*SG480[2][2];
SG490[1][3]=S4948[1][1]*SG480[1][3] + S4948[1][2]*SG480[2][3];

SG490[2][1]=S4948[2][1]*SG480[1][1] + S4948[2][2]*SG480[2][1];
SG490[2][2]=S4948[2][1]*SG480[1][2] + S4948[2][2]*SG480[2][2];
SG490[2][3]=S4948[2][1]*SG480[1][3] + S4948[2][2]*SG480[2][3];

SG490[3][1]=SG480[3][1];
SG490[3][2]=SG480[3][2];
SG490[3][3]=SG480[3][3];


SG500[1][1]=S5049[1][1]*SG490[1][1] + S5049[1][2]*SG490[2][1];
SG500[1][2]=S5049[1][1]*SG490[1][2] + S5049[1][2]*SG490[2][2];
SG500[1][3]=S5049[1][1]*SG490[1][3] + S5049[1][2]*SG490[2][3];

SG500[2][1]=S5049[2][1]*SG490[1][1] + S5049[2][2]*SG490[2][1];
SG500[2][2]=S5049[2][1]*SG490[1][2] + S5049[2][2]*SG490[2][2];
SG500[2][3]=S5049[2][1]*SG490[1][3] + S5049[2][2]*SG490[2][3];

SG500[3][1]=SG490[3][1];
SG500[3][2]=SG490[3][2];
SG500[3][3]=SG490[3][3];


SG510[1][1]=SG500[1][1];
SG510[1][2]=SG500[1][2];
SG510[1][3]=SG500[1][3];

SG510[2][1]=SG500[2][1];
SG510[2][2]=SG500[2][2];
SG510[2][3]=SG500[2][3];

SG510[3][1]=SG500[3][1];
SG510[3][2]=SG500[3][2];
SG510[3][3]=SG500[3][3];


SG520[1][1]=S5238[1][1]*SG380[1][1] + S5238[1][2]*SG380[2][1] + S5238[1][3]*SG380[3][1];
SG520[1][2]=S5238[1][1]*SG380[1][2] + S5238[1][2]*SG380[2][2] + S5238[1][3]*SG380[3][2];
SG520[1][3]=S5238[1][1]*SG380[1][3] + S5238[1][2]*SG380[2][3] + S5238[1][3]*SG380[3][3];

SG520[2][1]=S5238[2][1]*SG380[1][1] + S5238[2][2]*SG380[2][1] + S5238[2][3]*SG380[3][1];
SG520[2][2]=S5238[2][1]*SG380[1][2] + S5238[2][2]*SG380[2][2] + S5238[2][3]*SG380[3][2];
SG520[2][3]=S5238[2][1]*SG380[1][3] + S5238[2][2]*SG380[2][3] + S5238[2][3]*SG380[3][3];

SG520[3][1]=S5238[3][2]*SG380[2][1] + S5238[3][3]*SG380[3][1];
SG520[3][2]=S5238[3][2]*SG380[2][2] + S5238[3][3]*SG380[3][2];
SG520[3][3]=S5238[3][2]*SG380[2][3] + S5238[3][3]*SG380[3][3];


SG530[1][1]=S5352[1][1]*SG520[1][1] + S5352[1][2]*SG520[2][1];
SG530[1][2]=S5352[1][1]*SG520[1][2] + S5352[1][2]*SG520[2][2];
SG530[1][3]=S5352[1][1]*SG520[1][3] + S5352[1][2]*SG520[2][3];

SG530[2][1]=S5352[2][1]*SG520[1][1] + S5352[2][2]*SG520[2][1];
SG530[2][2]=S5352[2][1]*SG520[1][2] + S5352[2][2]*SG520[2][2];
SG530[2][3]=S5352[2][1]*SG520[1][3] + S5352[2][2]*SG520[2][3];

SG530[3][1]=SG520[3][1];
SG530[3][2]=SG520[3][2];
SG530[3][3]=SG520[3][3];


SG540[1][1]=S5453[1][1]*SG530[1][1] + S5453[1][2]*SG530[2][1];
SG540[1][2]=S5453[1][1]*SG530[1][2] + S5453[1][2]*SG530[2][2];
SG540[1][3]=S5453[1][1]*SG530[1][3] + S5453[1][2]*SG530[2][3];

SG540[2][1]=S5453[2][1]*SG530[1][1] + S5453[2][2]*SG530[2][1];
SG540[2][2]=S5453[2][1]*SG530[1][2] + S5453[2][2]*SG530[2][2];
SG540[2][3]=S5453[2][1]*SG530[1][3] + S5453[2][2]*SG530[2][3];

SG540[3][1]=SG530[3][1];
SG540[3][2]=SG530[3][2];
SG540[3][3]=SG530[3][3];


SG550[1][1]=SG540[1][1];
SG550[1][2]=SG540[1][2];
SG550[1][3]=SG540[1][3];

SG550[2][1]=SG540[2][1];
SG550[2][2]=SG540[2][2];
SG550[2][3]=SG540[2][3];

SG550[3][1]=SG540[3][1];
SG550[3][2]=SG540[3][2];
SG550[3][3]=SG540[3][3];


SG560[1][1]=S5638[1][1]*SG380[1][1] + S5638[1][2]*SG380[2][1] + S5638[1][3]*SG380[3][1];
SG560[1][2]=S5638[1][1]*SG380[1][2] + S5638[1][2]*SG380[2][2] + S5638[1][3]*SG380[3][2];
SG560[1][3]=S5638[1][1]*SG380[1][3] + S5638[1][2]*SG380[2][3] + S5638[1][3]*SG380[3][3];

SG560[2][1]=S5638[2][1]*SG380[1][1] + S5638[2][2]*SG380[2][1] + S5638[2][3]*SG380[3][1];
SG560[2][2]=S5638[2][1]*SG380[1][2] + S5638[2][2]*SG380[2][2] + S5638[2][3]*SG380[3][2];
SG560[2][3]=S5638[2][1]*SG380[1][3] + S5638[2][2]*SG380[2][3] + S5638[2][3]*SG380[3][3];

SG560[3][1]=S5638[3][2]*SG380[2][1] + S5638[3][3]*SG380[3][1];
SG560[3][2]=S5638[3][2]*SG380[2][2] + S5638[3][3]*SG380[3][2];
SG560[3][3]=S5638[3][2]*SG380[2][3] + S5638[3][3]*SG380[3][3];


SG570[1][1]=S5756[1][1]*SG560[1][1] + S5756[1][2]*SG560[2][1];
SG570[1][2]=S5756[1][1]*SG560[1][2] + S5756[1][2]*SG560[2][2];
SG570[1][3]=S5756[1][1]*SG560[1][3] + S5756[1][2]*SG560[2][3];

SG570[2][1]=S5756[2][1]*SG560[1][1] + S5756[2][2]*SG560[2][1];
SG570[2][2]=S5756[2][1]*SG560[1][2] + S5756[2][2]*SG560[2][2];
SG570[2][3]=S5756[2][1]*SG560[1][3] + S5756[2][2]*SG560[2][3];

SG570[3][1]=SG560[3][1];
SG570[3][2]=SG560[3][2];
SG570[3][3]=SG560[3][3];


SG580[1][1]=S5857[1][1]*SG570[1][1] + S5857[1][2]*SG570[2][1];
SG580[1][2]=S5857[1][1]*SG570[1][2] + S5857[1][2]*SG570[2][2];
SG580[1][3]=S5857[1][1]*SG570[1][3] + S5857[1][2]*SG570[2][3];

SG580[2][1]=S5857[2][1]*SG570[1][1] + S5857[2][2]*SG570[2][1];
SG580[2][2]=S5857[2][1]*SG570[1][2] + S5857[2][2]*SG570[2][2];
SG580[2][3]=S5857[2][1]*SG570[1][3] + S5857[2][2]*SG570[2][3];

SG580[3][1]=SG570[3][1];
SG580[3][2]=SG570[3][2];
SG580[3][3]=SG570[3][3];


SG590[1][1]=SG580[1][1];
SG590[1][2]=SG580[1][2];
SG590[1][3]=SG580[1][3];

SG590[2][1]=SG580[2][1];
SG590[2][2]=SG580[2][2];
SG590[2][3]=SG580[2][3];

SG590[3][1]=SG580[3][1];
SG590[3][2]=SG580[3][2];
SG590[3][3]=SG580[3][3];


SG600[1][1]=S603[1][1]*SG30[1][1] + S603[1][2]*SG30[2][1];
SG600[1][2]=S603[1][1]*SG30[1][2] + S603[1][2]*SG30[2][2];
SG600[1][3]=S603[1][1]*SG30[1][3] + S603[1][2]*SG30[2][3];

SG600[2][1]=S603[2][1]*SG30[1][1] + S603[2][2]*SG30[2][1];
SG600[2][2]=S603[2][1]*SG30[1][2] + S603[2][2]*SG30[2][2];
SG600[2][3]=S603[2][1]*SG30[1][3] + S603[2][2]*SG30[2][3];

SG600[3][1]=SG30[3][1];
SG600[3][2]=SG30[3][2];
SG600[3][3]=SG30[3][3];


SG610[1][1]=S6160[1][2]*SG600[2][1] + S6160[1][3]*SG600[3][1];
SG610[1][2]=S6160[1][2]*SG600[2][2] + S6160[1][3]*SG600[3][2];
SG610[1][3]=S6160[1][2]*SG600[2][3] + S6160[1][3]*SG600[3][3];

SG610[2][1]=S6160[2][2]*SG600[2][1] + S6160[2][3]*SG600[3][1];
SG610[2][2]=S6160[2][2]*SG600[2][2] + S6160[2][3]*SG600[3][2];
SG610[2][3]=S6160[2][2]*SG600[2][3] + S6160[2][3]*SG600[3][3];

SG610[3][1]=SG600[1][1];
SG610[3][2]=SG600[1][2];
SG610[3][3]=SG600[1][3];


SG620[1][1]=S6261[1][1]*SG610[1][1] + S6261[1][3]*SG610[3][1];
SG620[1][2]=S6261[1][1]*SG610[1][2] + S6261[1][3]*SG610[3][2];
SG620[1][3]=S6261[1][1]*SG610[1][3] + S6261[1][3]*SG610[3][3];

SG620[2][1]=S6261[2][1]*SG610[1][1] + S6261[2][3]*SG610[3][1];
SG620[2][2]=S6261[2][1]*SG610[1][2] + S6261[2][3]*SG610[3][2];
SG620[2][3]=S6261[2][1]*SG610[1][3] + S6261[2][3]*SG610[3][3];

SG620[3][1]=SG610[2][1];
SG620[3][2]=SG610[2][2];
SG620[3][3]=SG610[2][3];


SG630[1][1]=S6362[1][1]*SG620[1][1] + S6362[1][2]*SG620[2][1];
SG630[1][2]=S6362[1][1]*SG620[1][2] + S6362[1][2]*SG620[2][2];
SG630[1][3]=S6362[1][1]*SG620[1][3] + S6362[1][2]*SG620[2][3];

SG630[2][1]=S6362[2][1]*SG620[1][1] + S6362[2][2]*SG620[2][1];
SG630[2][2]=S6362[2][1]*SG620[1][2] + S6362[2][2]*SG620[2][2];
SG630[2][3]=S6362[2][1]*SG620[1][3] + S6362[2][2]*SG620[2][3];

SG630[3][1]=SG620[3][1];
SG630[3][2]=SG620[3][2];
SG630[3][3]=SG620[3][3];


SG640[1][1]=S6463[1][2]*SG630[2][1] + S6463[1][3]*SG630[3][1];
SG640[1][2]=S6463[1][2]*SG630[2][2] + S6463[1][3]*SG630[3][2];
SG640[1][3]=S6463[1][2]*SG630[2][3] + S6463[1][3]*SG630[3][3];

SG640[2][1]=S6463[2][2]*SG630[2][1] + S6463[2][3]*SG630[3][1];
SG640[2][2]=S6463[2][2]*SG630[2][2] + S6463[2][3]*SG630[3][2];
SG640[2][3]=S6463[2][2]*SG630[2][3] + S6463[2][3]*SG630[3][3];

SG640[3][1]=-SG630[1][1];
SG640[3][2]=-SG630[1][2];
SG640[3][3]=-SG630[1][3];


SG650[1][1]=-SG640[3][1];
SG650[1][2]=-SG640[3][2];
SG650[1][3]=-SG640[3][3];

SG650[2][1]=-SG640[2][1];
SG650[2][2]=-SG640[2][2];
SG650[2][3]=-SG640[2][3];

SG650[3][1]=-SG640[1][1];
SG650[3][2]=-SG640[1][2];
SG650[3][3]=-SG640[1][3];


SG660[1][1]=S6662[1][1]*SG620[1][1] + S6662[1][2]*SG620[2][1];
SG660[1][2]=S6662[1][1]*SG620[1][2] + S6662[1][2]*SG620[2][2];
SG660[1][3]=S6662[1][1]*SG620[1][3] + S6662[1][2]*SG620[2][3];

SG660[2][1]=S6662[2][1]*SG620[1][1] + S6662[2][2]*SG620[2][1];
SG660[2][2]=S6662[2][1]*SG620[1][2] + S6662[2][2]*SG620[2][2];
SG660[2][3]=S6662[2][1]*SG620[1][3] + S6662[2][2]*SG620[2][3];

SG660[3][1]=SG620[3][1];
SG660[3][2]=SG620[3][2];
SG660[3][3]=SG620[3][3];


SG670[1][1]=S6766[1][2]*SG660[2][1] + S6766[1][3]*SG660[3][1];
SG670[1][2]=S6766[1][2]*SG660[2][2] + S6766[1][3]*SG660[3][2];
SG670[1][3]=S6766[1][2]*SG660[2][3] + S6766[1][3]*SG660[3][3];

SG670[2][1]=S6766[2][2]*SG660[2][1] + S6766[2][3]*SG660[3][1];
SG670[2][2]=S6766[2][2]*SG660[2][2] + S6766[2][3]*SG660[3][2];
SG670[2][3]=S6766[2][2]*SG660[2][3] + S6766[2][3]*SG660[3][3];

SG670[3][1]=-SG660[1][1];
SG670[3][2]=-SG660[1][2];
SG670[3][3]=-SG660[1][3];


SG680[1][1]=-SG670[3][1];
SG680[1][2]=-SG670[3][2];
SG680[1][3]=-SG670[3][3];

SG680[2][1]=-SG670[2][1];
SG680[2][2]=-SG670[2][2];
SG680[2][3]=-SG670[2][3];

SG680[3][1]=-SG670[1][1];
SG680[3][2]=-SG670[1][2];
SG680[3][3]=-SG670[1][3];


SG690[1][1]=SG620[1][1];
SG690[1][2]=SG620[1][2];
SG690[1][3]=SG620[1][3];

SG690[2][1]=-SG620[2][1];
SG690[2][2]=-SG620[2][2];
SG690[2][3]=-SG620[2][3];

SG690[3][1]=-SG620[3][1];
SG690[3][2]=-SG620[3][2];
SG690[3][3]=-SG620[3][3];


SG700[1][1]=S00[1][1]*S700[1][1] + S00[3][1]*S700[1][3];
SG700[1][2]=S00[1][2]*S700[1][1] + S00[3][2]*S700[1][3];
SG700[1][3]=S00[1][3]*S700[1][1] + S00[3][3]*S700[1][3];

SG700[2][1]=S00[1][1]*S700[2][1] + S00[3][1]*S700[2][3];
SG700[2][2]=S00[1][2]*S700[2][1] + S00[3][2]*S700[2][3];
SG700[2][3]=S00[1][3]*S700[2][1] + S00[3][3]*S700[2][3];

SG700[3][1]=S00[2][1];
SG700[3][2]=S00[2][2];
SG700[3][3]=S00[2][3];


SG710[1][1]=S7170[1][1]*SG700[1][1] + S7170[1][3]*SG700[3][1];
SG710[1][2]=S7170[1][1]*SG700[1][2] + S7170[1][3]*SG700[3][2];
SG710[1][3]=S7170[1][1]*SG700[1][3] + S7170[1][3]*SG700[3][3];

SG710[2][1]=S7170[2][1]*SG700[1][1] + S7170[2][3]*SG700[3][1];
SG710[2][2]=S7170[2][1]*SG700[1][2] + S7170[2][3]*SG700[3][2];
SG710[2][3]=S7170[2][1]*SG700[1][3] + S7170[2][3]*SG700[3][3];

SG710[3][1]=-SG700[2][1];
SG710[3][2]=-SG700[2][2];
SG710[3][3]=-SG700[2][3];


SG720[1][1]=S7271[1][1]*SG710[1][1] + S7271[1][2]*SG710[2][1] + S7271[1][3]*SG710[3][1];
SG720[1][2]=S7271[1][1]*SG710[1][2] + S7271[1][2]*SG710[2][2] + S7271[1][3]*SG710[3][2];
SG720[1][3]=S7271[1][1]*SG710[1][3] + S7271[1][2]*SG710[2][3] + S7271[1][3]*SG710[3][3];

SG720[2][1]=S7271[2][1]*SG710[1][1] + S7271[2][2]*SG710[2][1] + S7271[2][3]*SG710[3][1];
SG720[2][2]=S7271[2][1]*SG710[1][2] + S7271[2][2]*SG710[2][2] + S7271[2][3]*SG710[3][2];
SG720[2][3]=S7271[2][1]*SG710[1][3] + S7271[2][2]*SG710[2][3] + S7271[2][3]*SG710[3][3];

SG720[3][1]=S7271[3][1]*SG710[1][1] + S7271[3][2]*SG710[2][1];
SG720[3][2]=S7271[3][1]*SG710[1][2] + S7271[3][2]*SG710[2][2];
SG720[3][3]=S7271[3][1]*SG710[1][3] + S7271[3][2]*SG710[2][3];


SG730[1][1]=S7372[1][1]*SG720[1][1] + S7372[1][3]*SG720[3][1];
SG730[1][2]=S7372[1][1]*SG720[1][2] + S7372[1][3]*SG720[3][2];
SG730[1][3]=S7372[1][1]*SG720[1][3] + S7372[1][3]*SG720[3][3];

SG730[2][1]=S7372[2][1]*SG720[1][1] + S7372[2][3]*SG720[3][1];
SG730[2][2]=S7372[2][1]*SG720[1][2] + S7372[2][3]*SG720[3][2];
SG730[2][3]=S7372[2][1]*SG720[1][3] + S7372[2][3]*SG720[3][3];

SG730[3][1]=SG720[2][1];
SG730[3][2]=SG720[2][2];
SG730[3][3]=SG720[2][3];


SG740[1][1]=S7473[1][1]*SG730[1][1] + S7473[1][3]*SG730[3][1];
SG740[1][2]=S7473[1][1]*SG730[1][2] + S7473[1][3]*SG730[3][2];
SG740[1][3]=S7473[1][1]*SG730[1][3] + S7473[1][3]*SG730[3][3];

SG740[2][1]=S7473[2][1]*SG730[1][1] + S7473[2][3]*SG730[3][1];
SG740[2][2]=S7473[2][1]*SG730[1][2] + S7473[2][3]*SG730[3][2];
SG740[2][3]=S7473[2][1]*SG730[1][3] + S7473[2][3]*SG730[3][3];

SG740[3][1]=-SG730[2][1];
SG740[3][2]=-SG730[2][2];
SG740[3][3]=-SG730[2][3];


SG750[1][1]=S7574[1][1]*SG740[1][1] + S7574[1][3]*SG740[3][1];
SG750[1][2]=S7574[1][1]*SG740[1][2] + S7574[1][3]*SG740[3][2];
SG750[1][3]=S7574[1][1]*SG740[1][3] + S7574[1][3]*SG740[3][3];

SG750[2][1]=S7574[2][1]*SG740[1][1] + S7574[2][3]*SG740[3][1];
SG750[2][2]=S7574[2][1]*SG740[1][2] + S7574[2][3]*SG740[3][2];
SG750[2][3]=S7574[2][1]*SG740[1][3] + S7574[2][3]*SG740[3][3];

SG750[3][1]=-SG740[2][1];
SG750[3][2]=-SG740[2][2];
SG750[3][3]=-SG740[2][3];


SG760[1][1]=S7675[1][1]*SG750[1][1] + S7675[1][3]*SG750[3][1];
SG760[1][2]=S7675[1][1]*SG750[1][2] + S7675[1][3]*SG750[3][2];
SG760[1][3]=S7675[1][1]*SG750[1][3] + S7675[1][3]*SG750[3][3];

SG760[2][1]=S7675[2][1]*SG750[1][1] + S7675[2][3]*SG750[3][1];
SG760[2][2]=S7675[2][1]*SG750[1][2] + S7675[2][3]*SG750[3][2];
SG760[2][3]=S7675[2][1]*SG750[1][3] + S7675[2][3]*SG750[3][3];

SG760[3][1]=SG750[2][1];
SG760[3][2]=SG750[2][2];
SG760[3][3]=SG750[2][3];


SG770[1][1]=SG760[1][1];
SG770[1][2]=SG760[1][2];
SG770[1][3]=SG760[1][3];

SG770[2][1]=SG760[2][1];
SG770[2][2]=SG760[2][2];
SG770[2][3]=SG760[2][3];

SG770[3][1]=SG760[3][1];
SG770[3][2]=SG760[3][2];
SG770[3][3]=SG760[3][3];


SG780[1][1]=SG760[1][1];
SG780[1][2]=SG760[1][2];
SG780[1][3]=SG760[1][3];

SG780[2][1]=SG760[2][1];
SG780[2][2]=SG760[2][2];
SG780[2][3]=SG760[2][3];

SG780[3][1]=SG760[3][1];
SG780[3][2]=SG760[3][2];
SG780[3][3]=SG760[3][3];


SG790[1][1]=SG760[1][1];
SG790[1][2]=SG760[1][2];
SG790[1][3]=SG760[1][3];

SG790[2][1]=SG760[2][1];
SG790[2][2]=SG760[2][2];
SG790[2][3]=SG760[2][3];

SG790[3][1]=SG760[3][1];
SG790[3][2]=SG760[3][2];
SG790[3][3]=SG760[3][3];


SG800[1][1]=SG760[1][1];
SG800[1][2]=SG760[1][2];
SG800[1][3]=SG760[1][3];

SG800[2][1]=SG760[2][1];
SG800[2][2]=SG760[2][2];
SG800[2][3]=SG760[2][3];

SG800[3][1]=SG760[3][1];
SG800[3][2]=SG760[3][2];
SG800[3][3]=SG760[3][3];


SG810[1][1]=SG760[1][1];
SG810[1][2]=SG760[1][2];
SG810[1][3]=SG760[1][3];

SG810[2][1]=SG760[2][1];
SG810[2][2]=SG760[2][2];
SG810[2][3]=SG760[2][3];

SG810[3][1]=SG760[3][1];
SG810[3][2]=SG760[3][2];
SG810[3][3]=SG760[3][3];


SG820[1][1]=SG760[1][1];
SG820[1][2]=SG760[1][2];
SG820[1][3]=SG760[1][3];

SG820[2][1]=SG760[2][1];
SG820[2][2]=SG760[2][2];
SG820[2][3]=SG760[2][3];

SG820[3][1]=SG760[3][1];
SG820[3][2]=SG760[3][2];
SG820[3][3]=SG760[3][3];


SG830[1][1]=S8376[1][1]*SG760[1][1] + S8376[1][2]*SG760[2][1] + S8376[1][3]*SG760[3][1];
SG830[1][2]=S8376[1][1]*SG760[1][2] + S8376[1][2]*SG760[2][2] + S8376[1][3]*SG760[3][2];
SG830[1][3]=S8376[1][1]*SG760[1][3] + S8376[1][2]*SG760[2][3] + S8376[1][3]*SG760[3][3];

SG830[2][1]=S8376[2][1]*SG760[1][1] + S8376[2][2]*SG760[2][1] + S8376[2][3]*SG760[3][1];
SG830[2][2]=S8376[2][1]*SG760[1][2] + S8376[2][2]*SG760[2][2] + S8376[2][3]*SG760[3][2];
SG830[2][3]=S8376[2][1]*SG760[1][3] + S8376[2][2]*SG760[2][3] + S8376[2][3]*SG760[3][3];

SG830[3][1]=S8376[3][1]*SG760[1][1] + S8376[3][2]*SG760[2][1] + S8376[3][3]*SG760[3][1];
SG830[3][2]=S8376[3][1]*SG760[1][2] + S8376[3][2]*SG760[2][2] + S8376[3][3]*SG760[3][2];
SG830[3][3]=S8376[3][1]*SG760[1][3] + S8376[3][2]*SG760[2][3] + S8376[3][3]*SG760[3][3];


SG840[1][1]=S00[1][1]*S840[1][1] + S00[3][1]*S840[1][3];
SG840[1][2]=S00[1][2]*S840[1][1] + S00[3][2]*S840[1][3];
SG840[1][3]=S00[1][3]*S840[1][1] + S00[3][3]*S840[1][3];

SG840[2][1]=S00[1][1]*S840[2][1] + S00[3][1]*S840[2][3];
SG840[2][2]=S00[1][2]*S840[2][1] + S00[3][2]*S840[2][3];
SG840[2][3]=S00[1][3]*S840[2][1] + S00[3][3]*S840[2][3];

SG840[3][1]=-S00[2][1];
SG840[3][2]=-S00[2][2];
SG840[3][3]=-S00[2][3];


SG850[1][1]=S8584[1][1]*SG840[1][1] + S8584[1][3]*SG840[3][1];
SG850[1][2]=S8584[1][1]*SG840[1][2] + S8584[1][3]*SG840[3][2];
SG850[1][3]=S8584[1][1]*SG840[1][3] + S8584[1][3]*SG840[3][3];

SG850[2][1]=S8584[2][1]*SG840[1][1] + S8584[2][3]*SG840[3][1];
SG850[2][2]=S8584[2][1]*SG840[1][2] + S8584[2][3]*SG840[3][2];
SG850[2][3]=S8584[2][1]*SG840[1][3] + S8584[2][3]*SG840[3][3];

SG850[3][1]=SG840[2][1];
SG850[3][2]=SG840[2][2];
SG850[3][3]=SG840[2][3];


SG860[1][1]=S8685[1][1]*SG850[1][1] + S8685[1][2]*SG850[2][1] + S8685[1][3]*SG850[3][1];
SG860[1][2]=S8685[1][1]*SG850[1][2] + S8685[1][2]*SG850[2][2] + S8685[1][3]*SG850[3][2];
SG860[1][3]=S8685[1][1]*SG850[1][3] + S8685[1][2]*SG850[2][3] + S8685[1][3]*SG850[3][3];

SG860[2][1]=S8685[2][1]*SG850[1][1] + S8685[2][2]*SG850[2][1] + S8685[2][3]*SG850[3][1];
SG860[2][2]=S8685[2][1]*SG850[1][2] + S8685[2][2]*SG850[2][2] + S8685[2][3]*SG850[3][2];
SG860[2][3]=S8685[2][1]*SG850[1][3] + S8685[2][2]*SG850[2][3] + S8685[2][3]*SG850[3][3];

SG860[3][1]=S8685[3][1]*SG850[1][1] + S8685[3][2]*SG850[2][1];
SG860[3][2]=S8685[3][1]*SG850[1][2] + S8685[3][2]*SG850[2][2];
SG860[3][3]=S8685[3][1]*SG850[1][3] + S8685[3][2]*SG850[2][3];


SG870[1][1]=S8786[1][1]*SG860[1][1] + S8786[1][3]*SG860[3][1];
SG870[1][2]=S8786[1][1]*SG860[1][2] + S8786[1][3]*SG860[3][2];
SG870[1][3]=S8786[1][1]*SG860[1][3] + S8786[1][3]*SG860[3][3];

SG870[2][1]=S8786[2][1]*SG860[1][1] + S8786[2][3]*SG860[3][1];
SG870[2][2]=S8786[2][1]*SG860[1][2] + S8786[2][3]*SG860[3][2];
SG870[2][3]=S8786[2][1]*SG860[1][3] + S8786[2][3]*SG860[3][3];

SG870[3][1]=-SG860[2][1];
SG870[3][2]=-SG860[2][2];
SG870[3][3]=-SG860[2][3];


SG880[1][1]=S8887[1][1]*SG870[1][1] + S8887[1][3]*SG870[3][1];
SG880[1][2]=S8887[1][1]*SG870[1][2] + S8887[1][3]*SG870[3][2];
SG880[1][3]=S8887[1][1]*SG870[1][3] + S8887[1][3]*SG870[3][3];

SG880[2][1]=S8887[2][1]*SG870[1][1] + S8887[2][3]*SG870[3][1];
SG880[2][2]=S8887[2][1]*SG870[1][2] + S8887[2][3]*SG870[3][2];
SG880[2][3]=S8887[2][1]*SG870[1][3] + S8887[2][3]*SG870[3][3];

SG880[3][1]=SG870[2][1];
SG880[3][2]=SG870[2][2];
SG880[3][3]=SG870[2][3];


SG890[1][1]=S8988[1][1]*SG880[1][1] + S8988[1][3]*SG880[3][1];
SG890[1][2]=S8988[1][1]*SG880[1][2] + S8988[1][3]*SG880[3][2];
SG890[1][3]=S8988[1][1]*SG880[1][3] + S8988[1][3]*SG880[3][3];

SG890[2][1]=S8988[2][1]*SG880[1][1] + S8988[2][3]*SG880[3][1];
SG890[2][2]=S8988[2][1]*SG880[1][2] + S8988[2][3]*SG880[3][2];
SG890[2][3]=S8988[2][1]*SG880[1][3] + S8988[2][3]*SG880[3][3];

SG890[3][1]=SG880[2][1];
SG890[3][2]=SG880[2][2];
SG890[3][3]=SG880[2][3];


SG900[1][1]=S9089[1][1]*SG890[1][1] + S9089[1][3]*SG890[3][1];
SG900[1][2]=S9089[1][1]*SG890[1][2] + S9089[1][3]*SG890[3][2];
SG900[1][3]=S9089[1][1]*SG890[1][3] + S9089[1][3]*SG890[3][3];

SG900[2][1]=S9089[2][1]*SG890[1][1] + S9089[2][3]*SG890[3][1];
SG900[2][2]=S9089[2][1]*SG890[1][2] + S9089[2][3]*SG890[3][2];
SG900[2][3]=S9089[2][1]*SG890[1][3] + S9089[2][3]*SG890[3][3];

SG900[3][1]=-SG890[2][1];
SG900[3][2]=-SG890[2][2];
SG900[3][3]=-SG890[2][3];


SG910[1][1]=SG900[1][1];
SG910[1][2]=SG900[1][2];
SG910[1][3]=SG900[1][3];

SG910[2][1]=SG900[2][1];
SG910[2][2]=SG900[2][2];
SG910[2][3]=SG900[2][3];

SG910[3][1]=SG900[3][1];
SG910[3][2]=SG900[3][2];
SG910[3][3]=SG900[3][3];


SG920[1][1]=SG900[1][1];
SG920[1][2]=SG900[1][2];
SG920[1][3]=SG900[1][3];

SG920[2][1]=SG900[2][1];
SG920[2][2]=SG900[2][2];
SG920[2][3]=SG900[2][3];

SG920[3][1]=SG900[3][1];
SG920[3][2]=SG900[3][2];
SG920[3][3]=SG900[3][3];


SG930[1][1]=SG900[1][1];
SG930[1][2]=SG900[1][2];
SG930[1][3]=SG900[1][3];

SG930[2][1]=SG900[2][1];
SG930[2][2]=SG900[2][2];
SG930[2][3]=SG900[2][3];

SG930[3][1]=SG900[3][1];
SG930[3][2]=SG900[3][2];
SG930[3][3]=SG900[3][3];


SG940[1][1]=SG900[1][1];
SG940[1][2]=SG900[1][2];
SG940[1][3]=SG900[1][3];

SG940[2][1]=SG900[2][1];
SG940[2][2]=SG900[2][2];
SG940[2][3]=SG900[2][3];

SG940[3][1]=SG900[3][1];
SG940[3][2]=SG900[3][2];
SG940[3][3]=SG900[3][3];


SG950[1][1]=SG900[1][1];
SG950[1][2]=SG900[1][2];
SG950[1][3]=SG900[1][3];

SG950[2][1]=SG900[2][1];
SG950[2][2]=SG900[2][2];
SG950[2][3]=SG900[2][3];

SG950[3][1]=SG900[3][1];
SG950[3][2]=SG900[3][2];
SG950[3][3]=SG900[3][3];


SG960[1][1]=SG900[1][1];
SG960[1][2]=SG900[1][2];
SG960[1][3]=SG900[1][3];

SG960[2][1]=SG900[2][1];
SG960[2][2]=SG900[2][2];
SG960[2][3]=SG900[2][3];

SG960[3][1]=SG900[3][1];
SG960[3][2]=SG900[3][2];
SG960[3][3]=SG900[3][3];


SG970[1][1]=S9790[1][1]*SG900[1][1] + S9790[1][2]*SG900[2][1] + S9790[1][3]*SG900[3][1];
SG970[1][2]=S9790[1][1]*SG900[1][2] + S9790[1][2]*SG900[2][2] + S9790[1][3]*SG900[3][2];
SG970[1][3]=S9790[1][1]*SG900[1][3] + S9790[1][2]*SG900[2][3] + S9790[1][3]*SG900[3][3];

SG970[2][1]=S9790[2][1]*SG900[1][1] + S9790[2][2]*SG900[2][1] + S9790[2][3]*SG900[3][1];
SG970[2][2]=S9790[2][1]*SG900[1][2] + S9790[2][2]*SG900[2][2] + S9790[2][3]*SG900[3][2];
SG970[2][3]=S9790[2][1]*SG900[1][3] + S9790[2][2]*SG900[2][3] + S9790[2][3]*SG900[3][3];

SG970[3][1]=S9790[3][1]*SG900[1][1] + S9790[3][2]*SG900[2][1] + S9790[3][3]*SG900[3][1];
SG970[3][2]=S9790[3][1]*SG900[1][2] + S9790[3][2]*SG900[2][2] + S9790[3][3]*SG900[3][2];
SG970[3][3]=S9790[3][1]*SG900[1][3] + S9790[3][2]*SG900[2][3] + S9790[3][3]*SG900[3][3];




}


void
hermes_InvDynArtfunc4(void)
     {
/* velocity vectors */
v0[1]=baseo[0].ad[1]*S00[1][1] + baseo[0].ad[2]*S00[1][2] + baseo[0].ad[3]*S00[1][3];
v0[2]=baseo[0].ad[1]*S00[2][1] + baseo[0].ad[2]*S00[2][2] + baseo[0].ad[3]*S00[2][3];
v0[3]=baseo[0].ad[1]*S00[3][1] + baseo[0].ad[2]*S00[3][2] + baseo[0].ad[3]*S00[3][3];
v0[4]=basec[0].xd[1]*S00[1][1] + basec[0].xd[2]*S00[1][2] + basec[0].xd[3]*S00[1][3];
v0[5]=basec[0].xd[1]*S00[2][1] + basec[0].xd[2]*S00[2][2] + basec[0].xd[3]*S00[2][3];
v0[6]=basec[0].xd[1]*S00[3][1] + basec[0].xd[2]*S00[3][2] + basec[0].xd[3]*S00[3][3];

v1[1]=v0[1]*S10[1][1] + v0[2]*S10[1][2];
v1[2]=v0[1]*S10[2][1] + v0[2]*S10[2][2];
v1[3]=state[29].thd - v0[3];
v1[4]=PELVIS2THORAX*v0[2]*S10[1][1] + PELVISOFFSET*v0[3]*S10[1][1] + v0[4]*S10[1][1] - PELVIS2THORAX*v0[1]*S10[1][2] + v0[5]*S10[1][2];
v1[5]=PELVIS2THORAX*v0[2]*S10[2][1] + PELVISOFFSET*v0[3]*S10[2][1] + v0[4]*S10[2][1] - PELVIS2THORAX*v0[1]*S10[2][2] + v0[5]*S10[2][2];
v1[6]=PELVISOFFSET*v0[1] - v0[6];

v2[1]=v1[1]*S21[1][1] + v1[3]*S21[1][3];
v2[2]=v1[1]*S21[2][1] + v1[3]*S21[2][3];
v2[3]=state[30].thd - v1[2];
v2[4]=v1[4]*S21[1][1] + v1[6]*S21[1][3];
v2[5]=v1[4]*S21[2][1] + v1[6]*S21[2][3];
v2[6]=-v1[5];

v3[1]=v2[1]*S32[1][1] + v2[3]*S32[1][3];
v3[2]=v2[1]*S32[2][1] + v2[3]*S32[2][3];
v3[3]=state[31].thd + v2[2];
v3[4]=v2[4]*S32[1][1] + v2[6]*S32[1][3];
v3[5]=v2[4]*S32[2][1] + v2[6]*S32[2][3];
v3[6]=v2[5];

v4[1]=v3[1]*S43[1][1] + v3[2]*S43[1][2] + v3[3]*S43[1][3];
v4[2]=v3[1]*S43[2][1] + v3[2]*S43[2][2] + v3[3]*S43[2][3];
v4[3]=state[1].thd + 0.7071067811865475*v3[1] - 0.7071067811865475*v3[3];
v4[4]=v3[4]*S43[1][1] - THORAX2SHOULDER*v3[3]*S43[1][2] + v3[5]*S43[1][2] + THORAX2SHOULDER*v3[2]*S43[1][3] + v3[6]*S43[1][3];
v4[5]=v3[4]*S43[2][1] - THORAX2SHOULDER*v3[3]*S43[2][2] + v3[5]*S43[2][2] + THORAX2SHOULDER*v3[2]*S43[2][3] + v3[6]*S43[2][3];
v4[6]=-0.7071067811865475*THORAX2SHOULDER*v3[2] + 0.7071067811865475*v3[4] - 0.7071067811865475*v3[6];

v5[1]=v4[1]*S54[1][1] + v4[3]*S54[1][3];
v5[2]=v4[1]*S54[2][1] + v4[3]*S54[2][3];
v5[3]=state[2].thd - v4[2];
v5[4]=-(SHOULDERX*v4[2]*S54[1][1]) + v4[4]*S54[1][1] + v4[6]*S54[1][3];
v5[5]=-(SHOULDERX*v4[2]*S54[2][1]) + v4[4]*S54[2][1] + v4[6]*S54[2][3];
v5[6]=-(SHOULDERX*v4[1]) - v4[5];

v6[1]=v5[1]*S65[1][1] + v5[3]*S65[1][3];
v6[2]=v5[1]*S65[2][1] + v5[3]*S65[2][3];
v6[3]=state[3].thd + v5[2];
v6[4]=v5[4]*S65[1][1] + SHOULDERY*v5[2]*S65[1][3] + v5[6]*S65[1][3];
v6[5]=v5[4]*S65[2][1] + SHOULDERY*v5[2]*S65[2][3] + v5[6]*S65[2][3];
v6[6]=-(SHOULDERY*v5[3]) + v5[5];

v7[1]=v6[2]*S76[1][2] + v6[3]*S76[1][3];
v7[2]=v6[2]*S76[2][2] + v6[3]*S76[2][3];
v7[3]=state[4].thd + v6[1];
v7[4]=UPPERARM*v6[1]*S76[1][2] + v6[5]*S76[1][2] + v6[6]*S76[1][3];
v7[5]=UPPERARM*v6[1]*S76[2][2] + v6[5]*S76[2][2] + v6[6]*S76[2][3];
v7[6]=-(UPPERARM*v6[2]) + v6[4];

v8[1]=v7[1]*S87[1][1] + v7[3]*S87[1][3];
v8[2]=v7[1]*S87[2][1] + v7[3]*S87[2][3];
v8[3]=state[5].thd + v7[2];
v8[4]=v7[4]*S87[1][1] + v7[6]*S87[1][3];
v8[5]=v7[4]*S87[2][1] + v7[6]*S87[2][3];
v8[6]=v7[5];

v9[1]=v8[2]*S98[1][2] + v8[3]*S98[1][3];
v9[2]=v8[2]*S98[2][2] + v8[3]*S98[2][3];
v9[3]=state[6].thd + v8[1];
v9[4]=v8[5]*S98[1][2] + v8[6]*S98[1][3] + v8[1]*(LOWERARM*S98[1][2] + WRISTY*S98[1][3]);
v9[5]=v8[5]*S98[2][2] + v8[6]*S98[2][3] + v8[1]*(LOWERARM*S98[2][2] + WRISTY*S98[2][3]);
v9[6]=-(LOWERARM*v8[2]) - WRISTY*v8[3] + v8[4];

v10[1]=v9[1]*S109[1][1] + v9[3]*S109[1][3];
v10[2]=v9[1]*S109[2][1] + v9[3]*S109[2][3];
v10[3]=state[7].thd - v9[2];
v10[4]=v9[4]*S109[1][1] + v9[6]*S109[1][3];
v10[5]=v9[4]*S109[2][1] + v9[6]*S109[2][3];
v10[6]=-v9[5];

v11[1]=v10[1]*S1110[1][1] + v10[2]*S1110[1][2] + v10[3]*S1110[1][3];
v11[2]=v10[1]*S1110[2][1] + v10[2]*S1110[2][2] + v10[3]*S1110[2][3];
v11[3]=v10[1]*S1110[3][1] + v10[2]*S1110[3][2] + v10[3]*S1110[3][3];
v11[4]=v10[4]*S1110[1][1] + v10[5]*S1110[1][2] + v10[3]*(-(eff[2].x[2]*S1110[1][1]) + eff[2].x[1]*S1110[1][2]) + v10[6]*S1110[1][3] + v10[2]*(eff[2].x[3]*S1110[1][1] - eff[2].x[1]*S1110[1][3]) + v10[1]*(-(eff[2].x[3]*S1110[1][2]) + eff[2].x[2]*S1110[1][3]);
v11[5]=v10[4]*S1110[2][1] + v10[5]*S1110[2][2] + v10[3]*(-(eff[2].x[2]*S1110[2][1]) + eff[2].x[1]*S1110[2][2]) + v10[6]*S1110[2][3] + v10[2]*(eff[2].x[3]*S1110[2][1] - eff[2].x[1]*S1110[2][3]) + v10[1]*(-(eff[2].x[3]*S1110[2][2]) + eff[2].x[2]*S1110[2][3]);
v11[6]=v10[4]*S1110[3][1] + v10[5]*S1110[3][2] + v10[3]*(-(eff[2].x[2]*S1110[3][1]) + eff[2].x[1]*S1110[3][2]) + v10[6]*S1110[3][3] + v10[2]*(eff[2].x[3]*S1110[3][1] - eff[2].x[1]*S1110[3][3]) + v10[1]*(-(eff[2].x[3]*S1110[3][2]) + eff[2].x[2]*S1110[3][3]);

v12[1]=state[39].thd + v10[1]*S1210[1][1] + v10[2]*S1210[1][2] + v10[3]*S1210[1][3];
v12[2]=v10[1]*S1210[2][1] + v10[2]*S1210[2][2] + v10[3]*S1210[2][3];
v12[3]=v10[1]*S1210[3][1] + v10[2]*S1210[3][2] + v10[3]*S1210[3][3];
v12[4]=v10[4]*S1210[1][1] + v10[5]*S1210[1][2] + v10[3]*(-(YTHUMB*S1210[1][1]) + XTHUMB*S1210[1][2]) + v10[6]*S1210[1][3] + v10[2]*(-(ZTHUMB*S1210[1][1]) - XTHUMB*S1210[1][3]) + v10[1]*(ZTHUMB*S1210[1][2] + YTHUMB*S1210[1][3]);
v12[5]=v10[4]*S1210[2][1] + v10[5]*S1210[2][2] + v10[3]*(-(YTHUMB*S1210[2][1]) + XTHUMB*S1210[2][2]) + v10[6]*S1210[2][3] + v10[2]*(-(ZTHUMB*S1210[2][1]) - XTHUMB*S1210[2][3]) + v10[1]*(ZTHUMB*S1210[2][2] + YTHUMB*S1210[2][3]);
v12[6]=v10[4]*S1210[3][1] + v10[5]*S1210[3][2] + v10[3]*(-(YTHUMB*S1210[3][1]) + XTHUMB*S1210[3][2]) + v10[6]*S1210[3][3] + v10[2]*(-(ZTHUMB*S1210[3][1]) - XTHUMB*S1210[3][3]) + v10[1]*(ZTHUMB*S1210[3][2] + YTHUMB*S1210[3][3]);

v13[1]=v12[1]*S1312[1][1] + v12[2]*S1312[1][2];
v13[2]=v12[1]*S1312[2][1] + v12[2]*S1312[2][2];
v13[3]=-state[40].thd + v12[3];
v13[4]=v12[4]*S1312[1][1] + v12[5]*S1312[1][2] + v12[3]*(-(YTHUMBFLEX*S1312[1][1]) + XTHUMBFLEX*S1312[1][2]);
v13[5]=v12[4]*S1312[2][1] + v12[5]*S1312[2][2] + v12[3]*(-(YTHUMBFLEX*S1312[2][1]) + XTHUMBFLEX*S1312[2][2]);
v13[6]=YTHUMBFLEX*v12[1] - XTHUMBFLEX*v12[2] + v12[6];

v14[1]=v13[1]*S1413[1][1] + v13[2]*S1413[1][2];
v14[2]=v13[1]*S1413[2][1] + v13[2]*S1413[2][2];
v14[3]=v13[3];
v14[4]=v13[4]*S1413[1][1] + TH1SEG*v13[3]*S1413[1][2] + v13[5]*S1413[1][2];
v14[5]=v13[4]*S1413[2][1] + TH1SEG*v13[3]*S1413[2][2] + v13[5]*S1413[2][2];
v14[6]=-(TH1SEG*v13[2]) + v13[6];

v15[1]=v14[1];
v15[2]=v14[2];
v15[3]=v14[3];
v15[4]=v14[4];
v15[5]=TH2SEG*v14[3] + v14[5];
v15[6]=-(TH2SEG*v14[2]) + v14[6];

v16[1]=v10[1]*S1610[1][1] + v10[2]*S1610[1][2] + v10[3]*S1610[1][3];
v16[2]=v10[1]*S1610[2][1] + v10[2]*S1610[2][2] + v10[3]*S1610[2][3];
v16[3]=state[41].thd + v10[2]*S1610[3][2] + v10[3]*S1610[3][3];
v16[4]=v10[4]*S1610[1][1] + v10[5]*S1610[1][2] + v10[3]*(-(YIF*S1610[1][1]) + XIF*S1610[1][2]) + v10[6]*S1610[1][3] + v10[2]*(-(ZIF*S1610[1][1]) - XIF*S1610[1][3]) + v10[1]*(ZIF*S1610[1][2] + YIF*S1610[1][3]);
v16[5]=v10[4]*S1610[2][1] + v10[5]*S1610[2][2] + v10[3]*(-(YIF*S1610[2][1]) + XIF*S1610[2][2]) + v10[6]*S1610[2][3] + v10[2]*(-(ZIF*S1610[2][1]) - XIF*S1610[2][3]) + v10[1]*(ZIF*S1610[2][2] + YIF*S1610[2][3]);
v16[6]=XIF*v10[3]*S1610[3][2] + v10[5]*S1610[3][2] - XIF*v10[2]*S1610[3][3] + v10[6]*S1610[3][3] + v10[1]*(ZIF*S1610[3][2] + YIF*S1610[3][3]);

v17[1]=v16[1]*S1716[1][1] + v16[2]*S1716[1][2];
v17[2]=v16[1]*S1716[2][1] + v16[2]*S1716[2][2];
v17[3]=v16[3];
v17[4]=v16[4]*S1716[1][1] + F1SEG*v16[3]*S1716[1][2] + v16[5]*S1716[1][2];
v17[5]=v16[4]*S1716[2][1] + F1SEG*v16[3]*S1716[2][2] + v16[5]*S1716[2][2];
v17[6]=-(F1SEG*v16[2]) + v16[6];

v18[1]=v17[1]*S1817[1][1] + v17[2]*S1817[1][2];
v18[2]=v17[1]*S1817[2][1] + v17[2]*S1817[2][2];
v18[3]=v17[3];
v18[4]=v17[4]*S1817[1][1] + F2SEG*v17[3]*S1817[1][2] + v17[5]*S1817[1][2];
v18[5]=v17[4]*S1817[2][1] + F2SEG*v17[3]*S1817[2][2] + v17[5]*S1817[2][2];
v18[6]=-(F2SEG*v17[2]) + v17[6];

v19[1]=v18[1];
v19[2]=v18[2];
v19[3]=v18[3];
v19[4]=v18[4];
v19[5]=F3SEG*v18[3] + v18[5];
v19[6]=-(F3SEG*v18[2]) + v18[6];

v20[1]=v10[1]*S2010[1][1] + v10[2]*S2010[1][2] + v10[3]*S2010[1][3];
v20[2]=v10[1]*S2010[2][1] + v10[2]*S2010[2][2] + v10[3]*S2010[2][3];
v20[3]=state[42].thd + v10[2]*S2010[3][2] + v10[3]*S2010[3][3];
v20[4]=v10[4]*S2010[1][1] + v10[5]*S2010[1][2] + v10[3]*(-(YMF*S2010[1][1]) + XMF*S2010[1][2]) + v10[6]*S2010[1][3] + v10[2]*(-(ZMF*S2010[1][1]) - XMF*S2010[1][3]) + v10[1]*(ZMF*S2010[1][2] + YMF*S2010[1][3]);
v20[5]=v10[4]*S2010[2][1] + v10[5]*S2010[2][2] + v10[3]*(-(YMF*S2010[2][1]) + XMF*S2010[2][2]) + v10[6]*S2010[2][3] + v10[2]*(-(ZMF*S2010[2][1]) - XMF*S2010[2][3]) + v10[1]*(ZMF*S2010[2][2] + YMF*S2010[2][3]);
v20[6]=XMF*v10[3]*S2010[3][2] + v10[5]*S2010[3][2] - XMF*v10[2]*S2010[3][3] + v10[6]*S2010[3][3] + v10[1]*(ZMF*S2010[3][2] + YMF*S2010[3][3]);

v21[1]=v20[1]*S2120[1][1] + v20[2]*S2120[1][2];
v21[2]=v20[1]*S2120[2][1] + v20[2]*S2120[2][2];
v21[3]=v20[3];
v21[4]=v20[4]*S2120[1][1] + F1SEG*v20[3]*S2120[1][2] + v20[5]*S2120[1][2];
v21[5]=v20[4]*S2120[2][1] + F1SEG*v20[3]*S2120[2][2] + v20[5]*S2120[2][2];
v21[6]=-(F1SEG*v20[2]) + v20[6];

v22[1]=v21[1]*S2221[1][1] + v21[2]*S2221[1][2];
v22[2]=v21[1]*S2221[2][1] + v21[2]*S2221[2][2];
v22[3]=v21[3];
v22[4]=v21[4]*S2221[1][1] + F2SEG*v21[3]*S2221[1][2] + v21[5]*S2221[1][2];
v22[5]=v21[4]*S2221[2][1] + F2SEG*v21[3]*S2221[2][2] + v21[5]*S2221[2][2];
v22[6]=-(F2SEG*v21[2]) + v21[6];

v23[1]=v22[1];
v23[2]=v22[2];
v23[3]=v22[3];
v23[4]=v22[4];
v23[5]=F3SEG*v22[3] + v22[5];
v23[6]=-(F3SEG*v22[2]) + v22[6];

v24[1]=v10[1]*S2410[1][1] + v10[2]*S2410[1][2] + v10[3]*S2410[1][3];
v24[2]=v10[1]*S2410[2][1] + v10[2]*S2410[2][2] + v10[3]*S2410[2][3];
v24[3]=state[43].thd + v10[2]*S2410[3][2] + v10[3]*S2410[3][3];
v24[4]=v10[4]*S2410[1][1] + v10[5]*S2410[1][2] + v10[3]*(-(YRF*S2410[1][1]) + XRF*S2410[1][2]) + v10[6]*S2410[1][3] + v10[2]*(-(ZRF*S2410[1][1]) - XRF*S2410[1][3]) + v10[1]*(ZRF*S2410[1][2] + YRF*S2410[1][3]);
v24[5]=v10[4]*S2410[2][1] + v10[5]*S2410[2][2] + v10[3]*(-(YRF*S2410[2][1]) + XRF*S2410[2][2]) + v10[6]*S2410[2][3] + v10[2]*(-(ZRF*S2410[2][1]) - XRF*S2410[2][3]) + v10[1]*(ZRF*S2410[2][2] + YRF*S2410[2][3]);
v24[6]=XRF*v10[3]*S2410[3][2] + v10[5]*S2410[3][2] - XRF*v10[2]*S2410[3][3] + v10[6]*S2410[3][3] + v10[1]*(ZRF*S2410[3][2] + YRF*S2410[3][3]);

v25[1]=v24[1]*S2524[1][1] + v24[2]*S2524[1][2];
v25[2]=v24[1]*S2524[2][1] + v24[2]*S2524[2][2];
v25[3]=v24[3];
v25[4]=v24[4]*S2524[1][1] + F1SEG*v24[3]*S2524[1][2] + v24[5]*S2524[1][2];
v25[5]=v24[4]*S2524[2][1] + F1SEG*v24[3]*S2524[2][2] + v24[5]*S2524[2][2];
v25[6]=-(F1SEG*v24[2]) + v24[6];

v26[1]=v25[1]*S2625[1][1] + v25[2]*S2625[1][2];
v26[2]=v25[1]*S2625[2][1] + v25[2]*S2625[2][2];
v26[3]=v25[3];
v26[4]=v25[4]*S2625[1][1] + F2SEG*v25[3]*S2625[1][2] + v25[5]*S2625[1][2];
v26[5]=v25[4]*S2625[2][1] + F2SEG*v25[3]*S2625[2][2] + v25[5]*S2625[2][2];
v26[6]=-(F2SEG*v25[2]) + v25[6];

v27[1]=v26[1];
v27[2]=v26[2];
v27[3]=v26[3];
v27[4]=v26[4];
v27[5]=F3SEG*v26[3] + v26[5];
v27[6]=-(F3SEG*v26[2]) + v26[6];

v28[1]=v10[1]*S2810[1][1] + v10[2]*S2810[1][2] + v10[3]*S2810[1][3];
v28[2]=v10[1]*S2810[2][1] + v10[2]*S2810[2][2] + v10[3]*S2810[2][3];
v28[3]=state[44].thd + v10[2]*S2810[3][2] + v10[3]*S2810[3][3];
v28[4]=v10[4]*S2810[1][1] + v10[5]*S2810[1][2] + v10[3]*(-(YLF*S2810[1][1]) + XLF*S2810[1][2]) + v10[6]*S2810[1][3] + v10[2]*(-(ZLF*S2810[1][1]) - XLF*S2810[1][3]) + v10[1]*(ZLF*S2810[1][2] + YLF*S2810[1][3]);
v28[5]=v10[4]*S2810[2][1] + v10[5]*S2810[2][2] + v10[3]*(-(YLF*S2810[2][1]) + XLF*S2810[2][2]) + v10[6]*S2810[2][3] + v10[2]*(-(ZLF*S2810[2][1]) - XLF*S2810[2][3]) + v10[1]*(ZLF*S2810[2][2] + YLF*S2810[2][3]);
v28[6]=XLF*v10[3]*S2810[3][2] + v10[5]*S2810[3][2] - XLF*v10[2]*S2810[3][3] + v10[6]*S2810[3][3] + v10[1]*(ZLF*S2810[3][2] + YLF*S2810[3][3]);

v29[1]=v28[1]*S2928[1][1] + v28[2]*S2928[1][2];
v29[2]=v28[1]*S2928[2][1] + v28[2]*S2928[2][2];
v29[3]=v28[3];
v29[4]=v28[4]*S2928[1][1] + F1SEG*v28[3]*S2928[1][2] + v28[5]*S2928[1][2];
v29[5]=v28[4]*S2928[2][1] + F1SEG*v28[3]*S2928[2][2] + v28[5]*S2928[2][2];
v29[6]=-(F1SEG*v28[2]) + v28[6];

v30[1]=v29[1]*S3029[1][1] + v29[2]*S3029[1][2];
v30[2]=v29[1]*S3029[2][1] + v29[2]*S3029[2][2];
v30[3]=v29[3];
v30[4]=v29[4]*S3029[1][1] + F2SEG*v29[3]*S3029[1][2] + v29[5]*S3029[1][2];
v30[5]=v29[4]*S3029[2][1] + F2SEG*v29[3]*S3029[2][2] + v29[5]*S3029[2][2];
v30[6]=-(F2SEG*v29[2]) + v29[6];

v31[1]=v30[1];
v31[2]=v30[2];
v31[3]=v30[3];
v31[4]=v30[4];
v31[5]=F3SEG*v30[3] + v30[5];
v31[6]=-(F3SEG*v30[2]) + v30[6];

v32[1]=v3[1]*S323[1][1] + v3[2]*S323[1][2] + v3[3]*S323[1][3];
v32[2]=v3[1]*S323[2][1] + v3[2]*S323[2][2] + v3[3]*S323[2][3];
v32[3]=state[8].thd - 0.7071067811865475*v3[1] - 0.7071067811865475*v3[3];
v32[4]=v3[4]*S323[1][1] - THORAX2SHOULDER*v3[3]*S323[1][2] + v3[5]*S323[1][2] + THORAX2SHOULDER*v3[2]*S323[1][3] + v3[6]*S323[1][3];
v32[5]=v3[4]*S323[2][1] - THORAX2SHOULDER*v3[3]*S323[2][2] + v3[5]*S323[2][2] + THORAX2SHOULDER*v3[2]*S323[2][3] + v3[6]*S323[2][3];
v32[6]=-0.7071067811865475*THORAX2SHOULDER*v3[2] - 0.7071067811865475*v3[4] - 0.7071067811865475*v3[6];

v33[1]=v32[1]*S3332[1][1] + v32[3]*S3332[1][3];
v33[2]=v32[1]*S3332[2][1] + v32[3]*S3332[2][3];
v33[3]=state[9].thd + v32[2];
v33[4]=SHOULDERX*v32[2]*S3332[1][1] + v32[4]*S3332[1][1] + v32[6]*S3332[1][3];
v33[5]=SHOULDERX*v32[2]*S3332[2][1] + v32[4]*S3332[2][1] + v32[6]*S3332[2][3];
v33[6]=-(SHOULDERX*v32[1]) + v32[5];

v34[1]=v33[1]*S3433[1][1] + v33[3]*S3433[1][3];
v34[2]=v33[1]*S3433[2][1] + v33[3]*S3433[2][3];
v34[3]=state[10].thd - v33[2];
v34[4]=v33[4]*S3433[1][1] + SHOULDERY*v33[2]*S3433[1][3] + v33[6]*S3433[1][3];
v34[5]=v33[4]*S3433[2][1] + SHOULDERY*v33[2]*S3433[2][3] + v33[6]*S3433[2][3];
v34[6]=SHOULDERY*v33[3] - v33[5];

v35[1]=v34[2]*S3534[1][2] + v34[3]*S3534[1][3];
v35[2]=v34[2]*S3534[2][2] + v34[3]*S3534[2][3];
v35[3]=state[11].thd - v34[1];
v35[4]=-(UPPERARM*v34[1]*S3534[1][2]) + v34[5]*S3534[1][2] + v34[6]*S3534[1][3];
v35[5]=-(UPPERARM*v34[1]*S3534[2][2]) + v34[5]*S3534[2][2] + v34[6]*S3534[2][3];
v35[6]=-(UPPERARM*v34[2]) - v34[4];

v36[1]=v35[1]*S3635[1][1] + v35[3]*S3635[1][3];
v36[2]=v35[1]*S3635[2][1] + v35[3]*S3635[2][3];
v36[3]=state[12].thd - v35[2];
v36[4]=v35[4]*S3635[1][1] + v35[6]*S3635[1][3];
v36[5]=v35[4]*S3635[2][1] + v35[6]*S3635[2][3];
v36[6]=-v35[5];

v37[1]=v36[2]*S3736[1][2] + v36[3]*S3736[1][3];
v37[2]=v36[2]*S3736[2][2] + v36[3]*S3736[2][3];
v37[3]=state[13].thd - v36[1];
v37[4]=v36[5]*S3736[1][2] + v36[6]*S3736[1][3] + v36[1]*(-(LOWERARM*S3736[1][2]) + WRISTY*S3736[1][3]);
v37[5]=v36[5]*S3736[2][2] + v36[6]*S3736[2][3] + v36[1]*(-(LOWERARM*S3736[2][2]) + WRISTY*S3736[2][3]);
v37[6]=-(LOWERARM*v36[2]) + WRISTY*v36[3] - v36[4];

v38[1]=v37[1]*S3837[1][1] + v37[3]*S3837[1][3];
v38[2]=v37[1]*S3837[2][1] + v37[3]*S3837[2][3];
v38[3]=state[14].thd + v37[2];
v38[4]=v37[4]*S3837[1][1] + v37[6]*S3837[1][3];
v38[5]=v37[4]*S3837[2][1] + v37[6]*S3837[2][3];
v38[6]=v37[5];

v39[1]=v38[1]*S3938[1][1] + v38[2]*S3938[1][2] + v38[3]*S3938[1][3];
v39[2]=v38[1]*S3938[2][1] + v38[2]*S3938[2][2] + v38[3]*S3938[2][3];
v39[3]=v38[1]*S3938[3][1] + v38[2]*S3938[3][2] + v38[3]*S3938[3][3];
v39[4]=v38[4]*S3938[1][1] + v38[5]*S3938[1][2] + v38[3]*(-(eff[1].x[2]*S3938[1][1]) + eff[1].x[1]*S3938[1][2]) + v38[6]*S3938[1][3] + v38[2]*(eff[1].x[3]*S3938[1][1] - eff[1].x[1]*S3938[1][3]) + v38[1]*(-(eff[1].x[3]*S3938[1][2]) + eff[1].x[2]*S3938[1][3]);
v39[5]=v38[4]*S3938[2][1] + v38[5]*S3938[2][2] + v38[3]*(-(eff[1].x[2]*S3938[2][1]) + eff[1].x[1]*S3938[2][2]) + v38[6]*S3938[2][3] + v38[2]*(eff[1].x[3]*S3938[2][1] - eff[1].x[1]*S3938[2][3]) + v38[1]*(-(eff[1].x[3]*S3938[2][2]) + eff[1].x[2]*S3938[2][3]);
v39[6]=v38[4]*S3938[3][1] + v38[5]*S3938[3][2] + v38[3]*(-(eff[1].x[2]*S3938[3][1]) + eff[1].x[1]*S3938[3][2]) + v38[6]*S3938[3][3] + v38[2]*(eff[1].x[3]*S3938[3][1] - eff[1].x[1]*S3938[3][3]) + v38[1]*(-(eff[1].x[3]*S3938[3][2]) + eff[1].x[2]*S3938[3][3]);

v40[1]=state[45].thd + v38[1]*S4038[1][1] + v38[2]*S4038[1][2] + v38[3]*S4038[1][3];
v40[2]=v38[1]*S4038[2][1] + v38[2]*S4038[2][2] + v38[3]*S4038[2][3];
v40[3]=v38[1]*S4038[3][1] + v38[2]*S4038[3][2] + v38[3]*S4038[3][3];
v40[4]=v38[4]*S4038[1][1] + v38[5]*S4038[1][2] + v38[3]*(-(YTHUMB*S4038[1][1]) + XTHUMB*S4038[1][2]) + v38[6]*S4038[1][3] + v38[2]*(ZTHUMB*S4038[1][1] - XTHUMB*S4038[1][3]) + v38[1]*(-(ZTHUMB*S4038[1][2]) + YTHUMB*S4038[1][3]);
v40[5]=v38[4]*S4038[2][1] + v38[5]*S4038[2][2] + v38[3]*(-(YTHUMB*S4038[2][1]) + XTHUMB*S4038[2][2]) + v38[6]*S4038[2][3] + v38[2]*(ZTHUMB*S4038[2][1] - XTHUMB*S4038[2][3]) + v38[1]*(-(ZTHUMB*S4038[2][2]) + YTHUMB*S4038[2][3]);
v40[6]=v38[4]*S4038[3][1] + v38[5]*S4038[3][2] + v38[3]*(-(YTHUMB*S4038[3][1]) + XTHUMB*S4038[3][2]) + v38[6]*S4038[3][3] + v38[2]*(ZTHUMB*S4038[3][1] - XTHUMB*S4038[3][3]) + v38[1]*(-(ZTHUMB*S4038[3][2]) + YTHUMB*S4038[3][3]);

v41[1]=v40[1]*S4140[1][1] + v40[2]*S4140[1][2];
v41[2]=v40[1]*S4140[2][1] + v40[2]*S4140[2][2];
v41[3]=-state[46].thd + v40[3];
v41[4]=v40[4]*S4140[1][1] + v40[5]*S4140[1][2] + v40[3]*(-(YTHUMBFLEX*S4140[1][1]) + XTHUMBFLEX*S4140[1][2]);
v41[5]=v40[4]*S4140[2][1] + v40[5]*S4140[2][2] + v40[3]*(-(YTHUMBFLEX*S4140[2][1]) + XTHUMBFLEX*S4140[2][2]);
v41[6]=YTHUMBFLEX*v40[1] - XTHUMBFLEX*v40[2] + v40[6];

v42[1]=v41[1]*S4241[1][1] + v41[2]*S4241[1][2];
v42[2]=v41[1]*S4241[2][1] + v41[2]*S4241[2][2];
v42[3]=v41[3];
v42[4]=v41[4]*S4241[1][1] + TH1SEG*v41[3]*S4241[1][2] + v41[5]*S4241[1][2];
v42[5]=v41[4]*S4241[2][1] + TH1SEG*v41[3]*S4241[2][2] + v41[5]*S4241[2][2];
v42[6]=-(TH1SEG*v41[2]) + v41[6];

v43[1]=v42[1];
v43[2]=v42[2];
v43[3]=v42[3];
v43[4]=v42[4];
v43[5]=TH2SEG*v42[3] + v42[5];
v43[6]=-(TH2SEG*v42[2]) + v42[6];

v44[1]=v38[1]*S4438[1][1] + v38[2]*S4438[1][2] + v38[3]*S4438[1][3];
v44[2]=v38[1]*S4438[2][1] + v38[2]*S4438[2][2] + v38[3]*S4438[2][3];
v44[3]=state[47].thd + v38[2]*S4438[3][2] + v38[3]*S4438[3][3];
v44[4]=v38[4]*S4438[1][1] + v38[5]*S4438[1][2] + v38[3]*(-(YIF*S4438[1][1]) + XIF*S4438[1][2]) + v38[6]*S4438[1][3] + v38[2]*(ZIF*S4438[1][1] - XIF*S4438[1][3]) + v38[1]*(-(ZIF*S4438[1][2]) + YIF*S4438[1][3]);
v44[5]=v38[4]*S4438[2][1] + v38[5]*S4438[2][2] + v38[3]*(-(YIF*S4438[2][1]) + XIF*S4438[2][2]) + v38[6]*S4438[2][3] + v38[2]*(ZIF*S4438[2][1] - XIF*S4438[2][3]) + v38[1]*(-(ZIF*S4438[2][2]) + YIF*S4438[2][3]);
v44[6]=XIF*v38[3]*S4438[3][2] + v38[5]*S4438[3][2] - XIF*v38[2]*S4438[3][3] + v38[6]*S4438[3][3] + v38[1]*(-(ZIF*S4438[3][2]) + YIF*S4438[3][3]);

v45[1]=v44[1]*S4544[1][1] + v44[2]*S4544[1][2];
v45[2]=v44[1]*S4544[2][1] + v44[2]*S4544[2][2];
v45[3]=v44[3];
v45[4]=v44[4]*S4544[1][1] + F1SEG*v44[3]*S4544[1][2] + v44[5]*S4544[1][2];
v45[5]=v44[4]*S4544[2][1] + F1SEG*v44[3]*S4544[2][2] + v44[5]*S4544[2][2];
v45[6]=-(F1SEG*v44[2]) + v44[6];

v46[1]=v45[1]*S4645[1][1] + v45[2]*S4645[1][2];
v46[2]=v45[1]*S4645[2][1] + v45[2]*S4645[2][2];
v46[3]=v45[3];
v46[4]=v45[4]*S4645[1][1] + F2SEG*v45[3]*S4645[1][2] + v45[5]*S4645[1][2];
v46[5]=v45[4]*S4645[2][1] + F2SEG*v45[3]*S4645[2][2] + v45[5]*S4645[2][2];
v46[6]=-(F2SEG*v45[2]) + v45[6];

v47[1]=v46[1];
v47[2]=v46[2];
v47[3]=v46[3];
v47[4]=v46[4];
v47[5]=F3SEG*v46[3] + v46[5];
v47[6]=-(F3SEG*v46[2]) + v46[6];

v48[1]=v38[1]*S4838[1][1] + v38[2]*S4838[1][2] + v38[3]*S4838[1][3];
v48[2]=v38[1]*S4838[2][1] + v38[2]*S4838[2][2] + v38[3]*S4838[2][3];
v48[3]=state[48].thd + v38[2]*S4838[3][2] + v38[3]*S4838[3][3];
v48[4]=v38[4]*S4838[1][1] + v38[5]*S4838[1][2] + v38[3]*(-(YMF*S4838[1][1]) + XMF*S4838[1][2]) + v38[6]*S4838[1][3] + v38[2]*(ZMF*S4838[1][1] - XMF*S4838[1][3]) + v38[1]*(-(ZMF*S4838[1][2]) + YMF*S4838[1][3]);
v48[5]=v38[4]*S4838[2][1] + v38[5]*S4838[2][2] + v38[3]*(-(YMF*S4838[2][1]) + XMF*S4838[2][2]) + v38[6]*S4838[2][3] + v38[2]*(ZMF*S4838[2][1] - XMF*S4838[2][3]) + v38[1]*(-(ZMF*S4838[2][2]) + YMF*S4838[2][3]);
v48[6]=XMF*v38[3]*S4838[3][2] + v38[5]*S4838[3][2] - XMF*v38[2]*S4838[3][3] + v38[6]*S4838[3][3] + v38[1]*(-(ZMF*S4838[3][2]) + YMF*S4838[3][3]);

v49[1]=v48[1]*S4948[1][1] + v48[2]*S4948[1][2];
v49[2]=v48[1]*S4948[2][1] + v48[2]*S4948[2][2];
v49[3]=v48[3];
v49[4]=v48[4]*S4948[1][1] + F1SEG*v48[3]*S4948[1][2] + v48[5]*S4948[1][2];
v49[5]=v48[4]*S4948[2][1] + F1SEG*v48[3]*S4948[2][2] + v48[5]*S4948[2][2];
v49[6]=-(F1SEG*v48[2]) + v48[6];

v50[1]=v49[1]*S5049[1][1] + v49[2]*S5049[1][2];
v50[2]=v49[1]*S5049[2][1] + v49[2]*S5049[2][2];
v50[3]=v49[3];
v50[4]=v49[4]*S5049[1][1] + F2SEG*v49[3]*S5049[1][2] + v49[5]*S5049[1][2];
v50[5]=v49[4]*S5049[2][1] + F2SEG*v49[3]*S5049[2][2] + v49[5]*S5049[2][2];
v50[6]=-(F2SEG*v49[2]) + v49[6];

v51[1]=v50[1];
v51[2]=v50[2];
v51[3]=v50[3];
v51[4]=v50[4];
v51[5]=F3SEG*v50[3] + v50[5];
v51[6]=-(F3SEG*v50[2]) + v50[6];

v52[1]=v38[1]*S5238[1][1] + v38[2]*S5238[1][2] + v38[3]*S5238[1][3];
v52[2]=v38[1]*S5238[2][1] + v38[2]*S5238[2][2] + v38[3]*S5238[2][3];
v52[3]=state[49].thd + v38[2]*S5238[3][2] + v38[3]*S5238[3][3];
v52[4]=v38[4]*S5238[1][1] + v38[5]*S5238[1][2] + v38[3]*(-(YRF*S5238[1][1]) + XRF*S5238[1][2]) + v38[6]*S5238[1][3] + v38[2]*(ZRF*S5238[1][1] - XRF*S5238[1][3]) + v38[1]*(-(ZRF*S5238[1][2]) + YRF*S5238[1][3]);
v52[5]=v38[4]*S5238[2][1] + v38[5]*S5238[2][2] + v38[3]*(-(YRF*S5238[2][1]) + XRF*S5238[2][2]) + v38[6]*S5238[2][3] + v38[2]*(ZRF*S5238[2][1] - XRF*S5238[2][3]) + v38[1]*(-(ZRF*S5238[2][2]) + YRF*S5238[2][3]);
v52[6]=XRF*v38[3]*S5238[3][2] + v38[5]*S5238[3][2] - XRF*v38[2]*S5238[3][3] + v38[6]*S5238[3][3] + v38[1]*(-(ZRF*S5238[3][2]) + YRF*S5238[3][3]);

v53[1]=v52[1]*S5352[1][1] + v52[2]*S5352[1][2];
v53[2]=v52[1]*S5352[2][1] + v52[2]*S5352[2][2];
v53[3]=v52[3];
v53[4]=v52[4]*S5352[1][1] + F1SEG*v52[3]*S5352[1][2] + v52[5]*S5352[1][2];
v53[5]=v52[4]*S5352[2][1] + F1SEG*v52[3]*S5352[2][2] + v52[5]*S5352[2][2];
v53[6]=-(F1SEG*v52[2]) + v52[6];

v54[1]=v53[1]*S5453[1][1] + v53[2]*S5453[1][2];
v54[2]=v53[1]*S5453[2][1] + v53[2]*S5453[2][2];
v54[3]=v53[3];
v54[4]=v53[4]*S5453[1][1] + F2SEG*v53[3]*S5453[1][2] + v53[5]*S5453[1][2];
v54[5]=v53[4]*S5453[2][1] + F2SEG*v53[3]*S5453[2][2] + v53[5]*S5453[2][2];
v54[6]=-(F2SEG*v53[2]) + v53[6];

v55[1]=v54[1];
v55[2]=v54[2];
v55[3]=v54[3];
v55[4]=v54[4];
v55[5]=F3SEG*v54[3] + v54[5];
v55[6]=-(F3SEG*v54[2]) + v54[6];

v56[1]=v38[1]*S5638[1][1] + v38[2]*S5638[1][2] + v38[3]*S5638[1][3];
v56[2]=v38[1]*S5638[2][1] + v38[2]*S5638[2][2] + v38[3]*S5638[2][3];
v56[3]=state[50].thd + v38[2]*S5638[3][2] + v38[3]*S5638[3][3];
v56[4]=v38[4]*S5638[1][1] + v38[5]*S5638[1][2] + v38[3]*(-(YLF*S5638[1][1]) + XLF*S5638[1][2]) + v38[6]*S5638[1][3] + v38[2]*(ZLF*S5638[1][1] - XLF*S5638[1][3]) + v38[1]*(-(ZLF*S5638[1][2]) + YLF*S5638[1][3]);
v56[5]=v38[4]*S5638[2][1] + v38[5]*S5638[2][2] + v38[3]*(-(YLF*S5638[2][1]) + XLF*S5638[2][2]) + v38[6]*S5638[2][3] + v38[2]*(ZLF*S5638[2][1] - XLF*S5638[2][3]) + v38[1]*(-(ZLF*S5638[2][2]) + YLF*S5638[2][3]);
v56[6]=XLF*v38[3]*S5638[3][2] + v38[5]*S5638[3][2] - XLF*v38[2]*S5638[3][3] + v38[6]*S5638[3][3] + v38[1]*(-(ZLF*S5638[3][2]) + YLF*S5638[3][3]);

v57[1]=v56[1]*S5756[1][1] + v56[2]*S5756[1][2];
v57[2]=v56[1]*S5756[2][1] + v56[2]*S5756[2][2];
v57[3]=v56[3];
v57[4]=v56[4]*S5756[1][1] + F1SEG*v56[3]*S5756[1][2] + v56[5]*S5756[1][2];
v57[5]=v56[4]*S5756[2][1] + F1SEG*v56[3]*S5756[2][2] + v56[5]*S5756[2][2];
v57[6]=-(F1SEG*v56[2]) + v56[6];

v58[1]=v57[1]*S5857[1][1] + v57[2]*S5857[1][2];
v58[2]=v57[1]*S5857[2][1] + v57[2]*S5857[2][2];
v58[3]=v57[3];
v58[4]=v57[4]*S5857[1][1] + F2SEG*v57[3]*S5857[1][2] + v57[5]*S5857[1][2];
v58[5]=v57[4]*S5857[2][1] + F2SEG*v57[3]*S5857[2][2] + v57[5]*S5857[2][2];
v58[6]=-(F2SEG*v57[2]) + v57[6];

v59[1]=v58[1];
v59[2]=v58[2];
v59[3]=v58[3];
v59[4]=v58[4];
v59[5]=F3SEG*v58[3] + v58[5];
v59[6]=-(F3SEG*v58[2]) + v58[6];

v60[1]=v3[1]*S603[1][1] + v3[2]*S603[1][2];
v60[2]=v3[1]*S603[2][1] + v3[2]*S603[2][2];
v60[3]=state[32].thd + v3[3];
v60[4]=v3[4]*S603[1][1] - THORAX2NECK*v3[3]*S603[1][2] + v3[5]*S603[1][2];
v60[5]=v3[4]*S603[2][1] - THORAX2NECK*v3[3]*S603[2][2] + v3[5]*S603[2][2];
v60[6]=THORAX2NECK*v3[2] + v3[6];

v61[1]=v60[2]*S6160[1][2] + v60[3]*S6160[1][3];
v61[2]=v60[2]*S6160[2][2] + v60[3]*S6160[2][3];
v61[3]=state[33].thd + v60[1];
v61[4]=v60[5]*S6160[1][2] - CERVICAL*v60[1]*S6160[1][3] + v60[6]*S6160[1][3];
v61[5]=v60[5]*S6160[2][2] - CERVICAL*v60[1]*S6160[2][3] + v60[6]*S6160[2][3];
v61[6]=CERVICAL*v60[3] + v60[4];

v62[1]=v61[1]*S6261[1][1] + v61[3]*S6261[1][3];
v62[2]=v61[1]*S6261[2][1] + v61[3]*S6261[2][3];
v62[3]=state[34].thd + v61[2];
v62[4]=v61[4]*S6261[1][1] + v61[6]*S6261[1][3];
v62[5]=v61[4]*S6261[2][1] + v61[6]*S6261[2][3];
v62[6]=v61[5];

v63[1]=v62[1]*S6362[1][1] + v62[2]*S6362[1][2];
v63[2]=v62[1]*S6362[2][1] + v62[2]*S6362[2][2];
v63[3]=state[35].thd + v62[3];
v63[4]=-(HEAD*v62[2]*S6362[1][1]) + v62[4]*S6362[1][1] + HEAD*v62[1]*S6362[1][2] + v62[5]*S6362[1][2] + v62[3]*(EYEYOFF*S6362[1][1] + EYEXOFF*S6362[1][2]);
v63[5]=-(HEAD*v62[2]*S6362[2][1]) + v62[4]*S6362[2][1] + HEAD*v62[1]*S6362[2][2] + v62[5]*S6362[2][2] + v62[3]*(EYEYOFF*S6362[2][1] + EYEXOFF*S6362[2][2]);
v63[6]=-(EYEYOFF*v62[1]) - EYEXOFF*v62[2] + v62[6];

v64[1]=v63[2]*S6463[1][2] + v63[3]*S6463[1][3];
v64[2]=v63[2]*S6463[2][2] + v63[3]*S6463[2][3];
v64[3]=state[36].thd - v63[1];
v64[4]=v63[5]*S6463[1][2] + v63[6]*S6463[1][3];
v64[5]=v63[5]*S6463[2][2] + v63[6]*S6463[2][3];
v64[6]=-v63[4];

v65[1]=-v64[3];
v65[2]=-v64[2];
v65[3]=-v64[1];
v65[4]=EYE*v64[1] - v64[6];
v65[5]=-v64[5];
v65[6]=-(EYE*v64[3]) - v64[4];

v66[1]=v62[1]*S6662[1][1] + v62[2]*S6662[1][2];
v66[2]=v62[1]*S6662[2][1] + v62[2]*S6662[2][2];
v66[3]=state[37].thd + v62[3];
v66[4]=-(HEAD*v62[2]*S6662[1][1]) + v62[4]*S6662[1][1] + HEAD*v62[1]*S6662[1][2] + v62[5]*S6662[1][2] + v62[3]*(EYEYOFF*S6662[1][1] - EYEXOFF*S6662[1][2]);
v66[5]=-(HEAD*v62[2]*S6662[2][1]) + v62[4]*S6662[2][1] + HEAD*v62[1]*S6662[2][2] + v62[5]*S6662[2][2] + v62[3]*(EYEYOFF*S6662[2][1] - EYEXOFF*S6662[2][2]);
v66[6]=-(EYEYOFF*v62[1]) + EYEXOFF*v62[2] + v62[6];

v67[1]=v66[2]*S6766[1][2] + v66[3]*S6766[1][3];
v67[2]=v66[2]*S6766[2][2] + v66[3]*S6766[2][3];
v67[3]=state[38].thd - v66[1];
v67[4]=v66[5]*S6766[1][2] + v66[6]*S6766[1][3];
v67[5]=v66[5]*S6766[2][2] + v66[6]*S6766[2][3];
v67[6]=-v66[4];

v68[1]=-v67[3];
v68[2]=-v67[2];
v68[3]=-v67[1];
v68[4]=EYE*v67[1] - v67[6];
v68[5]=-v67[5];
v68[6]=-(EYE*v67[3]) - v67[4];

v69[1]=v62[1];
v69[2]=-v62[2];
v69[3]=-v62[3];
v69[4]=-(TOPofHEAD*v62[2]) + v62[4];
v69[5]=-(TOPofHEAD*v62[1]) - v62[5];
v69[6]=-v62[6];

v70[1]=v0[1]*S700[1][1] + v0[3]*S700[1][3];
v70[2]=v0[1]*S700[2][1] + v0[3]*S700[2][3];
v70[3]=state[23].thd + v0[2];
v70[4]=v0[4]*S700[1][1] - XHIP*v0[2]*S700[1][3] + v0[6]*S700[1][3];
v70[5]=v0[4]*S700[2][1] - XHIP*v0[2]*S700[2][3] + v0[6]*S700[2][3];
v70[6]=XHIP*v0[3] + v0[5];

v71[1]=v70[1]*S7170[1][1] + v70[3]*S7170[1][3];
v71[2]=v70[1]*S7170[2][1] + v70[3]*S7170[2][3];
v71[3]=state[22].thd - v70[2];
v71[4]=v70[4]*S7170[1][1] + v70[6]*S7170[1][3];
v71[5]=v70[4]*S7170[2][1] + v70[6]*S7170[2][3];
v71[6]=-v70[5];

v72[1]=v71[1]*S7271[1][1] + v71[2]*S7271[1][2] + v71[3]*S7271[1][3];
v72[2]=v71[1]*S7271[2][1] + v71[2]*S7271[2][2] + v71[3]*S7271[2][3];
v72[3]=state[24].thd + v71[1]*S7271[3][1] + v71[2]*S7271[3][2];
v72[4]=v71[4]*S7271[1][1] + YHIP*v71[3]*S7271[1][2] + v71[5]*S7271[1][2] - YHIP*v71[2]*S7271[1][3] + v71[6]*S7271[1][3];
v72[5]=v71[4]*S7271[2][1] + YHIP*v71[3]*S7271[2][2] + v71[5]*S7271[2][2] - YHIP*v71[2]*S7271[2][3] + v71[6]*S7271[2][3];
v72[6]=v71[4]*S7271[3][1] + YHIP*v71[3]*S7271[3][2] + v71[5]*S7271[3][2];

v73[1]=v72[1]*S7372[1][1] + v72[3]*S7372[1][3];
v73[2]=v72[1]*S7372[2][1] + v72[3]*S7372[2][3];
v73[3]=state[25].thd + v72[2];
v73[4]=v72[4]*S7372[1][1] + v72[6]*S7372[1][3] + v72[2]*(UPPERLEGMOD*S7372[1][1] - YKNEE*S7372[1][3]);
v73[5]=v72[4]*S7372[2][1] + v72[6]*S7372[2][3] + v72[2]*(UPPERLEGMOD*S7372[2][1] - YKNEE*S7372[2][3]);
v73[6]=-(UPPERLEGMOD*v72[1]) + YKNEE*v72[3] + v72[5];

v74[1]=v73[1]*S7473[1][1] + v73[3]*S7473[1][3];
v74[2]=v73[1]*S7473[2][1] + v73[3]*S7473[2][3];
v74[3]=state[26].thd - v73[2];
v74[4]=v73[4]*S7473[1][1] + v73[6]*S7473[1][3];
v74[5]=v73[4]*S7473[2][1] + v73[6]*S7473[2][3];
v74[6]=-v73[5];

v75[1]=v74[1]*S7574[1][1] + v74[3]*S7574[1][3];
v75[2]=v74[1]*S7574[2][1] + v74[3]*S7574[2][3];
v75[3]=state[27].thd - v74[2];
v75[4]=LOWERLEG*v74[2]*S7574[1][1] + v74[4]*S7574[1][1] + v74[6]*S7574[1][3];
v75[5]=LOWERLEG*v74[2]*S7574[2][1] + v74[4]*S7574[2][1] + v74[6]*S7574[2][3];
v75[6]=LOWERLEG*v74[1] - v74[5];

v76[1]=v75[1]*S7675[1][1] + v75[3]*S7675[1][3];
v76[2]=v75[1]*S7675[2][1] + v75[3]*S7675[2][3];
v76[3]=state[28].thd + v75[2];
v76[4]=v75[4]*S7675[1][1] + v75[6]*S7675[1][3];
v76[5]=v75[4]*S7675[2][1] + v75[6]*S7675[2][3];
v76[6]=v75[5];

v77[1]=v76[1];
v77[2]=v76[2];
v77[3]=v76[3];
v77[4]=YTOE*v76[2] + XTOEOUTER*v76[3] + v76[4];
v77[5]=-(YTOE*v76[1]) + ZTOE*v76[3] + v76[5];
v77[6]=-(XTOEOUTER*v76[1]) - ZTOE*v76[2] + v76[6];

v78[1]=v76[1];
v78[2]=v76[2];
v78[3]=v76[3];
v78[4]=YTOE*v76[2] - XTOEINNER*v76[3] + v76[4];
v78[5]=-(YTOE*v76[1]) + ZTOE*v76[3] + v76[5];
v78[6]=XTOEINNER*v76[1] - ZTOE*v76[2] + v76[6];

v79[1]=v76[1];
v79[2]=v76[2];
v79[3]=v76[3];
v79[4]=YMETATARSAL*v76[2] + XMETATARSALOUTER*v76[3] + v76[4];
v79[5]=-(YMETATARSAL*v76[1]) + ZTOE*v76[3] + v76[5];
v79[6]=-(XMETATARSALOUTER*v76[1]) - ZTOE*v76[2] + v76[6];

v80[1]=v76[1];
v80[2]=v76[2];
v80[3]=v76[3];
v80[4]=YMETATARSAL*v76[2] - XMETATARSALINNER*v76[3] + v76[4];
v80[5]=-(YMETATARSAL*v76[1]) + ZTOE*v76[3] + v76[5];
v80[6]=XMETATARSALINNER*v76[1] - ZTOE*v76[2] + v76[6];

v81[1]=v76[1];
v81[2]=v76[2];
v81[3]=v76[3];
v81[4]=-(YHEEL*v76[2]) + XHEELOUTER*v76[3] + v76[4];
v81[5]=YHEEL*v76[1] + ZHEEL*v76[3] + v76[5];
v81[6]=-(XHEELOUTER*v76[1]) - ZHEEL*v76[2] + v76[6];

v82[1]=v76[1];
v82[2]=v76[2];
v82[3]=v76[3];
v82[4]=-(YHEEL*v76[2]) - XHEELINNER*v76[3] + v76[4];
v82[5]=YHEEL*v76[1] + ZHEEL*v76[3] + v76[5];
v82[6]=XHEELINNER*v76[1] - ZHEEL*v76[2] + v76[6];

v83[1]=v76[1]*S8376[1][1] + v76[2]*S8376[1][2] + v76[3]*S8376[1][3];
v83[2]=v76[1]*S8376[2][1] + v76[2]*S8376[2][2] + v76[3]*S8376[2][3];
v83[3]=v76[1]*S8376[3][1] + v76[2]*S8376[3][2] + v76[3]*S8376[3][3];
v83[4]=v76[4]*S8376[1][1] + v76[5]*S8376[1][2] + v76[3]*(-(eff[3].x[2]*S8376[1][1]) + eff[3].x[1]*S8376[1][2]) + v76[6]*S8376[1][3] + v76[2]*(eff[3].x[3]*S8376[1][1] - eff[3].x[1]*S8376[1][3]) + v76[1]*(-(eff[3].x[3]*S8376[1][2]) + eff[3].x[2]*S8376[1][3]);
v83[5]=v76[4]*S8376[2][1] + v76[5]*S8376[2][2] + v76[3]*(-(eff[3].x[2]*S8376[2][1]) + eff[3].x[1]*S8376[2][2]) + v76[6]*S8376[2][3] + v76[2]*(eff[3].x[3]*S8376[2][1] - eff[3].x[1]*S8376[2][3]) + v76[1]*(-(eff[3].x[3]*S8376[2][2]) + eff[3].x[2]*S8376[2][3]);
v83[6]=v76[4]*S8376[3][1] + v76[5]*S8376[3][2] + v76[3]*(-(eff[3].x[2]*S8376[3][1]) + eff[3].x[1]*S8376[3][2]) + v76[6]*S8376[3][3] + v76[2]*(eff[3].x[3]*S8376[3][1] - eff[3].x[1]*S8376[3][3]) + v76[1]*(-(eff[3].x[3]*S8376[3][2]) + eff[3].x[2]*S8376[3][3]);

v84[1]=v0[1]*S840[1][1] + v0[3]*S840[1][3];
v84[2]=v0[1]*S840[2][1] + v0[3]*S840[2][3];
v84[3]=state[16].thd - v0[2];
v84[4]=v0[4]*S840[1][1] + XHIP*v0[2]*S840[1][3] + v0[6]*S840[1][3];
v84[5]=v0[4]*S840[2][1] + XHIP*v0[2]*S840[2][3] + v0[6]*S840[2][3];
v84[6]=XHIP*v0[3] - v0[5];

v85[1]=v84[1]*S8584[1][1] + v84[3]*S8584[1][3];
v85[2]=v84[1]*S8584[2][1] + v84[3]*S8584[2][3];
v85[3]=state[15].thd + v84[2];
v85[4]=v84[4]*S8584[1][1] + v84[6]*S8584[1][3];
v85[5]=v84[4]*S8584[2][1] + v84[6]*S8584[2][3];
v85[6]=v84[5];

v86[1]=v85[1]*S8685[1][1] + v85[2]*S8685[1][2] + v85[3]*S8685[1][3];
v86[2]=v85[1]*S8685[2][1] + v85[2]*S8685[2][2] + v85[3]*S8685[2][3];
v86[3]=state[17].thd + v85[1]*S8685[3][1] + v85[2]*S8685[3][2];
v86[4]=v85[4]*S8685[1][1] + YHIP*v85[3]*S8685[1][2] + v85[5]*S8685[1][2] - YHIP*v85[2]*S8685[1][3] + v85[6]*S8685[1][3];
v86[5]=v85[4]*S8685[2][1] + YHIP*v85[3]*S8685[2][2] + v85[5]*S8685[2][2] - YHIP*v85[2]*S8685[2][3] + v85[6]*S8685[2][3];
v86[6]=v85[4]*S8685[3][1] + YHIP*v85[3]*S8685[3][2] + v85[5]*S8685[3][2];

v87[1]=v86[1]*S8786[1][1] + v86[3]*S8786[1][3];
v87[2]=v86[1]*S8786[2][1] + v86[3]*S8786[2][3];
v87[3]=state[18].thd - v86[2];
v87[4]=v86[4]*S8786[1][1] + v86[6]*S8786[1][3] + v86[2]*(-(UPPERLEGMOD*S8786[1][1]) - YKNEE*S8786[1][3]);
v87[5]=v86[4]*S8786[2][1] + v86[6]*S8786[2][3] + v86[2]*(-(UPPERLEGMOD*S8786[2][1]) - YKNEE*S8786[2][3]);
v87[6]=-(UPPERLEGMOD*v86[1]) - YKNEE*v86[3] - v86[5];

v88[1]=v87[1]*S8887[1][1] + v87[3]*S8887[1][3];
v88[2]=v87[1]*S8887[2][1] + v87[3]*S8887[2][3];
v88[3]=state[19].thd + v87[2];
v88[4]=v87[4]*S8887[1][1] + v87[6]*S8887[1][3];
v88[5]=v87[4]*S8887[2][1] + v87[6]*S8887[2][3];
v88[6]=v87[5];

v89[1]=v88[1]*S8988[1][1] + v88[3]*S8988[1][3];
v89[2]=v88[1]*S8988[2][1] + v88[3]*S8988[2][3];
v89[3]=state[20].thd + v88[2];
v89[4]=-(LOWERLEG*v88[2]*S8988[1][1]) + v88[4]*S8988[1][1] + v88[6]*S8988[1][3];
v89[5]=-(LOWERLEG*v88[2]*S8988[2][1]) + v88[4]*S8988[2][1] + v88[6]*S8988[2][3];
v89[6]=LOWERLEG*v88[1] + v88[5];

v90[1]=v89[1]*S9089[1][1] + v89[3]*S9089[1][3];
v90[2]=v89[1]*S9089[2][1] + v89[3]*S9089[2][3];
v90[3]=state[21].thd - v89[2];
v90[4]=v89[4]*S9089[1][1] + v89[6]*S9089[1][3];
v90[5]=v89[4]*S9089[2][1] + v89[6]*S9089[2][3];
v90[6]=-v89[5];

v91[1]=v90[1];
v91[2]=v90[2];
v91[3]=v90[3];
v91[4]=-(YTOE*v90[2]) + XTOEOUTER*v90[3] + v90[4];
v91[5]=YTOE*v90[1] + ZTOE*v90[3] + v90[5];
v91[6]=-(XTOEOUTER*v90[1]) - ZTOE*v90[2] + v90[6];

v92[1]=v90[1];
v92[2]=v90[2];
v92[3]=v90[3];
v92[4]=-(YTOE*v90[2]) - XTOEINNER*v90[3] + v90[4];
v92[5]=YTOE*v90[1] + ZTOE*v90[3] + v90[5];
v92[6]=XTOEINNER*v90[1] - ZTOE*v90[2] + v90[6];

v93[1]=v90[1];
v93[2]=v90[2];
v93[3]=v90[3];
v93[4]=-(YMETATARSAL*v90[2]) + XMETATARSALOUTER*v90[3] + v90[4];
v93[5]=YMETATARSAL*v90[1] + ZTOE*v90[3] + v90[5];
v93[6]=-(XMETATARSALOUTER*v90[1]) - ZTOE*v90[2] + v90[6];

v94[1]=v90[1];
v94[2]=v90[2];
v94[3]=v90[3];
v94[4]=-(YMETATARSAL*v90[2]) - XMETATARSALINNER*v90[3] + v90[4];
v94[5]=YMETATARSAL*v90[1] + ZTOE*v90[3] + v90[5];
v94[6]=XMETATARSALINNER*v90[1] - ZTOE*v90[2] + v90[6];

v95[1]=v90[1];
v95[2]=v90[2];
v95[3]=v90[3];
v95[4]=YHEEL*v90[2] + XHEELOUTER*v90[3] + v90[4];
v95[5]=-(YHEEL*v90[1]) + ZHEEL*v90[3] + v90[5];
v95[6]=-(XHEELOUTER*v90[1]) - ZHEEL*v90[2] + v90[6];

v96[1]=v90[1];
v96[2]=v90[2];
v96[3]=v90[3];
v96[4]=YHEEL*v90[2] - XHEELINNER*v90[3] + v90[4];
v96[5]=-(YHEEL*v90[1]) + ZHEEL*v90[3] + v90[5];
v96[6]=XHEELINNER*v90[1] - ZHEEL*v90[2] + v90[6];

v97[1]=v90[1]*S9790[1][1] + v90[2]*S9790[1][2] + v90[3]*S9790[1][3];
v97[2]=v90[1]*S9790[2][1] + v90[2]*S9790[2][2] + v90[3]*S9790[2][3];
v97[3]=v90[1]*S9790[3][1] + v90[2]*S9790[3][2] + v90[3]*S9790[3][3];
v97[4]=v90[4]*S9790[1][1] + v90[5]*S9790[1][2] + v90[3]*(-(eff[4].x[2]*S9790[1][1]) + eff[4].x[1]*S9790[1][2]) + v90[6]*S9790[1][3] + v90[2]*(eff[4].x[3]*S9790[1][1] - eff[4].x[1]*S9790[1][3]) + v90[1]*(-(eff[4].x[3]*S9790[1][2]) + eff[4].x[2]*S9790[1][3]);
v97[5]=v90[4]*S9790[2][1] + v90[5]*S9790[2][2] + v90[3]*(-(eff[4].x[2]*S9790[2][1]) + eff[4].x[1]*S9790[2][2]) + v90[6]*S9790[2][3] + v90[2]*(eff[4].x[3]*S9790[2][1] - eff[4].x[1]*S9790[2][3]) + v90[1]*(-(eff[4].x[3]*S9790[2][2]) + eff[4].x[2]*S9790[2][3]);
v97[6]=v90[4]*S9790[3][1] + v90[5]*S9790[3][2] + v90[3]*(-(eff[4].x[2]*S9790[3][1]) + eff[4].x[1]*S9790[3][2]) + v90[6]*S9790[3][3] + v90[2]*(eff[4].x[3]*S9790[3][1] - eff[4].x[1]*S9790[3][3]) + v90[1]*(-(eff[4].x[3]*S9790[3][2]) + eff[4].x[2]*S9790[3][3]);



}


void
hermes_InvDynArtfunc5(void)
     {
/* c-misc vectors */
c1[1]=state[29].thd*v1[2];
c1[2]=-(state[29].thd*v1[1]);
c1[4]=state[29].thd*v1[5];
c1[5]=-(state[29].thd*v1[4]);

c2[1]=state[30].thd*v2[2];
c2[2]=-(state[30].thd*v2[1]);
c2[4]=state[30].thd*v2[5];
c2[5]=-(state[30].thd*v2[4]);

c3[1]=state[31].thd*v3[2];
c3[2]=-(state[31].thd*v3[1]);
c3[4]=state[31].thd*v3[5];
c3[5]=-(state[31].thd*v3[4]);

c4[1]=state[1].thd*v4[2];
c4[2]=-(state[1].thd*v4[1]);
c4[4]=state[1].thd*v4[5];
c4[5]=-(state[1].thd*v4[4]);

c5[1]=state[2].thd*v5[2];
c5[2]=-(state[2].thd*v5[1]);
c5[4]=state[2].thd*v5[5];
c5[5]=-(state[2].thd*v5[4]);

c6[1]=state[3].thd*v6[2];
c6[2]=-(state[3].thd*v6[1]);
c6[4]=state[3].thd*v6[5];
c6[5]=-(state[3].thd*v6[4]);

c7[1]=state[4].thd*v7[2];
c7[2]=-(state[4].thd*v7[1]);
c7[4]=state[4].thd*v7[5];
c7[5]=-(state[4].thd*v7[4]);

c8[1]=state[5].thd*v8[2];
c8[2]=-(state[5].thd*v8[1]);
c8[4]=state[5].thd*v8[5];
c8[5]=-(state[5].thd*v8[4]);

c9[1]=state[6].thd*v9[2];
c9[2]=-(state[6].thd*v9[1]);
c9[4]=state[6].thd*v9[5];
c9[5]=-(state[6].thd*v9[4]);

c10[1]=state[7].thd*v10[2];
c10[2]=-(state[7].thd*v10[1]);
c10[4]=state[7].thd*v10[5];
c10[5]=-(state[7].thd*v10[4]);

c12[2]=state[39].thd*v12[3];
c12[3]=-(state[39].thd*v12[2]);
c12[5]=state[39].thd*v12[6];
c12[6]=-(state[39].thd*v12[5]);

c13[1]=-(state[40].thd*v13[2]);
c13[2]=state[40].thd*v13[1];
c13[4]=-(state[40].thd*v13[5]);
c13[5]=state[40].thd*v13[4];

c16[1]=state[41].thd*v16[2];
c16[2]=-(state[41].thd*v16[1]);
c16[4]=state[41].thd*v16[5];
c16[5]=-(state[41].thd*v16[4]);

c20[1]=state[42].thd*v20[2];
c20[2]=-(state[42].thd*v20[1]);
c20[4]=state[42].thd*v20[5];
c20[5]=-(state[42].thd*v20[4]);

c24[1]=state[43].thd*v24[2];
c24[2]=-(state[43].thd*v24[1]);
c24[4]=state[43].thd*v24[5];
c24[5]=-(state[43].thd*v24[4]);

c28[1]=state[44].thd*v28[2];
c28[2]=-(state[44].thd*v28[1]);
c28[4]=state[44].thd*v28[5];
c28[5]=-(state[44].thd*v28[4]);

c32[1]=state[8].thd*v32[2];
c32[2]=-(state[8].thd*v32[1]);
c32[4]=state[8].thd*v32[5];
c32[5]=-(state[8].thd*v32[4]);

c33[1]=state[9].thd*v33[2];
c33[2]=-(state[9].thd*v33[1]);
c33[4]=state[9].thd*v33[5];
c33[5]=-(state[9].thd*v33[4]);

c34[1]=state[10].thd*v34[2];
c34[2]=-(state[10].thd*v34[1]);
c34[4]=state[10].thd*v34[5];
c34[5]=-(state[10].thd*v34[4]);

c35[1]=state[11].thd*v35[2];
c35[2]=-(state[11].thd*v35[1]);
c35[4]=state[11].thd*v35[5];
c35[5]=-(state[11].thd*v35[4]);

c36[1]=state[12].thd*v36[2];
c36[2]=-(state[12].thd*v36[1]);
c36[4]=state[12].thd*v36[5];
c36[5]=-(state[12].thd*v36[4]);

c37[1]=state[13].thd*v37[2];
c37[2]=-(state[13].thd*v37[1]);
c37[4]=state[13].thd*v37[5];
c37[5]=-(state[13].thd*v37[4]);

c38[1]=state[14].thd*v38[2];
c38[2]=-(state[14].thd*v38[1]);
c38[4]=state[14].thd*v38[5];
c38[5]=-(state[14].thd*v38[4]);

c40[2]=state[45].thd*v40[3];
c40[3]=-(state[45].thd*v40[2]);
c40[5]=state[45].thd*v40[6];
c40[6]=-(state[45].thd*v40[5]);

c41[1]=-(state[46].thd*v41[2]);
c41[2]=state[46].thd*v41[1];
c41[4]=-(state[46].thd*v41[5]);
c41[5]=state[46].thd*v41[4];

c44[1]=state[47].thd*v44[2];
c44[2]=-(state[47].thd*v44[1]);
c44[4]=state[47].thd*v44[5];
c44[5]=-(state[47].thd*v44[4]);

c48[1]=state[48].thd*v48[2];
c48[2]=-(state[48].thd*v48[1]);
c48[4]=state[48].thd*v48[5];
c48[5]=-(state[48].thd*v48[4]);

c52[1]=state[49].thd*v52[2];
c52[2]=-(state[49].thd*v52[1]);
c52[4]=state[49].thd*v52[5];
c52[5]=-(state[49].thd*v52[4]);

c56[1]=state[50].thd*v56[2];
c56[2]=-(state[50].thd*v56[1]);
c56[4]=state[50].thd*v56[5];
c56[5]=-(state[50].thd*v56[4]);

c60[1]=state[32].thd*v60[2];
c60[2]=-(state[32].thd*v60[1]);
c60[4]=state[32].thd*v60[5];
c60[5]=-(state[32].thd*v60[4]);

c61[1]=state[33].thd*v61[2];
c61[2]=-(state[33].thd*v61[1]);
c61[4]=state[33].thd*v61[5];
c61[5]=-(state[33].thd*v61[4]);

c62[1]=state[34].thd*v62[2];
c62[2]=-(state[34].thd*v62[1]);
c62[4]=state[34].thd*v62[5];
c62[5]=-(state[34].thd*v62[4]);

c63[1]=state[35].thd*v63[2];
c63[2]=-(state[35].thd*v63[1]);
c63[4]=state[35].thd*v63[5];
c63[5]=-(state[35].thd*v63[4]);

c64[1]=state[36].thd*v64[2];
c64[2]=-(state[36].thd*v64[1]);
c64[4]=state[36].thd*v64[5];
c64[5]=-(state[36].thd*v64[4]);

c66[1]=state[37].thd*v66[2];
c66[2]=-(state[37].thd*v66[1]);
c66[4]=state[37].thd*v66[5];
c66[5]=-(state[37].thd*v66[4]);

c67[1]=state[38].thd*v67[2];
c67[2]=-(state[38].thd*v67[1]);
c67[4]=state[38].thd*v67[5];
c67[5]=-(state[38].thd*v67[4]);

c70[1]=state[23].thd*v70[2];
c70[2]=-(state[23].thd*v70[1]);
c70[4]=state[23].thd*v70[5];
c70[5]=-(state[23].thd*v70[4]);

c71[1]=state[22].thd*v71[2];
c71[2]=-(state[22].thd*v71[1]);
c71[4]=state[22].thd*v71[5];
c71[5]=-(state[22].thd*v71[4]);

c72[1]=state[24].thd*v72[2];
c72[2]=-(state[24].thd*v72[1]);
c72[4]=state[24].thd*v72[5];
c72[5]=-(state[24].thd*v72[4]);

c73[1]=state[25].thd*v73[2];
c73[2]=-(state[25].thd*v73[1]);
c73[4]=state[25].thd*v73[5];
c73[5]=-(state[25].thd*v73[4]);

c74[1]=state[26].thd*v74[2];
c74[2]=-(state[26].thd*v74[1]);
c74[4]=state[26].thd*v74[5];
c74[5]=-(state[26].thd*v74[4]);

c75[1]=state[27].thd*v75[2];
c75[2]=-(state[27].thd*v75[1]);
c75[4]=state[27].thd*v75[5];
c75[5]=-(state[27].thd*v75[4]);

c76[1]=state[28].thd*v76[2];
c76[2]=-(state[28].thd*v76[1]);
c76[4]=state[28].thd*v76[5];
c76[5]=-(state[28].thd*v76[4]);

c84[1]=state[16].thd*v84[2];
c84[2]=-(state[16].thd*v84[1]);
c84[4]=state[16].thd*v84[5];
c84[5]=-(state[16].thd*v84[4]);

c85[1]=state[15].thd*v85[2];
c85[2]=-(state[15].thd*v85[1]);
c85[4]=state[15].thd*v85[5];
c85[5]=-(state[15].thd*v85[4]);

c86[1]=state[17].thd*v86[2];
c86[2]=-(state[17].thd*v86[1]);
c86[4]=state[17].thd*v86[5];
c86[5]=-(state[17].thd*v86[4]);

c87[1]=state[18].thd*v87[2];
c87[2]=-(state[18].thd*v87[1]);
c87[4]=state[18].thd*v87[5];
c87[5]=-(state[18].thd*v87[4]);

c88[1]=state[19].thd*v88[2];
c88[2]=-(state[19].thd*v88[1]);
c88[4]=state[19].thd*v88[5];
c88[5]=-(state[19].thd*v88[4]);

c89[1]=state[20].thd*v89[2];
c89[2]=-(state[20].thd*v89[1]);
c89[4]=state[20].thd*v89[5];
c89[5]=-(state[20].thd*v89[4]);

c90[1]=state[21].thd*v90[2];
c90[2]=-(state[21].thd*v90[1]);
c90[4]=state[21].thd*v90[5];
c90[5]=-(state[21].thd*v90[4]);



}


void
hermes_InvDynArtfunc6(void)
     {
/* pv-misc vectors */
pv0[1]=-uex[0].f[1] - links[0].mcm[1]*Power(v0[2],2) - links[0].mcm[1]*Power(v0[3],2) + v0[1]*(links[0].mcm[2]*v0[2] + links[0].mcm[3]*v0[3]) - links[0].m*v0[3]*v0[5] + links[0].m*v0[2]*v0[6] + gravity*links[0].m*S00[1][3];
pv0[2]=-uex[0].f[2] - links[0].mcm[2]*Power(v0[1],2) - links[0].mcm[2]*Power(v0[3],2) + v0[2]*(links[0].mcm[1]*v0[1] + links[0].mcm[3]*v0[3]) + links[0].m*v0[3]*v0[4] - links[0].m*v0[1]*v0[6] + gravity*links[0].m*S00[2][3];
pv0[3]=-uex[0].f[3] - links[0].mcm[3]*Power(v0[1],2) - links[0].mcm[3]*Power(v0[2],2) + (links[0].mcm[1]*v0[1] + links[0].mcm[2]*v0[2])*v0[3] - links[0].m*v0[2]*v0[4] + links[0].m*v0[1]*v0[5] + gravity*links[0].m*S00[3][3];
pv0[4]=-uex[0].t[1] + (-(links[0].mcm[2]*v0[2]) - links[0].mcm[3]*v0[3])*v0[4] + (links[0].mcm[1]*v0[3] + links[0].m*v0[5])*v0[6] + v0[5]*(links[0].mcm[1]*v0[2] - links[0].m*v0[6]) + v0[1]*(links[0].mcm[2]*v0[5] + links[0].mcm[3]*v0[6] - v0[3]*links[0].inertia[1][2] + v0[2]*links[0].inertia[1][3]) + v0[2]*(-(links[0].mcm[1]*v0[5]) - v0[3]*links[0].inertia[2][2] + v0[2]*links[0].inertia[2][3]) + v0[3]*(-(links[0].mcm[1]*v0[6]) - v0[3]*links[0].inertia[2][3] + v0[2]*links[0].inertia[3][3]) - gravity*links[0].mcm[3]*S00[2][3] + gravity*links[0].mcm[2]*S00[3][3];
pv0[5]=-uex[0].t[2] + (-(links[0].mcm[1]*v0[1]) - links[0].mcm[3]*v0[3])*v0[5] + (links[0].mcm[2]*v0[3] - links[0].m*v0[4])*v0[6] + v0[4]*(links[0].mcm[2]*v0[1] + links[0].m*v0[6]) + v0[1]*(-(links[0].mcm[2]*v0[4]) + v0[3]*links[0].inertia[1][1] - v0[1]*links[0].inertia[1][3]) + v0[2]*(links[0].mcm[1]*v0[4] + links[0].mcm[3]*v0[6] + v0[3]*links[0].inertia[1][2] - v0[1]*links[0].inertia[2][3]) + v0[3]*(-(links[0].mcm[2]*v0[6]) + v0[3]*links[0].inertia[1][3] - v0[1]*links[0].inertia[3][3]) + gravity*links[0].mcm[3]*S00[1][3] - gravity*links[0].mcm[1]*S00[3][3];
pv0[6]=-uex[0].t[3] + (links[0].mcm[3]*v0[2] + links[0].m*v0[4])*v0[5] + v0[4]*(links[0].mcm[3]*v0[1] - links[0].m*v0[5]) + (-(links[0].mcm[1]*v0[1]) - links[0].mcm[2]*v0[2])*v0[6] + v0[1]*(-(links[0].mcm[3]*v0[4]) - v0[2]*links[0].inertia[1][1] + v0[1]*links[0].inertia[1][2]) + v0[2]*(-(links[0].mcm[3]*v0[5]) - v0[2]*links[0].inertia[1][2] + v0[1]*links[0].inertia[2][2]) + v0[3]*(links[0].mcm[1]*v0[4] + links[0].mcm[2]*v0[5] - v0[2]*links[0].inertia[1][3] + v0[1]*links[0].inertia[2][3]) - gravity*links[0].mcm[2]*S00[1][3] + gravity*links[0].mcm[1]*S00[2][3];

pv1[1]=-uex[29].f[1] - links[29].mcm[1]*Power(v1[2],2) - links[29].mcm[1]*Power(v1[3],2) + v1[1]*(links[29].mcm[2]*v1[2] + links[29].mcm[3]*v1[3]) - links[29].m*v1[3]*v1[5] + links[29].m*v1[2]*v1[6] + gravity*links[29].m*SG10[1][3];
pv1[2]=-uex[29].f[2] - links[29].mcm[2]*Power(v1[1],2) - links[29].mcm[2]*Power(v1[3],2) + v1[2]*(links[29].mcm[1]*v1[1] + links[29].mcm[3]*v1[3]) + links[29].m*v1[3]*v1[4] - links[29].m*v1[1]*v1[6] + gravity*links[29].m*SG10[2][3];
pv1[3]=-uex[29].f[3] - links[29].mcm[3]*Power(v1[1],2) - links[29].mcm[3]*Power(v1[2],2) + (links[29].mcm[1]*v1[1] + links[29].mcm[2]*v1[2])*v1[3] - links[29].m*v1[2]*v1[4] + links[29].m*v1[1]*v1[5] + gravity*links[29].m*SG10[3][3];
pv1[4]=-uex[29].t[1] + (-(links[29].mcm[2]*v1[2]) - links[29].mcm[3]*v1[3])*v1[4] + (links[29].mcm[1]*v1[3] + links[29].m*v1[5])*v1[6] + v1[5]*(links[29].mcm[1]*v1[2] - links[29].m*v1[6]) + v1[1]*(links[29].mcm[2]*v1[5] + links[29].mcm[3]*v1[6] - v1[3]*links[29].inertia[1][2] + v1[2]*links[29].inertia[1][3]) + v1[2]*(-(links[29].mcm[1]*v1[5]) - v1[3]*links[29].inertia[2][2] + v1[2]*links[29].inertia[2][3]) + v1[3]*(-(links[29].mcm[1]*v1[6]) - v1[3]*links[29].inertia[2][3] + v1[2]*links[29].inertia[3][3]) - gravity*links[29].mcm[3]*SG10[2][3] + gravity*links[29].mcm[2]*SG10[3][3];
pv1[5]=-uex[29].t[2] + (-(links[29].mcm[1]*v1[1]) - links[29].mcm[3]*v1[3])*v1[5] + (links[29].mcm[2]*v1[3] - links[29].m*v1[4])*v1[6] + v1[4]*(links[29].mcm[2]*v1[1] + links[29].m*v1[6]) + v1[1]*(-(links[29].mcm[2]*v1[4]) + v1[3]*links[29].inertia[1][1] - v1[1]*links[29].inertia[1][3]) + v1[2]*(links[29].mcm[1]*v1[4] + links[29].mcm[3]*v1[6] + v1[3]*links[29].inertia[1][2] - v1[1]*links[29].inertia[2][3]) + v1[3]*(-(links[29].mcm[2]*v1[6]) + v1[3]*links[29].inertia[1][3] - v1[1]*links[29].inertia[3][3]) + gravity*links[29].mcm[3]*SG10[1][3] - gravity*links[29].mcm[1]*SG10[3][3];
pv1[6]=-uex[29].t[3] + (links[29].mcm[3]*v1[2] + links[29].m*v1[4])*v1[5] + v1[4]*(links[29].mcm[3]*v1[1] - links[29].m*v1[5]) + (-(links[29].mcm[1]*v1[1]) - links[29].mcm[2]*v1[2])*v1[6] + v1[1]*(-(links[29].mcm[3]*v1[4]) - v1[2]*links[29].inertia[1][1] + v1[1]*links[29].inertia[1][2]) + v1[2]*(-(links[29].mcm[3]*v1[5]) - v1[2]*links[29].inertia[1][2] + v1[1]*links[29].inertia[2][2]) + v1[3]*(links[29].mcm[1]*v1[4] + links[29].mcm[2]*v1[5] - v1[2]*links[29].inertia[1][3] + v1[1]*links[29].inertia[2][3]) - gravity*links[29].mcm[2]*SG10[1][3] + gravity*links[29].mcm[1]*SG10[2][3];

pv2[1]=-uex[30].f[1] - links[30].mcm[1]*Power(v2[2],2) - links[30].mcm[1]*Power(v2[3],2) + v2[1]*(links[30].mcm[2]*v2[2] + links[30].mcm[3]*v2[3]) - links[30].m*v2[3]*v2[5] + links[30].m*v2[2]*v2[6] + gravity*links[30].m*SG20[1][3];
pv2[2]=-uex[30].f[2] - links[30].mcm[2]*Power(v2[1],2) - links[30].mcm[2]*Power(v2[3],2) + v2[2]*(links[30].mcm[1]*v2[1] + links[30].mcm[3]*v2[3]) + links[30].m*v2[3]*v2[4] - links[30].m*v2[1]*v2[6] + gravity*links[30].m*SG20[2][3];
pv2[3]=-uex[30].f[3] - links[30].mcm[3]*Power(v2[1],2) - links[30].mcm[3]*Power(v2[2],2) + (links[30].mcm[1]*v2[1] + links[30].mcm[2]*v2[2])*v2[3] - links[30].m*v2[2]*v2[4] + links[30].m*v2[1]*v2[5] + gravity*links[30].m*SG20[3][3];
pv2[4]=-uex[30].t[1] + (-(links[30].mcm[2]*v2[2]) - links[30].mcm[3]*v2[3])*v2[4] + (links[30].mcm[1]*v2[3] + links[30].m*v2[5])*v2[6] + v2[5]*(links[30].mcm[1]*v2[2] - links[30].m*v2[6]) + v2[1]*(links[30].mcm[2]*v2[5] + links[30].mcm[3]*v2[6] - v2[3]*links[30].inertia[1][2] + v2[2]*links[30].inertia[1][3]) + v2[2]*(-(links[30].mcm[1]*v2[5]) - v2[3]*links[30].inertia[2][2] + v2[2]*links[30].inertia[2][3]) + v2[3]*(-(links[30].mcm[1]*v2[6]) - v2[3]*links[30].inertia[2][3] + v2[2]*links[30].inertia[3][3]) - gravity*links[30].mcm[3]*SG20[2][3] + gravity*links[30].mcm[2]*SG20[3][3];
pv2[5]=-uex[30].t[2] + (-(links[30].mcm[1]*v2[1]) - links[30].mcm[3]*v2[3])*v2[5] + (links[30].mcm[2]*v2[3] - links[30].m*v2[4])*v2[6] + v2[4]*(links[30].mcm[2]*v2[1] + links[30].m*v2[6]) + v2[1]*(-(links[30].mcm[2]*v2[4]) + v2[3]*links[30].inertia[1][1] - v2[1]*links[30].inertia[1][3]) + v2[2]*(links[30].mcm[1]*v2[4] + links[30].mcm[3]*v2[6] + v2[3]*links[30].inertia[1][2] - v2[1]*links[30].inertia[2][3]) + v2[3]*(-(links[30].mcm[2]*v2[6]) + v2[3]*links[30].inertia[1][3] - v2[1]*links[30].inertia[3][3]) + gravity*links[30].mcm[3]*SG20[1][3] - gravity*links[30].mcm[1]*SG20[3][3];
pv2[6]=-uex[30].t[3] + (links[30].mcm[3]*v2[2] + links[30].m*v2[4])*v2[5] + v2[4]*(links[30].mcm[3]*v2[1] - links[30].m*v2[5]) + (-(links[30].mcm[1]*v2[1]) - links[30].mcm[2]*v2[2])*v2[6] + v2[1]*(-(links[30].mcm[3]*v2[4]) - v2[2]*links[30].inertia[1][1] + v2[1]*links[30].inertia[1][2]) + v2[2]*(-(links[30].mcm[3]*v2[5]) - v2[2]*links[30].inertia[1][2] + v2[1]*links[30].inertia[2][2]) + v2[3]*(links[30].mcm[1]*v2[4] + links[30].mcm[2]*v2[5] - v2[2]*links[30].inertia[1][3] + v2[1]*links[30].inertia[2][3]) - gravity*links[30].mcm[2]*SG20[1][3] + gravity*links[30].mcm[1]*SG20[2][3];

pv3[1]=-uex[31].f[1] - links[31].mcm[1]*Power(v3[2],2) - links[31].mcm[1]*Power(v3[3],2) + v3[1]*(links[31].mcm[2]*v3[2] + links[31].mcm[3]*v3[3]) - links[31].m*v3[3]*v3[5] + links[31].m*v3[2]*v3[6] + gravity*links[31].m*SG30[1][3];
pv3[2]=-uex[31].f[2] - links[31].mcm[2]*Power(v3[1],2) - links[31].mcm[2]*Power(v3[3],2) + v3[2]*(links[31].mcm[1]*v3[1] + links[31].mcm[3]*v3[3]) + links[31].m*v3[3]*v3[4] - links[31].m*v3[1]*v3[6] + gravity*links[31].m*SG30[2][3];
pv3[3]=-uex[31].f[3] - links[31].mcm[3]*Power(v3[1],2) - links[31].mcm[3]*Power(v3[2],2) + (links[31].mcm[1]*v3[1] + links[31].mcm[2]*v3[2])*v3[3] - links[31].m*v3[2]*v3[4] + links[31].m*v3[1]*v3[5] + gravity*links[31].m*SG30[3][3];
pv3[4]=-uex[31].t[1] + (-(links[31].mcm[2]*v3[2]) - links[31].mcm[3]*v3[3])*v3[4] + (links[31].mcm[1]*v3[3] + links[31].m*v3[5])*v3[6] + v3[5]*(links[31].mcm[1]*v3[2] - links[31].m*v3[6]) + v3[1]*(links[31].mcm[2]*v3[5] + links[31].mcm[3]*v3[6] - v3[3]*links[31].inertia[1][2] + v3[2]*links[31].inertia[1][3]) + v3[2]*(-(links[31].mcm[1]*v3[5]) - v3[3]*links[31].inertia[2][2] + v3[2]*links[31].inertia[2][3]) + v3[3]*(-(links[31].mcm[1]*v3[6]) - v3[3]*links[31].inertia[2][3] + v3[2]*links[31].inertia[3][3]) - gravity*links[31].mcm[3]*SG30[2][3] + gravity*links[31].mcm[2]*SG30[3][3];
pv3[5]=-uex[31].t[2] + (-(links[31].mcm[1]*v3[1]) - links[31].mcm[3]*v3[3])*v3[5] + (links[31].mcm[2]*v3[3] - links[31].m*v3[4])*v3[6] + v3[4]*(links[31].mcm[2]*v3[1] + links[31].m*v3[6]) + v3[1]*(-(links[31].mcm[2]*v3[4]) + v3[3]*links[31].inertia[1][1] - v3[1]*links[31].inertia[1][3]) + v3[2]*(links[31].mcm[1]*v3[4] + links[31].mcm[3]*v3[6] + v3[3]*links[31].inertia[1][2] - v3[1]*links[31].inertia[2][3]) + v3[3]*(-(links[31].mcm[2]*v3[6]) + v3[3]*links[31].inertia[1][3] - v3[1]*links[31].inertia[3][3]) + gravity*links[31].mcm[3]*SG30[1][3] - gravity*links[31].mcm[1]*SG30[3][3];
pv3[6]=-uex[31].t[3] + (links[31].mcm[3]*v3[2] + links[31].m*v3[4])*v3[5] + v3[4]*(links[31].mcm[3]*v3[1] - links[31].m*v3[5]) + (-(links[31].mcm[1]*v3[1]) - links[31].mcm[2]*v3[2])*v3[6] + v3[1]*(-(links[31].mcm[3]*v3[4]) - v3[2]*links[31].inertia[1][1] + v3[1]*links[31].inertia[1][2]) + v3[2]*(-(links[31].mcm[3]*v3[5]) - v3[2]*links[31].inertia[1][2] + v3[1]*links[31].inertia[2][2]) + v3[3]*(links[31].mcm[1]*v3[4] + links[31].mcm[2]*v3[5] - v3[2]*links[31].inertia[1][3] + v3[1]*links[31].inertia[2][3]) - gravity*links[31].mcm[2]*SG30[1][3] + gravity*links[31].mcm[1]*SG30[2][3];

pv4[1]=-(links[1].mcm[1]*Power(v4[2],2)) - links[1].mcm[1]*Power(v4[3],2) + v4[1]*(links[1].mcm[2]*v4[2] + links[1].mcm[3]*v4[3]) - links[1].m*v4[3]*v4[5] + links[1].m*v4[2]*v4[6] + gravity*links[1].m*SG40[1][3];
pv4[2]=-(links[1].mcm[2]*Power(v4[1],2)) - links[1].mcm[2]*Power(v4[3],2) + v4[2]*(links[1].mcm[1]*v4[1] + links[1].mcm[3]*v4[3]) + links[1].m*v4[3]*v4[4] - links[1].m*v4[1]*v4[6] + gravity*links[1].m*SG40[2][3];
pv4[3]=-(links[1].mcm[3]*Power(v4[1],2)) - links[1].mcm[3]*Power(v4[2],2) + (links[1].mcm[1]*v4[1] + links[1].mcm[2]*v4[2])*v4[3] - links[1].m*v4[2]*v4[4] + links[1].m*v4[1]*v4[5] + gravity*links[1].m*SG40[3][3];
pv4[4]=(-(links[1].mcm[2]*v4[2]) - links[1].mcm[3]*v4[3])*v4[4] + (links[1].mcm[1]*v4[3] + links[1].m*v4[5])*v4[6] + v4[5]*(links[1].mcm[1]*v4[2] - links[1].m*v4[6]) + v4[1]*(links[1].mcm[2]*v4[5] + links[1].mcm[3]*v4[6] - v4[3]*links[1].inertia[1][2] + v4[2]*links[1].inertia[1][3]) + v4[2]*(-(links[1].mcm[1]*v4[5]) - v4[3]*links[1].inertia[2][2] + v4[2]*links[1].inertia[2][3]) + v4[3]*(-(links[1].mcm[1]*v4[6]) - v4[3]*links[1].inertia[2][3] + v4[2]*links[1].inertia[3][3]) - gravity*links[1].mcm[3]*SG40[2][3] + gravity*links[1].mcm[2]*SG40[3][3];
pv4[5]=(-(links[1].mcm[1]*v4[1]) - links[1].mcm[3]*v4[3])*v4[5] + (links[1].mcm[2]*v4[3] - links[1].m*v4[4])*v4[6] + v4[4]*(links[1].mcm[2]*v4[1] + links[1].m*v4[6]) + v4[1]*(-(links[1].mcm[2]*v4[4]) + v4[3]*links[1].inertia[1][1] - v4[1]*links[1].inertia[1][3]) + v4[2]*(links[1].mcm[1]*v4[4] + links[1].mcm[3]*v4[6] + v4[3]*links[1].inertia[1][2] - v4[1]*links[1].inertia[2][3]) + v4[3]*(-(links[1].mcm[2]*v4[6]) + v4[3]*links[1].inertia[1][3] - v4[1]*links[1].inertia[3][3]) + gravity*links[1].mcm[3]*SG40[1][3] - gravity*links[1].mcm[1]*SG40[3][3];
pv4[6]=(links[1].mcm[3]*v4[2] + links[1].m*v4[4])*v4[5] + v4[4]*(links[1].mcm[3]*v4[1] - links[1].m*v4[5]) + (-(links[1].mcm[1]*v4[1]) - links[1].mcm[2]*v4[2])*v4[6] + v4[1]*(-(links[1].mcm[3]*v4[4]) - v4[2]*links[1].inertia[1][1] + v4[1]*links[1].inertia[1][2]) + v4[2]*(-(links[1].mcm[3]*v4[5]) - v4[2]*links[1].inertia[1][2] + v4[1]*links[1].inertia[2][2]) + v4[3]*(links[1].mcm[1]*v4[4] + links[1].mcm[2]*v4[5] - v4[2]*links[1].inertia[1][3] + v4[1]*links[1].inertia[2][3]) - gravity*links[1].mcm[2]*SG40[1][3] + gravity*links[1].mcm[1]*SG40[2][3];

pv5[1]=-uex[2].f[1] - links[2].mcm[1]*Power(v5[2],2) - links[2].mcm[1]*Power(v5[3],2) + v5[1]*(links[2].mcm[2]*v5[2] + links[2].mcm[3]*v5[3]) - links[2].m*v5[3]*v5[5] + links[2].m*v5[2]*v5[6] + gravity*links[2].m*SG50[1][3];
pv5[2]=-uex[2].f[2] - links[2].mcm[2]*Power(v5[1],2) - links[2].mcm[2]*Power(v5[3],2) + v5[2]*(links[2].mcm[1]*v5[1] + links[2].mcm[3]*v5[3]) + links[2].m*v5[3]*v5[4] - links[2].m*v5[1]*v5[6] + gravity*links[2].m*SG50[2][3];
pv5[3]=-uex[2].f[3] - links[2].mcm[3]*Power(v5[1],2) - links[2].mcm[3]*Power(v5[2],2) + (links[2].mcm[1]*v5[1] + links[2].mcm[2]*v5[2])*v5[3] - links[2].m*v5[2]*v5[4] + links[2].m*v5[1]*v5[5] + gravity*links[2].m*SG50[3][3];
pv5[4]=-uex[2].t[1] + (-(links[2].mcm[2]*v5[2]) - links[2].mcm[3]*v5[3])*v5[4] + (links[2].mcm[1]*v5[3] + links[2].m*v5[5])*v5[6] + v5[5]*(links[2].mcm[1]*v5[2] - links[2].m*v5[6]) + v5[1]*(links[2].mcm[2]*v5[5] + links[2].mcm[3]*v5[6] - v5[3]*links[2].inertia[1][2] + v5[2]*links[2].inertia[1][3]) + v5[2]*(-(links[2].mcm[1]*v5[5]) - v5[3]*links[2].inertia[2][2] + v5[2]*links[2].inertia[2][3]) + v5[3]*(-(links[2].mcm[1]*v5[6]) - v5[3]*links[2].inertia[2][3] + v5[2]*links[2].inertia[3][3]) - gravity*links[2].mcm[3]*SG50[2][3] + gravity*links[2].mcm[2]*SG50[3][3];
pv5[5]=-uex[2].t[2] + (-(links[2].mcm[1]*v5[1]) - links[2].mcm[3]*v5[3])*v5[5] + (links[2].mcm[2]*v5[3] - links[2].m*v5[4])*v5[6] + v5[4]*(links[2].mcm[2]*v5[1] + links[2].m*v5[6]) + v5[1]*(-(links[2].mcm[2]*v5[4]) + v5[3]*links[2].inertia[1][1] - v5[1]*links[2].inertia[1][3]) + v5[2]*(links[2].mcm[1]*v5[4] + links[2].mcm[3]*v5[6] + v5[3]*links[2].inertia[1][2] - v5[1]*links[2].inertia[2][3]) + v5[3]*(-(links[2].mcm[2]*v5[6]) + v5[3]*links[2].inertia[1][3] - v5[1]*links[2].inertia[3][3]) + gravity*links[2].mcm[3]*SG50[1][3] - gravity*links[2].mcm[1]*SG50[3][3];
pv5[6]=-uex[2].t[3] + (links[2].mcm[3]*v5[2] + links[2].m*v5[4])*v5[5] + v5[4]*(links[2].mcm[3]*v5[1] - links[2].m*v5[5]) + (-(links[2].mcm[1]*v5[1]) - links[2].mcm[2]*v5[2])*v5[6] + v5[1]*(-(links[2].mcm[3]*v5[4]) - v5[2]*links[2].inertia[1][1] + v5[1]*links[2].inertia[1][2]) + v5[2]*(-(links[2].mcm[3]*v5[5]) - v5[2]*links[2].inertia[1][2] + v5[1]*links[2].inertia[2][2]) + v5[3]*(links[2].mcm[1]*v5[4] + links[2].mcm[2]*v5[5] - v5[2]*links[2].inertia[1][3] + v5[1]*links[2].inertia[2][3]) - gravity*links[2].mcm[2]*SG50[1][3] + gravity*links[2].mcm[1]*SG50[2][3];

pv6[1]=-uex[3].f[1] - links[3].mcm[1]*Power(v6[2],2) - links[3].mcm[1]*Power(v6[3],2) + v6[1]*(links[3].mcm[2]*v6[2] + links[3].mcm[3]*v6[3]) - links[3].m*v6[3]*v6[5] + links[3].m*v6[2]*v6[6] + gravity*links[3].m*SG60[1][3];
pv6[2]=-uex[3].f[2] - links[3].mcm[2]*Power(v6[1],2) - links[3].mcm[2]*Power(v6[3],2) + v6[2]*(links[3].mcm[1]*v6[1] + links[3].mcm[3]*v6[3]) + links[3].m*v6[3]*v6[4] - links[3].m*v6[1]*v6[6] + gravity*links[3].m*SG60[2][3];
pv6[3]=-uex[3].f[3] - links[3].mcm[3]*Power(v6[1],2) - links[3].mcm[3]*Power(v6[2],2) + (links[3].mcm[1]*v6[1] + links[3].mcm[2]*v6[2])*v6[3] - links[3].m*v6[2]*v6[4] + links[3].m*v6[1]*v6[5] + gravity*links[3].m*SG60[3][3];
pv6[4]=-uex[3].t[1] + (-(links[3].mcm[2]*v6[2]) - links[3].mcm[3]*v6[3])*v6[4] + (links[3].mcm[1]*v6[3] + links[3].m*v6[5])*v6[6] + v6[5]*(links[3].mcm[1]*v6[2] - links[3].m*v6[6]) + v6[1]*(links[3].mcm[2]*v6[5] + links[3].mcm[3]*v6[6] - v6[3]*links[3].inertia[1][2] + v6[2]*links[3].inertia[1][3]) + v6[2]*(-(links[3].mcm[1]*v6[5]) - v6[3]*links[3].inertia[2][2] + v6[2]*links[3].inertia[2][3]) + v6[3]*(-(links[3].mcm[1]*v6[6]) - v6[3]*links[3].inertia[2][3] + v6[2]*links[3].inertia[3][3]) - gravity*links[3].mcm[3]*SG60[2][3] + gravity*links[3].mcm[2]*SG60[3][3];
pv6[5]=-uex[3].t[2] + (-(links[3].mcm[1]*v6[1]) - links[3].mcm[3]*v6[3])*v6[5] + (links[3].mcm[2]*v6[3] - links[3].m*v6[4])*v6[6] + v6[4]*(links[3].mcm[2]*v6[1] + links[3].m*v6[6]) + v6[1]*(-(links[3].mcm[2]*v6[4]) + v6[3]*links[3].inertia[1][1] - v6[1]*links[3].inertia[1][3]) + v6[2]*(links[3].mcm[1]*v6[4] + links[3].mcm[3]*v6[6] + v6[3]*links[3].inertia[1][2] - v6[1]*links[3].inertia[2][3]) + v6[3]*(-(links[3].mcm[2]*v6[6]) + v6[3]*links[3].inertia[1][3] - v6[1]*links[3].inertia[3][3]) + gravity*links[3].mcm[3]*SG60[1][3] - gravity*links[3].mcm[1]*SG60[3][3];
pv6[6]=-uex[3].t[3] + (links[3].mcm[3]*v6[2] + links[3].m*v6[4])*v6[5] + v6[4]*(links[3].mcm[3]*v6[1] - links[3].m*v6[5]) + (-(links[3].mcm[1]*v6[1]) - links[3].mcm[2]*v6[2])*v6[6] + v6[1]*(-(links[3].mcm[3]*v6[4]) - v6[2]*links[3].inertia[1][1] + v6[1]*links[3].inertia[1][2]) + v6[2]*(-(links[3].mcm[3]*v6[5]) - v6[2]*links[3].inertia[1][2] + v6[1]*links[3].inertia[2][2]) + v6[3]*(links[3].mcm[1]*v6[4] + links[3].mcm[2]*v6[5] - v6[2]*links[3].inertia[1][3] + v6[1]*links[3].inertia[2][3]) - gravity*links[3].mcm[2]*SG60[1][3] + gravity*links[3].mcm[1]*SG60[2][3];

pv7[1]=-uex[4].f[1] - links[4].mcm[1]*Power(v7[2],2) - links[4].mcm[1]*Power(v7[3],2) + v7[1]*(links[4].mcm[2]*v7[2] + links[4].mcm[3]*v7[3]) - links[4].m*v7[3]*v7[5] + links[4].m*v7[2]*v7[6] + gravity*links[4].m*SG70[1][3];
pv7[2]=-uex[4].f[2] - links[4].mcm[2]*Power(v7[1],2) - links[4].mcm[2]*Power(v7[3],2) + v7[2]*(links[4].mcm[1]*v7[1] + links[4].mcm[3]*v7[3]) + links[4].m*v7[3]*v7[4] - links[4].m*v7[1]*v7[6] + gravity*links[4].m*SG70[2][3];
pv7[3]=-uex[4].f[3] - links[4].mcm[3]*Power(v7[1],2) - links[4].mcm[3]*Power(v7[2],2) + (links[4].mcm[1]*v7[1] + links[4].mcm[2]*v7[2])*v7[3] - links[4].m*v7[2]*v7[4] + links[4].m*v7[1]*v7[5] + gravity*links[4].m*SG70[3][3];
pv7[4]=-uex[4].t[1] + (-(links[4].mcm[2]*v7[2]) - links[4].mcm[3]*v7[3])*v7[4] + (links[4].mcm[1]*v7[3] + links[4].m*v7[5])*v7[6] + v7[5]*(links[4].mcm[1]*v7[2] - links[4].m*v7[6]) + v7[1]*(links[4].mcm[2]*v7[5] + links[4].mcm[3]*v7[6] - v7[3]*links[4].inertia[1][2] + v7[2]*links[4].inertia[1][3]) + v7[2]*(-(links[4].mcm[1]*v7[5]) - v7[3]*links[4].inertia[2][2] + v7[2]*links[4].inertia[2][3]) + v7[3]*(-(links[4].mcm[1]*v7[6]) - v7[3]*links[4].inertia[2][3] + v7[2]*links[4].inertia[3][3]) - gravity*links[4].mcm[3]*SG70[2][3] + gravity*links[4].mcm[2]*SG70[3][3];
pv7[5]=-uex[4].t[2] + (-(links[4].mcm[1]*v7[1]) - links[4].mcm[3]*v7[3])*v7[5] + (links[4].mcm[2]*v7[3] - links[4].m*v7[4])*v7[6] + v7[4]*(links[4].mcm[2]*v7[1] + links[4].m*v7[6]) + v7[1]*(-(links[4].mcm[2]*v7[4]) + v7[3]*links[4].inertia[1][1] - v7[1]*links[4].inertia[1][3]) + v7[2]*(links[4].mcm[1]*v7[4] + links[4].mcm[3]*v7[6] + v7[3]*links[4].inertia[1][2] - v7[1]*links[4].inertia[2][3]) + v7[3]*(-(links[4].mcm[2]*v7[6]) + v7[3]*links[4].inertia[1][3] - v7[1]*links[4].inertia[3][3]) + gravity*links[4].mcm[3]*SG70[1][3] - gravity*links[4].mcm[1]*SG70[3][3];
pv7[6]=-uex[4].t[3] + (links[4].mcm[3]*v7[2] + links[4].m*v7[4])*v7[5] + v7[4]*(links[4].mcm[3]*v7[1] - links[4].m*v7[5]) + (-(links[4].mcm[1]*v7[1]) - links[4].mcm[2]*v7[2])*v7[6] + v7[1]*(-(links[4].mcm[3]*v7[4]) - v7[2]*links[4].inertia[1][1] + v7[1]*links[4].inertia[1][2]) + v7[2]*(-(links[4].mcm[3]*v7[5]) - v7[2]*links[4].inertia[1][2] + v7[1]*links[4].inertia[2][2]) + v7[3]*(links[4].mcm[1]*v7[4] + links[4].mcm[2]*v7[5] - v7[2]*links[4].inertia[1][3] + v7[1]*links[4].inertia[2][3]) - gravity*links[4].mcm[2]*SG70[1][3] + gravity*links[4].mcm[1]*SG70[2][3];

pv8[1]=-uex[5].f[1] - links[5].mcm[1]*Power(v8[2],2) - links[5].mcm[1]*Power(v8[3],2) + v8[1]*(links[5].mcm[2]*v8[2] + links[5].mcm[3]*v8[3]) - links[5].m*v8[3]*v8[5] + links[5].m*v8[2]*v8[6] + gravity*links[5].m*SG80[1][3];
pv8[2]=-uex[5].f[2] - links[5].mcm[2]*Power(v8[1],2) - links[5].mcm[2]*Power(v8[3],2) + v8[2]*(links[5].mcm[1]*v8[1] + links[5].mcm[3]*v8[3]) + links[5].m*v8[3]*v8[4] - links[5].m*v8[1]*v8[6] + gravity*links[5].m*SG80[2][3];
pv8[3]=-uex[5].f[3] - links[5].mcm[3]*Power(v8[1],2) - links[5].mcm[3]*Power(v8[2],2) + (links[5].mcm[1]*v8[1] + links[5].mcm[2]*v8[2])*v8[3] - links[5].m*v8[2]*v8[4] + links[5].m*v8[1]*v8[5] + gravity*links[5].m*SG80[3][3];
pv8[4]=-uex[5].t[1] + (-(links[5].mcm[2]*v8[2]) - links[5].mcm[3]*v8[3])*v8[4] + (links[5].mcm[1]*v8[3] + links[5].m*v8[5])*v8[6] + v8[5]*(links[5].mcm[1]*v8[2] - links[5].m*v8[6]) + v8[1]*(links[5].mcm[2]*v8[5] + links[5].mcm[3]*v8[6] - v8[3]*links[5].inertia[1][2] + v8[2]*links[5].inertia[1][3]) + v8[2]*(-(links[5].mcm[1]*v8[5]) - v8[3]*links[5].inertia[2][2] + v8[2]*links[5].inertia[2][3]) + v8[3]*(-(links[5].mcm[1]*v8[6]) - v8[3]*links[5].inertia[2][3] + v8[2]*links[5].inertia[3][3]) - gravity*links[5].mcm[3]*SG80[2][3] + gravity*links[5].mcm[2]*SG80[3][3];
pv8[5]=-uex[5].t[2] + (-(links[5].mcm[1]*v8[1]) - links[5].mcm[3]*v8[3])*v8[5] + (links[5].mcm[2]*v8[3] - links[5].m*v8[4])*v8[6] + v8[4]*(links[5].mcm[2]*v8[1] + links[5].m*v8[6]) + v8[1]*(-(links[5].mcm[2]*v8[4]) + v8[3]*links[5].inertia[1][1] - v8[1]*links[5].inertia[1][3]) + v8[2]*(links[5].mcm[1]*v8[4] + links[5].mcm[3]*v8[6] + v8[3]*links[5].inertia[1][2] - v8[1]*links[5].inertia[2][3]) + v8[3]*(-(links[5].mcm[2]*v8[6]) + v8[3]*links[5].inertia[1][3] - v8[1]*links[5].inertia[3][3]) + gravity*links[5].mcm[3]*SG80[1][3] - gravity*links[5].mcm[1]*SG80[3][3];
pv8[6]=-uex[5].t[3] + (links[5].mcm[3]*v8[2] + links[5].m*v8[4])*v8[5] + v8[4]*(links[5].mcm[3]*v8[1] - links[5].m*v8[5]) + (-(links[5].mcm[1]*v8[1]) - links[5].mcm[2]*v8[2])*v8[6] + v8[1]*(-(links[5].mcm[3]*v8[4]) - v8[2]*links[5].inertia[1][1] + v8[1]*links[5].inertia[1][2]) + v8[2]*(-(links[5].mcm[3]*v8[5]) - v8[2]*links[5].inertia[1][2] + v8[1]*links[5].inertia[2][2]) + v8[3]*(links[5].mcm[1]*v8[4] + links[5].mcm[2]*v8[5] - v8[2]*links[5].inertia[1][3] + v8[1]*links[5].inertia[2][3]) - gravity*links[5].mcm[2]*SG80[1][3] + gravity*links[5].mcm[1]*SG80[2][3];

pv9[1]=-uex[6].f[1] - links[6].mcm[1]*Power(v9[2],2) - links[6].mcm[1]*Power(v9[3],2) + v9[1]*(links[6].mcm[2]*v9[2] + links[6].mcm[3]*v9[3]) - links[6].m*v9[3]*v9[5] + links[6].m*v9[2]*v9[6] + gravity*links[6].m*SG90[1][3];
pv9[2]=-uex[6].f[2] - links[6].mcm[2]*Power(v9[1],2) - links[6].mcm[2]*Power(v9[3],2) + v9[2]*(links[6].mcm[1]*v9[1] + links[6].mcm[3]*v9[3]) + links[6].m*v9[3]*v9[4] - links[6].m*v9[1]*v9[6] + gravity*links[6].m*SG90[2][3];
pv9[3]=-uex[6].f[3] - links[6].mcm[3]*Power(v9[1],2) - links[6].mcm[3]*Power(v9[2],2) + (links[6].mcm[1]*v9[1] + links[6].mcm[2]*v9[2])*v9[3] - links[6].m*v9[2]*v9[4] + links[6].m*v9[1]*v9[5] + gravity*links[6].m*SG90[3][3];
pv9[4]=-uex[6].t[1] + (-(links[6].mcm[2]*v9[2]) - links[6].mcm[3]*v9[3])*v9[4] + (links[6].mcm[1]*v9[3] + links[6].m*v9[5])*v9[6] + v9[5]*(links[6].mcm[1]*v9[2] - links[6].m*v9[6]) + v9[1]*(links[6].mcm[2]*v9[5] + links[6].mcm[3]*v9[6] - v9[3]*links[6].inertia[1][2] + v9[2]*links[6].inertia[1][3]) + v9[2]*(-(links[6].mcm[1]*v9[5]) - v9[3]*links[6].inertia[2][2] + v9[2]*links[6].inertia[2][3]) + v9[3]*(-(links[6].mcm[1]*v9[6]) - v9[3]*links[6].inertia[2][3] + v9[2]*links[6].inertia[3][3]) - gravity*links[6].mcm[3]*SG90[2][3] + gravity*links[6].mcm[2]*SG90[3][3];
pv9[5]=-uex[6].t[2] + (-(links[6].mcm[1]*v9[1]) - links[6].mcm[3]*v9[3])*v9[5] + (links[6].mcm[2]*v9[3] - links[6].m*v9[4])*v9[6] + v9[4]*(links[6].mcm[2]*v9[1] + links[6].m*v9[6]) + v9[1]*(-(links[6].mcm[2]*v9[4]) + v9[3]*links[6].inertia[1][1] - v9[1]*links[6].inertia[1][3]) + v9[2]*(links[6].mcm[1]*v9[4] + links[6].mcm[3]*v9[6] + v9[3]*links[6].inertia[1][2] - v9[1]*links[6].inertia[2][3]) + v9[3]*(-(links[6].mcm[2]*v9[6]) + v9[3]*links[6].inertia[1][3] - v9[1]*links[6].inertia[3][3]) + gravity*links[6].mcm[3]*SG90[1][3] - gravity*links[6].mcm[1]*SG90[3][3];
pv9[6]=-uex[6].t[3] + (links[6].mcm[3]*v9[2] + links[6].m*v9[4])*v9[5] + v9[4]*(links[6].mcm[3]*v9[1] - links[6].m*v9[5]) + (-(links[6].mcm[1]*v9[1]) - links[6].mcm[2]*v9[2])*v9[6] + v9[1]*(-(links[6].mcm[3]*v9[4]) - v9[2]*links[6].inertia[1][1] + v9[1]*links[6].inertia[1][2]) + v9[2]*(-(links[6].mcm[3]*v9[5]) - v9[2]*links[6].inertia[1][2] + v9[1]*links[6].inertia[2][2]) + v9[3]*(links[6].mcm[1]*v9[4] + links[6].mcm[2]*v9[5] - v9[2]*links[6].inertia[1][3] + v9[1]*links[6].inertia[2][3]) - gravity*links[6].mcm[2]*SG90[1][3] + gravity*links[6].mcm[1]*SG90[2][3];

pv10[1]=-uex[7].f[1] - links[7].mcm[1]*Power(v10[2],2) - links[7].mcm[1]*Power(v10[3],2) + v10[1]*(links[7].mcm[2]*v10[2] + links[7].mcm[3]*v10[3]) - links[7].m*v10[3]*v10[5] + links[7].m*v10[2]*v10[6] + gravity*links[7].m*SG100[1][3];
pv10[2]=-uex[7].f[2] - links[7].mcm[2]*Power(v10[1],2) - links[7].mcm[2]*Power(v10[3],2) + v10[2]*(links[7].mcm[1]*v10[1] + links[7].mcm[3]*v10[3]) + links[7].m*v10[3]*v10[4] - links[7].m*v10[1]*v10[6] + gravity*links[7].m*SG100[2][3];
pv10[3]=-uex[7].f[3] - links[7].mcm[3]*Power(v10[1],2) - links[7].mcm[3]*Power(v10[2],2) + (links[7].mcm[1]*v10[1] + links[7].mcm[2]*v10[2])*v10[3] - links[7].m*v10[2]*v10[4] + links[7].m*v10[1]*v10[5] + gravity*links[7].m*SG100[3][3];
pv10[4]=-uex[7].t[1] + (-(links[7].mcm[2]*v10[2]) - links[7].mcm[3]*v10[3])*v10[4] + (links[7].mcm[1]*v10[3] + links[7].m*v10[5])*v10[6] + v10[5]*(links[7].mcm[1]*v10[2] - links[7].m*v10[6]) + v10[1]*(links[7].mcm[2]*v10[5] + links[7].mcm[3]*v10[6] - v10[3]*links[7].inertia[1][2] + v10[2]*links[7].inertia[1][3]) + v10[2]*(-(links[7].mcm[1]*v10[5]) - v10[3]*links[7].inertia[2][2] + v10[2]*links[7].inertia[2][3]) + v10[3]*(-(links[7].mcm[1]*v10[6]) - v10[3]*links[7].inertia[2][3] + v10[2]*links[7].inertia[3][3]) - gravity*links[7].mcm[3]*SG100[2][3] + gravity*links[7].mcm[2]*SG100[3][3];
pv10[5]=-uex[7].t[2] + (-(links[7].mcm[1]*v10[1]) - links[7].mcm[3]*v10[3])*v10[5] + (links[7].mcm[2]*v10[3] - links[7].m*v10[4])*v10[6] + v10[4]*(links[7].mcm[2]*v10[1] + links[7].m*v10[6]) + v10[1]*(-(links[7].mcm[2]*v10[4]) + v10[3]*links[7].inertia[1][1] - v10[1]*links[7].inertia[1][3]) + v10[2]*(links[7].mcm[1]*v10[4] + links[7].mcm[3]*v10[6] + v10[3]*links[7].inertia[1][2] - v10[1]*links[7].inertia[2][3]) + v10[3]*(-(links[7].mcm[2]*v10[6]) + v10[3]*links[7].inertia[1][3] - v10[1]*links[7].inertia[3][3]) + gravity*links[7].mcm[3]*SG100[1][3] - gravity*links[7].mcm[1]*SG100[3][3];
pv10[6]=-uex[7].t[3] + (links[7].mcm[3]*v10[2] + links[7].m*v10[4])*v10[5] + v10[4]*(links[7].mcm[3]*v10[1] - links[7].m*v10[5]) + (-(links[7].mcm[1]*v10[1]) - links[7].mcm[2]*v10[2])*v10[6] + v10[1]*(-(links[7].mcm[3]*v10[4]) - v10[2]*links[7].inertia[1][1] + v10[1]*links[7].inertia[1][2]) + v10[2]*(-(links[7].mcm[3]*v10[5]) - v10[2]*links[7].inertia[1][2] + v10[1]*links[7].inertia[2][2]) + v10[3]*(links[7].mcm[1]*v10[4] + links[7].mcm[2]*v10[5] - v10[2]*links[7].inertia[1][3] + v10[1]*links[7].inertia[2][3]) - gravity*links[7].mcm[2]*SG100[1][3] + gravity*links[7].mcm[1]*SG100[2][3];

pv11[1]=-(eff[2].mcm[1]*Power(v11[2],2)) - eff[2].mcm[1]*Power(v11[3],2) + v11[1]*(eff[2].mcm[2]*v11[2] + eff[2].mcm[3]*v11[3]) - eff[2].m*v11[3]*v11[5] + eff[2].m*v11[2]*v11[6] + eff[2].m*gravity*SG110[1][3];
pv11[2]=-(eff[2].mcm[2]*Power(v11[1],2)) - eff[2].mcm[2]*Power(v11[3],2) + v11[2]*(eff[2].mcm[1]*v11[1] + eff[2].mcm[3]*v11[3]) + eff[2].m*v11[3]*v11[4] - eff[2].m*v11[1]*v11[6] + eff[2].m*gravity*SG110[2][3];
pv11[3]=-(eff[2].mcm[3]*Power(v11[1],2)) - eff[2].mcm[3]*Power(v11[2],2) + (eff[2].mcm[1]*v11[1] + eff[2].mcm[2]*v11[2])*v11[3] - eff[2].m*v11[2]*v11[4] + eff[2].m*v11[1]*v11[5] + eff[2].m*gravity*SG110[3][3];
pv11[4]=(-(eff[2].mcm[2]*v11[2]) - eff[2].mcm[3]*v11[3])*v11[4] - eff[2].mcm[1]*v11[2]*v11[5] - eff[2].mcm[1]*v11[3]*v11[6] + (eff[2].mcm[1]*v11[3] + eff[2].m*v11[5])*v11[6] + v11[5]*(eff[2].mcm[1]*v11[2] - eff[2].m*v11[6]) + v11[1]*(eff[2].mcm[2]*v11[5] + eff[2].mcm[3]*v11[6]) - gravity*eff[2].mcm[3]*SG110[2][3] + gravity*eff[2].mcm[2]*SG110[3][3];
pv11[5]=-(eff[2].mcm[2]*v11[1]*v11[4]) + (-(eff[2].mcm[1]*v11[1]) - eff[2].mcm[3]*v11[3])*v11[5] - eff[2].mcm[2]*v11[3]*v11[6] + (eff[2].mcm[2]*v11[3] - eff[2].m*v11[4])*v11[6] + v11[4]*(eff[2].mcm[2]*v11[1] + eff[2].m*v11[6]) + v11[2]*(eff[2].mcm[1]*v11[4] + eff[2].mcm[3]*v11[6]) + gravity*eff[2].mcm[3]*SG110[1][3] - gravity*eff[2].mcm[1]*SG110[3][3];
pv11[6]=-(eff[2].mcm[3]*v11[1]*v11[4]) - eff[2].mcm[3]*v11[2]*v11[5] + (eff[2].mcm[3]*v11[2] + eff[2].m*v11[4])*v11[5] + v11[4]*(eff[2].mcm[3]*v11[1] - eff[2].m*v11[5]) + v11[3]*(eff[2].mcm[1]*v11[4] + eff[2].mcm[2]*v11[5]) + (-(eff[2].mcm[1]*v11[1]) - eff[2].mcm[2]*v11[2])*v11[6] - gravity*eff[2].mcm[2]*SG110[1][3] + gravity*eff[2].mcm[1]*SG110[2][3];

pv12[1]=-uex[39].f[1] - links[39].mcm[1]*Power(v12[2],2) - links[39].mcm[1]*Power(v12[3],2) + v12[1]*(links[39].mcm[2]*v12[2] + links[39].mcm[3]*v12[3]) - links[39].m*v12[3]*v12[5] + links[39].m*v12[2]*v12[6] + gravity*links[39].m*SG120[1][3];
pv12[2]=-uex[39].f[2] - links[39].mcm[2]*Power(v12[1],2) - links[39].mcm[2]*Power(v12[3],2) + v12[2]*(links[39].mcm[1]*v12[1] + links[39].mcm[3]*v12[3]) + links[39].m*v12[3]*v12[4] - links[39].m*v12[1]*v12[6] + gravity*links[39].m*SG120[2][3];
pv12[3]=-uex[39].f[3] - links[39].mcm[3]*Power(v12[1],2) - links[39].mcm[3]*Power(v12[2],2) + (links[39].mcm[1]*v12[1] + links[39].mcm[2]*v12[2])*v12[3] - links[39].m*v12[2]*v12[4] + links[39].m*v12[1]*v12[5] + gravity*links[39].m*SG120[3][3];
pv12[4]=-uex[39].t[1] + (-(links[39].mcm[2]*v12[2]) - links[39].mcm[3]*v12[3])*v12[4] + (links[39].mcm[1]*v12[3] + links[39].m*v12[5])*v12[6] + v12[5]*(links[39].mcm[1]*v12[2] - links[39].m*v12[6]) + v12[1]*(links[39].mcm[2]*v12[5] + links[39].mcm[3]*v12[6] - v12[3]*links[39].inertia[1][2] + v12[2]*links[39].inertia[1][3]) + v12[2]*(-(links[39].mcm[1]*v12[5]) - v12[3]*links[39].inertia[2][2] + v12[2]*links[39].inertia[2][3]) + v12[3]*(-(links[39].mcm[1]*v12[6]) - v12[3]*links[39].inertia[2][3] + v12[2]*links[39].inertia[3][3]) - gravity*links[39].mcm[3]*SG120[2][3] + gravity*links[39].mcm[2]*SG120[3][3];
pv12[5]=-uex[39].t[2] + (-(links[39].mcm[1]*v12[1]) - links[39].mcm[3]*v12[3])*v12[5] + (links[39].mcm[2]*v12[3] - links[39].m*v12[4])*v12[6] + v12[4]*(links[39].mcm[2]*v12[1] + links[39].m*v12[6]) + v12[1]*(-(links[39].mcm[2]*v12[4]) + v12[3]*links[39].inertia[1][1] - v12[1]*links[39].inertia[1][3]) + v12[2]*(links[39].mcm[1]*v12[4] + links[39].mcm[3]*v12[6] + v12[3]*links[39].inertia[1][2] - v12[1]*links[39].inertia[2][3]) + v12[3]*(-(links[39].mcm[2]*v12[6]) + v12[3]*links[39].inertia[1][3] - v12[1]*links[39].inertia[3][3]) + gravity*links[39].mcm[3]*SG120[1][3] - gravity*links[39].mcm[1]*SG120[3][3];
pv12[6]=-uex[39].t[3] + (links[39].mcm[3]*v12[2] + links[39].m*v12[4])*v12[5] + v12[4]*(links[39].mcm[3]*v12[1] - links[39].m*v12[5]) + (-(links[39].mcm[1]*v12[1]) - links[39].mcm[2]*v12[2])*v12[6] + v12[1]*(-(links[39].mcm[3]*v12[4]) - v12[2]*links[39].inertia[1][1] + v12[1]*links[39].inertia[1][2]) + v12[2]*(-(links[39].mcm[3]*v12[5]) - v12[2]*links[39].inertia[1][2] + v12[1]*links[39].inertia[2][2]) + v12[3]*(links[39].mcm[1]*v12[4] + links[39].mcm[2]*v12[5] - v12[2]*links[39].inertia[1][3] + v12[1]*links[39].inertia[2][3]) - gravity*links[39].mcm[2]*SG120[1][3] + gravity*links[39].mcm[1]*SG120[2][3];

pv13[1]=-uex[40].f[1] - links[40].mcm[1]*Power(v13[2],2) - links[40].mcm[1]*Power(v13[3],2) + v13[1]*(links[40].mcm[2]*v13[2] + links[40].mcm[3]*v13[3]) - links[40].m*v13[3]*v13[5] + links[40].m*v13[2]*v13[6] + gravity*links[40].m*SG130[1][3];
pv13[2]=-uex[40].f[2] - links[40].mcm[2]*Power(v13[1],2) - links[40].mcm[2]*Power(v13[3],2) + v13[2]*(links[40].mcm[1]*v13[1] + links[40].mcm[3]*v13[3]) + links[40].m*v13[3]*v13[4] - links[40].m*v13[1]*v13[6] + gravity*links[40].m*SG130[2][3];
pv13[3]=-uex[40].f[3] - links[40].mcm[3]*Power(v13[1],2) - links[40].mcm[3]*Power(v13[2],2) + (links[40].mcm[1]*v13[1] + links[40].mcm[2]*v13[2])*v13[3] - links[40].m*v13[2]*v13[4] + links[40].m*v13[1]*v13[5] + gravity*links[40].m*SG130[3][3];
pv13[4]=-uex[40].t[1] + (-(links[40].mcm[2]*v13[2]) - links[40].mcm[3]*v13[3])*v13[4] + (links[40].mcm[1]*v13[3] + links[40].m*v13[5])*v13[6] + v13[5]*(links[40].mcm[1]*v13[2] - links[40].m*v13[6]) + v13[1]*(links[40].mcm[2]*v13[5] + links[40].mcm[3]*v13[6] - v13[3]*links[40].inertia[1][2] + v13[2]*links[40].inertia[1][3]) + v13[2]*(-(links[40].mcm[1]*v13[5]) - v13[3]*links[40].inertia[2][2] + v13[2]*links[40].inertia[2][3]) + v13[3]*(-(links[40].mcm[1]*v13[6]) - v13[3]*links[40].inertia[2][3] + v13[2]*links[40].inertia[3][3]) - gravity*links[40].mcm[3]*SG130[2][3] + gravity*links[40].mcm[2]*SG130[3][3];
pv13[5]=-uex[40].t[2] + (-(links[40].mcm[1]*v13[1]) - links[40].mcm[3]*v13[3])*v13[5] + (links[40].mcm[2]*v13[3] - links[40].m*v13[4])*v13[6] + v13[4]*(links[40].mcm[2]*v13[1] + links[40].m*v13[6]) + v13[1]*(-(links[40].mcm[2]*v13[4]) + v13[3]*links[40].inertia[1][1] - v13[1]*links[40].inertia[1][3]) + v13[2]*(links[40].mcm[1]*v13[4] + links[40].mcm[3]*v13[6] + v13[3]*links[40].inertia[1][2] - v13[1]*links[40].inertia[2][3]) + v13[3]*(-(links[40].mcm[2]*v13[6]) + v13[3]*links[40].inertia[1][3] - v13[1]*links[40].inertia[3][3]) + gravity*links[40].mcm[3]*SG130[1][3] - gravity*links[40].mcm[1]*SG130[3][3];
pv13[6]=-uex[40].t[3] + (links[40].mcm[3]*v13[2] + links[40].m*v13[4])*v13[5] + v13[4]*(links[40].mcm[3]*v13[1] - links[40].m*v13[5]) + (-(links[40].mcm[1]*v13[1]) - links[40].mcm[2]*v13[2])*v13[6] + v13[1]*(-(links[40].mcm[3]*v13[4]) - v13[2]*links[40].inertia[1][1] + v13[1]*links[40].inertia[1][2]) + v13[2]*(-(links[40].mcm[3]*v13[5]) - v13[2]*links[40].inertia[1][2] + v13[1]*links[40].inertia[2][2]) + v13[3]*(links[40].mcm[1]*v13[4] + links[40].mcm[2]*v13[5] - v13[2]*links[40].inertia[1][3] + v13[1]*links[40].inertia[2][3]) - gravity*links[40].mcm[2]*SG130[1][3] + gravity*links[40].mcm[1]*SG130[2][3];

pv16[1]=-uex[41].f[1] - links[41].mcm[1]*Power(v16[2],2) - links[41].mcm[1]*Power(v16[3],2) + v16[1]*(links[41].mcm[2]*v16[2] + links[41].mcm[3]*v16[3]) - links[41].m*v16[3]*v16[5] + links[41].m*v16[2]*v16[6] + gravity*links[41].m*SG160[1][3];
pv16[2]=-uex[41].f[2] - links[41].mcm[2]*Power(v16[1],2) - links[41].mcm[2]*Power(v16[3],2) + v16[2]*(links[41].mcm[1]*v16[1] + links[41].mcm[3]*v16[3]) + links[41].m*v16[3]*v16[4] - links[41].m*v16[1]*v16[6] + gravity*links[41].m*SG160[2][3];
pv16[3]=-uex[41].f[3] - links[41].mcm[3]*Power(v16[1],2) - links[41].mcm[3]*Power(v16[2],2) + (links[41].mcm[1]*v16[1] + links[41].mcm[2]*v16[2])*v16[3] - links[41].m*v16[2]*v16[4] + links[41].m*v16[1]*v16[5] + gravity*links[41].m*SG160[3][3];
pv16[4]=-uex[41].t[1] + (-(links[41].mcm[2]*v16[2]) - links[41].mcm[3]*v16[3])*v16[4] + (links[41].mcm[1]*v16[3] + links[41].m*v16[5])*v16[6] + v16[5]*(links[41].mcm[1]*v16[2] - links[41].m*v16[6]) + v16[1]*(links[41].mcm[2]*v16[5] + links[41].mcm[3]*v16[6] - v16[3]*links[41].inertia[1][2] + v16[2]*links[41].inertia[1][3]) + v16[2]*(-(links[41].mcm[1]*v16[5]) - v16[3]*links[41].inertia[2][2] + v16[2]*links[41].inertia[2][3]) + v16[3]*(-(links[41].mcm[1]*v16[6]) - v16[3]*links[41].inertia[2][3] + v16[2]*links[41].inertia[3][3]) - gravity*links[41].mcm[3]*SG160[2][3] + gravity*links[41].mcm[2]*SG160[3][3];
pv16[5]=-uex[41].t[2] + (-(links[41].mcm[1]*v16[1]) - links[41].mcm[3]*v16[3])*v16[5] + (links[41].mcm[2]*v16[3] - links[41].m*v16[4])*v16[6] + v16[4]*(links[41].mcm[2]*v16[1] + links[41].m*v16[6]) + v16[1]*(-(links[41].mcm[2]*v16[4]) + v16[3]*links[41].inertia[1][1] - v16[1]*links[41].inertia[1][3]) + v16[2]*(links[41].mcm[1]*v16[4] + links[41].mcm[3]*v16[6] + v16[3]*links[41].inertia[1][2] - v16[1]*links[41].inertia[2][3]) + v16[3]*(-(links[41].mcm[2]*v16[6]) + v16[3]*links[41].inertia[1][3] - v16[1]*links[41].inertia[3][3]) + gravity*links[41].mcm[3]*SG160[1][3] - gravity*links[41].mcm[1]*SG160[3][3];
pv16[6]=-uex[41].t[3] + (links[41].mcm[3]*v16[2] + links[41].m*v16[4])*v16[5] + v16[4]*(links[41].mcm[3]*v16[1] - links[41].m*v16[5]) + (-(links[41].mcm[1]*v16[1]) - links[41].mcm[2]*v16[2])*v16[6] + v16[1]*(-(links[41].mcm[3]*v16[4]) - v16[2]*links[41].inertia[1][1] + v16[1]*links[41].inertia[1][2]) + v16[2]*(-(links[41].mcm[3]*v16[5]) - v16[2]*links[41].inertia[1][2] + v16[1]*links[41].inertia[2][2]) + v16[3]*(links[41].mcm[1]*v16[4] + links[41].mcm[2]*v16[5] - v16[2]*links[41].inertia[1][3] + v16[1]*links[41].inertia[2][3]) - gravity*links[41].mcm[2]*SG160[1][3] + gravity*links[41].mcm[1]*SG160[2][3];

pv20[1]=-uex[42].f[1] - links[42].mcm[1]*Power(v20[2],2) - links[42].mcm[1]*Power(v20[3],2) + v20[1]*(links[42].mcm[2]*v20[2] + links[42].mcm[3]*v20[3]) - links[42].m*v20[3]*v20[5] + links[42].m*v20[2]*v20[6] + gravity*links[42].m*SG200[1][3];
pv20[2]=-uex[42].f[2] - links[42].mcm[2]*Power(v20[1],2) - links[42].mcm[2]*Power(v20[3],2) + v20[2]*(links[42].mcm[1]*v20[1] + links[42].mcm[3]*v20[3]) + links[42].m*v20[3]*v20[4] - links[42].m*v20[1]*v20[6] + gravity*links[42].m*SG200[2][3];
pv20[3]=-uex[42].f[3] - links[42].mcm[3]*Power(v20[1],2) - links[42].mcm[3]*Power(v20[2],2) + (links[42].mcm[1]*v20[1] + links[42].mcm[2]*v20[2])*v20[3] - links[42].m*v20[2]*v20[4] + links[42].m*v20[1]*v20[5] + gravity*links[42].m*SG200[3][3];
pv20[4]=-uex[42].t[1] + (-(links[42].mcm[2]*v20[2]) - links[42].mcm[3]*v20[3])*v20[4] + (links[42].mcm[1]*v20[3] + links[42].m*v20[5])*v20[6] + v20[5]*(links[42].mcm[1]*v20[2] - links[42].m*v20[6]) + v20[1]*(links[42].mcm[2]*v20[5] + links[42].mcm[3]*v20[6] - v20[3]*links[42].inertia[1][2] + v20[2]*links[42].inertia[1][3]) + v20[2]*(-(links[42].mcm[1]*v20[5]) - v20[3]*links[42].inertia[2][2] + v20[2]*links[42].inertia[2][3]) + v20[3]*(-(links[42].mcm[1]*v20[6]) - v20[3]*links[42].inertia[2][3] + v20[2]*links[42].inertia[3][3]) - gravity*links[42].mcm[3]*SG200[2][3] + gravity*links[42].mcm[2]*SG200[3][3];
pv20[5]=-uex[42].t[2] + (-(links[42].mcm[1]*v20[1]) - links[42].mcm[3]*v20[3])*v20[5] + (links[42].mcm[2]*v20[3] - links[42].m*v20[4])*v20[6] + v20[4]*(links[42].mcm[2]*v20[1] + links[42].m*v20[6]) + v20[1]*(-(links[42].mcm[2]*v20[4]) + v20[3]*links[42].inertia[1][1] - v20[1]*links[42].inertia[1][3]) + v20[2]*(links[42].mcm[1]*v20[4] + links[42].mcm[3]*v20[6] + v20[3]*links[42].inertia[1][2] - v20[1]*links[42].inertia[2][3]) + v20[3]*(-(links[42].mcm[2]*v20[6]) + v20[3]*links[42].inertia[1][3] - v20[1]*links[42].inertia[3][3]) + gravity*links[42].mcm[3]*SG200[1][3] - gravity*links[42].mcm[1]*SG200[3][3];
pv20[6]=-uex[42].t[3] + (links[42].mcm[3]*v20[2] + links[42].m*v20[4])*v20[5] + v20[4]*(links[42].mcm[3]*v20[1] - links[42].m*v20[5]) + (-(links[42].mcm[1]*v20[1]) - links[42].mcm[2]*v20[2])*v20[6] + v20[1]*(-(links[42].mcm[3]*v20[4]) - v20[2]*links[42].inertia[1][1] + v20[1]*links[42].inertia[1][2]) + v20[2]*(-(links[42].mcm[3]*v20[5]) - v20[2]*links[42].inertia[1][2] + v20[1]*links[42].inertia[2][2]) + v20[3]*(links[42].mcm[1]*v20[4] + links[42].mcm[2]*v20[5] - v20[2]*links[42].inertia[1][3] + v20[1]*links[42].inertia[2][3]) - gravity*links[42].mcm[2]*SG200[1][3] + gravity*links[42].mcm[1]*SG200[2][3];

pv24[1]=-uex[43].f[1] - links[43].mcm[1]*Power(v24[2],2) - links[43].mcm[1]*Power(v24[3],2) + v24[1]*(links[43].mcm[2]*v24[2] + links[43].mcm[3]*v24[3]) - links[43].m*v24[3]*v24[5] + links[43].m*v24[2]*v24[6] + gravity*links[43].m*SG240[1][3];
pv24[2]=-uex[43].f[2] - links[43].mcm[2]*Power(v24[1],2) - links[43].mcm[2]*Power(v24[3],2) + v24[2]*(links[43].mcm[1]*v24[1] + links[43].mcm[3]*v24[3]) + links[43].m*v24[3]*v24[4] - links[43].m*v24[1]*v24[6] + gravity*links[43].m*SG240[2][3];
pv24[3]=-uex[43].f[3] - links[43].mcm[3]*Power(v24[1],2) - links[43].mcm[3]*Power(v24[2],2) + (links[43].mcm[1]*v24[1] + links[43].mcm[2]*v24[2])*v24[3] - links[43].m*v24[2]*v24[4] + links[43].m*v24[1]*v24[5] + gravity*links[43].m*SG240[3][3];
pv24[4]=-uex[43].t[1] + (-(links[43].mcm[2]*v24[2]) - links[43].mcm[3]*v24[3])*v24[4] + (links[43].mcm[1]*v24[3] + links[43].m*v24[5])*v24[6] + v24[5]*(links[43].mcm[1]*v24[2] - links[43].m*v24[6]) + v24[1]*(links[43].mcm[2]*v24[5] + links[43].mcm[3]*v24[6] - v24[3]*links[43].inertia[1][2] + v24[2]*links[43].inertia[1][3]) + v24[2]*(-(links[43].mcm[1]*v24[5]) - v24[3]*links[43].inertia[2][2] + v24[2]*links[43].inertia[2][3]) + v24[3]*(-(links[43].mcm[1]*v24[6]) - v24[3]*links[43].inertia[2][3] + v24[2]*links[43].inertia[3][3]) - gravity*links[43].mcm[3]*SG240[2][3] + gravity*links[43].mcm[2]*SG240[3][3];
pv24[5]=-uex[43].t[2] + (-(links[43].mcm[1]*v24[1]) - links[43].mcm[3]*v24[3])*v24[5] + (links[43].mcm[2]*v24[3] - links[43].m*v24[4])*v24[6] + v24[4]*(links[43].mcm[2]*v24[1] + links[43].m*v24[6]) + v24[1]*(-(links[43].mcm[2]*v24[4]) + v24[3]*links[43].inertia[1][1] - v24[1]*links[43].inertia[1][3]) + v24[2]*(links[43].mcm[1]*v24[4] + links[43].mcm[3]*v24[6] + v24[3]*links[43].inertia[1][2] - v24[1]*links[43].inertia[2][3]) + v24[3]*(-(links[43].mcm[2]*v24[6]) + v24[3]*links[43].inertia[1][3] - v24[1]*links[43].inertia[3][3]) + gravity*links[43].mcm[3]*SG240[1][3] - gravity*links[43].mcm[1]*SG240[3][3];
pv24[6]=-uex[43].t[3] + (links[43].mcm[3]*v24[2] + links[43].m*v24[4])*v24[5] + v24[4]*(links[43].mcm[3]*v24[1] - links[43].m*v24[5]) + (-(links[43].mcm[1]*v24[1]) - links[43].mcm[2]*v24[2])*v24[6] + v24[1]*(-(links[43].mcm[3]*v24[4]) - v24[2]*links[43].inertia[1][1] + v24[1]*links[43].inertia[1][2]) + v24[2]*(-(links[43].mcm[3]*v24[5]) - v24[2]*links[43].inertia[1][2] + v24[1]*links[43].inertia[2][2]) + v24[3]*(links[43].mcm[1]*v24[4] + links[43].mcm[2]*v24[5] - v24[2]*links[43].inertia[1][3] + v24[1]*links[43].inertia[2][3]) - gravity*links[43].mcm[2]*SG240[1][3] + gravity*links[43].mcm[1]*SG240[2][3];

pv28[1]=-uex[44].f[1] - links[44].mcm[1]*Power(v28[2],2) - links[44].mcm[1]*Power(v28[3],2) + v28[1]*(links[44].mcm[2]*v28[2] + links[44].mcm[3]*v28[3]) - links[44].m*v28[3]*v28[5] + links[44].m*v28[2]*v28[6] + gravity*links[44].m*SG280[1][3];
pv28[2]=-uex[44].f[2] - links[44].mcm[2]*Power(v28[1],2) - links[44].mcm[2]*Power(v28[3],2) + v28[2]*(links[44].mcm[1]*v28[1] + links[44].mcm[3]*v28[3]) + links[44].m*v28[3]*v28[4] - links[44].m*v28[1]*v28[6] + gravity*links[44].m*SG280[2][3];
pv28[3]=-uex[44].f[3] - links[44].mcm[3]*Power(v28[1],2) - links[44].mcm[3]*Power(v28[2],2) + (links[44].mcm[1]*v28[1] + links[44].mcm[2]*v28[2])*v28[3] - links[44].m*v28[2]*v28[4] + links[44].m*v28[1]*v28[5] + gravity*links[44].m*SG280[3][3];
pv28[4]=-uex[44].t[1] + (-(links[44].mcm[2]*v28[2]) - links[44].mcm[3]*v28[3])*v28[4] + (links[44].mcm[1]*v28[3] + links[44].m*v28[5])*v28[6] + v28[5]*(links[44].mcm[1]*v28[2] - links[44].m*v28[6]) + v28[1]*(links[44].mcm[2]*v28[5] + links[44].mcm[3]*v28[6] - v28[3]*links[44].inertia[1][2] + v28[2]*links[44].inertia[1][3]) + v28[2]*(-(links[44].mcm[1]*v28[5]) - v28[3]*links[44].inertia[2][2] + v28[2]*links[44].inertia[2][3]) + v28[3]*(-(links[44].mcm[1]*v28[6]) - v28[3]*links[44].inertia[2][3] + v28[2]*links[44].inertia[3][3]) - gravity*links[44].mcm[3]*SG280[2][3] + gravity*links[44].mcm[2]*SG280[3][3];
pv28[5]=-uex[44].t[2] + (-(links[44].mcm[1]*v28[1]) - links[44].mcm[3]*v28[3])*v28[5] + (links[44].mcm[2]*v28[3] - links[44].m*v28[4])*v28[6] + v28[4]*(links[44].mcm[2]*v28[1] + links[44].m*v28[6]) + v28[1]*(-(links[44].mcm[2]*v28[4]) + v28[3]*links[44].inertia[1][1] - v28[1]*links[44].inertia[1][3]) + v28[2]*(links[44].mcm[1]*v28[4] + links[44].mcm[3]*v28[6] + v28[3]*links[44].inertia[1][2] - v28[1]*links[44].inertia[2][3]) + v28[3]*(-(links[44].mcm[2]*v28[6]) + v28[3]*links[44].inertia[1][3] - v28[1]*links[44].inertia[3][3]) + gravity*links[44].mcm[3]*SG280[1][3] - gravity*links[44].mcm[1]*SG280[3][3];
pv28[6]=-uex[44].t[3] + (links[44].mcm[3]*v28[2] + links[44].m*v28[4])*v28[5] + v28[4]*(links[44].mcm[3]*v28[1] - links[44].m*v28[5]) + (-(links[44].mcm[1]*v28[1]) - links[44].mcm[2]*v28[2])*v28[6] + v28[1]*(-(links[44].mcm[3]*v28[4]) - v28[2]*links[44].inertia[1][1] + v28[1]*links[44].inertia[1][2]) + v28[2]*(-(links[44].mcm[3]*v28[5]) - v28[2]*links[44].inertia[1][2] + v28[1]*links[44].inertia[2][2]) + v28[3]*(links[44].mcm[1]*v28[4] + links[44].mcm[2]*v28[5] - v28[2]*links[44].inertia[1][3] + v28[1]*links[44].inertia[2][3]) - gravity*links[44].mcm[2]*SG280[1][3] + gravity*links[44].mcm[1]*SG280[2][3];

pv32[1]=-(links[8].mcm[1]*Power(v32[2],2)) - links[8].mcm[1]*Power(v32[3],2) + v32[1]*(links[8].mcm[2]*v32[2] + links[8].mcm[3]*v32[3]) - links[8].m*v32[3]*v32[5] + links[8].m*v32[2]*v32[6] + gravity*links[8].m*SG320[1][3];
pv32[2]=-(links[8].mcm[2]*Power(v32[1],2)) - links[8].mcm[2]*Power(v32[3],2) + v32[2]*(links[8].mcm[1]*v32[1] + links[8].mcm[3]*v32[3]) + links[8].m*v32[3]*v32[4] - links[8].m*v32[1]*v32[6] + gravity*links[8].m*SG320[2][3];
pv32[3]=-(links[8].mcm[3]*Power(v32[1],2)) - links[8].mcm[3]*Power(v32[2],2) + (links[8].mcm[1]*v32[1] + links[8].mcm[2]*v32[2])*v32[3] - links[8].m*v32[2]*v32[4] + links[8].m*v32[1]*v32[5] + gravity*links[8].m*SG320[3][3];
pv32[4]=(-(links[8].mcm[2]*v32[2]) - links[8].mcm[3]*v32[3])*v32[4] + (links[8].mcm[1]*v32[3] + links[8].m*v32[5])*v32[6] + v32[5]*(links[8].mcm[1]*v32[2] - links[8].m*v32[6]) + v32[1]*(links[8].mcm[2]*v32[5] + links[8].mcm[3]*v32[6] - v32[3]*links[8].inertia[1][2] + v32[2]*links[8].inertia[1][3]) + v32[2]*(-(links[8].mcm[1]*v32[5]) - v32[3]*links[8].inertia[2][2] + v32[2]*links[8].inertia[2][3]) + v32[3]*(-(links[8].mcm[1]*v32[6]) - v32[3]*links[8].inertia[2][3] + v32[2]*links[8].inertia[3][3]) - gravity*links[8].mcm[3]*SG320[2][3] + gravity*links[8].mcm[2]*SG320[3][3];
pv32[5]=(-(links[8].mcm[1]*v32[1]) - links[8].mcm[3]*v32[3])*v32[5] + (links[8].mcm[2]*v32[3] - links[8].m*v32[4])*v32[6] + v32[4]*(links[8].mcm[2]*v32[1] + links[8].m*v32[6]) + v32[1]*(-(links[8].mcm[2]*v32[4]) + v32[3]*links[8].inertia[1][1] - v32[1]*links[8].inertia[1][3]) + v32[2]*(links[8].mcm[1]*v32[4] + links[8].mcm[3]*v32[6] + v32[3]*links[8].inertia[1][2] - v32[1]*links[8].inertia[2][3]) + v32[3]*(-(links[8].mcm[2]*v32[6]) + v32[3]*links[8].inertia[1][3] - v32[1]*links[8].inertia[3][3]) + gravity*links[8].mcm[3]*SG320[1][3] - gravity*links[8].mcm[1]*SG320[3][3];
pv32[6]=(links[8].mcm[3]*v32[2] + links[8].m*v32[4])*v32[5] + v32[4]*(links[8].mcm[3]*v32[1] - links[8].m*v32[5]) + (-(links[8].mcm[1]*v32[1]) - links[8].mcm[2]*v32[2])*v32[6] + v32[1]*(-(links[8].mcm[3]*v32[4]) - v32[2]*links[8].inertia[1][1] + v32[1]*links[8].inertia[1][2]) + v32[2]*(-(links[8].mcm[3]*v32[5]) - v32[2]*links[8].inertia[1][2] + v32[1]*links[8].inertia[2][2]) + v32[3]*(links[8].mcm[1]*v32[4] + links[8].mcm[2]*v32[5] - v32[2]*links[8].inertia[1][3] + v32[1]*links[8].inertia[2][3]) - gravity*links[8].mcm[2]*SG320[1][3] + gravity*links[8].mcm[1]*SG320[2][3];

pv33[1]=-uex[9].f[1] - links[9].mcm[1]*Power(v33[2],2) - links[9].mcm[1]*Power(v33[3],2) + v33[1]*(links[9].mcm[2]*v33[2] + links[9].mcm[3]*v33[3]) - links[9].m*v33[3]*v33[5] + links[9].m*v33[2]*v33[6] + gravity*links[9].m*SG330[1][3];
pv33[2]=-uex[9].f[2] - links[9].mcm[2]*Power(v33[1],2) - links[9].mcm[2]*Power(v33[3],2) + v33[2]*(links[9].mcm[1]*v33[1] + links[9].mcm[3]*v33[3]) + links[9].m*v33[3]*v33[4] - links[9].m*v33[1]*v33[6] + gravity*links[9].m*SG330[2][3];
pv33[3]=-uex[9].f[3] - links[9].mcm[3]*Power(v33[1],2) - links[9].mcm[3]*Power(v33[2],2) + (links[9].mcm[1]*v33[1] + links[9].mcm[2]*v33[2])*v33[3] - links[9].m*v33[2]*v33[4] + links[9].m*v33[1]*v33[5] + gravity*links[9].m*SG330[3][3];
pv33[4]=-uex[9].t[1] + (-(links[9].mcm[2]*v33[2]) - links[9].mcm[3]*v33[3])*v33[4] + (links[9].mcm[1]*v33[3] + links[9].m*v33[5])*v33[6] + v33[5]*(links[9].mcm[1]*v33[2] - links[9].m*v33[6]) + v33[1]*(links[9].mcm[2]*v33[5] + links[9].mcm[3]*v33[6] - v33[3]*links[9].inertia[1][2] + v33[2]*links[9].inertia[1][3]) + v33[2]*(-(links[9].mcm[1]*v33[5]) - v33[3]*links[9].inertia[2][2] + v33[2]*links[9].inertia[2][3]) + v33[3]*(-(links[9].mcm[1]*v33[6]) - v33[3]*links[9].inertia[2][3] + v33[2]*links[9].inertia[3][3]) - gravity*links[9].mcm[3]*SG330[2][3] + gravity*links[9].mcm[2]*SG330[3][3];
pv33[5]=-uex[9].t[2] + (-(links[9].mcm[1]*v33[1]) - links[9].mcm[3]*v33[3])*v33[5] + (links[9].mcm[2]*v33[3] - links[9].m*v33[4])*v33[6] + v33[4]*(links[9].mcm[2]*v33[1] + links[9].m*v33[6]) + v33[1]*(-(links[9].mcm[2]*v33[4]) + v33[3]*links[9].inertia[1][1] - v33[1]*links[9].inertia[1][3]) + v33[2]*(links[9].mcm[1]*v33[4] + links[9].mcm[3]*v33[6] + v33[3]*links[9].inertia[1][2] - v33[1]*links[9].inertia[2][3]) + v33[3]*(-(links[9].mcm[2]*v33[6]) + v33[3]*links[9].inertia[1][3] - v33[1]*links[9].inertia[3][3]) + gravity*links[9].mcm[3]*SG330[1][3] - gravity*links[9].mcm[1]*SG330[3][3];
pv33[6]=-uex[9].t[3] + (links[9].mcm[3]*v33[2] + links[9].m*v33[4])*v33[5] + v33[4]*(links[9].mcm[3]*v33[1] - links[9].m*v33[5]) + (-(links[9].mcm[1]*v33[1]) - links[9].mcm[2]*v33[2])*v33[6] + v33[1]*(-(links[9].mcm[3]*v33[4]) - v33[2]*links[9].inertia[1][1] + v33[1]*links[9].inertia[1][2]) + v33[2]*(-(links[9].mcm[3]*v33[5]) - v33[2]*links[9].inertia[1][2] + v33[1]*links[9].inertia[2][2]) + v33[3]*(links[9].mcm[1]*v33[4] + links[9].mcm[2]*v33[5] - v33[2]*links[9].inertia[1][3] + v33[1]*links[9].inertia[2][3]) - gravity*links[9].mcm[2]*SG330[1][3] + gravity*links[9].mcm[1]*SG330[2][3];

pv34[1]=-uex[10].f[1] - links[10].mcm[1]*Power(v34[2],2) - links[10].mcm[1]*Power(v34[3],2) + v34[1]*(links[10].mcm[2]*v34[2] + links[10].mcm[3]*v34[3]) - links[10].m*v34[3]*v34[5] + links[10].m*v34[2]*v34[6] + gravity*links[10].m*SG340[1][3];
pv34[2]=-uex[10].f[2] - links[10].mcm[2]*Power(v34[1],2) - links[10].mcm[2]*Power(v34[3],2) + v34[2]*(links[10].mcm[1]*v34[1] + links[10].mcm[3]*v34[3]) + links[10].m*v34[3]*v34[4] - links[10].m*v34[1]*v34[6] + gravity*links[10].m*SG340[2][3];
pv34[3]=-uex[10].f[3] - links[10].mcm[3]*Power(v34[1],2) - links[10].mcm[3]*Power(v34[2],2) + (links[10].mcm[1]*v34[1] + links[10].mcm[2]*v34[2])*v34[3] - links[10].m*v34[2]*v34[4] + links[10].m*v34[1]*v34[5] + gravity*links[10].m*SG340[3][3];
pv34[4]=-uex[10].t[1] + (-(links[10].mcm[2]*v34[2]) - links[10].mcm[3]*v34[3])*v34[4] + (links[10].mcm[1]*v34[3] + links[10].m*v34[5])*v34[6] + v34[5]*(links[10].mcm[1]*v34[2] - links[10].m*v34[6]) + v34[1]*(links[10].mcm[2]*v34[5] + links[10].mcm[3]*v34[6] - v34[3]*links[10].inertia[1][2] + v34[2]*links[10].inertia[1][3]) + v34[2]*(-(links[10].mcm[1]*v34[5]) - v34[3]*links[10].inertia[2][2] + v34[2]*links[10].inertia[2][3]) + v34[3]*(-(links[10].mcm[1]*v34[6]) - v34[3]*links[10].inertia[2][3] + v34[2]*links[10].inertia[3][3]) - gravity*links[10].mcm[3]*SG340[2][3] + gravity*links[10].mcm[2]*SG340[3][3];
pv34[5]=-uex[10].t[2] + (-(links[10].mcm[1]*v34[1]) - links[10].mcm[3]*v34[3])*v34[5] + (links[10].mcm[2]*v34[3] - links[10].m*v34[4])*v34[6] + v34[4]*(links[10].mcm[2]*v34[1] + links[10].m*v34[6]) + v34[1]*(-(links[10].mcm[2]*v34[4]) + v34[3]*links[10].inertia[1][1] - v34[1]*links[10].inertia[1][3]) + v34[2]*(links[10].mcm[1]*v34[4] + links[10].mcm[3]*v34[6] + v34[3]*links[10].inertia[1][2] - v34[1]*links[10].inertia[2][3]) + v34[3]*(-(links[10].mcm[2]*v34[6]) + v34[3]*links[10].inertia[1][3] - v34[1]*links[10].inertia[3][3]) + gravity*links[10].mcm[3]*SG340[1][3] - gravity*links[10].mcm[1]*SG340[3][3];
pv34[6]=-uex[10].t[3] + (links[10].mcm[3]*v34[2] + links[10].m*v34[4])*v34[5] + v34[4]*(links[10].mcm[3]*v34[1] - links[10].m*v34[5]) + (-(links[10].mcm[1]*v34[1]) - links[10].mcm[2]*v34[2])*v34[6] + v34[1]*(-(links[10].mcm[3]*v34[4]) - v34[2]*links[10].inertia[1][1] + v34[1]*links[10].inertia[1][2]) + v34[2]*(-(links[10].mcm[3]*v34[5]) - v34[2]*links[10].inertia[1][2] + v34[1]*links[10].inertia[2][2]) + v34[3]*(links[10].mcm[1]*v34[4] + links[10].mcm[2]*v34[5] - v34[2]*links[10].inertia[1][3] + v34[1]*links[10].inertia[2][3]) - gravity*links[10].mcm[2]*SG340[1][3] + gravity*links[10].mcm[1]*SG340[2][3];

pv35[1]=-uex[11].f[1] - links[11].mcm[1]*Power(v35[2],2) - links[11].mcm[1]*Power(v35[3],2) + v35[1]*(links[11].mcm[2]*v35[2] + links[11].mcm[3]*v35[3]) - links[11].m*v35[3]*v35[5] + links[11].m*v35[2]*v35[6] + gravity*links[11].m*SG350[1][3];
pv35[2]=-uex[11].f[2] - links[11].mcm[2]*Power(v35[1],2) - links[11].mcm[2]*Power(v35[3],2) + v35[2]*(links[11].mcm[1]*v35[1] + links[11].mcm[3]*v35[3]) + links[11].m*v35[3]*v35[4] - links[11].m*v35[1]*v35[6] + gravity*links[11].m*SG350[2][3];
pv35[3]=-uex[11].f[3] - links[11].mcm[3]*Power(v35[1],2) - links[11].mcm[3]*Power(v35[2],2) + (links[11].mcm[1]*v35[1] + links[11].mcm[2]*v35[2])*v35[3] - links[11].m*v35[2]*v35[4] + links[11].m*v35[1]*v35[5] + gravity*links[11].m*SG350[3][3];
pv35[4]=-uex[11].t[1] + (-(links[11].mcm[2]*v35[2]) - links[11].mcm[3]*v35[3])*v35[4] + (links[11].mcm[1]*v35[3] + links[11].m*v35[5])*v35[6] + v35[5]*(links[11].mcm[1]*v35[2] - links[11].m*v35[6]) + v35[1]*(links[11].mcm[2]*v35[5] + links[11].mcm[3]*v35[6] - v35[3]*links[11].inertia[1][2] + v35[2]*links[11].inertia[1][3]) + v35[2]*(-(links[11].mcm[1]*v35[5]) - v35[3]*links[11].inertia[2][2] + v35[2]*links[11].inertia[2][3]) + v35[3]*(-(links[11].mcm[1]*v35[6]) - v35[3]*links[11].inertia[2][3] + v35[2]*links[11].inertia[3][3]) - gravity*links[11].mcm[3]*SG350[2][3] + gravity*links[11].mcm[2]*SG350[3][3];
pv35[5]=-uex[11].t[2] + (-(links[11].mcm[1]*v35[1]) - links[11].mcm[3]*v35[3])*v35[5] + (links[11].mcm[2]*v35[3] - links[11].m*v35[4])*v35[6] + v35[4]*(links[11].mcm[2]*v35[1] + links[11].m*v35[6]) + v35[1]*(-(links[11].mcm[2]*v35[4]) + v35[3]*links[11].inertia[1][1] - v35[1]*links[11].inertia[1][3]) + v35[2]*(links[11].mcm[1]*v35[4] + links[11].mcm[3]*v35[6] + v35[3]*links[11].inertia[1][2] - v35[1]*links[11].inertia[2][3]) + v35[3]*(-(links[11].mcm[2]*v35[6]) + v35[3]*links[11].inertia[1][3] - v35[1]*links[11].inertia[3][3]) + gravity*links[11].mcm[3]*SG350[1][3] - gravity*links[11].mcm[1]*SG350[3][3];
pv35[6]=-uex[11].t[3] + (links[11].mcm[3]*v35[2] + links[11].m*v35[4])*v35[5] + v35[4]*(links[11].mcm[3]*v35[1] - links[11].m*v35[5]) + (-(links[11].mcm[1]*v35[1]) - links[11].mcm[2]*v35[2])*v35[6] + v35[1]*(-(links[11].mcm[3]*v35[4]) - v35[2]*links[11].inertia[1][1] + v35[1]*links[11].inertia[1][2]) + v35[2]*(-(links[11].mcm[3]*v35[5]) - v35[2]*links[11].inertia[1][2] + v35[1]*links[11].inertia[2][2]) + v35[3]*(links[11].mcm[1]*v35[4] + links[11].mcm[2]*v35[5] - v35[2]*links[11].inertia[1][3] + v35[1]*links[11].inertia[2][3]) - gravity*links[11].mcm[2]*SG350[1][3] + gravity*links[11].mcm[1]*SG350[2][3];

pv36[1]=-uex[12].f[1] - links[12].mcm[1]*Power(v36[2],2) - links[12].mcm[1]*Power(v36[3],2) + v36[1]*(links[12].mcm[2]*v36[2] + links[12].mcm[3]*v36[3]) - links[12].m*v36[3]*v36[5] + links[12].m*v36[2]*v36[6] + gravity*links[12].m*SG360[1][3];
pv36[2]=-uex[12].f[2] - links[12].mcm[2]*Power(v36[1],2) - links[12].mcm[2]*Power(v36[3],2) + v36[2]*(links[12].mcm[1]*v36[1] + links[12].mcm[3]*v36[3]) + links[12].m*v36[3]*v36[4] - links[12].m*v36[1]*v36[6] + gravity*links[12].m*SG360[2][3];
pv36[3]=-uex[12].f[3] - links[12].mcm[3]*Power(v36[1],2) - links[12].mcm[3]*Power(v36[2],2) + (links[12].mcm[1]*v36[1] + links[12].mcm[2]*v36[2])*v36[3] - links[12].m*v36[2]*v36[4] + links[12].m*v36[1]*v36[5] + gravity*links[12].m*SG360[3][3];
pv36[4]=-uex[12].t[1] + (-(links[12].mcm[2]*v36[2]) - links[12].mcm[3]*v36[3])*v36[4] + (links[12].mcm[1]*v36[3] + links[12].m*v36[5])*v36[6] + v36[5]*(links[12].mcm[1]*v36[2] - links[12].m*v36[6]) + v36[1]*(links[12].mcm[2]*v36[5] + links[12].mcm[3]*v36[6] - v36[3]*links[12].inertia[1][2] + v36[2]*links[12].inertia[1][3]) + v36[2]*(-(links[12].mcm[1]*v36[5]) - v36[3]*links[12].inertia[2][2] + v36[2]*links[12].inertia[2][3]) + v36[3]*(-(links[12].mcm[1]*v36[6]) - v36[3]*links[12].inertia[2][3] + v36[2]*links[12].inertia[3][3]) - gravity*links[12].mcm[3]*SG360[2][3] + gravity*links[12].mcm[2]*SG360[3][3];
pv36[5]=-uex[12].t[2] + (-(links[12].mcm[1]*v36[1]) - links[12].mcm[3]*v36[3])*v36[5] + (links[12].mcm[2]*v36[3] - links[12].m*v36[4])*v36[6] + v36[4]*(links[12].mcm[2]*v36[1] + links[12].m*v36[6]) + v36[1]*(-(links[12].mcm[2]*v36[4]) + v36[3]*links[12].inertia[1][1] - v36[1]*links[12].inertia[1][3]) + v36[2]*(links[12].mcm[1]*v36[4] + links[12].mcm[3]*v36[6] + v36[3]*links[12].inertia[1][2] - v36[1]*links[12].inertia[2][3]) + v36[3]*(-(links[12].mcm[2]*v36[6]) + v36[3]*links[12].inertia[1][3] - v36[1]*links[12].inertia[3][3]) + gravity*links[12].mcm[3]*SG360[1][3] - gravity*links[12].mcm[1]*SG360[3][3];
pv36[6]=-uex[12].t[3] + (links[12].mcm[3]*v36[2] + links[12].m*v36[4])*v36[5] + v36[4]*(links[12].mcm[3]*v36[1] - links[12].m*v36[5]) + (-(links[12].mcm[1]*v36[1]) - links[12].mcm[2]*v36[2])*v36[6] + v36[1]*(-(links[12].mcm[3]*v36[4]) - v36[2]*links[12].inertia[1][1] + v36[1]*links[12].inertia[1][2]) + v36[2]*(-(links[12].mcm[3]*v36[5]) - v36[2]*links[12].inertia[1][2] + v36[1]*links[12].inertia[2][2]) + v36[3]*(links[12].mcm[1]*v36[4] + links[12].mcm[2]*v36[5] - v36[2]*links[12].inertia[1][3] + v36[1]*links[12].inertia[2][3]) - gravity*links[12].mcm[2]*SG360[1][3] + gravity*links[12].mcm[1]*SG360[2][3];

pv37[1]=-uex[13].f[1] - links[13].mcm[1]*Power(v37[2],2) - links[13].mcm[1]*Power(v37[3],2) + v37[1]*(links[13].mcm[2]*v37[2] + links[13].mcm[3]*v37[3]) - links[13].m*v37[3]*v37[5] + links[13].m*v37[2]*v37[6] + gravity*links[13].m*SG370[1][3];
pv37[2]=-uex[13].f[2] - links[13].mcm[2]*Power(v37[1],2) - links[13].mcm[2]*Power(v37[3],2) + v37[2]*(links[13].mcm[1]*v37[1] + links[13].mcm[3]*v37[3]) + links[13].m*v37[3]*v37[4] - links[13].m*v37[1]*v37[6] + gravity*links[13].m*SG370[2][3];
pv37[3]=-uex[13].f[3] - links[13].mcm[3]*Power(v37[1],2) - links[13].mcm[3]*Power(v37[2],2) + (links[13].mcm[1]*v37[1] + links[13].mcm[2]*v37[2])*v37[3] - links[13].m*v37[2]*v37[4] + links[13].m*v37[1]*v37[5] + gravity*links[13].m*SG370[3][3];
pv37[4]=-uex[13].t[1] + (-(links[13].mcm[2]*v37[2]) - links[13].mcm[3]*v37[3])*v37[4] + (links[13].mcm[1]*v37[3] + links[13].m*v37[5])*v37[6] + v37[5]*(links[13].mcm[1]*v37[2] - links[13].m*v37[6]) + v37[1]*(links[13].mcm[2]*v37[5] + links[13].mcm[3]*v37[6] - v37[3]*links[13].inertia[1][2] + v37[2]*links[13].inertia[1][3]) + v37[2]*(-(links[13].mcm[1]*v37[5]) - v37[3]*links[13].inertia[2][2] + v37[2]*links[13].inertia[2][3]) + v37[3]*(-(links[13].mcm[1]*v37[6]) - v37[3]*links[13].inertia[2][3] + v37[2]*links[13].inertia[3][3]) - gravity*links[13].mcm[3]*SG370[2][3] + gravity*links[13].mcm[2]*SG370[3][3];
pv37[5]=-uex[13].t[2] + (-(links[13].mcm[1]*v37[1]) - links[13].mcm[3]*v37[3])*v37[5] + (links[13].mcm[2]*v37[3] - links[13].m*v37[4])*v37[6] + v37[4]*(links[13].mcm[2]*v37[1] + links[13].m*v37[6]) + v37[1]*(-(links[13].mcm[2]*v37[4]) + v37[3]*links[13].inertia[1][1] - v37[1]*links[13].inertia[1][3]) + v37[2]*(links[13].mcm[1]*v37[4] + links[13].mcm[3]*v37[6] + v37[3]*links[13].inertia[1][2] - v37[1]*links[13].inertia[2][3]) + v37[3]*(-(links[13].mcm[2]*v37[6]) + v37[3]*links[13].inertia[1][3] - v37[1]*links[13].inertia[3][3]) + gravity*links[13].mcm[3]*SG370[1][3] - gravity*links[13].mcm[1]*SG370[3][3];
pv37[6]=-uex[13].t[3] + (links[13].mcm[3]*v37[2] + links[13].m*v37[4])*v37[5] + v37[4]*(links[13].mcm[3]*v37[1] - links[13].m*v37[5]) + (-(links[13].mcm[1]*v37[1]) - links[13].mcm[2]*v37[2])*v37[6] + v37[1]*(-(links[13].mcm[3]*v37[4]) - v37[2]*links[13].inertia[1][1] + v37[1]*links[13].inertia[1][2]) + v37[2]*(-(links[13].mcm[3]*v37[5]) - v37[2]*links[13].inertia[1][2] + v37[1]*links[13].inertia[2][2]) + v37[3]*(links[13].mcm[1]*v37[4] + links[13].mcm[2]*v37[5] - v37[2]*links[13].inertia[1][3] + v37[1]*links[13].inertia[2][3]) - gravity*links[13].mcm[2]*SG370[1][3] + gravity*links[13].mcm[1]*SG370[2][3];

pv38[1]=-uex[14].f[1] - links[14].mcm[1]*Power(v38[2],2) - links[14].mcm[1]*Power(v38[3],2) + v38[1]*(links[14].mcm[2]*v38[2] + links[14].mcm[3]*v38[3]) - links[14].m*v38[3]*v38[5] + links[14].m*v38[2]*v38[6] + gravity*links[14].m*SG380[1][3];
pv38[2]=-uex[14].f[2] - links[14].mcm[2]*Power(v38[1],2) - links[14].mcm[2]*Power(v38[3],2) + v38[2]*(links[14].mcm[1]*v38[1] + links[14].mcm[3]*v38[3]) + links[14].m*v38[3]*v38[4] - links[14].m*v38[1]*v38[6] + gravity*links[14].m*SG380[2][3];
pv38[3]=-uex[14].f[3] - links[14].mcm[3]*Power(v38[1],2) - links[14].mcm[3]*Power(v38[2],2) + (links[14].mcm[1]*v38[1] + links[14].mcm[2]*v38[2])*v38[3] - links[14].m*v38[2]*v38[4] + links[14].m*v38[1]*v38[5] + gravity*links[14].m*SG380[3][3];
pv38[4]=-uex[14].t[1] + (-(links[14].mcm[2]*v38[2]) - links[14].mcm[3]*v38[3])*v38[4] + (links[14].mcm[1]*v38[3] + links[14].m*v38[5])*v38[6] + v38[5]*(links[14].mcm[1]*v38[2] - links[14].m*v38[6]) + v38[1]*(links[14].mcm[2]*v38[5] + links[14].mcm[3]*v38[6] - v38[3]*links[14].inertia[1][2] + v38[2]*links[14].inertia[1][3]) + v38[2]*(-(links[14].mcm[1]*v38[5]) - v38[3]*links[14].inertia[2][2] + v38[2]*links[14].inertia[2][3]) + v38[3]*(-(links[14].mcm[1]*v38[6]) - v38[3]*links[14].inertia[2][3] + v38[2]*links[14].inertia[3][3]) - gravity*links[14].mcm[3]*SG380[2][3] + gravity*links[14].mcm[2]*SG380[3][3];
pv38[5]=-uex[14].t[2] + (-(links[14].mcm[1]*v38[1]) - links[14].mcm[3]*v38[3])*v38[5] + (links[14].mcm[2]*v38[3] - links[14].m*v38[4])*v38[6] + v38[4]*(links[14].mcm[2]*v38[1] + links[14].m*v38[6]) + v38[1]*(-(links[14].mcm[2]*v38[4]) + v38[3]*links[14].inertia[1][1] - v38[1]*links[14].inertia[1][3]) + v38[2]*(links[14].mcm[1]*v38[4] + links[14].mcm[3]*v38[6] + v38[3]*links[14].inertia[1][2] - v38[1]*links[14].inertia[2][3]) + v38[3]*(-(links[14].mcm[2]*v38[6]) + v38[3]*links[14].inertia[1][3] - v38[1]*links[14].inertia[3][3]) + gravity*links[14].mcm[3]*SG380[1][3] - gravity*links[14].mcm[1]*SG380[3][3];
pv38[6]=-uex[14].t[3] + (links[14].mcm[3]*v38[2] + links[14].m*v38[4])*v38[5] + v38[4]*(links[14].mcm[3]*v38[1] - links[14].m*v38[5]) + (-(links[14].mcm[1]*v38[1]) - links[14].mcm[2]*v38[2])*v38[6] + v38[1]*(-(links[14].mcm[3]*v38[4]) - v38[2]*links[14].inertia[1][1] + v38[1]*links[14].inertia[1][2]) + v38[2]*(-(links[14].mcm[3]*v38[5]) - v38[2]*links[14].inertia[1][2] + v38[1]*links[14].inertia[2][2]) + v38[3]*(links[14].mcm[1]*v38[4] + links[14].mcm[2]*v38[5] - v38[2]*links[14].inertia[1][3] + v38[1]*links[14].inertia[2][3]) - gravity*links[14].mcm[2]*SG380[1][3] + gravity*links[14].mcm[1]*SG380[2][3];

pv39[1]=-(eff[1].mcm[1]*Power(v39[2],2)) - eff[1].mcm[1]*Power(v39[3],2) + v39[1]*(eff[1].mcm[2]*v39[2] + eff[1].mcm[3]*v39[3]) - eff[1].m*v39[3]*v39[5] + eff[1].m*v39[2]*v39[6] + eff[1].m*gravity*SG390[1][3];
pv39[2]=-(eff[1].mcm[2]*Power(v39[1],2)) - eff[1].mcm[2]*Power(v39[3],2) + v39[2]*(eff[1].mcm[1]*v39[1] + eff[1].mcm[3]*v39[3]) + eff[1].m*v39[3]*v39[4] - eff[1].m*v39[1]*v39[6] + eff[1].m*gravity*SG390[2][3];
pv39[3]=-(eff[1].mcm[3]*Power(v39[1],2)) - eff[1].mcm[3]*Power(v39[2],2) + (eff[1].mcm[1]*v39[1] + eff[1].mcm[2]*v39[2])*v39[3] - eff[1].m*v39[2]*v39[4] + eff[1].m*v39[1]*v39[5] + eff[1].m*gravity*SG390[3][3];
pv39[4]=(-(eff[1].mcm[2]*v39[2]) - eff[1].mcm[3]*v39[3])*v39[4] - eff[1].mcm[1]*v39[2]*v39[5] - eff[1].mcm[1]*v39[3]*v39[6] + (eff[1].mcm[1]*v39[3] + eff[1].m*v39[5])*v39[6] + v39[5]*(eff[1].mcm[1]*v39[2] - eff[1].m*v39[6]) + v39[1]*(eff[1].mcm[2]*v39[5] + eff[1].mcm[3]*v39[6]) - gravity*eff[1].mcm[3]*SG390[2][3] + gravity*eff[1].mcm[2]*SG390[3][3];
pv39[5]=-(eff[1].mcm[2]*v39[1]*v39[4]) + (-(eff[1].mcm[1]*v39[1]) - eff[1].mcm[3]*v39[3])*v39[5] - eff[1].mcm[2]*v39[3]*v39[6] + (eff[1].mcm[2]*v39[3] - eff[1].m*v39[4])*v39[6] + v39[4]*(eff[1].mcm[2]*v39[1] + eff[1].m*v39[6]) + v39[2]*(eff[1].mcm[1]*v39[4] + eff[1].mcm[3]*v39[6]) + gravity*eff[1].mcm[3]*SG390[1][3] - gravity*eff[1].mcm[1]*SG390[3][3];
pv39[6]=-(eff[1].mcm[3]*v39[1]*v39[4]) - eff[1].mcm[3]*v39[2]*v39[5] + (eff[1].mcm[3]*v39[2] + eff[1].m*v39[4])*v39[5] + v39[4]*(eff[1].mcm[3]*v39[1] - eff[1].m*v39[5]) + v39[3]*(eff[1].mcm[1]*v39[4] + eff[1].mcm[2]*v39[5]) + (-(eff[1].mcm[1]*v39[1]) - eff[1].mcm[2]*v39[2])*v39[6] - gravity*eff[1].mcm[2]*SG390[1][3] + gravity*eff[1].mcm[1]*SG390[2][3];

pv40[1]=-uex[45].f[1] - links[45].mcm[1]*Power(v40[2],2) - links[45].mcm[1]*Power(v40[3],2) + v40[1]*(links[45].mcm[2]*v40[2] + links[45].mcm[3]*v40[3]) - links[45].m*v40[3]*v40[5] + links[45].m*v40[2]*v40[6] + gravity*links[45].m*SG400[1][3];
pv40[2]=-uex[45].f[2] - links[45].mcm[2]*Power(v40[1],2) - links[45].mcm[2]*Power(v40[3],2) + v40[2]*(links[45].mcm[1]*v40[1] + links[45].mcm[3]*v40[3]) + links[45].m*v40[3]*v40[4] - links[45].m*v40[1]*v40[6] + gravity*links[45].m*SG400[2][3];
pv40[3]=-uex[45].f[3] - links[45].mcm[3]*Power(v40[1],2) - links[45].mcm[3]*Power(v40[2],2) + (links[45].mcm[1]*v40[1] + links[45].mcm[2]*v40[2])*v40[3] - links[45].m*v40[2]*v40[4] + links[45].m*v40[1]*v40[5] + gravity*links[45].m*SG400[3][3];
pv40[4]=-uex[45].t[1] + (-(links[45].mcm[2]*v40[2]) - links[45].mcm[3]*v40[3])*v40[4] + (links[45].mcm[1]*v40[3] + links[45].m*v40[5])*v40[6] + v40[5]*(links[45].mcm[1]*v40[2] - links[45].m*v40[6]) + v40[1]*(links[45].mcm[2]*v40[5] + links[45].mcm[3]*v40[6] - v40[3]*links[45].inertia[1][2] + v40[2]*links[45].inertia[1][3]) + v40[2]*(-(links[45].mcm[1]*v40[5]) - v40[3]*links[45].inertia[2][2] + v40[2]*links[45].inertia[2][3]) + v40[3]*(-(links[45].mcm[1]*v40[6]) - v40[3]*links[45].inertia[2][3] + v40[2]*links[45].inertia[3][3]) - gravity*links[45].mcm[3]*SG400[2][3] + gravity*links[45].mcm[2]*SG400[3][3];
pv40[5]=-uex[45].t[2] + (-(links[45].mcm[1]*v40[1]) - links[45].mcm[3]*v40[3])*v40[5] + (links[45].mcm[2]*v40[3] - links[45].m*v40[4])*v40[6] + v40[4]*(links[45].mcm[2]*v40[1] + links[45].m*v40[6]) + v40[1]*(-(links[45].mcm[2]*v40[4]) + v40[3]*links[45].inertia[1][1] - v40[1]*links[45].inertia[1][3]) + v40[2]*(links[45].mcm[1]*v40[4] + links[45].mcm[3]*v40[6] + v40[3]*links[45].inertia[1][2] - v40[1]*links[45].inertia[2][3]) + v40[3]*(-(links[45].mcm[2]*v40[6]) + v40[3]*links[45].inertia[1][3] - v40[1]*links[45].inertia[3][3]) + gravity*links[45].mcm[3]*SG400[1][3] - gravity*links[45].mcm[1]*SG400[3][3];
pv40[6]=-uex[45].t[3] + (links[45].mcm[3]*v40[2] + links[45].m*v40[4])*v40[5] + v40[4]*(links[45].mcm[3]*v40[1] - links[45].m*v40[5]) + (-(links[45].mcm[1]*v40[1]) - links[45].mcm[2]*v40[2])*v40[6] + v40[1]*(-(links[45].mcm[3]*v40[4]) - v40[2]*links[45].inertia[1][1] + v40[1]*links[45].inertia[1][2]) + v40[2]*(-(links[45].mcm[3]*v40[5]) - v40[2]*links[45].inertia[1][2] + v40[1]*links[45].inertia[2][2]) + v40[3]*(links[45].mcm[1]*v40[4] + links[45].mcm[2]*v40[5] - v40[2]*links[45].inertia[1][3] + v40[1]*links[45].inertia[2][3]) - gravity*links[45].mcm[2]*SG400[1][3] + gravity*links[45].mcm[1]*SG400[2][3];

pv41[1]=-uex[46].f[1] - links[46].mcm[1]*Power(v41[2],2) - links[46].mcm[1]*Power(v41[3],2) + v41[1]*(links[46].mcm[2]*v41[2] + links[46].mcm[3]*v41[3]) - links[46].m*v41[3]*v41[5] + links[46].m*v41[2]*v41[6] + gravity*links[46].m*SG410[1][3];
pv41[2]=-uex[46].f[2] - links[46].mcm[2]*Power(v41[1],2) - links[46].mcm[2]*Power(v41[3],2) + v41[2]*(links[46].mcm[1]*v41[1] + links[46].mcm[3]*v41[3]) + links[46].m*v41[3]*v41[4] - links[46].m*v41[1]*v41[6] + gravity*links[46].m*SG410[2][3];
pv41[3]=-uex[46].f[3] - links[46].mcm[3]*Power(v41[1],2) - links[46].mcm[3]*Power(v41[2],2) + (links[46].mcm[1]*v41[1] + links[46].mcm[2]*v41[2])*v41[3] - links[46].m*v41[2]*v41[4] + links[46].m*v41[1]*v41[5] + gravity*links[46].m*SG410[3][3];
pv41[4]=-uex[46].t[1] + (-(links[46].mcm[2]*v41[2]) - links[46].mcm[3]*v41[3])*v41[4] + (links[46].mcm[1]*v41[3] + links[46].m*v41[5])*v41[6] + v41[5]*(links[46].mcm[1]*v41[2] - links[46].m*v41[6]) + v41[1]*(links[46].mcm[2]*v41[5] + links[46].mcm[3]*v41[6] - v41[3]*links[46].inertia[1][2] + v41[2]*links[46].inertia[1][3]) + v41[2]*(-(links[46].mcm[1]*v41[5]) - v41[3]*links[46].inertia[2][2] + v41[2]*links[46].inertia[2][3]) + v41[3]*(-(links[46].mcm[1]*v41[6]) - v41[3]*links[46].inertia[2][3] + v41[2]*links[46].inertia[3][3]) - gravity*links[46].mcm[3]*SG410[2][3] + gravity*links[46].mcm[2]*SG410[3][3];
pv41[5]=-uex[46].t[2] + (-(links[46].mcm[1]*v41[1]) - links[46].mcm[3]*v41[3])*v41[5] + (links[46].mcm[2]*v41[3] - links[46].m*v41[4])*v41[6] + v41[4]*(links[46].mcm[2]*v41[1] + links[46].m*v41[6]) + v41[1]*(-(links[46].mcm[2]*v41[4]) + v41[3]*links[46].inertia[1][1] - v41[1]*links[46].inertia[1][3]) + v41[2]*(links[46].mcm[1]*v41[4] + links[46].mcm[3]*v41[6] + v41[3]*links[46].inertia[1][2] - v41[1]*links[46].inertia[2][3]) + v41[3]*(-(links[46].mcm[2]*v41[6]) + v41[3]*links[46].inertia[1][3] - v41[1]*links[46].inertia[3][3]) + gravity*links[46].mcm[3]*SG410[1][3] - gravity*links[46].mcm[1]*SG410[3][3];
pv41[6]=-uex[46].t[3] + (links[46].mcm[3]*v41[2] + links[46].m*v41[4])*v41[5] + v41[4]*(links[46].mcm[3]*v41[1] - links[46].m*v41[5]) + (-(links[46].mcm[1]*v41[1]) - links[46].mcm[2]*v41[2])*v41[6] + v41[1]*(-(links[46].mcm[3]*v41[4]) - v41[2]*links[46].inertia[1][1] + v41[1]*links[46].inertia[1][2]) + v41[2]*(-(links[46].mcm[3]*v41[5]) - v41[2]*links[46].inertia[1][2] + v41[1]*links[46].inertia[2][2]) + v41[3]*(links[46].mcm[1]*v41[4] + links[46].mcm[2]*v41[5] - v41[2]*links[46].inertia[1][3] + v41[1]*links[46].inertia[2][3]) - gravity*links[46].mcm[2]*SG410[1][3] + gravity*links[46].mcm[1]*SG410[2][3];

pv44[1]=-uex[47].f[1] - links[47].mcm[1]*Power(v44[2],2) - links[47].mcm[1]*Power(v44[3],2) + v44[1]*(links[47].mcm[2]*v44[2] + links[47].mcm[3]*v44[3]) - links[47].m*v44[3]*v44[5] + links[47].m*v44[2]*v44[6] + gravity*links[47].m*SG440[1][3];
pv44[2]=-uex[47].f[2] - links[47].mcm[2]*Power(v44[1],2) - links[47].mcm[2]*Power(v44[3],2) + v44[2]*(links[47].mcm[1]*v44[1] + links[47].mcm[3]*v44[3]) + links[47].m*v44[3]*v44[4] - links[47].m*v44[1]*v44[6] + gravity*links[47].m*SG440[2][3];
pv44[3]=-uex[47].f[3] - links[47].mcm[3]*Power(v44[1],2) - links[47].mcm[3]*Power(v44[2],2) + (links[47].mcm[1]*v44[1] + links[47].mcm[2]*v44[2])*v44[3] - links[47].m*v44[2]*v44[4] + links[47].m*v44[1]*v44[5] + gravity*links[47].m*SG440[3][3];
pv44[4]=-uex[47].t[1] + (-(links[47].mcm[2]*v44[2]) - links[47].mcm[3]*v44[3])*v44[4] + (links[47].mcm[1]*v44[3] + links[47].m*v44[5])*v44[6] + v44[5]*(links[47].mcm[1]*v44[2] - links[47].m*v44[6]) + v44[1]*(links[47].mcm[2]*v44[5] + links[47].mcm[3]*v44[6] - v44[3]*links[47].inertia[1][2] + v44[2]*links[47].inertia[1][3]) + v44[2]*(-(links[47].mcm[1]*v44[5]) - v44[3]*links[47].inertia[2][2] + v44[2]*links[47].inertia[2][3]) + v44[3]*(-(links[47].mcm[1]*v44[6]) - v44[3]*links[47].inertia[2][3] + v44[2]*links[47].inertia[3][3]) - gravity*links[47].mcm[3]*SG440[2][3] + gravity*links[47].mcm[2]*SG440[3][3];
pv44[5]=-uex[47].t[2] + (-(links[47].mcm[1]*v44[1]) - links[47].mcm[3]*v44[3])*v44[5] + (links[47].mcm[2]*v44[3] - links[47].m*v44[4])*v44[6] + v44[4]*(links[47].mcm[2]*v44[1] + links[47].m*v44[6]) + v44[1]*(-(links[47].mcm[2]*v44[4]) + v44[3]*links[47].inertia[1][1] - v44[1]*links[47].inertia[1][3]) + v44[2]*(links[47].mcm[1]*v44[4] + links[47].mcm[3]*v44[6] + v44[3]*links[47].inertia[1][2] - v44[1]*links[47].inertia[2][3]) + v44[3]*(-(links[47].mcm[2]*v44[6]) + v44[3]*links[47].inertia[1][3] - v44[1]*links[47].inertia[3][3]) + gravity*links[47].mcm[3]*SG440[1][3] - gravity*links[47].mcm[1]*SG440[3][3];
pv44[6]=-uex[47].t[3] + (links[47].mcm[3]*v44[2] + links[47].m*v44[4])*v44[5] + v44[4]*(links[47].mcm[3]*v44[1] - links[47].m*v44[5]) + (-(links[47].mcm[1]*v44[1]) - links[47].mcm[2]*v44[2])*v44[6] + v44[1]*(-(links[47].mcm[3]*v44[4]) - v44[2]*links[47].inertia[1][1] + v44[1]*links[47].inertia[1][2]) + v44[2]*(-(links[47].mcm[3]*v44[5]) - v44[2]*links[47].inertia[1][2] + v44[1]*links[47].inertia[2][2]) + v44[3]*(links[47].mcm[1]*v44[4] + links[47].mcm[2]*v44[5] - v44[2]*links[47].inertia[1][3] + v44[1]*links[47].inertia[2][3]) - gravity*links[47].mcm[2]*SG440[1][3] + gravity*links[47].mcm[1]*SG440[2][3];

pv48[1]=-uex[48].f[1] - links[48].mcm[1]*Power(v48[2],2) - links[48].mcm[1]*Power(v48[3],2) + v48[1]*(links[48].mcm[2]*v48[2] + links[48].mcm[3]*v48[3]) - links[48].m*v48[3]*v48[5] + links[48].m*v48[2]*v48[6] + gravity*links[48].m*SG480[1][3];
pv48[2]=-uex[48].f[2] - links[48].mcm[2]*Power(v48[1],2) - links[48].mcm[2]*Power(v48[3],2) + v48[2]*(links[48].mcm[1]*v48[1] + links[48].mcm[3]*v48[3]) + links[48].m*v48[3]*v48[4] - links[48].m*v48[1]*v48[6] + gravity*links[48].m*SG480[2][3];
pv48[3]=-uex[48].f[3] - links[48].mcm[3]*Power(v48[1],2) - links[48].mcm[3]*Power(v48[2],2) + (links[48].mcm[1]*v48[1] + links[48].mcm[2]*v48[2])*v48[3] - links[48].m*v48[2]*v48[4] + links[48].m*v48[1]*v48[5] + gravity*links[48].m*SG480[3][3];
pv48[4]=-uex[48].t[1] + (-(links[48].mcm[2]*v48[2]) - links[48].mcm[3]*v48[3])*v48[4] + (links[48].mcm[1]*v48[3] + links[48].m*v48[5])*v48[6] + v48[5]*(links[48].mcm[1]*v48[2] - links[48].m*v48[6]) + v48[1]*(links[48].mcm[2]*v48[5] + links[48].mcm[3]*v48[6] - v48[3]*links[48].inertia[1][2] + v48[2]*links[48].inertia[1][3]) + v48[2]*(-(links[48].mcm[1]*v48[5]) - v48[3]*links[48].inertia[2][2] + v48[2]*links[48].inertia[2][3]) + v48[3]*(-(links[48].mcm[1]*v48[6]) - v48[3]*links[48].inertia[2][3] + v48[2]*links[48].inertia[3][3]) - gravity*links[48].mcm[3]*SG480[2][3] + gravity*links[48].mcm[2]*SG480[3][3];
pv48[5]=-uex[48].t[2] + (-(links[48].mcm[1]*v48[1]) - links[48].mcm[3]*v48[3])*v48[5] + (links[48].mcm[2]*v48[3] - links[48].m*v48[4])*v48[6] + v48[4]*(links[48].mcm[2]*v48[1] + links[48].m*v48[6]) + v48[1]*(-(links[48].mcm[2]*v48[4]) + v48[3]*links[48].inertia[1][1] - v48[1]*links[48].inertia[1][3]) + v48[2]*(links[48].mcm[1]*v48[4] + links[48].mcm[3]*v48[6] + v48[3]*links[48].inertia[1][2] - v48[1]*links[48].inertia[2][3]) + v48[3]*(-(links[48].mcm[2]*v48[6]) + v48[3]*links[48].inertia[1][3] - v48[1]*links[48].inertia[3][3]) + gravity*links[48].mcm[3]*SG480[1][3] - gravity*links[48].mcm[1]*SG480[3][3];
pv48[6]=-uex[48].t[3] + (links[48].mcm[3]*v48[2] + links[48].m*v48[4])*v48[5] + v48[4]*(links[48].mcm[3]*v48[1] - links[48].m*v48[5]) + (-(links[48].mcm[1]*v48[1]) - links[48].mcm[2]*v48[2])*v48[6] + v48[1]*(-(links[48].mcm[3]*v48[4]) - v48[2]*links[48].inertia[1][1] + v48[1]*links[48].inertia[1][2]) + v48[2]*(-(links[48].mcm[3]*v48[5]) - v48[2]*links[48].inertia[1][2] + v48[1]*links[48].inertia[2][2]) + v48[3]*(links[48].mcm[1]*v48[4] + links[48].mcm[2]*v48[5] - v48[2]*links[48].inertia[1][3] + v48[1]*links[48].inertia[2][3]) - gravity*links[48].mcm[2]*SG480[1][3] + gravity*links[48].mcm[1]*SG480[2][3];

pv52[1]=-uex[49].f[1] - links[49].mcm[1]*Power(v52[2],2) - links[49].mcm[1]*Power(v52[3],2) + v52[1]*(links[49].mcm[2]*v52[2] + links[49].mcm[3]*v52[3]) - links[49].m*v52[3]*v52[5] + links[49].m*v52[2]*v52[6] + gravity*links[49].m*SG520[1][3];
pv52[2]=-uex[49].f[2] - links[49].mcm[2]*Power(v52[1],2) - links[49].mcm[2]*Power(v52[3],2) + v52[2]*(links[49].mcm[1]*v52[1] + links[49].mcm[3]*v52[3]) + links[49].m*v52[3]*v52[4] - links[49].m*v52[1]*v52[6] + gravity*links[49].m*SG520[2][3];
pv52[3]=-uex[49].f[3] - links[49].mcm[3]*Power(v52[1],2) - links[49].mcm[3]*Power(v52[2],2) + (links[49].mcm[1]*v52[1] + links[49].mcm[2]*v52[2])*v52[3] - links[49].m*v52[2]*v52[4] + links[49].m*v52[1]*v52[5] + gravity*links[49].m*SG520[3][3];
pv52[4]=-uex[49].t[1] + (-(links[49].mcm[2]*v52[2]) - links[49].mcm[3]*v52[3])*v52[4] + (links[49].mcm[1]*v52[3] + links[49].m*v52[5])*v52[6] + v52[5]*(links[49].mcm[1]*v52[2] - links[49].m*v52[6]) + v52[1]*(links[49].mcm[2]*v52[5] + links[49].mcm[3]*v52[6] - v52[3]*links[49].inertia[1][2] + v52[2]*links[49].inertia[1][3]) + v52[2]*(-(links[49].mcm[1]*v52[5]) - v52[3]*links[49].inertia[2][2] + v52[2]*links[49].inertia[2][3]) + v52[3]*(-(links[49].mcm[1]*v52[6]) - v52[3]*links[49].inertia[2][3] + v52[2]*links[49].inertia[3][3]) - gravity*links[49].mcm[3]*SG520[2][3] + gravity*links[49].mcm[2]*SG520[3][3];
pv52[5]=-uex[49].t[2] + (-(links[49].mcm[1]*v52[1]) - links[49].mcm[3]*v52[3])*v52[5] + (links[49].mcm[2]*v52[3] - links[49].m*v52[4])*v52[6] + v52[4]*(links[49].mcm[2]*v52[1] + links[49].m*v52[6]) + v52[1]*(-(links[49].mcm[2]*v52[4]) + v52[3]*links[49].inertia[1][1] - v52[1]*links[49].inertia[1][3]) + v52[2]*(links[49].mcm[1]*v52[4] + links[49].mcm[3]*v52[6] + v52[3]*links[49].inertia[1][2] - v52[1]*links[49].inertia[2][3]) + v52[3]*(-(links[49].mcm[2]*v52[6]) + v52[3]*links[49].inertia[1][3] - v52[1]*links[49].inertia[3][3]) + gravity*links[49].mcm[3]*SG520[1][3] - gravity*links[49].mcm[1]*SG520[3][3];
pv52[6]=-uex[49].t[3] + (links[49].mcm[3]*v52[2] + links[49].m*v52[4])*v52[5] + v52[4]*(links[49].mcm[3]*v52[1] - links[49].m*v52[5]) + (-(links[49].mcm[1]*v52[1]) - links[49].mcm[2]*v52[2])*v52[6] + v52[1]*(-(links[49].mcm[3]*v52[4]) - v52[2]*links[49].inertia[1][1] + v52[1]*links[49].inertia[1][2]) + v52[2]*(-(links[49].mcm[3]*v52[5]) - v52[2]*links[49].inertia[1][2] + v52[1]*links[49].inertia[2][2]) + v52[3]*(links[49].mcm[1]*v52[4] + links[49].mcm[2]*v52[5] - v52[2]*links[49].inertia[1][3] + v52[1]*links[49].inertia[2][3]) - gravity*links[49].mcm[2]*SG520[1][3] + gravity*links[49].mcm[1]*SG520[2][3];

pv56[1]=-uex[50].f[1] - links[50].mcm[1]*Power(v56[2],2) - links[50].mcm[1]*Power(v56[3],2) + v56[1]*(links[50].mcm[2]*v56[2] + links[50].mcm[3]*v56[3]) - links[50].m*v56[3]*v56[5] + links[50].m*v56[2]*v56[6] + gravity*links[50].m*SG560[1][3];
pv56[2]=-uex[50].f[2] - links[50].mcm[2]*Power(v56[1],2) - links[50].mcm[2]*Power(v56[3],2) + v56[2]*(links[50].mcm[1]*v56[1] + links[50].mcm[3]*v56[3]) + links[50].m*v56[3]*v56[4] - links[50].m*v56[1]*v56[6] + gravity*links[50].m*SG560[2][3];
pv56[3]=-uex[50].f[3] - links[50].mcm[3]*Power(v56[1],2) - links[50].mcm[3]*Power(v56[2],2) + (links[50].mcm[1]*v56[1] + links[50].mcm[2]*v56[2])*v56[3] - links[50].m*v56[2]*v56[4] + links[50].m*v56[1]*v56[5] + gravity*links[50].m*SG560[3][3];
pv56[4]=-uex[50].t[1] + (-(links[50].mcm[2]*v56[2]) - links[50].mcm[3]*v56[3])*v56[4] + (links[50].mcm[1]*v56[3] + links[50].m*v56[5])*v56[6] + v56[5]*(links[50].mcm[1]*v56[2] - links[50].m*v56[6]) + v56[1]*(links[50].mcm[2]*v56[5] + links[50].mcm[3]*v56[6] - v56[3]*links[50].inertia[1][2] + v56[2]*links[50].inertia[1][3]) + v56[2]*(-(links[50].mcm[1]*v56[5]) - v56[3]*links[50].inertia[2][2] + v56[2]*links[50].inertia[2][3]) + v56[3]*(-(links[50].mcm[1]*v56[6]) - v56[3]*links[50].inertia[2][3] + v56[2]*links[50].inertia[3][3]) - gravity*links[50].mcm[3]*SG560[2][3] + gravity*links[50].mcm[2]*SG560[3][3];
pv56[5]=-uex[50].t[2] + (-(links[50].mcm[1]*v56[1]) - links[50].mcm[3]*v56[3])*v56[5] + (links[50].mcm[2]*v56[3] - links[50].m*v56[4])*v56[6] + v56[4]*(links[50].mcm[2]*v56[1] + links[50].m*v56[6]) + v56[1]*(-(links[50].mcm[2]*v56[4]) + v56[3]*links[50].inertia[1][1] - v56[1]*links[50].inertia[1][3]) + v56[2]*(links[50].mcm[1]*v56[4] + links[50].mcm[3]*v56[6] + v56[3]*links[50].inertia[1][2] - v56[1]*links[50].inertia[2][3]) + v56[3]*(-(links[50].mcm[2]*v56[6]) + v56[3]*links[50].inertia[1][3] - v56[1]*links[50].inertia[3][3]) + gravity*links[50].mcm[3]*SG560[1][3] - gravity*links[50].mcm[1]*SG560[3][3];
pv56[6]=-uex[50].t[3] + (links[50].mcm[3]*v56[2] + links[50].m*v56[4])*v56[5] + v56[4]*(links[50].mcm[3]*v56[1] - links[50].m*v56[5]) + (-(links[50].mcm[1]*v56[1]) - links[50].mcm[2]*v56[2])*v56[6] + v56[1]*(-(links[50].mcm[3]*v56[4]) - v56[2]*links[50].inertia[1][1] + v56[1]*links[50].inertia[1][2]) + v56[2]*(-(links[50].mcm[3]*v56[5]) - v56[2]*links[50].inertia[1][2] + v56[1]*links[50].inertia[2][2]) + v56[3]*(links[50].mcm[1]*v56[4] + links[50].mcm[2]*v56[5] - v56[2]*links[50].inertia[1][3] + v56[1]*links[50].inertia[2][3]) - gravity*links[50].mcm[2]*SG560[1][3] + gravity*links[50].mcm[1]*SG560[2][3];

pv60[1]=-(links[32].mcm[1]*Power(v60[2],2)) - links[32].mcm[1]*Power(v60[3],2) + v60[1]*(links[32].mcm[2]*v60[2] + links[32].mcm[3]*v60[3]) - links[32].m*v60[3]*v60[5] + links[32].m*v60[2]*v60[6] + gravity*links[32].m*SG600[1][3];
pv60[2]=-(links[32].mcm[2]*Power(v60[1],2)) - links[32].mcm[2]*Power(v60[3],2) + v60[2]*(links[32].mcm[1]*v60[1] + links[32].mcm[3]*v60[3]) + links[32].m*v60[3]*v60[4] - links[32].m*v60[1]*v60[6] + gravity*links[32].m*SG600[2][3];
pv60[3]=-(links[32].mcm[3]*Power(v60[1],2)) - links[32].mcm[3]*Power(v60[2],2) + (links[32].mcm[1]*v60[1] + links[32].mcm[2]*v60[2])*v60[3] - links[32].m*v60[2]*v60[4] + links[32].m*v60[1]*v60[5] + gravity*links[32].m*SG600[3][3];
pv60[4]=(-(links[32].mcm[2]*v60[2]) - links[32].mcm[3]*v60[3])*v60[4] + (links[32].mcm[1]*v60[3] + links[32].m*v60[5])*v60[6] + v60[5]*(links[32].mcm[1]*v60[2] - links[32].m*v60[6]) + v60[1]*(links[32].mcm[2]*v60[5] + links[32].mcm[3]*v60[6] - v60[3]*links[32].inertia[1][2] + v60[2]*links[32].inertia[1][3]) + v60[2]*(-(links[32].mcm[1]*v60[5]) - v60[3]*links[32].inertia[2][2] + v60[2]*links[32].inertia[2][3]) + v60[3]*(-(links[32].mcm[1]*v60[6]) - v60[3]*links[32].inertia[2][3] + v60[2]*links[32].inertia[3][3]) - gravity*links[32].mcm[3]*SG600[2][3] + gravity*links[32].mcm[2]*SG600[3][3];
pv60[5]=(-(links[32].mcm[1]*v60[1]) - links[32].mcm[3]*v60[3])*v60[5] + (links[32].mcm[2]*v60[3] - links[32].m*v60[4])*v60[6] + v60[4]*(links[32].mcm[2]*v60[1] + links[32].m*v60[6]) + v60[1]*(-(links[32].mcm[2]*v60[4]) + v60[3]*links[32].inertia[1][1] - v60[1]*links[32].inertia[1][3]) + v60[2]*(links[32].mcm[1]*v60[4] + links[32].mcm[3]*v60[6] + v60[3]*links[32].inertia[1][2] - v60[1]*links[32].inertia[2][3]) + v60[3]*(-(links[32].mcm[2]*v60[6]) + v60[3]*links[32].inertia[1][3] - v60[1]*links[32].inertia[3][3]) + gravity*links[32].mcm[3]*SG600[1][3] - gravity*links[32].mcm[1]*SG600[3][3];
pv60[6]=(links[32].mcm[3]*v60[2] + links[32].m*v60[4])*v60[5] + v60[4]*(links[32].mcm[3]*v60[1] - links[32].m*v60[5]) + (-(links[32].mcm[1]*v60[1]) - links[32].mcm[2]*v60[2])*v60[6] + v60[1]*(-(links[32].mcm[3]*v60[4]) - v60[2]*links[32].inertia[1][1] + v60[1]*links[32].inertia[1][2]) + v60[2]*(-(links[32].mcm[3]*v60[5]) - v60[2]*links[32].inertia[1][2] + v60[1]*links[32].inertia[2][2]) + v60[3]*(links[32].mcm[1]*v60[4] + links[32].mcm[2]*v60[5] - v60[2]*links[32].inertia[1][3] + v60[1]*links[32].inertia[2][3]) - gravity*links[32].mcm[2]*SG600[1][3] + gravity*links[32].mcm[1]*SG600[2][3];

pv61[1]=-(links[33].mcm[1]*Power(v61[2],2)) - links[33].mcm[1]*Power(v61[3],2) + v61[1]*(links[33].mcm[2]*v61[2] + links[33].mcm[3]*v61[3]) - links[33].m*v61[3]*v61[5] + links[33].m*v61[2]*v61[6] + gravity*links[33].m*SG610[1][3];
pv61[2]=-(links[33].mcm[2]*Power(v61[1],2)) - links[33].mcm[2]*Power(v61[3],2) + v61[2]*(links[33].mcm[1]*v61[1] + links[33].mcm[3]*v61[3]) + links[33].m*v61[3]*v61[4] - links[33].m*v61[1]*v61[6] + gravity*links[33].m*SG610[2][3];
pv61[3]=-(links[33].mcm[3]*Power(v61[1],2)) - links[33].mcm[3]*Power(v61[2],2) + (links[33].mcm[1]*v61[1] + links[33].mcm[2]*v61[2])*v61[3] - links[33].m*v61[2]*v61[4] + links[33].m*v61[1]*v61[5] + gravity*links[33].m*SG610[3][3];
pv61[4]=(-(links[33].mcm[2]*v61[2]) - links[33].mcm[3]*v61[3])*v61[4] + (links[33].mcm[1]*v61[3] + links[33].m*v61[5])*v61[6] + v61[5]*(links[33].mcm[1]*v61[2] - links[33].m*v61[6]) + v61[1]*(links[33].mcm[2]*v61[5] + links[33].mcm[3]*v61[6] - v61[3]*links[33].inertia[1][2] + v61[2]*links[33].inertia[1][3]) + v61[2]*(-(links[33].mcm[1]*v61[5]) - v61[3]*links[33].inertia[2][2] + v61[2]*links[33].inertia[2][3]) + v61[3]*(-(links[33].mcm[1]*v61[6]) - v61[3]*links[33].inertia[2][3] + v61[2]*links[33].inertia[3][3]) - gravity*links[33].mcm[3]*SG610[2][3] + gravity*links[33].mcm[2]*SG610[3][3];
pv61[5]=(-(links[33].mcm[1]*v61[1]) - links[33].mcm[3]*v61[3])*v61[5] + (links[33].mcm[2]*v61[3] - links[33].m*v61[4])*v61[6] + v61[4]*(links[33].mcm[2]*v61[1] + links[33].m*v61[6]) + v61[1]*(-(links[33].mcm[2]*v61[4]) + v61[3]*links[33].inertia[1][1] - v61[1]*links[33].inertia[1][3]) + v61[2]*(links[33].mcm[1]*v61[4] + links[33].mcm[3]*v61[6] + v61[3]*links[33].inertia[1][2] - v61[1]*links[33].inertia[2][3]) + v61[3]*(-(links[33].mcm[2]*v61[6]) + v61[3]*links[33].inertia[1][3] - v61[1]*links[33].inertia[3][3]) + gravity*links[33].mcm[3]*SG610[1][3] - gravity*links[33].mcm[1]*SG610[3][3];
pv61[6]=(links[33].mcm[3]*v61[2] + links[33].m*v61[4])*v61[5] + v61[4]*(links[33].mcm[3]*v61[1] - links[33].m*v61[5]) + (-(links[33].mcm[1]*v61[1]) - links[33].mcm[2]*v61[2])*v61[6] + v61[1]*(-(links[33].mcm[3]*v61[4]) - v61[2]*links[33].inertia[1][1] + v61[1]*links[33].inertia[1][2]) + v61[2]*(-(links[33].mcm[3]*v61[5]) - v61[2]*links[33].inertia[1][2] + v61[1]*links[33].inertia[2][2]) + v61[3]*(links[33].mcm[1]*v61[4] + links[33].mcm[2]*v61[5] - v61[2]*links[33].inertia[1][3] + v61[1]*links[33].inertia[2][3]) - gravity*links[33].mcm[2]*SG610[1][3] + gravity*links[33].mcm[1]*SG610[2][3];

pv62[1]=-uex[34].f[1] - links[34].mcm[1]*Power(v62[2],2) - links[34].mcm[1]*Power(v62[3],2) + v62[1]*(links[34].mcm[2]*v62[2] + links[34].mcm[3]*v62[3]) - links[34].m*v62[3]*v62[5] + links[34].m*v62[2]*v62[6] + gravity*links[34].m*SG620[1][3];
pv62[2]=-uex[34].f[2] - links[34].mcm[2]*Power(v62[1],2) - links[34].mcm[2]*Power(v62[3],2) + v62[2]*(links[34].mcm[1]*v62[1] + links[34].mcm[3]*v62[3]) + links[34].m*v62[3]*v62[4] - links[34].m*v62[1]*v62[6] + gravity*links[34].m*SG620[2][3];
pv62[3]=-uex[34].f[3] - links[34].mcm[3]*Power(v62[1],2) - links[34].mcm[3]*Power(v62[2],2) + (links[34].mcm[1]*v62[1] + links[34].mcm[2]*v62[2])*v62[3] - links[34].m*v62[2]*v62[4] + links[34].m*v62[1]*v62[5] + gravity*links[34].m*SG620[3][3];
pv62[4]=-uex[34].t[1] + (-(links[34].mcm[2]*v62[2]) - links[34].mcm[3]*v62[3])*v62[4] + (links[34].mcm[1]*v62[3] + links[34].m*v62[5])*v62[6] + v62[5]*(links[34].mcm[1]*v62[2] - links[34].m*v62[6]) + v62[1]*(links[34].mcm[2]*v62[5] + links[34].mcm[3]*v62[6] - v62[3]*links[34].inertia[1][2] + v62[2]*links[34].inertia[1][3]) + v62[2]*(-(links[34].mcm[1]*v62[5]) - v62[3]*links[34].inertia[2][2] + v62[2]*links[34].inertia[2][3]) + v62[3]*(-(links[34].mcm[1]*v62[6]) - v62[3]*links[34].inertia[2][3] + v62[2]*links[34].inertia[3][3]) - gravity*links[34].mcm[3]*SG620[2][3] + gravity*links[34].mcm[2]*SG620[3][3];
pv62[5]=-uex[34].t[2] + (-(links[34].mcm[1]*v62[1]) - links[34].mcm[3]*v62[3])*v62[5] + (links[34].mcm[2]*v62[3] - links[34].m*v62[4])*v62[6] + v62[4]*(links[34].mcm[2]*v62[1] + links[34].m*v62[6]) + v62[1]*(-(links[34].mcm[2]*v62[4]) + v62[3]*links[34].inertia[1][1] - v62[1]*links[34].inertia[1][3]) + v62[2]*(links[34].mcm[1]*v62[4] + links[34].mcm[3]*v62[6] + v62[3]*links[34].inertia[1][2] - v62[1]*links[34].inertia[2][3]) + v62[3]*(-(links[34].mcm[2]*v62[6]) + v62[3]*links[34].inertia[1][3] - v62[1]*links[34].inertia[3][3]) + gravity*links[34].mcm[3]*SG620[1][3] - gravity*links[34].mcm[1]*SG620[3][3];
pv62[6]=-uex[34].t[3] + (links[34].mcm[3]*v62[2] + links[34].m*v62[4])*v62[5] + v62[4]*(links[34].mcm[3]*v62[1] - links[34].m*v62[5]) + (-(links[34].mcm[1]*v62[1]) - links[34].mcm[2]*v62[2])*v62[6] + v62[1]*(-(links[34].mcm[3]*v62[4]) - v62[2]*links[34].inertia[1][1] + v62[1]*links[34].inertia[1][2]) + v62[2]*(-(links[34].mcm[3]*v62[5]) - v62[2]*links[34].inertia[1][2] + v62[1]*links[34].inertia[2][2]) + v62[3]*(links[34].mcm[1]*v62[4] + links[34].mcm[2]*v62[5] - v62[2]*links[34].inertia[1][3] + v62[1]*links[34].inertia[2][3]) - gravity*links[34].mcm[2]*SG620[1][3] + gravity*links[34].mcm[1]*SG620[2][3];

pv63[1]=-(links[35].mcm[1]*Power(v63[2],2)) - links[35].mcm[1]*Power(v63[3],2) + v63[1]*(links[35].mcm[2]*v63[2] + links[35].mcm[3]*v63[3]) - links[35].m*v63[3]*v63[5] + links[35].m*v63[2]*v63[6] + gravity*links[35].m*SG630[1][3];
pv63[2]=-(links[35].mcm[2]*Power(v63[1],2)) - links[35].mcm[2]*Power(v63[3],2) + v63[2]*(links[35].mcm[1]*v63[1] + links[35].mcm[3]*v63[3]) + links[35].m*v63[3]*v63[4] - links[35].m*v63[1]*v63[6] + gravity*links[35].m*SG630[2][3];
pv63[3]=-(links[35].mcm[3]*Power(v63[1],2)) - links[35].mcm[3]*Power(v63[2],2) + (links[35].mcm[1]*v63[1] + links[35].mcm[2]*v63[2])*v63[3] - links[35].m*v63[2]*v63[4] + links[35].m*v63[1]*v63[5] + gravity*links[35].m*SG630[3][3];
pv63[4]=(-(links[35].mcm[2]*v63[2]) - links[35].mcm[3]*v63[3])*v63[4] + (links[35].mcm[1]*v63[3] + links[35].m*v63[5])*v63[6] + v63[5]*(links[35].mcm[1]*v63[2] - links[35].m*v63[6]) + v63[1]*(links[35].mcm[2]*v63[5] + links[35].mcm[3]*v63[6] - v63[3]*links[35].inertia[1][2] + v63[2]*links[35].inertia[1][3]) + v63[2]*(-(links[35].mcm[1]*v63[5]) - v63[3]*links[35].inertia[2][2] + v63[2]*links[35].inertia[2][3]) + v63[3]*(-(links[35].mcm[1]*v63[6]) - v63[3]*links[35].inertia[2][3] + v63[2]*links[35].inertia[3][3]) - gravity*links[35].mcm[3]*SG630[2][3] + gravity*links[35].mcm[2]*SG630[3][3];
pv63[5]=(-(links[35].mcm[1]*v63[1]) - links[35].mcm[3]*v63[3])*v63[5] + (links[35].mcm[2]*v63[3] - links[35].m*v63[4])*v63[6] + v63[4]*(links[35].mcm[2]*v63[1] + links[35].m*v63[6]) + v63[1]*(-(links[35].mcm[2]*v63[4]) + v63[3]*links[35].inertia[1][1] - v63[1]*links[35].inertia[1][3]) + v63[2]*(links[35].mcm[1]*v63[4] + links[35].mcm[3]*v63[6] + v63[3]*links[35].inertia[1][2] - v63[1]*links[35].inertia[2][3]) + v63[3]*(-(links[35].mcm[2]*v63[6]) + v63[3]*links[35].inertia[1][3] - v63[1]*links[35].inertia[3][3]) + gravity*links[35].mcm[3]*SG630[1][3] - gravity*links[35].mcm[1]*SG630[3][3];
pv63[6]=(links[35].mcm[3]*v63[2] + links[35].m*v63[4])*v63[5] + v63[4]*(links[35].mcm[3]*v63[1] - links[35].m*v63[5]) + (-(links[35].mcm[1]*v63[1]) - links[35].mcm[2]*v63[2])*v63[6] + v63[1]*(-(links[35].mcm[3]*v63[4]) - v63[2]*links[35].inertia[1][1] + v63[1]*links[35].inertia[1][2]) + v63[2]*(-(links[35].mcm[3]*v63[5]) - v63[2]*links[35].inertia[1][2] + v63[1]*links[35].inertia[2][2]) + v63[3]*(links[35].mcm[1]*v63[4] + links[35].mcm[2]*v63[5] - v63[2]*links[35].inertia[1][3] + v63[1]*links[35].inertia[2][3]) - gravity*links[35].mcm[2]*SG630[1][3] + gravity*links[35].mcm[1]*SG630[2][3];

pv64[1]=-(links[36].mcm[1]*Power(v64[2],2)) - links[36].mcm[1]*Power(v64[3],2) + v64[1]*(links[36].mcm[2]*v64[2] + links[36].mcm[3]*v64[3]) - links[36].m*v64[3]*v64[5] + links[36].m*v64[2]*v64[6] + gravity*links[36].m*SG640[1][3];
pv64[2]=-(links[36].mcm[2]*Power(v64[1],2)) - links[36].mcm[2]*Power(v64[3],2) + v64[2]*(links[36].mcm[1]*v64[1] + links[36].mcm[3]*v64[3]) + links[36].m*v64[3]*v64[4] - links[36].m*v64[1]*v64[6] + gravity*links[36].m*SG640[2][3];
pv64[3]=-(links[36].mcm[3]*Power(v64[1],2)) - links[36].mcm[3]*Power(v64[2],2) + (links[36].mcm[1]*v64[1] + links[36].mcm[2]*v64[2])*v64[3] - links[36].m*v64[2]*v64[4] + links[36].m*v64[1]*v64[5] + gravity*links[36].m*SG640[3][3];
pv64[4]=(-(links[36].mcm[2]*v64[2]) - links[36].mcm[3]*v64[3])*v64[4] + (links[36].mcm[1]*v64[3] + links[36].m*v64[5])*v64[6] + v64[5]*(links[36].mcm[1]*v64[2] - links[36].m*v64[6]) + v64[1]*(links[36].mcm[2]*v64[5] + links[36].mcm[3]*v64[6] - v64[3]*links[36].inertia[1][2] + v64[2]*links[36].inertia[1][3]) + v64[2]*(-(links[36].mcm[1]*v64[5]) - v64[3]*links[36].inertia[2][2] + v64[2]*links[36].inertia[2][3]) + v64[3]*(-(links[36].mcm[1]*v64[6]) - v64[3]*links[36].inertia[2][3] + v64[2]*links[36].inertia[3][3]) - gravity*links[36].mcm[3]*SG640[2][3] + gravity*links[36].mcm[2]*SG640[3][3];
pv64[5]=(-(links[36].mcm[1]*v64[1]) - links[36].mcm[3]*v64[3])*v64[5] + (links[36].mcm[2]*v64[3] - links[36].m*v64[4])*v64[6] + v64[4]*(links[36].mcm[2]*v64[1] + links[36].m*v64[6]) + v64[1]*(-(links[36].mcm[2]*v64[4]) + v64[3]*links[36].inertia[1][1] - v64[1]*links[36].inertia[1][3]) + v64[2]*(links[36].mcm[1]*v64[4] + links[36].mcm[3]*v64[6] + v64[3]*links[36].inertia[1][2] - v64[1]*links[36].inertia[2][3]) + v64[3]*(-(links[36].mcm[2]*v64[6]) + v64[3]*links[36].inertia[1][3] - v64[1]*links[36].inertia[3][3]) + gravity*links[36].mcm[3]*SG640[1][3] - gravity*links[36].mcm[1]*SG640[3][3];
pv64[6]=(links[36].mcm[3]*v64[2] + links[36].m*v64[4])*v64[5] + v64[4]*(links[36].mcm[3]*v64[1] - links[36].m*v64[5]) + (-(links[36].mcm[1]*v64[1]) - links[36].mcm[2]*v64[2])*v64[6] + v64[1]*(-(links[36].mcm[3]*v64[4]) - v64[2]*links[36].inertia[1][1] + v64[1]*links[36].inertia[1][2]) + v64[2]*(-(links[36].mcm[3]*v64[5]) - v64[2]*links[36].inertia[1][2] + v64[1]*links[36].inertia[2][2]) + v64[3]*(links[36].mcm[1]*v64[4] + links[36].mcm[2]*v64[5] - v64[2]*links[36].inertia[1][3] + v64[1]*links[36].inertia[2][3]) - gravity*links[36].mcm[2]*SG640[1][3] + gravity*links[36].mcm[1]*SG640[2][3];

pv66[1]=-(links[37].mcm[1]*Power(v66[2],2)) - links[37].mcm[1]*Power(v66[3],2) + v66[1]*(links[37].mcm[2]*v66[2] + links[37].mcm[3]*v66[3]) - links[37].m*v66[3]*v66[5] + links[37].m*v66[2]*v66[6] + gravity*links[37].m*SG660[1][3];
pv66[2]=-(links[37].mcm[2]*Power(v66[1],2)) - links[37].mcm[2]*Power(v66[3],2) + v66[2]*(links[37].mcm[1]*v66[1] + links[37].mcm[3]*v66[3]) + links[37].m*v66[3]*v66[4] - links[37].m*v66[1]*v66[6] + gravity*links[37].m*SG660[2][3];
pv66[3]=-(links[37].mcm[3]*Power(v66[1],2)) - links[37].mcm[3]*Power(v66[2],2) + (links[37].mcm[1]*v66[1] + links[37].mcm[2]*v66[2])*v66[3] - links[37].m*v66[2]*v66[4] + links[37].m*v66[1]*v66[5] + gravity*links[37].m*SG660[3][3];
pv66[4]=(-(links[37].mcm[2]*v66[2]) - links[37].mcm[3]*v66[3])*v66[4] + (links[37].mcm[1]*v66[3] + links[37].m*v66[5])*v66[6] + v66[5]*(links[37].mcm[1]*v66[2] - links[37].m*v66[6]) + v66[1]*(links[37].mcm[2]*v66[5] + links[37].mcm[3]*v66[6] - v66[3]*links[37].inertia[1][2] + v66[2]*links[37].inertia[1][3]) + v66[2]*(-(links[37].mcm[1]*v66[5]) - v66[3]*links[37].inertia[2][2] + v66[2]*links[37].inertia[2][3]) + v66[3]*(-(links[37].mcm[1]*v66[6]) - v66[3]*links[37].inertia[2][3] + v66[2]*links[37].inertia[3][3]) - gravity*links[37].mcm[3]*SG660[2][3] + gravity*links[37].mcm[2]*SG660[3][3];
pv66[5]=(-(links[37].mcm[1]*v66[1]) - links[37].mcm[3]*v66[3])*v66[5] + (links[37].mcm[2]*v66[3] - links[37].m*v66[4])*v66[6] + v66[4]*(links[37].mcm[2]*v66[1] + links[37].m*v66[6]) + v66[1]*(-(links[37].mcm[2]*v66[4]) + v66[3]*links[37].inertia[1][1] - v66[1]*links[37].inertia[1][3]) + v66[2]*(links[37].mcm[1]*v66[4] + links[37].mcm[3]*v66[6] + v66[3]*links[37].inertia[1][2] - v66[1]*links[37].inertia[2][3]) + v66[3]*(-(links[37].mcm[2]*v66[6]) + v66[3]*links[37].inertia[1][3] - v66[1]*links[37].inertia[3][3]) + gravity*links[37].mcm[3]*SG660[1][3] - gravity*links[37].mcm[1]*SG660[3][3];
pv66[6]=(links[37].mcm[3]*v66[2] + links[37].m*v66[4])*v66[5] + v66[4]*(links[37].mcm[3]*v66[1] - links[37].m*v66[5]) + (-(links[37].mcm[1]*v66[1]) - links[37].mcm[2]*v66[2])*v66[6] + v66[1]*(-(links[37].mcm[3]*v66[4]) - v66[2]*links[37].inertia[1][1] + v66[1]*links[37].inertia[1][2]) + v66[2]*(-(links[37].mcm[3]*v66[5]) - v66[2]*links[37].inertia[1][2] + v66[1]*links[37].inertia[2][2]) + v66[3]*(links[37].mcm[1]*v66[4] + links[37].mcm[2]*v66[5] - v66[2]*links[37].inertia[1][3] + v66[1]*links[37].inertia[2][3]) - gravity*links[37].mcm[2]*SG660[1][3] + gravity*links[37].mcm[1]*SG660[2][3];

pv67[1]=-(links[38].mcm[1]*Power(v67[2],2)) - links[38].mcm[1]*Power(v67[3],2) + v67[1]*(links[38].mcm[2]*v67[2] + links[38].mcm[3]*v67[3]) - links[38].m*v67[3]*v67[5] + links[38].m*v67[2]*v67[6] + gravity*links[38].m*SG670[1][3];
pv67[2]=-(links[38].mcm[2]*Power(v67[1],2)) - links[38].mcm[2]*Power(v67[3],2) + v67[2]*(links[38].mcm[1]*v67[1] + links[38].mcm[3]*v67[3]) + links[38].m*v67[3]*v67[4] - links[38].m*v67[1]*v67[6] + gravity*links[38].m*SG670[2][3];
pv67[3]=-(links[38].mcm[3]*Power(v67[1],2)) - links[38].mcm[3]*Power(v67[2],2) + (links[38].mcm[1]*v67[1] + links[38].mcm[2]*v67[2])*v67[3] - links[38].m*v67[2]*v67[4] + links[38].m*v67[1]*v67[5] + gravity*links[38].m*SG670[3][3];
pv67[4]=(-(links[38].mcm[2]*v67[2]) - links[38].mcm[3]*v67[3])*v67[4] + (links[38].mcm[1]*v67[3] + links[38].m*v67[5])*v67[6] + v67[5]*(links[38].mcm[1]*v67[2] - links[38].m*v67[6]) + v67[1]*(links[38].mcm[2]*v67[5] + links[38].mcm[3]*v67[6] - v67[3]*links[38].inertia[1][2] + v67[2]*links[38].inertia[1][3]) + v67[2]*(-(links[38].mcm[1]*v67[5]) - v67[3]*links[38].inertia[2][2] + v67[2]*links[38].inertia[2][3]) + v67[3]*(-(links[38].mcm[1]*v67[6]) - v67[3]*links[38].inertia[2][3] + v67[2]*links[38].inertia[3][3]) - gravity*links[38].mcm[3]*SG670[2][3] + gravity*links[38].mcm[2]*SG670[3][3];
pv67[5]=(-(links[38].mcm[1]*v67[1]) - links[38].mcm[3]*v67[3])*v67[5] + (links[38].mcm[2]*v67[3] - links[38].m*v67[4])*v67[6] + v67[4]*(links[38].mcm[2]*v67[1] + links[38].m*v67[6]) + v67[1]*(-(links[38].mcm[2]*v67[4]) + v67[3]*links[38].inertia[1][1] - v67[1]*links[38].inertia[1][3]) + v67[2]*(links[38].mcm[1]*v67[4] + links[38].mcm[3]*v67[6] + v67[3]*links[38].inertia[1][2] - v67[1]*links[38].inertia[2][3]) + v67[3]*(-(links[38].mcm[2]*v67[6]) + v67[3]*links[38].inertia[1][3] - v67[1]*links[38].inertia[3][3]) + gravity*links[38].mcm[3]*SG670[1][3] - gravity*links[38].mcm[1]*SG670[3][3];
pv67[6]=(links[38].mcm[3]*v67[2] + links[38].m*v67[4])*v67[5] + v67[4]*(links[38].mcm[3]*v67[1] - links[38].m*v67[5]) + (-(links[38].mcm[1]*v67[1]) - links[38].mcm[2]*v67[2])*v67[6] + v67[1]*(-(links[38].mcm[3]*v67[4]) - v67[2]*links[38].inertia[1][1] + v67[1]*links[38].inertia[1][2]) + v67[2]*(-(links[38].mcm[3]*v67[5]) - v67[2]*links[38].inertia[1][2] + v67[1]*links[38].inertia[2][2]) + v67[3]*(links[38].mcm[1]*v67[4] + links[38].mcm[2]*v67[5] - v67[2]*links[38].inertia[1][3] + v67[1]*links[38].inertia[2][3]) - gravity*links[38].mcm[2]*SG670[1][3] + gravity*links[38].mcm[1]*SG670[2][3];

pv70[1]=-uex[23].f[1] - links[23].mcm[1]*Power(v70[2],2) - links[23].mcm[1]*Power(v70[3],2) + v70[1]*(links[23].mcm[2]*v70[2] + links[23].mcm[3]*v70[3]) - links[23].m*v70[3]*v70[5] + links[23].m*v70[2]*v70[6] + gravity*links[23].m*SG700[1][3];
pv70[2]=-uex[23].f[2] - links[23].mcm[2]*Power(v70[1],2) - links[23].mcm[2]*Power(v70[3],2) + v70[2]*(links[23].mcm[1]*v70[1] + links[23].mcm[3]*v70[3]) + links[23].m*v70[3]*v70[4] - links[23].m*v70[1]*v70[6] + gravity*links[23].m*SG700[2][3];
pv70[3]=-uex[23].f[3] - links[23].mcm[3]*Power(v70[1],2) - links[23].mcm[3]*Power(v70[2],2) + (links[23].mcm[1]*v70[1] + links[23].mcm[2]*v70[2])*v70[3] - links[23].m*v70[2]*v70[4] + links[23].m*v70[1]*v70[5] + gravity*links[23].m*SG700[3][3];
pv70[4]=-uex[23].t[1] + (-(links[23].mcm[2]*v70[2]) - links[23].mcm[3]*v70[3])*v70[4] + (links[23].mcm[1]*v70[3] + links[23].m*v70[5])*v70[6] + v70[5]*(links[23].mcm[1]*v70[2] - links[23].m*v70[6]) + v70[1]*(links[23].mcm[2]*v70[5] + links[23].mcm[3]*v70[6] - v70[3]*links[23].inertia[1][2] + v70[2]*links[23].inertia[1][3]) + v70[2]*(-(links[23].mcm[1]*v70[5]) - v70[3]*links[23].inertia[2][2] + v70[2]*links[23].inertia[2][3]) + v70[3]*(-(links[23].mcm[1]*v70[6]) - v70[3]*links[23].inertia[2][3] + v70[2]*links[23].inertia[3][3]) - gravity*links[23].mcm[3]*SG700[2][3] + gravity*links[23].mcm[2]*SG700[3][3];
pv70[5]=-uex[23].t[2] + (-(links[23].mcm[1]*v70[1]) - links[23].mcm[3]*v70[3])*v70[5] + (links[23].mcm[2]*v70[3] - links[23].m*v70[4])*v70[6] + v70[4]*(links[23].mcm[2]*v70[1] + links[23].m*v70[6]) + v70[1]*(-(links[23].mcm[2]*v70[4]) + v70[3]*links[23].inertia[1][1] - v70[1]*links[23].inertia[1][3]) + v70[2]*(links[23].mcm[1]*v70[4] + links[23].mcm[3]*v70[6] + v70[3]*links[23].inertia[1][2] - v70[1]*links[23].inertia[2][3]) + v70[3]*(-(links[23].mcm[2]*v70[6]) + v70[3]*links[23].inertia[1][3] - v70[1]*links[23].inertia[3][3]) + gravity*links[23].mcm[3]*SG700[1][3] - gravity*links[23].mcm[1]*SG700[3][3];
pv70[6]=-uex[23].t[3] + (links[23].mcm[3]*v70[2] + links[23].m*v70[4])*v70[5] + v70[4]*(links[23].mcm[3]*v70[1] - links[23].m*v70[5]) + (-(links[23].mcm[1]*v70[1]) - links[23].mcm[2]*v70[2])*v70[6] + v70[1]*(-(links[23].mcm[3]*v70[4]) - v70[2]*links[23].inertia[1][1] + v70[1]*links[23].inertia[1][2]) + v70[2]*(-(links[23].mcm[3]*v70[5]) - v70[2]*links[23].inertia[1][2] + v70[1]*links[23].inertia[2][2]) + v70[3]*(links[23].mcm[1]*v70[4] + links[23].mcm[2]*v70[5] - v70[2]*links[23].inertia[1][3] + v70[1]*links[23].inertia[2][3]) - gravity*links[23].mcm[2]*SG700[1][3] + gravity*links[23].mcm[1]*SG700[2][3];

pv71[1]=-(links[22].mcm[1]*Power(v71[2],2)) - links[22].mcm[1]*Power(v71[3],2) + v71[1]*(links[22].mcm[2]*v71[2] + links[22].mcm[3]*v71[3]) - links[22].m*v71[3]*v71[5] + links[22].m*v71[2]*v71[6] + gravity*links[22].m*SG710[1][3];
pv71[2]=-(links[22].mcm[2]*Power(v71[1],2)) - links[22].mcm[2]*Power(v71[3],2) + v71[2]*(links[22].mcm[1]*v71[1] + links[22].mcm[3]*v71[3]) + links[22].m*v71[3]*v71[4] - links[22].m*v71[1]*v71[6] + gravity*links[22].m*SG710[2][3];
pv71[3]=-(links[22].mcm[3]*Power(v71[1],2)) - links[22].mcm[3]*Power(v71[2],2) + (links[22].mcm[1]*v71[1] + links[22].mcm[2]*v71[2])*v71[3] - links[22].m*v71[2]*v71[4] + links[22].m*v71[1]*v71[5] + gravity*links[22].m*SG710[3][3];
pv71[4]=(-(links[22].mcm[2]*v71[2]) - links[22].mcm[3]*v71[3])*v71[4] + (links[22].mcm[1]*v71[3] + links[22].m*v71[5])*v71[6] + v71[5]*(links[22].mcm[1]*v71[2] - links[22].m*v71[6]) + v71[1]*(links[22].mcm[2]*v71[5] + links[22].mcm[3]*v71[6] - v71[3]*links[22].inertia[1][2] + v71[2]*links[22].inertia[1][3]) + v71[2]*(-(links[22].mcm[1]*v71[5]) - v71[3]*links[22].inertia[2][2] + v71[2]*links[22].inertia[2][3]) + v71[3]*(-(links[22].mcm[1]*v71[6]) - v71[3]*links[22].inertia[2][3] + v71[2]*links[22].inertia[3][3]) - gravity*links[22].mcm[3]*SG710[2][3] + gravity*links[22].mcm[2]*SG710[3][3];
pv71[5]=(-(links[22].mcm[1]*v71[1]) - links[22].mcm[3]*v71[3])*v71[5] + (links[22].mcm[2]*v71[3] - links[22].m*v71[4])*v71[6] + v71[4]*(links[22].mcm[2]*v71[1] + links[22].m*v71[6]) + v71[1]*(-(links[22].mcm[2]*v71[4]) + v71[3]*links[22].inertia[1][1] - v71[1]*links[22].inertia[1][3]) + v71[2]*(links[22].mcm[1]*v71[4] + links[22].mcm[3]*v71[6] + v71[3]*links[22].inertia[1][2] - v71[1]*links[22].inertia[2][3]) + v71[3]*(-(links[22].mcm[2]*v71[6]) + v71[3]*links[22].inertia[1][3] - v71[1]*links[22].inertia[3][3]) + gravity*links[22].mcm[3]*SG710[1][3] - gravity*links[22].mcm[1]*SG710[3][3];
pv71[6]=(links[22].mcm[3]*v71[2] + links[22].m*v71[4])*v71[5] + v71[4]*(links[22].mcm[3]*v71[1] - links[22].m*v71[5]) + (-(links[22].mcm[1]*v71[1]) - links[22].mcm[2]*v71[2])*v71[6] + v71[1]*(-(links[22].mcm[3]*v71[4]) - v71[2]*links[22].inertia[1][1] + v71[1]*links[22].inertia[1][2]) + v71[2]*(-(links[22].mcm[3]*v71[5]) - v71[2]*links[22].inertia[1][2] + v71[1]*links[22].inertia[2][2]) + v71[3]*(links[22].mcm[1]*v71[4] + links[22].mcm[2]*v71[5] - v71[2]*links[22].inertia[1][3] + v71[1]*links[22].inertia[2][3]) - gravity*links[22].mcm[2]*SG710[1][3] + gravity*links[22].mcm[1]*SG710[2][3];

pv72[1]=-(links[24].mcm[1]*Power(v72[2],2)) - links[24].mcm[1]*Power(v72[3],2) + v72[1]*(links[24].mcm[2]*v72[2] + links[24].mcm[3]*v72[3]) - links[24].m*v72[3]*v72[5] + links[24].m*v72[2]*v72[6] + gravity*links[24].m*SG720[1][3];
pv72[2]=-(links[24].mcm[2]*Power(v72[1],2)) - links[24].mcm[2]*Power(v72[3],2) + v72[2]*(links[24].mcm[1]*v72[1] + links[24].mcm[3]*v72[3]) + links[24].m*v72[3]*v72[4] - links[24].m*v72[1]*v72[6] + gravity*links[24].m*SG720[2][3];
pv72[3]=-(links[24].mcm[3]*Power(v72[1],2)) - links[24].mcm[3]*Power(v72[2],2) + (links[24].mcm[1]*v72[1] + links[24].mcm[2]*v72[2])*v72[3] - links[24].m*v72[2]*v72[4] + links[24].m*v72[1]*v72[5] + gravity*links[24].m*SG720[3][3];
pv72[4]=(-(links[24].mcm[2]*v72[2]) - links[24].mcm[3]*v72[3])*v72[4] + (links[24].mcm[1]*v72[3] + links[24].m*v72[5])*v72[6] + v72[5]*(links[24].mcm[1]*v72[2] - links[24].m*v72[6]) + v72[1]*(links[24].mcm[2]*v72[5] + links[24].mcm[3]*v72[6] - v72[3]*links[24].inertia[1][2] + v72[2]*links[24].inertia[1][3]) + v72[2]*(-(links[24].mcm[1]*v72[5]) - v72[3]*links[24].inertia[2][2] + v72[2]*links[24].inertia[2][3]) + v72[3]*(-(links[24].mcm[1]*v72[6]) - v72[3]*links[24].inertia[2][3] + v72[2]*links[24].inertia[3][3]) - gravity*links[24].mcm[3]*SG720[2][3] + gravity*links[24].mcm[2]*SG720[3][3];
pv72[5]=(-(links[24].mcm[1]*v72[1]) - links[24].mcm[3]*v72[3])*v72[5] + (links[24].mcm[2]*v72[3] - links[24].m*v72[4])*v72[6] + v72[4]*(links[24].mcm[2]*v72[1] + links[24].m*v72[6]) + v72[1]*(-(links[24].mcm[2]*v72[4]) + v72[3]*links[24].inertia[1][1] - v72[1]*links[24].inertia[1][3]) + v72[2]*(links[24].mcm[1]*v72[4] + links[24].mcm[3]*v72[6] + v72[3]*links[24].inertia[1][2] - v72[1]*links[24].inertia[2][3]) + v72[3]*(-(links[24].mcm[2]*v72[6]) + v72[3]*links[24].inertia[1][3] - v72[1]*links[24].inertia[3][3]) + gravity*links[24].mcm[3]*SG720[1][3] - gravity*links[24].mcm[1]*SG720[3][3];
pv72[6]=(links[24].mcm[3]*v72[2] + links[24].m*v72[4])*v72[5] + v72[4]*(links[24].mcm[3]*v72[1] - links[24].m*v72[5]) + (-(links[24].mcm[1]*v72[1]) - links[24].mcm[2]*v72[2])*v72[6] + v72[1]*(-(links[24].mcm[3]*v72[4]) - v72[2]*links[24].inertia[1][1] + v72[1]*links[24].inertia[1][2]) + v72[2]*(-(links[24].mcm[3]*v72[5]) - v72[2]*links[24].inertia[1][2] + v72[1]*links[24].inertia[2][2]) + v72[3]*(links[24].mcm[1]*v72[4] + links[24].mcm[2]*v72[5] - v72[2]*links[24].inertia[1][3] + v72[1]*links[24].inertia[2][3]) - gravity*links[24].mcm[2]*SG720[1][3] + gravity*links[24].mcm[1]*SG720[2][3];

pv73[1]=-uex[25].f[1] - links[25].mcm[1]*Power(v73[2],2) - links[25].mcm[1]*Power(v73[3],2) + v73[1]*(links[25].mcm[2]*v73[2] + links[25].mcm[3]*v73[3]) - links[25].m*v73[3]*v73[5] + links[25].m*v73[2]*v73[6] + gravity*links[25].m*SG730[1][3];
pv73[2]=-uex[25].f[2] - links[25].mcm[2]*Power(v73[1],2) - links[25].mcm[2]*Power(v73[3],2) + v73[2]*(links[25].mcm[1]*v73[1] + links[25].mcm[3]*v73[3]) + links[25].m*v73[3]*v73[4] - links[25].m*v73[1]*v73[6] + gravity*links[25].m*SG730[2][3];
pv73[3]=-uex[25].f[3] - links[25].mcm[3]*Power(v73[1],2) - links[25].mcm[3]*Power(v73[2],2) + (links[25].mcm[1]*v73[1] + links[25].mcm[2]*v73[2])*v73[3] - links[25].m*v73[2]*v73[4] + links[25].m*v73[1]*v73[5] + gravity*links[25].m*SG730[3][3];
pv73[4]=-uex[25].t[1] + (-(links[25].mcm[2]*v73[2]) - links[25].mcm[3]*v73[3])*v73[4] + (links[25].mcm[1]*v73[3] + links[25].m*v73[5])*v73[6] + v73[5]*(links[25].mcm[1]*v73[2] - links[25].m*v73[6]) + v73[1]*(links[25].mcm[2]*v73[5] + links[25].mcm[3]*v73[6] - v73[3]*links[25].inertia[1][2] + v73[2]*links[25].inertia[1][3]) + v73[2]*(-(links[25].mcm[1]*v73[5]) - v73[3]*links[25].inertia[2][2] + v73[2]*links[25].inertia[2][3]) + v73[3]*(-(links[25].mcm[1]*v73[6]) - v73[3]*links[25].inertia[2][3] + v73[2]*links[25].inertia[3][3]) - gravity*links[25].mcm[3]*SG730[2][3] + gravity*links[25].mcm[2]*SG730[3][3];
pv73[5]=-uex[25].t[2] + (-(links[25].mcm[1]*v73[1]) - links[25].mcm[3]*v73[3])*v73[5] + (links[25].mcm[2]*v73[3] - links[25].m*v73[4])*v73[6] + v73[4]*(links[25].mcm[2]*v73[1] + links[25].m*v73[6]) + v73[1]*(-(links[25].mcm[2]*v73[4]) + v73[3]*links[25].inertia[1][1] - v73[1]*links[25].inertia[1][3]) + v73[2]*(links[25].mcm[1]*v73[4] + links[25].mcm[3]*v73[6] + v73[3]*links[25].inertia[1][2] - v73[1]*links[25].inertia[2][3]) + v73[3]*(-(links[25].mcm[2]*v73[6]) + v73[3]*links[25].inertia[1][3] - v73[1]*links[25].inertia[3][3]) + gravity*links[25].mcm[3]*SG730[1][3] - gravity*links[25].mcm[1]*SG730[3][3];
pv73[6]=-uex[25].t[3] + (links[25].mcm[3]*v73[2] + links[25].m*v73[4])*v73[5] + v73[4]*(links[25].mcm[3]*v73[1] - links[25].m*v73[5]) + (-(links[25].mcm[1]*v73[1]) - links[25].mcm[2]*v73[2])*v73[6] + v73[1]*(-(links[25].mcm[3]*v73[4]) - v73[2]*links[25].inertia[1][1] + v73[1]*links[25].inertia[1][2]) + v73[2]*(-(links[25].mcm[3]*v73[5]) - v73[2]*links[25].inertia[1][2] + v73[1]*links[25].inertia[2][2]) + v73[3]*(links[25].mcm[1]*v73[4] + links[25].mcm[2]*v73[5] - v73[2]*links[25].inertia[1][3] + v73[1]*links[25].inertia[2][3]) - gravity*links[25].mcm[2]*SG730[1][3] + gravity*links[25].mcm[1]*SG730[2][3];

pv74[1]=-(links[26].mcm[1]*Power(v74[2],2)) - links[26].mcm[1]*Power(v74[3],2) + v74[1]*(links[26].mcm[2]*v74[2] + links[26].mcm[3]*v74[3]) - links[26].m*v74[3]*v74[5] + links[26].m*v74[2]*v74[6] + gravity*links[26].m*SG740[1][3];
pv74[2]=-(links[26].mcm[2]*Power(v74[1],2)) - links[26].mcm[2]*Power(v74[3],2) + v74[2]*(links[26].mcm[1]*v74[1] + links[26].mcm[3]*v74[3]) + links[26].m*v74[3]*v74[4] - links[26].m*v74[1]*v74[6] + gravity*links[26].m*SG740[2][3];
pv74[3]=-(links[26].mcm[3]*Power(v74[1],2)) - links[26].mcm[3]*Power(v74[2],2) + (links[26].mcm[1]*v74[1] + links[26].mcm[2]*v74[2])*v74[3] - links[26].m*v74[2]*v74[4] + links[26].m*v74[1]*v74[5] + gravity*links[26].m*SG740[3][3];
pv74[4]=(-(links[26].mcm[2]*v74[2]) - links[26].mcm[3]*v74[3])*v74[4] + (links[26].mcm[1]*v74[3] + links[26].m*v74[5])*v74[6] + v74[5]*(links[26].mcm[1]*v74[2] - links[26].m*v74[6]) + v74[1]*(links[26].mcm[2]*v74[5] + links[26].mcm[3]*v74[6] - v74[3]*links[26].inertia[1][2] + v74[2]*links[26].inertia[1][3]) + v74[2]*(-(links[26].mcm[1]*v74[5]) - v74[3]*links[26].inertia[2][2] + v74[2]*links[26].inertia[2][3]) + v74[3]*(-(links[26].mcm[1]*v74[6]) - v74[3]*links[26].inertia[2][3] + v74[2]*links[26].inertia[3][3]) - gravity*links[26].mcm[3]*SG740[2][3] + gravity*links[26].mcm[2]*SG740[3][3];
pv74[5]=(-(links[26].mcm[1]*v74[1]) - links[26].mcm[3]*v74[3])*v74[5] + (links[26].mcm[2]*v74[3] - links[26].m*v74[4])*v74[6] + v74[4]*(links[26].mcm[2]*v74[1] + links[26].m*v74[6]) + v74[1]*(-(links[26].mcm[2]*v74[4]) + v74[3]*links[26].inertia[1][1] - v74[1]*links[26].inertia[1][3]) + v74[2]*(links[26].mcm[1]*v74[4] + links[26].mcm[3]*v74[6] + v74[3]*links[26].inertia[1][2] - v74[1]*links[26].inertia[2][3]) + v74[3]*(-(links[26].mcm[2]*v74[6]) + v74[3]*links[26].inertia[1][3] - v74[1]*links[26].inertia[3][3]) + gravity*links[26].mcm[3]*SG740[1][3] - gravity*links[26].mcm[1]*SG740[3][3];
pv74[6]=(links[26].mcm[3]*v74[2] + links[26].m*v74[4])*v74[5] + v74[4]*(links[26].mcm[3]*v74[1] - links[26].m*v74[5]) + (-(links[26].mcm[1]*v74[1]) - links[26].mcm[2]*v74[2])*v74[6] + v74[1]*(-(links[26].mcm[3]*v74[4]) - v74[2]*links[26].inertia[1][1] + v74[1]*links[26].inertia[1][2]) + v74[2]*(-(links[26].mcm[3]*v74[5]) - v74[2]*links[26].inertia[1][2] + v74[1]*links[26].inertia[2][2]) + v74[3]*(links[26].mcm[1]*v74[4] + links[26].mcm[2]*v74[5] - v74[2]*links[26].inertia[1][3] + v74[1]*links[26].inertia[2][3]) - gravity*links[26].mcm[2]*SG740[1][3] + gravity*links[26].mcm[1]*SG740[2][3];

pv75[1]=-uex[27].f[1] - links[27].mcm[1]*Power(v75[2],2) - links[27].mcm[1]*Power(v75[3],2) + v75[1]*(links[27].mcm[2]*v75[2] + links[27].mcm[3]*v75[3]) - links[27].m*v75[3]*v75[5] + links[27].m*v75[2]*v75[6] + gravity*links[27].m*SG750[1][3];
pv75[2]=-uex[27].f[2] - links[27].mcm[2]*Power(v75[1],2) - links[27].mcm[2]*Power(v75[3],2) + v75[2]*(links[27].mcm[1]*v75[1] + links[27].mcm[3]*v75[3]) + links[27].m*v75[3]*v75[4] - links[27].m*v75[1]*v75[6] + gravity*links[27].m*SG750[2][3];
pv75[3]=-uex[27].f[3] - links[27].mcm[3]*Power(v75[1],2) - links[27].mcm[3]*Power(v75[2],2) + (links[27].mcm[1]*v75[1] + links[27].mcm[2]*v75[2])*v75[3] - links[27].m*v75[2]*v75[4] + links[27].m*v75[1]*v75[5] + gravity*links[27].m*SG750[3][3];
pv75[4]=-uex[27].t[1] + (-(links[27].mcm[2]*v75[2]) - links[27].mcm[3]*v75[3])*v75[4] + (links[27].mcm[1]*v75[3] + links[27].m*v75[5])*v75[6] + v75[5]*(links[27].mcm[1]*v75[2] - links[27].m*v75[6]) + v75[1]*(links[27].mcm[2]*v75[5] + links[27].mcm[3]*v75[6] - v75[3]*links[27].inertia[1][2] + v75[2]*links[27].inertia[1][3]) + v75[2]*(-(links[27].mcm[1]*v75[5]) - v75[3]*links[27].inertia[2][2] + v75[2]*links[27].inertia[2][3]) + v75[3]*(-(links[27].mcm[1]*v75[6]) - v75[3]*links[27].inertia[2][3] + v75[2]*links[27].inertia[3][3]) - gravity*links[27].mcm[3]*SG750[2][3] + gravity*links[27].mcm[2]*SG750[3][3];
pv75[5]=-uex[27].t[2] + (-(links[27].mcm[1]*v75[1]) - links[27].mcm[3]*v75[3])*v75[5] + (links[27].mcm[2]*v75[3] - links[27].m*v75[4])*v75[6] + v75[4]*(links[27].mcm[2]*v75[1] + links[27].m*v75[6]) + v75[1]*(-(links[27].mcm[2]*v75[4]) + v75[3]*links[27].inertia[1][1] - v75[1]*links[27].inertia[1][3]) + v75[2]*(links[27].mcm[1]*v75[4] + links[27].mcm[3]*v75[6] + v75[3]*links[27].inertia[1][2] - v75[1]*links[27].inertia[2][3]) + v75[3]*(-(links[27].mcm[2]*v75[6]) + v75[3]*links[27].inertia[1][3] - v75[1]*links[27].inertia[3][3]) + gravity*links[27].mcm[3]*SG750[1][3] - gravity*links[27].mcm[1]*SG750[3][3];
pv75[6]=-uex[27].t[3] + (links[27].mcm[3]*v75[2] + links[27].m*v75[4])*v75[5] + v75[4]*(links[27].mcm[3]*v75[1] - links[27].m*v75[5]) + (-(links[27].mcm[1]*v75[1]) - links[27].mcm[2]*v75[2])*v75[6] + v75[1]*(-(links[27].mcm[3]*v75[4]) - v75[2]*links[27].inertia[1][1] + v75[1]*links[27].inertia[1][2]) + v75[2]*(-(links[27].mcm[3]*v75[5]) - v75[2]*links[27].inertia[1][2] + v75[1]*links[27].inertia[2][2]) + v75[3]*(links[27].mcm[1]*v75[4] + links[27].mcm[2]*v75[5] - v75[2]*links[27].inertia[1][3] + v75[1]*links[27].inertia[2][3]) - gravity*links[27].mcm[2]*SG750[1][3] + gravity*links[27].mcm[1]*SG750[2][3];

pv76[1]=-uex[28].f[1] - links[28].mcm[1]*Power(v76[2],2) - links[28].mcm[1]*Power(v76[3],2) + v76[1]*(links[28].mcm[2]*v76[2] + links[28].mcm[3]*v76[3]) - links[28].m*v76[3]*v76[5] + links[28].m*v76[2]*v76[6] + gravity*links[28].m*SG760[1][3];
pv76[2]=-uex[28].f[2] - links[28].mcm[2]*Power(v76[1],2) - links[28].mcm[2]*Power(v76[3],2) + v76[2]*(links[28].mcm[1]*v76[1] + links[28].mcm[3]*v76[3]) + links[28].m*v76[3]*v76[4] - links[28].m*v76[1]*v76[6] + gravity*links[28].m*SG760[2][3];
pv76[3]=-uex[28].f[3] - links[28].mcm[3]*Power(v76[1],2) - links[28].mcm[3]*Power(v76[2],2) + (links[28].mcm[1]*v76[1] + links[28].mcm[2]*v76[2])*v76[3] - links[28].m*v76[2]*v76[4] + links[28].m*v76[1]*v76[5] + gravity*links[28].m*SG760[3][3];
pv76[4]=-uex[28].t[1] + (-(links[28].mcm[2]*v76[2]) - links[28].mcm[3]*v76[3])*v76[4] + (links[28].mcm[1]*v76[3] + links[28].m*v76[5])*v76[6] + v76[5]*(links[28].mcm[1]*v76[2] - links[28].m*v76[6]) + v76[1]*(links[28].mcm[2]*v76[5] + links[28].mcm[3]*v76[6] - v76[3]*links[28].inertia[1][2] + v76[2]*links[28].inertia[1][3]) + v76[2]*(-(links[28].mcm[1]*v76[5]) - v76[3]*links[28].inertia[2][2] + v76[2]*links[28].inertia[2][3]) + v76[3]*(-(links[28].mcm[1]*v76[6]) - v76[3]*links[28].inertia[2][3] + v76[2]*links[28].inertia[3][3]) - gravity*links[28].mcm[3]*SG760[2][3] + gravity*links[28].mcm[2]*SG760[3][3];
pv76[5]=-uex[28].t[2] + (-(links[28].mcm[1]*v76[1]) - links[28].mcm[3]*v76[3])*v76[5] + (links[28].mcm[2]*v76[3] - links[28].m*v76[4])*v76[6] + v76[4]*(links[28].mcm[2]*v76[1] + links[28].m*v76[6]) + v76[1]*(-(links[28].mcm[2]*v76[4]) + v76[3]*links[28].inertia[1][1] - v76[1]*links[28].inertia[1][3]) + v76[2]*(links[28].mcm[1]*v76[4] + links[28].mcm[3]*v76[6] + v76[3]*links[28].inertia[1][2] - v76[1]*links[28].inertia[2][3]) + v76[3]*(-(links[28].mcm[2]*v76[6]) + v76[3]*links[28].inertia[1][3] - v76[1]*links[28].inertia[3][3]) + gravity*links[28].mcm[3]*SG760[1][3] - gravity*links[28].mcm[1]*SG760[3][3];
pv76[6]=-uex[28].t[3] + (links[28].mcm[3]*v76[2] + links[28].m*v76[4])*v76[5] + v76[4]*(links[28].mcm[3]*v76[1] - links[28].m*v76[5]) + (-(links[28].mcm[1]*v76[1]) - links[28].mcm[2]*v76[2])*v76[6] + v76[1]*(-(links[28].mcm[3]*v76[4]) - v76[2]*links[28].inertia[1][1] + v76[1]*links[28].inertia[1][2]) + v76[2]*(-(links[28].mcm[3]*v76[5]) - v76[2]*links[28].inertia[1][2] + v76[1]*links[28].inertia[2][2]) + v76[3]*(links[28].mcm[1]*v76[4] + links[28].mcm[2]*v76[5] - v76[2]*links[28].inertia[1][3] + v76[1]*links[28].inertia[2][3]) - gravity*links[28].mcm[2]*SG760[1][3] + gravity*links[28].mcm[1]*SG760[2][3];

pv83[1]=-(eff[3].mcm[1]*Power(v83[2],2)) - eff[3].mcm[1]*Power(v83[3],2) + v83[1]*(eff[3].mcm[2]*v83[2] + eff[3].mcm[3]*v83[3]) - eff[3].m*v83[3]*v83[5] + eff[3].m*v83[2]*v83[6] + eff[3].m*gravity*SG830[1][3];
pv83[2]=-(eff[3].mcm[2]*Power(v83[1],2)) - eff[3].mcm[2]*Power(v83[3],2) + v83[2]*(eff[3].mcm[1]*v83[1] + eff[3].mcm[3]*v83[3]) + eff[3].m*v83[3]*v83[4] - eff[3].m*v83[1]*v83[6] + eff[3].m*gravity*SG830[2][3];
pv83[3]=-(eff[3].mcm[3]*Power(v83[1],2)) - eff[3].mcm[3]*Power(v83[2],2) + (eff[3].mcm[1]*v83[1] + eff[3].mcm[2]*v83[2])*v83[3] - eff[3].m*v83[2]*v83[4] + eff[3].m*v83[1]*v83[5] + eff[3].m*gravity*SG830[3][3];
pv83[4]=(-(eff[3].mcm[2]*v83[2]) - eff[3].mcm[3]*v83[3])*v83[4] - eff[3].mcm[1]*v83[2]*v83[5] - eff[3].mcm[1]*v83[3]*v83[6] + (eff[3].mcm[1]*v83[3] + eff[3].m*v83[5])*v83[6] + v83[5]*(eff[3].mcm[1]*v83[2] - eff[3].m*v83[6]) + v83[1]*(eff[3].mcm[2]*v83[5] + eff[3].mcm[3]*v83[6]) - gravity*eff[3].mcm[3]*SG830[2][3] + gravity*eff[3].mcm[2]*SG830[3][3];
pv83[5]=-(eff[3].mcm[2]*v83[1]*v83[4]) + (-(eff[3].mcm[1]*v83[1]) - eff[3].mcm[3]*v83[3])*v83[5] - eff[3].mcm[2]*v83[3]*v83[6] + (eff[3].mcm[2]*v83[3] - eff[3].m*v83[4])*v83[6] + v83[4]*(eff[3].mcm[2]*v83[1] + eff[3].m*v83[6]) + v83[2]*(eff[3].mcm[1]*v83[4] + eff[3].mcm[3]*v83[6]) + gravity*eff[3].mcm[3]*SG830[1][3] - gravity*eff[3].mcm[1]*SG830[3][3];
pv83[6]=-(eff[3].mcm[3]*v83[1]*v83[4]) - eff[3].mcm[3]*v83[2]*v83[5] + (eff[3].mcm[3]*v83[2] + eff[3].m*v83[4])*v83[5] + v83[4]*(eff[3].mcm[3]*v83[1] - eff[3].m*v83[5]) + v83[3]*(eff[3].mcm[1]*v83[4] + eff[3].mcm[2]*v83[5]) + (-(eff[3].mcm[1]*v83[1]) - eff[3].mcm[2]*v83[2])*v83[6] - gravity*eff[3].mcm[2]*SG830[1][3] + gravity*eff[3].mcm[1]*SG830[2][3];

pv84[1]=-uex[16].f[1] - links[16].mcm[1]*Power(v84[2],2) - links[16].mcm[1]*Power(v84[3],2) + v84[1]*(links[16].mcm[2]*v84[2] + links[16].mcm[3]*v84[3]) - links[16].m*v84[3]*v84[5] + links[16].m*v84[2]*v84[6] + gravity*links[16].m*SG840[1][3];
pv84[2]=-uex[16].f[2] - links[16].mcm[2]*Power(v84[1],2) - links[16].mcm[2]*Power(v84[3],2) + v84[2]*(links[16].mcm[1]*v84[1] + links[16].mcm[3]*v84[3]) + links[16].m*v84[3]*v84[4] - links[16].m*v84[1]*v84[6] + gravity*links[16].m*SG840[2][3];
pv84[3]=-uex[16].f[3] - links[16].mcm[3]*Power(v84[1],2) - links[16].mcm[3]*Power(v84[2],2) + (links[16].mcm[1]*v84[1] + links[16].mcm[2]*v84[2])*v84[3] - links[16].m*v84[2]*v84[4] + links[16].m*v84[1]*v84[5] + gravity*links[16].m*SG840[3][3];
pv84[4]=-uex[16].t[1] + (-(links[16].mcm[2]*v84[2]) - links[16].mcm[3]*v84[3])*v84[4] + (links[16].mcm[1]*v84[3] + links[16].m*v84[5])*v84[6] + v84[5]*(links[16].mcm[1]*v84[2] - links[16].m*v84[6]) + v84[1]*(links[16].mcm[2]*v84[5] + links[16].mcm[3]*v84[6] - v84[3]*links[16].inertia[1][2] + v84[2]*links[16].inertia[1][3]) + v84[2]*(-(links[16].mcm[1]*v84[5]) - v84[3]*links[16].inertia[2][2] + v84[2]*links[16].inertia[2][3]) + v84[3]*(-(links[16].mcm[1]*v84[6]) - v84[3]*links[16].inertia[2][3] + v84[2]*links[16].inertia[3][3]) - gravity*links[16].mcm[3]*SG840[2][3] + gravity*links[16].mcm[2]*SG840[3][3];
pv84[5]=-uex[16].t[2] + (-(links[16].mcm[1]*v84[1]) - links[16].mcm[3]*v84[3])*v84[5] + (links[16].mcm[2]*v84[3] - links[16].m*v84[4])*v84[6] + v84[4]*(links[16].mcm[2]*v84[1] + links[16].m*v84[6]) + v84[1]*(-(links[16].mcm[2]*v84[4]) + v84[3]*links[16].inertia[1][1] - v84[1]*links[16].inertia[1][3]) + v84[2]*(links[16].mcm[1]*v84[4] + links[16].mcm[3]*v84[6] + v84[3]*links[16].inertia[1][2] - v84[1]*links[16].inertia[2][3]) + v84[3]*(-(links[16].mcm[2]*v84[6]) + v84[3]*links[16].inertia[1][3] - v84[1]*links[16].inertia[3][3]) + gravity*links[16].mcm[3]*SG840[1][3] - gravity*links[16].mcm[1]*SG840[3][3];
pv84[6]=-uex[16].t[3] + (links[16].mcm[3]*v84[2] + links[16].m*v84[4])*v84[5] + v84[4]*(links[16].mcm[3]*v84[1] - links[16].m*v84[5]) + (-(links[16].mcm[1]*v84[1]) - links[16].mcm[2]*v84[2])*v84[6] + v84[1]*(-(links[16].mcm[3]*v84[4]) - v84[2]*links[16].inertia[1][1] + v84[1]*links[16].inertia[1][2]) + v84[2]*(-(links[16].mcm[3]*v84[5]) - v84[2]*links[16].inertia[1][2] + v84[1]*links[16].inertia[2][2]) + v84[3]*(links[16].mcm[1]*v84[4] + links[16].mcm[2]*v84[5] - v84[2]*links[16].inertia[1][3] + v84[1]*links[16].inertia[2][3]) - gravity*links[16].mcm[2]*SG840[1][3] + gravity*links[16].mcm[1]*SG840[2][3];

pv85[1]=-(links[15].mcm[1]*Power(v85[2],2)) - links[15].mcm[1]*Power(v85[3],2) + v85[1]*(links[15].mcm[2]*v85[2] + links[15].mcm[3]*v85[3]) - links[15].m*v85[3]*v85[5] + links[15].m*v85[2]*v85[6] + gravity*links[15].m*SG850[1][3];
pv85[2]=-(links[15].mcm[2]*Power(v85[1],2)) - links[15].mcm[2]*Power(v85[3],2) + v85[2]*(links[15].mcm[1]*v85[1] + links[15].mcm[3]*v85[3]) + links[15].m*v85[3]*v85[4] - links[15].m*v85[1]*v85[6] + gravity*links[15].m*SG850[2][3];
pv85[3]=-(links[15].mcm[3]*Power(v85[1],2)) - links[15].mcm[3]*Power(v85[2],2) + (links[15].mcm[1]*v85[1] + links[15].mcm[2]*v85[2])*v85[3] - links[15].m*v85[2]*v85[4] + links[15].m*v85[1]*v85[5] + gravity*links[15].m*SG850[3][3];
pv85[4]=(-(links[15].mcm[2]*v85[2]) - links[15].mcm[3]*v85[3])*v85[4] + (links[15].mcm[1]*v85[3] + links[15].m*v85[5])*v85[6] + v85[5]*(links[15].mcm[1]*v85[2] - links[15].m*v85[6]) + v85[1]*(links[15].mcm[2]*v85[5] + links[15].mcm[3]*v85[6] - v85[3]*links[15].inertia[1][2] + v85[2]*links[15].inertia[1][3]) + v85[2]*(-(links[15].mcm[1]*v85[5]) - v85[3]*links[15].inertia[2][2] + v85[2]*links[15].inertia[2][3]) + v85[3]*(-(links[15].mcm[1]*v85[6]) - v85[3]*links[15].inertia[2][3] + v85[2]*links[15].inertia[3][3]) - gravity*links[15].mcm[3]*SG850[2][3] + gravity*links[15].mcm[2]*SG850[3][3];
pv85[5]=(-(links[15].mcm[1]*v85[1]) - links[15].mcm[3]*v85[3])*v85[5] + (links[15].mcm[2]*v85[3] - links[15].m*v85[4])*v85[6] + v85[4]*(links[15].mcm[2]*v85[1] + links[15].m*v85[6]) + v85[1]*(-(links[15].mcm[2]*v85[4]) + v85[3]*links[15].inertia[1][1] - v85[1]*links[15].inertia[1][3]) + v85[2]*(links[15].mcm[1]*v85[4] + links[15].mcm[3]*v85[6] + v85[3]*links[15].inertia[1][2] - v85[1]*links[15].inertia[2][3]) + v85[3]*(-(links[15].mcm[2]*v85[6]) + v85[3]*links[15].inertia[1][3] - v85[1]*links[15].inertia[3][3]) + gravity*links[15].mcm[3]*SG850[1][3] - gravity*links[15].mcm[1]*SG850[3][3];
pv85[6]=(links[15].mcm[3]*v85[2] + links[15].m*v85[4])*v85[5] + v85[4]*(links[15].mcm[3]*v85[1] - links[15].m*v85[5]) + (-(links[15].mcm[1]*v85[1]) - links[15].mcm[2]*v85[2])*v85[6] + v85[1]*(-(links[15].mcm[3]*v85[4]) - v85[2]*links[15].inertia[1][1] + v85[1]*links[15].inertia[1][2]) + v85[2]*(-(links[15].mcm[3]*v85[5]) - v85[2]*links[15].inertia[1][2] + v85[1]*links[15].inertia[2][2]) + v85[3]*(links[15].mcm[1]*v85[4] + links[15].mcm[2]*v85[5] - v85[2]*links[15].inertia[1][3] + v85[1]*links[15].inertia[2][3]) - gravity*links[15].mcm[2]*SG850[1][3] + gravity*links[15].mcm[1]*SG850[2][3];

pv86[1]=-(links[17].mcm[1]*Power(v86[2],2)) - links[17].mcm[1]*Power(v86[3],2) + v86[1]*(links[17].mcm[2]*v86[2] + links[17].mcm[3]*v86[3]) - links[17].m*v86[3]*v86[5] + links[17].m*v86[2]*v86[6] + gravity*links[17].m*SG860[1][3];
pv86[2]=-(links[17].mcm[2]*Power(v86[1],2)) - links[17].mcm[2]*Power(v86[3],2) + v86[2]*(links[17].mcm[1]*v86[1] + links[17].mcm[3]*v86[3]) + links[17].m*v86[3]*v86[4] - links[17].m*v86[1]*v86[6] + gravity*links[17].m*SG860[2][3];
pv86[3]=-(links[17].mcm[3]*Power(v86[1],2)) - links[17].mcm[3]*Power(v86[2],2) + (links[17].mcm[1]*v86[1] + links[17].mcm[2]*v86[2])*v86[3] - links[17].m*v86[2]*v86[4] + links[17].m*v86[1]*v86[5] + gravity*links[17].m*SG860[3][3];
pv86[4]=(-(links[17].mcm[2]*v86[2]) - links[17].mcm[3]*v86[3])*v86[4] + (links[17].mcm[1]*v86[3] + links[17].m*v86[5])*v86[6] + v86[5]*(links[17].mcm[1]*v86[2] - links[17].m*v86[6]) + v86[1]*(links[17].mcm[2]*v86[5] + links[17].mcm[3]*v86[6] - v86[3]*links[17].inertia[1][2] + v86[2]*links[17].inertia[1][3]) + v86[2]*(-(links[17].mcm[1]*v86[5]) - v86[3]*links[17].inertia[2][2] + v86[2]*links[17].inertia[2][3]) + v86[3]*(-(links[17].mcm[1]*v86[6]) - v86[3]*links[17].inertia[2][3] + v86[2]*links[17].inertia[3][3]) - gravity*links[17].mcm[3]*SG860[2][3] + gravity*links[17].mcm[2]*SG860[3][3];
pv86[5]=(-(links[17].mcm[1]*v86[1]) - links[17].mcm[3]*v86[3])*v86[5] + (links[17].mcm[2]*v86[3] - links[17].m*v86[4])*v86[6] + v86[4]*(links[17].mcm[2]*v86[1] + links[17].m*v86[6]) + v86[1]*(-(links[17].mcm[2]*v86[4]) + v86[3]*links[17].inertia[1][1] - v86[1]*links[17].inertia[1][3]) + v86[2]*(links[17].mcm[1]*v86[4] + links[17].mcm[3]*v86[6] + v86[3]*links[17].inertia[1][2] - v86[1]*links[17].inertia[2][3]) + v86[3]*(-(links[17].mcm[2]*v86[6]) + v86[3]*links[17].inertia[1][3] - v86[1]*links[17].inertia[3][3]) + gravity*links[17].mcm[3]*SG860[1][3] - gravity*links[17].mcm[1]*SG860[3][3];
pv86[6]=(links[17].mcm[3]*v86[2] + links[17].m*v86[4])*v86[5] + v86[4]*(links[17].mcm[3]*v86[1] - links[17].m*v86[5]) + (-(links[17].mcm[1]*v86[1]) - links[17].mcm[2]*v86[2])*v86[6] + v86[1]*(-(links[17].mcm[3]*v86[4]) - v86[2]*links[17].inertia[1][1] + v86[1]*links[17].inertia[1][2]) + v86[2]*(-(links[17].mcm[3]*v86[5]) - v86[2]*links[17].inertia[1][2] + v86[1]*links[17].inertia[2][2]) + v86[3]*(links[17].mcm[1]*v86[4] + links[17].mcm[2]*v86[5] - v86[2]*links[17].inertia[1][3] + v86[1]*links[17].inertia[2][3]) - gravity*links[17].mcm[2]*SG860[1][3] + gravity*links[17].mcm[1]*SG860[2][3];

pv87[1]=-uex[18].f[1] - links[18].mcm[1]*Power(v87[2],2) - links[18].mcm[1]*Power(v87[3],2) + v87[1]*(links[18].mcm[2]*v87[2] + links[18].mcm[3]*v87[3]) - links[18].m*v87[3]*v87[5] + links[18].m*v87[2]*v87[6] + gravity*links[18].m*SG870[1][3];
pv87[2]=-uex[18].f[2] - links[18].mcm[2]*Power(v87[1],2) - links[18].mcm[2]*Power(v87[3],2) + v87[2]*(links[18].mcm[1]*v87[1] + links[18].mcm[3]*v87[3]) + links[18].m*v87[3]*v87[4] - links[18].m*v87[1]*v87[6] + gravity*links[18].m*SG870[2][3];
pv87[3]=-uex[18].f[3] - links[18].mcm[3]*Power(v87[1],2) - links[18].mcm[3]*Power(v87[2],2) + (links[18].mcm[1]*v87[1] + links[18].mcm[2]*v87[2])*v87[3] - links[18].m*v87[2]*v87[4] + links[18].m*v87[1]*v87[5] + gravity*links[18].m*SG870[3][3];
pv87[4]=-uex[18].t[1] + (-(links[18].mcm[2]*v87[2]) - links[18].mcm[3]*v87[3])*v87[4] + (links[18].mcm[1]*v87[3] + links[18].m*v87[5])*v87[6] + v87[5]*(links[18].mcm[1]*v87[2] - links[18].m*v87[6]) + v87[1]*(links[18].mcm[2]*v87[5] + links[18].mcm[3]*v87[6] - v87[3]*links[18].inertia[1][2] + v87[2]*links[18].inertia[1][3]) + v87[2]*(-(links[18].mcm[1]*v87[5]) - v87[3]*links[18].inertia[2][2] + v87[2]*links[18].inertia[2][3]) + v87[3]*(-(links[18].mcm[1]*v87[6]) - v87[3]*links[18].inertia[2][3] + v87[2]*links[18].inertia[3][3]) - gravity*links[18].mcm[3]*SG870[2][3] + gravity*links[18].mcm[2]*SG870[3][3];
pv87[5]=-uex[18].t[2] + (-(links[18].mcm[1]*v87[1]) - links[18].mcm[3]*v87[3])*v87[5] + (links[18].mcm[2]*v87[3] - links[18].m*v87[4])*v87[6] + v87[4]*(links[18].mcm[2]*v87[1] + links[18].m*v87[6]) + v87[1]*(-(links[18].mcm[2]*v87[4]) + v87[3]*links[18].inertia[1][1] - v87[1]*links[18].inertia[1][3]) + v87[2]*(links[18].mcm[1]*v87[4] + links[18].mcm[3]*v87[6] + v87[3]*links[18].inertia[1][2] - v87[1]*links[18].inertia[2][3]) + v87[3]*(-(links[18].mcm[2]*v87[6]) + v87[3]*links[18].inertia[1][3] - v87[1]*links[18].inertia[3][3]) + gravity*links[18].mcm[3]*SG870[1][3] - gravity*links[18].mcm[1]*SG870[3][3];
pv87[6]=-uex[18].t[3] + (links[18].mcm[3]*v87[2] + links[18].m*v87[4])*v87[5] + v87[4]*(links[18].mcm[3]*v87[1] - links[18].m*v87[5]) + (-(links[18].mcm[1]*v87[1]) - links[18].mcm[2]*v87[2])*v87[6] + v87[1]*(-(links[18].mcm[3]*v87[4]) - v87[2]*links[18].inertia[1][1] + v87[1]*links[18].inertia[1][2]) + v87[2]*(-(links[18].mcm[3]*v87[5]) - v87[2]*links[18].inertia[1][2] + v87[1]*links[18].inertia[2][2]) + v87[3]*(links[18].mcm[1]*v87[4] + links[18].mcm[2]*v87[5] - v87[2]*links[18].inertia[1][3] + v87[1]*links[18].inertia[2][3]) - gravity*links[18].mcm[2]*SG870[1][3] + gravity*links[18].mcm[1]*SG870[2][3];

pv88[1]=-(links[19].mcm[1]*Power(v88[2],2)) - links[19].mcm[1]*Power(v88[3],2) + v88[1]*(links[19].mcm[2]*v88[2] + links[19].mcm[3]*v88[3]) - links[19].m*v88[3]*v88[5] + links[19].m*v88[2]*v88[6] + gravity*links[19].m*SG880[1][3];
pv88[2]=-(links[19].mcm[2]*Power(v88[1],2)) - links[19].mcm[2]*Power(v88[3],2) + v88[2]*(links[19].mcm[1]*v88[1] + links[19].mcm[3]*v88[3]) + links[19].m*v88[3]*v88[4] - links[19].m*v88[1]*v88[6] + gravity*links[19].m*SG880[2][3];
pv88[3]=-(links[19].mcm[3]*Power(v88[1],2)) - links[19].mcm[3]*Power(v88[2],2) + (links[19].mcm[1]*v88[1] + links[19].mcm[2]*v88[2])*v88[3] - links[19].m*v88[2]*v88[4] + links[19].m*v88[1]*v88[5] + gravity*links[19].m*SG880[3][3];
pv88[4]=(-(links[19].mcm[2]*v88[2]) - links[19].mcm[3]*v88[3])*v88[4] + (links[19].mcm[1]*v88[3] + links[19].m*v88[5])*v88[6] + v88[5]*(links[19].mcm[1]*v88[2] - links[19].m*v88[6]) + v88[1]*(links[19].mcm[2]*v88[5] + links[19].mcm[3]*v88[6] - v88[3]*links[19].inertia[1][2] + v88[2]*links[19].inertia[1][3]) + v88[2]*(-(links[19].mcm[1]*v88[5]) - v88[3]*links[19].inertia[2][2] + v88[2]*links[19].inertia[2][3]) + v88[3]*(-(links[19].mcm[1]*v88[6]) - v88[3]*links[19].inertia[2][3] + v88[2]*links[19].inertia[3][3]) - gravity*links[19].mcm[3]*SG880[2][3] + gravity*links[19].mcm[2]*SG880[3][3];
pv88[5]=(-(links[19].mcm[1]*v88[1]) - links[19].mcm[3]*v88[3])*v88[5] + (links[19].mcm[2]*v88[3] - links[19].m*v88[4])*v88[6] + v88[4]*(links[19].mcm[2]*v88[1] + links[19].m*v88[6]) + v88[1]*(-(links[19].mcm[2]*v88[4]) + v88[3]*links[19].inertia[1][1] - v88[1]*links[19].inertia[1][3]) + v88[2]*(links[19].mcm[1]*v88[4] + links[19].mcm[3]*v88[6] + v88[3]*links[19].inertia[1][2] - v88[1]*links[19].inertia[2][3]) + v88[3]*(-(links[19].mcm[2]*v88[6]) + v88[3]*links[19].inertia[1][3] - v88[1]*links[19].inertia[3][3]) + gravity*links[19].mcm[3]*SG880[1][3] - gravity*links[19].mcm[1]*SG880[3][3];
pv88[6]=(links[19].mcm[3]*v88[2] + links[19].m*v88[4])*v88[5] + v88[4]*(links[19].mcm[3]*v88[1] - links[19].m*v88[5]) + (-(links[19].mcm[1]*v88[1]) - links[19].mcm[2]*v88[2])*v88[6] + v88[1]*(-(links[19].mcm[3]*v88[4]) - v88[2]*links[19].inertia[1][1] + v88[1]*links[19].inertia[1][2]) + v88[2]*(-(links[19].mcm[3]*v88[5]) - v88[2]*links[19].inertia[1][2] + v88[1]*links[19].inertia[2][2]) + v88[3]*(links[19].mcm[1]*v88[4] + links[19].mcm[2]*v88[5] - v88[2]*links[19].inertia[1][3] + v88[1]*links[19].inertia[2][3]) - gravity*links[19].mcm[2]*SG880[1][3] + gravity*links[19].mcm[1]*SG880[2][3];

pv89[1]=-uex[20].f[1] - links[20].mcm[1]*Power(v89[2],2) - links[20].mcm[1]*Power(v89[3],2) + v89[1]*(links[20].mcm[2]*v89[2] + links[20].mcm[3]*v89[3]) - links[20].m*v89[3]*v89[5] + links[20].m*v89[2]*v89[6] + gravity*links[20].m*SG890[1][3];
pv89[2]=-uex[20].f[2] - links[20].mcm[2]*Power(v89[1],2) - links[20].mcm[2]*Power(v89[3],2) + v89[2]*(links[20].mcm[1]*v89[1] + links[20].mcm[3]*v89[3]) + links[20].m*v89[3]*v89[4] - links[20].m*v89[1]*v89[6] + gravity*links[20].m*SG890[2][3];
pv89[3]=-uex[20].f[3] - links[20].mcm[3]*Power(v89[1],2) - links[20].mcm[3]*Power(v89[2],2) + (links[20].mcm[1]*v89[1] + links[20].mcm[2]*v89[2])*v89[3] - links[20].m*v89[2]*v89[4] + links[20].m*v89[1]*v89[5] + gravity*links[20].m*SG890[3][3];
pv89[4]=-uex[20].t[1] + (-(links[20].mcm[2]*v89[2]) - links[20].mcm[3]*v89[3])*v89[4] + (links[20].mcm[1]*v89[3] + links[20].m*v89[5])*v89[6] + v89[5]*(links[20].mcm[1]*v89[2] - links[20].m*v89[6]) + v89[1]*(links[20].mcm[2]*v89[5] + links[20].mcm[3]*v89[6] - v89[3]*links[20].inertia[1][2] + v89[2]*links[20].inertia[1][3]) + v89[2]*(-(links[20].mcm[1]*v89[5]) - v89[3]*links[20].inertia[2][2] + v89[2]*links[20].inertia[2][3]) + v89[3]*(-(links[20].mcm[1]*v89[6]) - v89[3]*links[20].inertia[2][3] + v89[2]*links[20].inertia[3][3]) - gravity*links[20].mcm[3]*SG890[2][3] + gravity*links[20].mcm[2]*SG890[3][3];
pv89[5]=-uex[20].t[2] + (-(links[20].mcm[1]*v89[1]) - links[20].mcm[3]*v89[3])*v89[5] + (links[20].mcm[2]*v89[3] - links[20].m*v89[4])*v89[6] + v89[4]*(links[20].mcm[2]*v89[1] + links[20].m*v89[6]) + v89[1]*(-(links[20].mcm[2]*v89[4]) + v89[3]*links[20].inertia[1][1] - v89[1]*links[20].inertia[1][3]) + v89[2]*(links[20].mcm[1]*v89[4] + links[20].mcm[3]*v89[6] + v89[3]*links[20].inertia[1][2] - v89[1]*links[20].inertia[2][3]) + v89[3]*(-(links[20].mcm[2]*v89[6]) + v89[3]*links[20].inertia[1][3] - v89[1]*links[20].inertia[3][3]) + gravity*links[20].mcm[3]*SG890[1][3] - gravity*links[20].mcm[1]*SG890[3][3];
pv89[6]=-uex[20].t[3] + (links[20].mcm[3]*v89[2] + links[20].m*v89[4])*v89[5] + v89[4]*(links[20].mcm[3]*v89[1] - links[20].m*v89[5]) + (-(links[20].mcm[1]*v89[1]) - links[20].mcm[2]*v89[2])*v89[6] + v89[1]*(-(links[20].mcm[3]*v89[4]) - v89[2]*links[20].inertia[1][1] + v89[1]*links[20].inertia[1][2]) + v89[2]*(-(links[20].mcm[3]*v89[5]) - v89[2]*links[20].inertia[1][2] + v89[1]*links[20].inertia[2][2]) + v89[3]*(links[20].mcm[1]*v89[4] + links[20].mcm[2]*v89[5] - v89[2]*links[20].inertia[1][3] + v89[1]*links[20].inertia[2][3]) - gravity*links[20].mcm[2]*SG890[1][3] + gravity*links[20].mcm[1]*SG890[2][3];

pv90[1]=-uex[21].f[1] - links[21].mcm[1]*Power(v90[2],2) - links[21].mcm[1]*Power(v90[3],2) + v90[1]*(links[21].mcm[2]*v90[2] + links[21].mcm[3]*v90[3]) - links[21].m*v90[3]*v90[5] + links[21].m*v90[2]*v90[6] + gravity*links[21].m*SG900[1][3];
pv90[2]=-uex[21].f[2] - links[21].mcm[2]*Power(v90[1],2) - links[21].mcm[2]*Power(v90[3],2) + v90[2]*(links[21].mcm[1]*v90[1] + links[21].mcm[3]*v90[3]) + links[21].m*v90[3]*v90[4] - links[21].m*v90[1]*v90[6] + gravity*links[21].m*SG900[2][3];
pv90[3]=-uex[21].f[3] - links[21].mcm[3]*Power(v90[1],2) - links[21].mcm[3]*Power(v90[2],2) + (links[21].mcm[1]*v90[1] + links[21].mcm[2]*v90[2])*v90[3] - links[21].m*v90[2]*v90[4] + links[21].m*v90[1]*v90[5] + gravity*links[21].m*SG900[3][3];
pv90[4]=-uex[21].t[1] + (-(links[21].mcm[2]*v90[2]) - links[21].mcm[3]*v90[3])*v90[4] + (links[21].mcm[1]*v90[3] + links[21].m*v90[5])*v90[6] + v90[5]*(links[21].mcm[1]*v90[2] - links[21].m*v90[6]) + v90[1]*(links[21].mcm[2]*v90[5] + links[21].mcm[3]*v90[6] - v90[3]*links[21].inertia[1][2] + v90[2]*links[21].inertia[1][3]) + v90[2]*(-(links[21].mcm[1]*v90[5]) - v90[3]*links[21].inertia[2][2] + v90[2]*links[21].inertia[2][3]) + v90[3]*(-(links[21].mcm[1]*v90[6]) - v90[3]*links[21].inertia[2][3] + v90[2]*links[21].inertia[3][3]) - gravity*links[21].mcm[3]*SG900[2][3] + gravity*links[21].mcm[2]*SG900[3][3];
pv90[5]=-uex[21].t[2] + (-(links[21].mcm[1]*v90[1]) - links[21].mcm[3]*v90[3])*v90[5] + (links[21].mcm[2]*v90[3] - links[21].m*v90[4])*v90[6] + v90[4]*(links[21].mcm[2]*v90[1] + links[21].m*v90[6]) + v90[1]*(-(links[21].mcm[2]*v90[4]) + v90[3]*links[21].inertia[1][1] - v90[1]*links[21].inertia[1][3]) + v90[2]*(links[21].mcm[1]*v90[4] + links[21].mcm[3]*v90[6] + v90[3]*links[21].inertia[1][2] - v90[1]*links[21].inertia[2][3]) + v90[3]*(-(links[21].mcm[2]*v90[6]) + v90[3]*links[21].inertia[1][3] - v90[1]*links[21].inertia[3][3]) + gravity*links[21].mcm[3]*SG900[1][3] - gravity*links[21].mcm[1]*SG900[3][3];
pv90[6]=-uex[21].t[3] + (links[21].mcm[3]*v90[2] + links[21].m*v90[4])*v90[5] + v90[4]*(links[21].mcm[3]*v90[1] - links[21].m*v90[5]) + (-(links[21].mcm[1]*v90[1]) - links[21].mcm[2]*v90[2])*v90[6] + v90[1]*(-(links[21].mcm[3]*v90[4]) - v90[2]*links[21].inertia[1][1] + v90[1]*links[21].inertia[1][2]) + v90[2]*(-(links[21].mcm[3]*v90[5]) - v90[2]*links[21].inertia[1][2] + v90[1]*links[21].inertia[2][2]) + v90[3]*(links[21].mcm[1]*v90[4] + links[21].mcm[2]*v90[5] - v90[2]*links[21].inertia[1][3] + v90[1]*links[21].inertia[2][3]) - gravity*links[21].mcm[2]*SG900[1][3] + gravity*links[21].mcm[1]*SG900[2][3];

pv97[1]=-(eff[4].mcm[1]*Power(v97[2],2)) - eff[4].mcm[1]*Power(v97[3],2) + v97[1]*(eff[4].mcm[2]*v97[2] + eff[4].mcm[3]*v97[3]) - eff[4].m*v97[3]*v97[5] + eff[4].m*v97[2]*v97[6] + eff[4].m*gravity*SG970[1][3];
pv97[2]=-(eff[4].mcm[2]*Power(v97[1],2)) - eff[4].mcm[2]*Power(v97[3],2) + v97[2]*(eff[4].mcm[1]*v97[1] + eff[4].mcm[3]*v97[3]) + eff[4].m*v97[3]*v97[4] - eff[4].m*v97[1]*v97[6] + eff[4].m*gravity*SG970[2][3];
pv97[3]=-(eff[4].mcm[3]*Power(v97[1],2)) - eff[4].mcm[3]*Power(v97[2],2) + (eff[4].mcm[1]*v97[1] + eff[4].mcm[2]*v97[2])*v97[3] - eff[4].m*v97[2]*v97[4] + eff[4].m*v97[1]*v97[5] + eff[4].m*gravity*SG970[3][3];
pv97[4]=(-(eff[4].mcm[2]*v97[2]) - eff[4].mcm[3]*v97[3])*v97[4] - eff[4].mcm[1]*v97[2]*v97[5] - eff[4].mcm[1]*v97[3]*v97[6] + (eff[4].mcm[1]*v97[3] + eff[4].m*v97[5])*v97[6] + v97[5]*(eff[4].mcm[1]*v97[2] - eff[4].m*v97[6]) + v97[1]*(eff[4].mcm[2]*v97[5] + eff[4].mcm[3]*v97[6]) - gravity*eff[4].mcm[3]*SG970[2][3] + gravity*eff[4].mcm[2]*SG970[3][3];
pv97[5]=-(eff[4].mcm[2]*v97[1]*v97[4]) + (-(eff[4].mcm[1]*v97[1]) - eff[4].mcm[3]*v97[3])*v97[5] - eff[4].mcm[2]*v97[3]*v97[6] + (eff[4].mcm[2]*v97[3] - eff[4].m*v97[4])*v97[6] + v97[4]*(eff[4].mcm[2]*v97[1] + eff[4].m*v97[6]) + v97[2]*(eff[4].mcm[1]*v97[4] + eff[4].mcm[3]*v97[6]) + gravity*eff[4].mcm[3]*SG970[1][3] - gravity*eff[4].mcm[1]*SG970[3][3];
pv97[6]=-(eff[4].mcm[3]*v97[1]*v97[4]) - eff[4].mcm[3]*v97[2]*v97[5] + (eff[4].mcm[3]*v97[2] + eff[4].m*v97[4])*v97[5] + v97[4]*(eff[4].mcm[3]*v97[1] - eff[4].m*v97[5]) + v97[3]*(eff[4].mcm[1]*v97[4] + eff[4].mcm[2]*v97[5]) + (-(eff[4].mcm[1]*v97[1]) - eff[4].mcm[2]*v97[2])*v97[6] - gravity*eff[4].mcm[2]*SG970[1][3] + gravity*eff[4].mcm[1]*SG970[2][3];



}

/* articulated body inertias and misc variables */

void
hermes_InvDynArtfunc7(void)
     {
JA97[1][2]=eff[4].mcm[3];
JA97[1][3]=-eff[4].mcm[2];
JA97[1][4]=eff[4].m;

JA97[2][1]=-eff[4].mcm[3];
JA97[2][3]=eff[4].mcm[1];
JA97[2][5]=eff[4].m;

JA97[3][1]=eff[4].mcm[2];
JA97[3][2]=-eff[4].mcm[1];
JA97[3][6]=eff[4].m;

JA97[4][5]=-eff[4].mcm[3];
JA97[4][6]=eff[4].mcm[2];

JA97[5][4]=eff[4].mcm[3];
JA97[5][6]=-eff[4].mcm[1];

JA97[6][4]=-eff[4].mcm[2];
JA97[6][5]=eff[4].mcm[1];


T19097[1][2]=JA97[1][2];
T19097[1][3]=JA97[1][3];
T19097[1][4]=JA97[1][4];

T19097[2][1]=JA97[2][1];
T19097[2][3]=JA97[2][3];
T19097[2][5]=JA97[2][5];

T19097[3][1]=JA97[3][1];
T19097[3][2]=JA97[3][2];
T19097[3][6]=JA97[3][6];

T19097[4][5]=JA97[4][5];
T19097[4][6]=JA97[4][6];

T19097[5][4]=JA97[5][4];
T19097[5][6]=JA97[5][6];

T19097[6][4]=JA97[6][4];
T19097[6][5]=JA97[6][5];


T9097[1][1]=(-(eff[4].x[3]*S9790[1][2]) + eff[4].x[2]*S9790[1][3])*Si9097[1][1]*T19097[1][4] + S9790[3][1]*(Si9097[1][1]*T19097[1][3] + Si9097[1][2]*T19097[2][3]) + (-(eff[4].x[3]*S9790[2][2]) + eff[4].x[2]*S9790[2][3])*Si9097[1][2]*T19097[2][5] + S9790[1][1]*(Si9097[1][2]*T19097[2][1] + Si9097[1][3]*T19097[3][1]) + S9790[2][1]*(Si9097[1][1]*T19097[1][2] + Si9097[1][3]*T19097[3][2]) + (-(eff[4].x[3]*S9790[3][2]) + eff[4].x[2]*S9790[3][3])*Si9097[1][3]*T19097[3][6];
T9097[1][2]=(eff[4].x[3]*S9790[1][1] - eff[4].x[1]*S9790[1][3])*Si9097[1][1]*T19097[1][4] + S9790[3][2]*(Si9097[1][1]*T19097[1][3] + Si9097[1][2]*T19097[2][3]) + (eff[4].x[3]*S9790[2][1] - eff[4].x[1]*S9790[2][3])*Si9097[1][2]*T19097[2][5] + S9790[1][2]*(Si9097[1][2]*T19097[2][1] + Si9097[1][3]*T19097[3][1]) + S9790[2][2]*(Si9097[1][1]*T19097[1][2] + Si9097[1][3]*T19097[3][2]) + (eff[4].x[3]*S9790[3][1] - eff[4].x[1]*S9790[3][3])*Si9097[1][3]*T19097[3][6];
T9097[1][3]=(-(eff[4].x[2]*S9790[1][1]) + eff[4].x[1]*S9790[1][2])*Si9097[1][1]*T19097[1][4] + S9790[3][3]*(Si9097[1][1]*T19097[1][3] + Si9097[1][2]*T19097[2][3]) + (-(eff[4].x[2]*S9790[2][1]) + eff[4].x[1]*S9790[2][2])*Si9097[1][2]*T19097[2][5] + S9790[1][3]*(Si9097[1][2]*T19097[2][1] + Si9097[1][3]*T19097[3][1]) + S9790[2][3]*(Si9097[1][1]*T19097[1][2] + Si9097[1][3]*T19097[3][2]) + (-(eff[4].x[2]*S9790[3][1]) + eff[4].x[1]*S9790[3][2])*Si9097[1][3]*T19097[3][6];
T9097[1][4]=S9790[1][1]*Si9097[1][1]*T19097[1][4] + S9790[2][1]*Si9097[1][2]*T19097[2][5] + S9790[3][1]*Si9097[1][3]*T19097[3][6];
T9097[1][5]=S9790[1][2]*Si9097[1][1]*T19097[1][4] + S9790[2][2]*Si9097[1][2]*T19097[2][5] + S9790[3][2]*Si9097[1][3]*T19097[3][6];
T9097[1][6]=S9790[1][3]*Si9097[1][1]*T19097[1][4] + S9790[2][3]*Si9097[1][2]*T19097[2][5] + S9790[3][3]*Si9097[1][3]*T19097[3][6];

T9097[2][1]=(-(eff[4].x[3]*S9790[1][2]) + eff[4].x[2]*S9790[1][3])*Si9097[2][1]*T19097[1][4] + S9790[3][1]*(Si9097[2][1]*T19097[1][3] + Si9097[2][2]*T19097[2][3]) + (-(eff[4].x[3]*S9790[2][2]) + eff[4].x[2]*S9790[2][3])*Si9097[2][2]*T19097[2][5] + S9790[1][1]*(Si9097[2][2]*T19097[2][1] + Si9097[2][3]*T19097[3][1]) + S9790[2][1]*(Si9097[2][1]*T19097[1][2] + Si9097[2][3]*T19097[3][2]) + (-(eff[4].x[3]*S9790[3][2]) + eff[4].x[2]*S9790[3][3])*Si9097[2][3]*T19097[3][6];
T9097[2][2]=(eff[4].x[3]*S9790[1][1] - eff[4].x[1]*S9790[1][3])*Si9097[2][1]*T19097[1][4] + S9790[3][2]*(Si9097[2][1]*T19097[1][3] + Si9097[2][2]*T19097[2][3]) + (eff[4].x[3]*S9790[2][1] - eff[4].x[1]*S9790[2][3])*Si9097[2][2]*T19097[2][5] + S9790[1][2]*(Si9097[2][2]*T19097[2][1] + Si9097[2][3]*T19097[3][1]) + S9790[2][2]*(Si9097[2][1]*T19097[1][2] + Si9097[2][3]*T19097[3][2]) + (eff[4].x[3]*S9790[3][1] - eff[4].x[1]*S9790[3][3])*Si9097[2][3]*T19097[3][6];
T9097[2][3]=(-(eff[4].x[2]*S9790[1][1]) + eff[4].x[1]*S9790[1][2])*Si9097[2][1]*T19097[1][4] + S9790[3][3]*(Si9097[2][1]*T19097[1][3] + Si9097[2][2]*T19097[2][3]) + (-(eff[4].x[2]*S9790[2][1]) + eff[4].x[1]*S9790[2][2])*Si9097[2][2]*T19097[2][5] + S9790[1][3]*(Si9097[2][2]*T19097[2][1] + Si9097[2][3]*T19097[3][1]) + S9790[2][3]*(Si9097[2][1]*T19097[1][2] + Si9097[2][3]*T19097[3][2]) + (-(eff[4].x[2]*S9790[3][1]) + eff[4].x[1]*S9790[3][2])*Si9097[2][3]*T19097[3][6];
T9097[2][4]=S9790[1][1]*Si9097[2][1]*T19097[1][4] + S9790[2][1]*Si9097[2][2]*T19097[2][5] + S9790[3][1]*Si9097[2][3]*T19097[3][6];
T9097[2][5]=S9790[1][2]*Si9097[2][1]*T19097[1][4] + S9790[2][2]*Si9097[2][2]*T19097[2][5] + S9790[3][2]*Si9097[2][3]*T19097[3][6];
T9097[2][6]=S9790[1][3]*Si9097[2][1]*T19097[1][4] + S9790[2][3]*Si9097[2][2]*T19097[2][5] + S9790[3][3]*Si9097[2][3]*T19097[3][6];

T9097[3][1]=(-(eff[4].x[3]*S9790[1][2]) + eff[4].x[2]*S9790[1][3])*Si9097[3][1]*T19097[1][4] + S9790[3][1]*(Si9097[3][1]*T19097[1][3] + Si9097[3][2]*T19097[2][3]) + (-(eff[4].x[3]*S9790[2][2]) + eff[4].x[2]*S9790[2][3])*Si9097[3][2]*T19097[2][5] + S9790[1][1]*(Si9097[3][2]*T19097[2][1] + Si9097[3][3]*T19097[3][1]) + S9790[2][1]*(Si9097[3][1]*T19097[1][2] + Si9097[3][3]*T19097[3][2]) + (-(eff[4].x[3]*S9790[3][2]) + eff[4].x[2]*S9790[3][3])*Si9097[3][3]*T19097[3][6];
T9097[3][2]=(eff[4].x[3]*S9790[1][1] - eff[4].x[1]*S9790[1][3])*Si9097[3][1]*T19097[1][4] + S9790[3][2]*(Si9097[3][1]*T19097[1][3] + Si9097[3][2]*T19097[2][3]) + (eff[4].x[3]*S9790[2][1] - eff[4].x[1]*S9790[2][3])*Si9097[3][2]*T19097[2][5] + S9790[1][2]*(Si9097[3][2]*T19097[2][1] + Si9097[3][3]*T19097[3][1]) + S9790[2][2]*(Si9097[3][1]*T19097[1][2] + Si9097[3][3]*T19097[3][2]) + (eff[4].x[3]*S9790[3][1] - eff[4].x[1]*S9790[3][3])*Si9097[3][3]*T19097[3][6];
T9097[3][3]=(-(eff[4].x[2]*S9790[1][1]) + eff[4].x[1]*S9790[1][2])*Si9097[3][1]*T19097[1][4] + S9790[3][3]*(Si9097[3][1]*T19097[1][3] + Si9097[3][2]*T19097[2][3]) + (-(eff[4].x[2]*S9790[2][1]) + eff[4].x[1]*S9790[2][2])*Si9097[3][2]*T19097[2][5] + S9790[1][3]*(Si9097[3][2]*T19097[2][1] + Si9097[3][3]*T19097[3][1]) + S9790[2][3]*(Si9097[3][1]*T19097[1][2] + Si9097[3][3]*T19097[3][2]) + (-(eff[4].x[2]*S9790[3][1]) + eff[4].x[1]*S9790[3][2])*Si9097[3][3]*T19097[3][6];
T9097[3][4]=S9790[1][1]*Si9097[3][1]*T19097[1][4] + S9790[2][1]*Si9097[3][2]*T19097[2][5] + S9790[3][1]*Si9097[3][3]*T19097[3][6];
T9097[3][5]=S9790[1][2]*Si9097[3][1]*T19097[1][4] + S9790[2][2]*Si9097[3][2]*T19097[2][5] + S9790[3][2]*Si9097[3][3]*T19097[3][6];
T9097[3][6]=S9790[1][3]*Si9097[3][1]*T19097[1][4] + S9790[2][3]*Si9097[3][2]*T19097[2][5] + S9790[3][3]*Si9097[3][3]*T19097[3][6];

T9097[4][1]=S9790[3][1]*((-(eff[4].x[3]*Si9097[2][1]) + eff[4].x[2]*Si9097[3][1])*T19097[1][3] + (-(eff[4].x[3]*Si9097[2][2]) + eff[4].x[2]*Si9097[3][2])*T19097[2][3]) + S9790[1][1]*((-(eff[4].x[3]*Si9097[2][2]) + eff[4].x[2]*Si9097[3][2])*T19097[2][1] + (-(eff[4].x[3]*Si9097[2][3]) + eff[4].x[2]*Si9097[3][3])*T19097[3][1]) + S9790[2][1]*((-(eff[4].x[3]*Si9097[2][1]) + eff[4].x[2]*Si9097[3][1])*T19097[1][2] + (-(eff[4].x[3]*Si9097[2][3]) + eff[4].x[2]*Si9097[3][3])*T19097[3][2]) + (-(eff[4].x[3]*S9790[3][2]) + eff[4].x[2]*S9790[3][3])*((-(eff[4].x[3]*Si9097[2][3]) + eff[4].x[2]*Si9097[3][3])*T19097[3][6] + Si9097[1][1]*T19097[4][6] + Si9097[1][2]*T19097[5][6]) + (-(eff[4].x[3]*S9790[1][2]) + eff[4].x[2]*S9790[1][3])*((-(eff[4].x[3]*Si9097[2][1]) + eff[4].x[2]*Si9097[3][1])*T19097[1][4] + Si9097[1][2]*T19097[5][4] + Si9097[1][3]*T19097[6][4]) + (-(eff[4].x[3]*S9790[2][2]) + eff[4].x[2]*S9790[2][3])*((-(eff[4].x[3]*Si9097[2][2]) + eff[4].x[2]*Si9097[3][2])*T19097[2][5] + Si9097[1][1]*T19097[4][5] + Si9097[1][3]*T19097[6][5]);
T9097[4][2]=S9790[3][2]*((-(eff[4].x[3]*Si9097[2][1]) + eff[4].x[2]*Si9097[3][1])*T19097[1][3] + (-(eff[4].x[3]*Si9097[2][2]) + eff[4].x[2]*Si9097[3][2])*T19097[2][3]) + S9790[1][2]*((-(eff[4].x[3]*Si9097[2][2]) + eff[4].x[2]*Si9097[3][2])*T19097[2][1] + (-(eff[4].x[3]*Si9097[2][3]) + eff[4].x[2]*Si9097[3][3])*T19097[3][1]) + S9790[2][2]*((-(eff[4].x[3]*Si9097[2][1]) + eff[4].x[2]*Si9097[3][1])*T19097[1][2] + (-(eff[4].x[3]*Si9097[2][3]) + eff[4].x[2]*Si9097[3][3])*T19097[3][2]) + (eff[4].x[3]*S9790[3][1] - eff[4].x[1]*S9790[3][3])*((-(eff[4].x[3]*Si9097[2][3]) + eff[4].x[2]*Si9097[3][3])*T19097[3][6] + Si9097[1][1]*T19097[4][6] + Si9097[1][2]*T19097[5][6]) + (eff[4].x[3]*S9790[1][1] - eff[4].x[1]*S9790[1][3])*((-(eff[4].x[3]*Si9097[2][1]) + eff[4].x[2]*Si9097[3][1])*T19097[1][4] + Si9097[1][2]*T19097[5][4] + Si9097[1][3]*T19097[6][4]) + (eff[4].x[3]*S9790[2][1] - eff[4].x[1]*S9790[2][3])*((-(eff[4].x[3]*Si9097[2][2]) + eff[4].x[2]*Si9097[3][2])*T19097[2][5] + Si9097[1][1]*T19097[4][5] + Si9097[1][3]*T19097[6][5]);
T9097[4][3]=S9790[3][3]*((-(eff[4].x[3]*Si9097[2][1]) + eff[4].x[2]*Si9097[3][1])*T19097[1][3] + (-(eff[4].x[3]*Si9097[2][2]) + eff[4].x[2]*Si9097[3][2])*T19097[2][3]) + S9790[1][3]*((-(eff[4].x[3]*Si9097[2][2]) + eff[4].x[2]*Si9097[3][2])*T19097[2][1] + (-(eff[4].x[3]*Si9097[2][3]) + eff[4].x[2]*Si9097[3][3])*T19097[3][1]) + S9790[2][3]*((-(eff[4].x[3]*Si9097[2][1]) + eff[4].x[2]*Si9097[3][1])*T19097[1][2] + (-(eff[4].x[3]*Si9097[2][3]) + eff[4].x[2]*Si9097[3][3])*T19097[3][2]) + (-(eff[4].x[2]*S9790[3][1]) + eff[4].x[1]*S9790[3][2])*((-(eff[4].x[3]*Si9097[2][3]) + eff[4].x[2]*Si9097[3][3])*T19097[3][6] + Si9097[1][1]*T19097[4][6] + Si9097[1][2]*T19097[5][6]) + (-(eff[4].x[2]*S9790[1][1]) + eff[4].x[1]*S9790[1][2])*((-(eff[4].x[3]*Si9097[2][1]) + eff[4].x[2]*Si9097[3][1])*T19097[1][4] + Si9097[1][2]*T19097[5][4] + Si9097[1][3]*T19097[6][4]) + (-(eff[4].x[2]*S9790[2][1]) + eff[4].x[1]*S9790[2][2])*((-(eff[4].x[3]*Si9097[2][2]) + eff[4].x[2]*Si9097[3][2])*T19097[2][5] + Si9097[1][1]*T19097[4][5] + Si9097[1][3]*T19097[6][5]);
T9097[4][4]=S9790[3][1]*((-(eff[4].x[3]*Si9097[2][3]) + eff[4].x[2]*Si9097[3][3])*T19097[3][6] + Si9097[1][1]*T19097[4][6] + Si9097[1][2]*T19097[5][6]) + S9790[1][1]*((-(eff[4].x[3]*Si9097[2][1]) + eff[4].x[2]*Si9097[3][1])*T19097[1][4] + Si9097[1][2]*T19097[5][4] + Si9097[1][3]*T19097[6][4]) + S9790[2][1]*((-(eff[4].x[3]*Si9097[2][2]) + eff[4].x[2]*Si9097[3][2])*T19097[2][5] + Si9097[1][1]*T19097[4][5] + Si9097[1][3]*T19097[6][5]);
T9097[4][5]=S9790[3][2]*((-(eff[4].x[3]*Si9097[2][3]) + eff[4].x[2]*Si9097[3][3])*T19097[3][6] + Si9097[1][1]*T19097[4][6] + Si9097[1][2]*T19097[5][6]) + S9790[1][2]*((-(eff[4].x[3]*Si9097[2][1]) + eff[4].x[2]*Si9097[3][1])*T19097[1][4] + Si9097[1][2]*T19097[5][4] + Si9097[1][3]*T19097[6][4]) + S9790[2][2]*((-(eff[4].x[3]*Si9097[2][2]) + eff[4].x[2]*Si9097[3][2])*T19097[2][5] + Si9097[1][1]*T19097[4][5] + Si9097[1][3]*T19097[6][5]);
T9097[4][6]=S9790[3][3]*((-(eff[4].x[3]*Si9097[2][3]) + eff[4].x[2]*Si9097[3][3])*T19097[3][6] + Si9097[1][1]*T19097[4][6] + Si9097[1][2]*T19097[5][6]) + S9790[1][3]*((-(eff[4].x[3]*Si9097[2][1]) + eff[4].x[2]*Si9097[3][1])*T19097[1][4] + Si9097[1][2]*T19097[5][4] + Si9097[1][3]*T19097[6][4]) + S9790[2][3]*((-(eff[4].x[3]*Si9097[2][2]) + eff[4].x[2]*Si9097[3][2])*T19097[2][5] + Si9097[1][1]*T19097[4][5] + Si9097[1][3]*T19097[6][5]);

T9097[5][1]=S9790[3][1]*((eff[4].x[3]*Si9097[1][1] - eff[4].x[1]*Si9097[3][1])*T19097[1][3] + (eff[4].x[3]*Si9097[1][2] - eff[4].x[1]*Si9097[3][2])*T19097[2][3]) + S9790[1][1]*((eff[4].x[3]*Si9097[1][2] - eff[4].x[1]*Si9097[3][2])*T19097[2][1] + (eff[4].x[3]*Si9097[1][3] - eff[4].x[1]*Si9097[3][3])*T19097[3][1]) + S9790[2][1]*((eff[4].x[3]*Si9097[1][1] - eff[4].x[1]*Si9097[3][1])*T19097[1][2] + (eff[4].x[3]*Si9097[1][3] - eff[4].x[1]*Si9097[3][3])*T19097[3][2]) + (-(eff[4].x[3]*S9790[3][2]) + eff[4].x[2]*S9790[3][3])*((eff[4].x[3]*Si9097[1][3] - eff[4].x[1]*Si9097[3][3])*T19097[3][6] + Si9097[2][1]*T19097[4][6] + Si9097[2][2]*T19097[5][6]) + (-(eff[4].x[3]*S9790[1][2]) + eff[4].x[2]*S9790[1][3])*((eff[4].x[3]*Si9097[1][1] - eff[4].x[1]*Si9097[3][1])*T19097[1][4] + Si9097[2][2]*T19097[5][4] + Si9097[2][3]*T19097[6][4]) + (-(eff[4].x[3]*S9790[2][2]) + eff[4].x[2]*S9790[2][3])*((eff[4].x[3]*Si9097[1][2] - eff[4].x[1]*Si9097[3][2])*T19097[2][5] + Si9097[2][1]*T19097[4][5] + Si9097[2][3]*T19097[6][5]);
T9097[5][2]=S9790[3][2]*((eff[4].x[3]*Si9097[1][1] - eff[4].x[1]*Si9097[3][1])*T19097[1][3] + (eff[4].x[3]*Si9097[1][2] - eff[4].x[1]*Si9097[3][2])*T19097[2][3]) + S9790[1][2]*((eff[4].x[3]*Si9097[1][2] - eff[4].x[1]*Si9097[3][2])*T19097[2][1] + (eff[4].x[3]*Si9097[1][3] - eff[4].x[1]*Si9097[3][3])*T19097[3][1]) + S9790[2][2]*((eff[4].x[3]*Si9097[1][1] - eff[4].x[1]*Si9097[3][1])*T19097[1][2] + (eff[4].x[3]*Si9097[1][3] - eff[4].x[1]*Si9097[3][3])*T19097[3][2]) + (eff[4].x[3]*S9790[3][1] - eff[4].x[1]*S9790[3][3])*((eff[4].x[3]*Si9097[1][3] - eff[4].x[1]*Si9097[3][3])*T19097[3][6] + Si9097[2][1]*T19097[4][6] + Si9097[2][2]*T19097[5][6]) + (eff[4].x[3]*S9790[1][1] - eff[4].x[1]*S9790[1][3])*((eff[4].x[3]*Si9097[1][1] - eff[4].x[1]*Si9097[3][1])*T19097[1][4] + Si9097[2][2]*T19097[5][4] + Si9097[2][3]*T19097[6][4]) + (eff[4].x[3]*S9790[2][1] - eff[4].x[1]*S9790[2][3])*((eff[4].x[3]*Si9097[1][2] - eff[4].x[1]*Si9097[3][2])*T19097[2][5] + Si9097[2][1]*T19097[4][5] + Si9097[2][3]*T19097[6][5]);
T9097[5][3]=S9790[3][3]*((eff[4].x[3]*Si9097[1][1] - eff[4].x[1]*Si9097[3][1])*T19097[1][3] + (eff[4].x[3]*Si9097[1][2] - eff[4].x[1]*Si9097[3][2])*T19097[2][3]) + S9790[1][3]*((eff[4].x[3]*Si9097[1][2] - eff[4].x[1]*Si9097[3][2])*T19097[2][1] + (eff[4].x[3]*Si9097[1][3] - eff[4].x[1]*Si9097[3][3])*T19097[3][1]) + S9790[2][3]*((eff[4].x[3]*Si9097[1][1] - eff[4].x[1]*Si9097[3][1])*T19097[1][2] + (eff[4].x[3]*Si9097[1][3] - eff[4].x[1]*Si9097[3][3])*T19097[3][2]) + (-(eff[4].x[2]*S9790[3][1]) + eff[4].x[1]*S9790[3][2])*((eff[4].x[3]*Si9097[1][3] - eff[4].x[1]*Si9097[3][3])*T19097[3][6] + Si9097[2][1]*T19097[4][6] + Si9097[2][2]*T19097[5][6]) + (-(eff[4].x[2]*S9790[1][1]) + eff[4].x[1]*S9790[1][2])*((eff[4].x[3]*Si9097[1][1] - eff[4].x[1]*Si9097[3][1])*T19097[1][4] + Si9097[2][2]*T19097[5][4] + Si9097[2][3]*T19097[6][4]) + (-(eff[4].x[2]*S9790[2][1]) + eff[4].x[1]*S9790[2][2])*((eff[4].x[3]*Si9097[1][2] - eff[4].x[1]*Si9097[3][2])*T19097[2][5] + Si9097[2][1]*T19097[4][5] + Si9097[2][3]*T19097[6][5]);
T9097[5][4]=S9790[3][1]*((eff[4].x[3]*Si9097[1][3] - eff[4].x[1]*Si9097[3][3])*T19097[3][6] + Si9097[2][1]*T19097[4][6] + Si9097[2][2]*T19097[5][6]) + S9790[1][1]*((eff[4].x[3]*Si9097[1][1] - eff[4].x[1]*Si9097[3][1])*T19097[1][4] + Si9097[2][2]*T19097[5][4] + Si9097[2][3]*T19097[6][4]) + S9790[2][1]*((eff[4].x[3]*Si9097[1][2] - eff[4].x[1]*Si9097[3][2])*T19097[2][5] + Si9097[2][1]*T19097[4][5] + Si9097[2][3]*T19097[6][5]);
T9097[5][5]=S9790[3][2]*((eff[4].x[3]*Si9097[1][3] - eff[4].x[1]*Si9097[3][3])*T19097[3][6] + Si9097[2][1]*T19097[4][6] + Si9097[2][2]*T19097[5][6]) + S9790[1][2]*((eff[4].x[3]*Si9097[1][1] - eff[4].x[1]*Si9097[3][1])*T19097[1][4] + Si9097[2][2]*T19097[5][4] + Si9097[2][3]*T19097[6][4]) + S9790[2][2]*((eff[4].x[3]*Si9097[1][2] - eff[4].x[1]*Si9097[3][2])*T19097[2][5] + Si9097[2][1]*T19097[4][5] + Si9097[2][3]*T19097[6][5]);
T9097[5][6]=S9790[3][3]*((eff[4].x[3]*Si9097[1][3] - eff[4].x[1]*Si9097[3][3])*T19097[3][6] + Si9097[2][1]*T19097[4][6] + Si9097[2][2]*T19097[5][6]) + S9790[1][3]*((eff[4].x[3]*Si9097[1][1] - eff[4].x[1]*Si9097[3][1])*T19097[1][4] + Si9097[2][2]*T19097[5][4] + Si9097[2][3]*T19097[6][4]) + S9790[2][3]*((eff[4].x[3]*Si9097[1][2] - eff[4].x[1]*Si9097[3][2])*T19097[2][5] + Si9097[2][1]*T19097[4][5] + Si9097[2][3]*T19097[6][5]);

T9097[6][1]=S9790[3][1]*((-(eff[4].x[2]*Si9097[1][1]) + eff[4].x[1]*Si9097[2][1])*T19097[1][3] + (-(eff[4].x[2]*Si9097[1][2]) + eff[4].x[1]*Si9097[2][2])*T19097[2][3]) + S9790[1][1]*((-(eff[4].x[2]*Si9097[1][2]) + eff[4].x[1]*Si9097[2][2])*T19097[2][1] + (-(eff[4].x[2]*Si9097[1][3]) + eff[4].x[1]*Si9097[2][3])*T19097[3][1]) + S9790[2][1]*((-(eff[4].x[2]*Si9097[1][1]) + eff[4].x[1]*Si9097[2][1])*T19097[1][2] + (-(eff[4].x[2]*Si9097[1][3]) + eff[4].x[1]*Si9097[2][3])*T19097[3][2]) + (-(eff[4].x[3]*S9790[3][2]) + eff[4].x[2]*S9790[3][3])*((-(eff[4].x[2]*Si9097[1][3]) + eff[4].x[1]*Si9097[2][3])*T19097[3][6] + Si9097[3][1]*T19097[4][6] + Si9097[3][2]*T19097[5][6]) + (-(eff[4].x[3]*S9790[1][2]) + eff[4].x[2]*S9790[1][3])*((-(eff[4].x[2]*Si9097[1][1]) + eff[4].x[1]*Si9097[2][1])*T19097[1][4] + Si9097[3][2]*T19097[5][4] + Si9097[3][3]*T19097[6][4]) + (-(eff[4].x[3]*S9790[2][2]) + eff[4].x[2]*S9790[2][3])*((-(eff[4].x[2]*Si9097[1][2]) + eff[4].x[1]*Si9097[2][2])*T19097[2][5] + Si9097[3][1]*T19097[4][5] + Si9097[3][3]*T19097[6][5]);
T9097[6][2]=S9790[3][2]*((-(eff[4].x[2]*Si9097[1][1]) + eff[4].x[1]*Si9097[2][1])*T19097[1][3] + (-(eff[4].x[2]*Si9097[1][2]) + eff[4].x[1]*Si9097[2][2])*T19097[2][3]) + S9790[1][2]*((-(eff[4].x[2]*Si9097[1][2]) + eff[4].x[1]*Si9097[2][2])*T19097[2][1] + (-(eff[4].x[2]*Si9097[1][3]) + eff[4].x[1]*Si9097[2][3])*T19097[3][1]) + S9790[2][2]*((-(eff[4].x[2]*Si9097[1][1]) + eff[4].x[1]*Si9097[2][1])*T19097[1][2] + (-(eff[4].x[2]*Si9097[1][3]) + eff[4].x[1]*Si9097[2][3])*T19097[3][2]) + (eff[4].x[3]*S9790[3][1] - eff[4].x[1]*S9790[3][3])*((-(eff[4].x[2]*Si9097[1][3]) + eff[4].x[1]*Si9097[2][3])*T19097[3][6] + Si9097[3][1]*T19097[4][6] + Si9097[3][2]*T19097[5][6]) + (eff[4].x[3]*S9790[1][1] - eff[4].x[1]*S9790[1][3])*((-(eff[4].x[2]*Si9097[1][1]) + eff[4].x[1]*Si9097[2][1])*T19097[1][4] + Si9097[3][2]*T19097[5][4] + Si9097[3][3]*T19097[6][4]) + (eff[4].x[3]*S9790[2][1] - eff[4].x[1]*S9790[2][3])*((-(eff[4].x[2]*Si9097[1][2]) + eff[4].x[1]*Si9097[2][2])*T19097[2][5] + Si9097[3][1]*T19097[4][5] + Si9097[3][3]*T19097[6][5]);
T9097[6][3]=S9790[3][3]*((-(eff[4].x[2]*Si9097[1][1]) + eff[4].x[1]*Si9097[2][1])*T19097[1][3] + (-(eff[4].x[2]*Si9097[1][2]) + eff[4].x[1]*Si9097[2][2])*T19097[2][3]) + S9790[1][3]*((-(eff[4].x[2]*Si9097[1][2]) + eff[4].x[1]*Si9097[2][2])*T19097[2][1] + (-(eff[4].x[2]*Si9097[1][3]) + eff[4].x[1]*Si9097[2][3])*T19097[3][1]) + S9790[2][3]*((-(eff[4].x[2]*Si9097[1][1]) + eff[4].x[1]*Si9097[2][1])*T19097[1][2] + (-(eff[4].x[2]*Si9097[1][3]) + eff[4].x[1]*Si9097[2][3])*T19097[3][2]) + (-(eff[4].x[2]*S9790[3][1]) + eff[4].x[1]*S9790[3][2])*((-(eff[4].x[2]*Si9097[1][3]) + eff[4].x[1]*Si9097[2][3])*T19097[3][6] + Si9097[3][1]*T19097[4][6] + Si9097[3][2]*T19097[5][6]) + (-(eff[4].x[2]*S9790[1][1]) + eff[4].x[1]*S9790[1][2])*((-(eff[4].x[2]*Si9097[1][1]) + eff[4].x[1]*Si9097[2][1])*T19097[1][4] + Si9097[3][2]*T19097[5][4] + Si9097[3][3]*T19097[6][4]) + (-(eff[4].x[2]*S9790[2][1]) + eff[4].x[1]*S9790[2][2])*((-(eff[4].x[2]*Si9097[1][2]) + eff[4].x[1]*Si9097[2][2])*T19097[2][5] + Si9097[3][1]*T19097[4][5] + Si9097[3][3]*T19097[6][5]);
T9097[6][4]=S9790[3][1]*((-(eff[4].x[2]*Si9097[1][3]) + eff[4].x[1]*Si9097[2][3])*T19097[3][6] + Si9097[3][1]*T19097[4][6] + Si9097[3][2]*T19097[5][6]) + S9790[1][1]*((-(eff[4].x[2]*Si9097[1][1]) + eff[4].x[1]*Si9097[2][1])*T19097[1][4] + Si9097[3][2]*T19097[5][4] + Si9097[3][3]*T19097[6][4]) + S9790[2][1]*((-(eff[4].x[2]*Si9097[1][2]) + eff[4].x[1]*Si9097[2][2])*T19097[2][5] + Si9097[3][1]*T19097[4][5] + Si9097[3][3]*T19097[6][5]);
T9097[6][5]=S9790[3][2]*((-(eff[4].x[2]*Si9097[1][3]) + eff[4].x[1]*Si9097[2][3])*T19097[3][6] + Si9097[3][1]*T19097[4][6] + Si9097[3][2]*T19097[5][6]) + S9790[1][2]*((-(eff[4].x[2]*Si9097[1][1]) + eff[4].x[1]*Si9097[2][1])*T19097[1][4] + Si9097[3][2]*T19097[5][4] + Si9097[3][3]*T19097[6][4]) + S9790[2][2]*((-(eff[4].x[2]*Si9097[1][2]) + eff[4].x[1]*Si9097[2][2])*T19097[2][5] + Si9097[3][1]*T19097[4][5] + Si9097[3][3]*T19097[6][5]);
T9097[6][6]=S9790[3][3]*((-(eff[4].x[2]*Si9097[1][3]) + eff[4].x[1]*Si9097[2][3])*T19097[3][6] + Si9097[3][1]*T19097[4][6] + Si9097[3][2]*T19097[5][6]) + S9790[1][3]*((-(eff[4].x[2]*Si9097[1][1]) + eff[4].x[1]*Si9097[2][1])*T19097[1][4] + Si9097[3][2]*T19097[5][4] + Si9097[3][3]*T19097[6][4]) + S9790[2][3]*((-(eff[4].x[2]*Si9097[1][2]) + eff[4].x[1]*Si9097[2][2])*T19097[2][5] + Si9097[3][1]*T19097[4][5] + Si9097[3][3]*T19097[6][5]);



}


void
hermes_InvDynArtfunc8(void)
     {




}


void
hermes_InvDynArtfunc9(void)
     {




}


void
hermes_InvDynArtfunc10(void)
      {




}


void
hermes_InvDynArtfunc11(void)
      {




}


void
hermes_InvDynArtfunc12(void)
      {




}


void
hermes_InvDynArtfunc13(void)
      {




}


void
hermes_InvDynArtfunc14(void)
      {
JA90[1][1]=0. + T9097[1][1];
JA90[1][2]=0. + links[21].mcm[3] + T9097[1][2];
JA90[1][3]=0. - links[21].mcm[2] + T9097[1][3];
JA90[1][4]=0. + links[21].m + T9097[1][4];
JA90[1][5]=0. + T9097[1][5];
JA90[1][6]=0. + T9097[1][6];

JA90[2][1]=0. - links[21].mcm[3] + T9097[2][1];
JA90[2][2]=0. + T9097[2][2];
JA90[2][3]=0. + links[21].mcm[1] + T9097[2][3];
JA90[2][4]=0. + T9097[2][4];
JA90[2][5]=0. + links[21].m + T9097[2][5];
JA90[2][6]=0. + T9097[2][6];

JA90[3][1]=0. + links[21].mcm[2] + T9097[3][1];
JA90[3][2]=0. - links[21].mcm[1] + T9097[3][2];
JA90[3][3]=0. + T9097[3][3];
JA90[3][4]=0. + T9097[3][4];
JA90[3][5]=0. + T9097[3][5];
JA90[3][6]=0. + links[21].m + T9097[3][6];

JA90[4][1]=0. + links[21].inertia[1][1] + T9097[4][1];
JA90[4][2]=0. + links[21].inertia[1][2] + T9097[4][2];
JA90[4][3]=0. + links[21].inertia[1][3] + T9097[4][3];
JA90[4][4]=0. + T9097[4][4];
JA90[4][5]=0. - links[21].mcm[3] + T9097[4][5];
JA90[4][6]=0. + links[21].mcm[2] + T9097[4][6];

JA90[5][1]=0. + links[21].inertia[1][2] + T9097[5][1];
JA90[5][2]=0. + links[21].inertia[2][2] + T9097[5][2];
JA90[5][3]=0. + links[21].inertia[2][3] + T9097[5][3];
JA90[5][4]=0. + links[21].mcm[3] + T9097[5][4];
JA90[5][5]=0. + T9097[5][5];
JA90[5][6]=0. - links[21].mcm[1] + T9097[5][6];

JA90[6][1]=0. + links[21].inertia[1][3] + T9097[6][1];
JA90[6][2]=0. + links[21].inertia[2][3] + T9097[6][2];
JA90[6][3]=0. + links[21].inertia[3][3] + T9097[6][3];
JA90[6][4]=0. - links[21].mcm[2] + T9097[6][4];
JA90[6][5]=0. + links[21].mcm[1] + T9097[6][5];
JA90[6][6]=0. + T9097[6][6];


h90[1]=JA90[1][3];
h90[2]=JA90[2][3];
h90[3]=JA90[3][3];
h90[4]=JA90[4][3];
h90[5]=JA90[5][3];
h90[6]=JA90[6][3];

T18990[1][1]=JA90[1][1];
T18990[1][2]=JA90[1][2];
T18990[1][3]=JA90[1][3];
T18990[1][4]=JA90[1][4];
T18990[1][5]=JA90[1][5];
T18990[1][6]=JA90[1][6];

T18990[2][1]=JA90[2][1];
T18990[2][2]=JA90[2][2];
T18990[2][3]=JA90[2][3];
T18990[2][4]=JA90[2][4];
T18990[2][5]=JA90[2][5];
T18990[2][6]=JA90[2][6];

T18990[3][1]=JA90[3][1];
T18990[3][2]=JA90[3][2];
T18990[3][3]=JA90[3][3];
T18990[3][4]=JA90[3][4];
T18990[3][5]=JA90[3][5];
T18990[3][6]=JA90[3][6];

T18990[4][1]=JA90[4][1];
T18990[4][2]=JA90[4][2];
T18990[4][3]=JA90[4][3];
T18990[4][4]=JA90[4][4];
T18990[4][5]=JA90[4][5];
T18990[4][6]=JA90[4][6];

T18990[5][1]=JA90[5][1];
T18990[5][2]=JA90[5][2];
T18990[5][3]=JA90[5][3];
T18990[5][4]=JA90[5][4];
T18990[5][5]=JA90[5][5];
T18990[5][6]=JA90[5][6];

T18990[6][1]=JA90[6][1];
T18990[6][2]=JA90[6][2];
T18990[6][3]=JA90[6][3];
T18990[6][4]=JA90[6][4];
T18990[6][5]=JA90[6][5];
T18990[6][6]=JA90[6][6];


T8990[1][1]=S9089[1][1]*(Si8990[1][1]*T18990[1][1] + Si8990[1][2]*T18990[2][1]) + S9089[2][1]*(Si8990[1][1]*T18990[1][2] + Si8990[1][2]*T18990[2][2]);
T8990[1][2]=-(Si8990[1][1]*T18990[1][3]) - Si8990[1][2]*T18990[2][3];
T8990[1][3]=S9089[1][3]*(Si8990[1][1]*T18990[1][1] + Si8990[1][2]*T18990[2][1]) + S9089[2][3]*(Si8990[1][1]*T18990[1][2] + Si8990[1][2]*T18990[2][2]);
T8990[1][4]=S9089[1][1]*(Si8990[1][1]*T18990[1][4] + Si8990[1][2]*T18990[2][4]) + S9089[2][1]*(Si8990[1][1]*T18990[1][5] + Si8990[1][2]*T18990[2][5]);
T8990[1][5]=-(Si8990[1][1]*T18990[1][6]) - Si8990[1][2]*T18990[2][6];
T8990[1][6]=S9089[1][3]*(Si8990[1][1]*T18990[1][4] + Si8990[1][2]*T18990[2][4]) + S9089[2][3]*(Si8990[1][1]*T18990[1][5] + Si8990[1][2]*T18990[2][5]);

T8990[2][1]=-(S9089[1][1]*T18990[3][1]) - S9089[2][1]*T18990[3][2];
T8990[2][2]=T18990[3][3];
T8990[2][3]=-(S9089[1][3]*T18990[3][1]) - S9089[2][3]*T18990[3][2];
T8990[2][4]=-(S9089[1][1]*T18990[3][4]) - S9089[2][1]*T18990[3][5];
T8990[2][5]=T18990[3][6];
T8990[2][6]=-(S9089[1][3]*T18990[3][4]) - S9089[2][3]*T18990[3][5];

T8990[3][1]=S9089[1][1]*(Si8990[3][1]*T18990[1][1] + Si8990[3][2]*T18990[2][1]) + S9089[2][1]*(Si8990[3][1]*T18990[1][2] + Si8990[3][2]*T18990[2][2]);
T8990[3][2]=-(Si8990[3][1]*T18990[1][3]) - Si8990[3][2]*T18990[2][3];
T8990[3][3]=S9089[1][3]*(Si8990[3][1]*T18990[1][1] + Si8990[3][2]*T18990[2][1]) + S9089[2][3]*(Si8990[3][1]*T18990[1][2] + Si8990[3][2]*T18990[2][2]);
T8990[3][4]=S9089[1][1]*(Si8990[3][1]*T18990[1][4] + Si8990[3][2]*T18990[2][4]) + S9089[2][1]*(Si8990[3][1]*T18990[1][5] + Si8990[3][2]*T18990[2][5]);
T8990[3][5]=-(Si8990[3][1]*T18990[1][6]) - Si8990[3][2]*T18990[2][6];
T8990[3][6]=S9089[1][3]*(Si8990[3][1]*T18990[1][4] + Si8990[3][2]*T18990[2][4]) + S9089[2][3]*(Si8990[3][1]*T18990[1][5] + Si8990[3][2]*T18990[2][5]);

T8990[4][1]=S9089[1][1]*(Si8990[1][1]*T18990[4][1] + Si8990[1][2]*T18990[5][1]) + S9089[2][1]*(Si8990[1][1]*T18990[4][2] + Si8990[1][2]*T18990[5][2]);
T8990[4][2]=-(Si8990[1][1]*T18990[4][3]) - Si8990[1][2]*T18990[5][3];
T8990[4][3]=S9089[1][3]*(Si8990[1][1]*T18990[4][1] + Si8990[1][2]*T18990[5][1]) + S9089[2][3]*(Si8990[1][1]*T18990[4][2] + Si8990[1][2]*T18990[5][2]);
T8990[4][4]=S9089[1][1]*(Si8990[1][1]*T18990[4][4] + Si8990[1][2]*T18990[5][4]) + S9089[2][1]*(Si8990[1][1]*T18990[4][5] + Si8990[1][2]*T18990[5][5]);
T8990[4][5]=-(Si8990[1][1]*T18990[4][6]) - Si8990[1][2]*T18990[5][6];
T8990[4][6]=S9089[1][3]*(Si8990[1][1]*T18990[4][4] + Si8990[1][2]*T18990[5][4]) + S9089[2][3]*(Si8990[1][1]*T18990[4][5] + Si8990[1][2]*T18990[5][5]);

T8990[5][1]=-(S9089[1][1]*T18990[6][1]) - S9089[2][1]*T18990[6][2];
T8990[5][2]=T18990[6][3];
T8990[5][3]=-(S9089[1][3]*T18990[6][1]) - S9089[2][3]*T18990[6][2];
T8990[5][4]=-(S9089[1][1]*T18990[6][4]) - S9089[2][1]*T18990[6][5];
T8990[5][5]=T18990[6][6];
T8990[5][6]=-(S9089[1][3]*T18990[6][4]) - S9089[2][3]*T18990[6][5];

T8990[6][1]=S9089[1][1]*(Si8990[3][1]*T18990[4][1] + Si8990[3][2]*T18990[5][1]) + S9089[2][1]*(Si8990[3][1]*T18990[4][2] + Si8990[3][2]*T18990[5][2]);
T8990[6][2]=-(Si8990[3][1]*T18990[4][3]) - Si8990[3][2]*T18990[5][3];
T8990[6][3]=S9089[1][3]*(Si8990[3][1]*T18990[4][1] + Si8990[3][2]*T18990[5][1]) + S9089[2][3]*(Si8990[3][1]*T18990[4][2] + Si8990[3][2]*T18990[5][2]);
T8990[6][4]=S9089[1][1]*(Si8990[3][1]*T18990[4][4] + Si8990[3][2]*T18990[5][4]) + S9089[2][1]*(Si8990[3][1]*T18990[4][5] + Si8990[3][2]*T18990[5][5]);
T8990[6][5]=-(Si8990[3][1]*T18990[4][6]) - Si8990[3][2]*T18990[5][6];
T8990[6][6]=S9089[1][3]*(Si8990[3][1]*T18990[4][4] + Si8990[3][2]*T18990[5][4]) + S9089[2][3]*(Si8990[3][1]*T18990[4][5] + Si8990[3][2]*T18990[5][5]);



}


void
hermes_InvDynArtfunc15(void)
      {
JA89[1][1]=T8990[1][1];
JA89[1][2]=links[20].mcm[3] + T8990[1][2];
JA89[1][3]=-links[20].mcm[2] + T8990[1][3];
JA89[1][4]=links[20].m + T8990[1][4];
JA89[1][5]=T8990[1][5];
JA89[1][6]=T8990[1][6];

JA89[2][1]=-links[20].mcm[3] + T8990[2][1];
JA89[2][2]=T8990[2][2];
JA89[2][3]=links[20].mcm[1] + T8990[2][3];
JA89[2][4]=T8990[2][4];
JA89[2][5]=links[20].m + T8990[2][5];
JA89[2][6]=T8990[2][6];

JA89[3][1]=links[20].mcm[2] + T8990[3][1];
JA89[3][2]=-links[20].mcm[1] + T8990[3][2];
JA89[3][3]=T8990[3][3];
JA89[3][4]=T8990[3][4];
JA89[3][5]=T8990[3][5];
JA89[3][6]=links[20].m + T8990[3][6];

JA89[4][1]=links[20].inertia[1][1] + T8990[4][1];
JA89[4][2]=links[20].inertia[1][2] + T8990[4][2];
JA89[4][3]=links[20].inertia[1][3] + T8990[4][3];
JA89[4][4]=T8990[4][4];
JA89[4][5]=-links[20].mcm[3] + T8990[4][5];
JA89[4][6]=links[20].mcm[2] + T8990[4][6];

JA89[5][1]=links[20].inertia[1][2] + T8990[5][1];
JA89[5][2]=links[20].inertia[2][2] + T8990[5][2];
JA89[5][3]=links[20].inertia[2][3] + T8990[5][3];
JA89[5][4]=links[20].mcm[3] + T8990[5][4];
JA89[5][5]=T8990[5][5];
JA89[5][6]=-links[20].mcm[1] + T8990[5][6];

JA89[6][1]=links[20].inertia[1][3] + T8990[6][1];
JA89[6][2]=links[20].inertia[2][3] + T8990[6][2];
JA89[6][3]=links[20].inertia[3][3] + T8990[6][3];
JA89[6][4]=-links[20].mcm[2] + T8990[6][4];
JA89[6][5]=links[20].mcm[1] + T8990[6][5];
JA89[6][6]=T8990[6][6];


h89[1]=JA89[1][3];
h89[2]=JA89[2][3];
h89[3]=JA89[3][3];
h89[4]=JA89[4][3];
h89[5]=JA89[5][3];
h89[6]=JA89[6][3];

T18889[1][1]=JA89[1][1];
T18889[1][2]=JA89[1][2];
T18889[1][3]=JA89[1][3];
T18889[1][4]=JA89[1][4];
T18889[1][5]=JA89[1][5];
T18889[1][6]=JA89[1][6];

T18889[2][1]=JA89[2][1];
T18889[2][2]=JA89[2][2];
T18889[2][3]=JA89[2][3];
T18889[2][4]=JA89[2][4];
T18889[2][5]=JA89[2][5];
T18889[2][6]=JA89[2][6];

T18889[3][1]=JA89[3][1];
T18889[3][2]=JA89[3][2];
T18889[3][3]=JA89[3][3];
T18889[3][4]=JA89[3][4];
T18889[3][5]=JA89[3][5];
T18889[3][6]=JA89[3][6];

T18889[4][1]=JA89[4][1];
T18889[4][2]=JA89[4][2];
T18889[4][3]=JA89[4][3];
T18889[4][4]=JA89[4][4];
T18889[4][5]=JA89[4][5];
T18889[4][6]=JA89[4][6];

T18889[5][1]=JA89[5][1];
T18889[5][2]=JA89[5][2];
T18889[5][3]=JA89[5][3];
T18889[5][4]=JA89[5][4];
T18889[5][5]=JA89[5][5];
T18889[5][6]=JA89[5][6];

T18889[6][1]=JA89[6][1];
T18889[6][2]=JA89[6][2];
T18889[6][3]=JA89[6][3];
T18889[6][4]=JA89[6][4];
T18889[6][5]=JA89[6][5];
T18889[6][6]=JA89[6][6];


T8889[1][1]=S8988[1][1]*(Si8889[1][1]*T18889[1][1] + Si8889[1][2]*T18889[2][1]) + S8988[2][1]*(Si8889[1][1]*T18889[1][2] + Si8889[1][2]*T18889[2][2]) + LOWERLEG*(Si8889[1][1]*T18889[1][6] + Si8889[1][2]*T18889[2][6]);
T8889[1][2]=Si8889[1][1]*T18889[1][3] + Si8889[1][2]*T18889[2][3] - LOWERLEG*S8988[1][1]*(Si8889[1][1]*T18889[1][4] + Si8889[1][2]*T18889[2][4]) - LOWERLEG*S8988[2][1]*(Si8889[1][1]*T18889[1][5] + Si8889[1][2]*T18889[2][5]);
T8889[1][3]=S8988[1][3]*(Si8889[1][1]*T18889[1][1] + Si8889[1][2]*T18889[2][1]) + S8988[2][3]*(Si8889[1][1]*T18889[1][2] + Si8889[1][2]*T18889[2][2]);
T8889[1][4]=S8988[1][1]*(Si8889[1][1]*T18889[1][4] + Si8889[1][2]*T18889[2][4]) + S8988[2][1]*(Si8889[1][1]*T18889[1][5] + Si8889[1][2]*T18889[2][5]);
T8889[1][5]=Si8889[1][1]*T18889[1][6] + Si8889[1][2]*T18889[2][6];
T8889[1][6]=S8988[1][3]*(Si8889[1][1]*T18889[1][4] + Si8889[1][2]*T18889[2][4]) + S8988[2][3]*(Si8889[1][1]*T18889[1][5] + Si8889[1][2]*T18889[2][5]);

T8889[2][1]=S8988[1][1]*T18889[3][1] + S8988[2][1]*T18889[3][2] + LOWERLEG*T18889[3][6];
T8889[2][2]=T18889[3][3] - LOWERLEG*S8988[1][1]*T18889[3][4] - LOWERLEG*S8988[2][1]*T18889[3][5];
T8889[2][3]=S8988[1][3]*T18889[3][1] + S8988[2][3]*T18889[3][2];
T8889[2][4]=S8988[1][1]*T18889[3][4] + S8988[2][1]*T18889[3][5];
T8889[2][5]=T18889[3][6];
T8889[2][6]=S8988[1][3]*T18889[3][4] + S8988[2][3]*T18889[3][5];

T8889[3][1]=S8988[1][1]*(Si8889[3][1]*T18889[1][1] + Si8889[3][2]*T18889[2][1]) + S8988[2][1]*(Si8889[3][1]*T18889[1][2] + Si8889[3][2]*T18889[2][2]) + LOWERLEG*(Si8889[3][1]*T18889[1][6] + Si8889[3][2]*T18889[2][6]);
T8889[3][2]=Si8889[3][1]*T18889[1][3] + Si8889[3][2]*T18889[2][3] - LOWERLEG*S8988[1][1]*(Si8889[3][1]*T18889[1][4] + Si8889[3][2]*T18889[2][4]) - LOWERLEG*S8988[2][1]*(Si8889[3][1]*T18889[1][5] + Si8889[3][2]*T18889[2][5]);
T8889[3][3]=S8988[1][3]*(Si8889[3][1]*T18889[1][1] + Si8889[3][2]*T18889[2][1]) + S8988[2][3]*(Si8889[3][1]*T18889[1][2] + Si8889[3][2]*T18889[2][2]);
T8889[3][4]=S8988[1][1]*(Si8889[3][1]*T18889[1][4] + Si8889[3][2]*T18889[2][4]) + S8988[2][1]*(Si8889[3][1]*T18889[1][5] + Si8889[3][2]*T18889[2][5]);
T8889[3][5]=Si8889[3][1]*T18889[1][6] + Si8889[3][2]*T18889[2][6];
T8889[3][6]=S8988[1][3]*(Si8889[3][1]*T18889[1][4] + Si8889[3][2]*T18889[2][4]) + S8988[2][3]*(Si8889[3][1]*T18889[1][5] + Si8889[3][2]*T18889[2][5]);

T8889[4][1]=S8988[1][1]*(LOWERLEG*T18889[3][1] + Si8889[1][1]*T18889[4][1] + Si8889[1][2]*T18889[5][1]) + S8988[2][1]*(LOWERLEG*T18889[3][2] + Si8889[1][1]*T18889[4][2] + Si8889[1][2]*T18889[5][2]) + LOWERLEG*(LOWERLEG*T18889[3][6] + Si8889[1][1]*T18889[4][6] + Si8889[1][2]*T18889[5][6]);
T8889[4][2]=LOWERLEG*T18889[3][3] + Si8889[1][1]*T18889[4][3] + Si8889[1][2]*T18889[5][3] - LOWERLEG*S8988[1][1]*(LOWERLEG*T18889[3][4] + Si8889[1][1]*T18889[4][4] + Si8889[1][2]*T18889[5][4]) - LOWERLEG*S8988[2][1]*(LOWERLEG*T18889[3][5] + Si8889[1][1]*T18889[4][5] + Si8889[1][2]*T18889[5][5]);
T8889[4][3]=S8988[1][3]*(LOWERLEG*T18889[3][1] + Si8889[1][1]*T18889[4][1] + Si8889[1][2]*T18889[5][1]) + S8988[2][3]*(LOWERLEG*T18889[3][2] + Si8889[1][1]*T18889[4][2] + Si8889[1][2]*T18889[5][2]);
T8889[4][4]=S8988[1][1]*(LOWERLEG*T18889[3][4] + Si8889[1][1]*T18889[4][4] + Si8889[1][2]*T18889[5][4]) + S8988[2][1]*(LOWERLEG*T18889[3][5] + Si8889[1][1]*T18889[4][5] + Si8889[1][2]*T18889[5][5]);
T8889[4][5]=LOWERLEG*T18889[3][6] + Si8889[1][1]*T18889[4][6] + Si8889[1][2]*T18889[5][6];
T8889[4][6]=S8988[1][3]*(LOWERLEG*T18889[3][4] + Si8889[1][1]*T18889[4][4] + Si8889[1][2]*T18889[5][4]) + S8988[2][3]*(LOWERLEG*T18889[3][5] + Si8889[1][1]*T18889[4][5] + Si8889[1][2]*T18889[5][5]);

T8889[5][1]=S8988[1][1]*(-(LOWERLEG*Si8889[1][1]*T18889[1][1]) - LOWERLEG*Si8889[1][2]*T18889[2][1] + T18889[6][1]) + S8988[2][1]*(-(LOWERLEG*Si8889[1][1]*T18889[1][2]) - LOWERLEG*Si8889[1][2]*T18889[2][2] + T18889[6][2]) + LOWERLEG*(-(LOWERLEG*Si8889[1][1]*T18889[1][6]) - LOWERLEG*Si8889[1][2]*T18889[2][6] + T18889[6][6]);
T8889[5][2]=-(LOWERLEG*Si8889[1][1]*T18889[1][3]) - LOWERLEG*Si8889[1][2]*T18889[2][3] + T18889[6][3] - LOWERLEG*S8988[1][1]*(-(LOWERLEG*Si8889[1][1]*T18889[1][4]) - LOWERLEG*Si8889[1][2]*T18889[2][4] + T18889[6][4]) - LOWERLEG*S8988[2][1]*(-(LOWERLEG*Si8889[1][1]*T18889[1][5]) - LOWERLEG*Si8889[1][2]*T18889[2][5] + T18889[6][5]);
T8889[5][3]=S8988[1][3]*(-(LOWERLEG*Si8889[1][1]*T18889[1][1]) - LOWERLEG*Si8889[1][2]*T18889[2][1] + T18889[6][1]) + S8988[2][3]*(-(LOWERLEG*Si8889[1][1]*T18889[1][2]) - LOWERLEG*Si8889[1][2]*T18889[2][2] + T18889[6][2]);
T8889[5][4]=S8988[1][1]*(-(LOWERLEG*Si8889[1][1]*T18889[1][4]) - LOWERLEG*Si8889[1][2]*T18889[2][4] + T18889[6][4]) + S8988[2][1]*(-(LOWERLEG*Si8889[1][1]*T18889[1][5]) - LOWERLEG*Si8889[1][2]*T18889[2][5] + T18889[6][5]);
T8889[5][5]=-(LOWERLEG*Si8889[1][1]*T18889[1][6]) - LOWERLEG*Si8889[1][2]*T18889[2][6] + T18889[6][6];
T8889[5][6]=S8988[1][3]*(-(LOWERLEG*Si8889[1][1]*T18889[1][4]) - LOWERLEG*Si8889[1][2]*T18889[2][4] + T18889[6][4]) + S8988[2][3]*(-(LOWERLEG*Si8889[1][1]*T18889[1][5]) - LOWERLEG*Si8889[1][2]*T18889[2][5] + T18889[6][5]);

T8889[6][1]=S8988[1][1]*(Si8889[3][1]*T18889[4][1] + Si8889[3][2]*T18889[5][1]) + S8988[2][1]*(Si8889[3][1]*T18889[4][2] + Si8889[3][2]*T18889[5][2]) + LOWERLEG*(Si8889[3][1]*T18889[4][6] + Si8889[3][2]*T18889[5][6]);
T8889[6][2]=Si8889[3][1]*T18889[4][3] + Si8889[3][2]*T18889[5][3] - LOWERLEG*S8988[1][1]*(Si8889[3][1]*T18889[4][4] + Si8889[3][2]*T18889[5][4]) - LOWERLEG*S8988[2][1]*(Si8889[3][1]*T18889[4][5] + Si8889[3][2]*T18889[5][5]);
T8889[6][3]=S8988[1][3]*(Si8889[3][1]*T18889[4][1] + Si8889[3][2]*T18889[5][1]) + S8988[2][3]*(Si8889[3][1]*T18889[4][2] + Si8889[3][2]*T18889[5][2]);
T8889[6][4]=S8988[1][1]*(Si8889[3][1]*T18889[4][4] + Si8889[3][2]*T18889[5][4]) + S8988[2][1]*(Si8889[3][1]*T18889[4][5] + Si8889[3][2]*T18889[5][5]);
T8889[6][5]=Si8889[3][1]*T18889[4][6] + Si8889[3][2]*T18889[5][6];
T8889[6][6]=S8988[1][3]*(Si8889[3][1]*T18889[4][4] + Si8889[3][2]*T18889[5][4]) + S8988[2][3]*(Si8889[3][1]*T18889[4][5] + Si8889[3][2]*T18889[5][5]);



}


void
hermes_InvDynArtfunc16(void)
      {
JA88[1][1]=T8889[1][1];
JA88[1][2]=links[19].mcm[3] + T8889[1][2];
JA88[1][3]=-links[19].mcm[2] + T8889[1][3];
JA88[1][4]=links[19].m + T8889[1][4];
JA88[1][5]=T8889[1][5];
JA88[1][6]=T8889[1][6];

JA88[2][1]=-links[19].mcm[3] + T8889[2][1];
JA88[2][2]=T8889[2][2];
JA88[2][3]=links[19].mcm[1] + T8889[2][3];
JA88[2][4]=T8889[2][4];
JA88[2][5]=links[19].m + T8889[2][5];
JA88[2][6]=T8889[2][6];

JA88[3][1]=links[19].mcm[2] + T8889[3][1];
JA88[3][2]=-links[19].mcm[1] + T8889[3][2];
JA88[3][3]=T8889[3][3];
JA88[3][4]=T8889[3][4];
JA88[3][5]=T8889[3][5];
JA88[3][6]=links[19].m + T8889[3][6];

JA88[4][1]=links[19].inertia[1][1] + T8889[4][1];
JA88[4][2]=links[19].inertia[1][2] + T8889[4][2];
JA88[4][3]=links[19].inertia[1][3] + T8889[4][3];
JA88[4][4]=T8889[4][4];
JA88[4][5]=-links[19].mcm[3] + T8889[4][5];
JA88[4][6]=links[19].mcm[2] + T8889[4][6];

JA88[5][1]=links[19].inertia[1][2] + T8889[5][1];
JA88[5][2]=links[19].inertia[2][2] + T8889[5][2];
JA88[5][3]=links[19].inertia[2][3] + T8889[5][3];
JA88[5][4]=links[19].mcm[3] + T8889[5][4];
JA88[5][5]=T8889[5][5];
JA88[5][6]=-links[19].mcm[1] + T8889[5][6];

JA88[6][1]=links[19].inertia[1][3] + T8889[6][1];
JA88[6][2]=links[19].inertia[2][3] + T8889[6][2];
JA88[6][3]=links[19].inertia[3][3] + T8889[6][3];
JA88[6][4]=-links[19].mcm[2] + T8889[6][4];
JA88[6][5]=links[19].mcm[1] + T8889[6][5];
JA88[6][6]=T8889[6][6];


h88[1]=JA88[1][3];
h88[2]=JA88[2][3];
h88[3]=JA88[3][3];
h88[4]=JA88[4][3];
h88[5]=JA88[5][3];
h88[6]=JA88[6][3];

T18788[1][1]=JA88[1][1];
T18788[1][2]=JA88[1][2];
T18788[1][3]=JA88[1][3];
T18788[1][4]=JA88[1][4];
T18788[1][5]=JA88[1][5];
T18788[1][6]=JA88[1][6];

T18788[2][1]=JA88[2][1];
T18788[2][2]=JA88[2][2];
T18788[2][3]=JA88[2][3];
T18788[2][4]=JA88[2][4];
T18788[2][5]=JA88[2][5];
T18788[2][6]=JA88[2][6];

T18788[3][1]=JA88[3][1];
T18788[3][2]=JA88[3][2];
T18788[3][3]=JA88[3][3];
T18788[3][4]=JA88[3][4];
T18788[3][5]=JA88[3][5];
T18788[3][6]=JA88[3][6];

T18788[4][1]=JA88[4][1];
T18788[4][2]=JA88[4][2];
T18788[4][3]=JA88[4][3];
T18788[4][4]=JA88[4][4];
T18788[4][5]=JA88[4][5];
T18788[4][6]=JA88[4][6];

T18788[5][1]=JA88[5][1];
T18788[5][2]=JA88[5][2];
T18788[5][3]=JA88[5][3];
T18788[5][4]=JA88[5][4];
T18788[5][5]=JA88[5][5];
T18788[5][6]=JA88[5][6];

T18788[6][1]=JA88[6][1];
T18788[6][2]=JA88[6][2];
T18788[6][3]=JA88[6][3];
T18788[6][4]=JA88[6][4];
T18788[6][5]=JA88[6][5];
T18788[6][6]=JA88[6][6];


T8788[1][1]=S8887[1][1]*(Si8788[1][1]*T18788[1][1] + Si8788[1][2]*T18788[2][1]) + S8887[2][1]*(Si8788[1][1]*T18788[1][2] + Si8788[1][2]*T18788[2][2]);
T8788[1][2]=Si8788[1][1]*T18788[1][3] + Si8788[1][2]*T18788[2][3];
T8788[1][3]=S8887[1][3]*(Si8788[1][1]*T18788[1][1] + Si8788[1][2]*T18788[2][1]) + S8887[2][3]*(Si8788[1][1]*T18788[1][2] + Si8788[1][2]*T18788[2][2]);
T8788[1][4]=S8887[1][1]*(Si8788[1][1]*T18788[1][4] + Si8788[1][2]*T18788[2][4]) + S8887[2][1]*(Si8788[1][1]*T18788[1][5] + Si8788[1][2]*T18788[2][5]);
T8788[1][5]=Si8788[1][1]*T18788[1][6] + Si8788[1][2]*T18788[2][6];
T8788[1][6]=S8887[1][3]*(Si8788[1][1]*T18788[1][4] + Si8788[1][2]*T18788[2][4]) + S8887[2][3]*(Si8788[1][1]*T18788[1][5] + Si8788[1][2]*T18788[2][5]);

T8788[2][1]=S8887[1][1]*T18788[3][1] + S8887[2][1]*T18788[3][2];
T8788[2][2]=T18788[3][3];
T8788[2][3]=S8887[1][3]*T18788[3][1] + S8887[2][3]*T18788[3][2];
T8788[2][4]=S8887[1][1]*T18788[3][4] + S8887[2][1]*T18788[3][5];
T8788[2][5]=T18788[3][6];
T8788[2][6]=S8887[1][3]*T18788[3][4] + S8887[2][3]*T18788[3][5];

T8788[3][1]=S8887[1][1]*(Si8788[3][1]*T18788[1][1] + Si8788[3][2]*T18788[2][1]) + S8887[2][1]*(Si8788[3][1]*T18788[1][2] + Si8788[3][2]*T18788[2][2]);
T8788[3][2]=Si8788[3][1]*T18788[1][3] + Si8788[3][2]*T18788[2][3];
T8788[3][3]=S8887[1][3]*(Si8788[3][1]*T18788[1][1] + Si8788[3][2]*T18788[2][1]) + S8887[2][3]*(Si8788[3][1]*T18788[1][2] + Si8788[3][2]*T18788[2][2]);
T8788[3][4]=S8887[1][1]*(Si8788[3][1]*T18788[1][4] + Si8788[3][2]*T18788[2][4]) + S8887[2][1]*(Si8788[3][1]*T18788[1][5] + Si8788[3][2]*T18788[2][5]);
T8788[3][5]=Si8788[3][1]*T18788[1][6] + Si8788[3][2]*T18788[2][6];
T8788[3][6]=S8887[1][3]*(Si8788[3][1]*T18788[1][4] + Si8788[3][2]*T18788[2][4]) + S8887[2][3]*(Si8788[3][1]*T18788[1][5] + Si8788[3][2]*T18788[2][5]);

T8788[4][1]=S8887[1][1]*(Si8788[1][1]*T18788[4][1] + Si8788[1][2]*T18788[5][1]) + S8887[2][1]*(Si8788[1][1]*T18788[4][2] + Si8788[1][2]*T18788[5][2]);
T8788[4][2]=Si8788[1][1]*T18788[4][3] + Si8788[1][2]*T18788[5][3];
T8788[4][3]=S8887[1][3]*(Si8788[1][1]*T18788[4][1] + Si8788[1][2]*T18788[5][1]) + S8887[2][3]*(Si8788[1][1]*T18788[4][2] + Si8788[1][2]*T18788[5][2]);
T8788[4][4]=S8887[1][1]*(Si8788[1][1]*T18788[4][4] + Si8788[1][2]*T18788[5][4]) + S8887[2][1]*(Si8788[1][1]*T18788[4][5] + Si8788[1][2]*T18788[5][5]);
T8788[4][5]=Si8788[1][1]*T18788[4][6] + Si8788[1][2]*T18788[5][6];
T8788[4][6]=S8887[1][3]*(Si8788[1][1]*T18788[4][4] + Si8788[1][2]*T18788[5][4]) + S8887[2][3]*(Si8788[1][1]*T18788[4][5] + Si8788[1][2]*T18788[5][5]);

T8788[5][1]=S8887[1][1]*T18788[6][1] + S8887[2][1]*T18788[6][2];
T8788[5][2]=T18788[6][3];
T8788[5][3]=S8887[1][3]*T18788[6][1] + S8887[2][3]*T18788[6][2];
T8788[5][4]=S8887[1][1]*T18788[6][4] + S8887[2][1]*T18788[6][5];
T8788[5][5]=T18788[6][6];
T8788[5][6]=S8887[1][3]*T18788[6][4] + S8887[2][3]*T18788[6][5];

T8788[6][1]=S8887[1][1]*(Si8788[3][1]*T18788[4][1] + Si8788[3][2]*T18788[5][1]) + S8887[2][1]*(Si8788[3][1]*T18788[4][2] + Si8788[3][2]*T18788[5][2]);
T8788[6][2]=Si8788[3][1]*T18788[4][3] + Si8788[3][2]*T18788[5][3];
T8788[6][3]=S8887[1][3]*(Si8788[3][1]*T18788[4][1] + Si8788[3][2]*T18788[5][1]) + S8887[2][3]*(Si8788[3][1]*T18788[4][2] + Si8788[3][2]*T18788[5][2]);
T8788[6][4]=S8887[1][1]*(Si8788[3][1]*T18788[4][4] + Si8788[3][2]*T18788[5][4]) + S8887[2][1]*(Si8788[3][1]*T18788[4][5] + Si8788[3][2]*T18788[5][5]);
T8788[6][5]=Si8788[3][1]*T18788[4][6] + Si8788[3][2]*T18788[5][6];
T8788[6][6]=S8887[1][3]*(Si8788[3][1]*T18788[4][4] + Si8788[3][2]*T18788[5][4]) + S8887[2][3]*(Si8788[3][1]*T18788[4][5] + Si8788[3][2]*T18788[5][5]);



}


void
hermes_InvDynArtfunc17(void)
      {
JA87[1][1]=T8788[1][1];
JA87[1][2]=links[18].mcm[3] + T8788[1][2];
JA87[1][3]=-links[18].mcm[2] + T8788[1][3];
JA87[1][4]=links[18].m + T8788[1][4];
JA87[1][5]=T8788[1][5];
JA87[1][6]=T8788[1][6];

JA87[2][1]=-links[18].mcm[3] + T8788[2][1];
JA87[2][2]=T8788[2][2];
JA87[2][3]=links[18].mcm[1] + T8788[2][3];
JA87[2][4]=T8788[2][4];
JA87[2][5]=links[18].m + T8788[2][5];
JA87[2][6]=T8788[2][6];

JA87[3][1]=links[18].mcm[2] + T8788[3][1];
JA87[3][2]=-links[18].mcm[1] + T8788[3][2];
JA87[3][3]=T8788[3][3];
JA87[3][4]=T8788[3][4];
JA87[3][5]=T8788[3][5];
JA87[3][6]=links[18].m + T8788[3][6];

JA87[4][1]=links[18].inertia[1][1] + T8788[4][1];
JA87[4][2]=links[18].inertia[1][2] + T8788[4][2];
JA87[4][3]=links[18].inertia[1][3] + T8788[4][3];
JA87[4][4]=T8788[4][4];
JA87[4][5]=-links[18].mcm[3] + T8788[4][5];
JA87[4][6]=links[18].mcm[2] + T8788[4][6];

JA87[5][1]=links[18].inertia[1][2] + T8788[5][1];
JA87[5][2]=links[18].inertia[2][2] + T8788[5][2];
JA87[5][3]=links[18].inertia[2][3] + T8788[5][3];
JA87[5][4]=links[18].mcm[3] + T8788[5][4];
JA87[5][5]=T8788[5][5];
JA87[5][6]=-links[18].mcm[1] + T8788[5][6];

JA87[6][1]=links[18].inertia[1][3] + T8788[6][1];
JA87[6][2]=links[18].inertia[2][3] + T8788[6][2];
JA87[6][3]=links[18].inertia[3][3] + T8788[6][3];
JA87[6][4]=-links[18].mcm[2] + T8788[6][4];
JA87[6][5]=links[18].mcm[1] + T8788[6][5];
JA87[6][6]=T8788[6][6];


h87[1]=JA87[1][3];
h87[2]=JA87[2][3];
h87[3]=JA87[3][3];
h87[4]=JA87[4][3];
h87[5]=JA87[5][3];
h87[6]=JA87[6][3];

T18687[1][1]=JA87[1][1];
T18687[1][2]=JA87[1][2];
T18687[1][3]=JA87[1][3];
T18687[1][4]=JA87[1][4];
T18687[1][5]=JA87[1][5];
T18687[1][6]=JA87[1][6];

T18687[2][1]=JA87[2][1];
T18687[2][2]=JA87[2][2];
T18687[2][3]=JA87[2][3];
T18687[2][4]=JA87[2][4];
T18687[2][5]=JA87[2][5];
T18687[2][6]=JA87[2][6];

T18687[3][1]=JA87[3][1];
T18687[3][2]=JA87[3][2];
T18687[3][3]=JA87[3][3];
T18687[3][4]=JA87[3][4];
T18687[3][5]=JA87[3][5];
T18687[3][6]=JA87[3][6];

T18687[4][1]=JA87[4][1];
T18687[4][2]=JA87[4][2];
T18687[4][3]=JA87[4][3];
T18687[4][4]=JA87[4][4];
T18687[4][5]=JA87[4][5];
T18687[4][6]=JA87[4][6];

T18687[5][1]=JA87[5][1];
T18687[5][2]=JA87[5][2];
T18687[5][3]=JA87[5][3];
T18687[5][4]=JA87[5][4];
T18687[5][5]=JA87[5][5];
T18687[5][6]=JA87[5][6];

T18687[6][1]=JA87[6][1];
T18687[6][2]=JA87[6][2];
T18687[6][3]=JA87[6][3];
T18687[6][4]=JA87[6][4];
T18687[6][5]=JA87[6][5];
T18687[6][6]=JA87[6][6];


T8687[1][1]=S8786[1][1]*(Si8687[1][1]*T18687[1][1] + Si8687[1][2]*T18687[2][1]) + S8786[2][1]*(Si8687[1][1]*T18687[1][2] + Si8687[1][2]*T18687[2][2]) - UPPERLEGMOD*(Si8687[1][1]*T18687[1][6] + Si8687[1][2]*T18687[2][6]);
T8687[1][2]=-(Si8687[1][1]*T18687[1][3]) - Si8687[1][2]*T18687[2][3] + (-(UPPERLEGMOD*S8786[1][1]) - YKNEE*S8786[1][3])*(Si8687[1][1]*T18687[1][4] + Si8687[1][2]*T18687[2][4]) + (-(UPPERLEGMOD*S8786[2][1]) - YKNEE*S8786[2][3])*(Si8687[1][1]*T18687[1][5] + Si8687[1][2]*T18687[2][5]);
T8687[1][3]=S8786[1][3]*(Si8687[1][1]*T18687[1][1] + Si8687[1][2]*T18687[2][1]) + S8786[2][3]*(Si8687[1][1]*T18687[1][2] + Si8687[1][2]*T18687[2][2]) - YKNEE*(Si8687[1][1]*T18687[1][6] + Si8687[1][2]*T18687[2][6]);
T8687[1][4]=S8786[1][1]*(Si8687[1][1]*T18687[1][4] + Si8687[1][2]*T18687[2][4]) + S8786[2][1]*(Si8687[1][1]*T18687[1][5] + Si8687[1][2]*T18687[2][5]);
T8687[1][5]=-(Si8687[1][1]*T18687[1][6]) - Si8687[1][2]*T18687[2][6];
T8687[1][6]=S8786[1][3]*(Si8687[1][1]*T18687[1][4] + Si8687[1][2]*T18687[2][4]) + S8786[2][3]*(Si8687[1][1]*T18687[1][5] + Si8687[1][2]*T18687[2][5]);

T8687[2][1]=-(S8786[1][1]*T18687[3][1]) - S8786[2][1]*T18687[3][2] + UPPERLEGMOD*T18687[3][6];
T8687[2][2]=T18687[3][3] - (-(UPPERLEGMOD*S8786[1][1]) - YKNEE*S8786[1][3])*T18687[3][4] - (-(UPPERLEGMOD*S8786[2][1]) - YKNEE*S8786[2][3])*T18687[3][5];
T8687[2][3]=-(S8786[1][3]*T18687[3][1]) - S8786[2][3]*T18687[3][2] + YKNEE*T18687[3][6];
T8687[2][4]=-(S8786[1][1]*T18687[3][4]) - S8786[2][1]*T18687[3][5];
T8687[2][5]=T18687[3][6];
T8687[2][6]=-(S8786[1][3]*T18687[3][4]) - S8786[2][3]*T18687[3][5];

T8687[3][1]=S8786[1][1]*(Si8687[3][1]*T18687[1][1] + Si8687[3][2]*T18687[2][1]) + S8786[2][1]*(Si8687[3][1]*T18687[1][2] + Si8687[3][2]*T18687[2][2]) - UPPERLEGMOD*(Si8687[3][1]*T18687[1][6] + Si8687[3][2]*T18687[2][6]);
T8687[3][2]=-(Si8687[3][1]*T18687[1][3]) - Si8687[3][2]*T18687[2][3] + (-(UPPERLEGMOD*S8786[1][1]) - YKNEE*S8786[1][3])*(Si8687[3][1]*T18687[1][4] + Si8687[3][2]*T18687[2][4]) + (-(UPPERLEGMOD*S8786[2][1]) - YKNEE*S8786[2][3])*(Si8687[3][1]*T18687[1][5] + Si8687[3][2]*T18687[2][5]);
T8687[3][3]=S8786[1][3]*(Si8687[3][1]*T18687[1][1] + Si8687[3][2]*T18687[2][1]) + S8786[2][3]*(Si8687[3][1]*T18687[1][2] + Si8687[3][2]*T18687[2][2]) - YKNEE*(Si8687[3][1]*T18687[1][6] + Si8687[3][2]*T18687[2][6]);
T8687[3][4]=S8786[1][1]*(Si8687[3][1]*T18687[1][4] + Si8687[3][2]*T18687[2][4]) + S8786[2][1]*(Si8687[3][1]*T18687[1][5] + Si8687[3][2]*T18687[2][5]);
T8687[3][5]=-(Si8687[3][1]*T18687[1][6]) - Si8687[3][2]*T18687[2][6];
T8687[3][6]=S8786[1][3]*(Si8687[3][1]*T18687[1][4] + Si8687[3][2]*T18687[2][4]) + S8786[2][3]*(Si8687[3][1]*T18687[1][5] + Si8687[3][2]*T18687[2][5]);

T8687[4][1]=S8786[1][1]*(-(UPPERLEGMOD*T18687[3][1]) + Si8687[1][1]*T18687[4][1] + Si8687[1][2]*T18687[5][1]) + S8786[2][1]*(-(UPPERLEGMOD*T18687[3][2]) + Si8687[1][1]*T18687[4][2] + Si8687[1][2]*T18687[5][2]) - UPPERLEGMOD*(-(UPPERLEGMOD*T18687[3][6]) + Si8687[1][1]*T18687[4][6] + Si8687[1][2]*T18687[5][6]);
T8687[4][2]=UPPERLEGMOD*T18687[3][3] - Si8687[1][1]*T18687[4][3] - Si8687[1][2]*T18687[5][3] + (-(UPPERLEGMOD*S8786[1][1]) - YKNEE*S8786[1][3])*(-(UPPERLEGMOD*T18687[3][4]) + Si8687[1][1]*T18687[4][4] + Si8687[1][2]*T18687[5][4]) + (-(UPPERLEGMOD*S8786[2][1]) - YKNEE*S8786[2][3])*(-(UPPERLEGMOD*T18687[3][5]) + Si8687[1][1]*T18687[4][5] + Si8687[1][2]*T18687[5][5]);
T8687[4][3]=S8786[1][3]*(-(UPPERLEGMOD*T18687[3][1]) + Si8687[1][1]*T18687[4][1] + Si8687[1][2]*T18687[5][1]) + S8786[2][3]*(-(UPPERLEGMOD*T18687[3][2]) + Si8687[1][1]*T18687[4][2] + Si8687[1][2]*T18687[5][2]) - YKNEE*(-(UPPERLEGMOD*T18687[3][6]) + Si8687[1][1]*T18687[4][6] + Si8687[1][2]*T18687[5][6]);
T8687[4][4]=S8786[1][1]*(-(UPPERLEGMOD*T18687[3][4]) + Si8687[1][1]*T18687[4][4] + Si8687[1][2]*T18687[5][4]) + S8786[2][1]*(-(UPPERLEGMOD*T18687[3][5]) + Si8687[1][1]*T18687[4][5] + Si8687[1][2]*T18687[5][5]);
T8687[4][5]=UPPERLEGMOD*T18687[3][6] - Si8687[1][1]*T18687[4][6] - Si8687[1][2]*T18687[5][6];
T8687[4][6]=S8786[1][3]*(-(UPPERLEGMOD*T18687[3][4]) + Si8687[1][1]*T18687[4][4] + Si8687[1][2]*T18687[5][4]) + S8786[2][3]*(-(UPPERLEGMOD*T18687[3][5]) + Si8687[1][1]*T18687[4][5] + Si8687[1][2]*T18687[5][5]);

T8687[5][1]=S8786[1][1]*((-(UPPERLEGMOD*Si8687[1][1]) - YKNEE*Si8687[3][1])*T18687[1][1] + (-(UPPERLEGMOD*Si8687[1][2]) - YKNEE*Si8687[3][2])*T18687[2][1] - T18687[6][1]) + S8786[2][1]*((-(UPPERLEGMOD*Si8687[1][1]) - YKNEE*Si8687[3][1])*T18687[1][2] + (-(UPPERLEGMOD*Si8687[1][2]) - YKNEE*Si8687[3][2])*T18687[2][2] - T18687[6][2]) - UPPERLEGMOD*((-(UPPERLEGMOD*Si8687[1][1]) - YKNEE*Si8687[3][1])*T18687[1][6] + (-(UPPERLEGMOD*Si8687[1][2]) - YKNEE*Si8687[3][2])*T18687[2][6] - T18687[6][6]);
T8687[5][2]=-((-(UPPERLEGMOD*Si8687[1][1]) - YKNEE*Si8687[3][1])*T18687[1][3]) - (-(UPPERLEGMOD*Si8687[1][2]) - YKNEE*Si8687[3][2])*T18687[2][3] + T18687[6][3] + (-(UPPERLEGMOD*S8786[1][1]) - YKNEE*S8786[1][3])*((-(UPPERLEGMOD*Si8687[1][1]) - YKNEE*Si8687[3][1])*T18687[1][4] + (-(UPPERLEGMOD*Si8687[1][2]) - YKNEE*Si8687[3][2])*T18687[2][4] - T18687[6][4]) + (-(UPPERLEGMOD*S8786[2][1]) - YKNEE*S8786[2][3])*((-(UPPERLEGMOD*Si8687[1][1]) - YKNEE*Si8687[3][1])*T18687[1][5] + (-(UPPERLEGMOD*Si8687[1][2]) - YKNEE*Si8687[3][2])*T18687[2][5] - T18687[6][5]);
T8687[5][3]=S8786[1][3]*((-(UPPERLEGMOD*Si8687[1][1]) - YKNEE*Si8687[3][1])*T18687[1][1] + (-(UPPERLEGMOD*Si8687[1][2]) - YKNEE*Si8687[3][2])*T18687[2][1] - T18687[6][1]) + S8786[2][3]*((-(UPPERLEGMOD*Si8687[1][1]) - YKNEE*Si8687[3][1])*T18687[1][2] + (-(UPPERLEGMOD*Si8687[1][2]) - YKNEE*Si8687[3][2])*T18687[2][2] - T18687[6][2]) - YKNEE*((-(UPPERLEGMOD*Si8687[1][1]) - YKNEE*Si8687[3][1])*T18687[1][6] + (-(UPPERLEGMOD*Si8687[1][2]) - YKNEE*Si8687[3][2])*T18687[2][6] - T18687[6][6]);
T8687[5][4]=S8786[1][1]*((-(UPPERLEGMOD*Si8687[1][1]) - YKNEE*Si8687[3][1])*T18687[1][4] + (-(UPPERLEGMOD*Si8687[1][2]) - YKNEE*Si8687[3][2])*T18687[2][4] - T18687[6][4]) + S8786[2][1]*((-(UPPERLEGMOD*Si8687[1][1]) - YKNEE*Si8687[3][1])*T18687[1][5] + (-(UPPERLEGMOD*Si8687[1][2]) - YKNEE*Si8687[3][2])*T18687[2][5] - T18687[6][5]);
T8687[5][5]=-((-(UPPERLEGMOD*Si8687[1][1]) - YKNEE*Si8687[3][1])*T18687[1][6]) - (-(UPPERLEGMOD*Si8687[1][2]) - YKNEE*Si8687[3][2])*T18687[2][6] + T18687[6][6];
T8687[5][6]=S8786[1][3]*((-(UPPERLEGMOD*Si8687[1][1]) - YKNEE*Si8687[3][1])*T18687[1][4] + (-(UPPERLEGMOD*Si8687[1][2]) - YKNEE*Si8687[3][2])*T18687[2][4] - T18687[6][4]) + S8786[2][3]*((-(UPPERLEGMOD*Si8687[1][1]) - YKNEE*Si8687[3][1])*T18687[1][5] + (-(UPPERLEGMOD*Si8687[1][2]) - YKNEE*Si8687[3][2])*T18687[2][5] - T18687[6][5]);

T8687[6][1]=S8786[1][1]*(-(YKNEE*T18687[3][1]) + Si8687[3][1]*T18687[4][1] + Si8687[3][2]*T18687[5][1]) + S8786[2][1]*(-(YKNEE*T18687[3][2]) + Si8687[3][1]*T18687[4][2] + Si8687[3][2]*T18687[5][2]) - UPPERLEGMOD*(-(YKNEE*T18687[3][6]) + Si8687[3][1]*T18687[4][6] + Si8687[3][2]*T18687[5][6]);
T8687[6][2]=YKNEE*T18687[3][3] - Si8687[3][1]*T18687[4][3] - Si8687[3][2]*T18687[5][3] + (-(UPPERLEGMOD*S8786[1][1]) - YKNEE*S8786[1][3])*(-(YKNEE*T18687[3][4]) + Si8687[3][1]*T18687[4][4] + Si8687[3][2]*T18687[5][4]) + (-(UPPERLEGMOD*S8786[2][1]) - YKNEE*S8786[2][3])*(-(YKNEE*T18687[3][5]) + Si8687[3][1]*T18687[4][5] + Si8687[3][2]*T18687[5][5]);
T8687[6][3]=S8786[1][3]*(-(YKNEE*T18687[3][1]) + Si8687[3][1]*T18687[4][1] + Si8687[3][2]*T18687[5][1]) + S8786[2][3]*(-(YKNEE*T18687[3][2]) + Si8687[3][1]*T18687[4][2] + Si8687[3][2]*T18687[5][2]) - YKNEE*(-(YKNEE*T18687[3][6]) + Si8687[3][1]*T18687[4][6] + Si8687[3][2]*T18687[5][6]);
T8687[6][4]=S8786[1][1]*(-(YKNEE*T18687[3][4]) + Si8687[3][1]*T18687[4][4] + Si8687[3][2]*T18687[5][4]) + S8786[2][1]*(-(YKNEE*T18687[3][5]) + Si8687[3][1]*T18687[4][5] + Si8687[3][2]*T18687[5][5]);
T8687[6][5]=YKNEE*T18687[3][6] - Si8687[3][1]*T18687[4][6] - Si8687[3][2]*T18687[5][6];
T8687[6][6]=S8786[1][3]*(-(YKNEE*T18687[3][4]) + Si8687[3][1]*T18687[4][4] + Si8687[3][2]*T18687[5][4]) + S8786[2][3]*(-(YKNEE*T18687[3][5]) + Si8687[3][1]*T18687[4][5] + Si8687[3][2]*T18687[5][5]);



}


void
hermes_InvDynArtfunc18(void)
      {
JA86[1][1]=T8687[1][1];
JA86[1][2]=links[17].mcm[3] + T8687[1][2];
JA86[1][3]=-links[17].mcm[2] + T8687[1][3];
JA86[1][4]=links[17].m + T8687[1][4];
JA86[1][5]=T8687[1][5];
JA86[1][6]=T8687[1][6];

JA86[2][1]=-links[17].mcm[3] + T8687[2][1];
JA86[2][2]=T8687[2][2];
JA86[2][3]=links[17].mcm[1] + T8687[2][3];
JA86[2][4]=T8687[2][4];
JA86[2][5]=links[17].m + T8687[2][5];
JA86[2][6]=T8687[2][6];

JA86[3][1]=links[17].mcm[2] + T8687[3][1];
JA86[3][2]=-links[17].mcm[1] + T8687[3][2];
JA86[3][3]=T8687[3][3];
JA86[3][4]=T8687[3][4];
JA86[3][5]=T8687[3][5];
JA86[3][6]=links[17].m + T8687[3][6];

JA86[4][1]=links[17].inertia[1][1] + T8687[4][1];
JA86[4][2]=links[17].inertia[1][2] + T8687[4][2];
JA86[4][3]=links[17].inertia[1][3] + T8687[4][3];
JA86[4][4]=T8687[4][4];
JA86[4][5]=-links[17].mcm[3] + T8687[4][5];
JA86[4][6]=links[17].mcm[2] + T8687[4][6];

JA86[5][1]=links[17].inertia[1][2] + T8687[5][1];
JA86[5][2]=links[17].inertia[2][2] + T8687[5][2];
JA86[5][3]=links[17].inertia[2][3] + T8687[5][3];
JA86[5][4]=links[17].mcm[3] + T8687[5][4];
JA86[5][5]=T8687[5][5];
JA86[5][6]=-links[17].mcm[1] + T8687[5][6];

JA86[6][1]=links[17].inertia[1][3] + T8687[6][1];
JA86[6][2]=links[17].inertia[2][3] + T8687[6][2];
JA86[6][3]=links[17].inertia[3][3] + T8687[6][3];
JA86[6][4]=-links[17].mcm[2] + T8687[6][4];
JA86[6][5]=links[17].mcm[1] + T8687[6][5];
JA86[6][6]=T8687[6][6];


h86[1]=JA86[1][3];
h86[2]=JA86[2][3];
h86[3]=JA86[3][3];
h86[4]=JA86[4][3];
h86[5]=JA86[5][3];
h86[6]=JA86[6][3];

T18586[1][1]=JA86[1][1];
T18586[1][2]=JA86[1][2];
T18586[1][3]=JA86[1][3];
T18586[1][4]=JA86[1][4];
T18586[1][5]=JA86[1][5];
T18586[1][6]=JA86[1][6];

T18586[2][1]=JA86[2][1];
T18586[2][2]=JA86[2][2];
T18586[2][3]=JA86[2][3];
T18586[2][4]=JA86[2][4];
T18586[2][5]=JA86[2][5];
T18586[2][6]=JA86[2][6];

T18586[3][1]=JA86[3][1];
T18586[3][2]=JA86[3][2];
T18586[3][3]=JA86[3][3];
T18586[3][4]=JA86[3][4];
T18586[3][5]=JA86[3][5];
T18586[3][6]=JA86[3][6];

T18586[4][1]=JA86[4][1];
T18586[4][2]=JA86[4][2];
T18586[4][3]=JA86[4][3];
T18586[4][4]=JA86[4][4];
T18586[4][5]=JA86[4][5];
T18586[4][6]=JA86[4][6];

T18586[5][1]=JA86[5][1];
T18586[5][2]=JA86[5][2];
T18586[5][3]=JA86[5][3];
T18586[5][4]=JA86[5][4];
T18586[5][5]=JA86[5][5];
T18586[5][6]=JA86[5][6];

T18586[6][1]=JA86[6][1];
T18586[6][2]=JA86[6][2];
T18586[6][3]=JA86[6][3];
T18586[6][4]=JA86[6][4];
T18586[6][5]=JA86[6][5];
T18586[6][6]=JA86[6][6];


T8586[1][1]=S8685[1][1]*(Si8586[1][1]*T18586[1][1] + Si8586[1][2]*T18586[2][1] + Si8586[1][3]*T18586[3][1]) + S8685[2][1]*(Si8586[1][1]*T18586[1][2] + Si8586[1][2]*T18586[2][2] + Si8586[1][3]*T18586[3][2]) + S8685[3][1]*(Si8586[1][1]*T18586[1][3] + Si8586[1][2]*T18586[2][3] + Si8586[1][3]*T18586[3][3]);
T8586[1][2]=S8685[1][2]*(Si8586[1][1]*T18586[1][1] + Si8586[1][2]*T18586[2][1] + Si8586[1][3]*T18586[3][1]) + S8685[2][2]*(Si8586[1][1]*T18586[1][2] + Si8586[1][2]*T18586[2][2] + Si8586[1][3]*T18586[3][2]) + S8685[3][2]*(Si8586[1][1]*T18586[1][3] + Si8586[1][2]*T18586[2][3] + Si8586[1][3]*T18586[3][3]) - YHIP*S8685[1][3]*(Si8586[1][1]*T18586[1][4] + Si8586[1][2]*T18586[2][4] + Si8586[1][3]*T18586[3][4]) - YHIP*S8685[2][3]*(Si8586[1][1]*T18586[1][5] + Si8586[1][2]*T18586[2][5] + Si8586[1][3]*T18586[3][5]);
T8586[1][3]=S8685[1][3]*(Si8586[1][1]*T18586[1][1] + Si8586[1][2]*T18586[2][1] + Si8586[1][3]*T18586[3][1]) + S8685[2][3]*(Si8586[1][1]*T18586[1][2] + Si8586[1][2]*T18586[2][2] + Si8586[1][3]*T18586[3][2]) + YHIP*S8685[1][2]*(Si8586[1][1]*T18586[1][4] + Si8586[1][2]*T18586[2][4] + Si8586[1][3]*T18586[3][4]) + YHIP*S8685[2][2]*(Si8586[1][1]*T18586[1][5] + Si8586[1][2]*T18586[2][5] + Si8586[1][3]*T18586[3][5]) + YHIP*S8685[3][2]*(Si8586[1][1]*T18586[1][6] + Si8586[1][2]*T18586[2][6] + Si8586[1][3]*T18586[3][6]);
T8586[1][4]=S8685[1][1]*(Si8586[1][1]*T18586[1][4] + Si8586[1][2]*T18586[2][4] + Si8586[1][3]*T18586[3][4]) + S8685[2][1]*(Si8586[1][1]*T18586[1][5] + Si8586[1][2]*T18586[2][5] + Si8586[1][3]*T18586[3][5]) + S8685[3][1]*(Si8586[1][1]*T18586[1][6] + Si8586[1][2]*T18586[2][6] + Si8586[1][3]*T18586[3][6]);
T8586[1][5]=S8685[1][2]*(Si8586[1][1]*T18586[1][4] + Si8586[1][2]*T18586[2][4] + Si8586[1][3]*T18586[3][4]) + S8685[2][2]*(Si8586[1][1]*T18586[1][5] + Si8586[1][2]*T18586[2][5] + Si8586[1][3]*T18586[3][5]) + S8685[3][2]*(Si8586[1][1]*T18586[1][6] + Si8586[1][2]*T18586[2][6] + Si8586[1][3]*T18586[3][6]);
T8586[1][6]=S8685[1][3]*(Si8586[1][1]*T18586[1][4] + Si8586[1][2]*T18586[2][4] + Si8586[1][3]*T18586[3][4]) + S8685[2][3]*(Si8586[1][1]*T18586[1][5] + Si8586[1][2]*T18586[2][5] + Si8586[1][3]*T18586[3][5]);

T8586[2][1]=S8685[1][1]*(Si8586[2][1]*T18586[1][1] + Si8586[2][2]*T18586[2][1] + Si8586[2][3]*T18586[3][1]) + S8685[2][1]*(Si8586[2][1]*T18586[1][2] + Si8586[2][2]*T18586[2][2] + Si8586[2][3]*T18586[3][2]) + S8685[3][1]*(Si8586[2][1]*T18586[1][3] + Si8586[2][2]*T18586[2][3] + Si8586[2][3]*T18586[3][3]);
T8586[2][2]=S8685[1][2]*(Si8586[2][1]*T18586[1][1] + Si8586[2][2]*T18586[2][1] + Si8586[2][3]*T18586[3][1]) + S8685[2][2]*(Si8586[2][1]*T18586[1][2] + Si8586[2][2]*T18586[2][2] + Si8586[2][3]*T18586[3][2]) + S8685[3][2]*(Si8586[2][1]*T18586[1][3] + Si8586[2][2]*T18586[2][3] + Si8586[2][3]*T18586[3][3]) - YHIP*S8685[1][3]*(Si8586[2][1]*T18586[1][4] + Si8586[2][2]*T18586[2][4] + Si8586[2][3]*T18586[3][4]) - YHIP*S8685[2][3]*(Si8586[2][1]*T18586[1][5] + Si8586[2][2]*T18586[2][5] + Si8586[2][3]*T18586[3][5]);
T8586[2][3]=S8685[1][3]*(Si8586[2][1]*T18586[1][1] + Si8586[2][2]*T18586[2][1] + Si8586[2][3]*T18586[3][1]) + S8685[2][3]*(Si8586[2][1]*T18586[1][2] + Si8586[2][2]*T18586[2][2] + Si8586[2][3]*T18586[3][2]) + YHIP*S8685[1][2]*(Si8586[2][1]*T18586[1][4] + Si8586[2][2]*T18586[2][4] + Si8586[2][3]*T18586[3][4]) + YHIP*S8685[2][2]*(Si8586[2][1]*T18586[1][5] + Si8586[2][2]*T18586[2][5] + Si8586[2][3]*T18586[3][5]) + YHIP*S8685[3][2]*(Si8586[2][1]*T18586[1][6] + Si8586[2][2]*T18586[2][6] + Si8586[2][3]*T18586[3][6]);
T8586[2][4]=S8685[1][1]*(Si8586[2][1]*T18586[1][4] + Si8586[2][2]*T18586[2][4] + Si8586[2][3]*T18586[3][4]) + S8685[2][1]*(Si8586[2][1]*T18586[1][5] + Si8586[2][2]*T18586[2][5] + Si8586[2][3]*T18586[3][5]) + S8685[3][1]*(Si8586[2][1]*T18586[1][6] + Si8586[2][2]*T18586[2][6] + Si8586[2][3]*T18586[3][6]);
T8586[2][5]=S8685[1][2]*(Si8586[2][1]*T18586[1][4] + Si8586[2][2]*T18586[2][4] + Si8586[2][3]*T18586[3][4]) + S8685[2][2]*(Si8586[2][1]*T18586[1][5] + Si8586[2][2]*T18586[2][5] + Si8586[2][3]*T18586[3][5]) + S8685[3][2]*(Si8586[2][1]*T18586[1][6] + Si8586[2][2]*T18586[2][6] + Si8586[2][3]*T18586[3][6]);
T8586[2][6]=S8685[1][3]*(Si8586[2][1]*T18586[1][4] + Si8586[2][2]*T18586[2][4] + Si8586[2][3]*T18586[3][4]) + S8685[2][3]*(Si8586[2][1]*T18586[1][5] + Si8586[2][2]*T18586[2][5] + Si8586[2][3]*T18586[3][5]);

T8586[3][1]=S8685[1][1]*(Si8586[3][1]*T18586[1][1] + Si8586[3][2]*T18586[2][1]) + S8685[2][1]*(Si8586[3][1]*T18586[1][2] + Si8586[3][2]*T18586[2][2]) + S8685[3][1]*(Si8586[3][1]*T18586[1][3] + Si8586[3][2]*T18586[2][3]);
T8586[3][2]=S8685[1][2]*(Si8586[3][1]*T18586[1][1] + Si8586[3][2]*T18586[2][1]) + S8685[2][2]*(Si8586[3][1]*T18586[1][2] + Si8586[3][2]*T18586[2][2]) + S8685[3][2]*(Si8586[3][1]*T18586[1][3] + Si8586[3][2]*T18586[2][3]) - YHIP*S8685[1][3]*(Si8586[3][1]*T18586[1][4] + Si8586[3][2]*T18586[2][4]) - YHIP*S8685[2][3]*(Si8586[3][1]*T18586[1][5] + Si8586[3][2]*T18586[2][5]);
T8586[3][3]=S8685[1][3]*(Si8586[3][1]*T18586[1][1] + Si8586[3][2]*T18586[2][1]) + S8685[2][3]*(Si8586[3][1]*T18586[1][2] + Si8586[3][2]*T18586[2][2]) + YHIP*S8685[1][2]*(Si8586[3][1]*T18586[1][4] + Si8586[3][2]*T18586[2][4]) + YHIP*S8685[2][2]*(Si8586[3][1]*T18586[1][5] + Si8586[3][2]*T18586[2][5]) + YHIP*S8685[3][2]*(Si8586[3][1]*T18586[1][6] + Si8586[3][2]*T18586[2][6]);
T8586[3][4]=S8685[1][1]*(Si8586[3][1]*T18586[1][4] + Si8586[3][2]*T18586[2][4]) + S8685[2][1]*(Si8586[3][1]*T18586[1][5] + Si8586[3][2]*T18586[2][5]) + S8685[3][1]*(Si8586[3][1]*T18586[1][6] + Si8586[3][2]*T18586[2][6]);
T8586[3][5]=S8685[1][2]*(Si8586[3][1]*T18586[1][4] + Si8586[3][2]*T18586[2][4]) + S8685[2][2]*(Si8586[3][1]*T18586[1][5] + Si8586[3][2]*T18586[2][5]) + S8685[3][2]*(Si8586[3][1]*T18586[1][6] + Si8586[3][2]*T18586[2][6]);
T8586[3][6]=S8685[1][3]*(Si8586[3][1]*T18586[1][4] + Si8586[3][2]*T18586[2][4]) + S8685[2][3]*(Si8586[3][1]*T18586[1][5] + Si8586[3][2]*T18586[2][5]);

T8586[4][1]=S8685[1][1]*(Si8586[1][1]*T18586[4][1] + Si8586[1][2]*T18586[5][1] + Si8586[1][3]*T18586[6][1]) + S8685[2][1]*(Si8586[1][1]*T18586[4][2] + Si8586[1][2]*T18586[5][2] + Si8586[1][3]*T18586[6][2]) + S8685[3][1]*(Si8586[1][1]*T18586[4][3] + Si8586[1][2]*T18586[5][3] + Si8586[1][3]*T18586[6][3]);
T8586[4][2]=S8685[1][2]*(Si8586[1][1]*T18586[4][1] + Si8586[1][2]*T18586[5][1] + Si8586[1][3]*T18586[6][1]) + S8685[2][2]*(Si8586[1][1]*T18586[4][2] + Si8586[1][2]*T18586[5][2] + Si8586[1][3]*T18586[6][2]) + S8685[3][2]*(Si8586[1][1]*T18586[4][3] + Si8586[1][2]*T18586[5][3] + Si8586[1][3]*T18586[6][3]) - YHIP*S8685[1][3]*(Si8586[1][1]*T18586[4][4] + Si8586[1][2]*T18586[5][4] + Si8586[1][3]*T18586[6][4]) - YHIP*S8685[2][3]*(Si8586[1][1]*T18586[4][5] + Si8586[1][2]*T18586[5][5] + Si8586[1][3]*T18586[6][5]);
T8586[4][3]=S8685[1][3]*(Si8586[1][1]*T18586[4][1] + Si8586[1][2]*T18586[5][1] + Si8586[1][3]*T18586[6][1]) + S8685[2][3]*(Si8586[1][1]*T18586[4][2] + Si8586[1][2]*T18586[5][2] + Si8586[1][3]*T18586[6][2]) + YHIP*S8685[1][2]*(Si8586[1][1]*T18586[4][4] + Si8586[1][2]*T18586[5][4] + Si8586[1][3]*T18586[6][4]) + YHIP*S8685[2][2]*(Si8586[1][1]*T18586[4][5] + Si8586[1][2]*T18586[5][5] + Si8586[1][3]*T18586[6][5]) + YHIP*S8685[3][2]*(Si8586[1][1]*T18586[4][6] + Si8586[1][2]*T18586[5][6] + Si8586[1][3]*T18586[6][6]);
T8586[4][4]=S8685[1][1]*(Si8586[1][1]*T18586[4][4] + Si8586[1][2]*T18586[5][4] + Si8586[1][3]*T18586[6][4]) + S8685[2][1]*(Si8586[1][1]*T18586[4][5] + Si8586[1][2]*T18586[5][5] + Si8586[1][3]*T18586[6][5]) + S8685[3][1]*(Si8586[1][1]*T18586[4][6] + Si8586[1][2]*T18586[5][6] + Si8586[1][3]*T18586[6][6]);
T8586[4][5]=S8685[1][2]*(Si8586[1][1]*T18586[4][4] + Si8586[1][2]*T18586[5][4] + Si8586[1][3]*T18586[6][4]) + S8685[2][2]*(Si8586[1][1]*T18586[4][5] + Si8586[1][2]*T18586[5][5] + Si8586[1][3]*T18586[6][5]) + S8685[3][2]*(Si8586[1][1]*T18586[4][6] + Si8586[1][2]*T18586[5][6] + Si8586[1][3]*T18586[6][6]);
T8586[4][6]=S8685[1][3]*(Si8586[1][1]*T18586[4][4] + Si8586[1][2]*T18586[5][4] + Si8586[1][3]*T18586[6][4]) + S8685[2][3]*(Si8586[1][1]*T18586[4][5] + Si8586[1][2]*T18586[5][5] + Si8586[1][3]*T18586[6][5]);

T8586[5][1]=S8685[1][1]*(-(YHIP*Si8586[3][1]*T18586[1][1]) - YHIP*Si8586[3][2]*T18586[2][1] + Si8586[2][1]*T18586[4][1] + Si8586[2][2]*T18586[5][1] + Si8586[2][3]*T18586[6][1]) + S8685[2][1]*(-(YHIP*Si8586[3][1]*T18586[1][2]) - YHIP*Si8586[3][2]*T18586[2][2] + Si8586[2][1]*T18586[4][2] + Si8586[2][2]*T18586[5][2] + Si8586[2][3]*T18586[6][2]) + S8685[3][1]*(-(YHIP*Si8586[3][1]*T18586[1][3]) - YHIP*Si8586[3][2]*T18586[2][3] + Si8586[2][1]*T18586[4][3] + Si8586[2][2]*T18586[5][3] + Si8586[2][3]*T18586[6][3]);
T8586[5][2]=S8685[1][2]*(-(YHIP*Si8586[3][1]*T18586[1][1]) - YHIP*Si8586[3][2]*T18586[2][1] + Si8586[2][1]*T18586[4][1] + Si8586[2][2]*T18586[5][1] + Si8586[2][3]*T18586[6][1]) + S8685[2][2]*(-(YHIP*Si8586[3][1]*T18586[1][2]) - YHIP*Si8586[3][2]*T18586[2][2] + Si8586[2][1]*T18586[4][2] + Si8586[2][2]*T18586[5][2] + Si8586[2][3]*T18586[6][2]) + S8685[3][2]*(-(YHIP*Si8586[3][1]*T18586[1][3]) - YHIP*Si8586[3][2]*T18586[2][3] + Si8586[2][1]*T18586[4][3] + Si8586[2][2]*T18586[5][3] + Si8586[2][3]*T18586[6][3]) - YHIP*S8685[1][3]*(-(YHIP*Si8586[3][1]*T18586[1][4]) - YHIP*Si8586[3][2]*T18586[2][4] + Si8586[2][1]*T18586[4][4] + Si8586[2][2]*T18586[5][4] + Si8586[2][3]*T18586[6][4]) - YHIP*S8685[2][3]*(-(YHIP*Si8586[3][1]*T18586[1][5]) - YHIP*Si8586[3][2]*T18586[2][5] + Si8586[2][1]*T18586[4][5] + Si8586[2][2]*T18586[5][5] + Si8586[2][3]*T18586[6][5]);
T8586[5][3]=S8685[1][3]*(-(YHIP*Si8586[3][1]*T18586[1][1]) - YHIP*Si8586[3][2]*T18586[2][1] + Si8586[2][1]*T18586[4][1] + Si8586[2][2]*T18586[5][1] + Si8586[2][3]*T18586[6][1]) + S8685[2][3]*(-(YHIP*Si8586[3][1]*T18586[1][2]) - YHIP*Si8586[3][2]*T18586[2][2] + Si8586[2][1]*T18586[4][2] + Si8586[2][2]*T18586[5][2] + Si8586[2][3]*T18586[6][2]) + YHIP*S8685[1][2]*(-(YHIP*Si8586[3][1]*T18586[1][4]) - YHIP*Si8586[3][2]*T18586[2][4] + Si8586[2][1]*T18586[4][4] + Si8586[2][2]*T18586[5][4] + Si8586[2][3]*T18586[6][4]) + YHIP*S8685[2][2]*(-(YHIP*Si8586[3][1]*T18586[1][5]) - YHIP*Si8586[3][2]*T18586[2][5] + Si8586[2][1]*T18586[4][5] + Si8586[2][2]*T18586[5][5] + Si8586[2][3]*T18586[6][5]) + YHIP*S8685[3][2]*(-(YHIP*Si8586[3][1]*T18586[1][6]) - YHIP*Si8586[3][2]*T18586[2][6] + Si8586[2][1]*T18586[4][6] + Si8586[2][2]*T18586[5][6] + Si8586[2][3]*T18586[6][6]);
T8586[5][4]=S8685[1][1]*(-(YHIP*Si8586[3][1]*T18586[1][4]) - YHIP*Si8586[3][2]*T18586[2][4] + Si8586[2][1]*T18586[4][4] + Si8586[2][2]*T18586[5][4] + Si8586[2][3]*T18586[6][4]) + S8685[2][1]*(-(YHIP*Si8586[3][1]*T18586[1][5]) - YHIP*Si8586[3][2]*T18586[2][5] + Si8586[2][1]*T18586[4][5] + Si8586[2][2]*T18586[5][5] + Si8586[2][3]*T18586[6][5]) + S8685[3][1]*(-(YHIP*Si8586[3][1]*T18586[1][6]) - YHIP*Si8586[3][2]*T18586[2][6] + Si8586[2][1]*T18586[4][6] + Si8586[2][2]*T18586[5][6] + Si8586[2][3]*T18586[6][6]);
T8586[5][5]=S8685[1][2]*(-(YHIP*Si8586[3][1]*T18586[1][4]) - YHIP*Si8586[3][2]*T18586[2][4] + Si8586[2][1]*T18586[4][4] + Si8586[2][2]*T18586[5][4] + Si8586[2][3]*T18586[6][4]) + S8685[2][2]*(-(YHIP*Si8586[3][1]*T18586[1][5]) - YHIP*Si8586[3][2]*T18586[2][5] + Si8586[2][1]*T18586[4][5] + Si8586[2][2]*T18586[5][5] + Si8586[2][3]*T18586[6][5]) + S8685[3][2]*(-(YHIP*Si8586[3][1]*T18586[1][6]) - YHIP*Si8586[3][2]*T18586[2][6] + Si8586[2][1]*T18586[4][6] + Si8586[2][2]*T18586[5][6] + Si8586[2][3]*T18586[6][6]);
T8586[5][6]=S8685[1][3]*(-(YHIP*Si8586[3][1]*T18586[1][4]) - YHIP*Si8586[3][2]*T18586[2][4] + Si8586[2][1]*T18586[4][4] + Si8586[2][2]*T18586[5][4] + Si8586[2][3]*T18586[6][4]) + S8685[2][3]*(-(YHIP*Si8586[3][1]*T18586[1][5]) - YHIP*Si8586[3][2]*T18586[2][5] + Si8586[2][1]*T18586[4][5] + Si8586[2][2]*T18586[5][5] + Si8586[2][3]*T18586[6][5]);

T8586[6][1]=S8685[1][1]*(YHIP*Si8586[2][1]*T18586[1][1] + YHIP*Si8586[2][2]*T18586[2][1] + YHIP*Si8586[2][3]*T18586[3][1] + Si8586[3][1]*T18586[4][1] + Si8586[3][2]*T18586[5][1]) + S8685[2][1]*(YHIP*Si8586[2][1]*T18586[1][2] + YHIP*Si8586[2][2]*T18586[2][2] + YHIP*Si8586[2][3]*T18586[3][2] + Si8586[3][1]*T18586[4][2] + Si8586[3][2]*T18586[5][2]) + S8685[3][1]*(YHIP*Si8586[2][1]*T18586[1][3] + YHIP*Si8586[2][2]*T18586[2][3] + YHIP*Si8586[2][3]*T18586[3][3] + Si8586[3][1]*T18586[4][3] + Si8586[3][2]*T18586[5][3]);
T8586[6][2]=S8685[1][2]*(YHIP*Si8586[2][1]*T18586[1][1] + YHIP*Si8586[2][2]*T18586[2][1] + YHIP*Si8586[2][3]*T18586[3][1] + Si8586[3][1]*T18586[4][1] + Si8586[3][2]*T18586[5][1]) + S8685[2][2]*(YHIP*Si8586[2][1]*T18586[1][2] + YHIP*Si8586[2][2]*T18586[2][2] + YHIP*Si8586[2][3]*T18586[3][2] + Si8586[3][1]*T18586[4][2] + Si8586[3][2]*T18586[5][2]) + S8685[3][2]*(YHIP*Si8586[2][1]*T18586[1][3] + YHIP*Si8586[2][2]*T18586[2][3] + YHIP*Si8586[2][3]*T18586[3][3] + Si8586[3][1]*T18586[4][3] + Si8586[3][2]*T18586[5][3]) - YHIP*S8685[1][3]*(YHIP*Si8586[2][1]*T18586[1][4] + YHIP*Si8586[2][2]*T18586[2][4] + YHIP*Si8586[2][3]*T18586[3][4] + Si8586[3][1]*T18586[4][4] + Si8586[3][2]*T18586[5][4]) - YHIP*S8685[2][3]*(YHIP*Si8586[2][1]*T18586[1][5] + YHIP*Si8586[2][2]*T18586[2][5] + YHIP*Si8586[2][3]*T18586[3][5] + Si8586[3][1]*T18586[4][5] + Si8586[3][2]*T18586[5][5]);
T8586[6][3]=S8685[1][3]*(YHIP*Si8586[2][1]*T18586[1][1] + YHIP*Si8586[2][2]*T18586[2][1] + YHIP*Si8586[2][3]*T18586[3][1] + Si8586[3][1]*T18586[4][1] + Si8586[3][2]*T18586[5][1]) + S8685[2][3]*(YHIP*Si8586[2][1]*T18586[1][2] + YHIP*Si8586[2][2]*T18586[2][2] + YHIP*Si8586[2][3]*T18586[3][2] + Si8586[3][1]*T18586[4][2] + Si8586[3][2]*T18586[5][2]) + YHIP*S8685[1][2]*(YHIP*Si8586[2][1]*T18586[1][4] + YHIP*Si8586[2][2]*T18586[2][4] + YHIP*Si8586[2][3]*T18586[3][4] + Si8586[3][1]*T18586[4][4] + Si8586[3][2]*T18586[5][4]) + YHIP*S8685[2][2]*(YHIP*Si8586[2][1]*T18586[1][5] + YHIP*Si8586[2][2]*T18586[2][5] + YHIP*Si8586[2][3]*T18586[3][5] + Si8586[3][1]*T18586[4][5] + Si8586[3][2]*T18586[5][5]) + YHIP*S8685[3][2]*(YHIP*Si8586[2][1]*T18586[1][6] + YHIP*Si8586[2][2]*T18586[2][6] + YHIP*Si8586[2][3]*T18586[3][6] + Si8586[3][1]*T18586[4][6] + Si8586[3][2]*T18586[5][6]);
T8586[6][4]=S8685[1][1]*(YHIP*Si8586[2][1]*T18586[1][4] + YHIP*Si8586[2][2]*T18586[2][4] + YHIP*Si8586[2][3]*T18586[3][4] + Si8586[3][1]*T18586[4][4] + Si8586[3][2]*T18586[5][4]) + S8685[2][1]*(YHIP*Si8586[2][1]*T18586[1][5] + YHIP*Si8586[2][2]*T18586[2][5] + YHIP*Si8586[2][3]*T18586[3][5] + Si8586[3][1]*T18586[4][5] + Si8586[3][2]*T18586[5][5]) + S8685[3][1]*(YHIP*Si8586[2][1]*T18586[1][6] + YHIP*Si8586[2][2]*T18586[2][6] + YHIP*Si8586[2][3]*T18586[3][6] + Si8586[3][1]*T18586[4][6] + Si8586[3][2]*T18586[5][6]);
T8586[6][5]=S8685[1][2]*(YHIP*Si8586[2][1]*T18586[1][4] + YHIP*Si8586[2][2]*T18586[2][4] + YHIP*Si8586[2][3]*T18586[3][4] + Si8586[3][1]*T18586[4][4] + Si8586[3][2]*T18586[5][4]) + S8685[2][2]*(YHIP*Si8586[2][1]*T18586[1][5] + YHIP*Si8586[2][2]*T18586[2][5] + YHIP*Si8586[2][3]*T18586[3][5] + Si8586[3][1]*T18586[4][5] + Si8586[3][2]*T18586[5][5]) + S8685[3][2]*(YHIP*Si8586[2][1]*T18586[1][6] + YHIP*Si8586[2][2]*T18586[2][6] + YHIP*Si8586[2][3]*T18586[3][6] + Si8586[3][1]*T18586[4][6] + Si8586[3][2]*T18586[5][6]);
T8586[6][6]=S8685[1][3]*(YHIP*Si8586[2][1]*T18586[1][4] + YHIP*Si8586[2][2]*T18586[2][4] + YHIP*Si8586[2][3]*T18586[3][4] + Si8586[3][1]*T18586[4][4] + Si8586[3][2]*T18586[5][4]) + S8685[2][3]*(YHIP*Si8586[2][1]*T18586[1][5] + YHIP*Si8586[2][2]*T18586[2][5] + YHIP*Si8586[2][3]*T18586[3][5] + Si8586[3][1]*T18586[4][5] + Si8586[3][2]*T18586[5][5]);



}


void
hermes_InvDynArtfunc19(void)
      {
JA85[1][1]=T8586[1][1];
JA85[1][2]=links[15].mcm[3] + T8586[1][2];
JA85[1][3]=-links[15].mcm[2] + T8586[1][3];
JA85[1][4]=links[15].m + T8586[1][4];
JA85[1][5]=T8586[1][5];
JA85[1][6]=T8586[1][6];

JA85[2][1]=-links[15].mcm[3] + T8586[2][1];
JA85[2][2]=T8586[2][2];
JA85[2][3]=links[15].mcm[1] + T8586[2][3];
JA85[2][4]=T8586[2][4];
JA85[2][5]=links[15].m + T8586[2][5];
JA85[2][6]=T8586[2][6];

JA85[3][1]=links[15].mcm[2] + T8586[3][1];
JA85[3][2]=-links[15].mcm[1] + T8586[3][2];
JA85[3][3]=T8586[3][3];
JA85[3][4]=T8586[3][4];
JA85[3][5]=T8586[3][5];
JA85[3][6]=links[15].m + T8586[3][6];

JA85[4][1]=links[15].inertia[1][1] + T8586[4][1];
JA85[4][2]=links[15].inertia[1][2] + T8586[4][2];
JA85[4][3]=links[15].inertia[1][3] + T8586[4][3];
JA85[4][4]=T8586[4][4];
JA85[4][5]=-links[15].mcm[3] + T8586[4][5];
JA85[4][6]=links[15].mcm[2] + T8586[4][6];

JA85[5][1]=links[15].inertia[1][2] + T8586[5][1];
JA85[5][2]=links[15].inertia[2][2] + T8586[5][2];
JA85[5][3]=links[15].inertia[2][3] + T8586[5][3];
JA85[5][4]=links[15].mcm[3] + T8586[5][4];
JA85[5][5]=T8586[5][5];
JA85[5][6]=-links[15].mcm[1] + T8586[5][6];

JA85[6][1]=links[15].inertia[1][3] + T8586[6][1];
JA85[6][2]=links[15].inertia[2][3] + T8586[6][2];
JA85[6][3]=links[15].inertia[3][3] + T8586[6][3];
JA85[6][4]=-links[15].mcm[2] + T8586[6][4];
JA85[6][5]=links[15].mcm[1] + T8586[6][5];
JA85[6][6]=T8586[6][6];


h85[1]=JA85[1][3];
h85[2]=JA85[2][3];
h85[3]=JA85[3][3];
h85[4]=JA85[4][3];
h85[5]=JA85[5][3];
h85[6]=JA85[6][3];

T18485[1][1]=JA85[1][1];
T18485[1][2]=JA85[1][2];
T18485[1][3]=JA85[1][3];
T18485[1][4]=JA85[1][4];
T18485[1][5]=JA85[1][5];
T18485[1][6]=JA85[1][6];

T18485[2][1]=JA85[2][1];
T18485[2][2]=JA85[2][2];
T18485[2][3]=JA85[2][3];
T18485[2][4]=JA85[2][4];
T18485[2][5]=JA85[2][5];
T18485[2][6]=JA85[2][6];

T18485[3][1]=JA85[3][1];
T18485[3][2]=JA85[3][2];
T18485[3][3]=JA85[3][3];
T18485[3][4]=JA85[3][4];
T18485[3][5]=JA85[3][5];
T18485[3][6]=JA85[3][6];

T18485[4][1]=JA85[4][1];
T18485[4][2]=JA85[4][2];
T18485[4][3]=JA85[4][3];
T18485[4][4]=JA85[4][4];
T18485[4][5]=JA85[4][5];
T18485[4][6]=JA85[4][6];

T18485[5][1]=JA85[5][1];
T18485[5][2]=JA85[5][2];
T18485[5][3]=JA85[5][3];
T18485[5][4]=JA85[5][4];
T18485[5][5]=JA85[5][5];
T18485[5][6]=JA85[5][6];

T18485[6][1]=JA85[6][1];
T18485[6][2]=JA85[6][2];
T18485[6][3]=JA85[6][3];
T18485[6][4]=JA85[6][4];
T18485[6][5]=JA85[6][5];
T18485[6][6]=JA85[6][6];


T8485[1][1]=S8584[1][1]*(Si8485[1][1]*T18485[1][1] + Si8485[1][2]*T18485[2][1]) + S8584[2][1]*(Si8485[1][1]*T18485[1][2] + Si8485[1][2]*T18485[2][2]);
T8485[1][2]=Si8485[1][1]*T18485[1][3] + Si8485[1][2]*T18485[2][3];
T8485[1][3]=S8584[1][3]*(Si8485[1][1]*T18485[1][1] + Si8485[1][2]*T18485[2][1]) + S8584[2][3]*(Si8485[1][1]*T18485[1][2] + Si8485[1][2]*T18485[2][2]);
T8485[1][4]=S8584[1][1]*(Si8485[1][1]*T18485[1][4] + Si8485[1][2]*T18485[2][4]) + S8584[2][1]*(Si8485[1][1]*T18485[1][5] + Si8485[1][2]*T18485[2][5]);
T8485[1][5]=Si8485[1][1]*T18485[1][6] + Si8485[1][2]*T18485[2][6];
T8485[1][6]=S8584[1][3]*(Si8485[1][1]*T18485[1][4] + Si8485[1][2]*T18485[2][4]) + S8584[2][3]*(Si8485[1][1]*T18485[1][5] + Si8485[1][2]*T18485[2][5]);

T8485[2][1]=S8584[1][1]*T18485[3][1] + S8584[2][1]*T18485[3][2];
T8485[2][2]=T18485[3][3];
T8485[2][3]=S8584[1][3]*T18485[3][1] + S8584[2][3]*T18485[3][2];
T8485[2][4]=S8584[1][1]*T18485[3][4] + S8584[2][1]*T18485[3][5];
T8485[2][5]=T18485[3][6];
T8485[2][6]=S8584[1][3]*T18485[3][4] + S8584[2][3]*T18485[3][5];

T8485[3][1]=S8584[1][1]*(Si8485[3][1]*T18485[1][1] + Si8485[3][2]*T18485[2][1]) + S8584[2][1]*(Si8485[3][1]*T18485[1][2] + Si8485[3][2]*T18485[2][2]);
T8485[3][2]=Si8485[3][1]*T18485[1][3] + Si8485[3][2]*T18485[2][3];
T8485[3][3]=S8584[1][3]*(Si8485[3][1]*T18485[1][1] + Si8485[3][2]*T18485[2][1]) + S8584[2][3]*(Si8485[3][1]*T18485[1][2] + Si8485[3][2]*T18485[2][2]);
T8485[3][4]=S8584[1][1]*(Si8485[3][1]*T18485[1][4] + Si8485[3][2]*T18485[2][4]) + S8584[2][1]*(Si8485[3][1]*T18485[1][5] + Si8485[3][2]*T18485[2][5]);
T8485[3][5]=Si8485[3][1]*T18485[1][6] + Si8485[3][2]*T18485[2][6];
T8485[3][6]=S8584[1][3]*(Si8485[3][1]*T18485[1][4] + Si8485[3][2]*T18485[2][4]) + S8584[2][3]*(Si8485[3][1]*T18485[1][5] + Si8485[3][2]*T18485[2][5]);

T8485[4][1]=S8584[1][1]*(Si8485[1][1]*T18485[4][1] + Si8485[1][2]*T18485[5][1]) + S8584[2][1]*(Si8485[1][1]*T18485[4][2] + Si8485[1][2]*T18485[5][2]);
T8485[4][2]=Si8485[1][1]*T18485[4][3] + Si8485[1][2]*T18485[5][3];
T8485[4][3]=S8584[1][3]*(Si8485[1][1]*T18485[4][1] + Si8485[1][2]*T18485[5][1]) + S8584[2][3]*(Si8485[1][1]*T18485[4][2] + Si8485[1][2]*T18485[5][2]);
T8485[4][4]=S8584[1][1]*(Si8485[1][1]*T18485[4][4] + Si8485[1][2]*T18485[5][4]) + S8584[2][1]*(Si8485[1][1]*T18485[4][5] + Si8485[1][2]*T18485[5][5]);
T8485[4][5]=Si8485[1][1]*T18485[4][6] + Si8485[1][2]*T18485[5][6];
T8485[4][6]=S8584[1][3]*(Si8485[1][1]*T18485[4][4] + Si8485[1][2]*T18485[5][4]) + S8584[2][3]*(Si8485[1][1]*T18485[4][5] + Si8485[1][2]*T18485[5][5]);

T8485[5][1]=S8584[1][1]*T18485[6][1] + S8584[2][1]*T18485[6][2];
T8485[5][2]=T18485[6][3];
T8485[5][3]=S8584[1][3]*T18485[6][1] + S8584[2][3]*T18485[6][2];
T8485[5][4]=S8584[1][1]*T18485[6][4] + S8584[2][1]*T18485[6][5];
T8485[5][5]=T18485[6][6];
T8485[5][6]=S8584[1][3]*T18485[6][4] + S8584[2][3]*T18485[6][5];

T8485[6][1]=S8584[1][1]*(Si8485[3][1]*T18485[4][1] + Si8485[3][2]*T18485[5][1]) + S8584[2][1]*(Si8485[3][1]*T18485[4][2] + Si8485[3][2]*T18485[5][2]);
T8485[6][2]=Si8485[3][1]*T18485[4][3] + Si8485[3][2]*T18485[5][3];
T8485[6][3]=S8584[1][3]*(Si8485[3][1]*T18485[4][1] + Si8485[3][2]*T18485[5][1]) + S8584[2][3]*(Si8485[3][1]*T18485[4][2] + Si8485[3][2]*T18485[5][2]);
T8485[6][4]=S8584[1][1]*(Si8485[3][1]*T18485[4][4] + Si8485[3][2]*T18485[5][4]) + S8584[2][1]*(Si8485[3][1]*T18485[4][5] + Si8485[3][2]*T18485[5][5]);
T8485[6][5]=Si8485[3][1]*T18485[4][6] + Si8485[3][2]*T18485[5][6];
T8485[6][6]=S8584[1][3]*(Si8485[3][1]*T18485[4][4] + Si8485[3][2]*T18485[5][4]) + S8584[2][3]*(Si8485[3][1]*T18485[4][5] + Si8485[3][2]*T18485[5][5]);



}


void
hermes_InvDynArtfunc20(void)
      {
JA84[1][1]=T8485[1][1];
JA84[1][2]=links[16].mcm[3] + T8485[1][2];
JA84[1][3]=-links[16].mcm[2] + T8485[1][3];
JA84[1][4]=links[16].m + T8485[1][4];
JA84[1][5]=T8485[1][5];
JA84[1][6]=T8485[1][6];

JA84[2][1]=-links[16].mcm[3] + T8485[2][1];
JA84[2][2]=T8485[2][2];
JA84[2][3]=links[16].mcm[1] + T8485[2][3];
JA84[2][4]=T8485[2][4];
JA84[2][5]=links[16].m + T8485[2][5];
JA84[2][6]=T8485[2][6];

JA84[3][1]=links[16].mcm[2] + T8485[3][1];
JA84[3][2]=-links[16].mcm[1] + T8485[3][2];
JA84[3][3]=T8485[3][3];
JA84[3][4]=T8485[3][4];
JA84[3][5]=T8485[3][5];
JA84[3][6]=links[16].m + T8485[3][6];

JA84[4][1]=links[16].inertia[1][1] + T8485[4][1];
JA84[4][2]=links[16].inertia[1][2] + T8485[4][2];
JA84[4][3]=links[16].inertia[1][3] + T8485[4][3];
JA84[4][4]=T8485[4][4];
JA84[4][5]=-links[16].mcm[3] + T8485[4][5];
JA84[4][6]=links[16].mcm[2] + T8485[4][6];

JA84[5][1]=links[16].inertia[1][2] + T8485[5][1];
JA84[5][2]=links[16].inertia[2][2] + T8485[5][2];
JA84[5][3]=links[16].inertia[2][3] + T8485[5][3];
JA84[5][4]=links[16].mcm[3] + T8485[5][4];
JA84[5][5]=T8485[5][5];
JA84[5][6]=-links[16].mcm[1] + T8485[5][6];

JA84[6][1]=links[16].inertia[1][3] + T8485[6][1];
JA84[6][2]=links[16].inertia[2][3] + T8485[6][2];
JA84[6][3]=links[16].inertia[3][3] + T8485[6][3];
JA84[6][4]=-links[16].mcm[2] + T8485[6][4];
JA84[6][5]=links[16].mcm[1] + T8485[6][5];
JA84[6][6]=T8485[6][6];


h84[1]=JA84[1][3];
h84[2]=JA84[2][3];
h84[3]=JA84[3][3];
h84[4]=JA84[4][3];
h84[5]=JA84[5][3];
h84[6]=JA84[6][3];

T1084[1][1]=JA84[1][1];
T1084[1][2]=JA84[1][2];
T1084[1][3]=JA84[1][3];
T1084[1][4]=JA84[1][4];
T1084[1][5]=JA84[1][5];
T1084[1][6]=JA84[1][6];

T1084[2][1]=JA84[2][1];
T1084[2][2]=JA84[2][2];
T1084[2][3]=JA84[2][3];
T1084[2][4]=JA84[2][4];
T1084[2][5]=JA84[2][5];
T1084[2][6]=JA84[2][6];

T1084[3][1]=JA84[3][1];
T1084[3][2]=JA84[3][2];
T1084[3][3]=JA84[3][3];
T1084[3][4]=JA84[3][4];
T1084[3][5]=JA84[3][5];
T1084[3][6]=JA84[3][6];

T1084[4][1]=JA84[4][1];
T1084[4][2]=JA84[4][2];
T1084[4][3]=JA84[4][3];
T1084[4][4]=JA84[4][4];
T1084[4][5]=JA84[4][5];
T1084[4][6]=JA84[4][6];

T1084[5][1]=JA84[5][1];
T1084[5][2]=JA84[5][2];
T1084[5][3]=JA84[5][3];
T1084[5][4]=JA84[5][4];
T1084[5][5]=JA84[5][5];
T1084[5][6]=JA84[5][6];

T1084[6][1]=JA84[6][1];
T1084[6][2]=JA84[6][2];
T1084[6][3]=JA84[6][3];
T1084[6][4]=JA84[6][4];
T1084[6][5]=JA84[6][5];
T1084[6][6]=JA84[6][6];


T084[1][1]=S840[1][1]*(Si084[1][1]*T1084[1][1] + Si084[1][2]*T1084[2][1]) + S840[2][1]*(Si084[1][1]*T1084[1][2] + Si084[1][2]*T1084[2][2]);
T084[1][2]=-(Si084[1][1]*T1084[1][3]) - Si084[1][2]*T1084[2][3] + XHIP*S840[1][3]*(Si084[1][1]*T1084[1][4] + Si084[1][2]*T1084[2][4]) + XHIP*S840[2][3]*(Si084[1][1]*T1084[1][5] + Si084[1][2]*T1084[2][5]);
T084[1][3]=S840[1][3]*(Si084[1][1]*T1084[1][1] + Si084[1][2]*T1084[2][1]) + S840[2][3]*(Si084[1][1]*T1084[1][2] + Si084[1][2]*T1084[2][2]) + XHIP*(Si084[1][1]*T1084[1][6] + Si084[1][2]*T1084[2][6]);
T084[1][4]=S840[1][1]*(Si084[1][1]*T1084[1][4] + Si084[1][2]*T1084[2][4]) + S840[2][1]*(Si084[1][1]*T1084[1][5] + Si084[1][2]*T1084[2][5]);
T084[1][5]=-(Si084[1][1]*T1084[1][6]) - Si084[1][2]*T1084[2][6];
T084[1][6]=S840[1][3]*(Si084[1][1]*T1084[1][4] + Si084[1][2]*T1084[2][4]) + S840[2][3]*(Si084[1][1]*T1084[1][5] + Si084[1][2]*T1084[2][5]);

T084[2][1]=-(S840[1][1]*T1084[3][1]) - S840[2][1]*T1084[3][2];
T084[2][2]=T1084[3][3] - XHIP*S840[1][3]*T1084[3][4] - XHIP*S840[2][3]*T1084[3][5];
T084[2][3]=-(S840[1][3]*T1084[3][1]) - S840[2][3]*T1084[3][2] - XHIP*T1084[3][6];
T084[2][4]=-(S840[1][1]*T1084[3][4]) - S840[2][1]*T1084[3][5];
T084[2][5]=T1084[3][6];
T084[2][6]=-(S840[1][3]*T1084[3][4]) - S840[2][3]*T1084[3][5];

T084[3][1]=S840[1][1]*(Si084[3][1]*T1084[1][1] + Si084[3][2]*T1084[2][1]) + S840[2][1]*(Si084[3][1]*T1084[1][2] + Si084[3][2]*T1084[2][2]);
T084[3][2]=-(Si084[3][1]*T1084[1][3]) - Si084[3][2]*T1084[2][3] + XHIP*S840[1][3]*(Si084[3][1]*T1084[1][4] + Si084[3][2]*T1084[2][4]) + XHIP*S840[2][3]*(Si084[3][1]*T1084[1][5] + Si084[3][2]*T1084[2][5]);
T084[3][3]=S840[1][3]*(Si084[3][1]*T1084[1][1] + Si084[3][2]*T1084[2][1]) + S840[2][3]*(Si084[3][1]*T1084[1][2] + Si084[3][2]*T1084[2][2]) + XHIP*(Si084[3][1]*T1084[1][6] + Si084[3][2]*T1084[2][6]);
T084[3][4]=S840[1][1]*(Si084[3][1]*T1084[1][4] + Si084[3][2]*T1084[2][4]) + S840[2][1]*(Si084[3][1]*T1084[1][5] + Si084[3][2]*T1084[2][5]);
T084[3][5]=-(Si084[3][1]*T1084[1][6]) - Si084[3][2]*T1084[2][6];
T084[3][6]=S840[1][3]*(Si084[3][1]*T1084[1][4] + Si084[3][2]*T1084[2][4]) + S840[2][3]*(Si084[3][1]*T1084[1][5] + Si084[3][2]*T1084[2][5]);

T084[4][1]=S840[1][1]*(Si084[1][1]*T1084[4][1] + Si084[1][2]*T1084[5][1]) + S840[2][1]*(Si084[1][1]*T1084[4][2] + Si084[1][2]*T1084[5][2]);
T084[4][2]=-(Si084[1][1]*T1084[4][3]) - Si084[1][2]*T1084[5][3] + XHIP*S840[1][3]*(Si084[1][1]*T1084[4][4] + Si084[1][2]*T1084[5][4]) + XHIP*S840[2][3]*(Si084[1][1]*T1084[4][5] + Si084[1][2]*T1084[5][5]);
T084[4][3]=S840[1][3]*(Si084[1][1]*T1084[4][1] + Si084[1][2]*T1084[5][1]) + S840[2][3]*(Si084[1][1]*T1084[4][2] + Si084[1][2]*T1084[5][2]) + XHIP*(Si084[1][1]*T1084[4][6] + Si084[1][2]*T1084[5][6]);
T084[4][4]=S840[1][1]*(Si084[1][1]*T1084[4][4] + Si084[1][2]*T1084[5][4]) + S840[2][1]*(Si084[1][1]*T1084[4][5] + Si084[1][2]*T1084[5][5]);
T084[4][5]=-(Si084[1][1]*T1084[4][6]) - Si084[1][2]*T1084[5][6];
T084[4][6]=S840[1][3]*(Si084[1][1]*T1084[4][4] + Si084[1][2]*T1084[5][4]) + S840[2][3]*(Si084[1][1]*T1084[4][5] + Si084[1][2]*T1084[5][5]);

T084[5][1]=S840[1][1]*(XHIP*Si084[3][1]*T1084[1][1] + XHIP*Si084[3][2]*T1084[2][1] - T1084[6][1]) + S840[2][1]*(XHIP*Si084[3][1]*T1084[1][2] + XHIP*Si084[3][2]*T1084[2][2] - T1084[6][2]);
T084[5][2]=-(XHIP*Si084[3][1]*T1084[1][3]) - XHIP*Si084[3][2]*T1084[2][3] + T1084[6][3] + XHIP*S840[1][3]*(XHIP*Si084[3][1]*T1084[1][4] + XHIP*Si084[3][2]*T1084[2][4] - T1084[6][4]) + XHIP*S840[2][3]*(XHIP*Si084[3][1]*T1084[1][5] + XHIP*Si084[3][2]*T1084[2][5] - T1084[6][5]);
T084[5][3]=S840[1][3]*(XHIP*Si084[3][1]*T1084[1][1] + XHIP*Si084[3][2]*T1084[2][1] - T1084[6][1]) + S840[2][3]*(XHIP*Si084[3][1]*T1084[1][2] + XHIP*Si084[3][2]*T1084[2][2] - T1084[6][2]) + XHIP*(XHIP*Si084[3][1]*T1084[1][6] + XHIP*Si084[3][2]*T1084[2][6] - T1084[6][6]);
T084[5][4]=S840[1][1]*(XHIP*Si084[3][1]*T1084[1][4] + XHIP*Si084[3][2]*T1084[2][4] - T1084[6][4]) + S840[2][1]*(XHIP*Si084[3][1]*T1084[1][5] + XHIP*Si084[3][2]*T1084[2][5] - T1084[6][5]);
T084[5][5]=-(XHIP*Si084[3][1]*T1084[1][6]) - XHIP*Si084[3][2]*T1084[2][6] + T1084[6][6];
T084[5][6]=S840[1][3]*(XHIP*Si084[3][1]*T1084[1][4] + XHIP*Si084[3][2]*T1084[2][4] - T1084[6][4]) + S840[2][3]*(XHIP*Si084[3][1]*T1084[1][5] + XHIP*Si084[3][2]*T1084[2][5] - T1084[6][5]);

T084[6][1]=S840[1][1]*(XHIP*T1084[3][1] + Si084[3][1]*T1084[4][1] + Si084[3][2]*T1084[5][1]) + S840[2][1]*(XHIP*T1084[3][2] + Si084[3][1]*T1084[4][2] + Si084[3][2]*T1084[5][2]);
T084[6][2]=-(XHIP*T1084[3][3]) - Si084[3][1]*T1084[4][3] - Si084[3][2]*T1084[5][3] + XHIP*S840[1][3]*(XHIP*T1084[3][4] + Si084[3][1]*T1084[4][4] + Si084[3][2]*T1084[5][4]) + XHIP*S840[2][3]*(XHIP*T1084[3][5] + Si084[3][1]*T1084[4][5] + Si084[3][2]*T1084[5][5]);
T084[6][3]=S840[1][3]*(XHIP*T1084[3][1] + Si084[3][1]*T1084[4][1] + Si084[3][2]*T1084[5][1]) + S840[2][3]*(XHIP*T1084[3][2] + Si084[3][1]*T1084[4][2] + Si084[3][2]*T1084[5][2]) + XHIP*(XHIP*T1084[3][6] + Si084[3][1]*T1084[4][6] + Si084[3][2]*T1084[5][6]);
T084[6][4]=S840[1][1]*(XHIP*T1084[3][4] + Si084[3][1]*T1084[4][4] + Si084[3][2]*T1084[5][4]) + S840[2][1]*(XHIP*T1084[3][5] + Si084[3][1]*T1084[4][5] + Si084[3][2]*T1084[5][5]);
T084[6][5]=-(XHIP*T1084[3][6]) - Si084[3][1]*T1084[4][6] - Si084[3][2]*T1084[5][6];
T084[6][6]=S840[1][3]*(XHIP*T1084[3][4] + Si084[3][1]*T1084[4][4] + Si084[3][2]*T1084[5][4]) + S840[2][3]*(XHIP*T1084[3][5] + Si084[3][1]*T1084[4][5] + Si084[3][2]*T1084[5][5]);



}


void
hermes_InvDynArtfunc21(void)
      {
JA83[1][2]=eff[3].mcm[3];
JA83[1][3]=-eff[3].mcm[2];
JA83[1][4]=eff[3].m;

JA83[2][1]=-eff[3].mcm[3];
JA83[2][3]=eff[3].mcm[1];
JA83[2][5]=eff[3].m;

JA83[3][1]=eff[3].mcm[2];
JA83[3][2]=-eff[3].mcm[1];
JA83[3][6]=eff[3].m;

JA83[4][5]=-eff[3].mcm[3];
JA83[4][6]=eff[3].mcm[2];

JA83[5][4]=eff[3].mcm[3];
JA83[5][6]=-eff[3].mcm[1];

JA83[6][4]=-eff[3].mcm[2];
JA83[6][5]=eff[3].mcm[1];


T17683[1][2]=JA83[1][2];
T17683[1][3]=JA83[1][3];
T17683[1][4]=JA83[1][4];

T17683[2][1]=JA83[2][1];
T17683[2][3]=JA83[2][3];
T17683[2][5]=JA83[2][5];

T17683[3][1]=JA83[3][1];
T17683[3][2]=JA83[3][2];
T17683[3][6]=JA83[3][6];

T17683[4][5]=JA83[4][5];
T17683[4][6]=JA83[4][6];

T17683[5][4]=JA83[5][4];
T17683[5][6]=JA83[5][6];

T17683[6][4]=JA83[6][4];
T17683[6][5]=JA83[6][5];


T7683[1][1]=(-(eff[3].x[3]*S8376[1][2]) + eff[3].x[2]*S8376[1][3])*Si7683[1][1]*T17683[1][4] + S8376[3][1]*(Si7683[1][1]*T17683[1][3] + Si7683[1][2]*T17683[2][3]) + (-(eff[3].x[3]*S8376[2][2]) + eff[3].x[2]*S8376[2][3])*Si7683[1][2]*T17683[2][5] + S8376[1][1]*(Si7683[1][2]*T17683[2][1] + Si7683[1][3]*T17683[3][1]) + S8376[2][1]*(Si7683[1][1]*T17683[1][2] + Si7683[1][3]*T17683[3][2]) + (-(eff[3].x[3]*S8376[3][2]) + eff[3].x[2]*S8376[3][3])*Si7683[1][3]*T17683[3][6];
T7683[1][2]=(eff[3].x[3]*S8376[1][1] - eff[3].x[1]*S8376[1][3])*Si7683[1][1]*T17683[1][4] + S8376[3][2]*(Si7683[1][1]*T17683[1][3] + Si7683[1][2]*T17683[2][3]) + (eff[3].x[3]*S8376[2][1] - eff[3].x[1]*S8376[2][3])*Si7683[1][2]*T17683[2][5] + S8376[1][2]*(Si7683[1][2]*T17683[2][1] + Si7683[1][3]*T17683[3][1]) + S8376[2][2]*(Si7683[1][1]*T17683[1][2] + Si7683[1][3]*T17683[3][2]) + (eff[3].x[3]*S8376[3][1] - eff[3].x[1]*S8376[3][3])*Si7683[1][3]*T17683[3][6];
T7683[1][3]=(-(eff[3].x[2]*S8376[1][1]) + eff[3].x[1]*S8376[1][2])*Si7683[1][1]*T17683[1][4] + S8376[3][3]*(Si7683[1][1]*T17683[1][3] + Si7683[1][2]*T17683[2][3]) + (-(eff[3].x[2]*S8376[2][1]) + eff[3].x[1]*S8376[2][2])*Si7683[1][2]*T17683[2][5] + S8376[1][3]*(Si7683[1][2]*T17683[2][1] + Si7683[1][3]*T17683[3][1]) + S8376[2][3]*(Si7683[1][1]*T17683[1][2] + Si7683[1][3]*T17683[3][2]) + (-(eff[3].x[2]*S8376[3][1]) + eff[3].x[1]*S8376[3][2])*Si7683[1][3]*T17683[3][6];
T7683[1][4]=S8376[1][1]*Si7683[1][1]*T17683[1][4] + S8376[2][1]*Si7683[1][2]*T17683[2][5] + S8376[3][1]*Si7683[1][3]*T17683[3][6];
T7683[1][5]=S8376[1][2]*Si7683[1][1]*T17683[1][4] + S8376[2][2]*Si7683[1][2]*T17683[2][5] + S8376[3][2]*Si7683[1][3]*T17683[3][6];
T7683[1][6]=S8376[1][3]*Si7683[1][1]*T17683[1][4] + S8376[2][3]*Si7683[1][2]*T17683[2][5] + S8376[3][3]*Si7683[1][3]*T17683[3][6];

T7683[2][1]=(-(eff[3].x[3]*S8376[1][2]) + eff[3].x[2]*S8376[1][3])*Si7683[2][1]*T17683[1][4] + S8376[3][1]*(Si7683[2][1]*T17683[1][3] + Si7683[2][2]*T17683[2][3]) + (-(eff[3].x[3]*S8376[2][2]) + eff[3].x[2]*S8376[2][3])*Si7683[2][2]*T17683[2][5] + S8376[1][1]*(Si7683[2][2]*T17683[2][1] + Si7683[2][3]*T17683[3][1]) + S8376[2][1]*(Si7683[2][1]*T17683[1][2] + Si7683[2][3]*T17683[3][2]) + (-(eff[3].x[3]*S8376[3][2]) + eff[3].x[2]*S8376[3][3])*Si7683[2][3]*T17683[3][6];
T7683[2][2]=(eff[3].x[3]*S8376[1][1] - eff[3].x[1]*S8376[1][3])*Si7683[2][1]*T17683[1][4] + S8376[3][2]*(Si7683[2][1]*T17683[1][3] + Si7683[2][2]*T17683[2][3]) + (eff[3].x[3]*S8376[2][1] - eff[3].x[1]*S8376[2][3])*Si7683[2][2]*T17683[2][5] + S8376[1][2]*(Si7683[2][2]*T17683[2][1] + Si7683[2][3]*T17683[3][1]) + S8376[2][2]*(Si7683[2][1]*T17683[1][2] + Si7683[2][3]*T17683[3][2]) + (eff[3].x[3]*S8376[3][1] - eff[3].x[1]*S8376[3][3])*Si7683[2][3]*T17683[3][6];
T7683[2][3]=(-(eff[3].x[2]*S8376[1][1]) + eff[3].x[1]*S8376[1][2])*Si7683[2][1]*T17683[1][4] + S8376[3][3]*(Si7683[2][1]*T17683[1][3] + Si7683[2][2]*T17683[2][3]) + (-(eff[3].x[2]*S8376[2][1]) + eff[3].x[1]*S8376[2][2])*Si7683[2][2]*T17683[2][5] + S8376[1][3]*(Si7683[2][2]*T17683[2][1] + Si7683[2][3]*T17683[3][1]) + S8376[2][3]*(Si7683[2][1]*T17683[1][2] + Si7683[2][3]*T17683[3][2]) + (-(eff[3].x[2]*S8376[3][1]) + eff[3].x[1]*S8376[3][2])*Si7683[2][3]*T17683[3][6];
T7683[2][4]=S8376[1][1]*Si7683[2][1]*T17683[1][4] + S8376[2][1]*Si7683[2][2]*T17683[2][5] + S8376[3][1]*Si7683[2][3]*T17683[3][6];
T7683[2][5]=S8376[1][2]*Si7683[2][1]*T17683[1][4] + S8376[2][2]*Si7683[2][2]*T17683[2][5] + S8376[3][2]*Si7683[2][3]*T17683[3][6];
T7683[2][6]=S8376[1][3]*Si7683[2][1]*T17683[1][4] + S8376[2][3]*Si7683[2][2]*T17683[2][5] + S8376[3][3]*Si7683[2][3]*T17683[3][6];

T7683[3][1]=(-(eff[3].x[3]*S8376[1][2]) + eff[3].x[2]*S8376[1][3])*Si7683[3][1]*T17683[1][4] + S8376[3][1]*(Si7683[3][1]*T17683[1][3] + Si7683[3][2]*T17683[2][3]) + (-(eff[3].x[3]*S8376[2][2]) + eff[3].x[2]*S8376[2][3])*Si7683[3][2]*T17683[2][5] + S8376[1][1]*(Si7683[3][2]*T17683[2][1] + Si7683[3][3]*T17683[3][1]) + S8376[2][1]*(Si7683[3][1]*T17683[1][2] + Si7683[3][3]*T17683[3][2]) + (-(eff[3].x[3]*S8376[3][2]) + eff[3].x[2]*S8376[3][3])*Si7683[3][3]*T17683[3][6];
T7683[3][2]=(eff[3].x[3]*S8376[1][1] - eff[3].x[1]*S8376[1][3])*Si7683[3][1]*T17683[1][4] + S8376[3][2]*(Si7683[3][1]*T17683[1][3] + Si7683[3][2]*T17683[2][3]) + (eff[3].x[3]*S8376[2][1] - eff[3].x[1]*S8376[2][3])*Si7683[3][2]*T17683[2][5] + S8376[1][2]*(Si7683[3][2]*T17683[2][1] + Si7683[3][3]*T17683[3][1]) + S8376[2][2]*(Si7683[3][1]*T17683[1][2] + Si7683[3][3]*T17683[3][2]) + (eff[3].x[3]*S8376[3][1] - eff[3].x[1]*S8376[3][3])*Si7683[3][3]*T17683[3][6];
T7683[3][3]=(-(eff[3].x[2]*S8376[1][1]) + eff[3].x[1]*S8376[1][2])*Si7683[3][1]*T17683[1][4] + S8376[3][3]*(Si7683[3][1]*T17683[1][3] + Si7683[3][2]*T17683[2][3]) + (-(eff[3].x[2]*S8376[2][1]) + eff[3].x[1]*S8376[2][2])*Si7683[3][2]*T17683[2][5] + S8376[1][3]*(Si7683[3][2]*T17683[2][1] + Si7683[3][3]*T17683[3][1]) + S8376[2][3]*(Si7683[3][1]*T17683[1][2] + Si7683[3][3]*T17683[3][2]) + (-(eff[3].x[2]*S8376[3][1]) + eff[3].x[1]*S8376[3][2])*Si7683[3][3]*T17683[3][6];
T7683[3][4]=S8376[1][1]*Si7683[3][1]*T17683[1][4] + S8376[2][1]*Si7683[3][2]*T17683[2][5] + S8376[3][1]*Si7683[3][3]*T17683[3][6];
T7683[3][5]=S8376[1][2]*Si7683[3][1]*T17683[1][4] + S8376[2][2]*Si7683[3][2]*T17683[2][5] + S8376[3][2]*Si7683[3][3]*T17683[3][6];
T7683[3][6]=S8376[1][3]*Si7683[3][1]*T17683[1][4] + S8376[2][3]*Si7683[3][2]*T17683[2][5] + S8376[3][3]*Si7683[3][3]*T17683[3][6];

T7683[4][1]=S8376[3][1]*((-(eff[3].x[3]*Si7683[2][1]) + eff[3].x[2]*Si7683[3][1])*T17683[1][3] + (-(eff[3].x[3]*Si7683[2][2]) + eff[3].x[2]*Si7683[3][2])*T17683[2][3]) + S8376[1][1]*((-(eff[3].x[3]*Si7683[2][2]) + eff[3].x[2]*Si7683[3][2])*T17683[2][1] + (-(eff[3].x[3]*Si7683[2][3]) + eff[3].x[2]*Si7683[3][3])*T17683[3][1]) + S8376[2][1]*((-(eff[3].x[3]*Si7683[2][1]) + eff[3].x[2]*Si7683[3][1])*T17683[1][2] + (-(eff[3].x[3]*Si7683[2][3]) + eff[3].x[2]*Si7683[3][3])*T17683[3][2]) + (-(eff[3].x[3]*S8376[3][2]) + eff[3].x[2]*S8376[3][3])*((-(eff[3].x[3]*Si7683[2][3]) + eff[3].x[2]*Si7683[3][3])*T17683[3][6] + Si7683[1][1]*T17683[4][6] + Si7683[1][2]*T17683[5][6]) + (-(eff[3].x[3]*S8376[1][2]) + eff[3].x[2]*S8376[1][3])*((-(eff[3].x[3]*Si7683[2][1]) + eff[3].x[2]*Si7683[3][1])*T17683[1][4] + Si7683[1][2]*T17683[5][4] + Si7683[1][3]*T17683[6][4]) + (-(eff[3].x[3]*S8376[2][2]) + eff[3].x[2]*S8376[2][3])*((-(eff[3].x[3]*Si7683[2][2]) + eff[3].x[2]*Si7683[3][2])*T17683[2][5] + Si7683[1][1]*T17683[4][5] + Si7683[1][3]*T17683[6][5]);
T7683[4][2]=S8376[3][2]*((-(eff[3].x[3]*Si7683[2][1]) + eff[3].x[2]*Si7683[3][1])*T17683[1][3] + (-(eff[3].x[3]*Si7683[2][2]) + eff[3].x[2]*Si7683[3][2])*T17683[2][3]) + S8376[1][2]*((-(eff[3].x[3]*Si7683[2][2]) + eff[3].x[2]*Si7683[3][2])*T17683[2][1] + (-(eff[3].x[3]*Si7683[2][3]) + eff[3].x[2]*Si7683[3][3])*T17683[3][1]) + S8376[2][2]*((-(eff[3].x[3]*Si7683[2][1]) + eff[3].x[2]*Si7683[3][1])*T17683[1][2] + (-(eff[3].x[3]*Si7683[2][3]) + eff[3].x[2]*Si7683[3][3])*T17683[3][2]) + (eff[3].x[3]*S8376[3][1] - eff[3].x[1]*S8376[3][3])*((-(eff[3].x[3]*Si7683[2][3]) + eff[3].x[2]*Si7683[3][3])*T17683[3][6] + Si7683[1][1]*T17683[4][6] + Si7683[1][2]*T17683[5][6]) + (eff[3].x[3]*S8376[1][1] - eff[3].x[1]*S8376[1][3])*((-(eff[3].x[3]*Si7683[2][1]) + eff[3].x[2]*Si7683[3][1])*T17683[1][4] + Si7683[1][2]*T17683[5][4] + Si7683[1][3]*T17683[6][4]) + (eff[3].x[3]*S8376[2][1] - eff[3].x[1]*S8376[2][3])*((-(eff[3].x[3]*Si7683[2][2]) + eff[3].x[2]*Si7683[3][2])*T17683[2][5] + Si7683[1][1]*T17683[4][5] + Si7683[1][3]*T17683[6][5]);
T7683[4][3]=S8376[3][3]*((-(eff[3].x[3]*Si7683[2][1]) + eff[3].x[2]*Si7683[3][1])*T17683[1][3] + (-(eff[3].x[3]*Si7683[2][2]) + eff[3].x[2]*Si7683[3][2])*T17683[2][3]) + S8376[1][3]*((-(eff[3].x[3]*Si7683[2][2]) + eff[3].x[2]*Si7683[3][2])*T17683[2][1] + (-(eff[3].x[3]*Si7683[2][3]) + eff[3].x[2]*Si7683[3][3])*T17683[3][1]) + S8376[2][3]*((-(eff[3].x[3]*Si7683[2][1]) + eff[3].x[2]*Si7683[3][1])*T17683[1][2] + (-(eff[3].x[3]*Si7683[2][3]) + eff[3].x[2]*Si7683[3][3])*T17683[3][2]) + (-(eff[3].x[2]*S8376[3][1]) + eff[3].x[1]*S8376[3][2])*((-(eff[3].x[3]*Si7683[2][3]) + eff[3].x[2]*Si7683[3][3])*T17683[3][6] + Si7683[1][1]*T17683[4][6] + Si7683[1][2]*T17683[5][6]) + (-(eff[3].x[2]*S8376[1][1]) + eff[3].x[1]*S8376[1][2])*((-(eff[3].x[3]*Si7683[2][1]) + eff[3].x[2]*Si7683[3][1])*T17683[1][4] + Si7683[1][2]*T17683[5][4] + Si7683[1][3]*T17683[6][4]) + (-(eff[3].x[2]*S8376[2][1]) + eff[3].x[1]*S8376[2][2])*((-(eff[3].x[3]*Si7683[2][2]) + eff[3].x[2]*Si7683[3][2])*T17683[2][5] + Si7683[1][1]*T17683[4][5] + Si7683[1][3]*T17683[6][5]);
T7683[4][4]=S8376[3][1]*((-(eff[3].x[3]*Si7683[2][3]) + eff[3].x[2]*Si7683[3][3])*T17683[3][6] + Si7683[1][1]*T17683[4][6] + Si7683[1][2]*T17683[5][6]) + S8376[1][1]*((-(eff[3].x[3]*Si7683[2][1]) + eff[3].x[2]*Si7683[3][1])*T17683[1][4] + Si7683[1][2]*T17683[5][4] + Si7683[1][3]*T17683[6][4]) + S8376[2][1]*((-(eff[3].x[3]*Si7683[2][2]) + eff[3].x[2]*Si7683[3][2])*T17683[2][5] + Si7683[1][1]*T17683[4][5] + Si7683[1][3]*T17683[6][5]);
T7683[4][5]=S8376[3][2]*((-(eff[3].x[3]*Si7683[2][3]) + eff[3].x[2]*Si7683[3][3])*T17683[3][6] + Si7683[1][1]*T17683[4][6] + Si7683[1][2]*T17683[5][6]) + S8376[1][2]*((-(eff[3].x[3]*Si7683[2][1]) + eff[3].x[2]*Si7683[3][1])*T17683[1][4] + Si7683[1][2]*T17683[5][4] + Si7683[1][3]*T17683[6][4]) + S8376[2][2]*((-(eff[3].x[3]*Si7683[2][2]) + eff[3].x[2]*Si7683[3][2])*T17683[2][5] + Si7683[1][1]*T17683[4][5] + Si7683[1][3]*T17683[6][5]);
T7683[4][6]=S8376[3][3]*((-(eff[3].x[3]*Si7683[2][3]) + eff[3].x[2]*Si7683[3][3])*T17683[3][6] + Si7683[1][1]*T17683[4][6] + Si7683[1][2]*T17683[5][6]) + S8376[1][3]*((-(eff[3].x[3]*Si7683[2][1]) + eff[3].x[2]*Si7683[3][1])*T17683[1][4] + Si7683[1][2]*T17683[5][4] + Si7683[1][3]*T17683[6][4]) + S8376[2][3]*((-(eff[3].x[3]*Si7683[2][2]) + eff[3].x[2]*Si7683[3][2])*T17683[2][5] + Si7683[1][1]*T17683[4][5] + Si7683[1][3]*T17683[6][5]);

T7683[5][1]=S8376[3][1]*((eff[3].x[3]*Si7683[1][1] - eff[3].x[1]*Si7683[3][1])*T17683[1][3] + (eff[3].x[3]*Si7683[1][2] - eff[3].x[1]*Si7683[3][2])*T17683[2][3]) + S8376[1][1]*((eff[3].x[3]*Si7683[1][2] - eff[3].x[1]*Si7683[3][2])*T17683[2][1] + (eff[3].x[3]*Si7683[1][3] - eff[3].x[1]*Si7683[3][3])*T17683[3][1]) + S8376[2][1]*((eff[3].x[3]*Si7683[1][1] - eff[3].x[1]*Si7683[3][1])*T17683[1][2] + (eff[3].x[3]*Si7683[1][3] - eff[3].x[1]*Si7683[3][3])*T17683[3][2]) + (-(eff[3].x[3]*S8376[3][2]) + eff[3].x[2]*S8376[3][3])*((eff[3].x[3]*Si7683[1][3] - eff[3].x[1]*Si7683[3][3])*T17683[3][6] + Si7683[2][1]*T17683[4][6] + Si7683[2][2]*T17683[5][6]) + (-(eff[3].x[3]*S8376[1][2]) + eff[3].x[2]*S8376[1][3])*((eff[3].x[3]*Si7683[1][1] - eff[3].x[1]*Si7683[3][1])*T17683[1][4] + Si7683[2][2]*T17683[5][4] + Si7683[2][3]*T17683[6][4]) + (-(eff[3].x[3]*S8376[2][2]) + eff[3].x[2]*S8376[2][3])*((eff[3].x[3]*Si7683[1][2] - eff[3].x[1]*Si7683[3][2])*T17683[2][5] + Si7683[2][1]*T17683[4][5] + Si7683[2][3]*T17683[6][5]);
T7683[5][2]=S8376[3][2]*((eff[3].x[3]*Si7683[1][1] - eff[3].x[1]*Si7683[3][1])*T17683[1][3] + (eff[3].x[3]*Si7683[1][2] - eff[3].x[1]*Si7683[3][2])*T17683[2][3]) + S8376[1][2]*((eff[3].x[3]*Si7683[1][2] - eff[3].x[1]*Si7683[3][2])*T17683[2][1] + (eff[3].x[3]*Si7683[1][3] - eff[3].x[1]*Si7683[3][3])*T17683[3][1]) + S8376[2][2]*((eff[3].x[3]*Si7683[1][1] - eff[3].x[1]*Si7683[3][1])*T17683[1][2] + (eff[3].x[3]*Si7683[1][3] - eff[3].x[1]*Si7683[3][3])*T17683[3][2]) + (eff[3].x[3]*S8376[3][1] - eff[3].x[1]*S8376[3][3])*((eff[3].x[3]*Si7683[1][3] - eff[3].x[1]*Si7683[3][3])*T17683[3][6] + Si7683[2][1]*T17683[4][6] + Si7683[2][2]*T17683[5][6]) + (eff[3].x[3]*S8376[1][1] - eff[3].x[1]*S8376[1][3])*((eff[3].x[3]*Si7683[1][1] - eff[3].x[1]*Si7683[3][1])*T17683[1][4] + Si7683[2][2]*T17683[5][4] + Si7683[2][3]*T17683[6][4]) + (eff[3].x[3]*S8376[2][1] - eff[3].x[1]*S8376[2][3])*((eff[3].x[3]*Si7683[1][2] - eff[3].x[1]*Si7683[3][2])*T17683[2][5] + Si7683[2][1]*T17683[4][5] + Si7683[2][3]*T17683[6][5]);
T7683[5][3]=S8376[3][3]*((eff[3].x[3]*Si7683[1][1] - eff[3].x[1]*Si7683[3][1])*T17683[1][3] + (eff[3].x[3]*Si7683[1][2] - eff[3].x[1]*Si7683[3][2])*T17683[2][3]) + S8376[1][3]*((eff[3].x[3]*Si7683[1][2] - eff[3].x[1]*Si7683[3][2])*T17683[2][1] + (eff[3].x[3]*Si7683[1][3] - eff[3].x[1]*Si7683[3][3])*T17683[3][1]) + S8376[2][3]*((eff[3].x[3]*Si7683[1][1] - eff[3].x[1]*Si7683[3][1])*T17683[1][2] + (eff[3].x[3]*Si7683[1][3] - eff[3].x[1]*Si7683[3][3])*T17683[3][2]) + (-(eff[3].x[2]*S8376[3][1]) + eff[3].x[1]*S8376[3][2])*((eff[3].x[3]*Si7683[1][3] - eff[3].x[1]*Si7683[3][3])*T17683[3][6] + Si7683[2][1]*T17683[4][6] + Si7683[2][2]*T17683[5][6]) + (-(eff[3].x[2]*S8376[1][1]) + eff[3].x[1]*S8376[1][2])*((eff[3].x[3]*Si7683[1][1] - eff[3].x[1]*Si7683[3][1])*T17683[1][4] + Si7683[2][2]*T17683[5][4] + Si7683[2][3]*T17683[6][4]) + (-(eff[3].x[2]*S8376[2][1]) + eff[3].x[1]*S8376[2][2])*((eff[3].x[3]*Si7683[1][2] - eff[3].x[1]*Si7683[3][2])*T17683[2][5] + Si7683[2][1]*T17683[4][5] + Si7683[2][3]*T17683[6][5]);
T7683[5][4]=S8376[3][1]*((eff[3].x[3]*Si7683[1][3] - eff[3].x[1]*Si7683[3][3])*T17683[3][6] + Si7683[2][1]*T17683[4][6] + Si7683[2][2]*T17683[5][6]) + S8376[1][1]*((eff[3].x[3]*Si7683[1][1] - eff[3].x[1]*Si7683[3][1])*T17683[1][4] + Si7683[2][2]*T17683[5][4] + Si7683[2][3]*T17683[6][4]) + S8376[2][1]*((eff[3].x[3]*Si7683[1][2] - eff[3].x[1]*Si7683[3][2])*T17683[2][5] + Si7683[2][1]*T17683[4][5] + Si7683[2][3]*T17683[6][5]);
T7683[5][5]=S8376[3][2]*((eff[3].x[3]*Si7683[1][3] - eff[3].x[1]*Si7683[3][3])*T17683[3][6] + Si7683[2][1]*T17683[4][6] + Si7683[2][2]*T17683[5][6]) + S8376[1][2]*((eff[3].x[3]*Si7683[1][1] - eff[3].x[1]*Si7683[3][1])*T17683[1][4] + Si7683[2][2]*T17683[5][4] + Si7683[2][3]*T17683[6][4]) + S8376[2][2]*((eff[3].x[3]*Si7683[1][2] - eff[3].x[1]*Si7683[3][2])*T17683[2][5] + Si7683[2][1]*T17683[4][5] + Si7683[2][3]*T17683[6][5]);
T7683[5][6]=S8376[3][3]*((eff[3].x[3]*Si7683[1][3] - eff[3].x[1]*Si7683[3][3])*T17683[3][6] + Si7683[2][1]*T17683[4][6] + Si7683[2][2]*T17683[5][6]) + S8376[1][3]*((eff[3].x[3]*Si7683[1][1] - eff[3].x[1]*Si7683[3][1])*T17683[1][4] + Si7683[2][2]*T17683[5][4] + Si7683[2][3]*T17683[6][4]) + S8376[2][3]*((eff[3].x[3]*Si7683[1][2] - eff[3].x[1]*Si7683[3][2])*T17683[2][5] + Si7683[2][1]*T17683[4][5] + Si7683[2][3]*T17683[6][5]);

T7683[6][1]=S8376[3][1]*((-(eff[3].x[2]*Si7683[1][1]) + eff[3].x[1]*Si7683[2][1])*T17683[1][3] + (-(eff[3].x[2]*Si7683[1][2]) + eff[3].x[1]*Si7683[2][2])*T17683[2][3]) + S8376[1][1]*((-(eff[3].x[2]*Si7683[1][2]) + eff[3].x[1]*Si7683[2][2])*T17683[2][1] + (-(eff[3].x[2]*Si7683[1][3]) + eff[3].x[1]*Si7683[2][3])*T17683[3][1]) + S8376[2][1]*((-(eff[3].x[2]*Si7683[1][1]) + eff[3].x[1]*Si7683[2][1])*T17683[1][2] + (-(eff[3].x[2]*Si7683[1][3]) + eff[3].x[1]*Si7683[2][3])*T17683[3][2]) + (-(eff[3].x[3]*S8376[3][2]) + eff[3].x[2]*S8376[3][3])*((-(eff[3].x[2]*Si7683[1][3]) + eff[3].x[1]*Si7683[2][3])*T17683[3][6] + Si7683[3][1]*T17683[4][6] + Si7683[3][2]*T17683[5][6]) + (-(eff[3].x[3]*S8376[1][2]) + eff[3].x[2]*S8376[1][3])*((-(eff[3].x[2]*Si7683[1][1]) + eff[3].x[1]*Si7683[2][1])*T17683[1][4] + Si7683[3][2]*T17683[5][4] + Si7683[3][3]*T17683[6][4]) + (-(eff[3].x[3]*S8376[2][2]) + eff[3].x[2]*S8376[2][3])*((-(eff[3].x[2]*Si7683[1][2]) + eff[3].x[1]*Si7683[2][2])*T17683[2][5] + Si7683[3][1]*T17683[4][5] + Si7683[3][3]*T17683[6][5]);
T7683[6][2]=S8376[3][2]*((-(eff[3].x[2]*Si7683[1][1]) + eff[3].x[1]*Si7683[2][1])*T17683[1][3] + (-(eff[3].x[2]*Si7683[1][2]) + eff[3].x[1]*Si7683[2][2])*T17683[2][3]) + S8376[1][2]*((-(eff[3].x[2]*Si7683[1][2]) + eff[3].x[1]*Si7683[2][2])*T17683[2][1] + (-(eff[3].x[2]*Si7683[1][3]) + eff[3].x[1]*Si7683[2][3])*T17683[3][1]) + S8376[2][2]*((-(eff[3].x[2]*Si7683[1][1]) + eff[3].x[1]*Si7683[2][1])*T17683[1][2] + (-(eff[3].x[2]*Si7683[1][3]) + eff[3].x[1]*Si7683[2][3])*T17683[3][2]) + (eff[3].x[3]*S8376[3][1] - eff[3].x[1]*S8376[3][3])*((-(eff[3].x[2]*Si7683[1][3]) + eff[3].x[1]*Si7683[2][3])*T17683[3][6] + Si7683[3][1]*T17683[4][6] + Si7683[3][2]*T17683[5][6]) + (eff[3].x[3]*S8376[1][1] - eff[3].x[1]*S8376[1][3])*((-(eff[3].x[2]*Si7683[1][1]) + eff[3].x[1]*Si7683[2][1])*T17683[1][4] + Si7683[3][2]*T17683[5][4] + Si7683[3][3]*T17683[6][4]) + (eff[3].x[3]*S8376[2][1] - eff[3].x[1]*S8376[2][3])*((-(eff[3].x[2]*Si7683[1][2]) + eff[3].x[1]*Si7683[2][2])*T17683[2][5] + Si7683[3][1]*T17683[4][5] + Si7683[3][3]*T17683[6][5]);
T7683[6][3]=S8376[3][3]*((-(eff[3].x[2]*Si7683[1][1]) + eff[3].x[1]*Si7683[2][1])*T17683[1][3] + (-(eff[3].x[2]*Si7683[1][2]) + eff[3].x[1]*Si7683[2][2])*T17683[2][3]) + S8376[1][3]*((-(eff[3].x[2]*Si7683[1][2]) + eff[3].x[1]*Si7683[2][2])*T17683[2][1] + (-(eff[3].x[2]*Si7683[1][3]) + eff[3].x[1]*Si7683[2][3])*T17683[3][1]) + S8376[2][3]*((-(eff[3].x[2]*Si7683[1][1]) + eff[3].x[1]*Si7683[2][1])*T17683[1][2] + (-(eff[3].x[2]*Si7683[1][3]) + eff[3].x[1]*Si7683[2][3])*T17683[3][2]) + (-(eff[3].x[2]*S8376[3][1]) + eff[3].x[1]*S8376[3][2])*((-(eff[3].x[2]*Si7683[1][3]) + eff[3].x[1]*Si7683[2][3])*T17683[3][6] + Si7683[3][1]*T17683[4][6] + Si7683[3][2]*T17683[5][6]) + (-(eff[3].x[2]*S8376[1][1]) + eff[3].x[1]*S8376[1][2])*((-(eff[3].x[2]*Si7683[1][1]) + eff[3].x[1]*Si7683[2][1])*T17683[1][4] + Si7683[3][2]*T17683[5][4] + Si7683[3][3]*T17683[6][4]) + (-(eff[3].x[2]*S8376[2][1]) + eff[3].x[1]*S8376[2][2])*((-(eff[3].x[2]*Si7683[1][2]) + eff[3].x[1]*Si7683[2][2])*T17683[2][5] + Si7683[3][1]*T17683[4][5] + Si7683[3][3]*T17683[6][5]);
T7683[6][4]=S8376[3][1]*((-(eff[3].x[2]*Si7683[1][3]) + eff[3].x[1]*Si7683[2][3])*T17683[3][6] + Si7683[3][1]*T17683[4][6] + Si7683[3][2]*T17683[5][6]) + S8376[1][1]*((-(eff[3].x[2]*Si7683[1][1]) + eff[3].x[1]*Si7683[2][1])*T17683[1][4] + Si7683[3][2]*T17683[5][4] + Si7683[3][3]*T17683[6][4]) + S8376[2][1]*((-(eff[3].x[2]*Si7683[1][2]) + eff[3].x[1]*Si7683[2][2])*T17683[2][5] + Si7683[3][1]*T17683[4][5] + Si7683[3][3]*T17683[6][5]);
T7683[6][5]=S8376[3][2]*((-(eff[3].x[2]*Si7683[1][3]) + eff[3].x[1]*Si7683[2][3])*T17683[3][6] + Si7683[3][1]*T17683[4][6] + Si7683[3][2]*T17683[5][6]) + S8376[1][2]*((-(eff[3].x[2]*Si7683[1][1]) + eff[3].x[1]*Si7683[2][1])*T17683[1][4] + Si7683[3][2]*T17683[5][4] + Si7683[3][3]*T17683[6][4]) + S8376[2][2]*((-(eff[3].x[2]*Si7683[1][2]) + eff[3].x[1]*Si7683[2][2])*T17683[2][5] + Si7683[3][1]*T17683[4][5] + Si7683[3][3]*T17683[6][5]);
T7683[6][6]=S8376[3][3]*((-(eff[3].x[2]*Si7683[1][3]) + eff[3].x[1]*Si7683[2][3])*T17683[3][6] + Si7683[3][1]*T17683[4][6] + Si7683[3][2]*T17683[5][6]) + S8376[1][3]*((-(eff[3].x[2]*Si7683[1][1]) + eff[3].x[1]*Si7683[2][1])*T17683[1][4] + Si7683[3][2]*T17683[5][4] + Si7683[3][3]*T17683[6][4]) + S8376[2][3]*((-(eff[3].x[2]*Si7683[1][2]) + eff[3].x[1]*Si7683[2][2])*T17683[2][5] + Si7683[3][1]*T17683[4][5] + Si7683[3][3]*T17683[6][5]);



}


void
hermes_InvDynArtfunc22(void)
      {




}


void
hermes_InvDynArtfunc23(void)
      {




}


void
hermes_InvDynArtfunc24(void)
      {




}


void
hermes_InvDynArtfunc25(void)
      {




}


void
hermes_InvDynArtfunc26(void)
      {




}


void
hermes_InvDynArtfunc27(void)
      {




}


void
hermes_InvDynArtfunc28(void)
      {
JA76[1][1]=0. + T7683[1][1];
JA76[1][2]=0. + links[28].mcm[3] + T7683[1][2];
JA76[1][3]=0. - links[28].mcm[2] + T7683[1][3];
JA76[1][4]=0. + links[28].m + T7683[1][4];
JA76[1][5]=0. + T7683[1][5];
JA76[1][6]=0. + T7683[1][6];

JA76[2][1]=0. - links[28].mcm[3] + T7683[2][1];
JA76[2][2]=0. + T7683[2][2];
JA76[2][3]=0. + links[28].mcm[1] + T7683[2][3];
JA76[2][4]=0. + T7683[2][4];
JA76[2][5]=0. + links[28].m + T7683[2][5];
JA76[2][6]=0. + T7683[2][6];

JA76[3][1]=0. + links[28].mcm[2] + T7683[3][1];
JA76[3][2]=0. - links[28].mcm[1] + T7683[3][2];
JA76[3][3]=0. + T7683[3][3];
JA76[3][4]=0. + T7683[3][4];
JA76[3][5]=0. + T7683[3][5];
JA76[3][6]=0. + links[28].m + T7683[3][6];

JA76[4][1]=0. + links[28].inertia[1][1] + T7683[4][1];
JA76[4][2]=0. + links[28].inertia[1][2] + T7683[4][2];
JA76[4][3]=0. + links[28].inertia[1][3] + T7683[4][3];
JA76[4][4]=0. + T7683[4][4];
JA76[4][5]=0. - links[28].mcm[3] + T7683[4][5];
JA76[4][6]=0. + links[28].mcm[2] + T7683[4][6];

JA76[5][1]=0. + links[28].inertia[1][2] + T7683[5][1];
JA76[5][2]=0. + links[28].inertia[2][2] + T7683[5][2];
JA76[5][3]=0. + links[28].inertia[2][3] + T7683[5][3];
JA76[5][4]=0. + links[28].mcm[3] + T7683[5][4];
JA76[5][5]=0. + T7683[5][5];
JA76[5][6]=0. - links[28].mcm[1] + T7683[5][6];

JA76[6][1]=0. + links[28].inertia[1][3] + T7683[6][1];
JA76[6][2]=0. + links[28].inertia[2][3] + T7683[6][2];
JA76[6][3]=0. + links[28].inertia[3][3] + T7683[6][3];
JA76[6][4]=0. - links[28].mcm[2] + T7683[6][4];
JA76[6][5]=0. + links[28].mcm[1] + T7683[6][5];
JA76[6][6]=0. + T7683[6][6];


h76[1]=JA76[1][3];
h76[2]=JA76[2][3];
h76[3]=JA76[3][3];
h76[4]=JA76[4][3];
h76[5]=JA76[5][3];
h76[6]=JA76[6][3];

T17576[1][1]=JA76[1][1];
T17576[1][2]=JA76[1][2];
T17576[1][3]=JA76[1][3];
T17576[1][4]=JA76[1][4];
T17576[1][5]=JA76[1][5];
T17576[1][6]=JA76[1][6];

T17576[2][1]=JA76[2][1];
T17576[2][2]=JA76[2][2];
T17576[2][3]=JA76[2][3];
T17576[2][4]=JA76[2][4];
T17576[2][5]=JA76[2][5];
T17576[2][6]=JA76[2][6];

T17576[3][1]=JA76[3][1];
T17576[3][2]=JA76[3][2];
T17576[3][3]=JA76[3][3];
T17576[3][4]=JA76[3][4];
T17576[3][5]=JA76[3][5];
T17576[3][6]=JA76[3][6];

T17576[4][1]=JA76[4][1];
T17576[4][2]=JA76[4][2];
T17576[4][3]=JA76[4][3];
T17576[4][4]=JA76[4][4];
T17576[4][5]=JA76[4][5];
T17576[4][6]=JA76[4][6];

T17576[5][1]=JA76[5][1];
T17576[5][2]=JA76[5][2];
T17576[5][3]=JA76[5][3];
T17576[5][4]=JA76[5][4];
T17576[5][5]=JA76[5][5];
T17576[5][6]=JA76[5][6];

T17576[6][1]=JA76[6][1];
T17576[6][2]=JA76[6][2];
T17576[6][3]=JA76[6][3];
T17576[6][4]=JA76[6][4];
T17576[6][5]=JA76[6][5];
T17576[6][6]=JA76[6][6];


T7576[1][1]=S7675[1][1]*(Si7576[1][1]*T17576[1][1] + Si7576[1][2]*T17576[2][1]) + S7675[2][1]*(Si7576[1][1]*T17576[1][2] + Si7576[1][2]*T17576[2][2]);
T7576[1][2]=Si7576[1][1]*T17576[1][3] + Si7576[1][2]*T17576[2][3];
T7576[1][3]=S7675[1][3]*(Si7576[1][1]*T17576[1][1] + Si7576[1][2]*T17576[2][1]) + S7675[2][3]*(Si7576[1][1]*T17576[1][2] + Si7576[1][2]*T17576[2][2]);
T7576[1][4]=S7675[1][1]*(Si7576[1][1]*T17576[1][4] + Si7576[1][2]*T17576[2][4]) + S7675[2][1]*(Si7576[1][1]*T17576[1][5] + Si7576[1][2]*T17576[2][5]);
T7576[1][5]=Si7576[1][1]*T17576[1][6] + Si7576[1][2]*T17576[2][6];
T7576[1][6]=S7675[1][3]*(Si7576[1][1]*T17576[1][4] + Si7576[1][2]*T17576[2][4]) + S7675[2][3]*(Si7576[1][1]*T17576[1][5] + Si7576[1][2]*T17576[2][5]);

T7576[2][1]=S7675[1][1]*T17576[3][1] + S7675[2][1]*T17576[3][2];
T7576[2][2]=T17576[3][3];
T7576[2][3]=S7675[1][3]*T17576[3][1] + S7675[2][3]*T17576[3][2];
T7576[2][4]=S7675[1][1]*T17576[3][4] + S7675[2][1]*T17576[3][5];
T7576[2][5]=T17576[3][6];
T7576[2][6]=S7675[1][3]*T17576[3][4] + S7675[2][3]*T17576[3][5];

T7576[3][1]=S7675[1][1]*(Si7576[3][1]*T17576[1][1] + Si7576[3][2]*T17576[2][1]) + S7675[2][1]*(Si7576[3][1]*T17576[1][2] + Si7576[3][2]*T17576[2][2]);
T7576[3][2]=Si7576[3][1]*T17576[1][3] + Si7576[3][2]*T17576[2][3];
T7576[3][3]=S7675[1][3]*(Si7576[3][1]*T17576[1][1] + Si7576[3][2]*T17576[2][1]) + S7675[2][3]*(Si7576[3][1]*T17576[1][2] + Si7576[3][2]*T17576[2][2]);
T7576[3][4]=S7675[1][1]*(Si7576[3][1]*T17576[1][4] + Si7576[3][2]*T17576[2][4]) + S7675[2][1]*(Si7576[3][1]*T17576[1][5] + Si7576[3][2]*T17576[2][5]);
T7576[3][5]=Si7576[3][1]*T17576[1][6] + Si7576[3][2]*T17576[2][6];
T7576[3][6]=S7675[1][3]*(Si7576[3][1]*T17576[1][4] + Si7576[3][2]*T17576[2][4]) + S7675[2][3]*(Si7576[3][1]*T17576[1][5] + Si7576[3][2]*T17576[2][5]);

T7576[4][1]=S7675[1][1]*(Si7576[1][1]*T17576[4][1] + Si7576[1][2]*T17576[5][1]) + S7675[2][1]*(Si7576[1][1]*T17576[4][2] + Si7576[1][2]*T17576[5][2]);
T7576[4][2]=Si7576[1][1]*T17576[4][3] + Si7576[1][2]*T17576[5][3];
T7576[4][3]=S7675[1][3]*(Si7576[1][1]*T17576[4][1] + Si7576[1][2]*T17576[5][1]) + S7675[2][3]*(Si7576[1][1]*T17576[4][2] + Si7576[1][2]*T17576[5][2]);
T7576[4][4]=S7675[1][1]*(Si7576[1][1]*T17576[4][4] + Si7576[1][2]*T17576[5][4]) + S7675[2][1]*(Si7576[1][1]*T17576[4][5] + Si7576[1][2]*T17576[5][5]);
T7576[4][5]=Si7576[1][1]*T17576[4][6] + Si7576[1][2]*T17576[5][6];
T7576[4][6]=S7675[1][3]*(Si7576[1][1]*T17576[4][4] + Si7576[1][2]*T17576[5][4]) + S7675[2][3]*(Si7576[1][1]*T17576[4][5] + Si7576[1][2]*T17576[5][5]);

T7576[5][1]=S7675[1][1]*T17576[6][1] + S7675[2][1]*T17576[6][2];
T7576[5][2]=T17576[6][3];
T7576[5][3]=S7675[1][3]*T17576[6][1] + S7675[2][3]*T17576[6][2];
T7576[5][4]=S7675[1][1]*T17576[6][4] + S7675[2][1]*T17576[6][5];
T7576[5][5]=T17576[6][6];
T7576[5][6]=S7675[1][3]*T17576[6][4] + S7675[2][3]*T17576[6][5];

T7576[6][1]=S7675[1][1]*(Si7576[3][1]*T17576[4][1] + Si7576[3][2]*T17576[5][1]) + S7675[2][1]*(Si7576[3][1]*T17576[4][2] + Si7576[3][2]*T17576[5][2]);
T7576[6][2]=Si7576[3][1]*T17576[4][3] + Si7576[3][2]*T17576[5][3];
T7576[6][3]=S7675[1][3]*(Si7576[3][1]*T17576[4][1] + Si7576[3][2]*T17576[5][1]) + S7675[2][3]*(Si7576[3][1]*T17576[4][2] + Si7576[3][2]*T17576[5][2]);
T7576[6][4]=S7675[1][1]*(Si7576[3][1]*T17576[4][4] + Si7576[3][2]*T17576[5][4]) + S7675[2][1]*(Si7576[3][1]*T17576[4][5] + Si7576[3][2]*T17576[5][5]);
T7576[6][5]=Si7576[3][1]*T17576[4][6] + Si7576[3][2]*T17576[5][6];
T7576[6][6]=S7675[1][3]*(Si7576[3][1]*T17576[4][4] + Si7576[3][2]*T17576[5][4]) + S7675[2][3]*(Si7576[3][1]*T17576[4][5] + Si7576[3][2]*T17576[5][5]);



}


void
hermes_InvDynArtfunc29(void)
      {
JA75[1][1]=T7576[1][1];
JA75[1][2]=links[27].mcm[3] + T7576[1][2];
JA75[1][3]=-links[27].mcm[2] + T7576[1][3];
JA75[1][4]=links[27].m + T7576[1][4];
JA75[1][5]=T7576[1][5];
JA75[1][6]=T7576[1][6];

JA75[2][1]=-links[27].mcm[3] + T7576[2][1];
JA75[2][2]=T7576[2][2];
JA75[2][3]=links[27].mcm[1] + T7576[2][3];
JA75[2][4]=T7576[2][4];
JA75[2][5]=links[27].m + T7576[2][5];
JA75[2][6]=T7576[2][6];

JA75[3][1]=links[27].mcm[2] + T7576[3][1];
JA75[3][2]=-links[27].mcm[1] + T7576[3][2];
JA75[3][3]=T7576[3][3];
JA75[3][4]=T7576[3][4];
JA75[3][5]=T7576[3][5];
JA75[3][6]=links[27].m + T7576[3][6];

JA75[4][1]=links[27].inertia[1][1] + T7576[4][1];
JA75[4][2]=links[27].inertia[1][2] + T7576[4][2];
JA75[4][3]=links[27].inertia[1][3] + T7576[4][3];
JA75[4][4]=T7576[4][4];
JA75[4][5]=-links[27].mcm[3] + T7576[4][5];
JA75[4][6]=links[27].mcm[2] + T7576[4][6];

JA75[5][1]=links[27].inertia[1][2] + T7576[5][1];
JA75[5][2]=links[27].inertia[2][2] + T7576[5][2];
JA75[5][3]=links[27].inertia[2][3] + T7576[5][3];
JA75[5][4]=links[27].mcm[3] + T7576[5][4];
JA75[5][5]=T7576[5][5];
JA75[5][6]=-links[27].mcm[1] + T7576[5][6];

JA75[6][1]=links[27].inertia[1][3] + T7576[6][1];
JA75[6][2]=links[27].inertia[2][3] + T7576[6][2];
JA75[6][3]=links[27].inertia[3][3] + T7576[6][3];
JA75[6][4]=-links[27].mcm[2] + T7576[6][4];
JA75[6][5]=links[27].mcm[1] + T7576[6][5];
JA75[6][6]=T7576[6][6];


h75[1]=JA75[1][3];
h75[2]=JA75[2][3];
h75[3]=JA75[3][3];
h75[4]=JA75[4][3];
h75[5]=JA75[5][3];
h75[6]=JA75[6][3];

T17475[1][1]=JA75[1][1];
T17475[1][2]=JA75[1][2];
T17475[1][3]=JA75[1][3];
T17475[1][4]=JA75[1][4];
T17475[1][5]=JA75[1][5];
T17475[1][6]=JA75[1][6];

T17475[2][1]=JA75[2][1];
T17475[2][2]=JA75[2][2];
T17475[2][3]=JA75[2][3];
T17475[2][4]=JA75[2][4];
T17475[2][5]=JA75[2][5];
T17475[2][6]=JA75[2][6];

T17475[3][1]=JA75[3][1];
T17475[3][2]=JA75[3][2];
T17475[3][3]=JA75[3][3];
T17475[3][4]=JA75[3][4];
T17475[3][5]=JA75[3][5];
T17475[3][6]=JA75[3][6];

T17475[4][1]=JA75[4][1];
T17475[4][2]=JA75[4][2];
T17475[4][3]=JA75[4][3];
T17475[4][4]=JA75[4][4];
T17475[4][5]=JA75[4][5];
T17475[4][6]=JA75[4][6];

T17475[5][1]=JA75[5][1];
T17475[5][2]=JA75[5][2];
T17475[5][3]=JA75[5][3];
T17475[5][4]=JA75[5][4];
T17475[5][5]=JA75[5][5];
T17475[5][6]=JA75[5][6];

T17475[6][1]=JA75[6][1];
T17475[6][2]=JA75[6][2];
T17475[6][3]=JA75[6][3];
T17475[6][4]=JA75[6][4];
T17475[6][5]=JA75[6][5];
T17475[6][6]=JA75[6][6];


T7475[1][1]=S7574[1][1]*(Si7475[1][1]*T17475[1][1] + Si7475[1][2]*T17475[2][1]) + S7574[2][1]*(Si7475[1][1]*T17475[1][2] + Si7475[1][2]*T17475[2][2]) + LOWERLEG*(Si7475[1][1]*T17475[1][6] + Si7475[1][2]*T17475[2][6]);
T7475[1][2]=-(Si7475[1][1]*T17475[1][3]) - Si7475[1][2]*T17475[2][3] + LOWERLEG*S7574[1][1]*(Si7475[1][1]*T17475[1][4] + Si7475[1][2]*T17475[2][4]) + LOWERLEG*S7574[2][1]*(Si7475[1][1]*T17475[1][5] + Si7475[1][2]*T17475[2][5]);
T7475[1][3]=S7574[1][3]*(Si7475[1][1]*T17475[1][1] + Si7475[1][2]*T17475[2][1]) + S7574[2][3]*(Si7475[1][1]*T17475[1][2] + Si7475[1][2]*T17475[2][2]);
T7475[1][4]=S7574[1][1]*(Si7475[1][1]*T17475[1][4] + Si7475[1][2]*T17475[2][4]) + S7574[2][1]*(Si7475[1][1]*T17475[1][5] + Si7475[1][2]*T17475[2][5]);
T7475[1][5]=-(Si7475[1][1]*T17475[1][6]) - Si7475[1][2]*T17475[2][6];
T7475[1][6]=S7574[1][3]*(Si7475[1][1]*T17475[1][4] + Si7475[1][2]*T17475[2][4]) + S7574[2][3]*(Si7475[1][1]*T17475[1][5] + Si7475[1][2]*T17475[2][5]);

T7475[2][1]=-(S7574[1][1]*T17475[3][1]) - S7574[2][1]*T17475[3][2] - LOWERLEG*T17475[3][6];
T7475[2][2]=T17475[3][3] - LOWERLEG*S7574[1][1]*T17475[3][4] - LOWERLEG*S7574[2][1]*T17475[3][5];
T7475[2][3]=-(S7574[1][3]*T17475[3][1]) - S7574[2][3]*T17475[3][2];
T7475[2][4]=-(S7574[1][1]*T17475[3][4]) - S7574[2][1]*T17475[3][5];
T7475[2][5]=T17475[3][6];
T7475[2][6]=-(S7574[1][3]*T17475[3][4]) - S7574[2][3]*T17475[3][5];

T7475[3][1]=S7574[1][1]*(Si7475[3][1]*T17475[1][1] + Si7475[3][2]*T17475[2][1]) + S7574[2][1]*(Si7475[3][1]*T17475[1][2] + Si7475[3][2]*T17475[2][2]) + LOWERLEG*(Si7475[3][1]*T17475[1][6] + Si7475[3][2]*T17475[2][6]);
T7475[3][2]=-(Si7475[3][1]*T17475[1][3]) - Si7475[3][2]*T17475[2][3] + LOWERLEG*S7574[1][1]*(Si7475[3][1]*T17475[1][4] + Si7475[3][2]*T17475[2][4]) + LOWERLEG*S7574[2][1]*(Si7475[3][1]*T17475[1][5] + Si7475[3][2]*T17475[2][5]);
T7475[3][3]=S7574[1][3]*(Si7475[3][1]*T17475[1][1] + Si7475[3][2]*T17475[2][1]) + S7574[2][3]*(Si7475[3][1]*T17475[1][2] + Si7475[3][2]*T17475[2][2]);
T7475[3][4]=S7574[1][1]*(Si7475[3][1]*T17475[1][4] + Si7475[3][2]*T17475[2][4]) + S7574[2][1]*(Si7475[3][1]*T17475[1][5] + Si7475[3][2]*T17475[2][5]);
T7475[3][5]=-(Si7475[3][1]*T17475[1][6]) - Si7475[3][2]*T17475[2][6];
T7475[3][6]=S7574[1][3]*(Si7475[3][1]*T17475[1][4] + Si7475[3][2]*T17475[2][4]) + S7574[2][3]*(Si7475[3][1]*T17475[1][5] + Si7475[3][2]*T17475[2][5]);

T7475[4][1]=S7574[1][1]*(LOWERLEG*T17475[3][1] + Si7475[1][1]*T17475[4][1] + Si7475[1][2]*T17475[5][1]) + S7574[2][1]*(LOWERLEG*T17475[3][2] + Si7475[1][1]*T17475[4][2] + Si7475[1][2]*T17475[5][2]) + LOWERLEG*(LOWERLEG*T17475[3][6] + Si7475[1][1]*T17475[4][6] + Si7475[1][2]*T17475[5][6]);
T7475[4][2]=-(LOWERLEG*T17475[3][3]) - Si7475[1][1]*T17475[4][3] - Si7475[1][2]*T17475[5][3] + LOWERLEG*S7574[1][1]*(LOWERLEG*T17475[3][4] + Si7475[1][1]*T17475[4][4] + Si7475[1][2]*T17475[5][4]) + LOWERLEG*S7574[2][1]*(LOWERLEG*T17475[3][5] + Si7475[1][1]*T17475[4][5] + Si7475[1][2]*T17475[5][5]);
T7475[4][3]=S7574[1][3]*(LOWERLEG*T17475[3][1] + Si7475[1][1]*T17475[4][1] + Si7475[1][2]*T17475[5][1]) + S7574[2][3]*(LOWERLEG*T17475[3][2] + Si7475[1][1]*T17475[4][2] + Si7475[1][2]*T17475[5][2]);
T7475[4][4]=S7574[1][1]*(LOWERLEG*T17475[3][4] + Si7475[1][1]*T17475[4][4] + Si7475[1][2]*T17475[5][4]) + S7574[2][1]*(LOWERLEG*T17475[3][5] + Si7475[1][1]*T17475[4][5] + Si7475[1][2]*T17475[5][5]);
T7475[4][5]=-(LOWERLEG*T17475[3][6]) - Si7475[1][1]*T17475[4][6] - Si7475[1][2]*T17475[5][6];
T7475[4][6]=S7574[1][3]*(LOWERLEG*T17475[3][4] + Si7475[1][1]*T17475[4][4] + Si7475[1][2]*T17475[5][4]) + S7574[2][3]*(LOWERLEG*T17475[3][5] + Si7475[1][1]*T17475[4][5] + Si7475[1][2]*T17475[5][5]);

T7475[5][1]=S7574[1][1]*(LOWERLEG*Si7475[1][1]*T17475[1][1] + LOWERLEG*Si7475[1][2]*T17475[2][1] - T17475[6][1]) + S7574[2][1]*(LOWERLEG*Si7475[1][1]*T17475[1][2] + LOWERLEG*Si7475[1][2]*T17475[2][2] - T17475[6][2]) + LOWERLEG*(LOWERLEG*Si7475[1][1]*T17475[1][6] + LOWERLEG*Si7475[1][2]*T17475[2][6] - T17475[6][6]);
T7475[5][2]=-(LOWERLEG*Si7475[1][1]*T17475[1][3]) - LOWERLEG*Si7475[1][2]*T17475[2][3] + T17475[6][3] + LOWERLEG*S7574[1][1]*(LOWERLEG*Si7475[1][1]*T17475[1][4] + LOWERLEG*Si7475[1][2]*T17475[2][4] - T17475[6][4]) + LOWERLEG*S7574[2][1]*(LOWERLEG*Si7475[1][1]*T17475[1][5] + LOWERLEG*Si7475[1][2]*T17475[2][5] - T17475[6][5]);
T7475[5][3]=S7574[1][3]*(LOWERLEG*Si7475[1][1]*T17475[1][1] + LOWERLEG*Si7475[1][2]*T17475[2][1] - T17475[6][1]) + S7574[2][3]*(LOWERLEG*Si7475[1][1]*T17475[1][2] + LOWERLEG*Si7475[1][2]*T17475[2][2] - T17475[6][2]);
T7475[5][4]=S7574[1][1]*(LOWERLEG*Si7475[1][1]*T17475[1][4] + LOWERLEG*Si7475[1][2]*T17475[2][4] - T17475[6][4]) + S7574[2][1]*(LOWERLEG*Si7475[1][1]*T17475[1][5] + LOWERLEG*Si7475[1][2]*T17475[2][5] - T17475[6][5]);
T7475[5][5]=-(LOWERLEG*Si7475[1][1]*T17475[1][6]) - LOWERLEG*Si7475[1][2]*T17475[2][6] + T17475[6][6];
T7475[5][6]=S7574[1][3]*(LOWERLEG*Si7475[1][1]*T17475[1][4] + LOWERLEG*Si7475[1][2]*T17475[2][4] - T17475[6][4]) + S7574[2][3]*(LOWERLEG*Si7475[1][1]*T17475[1][5] + LOWERLEG*Si7475[1][2]*T17475[2][5] - T17475[6][5]);

T7475[6][1]=S7574[1][1]*(Si7475[3][1]*T17475[4][1] + Si7475[3][2]*T17475[5][1]) + S7574[2][1]*(Si7475[3][1]*T17475[4][2] + Si7475[3][2]*T17475[5][2]) + LOWERLEG*(Si7475[3][1]*T17475[4][6] + Si7475[3][2]*T17475[5][6]);
T7475[6][2]=-(Si7475[3][1]*T17475[4][3]) - Si7475[3][2]*T17475[5][3] + LOWERLEG*S7574[1][1]*(Si7475[3][1]*T17475[4][4] + Si7475[3][2]*T17475[5][4]) + LOWERLEG*S7574[2][1]*(Si7475[3][1]*T17475[4][5] + Si7475[3][2]*T17475[5][5]);
T7475[6][3]=S7574[1][3]*(Si7475[3][1]*T17475[4][1] + Si7475[3][2]*T17475[5][1]) + S7574[2][3]*(Si7475[3][1]*T17475[4][2] + Si7475[3][2]*T17475[5][2]);
T7475[6][4]=S7574[1][1]*(Si7475[3][1]*T17475[4][4] + Si7475[3][2]*T17475[5][4]) + S7574[2][1]*(Si7475[3][1]*T17475[4][5] + Si7475[3][2]*T17475[5][5]);
T7475[6][5]=-(Si7475[3][1]*T17475[4][6]) - Si7475[3][2]*T17475[5][6];
T7475[6][6]=S7574[1][3]*(Si7475[3][1]*T17475[4][4] + Si7475[3][2]*T17475[5][4]) + S7574[2][3]*(Si7475[3][1]*T17475[4][5] + Si7475[3][2]*T17475[5][5]);



}


void
hermes_InvDynArtfunc30(void)
      {
JA74[1][1]=T7475[1][1];
JA74[1][2]=links[26].mcm[3] + T7475[1][2];
JA74[1][3]=-links[26].mcm[2] + T7475[1][3];
JA74[1][4]=links[26].m + T7475[1][4];
JA74[1][5]=T7475[1][5];
JA74[1][6]=T7475[1][6];

JA74[2][1]=-links[26].mcm[3] + T7475[2][1];
JA74[2][2]=T7475[2][2];
JA74[2][3]=links[26].mcm[1] + T7475[2][3];
JA74[2][4]=T7475[2][4];
JA74[2][5]=links[26].m + T7475[2][5];
JA74[2][6]=T7475[2][6];

JA74[3][1]=links[26].mcm[2] + T7475[3][1];
JA74[3][2]=-links[26].mcm[1] + T7475[3][2];
JA74[3][3]=T7475[3][3];
JA74[3][4]=T7475[3][4];
JA74[3][5]=T7475[3][5];
JA74[3][6]=links[26].m + T7475[3][6];

JA74[4][1]=links[26].inertia[1][1] + T7475[4][1];
JA74[4][2]=links[26].inertia[1][2] + T7475[4][2];
JA74[4][3]=links[26].inertia[1][3] + T7475[4][3];
JA74[4][4]=T7475[4][4];
JA74[4][5]=-links[26].mcm[3] + T7475[4][5];
JA74[4][6]=links[26].mcm[2] + T7475[4][6];

JA74[5][1]=links[26].inertia[1][2] + T7475[5][1];
JA74[5][2]=links[26].inertia[2][2] + T7475[5][2];
JA74[5][3]=links[26].inertia[2][3] + T7475[5][3];
JA74[5][4]=links[26].mcm[3] + T7475[5][4];
JA74[5][5]=T7475[5][5];
JA74[5][6]=-links[26].mcm[1] + T7475[5][6];

JA74[6][1]=links[26].inertia[1][3] + T7475[6][1];
JA74[6][2]=links[26].inertia[2][3] + T7475[6][2];
JA74[6][3]=links[26].inertia[3][3] + T7475[6][3];
JA74[6][4]=-links[26].mcm[2] + T7475[6][4];
JA74[6][5]=links[26].mcm[1] + T7475[6][5];
JA74[6][6]=T7475[6][6];


h74[1]=JA74[1][3];
h74[2]=JA74[2][3];
h74[3]=JA74[3][3];
h74[4]=JA74[4][3];
h74[5]=JA74[5][3];
h74[6]=JA74[6][3];

T17374[1][1]=JA74[1][1];
T17374[1][2]=JA74[1][2];
T17374[1][3]=JA74[1][3];
T17374[1][4]=JA74[1][4];
T17374[1][5]=JA74[1][5];
T17374[1][6]=JA74[1][6];

T17374[2][1]=JA74[2][1];
T17374[2][2]=JA74[2][2];
T17374[2][3]=JA74[2][3];
T17374[2][4]=JA74[2][4];
T17374[2][5]=JA74[2][5];
T17374[2][6]=JA74[2][6];

T17374[3][1]=JA74[3][1];
T17374[3][2]=JA74[3][2];
T17374[3][3]=JA74[3][3];
T17374[3][4]=JA74[3][4];
T17374[3][5]=JA74[3][5];
T17374[3][6]=JA74[3][6];

T17374[4][1]=JA74[4][1];
T17374[4][2]=JA74[4][2];
T17374[4][3]=JA74[4][3];
T17374[4][4]=JA74[4][4];
T17374[4][5]=JA74[4][5];
T17374[4][6]=JA74[4][6];

T17374[5][1]=JA74[5][1];
T17374[5][2]=JA74[5][2];
T17374[5][3]=JA74[5][3];
T17374[5][4]=JA74[5][4];
T17374[5][5]=JA74[5][5];
T17374[5][6]=JA74[5][6];

T17374[6][1]=JA74[6][1];
T17374[6][2]=JA74[6][2];
T17374[6][3]=JA74[6][3];
T17374[6][4]=JA74[6][4];
T17374[6][5]=JA74[6][5];
T17374[6][6]=JA74[6][6];


T7374[1][1]=S7473[1][1]*(Si7374[1][1]*T17374[1][1] + Si7374[1][2]*T17374[2][1]) + S7473[2][1]*(Si7374[1][1]*T17374[1][2] + Si7374[1][2]*T17374[2][2]);
T7374[1][2]=-(Si7374[1][1]*T17374[1][3]) - Si7374[1][2]*T17374[2][3];
T7374[1][3]=S7473[1][3]*(Si7374[1][1]*T17374[1][1] + Si7374[1][2]*T17374[2][1]) + S7473[2][3]*(Si7374[1][1]*T17374[1][2] + Si7374[1][2]*T17374[2][2]);
T7374[1][4]=S7473[1][1]*(Si7374[1][1]*T17374[1][4] + Si7374[1][2]*T17374[2][4]) + S7473[2][1]*(Si7374[1][1]*T17374[1][5] + Si7374[1][2]*T17374[2][5]);
T7374[1][5]=-(Si7374[1][1]*T17374[1][6]) - Si7374[1][2]*T17374[2][6];
T7374[1][6]=S7473[1][3]*(Si7374[1][1]*T17374[1][4] + Si7374[1][2]*T17374[2][4]) + S7473[2][3]*(Si7374[1][1]*T17374[1][5] + Si7374[1][2]*T17374[2][5]);

T7374[2][1]=-(S7473[1][1]*T17374[3][1]) - S7473[2][1]*T17374[3][2];
T7374[2][2]=T17374[3][3];
T7374[2][3]=-(S7473[1][3]*T17374[3][1]) - S7473[2][3]*T17374[3][2];
T7374[2][4]=-(S7473[1][1]*T17374[3][4]) - S7473[2][1]*T17374[3][5];
T7374[2][5]=T17374[3][6];
T7374[2][6]=-(S7473[1][3]*T17374[3][4]) - S7473[2][3]*T17374[3][5];

T7374[3][1]=S7473[1][1]*(Si7374[3][1]*T17374[1][1] + Si7374[3][2]*T17374[2][1]) + S7473[2][1]*(Si7374[3][1]*T17374[1][2] + Si7374[3][2]*T17374[2][2]);
T7374[3][2]=-(Si7374[3][1]*T17374[1][3]) - Si7374[3][2]*T17374[2][3];
T7374[3][3]=S7473[1][3]*(Si7374[3][1]*T17374[1][1] + Si7374[3][2]*T17374[2][1]) + S7473[2][3]*(Si7374[3][1]*T17374[1][2] + Si7374[3][2]*T17374[2][2]);
T7374[3][4]=S7473[1][1]*(Si7374[3][1]*T17374[1][4] + Si7374[3][2]*T17374[2][4]) + S7473[2][1]*(Si7374[3][1]*T17374[1][5] + Si7374[3][2]*T17374[2][5]);
T7374[3][5]=-(Si7374[3][1]*T17374[1][6]) - Si7374[3][2]*T17374[2][6];
T7374[3][6]=S7473[1][3]*(Si7374[3][1]*T17374[1][4] + Si7374[3][2]*T17374[2][4]) + S7473[2][3]*(Si7374[3][1]*T17374[1][5] + Si7374[3][2]*T17374[2][5]);

T7374[4][1]=S7473[1][1]*(Si7374[1][1]*T17374[4][1] + Si7374[1][2]*T17374[5][1]) + S7473[2][1]*(Si7374[1][1]*T17374[4][2] + Si7374[1][2]*T17374[5][2]);
T7374[4][2]=-(Si7374[1][1]*T17374[4][3]) - Si7374[1][2]*T17374[5][3];
T7374[4][3]=S7473[1][3]*(Si7374[1][1]*T17374[4][1] + Si7374[1][2]*T17374[5][1]) + S7473[2][3]*(Si7374[1][1]*T17374[4][2] + Si7374[1][2]*T17374[5][2]);
T7374[4][4]=S7473[1][1]*(Si7374[1][1]*T17374[4][4] + Si7374[1][2]*T17374[5][4]) + S7473[2][1]*(Si7374[1][1]*T17374[4][5] + Si7374[1][2]*T17374[5][5]);
T7374[4][5]=-(Si7374[1][1]*T17374[4][6]) - Si7374[1][2]*T17374[5][6];
T7374[4][6]=S7473[1][3]*(Si7374[1][1]*T17374[4][4] + Si7374[1][2]*T17374[5][4]) + S7473[2][3]*(Si7374[1][1]*T17374[4][5] + Si7374[1][2]*T17374[5][5]);

T7374[5][1]=-(S7473[1][1]*T17374[6][1]) - S7473[2][1]*T17374[6][2];
T7374[5][2]=T17374[6][3];
T7374[5][3]=-(S7473[1][3]*T17374[6][1]) - S7473[2][3]*T17374[6][2];
T7374[5][4]=-(S7473[1][1]*T17374[6][4]) - S7473[2][1]*T17374[6][5];
T7374[5][5]=T17374[6][6];
T7374[5][6]=-(S7473[1][3]*T17374[6][4]) - S7473[2][3]*T17374[6][5];

T7374[6][1]=S7473[1][1]*(Si7374[3][1]*T17374[4][1] + Si7374[3][2]*T17374[5][1]) + S7473[2][1]*(Si7374[3][1]*T17374[4][2] + Si7374[3][2]*T17374[5][2]);
T7374[6][2]=-(Si7374[3][1]*T17374[4][3]) - Si7374[3][2]*T17374[5][3];
T7374[6][3]=S7473[1][3]*(Si7374[3][1]*T17374[4][1] + Si7374[3][2]*T17374[5][1]) + S7473[2][3]*(Si7374[3][1]*T17374[4][2] + Si7374[3][2]*T17374[5][2]);
T7374[6][4]=S7473[1][1]*(Si7374[3][1]*T17374[4][4] + Si7374[3][2]*T17374[5][4]) + S7473[2][1]*(Si7374[3][1]*T17374[4][5] + Si7374[3][2]*T17374[5][5]);
T7374[6][5]=-(Si7374[3][1]*T17374[4][6]) - Si7374[3][2]*T17374[5][6];
T7374[6][6]=S7473[1][3]*(Si7374[3][1]*T17374[4][4] + Si7374[3][2]*T17374[5][4]) + S7473[2][3]*(Si7374[3][1]*T17374[4][5] + Si7374[3][2]*T17374[5][5]);



}


void
hermes_InvDynArtfunc31(void)
      {
JA73[1][1]=T7374[1][1];
JA73[1][2]=links[25].mcm[3] + T7374[1][2];
JA73[1][3]=-links[25].mcm[2] + T7374[1][3];
JA73[1][4]=links[25].m + T7374[1][4];
JA73[1][5]=T7374[1][5];
JA73[1][6]=T7374[1][6];

JA73[2][1]=-links[25].mcm[3] + T7374[2][1];
JA73[2][2]=T7374[2][2];
JA73[2][3]=links[25].mcm[1] + T7374[2][3];
JA73[2][4]=T7374[2][4];
JA73[2][5]=links[25].m + T7374[2][5];
JA73[2][6]=T7374[2][6];

JA73[3][1]=links[25].mcm[2] + T7374[3][1];
JA73[3][2]=-links[25].mcm[1] + T7374[3][2];
JA73[3][3]=T7374[3][3];
JA73[3][4]=T7374[3][4];
JA73[3][5]=T7374[3][5];
JA73[3][6]=links[25].m + T7374[3][6];

JA73[4][1]=links[25].inertia[1][1] + T7374[4][1];
JA73[4][2]=links[25].inertia[1][2] + T7374[4][2];
JA73[4][3]=links[25].inertia[1][3] + T7374[4][3];
JA73[4][4]=T7374[4][4];
JA73[4][5]=-links[25].mcm[3] + T7374[4][5];
JA73[4][6]=links[25].mcm[2] + T7374[4][6];

JA73[5][1]=links[25].inertia[1][2] + T7374[5][1];
JA73[5][2]=links[25].inertia[2][2] + T7374[5][2];
JA73[5][3]=links[25].inertia[2][3] + T7374[5][3];
JA73[5][4]=links[25].mcm[3] + T7374[5][4];
JA73[5][5]=T7374[5][5];
JA73[5][6]=-links[25].mcm[1] + T7374[5][6];

JA73[6][1]=links[25].inertia[1][3] + T7374[6][1];
JA73[6][2]=links[25].inertia[2][3] + T7374[6][2];
JA73[6][3]=links[25].inertia[3][3] + T7374[6][3];
JA73[6][4]=-links[25].mcm[2] + T7374[6][4];
JA73[6][5]=links[25].mcm[1] + T7374[6][5];
JA73[6][6]=T7374[6][6];


h73[1]=JA73[1][3];
h73[2]=JA73[2][3];
h73[3]=JA73[3][3];
h73[4]=JA73[4][3];
h73[5]=JA73[5][3];
h73[6]=JA73[6][3];

T17273[1][1]=JA73[1][1];
T17273[1][2]=JA73[1][2];
T17273[1][3]=JA73[1][3];
T17273[1][4]=JA73[1][4];
T17273[1][5]=JA73[1][5];
T17273[1][6]=JA73[1][6];

T17273[2][1]=JA73[2][1];
T17273[2][2]=JA73[2][2];
T17273[2][3]=JA73[2][3];
T17273[2][4]=JA73[2][4];
T17273[2][5]=JA73[2][5];
T17273[2][6]=JA73[2][6];

T17273[3][1]=JA73[3][1];
T17273[3][2]=JA73[3][2];
T17273[3][3]=JA73[3][3];
T17273[3][4]=JA73[3][4];
T17273[3][5]=JA73[3][5];
T17273[3][6]=JA73[3][6];

T17273[4][1]=JA73[4][1];
T17273[4][2]=JA73[4][2];
T17273[4][3]=JA73[4][3];
T17273[4][4]=JA73[4][4];
T17273[4][5]=JA73[4][5];
T17273[4][6]=JA73[4][6];

T17273[5][1]=JA73[5][1];
T17273[5][2]=JA73[5][2];
T17273[5][3]=JA73[5][3];
T17273[5][4]=JA73[5][4];
T17273[5][5]=JA73[5][5];
T17273[5][6]=JA73[5][6];

T17273[6][1]=JA73[6][1];
T17273[6][2]=JA73[6][2];
T17273[6][3]=JA73[6][3];
T17273[6][4]=JA73[6][4];
T17273[6][5]=JA73[6][5];
T17273[6][6]=JA73[6][6];


T7273[1][1]=S7372[1][1]*(Si7273[1][1]*T17273[1][1] + Si7273[1][2]*T17273[2][1]) + S7372[2][1]*(Si7273[1][1]*T17273[1][2] + Si7273[1][2]*T17273[2][2]) - UPPERLEGMOD*(Si7273[1][1]*T17273[1][6] + Si7273[1][2]*T17273[2][6]);
T7273[1][2]=Si7273[1][1]*T17273[1][3] + Si7273[1][2]*T17273[2][3] + (UPPERLEGMOD*S7372[1][1] - YKNEE*S7372[1][3])*(Si7273[1][1]*T17273[1][4] + Si7273[1][2]*T17273[2][4]) + (UPPERLEGMOD*S7372[2][1] - YKNEE*S7372[2][3])*(Si7273[1][1]*T17273[1][5] + Si7273[1][2]*T17273[2][5]);
T7273[1][3]=S7372[1][3]*(Si7273[1][1]*T17273[1][1] + Si7273[1][2]*T17273[2][1]) + S7372[2][3]*(Si7273[1][1]*T17273[1][2] + Si7273[1][2]*T17273[2][2]) + YKNEE*(Si7273[1][1]*T17273[1][6] + Si7273[1][2]*T17273[2][6]);
T7273[1][4]=S7372[1][1]*(Si7273[1][1]*T17273[1][4] + Si7273[1][2]*T17273[2][4]) + S7372[2][1]*(Si7273[1][1]*T17273[1][5] + Si7273[1][2]*T17273[2][5]);
T7273[1][5]=Si7273[1][1]*T17273[1][6] + Si7273[1][2]*T17273[2][6];
T7273[1][6]=S7372[1][3]*(Si7273[1][1]*T17273[1][4] + Si7273[1][2]*T17273[2][4]) + S7372[2][3]*(Si7273[1][1]*T17273[1][5] + Si7273[1][2]*T17273[2][5]);

T7273[2][1]=S7372[1][1]*T17273[3][1] + S7372[2][1]*T17273[3][2] - UPPERLEGMOD*T17273[3][6];
T7273[2][2]=T17273[3][3] + (UPPERLEGMOD*S7372[1][1] - YKNEE*S7372[1][3])*T17273[3][4] + (UPPERLEGMOD*S7372[2][1] - YKNEE*S7372[2][3])*T17273[3][5];
T7273[2][3]=S7372[1][3]*T17273[3][1] + S7372[2][3]*T17273[3][2] + YKNEE*T17273[3][6];
T7273[2][4]=S7372[1][1]*T17273[3][4] + S7372[2][1]*T17273[3][5];
T7273[2][5]=T17273[3][6];
T7273[2][6]=S7372[1][3]*T17273[3][4] + S7372[2][3]*T17273[3][5];

T7273[3][1]=S7372[1][1]*(Si7273[3][1]*T17273[1][1] + Si7273[3][2]*T17273[2][1]) + S7372[2][1]*(Si7273[3][1]*T17273[1][2] + Si7273[3][2]*T17273[2][2]) - UPPERLEGMOD*(Si7273[3][1]*T17273[1][6] + Si7273[3][2]*T17273[2][6]);
T7273[3][2]=Si7273[3][1]*T17273[1][3] + Si7273[3][2]*T17273[2][3] + (UPPERLEGMOD*S7372[1][1] - YKNEE*S7372[1][3])*(Si7273[3][1]*T17273[1][4] + Si7273[3][2]*T17273[2][4]) + (UPPERLEGMOD*S7372[2][1] - YKNEE*S7372[2][3])*(Si7273[3][1]*T17273[1][5] + Si7273[3][2]*T17273[2][5]);
T7273[3][3]=S7372[1][3]*(Si7273[3][1]*T17273[1][1] + Si7273[3][2]*T17273[2][1]) + S7372[2][3]*(Si7273[3][1]*T17273[1][2] + Si7273[3][2]*T17273[2][2]) + YKNEE*(Si7273[3][1]*T17273[1][6] + Si7273[3][2]*T17273[2][6]);
T7273[3][4]=S7372[1][1]*(Si7273[3][1]*T17273[1][4] + Si7273[3][2]*T17273[2][4]) + S7372[2][1]*(Si7273[3][1]*T17273[1][5] + Si7273[3][2]*T17273[2][5]);
T7273[3][5]=Si7273[3][1]*T17273[1][6] + Si7273[3][2]*T17273[2][6];
T7273[3][6]=S7372[1][3]*(Si7273[3][1]*T17273[1][4] + Si7273[3][2]*T17273[2][4]) + S7372[2][3]*(Si7273[3][1]*T17273[1][5] + Si7273[3][2]*T17273[2][5]);

T7273[4][1]=S7372[1][1]*(-(UPPERLEGMOD*T17273[3][1]) + Si7273[1][1]*T17273[4][1] + Si7273[1][2]*T17273[5][1]) + S7372[2][1]*(-(UPPERLEGMOD*T17273[3][2]) + Si7273[1][1]*T17273[4][2] + Si7273[1][2]*T17273[5][2]) - UPPERLEGMOD*(-(UPPERLEGMOD*T17273[3][6]) + Si7273[1][1]*T17273[4][6] + Si7273[1][2]*T17273[5][6]);
T7273[4][2]=-(UPPERLEGMOD*T17273[3][3]) + Si7273[1][1]*T17273[4][3] + Si7273[1][2]*T17273[5][3] + (UPPERLEGMOD*S7372[1][1] - YKNEE*S7372[1][3])*(-(UPPERLEGMOD*T17273[3][4]) + Si7273[1][1]*T17273[4][4] + Si7273[1][2]*T17273[5][4]) + (UPPERLEGMOD*S7372[2][1] - YKNEE*S7372[2][3])*(-(UPPERLEGMOD*T17273[3][5]) + Si7273[1][1]*T17273[4][5] + Si7273[1][2]*T17273[5][5]);
T7273[4][3]=S7372[1][3]*(-(UPPERLEGMOD*T17273[3][1]) + Si7273[1][1]*T17273[4][1] + Si7273[1][2]*T17273[5][1]) + S7372[2][3]*(-(UPPERLEGMOD*T17273[3][2]) + Si7273[1][1]*T17273[4][2] + Si7273[1][2]*T17273[5][2]) + YKNEE*(-(UPPERLEGMOD*T17273[3][6]) + Si7273[1][1]*T17273[4][6] + Si7273[1][2]*T17273[5][6]);
T7273[4][4]=S7372[1][1]*(-(UPPERLEGMOD*T17273[3][4]) + Si7273[1][1]*T17273[4][4] + Si7273[1][2]*T17273[5][4]) + S7372[2][1]*(-(UPPERLEGMOD*T17273[3][5]) + Si7273[1][1]*T17273[4][5] + Si7273[1][2]*T17273[5][5]);
T7273[4][5]=-(UPPERLEGMOD*T17273[3][6]) + Si7273[1][1]*T17273[4][6] + Si7273[1][2]*T17273[5][6];
T7273[4][6]=S7372[1][3]*(-(UPPERLEGMOD*T17273[3][4]) + Si7273[1][1]*T17273[4][4] + Si7273[1][2]*T17273[5][4]) + S7372[2][3]*(-(UPPERLEGMOD*T17273[3][5]) + Si7273[1][1]*T17273[4][5] + Si7273[1][2]*T17273[5][5]);

T7273[5][1]=S7372[1][1]*((UPPERLEGMOD*Si7273[1][1] - YKNEE*Si7273[3][1])*T17273[1][1] + (UPPERLEGMOD*Si7273[1][2] - YKNEE*Si7273[3][2])*T17273[2][1] + T17273[6][1]) + S7372[2][1]*((UPPERLEGMOD*Si7273[1][1] - YKNEE*Si7273[3][1])*T17273[1][2] + (UPPERLEGMOD*Si7273[1][2] - YKNEE*Si7273[3][2])*T17273[2][2] + T17273[6][2]) - UPPERLEGMOD*((UPPERLEGMOD*Si7273[1][1] - YKNEE*Si7273[3][1])*T17273[1][6] + (UPPERLEGMOD*Si7273[1][2] - YKNEE*Si7273[3][2])*T17273[2][6] + T17273[6][6]);
T7273[5][2]=(UPPERLEGMOD*Si7273[1][1] - YKNEE*Si7273[3][1])*T17273[1][3] + (UPPERLEGMOD*Si7273[1][2] - YKNEE*Si7273[3][2])*T17273[2][3] + T17273[6][3] + (UPPERLEGMOD*S7372[1][1] - YKNEE*S7372[1][3])*((UPPERLEGMOD*Si7273[1][1] - YKNEE*Si7273[3][1])*T17273[1][4] + (UPPERLEGMOD*Si7273[1][2] - YKNEE*Si7273[3][2])*T17273[2][4] + T17273[6][4]) + (UPPERLEGMOD*S7372[2][1] - YKNEE*S7372[2][3])*((UPPERLEGMOD*Si7273[1][1] - YKNEE*Si7273[3][1])*T17273[1][5] + (UPPERLEGMOD*Si7273[1][2] - YKNEE*Si7273[3][2])*T17273[2][5] + T17273[6][5]);
T7273[5][3]=S7372[1][3]*((UPPERLEGMOD*Si7273[1][1] - YKNEE*Si7273[3][1])*T17273[1][1] + (UPPERLEGMOD*Si7273[1][2] - YKNEE*Si7273[3][2])*T17273[2][1] + T17273[6][1]) + S7372[2][3]*((UPPERLEGMOD*Si7273[1][1] - YKNEE*Si7273[3][1])*T17273[1][2] + (UPPERLEGMOD*Si7273[1][2] - YKNEE*Si7273[3][2])*T17273[2][2] + T17273[6][2]) + YKNEE*((UPPERLEGMOD*Si7273[1][1] - YKNEE*Si7273[3][1])*T17273[1][6] + (UPPERLEGMOD*Si7273[1][2] - YKNEE*Si7273[3][2])*T17273[2][6] + T17273[6][6]);
T7273[5][4]=S7372[1][1]*((UPPERLEGMOD*Si7273[1][1] - YKNEE*Si7273[3][1])*T17273[1][4] + (UPPERLEGMOD*Si7273[1][2] - YKNEE*Si7273[3][2])*T17273[2][4] + T17273[6][4]) + S7372[2][1]*((UPPERLEGMOD*Si7273[1][1] - YKNEE*Si7273[3][1])*T17273[1][5] + (UPPERLEGMOD*Si7273[1][2] - YKNEE*Si7273[3][2])*T17273[2][5] + T17273[6][5]);
T7273[5][5]=(UPPERLEGMOD*Si7273[1][1] - YKNEE*Si7273[3][1])*T17273[1][6] + (UPPERLEGMOD*Si7273[1][2] - YKNEE*Si7273[3][2])*T17273[2][6] + T17273[6][6];
T7273[5][6]=S7372[1][3]*((UPPERLEGMOD*Si7273[1][1] - YKNEE*Si7273[3][1])*T17273[1][4] + (UPPERLEGMOD*Si7273[1][2] - YKNEE*Si7273[3][2])*T17273[2][4] + T17273[6][4]) + S7372[2][3]*((UPPERLEGMOD*Si7273[1][1] - YKNEE*Si7273[3][1])*T17273[1][5] + (UPPERLEGMOD*Si7273[1][2] - YKNEE*Si7273[3][2])*T17273[2][5] + T17273[6][5]);

T7273[6][1]=S7372[1][1]*(YKNEE*T17273[3][1] + Si7273[3][1]*T17273[4][1] + Si7273[3][2]*T17273[5][1]) + S7372[2][1]*(YKNEE*T17273[3][2] + Si7273[3][1]*T17273[4][2] + Si7273[3][2]*T17273[5][2]) - UPPERLEGMOD*(YKNEE*T17273[3][6] + Si7273[3][1]*T17273[4][6] + Si7273[3][2]*T17273[5][6]);
T7273[6][2]=YKNEE*T17273[3][3] + Si7273[3][1]*T17273[4][3] + Si7273[3][2]*T17273[5][3] + (UPPERLEGMOD*S7372[1][1] - YKNEE*S7372[1][3])*(YKNEE*T17273[3][4] + Si7273[3][1]*T17273[4][4] + Si7273[3][2]*T17273[5][4]) + (UPPERLEGMOD*S7372[2][1] - YKNEE*S7372[2][3])*(YKNEE*T17273[3][5] + Si7273[3][1]*T17273[4][5] + Si7273[3][2]*T17273[5][5]);
T7273[6][3]=S7372[1][3]*(YKNEE*T17273[3][1] + Si7273[3][1]*T17273[4][1] + Si7273[3][2]*T17273[5][1]) + S7372[2][3]*(YKNEE*T17273[3][2] + Si7273[3][1]*T17273[4][2] + Si7273[3][2]*T17273[5][2]) + YKNEE*(YKNEE*T17273[3][6] + Si7273[3][1]*T17273[4][6] + Si7273[3][2]*T17273[5][6]);
T7273[6][4]=S7372[1][1]*(YKNEE*T17273[3][4] + Si7273[3][1]*T17273[4][4] + Si7273[3][2]*T17273[5][4]) + S7372[2][1]*(YKNEE*T17273[3][5] + Si7273[3][1]*T17273[4][5] + Si7273[3][2]*T17273[5][5]);
T7273[6][5]=YKNEE*T17273[3][6] + Si7273[3][1]*T17273[4][6] + Si7273[3][2]*T17273[5][6];
T7273[6][6]=S7372[1][3]*(YKNEE*T17273[3][4] + Si7273[3][1]*T17273[4][4] + Si7273[3][2]*T17273[5][4]) + S7372[2][3]*(YKNEE*T17273[3][5] + Si7273[3][1]*T17273[4][5] + Si7273[3][2]*T17273[5][5]);



}


void
hermes_InvDynArtfunc32(void)
      {
JA72[1][1]=T7273[1][1];
JA72[1][2]=links[24].mcm[3] + T7273[1][2];
JA72[1][3]=-links[24].mcm[2] + T7273[1][3];
JA72[1][4]=links[24].m + T7273[1][4];
JA72[1][5]=T7273[1][5];
JA72[1][6]=T7273[1][6];

JA72[2][1]=-links[24].mcm[3] + T7273[2][1];
JA72[2][2]=T7273[2][2];
JA72[2][3]=links[24].mcm[1] + T7273[2][3];
JA72[2][4]=T7273[2][4];
JA72[2][5]=links[24].m + T7273[2][5];
JA72[2][6]=T7273[2][6];

JA72[3][1]=links[24].mcm[2] + T7273[3][1];
JA72[3][2]=-links[24].mcm[1] + T7273[3][2];
JA72[3][3]=T7273[3][3];
JA72[3][4]=T7273[3][4];
JA72[3][5]=T7273[3][5];
JA72[3][6]=links[24].m + T7273[3][6];

JA72[4][1]=links[24].inertia[1][1] + T7273[4][1];
JA72[4][2]=links[24].inertia[1][2] + T7273[4][2];
JA72[4][3]=links[24].inertia[1][3] + T7273[4][3];
JA72[4][4]=T7273[4][4];
JA72[4][5]=-links[24].mcm[3] + T7273[4][5];
JA72[4][6]=links[24].mcm[2] + T7273[4][6];

JA72[5][1]=links[24].inertia[1][2] + T7273[5][1];
JA72[5][2]=links[24].inertia[2][2] + T7273[5][2];
JA72[5][3]=links[24].inertia[2][3] + T7273[5][3];
JA72[5][4]=links[24].mcm[3] + T7273[5][4];
JA72[5][5]=T7273[5][5];
JA72[5][6]=-links[24].mcm[1] + T7273[5][6];

JA72[6][1]=links[24].inertia[1][3] + T7273[6][1];
JA72[6][2]=links[24].inertia[2][3] + T7273[6][2];
JA72[6][3]=links[24].inertia[3][3] + T7273[6][3];
JA72[6][4]=-links[24].mcm[2] + T7273[6][4];
JA72[6][5]=links[24].mcm[1] + T7273[6][5];
JA72[6][6]=T7273[6][6];


h72[1]=JA72[1][3];
h72[2]=JA72[2][3];
h72[3]=JA72[3][3];
h72[4]=JA72[4][3];
h72[5]=JA72[5][3];
h72[6]=JA72[6][3];

T17172[1][1]=JA72[1][1];
T17172[1][2]=JA72[1][2];
T17172[1][3]=JA72[1][3];
T17172[1][4]=JA72[1][4];
T17172[1][5]=JA72[1][5];
T17172[1][6]=JA72[1][6];

T17172[2][1]=JA72[2][1];
T17172[2][2]=JA72[2][2];
T17172[2][3]=JA72[2][3];
T17172[2][4]=JA72[2][4];
T17172[2][5]=JA72[2][5];
T17172[2][6]=JA72[2][6];

T17172[3][1]=JA72[3][1];
T17172[3][2]=JA72[3][2];
T17172[3][3]=JA72[3][3];
T17172[3][4]=JA72[3][4];
T17172[3][5]=JA72[3][5];
T17172[3][6]=JA72[3][6];

T17172[4][1]=JA72[4][1];
T17172[4][2]=JA72[4][2];
T17172[4][3]=JA72[4][3];
T17172[4][4]=JA72[4][4];
T17172[4][5]=JA72[4][5];
T17172[4][6]=JA72[4][6];

T17172[5][1]=JA72[5][1];
T17172[5][2]=JA72[5][2];
T17172[5][3]=JA72[5][3];
T17172[5][4]=JA72[5][4];
T17172[5][5]=JA72[5][5];
T17172[5][6]=JA72[5][6];

T17172[6][1]=JA72[6][1];
T17172[6][2]=JA72[6][2];
T17172[6][3]=JA72[6][3];
T17172[6][4]=JA72[6][4];
T17172[6][5]=JA72[6][5];
T17172[6][6]=JA72[6][6];


T7172[1][1]=S7271[1][1]*(Si7172[1][1]*T17172[1][1] + Si7172[1][2]*T17172[2][1] + Si7172[1][3]*T17172[3][1]) + S7271[2][1]*(Si7172[1][1]*T17172[1][2] + Si7172[1][2]*T17172[2][2] + Si7172[1][3]*T17172[3][2]) + S7271[3][1]*(Si7172[1][1]*T17172[1][3] + Si7172[1][2]*T17172[2][3] + Si7172[1][3]*T17172[3][3]);
T7172[1][2]=S7271[1][2]*(Si7172[1][1]*T17172[1][1] + Si7172[1][2]*T17172[2][1] + Si7172[1][3]*T17172[3][1]) + S7271[2][2]*(Si7172[1][1]*T17172[1][2] + Si7172[1][2]*T17172[2][2] + Si7172[1][3]*T17172[3][2]) + S7271[3][2]*(Si7172[1][1]*T17172[1][3] + Si7172[1][2]*T17172[2][3] + Si7172[1][3]*T17172[3][3]) - YHIP*S7271[1][3]*(Si7172[1][1]*T17172[1][4] + Si7172[1][2]*T17172[2][4] + Si7172[1][3]*T17172[3][4]) - YHIP*S7271[2][3]*(Si7172[1][1]*T17172[1][5] + Si7172[1][2]*T17172[2][5] + Si7172[1][3]*T17172[3][5]);
T7172[1][3]=S7271[1][3]*(Si7172[1][1]*T17172[1][1] + Si7172[1][2]*T17172[2][1] + Si7172[1][3]*T17172[3][1]) + S7271[2][3]*(Si7172[1][1]*T17172[1][2] + Si7172[1][2]*T17172[2][2] + Si7172[1][3]*T17172[3][2]) + YHIP*S7271[1][2]*(Si7172[1][1]*T17172[1][4] + Si7172[1][2]*T17172[2][4] + Si7172[1][3]*T17172[3][4]) + YHIP*S7271[2][2]*(Si7172[1][1]*T17172[1][5] + Si7172[1][2]*T17172[2][5] + Si7172[1][3]*T17172[3][5]) + YHIP*S7271[3][2]*(Si7172[1][1]*T17172[1][6] + Si7172[1][2]*T17172[2][6] + Si7172[1][3]*T17172[3][6]);
T7172[1][4]=S7271[1][1]*(Si7172[1][1]*T17172[1][4] + Si7172[1][2]*T17172[2][4] + Si7172[1][3]*T17172[3][4]) + S7271[2][1]*(Si7172[1][1]*T17172[1][5] + Si7172[1][2]*T17172[2][5] + Si7172[1][3]*T17172[3][5]) + S7271[3][1]*(Si7172[1][1]*T17172[1][6] + Si7172[1][2]*T17172[2][6] + Si7172[1][3]*T17172[3][6]);
T7172[1][5]=S7271[1][2]*(Si7172[1][1]*T17172[1][4] + Si7172[1][2]*T17172[2][4] + Si7172[1][3]*T17172[3][4]) + S7271[2][2]*(Si7172[1][1]*T17172[1][5] + Si7172[1][2]*T17172[2][5] + Si7172[1][3]*T17172[3][5]) + S7271[3][2]*(Si7172[1][1]*T17172[1][6] + Si7172[1][2]*T17172[2][6] + Si7172[1][3]*T17172[3][6]);
T7172[1][6]=S7271[1][3]*(Si7172[1][1]*T17172[1][4] + Si7172[1][2]*T17172[2][4] + Si7172[1][3]*T17172[3][4]) + S7271[2][3]*(Si7172[1][1]*T17172[1][5] + Si7172[1][2]*T17172[2][5] + Si7172[1][3]*T17172[3][5]);

T7172[2][1]=S7271[1][1]*(Si7172[2][1]*T17172[1][1] + Si7172[2][2]*T17172[2][1] + Si7172[2][3]*T17172[3][1]) + S7271[2][1]*(Si7172[2][1]*T17172[1][2] + Si7172[2][2]*T17172[2][2] + Si7172[2][3]*T17172[3][2]) + S7271[3][1]*(Si7172[2][1]*T17172[1][3] + Si7172[2][2]*T17172[2][3] + Si7172[2][3]*T17172[3][3]);
T7172[2][2]=S7271[1][2]*(Si7172[2][1]*T17172[1][1] + Si7172[2][2]*T17172[2][1] + Si7172[2][3]*T17172[3][1]) + S7271[2][2]*(Si7172[2][1]*T17172[1][2] + Si7172[2][2]*T17172[2][2] + Si7172[2][3]*T17172[3][2]) + S7271[3][2]*(Si7172[2][1]*T17172[1][3] + Si7172[2][2]*T17172[2][3] + Si7172[2][3]*T17172[3][3]) - YHIP*S7271[1][3]*(Si7172[2][1]*T17172[1][4] + Si7172[2][2]*T17172[2][4] + Si7172[2][3]*T17172[3][4]) - YHIP*S7271[2][3]*(Si7172[2][1]*T17172[1][5] + Si7172[2][2]*T17172[2][5] + Si7172[2][3]*T17172[3][5]);
T7172[2][3]=S7271[1][3]*(Si7172[2][1]*T17172[1][1] + Si7172[2][2]*T17172[2][1] + Si7172[2][3]*T17172[3][1]) + S7271[2][3]*(Si7172[2][1]*T17172[1][2] + Si7172[2][2]*T17172[2][2] + Si7172[2][3]*T17172[3][2]) + YHIP*S7271[1][2]*(Si7172[2][1]*T17172[1][4] + Si7172[2][2]*T17172[2][4] + Si7172[2][3]*T17172[3][4]) + YHIP*S7271[2][2]*(Si7172[2][1]*T17172[1][5] + Si7172[2][2]*T17172[2][5] + Si7172[2][3]*T17172[3][5]) + YHIP*S7271[3][2]*(Si7172[2][1]*T17172[1][6] + Si7172[2][2]*T17172[2][6] + Si7172[2][3]*T17172[3][6]);
T7172[2][4]=S7271[1][1]*(Si7172[2][1]*T17172[1][4] + Si7172[2][2]*T17172[2][4] + Si7172[2][3]*T17172[3][4]) + S7271[2][1]*(Si7172[2][1]*T17172[1][5] + Si7172[2][2]*T17172[2][5] + Si7172[2][3]*T17172[3][5]) + S7271[3][1]*(Si7172[2][1]*T17172[1][6] + Si7172[2][2]*T17172[2][6] + Si7172[2][3]*T17172[3][6]);
T7172[2][5]=S7271[1][2]*(Si7172[2][1]*T17172[1][4] + Si7172[2][2]*T17172[2][4] + Si7172[2][3]*T17172[3][4]) + S7271[2][2]*(Si7172[2][1]*T17172[1][5] + Si7172[2][2]*T17172[2][5] + Si7172[2][3]*T17172[3][5]) + S7271[3][2]*(Si7172[2][1]*T17172[1][6] + Si7172[2][2]*T17172[2][6] + Si7172[2][3]*T17172[3][6]);
T7172[2][6]=S7271[1][3]*(Si7172[2][1]*T17172[1][4] + Si7172[2][2]*T17172[2][4] + Si7172[2][3]*T17172[3][4]) + S7271[2][3]*(Si7172[2][1]*T17172[1][5] + Si7172[2][2]*T17172[2][5] + Si7172[2][3]*T17172[3][5]);

T7172[3][1]=S7271[1][1]*(Si7172[3][1]*T17172[1][1] + Si7172[3][2]*T17172[2][1]) + S7271[2][1]*(Si7172[3][1]*T17172[1][2] + Si7172[3][2]*T17172[2][2]) + S7271[3][1]*(Si7172[3][1]*T17172[1][3] + Si7172[3][2]*T17172[2][3]);
T7172[3][2]=S7271[1][2]*(Si7172[3][1]*T17172[1][1] + Si7172[3][2]*T17172[2][1]) + S7271[2][2]*(Si7172[3][1]*T17172[1][2] + Si7172[3][2]*T17172[2][2]) + S7271[3][2]*(Si7172[3][1]*T17172[1][3] + Si7172[3][2]*T17172[2][3]) - YHIP*S7271[1][3]*(Si7172[3][1]*T17172[1][4] + Si7172[3][2]*T17172[2][4]) - YHIP*S7271[2][3]*(Si7172[3][1]*T17172[1][5] + Si7172[3][2]*T17172[2][5]);
T7172[3][3]=S7271[1][3]*(Si7172[3][1]*T17172[1][1] + Si7172[3][2]*T17172[2][1]) + S7271[2][3]*(Si7172[3][1]*T17172[1][2] + Si7172[3][2]*T17172[2][2]) + YHIP*S7271[1][2]*(Si7172[3][1]*T17172[1][4] + Si7172[3][2]*T17172[2][4]) + YHIP*S7271[2][2]*(Si7172[3][1]*T17172[1][5] + Si7172[3][2]*T17172[2][5]) + YHIP*S7271[3][2]*(Si7172[3][1]*T17172[1][6] + Si7172[3][2]*T17172[2][6]);
T7172[3][4]=S7271[1][1]*(Si7172[3][1]*T17172[1][4] + Si7172[3][2]*T17172[2][4]) + S7271[2][1]*(Si7172[3][1]*T17172[1][5] + Si7172[3][2]*T17172[2][5]) + S7271[3][1]*(Si7172[3][1]*T17172[1][6] + Si7172[3][2]*T17172[2][6]);
T7172[3][5]=S7271[1][2]*(Si7172[3][1]*T17172[1][4] + Si7172[3][2]*T17172[2][4]) + S7271[2][2]*(Si7172[3][1]*T17172[1][5] + Si7172[3][2]*T17172[2][5]) + S7271[3][2]*(Si7172[3][1]*T17172[1][6] + Si7172[3][2]*T17172[2][6]);
T7172[3][6]=S7271[1][3]*(Si7172[3][1]*T17172[1][4] + Si7172[3][2]*T17172[2][4]) + S7271[2][3]*(Si7172[3][1]*T17172[1][5] + Si7172[3][2]*T17172[2][5]);

T7172[4][1]=S7271[1][1]*(Si7172[1][1]*T17172[4][1] + Si7172[1][2]*T17172[5][1] + Si7172[1][3]*T17172[6][1]) + S7271[2][1]*(Si7172[1][1]*T17172[4][2] + Si7172[1][2]*T17172[5][2] + Si7172[1][3]*T17172[6][2]) + S7271[3][1]*(Si7172[1][1]*T17172[4][3] + Si7172[1][2]*T17172[5][3] + Si7172[1][3]*T17172[6][3]);
T7172[4][2]=S7271[1][2]*(Si7172[1][1]*T17172[4][1] + Si7172[1][2]*T17172[5][1] + Si7172[1][3]*T17172[6][1]) + S7271[2][2]*(Si7172[1][1]*T17172[4][2] + Si7172[1][2]*T17172[5][2] + Si7172[1][3]*T17172[6][2]) + S7271[3][2]*(Si7172[1][1]*T17172[4][3] + Si7172[1][2]*T17172[5][3] + Si7172[1][3]*T17172[6][3]) - YHIP*S7271[1][3]*(Si7172[1][1]*T17172[4][4] + Si7172[1][2]*T17172[5][4] + Si7172[1][3]*T17172[6][4]) - YHIP*S7271[2][3]*(Si7172[1][1]*T17172[4][5] + Si7172[1][2]*T17172[5][5] + Si7172[1][3]*T17172[6][5]);
T7172[4][3]=S7271[1][3]*(Si7172[1][1]*T17172[4][1] + Si7172[1][2]*T17172[5][1] + Si7172[1][3]*T17172[6][1]) + S7271[2][3]*(Si7172[1][1]*T17172[4][2] + Si7172[1][2]*T17172[5][2] + Si7172[1][3]*T17172[6][2]) + YHIP*S7271[1][2]*(Si7172[1][1]*T17172[4][4] + Si7172[1][2]*T17172[5][4] + Si7172[1][3]*T17172[6][4]) + YHIP*S7271[2][2]*(Si7172[1][1]*T17172[4][5] + Si7172[1][2]*T17172[5][5] + Si7172[1][3]*T17172[6][5]) + YHIP*S7271[3][2]*(Si7172[1][1]*T17172[4][6] + Si7172[1][2]*T17172[5][6] + Si7172[1][3]*T17172[6][6]);
T7172[4][4]=S7271[1][1]*(Si7172[1][1]*T17172[4][4] + Si7172[1][2]*T17172[5][4] + Si7172[1][3]*T17172[6][4]) + S7271[2][1]*(Si7172[1][1]*T17172[4][5] + Si7172[1][2]*T17172[5][5] + Si7172[1][3]*T17172[6][5]) + S7271[3][1]*(Si7172[1][1]*T17172[4][6] + Si7172[1][2]*T17172[5][6] + Si7172[1][3]*T17172[6][6]);
T7172[4][5]=S7271[1][2]*(Si7172[1][1]*T17172[4][4] + Si7172[1][2]*T17172[5][4] + Si7172[1][3]*T17172[6][4]) + S7271[2][2]*(Si7172[1][1]*T17172[4][5] + Si7172[1][2]*T17172[5][5] + Si7172[1][3]*T17172[6][5]) + S7271[3][2]*(Si7172[1][1]*T17172[4][6] + Si7172[1][2]*T17172[5][6] + Si7172[1][3]*T17172[6][6]);
T7172[4][6]=S7271[1][3]*(Si7172[1][1]*T17172[4][4] + Si7172[1][2]*T17172[5][4] + Si7172[1][3]*T17172[6][4]) + S7271[2][3]*(Si7172[1][1]*T17172[4][5] + Si7172[1][2]*T17172[5][5] + Si7172[1][3]*T17172[6][5]);

T7172[5][1]=S7271[1][1]*(-(YHIP*Si7172[3][1]*T17172[1][1]) - YHIP*Si7172[3][2]*T17172[2][1] + Si7172[2][1]*T17172[4][1] + Si7172[2][2]*T17172[5][1] + Si7172[2][3]*T17172[6][1]) + S7271[2][1]*(-(YHIP*Si7172[3][1]*T17172[1][2]) - YHIP*Si7172[3][2]*T17172[2][2] + Si7172[2][1]*T17172[4][2] + Si7172[2][2]*T17172[5][2] + Si7172[2][3]*T17172[6][2]) + S7271[3][1]*(-(YHIP*Si7172[3][1]*T17172[1][3]) - YHIP*Si7172[3][2]*T17172[2][3] + Si7172[2][1]*T17172[4][3] + Si7172[2][2]*T17172[5][3] + Si7172[2][3]*T17172[6][3]);
T7172[5][2]=S7271[1][2]*(-(YHIP*Si7172[3][1]*T17172[1][1]) - YHIP*Si7172[3][2]*T17172[2][1] + Si7172[2][1]*T17172[4][1] + Si7172[2][2]*T17172[5][1] + Si7172[2][3]*T17172[6][1]) + S7271[2][2]*(-(YHIP*Si7172[3][1]*T17172[1][2]) - YHIP*Si7172[3][2]*T17172[2][2] + Si7172[2][1]*T17172[4][2] + Si7172[2][2]*T17172[5][2] + Si7172[2][3]*T17172[6][2]) + S7271[3][2]*(-(YHIP*Si7172[3][1]*T17172[1][3]) - YHIP*Si7172[3][2]*T17172[2][3] + Si7172[2][1]*T17172[4][3] + Si7172[2][2]*T17172[5][3] + Si7172[2][3]*T17172[6][3]) - YHIP*S7271[1][3]*(-(YHIP*Si7172[3][1]*T17172[1][4]) - YHIP*Si7172[3][2]*T17172[2][4] + Si7172[2][1]*T17172[4][4] + Si7172[2][2]*T17172[5][4] + Si7172[2][3]*T17172[6][4]) - YHIP*S7271[2][3]*(-(YHIP*Si7172[3][1]*T17172[1][5]) - YHIP*Si7172[3][2]*T17172[2][5] + Si7172[2][1]*T17172[4][5] + Si7172[2][2]*T17172[5][5] + Si7172[2][3]*T17172[6][5]);
T7172[5][3]=S7271[1][3]*(-(YHIP*Si7172[3][1]*T17172[1][1]) - YHIP*Si7172[3][2]*T17172[2][1] + Si7172[2][1]*T17172[4][1] + Si7172[2][2]*T17172[5][1] + Si7172[2][3]*T17172[6][1]) + S7271[2][3]*(-(YHIP*Si7172[3][1]*T17172[1][2]) - YHIP*Si7172[3][2]*T17172[2][2] + Si7172[2][1]*T17172[4][2] + Si7172[2][2]*T17172[5][2] + Si7172[2][3]*T17172[6][2]) + YHIP*S7271[1][2]*(-(YHIP*Si7172[3][1]*T17172[1][4]) - YHIP*Si7172[3][2]*T17172[2][4] + Si7172[2][1]*T17172[4][4] + Si7172[2][2]*T17172[5][4] + Si7172[2][3]*T17172[6][4]) + YHIP*S7271[2][2]*(-(YHIP*Si7172[3][1]*T17172[1][5]) - YHIP*Si7172[3][2]*T17172[2][5] + Si7172[2][1]*T17172[4][5] + Si7172[2][2]*T17172[5][5] + Si7172[2][3]*T17172[6][5]) + YHIP*S7271[3][2]*(-(YHIP*Si7172[3][1]*T17172[1][6]) - YHIP*Si7172[3][2]*T17172[2][6] + Si7172[2][1]*T17172[4][6] + Si7172[2][2]*T17172[5][6] + Si7172[2][3]*T17172[6][6]);
T7172[5][4]=S7271[1][1]*(-(YHIP*Si7172[3][1]*T17172[1][4]) - YHIP*Si7172[3][2]*T17172[2][4] + Si7172[2][1]*T17172[4][4] + Si7172[2][2]*T17172[5][4] + Si7172[2][3]*T17172[6][4]) + S7271[2][1]*(-(YHIP*Si7172[3][1]*T17172[1][5]) - YHIP*Si7172[3][2]*T17172[2][5] + Si7172[2][1]*T17172[4][5] + Si7172[2][2]*T17172[5][5] + Si7172[2][3]*T17172[6][5]) + S7271[3][1]*(-(YHIP*Si7172[3][1]*T17172[1][6]) - YHIP*Si7172[3][2]*T17172[2][6] + Si7172[2][1]*T17172[4][6] + Si7172[2][2]*T17172[5][6] + Si7172[2][3]*T17172[6][6]);
T7172[5][5]=S7271[1][2]*(-(YHIP*Si7172[3][1]*T17172[1][4]) - YHIP*Si7172[3][2]*T17172[2][4] + Si7172[2][1]*T17172[4][4] + Si7172[2][2]*T17172[5][4] + Si7172[2][3]*T17172[6][4]) + S7271[2][2]*(-(YHIP*Si7172[3][1]*T17172[1][5]) - YHIP*Si7172[3][2]*T17172[2][5] + Si7172[2][1]*T17172[4][5] + Si7172[2][2]*T17172[5][5] + Si7172[2][3]*T17172[6][5]) + S7271[3][2]*(-(YHIP*Si7172[3][1]*T17172[1][6]) - YHIP*Si7172[3][2]*T17172[2][6] + Si7172[2][1]*T17172[4][6] + Si7172[2][2]*T17172[5][6] + Si7172[2][3]*T17172[6][6]);
T7172[5][6]=S7271[1][3]*(-(YHIP*Si7172[3][1]*T17172[1][4]) - YHIP*Si7172[3][2]*T17172[2][4] + Si7172[2][1]*T17172[4][4] + Si7172[2][2]*T17172[5][4] + Si7172[2][3]*T17172[6][4]) + S7271[2][3]*(-(YHIP*Si7172[3][1]*T17172[1][5]) - YHIP*Si7172[3][2]*T17172[2][5] + Si7172[2][1]*T17172[4][5] + Si7172[2][2]*T17172[5][5] + Si7172[2][3]*T17172[6][5]);

T7172[6][1]=S7271[1][1]*(YHIP*Si7172[2][1]*T17172[1][1] + YHIP*Si7172[2][2]*T17172[2][1] + YHIP*Si7172[2][3]*T17172[3][1] + Si7172[3][1]*T17172[4][1] + Si7172[3][2]*T17172[5][1]) + S7271[2][1]*(YHIP*Si7172[2][1]*T17172[1][2] + YHIP*Si7172[2][2]*T17172[2][2] + YHIP*Si7172[2][3]*T17172[3][2] + Si7172[3][1]*T17172[4][2] + Si7172[3][2]*T17172[5][2]) + S7271[3][1]*(YHIP*Si7172[2][1]*T17172[1][3] + YHIP*Si7172[2][2]*T17172[2][3] + YHIP*Si7172[2][3]*T17172[3][3] + Si7172[3][1]*T17172[4][3] + Si7172[3][2]*T17172[5][3]);
T7172[6][2]=S7271[1][2]*(YHIP*Si7172[2][1]*T17172[1][1] + YHIP*Si7172[2][2]*T17172[2][1] + YHIP*Si7172[2][3]*T17172[3][1] + Si7172[3][1]*T17172[4][1] + Si7172[3][2]*T17172[5][1]) + S7271[2][2]*(YHIP*Si7172[2][1]*T17172[1][2] + YHIP*Si7172[2][2]*T17172[2][2] + YHIP*Si7172[2][3]*T17172[3][2] + Si7172[3][1]*T17172[4][2] + Si7172[3][2]*T17172[5][2]) + S7271[3][2]*(YHIP*Si7172[2][1]*T17172[1][3] + YHIP*Si7172[2][2]*T17172[2][3] + YHIP*Si7172[2][3]*T17172[3][3] + Si7172[3][1]*T17172[4][3] + Si7172[3][2]*T17172[5][3]) - YHIP*S7271[1][3]*(YHIP*Si7172[2][1]*T17172[1][4] + YHIP*Si7172[2][2]*T17172[2][4] + YHIP*Si7172[2][3]*T17172[3][4] + Si7172[3][1]*T17172[4][4] + Si7172[3][2]*T17172[5][4]) - YHIP*S7271[2][3]*(YHIP*Si7172[2][1]*T17172[1][5] + YHIP*Si7172[2][2]*T17172[2][5] + YHIP*Si7172[2][3]*T17172[3][5] + Si7172[3][1]*T17172[4][5] + Si7172[3][2]*T17172[5][5]);
T7172[6][3]=S7271[1][3]*(YHIP*Si7172[2][1]*T17172[1][1] + YHIP*Si7172[2][2]*T17172[2][1] + YHIP*Si7172[2][3]*T17172[3][1] + Si7172[3][1]*T17172[4][1] + Si7172[3][2]*T17172[5][1]) + S7271[2][3]*(YHIP*Si7172[2][1]*T17172[1][2] + YHIP*Si7172[2][2]*T17172[2][2] + YHIP*Si7172[2][3]*T17172[3][2] + Si7172[3][1]*T17172[4][2] + Si7172[3][2]*T17172[5][2]) + YHIP*S7271[1][2]*(YHIP*Si7172[2][1]*T17172[1][4] + YHIP*Si7172[2][2]*T17172[2][4] + YHIP*Si7172[2][3]*T17172[3][4] + Si7172[3][1]*T17172[4][4] + Si7172[3][2]*T17172[5][4]) + YHIP*S7271[2][2]*(YHIP*Si7172[2][1]*T17172[1][5] + YHIP*Si7172[2][2]*T17172[2][5] + YHIP*Si7172[2][3]*T17172[3][5] + Si7172[3][1]*T17172[4][5] + Si7172[3][2]*T17172[5][5]) + YHIP*S7271[3][2]*(YHIP*Si7172[2][1]*T17172[1][6] + YHIP*Si7172[2][2]*T17172[2][6] + YHIP*Si7172[2][3]*T17172[3][6] + Si7172[3][1]*T17172[4][6] + Si7172[3][2]*T17172[5][6]);
T7172[6][4]=S7271[1][1]*(YHIP*Si7172[2][1]*T17172[1][4] + YHIP*Si7172[2][2]*T17172[2][4] + YHIP*Si7172[2][3]*T17172[3][4] + Si7172[3][1]*T17172[4][4] + Si7172[3][2]*T17172[5][4]) + S7271[2][1]*(YHIP*Si7172[2][1]*T17172[1][5] + YHIP*Si7172[2][2]*T17172[2][5] + YHIP*Si7172[2][3]*T17172[3][5] + Si7172[3][1]*T17172[4][5] + Si7172[3][2]*T17172[5][5]) + S7271[3][1]*(YHIP*Si7172[2][1]*T17172[1][6] + YHIP*Si7172[2][2]*T17172[2][6] + YHIP*Si7172[2][3]*T17172[3][6] + Si7172[3][1]*T17172[4][6] + Si7172[3][2]*T17172[5][6]);
T7172[6][5]=S7271[1][2]*(YHIP*Si7172[2][1]*T17172[1][4] + YHIP*Si7172[2][2]*T17172[2][4] + YHIP*Si7172[2][3]*T17172[3][4] + Si7172[3][1]*T17172[4][4] + Si7172[3][2]*T17172[5][4]) + S7271[2][2]*(YHIP*Si7172[2][1]*T17172[1][5] + YHIP*Si7172[2][2]*T17172[2][5] + YHIP*Si7172[2][3]*T17172[3][5] + Si7172[3][1]*T17172[4][5] + Si7172[3][2]*T17172[5][5]) + S7271[3][2]*(YHIP*Si7172[2][1]*T17172[1][6] + YHIP*Si7172[2][2]*T17172[2][6] + YHIP*Si7172[2][3]*T17172[3][6] + Si7172[3][1]*T17172[4][6] + Si7172[3][2]*T17172[5][6]);
T7172[6][6]=S7271[1][3]*(YHIP*Si7172[2][1]*T17172[1][4] + YHIP*Si7172[2][2]*T17172[2][4] + YHIP*Si7172[2][3]*T17172[3][4] + Si7172[3][1]*T17172[4][4] + Si7172[3][2]*T17172[5][4]) + S7271[2][3]*(YHIP*Si7172[2][1]*T17172[1][5] + YHIP*Si7172[2][2]*T17172[2][5] + YHIP*Si7172[2][3]*T17172[3][5] + Si7172[3][1]*T17172[4][5] + Si7172[3][2]*T17172[5][5]);



}


void
hermes_InvDynArtfunc33(void)
      {
JA71[1][1]=T7172[1][1];
JA71[1][2]=links[22].mcm[3] + T7172[1][2];
JA71[1][3]=-links[22].mcm[2] + T7172[1][3];
JA71[1][4]=links[22].m + T7172[1][4];
JA71[1][5]=T7172[1][5];
JA71[1][6]=T7172[1][6];

JA71[2][1]=-links[22].mcm[3] + T7172[2][1];
JA71[2][2]=T7172[2][2];
JA71[2][3]=links[22].mcm[1] + T7172[2][3];
JA71[2][4]=T7172[2][4];
JA71[2][5]=links[22].m + T7172[2][5];
JA71[2][6]=T7172[2][6];

JA71[3][1]=links[22].mcm[2] + T7172[3][1];
JA71[3][2]=-links[22].mcm[1] + T7172[3][2];
JA71[3][3]=T7172[3][3];
JA71[3][4]=T7172[3][4];
JA71[3][5]=T7172[3][5];
JA71[3][6]=links[22].m + T7172[3][6];

JA71[4][1]=links[22].inertia[1][1] + T7172[4][1];
JA71[4][2]=links[22].inertia[1][2] + T7172[4][2];
JA71[4][3]=links[22].inertia[1][3] + T7172[4][3];
JA71[4][4]=T7172[4][4];
JA71[4][5]=-links[22].mcm[3] + T7172[4][5];
JA71[4][6]=links[22].mcm[2] + T7172[4][6];

JA71[5][1]=links[22].inertia[1][2] + T7172[5][1];
JA71[5][2]=links[22].inertia[2][2] + T7172[5][2];
JA71[5][3]=links[22].inertia[2][3] + T7172[5][3];
JA71[5][4]=links[22].mcm[3] + T7172[5][4];
JA71[5][5]=T7172[5][5];
JA71[5][6]=-links[22].mcm[1] + T7172[5][6];

JA71[6][1]=links[22].inertia[1][3] + T7172[6][1];
JA71[6][2]=links[22].inertia[2][3] + T7172[6][2];
JA71[6][3]=links[22].inertia[3][3] + T7172[6][3];
JA71[6][4]=-links[22].mcm[2] + T7172[6][4];
JA71[6][5]=links[22].mcm[1] + T7172[6][5];
JA71[6][6]=T7172[6][6];


h71[1]=JA71[1][3];
h71[2]=JA71[2][3];
h71[3]=JA71[3][3];
h71[4]=JA71[4][3];
h71[5]=JA71[5][3];
h71[6]=JA71[6][3];

T17071[1][1]=JA71[1][1];
T17071[1][2]=JA71[1][2];
T17071[1][3]=JA71[1][3];
T17071[1][4]=JA71[1][4];
T17071[1][5]=JA71[1][5];
T17071[1][6]=JA71[1][6];

T17071[2][1]=JA71[2][1];
T17071[2][2]=JA71[2][2];
T17071[2][3]=JA71[2][3];
T17071[2][4]=JA71[2][4];
T17071[2][5]=JA71[2][5];
T17071[2][6]=JA71[2][6];

T17071[3][1]=JA71[3][1];
T17071[3][2]=JA71[3][2];
T17071[3][3]=JA71[3][3];
T17071[3][4]=JA71[3][4];
T17071[3][5]=JA71[3][5];
T17071[3][6]=JA71[3][6];

T17071[4][1]=JA71[4][1];
T17071[4][2]=JA71[4][2];
T17071[4][3]=JA71[4][3];
T17071[4][4]=JA71[4][4];
T17071[4][5]=JA71[4][5];
T17071[4][6]=JA71[4][6];

T17071[5][1]=JA71[5][1];
T17071[5][2]=JA71[5][2];
T17071[5][3]=JA71[5][3];
T17071[5][4]=JA71[5][4];
T17071[5][5]=JA71[5][5];
T17071[5][6]=JA71[5][6];

T17071[6][1]=JA71[6][1];
T17071[6][2]=JA71[6][2];
T17071[6][3]=JA71[6][3];
T17071[6][4]=JA71[6][4];
T17071[6][5]=JA71[6][5];
T17071[6][6]=JA71[6][6];


T7071[1][1]=S7170[1][1]*(Si7071[1][1]*T17071[1][1] + Si7071[1][2]*T17071[2][1]) + S7170[2][1]*(Si7071[1][1]*T17071[1][2] + Si7071[1][2]*T17071[2][2]);
T7071[1][2]=-(Si7071[1][1]*T17071[1][3]) - Si7071[1][2]*T17071[2][3];
T7071[1][3]=S7170[1][3]*(Si7071[1][1]*T17071[1][1] + Si7071[1][2]*T17071[2][1]) + S7170[2][3]*(Si7071[1][1]*T17071[1][2] + Si7071[1][2]*T17071[2][2]);
T7071[1][4]=S7170[1][1]*(Si7071[1][1]*T17071[1][4] + Si7071[1][2]*T17071[2][4]) + S7170[2][1]*(Si7071[1][1]*T17071[1][5] + Si7071[1][2]*T17071[2][5]);
T7071[1][5]=-(Si7071[1][1]*T17071[1][6]) - Si7071[1][2]*T17071[2][6];
T7071[1][6]=S7170[1][3]*(Si7071[1][1]*T17071[1][4] + Si7071[1][2]*T17071[2][4]) + S7170[2][3]*(Si7071[1][1]*T17071[1][5] + Si7071[1][2]*T17071[2][5]);

T7071[2][1]=-(S7170[1][1]*T17071[3][1]) - S7170[2][1]*T17071[3][2];
T7071[2][2]=T17071[3][3];
T7071[2][3]=-(S7170[1][3]*T17071[3][1]) - S7170[2][3]*T17071[3][2];
T7071[2][4]=-(S7170[1][1]*T17071[3][4]) - S7170[2][1]*T17071[3][5];
T7071[2][5]=T17071[3][6];
T7071[2][6]=-(S7170[1][3]*T17071[3][4]) - S7170[2][3]*T17071[3][5];

T7071[3][1]=S7170[1][1]*(Si7071[3][1]*T17071[1][1] + Si7071[3][2]*T17071[2][1]) + S7170[2][1]*(Si7071[3][1]*T17071[1][2] + Si7071[3][2]*T17071[2][2]);
T7071[3][2]=-(Si7071[3][1]*T17071[1][3]) - Si7071[3][2]*T17071[2][3];
T7071[3][3]=S7170[1][3]*(Si7071[3][1]*T17071[1][1] + Si7071[3][2]*T17071[2][1]) + S7170[2][3]*(Si7071[3][1]*T17071[1][2] + Si7071[3][2]*T17071[2][2]);
T7071[3][4]=S7170[1][1]*(Si7071[3][1]*T17071[1][4] + Si7071[3][2]*T17071[2][4]) + S7170[2][1]*(Si7071[3][1]*T17071[1][5] + Si7071[3][2]*T17071[2][5]);
T7071[3][5]=-(Si7071[3][1]*T17071[1][6]) - Si7071[3][2]*T17071[2][6];
T7071[3][6]=S7170[1][3]*(Si7071[3][1]*T17071[1][4] + Si7071[3][2]*T17071[2][4]) + S7170[2][3]*(Si7071[3][1]*T17071[1][5] + Si7071[3][2]*T17071[2][5]);

T7071[4][1]=S7170[1][1]*(Si7071[1][1]*T17071[4][1] + Si7071[1][2]*T17071[5][1]) + S7170[2][1]*(Si7071[1][1]*T17071[4][2] + Si7071[1][2]*T17071[5][2]);
T7071[4][2]=-(Si7071[1][1]*T17071[4][3]) - Si7071[1][2]*T17071[5][3];
T7071[4][3]=S7170[1][3]*(Si7071[1][1]*T17071[4][1] + Si7071[1][2]*T17071[5][1]) + S7170[2][3]*(Si7071[1][1]*T17071[4][2] + Si7071[1][2]*T17071[5][2]);
T7071[4][4]=S7170[1][1]*(Si7071[1][1]*T17071[4][4] + Si7071[1][2]*T17071[5][4]) + S7170[2][1]*(Si7071[1][1]*T17071[4][5] + Si7071[1][2]*T17071[5][5]);
T7071[4][5]=-(Si7071[1][1]*T17071[4][6]) - Si7071[1][2]*T17071[5][6];
T7071[4][6]=S7170[1][3]*(Si7071[1][1]*T17071[4][4] + Si7071[1][2]*T17071[5][4]) + S7170[2][3]*(Si7071[1][1]*T17071[4][5] + Si7071[1][2]*T17071[5][5]);

T7071[5][1]=-(S7170[1][1]*T17071[6][1]) - S7170[2][1]*T17071[6][2];
T7071[5][2]=T17071[6][3];
T7071[5][3]=-(S7170[1][3]*T17071[6][1]) - S7170[2][3]*T17071[6][2];
T7071[5][4]=-(S7170[1][1]*T17071[6][4]) - S7170[2][1]*T17071[6][5];
T7071[5][5]=T17071[6][6];
T7071[5][6]=-(S7170[1][3]*T17071[6][4]) - S7170[2][3]*T17071[6][5];

T7071[6][1]=S7170[1][1]*(Si7071[3][1]*T17071[4][1] + Si7071[3][2]*T17071[5][1]) + S7170[2][1]*(Si7071[3][1]*T17071[4][2] + Si7071[3][2]*T17071[5][2]);
T7071[6][2]=-(Si7071[3][1]*T17071[4][3]) - Si7071[3][2]*T17071[5][3];
T7071[6][3]=S7170[1][3]*(Si7071[3][1]*T17071[4][1] + Si7071[3][2]*T17071[5][1]) + S7170[2][3]*(Si7071[3][1]*T17071[4][2] + Si7071[3][2]*T17071[5][2]);
T7071[6][4]=S7170[1][1]*(Si7071[3][1]*T17071[4][4] + Si7071[3][2]*T17071[5][4]) + S7170[2][1]*(Si7071[3][1]*T17071[4][5] + Si7071[3][2]*T17071[5][5]);
T7071[6][5]=-(Si7071[3][1]*T17071[4][6]) - Si7071[3][2]*T17071[5][6];
T7071[6][6]=S7170[1][3]*(Si7071[3][1]*T17071[4][4] + Si7071[3][2]*T17071[5][4]) + S7170[2][3]*(Si7071[3][1]*T17071[4][5] + Si7071[3][2]*T17071[5][5]);



}


void
hermes_InvDynArtfunc34(void)
      {
JA70[1][1]=T7071[1][1];
JA70[1][2]=links[23].mcm[3] + T7071[1][2];
JA70[1][3]=-links[23].mcm[2] + T7071[1][3];
JA70[1][4]=links[23].m + T7071[1][4];
JA70[1][5]=T7071[1][5];
JA70[1][6]=T7071[1][6];

JA70[2][1]=-links[23].mcm[3] + T7071[2][1];
JA70[2][2]=T7071[2][2];
JA70[2][3]=links[23].mcm[1] + T7071[2][3];
JA70[2][4]=T7071[2][4];
JA70[2][5]=links[23].m + T7071[2][5];
JA70[2][6]=T7071[2][6];

JA70[3][1]=links[23].mcm[2] + T7071[3][1];
JA70[3][2]=-links[23].mcm[1] + T7071[3][2];
JA70[3][3]=T7071[3][3];
JA70[3][4]=T7071[3][4];
JA70[3][5]=T7071[3][5];
JA70[3][6]=links[23].m + T7071[3][6];

JA70[4][1]=links[23].inertia[1][1] + T7071[4][1];
JA70[4][2]=links[23].inertia[1][2] + T7071[4][2];
JA70[4][3]=links[23].inertia[1][3] + T7071[4][3];
JA70[4][4]=T7071[4][4];
JA70[4][5]=-links[23].mcm[3] + T7071[4][5];
JA70[4][6]=links[23].mcm[2] + T7071[4][6];

JA70[5][1]=links[23].inertia[1][2] + T7071[5][1];
JA70[5][2]=links[23].inertia[2][2] + T7071[5][2];
JA70[5][3]=links[23].inertia[2][3] + T7071[5][3];
JA70[5][4]=links[23].mcm[3] + T7071[5][4];
JA70[5][5]=T7071[5][5];
JA70[5][6]=-links[23].mcm[1] + T7071[5][6];

JA70[6][1]=links[23].inertia[1][3] + T7071[6][1];
JA70[6][2]=links[23].inertia[2][3] + T7071[6][2];
JA70[6][3]=links[23].inertia[3][3] + T7071[6][3];
JA70[6][4]=-links[23].mcm[2] + T7071[6][4];
JA70[6][5]=links[23].mcm[1] + T7071[6][5];
JA70[6][6]=T7071[6][6];


h70[1]=JA70[1][3];
h70[2]=JA70[2][3];
h70[3]=JA70[3][3];
h70[4]=JA70[4][3];
h70[5]=JA70[5][3];
h70[6]=JA70[6][3];

T1070[1][1]=JA70[1][1];
T1070[1][2]=JA70[1][2];
T1070[1][3]=JA70[1][3];
T1070[1][4]=JA70[1][4];
T1070[1][5]=JA70[1][5];
T1070[1][6]=JA70[1][6];

T1070[2][1]=JA70[2][1];
T1070[2][2]=JA70[2][2];
T1070[2][3]=JA70[2][3];
T1070[2][4]=JA70[2][4];
T1070[2][5]=JA70[2][5];
T1070[2][6]=JA70[2][6];

T1070[3][1]=JA70[3][1];
T1070[3][2]=JA70[3][2];
T1070[3][3]=JA70[3][3];
T1070[3][4]=JA70[3][4];
T1070[3][5]=JA70[3][5];
T1070[3][6]=JA70[3][6];

T1070[4][1]=JA70[4][1];
T1070[4][2]=JA70[4][2];
T1070[4][3]=JA70[4][3];
T1070[4][4]=JA70[4][4];
T1070[4][5]=JA70[4][5];
T1070[4][6]=JA70[4][6];

T1070[5][1]=JA70[5][1];
T1070[5][2]=JA70[5][2];
T1070[5][3]=JA70[5][3];
T1070[5][4]=JA70[5][4];
T1070[5][5]=JA70[5][5];
T1070[5][6]=JA70[5][6];

T1070[6][1]=JA70[6][1];
T1070[6][2]=JA70[6][2];
T1070[6][3]=JA70[6][3];
T1070[6][4]=JA70[6][4];
T1070[6][5]=JA70[6][5];
T1070[6][6]=JA70[6][6];


T070[1][1]=S700[1][1]*(Si070[1][1]*T1070[1][1] + Si070[1][2]*T1070[2][1]) + S700[2][1]*(Si070[1][1]*T1070[1][2] + Si070[1][2]*T1070[2][2]);
T070[1][2]=Si070[1][1]*T1070[1][3] + Si070[1][2]*T1070[2][3] - XHIP*S700[1][3]*(Si070[1][1]*T1070[1][4] + Si070[1][2]*T1070[2][4]) - XHIP*S700[2][3]*(Si070[1][1]*T1070[1][5] + Si070[1][2]*T1070[2][5]);
T070[1][3]=S700[1][3]*(Si070[1][1]*T1070[1][1] + Si070[1][2]*T1070[2][1]) + S700[2][3]*(Si070[1][1]*T1070[1][2] + Si070[1][2]*T1070[2][2]) + XHIP*(Si070[1][1]*T1070[1][6] + Si070[1][2]*T1070[2][6]);
T070[1][4]=S700[1][1]*(Si070[1][1]*T1070[1][4] + Si070[1][2]*T1070[2][4]) + S700[2][1]*(Si070[1][1]*T1070[1][5] + Si070[1][2]*T1070[2][5]);
T070[1][5]=Si070[1][1]*T1070[1][6] + Si070[1][2]*T1070[2][6];
T070[1][6]=S700[1][3]*(Si070[1][1]*T1070[1][4] + Si070[1][2]*T1070[2][4]) + S700[2][3]*(Si070[1][1]*T1070[1][5] + Si070[1][2]*T1070[2][5]);

T070[2][1]=S700[1][1]*T1070[3][1] + S700[2][1]*T1070[3][2];
T070[2][2]=T1070[3][3] - XHIP*S700[1][3]*T1070[3][4] - XHIP*S700[2][3]*T1070[3][5];
T070[2][3]=S700[1][3]*T1070[3][1] + S700[2][3]*T1070[3][2] + XHIP*T1070[3][6];
T070[2][4]=S700[1][1]*T1070[3][4] + S700[2][1]*T1070[3][5];
T070[2][5]=T1070[3][6];
T070[2][6]=S700[1][3]*T1070[3][4] + S700[2][3]*T1070[3][5];

T070[3][1]=S700[1][1]*(Si070[3][1]*T1070[1][1] + Si070[3][2]*T1070[2][1]) + S700[2][1]*(Si070[3][1]*T1070[1][2] + Si070[3][2]*T1070[2][2]);
T070[3][2]=Si070[3][1]*T1070[1][3] + Si070[3][2]*T1070[2][3] - XHIP*S700[1][3]*(Si070[3][1]*T1070[1][4] + Si070[3][2]*T1070[2][4]) - XHIP*S700[2][3]*(Si070[3][1]*T1070[1][5] + Si070[3][2]*T1070[2][5]);
T070[3][3]=S700[1][3]*(Si070[3][1]*T1070[1][1] + Si070[3][2]*T1070[2][1]) + S700[2][3]*(Si070[3][1]*T1070[1][2] + Si070[3][2]*T1070[2][2]) + XHIP*(Si070[3][1]*T1070[1][6] + Si070[3][2]*T1070[2][6]);
T070[3][4]=S700[1][1]*(Si070[3][1]*T1070[1][4] + Si070[3][2]*T1070[2][4]) + S700[2][1]*(Si070[3][1]*T1070[1][5] + Si070[3][2]*T1070[2][5]);
T070[3][5]=Si070[3][1]*T1070[1][6] + Si070[3][2]*T1070[2][6];
T070[3][6]=S700[1][3]*(Si070[3][1]*T1070[1][4] + Si070[3][2]*T1070[2][4]) + S700[2][3]*(Si070[3][1]*T1070[1][5] + Si070[3][2]*T1070[2][5]);

T070[4][1]=S700[1][1]*(Si070[1][1]*T1070[4][1] + Si070[1][2]*T1070[5][1]) + S700[2][1]*(Si070[1][1]*T1070[4][2] + Si070[1][2]*T1070[5][2]);
T070[4][2]=Si070[1][1]*T1070[4][3] + Si070[1][2]*T1070[5][3] - XHIP*S700[1][3]*(Si070[1][1]*T1070[4][4] + Si070[1][2]*T1070[5][4]) - XHIP*S700[2][3]*(Si070[1][1]*T1070[4][5] + Si070[1][2]*T1070[5][5]);
T070[4][3]=S700[1][3]*(Si070[1][1]*T1070[4][1] + Si070[1][2]*T1070[5][1]) + S700[2][3]*(Si070[1][1]*T1070[4][2] + Si070[1][2]*T1070[5][2]) + XHIP*(Si070[1][1]*T1070[4][6] + Si070[1][2]*T1070[5][6]);
T070[4][4]=S700[1][1]*(Si070[1][1]*T1070[4][4] + Si070[1][2]*T1070[5][4]) + S700[2][1]*(Si070[1][1]*T1070[4][5] + Si070[1][2]*T1070[5][5]);
T070[4][5]=Si070[1][1]*T1070[4][6] + Si070[1][2]*T1070[5][6];
T070[4][6]=S700[1][3]*(Si070[1][1]*T1070[4][4] + Si070[1][2]*T1070[5][4]) + S700[2][3]*(Si070[1][1]*T1070[4][5] + Si070[1][2]*T1070[5][5]);

T070[5][1]=S700[1][1]*(-(XHIP*Si070[3][1]*T1070[1][1]) - XHIP*Si070[3][2]*T1070[2][1] + T1070[6][1]) + S700[2][1]*(-(XHIP*Si070[3][1]*T1070[1][2]) - XHIP*Si070[3][2]*T1070[2][2] + T1070[6][2]);
T070[5][2]=-(XHIP*Si070[3][1]*T1070[1][3]) - XHIP*Si070[3][2]*T1070[2][3] + T1070[6][3] - XHIP*S700[1][3]*(-(XHIP*Si070[3][1]*T1070[1][4]) - XHIP*Si070[3][2]*T1070[2][4] + T1070[6][4]) - XHIP*S700[2][3]*(-(XHIP*Si070[3][1]*T1070[1][5]) - XHIP*Si070[3][2]*T1070[2][5] + T1070[6][5]);
T070[5][3]=S700[1][3]*(-(XHIP*Si070[3][1]*T1070[1][1]) - XHIP*Si070[3][2]*T1070[2][1] + T1070[6][1]) + S700[2][3]*(-(XHIP*Si070[3][1]*T1070[1][2]) - XHIP*Si070[3][2]*T1070[2][2] + T1070[6][2]) + XHIP*(-(XHIP*Si070[3][1]*T1070[1][6]) - XHIP*Si070[3][2]*T1070[2][6] + T1070[6][6]);
T070[5][4]=S700[1][1]*(-(XHIP*Si070[3][1]*T1070[1][4]) - XHIP*Si070[3][2]*T1070[2][4] + T1070[6][4]) + S700[2][1]*(-(XHIP*Si070[3][1]*T1070[1][5]) - XHIP*Si070[3][2]*T1070[2][5] + T1070[6][5]);
T070[5][5]=-(XHIP*Si070[3][1]*T1070[1][6]) - XHIP*Si070[3][2]*T1070[2][6] + T1070[6][6];
T070[5][6]=S700[1][3]*(-(XHIP*Si070[3][1]*T1070[1][4]) - XHIP*Si070[3][2]*T1070[2][4] + T1070[6][4]) + S700[2][3]*(-(XHIP*Si070[3][1]*T1070[1][5]) - XHIP*Si070[3][2]*T1070[2][5] + T1070[6][5]);

T070[6][1]=S700[1][1]*(XHIP*T1070[3][1] + Si070[3][1]*T1070[4][1] + Si070[3][2]*T1070[5][1]) + S700[2][1]*(XHIP*T1070[3][2] + Si070[3][1]*T1070[4][2] + Si070[3][2]*T1070[5][2]);
T070[6][2]=XHIP*T1070[3][3] + Si070[3][1]*T1070[4][3] + Si070[3][2]*T1070[5][3] - XHIP*S700[1][3]*(XHIP*T1070[3][4] + Si070[3][1]*T1070[4][4] + Si070[3][2]*T1070[5][4]) - XHIP*S700[2][3]*(XHIP*T1070[3][5] + Si070[3][1]*T1070[4][5] + Si070[3][2]*T1070[5][5]);
T070[6][3]=S700[1][3]*(XHIP*T1070[3][1] + Si070[3][1]*T1070[4][1] + Si070[3][2]*T1070[5][1]) + S700[2][3]*(XHIP*T1070[3][2] + Si070[3][1]*T1070[4][2] + Si070[3][2]*T1070[5][2]) + XHIP*(XHIP*T1070[3][6] + Si070[3][1]*T1070[4][6] + Si070[3][2]*T1070[5][6]);
T070[6][4]=S700[1][1]*(XHIP*T1070[3][4] + Si070[3][1]*T1070[4][4] + Si070[3][2]*T1070[5][4]) + S700[2][1]*(XHIP*T1070[3][5] + Si070[3][1]*T1070[4][5] + Si070[3][2]*T1070[5][5]);
T070[6][5]=XHIP*T1070[3][6] + Si070[3][1]*T1070[4][6] + Si070[3][2]*T1070[5][6];
T070[6][6]=S700[1][3]*(XHIP*T1070[3][4] + Si070[3][1]*T1070[4][4] + Si070[3][2]*T1070[5][4]) + S700[2][3]*(XHIP*T1070[3][5] + Si070[3][1]*T1070[4][5] + Si070[3][2]*T1070[5][5]);



}


void
hermes_InvDynArtfunc35(void)
      {




}


void
hermes_InvDynArtfunc36(void)
      {




}


void
hermes_InvDynArtfunc37(void)
      {
JA67[1][2]=0. + links[38].mcm[3];
JA67[1][3]=0. - links[38].mcm[2];
JA67[1][4]=0. + links[38].m;

JA67[2][1]=0. - links[38].mcm[3];
JA67[2][3]=0. + links[38].mcm[1];
JA67[2][5]=0. + links[38].m;

JA67[3][1]=0. + links[38].mcm[2];
JA67[3][2]=0. - links[38].mcm[1];
JA67[3][6]=0. + links[38].m;

JA67[4][1]=0. + links[38].inertia[1][1];
JA67[4][2]=0. + links[38].inertia[1][2];
JA67[4][3]=0. + links[38].inertia[1][3];
JA67[4][5]=0. - links[38].mcm[3];
JA67[4][6]=0. + links[38].mcm[2];

JA67[5][1]=0. + links[38].inertia[1][2];
JA67[5][2]=0. + links[38].inertia[2][2];
JA67[5][3]=0. + links[38].inertia[2][3];
JA67[5][4]=0. + links[38].mcm[3];
JA67[5][6]=0. - links[38].mcm[1];

JA67[6][1]=0. + links[38].inertia[1][3];
JA67[6][2]=0. + links[38].inertia[2][3];
JA67[6][3]=0. + links[38].inertia[3][3];
JA67[6][4]=0. - links[38].mcm[2];
JA67[6][5]=0. + links[38].mcm[1];


h67[1]=JA67[1][3];
h67[2]=JA67[2][3];
h67[4]=JA67[4][3];
h67[5]=JA67[5][3];
h67[6]=JA67[6][3];

T16667[1][2]=JA67[1][2];
T16667[1][3]=JA67[1][3];
T16667[1][4]=JA67[1][4];

T16667[2][1]=JA67[2][1];
T16667[2][3]=JA67[2][3];
T16667[2][5]=JA67[2][5];

T16667[3][1]=JA67[3][1];
T16667[3][2]=JA67[3][2];
T16667[3][6]=JA67[3][6];

T16667[4][1]=JA67[4][1];
T16667[4][2]=JA67[4][2];
T16667[4][3]=JA67[4][3];
T16667[4][5]=JA67[4][5];
T16667[4][6]=JA67[4][6];

T16667[5][1]=JA67[5][1];
T16667[5][2]=JA67[5][2];
T16667[5][3]=JA67[5][3];
T16667[5][4]=JA67[5][4];
T16667[5][6]=JA67[5][6];

T16667[6][1]=JA67[6][1];
T16667[6][2]=JA67[6][2];
T16667[6][3]=JA67[6][3];
T16667[6][4]=JA67[6][4];
T16667[6][5]=JA67[6][5];


T6667[1][2]=-(S6766[1][2]*T16667[3][1]) - S6766[2][2]*T16667[3][2];
T6667[1][3]=-(S6766[1][3]*T16667[3][1]) - S6766[2][3]*T16667[3][2];
T6667[1][4]=T16667[3][6];

T6667[2][1]=-(Si6667[2][1]*T16667[1][3]) - Si6667[2][2]*T16667[2][3];
T6667[2][2]=S6766[2][2]*Si6667[2][1]*T16667[1][2] + S6766[1][2]*Si6667[2][2]*T16667[2][1];
T6667[2][3]=S6766[2][3]*Si6667[2][1]*T16667[1][2] + S6766[1][3]*Si6667[2][2]*T16667[2][1];
T6667[2][5]=S6766[1][2]*Si6667[2][1]*T16667[1][4] + S6766[2][2]*Si6667[2][2]*T16667[2][5];
T6667[2][6]=S6766[1][3]*Si6667[2][1]*T16667[1][4] + S6766[2][3]*Si6667[2][2]*T16667[2][5];

T6667[3][1]=-(Si6667[3][1]*T16667[1][3]) - Si6667[3][2]*T16667[2][3];
T6667[3][2]=S6766[2][2]*Si6667[3][1]*T16667[1][2] + S6766[1][2]*Si6667[3][2]*T16667[2][1];
T6667[3][3]=S6766[2][3]*Si6667[3][1]*T16667[1][2] + S6766[1][3]*Si6667[3][2]*T16667[2][1];
T6667[3][5]=S6766[1][2]*Si6667[3][1]*T16667[1][4] + S6766[2][2]*Si6667[3][2]*T16667[2][5];
T6667[3][6]=S6766[1][3]*Si6667[3][1]*T16667[1][4] + S6766[2][3]*Si6667[3][2]*T16667[2][5];

T6667[4][1]=T16667[6][3];
T6667[4][2]=-(S6766[1][2]*T16667[6][1]) - S6766[2][2]*T16667[6][2];
T6667[4][3]=-(S6766[1][3]*T16667[6][1]) - S6766[2][3]*T16667[6][2];
T6667[4][5]=-(S6766[1][2]*T16667[6][4]) - S6766[2][2]*T16667[6][5];
T6667[4][6]=-(S6766[1][3]*T16667[6][4]) - S6766[2][3]*T16667[6][5];

T6667[5][1]=-(Si6667[2][1]*T16667[4][3]) - Si6667[2][2]*T16667[5][3];
T6667[5][2]=S6766[1][2]*(Si6667[2][1]*T16667[4][1] + Si6667[2][2]*T16667[5][1]) + S6766[2][2]*(Si6667[2][1]*T16667[4][2] + Si6667[2][2]*T16667[5][2]);
T6667[5][3]=S6766[1][3]*(Si6667[2][1]*T16667[4][1] + Si6667[2][2]*T16667[5][1]) + S6766[2][3]*(Si6667[2][1]*T16667[4][2] + Si6667[2][2]*T16667[5][2]);
T6667[5][4]=-(Si6667[2][1]*T16667[4][6]) - Si6667[2][2]*T16667[5][6];
T6667[5][5]=S6766[2][2]*Si6667[2][1]*T16667[4][5] + S6766[1][2]*Si6667[2][2]*T16667[5][4];
T6667[5][6]=S6766[2][3]*Si6667[2][1]*T16667[4][5] + S6766[1][3]*Si6667[2][2]*T16667[5][4];

T6667[6][1]=-(Si6667[3][1]*T16667[4][3]) - Si6667[3][2]*T16667[5][3];
T6667[6][2]=S6766[1][2]*(Si6667[3][1]*T16667[4][1] + Si6667[3][2]*T16667[5][1]) + S6766[2][2]*(Si6667[3][1]*T16667[4][2] + Si6667[3][2]*T16667[5][2]);
T6667[6][3]=S6766[1][3]*(Si6667[3][1]*T16667[4][1] + Si6667[3][2]*T16667[5][1]) + S6766[2][3]*(Si6667[3][1]*T16667[4][2] + Si6667[3][2]*T16667[5][2]);
T6667[6][4]=-(Si6667[3][1]*T16667[4][6]) - Si6667[3][2]*T16667[5][6];
T6667[6][5]=S6766[2][2]*Si6667[3][1]*T16667[4][5] + S6766[1][2]*Si6667[3][2]*T16667[5][4];
T6667[6][6]=S6766[2][3]*Si6667[3][1]*T16667[4][5] + S6766[1][3]*Si6667[3][2]*T16667[5][4];



}


void
hermes_InvDynArtfunc38(void)
      {
JA66[1][2]=links[37].mcm[3] + T6667[1][2];
JA66[1][3]=-links[37].mcm[2] + T6667[1][3];
JA66[1][4]=links[37].m + T6667[1][4];

JA66[2][1]=-links[37].mcm[3] + T6667[2][1];
JA66[2][2]=T6667[2][2];
JA66[2][3]=links[37].mcm[1] + T6667[2][3];
JA66[2][5]=links[37].m + T6667[2][5];
JA66[2][6]=T6667[2][6];

JA66[3][1]=links[37].mcm[2] + T6667[3][1];
JA66[3][2]=-links[37].mcm[1] + T6667[3][2];
JA66[3][3]=T6667[3][3];
JA66[3][5]=T6667[3][5];
JA66[3][6]=links[37].m + T6667[3][6];

JA66[4][1]=links[37].inertia[1][1] + T6667[4][1];
JA66[4][2]=links[37].inertia[1][2] + T6667[4][2];
JA66[4][3]=links[37].inertia[1][3] + T6667[4][3];
JA66[4][5]=-links[37].mcm[3] + T6667[4][5];
JA66[4][6]=links[37].mcm[2] + T6667[4][6];

JA66[5][1]=links[37].inertia[1][2] + T6667[5][1];
JA66[5][2]=links[37].inertia[2][2] + T6667[5][2];
JA66[5][3]=links[37].inertia[2][3] + T6667[5][3];
JA66[5][4]=links[37].mcm[3] + T6667[5][4];
JA66[5][5]=T6667[5][5];
JA66[5][6]=-links[37].mcm[1] + T6667[5][6];

JA66[6][1]=links[37].inertia[1][3] + T6667[6][1];
JA66[6][2]=links[37].inertia[2][3] + T6667[6][2];
JA66[6][3]=links[37].inertia[3][3] + T6667[6][3];
JA66[6][4]=-links[37].mcm[2] + T6667[6][4];
JA66[6][5]=links[37].mcm[1] + T6667[6][5];
JA66[6][6]=T6667[6][6];


h66[1]=JA66[1][3];
h66[2]=JA66[2][3];
h66[3]=JA66[3][3];
h66[4]=JA66[4][3];
h66[5]=JA66[5][3];
h66[6]=JA66[6][3];

T16266[1][2]=JA66[1][2];
T16266[1][3]=JA66[1][3];
T16266[1][4]=JA66[1][4];

T16266[2][1]=JA66[2][1];
T16266[2][2]=JA66[2][2];
T16266[2][3]=JA66[2][3];
T16266[2][5]=JA66[2][5];
T16266[2][6]=JA66[2][6];

T16266[3][1]=JA66[3][1];
T16266[3][2]=JA66[3][2];
T16266[3][3]=JA66[3][3];
T16266[3][5]=JA66[3][5];
T16266[3][6]=JA66[3][6];

T16266[4][1]=JA66[4][1];
T16266[4][2]=JA66[4][2];
T16266[4][3]=JA66[4][3];
T16266[4][5]=JA66[4][5];
T16266[4][6]=JA66[4][6];

T16266[5][1]=JA66[5][1];
T16266[5][2]=JA66[5][2];
T16266[5][3]=JA66[5][3];
T16266[5][4]=JA66[5][4];
T16266[5][5]=JA66[5][5];
T16266[5][6]=JA66[5][6];

T16266[6][1]=JA66[6][1];
T16266[6][2]=JA66[6][2];
T16266[6][3]=JA66[6][3];
T16266[6][4]=JA66[6][4];
T16266[6][5]=JA66[6][5];
T16266[6][6]=JA66[6][6];


T6266[1][1]=HEAD*S6662[1][2]*Si6266[1][1]*T16266[1][4] + S6662[1][1]*Si6266[1][2]*T16266[2][1] + S6662[2][1]*(Si6266[1][1]*T16266[1][2] + Si6266[1][2]*T16266[2][2]) + HEAD*S6662[2][2]*Si6266[1][2]*T16266[2][5] - EYEYOFF*Si6266[1][2]*T16266[2][6];
T6266[1][2]=-(HEAD*S6662[1][1]*Si6266[1][1]*T16266[1][4]) + S6662[1][2]*Si6266[1][2]*T16266[2][1] + S6662[2][2]*(Si6266[1][1]*T16266[1][2] + Si6266[1][2]*T16266[2][2]) - HEAD*S6662[2][1]*Si6266[1][2]*T16266[2][5] + EYEXOFF*Si6266[1][2]*T16266[2][6];
T6266[1][3]=Si6266[1][1]*T16266[1][3] + (EYEYOFF*S6662[1][1] - EYEXOFF*S6662[1][2])*Si6266[1][1]*T16266[1][4] + Si6266[1][2]*T16266[2][3] + (EYEYOFF*S6662[2][1] - EYEXOFF*S6662[2][2])*Si6266[1][2]*T16266[2][5];
T6266[1][4]=S6662[1][1]*Si6266[1][1]*T16266[1][4] + S6662[2][1]*Si6266[1][2]*T16266[2][5];
T6266[1][5]=S6662[1][2]*Si6266[1][1]*T16266[1][4] + S6662[2][2]*Si6266[1][2]*T16266[2][5];
T6266[1][6]=Si6266[1][2]*T16266[2][6];

T6266[2][1]=HEAD*S6662[1][2]*Si6266[2][1]*T16266[1][4] + S6662[1][1]*Si6266[2][2]*T16266[2][1] + S6662[2][1]*(Si6266[2][1]*T16266[1][2] + Si6266[2][2]*T16266[2][2]) + HEAD*S6662[2][2]*Si6266[2][2]*T16266[2][5] - EYEYOFF*Si6266[2][2]*T16266[2][6];
T6266[2][2]=-(HEAD*S6662[1][1]*Si6266[2][1]*T16266[1][4]) + S6662[1][2]*Si6266[2][2]*T16266[2][1] + S6662[2][2]*(Si6266[2][1]*T16266[1][2] + Si6266[2][2]*T16266[2][2]) - HEAD*S6662[2][1]*Si6266[2][2]*T16266[2][5] + EYEXOFF*Si6266[2][2]*T16266[2][6];
T6266[2][3]=Si6266[2][1]*T16266[1][3] + (EYEYOFF*S6662[1][1] - EYEXOFF*S6662[1][2])*Si6266[2][1]*T16266[1][4] + Si6266[2][2]*T16266[2][3] + (EYEYOFF*S6662[2][1] - EYEXOFF*S6662[2][2])*Si6266[2][2]*T16266[2][5];
T6266[2][4]=S6662[1][1]*Si6266[2][1]*T16266[1][4] + S6662[2][1]*Si6266[2][2]*T16266[2][5];
T6266[2][5]=S6662[1][2]*Si6266[2][1]*T16266[1][4] + S6662[2][2]*Si6266[2][2]*T16266[2][5];
T6266[2][6]=Si6266[2][2]*T16266[2][6];

T6266[3][1]=S6662[1][1]*T16266[3][1] + S6662[2][1]*T16266[3][2] + HEAD*S6662[2][2]*T16266[3][5] - EYEYOFF*T16266[3][6];
T6266[3][2]=S6662[1][2]*T16266[3][1] + S6662[2][2]*T16266[3][2] - HEAD*S6662[2][1]*T16266[3][5] + EYEXOFF*T16266[3][6];
T6266[3][3]=T16266[3][3] + (EYEYOFF*S6662[2][1] - EYEXOFF*S6662[2][2])*T16266[3][5];
T6266[3][4]=S6662[2][1]*T16266[3][5];
T6266[3][5]=S6662[2][2]*T16266[3][5];
T6266[3][6]=T16266[3][6];

T6266[4][1]=S6662[1][1]*(HEAD*Si6266[2][2]*T16266[2][1] - EYEYOFF*T16266[3][1] + Si6266[1][1]*T16266[4][1] + Si6266[1][2]*T16266[5][1]) + S6662[2][1]*(HEAD*Si6266[2][1]*T16266[1][2] + HEAD*Si6266[2][2]*T16266[2][2] - EYEYOFF*T16266[3][2] + Si6266[1][1]*T16266[4][2] + Si6266[1][2]*T16266[5][2]) + HEAD*S6662[1][2]*(HEAD*Si6266[2][1]*T16266[1][4] + Si6266[1][2]*T16266[5][4]) + HEAD*S6662[2][2]*(HEAD*Si6266[2][2]*T16266[2][5] - EYEYOFF*T16266[3][5] + Si6266[1][1]*T16266[4][5] + Si6266[1][2]*T16266[5][5]) - EYEYOFF*(HEAD*Si6266[2][2]*T16266[2][6] - EYEYOFF*T16266[3][6] + Si6266[1][1]*T16266[4][6] + Si6266[1][2]*T16266[5][6]);
T6266[4][2]=S6662[1][2]*(HEAD*Si6266[2][2]*T16266[2][1] - EYEYOFF*T16266[3][1] + Si6266[1][1]*T16266[4][1] + Si6266[1][2]*T16266[5][1]) + S6662[2][2]*(HEAD*Si6266[2][1]*T16266[1][2] + HEAD*Si6266[2][2]*T16266[2][2] - EYEYOFF*T16266[3][2] + Si6266[1][1]*T16266[4][2] + Si6266[1][2]*T16266[5][2]) - HEAD*S6662[1][1]*(HEAD*Si6266[2][1]*T16266[1][4] + Si6266[1][2]*T16266[5][4]) - HEAD*S6662[2][1]*(HEAD*Si6266[2][2]*T16266[2][5] - EYEYOFF*T16266[3][5] + Si6266[1][1]*T16266[4][5] + Si6266[1][2]*T16266[5][5]) + EYEXOFF*(HEAD*Si6266[2][2]*T16266[2][6] - EYEYOFF*T16266[3][6] + Si6266[1][1]*T16266[4][6] + Si6266[1][2]*T16266[5][6]);
T6266[4][3]=HEAD*Si6266[2][1]*T16266[1][3] + HEAD*Si6266[2][2]*T16266[2][3] - EYEYOFF*T16266[3][3] + Si6266[1][1]*T16266[4][3] + Si6266[1][2]*T16266[5][3] + (EYEYOFF*S6662[1][1] - EYEXOFF*S6662[1][2])*(HEAD*Si6266[2][1]*T16266[1][4] + Si6266[1][2]*T16266[5][4]) + (EYEYOFF*S6662[2][1] - EYEXOFF*S6662[2][2])*(HEAD*Si6266[2][2]*T16266[2][5] - EYEYOFF*T16266[3][5] + Si6266[1][1]*T16266[4][5] + Si6266[1][2]*T16266[5][5]);
T6266[4][4]=S6662[1][1]*(HEAD*Si6266[2][1]*T16266[1][4] + Si6266[1][2]*T16266[5][4]) + S6662[2][1]*(HEAD*Si6266[2][2]*T16266[2][5] - EYEYOFF*T16266[3][5] + Si6266[1][1]*T16266[4][5] + Si6266[1][2]*T16266[5][5]);
T6266[4][5]=S6662[1][2]*(HEAD*Si6266[2][1]*T16266[1][4] + Si6266[1][2]*T16266[5][4]) + S6662[2][2]*(HEAD*Si6266[2][2]*T16266[2][5] - EYEYOFF*T16266[3][5] + Si6266[1][1]*T16266[4][5] + Si6266[1][2]*T16266[5][5]);
T6266[4][6]=HEAD*Si6266[2][2]*T16266[2][6] - EYEYOFF*T16266[3][6] + Si6266[1][1]*T16266[4][6] + Si6266[1][2]*T16266[5][6];

T6266[5][1]=S6662[1][1]*(-(HEAD*Si6266[1][2]*T16266[2][1]) + EYEXOFF*T16266[3][1] + Si6266[2][1]*T16266[4][1] + Si6266[2][2]*T16266[5][1]) + S6662[2][1]*(-(HEAD*Si6266[1][1]*T16266[1][2]) - HEAD*Si6266[1][2]*T16266[2][2] + EYEXOFF*T16266[3][2] + Si6266[2][1]*T16266[4][2] + Si6266[2][2]*T16266[5][2]) + HEAD*S6662[1][2]*(-(HEAD*Si6266[1][1]*T16266[1][4]) + Si6266[2][2]*T16266[5][4]) + HEAD*S6662[2][2]*(-(HEAD*Si6266[1][2]*T16266[2][5]) + EYEXOFF*T16266[3][5] + Si6266[2][1]*T16266[4][5] + Si6266[2][2]*T16266[5][5]) - EYEYOFF*(-(HEAD*Si6266[1][2]*T16266[2][6]) + EYEXOFF*T16266[3][6] + Si6266[2][1]*T16266[4][6] + Si6266[2][2]*T16266[5][6]);
T6266[5][2]=S6662[1][2]*(-(HEAD*Si6266[1][2]*T16266[2][1]) + EYEXOFF*T16266[3][1] + Si6266[2][1]*T16266[4][1] + Si6266[2][2]*T16266[5][1]) + S6662[2][2]*(-(HEAD*Si6266[1][1]*T16266[1][2]) - HEAD*Si6266[1][2]*T16266[2][2] + EYEXOFF*T16266[3][2] + Si6266[2][1]*T16266[4][2] + Si6266[2][2]*T16266[5][2]) - HEAD*S6662[1][1]*(-(HEAD*Si6266[1][1]*T16266[1][4]) + Si6266[2][2]*T16266[5][4]) - HEAD*S6662[2][1]*(-(HEAD*Si6266[1][2]*T16266[2][5]) + EYEXOFF*T16266[3][5] + Si6266[2][1]*T16266[4][5] + Si6266[2][2]*T16266[5][5]) + EYEXOFF*(-(HEAD*Si6266[1][2]*T16266[2][6]) + EYEXOFF*T16266[3][6] + Si6266[2][1]*T16266[4][6] + Si6266[2][2]*T16266[5][6]);
T6266[5][3]=-(HEAD*Si6266[1][1]*T16266[1][3]) - HEAD*Si6266[1][2]*T16266[2][3] + EYEXOFF*T16266[3][3] + Si6266[2][1]*T16266[4][3] + Si6266[2][2]*T16266[5][3] + (EYEYOFF*S6662[1][1] - EYEXOFF*S6662[1][2])*(-(HEAD*Si6266[1][1]*T16266[1][4]) + Si6266[2][2]*T16266[5][4]) + (EYEYOFF*S6662[2][1] - EYEXOFF*S6662[2][2])*(-(HEAD*Si6266[1][2]*T16266[2][5]) + EYEXOFF*T16266[3][5] + Si6266[2][1]*T16266[4][5] + Si6266[2][2]*T16266[5][5]);
T6266[5][4]=S6662[1][1]*(-(HEAD*Si6266[1][1]*T16266[1][4]) + Si6266[2][2]*T16266[5][4]) + S6662[2][1]*(-(HEAD*Si6266[1][2]*T16266[2][5]) + EYEXOFF*T16266[3][5] + Si6266[2][1]*T16266[4][5] + Si6266[2][2]*T16266[5][5]);
T6266[5][5]=S6662[1][2]*(-(HEAD*Si6266[1][1]*T16266[1][4]) + Si6266[2][2]*T16266[5][4]) + S6662[2][2]*(-(HEAD*Si6266[1][2]*T16266[2][5]) + EYEXOFF*T16266[3][5] + Si6266[2][1]*T16266[4][5] + Si6266[2][2]*T16266[5][5]);
T6266[5][6]=-(HEAD*Si6266[1][2]*T16266[2][6]) + EYEXOFF*T16266[3][6] + Si6266[2][1]*T16266[4][6] + Si6266[2][2]*T16266[5][6];

T6266[6][1]=S6662[1][1]*((EYEYOFF*Si6266[1][2] - EYEXOFF*Si6266[2][2])*T16266[2][1] + T16266[6][1]) + S6662[2][1]*((EYEYOFF*Si6266[1][1] - EYEXOFF*Si6266[2][1])*T16266[1][2] + (EYEYOFF*Si6266[1][2] - EYEXOFF*Si6266[2][2])*T16266[2][2] + T16266[6][2]) + HEAD*S6662[1][2]*((EYEYOFF*Si6266[1][1] - EYEXOFF*Si6266[2][1])*T16266[1][4] + T16266[6][4]) + HEAD*S6662[2][2]*((EYEYOFF*Si6266[1][2] - EYEXOFF*Si6266[2][2])*T16266[2][5] + T16266[6][5]) - EYEYOFF*((EYEYOFF*Si6266[1][2] - EYEXOFF*Si6266[2][2])*T16266[2][6] + T16266[6][6]);
T6266[6][2]=S6662[1][2]*((EYEYOFF*Si6266[1][2] - EYEXOFF*Si6266[2][2])*T16266[2][1] + T16266[6][1]) + S6662[2][2]*((EYEYOFF*Si6266[1][1] - EYEXOFF*Si6266[2][1])*T16266[1][2] + (EYEYOFF*Si6266[1][2] - EYEXOFF*Si6266[2][2])*T16266[2][2] + T16266[6][2]) - HEAD*S6662[1][1]*((EYEYOFF*Si6266[1][1] - EYEXOFF*Si6266[2][1])*T16266[1][4] + T16266[6][4]) - HEAD*S6662[2][1]*((EYEYOFF*Si6266[1][2] - EYEXOFF*Si6266[2][2])*T16266[2][5] + T16266[6][5]) + EYEXOFF*((EYEYOFF*Si6266[1][2] - EYEXOFF*Si6266[2][2])*T16266[2][6] + T16266[6][6]);
T6266[6][3]=(EYEYOFF*Si6266[1][1] - EYEXOFF*Si6266[2][1])*T16266[1][3] + (EYEYOFF*Si6266[1][2] - EYEXOFF*Si6266[2][2])*T16266[2][3] + T16266[6][3] + (EYEYOFF*S6662[1][1] - EYEXOFF*S6662[1][2])*((EYEYOFF*Si6266[1][1] - EYEXOFF*Si6266[2][1])*T16266[1][4] + T16266[6][4]) + (EYEYOFF*S6662[2][1] - EYEXOFF*S6662[2][2])*((EYEYOFF*Si6266[1][2] - EYEXOFF*Si6266[2][2])*T16266[2][5] + T16266[6][5]);
T6266[6][4]=S6662[1][1]*((EYEYOFF*Si6266[1][1] - EYEXOFF*Si6266[2][1])*T16266[1][4] + T16266[6][4]) + S6662[2][1]*((EYEYOFF*Si6266[1][2] - EYEXOFF*Si6266[2][2])*T16266[2][5] + T16266[6][5]);
T6266[6][5]=S6662[1][2]*((EYEYOFF*Si6266[1][1] - EYEXOFF*Si6266[2][1])*T16266[1][4] + T16266[6][4]) + S6662[2][2]*((EYEYOFF*Si6266[1][2] - EYEXOFF*Si6266[2][2])*T16266[2][5] + T16266[6][5]);
T6266[6][6]=(EYEYOFF*Si6266[1][2] - EYEXOFF*Si6266[2][2])*T16266[2][6] + T16266[6][6];



}


void
hermes_InvDynArtfunc39(void)
      {




}


void
hermes_InvDynArtfunc40(void)
      {
JA64[1][2]=0. + links[36].mcm[3];
JA64[1][3]=0. - links[36].mcm[2];
JA64[1][4]=0. + links[36].m;

JA64[2][1]=0. - links[36].mcm[3];
JA64[2][3]=0. + links[36].mcm[1];
JA64[2][5]=0. + links[36].m;

JA64[3][1]=0. + links[36].mcm[2];
JA64[3][2]=0. - links[36].mcm[1];
JA64[3][6]=0. + links[36].m;

JA64[4][1]=0. + links[36].inertia[1][1];
JA64[4][2]=0. + links[36].inertia[1][2];
JA64[4][3]=0. + links[36].inertia[1][3];
JA64[4][5]=0. - links[36].mcm[3];
JA64[4][6]=0. + links[36].mcm[2];

JA64[5][1]=0. + links[36].inertia[1][2];
JA64[5][2]=0. + links[36].inertia[2][2];
JA64[5][3]=0. + links[36].inertia[2][3];
JA64[5][4]=0. + links[36].mcm[3];
JA64[5][6]=0. - links[36].mcm[1];

JA64[6][1]=0. + links[36].inertia[1][3];
JA64[6][2]=0. + links[36].inertia[2][3];
JA64[6][3]=0. + links[36].inertia[3][3];
JA64[6][4]=0. - links[36].mcm[2];
JA64[6][5]=0. + links[36].mcm[1];


h64[1]=JA64[1][3];
h64[2]=JA64[2][3];
h64[4]=JA64[4][3];
h64[5]=JA64[5][3];
h64[6]=JA64[6][3];

T16364[1][2]=JA64[1][2];
T16364[1][3]=JA64[1][3];
T16364[1][4]=JA64[1][4];

T16364[2][1]=JA64[2][1];
T16364[2][3]=JA64[2][3];
T16364[2][5]=JA64[2][5];

T16364[3][1]=JA64[3][1];
T16364[3][2]=JA64[3][2];
T16364[3][6]=JA64[3][6];

T16364[4][1]=JA64[4][1];
T16364[4][2]=JA64[4][2];
T16364[4][3]=JA64[4][3];
T16364[4][5]=JA64[4][5];
T16364[4][6]=JA64[4][6];

T16364[5][1]=JA64[5][1];
T16364[5][2]=JA64[5][2];
T16364[5][3]=JA64[5][3];
T16364[5][4]=JA64[5][4];
T16364[5][6]=JA64[5][6];

T16364[6][1]=JA64[6][1];
T16364[6][2]=JA64[6][2];
T16364[6][3]=JA64[6][3];
T16364[6][4]=JA64[6][4];
T16364[6][5]=JA64[6][5];


T6364[1][2]=-(S6463[1][2]*T16364[3][1]) - S6463[2][2]*T16364[3][2];
T6364[1][3]=-(S6463[1][3]*T16364[3][1]) - S6463[2][3]*T16364[3][2];
T6364[1][4]=T16364[3][6];

T6364[2][1]=-(Si6364[2][1]*T16364[1][3]) - Si6364[2][2]*T16364[2][3];
T6364[2][2]=S6463[2][2]*Si6364[2][1]*T16364[1][2] + S6463[1][2]*Si6364[2][2]*T16364[2][1];
T6364[2][3]=S6463[2][3]*Si6364[2][1]*T16364[1][2] + S6463[1][3]*Si6364[2][2]*T16364[2][1];
T6364[2][5]=S6463[1][2]*Si6364[2][1]*T16364[1][4] + S6463[2][2]*Si6364[2][2]*T16364[2][5];
T6364[2][6]=S6463[1][3]*Si6364[2][1]*T16364[1][4] + S6463[2][3]*Si6364[2][2]*T16364[2][5];

T6364[3][1]=-(Si6364[3][1]*T16364[1][3]) - Si6364[3][2]*T16364[2][3];
T6364[3][2]=S6463[2][2]*Si6364[3][1]*T16364[1][2] + S6463[1][2]*Si6364[3][2]*T16364[2][1];
T6364[3][3]=S6463[2][3]*Si6364[3][1]*T16364[1][2] + S6463[1][3]*Si6364[3][2]*T16364[2][1];
T6364[3][5]=S6463[1][2]*Si6364[3][1]*T16364[1][4] + S6463[2][2]*Si6364[3][2]*T16364[2][5];
T6364[3][6]=S6463[1][3]*Si6364[3][1]*T16364[1][4] + S6463[2][3]*Si6364[3][2]*T16364[2][5];

T6364[4][1]=T16364[6][3];
T6364[4][2]=-(S6463[1][2]*T16364[6][1]) - S6463[2][2]*T16364[6][2];
T6364[4][3]=-(S6463[1][3]*T16364[6][1]) - S6463[2][3]*T16364[6][2];
T6364[4][5]=-(S6463[1][2]*T16364[6][4]) - S6463[2][2]*T16364[6][5];
T6364[4][6]=-(S6463[1][3]*T16364[6][4]) - S6463[2][3]*T16364[6][5];

T6364[5][1]=-(Si6364[2][1]*T16364[4][3]) - Si6364[2][2]*T16364[5][3];
T6364[5][2]=S6463[1][2]*(Si6364[2][1]*T16364[4][1] + Si6364[2][2]*T16364[5][1]) + S6463[2][2]*(Si6364[2][1]*T16364[4][2] + Si6364[2][2]*T16364[5][2]);
T6364[5][3]=S6463[1][3]*(Si6364[2][1]*T16364[4][1] + Si6364[2][2]*T16364[5][1]) + S6463[2][3]*(Si6364[2][1]*T16364[4][2] + Si6364[2][2]*T16364[5][2]);
T6364[5][4]=-(Si6364[2][1]*T16364[4][6]) - Si6364[2][2]*T16364[5][6];
T6364[5][5]=S6463[2][2]*Si6364[2][1]*T16364[4][5] + S6463[1][2]*Si6364[2][2]*T16364[5][4];
T6364[5][6]=S6463[2][3]*Si6364[2][1]*T16364[4][5] + S6463[1][3]*Si6364[2][2]*T16364[5][4];

T6364[6][1]=-(Si6364[3][1]*T16364[4][3]) - Si6364[3][2]*T16364[5][3];
T6364[6][2]=S6463[1][2]*(Si6364[3][1]*T16364[4][1] + Si6364[3][2]*T16364[5][1]) + S6463[2][2]*(Si6364[3][1]*T16364[4][2] + Si6364[3][2]*T16364[5][2]);
T6364[6][3]=S6463[1][3]*(Si6364[3][1]*T16364[4][1] + Si6364[3][2]*T16364[5][1]) + S6463[2][3]*(Si6364[3][1]*T16364[4][2] + Si6364[3][2]*T16364[5][2]);
T6364[6][4]=-(Si6364[3][1]*T16364[4][6]) - Si6364[3][2]*T16364[5][6];
T6364[6][5]=S6463[2][2]*Si6364[3][1]*T16364[4][5] + S6463[1][2]*Si6364[3][2]*T16364[5][4];
T6364[6][6]=S6463[2][3]*Si6364[3][1]*T16364[4][5] + S6463[1][3]*Si6364[3][2]*T16364[5][4];



}


void
hermes_InvDynArtfunc41(void)
      {
JA63[1][2]=links[35].mcm[3] + T6364[1][2];
JA63[1][3]=-links[35].mcm[2] + T6364[1][3];
JA63[1][4]=links[35].m + T6364[1][4];

JA63[2][1]=-links[35].mcm[3] + T6364[2][1];
JA63[2][2]=T6364[2][2];
JA63[2][3]=links[35].mcm[1] + T6364[2][3];
JA63[2][5]=links[35].m + T6364[2][5];
JA63[2][6]=T6364[2][6];

JA63[3][1]=links[35].mcm[2] + T6364[3][1];
JA63[3][2]=-links[35].mcm[1] + T6364[3][2];
JA63[3][3]=T6364[3][3];
JA63[3][5]=T6364[3][5];
JA63[3][6]=links[35].m + T6364[3][6];

JA63[4][1]=links[35].inertia[1][1] + T6364[4][1];
JA63[4][2]=links[35].inertia[1][2] + T6364[4][2];
JA63[4][3]=links[35].inertia[1][3] + T6364[4][3];
JA63[4][5]=-links[35].mcm[3] + T6364[4][5];
JA63[4][6]=links[35].mcm[2] + T6364[4][6];

JA63[5][1]=links[35].inertia[1][2] + T6364[5][1];
JA63[5][2]=links[35].inertia[2][2] + T6364[5][2];
JA63[5][3]=links[35].inertia[2][3] + T6364[5][3];
JA63[5][4]=links[35].mcm[3] + T6364[5][4];
JA63[5][5]=T6364[5][5];
JA63[5][6]=-links[35].mcm[1] + T6364[5][6];

JA63[6][1]=links[35].inertia[1][3] + T6364[6][1];
JA63[6][2]=links[35].inertia[2][3] + T6364[6][2];
JA63[6][3]=links[35].inertia[3][3] + T6364[6][3];
JA63[6][4]=-links[35].mcm[2] + T6364[6][4];
JA63[6][5]=links[35].mcm[1] + T6364[6][5];
JA63[6][6]=T6364[6][6];


h63[1]=JA63[1][3];
h63[2]=JA63[2][3];
h63[3]=JA63[3][3];
h63[4]=JA63[4][3];
h63[5]=JA63[5][3];
h63[6]=JA63[6][3];

T16263[1][2]=JA63[1][2];
T16263[1][3]=JA63[1][3];
T16263[1][4]=JA63[1][4];

T16263[2][1]=JA63[2][1];
T16263[2][2]=JA63[2][2];
T16263[2][3]=JA63[2][3];
T16263[2][5]=JA63[2][5];
T16263[2][6]=JA63[2][6];

T16263[3][1]=JA63[3][1];
T16263[3][2]=JA63[3][2];
T16263[3][3]=JA63[3][3];
T16263[3][5]=JA63[3][5];
T16263[3][6]=JA63[3][6];

T16263[4][1]=JA63[4][1];
T16263[4][2]=JA63[4][2];
T16263[4][3]=JA63[4][3];
T16263[4][5]=JA63[4][5];
T16263[4][6]=JA63[4][6];

T16263[5][1]=JA63[5][1];
T16263[5][2]=JA63[5][2];
T16263[5][3]=JA63[5][3];
T16263[5][4]=JA63[5][4];
T16263[5][5]=JA63[5][5];
T16263[5][6]=JA63[5][6];

T16263[6][1]=JA63[6][1];
T16263[6][2]=JA63[6][2];
T16263[6][3]=JA63[6][3];
T16263[6][4]=JA63[6][4];
T16263[6][5]=JA63[6][5];
T16263[6][6]=JA63[6][6];


T6263[1][1]=HEAD*S6362[1][2]*Si6263[1][1]*T16263[1][4] + S6362[1][1]*Si6263[1][2]*T16263[2][1] + S6362[2][1]*(Si6263[1][1]*T16263[1][2] + Si6263[1][2]*T16263[2][2]) + HEAD*S6362[2][2]*Si6263[1][2]*T16263[2][5] - EYEYOFF*Si6263[1][2]*T16263[2][6];
T6263[1][2]=-(HEAD*S6362[1][1]*Si6263[1][1]*T16263[1][4]) + S6362[1][2]*Si6263[1][2]*T16263[2][1] + S6362[2][2]*(Si6263[1][1]*T16263[1][2] + Si6263[1][2]*T16263[2][2]) - HEAD*S6362[2][1]*Si6263[1][2]*T16263[2][5] - EYEXOFF*Si6263[1][2]*T16263[2][6];
T6263[1][3]=Si6263[1][1]*T16263[1][3] + (EYEYOFF*S6362[1][1] + EYEXOFF*S6362[1][2])*Si6263[1][1]*T16263[1][4] + Si6263[1][2]*T16263[2][3] + (EYEYOFF*S6362[2][1] + EYEXOFF*S6362[2][2])*Si6263[1][2]*T16263[2][5];
T6263[1][4]=S6362[1][1]*Si6263[1][1]*T16263[1][4] + S6362[2][1]*Si6263[1][2]*T16263[2][5];
T6263[1][5]=S6362[1][2]*Si6263[1][1]*T16263[1][4] + S6362[2][2]*Si6263[1][2]*T16263[2][5];
T6263[1][6]=Si6263[1][2]*T16263[2][6];

T6263[2][1]=HEAD*S6362[1][2]*Si6263[2][1]*T16263[1][4] + S6362[1][1]*Si6263[2][2]*T16263[2][1] + S6362[2][1]*(Si6263[2][1]*T16263[1][2] + Si6263[2][2]*T16263[2][2]) + HEAD*S6362[2][2]*Si6263[2][2]*T16263[2][5] - EYEYOFF*Si6263[2][2]*T16263[2][6];
T6263[2][2]=-(HEAD*S6362[1][1]*Si6263[2][1]*T16263[1][4]) + S6362[1][2]*Si6263[2][2]*T16263[2][1] + S6362[2][2]*(Si6263[2][1]*T16263[1][2] + Si6263[2][2]*T16263[2][2]) - HEAD*S6362[2][1]*Si6263[2][2]*T16263[2][5] - EYEXOFF*Si6263[2][2]*T16263[2][6];
T6263[2][3]=Si6263[2][1]*T16263[1][3] + (EYEYOFF*S6362[1][1] + EYEXOFF*S6362[1][2])*Si6263[2][1]*T16263[1][4] + Si6263[2][2]*T16263[2][3] + (EYEYOFF*S6362[2][1] + EYEXOFF*S6362[2][2])*Si6263[2][2]*T16263[2][5];
T6263[2][4]=S6362[1][1]*Si6263[2][1]*T16263[1][4] + S6362[2][1]*Si6263[2][2]*T16263[2][5];
T6263[2][5]=S6362[1][2]*Si6263[2][1]*T16263[1][4] + S6362[2][2]*Si6263[2][2]*T16263[2][5];
T6263[2][6]=Si6263[2][2]*T16263[2][6];

T6263[3][1]=S6362[1][1]*T16263[3][1] + S6362[2][1]*T16263[3][2] + HEAD*S6362[2][2]*T16263[3][5] - EYEYOFF*T16263[3][6];
T6263[3][2]=S6362[1][2]*T16263[3][1] + S6362[2][2]*T16263[3][2] - HEAD*S6362[2][1]*T16263[3][5] - EYEXOFF*T16263[3][6];
T6263[3][3]=T16263[3][3] + (EYEYOFF*S6362[2][1] + EYEXOFF*S6362[2][2])*T16263[3][5];
T6263[3][4]=S6362[2][1]*T16263[3][5];
T6263[3][5]=S6362[2][2]*T16263[3][5];
T6263[3][6]=T16263[3][6];

T6263[4][1]=S6362[1][1]*(HEAD*Si6263[2][2]*T16263[2][1] - EYEYOFF*T16263[3][1] + Si6263[1][1]*T16263[4][1] + Si6263[1][2]*T16263[5][1]) + S6362[2][1]*(HEAD*Si6263[2][1]*T16263[1][2] + HEAD*Si6263[2][2]*T16263[2][2] - EYEYOFF*T16263[3][2] + Si6263[1][1]*T16263[4][2] + Si6263[1][2]*T16263[5][2]) + HEAD*S6362[1][2]*(HEAD*Si6263[2][1]*T16263[1][4] + Si6263[1][2]*T16263[5][4]) + HEAD*S6362[2][2]*(HEAD*Si6263[2][2]*T16263[2][5] - EYEYOFF*T16263[3][5] + Si6263[1][1]*T16263[4][5] + Si6263[1][2]*T16263[5][5]) - EYEYOFF*(HEAD*Si6263[2][2]*T16263[2][6] - EYEYOFF*T16263[3][6] + Si6263[1][1]*T16263[4][6] + Si6263[1][2]*T16263[5][6]);
T6263[4][2]=S6362[1][2]*(HEAD*Si6263[2][2]*T16263[2][1] - EYEYOFF*T16263[3][1] + Si6263[1][1]*T16263[4][1] + Si6263[1][2]*T16263[5][1]) + S6362[2][2]*(HEAD*Si6263[2][1]*T16263[1][2] + HEAD*Si6263[2][2]*T16263[2][2] - EYEYOFF*T16263[3][2] + Si6263[1][1]*T16263[4][2] + Si6263[1][2]*T16263[5][2]) - HEAD*S6362[1][1]*(HEAD*Si6263[2][1]*T16263[1][4] + Si6263[1][2]*T16263[5][4]) - HEAD*S6362[2][1]*(HEAD*Si6263[2][2]*T16263[2][5] - EYEYOFF*T16263[3][5] + Si6263[1][1]*T16263[4][5] + Si6263[1][2]*T16263[5][5]) - EYEXOFF*(HEAD*Si6263[2][2]*T16263[2][6] - EYEYOFF*T16263[3][6] + Si6263[1][1]*T16263[4][6] + Si6263[1][2]*T16263[5][6]);
T6263[4][3]=HEAD*Si6263[2][1]*T16263[1][3] + HEAD*Si6263[2][2]*T16263[2][3] - EYEYOFF*T16263[3][3] + Si6263[1][1]*T16263[4][3] + Si6263[1][2]*T16263[5][3] + (EYEYOFF*S6362[1][1] + EYEXOFF*S6362[1][2])*(HEAD*Si6263[2][1]*T16263[1][4] + Si6263[1][2]*T16263[5][4]) + (EYEYOFF*S6362[2][1] + EYEXOFF*S6362[2][2])*(HEAD*Si6263[2][2]*T16263[2][5] - EYEYOFF*T16263[3][5] + Si6263[1][1]*T16263[4][5] + Si6263[1][2]*T16263[5][5]);
T6263[4][4]=S6362[1][1]*(HEAD*Si6263[2][1]*T16263[1][4] + Si6263[1][2]*T16263[5][4]) + S6362[2][1]*(HEAD*Si6263[2][2]*T16263[2][5] - EYEYOFF*T16263[3][5] + Si6263[1][1]*T16263[4][5] + Si6263[1][2]*T16263[5][5]);
T6263[4][5]=S6362[1][2]*(HEAD*Si6263[2][1]*T16263[1][4] + Si6263[1][2]*T16263[5][4]) + S6362[2][2]*(HEAD*Si6263[2][2]*T16263[2][5] - EYEYOFF*T16263[3][5] + Si6263[1][1]*T16263[4][5] + Si6263[1][2]*T16263[5][5]);
T6263[4][6]=HEAD*Si6263[2][2]*T16263[2][6] - EYEYOFF*T16263[3][6] + Si6263[1][1]*T16263[4][6] + Si6263[1][2]*T16263[5][6];

T6263[5][1]=S6362[1][1]*(-(HEAD*Si6263[1][2]*T16263[2][1]) - EYEXOFF*T16263[3][1] + Si6263[2][1]*T16263[4][1] + Si6263[2][2]*T16263[5][1]) + S6362[2][1]*(-(HEAD*Si6263[1][1]*T16263[1][2]) - HEAD*Si6263[1][2]*T16263[2][2] - EYEXOFF*T16263[3][2] + Si6263[2][1]*T16263[4][2] + Si6263[2][2]*T16263[5][2]) + HEAD*S6362[1][2]*(-(HEAD*Si6263[1][1]*T16263[1][4]) + Si6263[2][2]*T16263[5][4]) + HEAD*S6362[2][2]*(-(HEAD*Si6263[1][2]*T16263[2][5]) - EYEXOFF*T16263[3][5] + Si6263[2][1]*T16263[4][5] + Si6263[2][2]*T16263[5][5]) - EYEYOFF*(-(HEAD*Si6263[1][2]*T16263[2][6]) - EYEXOFF*T16263[3][6] + Si6263[2][1]*T16263[4][6] + Si6263[2][2]*T16263[5][6]);
T6263[5][2]=S6362[1][2]*(-(HEAD*Si6263[1][2]*T16263[2][1]) - EYEXOFF*T16263[3][1] + Si6263[2][1]*T16263[4][1] + Si6263[2][2]*T16263[5][1]) + S6362[2][2]*(-(HEAD*Si6263[1][1]*T16263[1][2]) - HEAD*Si6263[1][2]*T16263[2][2] - EYEXOFF*T16263[3][2] + Si6263[2][1]*T16263[4][2] + Si6263[2][2]*T16263[5][2]) - HEAD*S6362[1][1]*(-(HEAD*Si6263[1][1]*T16263[1][4]) + Si6263[2][2]*T16263[5][4]) - HEAD*S6362[2][1]*(-(HEAD*Si6263[1][2]*T16263[2][5]) - EYEXOFF*T16263[3][5] + Si6263[2][1]*T16263[4][5] + Si6263[2][2]*T16263[5][5]) - EYEXOFF*(-(HEAD*Si6263[1][2]*T16263[2][6]) - EYEXOFF*T16263[3][6] + Si6263[2][1]*T16263[4][6] + Si6263[2][2]*T16263[5][6]);
T6263[5][3]=-(HEAD*Si6263[1][1]*T16263[1][3]) - HEAD*Si6263[1][2]*T16263[2][3] - EYEXOFF*T16263[3][3] + Si6263[2][1]*T16263[4][3] + Si6263[2][2]*T16263[5][3] + (EYEYOFF*S6362[1][1] + EYEXOFF*S6362[1][2])*(-(HEAD*Si6263[1][1]*T16263[1][4]) + Si6263[2][2]*T16263[5][4]) + (EYEYOFF*S6362[2][1] + EYEXOFF*S6362[2][2])*(-(HEAD*Si6263[1][2]*T16263[2][5]) - EYEXOFF*T16263[3][5] + Si6263[2][1]*T16263[4][5] + Si6263[2][2]*T16263[5][5]);
T6263[5][4]=S6362[1][1]*(-(HEAD*Si6263[1][1]*T16263[1][4]) + Si6263[2][2]*T16263[5][4]) + S6362[2][1]*(-(HEAD*Si6263[1][2]*T16263[2][5]) - EYEXOFF*T16263[3][5] + Si6263[2][1]*T16263[4][5] + Si6263[2][2]*T16263[5][5]);
T6263[5][5]=S6362[1][2]*(-(HEAD*Si6263[1][1]*T16263[1][4]) + Si6263[2][2]*T16263[5][4]) + S6362[2][2]*(-(HEAD*Si6263[1][2]*T16263[2][5]) - EYEXOFF*T16263[3][5] + Si6263[2][1]*T16263[4][5] + Si6263[2][2]*T16263[5][5]);
T6263[5][6]=-(HEAD*Si6263[1][2]*T16263[2][6]) - EYEXOFF*T16263[3][6] + Si6263[2][1]*T16263[4][6] + Si6263[2][2]*T16263[5][6];

T6263[6][1]=S6362[1][1]*((EYEYOFF*Si6263[1][2] + EYEXOFF*Si6263[2][2])*T16263[2][1] + T16263[6][1]) + S6362[2][1]*((EYEYOFF*Si6263[1][1] + EYEXOFF*Si6263[2][1])*T16263[1][2] + (EYEYOFF*Si6263[1][2] + EYEXOFF*Si6263[2][2])*T16263[2][2] + T16263[6][2]) + HEAD*S6362[1][2]*((EYEYOFF*Si6263[1][1] + EYEXOFF*Si6263[2][1])*T16263[1][4] + T16263[6][4]) + HEAD*S6362[2][2]*((EYEYOFF*Si6263[1][2] + EYEXOFF*Si6263[2][2])*T16263[2][5] + T16263[6][5]) - EYEYOFF*((EYEYOFF*Si6263[1][2] + EYEXOFF*Si6263[2][2])*T16263[2][6] + T16263[6][6]);
T6263[6][2]=S6362[1][2]*((EYEYOFF*Si6263[1][2] + EYEXOFF*Si6263[2][2])*T16263[2][1] + T16263[6][1]) + S6362[2][2]*((EYEYOFF*Si6263[1][1] + EYEXOFF*Si6263[2][1])*T16263[1][2] + (EYEYOFF*Si6263[1][2] + EYEXOFF*Si6263[2][2])*T16263[2][2] + T16263[6][2]) - HEAD*S6362[1][1]*((EYEYOFF*Si6263[1][1] + EYEXOFF*Si6263[2][1])*T16263[1][4] + T16263[6][4]) - HEAD*S6362[2][1]*((EYEYOFF*Si6263[1][2] + EYEXOFF*Si6263[2][2])*T16263[2][5] + T16263[6][5]) - EYEXOFF*((EYEYOFF*Si6263[1][2] + EYEXOFF*Si6263[2][2])*T16263[2][6] + T16263[6][6]);
T6263[6][3]=(EYEYOFF*Si6263[1][1] + EYEXOFF*Si6263[2][1])*T16263[1][3] + (EYEYOFF*Si6263[1][2] + EYEXOFF*Si6263[2][2])*T16263[2][3] + T16263[6][3] + (EYEYOFF*S6362[1][1] + EYEXOFF*S6362[1][2])*((EYEYOFF*Si6263[1][1] + EYEXOFF*Si6263[2][1])*T16263[1][4] + T16263[6][4]) + (EYEYOFF*S6362[2][1] + EYEXOFF*S6362[2][2])*((EYEYOFF*Si6263[1][2] + EYEXOFF*Si6263[2][2])*T16263[2][5] + T16263[6][5]);
T6263[6][4]=S6362[1][1]*((EYEYOFF*Si6263[1][1] + EYEXOFF*Si6263[2][1])*T16263[1][4] + T16263[6][4]) + S6362[2][1]*((EYEYOFF*Si6263[1][2] + EYEXOFF*Si6263[2][2])*T16263[2][5] + T16263[6][5]);
T6263[6][5]=S6362[1][2]*((EYEYOFF*Si6263[1][1] + EYEXOFF*Si6263[2][1])*T16263[1][4] + T16263[6][4]) + S6362[2][2]*((EYEYOFF*Si6263[1][2] + EYEXOFF*Si6263[2][2])*T16263[2][5] + T16263[6][5]);
T6263[6][6]=(EYEYOFF*Si6263[1][2] + EYEXOFF*Si6263[2][2])*T16263[2][6] + T16263[6][6];



}


void
hermes_InvDynArtfunc42(void)
      {
JA62[1][1]=0. + T6263[1][1] + T6266[1][1];
JA62[1][2]=0. + links[34].mcm[3] + T6263[1][2] + T6266[1][2];
JA62[1][3]=0. - links[34].mcm[2] + T6263[1][3] + T6266[1][3];
JA62[1][4]=0. + links[34].m + T6263[1][4] + T6266[1][4];
JA62[1][5]=0. + T6263[1][5] + T6266[1][5];
JA62[1][6]=0. + T6263[1][6] + T6266[1][6];

JA62[2][1]=0. - links[34].mcm[3] + T6263[2][1] + T6266[2][1];
JA62[2][2]=0. + T6263[2][2] + T6266[2][2];
JA62[2][3]=0. + links[34].mcm[1] + T6263[2][3] + T6266[2][3];
JA62[2][4]=0. + T6263[2][4] + T6266[2][4];
JA62[2][5]=0. + links[34].m + T6263[2][5] + T6266[2][5];
JA62[2][6]=0. + T6263[2][6] + T6266[2][6];

JA62[3][1]=0. + links[34].mcm[2] + T6263[3][1] + T6266[3][1];
JA62[3][2]=0. - links[34].mcm[1] + T6263[3][2] + T6266[3][2];
JA62[3][3]=0. + T6263[3][3] + T6266[3][3];
JA62[3][4]=0. + T6263[3][4] + T6266[3][4];
JA62[3][5]=0. + T6263[3][5] + T6266[3][5];
JA62[3][6]=0. + links[34].m + T6263[3][6] + T6266[3][6];

JA62[4][1]=0. + links[34].inertia[1][1] + T6263[4][1] + T6266[4][1];
JA62[4][2]=0. + links[34].inertia[1][2] + T6263[4][2] + T6266[4][2];
JA62[4][3]=0. + links[34].inertia[1][3] + T6263[4][3] + T6266[4][3];
JA62[4][4]=0. + T6263[4][4] + T6266[4][4];
JA62[4][5]=0. - links[34].mcm[3] + T6263[4][5] + T6266[4][5];
JA62[4][6]=0. + links[34].mcm[2] + T6263[4][6] + T6266[4][6];

JA62[5][1]=0. + links[34].inertia[1][2] + T6263[5][1] + T6266[5][1];
JA62[5][2]=0. + links[34].inertia[2][2] + T6263[5][2] + T6266[5][2];
JA62[5][3]=0. + links[34].inertia[2][3] + T6263[5][3] + T6266[5][3];
JA62[5][4]=0. + links[34].mcm[3] + T6263[5][4] + T6266[5][4];
JA62[5][5]=0. + T6263[5][5] + T6266[5][5];
JA62[5][6]=0. - links[34].mcm[1] + T6263[5][6] + T6266[5][6];

JA62[6][1]=0. + links[34].inertia[1][3] + T6263[6][1] + T6266[6][1];
JA62[6][2]=0. + links[34].inertia[2][3] + T6263[6][2] + T6266[6][2];
JA62[6][3]=0. + links[34].inertia[3][3] + T6263[6][3] + T6266[6][3];
JA62[6][4]=0. - links[34].mcm[2] + T6263[6][4] + T6266[6][4];
JA62[6][5]=0. + links[34].mcm[1] + T6263[6][5] + T6266[6][5];
JA62[6][6]=0. + T6263[6][6] + T6266[6][6];


h62[1]=JA62[1][3];
h62[2]=JA62[2][3];
h62[3]=JA62[3][3];
h62[4]=JA62[4][3];
h62[5]=JA62[5][3];
h62[6]=JA62[6][3];

T16162[1][1]=JA62[1][1];
T16162[1][2]=JA62[1][2];
T16162[1][3]=JA62[1][3];
T16162[1][4]=JA62[1][4];
T16162[1][5]=JA62[1][5];
T16162[1][6]=JA62[1][6];

T16162[2][1]=JA62[2][1];
T16162[2][2]=JA62[2][2];
T16162[2][3]=JA62[2][3];
T16162[2][4]=JA62[2][4];
T16162[2][5]=JA62[2][5];
T16162[2][6]=JA62[2][6];

T16162[3][1]=JA62[3][1];
T16162[3][2]=JA62[3][2];
T16162[3][3]=JA62[3][3];
T16162[3][4]=JA62[3][4];
T16162[3][5]=JA62[3][5];
T16162[3][6]=JA62[3][6];

T16162[4][1]=JA62[4][1];
T16162[4][2]=JA62[4][2];
T16162[4][3]=JA62[4][3];
T16162[4][4]=JA62[4][4];
T16162[4][5]=JA62[4][5];
T16162[4][6]=JA62[4][6];

T16162[5][1]=JA62[5][1];
T16162[5][2]=JA62[5][2];
T16162[5][3]=JA62[5][3];
T16162[5][4]=JA62[5][4];
T16162[5][5]=JA62[5][5];
T16162[5][6]=JA62[5][6];

T16162[6][1]=JA62[6][1];
T16162[6][2]=JA62[6][2];
T16162[6][3]=JA62[6][3];
T16162[6][4]=JA62[6][4];
T16162[6][5]=JA62[6][5];
T16162[6][6]=JA62[6][6];


T6162[1][1]=S6261[1][1]*(Si6162[1][1]*T16162[1][1] + Si6162[1][2]*T16162[2][1]) + S6261[2][1]*(Si6162[1][1]*T16162[1][2] + Si6162[1][2]*T16162[2][2]);
T6162[1][2]=Si6162[1][1]*T16162[1][3] + Si6162[1][2]*T16162[2][3];
T6162[1][3]=S6261[1][3]*(Si6162[1][1]*T16162[1][1] + Si6162[1][2]*T16162[2][1]) + S6261[2][3]*(Si6162[1][1]*T16162[1][2] + Si6162[1][2]*T16162[2][2]);
T6162[1][4]=S6261[1][1]*(Si6162[1][1]*T16162[1][4] + Si6162[1][2]*T16162[2][4]) + S6261[2][1]*(Si6162[1][1]*T16162[1][5] + Si6162[1][2]*T16162[2][5]);
T6162[1][5]=Si6162[1][1]*T16162[1][6] + Si6162[1][2]*T16162[2][6];
T6162[1][6]=S6261[1][3]*(Si6162[1][1]*T16162[1][4] + Si6162[1][2]*T16162[2][4]) + S6261[2][3]*(Si6162[1][1]*T16162[1][5] + Si6162[1][2]*T16162[2][5]);

T6162[2][1]=S6261[1][1]*T16162[3][1] + S6261[2][1]*T16162[3][2];
T6162[2][2]=T16162[3][3];
T6162[2][3]=S6261[1][3]*T16162[3][1] + S6261[2][3]*T16162[3][2];
T6162[2][4]=S6261[1][1]*T16162[3][4] + S6261[2][1]*T16162[3][5];
T6162[2][5]=T16162[3][6];
T6162[2][6]=S6261[1][3]*T16162[3][4] + S6261[2][3]*T16162[3][5];

T6162[3][1]=S6261[1][1]*(Si6162[3][1]*T16162[1][1] + Si6162[3][2]*T16162[2][1]) + S6261[2][1]*(Si6162[3][1]*T16162[1][2] + Si6162[3][2]*T16162[2][2]);
T6162[3][2]=Si6162[3][1]*T16162[1][3] + Si6162[3][2]*T16162[2][3];
T6162[3][3]=S6261[1][3]*(Si6162[3][1]*T16162[1][1] + Si6162[3][2]*T16162[2][1]) + S6261[2][3]*(Si6162[3][1]*T16162[1][2] + Si6162[3][2]*T16162[2][2]);
T6162[3][4]=S6261[1][1]*(Si6162[3][1]*T16162[1][4] + Si6162[3][2]*T16162[2][4]) + S6261[2][1]*(Si6162[3][1]*T16162[1][5] + Si6162[3][2]*T16162[2][5]);
T6162[3][5]=Si6162[3][1]*T16162[1][6] + Si6162[3][2]*T16162[2][6];
T6162[3][6]=S6261[1][3]*(Si6162[3][1]*T16162[1][4] + Si6162[3][2]*T16162[2][4]) + S6261[2][3]*(Si6162[3][1]*T16162[1][5] + Si6162[3][2]*T16162[2][5]);

T6162[4][1]=S6261[1][1]*(Si6162[1][1]*T16162[4][1] + Si6162[1][2]*T16162[5][1]) + S6261[2][1]*(Si6162[1][1]*T16162[4][2] + Si6162[1][2]*T16162[5][2]);
T6162[4][2]=Si6162[1][1]*T16162[4][3] + Si6162[1][2]*T16162[5][3];
T6162[4][3]=S6261[1][3]*(Si6162[1][1]*T16162[4][1] + Si6162[1][2]*T16162[5][1]) + S6261[2][3]*(Si6162[1][1]*T16162[4][2] + Si6162[1][2]*T16162[5][2]);
T6162[4][4]=S6261[1][1]*(Si6162[1][1]*T16162[4][4] + Si6162[1][2]*T16162[5][4]) + S6261[2][1]*(Si6162[1][1]*T16162[4][5] + Si6162[1][2]*T16162[5][5]);
T6162[4][5]=Si6162[1][1]*T16162[4][6] + Si6162[1][2]*T16162[5][6];
T6162[4][6]=S6261[1][3]*(Si6162[1][1]*T16162[4][4] + Si6162[1][2]*T16162[5][4]) + S6261[2][3]*(Si6162[1][1]*T16162[4][5] + Si6162[1][2]*T16162[5][5]);

T6162[5][1]=S6261[1][1]*T16162[6][1] + S6261[2][1]*T16162[6][2];
T6162[5][2]=T16162[6][3];
T6162[5][3]=S6261[1][3]*T16162[6][1] + S6261[2][3]*T16162[6][2];
T6162[5][4]=S6261[1][1]*T16162[6][4] + S6261[2][1]*T16162[6][5];
T6162[5][5]=T16162[6][6];
T6162[5][6]=S6261[1][3]*T16162[6][4] + S6261[2][3]*T16162[6][5];

T6162[6][1]=S6261[1][1]*(Si6162[3][1]*T16162[4][1] + Si6162[3][2]*T16162[5][1]) + S6261[2][1]*(Si6162[3][1]*T16162[4][2] + Si6162[3][2]*T16162[5][2]);
T6162[6][2]=Si6162[3][1]*T16162[4][3] + Si6162[3][2]*T16162[5][3];
T6162[6][3]=S6261[1][3]*(Si6162[3][1]*T16162[4][1] + Si6162[3][2]*T16162[5][1]) + S6261[2][3]*(Si6162[3][1]*T16162[4][2] + Si6162[3][2]*T16162[5][2]);
T6162[6][4]=S6261[1][1]*(Si6162[3][1]*T16162[4][4] + Si6162[3][2]*T16162[5][4]) + S6261[2][1]*(Si6162[3][1]*T16162[4][5] + Si6162[3][2]*T16162[5][5]);
T6162[6][5]=Si6162[3][1]*T16162[4][6] + Si6162[3][2]*T16162[5][6];
T6162[6][6]=S6261[1][3]*(Si6162[3][1]*T16162[4][4] + Si6162[3][2]*T16162[5][4]) + S6261[2][3]*(Si6162[3][1]*T16162[4][5] + Si6162[3][2]*T16162[5][5]);



}


void
hermes_InvDynArtfunc43(void)
      {
JA61[1][1]=T6162[1][1];
JA61[1][2]=links[33].mcm[3] + T6162[1][2];
JA61[1][3]=-links[33].mcm[2] + T6162[1][3];
JA61[1][4]=links[33].m + T6162[1][4];
JA61[1][5]=T6162[1][5];
JA61[1][6]=T6162[1][6];

JA61[2][1]=-links[33].mcm[3] + T6162[2][1];
JA61[2][2]=T6162[2][2];
JA61[2][3]=links[33].mcm[1] + T6162[2][3];
JA61[2][4]=T6162[2][4];
JA61[2][5]=links[33].m + T6162[2][5];
JA61[2][6]=T6162[2][6];

JA61[3][1]=links[33].mcm[2] + T6162[3][1];
JA61[3][2]=-links[33].mcm[1] + T6162[3][2];
JA61[3][3]=T6162[3][3];
JA61[3][4]=T6162[3][4];
JA61[3][5]=T6162[3][5];
JA61[3][6]=links[33].m + T6162[3][6];

JA61[4][1]=links[33].inertia[1][1] + T6162[4][1];
JA61[4][2]=links[33].inertia[1][2] + T6162[4][2];
JA61[4][3]=links[33].inertia[1][3] + T6162[4][3];
JA61[4][4]=T6162[4][4];
JA61[4][5]=-links[33].mcm[3] + T6162[4][5];
JA61[4][6]=links[33].mcm[2] + T6162[4][6];

JA61[5][1]=links[33].inertia[1][2] + T6162[5][1];
JA61[5][2]=links[33].inertia[2][2] + T6162[5][2];
JA61[5][3]=links[33].inertia[2][3] + T6162[5][3];
JA61[5][4]=links[33].mcm[3] + T6162[5][4];
JA61[5][5]=T6162[5][5];
JA61[5][6]=-links[33].mcm[1] + T6162[5][6];

JA61[6][1]=links[33].inertia[1][3] + T6162[6][1];
JA61[6][2]=links[33].inertia[2][3] + T6162[6][2];
JA61[6][3]=links[33].inertia[3][3] + T6162[6][3];
JA61[6][4]=-links[33].mcm[2] + T6162[6][4];
JA61[6][5]=links[33].mcm[1] + T6162[6][5];
JA61[6][6]=T6162[6][6];


h61[1]=JA61[1][3];
h61[2]=JA61[2][3];
h61[3]=JA61[3][3];
h61[4]=JA61[4][3];
h61[5]=JA61[5][3];
h61[6]=JA61[6][3];

T16061[1][1]=JA61[1][1];
T16061[1][2]=JA61[1][2];
T16061[1][3]=JA61[1][3];
T16061[1][4]=JA61[1][4];
T16061[1][5]=JA61[1][5];
T16061[1][6]=JA61[1][6];

T16061[2][1]=JA61[2][1];
T16061[2][2]=JA61[2][2];
T16061[2][3]=JA61[2][3];
T16061[2][4]=JA61[2][4];
T16061[2][5]=JA61[2][5];
T16061[2][6]=JA61[2][6];

T16061[3][1]=JA61[3][1];
T16061[3][2]=JA61[3][2];
T16061[3][3]=JA61[3][3];
T16061[3][4]=JA61[3][4];
T16061[3][5]=JA61[3][5];
T16061[3][6]=JA61[3][6];

T16061[4][1]=JA61[4][1];
T16061[4][2]=JA61[4][2];
T16061[4][3]=JA61[4][3];
T16061[4][4]=JA61[4][4];
T16061[4][5]=JA61[4][5];
T16061[4][6]=JA61[4][6];

T16061[5][1]=JA61[5][1];
T16061[5][2]=JA61[5][2];
T16061[5][3]=JA61[5][3];
T16061[5][4]=JA61[5][4];
T16061[5][5]=JA61[5][5];
T16061[5][6]=JA61[5][6];

T16061[6][1]=JA61[6][1];
T16061[6][2]=JA61[6][2];
T16061[6][3]=JA61[6][3];
T16061[6][4]=JA61[6][4];
T16061[6][5]=JA61[6][5];
T16061[6][6]=JA61[6][6];


T6061[1][1]=T16061[3][3] - CERVICAL*S6160[1][3]*T16061[3][4] - CERVICAL*S6160[2][3]*T16061[3][5];
T6061[1][2]=S6160[1][2]*T16061[3][1] + S6160[2][2]*T16061[3][2];
T6061[1][3]=S6160[1][3]*T16061[3][1] + S6160[2][3]*T16061[3][2] + CERVICAL*T16061[3][6];
T6061[1][4]=T16061[3][6];
T6061[1][5]=S6160[1][2]*T16061[3][4] + S6160[2][2]*T16061[3][5];
T6061[1][6]=S6160[1][3]*T16061[3][4] + S6160[2][3]*T16061[3][5];

T6061[2][1]=Si6061[2][1]*T16061[1][3] + Si6061[2][2]*T16061[2][3] - CERVICAL*S6160[1][3]*(Si6061[2][1]*T16061[1][4] + Si6061[2][2]*T16061[2][4]) - CERVICAL*S6160[2][3]*(Si6061[2][1]*T16061[1][5] + Si6061[2][2]*T16061[2][5]);
T6061[2][2]=S6160[1][2]*(Si6061[2][1]*T16061[1][1] + Si6061[2][2]*T16061[2][1]) + S6160[2][2]*(Si6061[2][1]*T16061[1][2] + Si6061[2][2]*T16061[2][2]);
T6061[2][3]=S6160[1][3]*(Si6061[2][1]*T16061[1][1] + Si6061[2][2]*T16061[2][1]) + S6160[2][3]*(Si6061[2][1]*T16061[1][2] + Si6061[2][2]*T16061[2][2]) + CERVICAL*(Si6061[2][1]*T16061[1][6] + Si6061[2][2]*T16061[2][6]);
T6061[2][4]=Si6061[2][1]*T16061[1][6] + Si6061[2][2]*T16061[2][6];
T6061[2][5]=S6160[1][2]*(Si6061[2][1]*T16061[1][4] + Si6061[2][2]*T16061[2][4]) + S6160[2][2]*(Si6061[2][1]*T16061[1][5] + Si6061[2][2]*T16061[2][5]);
T6061[2][6]=S6160[1][3]*(Si6061[2][1]*T16061[1][4] + Si6061[2][2]*T16061[2][4]) + S6160[2][3]*(Si6061[2][1]*T16061[1][5] + Si6061[2][2]*T16061[2][5]);

T6061[3][1]=Si6061[3][1]*T16061[1][3] + Si6061[3][2]*T16061[2][3] - CERVICAL*S6160[1][3]*(Si6061[3][1]*T16061[1][4] + Si6061[3][2]*T16061[2][4]) - CERVICAL*S6160[2][3]*(Si6061[3][1]*T16061[1][5] + Si6061[3][2]*T16061[2][5]);
T6061[3][2]=S6160[1][2]*(Si6061[3][1]*T16061[1][1] + Si6061[3][2]*T16061[2][1]) + S6160[2][2]*(Si6061[3][1]*T16061[1][2] + Si6061[3][2]*T16061[2][2]);
T6061[3][3]=S6160[1][3]*(Si6061[3][1]*T16061[1][1] + Si6061[3][2]*T16061[2][1]) + S6160[2][3]*(Si6061[3][1]*T16061[1][2] + Si6061[3][2]*T16061[2][2]) + CERVICAL*(Si6061[3][1]*T16061[1][6] + Si6061[3][2]*T16061[2][6]);
T6061[3][4]=Si6061[3][1]*T16061[1][6] + Si6061[3][2]*T16061[2][6];
T6061[3][5]=S6160[1][2]*(Si6061[3][1]*T16061[1][4] + Si6061[3][2]*T16061[2][4]) + S6160[2][2]*(Si6061[3][1]*T16061[1][5] + Si6061[3][2]*T16061[2][5]);
T6061[3][6]=S6160[1][3]*(Si6061[3][1]*T16061[1][4] + Si6061[3][2]*T16061[2][4]) + S6160[2][3]*(Si6061[3][1]*T16061[1][5] + Si6061[3][2]*T16061[2][5]);

T6061[4][1]=-(CERVICAL*Si6061[3][1]*T16061[1][3]) - CERVICAL*Si6061[3][2]*T16061[2][3] + T16061[6][3] - CERVICAL*S6160[1][3]*(-(CERVICAL*Si6061[3][1]*T16061[1][4]) - CERVICAL*Si6061[3][2]*T16061[2][4] + T16061[6][4]) - CERVICAL*S6160[2][3]*(-(CERVICAL*Si6061[3][1]*T16061[1][5]) - CERVICAL*Si6061[3][2]*T16061[2][5] + T16061[6][5]);
T6061[4][2]=S6160[1][2]*(-(CERVICAL*Si6061[3][1]*T16061[1][1]) - CERVICAL*Si6061[3][2]*T16061[2][1] + T16061[6][1]) + S6160[2][2]*(-(CERVICAL*Si6061[3][1]*T16061[1][2]) - CERVICAL*Si6061[3][2]*T16061[2][2] + T16061[6][2]);
T6061[4][3]=S6160[1][3]*(-(CERVICAL*Si6061[3][1]*T16061[1][1]) - CERVICAL*Si6061[3][2]*T16061[2][1] + T16061[6][1]) + S6160[2][3]*(-(CERVICAL*Si6061[3][1]*T16061[1][2]) - CERVICAL*Si6061[3][2]*T16061[2][2] + T16061[6][2]) + CERVICAL*(-(CERVICAL*Si6061[3][1]*T16061[1][6]) - CERVICAL*Si6061[3][2]*T16061[2][6] + T16061[6][6]);
T6061[4][4]=-(CERVICAL*Si6061[3][1]*T16061[1][6]) - CERVICAL*Si6061[3][2]*T16061[2][6] + T16061[6][6];
T6061[4][5]=S6160[1][2]*(-(CERVICAL*Si6061[3][1]*T16061[1][4]) - CERVICAL*Si6061[3][2]*T16061[2][4] + T16061[6][4]) + S6160[2][2]*(-(CERVICAL*Si6061[3][1]*T16061[1][5]) - CERVICAL*Si6061[3][2]*T16061[2][5] + T16061[6][5]);
T6061[4][6]=S6160[1][3]*(-(CERVICAL*Si6061[3][1]*T16061[1][4]) - CERVICAL*Si6061[3][2]*T16061[2][4] + T16061[6][4]) + S6160[2][3]*(-(CERVICAL*Si6061[3][1]*T16061[1][5]) - CERVICAL*Si6061[3][2]*T16061[2][5] + T16061[6][5]);

T6061[5][1]=Si6061[2][1]*T16061[4][3] + Si6061[2][2]*T16061[5][3] - CERVICAL*S6160[1][3]*(Si6061[2][1]*T16061[4][4] + Si6061[2][2]*T16061[5][4]) - CERVICAL*S6160[2][3]*(Si6061[2][1]*T16061[4][5] + Si6061[2][2]*T16061[5][5]);
T6061[5][2]=S6160[1][2]*(Si6061[2][1]*T16061[4][1] + Si6061[2][2]*T16061[5][1]) + S6160[2][2]*(Si6061[2][1]*T16061[4][2] + Si6061[2][2]*T16061[5][2]);
T6061[5][3]=S6160[1][3]*(Si6061[2][1]*T16061[4][1] + Si6061[2][2]*T16061[5][1]) + S6160[2][3]*(Si6061[2][1]*T16061[4][2] + Si6061[2][2]*T16061[5][2]) + CERVICAL*(Si6061[2][1]*T16061[4][6] + Si6061[2][2]*T16061[5][6]);
T6061[5][4]=Si6061[2][1]*T16061[4][6] + Si6061[2][2]*T16061[5][6];
T6061[5][5]=S6160[1][2]*(Si6061[2][1]*T16061[4][4] + Si6061[2][2]*T16061[5][4]) + S6160[2][2]*(Si6061[2][1]*T16061[4][5] + Si6061[2][2]*T16061[5][5]);
T6061[5][6]=S6160[1][3]*(Si6061[2][1]*T16061[4][4] + Si6061[2][2]*T16061[5][4]) + S6160[2][3]*(Si6061[2][1]*T16061[4][5] + Si6061[2][2]*T16061[5][5]);

T6061[6][1]=CERVICAL*T16061[3][3] + Si6061[3][1]*T16061[4][3] + Si6061[3][2]*T16061[5][3] - CERVICAL*S6160[1][3]*(CERVICAL*T16061[3][4] + Si6061[3][1]*T16061[4][4] + Si6061[3][2]*T16061[5][4]) - CERVICAL*S6160[2][3]*(CERVICAL*T16061[3][5] + Si6061[3][1]*T16061[4][5] + Si6061[3][2]*T16061[5][5]);
T6061[6][2]=S6160[1][2]*(CERVICAL*T16061[3][1] + Si6061[3][1]*T16061[4][1] + Si6061[3][2]*T16061[5][1]) + S6160[2][2]*(CERVICAL*T16061[3][2] + Si6061[3][1]*T16061[4][2] + Si6061[3][2]*T16061[5][2]);
T6061[6][3]=S6160[1][3]*(CERVICAL*T16061[3][1] + Si6061[3][1]*T16061[4][1] + Si6061[3][2]*T16061[5][1]) + S6160[2][3]*(CERVICAL*T16061[3][2] + Si6061[3][1]*T16061[4][2] + Si6061[3][2]*T16061[5][2]) + CERVICAL*(CERVICAL*T16061[3][6] + Si6061[3][1]*T16061[4][6] + Si6061[3][2]*T16061[5][6]);
T6061[6][4]=CERVICAL*T16061[3][6] + Si6061[3][1]*T16061[4][6] + Si6061[3][2]*T16061[5][6];
T6061[6][5]=S6160[1][2]*(CERVICAL*T16061[3][4] + Si6061[3][1]*T16061[4][4] + Si6061[3][2]*T16061[5][4]) + S6160[2][2]*(CERVICAL*T16061[3][5] + Si6061[3][1]*T16061[4][5] + Si6061[3][2]*T16061[5][5]);
T6061[6][6]=S6160[1][3]*(CERVICAL*T16061[3][4] + Si6061[3][1]*T16061[4][4] + Si6061[3][2]*T16061[5][4]) + S6160[2][3]*(CERVICAL*T16061[3][5] + Si6061[3][1]*T16061[4][5] + Si6061[3][2]*T16061[5][5]);



}


void
hermes_InvDynArtfunc44(void)
      {
JA60[1][1]=T6061[1][1];
JA60[1][2]=links[32].mcm[3] + T6061[1][2];
JA60[1][3]=-links[32].mcm[2] + T6061[1][3];
JA60[1][4]=links[32].m + T6061[1][4];
JA60[1][5]=T6061[1][5];
JA60[1][6]=T6061[1][6];

JA60[2][1]=-links[32].mcm[3] + T6061[2][1];
JA60[2][2]=T6061[2][2];
JA60[2][3]=links[32].mcm[1] + T6061[2][3];
JA60[2][4]=T6061[2][4];
JA60[2][5]=links[32].m + T6061[2][5];
JA60[2][6]=T6061[2][6];

JA60[3][1]=links[32].mcm[2] + T6061[3][1];
JA60[3][2]=-links[32].mcm[1] + T6061[3][2];
JA60[3][3]=T6061[3][3];
JA60[3][4]=T6061[3][4];
JA60[3][5]=T6061[3][5];
JA60[3][6]=links[32].m + T6061[3][6];

JA60[4][1]=links[32].inertia[1][1] + T6061[4][1];
JA60[4][2]=links[32].inertia[1][2] + T6061[4][2];
JA60[4][3]=links[32].inertia[1][3] + T6061[4][3];
JA60[4][4]=T6061[4][4];
JA60[4][5]=-links[32].mcm[3] + T6061[4][5];
JA60[4][6]=links[32].mcm[2] + T6061[4][6];

JA60[5][1]=links[32].inertia[1][2] + T6061[5][1];
JA60[5][2]=links[32].inertia[2][2] + T6061[5][2];
JA60[5][3]=links[32].inertia[2][3] + T6061[5][3];
JA60[5][4]=links[32].mcm[3] + T6061[5][4];
JA60[5][5]=T6061[5][5];
JA60[5][6]=-links[32].mcm[1] + T6061[5][6];

JA60[6][1]=links[32].inertia[1][3] + T6061[6][1];
JA60[6][2]=links[32].inertia[2][3] + T6061[6][2];
JA60[6][3]=links[32].inertia[3][3] + T6061[6][3];
JA60[6][4]=-links[32].mcm[2] + T6061[6][4];
JA60[6][5]=links[32].mcm[1] + T6061[6][5];
JA60[6][6]=T6061[6][6];


h60[1]=JA60[1][3];
h60[2]=JA60[2][3];
h60[3]=JA60[3][3];
h60[4]=JA60[4][3];
h60[5]=JA60[5][3];
h60[6]=JA60[6][3];

T1360[1][1]=JA60[1][1];
T1360[1][2]=JA60[1][2];
T1360[1][3]=JA60[1][3];
T1360[1][4]=JA60[1][4];
T1360[1][5]=JA60[1][5];
T1360[1][6]=JA60[1][6];

T1360[2][1]=JA60[2][1];
T1360[2][2]=JA60[2][2];
T1360[2][3]=JA60[2][3];
T1360[2][4]=JA60[2][4];
T1360[2][5]=JA60[2][5];
T1360[2][6]=JA60[2][6];

T1360[3][1]=JA60[3][1];
T1360[3][2]=JA60[3][2];
T1360[3][3]=JA60[3][3];
T1360[3][4]=JA60[3][4];
T1360[3][5]=JA60[3][5];
T1360[3][6]=JA60[3][6];

T1360[4][1]=JA60[4][1];
T1360[4][2]=JA60[4][2];
T1360[4][3]=JA60[4][3];
T1360[4][4]=JA60[4][4];
T1360[4][5]=JA60[4][5];
T1360[4][6]=JA60[4][6];

T1360[5][1]=JA60[5][1];
T1360[5][2]=JA60[5][2];
T1360[5][3]=JA60[5][3];
T1360[5][4]=JA60[5][4];
T1360[5][5]=JA60[5][5];
T1360[5][6]=JA60[5][6];

T1360[6][1]=JA60[6][1];
T1360[6][2]=JA60[6][2];
T1360[6][3]=JA60[6][3];
T1360[6][4]=JA60[6][4];
T1360[6][5]=JA60[6][5];
T1360[6][6]=JA60[6][6];


T360[1][1]=S603[1][1]*(Si360[1][1]*T1360[1][1] + Si360[1][2]*T1360[2][1]) + S603[2][1]*(Si360[1][1]*T1360[1][2] + Si360[1][2]*T1360[2][2]);
T360[1][2]=S603[1][2]*(Si360[1][1]*T1360[1][1] + Si360[1][2]*T1360[2][1]) + S603[2][2]*(Si360[1][1]*T1360[1][2] + Si360[1][2]*T1360[2][2]) + THORAX2NECK*(Si360[1][1]*T1360[1][6] + Si360[1][2]*T1360[2][6]);
T360[1][3]=Si360[1][1]*T1360[1][3] + Si360[1][2]*T1360[2][3] - THORAX2NECK*S603[1][2]*(Si360[1][1]*T1360[1][4] + Si360[1][2]*T1360[2][4]) - THORAX2NECK*S603[2][2]*(Si360[1][1]*T1360[1][5] + Si360[1][2]*T1360[2][5]);
T360[1][4]=S603[1][1]*(Si360[1][1]*T1360[1][4] + Si360[1][2]*T1360[2][4]) + S603[2][1]*(Si360[1][1]*T1360[1][5] + Si360[1][2]*T1360[2][5]);
T360[1][5]=S603[1][2]*(Si360[1][1]*T1360[1][4] + Si360[1][2]*T1360[2][4]) + S603[2][2]*(Si360[1][1]*T1360[1][5] + Si360[1][2]*T1360[2][5]);
T360[1][6]=Si360[1][1]*T1360[1][6] + Si360[1][2]*T1360[2][6];

T360[2][1]=S603[1][1]*(Si360[2][1]*T1360[1][1] + Si360[2][2]*T1360[2][1]) + S603[2][1]*(Si360[2][1]*T1360[1][2] + Si360[2][2]*T1360[2][2]);
T360[2][2]=S603[1][2]*(Si360[2][1]*T1360[1][1] + Si360[2][2]*T1360[2][1]) + S603[2][2]*(Si360[2][1]*T1360[1][2] + Si360[2][2]*T1360[2][2]) + THORAX2NECK*(Si360[2][1]*T1360[1][6] + Si360[2][2]*T1360[2][6]);
T360[2][3]=Si360[2][1]*T1360[1][3] + Si360[2][2]*T1360[2][3] - THORAX2NECK*S603[1][2]*(Si360[2][1]*T1360[1][4] + Si360[2][2]*T1360[2][4]) - THORAX2NECK*S603[2][2]*(Si360[2][1]*T1360[1][5] + Si360[2][2]*T1360[2][5]);
T360[2][4]=S603[1][1]*(Si360[2][1]*T1360[1][4] + Si360[2][2]*T1360[2][4]) + S603[2][1]*(Si360[2][1]*T1360[1][5] + Si360[2][2]*T1360[2][5]);
T360[2][5]=S603[1][2]*(Si360[2][1]*T1360[1][4] + Si360[2][2]*T1360[2][4]) + S603[2][2]*(Si360[2][1]*T1360[1][5] + Si360[2][2]*T1360[2][5]);
T360[2][6]=Si360[2][1]*T1360[1][6] + Si360[2][2]*T1360[2][6];

T360[3][1]=S603[1][1]*T1360[3][1] + S603[2][1]*T1360[3][2];
T360[3][2]=S603[1][2]*T1360[3][1] + S603[2][2]*T1360[3][2] + THORAX2NECK*T1360[3][6];
T360[3][3]=T1360[3][3] - THORAX2NECK*S603[1][2]*T1360[3][4] - THORAX2NECK*S603[2][2]*T1360[3][5];
T360[3][4]=S603[1][1]*T1360[3][4] + S603[2][1]*T1360[3][5];
T360[3][5]=S603[1][2]*T1360[3][4] + S603[2][2]*T1360[3][5];
T360[3][6]=T1360[3][6];

T360[4][1]=S603[1][1]*(Si360[1][1]*T1360[4][1] + Si360[1][2]*T1360[5][1]) + S603[2][1]*(Si360[1][1]*T1360[4][2] + Si360[1][2]*T1360[5][2]);
T360[4][2]=S603[1][2]*(Si360[1][1]*T1360[4][1] + Si360[1][2]*T1360[5][1]) + S603[2][2]*(Si360[1][1]*T1360[4][2] + Si360[1][2]*T1360[5][2]) + THORAX2NECK*(Si360[1][1]*T1360[4][6] + Si360[1][2]*T1360[5][6]);
T360[4][3]=Si360[1][1]*T1360[4][3] + Si360[1][2]*T1360[5][3] - THORAX2NECK*S603[1][2]*(Si360[1][1]*T1360[4][4] + Si360[1][2]*T1360[5][4]) - THORAX2NECK*S603[2][2]*(Si360[1][1]*T1360[4][5] + Si360[1][2]*T1360[5][5]);
T360[4][4]=S603[1][1]*(Si360[1][1]*T1360[4][4] + Si360[1][2]*T1360[5][4]) + S603[2][1]*(Si360[1][1]*T1360[4][5] + Si360[1][2]*T1360[5][5]);
T360[4][5]=S603[1][2]*(Si360[1][1]*T1360[4][4] + Si360[1][2]*T1360[5][4]) + S603[2][2]*(Si360[1][1]*T1360[4][5] + Si360[1][2]*T1360[5][5]);
T360[4][6]=Si360[1][1]*T1360[4][6] + Si360[1][2]*T1360[5][6];

T360[5][1]=S603[1][1]*(THORAX2NECK*T1360[3][1] + Si360[2][1]*T1360[4][1] + Si360[2][2]*T1360[5][1]) + S603[2][1]*(THORAX2NECK*T1360[3][2] + Si360[2][1]*T1360[4][2] + Si360[2][2]*T1360[5][2]);
T360[5][2]=S603[1][2]*(THORAX2NECK*T1360[3][1] + Si360[2][1]*T1360[4][1] + Si360[2][2]*T1360[5][1]) + S603[2][2]*(THORAX2NECK*T1360[3][2] + Si360[2][1]*T1360[4][2] + Si360[2][2]*T1360[5][2]) + THORAX2NECK*(THORAX2NECK*T1360[3][6] + Si360[2][1]*T1360[4][6] + Si360[2][2]*T1360[5][6]);
T360[5][3]=THORAX2NECK*T1360[3][3] + Si360[2][1]*T1360[4][3] + Si360[2][2]*T1360[5][3] - THORAX2NECK*S603[1][2]*(THORAX2NECK*T1360[3][4] + Si360[2][1]*T1360[4][4] + Si360[2][2]*T1360[5][4]) - THORAX2NECK*S603[2][2]*(THORAX2NECK*T1360[3][5] + Si360[2][1]*T1360[4][5] + Si360[2][2]*T1360[5][5]);
T360[5][4]=S603[1][1]*(THORAX2NECK*T1360[3][4] + Si360[2][1]*T1360[4][4] + Si360[2][2]*T1360[5][4]) + S603[2][1]*(THORAX2NECK*T1360[3][5] + Si360[2][1]*T1360[4][5] + Si360[2][2]*T1360[5][5]);
T360[5][5]=S603[1][2]*(THORAX2NECK*T1360[3][4] + Si360[2][1]*T1360[4][4] + Si360[2][2]*T1360[5][4]) + S603[2][2]*(THORAX2NECK*T1360[3][5] + Si360[2][1]*T1360[4][5] + Si360[2][2]*T1360[5][5]);
T360[5][6]=THORAX2NECK*T1360[3][6] + Si360[2][1]*T1360[4][6] + Si360[2][2]*T1360[5][6];

T360[6][1]=S603[1][1]*(-(THORAX2NECK*Si360[2][1]*T1360[1][1]) - THORAX2NECK*Si360[2][2]*T1360[2][1] + T1360[6][1]) + S603[2][1]*(-(THORAX2NECK*Si360[2][1]*T1360[1][2]) - THORAX2NECK*Si360[2][2]*T1360[2][2] + T1360[6][2]);
T360[6][2]=S603[1][2]*(-(THORAX2NECK*Si360[2][1]*T1360[1][1]) - THORAX2NECK*Si360[2][2]*T1360[2][1] + T1360[6][1]) + S603[2][2]*(-(THORAX2NECK*Si360[2][1]*T1360[1][2]) - THORAX2NECK*Si360[2][2]*T1360[2][2] + T1360[6][2]) + THORAX2NECK*(-(THORAX2NECK*Si360[2][1]*T1360[1][6]) - THORAX2NECK*Si360[2][2]*T1360[2][6] + T1360[6][6]);
T360[6][3]=-(THORAX2NECK*Si360[2][1]*T1360[1][3]) - THORAX2NECK*Si360[2][2]*T1360[2][3] + T1360[6][3] - THORAX2NECK*S603[1][2]*(-(THORAX2NECK*Si360[2][1]*T1360[1][4]) - THORAX2NECK*Si360[2][2]*T1360[2][4] + T1360[6][4]) - THORAX2NECK*S603[2][2]*(-(THORAX2NECK*Si360[2][1]*T1360[1][5]) - THORAX2NECK*Si360[2][2]*T1360[2][5] + T1360[6][5]);
T360[6][4]=S603[1][1]*(-(THORAX2NECK*Si360[2][1]*T1360[1][4]) - THORAX2NECK*Si360[2][2]*T1360[2][4] + T1360[6][4]) + S603[2][1]*(-(THORAX2NECK*Si360[2][1]*T1360[1][5]) - THORAX2NECK*Si360[2][2]*T1360[2][5] + T1360[6][5]);
T360[6][5]=S603[1][2]*(-(THORAX2NECK*Si360[2][1]*T1360[1][4]) - THORAX2NECK*Si360[2][2]*T1360[2][4] + T1360[6][4]) + S603[2][2]*(-(THORAX2NECK*Si360[2][1]*T1360[1][5]) - THORAX2NECK*Si360[2][2]*T1360[2][5] + T1360[6][5]);
T360[6][6]=-(THORAX2NECK*Si360[2][1]*T1360[1][6]) - THORAX2NECK*Si360[2][2]*T1360[2][6] + T1360[6][6];



}


void
hermes_InvDynArtfunc45(void)
      {




}


void
hermes_InvDynArtfunc46(void)
      {




}


void
hermes_InvDynArtfunc47(void)
      {




}


void
hermes_InvDynArtfunc48(void)
      {
JA56[1][2]=0. + links[50].mcm[3];
JA56[1][3]=0. - links[50].mcm[2];
JA56[1][4]=0. + links[50].m;

JA56[2][1]=0. - links[50].mcm[3];
JA56[2][3]=0. + links[50].mcm[1];
JA56[2][5]=0. + links[50].m;

JA56[3][1]=0. + links[50].mcm[2];
JA56[3][2]=0. - links[50].mcm[1];
JA56[3][6]=0. + links[50].m;

JA56[4][1]=0. + links[50].inertia[1][1];
JA56[4][2]=0. + links[50].inertia[1][2];
JA56[4][3]=0. + links[50].inertia[1][3];
JA56[4][5]=0. - links[50].mcm[3];
JA56[4][6]=0. + links[50].mcm[2];

JA56[5][1]=0. + links[50].inertia[1][2];
JA56[5][2]=0. + links[50].inertia[2][2];
JA56[5][3]=0. + links[50].inertia[2][3];
JA56[5][4]=0. + links[50].mcm[3];
JA56[5][6]=0. - links[50].mcm[1];

JA56[6][1]=0. + links[50].inertia[1][3];
JA56[6][2]=0. + links[50].inertia[2][3];
JA56[6][3]=0. + links[50].inertia[3][3];
JA56[6][4]=0. - links[50].mcm[2];
JA56[6][5]=0. + links[50].mcm[1];


h56[1]=JA56[1][3];
h56[2]=JA56[2][3];
h56[4]=JA56[4][3];
h56[5]=JA56[5][3];
h56[6]=JA56[6][3];

T13856[1][2]=JA56[1][2];
T13856[1][3]=JA56[1][3];
T13856[1][4]=JA56[1][4];

T13856[2][1]=JA56[2][1];
T13856[2][3]=JA56[2][3];
T13856[2][5]=JA56[2][5];

T13856[3][1]=JA56[3][1];
T13856[3][2]=JA56[3][2];
T13856[3][6]=JA56[3][6];

T13856[4][1]=JA56[4][1];
T13856[4][2]=JA56[4][2];
T13856[4][3]=JA56[4][3];
T13856[4][5]=JA56[4][5];
T13856[4][6]=JA56[4][6];

T13856[5][1]=JA56[5][1];
T13856[5][2]=JA56[5][2];
T13856[5][3]=JA56[5][3];
T13856[5][4]=JA56[5][4];
T13856[5][6]=JA56[5][6];

T13856[6][1]=JA56[6][1];
T13856[6][2]=JA56[6][2];
T13856[6][3]=JA56[6][3];
T13856[6][4]=JA56[6][4];
T13856[6][5]=JA56[6][5];


T3856[1][1]=S5638[2][1]*Si3856[1][1]*T13856[1][2] + (-(ZLF*S5638[1][2]) + YLF*S5638[1][3])*Si3856[1][1]*T13856[1][4] + S5638[1][1]*Si3856[1][2]*T13856[2][1] + (-(ZLF*S5638[2][2]) + YLF*S5638[2][3])*Si3856[1][2]*T13856[2][5];
T3856[1][2]=S5638[2][2]*Si3856[1][1]*T13856[1][2] + (ZLF*S5638[1][1] - XLF*S5638[1][3])*Si3856[1][1]*T13856[1][4] + S5638[1][2]*Si3856[1][2]*T13856[2][1] + S5638[3][2]*(Si3856[1][1]*T13856[1][3] + Si3856[1][2]*T13856[2][3]) + (ZLF*S5638[2][1] - XLF*S5638[2][3])*Si3856[1][2]*T13856[2][5];
T3856[1][3]=S5638[2][3]*Si3856[1][1]*T13856[1][2] + (-(YLF*S5638[1][1]) + XLF*S5638[1][2])*Si3856[1][1]*T13856[1][4] + S5638[1][3]*Si3856[1][2]*T13856[2][1] + S5638[3][3]*(Si3856[1][1]*T13856[1][3] + Si3856[1][2]*T13856[2][3]) + (-(YLF*S5638[2][1]) + XLF*S5638[2][2])*Si3856[1][2]*T13856[2][5];
T3856[1][4]=S5638[1][1]*Si3856[1][1]*T13856[1][4] + S5638[2][1]*Si3856[1][2]*T13856[2][5];
T3856[1][5]=S5638[1][2]*Si3856[1][1]*T13856[1][4] + S5638[2][2]*Si3856[1][2]*T13856[2][5];
T3856[1][6]=S5638[1][3]*Si3856[1][1]*T13856[1][4] + S5638[2][3]*Si3856[1][2]*T13856[2][5];

T3856[2][1]=(-(ZLF*S5638[1][2]) + YLF*S5638[1][3])*Si3856[2][1]*T13856[1][4] + (-(ZLF*S5638[2][2]) + YLF*S5638[2][3])*Si3856[2][2]*T13856[2][5] + S5638[1][1]*(Si3856[2][2]*T13856[2][1] + Si3856[2][3]*T13856[3][1]) + S5638[2][1]*(Si3856[2][1]*T13856[1][2] + Si3856[2][3]*T13856[3][2]) + (-(ZLF*S5638[3][2]) + YLF*S5638[3][3])*Si3856[2][3]*T13856[3][6];
T3856[2][2]=(ZLF*S5638[1][1] - XLF*S5638[1][3])*Si3856[2][1]*T13856[1][4] + S5638[3][2]*(Si3856[2][1]*T13856[1][3] + Si3856[2][2]*T13856[2][3]) + (ZLF*S5638[2][1] - XLF*S5638[2][3])*Si3856[2][2]*T13856[2][5] + S5638[1][2]*(Si3856[2][2]*T13856[2][1] + Si3856[2][3]*T13856[3][1]) + S5638[2][2]*(Si3856[2][1]*T13856[1][2] + Si3856[2][3]*T13856[3][2]) - XLF*S5638[3][3]*Si3856[2][3]*T13856[3][6];
T3856[2][3]=(-(YLF*S5638[1][1]) + XLF*S5638[1][2])*Si3856[2][1]*T13856[1][4] + S5638[3][3]*(Si3856[2][1]*T13856[1][3] + Si3856[2][2]*T13856[2][3]) + (-(YLF*S5638[2][1]) + XLF*S5638[2][2])*Si3856[2][2]*T13856[2][5] + S5638[1][3]*(Si3856[2][2]*T13856[2][1] + Si3856[2][3]*T13856[3][1]) + S5638[2][3]*(Si3856[2][1]*T13856[1][2] + Si3856[2][3]*T13856[3][2]) + XLF*S5638[3][2]*Si3856[2][3]*T13856[3][6];
T3856[2][4]=S5638[1][1]*Si3856[2][1]*T13856[1][4] + S5638[2][1]*Si3856[2][2]*T13856[2][5];
T3856[2][5]=S5638[1][2]*Si3856[2][1]*T13856[1][4] + S5638[2][2]*Si3856[2][2]*T13856[2][5] + S5638[3][2]*Si3856[2][3]*T13856[3][6];
T3856[2][6]=S5638[1][3]*Si3856[2][1]*T13856[1][4] + S5638[2][3]*Si3856[2][2]*T13856[2][5] + S5638[3][3]*Si3856[2][3]*T13856[3][6];

T3856[3][1]=(-(ZLF*S5638[1][2]) + YLF*S5638[1][3])*Si3856[3][1]*T13856[1][4] + (-(ZLF*S5638[2][2]) + YLF*S5638[2][3])*Si3856[3][2]*T13856[2][5] + S5638[1][1]*(Si3856[3][2]*T13856[2][1] + Si3856[3][3]*T13856[3][1]) + S5638[2][1]*(Si3856[3][1]*T13856[1][2] + Si3856[3][3]*T13856[3][2]) + (-(ZLF*S5638[3][2]) + YLF*S5638[3][3])*Si3856[3][3]*T13856[3][6];
T3856[3][2]=(ZLF*S5638[1][1] - XLF*S5638[1][3])*Si3856[3][1]*T13856[1][4] + S5638[3][2]*(Si3856[3][1]*T13856[1][3] + Si3856[3][2]*T13856[2][3]) + (ZLF*S5638[2][1] - XLF*S5638[2][3])*Si3856[3][2]*T13856[2][5] + S5638[1][2]*(Si3856[3][2]*T13856[2][1] + Si3856[3][3]*T13856[3][1]) + S5638[2][2]*(Si3856[3][1]*T13856[1][2] + Si3856[3][3]*T13856[3][2]) - XLF*S5638[3][3]*Si3856[3][3]*T13856[3][6];
T3856[3][3]=(-(YLF*S5638[1][1]) + XLF*S5638[1][2])*Si3856[3][1]*T13856[1][4] + S5638[3][3]*(Si3856[3][1]*T13856[1][3] + Si3856[3][2]*T13856[2][3]) + (-(YLF*S5638[2][1]) + XLF*S5638[2][2])*Si3856[3][2]*T13856[2][5] + S5638[1][3]*(Si3856[3][2]*T13856[2][1] + Si3856[3][3]*T13856[3][1]) + S5638[2][3]*(Si3856[3][1]*T13856[1][2] + Si3856[3][3]*T13856[3][2]) + XLF*S5638[3][2]*Si3856[3][3]*T13856[3][6];
T3856[3][4]=S5638[1][1]*Si3856[3][1]*T13856[1][4] + S5638[2][1]*Si3856[3][2]*T13856[2][5];
T3856[3][5]=S5638[1][2]*Si3856[3][1]*T13856[1][4] + S5638[2][2]*Si3856[3][2]*T13856[2][5] + S5638[3][2]*Si3856[3][3]*T13856[3][6];
T3856[3][6]=S5638[1][3]*Si3856[3][1]*T13856[1][4] + S5638[2][3]*Si3856[3][2]*T13856[2][5] + S5638[3][3]*Si3856[3][3]*T13856[3][6];

T3856[4][1]=(-(ZLF*S5638[2][2]) + YLF*S5638[2][3])*((-(ZLF*Si3856[2][2]) + YLF*Si3856[3][2])*T13856[2][5] + Si3856[1][1]*T13856[4][5]) + S5638[1][1]*((-(ZLF*Si3856[2][2]) + YLF*Si3856[3][2])*T13856[2][1] + (-(ZLF*Si3856[2][3]) + YLF*Si3856[3][3])*T13856[3][1] + Si3856[1][1]*T13856[4][1] + Si3856[1][2]*T13856[5][1]) + S5638[2][1]*((-(ZLF*Si3856[2][1]) + YLF*Si3856[3][1])*T13856[1][2] + (-(ZLF*Si3856[2][3]) + YLF*Si3856[3][3])*T13856[3][2] + Si3856[1][1]*T13856[4][2] + Si3856[1][2]*T13856[5][2]) + (-(ZLF*S5638[1][2]) + YLF*S5638[1][3])*((-(ZLF*Si3856[2][1]) + YLF*Si3856[3][1])*T13856[1][4] + Si3856[1][2]*T13856[5][4]) + (-(ZLF*S5638[3][2]) + YLF*S5638[3][3])*((-(ZLF*Si3856[2][3]) + YLF*Si3856[3][3])*T13856[3][6] + Si3856[1][1]*T13856[4][6] + Si3856[1][2]*T13856[5][6]);
T3856[4][2]=(ZLF*S5638[2][1] - XLF*S5638[2][3])*((-(ZLF*Si3856[2][2]) + YLF*Si3856[3][2])*T13856[2][5] + Si3856[1][1]*T13856[4][5]) + S5638[1][2]*((-(ZLF*Si3856[2][2]) + YLF*Si3856[3][2])*T13856[2][1] + (-(ZLF*Si3856[2][3]) + YLF*Si3856[3][3])*T13856[3][1] + Si3856[1][1]*T13856[4][1] + Si3856[1][2]*T13856[5][1]) + S5638[2][2]*((-(ZLF*Si3856[2][1]) + YLF*Si3856[3][1])*T13856[1][2] + (-(ZLF*Si3856[2][3]) + YLF*Si3856[3][3])*T13856[3][2] + Si3856[1][1]*T13856[4][2] + Si3856[1][2]*T13856[5][2]) + S5638[3][2]*((-(ZLF*Si3856[2][1]) + YLF*Si3856[3][1])*T13856[1][3] + (-(ZLF*Si3856[2][2]) + YLF*Si3856[3][2])*T13856[2][3] + Si3856[1][1]*T13856[4][3] + Si3856[1][2]*T13856[5][3]) + (ZLF*S5638[1][1] - XLF*S5638[1][3])*((-(ZLF*Si3856[2][1]) + YLF*Si3856[3][1])*T13856[1][4] + Si3856[1][2]*T13856[5][4]) - XLF*S5638[3][3]*((-(ZLF*Si3856[2][3]) + YLF*Si3856[3][3])*T13856[3][6] + Si3856[1][1]*T13856[4][6] + Si3856[1][2]*T13856[5][6]);
T3856[4][3]=(-(YLF*S5638[2][1]) + XLF*S5638[2][2])*((-(ZLF*Si3856[2][2]) + YLF*Si3856[3][2])*T13856[2][5] + Si3856[1][1]*T13856[4][5]) + S5638[1][3]*((-(ZLF*Si3856[2][2]) + YLF*Si3856[3][2])*T13856[2][1] + (-(ZLF*Si3856[2][3]) + YLF*Si3856[3][3])*T13856[3][1] + Si3856[1][1]*T13856[4][1] + Si3856[1][2]*T13856[5][1]) + S5638[2][3]*((-(ZLF*Si3856[2][1]) + YLF*Si3856[3][1])*T13856[1][2] + (-(ZLF*Si3856[2][3]) + YLF*Si3856[3][3])*T13856[3][2] + Si3856[1][1]*T13856[4][2] + Si3856[1][2]*T13856[5][2]) + S5638[3][3]*((-(ZLF*Si3856[2][1]) + YLF*Si3856[3][1])*T13856[1][3] + (-(ZLF*Si3856[2][2]) + YLF*Si3856[3][2])*T13856[2][3] + Si3856[1][1]*T13856[4][3] + Si3856[1][2]*T13856[5][3]) + (-(YLF*S5638[1][1]) + XLF*S5638[1][2])*((-(ZLF*Si3856[2][1]) + YLF*Si3856[3][1])*T13856[1][4] + Si3856[1][2]*T13856[5][4]) + XLF*S5638[3][2]*((-(ZLF*Si3856[2][3]) + YLF*Si3856[3][3])*T13856[3][6] + Si3856[1][1]*T13856[4][6] + Si3856[1][2]*T13856[5][6]);
T3856[4][4]=S5638[2][1]*((-(ZLF*Si3856[2][2]) + YLF*Si3856[3][2])*T13856[2][5] + Si3856[1][1]*T13856[4][5]) + S5638[1][1]*((-(ZLF*Si3856[2][1]) + YLF*Si3856[3][1])*T13856[1][4] + Si3856[1][2]*T13856[5][4]);
T3856[4][5]=S5638[2][2]*((-(ZLF*Si3856[2][2]) + YLF*Si3856[3][2])*T13856[2][5] + Si3856[1][1]*T13856[4][5]) + S5638[1][2]*((-(ZLF*Si3856[2][1]) + YLF*Si3856[3][1])*T13856[1][4] + Si3856[1][2]*T13856[5][4]) + S5638[3][2]*((-(ZLF*Si3856[2][3]) + YLF*Si3856[3][3])*T13856[3][6] + Si3856[1][1]*T13856[4][6] + Si3856[1][2]*T13856[5][6]);
T3856[4][6]=S5638[2][3]*((-(ZLF*Si3856[2][2]) + YLF*Si3856[3][2])*T13856[2][5] + Si3856[1][1]*T13856[4][5]) + S5638[1][3]*((-(ZLF*Si3856[2][1]) + YLF*Si3856[3][1])*T13856[1][4] + Si3856[1][2]*T13856[5][4]) + S5638[3][3]*((-(ZLF*Si3856[2][3]) + YLF*Si3856[3][3])*T13856[3][6] + Si3856[1][1]*T13856[4][6] + Si3856[1][2]*T13856[5][6]);

T3856[5][1]=(-(ZLF*S5638[3][2]) + YLF*S5638[3][3])*(-(XLF*Si3856[3][3]*T13856[3][6]) + Si3856[2][1]*T13856[4][6] + Si3856[2][2]*T13856[5][6]) + S5638[1][1]*((ZLF*Si3856[1][2] - XLF*Si3856[3][2])*T13856[2][1] - XLF*Si3856[3][3]*T13856[3][1] + Si3856[2][1]*T13856[4][1] + Si3856[2][2]*T13856[5][1] + Si3856[2][3]*T13856[6][1]) + S5638[2][1]*((ZLF*Si3856[1][1] - XLF*Si3856[3][1])*T13856[1][2] - XLF*Si3856[3][3]*T13856[3][2] + Si3856[2][1]*T13856[4][2] + Si3856[2][2]*T13856[5][2] + Si3856[2][3]*T13856[6][2]) + (-(ZLF*S5638[1][2]) + YLF*S5638[1][3])*((ZLF*Si3856[1][1] - XLF*Si3856[3][1])*T13856[1][4] + Si3856[2][2]*T13856[5][4] + Si3856[2][3]*T13856[6][4]) + (-(ZLF*S5638[2][2]) + YLF*S5638[2][3])*((ZLF*Si3856[1][2] - XLF*Si3856[3][2])*T13856[2][5] + Si3856[2][1]*T13856[4][5] + Si3856[2][3]*T13856[6][5]);
T3856[5][2]=-(XLF*S5638[3][3]*(-(XLF*Si3856[3][3]*T13856[3][6]) + Si3856[2][1]*T13856[4][6] + Si3856[2][2]*T13856[5][6])) + S5638[1][2]*((ZLF*Si3856[1][2] - XLF*Si3856[3][2])*T13856[2][1] - XLF*Si3856[3][3]*T13856[3][1] + Si3856[2][1]*T13856[4][1] + Si3856[2][2]*T13856[5][1] + Si3856[2][3]*T13856[6][1]) + S5638[2][2]*((ZLF*Si3856[1][1] - XLF*Si3856[3][1])*T13856[1][2] - XLF*Si3856[3][3]*T13856[3][2] + Si3856[2][1]*T13856[4][2] + Si3856[2][2]*T13856[5][2] + Si3856[2][3]*T13856[6][2]) + S5638[3][2]*((ZLF*Si3856[1][1] - XLF*Si3856[3][1])*T13856[1][3] + (ZLF*Si3856[1][2] - XLF*Si3856[3][2])*T13856[2][3] + Si3856[2][1]*T13856[4][3] + Si3856[2][2]*T13856[5][3] + Si3856[2][3]*T13856[6][3]) + (ZLF*S5638[1][1] - XLF*S5638[1][3])*((ZLF*Si3856[1][1] - XLF*Si3856[3][1])*T13856[1][4] + Si3856[2][2]*T13856[5][4] + Si3856[2][3]*T13856[6][4]) + (ZLF*S5638[2][1] - XLF*S5638[2][3])*((ZLF*Si3856[1][2] - XLF*Si3856[3][2])*T13856[2][5] + Si3856[2][1]*T13856[4][5] + Si3856[2][3]*T13856[6][5]);
T3856[5][3]=XLF*S5638[3][2]*(-(XLF*Si3856[3][3]*T13856[3][6]) + Si3856[2][1]*T13856[4][6] + Si3856[2][2]*T13856[5][6]) + S5638[1][3]*((ZLF*Si3856[1][2] - XLF*Si3856[3][2])*T13856[2][1] - XLF*Si3856[3][3]*T13856[3][1] + Si3856[2][1]*T13856[4][1] + Si3856[2][2]*T13856[5][1] + Si3856[2][3]*T13856[6][1]) + S5638[2][3]*((ZLF*Si3856[1][1] - XLF*Si3856[3][1])*T13856[1][2] - XLF*Si3856[3][3]*T13856[3][2] + Si3856[2][1]*T13856[4][2] + Si3856[2][2]*T13856[5][2] + Si3856[2][3]*T13856[6][2]) + S5638[3][3]*((ZLF*Si3856[1][1] - XLF*Si3856[3][1])*T13856[1][3] + (ZLF*Si3856[1][2] - XLF*Si3856[3][2])*T13856[2][3] + Si3856[2][1]*T13856[4][3] + Si3856[2][2]*T13856[5][3] + Si3856[2][3]*T13856[6][3]) + (-(YLF*S5638[1][1]) + XLF*S5638[1][2])*((ZLF*Si3856[1][1] - XLF*Si3856[3][1])*T13856[1][4] + Si3856[2][2]*T13856[5][4] + Si3856[2][3]*T13856[6][4]) + (-(YLF*S5638[2][1]) + XLF*S5638[2][2])*((ZLF*Si3856[1][2] - XLF*Si3856[3][2])*T13856[2][5] + Si3856[2][1]*T13856[4][5] + Si3856[2][3]*T13856[6][5]);
T3856[5][4]=S5638[1][1]*((ZLF*Si3856[1][1] - XLF*Si3856[3][1])*T13856[1][4] + Si3856[2][2]*T13856[5][4] + Si3856[2][3]*T13856[6][4]) + S5638[2][1]*((ZLF*Si3856[1][2] - XLF*Si3856[3][2])*T13856[2][5] + Si3856[2][1]*T13856[4][5] + Si3856[2][3]*T13856[6][5]);
T3856[5][5]=S5638[3][2]*(-(XLF*Si3856[3][3]*T13856[3][6]) + Si3856[2][1]*T13856[4][6] + Si3856[2][2]*T13856[5][6]) + S5638[1][2]*((ZLF*Si3856[1][1] - XLF*Si3856[3][1])*T13856[1][4] + Si3856[2][2]*T13856[5][4] + Si3856[2][3]*T13856[6][4]) + S5638[2][2]*((ZLF*Si3856[1][2] - XLF*Si3856[3][2])*T13856[2][5] + Si3856[2][1]*T13856[4][5] + Si3856[2][3]*T13856[6][5]);
T3856[5][6]=S5638[3][3]*(-(XLF*Si3856[3][3]*T13856[3][6]) + Si3856[2][1]*T13856[4][6] + Si3856[2][2]*T13856[5][6]) + S5638[1][3]*((ZLF*Si3856[1][1] - XLF*Si3856[3][1])*T13856[1][4] + Si3856[2][2]*T13856[5][4] + Si3856[2][3]*T13856[6][4]) + S5638[2][3]*((ZLF*Si3856[1][2] - XLF*Si3856[3][2])*T13856[2][5] + Si3856[2][1]*T13856[4][5] + Si3856[2][3]*T13856[6][5]);

T3856[6][1]=(-(ZLF*S5638[3][2]) + YLF*S5638[3][3])*(XLF*Si3856[2][3]*T13856[3][6] + Si3856[3][1]*T13856[4][6] + Si3856[3][2]*T13856[5][6]) + S5638[1][1]*((-(YLF*Si3856[1][2]) + XLF*Si3856[2][2])*T13856[2][1] + XLF*Si3856[2][3]*T13856[3][1] + Si3856[3][1]*T13856[4][1] + Si3856[3][2]*T13856[5][1] + Si3856[3][3]*T13856[6][1]) + S5638[2][1]*((-(YLF*Si3856[1][1]) + XLF*Si3856[2][1])*T13856[1][2] + XLF*Si3856[2][3]*T13856[3][2] + Si3856[3][1]*T13856[4][2] + Si3856[3][2]*T13856[5][2] + Si3856[3][3]*T13856[6][2]) + (-(ZLF*S5638[1][2]) + YLF*S5638[1][3])*((-(YLF*Si3856[1][1]) + XLF*Si3856[2][1])*T13856[1][4] + Si3856[3][2]*T13856[5][4] + Si3856[3][3]*T13856[6][4]) + (-(ZLF*S5638[2][2]) + YLF*S5638[2][3])*((-(YLF*Si3856[1][2]) + XLF*Si3856[2][2])*T13856[2][5] + Si3856[3][1]*T13856[4][5] + Si3856[3][3]*T13856[6][5]);
T3856[6][2]=-(XLF*S5638[3][3]*(XLF*Si3856[2][3]*T13856[3][6] + Si3856[3][1]*T13856[4][6] + Si3856[3][2]*T13856[5][6])) + S5638[1][2]*((-(YLF*Si3856[1][2]) + XLF*Si3856[2][2])*T13856[2][1] + XLF*Si3856[2][3]*T13856[3][1] + Si3856[3][1]*T13856[4][1] + Si3856[3][2]*T13856[5][1] + Si3856[3][3]*T13856[6][1]) + S5638[2][2]*((-(YLF*Si3856[1][1]) + XLF*Si3856[2][1])*T13856[1][2] + XLF*Si3856[2][3]*T13856[3][2] + Si3856[3][1]*T13856[4][2] + Si3856[3][2]*T13856[5][2] + Si3856[3][3]*T13856[6][2]) + S5638[3][2]*((-(YLF*Si3856[1][1]) + XLF*Si3856[2][1])*T13856[1][3] + (-(YLF*Si3856[1][2]) + XLF*Si3856[2][2])*T13856[2][3] + Si3856[3][1]*T13856[4][3] + Si3856[3][2]*T13856[5][3] + Si3856[3][3]*T13856[6][3]) + (ZLF*S5638[1][1] - XLF*S5638[1][3])*((-(YLF*Si3856[1][1]) + XLF*Si3856[2][1])*T13856[1][4] + Si3856[3][2]*T13856[5][4] + Si3856[3][3]*T13856[6][4]) + (ZLF*S5638[2][1] - XLF*S5638[2][3])*((-(YLF*Si3856[1][2]) + XLF*Si3856[2][2])*T13856[2][5] + Si3856[3][1]*T13856[4][5] + Si3856[3][3]*T13856[6][5]);
T3856[6][3]=XLF*S5638[3][2]*(XLF*Si3856[2][3]*T13856[3][6] + Si3856[3][1]*T13856[4][6] + Si3856[3][2]*T13856[5][6]) + S5638[1][3]*((-(YLF*Si3856[1][2]) + XLF*Si3856[2][2])*T13856[2][1] + XLF*Si3856[2][3]*T13856[3][1] + Si3856[3][1]*T13856[4][1] + Si3856[3][2]*T13856[5][1] + Si3856[3][3]*T13856[6][1]) + S5638[2][3]*((-(YLF*Si3856[1][1]) + XLF*Si3856[2][1])*T13856[1][2] + XLF*Si3856[2][3]*T13856[3][2] + Si3856[3][1]*T13856[4][2] + Si3856[3][2]*T13856[5][2] + Si3856[3][3]*T13856[6][2]) + S5638[3][3]*((-(YLF*Si3856[1][1]) + XLF*Si3856[2][1])*T13856[1][3] + (-(YLF*Si3856[1][2]) + XLF*Si3856[2][2])*T13856[2][3] + Si3856[3][1]*T13856[4][3] + Si3856[3][2]*T13856[5][3] + Si3856[3][3]*T13856[6][3]) + (-(YLF*S5638[1][1]) + XLF*S5638[1][2])*((-(YLF*Si3856[1][1]) + XLF*Si3856[2][1])*T13856[1][4] + Si3856[3][2]*T13856[5][4] + Si3856[3][3]*T13856[6][4]) + (-(YLF*S5638[2][1]) + XLF*S5638[2][2])*((-(YLF*Si3856[1][2]) + XLF*Si3856[2][2])*T13856[2][5] + Si3856[3][1]*T13856[4][5] + Si3856[3][3]*T13856[6][5]);
T3856[6][4]=S5638[1][1]*((-(YLF*Si3856[1][1]) + XLF*Si3856[2][1])*T13856[1][4] + Si3856[3][2]*T13856[5][4] + Si3856[3][3]*T13856[6][4]) + S5638[2][1]*((-(YLF*Si3856[1][2]) + XLF*Si3856[2][2])*T13856[2][5] + Si3856[3][1]*T13856[4][5] + Si3856[3][3]*T13856[6][5]);
T3856[6][5]=S5638[3][2]*(XLF*Si3856[2][3]*T13856[3][6] + Si3856[3][1]*T13856[4][6] + Si3856[3][2]*T13856[5][6]) + S5638[1][2]*((-(YLF*Si3856[1][1]) + XLF*Si3856[2][1])*T13856[1][4] + Si3856[3][2]*T13856[5][4] + Si3856[3][3]*T13856[6][4]) + S5638[2][2]*((-(YLF*Si3856[1][2]) + XLF*Si3856[2][2])*T13856[2][5] + Si3856[3][1]*T13856[4][5] + Si3856[3][3]*T13856[6][5]);
T3856[6][6]=S5638[3][3]*(XLF*Si3856[2][3]*T13856[3][6] + Si3856[3][1]*T13856[4][6] + Si3856[3][2]*T13856[5][6]) + S5638[1][3]*((-(YLF*Si3856[1][1]) + XLF*Si3856[2][1])*T13856[1][4] + Si3856[3][2]*T13856[5][4] + Si3856[3][3]*T13856[6][4]) + S5638[2][3]*((-(YLF*Si3856[1][2]) + XLF*Si3856[2][2])*T13856[2][5] + Si3856[3][1]*T13856[4][5] + Si3856[3][3]*T13856[6][5]);



}


void
hermes_InvDynArtfunc49(void)
      {




}


void
hermes_InvDynArtfunc50(void)
      {




}


void
hermes_InvDynArtfunc51(void)
      {




}


void
hermes_InvDynArtfunc52(void)
      {
JA52[1][2]=0. + links[49].mcm[3];
JA52[1][3]=0. - links[49].mcm[2];
JA52[1][4]=0. + links[49].m;

JA52[2][1]=0. - links[49].mcm[3];
JA52[2][3]=0. + links[49].mcm[1];
JA52[2][5]=0. + links[49].m;

JA52[3][1]=0. + links[49].mcm[2];
JA52[3][2]=0. - links[49].mcm[1];
JA52[3][6]=0. + links[49].m;

JA52[4][1]=0. + links[49].inertia[1][1];
JA52[4][2]=0. + links[49].inertia[1][2];
JA52[4][3]=0. + links[49].inertia[1][3];
JA52[4][5]=0. - links[49].mcm[3];
JA52[4][6]=0. + links[49].mcm[2];

JA52[5][1]=0. + links[49].inertia[1][2];
JA52[5][2]=0. + links[49].inertia[2][2];
JA52[5][3]=0. + links[49].inertia[2][3];
JA52[5][4]=0. + links[49].mcm[3];
JA52[5][6]=0. - links[49].mcm[1];

JA52[6][1]=0. + links[49].inertia[1][3];
JA52[6][2]=0. + links[49].inertia[2][3];
JA52[6][3]=0. + links[49].inertia[3][3];
JA52[6][4]=0. - links[49].mcm[2];
JA52[6][5]=0. + links[49].mcm[1];


h52[1]=JA52[1][3];
h52[2]=JA52[2][3];
h52[4]=JA52[4][3];
h52[5]=JA52[5][3];
h52[6]=JA52[6][3];

T13852[1][2]=JA52[1][2];
T13852[1][3]=JA52[1][3];
T13852[1][4]=JA52[1][4];

T13852[2][1]=JA52[2][1];
T13852[2][3]=JA52[2][3];
T13852[2][5]=JA52[2][5];

T13852[3][1]=JA52[3][1];
T13852[3][2]=JA52[3][2];
T13852[3][6]=JA52[3][6];

T13852[4][1]=JA52[4][1];
T13852[4][2]=JA52[4][2];
T13852[4][3]=JA52[4][3];
T13852[4][5]=JA52[4][5];
T13852[4][6]=JA52[4][6];

T13852[5][1]=JA52[5][1];
T13852[5][2]=JA52[5][2];
T13852[5][3]=JA52[5][3];
T13852[5][4]=JA52[5][4];
T13852[5][6]=JA52[5][6];

T13852[6][1]=JA52[6][1];
T13852[6][2]=JA52[6][2];
T13852[6][3]=JA52[6][3];
T13852[6][4]=JA52[6][4];
T13852[6][5]=JA52[6][5];


T3852[1][1]=S5238[2][1]*Si3852[1][1]*T13852[1][2] + (-(ZRF*S5238[1][2]) + YRF*S5238[1][3])*Si3852[1][1]*T13852[1][4] + S5238[1][1]*Si3852[1][2]*T13852[2][1] + (-(ZRF*S5238[2][2]) + YRF*S5238[2][3])*Si3852[1][2]*T13852[2][5];
T3852[1][2]=S5238[2][2]*Si3852[1][1]*T13852[1][2] + (ZRF*S5238[1][1] - XRF*S5238[1][3])*Si3852[1][1]*T13852[1][4] + S5238[1][2]*Si3852[1][2]*T13852[2][1] + S5238[3][2]*(Si3852[1][1]*T13852[1][3] + Si3852[1][2]*T13852[2][3]) + (ZRF*S5238[2][1] - XRF*S5238[2][3])*Si3852[1][2]*T13852[2][5];
T3852[1][3]=S5238[2][3]*Si3852[1][1]*T13852[1][2] + (-(YRF*S5238[1][1]) + XRF*S5238[1][2])*Si3852[1][1]*T13852[1][4] + S5238[1][3]*Si3852[1][2]*T13852[2][1] + S5238[3][3]*(Si3852[1][1]*T13852[1][3] + Si3852[1][2]*T13852[2][3]) + (-(YRF*S5238[2][1]) + XRF*S5238[2][2])*Si3852[1][2]*T13852[2][5];
T3852[1][4]=S5238[1][1]*Si3852[1][1]*T13852[1][4] + S5238[2][1]*Si3852[1][2]*T13852[2][5];
T3852[1][5]=S5238[1][2]*Si3852[1][1]*T13852[1][4] + S5238[2][2]*Si3852[1][2]*T13852[2][5];
T3852[1][6]=S5238[1][3]*Si3852[1][1]*T13852[1][4] + S5238[2][3]*Si3852[1][2]*T13852[2][5];

T3852[2][1]=(-(ZRF*S5238[1][2]) + YRF*S5238[1][3])*Si3852[2][1]*T13852[1][4] + (-(ZRF*S5238[2][2]) + YRF*S5238[2][3])*Si3852[2][2]*T13852[2][5] + S5238[1][1]*(Si3852[2][2]*T13852[2][1] + Si3852[2][3]*T13852[3][1]) + S5238[2][1]*(Si3852[2][1]*T13852[1][2] + Si3852[2][3]*T13852[3][2]) + (-(ZRF*S5238[3][2]) + YRF*S5238[3][3])*Si3852[2][3]*T13852[3][6];
T3852[2][2]=(ZRF*S5238[1][1] - XRF*S5238[1][3])*Si3852[2][1]*T13852[1][4] + S5238[3][2]*(Si3852[2][1]*T13852[1][3] + Si3852[2][2]*T13852[2][3]) + (ZRF*S5238[2][1] - XRF*S5238[2][3])*Si3852[2][2]*T13852[2][5] + S5238[1][2]*(Si3852[2][2]*T13852[2][1] + Si3852[2][3]*T13852[3][1]) + S5238[2][2]*(Si3852[2][1]*T13852[1][2] + Si3852[2][3]*T13852[3][2]) - XRF*S5238[3][3]*Si3852[2][3]*T13852[3][6];
T3852[2][3]=(-(YRF*S5238[1][1]) + XRF*S5238[1][2])*Si3852[2][1]*T13852[1][4] + S5238[3][3]*(Si3852[2][1]*T13852[1][3] + Si3852[2][2]*T13852[2][3]) + (-(YRF*S5238[2][1]) + XRF*S5238[2][2])*Si3852[2][2]*T13852[2][5] + S5238[1][3]*(Si3852[2][2]*T13852[2][1] + Si3852[2][3]*T13852[3][1]) + S5238[2][3]*(Si3852[2][1]*T13852[1][2] + Si3852[2][3]*T13852[3][2]) + XRF*S5238[3][2]*Si3852[2][3]*T13852[3][6];
T3852[2][4]=S5238[1][1]*Si3852[2][1]*T13852[1][4] + S5238[2][1]*Si3852[2][2]*T13852[2][5];
T3852[2][5]=S5238[1][2]*Si3852[2][1]*T13852[1][4] + S5238[2][2]*Si3852[2][2]*T13852[2][5] + S5238[3][2]*Si3852[2][3]*T13852[3][6];
T3852[2][6]=S5238[1][3]*Si3852[2][1]*T13852[1][4] + S5238[2][3]*Si3852[2][2]*T13852[2][5] + S5238[3][3]*Si3852[2][3]*T13852[3][6];

T3852[3][1]=(-(ZRF*S5238[1][2]) + YRF*S5238[1][3])*Si3852[3][1]*T13852[1][4] + (-(ZRF*S5238[2][2]) + YRF*S5238[2][3])*Si3852[3][2]*T13852[2][5] + S5238[1][1]*(Si3852[3][2]*T13852[2][1] + Si3852[3][3]*T13852[3][1]) + S5238[2][1]*(Si3852[3][1]*T13852[1][2] + Si3852[3][3]*T13852[3][2]) + (-(ZRF*S5238[3][2]) + YRF*S5238[3][3])*Si3852[3][3]*T13852[3][6];
T3852[3][2]=(ZRF*S5238[1][1] - XRF*S5238[1][3])*Si3852[3][1]*T13852[1][4] + S5238[3][2]*(Si3852[3][1]*T13852[1][3] + Si3852[3][2]*T13852[2][3]) + (ZRF*S5238[2][1] - XRF*S5238[2][3])*Si3852[3][2]*T13852[2][5] + S5238[1][2]*(Si3852[3][2]*T13852[2][1] + Si3852[3][3]*T13852[3][1]) + S5238[2][2]*(Si3852[3][1]*T13852[1][2] + Si3852[3][3]*T13852[3][2]) - XRF*S5238[3][3]*Si3852[3][3]*T13852[3][6];
T3852[3][3]=(-(YRF*S5238[1][1]) + XRF*S5238[1][2])*Si3852[3][1]*T13852[1][4] + S5238[3][3]*(Si3852[3][1]*T13852[1][3] + Si3852[3][2]*T13852[2][3]) + (-(YRF*S5238[2][1]) + XRF*S5238[2][2])*Si3852[3][2]*T13852[2][5] + S5238[1][3]*(Si3852[3][2]*T13852[2][1] + Si3852[3][3]*T13852[3][1]) + S5238[2][3]*(Si3852[3][1]*T13852[1][2] + Si3852[3][3]*T13852[3][2]) + XRF*S5238[3][2]*Si3852[3][3]*T13852[3][6];
T3852[3][4]=S5238[1][1]*Si3852[3][1]*T13852[1][4] + S5238[2][1]*Si3852[3][2]*T13852[2][5];
T3852[3][5]=S5238[1][2]*Si3852[3][1]*T13852[1][4] + S5238[2][2]*Si3852[3][2]*T13852[2][5] + S5238[3][2]*Si3852[3][3]*T13852[3][6];
T3852[3][6]=S5238[1][3]*Si3852[3][1]*T13852[1][4] + S5238[2][3]*Si3852[3][2]*T13852[2][5] + S5238[3][3]*Si3852[3][3]*T13852[3][6];

T3852[4][1]=(-(ZRF*S5238[2][2]) + YRF*S5238[2][3])*((-(ZRF*Si3852[2][2]) + YRF*Si3852[3][2])*T13852[2][5] + Si3852[1][1]*T13852[4][5]) + S5238[1][1]*((-(ZRF*Si3852[2][2]) + YRF*Si3852[3][2])*T13852[2][1] + (-(ZRF*Si3852[2][3]) + YRF*Si3852[3][3])*T13852[3][1] + Si3852[1][1]*T13852[4][1] + Si3852[1][2]*T13852[5][1]) + S5238[2][1]*((-(ZRF*Si3852[2][1]) + YRF*Si3852[3][1])*T13852[1][2] + (-(ZRF*Si3852[2][3]) + YRF*Si3852[3][3])*T13852[3][2] + Si3852[1][1]*T13852[4][2] + Si3852[1][2]*T13852[5][2]) + (-(ZRF*S5238[1][2]) + YRF*S5238[1][3])*((-(ZRF*Si3852[2][1]) + YRF*Si3852[3][1])*T13852[1][4] + Si3852[1][2]*T13852[5][4]) + (-(ZRF*S5238[3][2]) + YRF*S5238[3][3])*((-(ZRF*Si3852[2][3]) + YRF*Si3852[3][3])*T13852[3][6] + Si3852[1][1]*T13852[4][6] + Si3852[1][2]*T13852[5][6]);
T3852[4][2]=(ZRF*S5238[2][1] - XRF*S5238[2][3])*((-(ZRF*Si3852[2][2]) + YRF*Si3852[3][2])*T13852[2][5] + Si3852[1][1]*T13852[4][5]) + S5238[1][2]*((-(ZRF*Si3852[2][2]) + YRF*Si3852[3][2])*T13852[2][1] + (-(ZRF*Si3852[2][3]) + YRF*Si3852[3][3])*T13852[3][1] + Si3852[1][1]*T13852[4][1] + Si3852[1][2]*T13852[5][1]) + S5238[2][2]*((-(ZRF*Si3852[2][1]) + YRF*Si3852[3][1])*T13852[1][2] + (-(ZRF*Si3852[2][3]) + YRF*Si3852[3][3])*T13852[3][2] + Si3852[1][1]*T13852[4][2] + Si3852[1][2]*T13852[5][2]) + S5238[3][2]*((-(ZRF*Si3852[2][1]) + YRF*Si3852[3][1])*T13852[1][3] + (-(ZRF*Si3852[2][2]) + YRF*Si3852[3][2])*T13852[2][3] + Si3852[1][1]*T13852[4][3] + Si3852[1][2]*T13852[5][3]) + (ZRF*S5238[1][1] - XRF*S5238[1][3])*((-(ZRF*Si3852[2][1]) + YRF*Si3852[3][1])*T13852[1][4] + Si3852[1][2]*T13852[5][4]) - XRF*S5238[3][3]*((-(ZRF*Si3852[2][3]) + YRF*Si3852[3][3])*T13852[3][6] + Si3852[1][1]*T13852[4][6] + Si3852[1][2]*T13852[5][6]);
T3852[4][3]=(-(YRF*S5238[2][1]) + XRF*S5238[2][2])*((-(ZRF*Si3852[2][2]) + YRF*Si3852[3][2])*T13852[2][5] + Si3852[1][1]*T13852[4][5]) + S5238[1][3]*((-(ZRF*Si3852[2][2]) + YRF*Si3852[3][2])*T13852[2][1] + (-(ZRF*Si3852[2][3]) + YRF*Si3852[3][3])*T13852[3][1] + Si3852[1][1]*T13852[4][1] + Si3852[1][2]*T13852[5][1]) + S5238[2][3]*((-(ZRF*Si3852[2][1]) + YRF*Si3852[3][1])*T13852[1][2] + (-(ZRF*Si3852[2][3]) + YRF*Si3852[3][3])*T13852[3][2] + Si3852[1][1]*T13852[4][2] + Si3852[1][2]*T13852[5][2]) + S5238[3][3]*((-(ZRF*Si3852[2][1]) + YRF*Si3852[3][1])*T13852[1][3] + (-(ZRF*Si3852[2][2]) + YRF*Si3852[3][2])*T13852[2][3] + Si3852[1][1]*T13852[4][3] + Si3852[1][2]*T13852[5][3]) + (-(YRF*S5238[1][1]) + XRF*S5238[1][2])*((-(ZRF*Si3852[2][1]) + YRF*Si3852[3][1])*T13852[1][4] + Si3852[1][2]*T13852[5][4]) + XRF*S5238[3][2]*((-(ZRF*Si3852[2][3]) + YRF*Si3852[3][3])*T13852[3][6] + Si3852[1][1]*T13852[4][6] + Si3852[1][2]*T13852[5][6]);
T3852[4][4]=S5238[2][1]*((-(ZRF*Si3852[2][2]) + YRF*Si3852[3][2])*T13852[2][5] + Si3852[1][1]*T13852[4][5]) + S5238[1][1]*((-(ZRF*Si3852[2][1]) + YRF*Si3852[3][1])*T13852[1][4] + Si3852[1][2]*T13852[5][4]);
T3852[4][5]=S5238[2][2]*((-(ZRF*Si3852[2][2]) + YRF*Si3852[3][2])*T13852[2][5] + Si3852[1][1]*T13852[4][5]) + S5238[1][2]*((-(ZRF*Si3852[2][1]) + YRF*Si3852[3][1])*T13852[1][4] + Si3852[1][2]*T13852[5][4]) + S5238[3][2]*((-(ZRF*Si3852[2][3]) + YRF*Si3852[3][3])*T13852[3][6] + Si3852[1][1]*T13852[4][6] + Si3852[1][2]*T13852[5][6]);
T3852[4][6]=S5238[2][3]*((-(ZRF*Si3852[2][2]) + YRF*Si3852[3][2])*T13852[2][5] + Si3852[1][1]*T13852[4][5]) + S5238[1][3]*((-(ZRF*Si3852[2][1]) + YRF*Si3852[3][1])*T13852[1][4] + Si3852[1][2]*T13852[5][4]) + S5238[3][3]*((-(ZRF*Si3852[2][3]) + YRF*Si3852[3][3])*T13852[3][6] + Si3852[1][1]*T13852[4][6] + Si3852[1][2]*T13852[5][6]);

T3852[5][1]=(-(ZRF*S5238[3][2]) + YRF*S5238[3][3])*(-(XRF*Si3852[3][3]*T13852[3][6]) + Si3852[2][1]*T13852[4][6] + Si3852[2][2]*T13852[5][6]) + S5238[1][1]*((ZRF*Si3852[1][2] - XRF*Si3852[3][2])*T13852[2][1] - XRF*Si3852[3][3]*T13852[3][1] + Si3852[2][1]*T13852[4][1] + Si3852[2][2]*T13852[5][1] + Si3852[2][3]*T13852[6][1]) + S5238[2][1]*((ZRF*Si3852[1][1] - XRF*Si3852[3][1])*T13852[1][2] - XRF*Si3852[3][3]*T13852[3][2] + Si3852[2][1]*T13852[4][2] + Si3852[2][2]*T13852[5][2] + Si3852[2][3]*T13852[6][2]) + (-(ZRF*S5238[1][2]) + YRF*S5238[1][3])*((ZRF*Si3852[1][1] - XRF*Si3852[3][1])*T13852[1][4] + Si3852[2][2]*T13852[5][4] + Si3852[2][3]*T13852[6][4]) + (-(ZRF*S5238[2][2]) + YRF*S5238[2][3])*((ZRF*Si3852[1][2] - XRF*Si3852[3][2])*T13852[2][5] + Si3852[2][1]*T13852[4][5] + Si3852[2][3]*T13852[6][5]);
T3852[5][2]=-(XRF*S5238[3][3]*(-(XRF*Si3852[3][3]*T13852[3][6]) + Si3852[2][1]*T13852[4][6] + Si3852[2][2]*T13852[5][6])) + S5238[1][2]*((ZRF*Si3852[1][2] - XRF*Si3852[3][2])*T13852[2][1] - XRF*Si3852[3][3]*T13852[3][1] + Si3852[2][1]*T13852[4][1] + Si3852[2][2]*T13852[5][1] + Si3852[2][3]*T13852[6][1]) + S5238[2][2]*((ZRF*Si3852[1][1] - XRF*Si3852[3][1])*T13852[1][2] - XRF*Si3852[3][3]*T13852[3][2] + Si3852[2][1]*T13852[4][2] + Si3852[2][2]*T13852[5][2] + Si3852[2][3]*T13852[6][2]) + S5238[3][2]*((ZRF*Si3852[1][1] - XRF*Si3852[3][1])*T13852[1][3] + (ZRF*Si3852[1][2] - XRF*Si3852[3][2])*T13852[2][3] + Si3852[2][1]*T13852[4][3] + Si3852[2][2]*T13852[5][3] + Si3852[2][3]*T13852[6][3]) + (ZRF*S5238[1][1] - XRF*S5238[1][3])*((ZRF*Si3852[1][1] - XRF*Si3852[3][1])*T13852[1][4] + Si3852[2][2]*T13852[5][4] + Si3852[2][3]*T13852[6][4]) + (ZRF*S5238[2][1] - XRF*S5238[2][3])*((ZRF*Si3852[1][2] - XRF*Si3852[3][2])*T13852[2][5] + Si3852[2][1]*T13852[4][5] + Si3852[2][3]*T13852[6][5]);
T3852[5][3]=XRF*S5238[3][2]*(-(XRF*Si3852[3][3]*T13852[3][6]) + Si3852[2][1]*T13852[4][6] + Si3852[2][2]*T13852[5][6]) + S5238[1][3]*((ZRF*Si3852[1][2] - XRF*Si3852[3][2])*T13852[2][1] - XRF*Si3852[3][3]*T13852[3][1] + Si3852[2][1]*T13852[4][1] + Si3852[2][2]*T13852[5][1] + Si3852[2][3]*T13852[6][1]) + S5238[2][3]*((ZRF*Si3852[1][1] - XRF*Si3852[3][1])*T13852[1][2] - XRF*Si3852[3][3]*T13852[3][2] + Si3852[2][1]*T13852[4][2] + Si3852[2][2]*T13852[5][2] + Si3852[2][3]*T13852[6][2]) + S5238[3][3]*((ZRF*Si3852[1][1] - XRF*Si3852[3][1])*T13852[1][3] + (ZRF*Si3852[1][2] - XRF*Si3852[3][2])*T13852[2][3] + Si3852[2][1]*T13852[4][3] + Si3852[2][2]*T13852[5][3] + Si3852[2][3]*T13852[6][3]) + (-(YRF*S5238[1][1]) + XRF*S5238[1][2])*((ZRF*Si3852[1][1] - XRF*Si3852[3][1])*T13852[1][4] + Si3852[2][2]*T13852[5][4] + Si3852[2][3]*T13852[6][4]) + (-(YRF*S5238[2][1]) + XRF*S5238[2][2])*((ZRF*Si3852[1][2] - XRF*Si3852[3][2])*T13852[2][5] + Si3852[2][1]*T13852[4][5] + Si3852[2][3]*T13852[6][5]);
T3852[5][4]=S5238[1][1]*((ZRF*Si3852[1][1] - XRF*Si3852[3][1])*T13852[1][4] + Si3852[2][2]*T13852[5][4] + Si3852[2][3]*T13852[6][4]) + S5238[2][1]*((ZRF*Si3852[1][2] - XRF*Si3852[3][2])*T13852[2][5] + Si3852[2][1]*T13852[4][5] + Si3852[2][3]*T13852[6][5]);
T3852[5][5]=S5238[3][2]*(-(XRF*Si3852[3][3]*T13852[3][6]) + Si3852[2][1]*T13852[4][6] + Si3852[2][2]*T13852[5][6]) + S5238[1][2]*((ZRF*Si3852[1][1] - XRF*Si3852[3][1])*T13852[1][4] + Si3852[2][2]*T13852[5][4] + Si3852[2][3]*T13852[6][4]) + S5238[2][2]*((ZRF*Si3852[1][2] - XRF*Si3852[3][2])*T13852[2][5] + Si3852[2][1]*T13852[4][5] + Si3852[2][3]*T13852[6][5]);
T3852[5][6]=S5238[3][3]*(-(XRF*Si3852[3][3]*T13852[3][6]) + Si3852[2][1]*T13852[4][6] + Si3852[2][2]*T13852[5][6]) + S5238[1][3]*((ZRF*Si3852[1][1] - XRF*Si3852[3][1])*T13852[1][4] + Si3852[2][2]*T13852[5][4] + Si3852[2][3]*T13852[6][4]) + S5238[2][3]*((ZRF*Si3852[1][2] - XRF*Si3852[3][2])*T13852[2][5] + Si3852[2][1]*T13852[4][5] + Si3852[2][3]*T13852[6][5]);

T3852[6][1]=(-(ZRF*S5238[3][2]) + YRF*S5238[3][3])*(XRF*Si3852[2][3]*T13852[3][6] + Si3852[3][1]*T13852[4][6] + Si3852[3][2]*T13852[5][6]) + S5238[1][1]*((-(YRF*Si3852[1][2]) + XRF*Si3852[2][2])*T13852[2][1] + XRF*Si3852[2][3]*T13852[3][1] + Si3852[3][1]*T13852[4][1] + Si3852[3][2]*T13852[5][1] + Si3852[3][3]*T13852[6][1]) + S5238[2][1]*((-(YRF*Si3852[1][1]) + XRF*Si3852[2][1])*T13852[1][2] + XRF*Si3852[2][3]*T13852[3][2] + Si3852[3][1]*T13852[4][2] + Si3852[3][2]*T13852[5][2] + Si3852[3][3]*T13852[6][2]) + (-(ZRF*S5238[1][2]) + YRF*S5238[1][3])*((-(YRF*Si3852[1][1]) + XRF*Si3852[2][1])*T13852[1][4] + Si3852[3][2]*T13852[5][4] + Si3852[3][3]*T13852[6][4]) + (-(ZRF*S5238[2][2]) + YRF*S5238[2][3])*((-(YRF*Si3852[1][2]) + XRF*Si3852[2][2])*T13852[2][5] + Si3852[3][1]*T13852[4][5] + Si3852[3][3]*T13852[6][5]);
T3852[6][2]=-(XRF*S5238[3][3]*(XRF*Si3852[2][3]*T13852[3][6] + Si3852[3][1]*T13852[4][6] + Si3852[3][2]*T13852[5][6])) + S5238[1][2]*((-(YRF*Si3852[1][2]) + XRF*Si3852[2][2])*T13852[2][1] + XRF*Si3852[2][3]*T13852[3][1] + Si3852[3][1]*T13852[4][1] + Si3852[3][2]*T13852[5][1] + Si3852[3][3]*T13852[6][1]) + S5238[2][2]*((-(YRF*Si3852[1][1]) + XRF*Si3852[2][1])*T13852[1][2] + XRF*Si3852[2][3]*T13852[3][2] + Si3852[3][1]*T13852[4][2] + Si3852[3][2]*T13852[5][2] + Si3852[3][3]*T13852[6][2]) + S5238[3][2]*((-(YRF*Si3852[1][1]) + XRF*Si3852[2][1])*T13852[1][3] + (-(YRF*Si3852[1][2]) + XRF*Si3852[2][2])*T13852[2][3] + Si3852[3][1]*T13852[4][3] + Si3852[3][2]*T13852[5][3] + Si3852[3][3]*T13852[6][3]) + (ZRF*S5238[1][1] - XRF*S5238[1][3])*((-(YRF*Si3852[1][1]) + XRF*Si3852[2][1])*T13852[1][4] + Si3852[3][2]*T13852[5][4] + Si3852[3][3]*T13852[6][4]) + (ZRF*S5238[2][1] - XRF*S5238[2][3])*((-(YRF*Si3852[1][2]) + XRF*Si3852[2][2])*T13852[2][5] + Si3852[3][1]*T13852[4][5] + Si3852[3][3]*T13852[6][5]);
T3852[6][3]=XRF*S5238[3][2]*(XRF*Si3852[2][3]*T13852[3][6] + Si3852[3][1]*T13852[4][6] + Si3852[3][2]*T13852[5][6]) + S5238[1][3]*((-(YRF*Si3852[1][2]) + XRF*Si3852[2][2])*T13852[2][1] + XRF*Si3852[2][3]*T13852[3][1] + Si3852[3][1]*T13852[4][1] + Si3852[3][2]*T13852[5][1] + Si3852[3][3]*T13852[6][1]) + S5238[2][3]*((-(YRF*Si3852[1][1]) + XRF*Si3852[2][1])*T13852[1][2] + XRF*Si3852[2][3]*T13852[3][2] + Si3852[3][1]*T13852[4][2] + Si3852[3][2]*T13852[5][2] + Si3852[3][3]*T13852[6][2]) + S5238[3][3]*((-(YRF*Si3852[1][1]) + XRF*Si3852[2][1])*T13852[1][3] + (-(YRF*Si3852[1][2]) + XRF*Si3852[2][2])*T13852[2][3] + Si3852[3][1]*T13852[4][3] + Si3852[3][2]*T13852[5][3] + Si3852[3][3]*T13852[6][3]) + (-(YRF*S5238[1][1]) + XRF*S5238[1][2])*((-(YRF*Si3852[1][1]) + XRF*Si3852[2][1])*T13852[1][4] + Si3852[3][2]*T13852[5][4] + Si3852[3][3]*T13852[6][4]) + (-(YRF*S5238[2][1]) + XRF*S5238[2][2])*((-(YRF*Si3852[1][2]) + XRF*Si3852[2][2])*T13852[2][5] + Si3852[3][1]*T13852[4][5] + Si3852[3][3]*T13852[6][5]);
T3852[6][4]=S5238[1][1]*((-(YRF*Si3852[1][1]) + XRF*Si3852[2][1])*T13852[1][4] + Si3852[3][2]*T13852[5][4] + Si3852[3][3]*T13852[6][4]) + S5238[2][1]*((-(YRF*Si3852[1][2]) + XRF*Si3852[2][2])*T13852[2][5] + Si3852[3][1]*T13852[4][5] + Si3852[3][3]*T13852[6][5]);
T3852[6][5]=S5238[3][2]*(XRF*Si3852[2][3]*T13852[3][6] + Si3852[3][1]*T13852[4][6] + Si3852[3][2]*T13852[5][6]) + S5238[1][2]*((-(YRF*Si3852[1][1]) + XRF*Si3852[2][1])*T13852[1][4] + Si3852[3][2]*T13852[5][4] + Si3852[3][3]*T13852[6][4]) + S5238[2][2]*((-(YRF*Si3852[1][2]) + XRF*Si3852[2][2])*T13852[2][5] + Si3852[3][1]*T13852[4][5] + Si3852[3][3]*T13852[6][5]);
T3852[6][6]=S5238[3][3]*(XRF*Si3852[2][3]*T13852[3][6] + Si3852[3][1]*T13852[4][6] + Si3852[3][2]*T13852[5][6]) + S5238[1][3]*((-(YRF*Si3852[1][1]) + XRF*Si3852[2][1])*T13852[1][4] + Si3852[3][2]*T13852[5][4] + Si3852[3][3]*T13852[6][4]) + S5238[2][3]*((-(YRF*Si3852[1][2]) + XRF*Si3852[2][2])*T13852[2][5] + Si3852[3][1]*T13852[4][5] + Si3852[3][3]*T13852[6][5]);



}


void
hermes_InvDynArtfunc53(void)
      {




}


void
hermes_InvDynArtfunc54(void)
      {




}


void
hermes_InvDynArtfunc55(void)
      {




}


void
hermes_InvDynArtfunc56(void)
      {
JA48[1][2]=0. + links[48].mcm[3];
JA48[1][3]=0. - links[48].mcm[2];
JA48[1][4]=0. + links[48].m;

JA48[2][1]=0. - links[48].mcm[3];
JA48[2][3]=0. + links[48].mcm[1];
JA48[2][5]=0. + links[48].m;

JA48[3][1]=0. + links[48].mcm[2];
JA48[3][2]=0. - links[48].mcm[1];
JA48[3][6]=0. + links[48].m;

JA48[4][1]=0. + links[48].inertia[1][1];
JA48[4][2]=0. + links[48].inertia[1][2];
JA48[4][3]=0. + links[48].inertia[1][3];
JA48[4][5]=0. - links[48].mcm[3];
JA48[4][6]=0. + links[48].mcm[2];

JA48[5][1]=0. + links[48].inertia[1][2];
JA48[5][2]=0. + links[48].inertia[2][2];
JA48[5][3]=0. + links[48].inertia[2][3];
JA48[5][4]=0. + links[48].mcm[3];
JA48[5][6]=0. - links[48].mcm[1];

JA48[6][1]=0. + links[48].inertia[1][3];
JA48[6][2]=0. + links[48].inertia[2][3];
JA48[6][3]=0. + links[48].inertia[3][3];
JA48[6][4]=0. - links[48].mcm[2];
JA48[6][5]=0. + links[48].mcm[1];


h48[1]=JA48[1][3];
h48[2]=JA48[2][3];
h48[4]=JA48[4][3];
h48[5]=JA48[5][3];
h48[6]=JA48[6][3];

T13848[1][2]=JA48[1][2];
T13848[1][3]=JA48[1][3];
T13848[1][4]=JA48[1][4];

T13848[2][1]=JA48[2][1];
T13848[2][3]=JA48[2][3];
T13848[2][5]=JA48[2][5];

T13848[3][1]=JA48[3][1];
T13848[3][2]=JA48[3][2];
T13848[3][6]=JA48[3][6];

T13848[4][1]=JA48[4][1];
T13848[4][2]=JA48[4][2];
T13848[4][3]=JA48[4][3];
T13848[4][5]=JA48[4][5];
T13848[4][6]=JA48[4][6];

T13848[5][1]=JA48[5][1];
T13848[5][2]=JA48[5][2];
T13848[5][3]=JA48[5][3];
T13848[5][4]=JA48[5][4];
T13848[5][6]=JA48[5][6];

T13848[6][1]=JA48[6][1];
T13848[6][2]=JA48[6][2];
T13848[6][3]=JA48[6][3];
T13848[6][4]=JA48[6][4];
T13848[6][5]=JA48[6][5];


T3848[1][1]=S4838[2][1]*Si3848[1][1]*T13848[1][2] + (-(ZMF*S4838[1][2]) + YMF*S4838[1][3])*Si3848[1][1]*T13848[1][4] + S4838[1][1]*Si3848[1][2]*T13848[2][1] + (-(ZMF*S4838[2][2]) + YMF*S4838[2][3])*Si3848[1][2]*T13848[2][5];
T3848[1][2]=S4838[2][2]*Si3848[1][1]*T13848[1][2] + (ZMF*S4838[1][1] - XMF*S4838[1][3])*Si3848[1][1]*T13848[1][4] + S4838[1][2]*Si3848[1][2]*T13848[2][1] + S4838[3][2]*(Si3848[1][1]*T13848[1][3] + Si3848[1][2]*T13848[2][3]) + (ZMF*S4838[2][1] - XMF*S4838[2][3])*Si3848[1][2]*T13848[2][5];
T3848[1][3]=S4838[2][3]*Si3848[1][1]*T13848[1][2] + (-(YMF*S4838[1][1]) + XMF*S4838[1][2])*Si3848[1][1]*T13848[1][4] + S4838[1][3]*Si3848[1][2]*T13848[2][1] + S4838[3][3]*(Si3848[1][1]*T13848[1][3] + Si3848[1][2]*T13848[2][3]) + (-(YMF*S4838[2][1]) + XMF*S4838[2][2])*Si3848[1][2]*T13848[2][5];
T3848[1][4]=S4838[1][1]*Si3848[1][1]*T13848[1][4] + S4838[2][1]*Si3848[1][2]*T13848[2][5];
T3848[1][5]=S4838[1][2]*Si3848[1][1]*T13848[1][4] + S4838[2][2]*Si3848[1][2]*T13848[2][5];
T3848[1][6]=S4838[1][3]*Si3848[1][1]*T13848[1][4] + S4838[2][3]*Si3848[1][2]*T13848[2][5];

T3848[2][1]=(-(ZMF*S4838[1][2]) + YMF*S4838[1][3])*Si3848[2][1]*T13848[1][4] + (-(ZMF*S4838[2][2]) + YMF*S4838[2][3])*Si3848[2][2]*T13848[2][5] + S4838[1][1]*(Si3848[2][2]*T13848[2][1] + Si3848[2][3]*T13848[3][1]) + S4838[2][1]*(Si3848[2][1]*T13848[1][2] + Si3848[2][3]*T13848[3][2]) + (-(ZMF*S4838[3][2]) + YMF*S4838[3][3])*Si3848[2][3]*T13848[3][6];
T3848[2][2]=(ZMF*S4838[1][1] - XMF*S4838[1][3])*Si3848[2][1]*T13848[1][4] + S4838[3][2]*(Si3848[2][1]*T13848[1][3] + Si3848[2][2]*T13848[2][3]) + (ZMF*S4838[2][1] - XMF*S4838[2][3])*Si3848[2][2]*T13848[2][5] + S4838[1][2]*(Si3848[2][2]*T13848[2][1] + Si3848[2][3]*T13848[3][1]) + S4838[2][2]*(Si3848[2][1]*T13848[1][2] + Si3848[2][3]*T13848[3][2]) - XMF*S4838[3][3]*Si3848[2][3]*T13848[3][6];
T3848[2][3]=(-(YMF*S4838[1][1]) + XMF*S4838[1][2])*Si3848[2][1]*T13848[1][4] + S4838[3][3]*(Si3848[2][1]*T13848[1][3] + Si3848[2][2]*T13848[2][3]) + (-(YMF*S4838[2][1]) + XMF*S4838[2][2])*Si3848[2][2]*T13848[2][5] + S4838[1][3]*(Si3848[2][2]*T13848[2][1] + Si3848[2][3]*T13848[3][1]) + S4838[2][3]*(Si3848[2][1]*T13848[1][2] + Si3848[2][3]*T13848[3][2]) + XMF*S4838[3][2]*Si3848[2][3]*T13848[3][6];
T3848[2][4]=S4838[1][1]*Si3848[2][1]*T13848[1][4] + S4838[2][1]*Si3848[2][2]*T13848[2][5];
T3848[2][5]=S4838[1][2]*Si3848[2][1]*T13848[1][4] + S4838[2][2]*Si3848[2][2]*T13848[2][5] + S4838[3][2]*Si3848[2][3]*T13848[3][6];
T3848[2][6]=S4838[1][3]*Si3848[2][1]*T13848[1][4] + S4838[2][3]*Si3848[2][2]*T13848[2][5] + S4838[3][3]*Si3848[2][3]*T13848[3][6];

T3848[3][1]=(-(ZMF*S4838[1][2]) + YMF*S4838[1][3])*Si3848[3][1]*T13848[1][4] + (-(ZMF*S4838[2][2]) + YMF*S4838[2][3])*Si3848[3][2]*T13848[2][5] + S4838[1][1]*(Si3848[3][2]*T13848[2][1] + Si3848[3][3]*T13848[3][1]) + S4838[2][1]*(Si3848[3][1]*T13848[1][2] + Si3848[3][3]*T13848[3][2]) + (-(ZMF*S4838[3][2]) + YMF*S4838[3][3])*Si3848[3][3]*T13848[3][6];
T3848[3][2]=(ZMF*S4838[1][1] - XMF*S4838[1][3])*Si3848[3][1]*T13848[1][4] + S4838[3][2]*(Si3848[3][1]*T13848[1][3] + Si3848[3][2]*T13848[2][3]) + (ZMF*S4838[2][1] - XMF*S4838[2][3])*Si3848[3][2]*T13848[2][5] + S4838[1][2]*(Si3848[3][2]*T13848[2][1] + Si3848[3][3]*T13848[3][1]) + S4838[2][2]*(Si3848[3][1]*T13848[1][2] + Si3848[3][3]*T13848[3][2]) - XMF*S4838[3][3]*Si3848[3][3]*T13848[3][6];
T3848[3][3]=(-(YMF*S4838[1][1]) + XMF*S4838[1][2])*Si3848[3][1]*T13848[1][4] + S4838[3][3]*(Si3848[3][1]*T13848[1][3] + Si3848[3][2]*T13848[2][3]) + (-(YMF*S4838[2][1]) + XMF*S4838[2][2])*Si3848[3][2]*T13848[2][5] + S4838[1][3]*(Si3848[3][2]*T13848[2][1] + Si3848[3][3]*T13848[3][1]) + S4838[2][3]*(Si3848[3][1]*T13848[1][2] + Si3848[3][3]*T13848[3][2]) + XMF*S4838[3][2]*Si3848[3][3]*T13848[3][6];
T3848[3][4]=S4838[1][1]*Si3848[3][1]*T13848[1][4] + S4838[2][1]*Si3848[3][2]*T13848[2][5];
T3848[3][5]=S4838[1][2]*Si3848[3][1]*T13848[1][4] + S4838[2][2]*Si3848[3][2]*T13848[2][5] + S4838[3][2]*Si3848[3][3]*T13848[3][6];
T3848[3][6]=S4838[1][3]*Si3848[3][1]*T13848[1][4] + S4838[2][3]*Si3848[3][2]*T13848[2][5] + S4838[3][3]*Si3848[3][3]*T13848[3][6];

T3848[4][1]=(-(ZMF*S4838[2][2]) + YMF*S4838[2][3])*((-(ZMF*Si3848[2][2]) + YMF*Si3848[3][2])*T13848[2][5] + Si3848[1][1]*T13848[4][5]) + S4838[1][1]*((-(ZMF*Si3848[2][2]) + YMF*Si3848[3][2])*T13848[2][1] + (-(ZMF*Si3848[2][3]) + YMF*Si3848[3][3])*T13848[3][1] + Si3848[1][1]*T13848[4][1] + Si3848[1][2]*T13848[5][1]) + S4838[2][1]*((-(ZMF*Si3848[2][1]) + YMF*Si3848[3][1])*T13848[1][2] + (-(ZMF*Si3848[2][3]) + YMF*Si3848[3][3])*T13848[3][2] + Si3848[1][1]*T13848[4][2] + Si3848[1][2]*T13848[5][2]) + (-(ZMF*S4838[1][2]) + YMF*S4838[1][3])*((-(ZMF*Si3848[2][1]) + YMF*Si3848[3][1])*T13848[1][4] + Si3848[1][2]*T13848[5][4]) + (-(ZMF*S4838[3][2]) + YMF*S4838[3][3])*((-(ZMF*Si3848[2][3]) + YMF*Si3848[3][3])*T13848[3][6] + Si3848[1][1]*T13848[4][6] + Si3848[1][2]*T13848[5][6]);
T3848[4][2]=(ZMF*S4838[2][1] - XMF*S4838[2][3])*((-(ZMF*Si3848[2][2]) + YMF*Si3848[3][2])*T13848[2][5] + Si3848[1][1]*T13848[4][5]) + S4838[1][2]*((-(ZMF*Si3848[2][2]) + YMF*Si3848[3][2])*T13848[2][1] + (-(ZMF*Si3848[2][3]) + YMF*Si3848[3][3])*T13848[3][1] + Si3848[1][1]*T13848[4][1] + Si3848[1][2]*T13848[5][1]) + S4838[2][2]*((-(ZMF*Si3848[2][1]) + YMF*Si3848[3][1])*T13848[1][2] + (-(ZMF*Si3848[2][3]) + YMF*Si3848[3][3])*T13848[3][2] + Si3848[1][1]*T13848[4][2] + Si3848[1][2]*T13848[5][2]) + S4838[3][2]*((-(ZMF*Si3848[2][1]) + YMF*Si3848[3][1])*T13848[1][3] + (-(ZMF*Si3848[2][2]) + YMF*Si3848[3][2])*T13848[2][3] + Si3848[1][1]*T13848[4][3] + Si3848[1][2]*T13848[5][3]) + (ZMF*S4838[1][1] - XMF*S4838[1][3])*((-(ZMF*Si3848[2][1]) + YMF*Si3848[3][1])*T13848[1][4] + Si3848[1][2]*T13848[5][4]) - XMF*S4838[3][3]*((-(ZMF*Si3848[2][3]) + YMF*Si3848[3][3])*T13848[3][6] + Si3848[1][1]*T13848[4][6] + Si3848[1][2]*T13848[5][6]);
T3848[4][3]=(-(YMF*S4838[2][1]) + XMF*S4838[2][2])*((-(ZMF*Si3848[2][2]) + YMF*Si3848[3][2])*T13848[2][5] + Si3848[1][1]*T13848[4][5]) + S4838[1][3]*((-(ZMF*Si3848[2][2]) + YMF*Si3848[3][2])*T13848[2][1] + (-(ZMF*Si3848[2][3]) + YMF*Si3848[3][3])*T13848[3][1] + Si3848[1][1]*T13848[4][1] + Si3848[1][2]*T13848[5][1]) + S4838[2][3]*((-(ZMF*Si3848[2][1]) + YMF*Si3848[3][1])*T13848[1][2] + (-(ZMF*Si3848[2][3]) + YMF*Si3848[3][3])*T13848[3][2] + Si3848[1][1]*T13848[4][2] + Si3848[1][2]*T13848[5][2]) + S4838[3][3]*((-(ZMF*Si3848[2][1]) + YMF*Si3848[3][1])*T13848[1][3] + (-(ZMF*Si3848[2][2]) + YMF*Si3848[3][2])*T13848[2][3] + Si3848[1][1]*T13848[4][3] + Si3848[1][2]*T13848[5][3]) + (-(YMF*S4838[1][1]) + XMF*S4838[1][2])*((-(ZMF*Si3848[2][1]) + YMF*Si3848[3][1])*T13848[1][4] + Si3848[1][2]*T13848[5][4]) + XMF*S4838[3][2]*((-(ZMF*Si3848[2][3]) + YMF*Si3848[3][3])*T13848[3][6] + Si3848[1][1]*T13848[4][6] + Si3848[1][2]*T13848[5][6]);
T3848[4][4]=S4838[2][1]*((-(ZMF*Si3848[2][2]) + YMF*Si3848[3][2])*T13848[2][5] + Si3848[1][1]*T13848[4][5]) + S4838[1][1]*((-(ZMF*Si3848[2][1]) + YMF*Si3848[3][1])*T13848[1][4] + Si3848[1][2]*T13848[5][4]);
T3848[4][5]=S4838[2][2]*((-(ZMF*Si3848[2][2]) + YMF*Si3848[3][2])*T13848[2][5] + Si3848[1][1]*T13848[4][5]) + S4838[1][2]*((-(ZMF*Si3848[2][1]) + YMF*Si3848[3][1])*T13848[1][4] + Si3848[1][2]*T13848[5][4]) + S4838[3][2]*((-(ZMF*Si3848[2][3]) + YMF*Si3848[3][3])*T13848[3][6] + Si3848[1][1]*T13848[4][6] + Si3848[1][2]*T13848[5][6]);
T3848[4][6]=S4838[2][3]*((-(ZMF*Si3848[2][2]) + YMF*Si3848[3][2])*T13848[2][5] + Si3848[1][1]*T13848[4][5]) + S4838[1][3]*((-(ZMF*Si3848[2][1]) + YMF*Si3848[3][1])*T13848[1][4] + Si3848[1][2]*T13848[5][4]) + S4838[3][3]*((-(ZMF*Si3848[2][3]) + YMF*Si3848[3][3])*T13848[3][6] + Si3848[1][1]*T13848[4][6] + Si3848[1][2]*T13848[5][6]);

T3848[5][1]=(-(ZMF*S4838[3][2]) + YMF*S4838[3][3])*(-(XMF*Si3848[3][3]*T13848[3][6]) + Si3848[2][1]*T13848[4][6] + Si3848[2][2]*T13848[5][6]) + S4838[1][1]*((ZMF*Si3848[1][2] - XMF*Si3848[3][2])*T13848[2][1] - XMF*Si3848[3][3]*T13848[3][1] + Si3848[2][1]*T13848[4][1] + Si3848[2][2]*T13848[5][1] + Si3848[2][3]*T13848[6][1]) + S4838[2][1]*((ZMF*Si3848[1][1] - XMF*Si3848[3][1])*T13848[1][2] - XMF*Si3848[3][3]*T13848[3][2] + Si3848[2][1]*T13848[4][2] + Si3848[2][2]*T13848[5][2] + Si3848[2][3]*T13848[6][2]) + (-(ZMF*S4838[1][2]) + YMF*S4838[1][3])*((ZMF*Si3848[1][1] - XMF*Si3848[3][1])*T13848[1][4] + Si3848[2][2]*T13848[5][4] + Si3848[2][3]*T13848[6][4]) + (-(ZMF*S4838[2][2]) + YMF*S4838[2][3])*((ZMF*Si3848[1][2] - XMF*Si3848[3][2])*T13848[2][5] + Si3848[2][1]*T13848[4][5] + Si3848[2][3]*T13848[6][5]);
T3848[5][2]=-(XMF*S4838[3][3]*(-(XMF*Si3848[3][3]*T13848[3][6]) + Si3848[2][1]*T13848[4][6] + Si3848[2][2]*T13848[5][6])) + S4838[1][2]*((ZMF*Si3848[1][2] - XMF*Si3848[3][2])*T13848[2][1] - XMF*Si3848[3][3]*T13848[3][1] + Si3848[2][1]*T13848[4][1] + Si3848[2][2]*T13848[5][1] + Si3848[2][3]*T13848[6][1]) + S4838[2][2]*((ZMF*Si3848[1][1] - XMF*Si3848[3][1])*T13848[1][2] - XMF*Si3848[3][3]*T13848[3][2] + Si3848[2][1]*T13848[4][2] + Si3848[2][2]*T13848[5][2] + Si3848[2][3]*T13848[6][2]) + S4838[3][2]*((ZMF*Si3848[1][1] - XMF*Si3848[3][1])*T13848[1][3] + (ZMF*Si3848[1][2] - XMF*Si3848[3][2])*T13848[2][3] + Si3848[2][1]*T13848[4][3] + Si3848[2][2]*T13848[5][3] + Si3848[2][3]*T13848[6][3]) + (ZMF*S4838[1][1] - XMF*S4838[1][3])*((ZMF*Si3848[1][1] - XMF*Si3848[3][1])*T13848[1][4] + Si3848[2][2]*T13848[5][4] + Si3848[2][3]*T13848[6][4]) + (ZMF*S4838[2][1] - XMF*S4838[2][3])*((ZMF*Si3848[1][2] - XMF*Si3848[3][2])*T13848[2][5] + Si3848[2][1]*T13848[4][5] + Si3848[2][3]*T13848[6][5]);
T3848[5][3]=XMF*S4838[3][2]*(-(XMF*Si3848[3][3]*T13848[3][6]) + Si3848[2][1]*T13848[4][6] + Si3848[2][2]*T13848[5][6]) + S4838[1][3]*((ZMF*Si3848[1][2] - XMF*Si3848[3][2])*T13848[2][1] - XMF*Si3848[3][3]*T13848[3][1] + Si3848[2][1]*T13848[4][1] + Si3848[2][2]*T13848[5][1] + Si3848[2][3]*T13848[6][1]) + S4838[2][3]*((ZMF*Si3848[1][1] - XMF*Si3848[3][1])*T13848[1][2] - XMF*Si3848[3][3]*T13848[3][2] + Si3848[2][1]*T13848[4][2] + Si3848[2][2]*T13848[5][2] + Si3848[2][3]*T13848[6][2]) + S4838[3][3]*((ZMF*Si3848[1][1] - XMF*Si3848[3][1])*T13848[1][3] + (ZMF*Si3848[1][2] - XMF*Si3848[3][2])*T13848[2][3] + Si3848[2][1]*T13848[4][3] + Si3848[2][2]*T13848[5][3] + Si3848[2][3]*T13848[6][3]) + (-(YMF*S4838[1][1]) + XMF*S4838[1][2])*((ZMF*Si3848[1][1] - XMF*Si3848[3][1])*T13848[1][4] + Si3848[2][2]*T13848[5][4] + Si3848[2][3]*T13848[6][4]) + (-(YMF*S4838[2][1]) + XMF*S4838[2][2])*((ZMF*Si3848[1][2] - XMF*Si3848[3][2])*T13848[2][5] + Si3848[2][1]*T13848[4][5] + Si3848[2][3]*T13848[6][5]);
T3848[5][4]=S4838[1][1]*((ZMF*Si3848[1][1] - XMF*Si3848[3][1])*T13848[1][4] + Si3848[2][2]*T13848[5][4] + Si3848[2][3]*T13848[6][4]) + S4838[2][1]*((ZMF*Si3848[1][2] - XMF*Si3848[3][2])*T13848[2][5] + Si3848[2][1]*T13848[4][5] + Si3848[2][3]*T13848[6][5]);
T3848[5][5]=S4838[3][2]*(-(XMF*Si3848[3][3]*T13848[3][6]) + Si3848[2][1]*T13848[4][6] + Si3848[2][2]*T13848[5][6]) + S4838[1][2]*((ZMF*Si3848[1][1] - XMF*Si3848[3][1])*T13848[1][4] + Si3848[2][2]*T13848[5][4] + Si3848[2][3]*T13848[6][4]) + S4838[2][2]*((ZMF*Si3848[1][2] - XMF*Si3848[3][2])*T13848[2][5] + Si3848[2][1]*T13848[4][5] + Si3848[2][3]*T13848[6][5]);
T3848[5][6]=S4838[3][3]*(-(XMF*Si3848[3][3]*T13848[3][6]) + Si3848[2][1]*T13848[4][6] + Si3848[2][2]*T13848[5][6]) + S4838[1][3]*((ZMF*Si3848[1][1] - XMF*Si3848[3][1])*T13848[1][4] + Si3848[2][2]*T13848[5][4] + Si3848[2][3]*T13848[6][4]) + S4838[2][3]*((ZMF*Si3848[1][2] - XMF*Si3848[3][2])*T13848[2][5] + Si3848[2][1]*T13848[4][5] + Si3848[2][3]*T13848[6][5]);

T3848[6][1]=(-(ZMF*S4838[3][2]) + YMF*S4838[3][3])*(XMF*Si3848[2][3]*T13848[3][6] + Si3848[3][1]*T13848[4][6] + Si3848[3][2]*T13848[5][6]) + S4838[1][1]*((-(YMF*Si3848[1][2]) + XMF*Si3848[2][2])*T13848[2][1] + XMF*Si3848[2][3]*T13848[3][1] + Si3848[3][1]*T13848[4][1] + Si3848[3][2]*T13848[5][1] + Si3848[3][3]*T13848[6][1]) + S4838[2][1]*((-(YMF*Si3848[1][1]) + XMF*Si3848[2][1])*T13848[1][2] + XMF*Si3848[2][3]*T13848[3][2] + Si3848[3][1]*T13848[4][2] + Si3848[3][2]*T13848[5][2] + Si3848[3][3]*T13848[6][2]) + (-(ZMF*S4838[1][2]) + YMF*S4838[1][3])*((-(YMF*Si3848[1][1]) + XMF*Si3848[2][1])*T13848[1][4] + Si3848[3][2]*T13848[5][4] + Si3848[3][3]*T13848[6][4]) + (-(ZMF*S4838[2][2]) + YMF*S4838[2][3])*((-(YMF*Si3848[1][2]) + XMF*Si3848[2][2])*T13848[2][5] + Si3848[3][1]*T13848[4][5] + Si3848[3][3]*T13848[6][5]);
T3848[6][2]=-(XMF*S4838[3][3]*(XMF*Si3848[2][3]*T13848[3][6] + Si3848[3][1]*T13848[4][6] + Si3848[3][2]*T13848[5][6])) + S4838[1][2]*((-(YMF*Si3848[1][2]) + XMF*Si3848[2][2])*T13848[2][1] + XMF*Si3848[2][3]*T13848[3][1] + Si3848[3][1]*T13848[4][1] + Si3848[3][2]*T13848[5][1] + Si3848[3][3]*T13848[6][1]) + S4838[2][2]*((-(YMF*Si3848[1][1]) + XMF*Si3848[2][1])*T13848[1][2] + XMF*Si3848[2][3]*T13848[3][2] + Si3848[3][1]*T13848[4][2] + Si3848[3][2]*T13848[5][2] + Si3848[3][3]*T13848[6][2]) + S4838[3][2]*((-(YMF*Si3848[1][1]) + XMF*Si3848[2][1])*T13848[1][3] + (-(YMF*Si3848[1][2]) + XMF*Si3848[2][2])*T13848[2][3] + Si3848[3][1]*T13848[4][3] + Si3848[3][2]*T13848[5][3] + Si3848[3][3]*T13848[6][3]) + (ZMF*S4838[1][1] - XMF*S4838[1][3])*((-(YMF*Si3848[1][1]) + XMF*Si3848[2][1])*T13848[1][4] + Si3848[3][2]*T13848[5][4] + Si3848[3][3]*T13848[6][4]) + (ZMF*S4838[2][1] - XMF*S4838[2][3])*((-(YMF*Si3848[1][2]) + XMF*Si3848[2][2])*T13848[2][5] + Si3848[3][1]*T13848[4][5] + Si3848[3][3]*T13848[6][5]);
T3848[6][3]=XMF*S4838[3][2]*(XMF*Si3848[2][3]*T13848[3][6] + Si3848[3][1]*T13848[4][6] + Si3848[3][2]*T13848[5][6]) + S4838[1][3]*((-(YMF*Si3848[1][2]) + XMF*Si3848[2][2])*T13848[2][1] + XMF*Si3848[2][3]*T13848[3][1] + Si3848[3][1]*T13848[4][1] + Si3848[3][2]*T13848[5][1] + Si3848[3][3]*T13848[6][1]) + S4838[2][3]*((-(YMF*Si3848[1][1]) + XMF*Si3848[2][1])*T13848[1][2] + XMF*Si3848[2][3]*T13848[3][2] + Si3848[3][1]*T13848[4][2] + Si3848[3][2]*T13848[5][2] + Si3848[3][3]*T13848[6][2]) + S4838[3][3]*((-(YMF*Si3848[1][1]) + XMF*Si3848[2][1])*T13848[1][3] + (-(YMF*Si3848[1][2]) + XMF*Si3848[2][2])*T13848[2][3] + Si3848[3][1]*T13848[4][3] + Si3848[3][2]*T13848[5][3] + Si3848[3][3]*T13848[6][3]) + (-(YMF*S4838[1][1]) + XMF*S4838[1][2])*((-(YMF*Si3848[1][1]) + XMF*Si3848[2][1])*T13848[1][4] + Si3848[3][2]*T13848[5][4] + Si3848[3][3]*T13848[6][4]) + (-(YMF*S4838[2][1]) + XMF*S4838[2][2])*((-(YMF*Si3848[1][2]) + XMF*Si3848[2][2])*T13848[2][5] + Si3848[3][1]*T13848[4][5] + Si3848[3][3]*T13848[6][5]);
T3848[6][4]=S4838[1][1]*((-(YMF*Si3848[1][1]) + XMF*Si3848[2][1])*T13848[1][4] + Si3848[3][2]*T13848[5][4] + Si3848[3][3]*T13848[6][4]) + S4838[2][1]*((-(YMF*Si3848[1][2]) + XMF*Si3848[2][2])*T13848[2][5] + Si3848[3][1]*T13848[4][5] + Si3848[3][3]*T13848[6][5]);
T3848[6][5]=S4838[3][2]*(XMF*Si3848[2][3]*T13848[3][6] + Si3848[3][1]*T13848[4][6] + Si3848[3][2]*T13848[5][6]) + S4838[1][2]*((-(YMF*Si3848[1][1]) + XMF*Si3848[2][1])*T13848[1][4] + Si3848[3][2]*T13848[5][4] + Si3848[3][3]*T13848[6][4]) + S4838[2][2]*((-(YMF*Si3848[1][2]) + XMF*Si3848[2][2])*T13848[2][5] + Si3848[3][1]*T13848[4][5] + Si3848[3][3]*T13848[6][5]);
T3848[6][6]=S4838[3][3]*(XMF*Si3848[2][3]*T13848[3][6] + Si3848[3][1]*T13848[4][6] + Si3848[3][2]*T13848[5][6]) + S4838[1][3]*((-(YMF*Si3848[1][1]) + XMF*Si3848[2][1])*T13848[1][4] + Si3848[3][2]*T13848[5][4] + Si3848[3][3]*T13848[6][4]) + S4838[2][3]*((-(YMF*Si3848[1][2]) + XMF*Si3848[2][2])*T13848[2][5] + Si3848[3][1]*T13848[4][5] + Si3848[3][3]*T13848[6][5]);



}


void
hermes_InvDynArtfunc57(void)
      {




}


void
hermes_InvDynArtfunc58(void)
      {




}


void
hermes_InvDynArtfunc59(void)
      {




}


void
hermes_InvDynArtfunc60(void)
      {
JA44[1][2]=0. + links[47].mcm[3];
JA44[1][3]=0. - links[47].mcm[2];
JA44[1][4]=0. + links[47].m;

JA44[2][1]=0. - links[47].mcm[3];
JA44[2][3]=0. + links[47].mcm[1];
JA44[2][5]=0. + links[47].m;

JA44[3][1]=0. + links[47].mcm[2];
JA44[3][2]=0. - links[47].mcm[1];
JA44[3][6]=0. + links[47].m;

JA44[4][1]=0. + links[47].inertia[1][1];
JA44[4][2]=0. + links[47].inertia[1][2];
JA44[4][3]=0. + links[47].inertia[1][3];
JA44[4][5]=0. - links[47].mcm[3];
JA44[4][6]=0. + links[47].mcm[2];

JA44[5][1]=0. + links[47].inertia[1][2];
JA44[5][2]=0. + links[47].inertia[2][2];
JA44[5][3]=0. + links[47].inertia[2][3];
JA44[5][4]=0. + links[47].mcm[3];
JA44[5][6]=0. - links[47].mcm[1];

JA44[6][1]=0. + links[47].inertia[1][3];
JA44[6][2]=0. + links[47].inertia[2][3];
JA44[6][3]=0. + links[47].inertia[3][3];
JA44[6][4]=0. - links[47].mcm[2];
JA44[6][5]=0. + links[47].mcm[1];


h44[1]=JA44[1][3];
h44[2]=JA44[2][3];
h44[4]=JA44[4][3];
h44[5]=JA44[5][3];
h44[6]=JA44[6][3];

T13844[1][2]=JA44[1][2];
T13844[1][3]=JA44[1][3];
T13844[1][4]=JA44[1][4];

T13844[2][1]=JA44[2][1];
T13844[2][3]=JA44[2][3];
T13844[2][5]=JA44[2][5];

T13844[3][1]=JA44[3][1];
T13844[3][2]=JA44[3][2];
T13844[3][6]=JA44[3][6];

T13844[4][1]=JA44[4][1];
T13844[4][2]=JA44[4][2];
T13844[4][3]=JA44[4][3];
T13844[4][5]=JA44[4][5];
T13844[4][6]=JA44[4][6];

T13844[5][1]=JA44[5][1];
T13844[5][2]=JA44[5][2];
T13844[5][3]=JA44[5][3];
T13844[5][4]=JA44[5][4];
T13844[5][6]=JA44[5][6];

T13844[6][1]=JA44[6][1];
T13844[6][2]=JA44[6][2];
T13844[6][3]=JA44[6][3];
T13844[6][4]=JA44[6][4];
T13844[6][5]=JA44[6][5];


T3844[1][1]=S4438[2][1]*Si3844[1][1]*T13844[1][2] + (-(ZIF*S4438[1][2]) + YIF*S4438[1][3])*Si3844[1][1]*T13844[1][4] + S4438[1][1]*Si3844[1][2]*T13844[2][1] + (-(ZIF*S4438[2][2]) + YIF*S4438[2][3])*Si3844[1][2]*T13844[2][5];
T3844[1][2]=S4438[2][2]*Si3844[1][1]*T13844[1][2] + (ZIF*S4438[1][1] - XIF*S4438[1][3])*Si3844[1][1]*T13844[1][4] + S4438[1][2]*Si3844[1][2]*T13844[2][1] + S4438[3][2]*(Si3844[1][1]*T13844[1][3] + Si3844[1][2]*T13844[2][3]) + (ZIF*S4438[2][1] - XIF*S4438[2][3])*Si3844[1][2]*T13844[2][5];
T3844[1][3]=S4438[2][3]*Si3844[1][1]*T13844[1][2] + (-(YIF*S4438[1][1]) + XIF*S4438[1][2])*Si3844[1][1]*T13844[1][4] + S4438[1][3]*Si3844[1][2]*T13844[2][1] + S4438[3][3]*(Si3844[1][1]*T13844[1][3] + Si3844[1][2]*T13844[2][3]) + (-(YIF*S4438[2][1]) + XIF*S4438[2][2])*Si3844[1][2]*T13844[2][5];
T3844[1][4]=S4438[1][1]*Si3844[1][1]*T13844[1][4] + S4438[2][1]*Si3844[1][2]*T13844[2][5];
T3844[1][5]=S4438[1][2]*Si3844[1][1]*T13844[1][4] + S4438[2][2]*Si3844[1][2]*T13844[2][5];
T3844[1][6]=S4438[1][3]*Si3844[1][1]*T13844[1][4] + S4438[2][3]*Si3844[1][2]*T13844[2][5];

T3844[2][1]=(-(ZIF*S4438[1][2]) + YIF*S4438[1][3])*Si3844[2][1]*T13844[1][4] + (-(ZIF*S4438[2][2]) + YIF*S4438[2][3])*Si3844[2][2]*T13844[2][5] + S4438[1][1]*(Si3844[2][2]*T13844[2][1] + Si3844[2][3]*T13844[3][1]) + S4438[2][1]*(Si3844[2][1]*T13844[1][2] + Si3844[2][3]*T13844[3][2]) + (-(ZIF*S4438[3][2]) + YIF*S4438[3][3])*Si3844[2][3]*T13844[3][6];
T3844[2][2]=(ZIF*S4438[1][1] - XIF*S4438[1][3])*Si3844[2][1]*T13844[1][4] + S4438[3][2]*(Si3844[2][1]*T13844[1][3] + Si3844[2][2]*T13844[2][3]) + (ZIF*S4438[2][1] - XIF*S4438[2][3])*Si3844[2][2]*T13844[2][5] + S4438[1][2]*(Si3844[2][2]*T13844[2][1] + Si3844[2][3]*T13844[3][1]) + S4438[2][2]*(Si3844[2][1]*T13844[1][2] + Si3844[2][3]*T13844[3][2]) - XIF*S4438[3][3]*Si3844[2][3]*T13844[3][6];
T3844[2][3]=(-(YIF*S4438[1][1]) + XIF*S4438[1][2])*Si3844[2][1]*T13844[1][4] + S4438[3][3]*(Si3844[2][1]*T13844[1][3] + Si3844[2][2]*T13844[2][3]) + (-(YIF*S4438[2][1]) + XIF*S4438[2][2])*Si3844[2][2]*T13844[2][5] + S4438[1][3]*(Si3844[2][2]*T13844[2][1] + Si3844[2][3]*T13844[3][1]) + S4438[2][3]*(Si3844[2][1]*T13844[1][2] + Si3844[2][3]*T13844[3][2]) + XIF*S4438[3][2]*Si3844[2][3]*T13844[3][6];
T3844[2][4]=S4438[1][1]*Si3844[2][1]*T13844[1][4] + S4438[2][1]*Si3844[2][2]*T13844[2][5];
T3844[2][5]=S4438[1][2]*Si3844[2][1]*T13844[1][4] + S4438[2][2]*Si3844[2][2]*T13844[2][5] + S4438[3][2]*Si3844[2][3]*T13844[3][6];
T3844[2][6]=S4438[1][3]*Si3844[2][1]*T13844[1][4] + S4438[2][3]*Si3844[2][2]*T13844[2][5] + S4438[3][3]*Si3844[2][3]*T13844[3][6];

T3844[3][1]=(-(ZIF*S4438[1][2]) + YIF*S4438[1][3])*Si3844[3][1]*T13844[1][4] + (-(ZIF*S4438[2][2]) + YIF*S4438[2][3])*Si3844[3][2]*T13844[2][5] + S4438[1][1]*(Si3844[3][2]*T13844[2][1] + Si3844[3][3]*T13844[3][1]) + S4438[2][1]*(Si3844[3][1]*T13844[1][2] + Si3844[3][3]*T13844[3][2]) + (-(ZIF*S4438[3][2]) + YIF*S4438[3][3])*Si3844[3][3]*T13844[3][6];
T3844[3][2]=(ZIF*S4438[1][1] - XIF*S4438[1][3])*Si3844[3][1]*T13844[1][4] + S4438[3][2]*(Si3844[3][1]*T13844[1][3] + Si3844[3][2]*T13844[2][3]) + (ZIF*S4438[2][1] - XIF*S4438[2][3])*Si3844[3][2]*T13844[2][5] + S4438[1][2]*(Si3844[3][2]*T13844[2][1] + Si3844[3][3]*T13844[3][1]) + S4438[2][2]*(Si3844[3][1]*T13844[1][2] + Si3844[3][3]*T13844[3][2]) - XIF*S4438[3][3]*Si3844[3][3]*T13844[3][6];
T3844[3][3]=(-(YIF*S4438[1][1]) + XIF*S4438[1][2])*Si3844[3][1]*T13844[1][4] + S4438[3][3]*(Si3844[3][1]*T13844[1][3] + Si3844[3][2]*T13844[2][3]) + (-(YIF*S4438[2][1]) + XIF*S4438[2][2])*Si3844[3][2]*T13844[2][5] + S4438[1][3]*(Si3844[3][2]*T13844[2][1] + Si3844[3][3]*T13844[3][1]) + S4438[2][3]*(Si3844[3][1]*T13844[1][2] + Si3844[3][3]*T13844[3][2]) + XIF*S4438[3][2]*Si3844[3][3]*T13844[3][6];
T3844[3][4]=S4438[1][1]*Si3844[3][1]*T13844[1][4] + S4438[2][1]*Si3844[3][2]*T13844[2][5];
T3844[3][5]=S4438[1][2]*Si3844[3][1]*T13844[1][4] + S4438[2][2]*Si3844[3][2]*T13844[2][5] + S4438[3][2]*Si3844[3][3]*T13844[3][6];
T3844[3][6]=S4438[1][3]*Si3844[3][1]*T13844[1][4] + S4438[2][3]*Si3844[3][2]*T13844[2][5] + S4438[3][3]*Si3844[3][3]*T13844[3][6];

T3844[4][1]=(-(ZIF*S4438[2][2]) + YIF*S4438[2][3])*((-(ZIF*Si3844[2][2]) + YIF*Si3844[3][2])*T13844[2][5] + Si3844[1][1]*T13844[4][5]) + S4438[1][1]*((-(ZIF*Si3844[2][2]) + YIF*Si3844[3][2])*T13844[2][1] + (-(ZIF*Si3844[2][3]) + YIF*Si3844[3][3])*T13844[3][1] + Si3844[1][1]*T13844[4][1] + Si3844[1][2]*T13844[5][1]) + S4438[2][1]*((-(ZIF*Si3844[2][1]) + YIF*Si3844[3][1])*T13844[1][2] + (-(ZIF*Si3844[2][3]) + YIF*Si3844[3][3])*T13844[3][2] + Si3844[1][1]*T13844[4][2] + Si3844[1][2]*T13844[5][2]) + (-(ZIF*S4438[1][2]) + YIF*S4438[1][3])*((-(ZIF*Si3844[2][1]) + YIF*Si3844[3][1])*T13844[1][4] + Si3844[1][2]*T13844[5][4]) + (-(ZIF*S4438[3][2]) + YIF*S4438[3][3])*((-(ZIF*Si3844[2][3]) + YIF*Si3844[3][3])*T13844[3][6] + Si3844[1][1]*T13844[4][6] + Si3844[1][2]*T13844[5][6]);
T3844[4][2]=(ZIF*S4438[2][1] - XIF*S4438[2][3])*((-(ZIF*Si3844[2][2]) + YIF*Si3844[3][2])*T13844[2][5] + Si3844[1][1]*T13844[4][5]) + S4438[1][2]*((-(ZIF*Si3844[2][2]) + YIF*Si3844[3][2])*T13844[2][1] + (-(ZIF*Si3844[2][3]) + YIF*Si3844[3][3])*T13844[3][1] + Si3844[1][1]*T13844[4][1] + Si3844[1][2]*T13844[5][1]) + S4438[2][2]*((-(ZIF*Si3844[2][1]) + YIF*Si3844[3][1])*T13844[1][2] + (-(ZIF*Si3844[2][3]) + YIF*Si3844[3][3])*T13844[3][2] + Si3844[1][1]*T13844[4][2] + Si3844[1][2]*T13844[5][2]) + S4438[3][2]*((-(ZIF*Si3844[2][1]) + YIF*Si3844[3][1])*T13844[1][3] + (-(ZIF*Si3844[2][2]) + YIF*Si3844[3][2])*T13844[2][3] + Si3844[1][1]*T13844[4][3] + Si3844[1][2]*T13844[5][3]) + (ZIF*S4438[1][1] - XIF*S4438[1][3])*((-(ZIF*Si3844[2][1]) + YIF*Si3844[3][1])*T13844[1][4] + Si3844[1][2]*T13844[5][4]) - XIF*S4438[3][3]*((-(ZIF*Si3844[2][3]) + YIF*Si3844[3][3])*T13844[3][6] + Si3844[1][1]*T13844[4][6] + Si3844[1][2]*T13844[5][6]);
T3844[4][3]=(-(YIF*S4438[2][1]) + XIF*S4438[2][2])*((-(ZIF*Si3844[2][2]) + YIF*Si3844[3][2])*T13844[2][5] + Si3844[1][1]*T13844[4][5]) + S4438[1][3]*((-(ZIF*Si3844[2][2]) + YIF*Si3844[3][2])*T13844[2][1] + (-(ZIF*Si3844[2][3]) + YIF*Si3844[3][3])*T13844[3][1] + Si3844[1][1]*T13844[4][1] + Si3844[1][2]*T13844[5][1]) + S4438[2][3]*((-(ZIF*Si3844[2][1]) + YIF*Si3844[3][1])*T13844[1][2] + (-(ZIF*Si3844[2][3]) + YIF*Si3844[3][3])*T13844[3][2] + Si3844[1][1]*T13844[4][2] + Si3844[1][2]*T13844[5][2]) + S4438[3][3]*((-(ZIF*Si3844[2][1]) + YIF*Si3844[3][1])*T13844[1][3] + (-(ZIF*Si3844[2][2]) + YIF*Si3844[3][2])*T13844[2][3] + Si3844[1][1]*T13844[4][3] + Si3844[1][2]*T13844[5][3]) + (-(YIF*S4438[1][1]) + XIF*S4438[1][2])*((-(ZIF*Si3844[2][1]) + YIF*Si3844[3][1])*T13844[1][4] + Si3844[1][2]*T13844[5][4]) + XIF*S4438[3][2]*((-(ZIF*Si3844[2][3]) + YIF*Si3844[3][3])*T13844[3][6] + Si3844[1][1]*T13844[4][6] + Si3844[1][2]*T13844[5][6]);
T3844[4][4]=S4438[2][1]*((-(ZIF*Si3844[2][2]) + YIF*Si3844[3][2])*T13844[2][5] + Si3844[1][1]*T13844[4][5]) + S4438[1][1]*((-(ZIF*Si3844[2][1]) + YIF*Si3844[3][1])*T13844[1][4] + Si3844[1][2]*T13844[5][4]);
T3844[4][5]=S4438[2][2]*((-(ZIF*Si3844[2][2]) + YIF*Si3844[3][2])*T13844[2][5] + Si3844[1][1]*T13844[4][5]) + S4438[1][2]*((-(ZIF*Si3844[2][1]) + YIF*Si3844[3][1])*T13844[1][4] + Si3844[1][2]*T13844[5][4]) + S4438[3][2]*((-(ZIF*Si3844[2][3]) + YIF*Si3844[3][3])*T13844[3][6] + Si3844[1][1]*T13844[4][6] + Si3844[1][2]*T13844[5][6]);
T3844[4][6]=S4438[2][3]*((-(ZIF*Si3844[2][2]) + YIF*Si3844[3][2])*T13844[2][5] + Si3844[1][1]*T13844[4][5]) + S4438[1][3]*((-(ZIF*Si3844[2][1]) + YIF*Si3844[3][1])*T13844[1][4] + Si3844[1][2]*T13844[5][4]) + S4438[3][3]*((-(ZIF*Si3844[2][3]) + YIF*Si3844[3][3])*T13844[3][6] + Si3844[1][1]*T13844[4][6] + Si3844[1][2]*T13844[5][6]);

T3844[5][1]=(-(ZIF*S4438[3][2]) + YIF*S4438[3][3])*(-(XIF*Si3844[3][3]*T13844[3][6]) + Si3844[2][1]*T13844[4][6] + Si3844[2][2]*T13844[5][6]) + S4438[1][1]*((ZIF*Si3844[1][2] - XIF*Si3844[3][2])*T13844[2][1] - XIF*Si3844[3][3]*T13844[3][1] + Si3844[2][1]*T13844[4][1] + Si3844[2][2]*T13844[5][1] + Si3844[2][3]*T13844[6][1]) + S4438[2][1]*((ZIF*Si3844[1][1] - XIF*Si3844[3][1])*T13844[1][2] - XIF*Si3844[3][3]*T13844[3][2] + Si3844[2][1]*T13844[4][2] + Si3844[2][2]*T13844[5][2] + Si3844[2][3]*T13844[6][2]) + (-(ZIF*S4438[1][2]) + YIF*S4438[1][3])*((ZIF*Si3844[1][1] - XIF*Si3844[3][1])*T13844[1][4] + Si3844[2][2]*T13844[5][4] + Si3844[2][3]*T13844[6][4]) + (-(ZIF*S4438[2][2]) + YIF*S4438[2][3])*((ZIF*Si3844[1][2] - XIF*Si3844[3][2])*T13844[2][5] + Si3844[2][1]*T13844[4][5] + Si3844[2][3]*T13844[6][5]);
T3844[5][2]=-(XIF*S4438[3][3]*(-(XIF*Si3844[3][3]*T13844[3][6]) + Si3844[2][1]*T13844[4][6] + Si3844[2][2]*T13844[5][6])) + S4438[1][2]*((ZIF*Si3844[1][2] - XIF*Si3844[3][2])*T13844[2][1] - XIF*Si3844[3][3]*T13844[3][1] + Si3844[2][1]*T13844[4][1] + Si3844[2][2]*T13844[5][1] + Si3844[2][3]*T13844[6][1]) + S4438[2][2]*((ZIF*Si3844[1][1] - XIF*Si3844[3][1])*T13844[1][2] - XIF*Si3844[3][3]*T13844[3][2] + Si3844[2][1]*T13844[4][2] + Si3844[2][2]*T13844[5][2] + Si3844[2][3]*T13844[6][2]) + S4438[3][2]*((ZIF*Si3844[1][1] - XIF*Si3844[3][1])*T13844[1][3] + (ZIF*Si3844[1][2] - XIF*Si3844[3][2])*T13844[2][3] + Si3844[2][1]*T13844[4][3] + Si3844[2][2]*T13844[5][3] + Si3844[2][3]*T13844[6][3]) + (ZIF*S4438[1][1] - XIF*S4438[1][3])*((ZIF*Si3844[1][1] - XIF*Si3844[3][1])*T13844[1][4] + Si3844[2][2]*T13844[5][4] + Si3844[2][3]*T13844[6][4]) + (ZIF*S4438[2][1] - XIF*S4438[2][3])*((ZIF*Si3844[1][2] - XIF*Si3844[3][2])*T13844[2][5] + Si3844[2][1]*T13844[4][5] + Si3844[2][3]*T13844[6][5]);
T3844[5][3]=XIF*S4438[3][2]*(-(XIF*Si3844[3][3]*T13844[3][6]) + Si3844[2][1]*T13844[4][6] + Si3844[2][2]*T13844[5][6]) + S4438[1][3]*((ZIF*Si3844[1][2] - XIF*Si3844[3][2])*T13844[2][1] - XIF*Si3844[3][3]*T13844[3][1] + Si3844[2][1]*T13844[4][1] + Si3844[2][2]*T13844[5][1] + Si3844[2][3]*T13844[6][1]) + S4438[2][3]*((ZIF*Si3844[1][1] - XIF*Si3844[3][1])*T13844[1][2] - XIF*Si3844[3][3]*T13844[3][2] + Si3844[2][1]*T13844[4][2] + Si3844[2][2]*T13844[5][2] + Si3844[2][3]*T13844[6][2]) + S4438[3][3]*((ZIF*Si3844[1][1] - XIF*Si3844[3][1])*T13844[1][3] + (ZIF*Si3844[1][2] - XIF*Si3844[3][2])*T13844[2][3] + Si3844[2][1]*T13844[4][3] + Si3844[2][2]*T13844[5][3] + Si3844[2][3]*T13844[6][3]) + (-(YIF*S4438[1][1]) + XIF*S4438[1][2])*((ZIF*Si3844[1][1] - XIF*Si3844[3][1])*T13844[1][4] + Si3844[2][2]*T13844[5][4] + Si3844[2][3]*T13844[6][4]) + (-(YIF*S4438[2][1]) + XIF*S4438[2][2])*((ZIF*Si3844[1][2] - XIF*Si3844[3][2])*T13844[2][5] + Si3844[2][1]*T13844[4][5] + Si3844[2][3]*T13844[6][5]);
T3844[5][4]=S4438[1][1]*((ZIF*Si3844[1][1] - XIF*Si3844[3][1])*T13844[1][4] + Si3844[2][2]*T13844[5][4] + Si3844[2][3]*T13844[6][4]) + S4438[2][1]*((ZIF*Si3844[1][2] - XIF*Si3844[3][2])*T13844[2][5] + Si3844[2][1]*T13844[4][5] + Si3844[2][3]*T13844[6][5]);
T3844[5][5]=S4438[3][2]*(-(XIF*Si3844[3][3]*T13844[3][6]) + Si3844[2][1]*T13844[4][6] + Si3844[2][2]*T13844[5][6]) + S4438[1][2]*((ZIF*Si3844[1][1] - XIF*Si3844[3][1])*T13844[1][4] + Si3844[2][2]*T13844[5][4] + Si3844[2][3]*T13844[6][4]) + S4438[2][2]*((ZIF*Si3844[1][2] - XIF*Si3844[3][2])*T13844[2][5] + Si3844[2][1]*T13844[4][5] + Si3844[2][3]*T13844[6][5]);
T3844[5][6]=S4438[3][3]*(-(XIF*Si3844[3][3]*T13844[3][6]) + Si3844[2][1]*T13844[4][6] + Si3844[2][2]*T13844[5][6]) + S4438[1][3]*((ZIF*Si3844[1][1] - XIF*Si3844[3][1])*T13844[1][4] + Si3844[2][2]*T13844[5][4] + Si3844[2][3]*T13844[6][4]) + S4438[2][3]*((ZIF*Si3844[1][2] - XIF*Si3844[3][2])*T13844[2][5] + Si3844[2][1]*T13844[4][5] + Si3844[2][3]*T13844[6][5]);

T3844[6][1]=(-(ZIF*S4438[3][2]) + YIF*S4438[3][3])*(XIF*Si3844[2][3]*T13844[3][6] + Si3844[3][1]*T13844[4][6] + Si3844[3][2]*T13844[5][6]) + S4438[1][1]*((-(YIF*Si3844[1][2]) + XIF*Si3844[2][2])*T13844[2][1] + XIF*Si3844[2][3]*T13844[3][1] + Si3844[3][1]*T13844[4][1] + Si3844[3][2]*T13844[5][1] + Si3844[3][3]*T13844[6][1]) + S4438[2][1]*((-(YIF*Si3844[1][1]) + XIF*Si3844[2][1])*T13844[1][2] + XIF*Si3844[2][3]*T13844[3][2] + Si3844[3][1]*T13844[4][2] + Si3844[3][2]*T13844[5][2] + Si3844[3][3]*T13844[6][2]) + (-(ZIF*S4438[1][2]) + YIF*S4438[1][3])*((-(YIF*Si3844[1][1]) + XIF*Si3844[2][1])*T13844[1][4] + Si3844[3][2]*T13844[5][4] + Si3844[3][3]*T13844[6][4]) + (-(ZIF*S4438[2][2]) + YIF*S4438[2][3])*((-(YIF*Si3844[1][2]) + XIF*Si3844[2][2])*T13844[2][5] + Si3844[3][1]*T13844[4][5] + Si3844[3][3]*T13844[6][5]);
T3844[6][2]=-(XIF*S4438[3][3]*(XIF*Si3844[2][3]*T13844[3][6] + Si3844[3][1]*T13844[4][6] + Si3844[3][2]*T13844[5][6])) + S4438[1][2]*((-(YIF*Si3844[1][2]) + XIF*Si3844[2][2])*T13844[2][1] + XIF*Si3844[2][3]*T13844[3][1] + Si3844[3][1]*T13844[4][1] + Si3844[3][2]*T13844[5][1] + Si3844[3][3]*T13844[6][1]) + S4438[2][2]*((-(YIF*Si3844[1][1]) + XIF*Si3844[2][1])*T13844[1][2] + XIF*Si3844[2][3]*T13844[3][2] + Si3844[3][1]*T13844[4][2] + Si3844[3][2]*T13844[5][2] + Si3844[3][3]*T13844[6][2]) + S4438[3][2]*((-(YIF*Si3844[1][1]) + XIF*Si3844[2][1])*T13844[1][3] + (-(YIF*Si3844[1][2]) + XIF*Si3844[2][2])*T13844[2][3] + Si3844[3][1]*T13844[4][3] + Si3844[3][2]*T13844[5][3] + Si3844[3][3]*T13844[6][3]) + (ZIF*S4438[1][1] - XIF*S4438[1][3])*((-(YIF*Si3844[1][1]) + XIF*Si3844[2][1])*T13844[1][4] + Si3844[3][2]*T13844[5][4] + Si3844[3][3]*T13844[6][4]) + (ZIF*S4438[2][1] - XIF*S4438[2][3])*((-(YIF*Si3844[1][2]) + XIF*Si3844[2][2])*T13844[2][5] + Si3844[3][1]*T13844[4][5] + Si3844[3][3]*T13844[6][5]);
T3844[6][3]=XIF*S4438[3][2]*(XIF*Si3844[2][3]*T13844[3][6] + Si3844[3][1]*T13844[4][6] + Si3844[3][2]*T13844[5][6]) + S4438[1][3]*((-(YIF*Si3844[1][2]) + XIF*Si3844[2][2])*T13844[2][1] + XIF*Si3844[2][3]*T13844[3][1] + Si3844[3][1]*T13844[4][1] + Si3844[3][2]*T13844[5][1] + Si3844[3][3]*T13844[6][1]) + S4438[2][3]*((-(YIF*Si3844[1][1]) + XIF*Si3844[2][1])*T13844[1][2] + XIF*Si3844[2][3]*T13844[3][2] + Si3844[3][1]*T13844[4][2] + Si3844[3][2]*T13844[5][2] + Si3844[3][3]*T13844[6][2]) + S4438[3][3]*((-(YIF*Si3844[1][1]) + XIF*Si3844[2][1])*T13844[1][3] + (-(YIF*Si3844[1][2]) + XIF*Si3844[2][2])*T13844[2][3] + Si3844[3][1]*T13844[4][3] + Si3844[3][2]*T13844[5][3] + Si3844[3][3]*T13844[6][3]) + (-(YIF*S4438[1][1]) + XIF*S4438[1][2])*((-(YIF*Si3844[1][1]) + XIF*Si3844[2][1])*T13844[1][4] + Si3844[3][2]*T13844[5][4] + Si3844[3][3]*T13844[6][4]) + (-(YIF*S4438[2][1]) + XIF*S4438[2][2])*((-(YIF*Si3844[1][2]) + XIF*Si3844[2][2])*T13844[2][5] + Si3844[3][1]*T13844[4][5] + Si3844[3][3]*T13844[6][5]);
T3844[6][4]=S4438[1][1]*((-(YIF*Si3844[1][1]) + XIF*Si3844[2][1])*T13844[1][4] + Si3844[3][2]*T13844[5][4] + Si3844[3][3]*T13844[6][4]) + S4438[2][1]*((-(YIF*Si3844[1][2]) + XIF*Si3844[2][2])*T13844[2][5] + Si3844[3][1]*T13844[4][5] + Si3844[3][3]*T13844[6][5]);
T3844[6][5]=S4438[3][2]*(XIF*Si3844[2][3]*T13844[3][6] + Si3844[3][1]*T13844[4][6] + Si3844[3][2]*T13844[5][6]) + S4438[1][2]*((-(YIF*Si3844[1][1]) + XIF*Si3844[2][1])*T13844[1][4] + Si3844[3][2]*T13844[5][4] + Si3844[3][3]*T13844[6][4]) + S4438[2][2]*((-(YIF*Si3844[1][2]) + XIF*Si3844[2][2])*T13844[2][5] + Si3844[3][1]*T13844[4][5] + Si3844[3][3]*T13844[6][5]);
T3844[6][6]=S4438[3][3]*(XIF*Si3844[2][3]*T13844[3][6] + Si3844[3][1]*T13844[4][6] + Si3844[3][2]*T13844[5][6]) + S4438[1][3]*((-(YIF*Si3844[1][1]) + XIF*Si3844[2][1])*T13844[1][4] + Si3844[3][2]*T13844[5][4] + Si3844[3][3]*T13844[6][4]) + S4438[2][3]*((-(YIF*Si3844[1][2]) + XIF*Si3844[2][2])*T13844[2][5] + Si3844[3][1]*T13844[4][5] + Si3844[3][3]*T13844[6][5]);



}


void
hermes_InvDynArtfunc61(void)
      {




}


void
hermes_InvDynArtfunc62(void)
      {




}


void
hermes_InvDynArtfunc63(void)
      {
JA41[1][2]=0. + links[46].mcm[3];
JA41[1][3]=0. - links[46].mcm[2];
JA41[1][4]=0. + links[46].m;

JA41[2][1]=0. - links[46].mcm[3];
JA41[2][3]=0. + links[46].mcm[1];
JA41[2][5]=0. + links[46].m;

JA41[3][1]=0. + links[46].mcm[2];
JA41[3][2]=0. - links[46].mcm[1];
JA41[3][6]=0. + links[46].m;

JA41[4][1]=0. + links[46].inertia[1][1];
JA41[4][2]=0. + links[46].inertia[1][2];
JA41[4][3]=0. + links[46].inertia[1][3];
JA41[4][5]=0. - links[46].mcm[3];
JA41[4][6]=0. + links[46].mcm[2];

JA41[5][1]=0. + links[46].inertia[1][2];
JA41[5][2]=0. + links[46].inertia[2][2];
JA41[5][3]=0. + links[46].inertia[2][3];
JA41[5][4]=0. + links[46].mcm[3];
JA41[5][6]=0. - links[46].mcm[1];

JA41[6][1]=0. + links[46].inertia[1][3];
JA41[6][2]=0. + links[46].inertia[2][3];
JA41[6][3]=0. + links[46].inertia[3][3];
JA41[6][4]=0. - links[46].mcm[2];
JA41[6][5]=0. + links[46].mcm[1];


h41[1]=-JA41[1][3];
h41[2]=-JA41[2][3];
h41[4]=-JA41[4][3];
h41[5]=-JA41[5][3];
h41[6]=-JA41[6][3];

T14041[1][2]=JA41[1][2];
T14041[1][3]=JA41[1][3];
T14041[1][4]=JA41[1][4];

T14041[2][1]=JA41[2][1];
T14041[2][3]=JA41[2][3];
T14041[2][5]=JA41[2][5];

T14041[3][1]=JA41[3][1];
T14041[3][2]=JA41[3][2];
T14041[3][6]=JA41[3][6];

T14041[4][1]=JA41[4][1];
T14041[4][2]=JA41[4][2];
T14041[4][3]=JA41[4][3];
T14041[4][5]=JA41[4][5];
T14041[4][6]=JA41[4][6];

T14041[5][1]=JA41[5][1];
T14041[5][2]=JA41[5][2];
T14041[5][3]=JA41[5][3];
T14041[5][4]=JA41[5][4];
T14041[5][6]=JA41[5][6];

T14041[6][1]=JA41[6][1];
T14041[6][2]=JA41[6][2];
T14041[6][3]=JA41[6][3];
T14041[6][4]=JA41[6][4];
T14041[6][5]=JA41[6][5];


T4041[1][1]=S4140[2][1]*Si4041[1][1]*T14041[1][2] + S4140[1][1]*Si4041[1][2]*T14041[2][1];
T4041[1][2]=S4140[2][2]*Si4041[1][1]*T14041[1][2] + S4140[1][2]*Si4041[1][2]*T14041[2][1];
T4041[1][3]=Si4041[1][1]*T14041[1][3] + (-(YTHUMBFLEX*S4140[1][1]) + XTHUMBFLEX*S4140[1][2])*Si4041[1][1]*T14041[1][4] + Si4041[1][2]*T14041[2][3] + (-(YTHUMBFLEX*S4140[2][1]) + XTHUMBFLEX*S4140[2][2])*Si4041[1][2]*T14041[2][5];
T4041[1][4]=S4140[1][1]*Si4041[1][1]*T14041[1][4] + S4140[2][1]*Si4041[1][2]*T14041[2][5];
T4041[1][5]=S4140[1][2]*Si4041[1][1]*T14041[1][4] + S4140[2][2]*Si4041[1][2]*T14041[2][5];

T4041[2][1]=S4140[2][1]*Si4041[2][1]*T14041[1][2] + S4140[1][1]*Si4041[2][2]*T14041[2][1];
T4041[2][2]=S4140[2][2]*Si4041[2][1]*T14041[1][2] + S4140[1][2]*Si4041[2][2]*T14041[2][1];
T4041[2][3]=Si4041[2][1]*T14041[1][3] + (-(YTHUMBFLEX*S4140[1][1]) + XTHUMBFLEX*S4140[1][2])*Si4041[2][1]*T14041[1][4] + Si4041[2][2]*T14041[2][3] + (-(YTHUMBFLEX*S4140[2][1]) + XTHUMBFLEX*S4140[2][2])*Si4041[2][2]*T14041[2][5];
T4041[2][4]=S4140[1][1]*Si4041[2][1]*T14041[1][4] + S4140[2][1]*Si4041[2][2]*T14041[2][5];
T4041[2][5]=S4140[1][2]*Si4041[2][1]*T14041[1][4] + S4140[2][2]*Si4041[2][2]*T14041[2][5];

T4041[3][1]=S4140[1][1]*T14041[3][1] + S4140[2][1]*T14041[3][2] + YTHUMBFLEX*T14041[3][6];
T4041[3][2]=S4140[1][2]*T14041[3][1] + S4140[2][2]*T14041[3][2] - XTHUMBFLEX*T14041[3][6];
T4041[3][6]=T14041[3][6];

T4041[4][1]=S4140[1][1]*(YTHUMBFLEX*T14041[3][1] + Si4041[1][1]*T14041[4][1] + Si4041[1][2]*T14041[5][1]) + S4140[2][1]*(YTHUMBFLEX*T14041[3][2] + Si4041[1][1]*T14041[4][2] + Si4041[1][2]*T14041[5][2]) + YTHUMBFLEX*(YTHUMBFLEX*T14041[3][6] + Si4041[1][1]*T14041[4][6] + Si4041[1][2]*T14041[5][6]);
T4041[4][2]=S4140[1][2]*(YTHUMBFLEX*T14041[3][1] + Si4041[1][1]*T14041[4][1] + Si4041[1][2]*T14041[5][1]) + S4140[2][2]*(YTHUMBFLEX*T14041[3][2] + Si4041[1][1]*T14041[4][2] + Si4041[1][2]*T14041[5][2]) - XTHUMBFLEX*(YTHUMBFLEX*T14041[3][6] + Si4041[1][1]*T14041[4][6] + Si4041[1][2]*T14041[5][6]);
T4041[4][3]=Si4041[1][1]*T14041[4][3] + (-(YTHUMBFLEX*S4140[2][1]) + XTHUMBFLEX*S4140[2][2])*Si4041[1][1]*T14041[4][5] + Si4041[1][2]*T14041[5][3] + (-(YTHUMBFLEX*S4140[1][1]) + XTHUMBFLEX*S4140[1][2])*Si4041[1][2]*T14041[5][4];
T4041[4][4]=S4140[2][1]*Si4041[1][1]*T14041[4][5] + S4140[1][1]*Si4041[1][2]*T14041[5][4];
T4041[4][5]=S4140[2][2]*Si4041[1][1]*T14041[4][5] + S4140[1][2]*Si4041[1][2]*T14041[5][4];
T4041[4][6]=YTHUMBFLEX*T14041[3][6] + Si4041[1][1]*T14041[4][6] + Si4041[1][2]*T14041[5][6];

T4041[5][1]=S4140[1][1]*(-(XTHUMBFLEX*T14041[3][1]) + Si4041[2][1]*T14041[4][1] + Si4041[2][2]*T14041[5][1]) + S4140[2][1]*(-(XTHUMBFLEX*T14041[3][2]) + Si4041[2][1]*T14041[4][2] + Si4041[2][2]*T14041[5][2]) + YTHUMBFLEX*(-(XTHUMBFLEX*T14041[3][6]) + Si4041[2][1]*T14041[4][6] + Si4041[2][2]*T14041[5][6]);
T4041[5][2]=S4140[1][2]*(-(XTHUMBFLEX*T14041[3][1]) + Si4041[2][1]*T14041[4][1] + Si4041[2][2]*T14041[5][1]) + S4140[2][2]*(-(XTHUMBFLEX*T14041[3][2]) + Si4041[2][1]*T14041[4][2] + Si4041[2][2]*T14041[5][2]) - XTHUMBFLEX*(-(XTHUMBFLEX*T14041[3][6]) + Si4041[2][1]*T14041[4][6] + Si4041[2][2]*T14041[5][6]);
T4041[5][3]=Si4041[2][1]*T14041[4][3] + (-(YTHUMBFLEX*S4140[2][1]) + XTHUMBFLEX*S4140[2][2])*Si4041[2][1]*T14041[4][5] + Si4041[2][2]*T14041[5][3] + (-(YTHUMBFLEX*S4140[1][1]) + XTHUMBFLEX*S4140[1][2])*Si4041[2][2]*T14041[5][4];
T4041[5][4]=S4140[2][1]*Si4041[2][1]*T14041[4][5] + S4140[1][1]*Si4041[2][2]*T14041[5][4];
T4041[5][5]=S4140[2][2]*Si4041[2][1]*T14041[4][5] + S4140[1][2]*Si4041[2][2]*T14041[5][4];
T4041[5][6]=-(XTHUMBFLEX*T14041[3][6]) + Si4041[2][1]*T14041[4][6] + Si4041[2][2]*T14041[5][6];

T4041[6][1]=S4140[1][1]*((-(YTHUMBFLEX*Si4041[1][2]) + XTHUMBFLEX*Si4041[2][2])*T14041[2][1] + T14041[6][1]) + S4140[2][1]*((-(YTHUMBFLEX*Si4041[1][1]) + XTHUMBFLEX*Si4041[2][1])*T14041[1][2] + T14041[6][2]);
T4041[6][2]=S4140[1][2]*((-(YTHUMBFLEX*Si4041[1][2]) + XTHUMBFLEX*Si4041[2][2])*T14041[2][1] + T14041[6][1]) + S4140[2][2]*((-(YTHUMBFLEX*Si4041[1][1]) + XTHUMBFLEX*Si4041[2][1])*T14041[1][2] + T14041[6][2]);
T4041[6][3]=(-(YTHUMBFLEX*Si4041[1][1]) + XTHUMBFLEX*Si4041[2][1])*T14041[1][3] + (-(YTHUMBFLEX*Si4041[1][2]) + XTHUMBFLEX*Si4041[2][2])*T14041[2][3] + T14041[6][3] + (-(YTHUMBFLEX*S4140[1][1]) + XTHUMBFLEX*S4140[1][2])*((-(YTHUMBFLEX*Si4041[1][1]) + XTHUMBFLEX*Si4041[2][1])*T14041[1][4] + T14041[6][4]) + (-(YTHUMBFLEX*S4140[2][1]) + XTHUMBFLEX*S4140[2][2])*((-(YTHUMBFLEX*Si4041[1][2]) + XTHUMBFLEX*Si4041[2][2])*T14041[2][5] + T14041[6][5]);
T4041[6][4]=S4140[1][1]*((-(YTHUMBFLEX*Si4041[1][1]) + XTHUMBFLEX*Si4041[2][1])*T14041[1][4] + T14041[6][4]) + S4140[2][1]*((-(YTHUMBFLEX*Si4041[1][2]) + XTHUMBFLEX*Si4041[2][2])*T14041[2][5] + T14041[6][5]);
T4041[6][5]=S4140[1][2]*((-(YTHUMBFLEX*Si4041[1][1]) + XTHUMBFLEX*Si4041[2][1])*T14041[1][4] + T14041[6][4]) + S4140[2][2]*((-(YTHUMBFLEX*Si4041[1][2]) + XTHUMBFLEX*Si4041[2][2])*T14041[2][5] + T14041[6][5]);



}


void
hermes_InvDynArtfunc64(void)
      {
JA40[1][1]=T4041[1][1];
JA40[1][2]=links[45].mcm[3] + T4041[1][2];
JA40[1][3]=-links[45].mcm[2] + T4041[1][3];
JA40[1][4]=links[45].m + T4041[1][4];
JA40[1][5]=T4041[1][5];

JA40[2][1]=-links[45].mcm[3] + T4041[2][1];
JA40[2][2]=T4041[2][2];
JA40[2][3]=links[45].mcm[1] + T4041[2][3];
JA40[2][4]=T4041[2][4];
JA40[2][5]=links[45].m + T4041[2][5];

JA40[3][1]=links[45].mcm[2] + T4041[3][1];
JA40[3][2]=-links[45].mcm[1] + T4041[3][2];
JA40[3][6]=links[45].m + T4041[3][6];

JA40[4][1]=links[45].inertia[1][1] + T4041[4][1];
JA40[4][2]=links[45].inertia[1][2] + T4041[4][2];
JA40[4][3]=links[45].inertia[1][3] + T4041[4][3];
JA40[4][4]=T4041[4][4];
JA40[4][5]=-links[45].mcm[3] + T4041[4][5];
JA40[4][6]=links[45].mcm[2] + T4041[4][6];

JA40[5][1]=links[45].inertia[1][2] + T4041[5][1];
JA40[5][2]=links[45].inertia[2][2] + T4041[5][2];
JA40[5][3]=links[45].inertia[2][3] + T4041[5][3];
JA40[5][4]=links[45].mcm[3] + T4041[5][4];
JA40[5][5]=T4041[5][5];
JA40[5][6]=-links[45].mcm[1] + T4041[5][6];

JA40[6][1]=links[45].inertia[1][3] + T4041[6][1];
JA40[6][2]=links[45].inertia[2][3] + T4041[6][2];
JA40[6][3]=links[45].inertia[3][3] + T4041[6][3];
JA40[6][4]=-links[45].mcm[2] + T4041[6][4];
JA40[6][5]=links[45].mcm[1] + T4041[6][5];


h40[1]=JA40[1][1];
h40[2]=JA40[2][1];
h40[3]=JA40[3][1];
h40[4]=JA40[4][1];
h40[5]=JA40[5][1];
h40[6]=JA40[6][1];

T13840[1][1]=JA40[1][1];
T13840[1][2]=JA40[1][2];
T13840[1][3]=JA40[1][3];
T13840[1][4]=JA40[1][4];
T13840[1][5]=JA40[1][5];

T13840[2][1]=JA40[2][1];
T13840[2][2]=JA40[2][2];
T13840[2][3]=JA40[2][3];
T13840[2][4]=JA40[2][4];
T13840[2][5]=JA40[2][5];

T13840[3][1]=JA40[3][1];
T13840[3][2]=JA40[3][2];
T13840[3][6]=JA40[3][6];

T13840[4][1]=JA40[4][1];
T13840[4][2]=JA40[4][2];
T13840[4][3]=JA40[4][3];
T13840[4][4]=JA40[4][4];
T13840[4][5]=JA40[4][5];
T13840[4][6]=JA40[4][6];

T13840[5][1]=JA40[5][1];
T13840[5][2]=JA40[5][2];
T13840[5][3]=JA40[5][3];
T13840[5][4]=JA40[5][4];
T13840[5][5]=JA40[5][5];
T13840[5][6]=JA40[5][6];

T13840[6][1]=JA40[6][1];
T13840[6][2]=JA40[6][2];
T13840[6][3]=JA40[6][3];
T13840[6][4]=JA40[6][4];
T13840[6][5]=JA40[6][5];


T3840[1][1]=S4038[3][1]*(Si3840[1][1]*T13840[1][3] + Si3840[1][2]*T13840[2][3]) + (-(ZTHUMB*S4038[1][2]) + YTHUMB*S4038[1][3])*(Si3840[1][1]*T13840[1][4] + Si3840[1][2]*T13840[2][4]) + (-(ZTHUMB*S4038[2][2]) + YTHUMB*S4038[2][3])*(Si3840[1][1]*T13840[1][5] + Si3840[1][2]*T13840[2][5]) + S4038[1][1]*(Si3840[1][1]*T13840[1][1] + Si3840[1][2]*T13840[2][1] + Si3840[1][3]*T13840[3][1]) + S4038[2][1]*(Si3840[1][1]*T13840[1][2] + Si3840[1][2]*T13840[2][2] + Si3840[1][3]*T13840[3][2]) + (-(ZTHUMB*S4038[3][2]) + YTHUMB*S4038[3][3])*Si3840[1][3]*T13840[3][6];
T3840[1][2]=S4038[3][2]*(Si3840[1][1]*T13840[1][3] + Si3840[1][2]*T13840[2][3]) + (ZTHUMB*S4038[1][1] - XTHUMB*S4038[1][3])*(Si3840[1][1]*T13840[1][4] + Si3840[1][2]*T13840[2][4]) + (ZTHUMB*S4038[2][1] - XTHUMB*S4038[2][3])*(Si3840[1][1]*T13840[1][5] + Si3840[1][2]*T13840[2][5]) + S4038[1][2]*(Si3840[1][1]*T13840[1][1] + Si3840[1][2]*T13840[2][1] + Si3840[1][3]*T13840[3][1]) + S4038[2][2]*(Si3840[1][1]*T13840[1][2] + Si3840[1][2]*T13840[2][2] + Si3840[1][3]*T13840[3][2]) + (ZTHUMB*S4038[3][1] - XTHUMB*S4038[3][3])*Si3840[1][3]*T13840[3][6];
T3840[1][3]=S4038[3][3]*(Si3840[1][1]*T13840[1][3] + Si3840[1][2]*T13840[2][3]) + (-(YTHUMB*S4038[1][1]) + XTHUMB*S4038[1][2])*(Si3840[1][1]*T13840[1][4] + Si3840[1][2]*T13840[2][4]) + (-(YTHUMB*S4038[2][1]) + XTHUMB*S4038[2][2])*(Si3840[1][1]*T13840[1][5] + Si3840[1][2]*T13840[2][5]) + S4038[1][3]*(Si3840[1][1]*T13840[1][1] + Si3840[1][2]*T13840[2][1] + Si3840[1][3]*T13840[3][1]) + S4038[2][3]*(Si3840[1][1]*T13840[1][2] + Si3840[1][2]*T13840[2][2] + Si3840[1][3]*T13840[3][2]) + (-(YTHUMB*S4038[3][1]) + XTHUMB*S4038[3][2])*Si3840[1][3]*T13840[3][6];
T3840[1][4]=S4038[1][1]*(Si3840[1][1]*T13840[1][4] + Si3840[1][2]*T13840[2][4]) + S4038[2][1]*(Si3840[1][1]*T13840[1][5] + Si3840[1][2]*T13840[2][5]) + S4038[3][1]*Si3840[1][3]*T13840[3][6];
T3840[1][5]=S4038[1][2]*(Si3840[1][1]*T13840[1][4] + Si3840[1][2]*T13840[2][4]) + S4038[2][2]*(Si3840[1][1]*T13840[1][5] + Si3840[1][2]*T13840[2][5]) + S4038[3][2]*Si3840[1][3]*T13840[3][6];
T3840[1][6]=S4038[1][3]*(Si3840[1][1]*T13840[1][4] + Si3840[1][2]*T13840[2][4]) + S4038[2][3]*(Si3840[1][1]*T13840[1][5] + Si3840[1][2]*T13840[2][5]) + S4038[3][3]*Si3840[1][3]*T13840[3][6];

T3840[2][1]=S4038[3][1]*(Si3840[2][1]*T13840[1][3] + Si3840[2][2]*T13840[2][3]) + (-(ZTHUMB*S4038[1][2]) + YTHUMB*S4038[1][3])*(Si3840[2][1]*T13840[1][4] + Si3840[2][2]*T13840[2][4]) + (-(ZTHUMB*S4038[2][2]) + YTHUMB*S4038[2][3])*(Si3840[2][1]*T13840[1][5] + Si3840[2][2]*T13840[2][5]) + S4038[1][1]*(Si3840[2][1]*T13840[1][1] + Si3840[2][2]*T13840[2][1] + Si3840[2][3]*T13840[3][1]) + S4038[2][1]*(Si3840[2][1]*T13840[1][2] + Si3840[2][2]*T13840[2][2] + Si3840[2][3]*T13840[3][2]) + (-(ZTHUMB*S4038[3][2]) + YTHUMB*S4038[3][3])*Si3840[2][3]*T13840[3][6];
T3840[2][2]=S4038[3][2]*(Si3840[2][1]*T13840[1][3] + Si3840[2][2]*T13840[2][3]) + (ZTHUMB*S4038[1][1] - XTHUMB*S4038[1][3])*(Si3840[2][1]*T13840[1][4] + Si3840[2][2]*T13840[2][4]) + (ZTHUMB*S4038[2][1] - XTHUMB*S4038[2][3])*(Si3840[2][1]*T13840[1][5] + Si3840[2][2]*T13840[2][5]) + S4038[1][2]*(Si3840[2][1]*T13840[1][1] + Si3840[2][2]*T13840[2][1] + Si3840[2][3]*T13840[3][1]) + S4038[2][2]*(Si3840[2][1]*T13840[1][2] + Si3840[2][2]*T13840[2][2] + Si3840[2][3]*T13840[3][2]) + (ZTHUMB*S4038[3][1] - XTHUMB*S4038[3][3])*Si3840[2][3]*T13840[3][6];
T3840[2][3]=S4038[3][3]*(Si3840[2][1]*T13840[1][3] + Si3840[2][2]*T13840[2][3]) + (-(YTHUMB*S4038[1][1]) + XTHUMB*S4038[1][2])*(Si3840[2][1]*T13840[1][4] + Si3840[2][2]*T13840[2][4]) + (-(YTHUMB*S4038[2][1]) + XTHUMB*S4038[2][2])*(Si3840[2][1]*T13840[1][5] + Si3840[2][2]*T13840[2][5]) + S4038[1][3]*(Si3840[2][1]*T13840[1][1] + Si3840[2][2]*T13840[2][1] + Si3840[2][3]*T13840[3][1]) + S4038[2][3]*(Si3840[2][1]*T13840[1][2] + Si3840[2][2]*T13840[2][2] + Si3840[2][3]*T13840[3][2]) + (-(YTHUMB*S4038[3][1]) + XTHUMB*S4038[3][2])*Si3840[2][3]*T13840[3][6];
T3840[2][4]=S4038[1][1]*(Si3840[2][1]*T13840[1][4] + Si3840[2][2]*T13840[2][4]) + S4038[2][1]*(Si3840[2][1]*T13840[1][5] + Si3840[2][2]*T13840[2][5]) + S4038[3][1]*Si3840[2][3]*T13840[3][6];
T3840[2][5]=S4038[1][2]*(Si3840[2][1]*T13840[1][4] + Si3840[2][2]*T13840[2][4]) + S4038[2][2]*(Si3840[2][1]*T13840[1][5] + Si3840[2][2]*T13840[2][5]) + S4038[3][2]*Si3840[2][3]*T13840[3][6];
T3840[2][6]=S4038[1][3]*(Si3840[2][1]*T13840[1][4] + Si3840[2][2]*T13840[2][4]) + S4038[2][3]*(Si3840[2][1]*T13840[1][5] + Si3840[2][2]*T13840[2][5]) + S4038[3][3]*Si3840[2][3]*T13840[3][6];

T3840[3][1]=S4038[3][1]*(Si3840[3][1]*T13840[1][3] + Si3840[3][2]*T13840[2][3]) + (-(ZTHUMB*S4038[1][2]) + YTHUMB*S4038[1][3])*(Si3840[3][1]*T13840[1][4] + Si3840[3][2]*T13840[2][4]) + (-(ZTHUMB*S4038[2][2]) + YTHUMB*S4038[2][3])*(Si3840[3][1]*T13840[1][5] + Si3840[3][2]*T13840[2][5]) + S4038[1][1]*(Si3840[3][1]*T13840[1][1] + Si3840[3][2]*T13840[2][1] + Si3840[3][3]*T13840[3][1]) + S4038[2][1]*(Si3840[3][1]*T13840[1][2] + Si3840[3][2]*T13840[2][2] + Si3840[3][3]*T13840[3][2]) + (-(ZTHUMB*S4038[3][2]) + YTHUMB*S4038[3][3])*Si3840[3][3]*T13840[3][6];
T3840[3][2]=S4038[3][2]*(Si3840[3][1]*T13840[1][3] + Si3840[3][2]*T13840[2][3]) + (ZTHUMB*S4038[1][1] - XTHUMB*S4038[1][3])*(Si3840[3][1]*T13840[1][4] + Si3840[3][2]*T13840[2][4]) + (ZTHUMB*S4038[2][1] - XTHUMB*S4038[2][3])*(Si3840[3][1]*T13840[1][5] + Si3840[3][2]*T13840[2][5]) + S4038[1][2]*(Si3840[3][1]*T13840[1][1] + Si3840[3][2]*T13840[2][1] + Si3840[3][3]*T13840[3][1]) + S4038[2][2]*(Si3840[3][1]*T13840[1][2] + Si3840[3][2]*T13840[2][2] + Si3840[3][3]*T13840[3][2]) + (ZTHUMB*S4038[3][1] - XTHUMB*S4038[3][3])*Si3840[3][3]*T13840[3][6];
T3840[3][3]=S4038[3][3]*(Si3840[3][1]*T13840[1][3] + Si3840[3][2]*T13840[2][3]) + (-(YTHUMB*S4038[1][1]) + XTHUMB*S4038[1][2])*(Si3840[3][1]*T13840[1][4] + Si3840[3][2]*T13840[2][4]) + (-(YTHUMB*S4038[2][1]) + XTHUMB*S4038[2][2])*(Si3840[3][1]*T13840[1][5] + Si3840[3][2]*T13840[2][5]) + S4038[1][3]*(Si3840[3][1]*T13840[1][1] + Si3840[3][2]*T13840[2][1] + Si3840[3][3]*T13840[3][1]) + S4038[2][3]*(Si3840[3][1]*T13840[1][2] + Si3840[3][2]*T13840[2][2] + Si3840[3][3]*T13840[3][2]) + (-(YTHUMB*S4038[3][1]) + XTHUMB*S4038[3][2])*Si3840[3][3]*T13840[3][6];
T3840[3][4]=S4038[1][1]*(Si3840[3][1]*T13840[1][4] + Si3840[3][2]*T13840[2][4]) + S4038[2][1]*(Si3840[3][1]*T13840[1][5] + Si3840[3][2]*T13840[2][5]) + S4038[3][1]*Si3840[3][3]*T13840[3][6];
T3840[3][5]=S4038[1][2]*(Si3840[3][1]*T13840[1][4] + Si3840[3][2]*T13840[2][4]) + S4038[2][2]*(Si3840[3][1]*T13840[1][5] + Si3840[3][2]*T13840[2][5]) + S4038[3][2]*Si3840[3][3]*T13840[3][6];
T3840[3][6]=S4038[1][3]*(Si3840[3][1]*T13840[1][4] + Si3840[3][2]*T13840[2][4]) + S4038[2][3]*(Si3840[3][1]*T13840[1][5] + Si3840[3][2]*T13840[2][5]) + S4038[3][3]*Si3840[3][3]*T13840[3][6];

T3840[4][1]=(-(ZTHUMB*S4038[3][2]) + YTHUMB*S4038[3][3])*((-(ZTHUMB*Si3840[2][3]) + YTHUMB*Si3840[3][3])*T13840[3][6] + Si3840[1][1]*T13840[4][6] + Si3840[1][2]*T13840[5][6]) + S4038[1][1]*((-(ZTHUMB*Si3840[2][1]) + YTHUMB*Si3840[3][1])*T13840[1][1] + (-(ZTHUMB*Si3840[2][2]) + YTHUMB*Si3840[3][2])*T13840[2][1] + (-(ZTHUMB*Si3840[2][3]) + YTHUMB*Si3840[3][3])*T13840[3][1] + Si3840[1][1]*T13840[4][1] + Si3840[1][2]*T13840[5][1] + Si3840[1][3]*T13840[6][1]) + S4038[2][1]*((-(ZTHUMB*Si3840[2][1]) + YTHUMB*Si3840[3][1])*T13840[1][2] + (-(ZTHUMB*Si3840[2][2]) + YTHUMB*Si3840[3][2])*T13840[2][2] + (-(ZTHUMB*Si3840[2][3]) + YTHUMB*Si3840[3][3])*T13840[3][2] + Si3840[1][1]*T13840[4][2] + Si3840[1][2]*T13840[5][2] + Si3840[1][3]*T13840[6][2]) + S4038[3][1]*((-(ZTHUMB*Si3840[2][1]) + YTHUMB*Si3840[3][1])*T13840[1][3] + (-(ZTHUMB*Si3840[2][2]) + YTHUMB*Si3840[3][2])*T13840[2][3] + Si3840[1][1]*T13840[4][3] + Si3840[1][2]*T13840[5][3] + Si3840[1][3]*T13840[6][3]) + (-(ZTHUMB*S4038[1][2]) + YTHUMB*S4038[1][3])*((-(ZTHUMB*Si3840[2][1]) + YTHUMB*Si3840[3][1])*T13840[1][4] + (-(ZTHUMB*Si3840[2][2]) + YTHUMB*Si3840[3][2])*T13840[2][4] + Si3840[1][1]*T13840[4][4] + Si3840[1][2]*T13840[5][4] + Si3840[1][3]*T13840[6][4]) + (-(ZTHUMB*S4038[2][2]) + YTHUMB*S4038[2][3])*((-(ZTHUMB*Si3840[2][1]) + YTHUMB*Si3840[3][1])*T13840[1][5] + (-(ZTHUMB*Si3840[2][2]) + YTHUMB*Si3840[3][2])*T13840[2][5] + Si3840[1][1]*T13840[4][5] + Si3840[1][2]*T13840[5][5] + Si3840[1][3]*T13840[6][5]);
T3840[4][2]=(ZTHUMB*S4038[3][1] - XTHUMB*S4038[3][3])*((-(ZTHUMB*Si3840[2][3]) + YTHUMB*Si3840[3][3])*T13840[3][6] + Si3840[1][1]*T13840[4][6] + Si3840[1][2]*T13840[5][6]) + S4038[1][2]*((-(ZTHUMB*Si3840[2][1]) + YTHUMB*Si3840[3][1])*T13840[1][1] + (-(ZTHUMB*Si3840[2][2]) + YTHUMB*Si3840[3][2])*T13840[2][1] + (-(ZTHUMB*Si3840[2][3]) + YTHUMB*Si3840[3][3])*T13840[3][1] + Si3840[1][1]*T13840[4][1] + Si3840[1][2]*T13840[5][1] + Si3840[1][3]*T13840[6][1]) + S4038[2][2]*((-(ZTHUMB*Si3840[2][1]) + YTHUMB*Si3840[3][1])*T13840[1][2] + (-(ZTHUMB*Si3840[2][2]) + YTHUMB*Si3840[3][2])*T13840[2][2] + (-(ZTHUMB*Si3840[2][3]) + YTHUMB*Si3840[3][3])*T13840[3][2] + Si3840[1][1]*T13840[4][2] + Si3840[1][2]*T13840[5][2] + Si3840[1][3]*T13840[6][2]) + S4038[3][2]*((-(ZTHUMB*Si3840[2][1]) + YTHUMB*Si3840[3][1])*T13840[1][3] + (-(ZTHUMB*Si3840[2][2]) + YTHUMB*Si3840[3][2])*T13840[2][3] + Si3840[1][1]*T13840[4][3] + Si3840[1][2]*T13840[5][3] + Si3840[1][3]*T13840[6][3]) + (ZTHUMB*S4038[1][1] - XTHUMB*S4038[1][3])*((-(ZTHUMB*Si3840[2][1]) + YTHUMB*Si3840[3][1])*T13840[1][4] + (-(ZTHUMB*Si3840[2][2]) + YTHUMB*Si3840[3][2])*T13840[2][4] + Si3840[1][1]*T13840[4][4] + Si3840[1][2]*T13840[5][4] + Si3840[1][3]*T13840[6][4]) + (ZTHUMB*S4038[2][1] - XTHUMB*S4038[2][3])*((-(ZTHUMB*Si3840[2][1]) + YTHUMB*Si3840[3][1])*T13840[1][5] + (-(ZTHUMB*Si3840[2][2]) + YTHUMB*Si3840[3][2])*T13840[2][5] + Si3840[1][1]*T13840[4][5] + Si3840[1][2]*T13840[5][5] + Si3840[1][3]*T13840[6][5]);
T3840[4][3]=(-(YTHUMB*S4038[3][1]) + XTHUMB*S4038[3][2])*((-(ZTHUMB*Si3840[2][3]) + YTHUMB*Si3840[3][3])*T13840[3][6] + Si3840[1][1]*T13840[4][6] + Si3840[1][2]*T13840[5][6]) + S4038[1][3]*((-(ZTHUMB*Si3840[2][1]) + YTHUMB*Si3840[3][1])*T13840[1][1] + (-(ZTHUMB*Si3840[2][2]) + YTHUMB*Si3840[3][2])*T13840[2][1] + (-(ZTHUMB*Si3840[2][3]) + YTHUMB*Si3840[3][3])*T13840[3][1] + Si3840[1][1]*T13840[4][1] + Si3840[1][2]*T13840[5][1] + Si3840[1][3]*T13840[6][1]) + S4038[2][3]*((-(ZTHUMB*Si3840[2][1]) + YTHUMB*Si3840[3][1])*T13840[1][2] + (-(ZTHUMB*Si3840[2][2]) + YTHUMB*Si3840[3][2])*T13840[2][2] + (-(ZTHUMB*Si3840[2][3]) + YTHUMB*Si3840[3][3])*T13840[3][2] + Si3840[1][1]*T13840[4][2] + Si3840[1][2]*T13840[5][2] + Si3840[1][3]*T13840[6][2]) + S4038[3][3]*((-(ZTHUMB*Si3840[2][1]) + YTHUMB*Si3840[3][1])*T13840[1][3] + (-(ZTHUMB*Si3840[2][2]) + YTHUMB*Si3840[3][2])*T13840[2][3] + Si3840[1][1]*T13840[4][3] + Si3840[1][2]*T13840[5][3] + Si3840[1][3]*T13840[6][3]) + (-(YTHUMB*S4038[1][1]) + XTHUMB*S4038[1][2])*((-(ZTHUMB*Si3840[2][1]) + YTHUMB*Si3840[3][1])*T13840[1][4] + (-(ZTHUMB*Si3840[2][2]) + YTHUMB*Si3840[3][2])*T13840[2][4] + Si3840[1][1]*T13840[4][4] + Si3840[1][2]*T13840[5][4] + Si3840[1][3]*T13840[6][4]) + (-(YTHUMB*S4038[2][1]) + XTHUMB*S4038[2][2])*((-(ZTHUMB*Si3840[2][1]) + YTHUMB*Si3840[3][1])*T13840[1][5] + (-(ZTHUMB*Si3840[2][2]) + YTHUMB*Si3840[3][2])*T13840[2][5] + Si3840[1][1]*T13840[4][5] + Si3840[1][2]*T13840[5][5] + Si3840[1][3]*T13840[6][5]);
T3840[4][4]=S4038[3][1]*((-(ZTHUMB*Si3840[2][3]) + YTHUMB*Si3840[3][3])*T13840[3][6] + Si3840[1][1]*T13840[4][6] + Si3840[1][2]*T13840[5][6]) + S4038[1][1]*((-(ZTHUMB*Si3840[2][1]) + YTHUMB*Si3840[3][1])*T13840[1][4] + (-(ZTHUMB*Si3840[2][2]) + YTHUMB*Si3840[3][2])*T13840[2][4] + Si3840[1][1]*T13840[4][4] + Si3840[1][2]*T13840[5][4] + Si3840[1][3]*T13840[6][4]) + S4038[2][1]*((-(ZTHUMB*Si3840[2][1]) + YTHUMB*Si3840[3][1])*T13840[1][5] + (-(ZTHUMB*Si3840[2][2]) + YTHUMB*Si3840[3][2])*T13840[2][5] + Si3840[1][1]*T13840[4][5] + Si3840[1][2]*T13840[5][5] + Si3840[1][3]*T13840[6][5]);
T3840[4][5]=S4038[3][2]*((-(ZTHUMB*Si3840[2][3]) + YTHUMB*Si3840[3][3])*T13840[3][6] + Si3840[1][1]*T13840[4][6] + Si3840[1][2]*T13840[5][6]) + S4038[1][2]*((-(ZTHUMB*Si3840[2][1]) + YTHUMB*Si3840[3][1])*T13840[1][4] + (-(ZTHUMB*Si3840[2][2]) + YTHUMB*Si3840[3][2])*T13840[2][4] + Si3840[1][1]*T13840[4][4] + Si3840[1][2]*T13840[5][4] + Si3840[1][3]*T13840[6][4]) + S4038[2][2]*((-(ZTHUMB*Si3840[2][1]) + YTHUMB*Si3840[3][1])*T13840[1][5] + (-(ZTHUMB*Si3840[2][2]) + YTHUMB*Si3840[3][2])*T13840[2][5] + Si3840[1][1]*T13840[4][5] + Si3840[1][2]*T13840[5][5] + Si3840[1][3]*T13840[6][5]);
T3840[4][6]=S4038[3][3]*((-(ZTHUMB*Si3840[2][3]) + YTHUMB*Si3840[3][3])*T13840[3][6] + Si3840[1][1]*T13840[4][6] + Si3840[1][2]*T13840[5][6]) + S4038[1][3]*((-(ZTHUMB*Si3840[2][1]) + YTHUMB*Si3840[3][1])*T13840[1][4] + (-(ZTHUMB*Si3840[2][2]) + YTHUMB*Si3840[3][2])*T13840[2][4] + Si3840[1][1]*T13840[4][4] + Si3840[1][2]*T13840[5][4] + Si3840[1][3]*T13840[6][4]) + S4038[2][3]*((-(ZTHUMB*Si3840[2][1]) + YTHUMB*Si3840[3][1])*T13840[1][5] + (-(ZTHUMB*Si3840[2][2]) + YTHUMB*Si3840[3][2])*T13840[2][5] + Si3840[1][1]*T13840[4][5] + Si3840[1][2]*T13840[5][5] + Si3840[1][3]*T13840[6][5]);

T3840[5][1]=(-(ZTHUMB*S4038[3][2]) + YTHUMB*S4038[3][3])*((ZTHUMB*Si3840[1][3] - XTHUMB*Si3840[3][3])*T13840[3][6] + Si3840[2][1]*T13840[4][6] + Si3840[2][2]*T13840[5][6]) + S4038[1][1]*((ZTHUMB*Si3840[1][1] - XTHUMB*Si3840[3][1])*T13840[1][1] + (ZTHUMB*Si3840[1][2] - XTHUMB*Si3840[3][2])*T13840[2][1] + (ZTHUMB*Si3840[1][3] - XTHUMB*Si3840[3][3])*T13840[3][1] + Si3840[2][1]*T13840[4][1] + Si3840[2][2]*T13840[5][1] + Si3840[2][3]*T13840[6][1]) + S4038[2][1]*((ZTHUMB*Si3840[1][1] - XTHUMB*Si3840[3][1])*T13840[1][2] + (ZTHUMB*Si3840[1][2] - XTHUMB*Si3840[3][2])*T13840[2][2] + (ZTHUMB*Si3840[1][3] - XTHUMB*Si3840[3][3])*T13840[3][2] + Si3840[2][1]*T13840[4][2] + Si3840[2][2]*T13840[5][2] + Si3840[2][3]*T13840[6][2]) + S4038[3][1]*((ZTHUMB*Si3840[1][1] - XTHUMB*Si3840[3][1])*T13840[1][3] + (ZTHUMB*Si3840[1][2] - XTHUMB*Si3840[3][2])*T13840[2][3] + Si3840[2][1]*T13840[4][3] + Si3840[2][2]*T13840[5][3] + Si3840[2][3]*T13840[6][3]) + (-(ZTHUMB*S4038[1][2]) + YTHUMB*S4038[1][3])*((ZTHUMB*Si3840[1][1] - XTHUMB*Si3840[3][1])*T13840[1][4] + (ZTHUMB*Si3840[1][2] - XTHUMB*Si3840[3][2])*T13840[2][4] + Si3840[2][1]*T13840[4][4] + Si3840[2][2]*T13840[5][4] + Si3840[2][3]*T13840[6][4]) + (-(ZTHUMB*S4038[2][2]) + YTHUMB*S4038[2][3])*((ZTHUMB*Si3840[1][1] - XTHUMB*Si3840[3][1])*T13840[1][5] + (ZTHUMB*Si3840[1][2] - XTHUMB*Si3840[3][2])*T13840[2][5] + Si3840[2][1]*T13840[4][5] + Si3840[2][2]*T13840[5][5] + Si3840[2][3]*T13840[6][5]);
T3840[5][2]=(ZTHUMB*S4038[3][1] - XTHUMB*S4038[3][3])*((ZTHUMB*Si3840[1][3] - XTHUMB*Si3840[3][3])*T13840[3][6] + Si3840[2][1]*T13840[4][6] + Si3840[2][2]*T13840[5][6]) + S4038[1][2]*((ZTHUMB*Si3840[1][1] - XTHUMB*Si3840[3][1])*T13840[1][1] + (ZTHUMB*Si3840[1][2] - XTHUMB*Si3840[3][2])*T13840[2][1] + (ZTHUMB*Si3840[1][3] - XTHUMB*Si3840[3][3])*T13840[3][1] + Si3840[2][1]*T13840[4][1] + Si3840[2][2]*T13840[5][1] + Si3840[2][3]*T13840[6][1]) + S4038[2][2]*((ZTHUMB*Si3840[1][1] - XTHUMB*Si3840[3][1])*T13840[1][2] + (ZTHUMB*Si3840[1][2] - XTHUMB*Si3840[3][2])*T13840[2][2] + (ZTHUMB*Si3840[1][3] - XTHUMB*Si3840[3][3])*T13840[3][2] + Si3840[2][1]*T13840[4][2] + Si3840[2][2]*T13840[5][2] + Si3840[2][3]*T13840[6][2]) + S4038[3][2]*((ZTHUMB*Si3840[1][1] - XTHUMB*Si3840[3][1])*T13840[1][3] + (ZTHUMB*Si3840[1][2] - XTHUMB*Si3840[3][2])*T13840[2][3] + Si3840[2][1]*T13840[4][3] + Si3840[2][2]*T13840[5][3] + Si3840[2][3]*T13840[6][3]) + (ZTHUMB*S4038[1][1] - XTHUMB*S4038[1][3])*((ZTHUMB*Si3840[1][1] - XTHUMB*Si3840[3][1])*T13840[1][4] + (ZTHUMB*Si3840[1][2] - XTHUMB*Si3840[3][2])*T13840[2][4] + Si3840[2][1]*T13840[4][4] + Si3840[2][2]*T13840[5][4] + Si3840[2][3]*T13840[6][4]) + (ZTHUMB*S4038[2][1] - XTHUMB*S4038[2][3])*((ZTHUMB*Si3840[1][1] - XTHUMB*Si3840[3][1])*T13840[1][5] + (ZTHUMB*Si3840[1][2] - XTHUMB*Si3840[3][2])*T13840[2][5] + Si3840[2][1]*T13840[4][5] + Si3840[2][2]*T13840[5][5] + Si3840[2][3]*T13840[6][5]);
T3840[5][3]=(-(YTHUMB*S4038[3][1]) + XTHUMB*S4038[3][2])*((ZTHUMB*Si3840[1][3] - XTHUMB*Si3840[3][3])*T13840[3][6] + Si3840[2][1]*T13840[4][6] + Si3840[2][2]*T13840[5][6]) + S4038[1][3]*((ZTHUMB*Si3840[1][1] - XTHUMB*Si3840[3][1])*T13840[1][1] + (ZTHUMB*Si3840[1][2] - XTHUMB*Si3840[3][2])*T13840[2][1] + (ZTHUMB*Si3840[1][3] - XTHUMB*Si3840[3][3])*T13840[3][1] + Si3840[2][1]*T13840[4][1] + Si3840[2][2]*T13840[5][1] + Si3840[2][3]*T13840[6][1]) + S4038[2][3]*((ZTHUMB*Si3840[1][1] - XTHUMB*Si3840[3][1])*T13840[1][2] + (ZTHUMB*Si3840[1][2] - XTHUMB*Si3840[3][2])*T13840[2][2] + (ZTHUMB*Si3840[1][3] - XTHUMB*Si3840[3][3])*T13840[3][2] + Si3840[2][1]*T13840[4][2] + Si3840[2][2]*T13840[5][2] + Si3840[2][3]*T13840[6][2]) + S4038[3][3]*((ZTHUMB*Si3840[1][1] - XTHUMB*Si3840[3][1])*T13840[1][3] + (ZTHUMB*Si3840[1][2] - XTHUMB*Si3840[3][2])*T13840[2][3] + Si3840[2][1]*T13840[4][3] + Si3840[2][2]*T13840[5][3] + Si3840[2][3]*T13840[6][3]) + (-(YTHUMB*S4038[1][1]) + XTHUMB*S4038[1][2])*((ZTHUMB*Si3840[1][1] - XTHUMB*Si3840[3][1])*T13840[1][4] + (ZTHUMB*Si3840[1][2] - XTHUMB*Si3840[3][2])*T13840[2][4] + Si3840[2][1]*T13840[4][4] + Si3840[2][2]*T13840[5][4] + Si3840[2][3]*T13840[6][4]) + (-(YTHUMB*S4038[2][1]) + XTHUMB*S4038[2][2])*((ZTHUMB*Si3840[1][1] - XTHUMB*Si3840[3][1])*T13840[1][5] + (ZTHUMB*Si3840[1][2] - XTHUMB*Si3840[3][2])*T13840[2][5] + Si3840[2][1]*T13840[4][5] + Si3840[2][2]*T13840[5][5] + Si3840[2][3]*T13840[6][5]);
T3840[5][4]=S4038[3][1]*((ZTHUMB*Si3840[1][3] - XTHUMB*Si3840[3][3])*T13840[3][6] + Si3840[2][1]*T13840[4][6] + Si3840[2][2]*T13840[5][6]) + S4038[1][1]*((ZTHUMB*Si3840[1][1] - XTHUMB*Si3840[3][1])*T13840[1][4] + (ZTHUMB*Si3840[1][2] - XTHUMB*Si3840[3][2])*T13840[2][4] + Si3840[2][1]*T13840[4][4] + Si3840[2][2]*T13840[5][4] + Si3840[2][3]*T13840[6][4]) + S4038[2][1]*((ZTHUMB*Si3840[1][1] - XTHUMB*Si3840[3][1])*T13840[1][5] + (ZTHUMB*Si3840[1][2] - XTHUMB*Si3840[3][2])*T13840[2][5] + Si3840[2][1]*T13840[4][5] + Si3840[2][2]*T13840[5][5] + Si3840[2][3]*T13840[6][5]);
T3840[5][5]=S4038[3][2]*((ZTHUMB*Si3840[1][3] - XTHUMB*Si3840[3][3])*T13840[3][6] + Si3840[2][1]*T13840[4][6] + Si3840[2][2]*T13840[5][6]) + S4038[1][2]*((ZTHUMB*Si3840[1][1] - XTHUMB*Si3840[3][1])*T13840[1][4] + (ZTHUMB*Si3840[1][2] - XTHUMB*Si3840[3][2])*T13840[2][4] + Si3840[2][1]*T13840[4][4] + Si3840[2][2]*T13840[5][4] + Si3840[2][3]*T13840[6][4]) + S4038[2][2]*((ZTHUMB*Si3840[1][1] - XTHUMB*Si3840[3][1])*T13840[1][5] + (ZTHUMB*Si3840[1][2] - XTHUMB*Si3840[3][2])*T13840[2][5] + Si3840[2][1]*T13840[4][5] + Si3840[2][2]*T13840[5][5] + Si3840[2][3]*T13840[6][5]);
T3840[5][6]=S4038[3][3]*((ZTHUMB*Si3840[1][3] - XTHUMB*Si3840[3][3])*T13840[3][6] + Si3840[2][1]*T13840[4][6] + Si3840[2][2]*T13840[5][6]) + S4038[1][3]*((ZTHUMB*Si3840[1][1] - XTHUMB*Si3840[3][1])*T13840[1][4] + (ZTHUMB*Si3840[1][2] - XTHUMB*Si3840[3][2])*T13840[2][4] + Si3840[2][1]*T13840[4][4] + Si3840[2][2]*T13840[5][4] + Si3840[2][3]*T13840[6][4]) + S4038[2][3]*((ZTHUMB*Si3840[1][1] - XTHUMB*Si3840[3][1])*T13840[1][5] + (ZTHUMB*Si3840[1][2] - XTHUMB*Si3840[3][2])*T13840[2][5] + Si3840[2][1]*T13840[4][5] + Si3840[2][2]*T13840[5][5] + Si3840[2][3]*T13840[6][5]);

T3840[6][1]=(-(ZTHUMB*S4038[3][2]) + YTHUMB*S4038[3][3])*((-(YTHUMB*Si3840[1][3]) + XTHUMB*Si3840[2][3])*T13840[3][6] + Si3840[3][1]*T13840[4][6] + Si3840[3][2]*T13840[5][6]) + S4038[1][1]*((-(YTHUMB*Si3840[1][1]) + XTHUMB*Si3840[2][1])*T13840[1][1] + (-(YTHUMB*Si3840[1][2]) + XTHUMB*Si3840[2][2])*T13840[2][1] + (-(YTHUMB*Si3840[1][3]) + XTHUMB*Si3840[2][3])*T13840[3][1] + Si3840[3][1]*T13840[4][1] + Si3840[3][2]*T13840[5][1] + Si3840[3][3]*T13840[6][1]) + S4038[2][1]*((-(YTHUMB*Si3840[1][1]) + XTHUMB*Si3840[2][1])*T13840[1][2] + (-(YTHUMB*Si3840[1][2]) + XTHUMB*Si3840[2][2])*T13840[2][2] + (-(YTHUMB*Si3840[1][3]) + XTHUMB*Si3840[2][3])*T13840[3][2] + Si3840[3][1]*T13840[4][2] + Si3840[3][2]*T13840[5][2] + Si3840[3][3]*T13840[6][2]) + S4038[3][1]*((-(YTHUMB*Si3840[1][1]) + XTHUMB*Si3840[2][1])*T13840[1][3] + (-(YTHUMB*Si3840[1][2]) + XTHUMB*Si3840[2][2])*T13840[2][3] + Si3840[3][1]*T13840[4][3] + Si3840[3][2]*T13840[5][3] + Si3840[3][3]*T13840[6][3]) + (-(ZTHUMB*S4038[1][2]) + YTHUMB*S4038[1][3])*((-(YTHUMB*Si3840[1][1]) + XTHUMB*Si3840[2][1])*T13840[1][4] + (-(YTHUMB*Si3840[1][2]) + XTHUMB*Si3840[2][2])*T13840[2][4] + Si3840[3][1]*T13840[4][4] + Si3840[3][2]*T13840[5][4] + Si3840[3][3]*T13840[6][4]) + (-(ZTHUMB*S4038[2][2]) + YTHUMB*S4038[2][3])*((-(YTHUMB*Si3840[1][1]) + XTHUMB*Si3840[2][1])*T13840[1][5] + (-(YTHUMB*Si3840[1][2]) + XTHUMB*Si3840[2][2])*T13840[2][5] + Si3840[3][1]*T13840[4][5] + Si3840[3][2]*T13840[5][5] + Si3840[3][3]*T13840[6][5]);
T3840[6][2]=(ZTHUMB*S4038[3][1] - XTHUMB*S4038[3][3])*((-(YTHUMB*Si3840[1][3]) + XTHUMB*Si3840[2][3])*T13840[3][6] + Si3840[3][1]*T13840[4][6] + Si3840[3][2]*T13840[5][6]) + S4038[1][2]*((-(YTHUMB*Si3840[1][1]) + XTHUMB*Si3840[2][1])*T13840[1][1] + (-(YTHUMB*Si3840[1][2]) + XTHUMB*Si3840[2][2])*T13840[2][1] + (-(YTHUMB*Si3840[1][3]) + XTHUMB*Si3840[2][3])*T13840[3][1] + Si3840[3][1]*T13840[4][1] + Si3840[3][2]*T13840[5][1] + Si3840[3][3]*T13840[6][1]) + S4038[2][2]*((-(YTHUMB*Si3840[1][1]) + XTHUMB*Si3840[2][1])*T13840[1][2] + (-(YTHUMB*Si3840[1][2]) + XTHUMB*Si3840[2][2])*T13840[2][2] + (-(YTHUMB*Si3840[1][3]) + XTHUMB*Si3840[2][3])*T13840[3][2] + Si3840[3][1]*T13840[4][2] + Si3840[3][2]*T13840[5][2] + Si3840[3][3]*T13840[6][2]) + S4038[3][2]*((-(YTHUMB*Si3840[1][1]) + XTHUMB*Si3840[2][1])*T13840[1][3] + (-(YTHUMB*Si3840[1][2]) + XTHUMB*Si3840[2][2])*T13840[2][3] + Si3840[3][1]*T13840[4][3] + Si3840[3][2]*T13840[5][3] + Si3840[3][3]*T13840[6][3]) + (ZTHUMB*S4038[1][1] - XTHUMB*S4038[1][3])*((-(YTHUMB*Si3840[1][1]) + XTHUMB*Si3840[2][1])*T13840[1][4] + (-(YTHUMB*Si3840[1][2]) + XTHUMB*Si3840[2][2])*T13840[2][4] + Si3840[3][1]*T13840[4][4] + Si3840[3][2]*T13840[5][4] + Si3840[3][3]*T13840[6][4]) + (ZTHUMB*S4038[2][1] - XTHUMB*S4038[2][3])*((-(YTHUMB*Si3840[1][1]) + XTHUMB*Si3840[2][1])*T13840[1][5] + (-(YTHUMB*Si3840[1][2]) + XTHUMB*Si3840[2][2])*T13840[2][5] + Si3840[3][1]*T13840[4][5] + Si3840[3][2]*T13840[5][5] + Si3840[3][3]*T13840[6][5]);
T3840[6][3]=(-(YTHUMB*S4038[3][1]) + XTHUMB*S4038[3][2])*((-(YTHUMB*Si3840[1][3]) + XTHUMB*Si3840[2][3])*T13840[3][6] + Si3840[3][1]*T13840[4][6] + Si3840[3][2]*T13840[5][6]) + S4038[1][3]*((-(YTHUMB*Si3840[1][1]) + XTHUMB*Si3840[2][1])*T13840[1][1] + (-(YTHUMB*Si3840[1][2]) + XTHUMB*Si3840[2][2])*T13840[2][1] + (-(YTHUMB*Si3840[1][3]) + XTHUMB*Si3840[2][3])*T13840[3][1] + Si3840[3][1]*T13840[4][1] + Si3840[3][2]*T13840[5][1] + Si3840[3][3]*T13840[6][1]) + S4038[2][3]*((-(YTHUMB*Si3840[1][1]) + XTHUMB*Si3840[2][1])*T13840[1][2] + (-(YTHUMB*Si3840[1][2]) + XTHUMB*Si3840[2][2])*T13840[2][2] + (-(YTHUMB*Si3840[1][3]) + XTHUMB*Si3840[2][3])*T13840[3][2] + Si3840[3][1]*T13840[4][2] + Si3840[3][2]*T13840[5][2] + Si3840[3][3]*T13840[6][2]) + S4038[3][3]*((-(YTHUMB*Si3840[1][1]) + XTHUMB*Si3840[2][1])*T13840[1][3] + (-(YTHUMB*Si3840[1][2]) + XTHUMB*Si3840[2][2])*T13840[2][3] + Si3840[3][1]*T13840[4][3] + Si3840[3][2]*T13840[5][3] + Si3840[3][3]*T13840[6][3]) + (-(YTHUMB*S4038[1][1]) + XTHUMB*S4038[1][2])*((-(YTHUMB*Si3840[1][1]) + XTHUMB*Si3840[2][1])*T13840[1][4] + (-(YTHUMB*Si3840[1][2]) + XTHUMB*Si3840[2][2])*T13840[2][4] + Si3840[3][1]*T13840[4][4] + Si3840[3][2]*T13840[5][4] + Si3840[3][3]*T13840[6][4]) + (-(YTHUMB*S4038[2][1]) + XTHUMB*S4038[2][2])*((-(YTHUMB*Si3840[1][1]) + XTHUMB*Si3840[2][1])*T13840[1][5] + (-(YTHUMB*Si3840[1][2]) + XTHUMB*Si3840[2][2])*T13840[2][5] + Si3840[3][1]*T13840[4][5] + Si3840[3][2]*T13840[5][5] + Si3840[3][3]*T13840[6][5]);
T3840[6][4]=S4038[3][1]*((-(YTHUMB*Si3840[1][3]) + XTHUMB*Si3840[2][3])*T13840[3][6] + Si3840[3][1]*T13840[4][6] + Si3840[3][2]*T13840[5][6]) + S4038[1][1]*((-(YTHUMB*Si3840[1][1]) + XTHUMB*Si3840[2][1])*T13840[1][4] + (-(YTHUMB*Si3840[1][2]) + XTHUMB*Si3840[2][2])*T13840[2][4] + Si3840[3][1]*T13840[4][4] + Si3840[3][2]*T13840[5][4] + Si3840[3][3]*T13840[6][4]) + S4038[2][1]*((-(YTHUMB*Si3840[1][1]) + XTHUMB*Si3840[2][1])*T13840[1][5] + (-(YTHUMB*Si3840[1][2]) + XTHUMB*Si3840[2][2])*T13840[2][5] + Si3840[3][1]*T13840[4][5] + Si3840[3][2]*T13840[5][5] + Si3840[3][3]*T13840[6][5]);
T3840[6][5]=S4038[3][2]*((-(YTHUMB*Si3840[1][3]) + XTHUMB*Si3840[2][3])*T13840[3][6] + Si3840[3][1]*T13840[4][6] + Si3840[3][2]*T13840[5][6]) + S4038[1][2]*((-(YTHUMB*Si3840[1][1]) + XTHUMB*Si3840[2][1])*T13840[1][4] + (-(YTHUMB*Si3840[1][2]) + XTHUMB*Si3840[2][2])*T13840[2][4] + Si3840[3][1]*T13840[4][4] + Si3840[3][2]*T13840[5][4] + Si3840[3][3]*T13840[6][4]) + S4038[2][2]*((-(YTHUMB*Si3840[1][1]) + XTHUMB*Si3840[2][1])*T13840[1][5] + (-(YTHUMB*Si3840[1][2]) + XTHUMB*Si3840[2][2])*T13840[2][5] + Si3840[3][1]*T13840[4][5] + Si3840[3][2]*T13840[5][5] + Si3840[3][3]*T13840[6][5]);
T3840[6][6]=S4038[3][3]*((-(YTHUMB*Si3840[1][3]) + XTHUMB*Si3840[2][3])*T13840[3][6] + Si3840[3][1]*T13840[4][6] + Si3840[3][2]*T13840[5][6]) + S4038[1][3]*((-(YTHUMB*Si3840[1][1]) + XTHUMB*Si3840[2][1])*T13840[1][4] + (-(YTHUMB*Si3840[1][2]) + XTHUMB*Si3840[2][2])*T13840[2][4] + Si3840[3][1]*T13840[4][4] + Si3840[3][2]*T13840[5][4] + Si3840[3][3]*T13840[6][4]) + S4038[2][3]*((-(YTHUMB*Si3840[1][1]) + XTHUMB*Si3840[2][1])*T13840[1][5] + (-(YTHUMB*Si3840[1][2]) + XTHUMB*Si3840[2][2])*T13840[2][5] + Si3840[3][1]*T13840[4][5] + Si3840[3][2]*T13840[5][5] + Si3840[3][3]*T13840[6][5]);



}


void
hermes_InvDynArtfunc65(void)
      {
JA39[1][2]=eff[1].mcm[3];
JA39[1][3]=-eff[1].mcm[2];
JA39[1][4]=eff[1].m;

JA39[2][1]=-eff[1].mcm[3];
JA39[2][3]=eff[1].mcm[1];
JA39[2][5]=eff[1].m;

JA39[3][1]=eff[1].mcm[2];
JA39[3][2]=-eff[1].mcm[1];
JA39[3][6]=eff[1].m;

JA39[4][5]=-eff[1].mcm[3];
JA39[4][6]=eff[1].mcm[2];

JA39[5][4]=eff[1].mcm[3];
JA39[5][6]=-eff[1].mcm[1];

JA39[6][4]=-eff[1].mcm[2];
JA39[6][5]=eff[1].mcm[1];


T13839[1][2]=JA39[1][2];
T13839[1][3]=JA39[1][3];
T13839[1][4]=JA39[1][4];

T13839[2][1]=JA39[2][1];
T13839[2][3]=JA39[2][3];
T13839[2][5]=JA39[2][5];

T13839[3][1]=JA39[3][1];
T13839[3][2]=JA39[3][2];
T13839[3][6]=JA39[3][6];

T13839[4][5]=JA39[4][5];
T13839[4][6]=JA39[4][6];

T13839[5][4]=JA39[5][4];
T13839[5][6]=JA39[5][6];

T13839[6][4]=JA39[6][4];
T13839[6][5]=JA39[6][5];


T3839[1][1]=(-(eff[1].x[3]*S3938[1][2]) + eff[1].x[2]*S3938[1][3])*Si3839[1][1]*T13839[1][4] + S3938[3][1]*(Si3839[1][1]*T13839[1][3] + Si3839[1][2]*T13839[2][3]) + (-(eff[1].x[3]*S3938[2][2]) + eff[1].x[2]*S3938[2][3])*Si3839[1][2]*T13839[2][5] + S3938[1][1]*(Si3839[1][2]*T13839[2][1] + Si3839[1][3]*T13839[3][1]) + S3938[2][1]*(Si3839[1][1]*T13839[1][2] + Si3839[1][3]*T13839[3][2]) + (-(eff[1].x[3]*S3938[3][2]) + eff[1].x[2]*S3938[3][3])*Si3839[1][3]*T13839[3][6];
T3839[1][2]=(eff[1].x[3]*S3938[1][1] - eff[1].x[1]*S3938[1][3])*Si3839[1][1]*T13839[1][4] + S3938[3][2]*(Si3839[1][1]*T13839[1][3] + Si3839[1][2]*T13839[2][3]) + (eff[1].x[3]*S3938[2][1] - eff[1].x[1]*S3938[2][3])*Si3839[1][2]*T13839[2][5] + S3938[1][2]*(Si3839[1][2]*T13839[2][1] + Si3839[1][3]*T13839[3][1]) + S3938[2][2]*(Si3839[1][1]*T13839[1][2] + Si3839[1][3]*T13839[3][2]) + (eff[1].x[3]*S3938[3][1] - eff[1].x[1]*S3938[3][3])*Si3839[1][3]*T13839[3][6];
T3839[1][3]=(-(eff[1].x[2]*S3938[1][1]) + eff[1].x[1]*S3938[1][2])*Si3839[1][1]*T13839[1][4] + S3938[3][3]*(Si3839[1][1]*T13839[1][3] + Si3839[1][2]*T13839[2][3]) + (-(eff[1].x[2]*S3938[2][1]) + eff[1].x[1]*S3938[2][2])*Si3839[1][2]*T13839[2][5] + S3938[1][3]*(Si3839[1][2]*T13839[2][1] + Si3839[1][3]*T13839[3][1]) + S3938[2][3]*(Si3839[1][1]*T13839[1][2] + Si3839[1][3]*T13839[3][2]) + (-(eff[1].x[2]*S3938[3][1]) + eff[1].x[1]*S3938[3][2])*Si3839[1][3]*T13839[3][6];
T3839[1][4]=S3938[1][1]*Si3839[1][1]*T13839[1][4] + S3938[2][1]*Si3839[1][2]*T13839[2][5] + S3938[3][1]*Si3839[1][3]*T13839[3][6];
T3839[1][5]=S3938[1][2]*Si3839[1][1]*T13839[1][4] + S3938[2][2]*Si3839[1][2]*T13839[2][5] + S3938[3][2]*Si3839[1][3]*T13839[3][6];
T3839[1][6]=S3938[1][3]*Si3839[1][1]*T13839[1][4] + S3938[2][3]*Si3839[1][2]*T13839[2][5] + S3938[3][3]*Si3839[1][3]*T13839[3][6];

T3839[2][1]=(-(eff[1].x[3]*S3938[1][2]) + eff[1].x[2]*S3938[1][3])*Si3839[2][1]*T13839[1][4] + S3938[3][1]*(Si3839[2][1]*T13839[1][3] + Si3839[2][2]*T13839[2][3]) + (-(eff[1].x[3]*S3938[2][2]) + eff[1].x[2]*S3938[2][3])*Si3839[2][2]*T13839[2][5] + S3938[1][1]*(Si3839[2][2]*T13839[2][1] + Si3839[2][3]*T13839[3][1]) + S3938[2][1]*(Si3839[2][1]*T13839[1][2] + Si3839[2][3]*T13839[3][2]) + (-(eff[1].x[3]*S3938[3][2]) + eff[1].x[2]*S3938[3][3])*Si3839[2][3]*T13839[3][6];
T3839[2][2]=(eff[1].x[3]*S3938[1][1] - eff[1].x[1]*S3938[1][3])*Si3839[2][1]*T13839[1][4] + S3938[3][2]*(Si3839[2][1]*T13839[1][3] + Si3839[2][2]*T13839[2][3]) + (eff[1].x[3]*S3938[2][1] - eff[1].x[1]*S3938[2][3])*Si3839[2][2]*T13839[2][5] + S3938[1][2]*(Si3839[2][2]*T13839[2][1] + Si3839[2][3]*T13839[3][1]) + S3938[2][2]*(Si3839[2][1]*T13839[1][2] + Si3839[2][3]*T13839[3][2]) + (eff[1].x[3]*S3938[3][1] - eff[1].x[1]*S3938[3][3])*Si3839[2][3]*T13839[3][6];
T3839[2][3]=(-(eff[1].x[2]*S3938[1][1]) + eff[1].x[1]*S3938[1][2])*Si3839[2][1]*T13839[1][4] + S3938[3][3]*(Si3839[2][1]*T13839[1][3] + Si3839[2][2]*T13839[2][3]) + (-(eff[1].x[2]*S3938[2][1]) + eff[1].x[1]*S3938[2][2])*Si3839[2][2]*T13839[2][5] + S3938[1][3]*(Si3839[2][2]*T13839[2][1] + Si3839[2][3]*T13839[3][1]) + S3938[2][3]*(Si3839[2][1]*T13839[1][2] + Si3839[2][3]*T13839[3][2]) + (-(eff[1].x[2]*S3938[3][1]) + eff[1].x[1]*S3938[3][2])*Si3839[2][3]*T13839[3][6];
T3839[2][4]=S3938[1][1]*Si3839[2][1]*T13839[1][4] + S3938[2][1]*Si3839[2][2]*T13839[2][5] + S3938[3][1]*Si3839[2][3]*T13839[3][6];
T3839[2][5]=S3938[1][2]*Si3839[2][1]*T13839[1][4] + S3938[2][2]*Si3839[2][2]*T13839[2][5] + S3938[3][2]*Si3839[2][3]*T13839[3][6];
T3839[2][6]=S3938[1][3]*Si3839[2][1]*T13839[1][4] + S3938[2][3]*Si3839[2][2]*T13839[2][5] + S3938[3][3]*Si3839[2][3]*T13839[3][6];

T3839[3][1]=(-(eff[1].x[3]*S3938[1][2]) + eff[1].x[2]*S3938[1][3])*Si3839[3][1]*T13839[1][4] + S3938[3][1]*(Si3839[3][1]*T13839[1][3] + Si3839[3][2]*T13839[2][3]) + (-(eff[1].x[3]*S3938[2][2]) + eff[1].x[2]*S3938[2][3])*Si3839[3][2]*T13839[2][5] + S3938[1][1]*(Si3839[3][2]*T13839[2][1] + Si3839[3][3]*T13839[3][1]) + S3938[2][1]*(Si3839[3][1]*T13839[1][2] + Si3839[3][3]*T13839[3][2]) + (-(eff[1].x[3]*S3938[3][2]) + eff[1].x[2]*S3938[3][3])*Si3839[3][3]*T13839[3][6];
T3839[3][2]=(eff[1].x[3]*S3938[1][1] - eff[1].x[1]*S3938[1][3])*Si3839[3][1]*T13839[1][4] + S3938[3][2]*(Si3839[3][1]*T13839[1][3] + Si3839[3][2]*T13839[2][3]) + (eff[1].x[3]*S3938[2][1] - eff[1].x[1]*S3938[2][3])*Si3839[3][2]*T13839[2][5] + S3938[1][2]*(Si3839[3][2]*T13839[2][1] + Si3839[3][3]*T13839[3][1]) + S3938[2][2]*(Si3839[3][1]*T13839[1][2] + Si3839[3][3]*T13839[3][2]) + (eff[1].x[3]*S3938[3][1] - eff[1].x[1]*S3938[3][3])*Si3839[3][3]*T13839[3][6];
T3839[3][3]=(-(eff[1].x[2]*S3938[1][1]) + eff[1].x[1]*S3938[1][2])*Si3839[3][1]*T13839[1][4] + S3938[3][3]*(Si3839[3][1]*T13839[1][3] + Si3839[3][2]*T13839[2][3]) + (-(eff[1].x[2]*S3938[2][1]) + eff[1].x[1]*S3938[2][2])*Si3839[3][2]*T13839[2][5] + S3938[1][3]*(Si3839[3][2]*T13839[2][1] + Si3839[3][3]*T13839[3][1]) + S3938[2][3]*(Si3839[3][1]*T13839[1][2] + Si3839[3][3]*T13839[3][2]) + (-(eff[1].x[2]*S3938[3][1]) + eff[1].x[1]*S3938[3][2])*Si3839[3][3]*T13839[3][6];
T3839[3][4]=S3938[1][1]*Si3839[3][1]*T13839[1][4] + S3938[2][1]*Si3839[3][2]*T13839[2][5] + S3938[3][1]*Si3839[3][3]*T13839[3][6];
T3839[3][5]=S3938[1][2]*Si3839[3][1]*T13839[1][4] + S3938[2][2]*Si3839[3][2]*T13839[2][5] + S3938[3][2]*Si3839[3][3]*T13839[3][6];
T3839[3][6]=S3938[1][3]*Si3839[3][1]*T13839[1][4] + S3938[2][3]*Si3839[3][2]*T13839[2][5] + S3938[3][3]*Si3839[3][3]*T13839[3][6];

T3839[4][1]=S3938[3][1]*((-(eff[1].x[3]*Si3839[2][1]) + eff[1].x[2]*Si3839[3][1])*T13839[1][3] + (-(eff[1].x[3]*Si3839[2][2]) + eff[1].x[2]*Si3839[3][2])*T13839[2][3]) + S3938[1][1]*((-(eff[1].x[3]*Si3839[2][2]) + eff[1].x[2]*Si3839[3][2])*T13839[2][1] + (-(eff[1].x[3]*Si3839[2][3]) + eff[1].x[2]*Si3839[3][3])*T13839[3][1]) + S3938[2][1]*((-(eff[1].x[3]*Si3839[2][1]) + eff[1].x[2]*Si3839[3][1])*T13839[1][2] + (-(eff[1].x[3]*Si3839[2][3]) + eff[1].x[2]*Si3839[3][3])*T13839[3][2]) + (-(eff[1].x[3]*S3938[3][2]) + eff[1].x[2]*S3938[3][3])*((-(eff[1].x[3]*Si3839[2][3]) + eff[1].x[2]*Si3839[3][3])*T13839[3][6] + Si3839[1][1]*T13839[4][6] + Si3839[1][2]*T13839[5][6]) + (-(eff[1].x[3]*S3938[1][2]) + eff[1].x[2]*S3938[1][3])*((-(eff[1].x[3]*Si3839[2][1]) + eff[1].x[2]*Si3839[3][1])*T13839[1][4] + Si3839[1][2]*T13839[5][4] + Si3839[1][3]*T13839[6][4]) + (-(eff[1].x[3]*S3938[2][2]) + eff[1].x[2]*S3938[2][3])*((-(eff[1].x[3]*Si3839[2][2]) + eff[1].x[2]*Si3839[3][2])*T13839[2][5] + Si3839[1][1]*T13839[4][5] + Si3839[1][3]*T13839[6][5]);
T3839[4][2]=S3938[3][2]*((-(eff[1].x[3]*Si3839[2][1]) + eff[1].x[2]*Si3839[3][1])*T13839[1][3] + (-(eff[1].x[3]*Si3839[2][2]) + eff[1].x[2]*Si3839[3][2])*T13839[2][3]) + S3938[1][2]*((-(eff[1].x[3]*Si3839[2][2]) + eff[1].x[2]*Si3839[3][2])*T13839[2][1] + (-(eff[1].x[3]*Si3839[2][3]) + eff[1].x[2]*Si3839[3][3])*T13839[3][1]) + S3938[2][2]*((-(eff[1].x[3]*Si3839[2][1]) + eff[1].x[2]*Si3839[3][1])*T13839[1][2] + (-(eff[1].x[3]*Si3839[2][3]) + eff[1].x[2]*Si3839[3][3])*T13839[3][2]) + (eff[1].x[3]*S3938[3][1] - eff[1].x[1]*S3938[3][3])*((-(eff[1].x[3]*Si3839[2][3]) + eff[1].x[2]*Si3839[3][3])*T13839[3][6] + Si3839[1][1]*T13839[4][6] + Si3839[1][2]*T13839[5][6]) + (eff[1].x[3]*S3938[1][1] - eff[1].x[1]*S3938[1][3])*((-(eff[1].x[3]*Si3839[2][1]) + eff[1].x[2]*Si3839[3][1])*T13839[1][4] + Si3839[1][2]*T13839[5][4] + Si3839[1][3]*T13839[6][4]) + (eff[1].x[3]*S3938[2][1] - eff[1].x[1]*S3938[2][3])*((-(eff[1].x[3]*Si3839[2][2]) + eff[1].x[2]*Si3839[3][2])*T13839[2][5] + Si3839[1][1]*T13839[4][5] + Si3839[1][3]*T13839[6][5]);
T3839[4][3]=S3938[3][3]*((-(eff[1].x[3]*Si3839[2][1]) + eff[1].x[2]*Si3839[3][1])*T13839[1][3] + (-(eff[1].x[3]*Si3839[2][2]) + eff[1].x[2]*Si3839[3][2])*T13839[2][3]) + S3938[1][3]*((-(eff[1].x[3]*Si3839[2][2]) + eff[1].x[2]*Si3839[3][2])*T13839[2][1] + (-(eff[1].x[3]*Si3839[2][3]) + eff[1].x[2]*Si3839[3][3])*T13839[3][1]) + S3938[2][3]*((-(eff[1].x[3]*Si3839[2][1]) + eff[1].x[2]*Si3839[3][1])*T13839[1][2] + (-(eff[1].x[3]*Si3839[2][3]) + eff[1].x[2]*Si3839[3][3])*T13839[3][2]) + (-(eff[1].x[2]*S3938[3][1]) + eff[1].x[1]*S3938[3][2])*((-(eff[1].x[3]*Si3839[2][3]) + eff[1].x[2]*Si3839[3][3])*T13839[3][6] + Si3839[1][1]*T13839[4][6] + Si3839[1][2]*T13839[5][6]) + (-(eff[1].x[2]*S3938[1][1]) + eff[1].x[1]*S3938[1][2])*((-(eff[1].x[3]*Si3839[2][1]) + eff[1].x[2]*Si3839[3][1])*T13839[1][4] + Si3839[1][2]*T13839[5][4] + Si3839[1][3]*T13839[6][4]) + (-(eff[1].x[2]*S3938[2][1]) + eff[1].x[1]*S3938[2][2])*((-(eff[1].x[3]*Si3839[2][2]) + eff[1].x[2]*Si3839[3][2])*T13839[2][5] + Si3839[1][1]*T13839[4][5] + Si3839[1][3]*T13839[6][5]);
T3839[4][4]=S3938[3][1]*((-(eff[1].x[3]*Si3839[2][3]) + eff[1].x[2]*Si3839[3][3])*T13839[3][6] + Si3839[1][1]*T13839[4][6] + Si3839[1][2]*T13839[5][6]) + S3938[1][1]*((-(eff[1].x[3]*Si3839[2][1]) + eff[1].x[2]*Si3839[3][1])*T13839[1][4] + Si3839[1][2]*T13839[5][4] + Si3839[1][3]*T13839[6][4]) + S3938[2][1]*((-(eff[1].x[3]*Si3839[2][2]) + eff[1].x[2]*Si3839[3][2])*T13839[2][5] + Si3839[1][1]*T13839[4][5] + Si3839[1][3]*T13839[6][5]);
T3839[4][5]=S3938[3][2]*((-(eff[1].x[3]*Si3839[2][3]) + eff[1].x[2]*Si3839[3][3])*T13839[3][6] + Si3839[1][1]*T13839[4][6] + Si3839[1][2]*T13839[5][6]) + S3938[1][2]*((-(eff[1].x[3]*Si3839[2][1]) + eff[1].x[2]*Si3839[3][1])*T13839[1][4] + Si3839[1][2]*T13839[5][4] + Si3839[1][3]*T13839[6][4]) + S3938[2][2]*((-(eff[1].x[3]*Si3839[2][2]) + eff[1].x[2]*Si3839[3][2])*T13839[2][5] + Si3839[1][1]*T13839[4][5] + Si3839[1][3]*T13839[6][5]);
T3839[4][6]=S3938[3][3]*((-(eff[1].x[3]*Si3839[2][3]) + eff[1].x[2]*Si3839[3][3])*T13839[3][6] + Si3839[1][1]*T13839[4][6] + Si3839[1][2]*T13839[5][6]) + S3938[1][3]*((-(eff[1].x[3]*Si3839[2][1]) + eff[1].x[2]*Si3839[3][1])*T13839[1][4] + Si3839[1][2]*T13839[5][4] + Si3839[1][3]*T13839[6][4]) + S3938[2][3]*((-(eff[1].x[3]*Si3839[2][2]) + eff[1].x[2]*Si3839[3][2])*T13839[2][5] + Si3839[1][1]*T13839[4][5] + Si3839[1][3]*T13839[6][5]);

T3839[5][1]=S3938[3][1]*((eff[1].x[3]*Si3839[1][1] - eff[1].x[1]*Si3839[3][1])*T13839[1][3] + (eff[1].x[3]*Si3839[1][2] - eff[1].x[1]*Si3839[3][2])*T13839[2][3]) + S3938[1][1]*((eff[1].x[3]*Si3839[1][2] - eff[1].x[1]*Si3839[3][2])*T13839[2][1] + (eff[1].x[3]*Si3839[1][3] - eff[1].x[1]*Si3839[3][3])*T13839[3][1]) + S3938[2][1]*((eff[1].x[3]*Si3839[1][1] - eff[1].x[1]*Si3839[3][1])*T13839[1][2] + (eff[1].x[3]*Si3839[1][3] - eff[1].x[1]*Si3839[3][3])*T13839[3][2]) + (-(eff[1].x[3]*S3938[3][2]) + eff[1].x[2]*S3938[3][3])*((eff[1].x[3]*Si3839[1][3] - eff[1].x[1]*Si3839[3][3])*T13839[3][6] + Si3839[2][1]*T13839[4][6] + Si3839[2][2]*T13839[5][6]) + (-(eff[1].x[3]*S3938[1][2]) + eff[1].x[2]*S3938[1][3])*((eff[1].x[3]*Si3839[1][1] - eff[1].x[1]*Si3839[3][1])*T13839[1][4] + Si3839[2][2]*T13839[5][4] + Si3839[2][3]*T13839[6][4]) + (-(eff[1].x[3]*S3938[2][2]) + eff[1].x[2]*S3938[2][3])*((eff[1].x[3]*Si3839[1][2] - eff[1].x[1]*Si3839[3][2])*T13839[2][5] + Si3839[2][1]*T13839[4][5] + Si3839[2][3]*T13839[6][5]);
T3839[5][2]=S3938[3][2]*((eff[1].x[3]*Si3839[1][1] - eff[1].x[1]*Si3839[3][1])*T13839[1][3] + (eff[1].x[3]*Si3839[1][2] - eff[1].x[1]*Si3839[3][2])*T13839[2][3]) + S3938[1][2]*((eff[1].x[3]*Si3839[1][2] - eff[1].x[1]*Si3839[3][2])*T13839[2][1] + (eff[1].x[3]*Si3839[1][3] - eff[1].x[1]*Si3839[3][3])*T13839[3][1]) + S3938[2][2]*((eff[1].x[3]*Si3839[1][1] - eff[1].x[1]*Si3839[3][1])*T13839[1][2] + (eff[1].x[3]*Si3839[1][3] - eff[1].x[1]*Si3839[3][3])*T13839[3][2]) + (eff[1].x[3]*S3938[3][1] - eff[1].x[1]*S3938[3][3])*((eff[1].x[3]*Si3839[1][3] - eff[1].x[1]*Si3839[3][3])*T13839[3][6] + Si3839[2][1]*T13839[4][6] + Si3839[2][2]*T13839[5][6]) + (eff[1].x[3]*S3938[1][1] - eff[1].x[1]*S3938[1][3])*((eff[1].x[3]*Si3839[1][1] - eff[1].x[1]*Si3839[3][1])*T13839[1][4] + Si3839[2][2]*T13839[5][4] + Si3839[2][3]*T13839[6][4]) + (eff[1].x[3]*S3938[2][1] - eff[1].x[1]*S3938[2][3])*((eff[1].x[3]*Si3839[1][2] - eff[1].x[1]*Si3839[3][2])*T13839[2][5] + Si3839[2][1]*T13839[4][5] + Si3839[2][3]*T13839[6][5]);
T3839[5][3]=S3938[3][3]*((eff[1].x[3]*Si3839[1][1] - eff[1].x[1]*Si3839[3][1])*T13839[1][3] + (eff[1].x[3]*Si3839[1][2] - eff[1].x[1]*Si3839[3][2])*T13839[2][3]) + S3938[1][3]*((eff[1].x[3]*Si3839[1][2] - eff[1].x[1]*Si3839[3][2])*T13839[2][1] + (eff[1].x[3]*Si3839[1][3] - eff[1].x[1]*Si3839[3][3])*T13839[3][1]) + S3938[2][3]*((eff[1].x[3]*Si3839[1][1] - eff[1].x[1]*Si3839[3][1])*T13839[1][2] + (eff[1].x[3]*Si3839[1][3] - eff[1].x[1]*Si3839[3][3])*T13839[3][2]) + (-(eff[1].x[2]*S3938[3][1]) + eff[1].x[1]*S3938[3][2])*((eff[1].x[3]*Si3839[1][3] - eff[1].x[1]*Si3839[3][3])*T13839[3][6] + Si3839[2][1]*T13839[4][6] + Si3839[2][2]*T13839[5][6]) + (-(eff[1].x[2]*S3938[1][1]) + eff[1].x[1]*S3938[1][2])*((eff[1].x[3]*Si3839[1][1] - eff[1].x[1]*Si3839[3][1])*T13839[1][4] + Si3839[2][2]*T13839[5][4] + Si3839[2][3]*T13839[6][4]) + (-(eff[1].x[2]*S3938[2][1]) + eff[1].x[1]*S3938[2][2])*((eff[1].x[3]*Si3839[1][2] - eff[1].x[1]*Si3839[3][2])*T13839[2][5] + Si3839[2][1]*T13839[4][5] + Si3839[2][3]*T13839[6][5]);
T3839[5][4]=S3938[3][1]*((eff[1].x[3]*Si3839[1][3] - eff[1].x[1]*Si3839[3][3])*T13839[3][6] + Si3839[2][1]*T13839[4][6] + Si3839[2][2]*T13839[5][6]) + S3938[1][1]*((eff[1].x[3]*Si3839[1][1] - eff[1].x[1]*Si3839[3][1])*T13839[1][4] + Si3839[2][2]*T13839[5][4] + Si3839[2][3]*T13839[6][4]) + S3938[2][1]*((eff[1].x[3]*Si3839[1][2] - eff[1].x[1]*Si3839[3][2])*T13839[2][5] + Si3839[2][1]*T13839[4][5] + Si3839[2][3]*T13839[6][5]);
T3839[5][5]=S3938[3][2]*((eff[1].x[3]*Si3839[1][3] - eff[1].x[1]*Si3839[3][3])*T13839[3][6] + Si3839[2][1]*T13839[4][6] + Si3839[2][2]*T13839[5][6]) + S3938[1][2]*((eff[1].x[3]*Si3839[1][1] - eff[1].x[1]*Si3839[3][1])*T13839[1][4] + Si3839[2][2]*T13839[5][4] + Si3839[2][3]*T13839[6][4]) + S3938[2][2]*((eff[1].x[3]*Si3839[1][2] - eff[1].x[1]*Si3839[3][2])*T13839[2][5] + Si3839[2][1]*T13839[4][5] + Si3839[2][3]*T13839[6][5]);
T3839[5][6]=S3938[3][3]*((eff[1].x[3]*Si3839[1][3] - eff[1].x[1]*Si3839[3][3])*T13839[3][6] + Si3839[2][1]*T13839[4][6] + Si3839[2][2]*T13839[5][6]) + S3938[1][3]*((eff[1].x[3]*Si3839[1][1] - eff[1].x[1]*Si3839[3][1])*T13839[1][4] + Si3839[2][2]*T13839[5][4] + Si3839[2][3]*T13839[6][4]) + S3938[2][3]*((eff[1].x[3]*Si3839[1][2] - eff[1].x[1]*Si3839[3][2])*T13839[2][5] + Si3839[2][1]*T13839[4][5] + Si3839[2][3]*T13839[6][5]);

T3839[6][1]=S3938[3][1]*((-(eff[1].x[2]*Si3839[1][1]) + eff[1].x[1]*Si3839[2][1])*T13839[1][3] + (-(eff[1].x[2]*Si3839[1][2]) + eff[1].x[1]*Si3839[2][2])*T13839[2][3]) + S3938[1][1]*((-(eff[1].x[2]*Si3839[1][2]) + eff[1].x[1]*Si3839[2][2])*T13839[2][1] + (-(eff[1].x[2]*Si3839[1][3]) + eff[1].x[1]*Si3839[2][3])*T13839[3][1]) + S3938[2][1]*((-(eff[1].x[2]*Si3839[1][1]) + eff[1].x[1]*Si3839[2][1])*T13839[1][2] + (-(eff[1].x[2]*Si3839[1][3]) + eff[1].x[1]*Si3839[2][3])*T13839[3][2]) + (-(eff[1].x[3]*S3938[3][2]) + eff[1].x[2]*S3938[3][3])*((-(eff[1].x[2]*Si3839[1][3]) + eff[1].x[1]*Si3839[2][3])*T13839[3][6] + Si3839[3][1]*T13839[4][6] + Si3839[3][2]*T13839[5][6]) + (-(eff[1].x[3]*S3938[1][2]) + eff[1].x[2]*S3938[1][3])*((-(eff[1].x[2]*Si3839[1][1]) + eff[1].x[1]*Si3839[2][1])*T13839[1][4] + Si3839[3][2]*T13839[5][4] + Si3839[3][3]*T13839[6][4]) + (-(eff[1].x[3]*S3938[2][2]) + eff[1].x[2]*S3938[2][3])*((-(eff[1].x[2]*Si3839[1][2]) + eff[1].x[1]*Si3839[2][2])*T13839[2][5] + Si3839[3][1]*T13839[4][5] + Si3839[3][3]*T13839[6][5]);
T3839[6][2]=S3938[3][2]*((-(eff[1].x[2]*Si3839[1][1]) + eff[1].x[1]*Si3839[2][1])*T13839[1][3] + (-(eff[1].x[2]*Si3839[1][2]) + eff[1].x[1]*Si3839[2][2])*T13839[2][3]) + S3938[1][2]*((-(eff[1].x[2]*Si3839[1][2]) + eff[1].x[1]*Si3839[2][2])*T13839[2][1] + (-(eff[1].x[2]*Si3839[1][3]) + eff[1].x[1]*Si3839[2][3])*T13839[3][1]) + S3938[2][2]*((-(eff[1].x[2]*Si3839[1][1]) + eff[1].x[1]*Si3839[2][1])*T13839[1][2] + (-(eff[1].x[2]*Si3839[1][3]) + eff[1].x[1]*Si3839[2][3])*T13839[3][2]) + (eff[1].x[3]*S3938[3][1] - eff[1].x[1]*S3938[3][3])*((-(eff[1].x[2]*Si3839[1][3]) + eff[1].x[1]*Si3839[2][3])*T13839[3][6] + Si3839[3][1]*T13839[4][6] + Si3839[3][2]*T13839[5][6]) + (eff[1].x[3]*S3938[1][1] - eff[1].x[1]*S3938[1][3])*((-(eff[1].x[2]*Si3839[1][1]) + eff[1].x[1]*Si3839[2][1])*T13839[1][4] + Si3839[3][2]*T13839[5][4] + Si3839[3][3]*T13839[6][4]) + (eff[1].x[3]*S3938[2][1] - eff[1].x[1]*S3938[2][3])*((-(eff[1].x[2]*Si3839[1][2]) + eff[1].x[1]*Si3839[2][2])*T13839[2][5] + Si3839[3][1]*T13839[4][5] + Si3839[3][3]*T13839[6][5]);
T3839[6][3]=S3938[3][3]*((-(eff[1].x[2]*Si3839[1][1]) + eff[1].x[1]*Si3839[2][1])*T13839[1][3] + (-(eff[1].x[2]*Si3839[1][2]) + eff[1].x[1]*Si3839[2][2])*T13839[2][3]) + S3938[1][3]*((-(eff[1].x[2]*Si3839[1][2]) + eff[1].x[1]*Si3839[2][2])*T13839[2][1] + (-(eff[1].x[2]*Si3839[1][3]) + eff[1].x[1]*Si3839[2][3])*T13839[3][1]) + S3938[2][3]*((-(eff[1].x[2]*Si3839[1][1]) + eff[1].x[1]*Si3839[2][1])*T13839[1][2] + (-(eff[1].x[2]*Si3839[1][3]) + eff[1].x[1]*Si3839[2][3])*T13839[3][2]) + (-(eff[1].x[2]*S3938[3][1]) + eff[1].x[1]*S3938[3][2])*((-(eff[1].x[2]*Si3839[1][3]) + eff[1].x[1]*Si3839[2][3])*T13839[3][6] + Si3839[3][1]*T13839[4][6] + Si3839[3][2]*T13839[5][6]) + (-(eff[1].x[2]*S3938[1][1]) + eff[1].x[1]*S3938[1][2])*((-(eff[1].x[2]*Si3839[1][1]) + eff[1].x[1]*Si3839[2][1])*T13839[1][4] + Si3839[3][2]*T13839[5][4] + Si3839[3][3]*T13839[6][4]) + (-(eff[1].x[2]*S3938[2][1]) + eff[1].x[1]*S3938[2][2])*((-(eff[1].x[2]*Si3839[1][2]) + eff[1].x[1]*Si3839[2][2])*T13839[2][5] + Si3839[3][1]*T13839[4][5] + Si3839[3][3]*T13839[6][5]);
T3839[6][4]=S3938[3][1]*((-(eff[1].x[2]*Si3839[1][3]) + eff[1].x[1]*Si3839[2][3])*T13839[3][6] + Si3839[3][1]*T13839[4][6] + Si3839[3][2]*T13839[5][6]) + S3938[1][1]*((-(eff[1].x[2]*Si3839[1][1]) + eff[1].x[1]*Si3839[2][1])*T13839[1][4] + Si3839[3][2]*T13839[5][4] + Si3839[3][3]*T13839[6][4]) + S3938[2][1]*((-(eff[1].x[2]*Si3839[1][2]) + eff[1].x[1]*Si3839[2][2])*T13839[2][5] + Si3839[3][1]*T13839[4][5] + Si3839[3][3]*T13839[6][5]);
T3839[6][5]=S3938[3][2]*((-(eff[1].x[2]*Si3839[1][3]) + eff[1].x[1]*Si3839[2][3])*T13839[3][6] + Si3839[3][1]*T13839[4][6] + Si3839[3][2]*T13839[5][6]) + S3938[1][2]*((-(eff[1].x[2]*Si3839[1][1]) + eff[1].x[1]*Si3839[2][1])*T13839[1][4] + Si3839[3][2]*T13839[5][4] + Si3839[3][3]*T13839[6][4]) + S3938[2][2]*((-(eff[1].x[2]*Si3839[1][2]) + eff[1].x[1]*Si3839[2][2])*T13839[2][5] + Si3839[3][1]*T13839[4][5] + Si3839[3][3]*T13839[6][5]);
T3839[6][6]=S3938[3][3]*((-(eff[1].x[2]*Si3839[1][3]) + eff[1].x[1]*Si3839[2][3])*T13839[3][6] + Si3839[3][1]*T13839[4][6] + Si3839[3][2]*T13839[5][6]) + S3938[1][3]*((-(eff[1].x[2]*Si3839[1][1]) + eff[1].x[1]*Si3839[2][1])*T13839[1][4] + Si3839[3][2]*T13839[5][4] + Si3839[3][3]*T13839[6][4]) + S3938[2][3]*((-(eff[1].x[2]*Si3839[1][2]) + eff[1].x[1]*Si3839[2][2])*T13839[2][5] + Si3839[3][1]*T13839[4][5] + Si3839[3][3]*T13839[6][5]);



}


void
hermes_InvDynArtfunc66(void)
      {
JA38[1][1]=T3839[1][1] + T3840[1][1] + T3844[1][1] + T3848[1][1] + T3852[1][1] + T3856[1][1];
JA38[1][2]=links[14].mcm[3] + T3839[1][2] + T3840[1][2] + T3844[1][2] + T3848[1][2] + T3852[1][2] + T3856[1][2];
JA38[1][3]=-links[14].mcm[2] + T3839[1][3] + T3840[1][3] + T3844[1][3] + T3848[1][3] + T3852[1][3] + T3856[1][3];
JA38[1][4]=links[14].m + T3839[1][4] + T3840[1][4] + T3844[1][4] + T3848[1][4] + T3852[1][4] + T3856[1][4];
JA38[1][5]=T3839[1][5] + T3840[1][5] + T3844[1][5] + T3848[1][5] + T3852[1][5] + T3856[1][5];
JA38[1][6]=T3839[1][6] + T3840[1][6] + T3844[1][6] + T3848[1][6] + T3852[1][6] + T3856[1][6];

JA38[2][1]=-links[14].mcm[3] + T3839[2][1] + T3840[2][1] + T3844[2][1] + T3848[2][1] + T3852[2][1] + T3856[2][1];
JA38[2][2]=T3839[2][2] + T3840[2][2] + T3844[2][2] + T3848[2][2] + T3852[2][2] + T3856[2][2];
JA38[2][3]=links[14].mcm[1] + T3839[2][3] + T3840[2][3] + T3844[2][3] + T3848[2][3] + T3852[2][3] + T3856[2][3];
JA38[2][4]=T3839[2][4] + T3840[2][4] + T3844[2][4] + T3848[2][4] + T3852[2][4] + T3856[2][4];
JA38[2][5]=links[14].m + T3839[2][5] + T3840[2][5] + T3844[2][5] + T3848[2][5] + T3852[2][5] + T3856[2][5];
JA38[2][6]=T3839[2][6] + T3840[2][6] + T3844[2][6] + T3848[2][6] + T3852[2][6] + T3856[2][6];

JA38[3][1]=links[14].mcm[2] + T3839[3][1] + T3840[3][1] + T3844[3][1] + T3848[3][1] + T3852[3][1] + T3856[3][1];
JA38[3][2]=-links[14].mcm[1] + T3839[3][2] + T3840[3][2] + T3844[3][2] + T3848[3][2] + T3852[3][2] + T3856[3][2];
JA38[3][3]=T3839[3][3] + T3840[3][3] + T3844[3][3] + T3848[3][3] + T3852[3][3] + T3856[3][3];
JA38[3][4]=T3839[3][4] + T3840[3][4] + T3844[3][4] + T3848[3][4] + T3852[3][4] + T3856[3][4];
JA38[3][5]=T3839[3][5] + T3840[3][5] + T3844[3][5] + T3848[3][5] + T3852[3][5] + T3856[3][5];
JA38[3][6]=links[14].m + T3839[3][6] + T3840[3][6] + T3844[3][6] + T3848[3][6] + T3852[3][6] + T3856[3][6];

JA38[4][1]=links[14].inertia[1][1] + T3839[4][1] + T3840[4][1] + T3844[4][1] + T3848[4][1] + T3852[4][1] + T3856[4][1];
JA38[4][2]=links[14].inertia[1][2] + T3839[4][2] + T3840[4][2] + T3844[4][2] + T3848[4][2] + T3852[4][2] + T3856[4][2];
JA38[4][3]=links[14].inertia[1][3] + T3839[4][3] + T3840[4][3] + T3844[4][3] + T3848[4][3] + T3852[4][3] + T3856[4][3];
JA38[4][4]=T3839[4][4] + T3840[4][4] + T3844[4][4] + T3848[4][4] + T3852[4][4] + T3856[4][4];
JA38[4][5]=-links[14].mcm[3] + T3839[4][5] + T3840[4][5] + T3844[4][5] + T3848[4][5] + T3852[4][5] + T3856[4][5];
JA38[4][6]=links[14].mcm[2] + T3839[4][6] + T3840[4][6] + T3844[4][6] + T3848[4][6] + T3852[4][6] + T3856[4][6];

JA38[5][1]=links[14].inertia[1][2] + T3839[5][1] + T3840[5][1] + T3844[5][1] + T3848[5][1] + T3852[5][1] + T3856[5][1];
JA38[5][2]=links[14].inertia[2][2] + T3839[5][2] + T3840[5][2] + T3844[5][2] + T3848[5][2] + T3852[5][2] + T3856[5][2];
JA38[5][3]=links[14].inertia[2][3] + T3839[5][3] + T3840[5][3] + T3844[5][3] + T3848[5][3] + T3852[5][3] + T3856[5][3];
JA38[5][4]=links[14].mcm[3] + T3839[5][4] + T3840[5][4] + T3844[5][4] + T3848[5][4] + T3852[5][4] + T3856[5][4];
JA38[5][5]=T3839[5][5] + T3840[5][5] + T3844[5][5] + T3848[5][5] + T3852[5][5] + T3856[5][5];
JA38[5][6]=-links[14].mcm[1] + T3839[5][6] + T3840[5][6] + T3844[5][6] + T3848[5][6] + T3852[5][6] + T3856[5][6];

JA38[6][1]=links[14].inertia[1][3] + T3839[6][1] + T3840[6][1] + T3844[6][1] + T3848[6][1] + T3852[6][1] + T3856[6][1];
JA38[6][2]=links[14].inertia[2][3] + T3839[6][2] + T3840[6][2] + T3844[6][2] + T3848[6][2] + T3852[6][2] + T3856[6][2];
JA38[6][3]=links[14].inertia[3][3] + T3839[6][3] + T3840[6][3] + T3844[6][3] + T3848[6][3] + T3852[6][3] + T3856[6][3];
JA38[6][4]=-links[14].mcm[2] + T3839[6][4] + T3840[6][4] + T3844[6][4] + T3848[6][4] + T3852[6][4] + T3856[6][4];
JA38[6][5]=links[14].mcm[1] + T3839[6][5] + T3840[6][5] + T3844[6][5] + T3848[6][5] + T3852[6][5] + T3856[6][5];
JA38[6][6]=T3839[6][6] + T3840[6][6] + T3844[6][6] + T3848[6][6] + T3852[6][6] + T3856[6][6];


h38[1]=JA38[1][3];
h38[2]=JA38[2][3];
h38[3]=JA38[3][3];
h38[4]=JA38[4][3];
h38[5]=JA38[5][3];
h38[6]=JA38[6][3];

T13738[1][1]=JA38[1][1];
T13738[1][2]=JA38[1][2];
T13738[1][3]=JA38[1][3];
T13738[1][4]=JA38[1][4];
T13738[1][5]=JA38[1][5];
T13738[1][6]=JA38[1][6];

T13738[2][1]=JA38[2][1];
T13738[2][2]=JA38[2][2];
T13738[2][3]=JA38[2][3];
T13738[2][4]=JA38[2][4];
T13738[2][5]=JA38[2][5];
T13738[2][6]=JA38[2][6];

T13738[3][1]=JA38[3][1];
T13738[3][2]=JA38[3][2];
T13738[3][3]=JA38[3][3];
T13738[3][4]=JA38[3][4];
T13738[3][5]=JA38[3][5];
T13738[3][6]=JA38[3][6];

T13738[4][1]=JA38[4][1];
T13738[4][2]=JA38[4][2];
T13738[4][3]=JA38[4][3];
T13738[4][4]=JA38[4][4];
T13738[4][5]=JA38[4][5];
T13738[4][6]=JA38[4][6];

T13738[5][1]=JA38[5][1];
T13738[5][2]=JA38[5][2];
T13738[5][3]=JA38[5][3];
T13738[5][4]=JA38[5][4];
T13738[5][5]=JA38[5][5];
T13738[5][6]=JA38[5][6];

T13738[6][1]=JA38[6][1];
T13738[6][2]=JA38[6][2];
T13738[6][3]=JA38[6][3];
T13738[6][4]=JA38[6][4];
T13738[6][5]=JA38[6][5];
T13738[6][6]=JA38[6][6];


T3738[1][1]=S3837[1][1]*(Si3738[1][1]*T13738[1][1] + Si3738[1][2]*T13738[2][1]) + S3837[2][1]*(Si3738[1][1]*T13738[1][2] + Si3738[1][2]*T13738[2][2]);
T3738[1][2]=Si3738[1][1]*T13738[1][3] + Si3738[1][2]*T13738[2][3];
T3738[1][3]=S3837[1][3]*(Si3738[1][1]*T13738[1][1] + Si3738[1][2]*T13738[2][1]) + S3837[2][3]*(Si3738[1][1]*T13738[1][2] + Si3738[1][2]*T13738[2][2]);
T3738[1][4]=S3837[1][1]*(Si3738[1][1]*T13738[1][4] + Si3738[1][2]*T13738[2][4]) + S3837[2][1]*(Si3738[1][1]*T13738[1][5] + Si3738[1][2]*T13738[2][5]);
T3738[1][5]=Si3738[1][1]*T13738[1][6] + Si3738[1][2]*T13738[2][6];
T3738[1][6]=S3837[1][3]*(Si3738[1][1]*T13738[1][4] + Si3738[1][2]*T13738[2][4]) + S3837[2][3]*(Si3738[1][1]*T13738[1][5] + Si3738[1][2]*T13738[2][5]);

T3738[2][1]=S3837[1][1]*T13738[3][1] + S3837[2][1]*T13738[3][2];
T3738[2][2]=T13738[3][3];
T3738[2][3]=S3837[1][3]*T13738[3][1] + S3837[2][3]*T13738[3][2];
T3738[2][4]=S3837[1][1]*T13738[3][4] + S3837[2][1]*T13738[3][5];
T3738[2][5]=T13738[3][6];
T3738[2][6]=S3837[1][3]*T13738[3][4] + S3837[2][3]*T13738[3][5];

T3738[3][1]=S3837[1][1]*(Si3738[3][1]*T13738[1][1] + Si3738[3][2]*T13738[2][1]) + S3837[2][1]*(Si3738[3][1]*T13738[1][2] + Si3738[3][2]*T13738[2][2]);
T3738[3][2]=Si3738[3][1]*T13738[1][3] + Si3738[3][2]*T13738[2][3];
T3738[3][3]=S3837[1][3]*(Si3738[3][1]*T13738[1][1] + Si3738[3][2]*T13738[2][1]) + S3837[2][3]*(Si3738[3][1]*T13738[1][2] + Si3738[3][2]*T13738[2][2]);
T3738[3][4]=S3837[1][1]*(Si3738[3][1]*T13738[1][4] + Si3738[3][2]*T13738[2][4]) + S3837[2][1]*(Si3738[3][1]*T13738[1][5] + Si3738[3][2]*T13738[2][5]);
T3738[3][5]=Si3738[3][1]*T13738[1][6] + Si3738[3][2]*T13738[2][6];
T3738[3][6]=S3837[1][3]*(Si3738[3][1]*T13738[1][4] + Si3738[3][2]*T13738[2][4]) + S3837[2][3]*(Si3738[3][1]*T13738[1][5] + Si3738[3][2]*T13738[2][5]);

T3738[4][1]=S3837[1][1]*(Si3738[1][1]*T13738[4][1] + Si3738[1][2]*T13738[5][1]) + S3837[2][1]*(Si3738[1][1]*T13738[4][2] + Si3738[1][2]*T13738[5][2]);
T3738[4][2]=Si3738[1][1]*T13738[4][3] + Si3738[1][2]*T13738[5][3];
T3738[4][3]=S3837[1][3]*(Si3738[1][1]*T13738[4][1] + Si3738[1][2]*T13738[5][1]) + S3837[2][3]*(Si3738[1][1]*T13738[4][2] + Si3738[1][2]*T13738[5][2]);
T3738[4][4]=S3837[1][1]*(Si3738[1][1]*T13738[4][4] + Si3738[1][2]*T13738[5][4]) + S3837[2][1]*(Si3738[1][1]*T13738[4][5] + Si3738[1][2]*T13738[5][5]);
T3738[4][5]=Si3738[1][1]*T13738[4][6] + Si3738[1][2]*T13738[5][6];
T3738[4][6]=S3837[1][3]*(Si3738[1][1]*T13738[4][4] + Si3738[1][2]*T13738[5][4]) + S3837[2][3]*(Si3738[1][1]*T13738[4][5] + Si3738[1][2]*T13738[5][5]);

T3738[5][1]=S3837[1][1]*T13738[6][1] + S3837[2][1]*T13738[6][2];
T3738[5][2]=T13738[6][3];
T3738[5][3]=S3837[1][3]*T13738[6][1] + S3837[2][3]*T13738[6][2];
T3738[5][4]=S3837[1][1]*T13738[6][4] + S3837[2][1]*T13738[6][5];
T3738[5][5]=T13738[6][6];
T3738[5][6]=S3837[1][3]*T13738[6][4] + S3837[2][3]*T13738[6][5];

T3738[6][1]=S3837[1][1]*(Si3738[3][1]*T13738[4][1] + Si3738[3][2]*T13738[5][1]) + S3837[2][1]*(Si3738[3][1]*T13738[4][2] + Si3738[3][2]*T13738[5][2]);
T3738[6][2]=Si3738[3][1]*T13738[4][3] + Si3738[3][2]*T13738[5][3];
T3738[6][3]=S3837[1][3]*(Si3738[3][1]*T13738[4][1] + Si3738[3][2]*T13738[5][1]) + S3837[2][3]*(Si3738[3][1]*T13738[4][2] + Si3738[3][2]*T13738[5][2]);
T3738[6][4]=S3837[1][1]*(Si3738[3][1]*T13738[4][4] + Si3738[3][2]*T13738[5][4]) + S3837[2][1]*(Si3738[3][1]*T13738[4][5] + Si3738[3][2]*T13738[5][5]);
T3738[6][5]=Si3738[3][1]*T13738[4][6] + Si3738[3][2]*T13738[5][6];
T3738[6][6]=S3837[1][3]*(Si3738[3][1]*T13738[4][4] + Si3738[3][2]*T13738[5][4]) + S3837[2][3]*(Si3738[3][1]*T13738[4][5] + Si3738[3][2]*T13738[5][5]);



}


void
hermes_InvDynArtfunc67(void)
      {
JA37[1][1]=T3738[1][1];
JA37[1][2]=links[13].mcm[3] + T3738[1][2];
JA37[1][3]=-links[13].mcm[2] + T3738[1][3];
JA37[1][4]=links[13].m + T3738[1][4];
JA37[1][5]=T3738[1][5];
JA37[1][6]=T3738[1][6];

JA37[2][1]=-links[13].mcm[3] + T3738[2][1];
JA37[2][2]=T3738[2][2];
JA37[2][3]=links[13].mcm[1] + T3738[2][3];
JA37[2][4]=T3738[2][4];
JA37[2][5]=links[13].m + T3738[2][5];
JA37[2][6]=T3738[2][6];

JA37[3][1]=links[13].mcm[2] + T3738[3][1];
JA37[3][2]=-links[13].mcm[1] + T3738[3][2];
JA37[3][3]=T3738[3][3];
JA37[3][4]=T3738[3][4];
JA37[3][5]=T3738[3][5];
JA37[3][6]=links[13].m + T3738[3][6];

JA37[4][1]=links[13].inertia[1][1] + T3738[4][1];
JA37[4][2]=links[13].inertia[1][2] + T3738[4][2];
JA37[4][3]=links[13].inertia[1][3] + T3738[4][3];
JA37[4][4]=T3738[4][4];
JA37[4][5]=-links[13].mcm[3] + T3738[4][5];
JA37[4][6]=links[13].mcm[2] + T3738[4][6];

JA37[5][1]=links[13].inertia[1][2] + T3738[5][1];
JA37[5][2]=links[13].inertia[2][2] + T3738[5][2];
JA37[5][3]=links[13].inertia[2][3] + T3738[5][3];
JA37[5][4]=links[13].mcm[3] + T3738[5][4];
JA37[5][5]=T3738[5][5];
JA37[5][6]=-links[13].mcm[1] + T3738[5][6];

JA37[6][1]=links[13].inertia[1][3] + T3738[6][1];
JA37[6][2]=links[13].inertia[2][3] + T3738[6][2];
JA37[6][3]=links[13].inertia[3][3] + T3738[6][3];
JA37[6][4]=-links[13].mcm[2] + T3738[6][4];
JA37[6][5]=links[13].mcm[1] + T3738[6][5];
JA37[6][6]=T3738[6][6];


h37[1]=JA37[1][3];
h37[2]=JA37[2][3];
h37[3]=JA37[3][3];
h37[4]=JA37[4][3];
h37[5]=JA37[5][3];
h37[6]=JA37[6][3];

T13637[1][1]=JA37[1][1];
T13637[1][2]=JA37[1][2];
T13637[1][3]=JA37[1][3];
T13637[1][4]=JA37[1][4];
T13637[1][5]=JA37[1][5];
T13637[1][6]=JA37[1][6];

T13637[2][1]=JA37[2][1];
T13637[2][2]=JA37[2][2];
T13637[2][3]=JA37[2][3];
T13637[2][4]=JA37[2][4];
T13637[2][5]=JA37[2][5];
T13637[2][6]=JA37[2][6];

T13637[3][1]=JA37[3][1];
T13637[3][2]=JA37[3][2];
T13637[3][3]=JA37[3][3];
T13637[3][4]=JA37[3][4];
T13637[3][5]=JA37[3][5];
T13637[3][6]=JA37[3][6];

T13637[4][1]=JA37[4][1];
T13637[4][2]=JA37[4][2];
T13637[4][3]=JA37[4][3];
T13637[4][4]=JA37[4][4];
T13637[4][5]=JA37[4][5];
T13637[4][6]=JA37[4][6];

T13637[5][1]=JA37[5][1];
T13637[5][2]=JA37[5][2];
T13637[5][3]=JA37[5][3];
T13637[5][4]=JA37[5][4];
T13637[5][5]=JA37[5][5];
T13637[5][6]=JA37[5][6];

T13637[6][1]=JA37[6][1];
T13637[6][2]=JA37[6][2];
T13637[6][3]=JA37[6][3];
T13637[6][4]=JA37[6][4];
T13637[6][5]=JA37[6][5];
T13637[6][6]=JA37[6][6];


T3637[1][1]=T13637[3][3] - (-(LOWERARM*S3736[1][2]) + WRISTY*S3736[1][3])*T13637[3][4] - (-(LOWERARM*S3736[2][2]) + WRISTY*S3736[2][3])*T13637[3][5];
T3637[1][2]=-(S3736[1][2]*T13637[3][1]) - S3736[2][2]*T13637[3][2] + LOWERARM*T13637[3][6];
T3637[1][3]=-(S3736[1][3]*T13637[3][1]) - S3736[2][3]*T13637[3][2] - WRISTY*T13637[3][6];
T3637[1][4]=T13637[3][6];
T3637[1][5]=-(S3736[1][2]*T13637[3][4]) - S3736[2][2]*T13637[3][5];
T3637[1][6]=-(S3736[1][3]*T13637[3][4]) - S3736[2][3]*T13637[3][5];

T3637[2][1]=-(Si3637[2][1]*T13637[1][3]) - Si3637[2][2]*T13637[2][3] + (-(LOWERARM*S3736[1][2]) + WRISTY*S3736[1][3])*(Si3637[2][1]*T13637[1][4] + Si3637[2][2]*T13637[2][4]) + (-(LOWERARM*S3736[2][2]) + WRISTY*S3736[2][3])*(Si3637[2][1]*T13637[1][5] + Si3637[2][2]*T13637[2][5]);
T3637[2][2]=S3736[1][2]*(Si3637[2][1]*T13637[1][1] + Si3637[2][2]*T13637[2][1]) + S3736[2][2]*(Si3637[2][1]*T13637[1][2] + Si3637[2][2]*T13637[2][2]) - LOWERARM*(Si3637[2][1]*T13637[1][6] + Si3637[2][2]*T13637[2][6]);
T3637[2][3]=S3736[1][3]*(Si3637[2][1]*T13637[1][1] + Si3637[2][2]*T13637[2][1]) + S3736[2][3]*(Si3637[2][1]*T13637[1][2] + Si3637[2][2]*T13637[2][2]) + WRISTY*(Si3637[2][1]*T13637[1][6] + Si3637[2][2]*T13637[2][6]);
T3637[2][4]=-(Si3637[2][1]*T13637[1][6]) - Si3637[2][2]*T13637[2][6];
T3637[2][5]=S3736[1][2]*(Si3637[2][1]*T13637[1][4] + Si3637[2][2]*T13637[2][4]) + S3736[2][2]*(Si3637[2][1]*T13637[1][5] + Si3637[2][2]*T13637[2][5]);
T3637[2][6]=S3736[1][3]*(Si3637[2][1]*T13637[1][4] + Si3637[2][2]*T13637[2][4]) + S3736[2][3]*(Si3637[2][1]*T13637[1][5] + Si3637[2][2]*T13637[2][5]);

T3637[3][1]=-(Si3637[3][1]*T13637[1][3]) - Si3637[3][2]*T13637[2][3] + (-(LOWERARM*S3736[1][2]) + WRISTY*S3736[1][3])*(Si3637[3][1]*T13637[1][4] + Si3637[3][2]*T13637[2][4]) + (-(LOWERARM*S3736[2][2]) + WRISTY*S3736[2][3])*(Si3637[3][1]*T13637[1][5] + Si3637[3][2]*T13637[2][5]);
T3637[3][2]=S3736[1][2]*(Si3637[3][1]*T13637[1][1] + Si3637[3][2]*T13637[2][1]) + S3736[2][2]*(Si3637[3][1]*T13637[1][2] + Si3637[3][2]*T13637[2][2]) - LOWERARM*(Si3637[3][1]*T13637[1][6] + Si3637[3][2]*T13637[2][6]);
T3637[3][3]=S3736[1][3]*(Si3637[3][1]*T13637[1][1] + Si3637[3][2]*T13637[2][1]) + S3736[2][3]*(Si3637[3][1]*T13637[1][2] + Si3637[3][2]*T13637[2][2]) + WRISTY*(Si3637[3][1]*T13637[1][6] + Si3637[3][2]*T13637[2][6]);
T3637[3][4]=-(Si3637[3][1]*T13637[1][6]) - Si3637[3][2]*T13637[2][6];
T3637[3][5]=S3736[1][2]*(Si3637[3][1]*T13637[1][4] + Si3637[3][2]*T13637[2][4]) + S3736[2][2]*(Si3637[3][1]*T13637[1][5] + Si3637[3][2]*T13637[2][5]);
T3637[3][6]=S3736[1][3]*(Si3637[3][1]*T13637[1][4] + Si3637[3][2]*T13637[2][4]) + S3736[2][3]*(Si3637[3][1]*T13637[1][5] + Si3637[3][2]*T13637[2][5]);

T3637[4][1]=-((-(LOWERARM*Si3637[2][1]) + WRISTY*Si3637[3][1])*T13637[1][3]) - (-(LOWERARM*Si3637[2][2]) + WRISTY*Si3637[3][2])*T13637[2][3] + T13637[6][3] + (-(LOWERARM*S3736[1][2]) + WRISTY*S3736[1][3])*((-(LOWERARM*Si3637[2][1]) + WRISTY*Si3637[3][1])*T13637[1][4] + (-(LOWERARM*Si3637[2][2]) + WRISTY*Si3637[3][2])*T13637[2][4] - T13637[6][4]) + (-(LOWERARM*S3736[2][2]) + WRISTY*S3736[2][3])*((-(LOWERARM*Si3637[2][1]) + WRISTY*Si3637[3][1])*T13637[1][5] + (-(LOWERARM*Si3637[2][2]) + WRISTY*Si3637[3][2])*T13637[2][5] - T13637[6][5]);
T3637[4][2]=S3736[1][2]*((-(LOWERARM*Si3637[2][1]) + WRISTY*Si3637[3][1])*T13637[1][1] + (-(LOWERARM*Si3637[2][2]) + WRISTY*Si3637[3][2])*T13637[2][1] - T13637[6][1]) + S3736[2][2]*((-(LOWERARM*Si3637[2][1]) + WRISTY*Si3637[3][1])*T13637[1][2] + (-(LOWERARM*Si3637[2][2]) + WRISTY*Si3637[3][2])*T13637[2][2] - T13637[6][2]) - LOWERARM*((-(LOWERARM*Si3637[2][1]) + WRISTY*Si3637[3][1])*T13637[1][6] + (-(LOWERARM*Si3637[2][2]) + WRISTY*Si3637[3][2])*T13637[2][6] - T13637[6][6]);
T3637[4][3]=S3736[1][3]*((-(LOWERARM*Si3637[2][1]) + WRISTY*Si3637[3][1])*T13637[1][1] + (-(LOWERARM*Si3637[2][2]) + WRISTY*Si3637[3][2])*T13637[2][1] - T13637[6][1]) + S3736[2][3]*((-(LOWERARM*Si3637[2][1]) + WRISTY*Si3637[3][1])*T13637[1][2] + (-(LOWERARM*Si3637[2][2]) + WRISTY*Si3637[3][2])*T13637[2][2] - T13637[6][2]) + WRISTY*((-(LOWERARM*Si3637[2][1]) + WRISTY*Si3637[3][1])*T13637[1][6] + (-(LOWERARM*Si3637[2][2]) + WRISTY*Si3637[3][2])*T13637[2][6] - T13637[6][6]);
T3637[4][4]=-((-(LOWERARM*Si3637[2][1]) + WRISTY*Si3637[3][1])*T13637[1][6]) - (-(LOWERARM*Si3637[2][2]) + WRISTY*Si3637[3][2])*T13637[2][6] + T13637[6][6];
T3637[4][5]=S3736[1][2]*((-(LOWERARM*Si3637[2][1]) + WRISTY*Si3637[3][1])*T13637[1][4] + (-(LOWERARM*Si3637[2][2]) + WRISTY*Si3637[3][2])*T13637[2][4] - T13637[6][4]) + S3736[2][2]*((-(LOWERARM*Si3637[2][1]) + WRISTY*Si3637[3][1])*T13637[1][5] + (-(LOWERARM*Si3637[2][2]) + WRISTY*Si3637[3][2])*T13637[2][5] - T13637[6][5]);
T3637[4][6]=S3736[1][3]*((-(LOWERARM*Si3637[2][1]) + WRISTY*Si3637[3][1])*T13637[1][4] + (-(LOWERARM*Si3637[2][2]) + WRISTY*Si3637[3][2])*T13637[2][4] - T13637[6][4]) + S3736[2][3]*((-(LOWERARM*Si3637[2][1]) + WRISTY*Si3637[3][1])*T13637[1][5] + (-(LOWERARM*Si3637[2][2]) + WRISTY*Si3637[3][2])*T13637[2][5] - T13637[6][5]);

T3637[5][1]=LOWERARM*T13637[3][3] - Si3637[2][1]*T13637[4][3] - Si3637[2][2]*T13637[5][3] + (-(LOWERARM*S3736[1][2]) + WRISTY*S3736[1][3])*(-(LOWERARM*T13637[3][4]) + Si3637[2][1]*T13637[4][4] + Si3637[2][2]*T13637[5][4]) + (-(LOWERARM*S3736[2][2]) + WRISTY*S3736[2][3])*(-(LOWERARM*T13637[3][5]) + Si3637[2][1]*T13637[4][5] + Si3637[2][2]*T13637[5][5]);
T3637[5][2]=S3736[1][2]*(-(LOWERARM*T13637[3][1]) + Si3637[2][1]*T13637[4][1] + Si3637[2][2]*T13637[5][1]) + S3736[2][2]*(-(LOWERARM*T13637[3][2]) + Si3637[2][1]*T13637[4][2] + Si3637[2][2]*T13637[5][2]) - LOWERARM*(-(LOWERARM*T13637[3][6]) + Si3637[2][1]*T13637[4][6] + Si3637[2][2]*T13637[5][6]);
T3637[5][3]=S3736[1][3]*(-(LOWERARM*T13637[3][1]) + Si3637[2][1]*T13637[4][1] + Si3637[2][2]*T13637[5][1]) + S3736[2][3]*(-(LOWERARM*T13637[3][2]) + Si3637[2][1]*T13637[4][2] + Si3637[2][2]*T13637[5][2]) + WRISTY*(-(LOWERARM*T13637[3][6]) + Si3637[2][1]*T13637[4][6] + Si3637[2][2]*T13637[5][6]);
T3637[5][4]=LOWERARM*T13637[3][6] - Si3637[2][1]*T13637[4][6] - Si3637[2][2]*T13637[5][6];
T3637[5][5]=S3736[1][2]*(-(LOWERARM*T13637[3][4]) + Si3637[2][1]*T13637[4][4] + Si3637[2][2]*T13637[5][4]) + S3736[2][2]*(-(LOWERARM*T13637[3][5]) + Si3637[2][1]*T13637[4][5] + Si3637[2][2]*T13637[5][5]);
T3637[5][6]=S3736[1][3]*(-(LOWERARM*T13637[3][4]) + Si3637[2][1]*T13637[4][4] + Si3637[2][2]*T13637[5][4]) + S3736[2][3]*(-(LOWERARM*T13637[3][5]) + Si3637[2][1]*T13637[4][5] + Si3637[2][2]*T13637[5][5]);

T3637[6][1]=-(WRISTY*T13637[3][3]) - Si3637[3][1]*T13637[4][3] - Si3637[3][2]*T13637[5][3] + (-(LOWERARM*S3736[1][2]) + WRISTY*S3736[1][3])*(WRISTY*T13637[3][4] + Si3637[3][1]*T13637[4][4] + Si3637[3][2]*T13637[5][4]) + (-(LOWERARM*S3736[2][2]) + WRISTY*S3736[2][3])*(WRISTY*T13637[3][5] + Si3637[3][1]*T13637[4][5] + Si3637[3][2]*T13637[5][5]);
T3637[6][2]=S3736[1][2]*(WRISTY*T13637[3][1] + Si3637[3][1]*T13637[4][1] + Si3637[3][2]*T13637[5][1]) + S3736[2][2]*(WRISTY*T13637[3][2] + Si3637[3][1]*T13637[4][2] + Si3637[3][2]*T13637[5][2]) - LOWERARM*(WRISTY*T13637[3][6] + Si3637[3][1]*T13637[4][6] + Si3637[3][2]*T13637[5][6]);
T3637[6][3]=S3736[1][3]*(WRISTY*T13637[3][1] + Si3637[3][1]*T13637[4][1] + Si3637[3][2]*T13637[5][1]) + S3736[2][3]*(WRISTY*T13637[3][2] + Si3637[3][1]*T13637[4][2] + Si3637[3][2]*T13637[5][2]) + WRISTY*(WRISTY*T13637[3][6] + Si3637[3][1]*T13637[4][6] + Si3637[3][2]*T13637[5][6]);
T3637[6][4]=-(WRISTY*T13637[3][6]) - Si3637[3][1]*T13637[4][6] - Si3637[3][2]*T13637[5][6];
T3637[6][5]=S3736[1][2]*(WRISTY*T13637[3][4] + Si3637[3][1]*T13637[4][4] + Si3637[3][2]*T13637[5][4]) + S3736[2][2]*(WRISTY*T13637[3][5] + Si3637[3][1]*T13637[4][5] + Si3637[3][2]*T13637[5][5]);
T3637[6][6]=S3736[1][3]*(WRISTY*T13637[3][4] + Si3637[3][1]*T13637[4][4] + Si3637[3][2]*T13637[5][4]) + S3736[2][3]*(WRISTY*T13637[3][5] + Si3637[3][1]*T13637[4][5] + Si3637[3][2]*T13637[5][5]);



}


void
hermes_InvDynArtfunc68(void)
      {
JA36[1][1]=T3637[1][1];
JA36[1][2]=links[12].mcm[3] + T3637[1][2];
JA36[1][3]=-links[12].mcm[2] + T3637[1][3];
JA36[1][4]=links[12].m + T3637[1][4];
JA36[1][5]=T3637[1][5];
JA36[1][6]=T3637[1][6];

JA36[2][1]=-links[12].mcm[3] + T3637[2][1];
JA36[2][2]=T3637[2][2];
JA36[2][3]=links[12].mcm[1] + T3637[2][3];
JA36[2][4]=T3637[2][4];
JA36[2][5]=links[12].m + T3637[2][5];
JA36[2][6]=T3637[2][6];

JA36[3][1]=links[12].mcm[2] + T3637[3][1];
JA36[3][2]=-links[12].mcm[1] + T3637[3][2];
JA36[3][3]=T3637[3][3];
JA36[3][4]=T3637[3][4];
JA36[3][5]=T3637[3][5];
JA36[3][6]=links[12].m + T3637[3][6];

JA36[4][1]=links[12].inertia[1][1] + T3637[4][1];
JA36[4][2]=links[12].inertia[1][2] + T3637[4][2];
JA36[4][3]=links[12].inertia[1][3] + T3637[4][3];
JA36[4][4]=T3637[4][4];
JA36[4][5]=-links[12].mcm[3] + T3637[4][5];
JA36[4][6]=links[12].mcm[2] + T3637[4][6];

JA36[5][1]=links[12].inertia[1][2] + T3637[5][1];
JA36[5][2]=links[12].inertia[2][2] + T3637[5][2];
JA36[5][3]=links[12].inertia[2][3] + T3637[5][3];
JA36[5][4]=links[12].mcm[3] + T3637[5][4];
JA36[5][5]=T3637[5][5];
JA36[5][6]=-links[12].mcm[1] + T3637[5][6];

JA36[6][1]=links[12].inertia[1][3] + T3637[6][1];
JA36[6][2]=links[12].inertia[2][3] + T3637[6][2];
JA36[6][3]=links[12].inertia[3][3] + T3637[6][3];
JA36[6][4]=-links[12].mcm[2] + T3637[6][4];
JA36[6][5]=links[12].mcm[1] + T3637[6][5];
JA36[6][6]=T3637[6][6];


h36[1]=JA36[1][3];
h36[2]=JA36[2][3];
h36[3]=JA36[3][3];
h36[4]=JA36[4][3];
h36[5]=JA36[5][3];
h36[6]=JA36[6][3];

T13536[1][1]=JA36[1][1];
T13536[1][2]=JA36[1][2];
T13536[1][3]=JA36[1][3];
T13536[1][4]=JA36[1][4];
T13536[1][5]=JA36[1][5];
T13536[1][6]=JA36[1][6];

T13536[2][1]=JA36[2][1];
T13536[2][2]=JA36[2][2];
T13536[2][3]=JA36[2][3];
T13536[2][4]=JA36[2][4];
T13536[2][5]=JA36[2][5];
T13536[2][6]=JA36[2][6];

T13536[3][1]=JA36[3][1];
T13536[3][2]=JA36[3][2];
T13536[3][3]=JA36[3][3];
T13536[3][4]=JA36[3][4];
T13536[3][5]=JA36[3][5];
T13536[3][6]=JA36[3][6];

T13536[4][1]=JA36[4][1];
T13536[4][2]=JA36[4][2];
T13536[4][3]=JA36[4][3];
T13536[4][4]=JA36[4][4];
T13536[4][5]=JA36[4][5];
T13536[4][6]=JA36[4][6];

T13536[5][1]=JA36[5][1];
T13536[5][2]=JA36[5][2];
T13536[5][3]=JA36[5][3];
T13536[5][4]=JA36[5][4];
T13536[5][5]=JA36[5][5];
T13536[5][6]=JA36[5][6];

T13536[6][1]=JA36[6][1];
T13536[6][2]=JA36[6][2];
T13536[6][3]=JA36[6][3];
T13536[6][4]=JA36[6][4];
T13536[6][5]=JA36[6][5];
T13536[6][6]=JA36[6][6];


T3536[1][1]=S3635[1][1]*(Si3536[1][1]*T13536[1][1] + Si3536[1][2]*T13536[2][1]) + S3635[2][1]*(Si3536[1][1]*T13536[1][2] + Si3536[1][2]*T13536[2][2]);
T3536[1][2]=-(Si3536[1][1]*T13536[1][3]) - Si3536[1][2]*T13536[2][3];
T3536[1][3]=S3635[1][3]*(Si3536[1][1]*T13536[1][1] + Si3536[1][2]*T13536[2][1]) + S3635[2][3]*(Si3536[1][1]*T13536[1][2] + Si3536[1][2]*T13536[2][2]);
T3536[1][4]=S3635[1][1]*(Si3536[1][1]*T13536[1][4] + Si3536[1][2]*T13536[2][4]) + S3635[2][1]*(Si3536[1][1]*T13536[1][5] + Si3536[1][2]*T13536[2][5]);
T3536[1][5]=-(Si3536[1][1]*T13536[1][6]) - Si3536[1][2]*T13536[2][6];
T3536[1][6]=S3635[1][3]*(Si3536[1][1]*T13536[1][4] + Si3536[1][2]*T13536[2][4]) + S3635[2][3]*(Si3536[1][1]*T13536[1][5] + Si3536[1][2]*T13536[2][5]);

T3536[2][1]=-(S3635[1][1]*T13536[3][1]) - S3635[2][1]*T13536[3][2];
T3536[2][2]=T13536[3][3];
T3536[2][3]=-(S3635[1][3]*T13536[3][1]) - S3635[2][3]*T13536[3][2];
T3536[2][4]=-(S3635[1][1]*T13536[3][4]) - S3635[2][1]*T13536[3][5];
T3536[2][5]=T13536[3][6];
T3536[2][6]=-(S3635[1][3]*T13536[3][4]) - S3635[2][3]*T13536[3][5];

T3536[3][1]=S3635[1][1]*(Si3536[3][1]*T13536[1][1] + Si3536[3][2]*T13536[2][1]) + S3635[2][1]*(Si3536[3][1]*T13536[1][2] + Si3536[3][2]*T13536[2][2]);
T3536[3][2]=-(Si3536[3][1]*T13536[1][3]) - Si3536[3][2]*T13536[2][3];
T3536[3][3]=S3635[1][3]*(Si3536[3][1]*T13536[1][1] + Si3536[3][2]*T13536[2][1]) + S3635[2][3]*(Si3536[3][1]*T13536[1][2] + Si3536[3][2]*T13536[2][2]);
T3536[3][4]=S3635[1][1]*(Si3536[3][1]*T13536[1][4] + Si3536[3][2]*T13536[2][4]) + S3635[2][1]*(Si3536[3][1]*T13536[1][5] + Si3536[3][2]*T13536[2][5]);
T3536[3][5]=-(Si3536[3][1]*T13536[1][6]) - Si3536[3][2]*T13536[2][6];
T3536[3][6]=S3635[1][3]*(Si3536[3][1]*T13536[1][4] + Si3536[3][2]*T13536[2][4]) + S3635[2][3]*(Si3536[3][1]*T13536[1][5] + Si3536[3][2]*T13536[2][5]);

T3536[4][1]=S3635[1][1]*(Si3536[1][1]*T13536[4][1] + Si3536[1][2]*T13536[5][1]) + S3635[2][1]*(Si3536[1][1]*T13536[4][2] + Si3536[1][2]*T13536[5][2]);
T3536[4][2]=-(Si3536[1][1]*T13536[4][3]) - Si3536[1][2]*T13536[5][3];
T3536[4][3]=S3635[1][3]*(Si3536[1][1]*T13536[4][1] + Si3536[1][2]*T13536[5][1]) + S3635[2][3]*(Si3536[1][1]*T13536[4][2] + Si3536[1][2]*T13536[5][2]);
T3536[4][4]=S3635[1][1]*(Si3536[1][1]*T13536[4][4] + Si3536[1][2]*T13536[5][4]) + S3635[2][1]*(Si3536[1][1]*T13536[4][5] + Si3536[1][2]*T13536[5][5]);
T3536[4][5]=-(Si3536[1][1]*T13536[4][6]) - Si3536[1][2]*T13536[5][6];
T3536[4][6]=S3635[1][3]*(Si3536[1][1]*T13536[4][4] + Si3536[1][2]*T13536[5][4]) + S3635[2][3]*(Si3536[1][1]*T13536[4][5] + Si3536[1][2]*T13536[5][5]);

T3536[5][1]=-(S3635[1][1]*T13536[6][1]) - S3635[2][1]*T13536[6][2];
T3536[5][2]=T13536[6][3];
T3536[5][3]=-(S3635[1][3]*T13536[6][1]) - S3635[2][3]*T13536[6][2];
T3536[5][4]=-(S3635[1][1]*T13536[6][4]) - S3635[2][1]*T13536[6][5];
T3536[5][5]=T13536[6][6];
T3536[5][6]=-(S3635[1][3]*T13536[6][4]) - S3635[2][3]*T13536[6][5];

T3536[6][1]=S3635[1][1]*(Si3536[3][1]*T13536[4][1] + Si3536[3][2]*T13536[5][1]) + S3635[2][1]*(Si3536[3][1]*T13536[4][2] + Si3536[3][2]*T13536[5][2]);
T3536[6][2]=-(Si3536[3][1]*T13536[4][3]) - Si3536[3][2]*T13536[5][3];
T3536[6][3]=S3635[1][3]*(Si3536[3][1]*T13536[4][1] + Si3536[3][2]*T13536[5][1]) + S3635[2][3]*(Si3536[3][1]*T13536[4][2] + Si3536[3][2]*T13536[5][2]);
T3536[6][4]=S3635[1][1]*(Si3536[3][1]*T13536[4][4] + Si3536[3][2]*T13536[5][4]) + S3635[2][1]*(Si3536[3][1]*T13536[4][5] + Si3536[3][2]*T13536[5][5]);
T3536[6][5]=-(Si3536[3][1]*T13536[4][6]) - Si3536[3][2]*T13536[5][6];
T3536[6][6]=S3635[1][3]*(Si3536[3][1]*T13536[4][4] + Si3536[3][2]*T13536[5][4]) + S3635[2][3]*(Si3536[3][1]*T13536[4][5] + Si3536[3][2]*T13536[5][5]);



}


void
hermes_InvDynArtfunc69(void)
      {
JA35[1][1]=T3536[1][1];
JA35[1][2]=links[11].mcm[3] + T3536[1][2];
JA35[1][3]=-links[11].mcm[2] + T3536[1][3];
JA35[1][4]=links[11].m + T3536[1][4];
JA35[1][5]=T3536[1][5];
JA35[1][6]=T3536[1][6];

JA35[2][1]=-links[11].mcm[3] + T3536[2][1];
JA35[2][2]=T3536[2][2];
JA35[2][3]=links[11].mcm[1] + T3536[2][3];
JA35[2][4]=T3536[2][4];
JA35[2][5]=links[11].m + T3536[2][5];
JA35[2][6]=T3536[2][6];

JA35[3][1]=links[11].mcm[2] + T3536[3][1];
JA35[3][2]=-links[11].mcm[1] + T3536[3][2];
JA35[3][3]=T3536[3][3];
JA35[3][4]=T3536[3][4];
JA35[3][5]=T3536[3][5];
JA35[3][6]=links[11].m + T3536[3][6];

JA35[4][1]=links[11].inertia[1][1] + T3536[4][1];
JA35[4][2]=links[11].inertia[1][2] + T3536[4][2];
JA35[4][3]=links[11].inertia[1][3] + T3536[4][3];
JA35[4][4]=T3536[4][4];
JA35[4][5]=-links[11].mcm[3] + T3536[4][5];
JA35[4][6]=links[11].mcm[2] + T3536[4][6];

JA35[5][1]=links[11].inertia[1][2] + T3536[5][1];
JA35[5][2]=links[11].inertia[2][2] + T3536[5][2];
JA35[5][3]=links[11].inertia[2][3] + T3536[5][3];
JA35[5][4]=links[11].mcm[3] + T3536[5][4];
JA35[5][5]=T3536[5][5];
JA35[5][6]=-links[11].mcm[1] + T3536[5][6];

JA35[6][1]=links[11].inertia[1][3] + T3536[6][1];
JA35[6][2]=links[11].inertia[2][3] + T3536[6][2];
JA35[6][3]=links[11].inertia[3][3] + T3536[6][3];
JA35[6][4]=-links[11].mcm[2] + T3536[6][4];
JA35[6][5]=links[11].mcm[1] + T3536[6][5];
JA35[6][6]=T3536[6][6];


h35[1]=JA35[1][3];
h35[2]=JA35[2][3];
h35[3]=JA35[3][3];
h35[4]=JA35[4][3];
h35[5]=JA35[5][3];
h35[6]=JA35[6][3];

T13435[1][1]=JA35[1][1];
T13435[1][2]=JA35[1][2];
T13435[1][3]=JA35[1][3];
T13435[1][4]=JA35[1][4];
T13435[1][5]=JA35[1][5];
T13435[1][6]=JA35[1][6];

T13435[2][1]=JA35[2][1];
T13435[2][2]=JA35[2][2];
T13435[2][3]=JA35[2][3];
T13435[2][4]=JA35[2][4];
T13435[2][5]=JA35[2][5];
T13435[2][6]=JA35[2][6];

T13435[3][1]=JA35[3][1];
T13435[3][2]=JA35[3][2];
T13435[3][3]=JA35[3][3];
T13435[3][4]=JA35[3][4];
T13435[3][5]=JA35[3][5];
T13435[3][6]=JA35[3][6];

T13435[4][1]=JA35[4][1];
T13435[4][2]=JA35[4][2];
T13435[4][3]=JA35[4][3];
T13435[4][4]=JA35[4][4];
T13435[4][5]=JA35[4][5];
T13435[4][6]=JA35[4][6];

T13435[5][1]=JA35[5][1];
T13435[5][2]=JA35[5][2];
T13435[5][3]=JA35[5][3];
T13435[5][4]=JA35[5][4];
T13435[5][5]=JA35[5][5];
T13435[5][6]=JA35[5][6];

T13435[6][1]=JA35[6][1];
T13435[6][2]=JA35[6][2];
T13435[6][3]=JA35[6][3];
T13435[6][4]=JA35[6][4];
T13435[6][5]=JA35[6][5];
T13435[6][6]=JA35[6][6];


T3435[1][1]=T13435[3][3] + UPPERARM*S3534[1][2]*T13435[3][4] + UPPERARM*S3534[2][2]*T13435[3][5];
T3435[1][2]=-(S3534[1][2]*T13435[3][1]) - S3534[2][2]*T13435[3][2] + UPPERARM*T13435[3][6];
T3435[1][3]=-(S3534[1][3]*T13435[3][1]) - S3534[2][3]*T13435[3][2];
T3435[1][4]=T13435[3][6];
T3435[1][5]=-(S3534[1][2]*T13435[3][4]) - S3534[2][2]*T13435[3][5];
T3435[1][6]=-(S3534[1][3]*T13435[3][4]) - S3534[2][3]*T13435[3][5];

T3435[2][1]=-(Si3435[2][1]*T13435[1][3]) - Si3435[2][2]*T13435[2][3] - UPPERARM*S3534[1][2]*(Si3435[2][1]*T13435[1][4] + Si3435[2][2]*T13435[2][4]) - UPPERARM*S3534[2][2]*(Si3435[2][1]*T13435[1][5] + Si3435[2][2]*T13435[2][5]);
T3435[2][2]=S3534[1][2]*(Si3435[2][1]*T13435[1][1] + Si3435[2][2]*T13435[2][1]) + S3534[2][2]*(Si3435[2][1]*T13435[1][2] + Si3435[2][2]*T13435[2][2]) - UPPERARM*(Si3435[2][1]*T13435[1][6] + Si3435[2][2]*T13435[2][6]);
T3435[2][3]=S3534[1][3]*(Si3435[2][1]*T13435[1][1] + Si3435[2][2]*T13435[2][1]) + S3534[2][3]*(Si3435[2][1]*T13435[1][2] + Si3435[2][2]*T13435[2][2]);
T3435[2][4]=-(Si3435[2][1]*T13435[1][6]) - Si3435[2][2]*T13435[2][6];
T3435[2][5]=S3534[1][2]*(Si3435[2][1]*T13435[1][4] + Si3435[2][2]*T13435[2][4]) + S3534[2][2]*(Si3435[2][1]*T13435[1][5] + Si3435[2][2]*T13435[2][5]);
T3435[2][6]=S3534[1][3]*(Si3435[2][1]*T13435[1][4] + Si3435[2][2]*T13435[2][4]) + S3534[2][3]*(Si3435[2][1]*T13435[1][5] + Si3435[2][2]*T13435[2][5]);

T3435[3][1]=-(Si3435[3][1]*T13435[1][3]) - Si3435[3][2]*T13435[2][3] - UPPERARM*S3534[1][2]*(Si3435[3][1]*T13435[1][4] + Si3435[3][2]*T13435[2][4]) - UPPERARM*S3534[2][2]*(Si3435[3][1]*T13435[1][5] + Si3435[3][2]*T13435[2][5]);
T3435[3][2]=S3534[1][2]*(Si3435[3][1]*T13435[1][1] + Si3435[3][2]*T13435[2][1]) + S3534[2][2]*(Si3435[3][1]*T13435[1][2] + Si3435[3][2]*T13435[2][2]) - UPPERARM*(Si3435[3][1]*T13435[1][6] + Si3435[3][2]*T13435[2][6]);
T3435[3][3]=S3534[1][3]*(Si3435[3][1]*T13435[1][1] + Si3435[3][2]*T13435[2][1]) + S3534[2][3]*(Si3435[3][1]*T13435[1][2] + Si3435[3][2]*T13435[2][2]);
T3435[3][4]=-(Si3435[3][1]*T13435[1][6]) - Si3435[3][2]*T13435[2][6];
T3435[3][5]=S3534[1][2]*(Si3435[3][1]*T13435[1][4] + Si3435[3][2]*T13435[2][4]) + S3534[2][2]*(Si3435[3][1]*T13435[1][5] + Si3435[3][2]*T13435[2][5]);
T3435[3][6]=S3534[1][3]*(Si3435[3][1]*T13435[1][4] + Si3435[3][2]*T13435[2][4]) + S3534[2][3]*(Si3435[3][1]*T13435[1][5] + Si3435[3][2]*T13435[2][5]);

T3435[4][1]=UPPERARM*Si3435[2][1]*T13435[1][3] + UPPERARM*Si3435[2][2]*T13435[2][3] + T13435[6][3] - UPPERARM*S3534[1][2]*(-(UPPERARM*Si3435[2][1]*T13435[1][4]) - UPPERARM*Si3435[2][2]*T13435[2][4] - T13435[6][4]) - UPPERARM*S3534[2][2]*(-(UPPERARM*Si3435[2][1]*T13435[1][5]) - UPPERARM*Si3435[2][2]*T13435[2][5] - T13435[6][5]);
T3435[4][2]=S3534[1][2]*(-(UPPERARM*Si3435[2][1]*T13435[1][1]) - UPPERARM*Si3435[2][2]*T13435[2][1] - T13435[6][1]) + S3534[2][2]*(-(UPPERARM*Si3435[2][1]*T13435[1][2]) - UPPERARM*Si3435[2][2]*T13435[2][2] - T13435[6][2]) - UPPERARM*(-(UPPERARM*Si3435[2][1]*T13435[1][6]) - UPPERARM*Si3435[2][2]*T13435[2][6] - T13435[6][6]);
T3435[4][3]=S3534[1][3]*(-(UPPERARM*Si3435[2][1]*T13435[1][1]) - UPPERARM*Si3435[2][2]*T13435[2][1] - T13435[6][1]) + S3534[2][3]*(-(UPPERARM*Si3435[2][1]*T13435[1][2]) - UPPERARM*Si3435[2][2]*T13435[2][2] - T13435[6][2]);
T3435[4][4]=UPPERARM*Si3435[2][1]*T13435[1][6] + UPPERARM*Si3435[2][2]*T13435[2][6] + T13435[6][6];
T3435[4][5]=S3534[1][2]*(-(UPPERARM*Si3435[2][1]*T13435[1][4]) - UPPERARM*Si3435[2][2]*T13435[2][4] - T13435[6][4]) + S3534[2][2]*(-(UPPERARM*Si3435[2][1]*T13435[1][5]) - UPPERARM*Si3435[2][2]*T13435[2][5] - T13435[6][5]);
T3435[4][6]=S3534[1][3]*(-(UPPERARM*Si3435[2][1]*T13435[1][4]) - UPPERARM*Si3435[2][2]*T13435[2][4] - T13435[6][4]) + S3534[2][3]*(-(UPPERARM*Si3435[2][1]*T13435[1][5]) - UPPERARM*Si3435[2][2]*T13435[2][5] - T13435[6][5]);

T3435[5][1]=UPPERARM*T13435[3][3] - Si3435[2][1]*T13435[4][3] - Si3435[2][2]*T13435[5][3] - UPPERARM*S3534[1][2]*(-(UPPERARM*T13435[3][4]) + Si3435[2][1]*T13435[4][4] + Si3435[2][2]*T13435[5][4]) - UPPERARM*S3534[2][2]*(-(UPPERARM*T13435[3][5]) + Si3435[2][1]*T13435[4][5] + Si3435[2][2]*T13435[5][5]);
T3435[5][2]=S3534[1][2]*(-(UPPERARM*T13435[3][1]) + Si3435[2][1]*T13435[4][1] + Si3435[2][2]*T13435[5][1]) + S3534[2][2]*(-(UPPERARM*T13435[3][2]) + Si3435[2][1]*T13435[4][2] + Si3435[2][2]*T13435[5][2]) - UPPERARM*(-(UPPERARM*T13435[3][6]) + Si3435[2][1]*T13435[4][6] + Si3435[2][2]*T13435[5][6]);
T3435[5][3]=S3534[1][3]*(-(UPPERARM*T13435[3][1]) + Si3435[2][1]*T13435[4][1] + Si3435[2][2]*T13435[5][1]) + S3534[2][3]*(-(UPPERARM*T13435[3][2]) + Si3435[2][1]*T13435[4][2] + Si3435[2][2]*T13435[5][2]);
T3435[5][4]=UPPERARM*T13435[3][6] - Si3435[2][1]*T13435[4][6] - Si3435[2][2]*T13435[5][6];
T3435[5][5]=S3534[1][2]*(-(UPPERARM*T13435[3][4]) + Si3435[2][1]*T13435[4][4] + Si3435[2][2]*T13435[5][4]) + S3534[2][2]*(-(UPPERARM*T13435[3][5]) + Si3435[2][1]*T13435[4][5] + Si3435[2][2]*T13435[5][5]);
T3435[5][6]=S3534[1][3]*(-(UPPERARM*T13435[3][4]) + Si3435[2][1]*T13435[4][4] + Si3435[2][2]*T13435[5][4]) + S3534[2][3]*(-(UPPERARM*T13435[3][5]) + Si3435[2][1]*T13435[4][5] + Si3435[2][2]*T13435[5][5]);

T3435[6][1]=-(Si3435[3][1]*T13435[4][3]) - Si3435[3][2]*T13435[5][3] - UPPERARM*S3534[1][2]*(Si3435[3][1]*T13435[4][4] + Si3435[3][2]*T13435[5][4]) - UPPERARM*S3534[2][2]*(Si3435[3][1]*T13435[4][5] + Si3435[3][2]*T13435[5][5]);
T3435[6][2]=S3534[1][2]*(Si3435[3][1]*T13435[4][1] + Si3435[3][2]*T13435[5][1]) + S3534[2][2]*(Si3435[3][1]*T13435[4][2] + Si3435[3][2]*T13435[5][2]) - UPPERARM*(Si3435[3][1]*T13435[4][6] + Si3435[3][2]*T13435[5][6]);
T3435[6][3]=S3534[1][3]*(Si3435[3][1]*T13435[4][1] + Si3435[3][2]*T13435[5][1]) + S3534[2][3]*(Si3435[3][1]*T13435[4][2] + Si3435[3][2]*T13435[5][2]);
T3435[6][4]=-(Si3435[3][1]*T13435[4][6]) - Si3435[3][2]*T13435[5][6];
T3435[6][5]=S3534[1][2]*(Si3435[3][1]*T13435[4][4] + Si3435[3][2]*T13435[5][4]) + S3534[2][2]*(Si3435[3][1]*T13435[4][5] + Si3435[3][2]*T13435[5][5]);
T3435[6][6]=S3534[1][3]*(Si3435[3][1]*T13435[4][4] + Si3435[3][2]*T13435[5][4]) + S3534[2][3]*(Si3435[3][1]*T13435[4][5] + Si3435[3][2]*T13435[5][5]);



}


void
hermes_InvDynArtfunc70(void)
      {
JA34[1][1]=T3435[1][1];
JA34[1][2]=links[10].mcm[3] + T3435[1][2];
JA34[1][3]=-links[10].mcm[2] + T3435[1][3];
JA34[1][4]=links[10].m + T3435[1][4];
JA34[1][5]=T3435[1][5];
JA34[1][6]=T3435[1][6];

JA34[2][1]=-links[10].mcm[3] + T3435[2][1];
JA34[2][2]=T3435[2][2];
JA34[2][3]=links[10].mcm[1] + T3435[2][3];
JA34[2][4]=T3435[2][4];
JA34[2][5]=links[10].m + T3435[2][5];
JA34[2][6]=T3435[2][6];

JA34[3][1]=links[10].mcm[2] + T3435[3][1];
JA34[3][2]=-links[10].mcm[1] + T3435[3][2];
JA34[3][3]=T3435[3][3];
JA34[3][4]=T3435[3][4];
JA34[3][5]=T3435[3][5];
JA34[3][6]=links[10].m + T3435[3][6];

JA34[4][1]=links[10].inertia[1][1] + T3435[4][1];
JA34[4][2]=links[10].inertia[1][2] + T3435[4][2];
JA34[4][3]=links[10].inertia[1][3] + T3435[4][3];
JA34[4][4]=T3435[4][4];
JA34[4][5]=-links[10].mcm[3] + T3435[4][5];
JA34[4][6]=links[10].mcm[2] + T3435[4][6];

JA34[5][1]=links[10].inertia[1][2] + T3435[5][1];
JA34[5][2]=links[10].inertia[2][2] + T3435[5][2];
JA34[5][3]=links[10].inertia[2][3] + T3435[5][3];
JA34[5][4]=links[10].mcm[3] + T3435[5][4];
JA34[5][5]=T3435[5][5];
JA34[5][6]=-links[10].mcm[1] + T3435[5][6];

JA34[6][1]=links[10].inertia[1][3] + T3435[6][1];
JA34[6][2]=links[10].inertia[2][3] + T3435[6][2];
JA34[6][3]=links[10].inertia[3][3] + T3435[6][3];
JA34[6][4]=-links[10].mcm[2] + T3435[6][4];
JA34[6][5]=links[10].mcm[1] + T3435[6][5];
JA34[6][6]=T3435[6][6];


h34[1]=JA34[1][3];
h34[2]=JA34[2][3];
h34[3]=JA34[3][3];
h34[4]=JA34[4][3];
h34[5]=JA34[5][3];
h34[6]=JA34[6][3];

T13334[1][1]=JA34[1][1];
T13334[1][2]=JA34[1][2];
T13334[1][3]=JA34[1][3];
T13334[1][4]=JA34[1][4];
T13334[1][5]=JA34[1][5];
T13334[1][6]=JA34[1][6];

T13334[2][1]=JA34[2][1];
T13334[2][2]=JA34[2][2];
T13334[2][3]=JA34[2][3];
T13334[2][4]=JA34[2][4];
T13334[2][5]=JA34[2][5];
T13334[2][6]=JA34[2][6];

T13334[3][1]=JA34[3][1];
T13334[3][2]=JA34[3][2];
T13334[3][3]=JA34[3][3];
T13334[3][4]=JA34[3][4];
T13334[3][5]=JA34[3][5];
T13334[3][6]=JA34[3][6];

T13334[4][1]=JA34[4][1];
T13334[4][2]=JA34[4][2];
T13334[4][3]=JA34[4][3];
T13334[4][4]=JA34[4][4];
T13334[4][5]=JA34[4][5];
T13334[4][6]=JA34[4][6];

T13334[5][1]=JA34[5][1];
T13334[5][2]=JA34[5][2];
T13334[5][3]=JA34[5][3];
T13334[5][4]=JA34[5][4];
T13334[5][5]=JA34[5][5];
T13334[5][6]=JA34[5][6];

T13334[6][1]=JA34[6][1];
T13334[6][2]=JA34[6][2];
T13334[6][3]=JA34[6][3];
T13334[6][4]=JA34[6][4];
T13334[6][5]=JA34[6][5];
T13334[6][6]=JA34[6][6];


T3334[1][1]=S3433[1][1]*(Si3334[1][1]*T13334[1][1] + Si3334[1][2]*T13334[2][1]) + S3433[2][1]*(Si3334[1][1]*T13334[1][2] + Si3334[1][2]*T13334[2][2]);
T3334[1][2]=-(Si3334[1][1]*T13334[1][3]) - Si3334[1][2]*T13334[2][3] + SHOULDERY*S3433[1][3]*(Si3334[1][1]*T13334[1][4] + Si3334[1][2]*T13334[2][4]) + SHOULDERY*S3433[2][3]*(Si3334[1][1]*T13334[1][5] + Si3334[1][2]*T13334[2][5]);
T3334[1][3]=S3433[1][3]*(Si3334[1][1]*T13334[1][1] + Si3334[1][2]*T13334[2][1]) + S3433[2][3]*(Si3334[1][1]*T13334[1][2] + Si3334[1][2]*T13334[2][2]) + SHOULDERY*(Si3334[1][1]*T13334[1][6] + Si3334[1][2]*T13334[2][6]);
T3334[1][4]=S3433[1][1]*(Si3334[1][1]*T13334[1][4] + Si3334[1][2]*T13334[2][4]) + S3433[2][1]*(Si3334[1][1]*T13334[1][5] + Si3334[1][2]*T13334[2][5]);
T3334[1][5]=-(Si3334[1][1]*T13334[1][6]) - Si3334[1][2]*T13334[2][6];
T3334[1][6]=S3433[1][3]*(Si3334[1][1]*T13334[1][4] + Si3334[1][2]*T13334[2][4]) + S3433[2][3]*(Si3334[1][1]*T13334[1][5] + Si3334[1][2]*T13334[2][5]);

T3334[2][1]=-(S3433[1][1]*T13334[3][1]) - S3433[2][1]*T13334[3][2];
T3334[2][2]=T13334[3][3] - SHOULDERY*S3433[1][3]*T13334[3][4] - SHOULDERY*S3433[2][3]*T13334[3][5];
T3334[2][3]=-(S3433[1][3]*T13334[3][1]) - S3433[2][3]*T13334[3][2] - SHOULDERY*T13334[3][6];
T3334[2][4]=-(S3433[1][1]*T13334[3][4]) - S3433[2][1]*T13334[3][5];
T3334[2][5]=T13334[3][6];
T3334[2][6]=-(S3433[1][3]*T13334[3][4]) - S3433[2][3]*T13334[3][5];

T3334[3][1]=S3433[1][1]*(Si3334[3][1]*T13334[1][1] + Si3334[3][2]*T13334[2][1]) + S3433[2][1]*(Si3334[3][1]*T13334[1][2] + Si3334[3][2]*T13334[2][2]);
T3334[3][2]=-(Si3334[3][1]*T13334[1][3]) - Si3334[3][2]*T13334[2][3] + SHOULDERY*S3433[1][3]*(Si3334[3][1]*T13334[1][4] + Si3334[3][2]*T13334[2][4]) + SHOULDERY*S3433[2][3]*(Si3334[3][1]*T13334[1][5] + Si3334[3][2]*T13334[2][5]);
T3334[3][3]=S3433[1][3]*(Si3334[3][1]*T13334[1][1] + Si3334[3][2]*T13334[2][1]) + S3433[2][3]*(Si3334[3][1]*T13334[1][2] + Si3334[3][2]*T13334[2][2]) + SHOULDERY*(Si3334[3][1]*T13334[1][6] + Si3334[3][2]*T13334[2][6]);
T3334[3][4]=S3433[1][1]*(Si3334[3][1]*T13334[1][4] + Si3334[3][2]*T13334[2][4]) + S3433[2][1]*(Si3334[3][1]*T13334[1][5] + Si3334[3][2]*T13334[2][5]);
T3334[3][5]=-(Si3334[3][1]*T13334[1][6]) - Si3334[3][2]*T13334[2][6];
T3334[3][6]=S3433[1][3]*(Si3334[3][1]*T13334[1][4] + Si3334[3][2]*T13334[2][4]) + S3433[2][3]*(Si3334[3][1]*T13334[1][5] + Si3334[3][2]*T13334[2][5]);

T3334[4][1]=S3433[1][1]*(Si3334[1][1]*T13334[4][1] + Si3334[1][2]*T13334[5][1]) + S3433[2][1]*(Si3334[1][1]*T13334[4][2] + Si3334[1][2]*T13334[5][2]);
T3334[4][2]=-(Si3334[1][1]*T13334[4][3]) - Si3334[1][2]*T13334[5][3] + SHOULDERY*S3433[1][3]*(Si3334[1][1]*T13334[4][4] + Si3334[1][2]*T13334[5][4]) + SHOULDERY*S3433[2][3]*(Si3334[1][1]*T13334[4][5] + Si3334[1][2]*T13334[5][5]);
T3334[4][3]=S3433[1][3]*(Si3334[1][1]*T13334[4][1] + Si3334[1][2]*T13334[5][1]) + S3433[2][3]*(Si3334[1][1]*T13334[4][2] + Si3334[1][2]*T13334[5][2]) + SHOULDERY*(Si3334[1][1]*T13334[4][6] + Si3334[1][2]*T13334[5][6]);
T3334[4][4]=S3433[1][1]*(Si3334[1][1]*T13334[4][4] + Si3334[1][2]*T13334[5][4]) + S3433[2][1]*(Si3334[1][1]*T13334[4][5] + Si3334[1][2]*T13334[5][5]);
T3334[4][5]=-(Si3334[1][1]*T13334[4][6]) - Si3334[1][2]*T13334[5][6];
T3334[4][6]=S3433[1][3]*(Si3334[1][1]*T13334[4][4] + Si3334[1][2]*T13334[5][4]) + S3433[2][3]*(Si3334[1][1]*T13334[4][5] + Si3334[1][2]*T13334[5][5]);

T3334[5][1]=S3433[1][1]*(SHOULDERY*Si3334[3][1]*T13334[1][1] + SHOULDERY*Si3334[3][2]*T13334[2][1] - T13334[6][1]) + S3433[2][1]*(SHOULDERY*Si3334[3][1]*T13334[1][2] + SHOULDERY*Si3334[3][2]*T13334[2][2] - T13334[6][2]);
T3334[5][2]=-(SHOULDERY*Si3334[3][1]*T13334[1][3]) - SHOULDERY*Si3334[3][2]*T13334[2][3] + T13334[6][3] + SHOULDERY*S3433[1][3]*(SHOULDERY*Si3334[3][1]*T13334[1][4] + SHOULDERY*Si3334[3][2]*T13334[2][4] - T13334[6][4]) + SHOULDERY*S3433[2][3]*(SHOULDERY*Si3334[3][1]*T13334[1][5] + SHOULDERY*Si3334[3][2]*T13334[2][5] - T13334[6][5]);
T3334[5][3]=S3433[1][3]*(SHOULDERY*Si3334[3][1]*T13334[1][1] + SHOULDERY*Si3334[3][2]*T13334[2][1] - T13334[6][1]) + S3433[2][3]*(SHOULDERY*Si3334[3][1]*T13334[1][2] + SHOULDERY*Si3334[3][2]*T13334[2][2] - T13334[6][2]) + SHOULDERY*(SHOULDERY*Si3334[3][1]*T13334[1][6] + SHOULDERY*Si3334[3][2]*T13334[2][6] - T13334[6][6]);
T3334[5][4]=S3433[1][1]*(SHOULDERY*Si3334[3][1]*T13334[1][4] + SHOULDERY*Si3334[3][2]*T13334[2][4] - T13334[6][4]) + S3433[2][1]*(SHOULDERY*Si3334[3][1]*T13334[1][5] + SHOULDERY*Si3334[3][2]*T13334[2][5] - T13334[6][5]);
T3334[5][5]=-(SHOULDERY*Si3334[3][1]*T13334[1][6]) - SHOULDERY*Si3334[3][2]*T13334[2][6] + T13334[6][6];
T3334[5][6]=S3433[1][3]*(SHOULDERY*Si3334[3][1]*T13334[1][4] + SHOULDERY*Si3334[3][2]*T13334[2][4] - T13334[6][4]) + S3433[2][3]*(SHOULDERY*Si3334[3][1]*T13334[1][5] + SHOULDERY*Si3334[3][2]*T13334[2][5] - T13334[6][5]);

T3334[6][1]=S3433[1][1]*(SHOULDERY*T13334[3][1] + Si3334[3][1]*T13334[4][1] + Si3334[3][2]*T13334[5][1]) + S3433[2][1]*(SHOULDERY*T13334[3][2] + Si3334[3][1]*T13334[4][2] + Si3334[3][2]*T13334[5][2]);
T3334[6][2]=-(SHOULDERY*T13334[3][3]) - Si3334[3][1]*T13334[4][3] - Si3334[3][2]*T13334[5][3] + SHOULDERY*S3433[1][3]*(SHOULDERY*T13334[3][4] + Si3334[3][1]*T13334[4][4] + Si3334[3][2]*T13334[5][4]) + SHOULDERY*S3433[2][3]*(SHOULDERY*T13334[3][5] + Si3334[3][1]*T13334[4][5] + Si3334[3][2]*T13334[5][5]);
T3334[6][3]=S3433[1][3]*(SHOULDERY*T13334[3][1] + Si3334[3][1]*T13334[4][1] + Si3334[3][2]*T13334[5][1]) + S3433[2][3]*(SHOULDERY*T13334[3][2] + Si3334[3][1]*T13334[4][2] + Si3334[3][2]*T13334[5][2]) + SHOULDERY*(SHOULDERY*T13334[3][6] + Si3334[3][1]*T13334[4][6] + Si3334[3][2]*T13334[5][6]);
T3334[6][4]=S3433[1][1]*(SHOULDERY*T13334[3][4] + Si3334[3][1]*T13334[4][4] + Si3334[3][2]*T13334[5][4]) + S3433[2][1]*(SHOULDERY*T13334[3][5] + Si3334[3][1]*T13334[4][5] + Si3334[3][2]*T13334[5][5]);
T3334[6][5]=-(SHOULDERY*T13334[3][6]) - Si3334[3][1]*T13334[4][6] - Si3334[3][2]*T13334[5][6];
T3334[6][6]=S3433[1][3]*(SHOULDERY*T13334[3][4] + Si3334[3][1]*T13334[4][4] + Si3334[3][2]*T13334[5][4]) + S3433[2][3]*(SHOULDERY*T13334[3][5] + Si3334[3][1]*T13334[4][5] + Si3334[3][2]*T13334[5][5]);



}


void
hermes_InvDynArtfunc71(void)
      {
JA33[1][1]=T3334[1][1];
JA33[1][2]=links[9].mcm[3] + T3334[1][2];
JA33[1][3]=-links[9].mcm[2] + T3334[1][3];
JA33[1][4]=links[9].m + T3334[1][4];
JA33[1][5]=T3334[1][5];
JA33[1][6]=T3334[1][6];

JA33[2][1]=-links[9].mcm[3] + T3334[2][1];
JA33[2][2]=T3334[2][2];
JA33[2][3]=links[9].mcm[1] + T3334[2][3];
JA33[2][4]=T3334[2][4];
JA33[2][5]=links[9].m + T3334[2][5];
JA33[2][6]=T3334[2][6];

JA33[3][1]=links[9].mcm[2] + T3334[3][1];
JA33[3][2]=-links[9].mcm[1] + T3334[3][2];
JA33[3][3]=T3334[3][3];
JA33[3][4]=T3334[3][4];
JA33[3][5]=T3334[3][5];
JA33[3][6]=links[9].m + T3334[3][6];

JA33[4][1]=links[9].inertia[1][1] + T3334[4][1];
JA33[4][2]=links[9].inertia[1][2] + T3334[4][2];
JA33[4][3]=links[9].inertia[1][3] + T3334[4][3];
JA33[4][4]=T3334[4][4];
JA33[4][5]=-links[9].mcm[3] + T3334[4][5];
JA33[4][6]=links[9].mcm[2] + T3334[4][6];

JA33[5][1]=links[9].inertia[1][2] + T3334[5][1];
JA33[5][2]=links[9].inertia[2][2] + T3334[5][2];
JA33[5][3]=links[9].inertia[2][3] + T3334[5][3];
JA33[5][4]=links[9].mcm[3] + T3334[5][4];
JA33[5][5]=T3334[5][5];
JA33[5][6]=-links[9].mcm[1] + T3334[5][6];

JA33[6][1]=links[9].inertia[1][3] + T3334[6][1];
JA33[6][2]=links[9].inertia[2][3] + T3334[6][2];
JA33[6][3]=links[9].inertia[3][3] + T3334[6][3];
JA33[6][4]=-links[9].mcm[2] + T3334[6][4];
JA33[6][5]=links[9].mcm[1] + T3334[6][5];
JA33[6][6]=T3334[6][6];


h33[1]=JA33[1][3];
h33[2]=JA33[2][3];
h33[3]=JA33[3][3];
h33[4]=JA33[4][3];
h33[5]=JA33[5][3];
h33[6]=JA33[6][3];

T13233[1][1]=JA33[1][1];
T13233[1][2]=JA33[1][2];
T13233[1][3]=JA33[1][3];
T13233[1][4]=JA33[1][4];
T13233[1][5]=JA33[1][5];
T13233[1][6]=JA33[1][6];

T13233[2][1]=JA33[2][1];
T13233[2][2]=JA33[2][2];
T13233[2][3]=JA33[2][3];
T13233[2][4]=JA33[2][4];
T13233[2][5]=JA33[2][5];
T13233[2][6]=JA33[2][6];

T13233[3][1]=JA33[3][1];
T13233[3][2]=JA33[3][2];
T13233[3][3]=JA33[3][3];
T13233[3][4]=JA33[3][4];
T13233[3][5]=JA33[3][5];
T13233[3][6]=JA33[3][6];

T13233[4][1]=JA33[4][1];
T13233[4][2]=JA33[4][2];
T13233[4][3]=JA33[4][3];
T13233[4][4]=JA33[4][4];
T13233[4][5]=JA33[4][5];
T13233[4][6]=JA33[4][6];

T13233[5][1]=JA33[5][1];
T13233[5][2]=JA33[5][2];
T13233[5][3]=JA33[5][3];
T13233[5][4]=JA33[5][4];
T13233[5][5]=JA33[5][5];
T13233[5][6]=JA33[5][6];

T13233[6][1]=JA33[6][1];
T13233[6][2]=JA33[6][2];
T13233[6][3]=JA33[6][3];
T13233[6][4]=JA33[6][4];
T13233[6][5]=JA33[6][5];
T13233[6][6]=JA33[6][6];


T3233[1][1]=S3332[1][1]*(Si3233[1][1]*T13233[1][1] + Si3233[1][2]*T13233[2][1]) + S3332[2][1]*(Si3233[1][1]*T13233[1][2] + Si3233[1][2]*T13233[2][2]) - SHOULDERX*(Si3233[1][1]*T13233[1][6] + Si3233[1][2]*T13233[2][6]);
T3233[1][2]=Si3233[1][1]*T13233[1][3] + Si3233[1][2]*T13233[2][3] + SHOULDERX*S3332[1][1]*(Si3233[1][1]*T13233[1][4] + Si3233[1][2]*T13233[2][4]) + SHOULDERX*S3332[2][1]*(Si3233[1][1]*T13233[1][5] + Si3233[1][2]*T13233[2][5]);
T3233[1][3]=S3332[1][3]*(Si3233[1][1]*T13233[1][1] + Si3233[1][2]*T13233[2][1]) + S3332[2][3]*(Si3233[1][1]*T13233[1][2] + Si3233[1][2]*T13233[2][2]);
T3233[1][4]=S3332[1][1]*(Si3233[1][1]*T13233[1][4] + Si3233[1][2]*T13233[2][4]) + S3332[2][1]*(Si3233[1][1]*T13233[1][5] + Si3233[1][2]*T13233[2][5]);
T3233[1][5]=Si3233[1][1]*T13233[1][6] + Si3233[1][2]*T13233[2][6];
T3233[1][6]=S3332[1][3]*(Si3233[1][1]*T13233[1][4] + Si3233[1][2]*T13233[2][4]) + S3332[2][3]*(Si3233[1][1]*T13233[1][5] + Si3233[1][2]*T13233[2][5]);

T3233[2][1]=S3332[1][1]*T13233[3][1] + S3332[2][1]*T13233[3][2] - SHOULDERX*T13233[3][6];
T3233[2][2]=T13233[3][3] + SHOULDERX*S3332[1][1]*T13233[3][4] + SHOULDERX*S3332[2][1]*T13233[3][5];
T3233[2][3]=S3332[1][3]*T13233[3][1] + S3332[2][3]*T13233[3][2];
T3233[2][4]=S3332[1][1]*T13233[3][4] + S3332[2][1]*T13233[3][5];
T3233[2][5]=T13233[3][6];
T3233[2][6]=S3332[1][3]*T13233[3][4] + S3332[2][3]*T13233[3][5];

T3233[3][1]=S3332[1][1]*(Si3233[3][1]*T13233[1][1] + Si3233[3][2]*T13233[2][1]) + S3332[2][1]*(Si3233[3][1]*T13233[1][2] + Si3233[3][2]*T13233[2][2]) - SHOULDERX*(Si3233[3][1]*T13233[1][6] + Si3233[3][2]*T13233[2][6]);
T3233[3][2]=Si3233[3][1]*T13233[1][3] + Si3233[3][2]*T13233[2][3] + SHOULDERX*S3332[1][1]*(Si3233[3][1]*T13233[1][4] + Si3233[3][2]*T13233[2][4]) + SHOULDERX*S3332[2][1]*(Si3233[3][1]*T13233[1][5] + Si3233[3][2]*T13233[2][5]);
T3233[3][3]=S3332[1][3]*(Si3233[3][1]*T13233[1][1] + Si3233[3][2]*T13233[2][1]) + S3332[2][3]*(Si3233[3][1]*T13233[1][2] + Si3233[3][2]*T13233[2][2]);
T3233[3][4]=S3332[1][1]*(Si3233[3][1]*T13233[1][4] + Si3233[3][2]*T13233[2][4]) + S3332[2][1]*(Si3233[3][1]*T13233[1][5] + Si3233[3][2]*T13233[2][5]);
T3233[3][5]=Si3233[3][1]*T13233[1][6] + Si3233[3][2]*T13233[2][6];
T3233[3][6]=S3332[1][3]*(Si3233[3][1]*T13233[1][4] + Si3233[3][2]*T13233[2][4]) + S3332[2][3]*(Si3233[3][1]*T13233[1][5] + Si3233[3][2]*T13233[2][5]);

T3233[4][1]=S3332[1][1]*(-(SHOULDERX*T13233[3][1]) + Si3233[1][1]*T13233[4][1] + Si3233[1][2]*T13233[5][1]) + S3332[2][1]*(-(SHOULDERX*T13233[3][2]) + Si3233[1][1]*T13233[4][2] + Si3233[1][2]*T13233[5][2]) - SHOULDERX*(-(SHOULDERX*T13233[3][6]) + Si3233[1][1]*T13233[4][6] + Si3233[1][2]*T13233[5][6]);
T3233[4][2]=-(SHOULDERX*T13233[3][3]) + Si3233[1][1]*T13233[4][3] + Si3233[1][2]*T13233[5][3] + SHOULDERX*S3332[1][1]*(-(SHOULDERX*T13233[3][4]) + Si3233[1][1]*T13233[4][4] + Si3233[1][2]*T13233[5][4]) + SHOULDERX*S3332[2][1]*(-(SHOULDERX*T13233[3][5]) + Si3233[1][1]*T13233[4][5] + Si3233[1][2]*T13233[5][5]);
T3233[4][3]=S3332[1][3]*(-(SHOULDERX*T13233[3][1]) + Si3233[1][1]*T13233[4][1] + Si3233[1][2]*T13233[5][1]) + S3332[2][3]*(-(SHOULDERX*T13233[3][2]) + Si3233[1][1]*T13233[4][2] + Si3233[1][2]*T13233[5][2]);
T3233[4][4]=S3332[1][1]*(-(SHOULDERX*T13233[3][4]) + Si3233[1][1]*T13233[4][4] + Si3233[1][2]*T13233[5][4]) + S3332[2][1]*(-(SHOULDERX*T13233[3][5]) + Si3233[1][1]*T13233[4][5] + Si3233[1][2]*T13233[5][5]);
T3233[4][5]=-(SHOULDERX*T13233[3][6]) + Si3233[1][1]*T13233[4][6] + Si3233[1][2]*T13233[5][6];
T3233[4][6]=S3332[1][3]*(-(SHOULDERX*T13233[3][4]) + Si3233[1][1]*T13233[4][4] + Si3233[1][2]*T13233[5][4]) + S3332[2][3]*(-(SHOULDERX*T13233[3][5]) + Si3233[1][1]*T13233[4][5] + Si3233[1][2]*T13233[5][5]);

T3233[5][1]=S3332[1][1]*(SHOULDERX*Si3233[1][1]*T13233[1][1] + SHOULDERX*Si3233[1][2]*T13233[2][1] + T13233[6][1]) + S3332[2][1]*(SHOULDERX*Si3233[1][1]*T13233[1][2] + SHOULDERX*Si3233[1][2]*T13233[2][2] + T13233[6][2]) - SHOULDERX*(SHOULDERX*Si3233[1][1]*T13233[1][6] + SHOULDERX*Si3233[1][2]*T13233[2][6] + T13233[6][6]);
T3233[5][2]=SHOULDERX*Si3233[1][1]*T13233[1][3] + SHOULDERX*Si3233[1][2]*T13233[2][3] + T13233[6][3] + SHOULDERX*S3332[1][1]*(SHOULDERX*Si3233[1][1]*T13233[1][4] + SHOULDERX*Si3233[1][2]*T13233[2][4] + T13233[6][4]) + SHOULDERX*S3332[2][1]*(SHOULDERX*Si3233[1][1]*T13233[1][5] + SHOULDERX*Si3233[1][2]*T13233[2][5] + T13233[6][5]);
T3233[5][3]=S3332[1][3]*(SHOULDERX*Si3233[1][1]*T13233[1][1] + SHOULDERX*Si3233[1][2]*T13233[2][1] + T13233[6][1]) + S3332[2][3]*(SHOULDERX*Si3233[1][1]*T13233[1][2] + SHOULDERX*Si3233[1][2]*T13233[2][2] + T13233[6][2]);
T3233[5][4]=S3332[1][1]*(SHOULDERX*Si3233[1][1]*T13233[1][4] + SHOULDERX*Si3233[1][2]*T13233[2][4] + T13233[6][4]) + S3332[2][1]*(SHOULDERX*Si3233[1][1]*T13233[1][5] + SHOULDERX*Si3233[1][2]*T13233[2][5] + T13233[6][5]);
T3233[5][5]=SHOULDERX*Si3233[1][1]*T13233[1][6] + SHOULDERX*Si3233[1][2]*T13233[2][6] + T13233[6][6];
T3233[5][6]=S3332[1][3]*(SHOULDERX*Si3233[1][1]*T13233[1][4] + SHOULDERX*Si3233[1][2]*T13233[2][4] + T13233[6][4]) + S3332[2][3]*(SHOULDERX*Si3233[1][1]*T13233[1][5] + SHOULDERX*Si3233[1][2]*T13233[2][5] + T13233[6][5]);

T3233[6][1]=S3332[1][1]*(Si3233[3][1]*T13233[4][1] + Si3233[3][2]*T13233[5][1]) + S3332[2][1]*(Si3233[3][1]*T13233[4][2] + Si3233[3][2]*T13233[5][2]) - SHOULDERX*(Si3233[3][1]*T13233[4][6] + Si3233[3][2]*T13233[5][6]);
T3233[6][2]=Si3233[3][1]*T13233[4][3] + Si3233[3][2]*T13233[5][3] + SHOULDERX*S3332[1][1]*(Si3233[3][1]*T13233[4][4] + Si3233[3][2]*T13233[5][4]) + SHOULDERX*S3332[2][1]*(Si3233[3][1]*T13233[4][5] + Si3233[3][2]*T13233[5][5]);
T3233[6][3]=S3332[1][3]*(Si3233[3][1]*T13233[4][1] + Si3233[3][2]*T13233[5][1]) + S3332[2][3]*(Si3233[3][1]*T13233[4][2] + Si3233[3][2]*T13233[5][2]);
T3233[6][4]=S3332[1][1]*(Si3233[3][1]*T13233[4][4] + Si3233[3][2]*T13233[5][4]) + S3332[2][1]*(Si3233[3][1]*T13233[4][5] + Si3233[3][2]*T13233[5][5]);
T3233[6][5]=Si3233[3][1]*T13233[4][6] + Si3233[3][2]*T13233[5][6];
T3233[6][6]=S3332[1][3]*(Si3233[3][1]*T13233[4][4] + Si3233[3][2]*T13233[5][4]) + S3332[2][3]*(Si3233[3][1]*T13233[4][5] + Si3233[3][2]*T13233[5][5]);



}


void
hermes_InvDynArtfunc72(void)
      {
JA32[1][1]=T3233[1][1];
JA32[1][2]=links[8].mcm[3] + T3233[1][2];
JA32[1][3]=-links[8].mcm[2] + T3233[1][3];
JA32[1][4]=links[8].m + T3233[1][4];
JA32[1][5]=T3233[1][5];
JA32[1][6]=T3233[1][6];

JA32[2][1]=-links[8].mcm[3] + T3233[2][1];
JA32[2][2]=T3233[2][2];
JA32[2][3]=links[8].mcm[1] + T3233[2][3];
JA32[2][4]=T3233[2][4];
JA32[2][5]=links[8].m + T3233[2][5];
JA32[2][6]=T3233[2][6];

JA32[3][1]=links[8].mcm[2] + T3233[3][1];
JA32[3][2]=-links[8].mcm[1] + T3233[3][2];
JA32[3][3]=T3233[3][3];
JA32[3][4]=T3233[3][4];
JA32[3][5]=T3233[3][5];
JA32[3][6]=links[8].m + T3233[3][6];

JA32[4][1]=links[8].inertia[1][1] + T3233[4][1];
JA32[4][2]=links[8].inertia[1][2] + T3233[4][2];
JA32[4][3]=links[8].inertia[1][3] + T3233[4][3];
JA32[4][4]=T3233[4][4];
JA32[4][5]=-links[8].mcm[3] + T3233[4][5];
JA32[4][6]=links[8].mcm[2] + T3233[4][6];

JA32[5][1]=links[8].inertia[1][2] + T3233[5][1];
JA32[5][2]=links[8].inertia[2][2] + T3233[5][2];
JA32[5][3]=links[8].inertia[2][3] + T3233[5][3];
JA32[5][4]=links[8].mcm[3] + T3233[5][4];
JA32[5][5]=T3233[5][5];
JA32[5][6]=-links[8].mcm[1] + T3233[5][6];

JA32[6][1]=links[8].inertia[1][3] + T3233[6][1];
JA32[6][2]=links[8].inertia[2][3] + T3233[6][2];
JA32[6][3]=links[8].inertia[3][3] + T3233[6][3];
JA32[6][4]=-links[8].mcm[2] + T3233[6][4];
JA32[6][5]=links[8].mcm[1] + T3233[6][5];
JA32[6][6]=T3233[6][6];


h32[1]=JA32[1][3];
h32[2]=JA32[2][3];
h32[3]=JA32[3][3];
h32[4]=JA32[4][3];
h32[5]=JA32[5][3];
h32[6]=JA32[6][3];

T1332[1][1]=JA32[1][1];
T1332[1][2]=JA32[1][2];
T1332[1][3]=JA32[1][3];
T1332[1][4]=JA32[1][4];
T1332[1][5]=JA32[1][5];
T1332[1][6]=JA32[1][6];

T1332[2][1]=JA32[2][1];
T1332[2][2]=JA32[2][2];
T1332[2][3]=JA32[2][3];
T1332[2][4]=JA32[2][4];
T1332[2][5]=JA32[2][5];
T1332[2][6]=JA32[2][6];

T1332[3][1]=JA32[3][1];
T1332[3][2]=JA32[3][2];
T1332[3][3]=JA32[3][3];
T1332[3][4]=JA32[3][4];
T1332[3][5]=JA32[3][5];
T1332[3][6]=JA32[3][6];

T1332[4][1]=JA32[4][1];
T1332[4][2]=JA32[4][2];
T1332[4][3]=JA32[4][3];
T1332[4][4]=JA32[4][4];
T1332[4][5]=JA32[4][5];
T1332[4][6]=JA32[4][6];

T1332[5][1]=JA32[5][1];
T1332[5][2]=JA32[5][2];
T1332[5][3]=JA32[5][3];
T1332[5][4]=JA32[5][4];
T1332[5][5]=JA32[5][5];
T1332[5][6]=JA32[5][6];

T1332[6][1]=JA32[6][1];
T1332[6][2]=JA32[6][2];
T1332[6][3]=JA32[6][3];
T1332[6][4]=JA32[6][4];
T1332[6][5]=JA32[6][5];
T1332[6][6]=JA32[6][6];


T332[1][1]=S323[1][1]*(Si332[1][1]*T1332[1][1] + Si332[1][2]*T1332[2][1] - 0.7071067811865475*T1332[3][1]) + S323[2][1]*(Si332[1][1]*T1332[1][2] + Si332[1][2]*T1332[2][2] - 0.7071067811865475*T1332[3][2]) - 0.7071067811865475*(Si332[1][1]*T1332[1][3] + Si332[1][2]*T1332[2][3] - 0.7071067811865475*T1332[3][3]);
T332[1][2]=S323[1][2]*(Si332[1][1]*T1332[1][1] + Si332[1][2]*T1332[2][1] - 0.7071067811865475*T1332[3][1]) + S323[2][2]*(Si332[1][1]*T1332[1][2] + Si332[1][2]*T1332[2][2] - 0.7071067811865475*T1332[3][2]) + THORAX2SHOULDER*S323[1][3]*(Si332[1][1]*T1332[1][4] + Si332[1][2]*T1332[2][4] - 0.7071067811865475*T1332[3][4]) + THORAX2SHOULDER*S323[2][3]*(Si332[1][1]*T1332[1][5] + Si332[1][2]*T1332[2][5] - 0.7071067811865475*T1332[3][5]) - 0.7071067811865475*THORAX2SHOULDER*(Si332[1][1]*T1332[1][6] + Si332[1][2]*T1332[2][6] - 0.7071067811865475*T1332[3][6]);
T332[1][3]=S323[1][3]*(Si332[1][1]*T1332[1][1] + Si332[1][2]*T1332[2][1] - 0.7071067811865475*T1332[3][1]) + S323[2][3]*(Si332[1][1]*T1332[1][2] + Si332[1][2]*T1332[2][2] - 0.7071067811865475*T1332[3][2]) - 0.7071067811865475*(Si332[1][1]*T1332[1][3] + Si332[1][2]*T1332[2][3] - 0.7071067811865475*T1332[3][3]) - THORAX2SHOULDER*S323[1][2]*(Si332[1][1]*T1332[1][4] + Si332[1][2]*T1332[2][4] - 0.7071067811865475*T1332[3][4]) - THORAX2SHOULDER*S323[2][2]*(Si332[1][1]*T1332[1][5] + Si332[1][2]*T1332[2][5] - 0.7071067811865475*T1332[3][5]);
T332[1][4]=S323[1][1]*(Si332[1][1]*T1332[1][4] + Si332[1][2]*T1332[2][4] - 0.7071067811865475*T1332[3][4]) + S323[2][1]*(Si332[1][1]*T1332[1][5] + Si332[1][2]*T1332[2][5] - 0.7071067811865475*T1332[3][5]) - 0.7071067811865475*(Si332[1][1]*T1332[1][6] + Si332[1][2]*T1332[2][6] - 0.7071067811865475*T1332[3][6]);
T332[1][5]=S323[1][2]*(Si332[1][1]*T1332[1][4] + Si332[1][2]*T1332[2][4] - 0.7071067811865475*T1332[3][4]) + S323[2][2]*(Si332[1][1]*T1332[1][5] + Si332[1][2]*T1332[2][5] - 0.7071067811865475*T1332[3][5]);
T332[1][6]=S323[1][3]*(Si332[1][1]*T1332[1][4] + Si332[1][2]*T1332[2][4] - 0.7071067811865475*T1332[3][4]) + S323[2][3]*(Si332[1][1]*T1332[1][5] + Si332[1][2]*T1332[2][5] - 0.7071067811865475*T1332[3][5]) - 0.7071067811865475*(Si332[1][1]*T1332[1][6] + Si332[1][2]*T1332[2][6] - 0.7071067811865475*T1332[3][6]);

T332[2][1]=S323[1][1]*(Si332[2][1]*T1332[1][1] + Si332[2][2]*T1332[2][1]) + S323[2][1]*(Si332[2][1]*T1332[1][2] + Si332[2][2]*T1332[2][2]) - 0.7071067811865475*(Si332[2][1]*T1332[1][3] + Si332[2][2]*T1332[2][3]);
T332[2][2]=S323[1][2]*(Si332[2][1]*T1332[1][1] + Si332[2][2]*T1332[2][1]) + S323[2][2]*(Si332[2][1]*T1332[1][2] + Si332[2][2]*T1332[2][2]) + THORAX2SHOULDER*S323[1][3]*(Si332[2][1]*T1332[1][4] + Si332[2][2]*T1332[2][4]) + THORAX2SHOULDER*S323[2][3]*(Si332[2][1]*T1332[1][5] + Si332[2][2]*T1332[2][5]) - 0.7071067811865475*THORAX2SHOULDER*(Si332[2][1]*T1332[1][6] + Si332[2][2]*T1332[2][6]);
T332[2][3]=S323[1][3]*(Si332[2][1]*T1332[1][1] + Si332[2][2]*T1332[2][1]) + S323[2][3]*(Si332[2][1]*T1332[1][2] + Si332[2][2]*T1332[2][2]) - 0.7071067811865475*(Si332[2][1]*T1332[1][3] + Si332[2][2]*T1332[2][3]) - THORAX2SHOULDER*S323[1][2]*(Si332[2][1]*T1332[1][4] + Si332[2][2]*T1332[2][4]) - THORAX2SHOULDER*S323[2][2]*(Si332[2][1]*T1332[1][5] + Si332[2][2]*T1332[2][5]);
T332[2][4]=S323[1][1]*(Si332[2][1]*T1332[1][4] + Si332[2][2]*T1332[2][4]) + S323[2][1]*(Si332[2][1]*T1332[1][5] + Si332[2][2]*T1332[2][5]) - 0.7071067811865475*(Si332[2][1]*T1332[1][6] + Si332[2][2]*T1332[2][6]);
T332[2][5]=S323[1][2]*(Si332[2][1]*T1332[1][4] + Si332[2][2]*T1332[2][4]) + S323[2][2]*(Si332[2][1]*T1332[1][5] + Si332[2][2]*T1332[2][5]);
T332[2][6]=S323[1][3]*(Si332[2][1]*T1332[1][4] + Si332[2][2]*T1332[2][4]) + S323[2][3]*(Si332[2][1]*T1332[1][5] + Si332[2][2]*T1332[2][5]) - 0.7071067811865475*(Si332[2][1]*T1332[1][6] + Si332[2][2]*T1332[2][6]);

T332[3][1]=S323[1][1]*(Si332[3][1]*T1332[1][1] + Si332[3][2]*T1332[2][1] - 0.7071067811865475*T1332[3][1]) + S323[2][1]*(Si332[3][1]*T1332[1][2] + Si332[3][2]*T1332[2][2] - 0.7071067811865475*T1332[3][2]) - 0.7071067811865475*(Si332[3][1]*T1332[1][3] + Si332[3][2]*T1332[2][3] - 0.7071067811865475*T1332[3][3]);
T332[3][2]=S323[1][2]*(Si332[3][1]*T1332[1][1] + Si332[3][2]*T1332[2][1] - 0.7071067811865475*T1332[3][1]) + S323[2][2]*(Si332[3][1]*T1332[1][2] + Si332[3][2]*T1332[2][2] - 0.7071067811865475*T1332[3][2]) + THORAX2SHOULDER*S323[1][3]*(Si332[3][1]*T1332[1][4] + Si332[3][2]*T1332[2][4] - 0.7071067811865475*T1332[3][4]) + THORAX2SHOULDER*S323[2][3]*(Si332[3][1]*T1332[1][5] + Si332[3][2]*T1332[2][5] - 0.7071067811865475*T1332[3][5]) - 0.7071067811865475*THORAX2SHOULDER*(Si332[3][1]*T1332[1][6] + Si332[3][2]*T1332[2][6] - 0.7071067811865475*T1332[3][6]);
T332[3][3]=S323[1][3]*(Si332[3][1]*T1332[1][1] + Si332[3][2]*T1332[2][1] - 0.7071067811865475*T1332[3][1]) + S323[2][3]*(Si332[3][1]*T1332[1][2] + Si332[3][2]*T1332[2][2] - 0.7071067811865475*T1332[3][2]) - 0.7071067811865475*(Si332[3][1]*T1332[1][3] + Si332[3][2]*T1332[2][3] - 0.7071067811865475*T1332[3][3]) - THORAX2SHOULDER*S323[1][2]*(Si332[3][1]*T1332[1][4] + Si332[3][2]*T1332[2][4] - 0.7071067811865475*T1332[3][4]) - THORAX2SHOULDER*S323[2][2]*(Si332[3][1]*T1332[1][5] + Si332[3][2]*T1332[2][5] - 0.7071067811865475*T1332[3][5]);
T332[3][4]=S323[1][1]*(Si332[3][1]*T1332[1][4] + Si332[3][2]*T1332[2][4] - 0.7071067811865475*T1332[3][4]) + S323[2][1]*(Si332[3][1]*T1332[1][5] + Si332[3][2]*T1332[2][5] - 0.7071067811865475*T1332[3][5]) - 0.7071067811865475*(Si332[3][1]*T1332[1][6] + Si332[3][2]*T1332[2][6] - 0.7071067811865475*T1332[3][6]);
T332[3][5]=S323[1][2]*(Si332[3][1]*T1332[1][4] + Si332[3][2]*T1332[2][4] - 0.7071067811865475*T1332[3][4]) + S323[2][2]*(Si332[3][1]*T1332[1][5] + Si332[3][2]*T1332[2][5] - 0.7071067811865475*T1332[3][5]);
T332[3][6]=S323[1][3]*(Si332[3][1]*T1332[1][4] + Si332[3][2]*T1332[2][4] - 0.7071067811865475*T1332[3][4]) + S323[2][3]*(Si332[3][1]*T1332[1][5] + Si332[3][2]*T1332[2][5] - 0.7071067811865475*T1332[3][5]) - 0.7071067811865475*(Si332[3][1]*T1332[1][6] + Si332[3][2]*T1332[2][6] - 0.7071067811865475*T1332[3][6]);

T332[4][1]=S323[1][1]*(Si332[1][1]*T1332[4][1] + Si332[1][2]*T1332[5][1] - 0.7071067811865475*T1332[6][1]) + S323[2][1]*(Si332[1][1]*T1332[4][2] + Si332[1][2]*T1332[5][2] - 0.7071067811865475*T1332[6][2]) - 0.7071067811865475*(Si332[1][1]*T1332[4][3] + Si332[1][2]*T1332[5][3] - 0.7071067811865475*T1332[6][3]);
T332[4][2]=S323[1][2]*(Si332[1][1]*T1332[4][1] + Si332[1][2]*T1332[5][1] - 0.7071067811865475*T1332[6][1]) + S323[2][2]*(Si332[1][1]*T1332[4][2] + Si332[1][2]*T1332[5][2] - 0.7071067811865475*T1332[6][2]) + THORAX2SHOULDER*S323[1][3]*(Si332[1][1]*T1332[4][4] + Si332[1][2]*T1332[5][4] - 0.7071067811865475*T1332[6][4]) + THORAX2SHOULDER*S323[2][3]*(Si332[1][1]*T1332[4][5] + Si332[1][2]*T1332[5][5] - 0.7071067811865475*T1332[6][5]) - 0.7071067811865475*THORAX2SHOULDER*(Si332[1][1]*T1332[4][6] + Si332[1][2]*T1332[5][6] - 0.7071067811865475*T1332[6][6]);
T332[4][3]=S323[1][3]*(Si332[1][1]*T1332[4][1] + Si332[1][2]*T1332[5][1] - 0.7071067811865475*T1332[6][1]) + S323[2][3]*(Si332[1][1]*T1332[4][2] + Si332[1][2]*T1332[5][2] - 0.7071067811865475*T1332[6][2]) - 0.7071067811865475*(Si332[1][1]*T1332[4][3] + Si332[1][2]*T1332[5][3] - 0.7071067811865475*T1332[6][3]) - THORAX2SHOULDER*S323[1][2]*(Si332[1][1]*T1332[4][4] + Si332[1][2]*T1332[5][4] - 0.7071067811865475*T1332[6][4]) - THORAX2SHOULDER*S323[2][2]*(Si332[1][1]*T1332[4][5] + Si332[1][2]*T1332[5][5] - 0.7071067811865475*T1332[6][5]);
T332[4][4]=S323[1][1]*(Si332[1][1]*T1332[4][4] + Si332[1][2]*T1332[5][4] - 0.7071067811865475*T1332[6][4]) + S323[2][1]*(Si332[1][1]*T1332[4][5] + Si332[1][2]*T1332[5][5] - 0.7071067811865475*T1332[6][5]) - 0.7071067811865475*(Si332[1][1]*T1332[4][6] + Si332[1][2]*T1332[5][6] - 0.7071067811865475*T1332[6][6]);
T332[4][5]=S323[1][2]*(Si332[1][1]*T1332[4][4] + Si332[1][2]*T1332[5][4] - 0.7071067811865475*T1332[6][4]) + S323[2][2]*(Si332[1][1]*T1332[4][5] + Si332[1][2]*T1332[5][5] - 0.7071067811865475*T1332[6][5]);
T332[4][6]=S323[1][3]*(Si332[1][1]*T1332[4][4] + Si332[1][2]*T1332[5][4] - 0.7071067811865475*T1332[6][4]) + S323[2][3]*(Si332[1][1]*T1332[4][5] + Si332[1][2]*T1332[5][5] - 0.7071067811865475*T1332[6][5]) - 0.7071067811865475*(Si332[1][1]*T1332[4][6] + Si332[1][2]*T1332[5][6] - 0.7071067811865475*T1332[6][6]);

T332[5][1]=S323[1][1]*(THORAX2SHOULDER*Si332[3][1]*T1332[1][1] + THORAX2SHOULDER*Si332[3][2]*T1332[2][1] - 0.7071067811865475*THORAX2SHOULDER*T1332[3][1] + Si332[2][1]*T1332[4][1] + Si332[2][2]*T1332[5][1]) + S323[2][1]*(THORAX2SHOULDER*Si332[3][1]*T1332[1][2] + THORAX2SHOULDER*Si332[3][2]*T1332[2][2] - 0.7071067811865475*THORAX2SHOULDER*T1332[3][2] + Si332[2][1]*T1332[4][2] + Si332[2][2]*T1332[5][2]) - 0.7071067811865475*(THORAX2SHOULDER*Si332[3][1]*T1332[1][3] + THORAX2SHOULDER*Si332[3][2]*T1332[2][3] - 0.7071067811865475*THORAX2SHOULDER*T1332[3][3] + Si332[2][1]*T1332[4][3] + Si332[2][2]*T1332[5][3]);
T332[5][2]=S323[1][2]*(THORAX2SHOULDER*Si332[3][1]*T1332[1][1] + THORAX2SHOULDER*Si332[3][2]*T1332[2][1] - 0.7071067811865475*THORAX2SHOULDER*T1332[3][1] + Si332[2][1]*T1332[4][1] + Si332[2][2]*T1332[5][1]) + S323[2][2]*(THORAX2SHOULDER*Si332[3][1]*T1332[1][2] + THORAX2SHOULDER*Si332[3][2]*T1332[2][2] - 0.7071067811865475*THORAX2SHOULDER*T1332[3][2] + Si332[2][1]*T1332[4][2] + Si332[2][2]*T1332[5][2]) + THORAX2SHOULDER*S323[1][3]*(THORAX2SHOULDER*Si332[3][1]*T1332[1][4] + THORAX2SHOULDER*Si332[3][2]*T1332[2][4] - 0.7071067811865475*THORAX2SHOULDER*T1332[3][4] + Si332[2][1]*T1332[4][4] + Si332[2][2]*T1332[5][4]) + THORAX2SHOULDER*S323[2][3]*(THORAX2SHOULDER*Si332[3][1]*T1332[1][5] + THORAX2SHOULDER*Si332[3][2]*T1332[2][5] - 0.7071067811865475*THORAX2SHOULDER*T1332[3][5] + Si332[2][1]*T1332[4][5] + Si332[2][2]*T1332[5][5]) - 0.7071067811865475*THORAX2SHOULDER*(THORAX2SHOULDER*Si332[3][1]*T1332[1][6] + THORAX2SHOULDER*Si332[3][2]*T1332[2][6] - 0.7071067811865475*THORAX2SHOULDER*T1332[3][6] + Si332[2][1]*T1332[4][6] + Si332[2][2]*T1332[5][6]);
T332[5][3]=S323[1][3]*(THORAX2SHOULDER*Si332[3][1]*T1332[1][1] + THORAX2SHOULDER*Si332[3][2]*T1332[2][1] - 0.7071067811865475*THORAX2SHOULDER*T1332[3][1] + Si332[2][1]*T1332[4][1] + Si332[2][2]*T1332[5][1]) + S323[2][3]*(THORAX2SHOULDER*Si332[3][1]*T1332[1][2] + THORAX2SHOULDER*Si332[3][2]*T1332[2][2] - 0.7071067811865475*THORAX2SHOULDER*T1332[3][2] + Si332[2][1]*T1332[4][2] + Si332[2][2]*T1332[5][2]) - 0.7071067811865475*(THORAX2SHOULDER*Si332[3][1]*T1332[1][3] + THORAX2SHOULDER*Si332[3][2]*T1332[2][3] - 0.7071067811865475*THORAX2SHOULDER*T1332[3][3] + Si332[2][1]*T1332[4][3] + Si332[2][2]*T1332[5][3]) - THORAX2SHOULDER*S323[1][2]*(THORAX2SHOULDER*Si332[3][1]*T1332[1][4] + THORAX2SHOULDER*Si332[3][2]*T1332[2][4] - 0.7071067811865475*THORAX2SHOULDER*T1332[3][4] + Si332[2][1]*T1332[4][4] + Si332[2][2]*T1332[5][4]) - THORAX2SHOULDER*S323[2][2]*(THORAX2SHOULDER*Si332[3][1]*T1332[1][5] + THORAX2SHOULDER*Si332[3][2]*T1332[2][5] - 0.7071067811865475*THORAX2SHOULDER*T1332[3][5] + Si332[2][1]*T1332[4][5] + Si332[2][2]*T1332[5][5]);
T332[5][4]=S323[1][1]*(THORAX2SHOULDER*Si332[3][1]*T1332[1][4] + THORAX2SHOULDER*Si332[3][2]*T1332[2][4] - 0.7071067811865475*THORAX2SHOULDER*T1332[3][4] + Si332[2][1]*T1332[4][4] + Si332[2][2]*T1332[5][4]) + S323[2][1]*(THORAX2SHOULDER*Si332[3][1]*T1332[1][5] + THORAX2SHOULDER*Si332[3][2]*T1332[2][5] - 0.7071067811865475*THORAX2SHOULDER*T1332[3][5] + Si332[2][1]*T1332[4][5] + Si332[2][2]*T1332[5][5]) - 0.7071067811865475*(THORAX2SHOULDER*Si332[3][1]*T1332[1][6] + THORAX2SHOULDER*Si332[3][2]*T1332[2][6] - 0.7071067811865475*THORAX2SHOULDER*T1332[3][6] + Si332[2][1]*T1332[4][6] + Si332[2][2]*T1332[5][6]);
T332[5][5]=S323[1][2]*(THORAX2SHOULDER*Si332[3][1]*T1332[1][4] + THORAX2SHOULDER*Si332[3][2]*T1332[2][4] - 0.7071067811865475*THORAX2SHOULDER*T1332[3][4] + Si332[2][1]*T1332[4][4] + Si332[2][2]*T1332[5][4]) + S323[2][2]*(THORAX2SHOULDER*Si332[3][1]*T1332[1][5] + THORAX2SHOULDER*Si332[3][2]*T1332[2][5] - 0.7071067811865475*THORAX2SHOULDER*T1332[3][5] + Si332[2][1]*T1332[4][5] + Si332[2][2]*T1332[5][5]);
T332[5][6]=S323[1][3]*(THORAX2SHOULDER*Si332[3][1]*T1332[1][4] + THORAX2SHOULDER*Si332[3][2]*T1332[2][4] - 0.7071067811865475*THORAX2SHOULDER*T1332[3][4] + Si332[2][1]*T1332[4][4] + Si332[2][2]*T1332[5][4]) + S323[2][3]*(THORAX2SHOULDER*Si332[3][1]*T1332[1][5] + THORAX2SHOULDER*Si332[3][2]*T1332[2][5] - 0.7071067811865475*THORAX2SHOULDER*T1332[3][5] + Si332[2][1]*T1332[4][5] + Si332[2][2]*T1332[5][5]) - 0.7071067811865475*(THORAX2SHOULDER*Si332[3][1]*T1332[1][6] + THORAX2SHOULDER*Si332[3][2]*T1332[2][6] - 0.7071067811865475*THORAX2SHOULDER*T1332[3][6] + Si332[2][1]*T1332[4][6] + Si332[2][2]*T1332[5][6]);

T332[6][1]=S323[1][1]*(-(THORAX2SHOULDER*Si332[2][1]*T1332[1][1]) - THORAX2SHOULDER*Si332[2][2]*T1332[2][1] + Si332[3][1]*T1332[4][1] + Si332[3][2]*T1332[5][1] - 0.7071067811865475*T1332[6][1]) + S323[2][1]*(-(THORAX2SHOULDER*Si332[2][1]*T1332[1][2]) - THORAX2SHOULDER*Si332[2][2]*T1332[2][2] + Si332[3][1]*T1332[4][2] + Si332[3][2]*T1332[5][2] - 0.7071067811865475*T1332[6][2]) - 0.7071067811865475*(-(THORAX2SHOULDER*Si332[2][1]*T1332[1][3]) - THORAX2SHOULDER*Si332[2][2]*T1332[2][3] + Si332[3][1]*T1332[4][3] + Si332[3][2]*T1332[5][3] - 0.7071067811865475*T1332[6][3]);
T332[6][2]=S323[1][2]*(-(THORAX2SHOULDER*Si332[2][1]*T1332[1][1]) - THORAX2SHOULDER*Si332[2][2]*T1332[2][1] + Si332[3][1]*T1332[4][1] + Si332[3][2]*T1332[5][1] - 0.7071067811865475*T1332[6][1]) + S323[2][2]*(-(THORAX2SHOULDER*Si332[2][1]*T1332[1][2]) - THORAX2SHOULDER*Si332[2][2]*T1332[2][2] + Si332[3][1]*T1332[4][2] + Si332[3][2]*T1332[5][2] - 0.7071067811865475*T1332[6][2]) + THORAX2SHOULDER*S323[1][3]*(-(THORAX2SHOULDER*Si332[2][1]*T1332[1][4]) - THORAX2SHOULDER*Si332[2][2]*T1332[2][4] + Si332[3][1]*T1332[4][4] + Si332[3][2]*T1332[5][4] - 0.7071067811865475*T1332[6][4]) + THORAX2SHOULDER*S323[2][3]*(-(THORAX2SHOULDER*Si332[2][1]*T1332[1][5]) - THORAX2SHOULDER*Si332[2][2]*T1332[2][5] + Si332[3][1]*T1332[4][5] + Si332[3][2]*T1332[5][5] - 0.7071067811865475*T1332[6][5]) - 0.7071067811865475*THORAX2SHOULDER*(-(THORAX2SHOULDER*Si332[2][1]*T1332[1][6]) - THORAX2SHOULDER*Si332[2][2]*T1332[2][6] + Si332[3][1]*T1332[4][6] + Si332[3][2]*T1332[5][6] - 0.7071067811865475*T1332[6][6]);
T332[6][3]=S323[1][3]*(-(THORAX2SHOULDER*Si332[2][1]*T1332[1][1]) - THORAX2SHOULDER*Si332[2][2]*T1332[2][1] + Si332[3][1]*T1332[4][1] + Si332[3][2]*T1332[5][1] - 0.7071067811865475*T1332[6][1]) + S323[2][3]*(-(THORAX2SHOULDER*Si332[2][1]*T1332[1][2]) - THORAX2SHOULDER*Si332[2][2]*T1332[2][2] + Si332[3][1]*T1332[4][2] + Si332[3][2]*T1332[5][2] - 0.7071067811865475*T1332[6][2]) - 0.7071067811865475*(-(THORAX2SHOULDER*Si332[2][1]*T1332[1][3]) - THORAX2SHOULDER*Si332[2][2]*T1332[2][3] + Si332[3][1]*T1332[4][3] + Si332[3][2]*T1332[5][3] - 0.7071067811865475*T1332[6][3]) - THORAX2SHOULDER*S323[1][2]*(-(THORAX2SHOULDER*Si332[2][1]*T1332[1][4]) - THORAX2SHOULDER*Si332[2][2]*T1332[2][4] + Si332[3][1]*T1332[4][4] + Si332[3][2]*T1332[5][4] - 0.7071067811865475*T1332[6][4]) - THORAX2SHOULDER*S323[2][2]*(-(THORAX2SHOULDER*Si332[2][1]*T1332[1][5]) - THORAX2SHOULDER*Si332[2][2]*T1332[2][5] + Si332[3][1]*T1332[4][5] + Si332[3][2]*T1332[5][5] - 0.7071067811865475*T1332[6][5]);
T332[6][4]=S323[1][1]*(-(THORAX2SHOULDER*Si332[2][1]*T1332[1][4]) - THORAX2SHOULDER*Si332[2][2]*T1332[2][4] + Si332[3][1]*T1332[4][4] + Si332[3][2]*T1332[5][4] - 0.7071067811865475*T1332[6][4]) + S323[2][1]*(-(THORAX2SHOULDER*Si332[2][1]*T1332[1][5]) - THORAX2SHOULDER*Si332[2][2]*T1332[2][5] + Si332[3][1]*T1332[4][5] + Si332[3][2]*T1332[5][5] - 0.7071067811865475*T1332[6][5]) - 0.7071067811865475*(-(THORAX2SHOULDER*Si332[2][1]*T1332[1][6]) - THORAX2SHOULDER*Si332[2][2]*T1332[2][6] + Si332[3][1]*T1332[4][6] + Si332[3][2]*T1332[5][6] - 0.7071067811865475*T1332[6][6]);
T332[6][5]=S323[1][2]*(-(THORAX2SHOULDER*Si332[2][1]*T1332[1][4]) - THORAX2SHOULDER*Si332[2][2]*T1332[2][4] + Si332[3][1]*T1332[4][4] + Si332[3][2]*T1332[5][4] - 0.7071067811865475*T1332[6][4]) + S323[2][2]*(-(THORAX2SHOULDER*Si332[2][1]*T1332[1][5]) - THORAX2SHOULDER*Si332[2][2]*T1332[2][5] + Si332[3][1]*T1332[4][5] + Si332[3][2]*T1332[5][5] - 0.7071067811865475*T1332[6][5]);
T332[6][6]=S323[1][3]*(-(THORAX2SHOULDER*Si332[2][1]*T1332[1][4]) - THORAX2SHOULDER*Si332[2][2]*T1332[2][4] + Si332[3][1]*T1332[4][4] + Si332[3][2]*T1332[5][4] - 0.7071067811865475*T1332[6][4]) + S323[2][3]*(-(THORAX2SHOULDER*Si332[2][1]*T1332[1][5]) - THORAX2SHOULDER*Si332[2][2]*T1332[2][5] + Si332[3][1]*T1332[4][5] + Si332[3][2]*T1332[5][5] - 0.7071067811865475*T1332[6][5]) - 0.7071067811865475*(-(THORAX2SHOULDER*Si332[2][1]*T1332[1][6]) - THORAX2SHOULDER*Si332[2][2]*T1332[2][6] + Si332[3][1]*T1332[4][6] + Si332[3][2]*T1332[5][6] - 0.7071067811865475*T1332[6][6]);



}


void
hermes_InvDynArtfunc73(void)
      {




}


void
hermes_InvDynArtfunc74(void)
      {




}


void
hermes_InvDynArtfunc75(void)
      {




}


void
hermes_InvDynArtfunc76(void)
      {
JA28[1][2]=0. + links[44].mcm[3];
JA28[1][3]=0. - links[44].mcm[2];
JA28[1][4]=0. + links[44].m;

JA28[2][1]=0. - links[44].mcm[3];
JA28[2][3]=0. + links[44].mcm[1];
JA28[2][5]=0. + links[44].m;

JA28[3][1]=0. + links[44].mcm[2];
JA28[3][2]=0. - links[44].mcm[1];
JA28[3][6]=0. + links[44].m;

JA28[4][1]=0. + links[44].inertia[1][1];
JA28[4][2]=0. + links[44].inertia[1][2];
JA28[4][3]=0. + links[44].inertia[1][3];
JA28[4][5]=0. - links[44].mcm[3];
JA28[4][6]=0. + links[44].mcm[2];

JA28[5][1]=0. + links[44].inertia[1][2];
JA28[5][2]=0. + links[44].inertia[2][2];
JA28[5][3]=0. + links[44].inertia[2][3];
JA28[5][4]=0. + links[44].mcm[3];
JA28[5][6]=0. - links[44].mcm[1];

JA28[6][1]=0. + links[44].inertia[1][3];
JA28[6][2]=0. + links[44].inertia[2][3];
JA28[6][3]=0. + links[44].inertia[3][3];
JA28[6][4]=0. - links[44].mcm[2];
JA28[6][5]=0. + links[44].mcm[1];


h28[1]=JA28[1][3];
h28[2]=JA28[2][3];
h28[4]=JA28[4][3];
h28[5]=JA28[5][3];
h28[6]=JA28[6][3];

T11028[1][2]=JA28[1][2];
T11028[1][3]=JA28[1][3];
T11028[1][4]=JA28[1][4];

T11028[2][1]=JA28[2][1];
T11028[2][3]=JA28[2][3];
T11028[2][5]=JA28[2][5];

T11028[3][1]=JA28[3][1];
T11028[3][2]=JA28[3][2];
T11028[3][6]=JA28[3][6];

T11028[4][1]=JA28[4][1];
T11028[4][2]=JA28[4][2];
T11028[4][3]=JA28[4][3];
T11028[4][5]=JA28[4][5];
T11028[4][6]=JA28[4][6];

T11028[5][1]=JA28[5][1];
T11028[5][2]=JA28[5][2];
T11028[5][3]=JA28[5][3];
T11028[5][4]=JA28[5][4];
T11028[5][6]=JA28[5][6];

T11028[6][1]=JA28[6][1];
T11028[6][2]=JA28[6][2];
T11028[6][3]=JA28[6][3];
T11028[6][4]=JA28[6][4];
T11028[6][5]=JA28[6][5];


T1028[1][1]=S2810[2][1]*Si1028[1][1]*T11028[1][2] + (ZLF*S2810[1][2] + YLF*S2810[1][3])*Si1028[1][1]*T11028[1][4] + S2810[1][1]*Si1028[1][2]*T11028[2][1] + (ZLF*S2810[2][2] + YLF*S2810[2][3])*Si1028[1][2]*T11028[2][5];
T1028[1][2]=S2810[2][2]*Si1028[1][1]*T11028[1][2] + (-(ZLF*S2810[1][1]) - XLF*S2810[1][3])*Si1028[1][1]*T11028[1][4] + S2810[1][2]*Si1028[1][2]*T11028[2][1] + S2810[3][2]*(Si1028[1][1]*T11028[1][3] + Si1028[1][2]*T11028[2][3]) + (-(ZLF*S2810[2][1]) - XLF*S2810[2][3])*Si1028[1][2]*T11028[2][5];
T1028[1][3]=S2810[2][3]*Si1028[1][1]*T11028[1][2] + (-(YLF*S2810[1][1]) + XLF*S2810[1][2])*Si1028[1][1]*T11028[1][4] + S2810[1][3]*Si1028[1][2]*T11028[2][1] + S2810[3][3]*(Si1028[1][1]*T11028[1][3] + Si1028[1][2]*T11028[2][3]) + (-(YLF*S2810[2][1]) + XLF*S2810[2][2])*Si1028[1][2]*T11028[2][5];
T1028[1][4]=S2810[1][1]*Si1028[1][1]*T11028[1][4] + S2810[2][1]*Si1028[1][2]*T11028[2][5];
T1028[1][5]=S2810[1][2]*Si1028[1][1]*T11028[1][4] + S2810[2][2]*Si1028[1][2]*T11028[2][5];
T1028[1][6]=S2810[1][3]*Si1028[1][1]*T11028[1][4] + S2810[2][3]*Si1028[1][2]*T11028[2][5];

T1028[2][1]=(ZLF*S2810[1][2] + YLF*S2810[1][3])*Si1028[2][1]*T11028[1][4] + (ZLF*S2810[2][2] + YLF*S2810[2][3])*Si1028[2][2]*T11028[2][5] + S2810[1][1]*(Si1028[2][2]*T11028[2][1] + Si1028[2][3]*T11028[3][1]) + S2810[2][1]*(Si1028[2][1]*T11028[1][2] + Si1028[2][3]*T11028[3][2]) + (ZLF*S2810[3][2] + YLF*S2810[3][3])*Si1028[2][3]*T11028[3][6];
T1028[2][2]=(-(ZLF*S2810[1][1]) - XLF*S2810[1][3])*Si1028[2][1]*T11028[1][4] + S2810[3][2]*(Si1028[2][1]*T11028[1][3] + Si1028[2][2]*T11028[2][3]) + (-(ZLF*S2810[2][1]) - XLF*S2810[2][3])*Si1028[2][2]*T11028[2][5] + S2810[1][2]*(Si1028[2][2]*T11028[2][1] + Si1028[2][3]*T11028[3][1]) + S2810[2][2]*(Si1028[2][1]*T11028[1][2] + Si1028[2][3]*T11028[3][2]) - XLF*S2810[3][3]*Si1028[2][3]*T11028[3][6];
T1028[2][3]=(-(YLF*S2810[1][1]) + XLF*S2810[1][2])*Si1028[2][1]*T11028[1][4] + S2810[3][3]*(Si1028[2][1]*T11028[1][3] + Si1028[2][2]*T11028[2][3]) + (-(YLF*S2810[2][1]) + XLF*S2810[2][2])*Si1028[2][2]*T11028[2][5] + S2810[1][3]*(Si1028[2][2]*T11028[2][1] + Si1028[2][3]*T11028[3][1]) + S2810[2][3]*(Si1028[2][1]*T11028[1][2] + Si1028[2][3]*T11028[3][2]) + XLF*S2810[3][2]*Si1028[2][3]*T11028[3][6];
T1028[2][4]=S2810[1][1]*Si1028[2][1]*T11028[1][4] + S2810[2][1]*Si1028[2][2]*T11028[2][5];
T1028[2][5]=S2810[1][2]*Si1028[2][1]*T11028[1][4] + S2810[2][2]*Si1028[2][2]*T11028[2][5] + S2810[3][2]*Si1028[2][3]*T11028[3][6];
T1028[2][6]=S2810[1][3]*Si1028[2][1]*T11028[1][4] + S2810[2][3]*Si1028[2][2]*T11028[2][5] + S2810[3][3]*Si1028[2][3]*T11028[3][6];

T1028[3][1]=(ZLF*S2810[1][2] + YLF*S2810[1][3])*Si1028[3][1]*T11028[1][4] + (ZLF*S2810[2][2] + YLF*S2810[2][3])*Si1028[3][2]*T11028[2][5] + S2810[1][1]*(Si1028[3][2]*T11028[2][1] + Si1028[3][3]*T11028[3][1]) + S2810[2][1]*(Si1028[3][1]*T11028[1][2] + Si1028[3][3]*T11028[3][2]) + (ZLF*S2810[3][2] + YLF*S2810[3][3])*Si1028[3][3]*T11028[3][6];
T1028[3][2]=(-(ZLF*S2810[1][1]) - XLF*S2810[1][3])*Si1028[3][1]*T11028[1][4] + S2810[3][2]*(Si1028[3][1]*T11028[1][3] + Si1028[3][2]*T11028[2][3]) + (-(ZLF*S2810[2][1]) - XLF*S2810[2][3])*Si1028[3][2]*T11028[2][5] + S2810[1][2]*(Si1028[3][2]*T11028[2][1] + Si1028[3][3]*T11028[3][1]) + S2810[2][2]*(Si1028[3][1]*T11028[1][2] + Si1028[3][3]*T11028[3][2]) - XLF*S2810[3][3]*Si1028[3][3]*T11028[3][6];
T1028[3][3]=(-(YLF*S2810[1][1]) + XLF*S2810[1][2])*Si1028[3][1]*T11028[1][4] + S2810[3][3]*(Si1028[3][1]*T11028[1][3] + Si1028[3][2]*T11028[2][3]) + (-(YLF*S2810[2][1]) + XLF*S2810[2][2])*Si1028[3][2]*T11028[2][5] + S2810[1][3]*(Si1028[3][2]*T11028[2][1] + Si1028[3][3]*T11028[3][1]) + S2810[2][3]*(Si1028[3][1]*T11028[1][2] + Si1028[3][3]*T11028[3][2]) + XLF*S2810[3][2]*Si1028[3][3]*T11028[3][6];
T1028[3][4]=S2810[1][1]*Si1028[3][1]*T11028[1][4] + S2810[2][1]*Si1028[3][2]*T11028[2][5];
T1028[3][5]=S2810[1][2]*Si1028[3][1]*T11028[1][4] + S2810[2][2]*Si1028[3][2]*T11028[2][5] + S2810[3][2]*Si1028[3][3]*T11028[3][6];
T1028[3][6]=S2810[1][3]*Si1028[3][1]*T11028[1][4] + S2810[2][3]*Si1028[3][2]*T11028[2][5] + S2810[3][3]*Si1028[3][3]*T11028[3][6];

T1028[4][1]=(ZLF*S2810[2][2] + YLF*S2810[2][3])*((ZLF*Si1028[2][2] + YLF*Si1028[3][2])*T11028[2][5] + Si1028[1][1]*T11028[4][5]) + S2810[1][1]*((ZLF*Si1028[2][2] + YLF*Si1028[3][2])*T11028[2][1] + (ZLF*Si1028[2][3] + YLF*Si1028[3][3])*T11028[3][1] + Si1028[1][1]*T11028[4][1] + Si1028[1][2]*T11028[5][1]) + S2810[2][1]*((ZLF*Si1028[2][1] + YLF*Si1028[3][1])*T11028[1][2] + (ZLF*Si1028[2][3] + YLF*Si1028[3][3])*T11028[3][2] + Si1028[1][1]*T11028[4][2] + Si1028[1][2]*T11028[5][2]) + (ZLF*S2810[1][2] + YLF*S2810[1][3])*((ZLF*Si1028[2][1] + YLF*Si1028[3][1])*T11028[1][4] + Si1028[1][2]*T11028[5][4]) + (ZLF*S2810[3][2] + YLF*S2810[3][3])*((ZLF*Si1028[2][3] + YLF*Si1028[3][3])*T11028[3][6] + Si1028[1][1]*T11028[4][6] + Si1028[1][2]*T11028[5][6]);
T1028[4][2]=(-(ZLF*S2810[2][1]) - XLF*S2810[2][3])*((ZLF*Si1028[2][2] + YLF*Si1028[3][2])*T11028[2][5] + Si1028[1][1]*T11028[4][5]) + S2810[1][2]*((ZLF*Si1028[2][2] + YLF*Si1028[3][2])*T11028[2][1] + (ZLF*Si1028[2][3] + YLF*Si1028[3][3])*T11028[3][1] + Si1028[1][1]*T11028[4][1] + Si1028[1][2]*T11028[5][1]) + S2810[2][2]*((ZLF*Si1028[2][1] + YLF*Si1028[3][1])*T11028[1][2] + (ZLF*Si1028[2][3] + YLF*Si1028[3][3])*T11028[3][2] + Si1028[1][1]*T11028[4][2] + Si1028[1][2]*T11028[5][2]) + S2810[3][2]*((ZLF*Si1028[2][1] + YLF*Si1028[3][1])*T11028[1][3] + (ZLF*Si1028[2][2] + YLF*Si1028[3][2])*T11028[2][3] + Si1028[1][1]*T11028[4][3] + Si1028[1][2]*T11028[5][3]) + (-(ZLF*S2810[1][1]) - XLF*S2810[1][3])*((ZLF*Si1028[2][1] + YLF*Si1028[3][1])*T11028[1][4] + Si1028[1][2]*T11028[5][4]) - XLF*S2810[3][3]*((ZLF*Si1028[2][3] + YLF*Si1028[3][3])*T11028[3][6] + Si1028[1][1]*T11028[4][6] + Si1028[1][2]*T11028[5][6]);
T1028[4][3]=(-(YLF*S2810[2][1]) + XLF*S2810[2][2])*((ZLF*Si1028[2][2] + YLF*Si1028[3][2])*T11028[2][5] + Si1028[1][1]*T11028[4][5]) + S2810[1][3]*((ZLF*Si1028[2][2] + YLF*Si1028[3][2])*T11028[2][1] + (ZLF*Si1028[2][3] + YLF*Si1028[3][3])*T11028[3][1] + Si1028[1][1]*T11028[4][1] + Si1028[1][2]*T11028[5][1]) + S2810[2][3]*((ZLF*Si1028[2][1] + YLF*Si1028[3][1])*T11028[1][2] + (ZLF*Si1028[2][3] + YLF*Si1028[3][3])*T11028[3][2] + Si1028[1][1]*T11028[4][2] + Si1028[1][2]*T11028[5][2]) + S2810[3][3]*((ZLF*Si1028[2][1] + YLF*Si1028[3][1])*T11028[1][3] + (ZLF*Si1028[2][2] + YLF*Si1028[3][2])*T11028[2][3] + Si1028[1][1]*T11028[4][3] + Si1028[1][2]*T11028[5][3]) + (-(YLF*S2810[1][1]) + XLF*S2810[1][2])*((ZLF*Si1028[2][1] + YLF*Si1028[3][1])*T11028[1][4] + Si1028[1][2]*T11028[5][4]) + XLF*S2810[3][2]*((ZLF*Si1028[2][3] + YLF*Si1028[3][3])*T11028[3][6] + Si1028[1][1]*T11028[4][6] + Si1028[1][2]*T11028[5][6]);
T1028[4][4]=S2810[2][1]*((ZLF*Si1028[2][2] + YLF*Si1028[3][2])*T11028[2][5] + Si1028[1][1]*T11028[4][5]) + S2810[1][1]*((ZLF*Si1028[2][1] + YLF*Si1028[3][1])*T11028[1][4] + Si1028[1][2]*T11028[5][4]);
T1028[4][5]=S2810[2][2]*((ZLF*Si1028[2][2] + YLF*Si1028[3][2])*T11028[2][5] + Si1028[1][1]*T11028[4][5]) + S2810[1][2]*((ZLF*Si1028[2][1] + YLF*Si1028[3][1])*T11028[1][4] + Si1028[1][2]*T11028[5][4]) + S2810[3][2]*((ZLF*Si1028[2][3] + YLF*Si1028[3][3])*T11028[3][6] + Si1028[1][1]*T11028[4][6] + Si1028[1][2]*T11028[5][6]);
T1028[4][6]=S2810[2][3]*((ZLF*Si1028[2][2] + YLF*Si1028[3][2])*T11028[2][5] + Si1028[1][1]*T11028[4][5]) + S2810[1][3]*((ZLF*Si1028[2][1] + YLF*Si1028[3][1])*T11028[1][4] + Si1028[1][2]*T11028[5][4]) + S2810[3][3]*((ZLF*Si1028[2][3] + YLF*Si1028[3][3])*T11028[3][6] + Si1028[1][1]*T11028[4][6] + Si1028[1][2]*T11028[5][6]);

T1028[5][1]=(ZLF*S2810[3][2] + YLF*S2810[3][3])*(-(XLF*Si1028[3][3]*T11028[3][6]) + Si1028[2][1]*T11028[4][6] + Si1028[2][2]*T11028[5][6]) + S2810[1][1]*((-(ZLF*Si1028[1][2]) - XLF*Si1028[3][2])*T11028[2][1] - XLF*Si1028[3][3]*T11028[3][1] + Si1028[2][1]*T11028[4][1] + Si1028[2][2]*T11028[5][1] + Si1028[2][3]*T11028[6][1]) + S2810[2][1]*((-(ZLF*Si1028[1][1]) - XLF*Si1028[3][1])*T11028[1][2] - XLF*Si1028[3][3]*T11028[3][2] + Si1028[2][1]*T11028[4][2] + Si1028[2][2]*T11028[5][2] + Si1028[2][3]*T11028[6][2]) + (ZLF*S2810[1][2] + YLF*S2810[1][3])*((-(ZLF*Si1028[1][1]) - XLF*Si1028[3][1])*T11028[1][4] + Si1028[2][2]*T11028[5][4] + Si1028[2][3]*T11028[6][4]) + (ZLF*S2810[2][2] + YLF*S2810[2][3])*((-(ZLF*Si1028[1][2]) - XLF*Si1028[3][2])*T11028[2][5] + Si1028[2][1]*T11028[4][5] + Si1028[2][3]*T11028[6][5]);
T1028[5][2]=-(XLF*S2810[3][3]*(-(XLF*Si1028[3][3]*T11028[3][6]) + Si1028[2][1]*T11028[4][6] + Si1028[2][2]*T11028[5][6])) + S2810[1][2]*((-(ZLF*Si1028[1][2]) - XLF*Si1028[3][2])*T11028[2][1] - XLF*Si1028[3][3]*T11028[3][1] + Si1028[2][1]*T11028[4][1] + Si1028[2][2]*T11028[5][1] + Si1028[2][3]*T11028[6][1]) + S2810[2][2]*((-(ZLF*Si1028[1][1]) - XLF*Si1028[3][1])*T11028[1][2] - XLF*Si1028[3][3]*T11028[3][2] + Si1028[2][1]*T11028[4][2] + Si1028[2][2]*T11028[5][2] + Si1028[2][3]*T11028[6][2]) + S2810[3][2]*((-(ZLF*Si1028[1][1]) - XLF*Si1028[3][1])*T11028[1][3] + (-(ZLF*Si1028[1][2]) - XLF*Si1028[3][2])*T11028[2][3] + Si1028[2][1]*T11028[4][3] + Si1028[2][2]*T11028[5][3] + Si1028[2][3]*T11028[6][3]) + (-(ZLF*S2810[1][1]) - XLF*S2810[1][3])*((-(ZLF*Si1028[1][1]) - XLF*Si1028[3][1])*T11028[1][4] + Si1028[2][2]*T11028[5][4] + Si1028[2][3]*T11028[6][4]) + (-(ZLF*S2810[2][1]) - XLF*S2810[2][3])*((-(ZLF*Si1028[1][2]) - XLF*Si1028[3][2])*T11028[2][5] + Si1028[2][1]*T11028[4][5] + Si1028[2][3]*T11028[6][5]);
T1028[5][3]=XLF*S2810[3][2]*(-(XLF*Si1028[3][3]*T11028[3][6]) + Si1028[2][1]*T11028[4][6] + Si1028[2][2]*T11028[5][6]) + S2810[1][3]*((-(ZLF*Si1028[1][2]) - XLF*Si1028[3][2])*T11028[2][1] - XLF*Si1028[3][3]*T11028[3][1] + Si1028[2][1]*T11028[4][1] + Si1028[2][2]*T11028[5][1] + Si1028[2][3]*T11028[6][1]) + S2810[2][3]*((-(ZLF*Si1028[1][1]) - XLF*Si1028[3][1])*T11028[1][2] - XLF*Si1028[3][3]*T11028[3][2] + Si1028[2][1]*T11028[4][2] + Si1028[2][2]*T11028[5][2] + Si1028[2][3]*T11028[6][2]) + S2810[3][3]*((-(ZLF*Si1028[1][1]) - XLF*Si1028[3][1])*T11028[1][3] + (-(ZLF*Si1028[1][2]) - XLF*Si1028[3][2])*T11028[2][3] + Si1028[2][1]*T11028[4][3] + Si1028[2][2]*T11028[5][3] + Si1028[2][3]*T11028[6][3]) + (-(YLF*S2810[1][1]) + XLF*S2810[1][2])*((-(ZLF*Si1028[1][1]) - XLF*Si1028[3][1])*T11028[1][4] + Si1028[2][2]*T11028[5][4] + Si1028[2][3]*T11028[6][4]) + (-(YLF*S2810[2][1]) + XLF*S2810[2][2])*((-(ZLF*Si1028[1][2]) - XLF*Si1028[3][2])*T11028[2][5] + Si1028[2][1]*T11028[4][5] + Si1028[2][3]*T11028[6][5]);
T1028[5][4]=S2810[1][1]*((-(ZLF*Si1028[1][1]) - XLF*Si1028[3][1])*T11028[1][4] + Si1028[2][2]*T11028[5][4] + Si1028[2][3]*T11028[6][4]) + S2810[2][1]*((-(ZLF*Si1028[1][2]) - XLF*Si1028[3][2])*T11028[2][5] + Si1028[2][1]*T11028[4][5] + Si1028[2][3]*T11028[6][5]);
T1028[5][5]=S2810[3][2]*(-(XLF*Si1028[3][3]*T11028[3][6]) + Si1028[2][1]*T11028[4][6] + Si1028[2][2]*T11028[5][6]) + S2810[1][2]*((-(ZLF*Si1028[1][1]) - XLF*Si1028[3][1])*T11028[1][4] + Si1028[2][2]*T11028[5][4] + Si1028[2][3]*T11028[6][4]) + S2810[2][2]*((-(ZLF*Si1028[1][2]) - XLF*Si1028[3][2])*T11028[2][5] + Si1028[2][1]*T11028[4][5] + Si1028[2][3]*T11028[6][5]);
T1028[5][6]=S2810[3][3]*(-(XLF*Si1028[3][3]*T11028[3][6]) + Si1028[2][1]*T11028[4][6] + Si1028[2][2]*T11028[5][6]) + S2810[1][3]*((-(ZLF*Si1028[1][1]) - XLF*Si1028[3][1])*T11028[1][4] + Si1028[2][2]*T11028[5][4] + Si1028[2][3]*T11028[6][4]) + S2810[2][3]*((-(ZLF*Si1028[1][2]) - XLF*Si1028[3][2])*T11028[2][5] + Si1028[2][1]*T11028[4][5] + Si1028[2][3]*T11028[6][5]);

T1028[6][1]=(ZLF*S2810[3][2] + YLF*S2810[3][3])*(XLF*Si1028[2][3]*T11028[3][6] + Si1028[3][1]*T11028[4][6] + Si1028[3][2]*T11028[5][6]) + S2810[1][1]*((-(YLF*Si1028[1][2]) + XLF*Si1028[2][2])*T11028[2][1] + XLF*Si1028[2][3]*T11028[3][1] + Si1028[3][1]*T11028[4][1] + Si1028[3][2]*T11028[5][1] + Si1028[3][3]*T11028[6][1]) + S2810[2][1]*((-(YLF*Si1028[1][1]) + XLF*Si1028[2][1])*T11028[1][2] + XLF*Si1028[2][3]*T11028[3][2] + Si1028[3][1]*T11028[4][2] + Si1028[3][2]*T11028[5][2] + Si1028[3][3]*T11028[6][2]) + (ZLF*S2810[1][2] + YLF*S2810[1][3])*((-(YLF*Si1028[1][1]) + XLF*Si1028[2][1])*T11028[1][4] + Si1028[3][2]*T11028[5][4] + Si1028[3][3]*T11028[6][4]) + (ZLF*S2810[2][2] + YLF*S2810[2][3])*((-(YLF*Si1028[1][2]) + XLF*Si1028[2][2])*T11028[2][5] + Si1028[3][1]*T11028[4][5] + Si1028[3][3]*T11028[6][5]);
T1028[6][2]=-(XLF*S2810[3][3]*(XLF*Si1028[2][3]*T11028[3][6] + Si1028[3][1]*T11028[4][6] + Si1028[3][2]*T11028[5][6])) + S2810[1][2]*((-(YLF*Si1028[1][2]) + XLF*Si1028[2][2])*T11028[2][1] + XLF*Si1028[2][3]*T11028[3][1] + Si1028[3][1]*T11028[4][1] + Si1028[3][2]*T11028[5][1] + Si1028[3][3]*T11028[6][1]) + S2810[2][2]*((-(YLF*Si1028[1][1]) + XLF*Si1028[2][1])*T11028[1][2] + XLF*Si1028[2][3]*T11028[3][2] + Si1028[3][1]*T11028[4][2] + Si1028[3][2]*T11028[5][2] + Si1028[3][3]*T11028[6][2]) + S2810[3][2]*((-(YLF*Si1028[1][1]) + XLF*Si1028[2][1])*T11028[1][3] + (-(YLF*Si1028[1][2]) + XLF*Si1028[2][2])*T11028[2][3] + Si1028[3][1]*T11028[4][3] + Si1028[3][2]*T11028[5][3] + Si1028[3][3]*T11028[6][3]) + (-(ZLF*S2810[1][1]) - XLF*S2810[1][3])*((-(YLF*Si1028[1][1]) + XLF*Si1028[2][1])*T11028[1][4] + Si1028[3][2]*T11028[5][4] + Si1028[3][3]*T11028[6][4]) + (-(ZLF*S2810[2][1]) - XLF*S2810[2][3])*((-(YLF*Si1028[1][2]) + XLF*Si1028[2][2])*T11028[2][5] + Si1028[3][1]*T11028[4][5] + Si1028[3][3]*T11028[6][5]);
T1028[6][3]=XLF*S2810[3][2]*(XLF*Si1028[2][3]*T11028[3][6] + Si1028[3][1]*T11028[4][6] + Si1028[3][2]*T11028[5][6]) + S2810[1][3]*((-(YLF*Si1028[1][2]) + XLF*Si1028[2][2])*T11028[2][1] + XLF*Si1028[2][3]*T11028[3][1] + Si1028[3][1]*T11028[4][1] + Si1028[3][2]*T11028[5][1] + Si1028[3][3]*T11028[6][1]) + S2810[2][3]*((-(YLF*Si1028[1][1]) + XLF*Si1028[2][1])*T11028[1][2] + XLF*Si1028[2][3]*T11028[3][2] + Si1028[3][1]*T11028[4][2] + Si1028[3][2]*T11028[5][2] + Si1028[3][3]*T11028[6][2]) + S2810[3][3]*((-(YLF*Si1028[1][1]) + XLF*Si1028[2][1])*T11028[1][3] + (-(YLF*Si1028[1][2]) + XLF*Si1028[2][2])*T11028[2][3] + Si1028[3][1]*T11028[4][3] + Si1028[3][2]*T11028[5][3] + Si1028[3][3]*T11028[6][3]) + (-(YLF*S2810[1][1]) + XLF*S2810[1][2])*((-(YLF*Si1028[1][1]) + XLF*Si1028[2][1])*T11028[1][4] + Si1028[3][2]*T11028[5][4] + Si1028[3][3]*T11028[6][4]) + (-(YLF*S2810[2][1]) + XLF*S2810[2][2])*((-(YLF*Si1028[1][2]) + XLF*Si1028[2][2])*T11028[2][5] + Si1028[3][1]*T11028[4][5] + Si1028[3][3]*T11028[6][5]);
T1028[6][4]=S2810[1][1]*((-(YLF*Si1028[1][1]) + XLF*Si1028[2][1])*T11028[1][4] + Si1028[3][2]*T11028[5][4] + Si1028[3][3]*T11028[6][4]) + S2810[2][1]*((-(YLF*Si1028[1][2]) + XLF*Si1028[2][2])*T11028[2][5] + Si1028[3][1]*T11028[4][5] + Si1028[3][3]*T11028[6][5]);
T1028[6][5]=S2810[3][2]*(XLF*Si1028[2][3]*T11028[3][6] + Si1028[3][1]*T11028[4][6] + Si1028[3][2]*T11028[5][6]) + S2810[1][2]*((-(YLF*Si1028[1][1]) + XLF*Si1028[2][1])*T11028[1][4] + Si1028[3][2]*T11028[5][4] + Si1028[3][3]*T11028[6][4]) + S2810[2][2]*((-(YLF*Si1028[1][2]) + XLF*Si1028[2][2])*T11028[2][5] + Si1028[3][1]*T11028[4][5] + Si1028[3][3]*T11028[6][5]);
T1028[6][6]=S2810[3][3]*(XLF*Si1028[2][3]*T11028[3][6] + Si1028[3][1]*T11028[4][6] + Si1028[3][2]*T11028[5][6]) + S2810[1][3]*((-(YLF*Si1028[1][1]) + XLF*Si1028[2][1])*T11028[1][4] + Si1028[3][2]*T11028[5][4] + Si1028[3][3]*T11028[6][4]) + S2810[2][3]*((-(YLF*Si1028[1][2]) + XLF*Si1028[2][2])*T11028[2][5] + Si1028[3][1]*T11028[4][5] + Si1028[3][3]*T11028[6][5]);



}


void
hermes_InvDynArtfunc77(void)
      {




}


void
hermes_InvDynArtfunc78(void)
      {




}


void
hermes_InvDynArtfunc79(void)
      {




}


void
hermes_InvDynArtfunc80(void)
      {
JA24[1][2]=0. + links[43].mcm[3];
JA24[1][3]=0. - links[43].mcm[2];
JA24[1][4]=0. + links[43].m;

JA24[2][1]=0. - links[43].mcm[3];
JA24[2][3]=0. + links[43].mcm[1];
JA24[2][5]=0. + links[43].m;

JA24[3][1]=0. + links[43].mcm[2];
JA24[3][2]=0. - links[43].mcm[1];
JA24[3][6]=0. + links[43].m;

JA24[4][1]=0. + links[43].inertia[1][1];
JA24[4][2]=0. + links[43].inertia[1][2];
JA24[4][3]=0. + links[43].inertia[1][3];
JA24[4][5]=0. - links[43].mcm[3];
JA24[4][6]=0. + links[43].mcm[2];

JA24[5][1]=0. + links[43].inertia[1][2];
JA24[5][2]=0. + links[43].inertia[2][2];
JA24[5][3]=0. + links[43].inertia[2][3];
JA24[5][4]=0. + links[43].mcm[3];
JA24[5][6]=0. - links[43].mcm[1];

JA24[6][1]=0. + links[43].inertia[1][3];
JA24[6][2]=0. + links[43].inertia[2][3];
JA24[6][3]=0. + links[43].inertia[3][3];
JA24[6][4]=0. - links[43].mcm[2];
JA24[6][5]=0. + links[43].mcm[1];


h24[1]=JA24[1][3];
h24[2]=JA24[2][3];
h24[4]=JA24[4][3];
h24[5]=JA24[5][3];
h24[6]=JA24[6][3];

T11024[1][2]=JA24[1][2];
T11024[1][3]=JA24[1][3];
T11024[1][4]=JA24[1][4];

T11024[2][1]=JA24[2][1];
T11024[2][3]=JA24[2][3];
T11024[2][5]=JA24[2][5];

T11024[3][1]=JA24[3][1];
T11024[3][2]=JA24[3][2];
T11024[3][6]=JA24[3][6];

T11024[4][1]=JA24[4][1];
T11024[4][2]=JA24[4][2];
T11024[4][3]=JA24[4][3];
T11024[4][5]=JA24[4][5];
T11024[4][6]=JA24[4][6];

T11024[5][1]=JA24[5][1];
T11024[5][2]=JA24[5][2];
T11024[5][3]=JA24[5][3];
T11024[5][4]=JA24[5][4];
T11024[5][6]=JA24[5][6];

T11024[6][1]=JA24[6][1];
T11024[6][2]=JA24[6][2];
T11024[6][3]=JA24[6][3];
T11024[6][4]=JA24[6][4];
T11024[6][5]=JA24[6][5];


T1024[1][1]=S2410[2][1]*Si1024[1][1]*T11024[1][2] + (ZRF*S2410[1][2] + YRF*S2410[1][3])*Si1024[1][1]*T11024[1][4] + S2410[1][1]*Si1024[1][2]*T11024[2][1] + (ZRF*S2410[2][2] + YRF*S2410[2][3])*Si1024[1][2]*T11024[2][5];
T1024[1][2]=S2410[2][2]*Si1024[1][1]*T11024[1][2] + (-(ZRF*S2410[1][1]) - XRF*S2410[1][3])*Si1024[1][1]*T11024[1][4] + S2410[1][2]*Si1024[1][2]*T11024[2][1] + S2410[3][2]*(Si1024[1][1]*T11024[1][3] + Si1024[1][2]*T11024[2][3]) + (-(ZRF*S2410[2][1]) - XRF*S2410[2][3])*Si1024[1][2]*T11024[2][5];
T1024[1][3]=S2410[2][3]*Si1024[1][1]*T11024[1][2] + (-(YRF*S2410[1][1]) + XRF*S2410[1][2])*Si1024[1][1]*T11024[1][4] + S2410[1][3]*Si1024[1][2]*T11024[2][1] + S2410[3][3]*(Si1024[1][1]*T11024[1][3] + Si1024[1][2]*T11024[2][3]) + (-(YRF*S2410[2][1]) + XRF*S2410[2][2])*Si1024[1][2]*T11024[2][5];
T1024[1][4]=S2410[1][1]*Si1024[1][1]*T11024[1][4] + S2410[2][1]*Si1024[1][2]*T11024[2][5];
T1024[1][5]=S2410[1][2]*Si1024[1][1]*T11024[1][4] + S2410[2][2]*Si1024[1][2]*T11024[2][5];
T1024[1][6]=S2410[1][3]*Si1024[1][1]*T11024[1][4] + S2410[2][3]*Si1024[1][2]*T11024[2][5];

T1024[2][1]=(ZRF*S2410[1][2] + YRF*S2410[1][3])*Si1024[2][1]*T11024[1][4] + (ZRF*S2410[2][2] + YRF*S2410[2][3])*Si1024[2][2]*T11024[2][5] + S2410[1][1]*(Si1024[2][2]*T11024[2][1] + Si1024[2][3]*T11024[3][1]) + S2410[2][1]*(Si1024[2][1]*T11024[1][2] + Si1024[2][3]*T11024[3][2]) + (ZRF*S2410[3][2] + YRF*S2410[3][3])*Si1024[2][3]*T11024[3][6];
T1024[2][2]=(-(ZRF*S2410[1][1]) - XRF*S2410[1][3])*Si1024[2][1]*T11024[1][4] + S2410[3][2]*(Si1024[2][1]*T11024[1][3] + Si1024[2][2]*T11024[2][3]) + (-(ZRF*S2410[2][1]) - XRF*S2410[2][3])*Si1024[2][2]*T11024[2][5] + S2410[1][2]*(Si1024[2][2]*T11024[2][1] + Si1024[2][3]*T11024[3][1]) + S2410[2][2]*(Si1024[2][1]*T11024[1][2] + Si1024[2][3]*T11024[3][2]) - XRF*S2410[3][3]*Si1024[2][3]*T11024[3][6];
T1024[2][3]=(-(YRF*S2410[1][1]) + XRF*S2410[1][2])*Si1024[2][1]*T11024[1][4] + S2410[3][3]*(Si1024[2][1]*T11024[1][3] + Si1024[2][2]*T11024[2][3]) + (-(YRF*S2410[2][1]) + XRF*S2410[2][2])*Si1024[2][2]*T11024[2][5] + S2410[1][3]*(Si1024[2][2]*T11024[2][1] + Si1024[2][3]*T11024[3][1]) + S2410[2][3]*(Si1024[2][1]*T11024[1][2] + Si1024[2][3]*T11024[3][2]) + XRF*S2410[3][2]*Si1024[2][3]*T11024[3][6];
T1024[2][4]=S2410[1][1]*Si1024[2][1]*T11024[1][4] + S2410[2][1]*Si1024[2][2]*T11024[2][5];
T1024[2][5]=S2410[1][2]*Si1024[2][1]*T11024[1][4] + S2410[2][2]*Si1024[2][2]*T11024[2][5] + S2410[3][2]*Si1024[2][3]*T11024[3][6];
T1024[2][6]=S2410[1][3]*Si1024[2][1]*T11024[1][4] + S2410[2][3]*Si1024[2][2]*T11024[2][5] + S2410[3][3]*Si1024[2][3]*T11024[3][6];

T1024[3][1]=(ZRF*S2410[1][2] + YRF*S2410[1][3])*Si1024[3][1]*T11024[1][4] + (ZRF*S2410[2][2] + YRF*S2410[2][3])*Si1024[3][2]*T11024[2][5] + S2410[1][1]*(Si1024[3][2]*T11024[2][1] + Si1024[3][3]*T11024[3][1]) + S2410[2][1]*(Si1024[3][1]*T11024[1][2] + Si1024[3][3]*T11024[3][2]) + (ZRF*S2410[3][2] + YRF*S2410[3][3])*Si1024[3][3]*T11024[3][6];
T1024[3][2]=(-(ZRF*S2410[1][1]) - XRF*S2410[1][3])*Si1024[3][1]*T11024[1][4] + S2410[3][2]*(Si1024[3][1]*T11024[1][3] + Si1024[3][2]*T11024[2][3]) + (-(ZRF*S2410[2][1]) - XRF*S2410[2][3])*Si1024[3][2]*T11024[2][5] + S2410[1][2]*(Si1024[3][2]*T11024[2][1] + Si1024[3][3]*T11024[3][1]) + S2410[2][2]*(Si1024[3][1]*T11024[1][2] + Si1024[3][3]*T11024[3][2]) - XRF*S2410[3][3]*Si1024[3][3]*T11024[3][6];
T1024[3][3]=(-(YRF*S2410[1][1]) + XRF*S2410[1][2])*Si1024[3][1]*T11024[1][4] + S2410[3][3]*(Si1024[3][1]*T11024[1][3] + Si1024[3][2]*T11024[2][3]) + (-(YRF*S2410[2][1]) + XRF*S2410[2][2])*Si1024[3][2]*T11024[2][5] + S2410[1][3]*(Si1024[3][2]*T11024[2][1] + Si1024[3][3]*T11024[3][1]) + S2410[2][3]*(Si1024[3][1]*T11024[1][2] + Si1024[3][3]*T11024[3][2]) + XRF*S2410[3][2]*Si1024[3][3]*T11024[3][6];
T1024[3][4]=S2410[1][1]*Si1024[3][1]*T11024[1][4] + S2410[2][1]*Si1024[3][2]*T11024[2][5];
T1024[3][5]=S2410[1][2]*Si1024[3][1]*T11024[1][4] + S2410[2][2]*Si1024[3][2]*T11024[2][5] + S2410[3][2]*Si1024[3][3]*T11024[3][6];
T1024[3][6]=S2410[1][3]*Si1024[3][1]*T11024[1][4] + S2410[2][3]*Si1024[3][2]*T11024[2][5] + S2410[3][3]*Si1024[3][3]*T11024[3][6];

T1024[4][1]=(ZRF*S2410[2][2] + YRF*S2410[2][3])*((ZRF*Si1024[2][2] + YRF*Si1024[3][2])*T11024[2][5] + Si1024[1][1]*T11024[4][5]) + S2410[1][1]*((ZRF*Si1024[2][2] + YRF*Si1024[3][2])*T11024[2][1] + (ZRF*Si1024[2][3] + YRF*Si1024[3][3])*T11024[3][1] + Si1024[1][1]*T11024[4][1] + Si1024[1][2]*T11024[5][1]) + S2410[2][1]*((ZRF*Si1024[2][1] + YRF*Si1024[3][1])*T11024[1][2] + (ZRF*Si1024[2][3] + YRF*Si1024[3][3])*T11024[3][2] + Si1024[1][1]*T11024[4][2] + Si1024[1][2]*T11024[5][2]) + (ZRF*S2410[1][2] + YRF*S2410[1][3])*((ZRF*Si1024[2][1] + YRF*Si1024[3][1])*T11024[1][4] + Si1024[1][2]*T11024[5][4]) + (ZRF*S2410[3][2] + YRF*S2410[3][3])*((ZRF*Si1024[2][3] + YRF*Si1024[3][3])*T11024[3][6] + Si1024[1][1]*T11024[4][6] + Si1024[1][2]*T11024[5][6]);
T1024[4][2]=(-(ZRF*S2410[2][1]) - XRF*S2410[2][3])*((ZRF*Si1024[2][2] + YRF*Si1024[3][2])*T11024[2][5] + Si1024[1][1]*T11024[4][5]) + S2410[1][2]*((ZRF*Si1024[2][2] + YRF*Si1024[3][2])*T11024[2][1] + (ZRF*Si1024[2][3] + YRF*Si1024[3][3])*T11024[3][1] + Si1024[1][1]*T11024[4][1] + Si1024[1][2]*T11024[5][1]) + S2410[2][2]*((ZRF*Si1024[2][1] + YRF*Si1024[3][1])*T11024[1][2] + (ZRF*Si1024[2][3] + YRF*Si1024[3][3])*T11024[3][2] + Si1024[1][1]*T11024[4][2] + Si1024[1][2]*T11024[5][2]) + S2410[3][2]*((ZRF*Si1024[2][1] + YRF*Si1024[3][1])*T11024[1][3] + (ZRF*Si1024[2][2] + YRF*Si1024[3][2])*T11024[2][3] + Si1024[1][1]*T11024[4][3] + Si1024[1][2]*T11024[5][3]) + (-(ZRF*S2410[1][1]) - XRF*S2410[1][3])*((ZRF*Si1024[2][1] + YRF*Si1024[3][1])*T11024[1][4] + Si1024[1][2]*T11024[5][4]) - XRF*S2410[3][3]*((ZRF*Si1024[2][3] + YRF*Si1024[3][3])*T11024[3][6] + Si1024[1][1]*T11024[4][6] + Si1024[1][2]*T11024[5][6]);
T1024[4][3]=(-(YRF*S2410[2][1]) + XRF*S2410[2][2])*((ZRF*Si1024[2][2] + YRF*Si1024[3][2])*T11024[2][5] + Si1024[1][1]*T11024[4][5]) + S2410[1][3]*((ZRF*Si1024[2][2] + YRF*Si1024[3][2])*T11024[2][1] + (ZRF*Si1024[2][3] + YRF*Si1024[3][3])*T11024[3][1] + Si1024[1][1]*T11024[4][1] + Si1024[1][2]*T11024[5][1]) + S2410[2][3]*((ZRF*Si1024[2][1] + YRF*Si1024[3][1])*T11024[1][2] + (ZRF*Si1024[2][3] + YRF*Si1024[3][3])*T11024[3][2] + Si1024[1][1]*T11024[4][2] + Si1024[1][2]*T11024[5][2]) + S2410[3][3]*((ZRF*Si1024[2][1] + YRF*Si1024[3][1])*T11024[1][3] + (ZRF*Si1024[2][2] + YRF*Si1024[3][2])*T11024[2][3] + Si1024[1][1]*T11024[4][3] + Si1024[1][2]*T11024[5][3]) + (-(YRF*S2410[1][1]) + XRF*S2410[1][2])*((ZRF*Si1024[2][1] + YRF*Si1024[3][1])*T11024[1][4] + Si1024[1][2]*T11024[5][4]) + XRF*S2410[3][2]*((ZRF*Si1024[2][3] + YRF*Si1024[3][3])*T11024[3][6] + Si1024[1][1]*T11024[4][6] + Si1024[1][2]*T11024[5][6]);
T1024[4][4]=S2410[2][1]*((ZRF*Si1024[2][2] + YRF*Si1024[3][2])*T11024[2][5] + Si1024[1][1]*T11024[4][5]) + S2410[1][1]*((ZRF*Si1024[2][1] + YRF*Si1024[3][1])*T11024[1][4] + Si1024[1][2]*T11024[5][4]);
T1024[4][5]=S2410[2][2]*((ZRF*Si1024[2][2] + YRF*Si1024[3][2])*T11024[2][5] + Si1024[1][1]*T11024[4][5]) + S2410[1][2]*((ZRF*Si1024[2][1] + YRF*Si1024[3][1])*T11024[1][4] + Si1024[1][2]*T11024[5][4]) + S2410[3][2]*((ZRF*Si1024[2][3] + YRF*Si1024[3][3])*T11024[3][6] + Si1024[1][1]*T11024[4][6] + Si1024[1][2]*T11024[5][6]);
T1024[4][6]=S2410[2][3]*((ZRF*Si1024[2][2] + YRF*Si1024[3][2])*T11024[2][5] + Si1024[1][1]*T11024[4][5]) + S2410[1][3]*((ZRF*Si1024[2][1] + YRF*Si1024[3][1])*T11024[1][4] + Si1024[1][2]*T11024[5][4]) + S2410[3][3]*((ZRF*Si1024[2][3] + YRF*Si1024[3][3])*T11024[3][6] + Si1024[1][1]*T11024[4][6] + Si1024[1][2]*T11024[5][6]);

T1024[5][1]=(ZRF*S2410[3][2] + YRF*S2410[3][3])*(-(XRF*Si1024[3][3]*T11024[3][6]) + Si1024[2][1]*T11024[4][6] + Si1024[2][2]*T11024[5][6]) + S2410[1][1]*((-(ZRF*Si1024[1][2]) - XRF*Si1024[3][2])*T11024[2][1] - XRF*Si1024[3][3]*T11024[3][1] + Si1024[2][1]*T11024[4][1] + Si1024[2][2]*T11024[5][1] + Si1024[2][3]*T11024[6][1]) + S2410[2][1]*((-(ZRF*Si1024[1][1]) - XRF*Si1024[3][1])*T11024[1][2] - XRF*Si1024[3][3]*T11024[3][2] + Si1024[2][1]*T11024[4][2] + Si1024[2][2]*T11024[5][2] + Si1024[2][3]*T11024[6][2]) + (ZRF*S2410[1][2] + YRF*S2410[1][3])*((-(ZRF*Si1024[1][1]) - XRF*Si1024[3][1])*T11024[1][4] + Si1024[2][2]*T11024[5][4] + Si1024[2][3]*T11024[6][4]) + (ZRF*S2410[2][2] + YRF*S2410[2][3])*((-(ZRF*Si1024[1][2]) - XRF*Si1024[3][2])*T11024[2][5] + Si1024[2][1]*T11024[4][5] + Si1024[2][3]*T11024[6][5]);
T1024[5][2]=-(XRF*S2410[3][3]*(-(XRF*Si1024[3][3]*T11024[3][6]) + Si1024[2][1]*T11024[4][6] + Si1024[2][2]*T11024[5][6])) + S2410[1][2]*((-(ZRF*Si1024[1][2]) - XRF*Si1024[3][2])*T11024[2][1] - XRF*Si1024[3][3]*T11024[3][1] + Si1024[2][1]*T11024[4][1] + Si1024[2][2]*T11024[5][1] + Si1024[2][3]*T11024[6][1]) + S2410[2][2]*((-(ZRF*Si1024[1][1]) - XRF*Si1024[3][1])*T11024[1][2] - XRF*Si1024[3][3]*T11024[3][2] + Si1024[2][1]*T11024[4][2] + Si1024[2][2]*T11024[5][2] + Si1024[2][3]*T11024[6][2]) + S2410[3][2]*((-(ZRF*Si1024[1][1]) - XRF*Si1024[3][1])*T11024[1][3] + (-(ZRF*Si1024[1][2]) - XRF*Si1024[3][2])*T11024[2][3] + Si1024[2][1]*T11024[4][3] + Si1024[2][2]*T11024[5][3] + Si1024[2][3]*T11024[6][3]) + (-(ZRF*S2410[1][1]) - XRF*S2410[1][3])*((-(ZRF*Si1024[1][1]) - XRF*Si1024[3][1])*T11024[1][4] + Si1024[2][2]*T11024[5][4] + Si1024[2][3]*T11024[6][4]) + (-(ZRF*S2410[2][1]) - XRF*S2410[2][3])*((-(ZRF*Si1024[1][2]) - XRF*Si1024[3][2])*T11024[2][5] + Si1024[2][1]*T11024[4][5] + Si1024[2][3]*T11024[6][5]);
T1024[5][3]=XRF*S2410[3][2]*(-(XRF*Si1024[3][3]*T11024[3][6]) + Si1024[2][1]*T11024[4][6] + Si1024[2][2]*T11024[5][6]) + S2410[1][3]*((-(ZRF*Si1024[1][2]) - XRF*Si1024[3][2])*T11024[2][1] - XRF*Si1024[3][3]*T11024[3][1] + Si1024[2][1]*T11024[4][1] + Si1024[2][2]*T11024[5][1] + Si1024[2][3]*T11024[6][1]) + S2410[2][3]*((-(ZRF*Si1024[1][1]) - XRF*Si1024[3][1])*T11024[1][2] - XRF*Si1024[3][3]*T11024[3][2] + Si1024[2][1]*T11024[4][2] + Si1024[2][2]*T11024[5][2] + Si1024[2][3]*T11024[6][2]) + S2410[3][3]*((-(ZRF*Si1024[1][1]) - XRF*Si1024[3][1])*T11024[1][3] + (-(ZRF*Si1024[1][2]) - XRF*Si1024[3][2])*T11024[2][3] + Si1024[2][1]*T11024[4][3] + Si1024[2][2]*T11024[5][3] + Si1024[2][3]*T11024[6][3]) + (-(YRF*S2410[1][1]) + XRF*S2410[1][2])*((-(ZRF*Si1024[1][1]) - XRF*Si1024[3][1])*T11024[1][4] + Si1024[2][2]*T11024[5][4] + Si1024[2][3]*T11024[6][4]) + (-(YRF*S2410[2][1]) + XRF*S2410[2][2])*((-(ZRF*Si1024[1][2]) - XRF*Si1024[3][2])*T11024[2][5] + Si1024[2][1]*T11024[4][5] + Si1024[2][3]*T11024[6][5]);
T1024[5][4]=S2410[1][1]*((-(ZRF*Si1024[1][1]) - XRF*Si1024[3][1])*T11024[1][4] + Si1024[2][2]*T11024[5][4] + Si1024[2][3]*T11024[6][4]) + S2410[2][1]*((-(ZRF*Si1024[1][2]) - XRF*Si1024[3][2])*T11024[2][5] + Si1024[2][1]*T11024[4][5] + Si1024[2][3]*T11024[6][5]);
T1024[5][5]=S2410[3][2]*(-(XRF*Si1024[3][3]*T11024[3][6]) + Si1024[2][1]*T11024[4][6] + Si1024[2][2]*T11024[5][6]) + S2410[1][2]*((-(ZRF*Si1024[1][1]) - XRF*Si1024[3][1])*T11024[1][4] + Si1024[2][2]*T11024[5][4] + Si1024[2][3]*T11024[6][4]) + S2410[2][2]*((-(ZRF*Si1024[1][2]) - XRF*Si1024[3][2])*T11024[2][5] + Si1024[2][1]*T11024[4][5] + Si1024[2][3]*T11024[6][5]);
T1024[5][6]=S2410[3][3]*(-(XRF*Si1024[3][3]*T11024[3][6]) + Si1024[2][1]*T11024[4][6] + Si1024[2][2]*T11024[5][6]) + S2410[1][3]*((-(ZRF*Si1024[1][1]) - XRF*Si1024[3][1])*T11024[1][4] + Si1024[2][2]*T11024[5][4] + Si1024[2][3]*T11024[6][4]) + S2410[2][3]*((-(ZRF*Si1024[1][2]) - XRF*Si1024[3][2])*T11024[2][5] + Si1024[2][1]*T11024[4][5] + Si1024[2][3]*T11024[6][5]);

T1024[6][1]=(ZRF*S2410[3][2] + YRF*S2410[3][3])*(XRF*Si1024[2][3]*T11024[3][6] + Si1024[3][1]*T11024[4][6] + Si1024[3][2]*T11024[5][6]) + S2410[1][1]*((-(YRF*Si1024[1][2]) + XRF*Si1024[2][2])*T11024[2][1] + XRF*Si1024[2][3]*T11024[3][1] + Si1024[3][1]*T11024[4][1] + Si1024[3][2]*T11024[5][1] + Si1024[3][3]*T11024[6][1]) + S2410[2][1]*((-(YRF*Si1024[1][1]) + XRF*Si1024[2][1])*T11024[1][2] + XRF*Si1024[2][3]*T11024[3][2] + Si1024[3][1]*T11024[4][2] + Si1024[3][2]*T11024[5][2] + Si1024[3][3]*T11024[6][2]) + (ZRF*S2410[1][2] + YRF*S2410[1][3])*((-(YRF*Si1024[1][1]) + XRF*Si1024[2][1])*T11024[1][4] + Si1024[3][2]*T11024[5][4] + Si1024[3][3]*T11024[6][4]) + (ZRF*S2410[2][2] + YRF*S2410[2][3])*((-(YRF*Si1024[1][2]) + XRF*Si1024[2][2])*T11024[2][5] + Si1024[3][1]*T11024[4][5] + Si1024[3][3]*T11024[6][5]);
T1024[6][2]=-(XRF*S2410[3][3]*(XRF*Si1024[2][3]*T11024[3][6] + Si1024[3][1]*T11024[4][6] + Si1024[3][2]*T11024[5][6])) + S2410[1][2]*((-(YRF*Si1024[1][2]) + XRF*Si1024[2][2])*T11024[2][1] + XRF*Si1024[2][3]*T11024[3][1] + Si1024[3][1]*T11024[4][1] + Si1024[3][2]*T11024[5][1] + Si1024[3][3]*T11024[6][1]) + S2410[2][2]*((-(YRF*Si1024[1][1]) + XRF*Si1024[2][1])*T11024[1][2] + XRF*Si1024[2][3]*T11024[3][2] + Si1024[3][1]*T11024[4][2] + Si1024[3][2]*T11024[5][2] + Si1024[3][3]*T11024[6][2]) + S2410[3][2]*((-(YRF*Si1024[1][1]) + XRF*Si1024[2][1])*T11024[1][3] + (-(YRF*Si1024[1][2]) + XRF*Si1024[2][2])*T11024[2][3] + Si1024[3][1]*T11024[4][3] + Si1024[3][2]*T11024[5][3] + Si1024[3][3]*T11024[6][3]) + (-(ZRF*S2410[1][1]) - XRF*S2410[1][3])*((-(YRF*Si1024[1][1]) + XRF*Si1024[2][1])*T11024[1][4] + Si1024[3][2]*T11024[5][4] + Si1024[3][3]*T11024[6][4]) + (-(ZRF*S2410[2][1]) - XRF*S2410[2][3])*((-(YRF*Si1024[1][2]) + XRF*Si1024[2][2])*T11024[2][5] + Si1024[3][1]*T11024[4][5] + Si1024[3][3]*T11024[6][5]);
T1024[6][3]=XRF*S2410[3][2]*(XRF*Si1024[2][3]*T11024[3][6] + Si1024[3][1]*T11024[4][6] + Si1024[3][2]*T11024[5][6]) + S2410[1][3]*((-(YRF*Si1024[1][2]) + XRF*Si1024[2][2])*T11024[2][1] + XRF*Si1024[2][3]*T11024[3][1] + Si1024[3][1]*T11024[4][1] + Si1024[3][2]*T11024[5][1] + Si1024[3][3]*T11024[6][1]) + S2410[2][3]*((-(YRF*Si1024[1][1]) + XRF*Si1024[2][1])*T11024[1][2] + XRF*Si1024[2][3]*T11024[3][2] + Si1024[3][1]*T11024[4][2] + Si1024[3][2]*T11024[5][2] + Si1024[3][3]*T11024[6][2]) + S2410[3][3]*((-(YRF*Si1024[1][1]) + XRF*Si1024[2][1])*T11024[1][3] + (-(YRF*Si1024[1][2]) + XRF*Si1024[2][2])*T11024[2][3] + Si1024[3][1]*T11024[4][3] + Si1024[3][2]*T11024[5][3] + Si1024[3][3]*T11024[6][3]) + (-(YRF*S2410[1][1]) + XRF*S2410[1][2])*((-(YRF*Si1024[1][1]) + XRF*Si1024[2][1])*T11024[1][4] + Si1024[3][2]*T11024[5][4] + Si1024[3][3]*T11024[6][4]) + (-(YRF*S2410[2][1]) + XRF*S2410[2][2])*((-(YRF*Si1024[1][2]) + XRF*Si1024[2][2])*T11024[2][5] + Si1024[3][1]*T11024[4][5] + Si1024[3][3]*T11024[6][5]);
T1024[6][4]=S2410[1][1]*((-(YRF*Si1024[1][1]) + XRF*Si1024[2][1])*T11024[1][4] + Si1024[3][2]*T11024[5][4] + Si1024[3][3]*T11024[6][4]) + S2410[2][1]*((-(YRF*Si1024[1][2]) + XRF*Si1024[2][2])*T11024[2][5] + Si1024[3][1]*T11024[4][5] + Si1024[3][3]*T11024[6][5]);
T1024[6][5]=S2410[3][2]*(XRF*Si1024[2][3]*T11024[3][6] + Si1024[3][1]*T11024[4][6] + Si1024[3][2]*T11024[5][6]) + S2410[1][2]*((-(YRF*Si1024[1][1]) + XRF*Si1024[2][1])*T11024[1][4] + Si1024[3][2]*T11024[5][4] + Si1024[3][3]*T11024[6][4]) + S2410[2][2]*((-(YRF*Si1024[1][2]) + XRF*Si1024[2][2])*T11024[2][5] + Si1024[3][1]*T11024[4][5] + Si1024[3][3]*T11024[6][5]);
T1024[6][6]=S2410[3][3]*(XRF*Si1024[2][3]*T11024[3][6] + Si1024[3][1]*T11024[4][6] + Si1024[3][2]*T11024[5][6]) + S2410[1][3]*((-(YRF*Si1024[1][1]) + XRF*Si1024[2][1])*T11024[1][4] + Si1024[3][2]*T11024[5][4] + Si1024[3][3]*T11024[6][4]) + S2410[2][3]*((-(YRF*Si1024[1][2]) + XRF*Si1024[2][2])*T11024[2][5] + Si1024[3][1]*T11024[4][5] + Si1024[3][3]*T11024[6][5]);



}


void
hermes_InvDynArtfunc81(void)
      {




}


void
hermes_InvDynArtfunc82(void)
      {




}


void
hermes_InvDynArtfunc83(void)
      {




}


void
hermes_InvDynArtfunc84(void)
      {
JA20[1][2]=0. + links[42].mcm[3];
JA20[1][3]=0. - links[42].mcm[2];
JA20[1][4]=0. + links[42].m;

JA20[2][1]=0. - links[42].mcm[3];
JA20[2][3]=0. + links[42].mcm[1];
JA20[2][5]=0. + links[42].m;

JA20[3][1]=0. + links[42].mcm[2];
JA20[3][2]=0. - links[42].mcm[1];
JA20[3][6]=0. + links[42].m;

JA20[4][1]=0. + links[42].inertia[1][1];
JA20[4][2]=0. + links[42].inertia[1][2];
JA20[4][3]=0. + links[42].inertia[1][3];
JA20[4][5]=0. - links[42].mcm[3];
JA20[4][6]=0. + links[42].mcm[2];

JA20[5][1]=0. + links[42].inertia[1][2];
JA20[5][2]=0. + links[42].inertia[2][2];
JA20[5][3]=0. + links[42].inertia[2][3];
JA20[5][4]=0. + links[42].mcm[3];
JA20[5][6]=0. - links[42].mcm[1];

JA20[6][1]=0. + links[42].inertia[1][3];
JA20[6][2]=0. + links[42].inertia[2][3];
JA20[6][3]=0. + links[42].inertia[3][3];
JA20[6][4]=0. - links[42].mcm[2];
JA20[6][5]=0. + links[42].mcm[1];


h20[1]=JA20[1][3];
h20[2]=JA20[2][3];
h20[4]=JA20[4][3];
h20[5]=JA20[5][3];
h20[6]=JA20[6][3];

T11020[1][2]=JA20[1][2];
T11020[1][3]=JA20[1][3];
T11020[1][4]=JA20[1][4];

T11020[2][1]=JA20[2][1];
T11020[2][3]=JA20[2][3];
T11020[2][5]=JA20[2][5];

T11020[3][1]=JA20[3][1];
T11020[3][2]=JA20[3][2];
T11020[3][6]=JA20[3][6];

T11020[4][1]=JA20[4][1];
T11020[4][2]=JA20[4][2];
T11020[4][3]=JA20[4][3];
T11020[4][5]=JA20[4][5];
T11020[4][6]=JA20[4][6];

T11020[5][1]=JA20[5][1];
T11020[5][2]=JA20[5][2];
T11020[5][3]=JA20[5][3];
T11020[5][4]=JA20[5][4];
T11020[5][6]=JA20[5][6];

T11020[6][1]=JA20[6][1];
T11020[6][2]=JA20[6][2];
T11020[6][3]=JA20[6][3];
T11020[6][4]=JA20[6][4];
T11020[6][5]=JA20[6][5];


T1020[1][1]=S2010[2][1]*Si1020[1][1]*T11020[1][2] + (ZMF*S2010[1][2] + YMF*S2010[1][3])*Si1020[1][1]*T11020[1][4] + S2010[1][1]*Si1020[1][2]*T11020[2][1] + (ZMF*S2010[2][2] + YMF*S2010[2][3])*Si1020[1][2]*T11020[2][5];
T1020[1][2]=S2010[2][2]*Si1020[1][1]*T11020[1][2] + (-(ZMF*S2010[1][1]) - XMF*S2010[1][3])*Si1020[1][1]*T11020[1][4] + S2010[1][2]*Si1020[1][2]*T11020[2][1] + S2010[3][2]*(Si1020[1][1]*T11020[1][3] + Si1020[1][2]*T11020[2][3]) + (-(ZMF*S2010[2][1]) - XMF*S2010[2][3])*Si1020[1][2]*T11020[2][5];
T1020[1][3]=S2010[2][3]*Si1020[1][1]*T11020[1][2] + (-(YMF*S2010[1][1]) + XMF*S2010[1][2])*Si1020[1][1]*T11020[1][4] + S2010[1][3]*Si1020[1][2]*T11020[2][1] + S2010[3][3]*(Si1020[1][1]*T11020[1][3] + Si1020[1][2]*T11020[2][3]) + (-(YMF*S2010[2][1]) + XMF*S2010[2][2])*Si1020[1][2]*T11020[2][5];
T1020[1][4]=S2010[1][1]*Si1020[1][1]*T11020[1][4] + S2010[2][1]*Si1020[1][2]*T11020[2][5];
T1020[1][5]=S2010[1][2]*Si1020[1][1]*T11020[1][4] + S2010[2][2]*Si1020[1][2]*T11020[2][5];
T1020[1][6]=S2010[1][3]*Si1020[1][1]*T11020[1][4] + S2010[2][3]*Si1020[1][2]*T11020[2][5];

T1020[2][1]=(ZMF*S2010[1][2] + YMF*S2010[1][3])*Si1020[2][1]*T11020[1][4] + (ZMF*S2010[2][2] + YMF*S2010[2][3])*Si1020[2][2]*T11020[2][5] + S2010[1][1]*(Si1020[2][2]*T11020[2][1] + Si1020[2][3]*T11020[3][1]) + S2010[2][1]*(Si1020[2][1]*T11020[1][2] + Si1020[2][3]*T11020[3][2]) + (ZMF*S2010[3][2] + YMF*S2010[3][3])*Si1020[2][3]*T11020[3][6];
T1020[2][2]=(-(ZMF*S2010[1][1]) - XMF*S2010[1][3])*Si1020[2][1]*T11020[1][4] + S2010[3][2]*(Si1020[2][1]*T11020[1][3] + Si1020[2][2]*T11020[2][3]) + (-(ZMF*S2010[2][1]) - XMF*S2010[2][3])*Si1020[2][2]*T11020[2][5] + S2010[1][2]*(Si1020[2][2]*T11020[2][1] + Si1020[2][3]*T11020[3][1]) + S2010[2][2]*(Si1020[2][1]*T11020[1][2] + Si1020[2][3]*T11020[3][2]) - XMF*S2010[3][3]*Si1020[2][3]*T11020[3][6];
T1020[2][3]=(-(YMF*S2010[1][1]) + XMF*S2010[1][2])*Si1020[2][1]*T11020[1][4] + S2010[3][3]*(Si1020[2][1]*T11020[1][3] + Si1020[2][2]*T11020[2][3]) + (-(YMF*S2010[2][1]) + XMF*S2010[2][2])*Si1020[2][2]*T11020[2][5] + S2010[1][3]*(Si1020[2][2]*T11020[2][1] + Si1020[2][3]*T11020[3][1]) + S2010[2][3]*(Si1020[2][1]*T11020[1][2] + Si1020[2][3]*T11020[3][2]) + XMF*S2010[3][2]*Si1020[2][3]*T11020[3][6];
T1020[2][4]=S2010[1][1]*Si1020[2][1]*T11020[1][4] + S2010[2][1]*Si1020[2][2]*T11020[2][5];
T1020[2][5]=S2010[1][2]*Si1020[2][1]*T11020[1][4] + S2010[2][2]*Si1020[2][2]*T11020[2][5] + S2010[3][2]*Si1020[2][3]*T11020[3][6];
T1020[2][6]=S2010[1][3]*Si1020[2][1]*T11020[1][4] + S2010[2][3]*Si1020[2][2]*T11020[2][5] + S2010[3][3]*Si1020[2][3]*T11020[3][6];

T1020[3][1]=(ZMF*S2010[1][2] + YMF*S2010[1][3])*Si1020[3][1]*T11020[1][4] + (ZMF*S2010[2][2] + YMF*S2010[2][3])*Si1020[3][2]*T11020[2][5] + S2010[1][1]*(Si1020[3][2]*T11020[2][1] + Si1020[3][3]*T11020[3][1]) + S2010[2][1]*(Si1020[3][1]*T11020[1][2] + Si1020[3][3]*T11020[3][2]) + (ZMF*S2010[3][2] + YMF*S2010[3][3])*Si1020[3][3]*T11020[3][6];
T1020[3][2]=(-(ZMF*S2010[1][1]) - XMF*S2010[1][3])*Si1020[3][1]*T11020[1][4] + S2010[3][2]*(Si1020[3][1]*T11020[1][3] + Si1020[3][2]*T11020[2][3]) + (-(ZMF*S2010[2][1]) - XMF*S2010[2][3])*Si1020[3][2]*T11020[2][5] + S2010[1][2]*(Si1020[3][2]*T11020[2][1] + Si1020[3][3]*T11020[3][1]) + S2010[2][2]*(Si1020[3][1]*T11020[1][2] + Si1020[3][3]*T11020[3][2]) - XMF*S2010[3][3]*Si1020[3][3]*T11020[3][6];
T1020[3][3]=(-(YMF*S2010[1][1]) + XMF*S2010[1][2])*Si1020[3][1]*T11020[1][4] + S2010[3][3]*(Si1020[3][1]*T11020[1][3] + Si1020[3][2]*T11020[2][3]) + (-(YMF*S2010[2][1]) + XMF*S2010[2][2])*Si1020[3][2]*T11020[2][5] + S2010[1][3]*(Si1020[3][2]*T11020[2][1] + Si1020[3][3]*T11020[3][1]) + S2010[2][3]*(Si1020[3][1]*T11020[1][2] + Si1020[3][3]*T11020[3][2]) + XMF*S2010[3][2]*Si1020[3][3]*T11020[3][6];
T1020[3][4]=S2010[1][1]*Si1020[3][1]*T11020[1][4] + S2010[2][1]*Si1020[3][2]*T11020[2][5];
T1020[3][5]=S2010[1][2]*Si1020[3][1]*T11020[1][4] + S2010[2][2]*Si1020[3][2]*T11020[2][5] + S2010[3][2]*Si1020[3][3]*T11020[3][6];
T1020[3][6]=S2010[1][3]*Si1020[3][1]*T11020[1][4] + S2010[2][3]*Si1020[3][2]*T11020[2][5] + S2010[3][3]*Si1020[3][3]*T11020[3][6];

T1020[4][1]=(ZMF*S2010[2][2] + YMF*S2010[2][3])*((ZMF*Si1020[2][2] + YMF*Si1020[3][2])*T11020[2][5] + Si1020[1][1]*T11020[4][5]) + S2010[1][1]*((ZMF*Si1020[2][2] + YMF*Si1020[3][2])*T11020[2][1] + (ZMF*Si1020[2][3] + YMF*Si1020[3][3])*T11020[3][1] + Si1020[1][1]*T11020[4][1] + Si1020[1][2]*T11020[5][1]) + S2010[2][1]*((ZMF*Si1020[2][1] + YMF*Si1020[3][1])*T11020[1][2] + (ZMF*Si1020[2][3] + YMF*Si1020[3][3])*T11020[3][2] + Si1020[1][1]*T11020[4][2] + Si1020[1][2]*T11020[5][2]) + (ZMF*S2010[1][2] + YMF*S2010[1][3])*((ZMF*Si1020[2][1] + YMF*Si1020[3][1])*T11020[1][4] + Si1020[1][2]*T11020[5][4]) + (ZMF*S2010[3][2] + YMF*S2010[3][3])*((ZMF*Si1020[2][3] + YMF*Si1020[3][3])*T11020[3][6] + Si1020[1][1]*T11020[4][6] + Si1020[1][2]*T11020[5][6]);
T1020[4][2]=(-(ZMF*S2010[2][1]) - XMF*S2010[2][3])*((ZMF*Si1020[2][2] + YMF*Si1020[3][2])*T11020[2][5] + Si1020[1][1]*T11020[4][5]) + S2010[1][2]*((ZMF*Si1020[2][2] + YMF*Si1020[3][2])*T11020[2][1] + (ZMF*Si1020[2][3] + YMF*Si1020[3][3])*T11020[3][1] + Si1020[1][1]*T11020[4][1] + Si1020[1][2]*T11020[5][1]) + S2010[2][2]*((ZMF*Si1020[2][1] + YMF*Si1020[3][1])*T11020[1][2] + (ZMF*Si1020[2][3] + YMF*Si1020[3][3])*T11020[3][2] + Si1020[1][1]*T11020[4][2] + Si1020[1][2]*T11020[5][2]) + S2010[3][2]*((ZMF*Si1020[2][1] + YMF*Si1020[3][1])*T11020[1][3] + (ZMF*Si1020[2][2] + YMF*Si1020[3][2])*T11020[2][3] + Si1020[1][1]*T11020[4][3] + Si1020[1][2]*T11020[5][3]) + (-(ZMF*S2010[1][1]) - XMF*S2010[1][3])*((ZMF*Si1020[2][1] + YMF*Si1020[3][1])*T11020[1][4] + Si1020[1][2]*T11020[5][4]) - XMF*S2010[3][3]*((ZMF*Si1020[2][3] + YMF*Si1020[3][3])*T11020[3][6] + Si1020[1][1]*T11020[4][6] + Si1020[1][2]*T11020[5][6]);
T1020[4][3]=(-(YMF*S2010[2][1]) + XMF*S2010[2][2])*((ZMF*Si1020[2][2] + YMF*Si1020[3][2])*T11020[2][5] + Si1020[1][1]*T11020[4][5]) + S2010[1][3]*((ZMF*Si1020[2][2] + YMF*Si1020[3][2])*T11020[2][1] + (ZMF*Si1020[2][3] + YMF*Si1020[3][3])*T11020[3][1] + Si1020[1][1]*T11020[4][1] + Si1020[1][2]*T11020[5][1]) + S2010[2][3]*((ZMF*Si1020[2][1] + YMF*Si1020[3][1])*T11020[1][2] + (ZMF*Si1020[2][3] + YMF*Si1020[3][3])*T11020[3][2] + Si1020[1][1]*T11020[4][2] + Si1020[1][2]*T11020[5][2]) + S2010[3][3]*((ZMF*Si1020[2][1] + YMF*Si1020[3][1])*T11020[1][3] + (ZMF*Si1020[2][2] + YMF*Si1020[3][2])*T11020[2][3] + Si1020[1][1]*T11020[4][3] + Si1020[1][2]*T11020[5][3]) + (-(YMF*S2010[1][1]) + XMF*S2010[1][2])*((ZMF*Si1020[2][1] + YMF*Si1020[3][1])*T11020[1][4] + Si1020[1][2]*T11020[5][4]) + XMF*S2010[3][2]*((ZMF*Si1020[2][3] + YMF*Si1020[3][3])*T11020[3][6] + Si1020[1][1]*T11020[4][6] + Si1020[1][2]*T11020[5][6]);
T1020[4][4]=S2010[2][1]*((ZMF*Si1020[2][2] + YMF*Si1020[3][2])*T11020[2][5] + Si1020[1][1]*T11020[4][5]) + S2010[1][1]*((ZMF*Si1020[2][1] + YMF*Si1020[3][1])*T11020[1][4] + Si1020[1][2]*T11020[5][4]);
T1020[4][5]=S2010[2][2]*((ZMF*Si1020[2][2] + YMF*Si1020[3][2])*T11020[2][5] + Si1020[1][1]*T11020[4][5]) + S2010[1][2]*((ZMF*Si1020[2][1] + YMF*Si1020[3][1])*T11020[1][4] + Si1020[1][2]*T11020[5][4]) + S2010[3][2]*((ZMF*Si1020[2][3] + YMF*Si1020[3][3])*T11020[3][6] + Si1020[1][1]*T11020[4][6] + Si1020[1][2]*T11020[5][6]);
T1020[4][6]=S2010[2][3]*((ZMF*Si1020[2][2] + YMF*Si1020[3][2])*T11020[2][5] + Si1020[1][1]*T11020[4][5]) + S2010[1][3]*((ZMF*Si1020[2][1] + YMF*Si1020[3][1])*T11020[1][4] + Si1020[1][2]*T11020[5][4]) + S2010[3][3]*((ZMF*Si1020[2][3] + YMF*Si1020[3][3])*T11020[3][6] + Si1020[1][1]*T11020[4][6] + Si1020[1][2]*T11020[5][6]);

T1020[5][1]=(ZMF*S2010[3][2] + YMF*S2010[3][3])*(-(XMF*Si1020[3][3]*T11020[3][6]) + Si1020[2][1]*T11020[4][6] + Si1020[2][2]*T11020[5][6]) + S2010[1][1]*((-(ZMF*Si1020[1][2]) - XMF*Si1020[3][2])*T11020[2][1] - XMF*Si1020[3][3]*T11020[3][1] + Si1020[2][1]*T11020[4][1] + Si1020[2][2]*T11020[5][1] + Si1020[2][3]*T11020[6][1]) + S2010[2][1]*((-(ZMF*Si1020[1][1]) - XMF*Si1020[3][1])*T11020[1][2] - XMF*Si1020[3][3]*T11020[3][2] + Si1020[2][1]*T11020[4][2] + Si1020[2][2]*T11020[5][2] + Si1020[2][3]*T11020[6][2]) + (ZMF*S2010[1][2] + YMF*S2010[1][3])*((-(ZMF*Si1020[1][1]) - XMF*Si1020[3][1])*T11020[1][4] + Si1020[2][2]*T11020[5][4] + Si1020[2][3]*T11020[6][4]) + (ZMF*S2010[2][2] + YMF*S2010[2][3])*((-(ZMF*Si1020[1][2]) - XMF*Si1020[3][2])*T11020[2][5] + Si1020[2][1]*T11020[4][5] + Si1020[2][3]*T11020[6][5]);
T1020[5][2]=-(XMF*S2010[3][3]*(-(XMF*Si1020[3][3]*T11020[3][6]) + Si1020[2][1]*T11020[4][6] + Si1020[2][2]*T11020[5][6])) + S2010[1][2]*((-(ZMF*Si1020[1][2]) - XMF*Si1020[3][2])*T11020[2][1] - XMF*Si1020[3][3]*T11020[3][1] + Si1020[2][1]*T11020[4][1] + Si1020[2][2]*T11020[5][1] + Si1020[2][3]*T11020[6][1]) + S2010[2][2]*((-(ZMF*Si1020[1][1]) - XMF*Si1020[3][1])*T11020[1][2] - XMF*Si1020[3][3]*T11020[3][2] + Si1020[2][1]*T11020[4][2] + Si1020[2][2]*T11020[5][2] + Si1020[2][3]*T11020[6][2]) + S2010[3][2]*((-(ZMF*Si1020[1][1]) - XMF*Si1020[3][1])*T11020[1][3] + (-(ZMF*Si1020[1][2]) - XMF*Si1020[3][2])*T11020[2][3] + Si1020[2][1]*T11020[4][3] + Si1020[2][2]*T11020[5][3] + Si1020[2][3]*T11020[6][3]) + (-(ZMF*S2010[1][1]) - XMF*S2010[1][3])*((-(ZMF*Si1020[1][1]) - XMF*Si1020[3][1])*T11020[1][4] + Si1020[2][2]*T11020[5][4] + Si1020[2][3]*T11020[6][4]) + (-(ZMF*S2010[2][1]) - XMF*S2010[2][3])*((-(ZMF*Si1020[1][2]) - XMF*Si1020[3][2])*T11020[2][5] + Si1020[2][1]*T11020[4][5] + Si1020[2][3]*T11020[6][5]);
T1020[5][3]=XMF*S2010[3][2]*(-(XMF*Si1020[3][3]*T11020[3][6]) + Si1020[2][1]*T11020[4][6] + Si1020[2][2]*T11020[5][6]) + S2010[1][3]*((-(ZMF*Si1020[1][2]) - XMF*Si1020[3][2])*T11020[2][1] - XMF*Si1020[3][3]*T11020[3][1] + Si1020[2][1]*T11020[4][1] + Si1020[2][2]*T11020[5][1] + Si1020[2][3]*T11020[6][1]) + S2010[2][3]*((-(ZMF*Si1020[1][1]) - XMF*Si1020[3][1])*T11020[1][2] - XMF*Si1020[3][3]*T11020[3][2] + Si1020[2][1]*T11020[4][2] + Si1020[2][2]*T11020[5][2] + Si1020[2][3]*T11020[6][2]) + S2010[3][3]*((-(ZMF*Si1020[1][1]) - XMF*Si1020[3][1])*T11020[1][3] + (-(ZMF*Si1020[1][2]) - XMF*Si1020[3][2])*T11020[2][3] + Si1020[2][1]*T11020[4][3] + Si1020[2][2]*T11020[5][3] + Si1020[2][3]*T11020[6][3]) + (-(YMF*S2010[1][1]) + XMF*S2010[1][2])*((-(ZMF*Si1020[1][1]) - XMF*Si1020[3][1])*T11020[1][4] + Si1020[2][2]*T11020[5][4] + Si1020[2][3]*T11020[6][4]) + (-(YMF*S2010[2][1]) + XMF*S2010[2][2])*((-(ZMF*Si1020[1][2]) - XMF*Si1020[3][2])*T11020[2][5] + Si1020[2][1]*T11020[4][5] + Si1020[2][3]*T11020[6][5]);
T1020[5][4]=S2010[1][1]*((-(ZMF*Si1020[1][1]) - XMF*Si1020[3][1])*T11020[1][4] + Si1020[2][2]*T11020[5][4] + Si1020[2][3]*T11020[6][4]) + S2010[2][1]*((-(ZMF*Si1020[1][2]) - XMF*Si1020[3][2])*T11020[2][5] + Si1020[2][1]*T11020[4][5] + Si1020[2][3]*T11020[6][5]);
T1020[5][5]=S2010[3][2]*(-(XMF*Si1020[3][3]*T11020[3][6]) + Si1020[2][1]*T11020[4][6] + Si1020[2][2]*T11020[5][6]) + S2010[1][2]*((-(ZMF*Si1020[1][1]) - XMF*Si1020[3][1])*T11020[1][4] + Si1020[2][2]*T11020[5][4] + Si1020[2][3]*T11020[6][4]) + S2010[2][2]*((-(ZMF*Si1020[1][2]) - XMF*Si1020[3][2])*T11020[2][5] + Si1020[2][1]*T11020[4][5] + Si1020[2][3]*T11020[6][5]);
T1020[5][6]=S2010[3][3]*(-(XMF*Si1020[3][3]*T11020[3][6]) + Si1020[2][1]*T11020[4][6] + Si1020[2][2]*T11020[5][6]) + S2010[1][3]*((-(ZMF*Si1020[1][1]) - XMF*Si1020[3][1])*T11020[1][4] + Si1020[2][2]*T11020[5][4] + Si1020[2][3]*T11020[6][4]) + S2010[2][3]*((-(ZMF*Si1020[1][2]) - XMF*Si1020[3][2])*T11020[2][5] + Si1020[2][1]*T11020[4][5] + Si1020[2][3]*T11020[6][5]);

T1020[6][1]=(ZMF*S2010[3][2] + YMF*S2010[3][3])*(XMF*Si1020[2][3]*T11020[3][6] + Si1020[3][1]*T11020[4][6] + Si1020[3][2]*T11020[5][6]) + S2010[1][1]*((-(YMF*Si1020[1][2]) + XMF*Si1020[2][2])*T11020[2][1] + XMF*Si1020[2][3]*T11020[3][1] + Si1020[3][1]*T11020[4][1] + Si1020[3][2]*T11020[5][1] + Si1020[3][3]*T11020[6][1]) + S2010[2][1]*((-(YMF*Si1020[1][1]) + XMF*Si1020[2][1])*T11020[1][2] + XMF*Si1020[2][3]*T11020[3][2] + Si1020[3][1]*T11020[4][2] + Si1020[3][2]*T11020[5][2] + Si1020[3][3]*T11020[6][2]) + (ZMF*S2010[1][2] + YMF*S2010[1][3])*((-(YMF*Si1020[1][1]) + XMF*Si1020[2][1])*T11020[1][4] + Si1020[3][2]*T11020[5][4] + Si1020[3][3]*T11020[6][4]) + (ZMF*S2010[2][2] + YMF*S2010[2][3])*((-(YMF*Si1020[1][2]) + XMF*Si1020[2][2])*T11020[2][5] + Si1020[3][1]*T11020[4][5] + Si1020[3][3]*T11020[6][5]);
T1020[6][2]=-(XMF*S2010[3][3]*(XMF*Si1020[2][3]*T11020[3][6] + Si1020[3][1]*T11020[4][6] + Si1020[3][2]*T11020[5][6])) + S2010[1][2]*((-(YMF*Si1020[1][2]) + XMF*Si1020[2][2])*T11020[2][1] + XMF*Si1020[2][3]*T11020[3][1] + Si1020[3][1]*T11020[4][1] + Si1020[3][2]*T11020[5][1] + Si1020[3][3]*T11020[6][1]) + S2010[2][2]*((-(YMF*Si1020[1][1]) + XMF*Si1020[2][1])*T11020[1][2] + XMF*Si1020[2][3]*T11020[3][2] + Si1020[3][1]*T11020[4][2] + Si1020[3][2]*T11020[5][2] + Si1020[3][3]*T11020[6][2]) + S2010[3][2]*((-(YMF*Si1020[1][1]) + XMF*Si1020[2][1])*T11020[1][3] + (-(YMF*Si1020[1][2]) + XMF*Si1020[2][2])*T11020[2][3] + Si1020[3][1]*T11020[4][3] + Si1020[3][2]*T11020[5][3] + Si1020[3][3]*T11020[6][3]) + (-(ZMF*S2010[1][1]) - XMF*S2010[1][3])*((-(YMF*Si1020[1][1]) + XMF*Si1020[2][1])*T11020[1][4] + Si1020[3][2]*T11020[5][4] + Si1020[3][3]*T11020[6][4]) + (-(ZMF*S2010[2][1]) - XMF*S2010[2][3])*((-(YMF*Si1020[1][2]) + XMF*Si1020[2][2])*T11020[2][5] + Si1020[3][1]*T11020[4][5] + Si1020[3][3]*T11020[6][5]);
T1020[6][3]=XMF*S2010[3][2]*(XMF*Si1020[2][3]*T11020[3][6] + Si1020[3][1]*T11020[4][6] + Si1020[3][2]*T11020[5][6]) + S2010[1][3]*((-(YMF*Si1020[1][2]) + XMF*Si1020[2][2])*T11020[2][1] + XMF*Si1020[2][3]*T11020[3][1] + Si1020[3][1]*T11020[4][1] + Si1020[3][2]*T11020[5][1] + Si1020[3][3]*T11020[6][1]) + S2010[2][3]*((-(YMF*Si1020[1][1]) + XMF*Si1020[2][1])*T11020[1][2] + XMF*Si1020[2][3]*T11020[3][2] + Si1020[3][1]*T11020[4][2] + Si1020[3][2]*T11020[5][2] + Si1020[3][3]*T11020[6][2]) + S2010[3][3]*((-(YMF*Si1020[1][1]) + XMF*Si1020[2][1])*T11020[1][3] + (-(YMF*Si1020[1][2]) + XMF*Si1020[2][2])*T11020[2][3] + Si1020[3][1]*T11020[4][3] + Si1020[3][2]*T11020[5][3] + Si1020[3][3]*T11020[6][3]) + (-(YMF*S2010[1][1]) + XMF*S2010[1][2])*((-(YMF*Si1020[1][1]) + XMF*Si1020[2][1])*T11020[1][4] + Si1020[3][2]*T11020[5][4] + Si1020[3][3]*T11020[6][4]) + (-(YMF*S2010[2][1]) + XMF*S2010[2][2])*((-(YMF*Si1020[1][2]) + XMF*Si1020[2][2])*T11020[2][5] + Si1020[3][1]*T11020[4][5] + Si1020[3][3]*T11020[6][5]);
T1020[6][4]=S2010[1][1]*((-(YMF*Si1020[1][1]) + XMF*Si1020[2][1])*T11020[1][4] + Si1020[3][2]*T11020[5][4] + Si1020[3][3]*T11020[6][4]) + S2010[2][1]*((-(YMF*Si1020[1][2]) + XMF*Si1020[2][2])*T11020[2][5] + Si1020[3][1]*T11020[4][5] + Si1020[3][3]*T11020[6][5]);
T1020[6][5]=S2010[3][2]*(XMF*Si1020[2][3]*T11020[3][6] + Si1020[3][1]*T11020[4][6] + Si1020[3][2]*T11020[5][6]) + S2010[1][2]*((-(YMF*Si1020[1][1]) + XMF*Si1020[2][1])*T11020[1][4] + Si1020[3][2]*T11020[5][4] + Si1020[3][3]*T11020[6][4]) + S2010[2][2]*((-(YMF*Si1020[1][2]) + XMF*Si1020[2][2])*T11020[2][5] + Si1020[3][1]*T11020[4][5] + Si1020[3][3]*T11020[6][5]);
T1020[6][6]=S2010[3][3]*(XMF*Si1020[2][3]*T11020[3][6] + Si1020[3][1]*T11020[4][6] + Si1020[3][2]*T11020[5][6]) + S2010[1][3]*((-(YMF*Si1020[1][1]) + XMF*Si1020[2][1])*T11020[1][4] + Si1020[3][2]*T11020[5][4] + Si1020[3][3]*T11020[6][4]) + S2010[2][3]*((-(YMF*Si1020[1][2]) + XMF*Si1020[2][2])*T11020[2][5] + Si1020[3][1]*T11020[4][5] + Si1020[3][3]*T11020[6][5]);



}


void
hermes_InvDynArtfunc85(void)
      {




}


void
hermes_InvDynArtfunc86(void)
      {




}


void
hermes_InvDynArtfunc87(void)
      {




}


void
hermes_InvDynArtfunc88(void)
      {
JA16[1][2]=0. + links[41].mcm[3];
JA16[1][3]=0. - links[41].mcm[2];
JA16[1][4]=0. + links[41].m;

JA16[2][1]=0. - links[41].mcm[3];
JA16[2][3]=0. + links[41].mcm[1];
JA16[2][5]=0. + links[41].m;

JA16[3][1]=0. + links[41].mcm[2];
JA16[3][2]=0. - links[41].mcm[1];
JA16[3][6]=0. + links[41].m;

JA16[4][1]=0. + links[41].inertia[1][1];
JA16[4][2]=0. + links[41].inertia[1][2];
JA16[4][3]=0. + links[41].inertia[1][3];
JA16[4][5]=0. - links[41].mcm[3];
JA16[4][6]=0. + links[41].mcm[2];

JA16[5][1]=0. + links[41].inertia[1][2];
JA16[5][2]=0. + links[41].inertia[2][2];
JA16[5][3]=0. + links[41].inertia[2][3];
JA16[5][4]=0. + links[41].mcm[3];
JA16[5][6]=0. - links[41].mcm[1];

JA16[6][1]=0. + links[41].inertia[1][3];
JA16[6][2]=0. + links[41].inertia[2][3];
JA16[6][3]=0. + links[41].inertia[3][3];
JA16[6][4]=0. - links[41].mcm[2];
JA16[6][5]=0. + links[41].mcm[1];


h16[1]=JA16[1][3];
h16[2]=JA16[2][3];
h16[4]=JA16[4][3];
h16[5]=JA16[5][3];
h16[6]=JA16[6][3];

T11016[1][2]=JA16[1][2];
T11016[1][3]=JA16[1][3];
T11016[1][4]=JA16[1][4];

T11016[2][1]=JA16[2][1];
T11016[2][3]=JA16[2][3];
T11016[2][5]=JA16[2][5];

T11016[3][1]=JA16[3][1];
T11016[3][2]=JA16[3][2];
T11016[3][6]=JA16[3][6];

T11016[4][1]=JA16[4][1];
T11016[4][2]=JA16[4][2];
T11016[4][3]=JA16[4][3];
T11016[4][5]=JA16[4][5];
T11016[4][6]=JA16[4][6];

T11016[5][1]=JA16[5][1];
T11016[5][2]=JA16[5][2];
T11016[5][3]=JA16[5][3];
T11016[5][4]=JA16[5][4];
T11016[5][6]=JA16[5][6];

T11016[6][1]=JA16[6][1];
T11016[6][2]=JA16[6][2];
T11016[6][3]=JA16[6][3];
T11016[6][4]=JA16[6][4];
T11016[6][5]=JA16[6][5];


T1016[1][1]=S1610[2][1]*Si1016[1][1]*T11016[1][2] + (ZIF*S1610[1][2] + YIF*S1610[1][3])*Si1016[1][1]*T11016[1][4] + S1610[1][1]*Si1016[1][2]*T11016[2][1] + (ZIF*S1610[2][2] + YIF*S1610[2][3])*Si1016[1][2]*T11016[2][5];
T1016[1][2]=S1610[2][2]*Si1016[1][1]*T11016[1][2] + (-(ZIF*S1610[1][1]) - XIF*S1610[1][3])*Si1016[1][1]*T11016[1][4] + S1610[1][2]*Si1016[1][2]*T11016[2][1] + S1610[3][2]*(Si1016[1][1]*T11016[1][3] + Si1016[1][2]*T11016[2][3]) + (-(ZIF*S1610[2][1]) - XIF*S1610[2][3])*Si1016[1][2]*T11016[2][5];
T1016[1][3]=S1610[2][3]*Si1016[1][1]*T11016[1][2] + (-(YIF*S1610[1][1]) + XIF*S1610[1][2])*Si1016[1][1]*T11016[1][4] + S1610[1][3]*Si1016[1][2]*T11016[2][1] + S1610[3][3]*(Si1016[1][1]*T11016[1][3] + Si1016[1][2]*T11016[2][3]) + (-(YIF*S1610[2][1]) + XIF*S1610[2][2])*Si1016[1][2]*T11016[2][5];
T1016[1][4]=S1610[1][1]*Si1016[1][1]*T11016[1][4] + S1610[2][1]*Si1016[1][2]*T11016[2][5];
T1016[1][5]=S1610[1][2]*Si1016[1][1]*T11016[1][4] + S1610[2][2]*Si1016[1][2]*T11016[2][5];
T1016[1][6]=S1610[1][3]*Si1016[1][1]*T11016[1][4] + S1610[2][3]*Si1016[1][2]*T11016[2][5];

T1016[2][1]=(ZIF*S1610[1][2] + YIF*S1610[1][3])*Si1016[2][1]*T11016[1][4] + (ZIF*S1610[2][2] + YIF*S1610[2][3])*Si1016[2][2]*T11016[2][5] + S1610[1][1]*(Si1016[2][2]*T11016[2][1] + Si1016[2][3]*T11016[3][1]) + S1610[2][1]*(Si1016[2][1]*T11016[1][2] + Si1016[2][3]*T11016[3][2]) + (ZIF*S1610[3][2] + YIF*S1610[3][3])*Si1016[2][3]*T11016[3][6];
T1016[2][2]=(-(ZIF*S1610[1][1]) - XIF*S1610[1][3])*Si1016[2][1]*T11016[1][4] + S1610[3][2]*(Si1016[2][1]*T11016[1][3] + Si1016[2][2]*T11016[2][3]) + (-(ZIF*S1610[2][1]) - XIF*S1610[2][3])*Si1016[2][2]*T11016[2][5] + S1610[1][2]*(Si1016[2][2]*T11016[2][1] + Si1016[2][3]*T11016[3][1]) + S1610[2][2]*(Si1016[2][1]*T11016[1][2] + Si1016[2][3]*T11016[3][2]) - XIF*S1610[3][3]*Si1016[2][3]*T11016[3][6];
T1016[2][3]=(-(YIF*S1610[1][1]) + XIF*S1610[1][2])*Si1016[2][1]*T11016[1][4] + S1610[3][3]*(Si1016[2][1]*T11016[1][3] + Si1016[2][2]*T11016[2][3]) + (-(YIF*S1610[2][1]) + XIF*S1610[2][2])*Si1016[2][2]*T11016[2][5] + S1610[1][3]*(Si1016[2][2]*T11016[2][1] + Si1016[2][3]*T11016[3][1]) + S1610[2][3]*(Si1016[2][1]*T11016[1][2] + Si1016[2][3]*T11016[3][2]) + XIF*S1610[3][2]*Si1016[2][3]*T11016[3][6];
T1016[2][4]=S1610[1][1]*Si1016[2][1]*T11016[1][4] + S1610[2][1]*Si1016[2][2]*T11016[2][5];
T1016[2][5]=S1610[1][2]*Si1016[2][1]*T11016[1][4] + S1610[2][2]*Si1016[2][2]*T11016[2][5] + S1610[3][2]*Si1016[2][3]*T11016[3][6];
T1016[2][6]=S1610[1][3]*Si1016[2][1]*T11016[1][4] + S1610[2][3]*Si1016[2][2]*T11016[2][5] + S1610[3][3]*Si1016[2][3]*T11016[3][6];

T1016[3][1]=(ZIF*S1610[1][2] + YIF*S1610[1][3])*Si1016[3][1]*T11016[1][4] + (ZIF*S1610[2][2] + YIF*S1610[2][3])*Si1016[3][2]*T11016[2][5] + S1610[1][1]*(Si1016[3][2]*T11016[2][1] + Si1016[3][3]*T11016[3][1]) + S1610[2][1]*(Si1016[3][1]*T11016[1][2] + Si1016[3][3]*T11016[3][2]) + (ZIF*S1610[3][2] + YIF*S1610[3][3])*Si1016[3][3]*T11016[3][6];
T1016[3][2]=(-(ZIF*S1610[1][1]) - XIF*S1610[1][3])*Si1016[3][1]*T11016[1][4] + S1610[3][2]*(Si1016[3][1]*T11016[1][3] + Si1016[3][2]*T11016[2][3]) + (-(ZIF*S1610[2][1]) - XIF*S1610[2][3])*Si1016[3][2]*T11016[2][5] + S1610[1][2]*(Si1016[3][2]*T11016[2][1] + Si1016[3][3]*T11016[3][1]) + S1610[2][2]*(Si1016[3][1]*T11016[1][2] + Si1016[3][3]*T11016[3][2]) - XIF*S1610[3][3]*Si1016[3][3]*T11016[3][6];
T1016[3][3]=(-(YIF*S1610[1][1]) + XIF*S1610[1][2])*Si1016[3][1]*T11016[1][4] + S1610[3][3]*(Si1016[3][1]*T11016[1][3] + Si1016[3][2]*T11016[2][3]) + (-(YIF*S1610[2][1]) + XIF*S1610[2][2])*Si1016[3][2]*T11016[2][5] + S1610[1][3]*(Si1016[3][2]*T11016[2][1] + Si1016[3][3]*T11016[3][1]) + S1610[2][3]*(Si1016[3][1]*T11016[1][2] + Si1016[3][3]*T11016[3][2]) + XIF*S1610[3][2]*Si1016[3][3]*T11016[3][6];
T1016[3][4]=S1610[1][1]*Si1016[3][1]*T11016[1][4] + S1610[2][1]*Si1016[3][2]*T11016[2][5];
T1016[3][5]=S1610[1][2]*Si1016[3][1]*T11016[1][4] + S1610[2][2]*Si1016[3][2]*T11016[2][5] + S1610[3][2]*Si1016[3][3]*T11016[3][6];
T1016[3][6]=S1610[1][3]*Si1016[3][1]*T11016[1][4] + S1610[2][3]*Si1016[3][2]*T11016[2][5] + S1610[3][3]*Si1016[3][3]*T11016[3][6];

T1016[4][1]=(ZIF*S1610[2][2] + YIF*S1610[2][3])*((ZIF*Si1016[2][2] + YIF*Si1016[3][2])*T11016[2][5] + Si1016[1][1]*T11016[4][5]) + S1610[1][1]*((ZIF*Si1016[2][2] + YIF*Si1016[3][2])*T11016[2][1] + (ZIF*Si1016[2][3] + YIF*Si1016[3][3])*T11016[3][1] + Si1016[1][1]*T11016[4][1] + Si1016[1][2]*T11016[5][1]) + S1610[2][1]*((ZIF*Si1016[2][1] + YIF*Si1016[3][1])*T11016[1][2] + (ZIF*Si1016[2][3] + YIF*Si1016[3][3])*T11016[3][2] + Si1016[1][1]*T11016[4][2] + Si1016[1][2]*T11016[5][2]) + (ZIF*S1610[1][2] + YIF*S1610[1][3])*((ZIF*Si1016[2][1] + YIF*Si1016[3][1])*T11016[1][4] + Si1016[1][2]*T11016[5][4]) + (ZIF*S1610[3][2] + YIF*S1610[3][3])*((ZIF*Si1016[2][3] + YIF*Si1016[3][3])*T11016[3][6] + Si1016[1][1]*T11016[4][6] + Si1016[1][2]*T11016[5][6]);
T1016[4][2]=(-(ZIF*S1610[2][1]) - XIF*S1610[2][3])*((ZIF*Si1016[2][2] + YIF*Si1016[3][2])*T11016[2][5] + Si1016[1][1]*T11016[4][5]) + S1610[1][2]*((ZIF*Si1016[2][2] + YIF*Si1016[3][2])*T11016[2][1] + (ZIF*Si1016[2][3] + YIF*Si1016[3][3])*T11016[3][1] + Si1016[1][1]*T11016[4][1] + Si1016[1][2]*T11016[5][1]) + S1610[2][2]*((ZIF*Si1016[2][1] + YIF*Si1016[3][1])*T11016[1][2] + (ZIF*Si1016[2][3] + YIF*Si1016[3][3])*T11016[3][2] + Si1016[1][1]*T11016[4][2] + Si1016[1][2]*T11016[5][2]) + S1610[3][2]*((ZIF*Si1016[2][1] + YIF*Si1016[3][1])*T11016[1][3] + (ZIF*Si1016[2][2] + YIF*Si1016[3][2])*T11016[2][3] + Si1016[1][1]*T11016[4][3] + Si1016[1][2]*T11016[5][3]) + (-(ZIF*S1610[1][1]) - XIF*S1610[1][3])*((ZIF*Si1016[2][1] + YIF*Si1016[3][1])*T11016[1][4] + Si1016[1][2]*T11016[5][4]) - XIF*S1610[3][3]*((ZIF*Si1016[2][3] + YIF*Si1016[3][3])*T11016[3][6] + Si1016[1][1]*T11016[4][6] + Si1016[1][2]*T11016[5][6]);
T1016[4][3]=(-(YIF*S1610[2][1]) + XIF*S1610[2][2])*((ZIF*Si1016[2][2] + YIF*Si1016[3][2])*T11016[2][5] + Si1016[1][1]*T11016[4][5]) + S1610[1][3]*((ZIF*Si1016[2][2] + YIF*Si1016[3][2])*T11016[2][1] + (ZIF*Si1016[2][3] + YIF*Si1016[3][3])*T11016[3][1] + Si1016[1][1]*T11016[4][1] + Si1016[1][2]*T11016[5][1]) + S1610[2][3]*((ZIF*Si1016[2][1] + YIF*Si1016[3][1])*T11016[1][2] + (ZIF*Si1016[2][3] + YIF*Si1016[3][3])*T11016[3][2] + Si1016[1][1]*T11016[4][2] + Si1016[1][2]*T11016[5][2]) + S1610[3][3]*((ZIF*Si1016[2][1] + YIF*Si1016[3][1])*T11016[1][3] + (ZIF*Si1016[2][2] + YIF*Si1016[3][2])*T11016[2][3] + Si1016[1][1]*T11016[4][3] + Si1016[1][2]*T11016[5][3]) + (-(YIF*S1610[1][1]) + XIF*S1610[1][2])*((ZIF*Si1016[2][1] + YIF*Si1016[3][1])*T11016[1][4] + Si1016[1][2]*T11016[5][4]) + XIF*S1610[3][2]*((ZIF*Si1016[2][3] + YIF*Si1016[3][3])*T11016[3][6] + Si1016[1][1]*T11016[4][6] + Si1016[1][2]*T11016[5][6]);
T1016[4][4]=S1610[2][1]*((ZIF*Si1016[2][2] + YIF*Si1016[3][2])*T11016[2][5] + Si1016[1][1]*T11016[4][5]) + S1610[1][1]*((ZIF*Si1016[2][1] + YIF*Si1016[3][1])*T11016[1][4] + Si1016[1][2]*T11016[5][4]);
T1016[4][5]=S1610[2][2]*((ZIF*Si1016[2][2] + YIF*Si1016[3][2])*T11016[2][5] + Si1016[1][1]*T11016[4][5]) + S1610[1][2]*((ZIF*Si1016[2][1] + YIF*Si1016[3][1])*T11016[1][4] + Si1016[1][2]*T11016[5][4]) + S1610[3][2]*((ZIF*Si1016[2][3] + YIF*Si1016[3][3])*T11016[3][6] + Si1016[1][1]*T11016[4][6] + Si1016[1][2]*T11016[5][6]);
T1016[4][6]=S1610[2][3]*((ZIF*Si1016[2][2] + YIF*Si1016[3][2])*T11016[2][5] + Si1016[1][1]*T11016[4][5]) + S1610[1][3]*((ZIF*Si1016[2][1] + YIF*Si1016[3][1])*T11016[1][4] + Si1016[1][2]*T11016[5][4]) + S1610[3][3]*((ZIF*Si1016[2][3] + YIF*Si1016[3][3])*T11016[3][6] + Si1016[1][1]*T11016[4][6] + Si1016[1][2]*T11016[5][6]);

T1016[5][1]=(ZIF*S1610[3][2] + YIF*S1610[3][3])*(-(XIF*Si1016[3][3]*T11016[3][6]) + Si1016[2][1]*T11016[4][6] + Si1016[2][2]*T11016[5][6]) + S1610[1][1]*((-(ZIF*Si1016[1][2]) - XIF*Si1016[3][2])*T11016[2][1] - XIF*Si1016[3][3]*T11016[3][1] + Si1016[2][1]*T11016[4][1] + Si1016[2][2]*T11016[5][1] + Si1016[2][3]*T11016[6][1]) + S1610[2][1]*((-(ZIF*Si1016[1][1]) - XIF*Si1016[3][1])*T11016[1][2] - XIF*Si1016[3][3]*T11016[3][2] + Si1016[2][1]*T11016[4][2] + Si1016[2][2]*T11016[5][2] + Si1016[2][3]*T11016[6][2]) + (ZIF*S1610[1][2] + YIF*S1610[1][3])*((-(ZIF*Si1016[1][1]) - XIF*Si1016[3][1])*T11016[1][4] + Si1016[2][2]*T11016[5][4] + Si1016[2][3]*T11016[6][4]) + (ZIF*S1610[2][2] + YIF*S1610[2][3])*((-(ZIF*Si1016[1][2]) - XIF*Si1016[3][2])*T11016[2][5] + Si1016[2][1]*T11016[4][5] + Si1016[2][3]*T11016[6][5]);
T1016[5][2]=-(XIF*S1610[3][3]*(-(XIF*Si1016[3][3]*T11016[3][6]) + Si1016[2][1]*T11016[4][6] + Si1016[2][2]*T11016[5][6])) + S1610[1][2]*((-(ZIF*Si1016[1][2]) - XIF*Si1016[3][2])*T11016[2][1] - XIF*Si1016[3][3]*T11016[3][1] + Si1016[2][1]*T11016[4][1] + Si1016[2][2]*T11016[5][1] + Si1016[2][3]*T11016[6][1]) + S1610[2][2]*((-(ZIF*Si1016[1][1]) - XIF*Si1016[3][1])*T11016[1][2] - XIF*Si1016[3][3]*T11016[3][2] + Si1016[2][1]*T11016[4][2] + Si1016[2][2]*T11016[5][2] + Si1016[2][3]*T11016[6][2]) + S1610[3][2]*((-(ZIF*Si1016[1][1]) - XIF*Si1016[3][1])*T11016[1][3] + (-(ZIF*Si1016[1][2]) - XIF*Si1016[3][2])*T11016[2][3] + Si1016[2][1]*T11016[4][3] + Si1016[2][2]*T11016[5][3] + Si1016[2][3]*T11016[6][3]) + (-(ZIF*S1610[1][1]) - XIF*S1610[1][3])*((-(ZIF*Si1016[1][1]) - XIF*Si1016[3][1])*T11016[1][4] + Si1016[2][2]*T11016[5][4] + Si1016[2][3]*T11016[6][4]) + (-(ZIF*S1610[2][1]) - XIF*S1610[2][3])*((-(ZIF*Si1016[1][2]) - XIF*Si1016[3][2])*T11016[2][5] + Si1016[2][1]*T11016[4][5] + Si1016[2][3]*T11016[6][5]);
T1016[5][3]=XIF*S1610[3][2]*(-(XIF*Si1016[3][3]*T11016[3][6]) + Si1016[2][1]*T11016[4][6] + Si1016[2][2]*T11016[5][6]) + S1610[1][3]*((-(ZIF*Si1016[1][2]) - XIF*Si1016[3][2])*T11016[2][1] - XIF*Si1016[3][3]*T11016[3][1] + Si1016[2][1]*T11016[4][1] + Si1016[2][2]*T11016[5][1] + Si1016[2][3]*T11016[6][1]) + S1610[2][3]*((-(ZIF*Si1016[1][1]) - XIF*Si1016[3][1])*T11016[1][2] - XIF*Si1016[3][3]*T11016[3][2] + Si1016[2][1]*T11016[4][2] + Si1016[2][2]*T11016[5][2] + Si1016[2][3]*T11016[6][2]) + S1610[3][3]*((-(ZIF*Si1016[1][1]) - XIF*Si1016[3][1])*T11016[1][3] + (-(ZIF*Si1016[1][2]) - XIF*Si1016[3][2])*T11016[2][3] + Si1016[2][1]*T11016[4][3] + Si1016[2][2]*T11016[5][3] + Si1016[2][3]*T11016[6][3]) + (-(YIF*S1610[1][1]) + XIF*S1610[1][2])*((-(ZIF*Si1016[1][1]) - XIF*Si1016[3][1])*T11016[1][4] + Si1016[2][2]*T11016[5][4] + Si1016[2][3]*T11016[6][4]) + (-(YIF*S1610[2][1]) + XIF*S1610[2][2])*((-(ZIF*Si1016[1][2]) - XIF*Si1016[3][2])*T11016[2][5] + Si1016[2][1]*T11016[4][5] + Si1016[2][3]*T11016[6][5]);
T1016[5][4]=S1610[1][1]*((-(ZIF*Si1016[1][1]) - XIF*Si1016[3][1])*T11016[1][4] + Si1016[2][2]*T11016[5][4] + Si1016[2][3]*T11016[6][4]) + S1610[2][1]*((-(ZIF*Si1016[1][2]) - XIF*Si1016[3][2])*T11016[2][5] + Si1016[2][1]*T11016[4][5] + Si1016[2][3]*T11016[6][5]);
T1016[5][5]=S1610[3][2]*(-(XIF*Si1016[3][3]*T11016[3][6]) + Si1016[2][1]*T11016[4][6] + Si1016[2][2]*T11016[5][6]) + S1610[1][2]*((-(ZIF*Si1016[1][1]) - XIF*Si1016[3][1])*T11016[1][4] + Si1016[2][2]*T11016[5][4] + Si1016[2][3]*T11016[6][4]) + S1610[2][2]*((-(ZIF*Si1016[1][2]) - XIF*Si1016[3][2])*T11016[2][5] + Si1016[2][1]*T11016[4][5] + Si1016[2][3]*T11016[6][5]);
T1016[5][6]=S1610[3][3]*(-(XIF*Si1016[3][3]*T11016[3][6]) + Si1016[2][1]*T11016[4][6] + Si1016[2][2]*T11016[5][6]) + S1610[1][3]*((-(ZIF*Si1016[1][1]) - XIF*Si1016[3][1])*T11016[1][4] + Si1016[2][2]*T11016[5][4] + Si1016[2][3]*T11016[6][4]) + S1610[2][3]*((-(ZIF*Si1016[1][2]) - XIF*Si1016[3][2])*T11016[2][5] + Si1016[2][1]*T11016[4][5] + Si1016[2][3]*T11016[6][5]);

T1016[6][1]=(ZIF*S1610[3][2] + YIF*S1610[3][3])*(XIF*Si1016[2][3]*T11016[3][6] + Si1016[3][1]*T11016[4][6] + Si1016[3][2]*T11016[5][6]) + S1610[1][1]*((-(YIF*Si1016[1][2]) + XIF*Si1016[2][2])*T11016[2][1] + XIF*Si1016[2][3]*T11016[3][1] + Si1016[3][1]*T11016[4][1] + Si1016[3][2]*T11016[5][1] + Si1016[3][3]*T11016[6][1]) + S1610[2][1]*((-(YIF*Si1016[1][1]) + XIF*Si1016[2][1])*T11016[1][2] + XIF*Si1016[2][3]*T11016[3][2] + Si1016[3][1]*T11016[4][2] + Si1016[3][2]*T11016[5][2] + Si1016[3][3]*T11016[6][2]) + (ZIF*S1610[1][2] + YIF*S1610[1][3])*((-(YIF*Si1016[1][1]) + XIF*Si1016[2][1])*T11016[1][4] + Si1016[3][2]*T11016[5][4] + Si1016[3][3]*T11016[6][4]) + (ZIF*S1610[2][2] + YIF*S1610[2][3])*((-(YIF*Si1016[1][2]) + XIF*Si1016[2][2])*T11016[2][5] + Si1016[3][1]*T11016[4][5] + Si1016[3][3]*T11016[6][5]);
T1016[6][2]=-(XIF*S1610[3][3]*(XIF*Si1016[2][3]*T11016[3][6] + Si1016[3][1]*T11016[4][6] + Si1016[3][2]*T11016[5][6])) + S1610[1][2]*((-(YIF*Si1016[1][2]) + XIF*Si1016[2][2])*T11016[2][1] + XIF*Si1016[2][3]*T11016[3][1] + Si1016[3][1]*T11016[4][1] + Si1016[3][2]*T11016[5][1] + Si1016[3][3]*T11016[6][1]) + S1610[2][2]*((-(YIF*Si1016[1][1]) + XIF*Si1016[2][1])*T11016[1][2] + XIF*Si1016[2][3]*T11016[3][2] + Si1016[3][1]*T11016[4][2] + Si1016[3][2]*T11016[5][2] + Si1016[3][3]*T11016[6][2]) + S1610[3][2]*((-(YIF*Si1016[1][1]) + XIF*Si1016[2][1])*T11016[1][3] + (-(YIF*Si1016[1][2]) + XIF*Si1016[2][2])*T11016[2][3] + Si1016[3][1]*T11016[4][3] + Si1016[3][2]*T11016[5][3] + Si1016[3][3]*T11016[6][3]) + (-(ZIF*S1610[1][1]) - XIF*S1610[1][3])*((-(YIF*Si1016[1][1]) + XIF*Si1016[2][1])*T11016[1][4] + Si1016[3][2]*T11016[5][4] + Si1016[3][3]*T11016[6][4]) + (-(ZIF*S1610[2][1]) - XIF*S1610[2][3])*((-(YIF*Si1016[1][2]) + XIF*Si1016[2][2])*T11016[2][5] + Si1016[3][1]*T11016[4][5] + Si1016[3][3]*T11016[6][5]);
T1016[6][3]=XIF*S1610[3][2]*(XIF*Si1016[2][3]*T11016[3][6] + Si1016[3][1]*T11016[4][6] + Si1016[3][2]*T11016[5][6]) + S1610[1][3]*((-(YIF*Si1016[1][2]) + XIF*Si1016[2][2])*T11016[2][1] + XIF*Si1016[2][3]*T11016[3][1] + Si1016[3][1]*T11016[4][1] + Si1016[3][2]*T11016[5][1] + Si1016[3][3]*T11016[6][1]) + S1610[2][3]*((-(YIF*Si1016[1][1]) + XIF*Si1016[2][1])*T11016[1][2] + XIF*Si1016[2][3]*T11016[3][2] + Si1016[3][1]*T11016[4][2] + Si1016[3][2]*T11016[5][2] + Si1016[3][3]*T11016[6][2]) + S1610[3][3]*((-(YIF*Si1016[1][1]) + XIF*Si1016[2][1])*T11016[1][3] + (-(YIF*Si1016[1][2]) + XIF*Si1016[2][2])*T11016[2][3] + Si1016[3][1]*T11016[4][3] + Si1016[3][2]*T11016[5][3] + Si1016[3][3]*T11016[6][3]) + (-(YIF*S1610[1][1]) + XIF*S1610[1][2])*((-(YIF*Si1016[1][1]) + XIF*Si1016[2][1])*T11016[1][4] + Si1016[3][2]*T11016[5][4] + Si1016[3][3]*T11016[6][4]) + (-(YIF*S1610[2][1]) + XIF*S1610[2][2])*((-(YIF*Si1016[1][2]) + XIF*Si1016[2][2])*T11016[2][5] + Si1016[3][1]*T11016[4][5] + Si1016[3][3]*T11016[6][5]);
T1016[6][4]=S1610[1][1]*((-(YIF*Si1016[1][1]) + XIF*Si1016[2][1])*T11016[1][4] + Si1016[3][2]*T11016[5][4] + Si1016[3][3]*T11016[6][4]) + S1610[2][1]*((-(YIF*Si1016[1][2]) + XIF*Si1016[2][2])*T11016[2][5] + Si1016[3][1]*T11016[4][5] + Si1016[3][3]*T11016[6][5]);
T1016[6][5]=S1610[3][2]*(XIF*Si1016[2][3]*T11016[3][6] + Si1016[3][1]*T11016[4][6] + Si1016[3][2]*T11016[5][6]) + S1610[1][2]*((-(YIF*Si1016[1][1]) + XIF*Si1016[2][1])*T11016[1][4] + Si1016[3][2]*T11016[5][4] + Si1016[3][3]*T11016[6][4]) + S1610[2][2]*((-(YIF*Si1016[1][2]) + XIF*Si1016[2][2])*T11016[2][5] + Si1016[3][1]*T11016[4][5] + Si1016[3][3]*T11016[6][5]);
T1016[6][6]=S1610[3][3]*(XIF*Si1016[2][3]*T11016[3][6] + Si1016[3][1]*T11016[4][6] + Si1016[3][2]*T11016[5][6]) + S1610[1][3]*((-(YIF*Si1016[1][1]) + XIF*Si1016[2][1])*T11016[1][4] + Si1016[3][2]*T11016[5][4] + Si1016[3][3]*T11016[6][4]) + S1610[2][3]*((-(YIF*Si1016[1][2]) + XIF*Si1016[2][2])*T11016[2][5] + Si1016[3][1]*T11016[4][5] + Si1016[3][3]*T11016[6][5]);



}


void
hermes_InvDynArtfunc89(void)
      {




}


void
hermes_InvDynArtfunc90(void)
      {




}


void
hermes_InvDynArtfunc91(void)
      {
JA13[1][2]=0. + links[40].mcm[3];
JA13[1][3]=0. - links[40].mcm[2];
JA13[1][4]=0. + links[40].m;

JA13[2][1]=0. - links[40].mcm[3];
JA13[2][3]=0. + links[40].mcm[1];
JA13[2][5]=0. + links[40].m;

JA13[3][1]=0. + links[40].mcm[2];
JA13[3][2]=0. - links[40].mcm[1];
JA13[3][6]=0. + links[40].m;

JA13[4][1]=0. + links[40].inertia[1][1];
JA13[4][2]=0. + links[40].inertia[1][2];
JA13[4][3]=0. + links[40].inertia[1][3];
JA13[4][5]=0. - links[40].mcm[3];
JA13[4][6]=0. + links[40].mcm[2];

JA13[5][1]=0. + links[40].inertia[1][2];
JA13[5][2]=0. + links[40].inertia[2][2];
JA13[5][3]=0. + links[40].inertia[2][3];
JA13[5][4]=0. + links[40].mcm[3];
JA13[5][6]=0. - links[40].mcm[1];

JA13[6][1]=0. + links[40].inertia[1][3];
JA13[6][2]=0. + links[40].inertia[2][3];
JA13[6][3]=0. + links[40].inertia[3][3];
JA13[6][4]=0. - links[40].mcm[2];
JA13[6][5]=0. + links[40].mcm[1];


h13[1]=-JA13[1][3];
h13[2]=-JA13[2][3];
h13[4]=-JA13[4][3];
h13[5]=-JA13[5][3];
h13[6]=-JA13[6][3];

T11213[1][2]=JA13[1][2];
T11213[1][3]=JA13[1][3];
T11213[1][4]=JA13[1][4];

T11213[2][1]=JA13[2][1];
T11213[2][3]=JA13[2][3];
T11213[2][5]=JA13[2][5];

T11213[3][1]=JA13[3][1];
T11213[3][2]=JA13[3][2];
T11213[3][6]=JA13[3][6];

T11213[4][1]=JA13[4][1];
T11213[4][2]=JA13[4][2];
T11213[4][3]=JA13[4][3];
T11213[4][5]=JA13[4][5];
T11213[4][6]=JA13[4][6];

T11213[5][1]=JA13[5][1];
T11213[5][2]=JA13[5][2];
T11213[5][3]=JA13[5][3];
T11213[5][4]=JA13[5][4];
T11213[5][6]=JA13[5][6];

T11213[6][1]=JA13[6][1];
T11213[6][2]=JA13[6][2];
T11213[6][3]=JA13[6][3];
T11213[6][4]=JA13[6][4];
T11213[6][5]=JA13[6][5];


T1213[1][1]=S1312[2][1]*Si1213[1][1]*T11213[1][2] + S1312[1][1]*Si1213[1][2]*T11213[2][1];
T1213[1][2]=S1312[2][2]*Si1213[1][1]*T11213[1][2] + S1312[1][2]*Si1213[1][2]*T11213[2][1];
T1213[1][3]=Si1213[1][1]*T11213[1][3] + (-(YTHUMBFLEX*S1312[1][1]) + XTHUMBFLEX*S1312[1][2])*Si1213[1][1]*T11213[1][4] + Si1213[1][2]*T11213[2][3] + (-(YTHUMBFLEX*S1312[2][1]) + XTHUMBFLEX*S1312[2][2])*Si1213[1][2]*T11213[2][5];
T1213[1][4]=S1312[1][1]*Si1213[1][1]*T11213[1][4] + S1312[2][1]*Si1213[1][2]*T11213[2][5];
T1213[1][5]=S1312[1][2]*Si1213[1][1]*T11213[1][4] + S1312[2][2]*Si1213[1][2]*T11213[2][5];

T1213[2][1]=S1312[2][1]*Si1213[2][1]*T11213[1][2] + S1312[1][1]*Si1213[2][2]*T11213[2][1];
T1213[2][2]=S1312[2][2]*Si1213[2][1]*T11213[1][2] + S1312[1][2]*Si1213[2][2]*T11213[2][1];
T1213[2][3]=Si1213[2][1]*T11213[1][3] + (-(YTHUMBFLEX*S1312[1][1]) + XTHUMBFLEX*S1312[1][2])*Si1213[2][1]*T11213[1][4] + Si1213[2][2]*T11213[2][3] + (-(YTHUMBFLEX*S1312[2][1]) + XTHUMBFLEX*S1312[2][2])*Si1213[2][2]*T11213[2][5];
T1213[2][4]=S1312[1][1]*Si1213[2][1]*T11213[1][4] + S1312[2][1]*Si1213[2][2]*T11213[2][5];
T1213[2][5]=S1312[1][2]*Si1213[2][1]*T11213[1][4] + S1312[2][2]*Si1213[2][2]*T11213[2][5];

T1213[3][1]=S1312[1][1]*T11213[3][1] + S1312[2][1]*T11213[3][2] + YTHUMBFLEX*T11213[3][6];
T1213[3][2]=S1312[1][2]*T11213[3][1] + S1312[2][2]*T11213[3][2] - XTHUMBFLEX*T11213[3][6];
T1213[3][6]=T11213[3][6];

T1213[4][1]=S1312[1][1]*(YTHUMBFLEX*T11213[3][1] + Si1213[1][1]*T11213[4][1] + Si1213[1][2]*T11213[5][1]) + S1312[2][1]*(YTHUMBFLEX*T11213[3][2] + Si1213[1][1]*T11213[4][2] + Si1213[1][2]*T11213[5][2]) + YTHUMBFLEX*(YTHUMBFLEX*T11213[3][6] + Si1213[1][1]*T11213[4][6] + Si1213[1][2]*T11213[5][6]);
T1213[4][2]=S1312[1][2]*(YTHUMBFLEX*T11213[3][1] + Si1213[1][1]*T11213[4][1] + Si1213[1][2]*T11213[5][1]) + S1312[2][2]*(YTHUMBFLEX*T11213[3][2] + Si1213[1][1]*T11213[4][2] + Si1213[1][2]*T11213[5][2]) - XTHUMBFLEX*(YTHUMBFLEX*T11213[3][6] + Si1213[1][1]*T11213[4][6] + Si1213[1][2]*T11213[5][6]);
T1213[4][3]=Si1213[1][1]*T11213[4][3] + (-(YTHUMBFLEX*S1312[2][1]) + XTHUMBFLEX*S1312[2][2])*Si1213[1][1]*T11213[4][5] + Si1213[1][2]*T11213[5][3] + (-(YTHUMBFLEX*S1312[1][1]) + XTHUMBFLEX*S1312[1][2])*Si1213[1][2]*T11213[5][4];
T1213[4][4]=S1312[2][1]*Si1213[1][1]*T11213[4][5] + S1312[1][1]*Si1213[1][2]*T11213[5][4];
T1213[4][5]=S1312[2][2]*Si1213[1][1]*T11213[4][5] + S1312[1][2]*Si1213[1][2]*T11213[5][4];
T1213[4][6]=YTHUMBFLEX*T11213[3][6] + Si1213[1][1]*T11213[4][6] + Si1213[1][2]*T11213[5][6];

T1213[5][1]=S1312[1][1]*(-(XTHUMBFLEX*T11213[3][1]) + Si1213[2][1]*T11213[4][1] + Si1213[2][2]*T11213[5][1]) + S1312[2][1]*(-(XTHUMBFLEX*T11213[3][2]) + Si1213[2][1]*T11213[4][2] + Si1213[2][2]*T11213[5][2]) + YTHUMBFLEX*(-(XTHUMBFLEX*T11213[3][6]) + Si1213[2][1]*T11213[4][6] + Si1213[2][2]*T11213[5][6]);
T1213[5][2]=S1312[1][2]*(-(XTHUMBFLEX*T11213[3][1]) + Si1213[2][1]*T11213[4][1] + Si1213[2][2]*T11213[5][1]) + S1312[2][2]*(-(XTHUMBFLEX*T11213[3][2]) + Si1213[2][1]*T11213[4][2] + Si1213[2][2]*T11213[5][2]) - XTHUMBFLEX*(-(XTHUMBFLEX*T11213[3][6]) + Si1213[2][1]*T11213[4][6] + Si1213[2][2]*T11213[5][6]);
T1213[5][3]=Si1213[2][1]*T11213[4][3] + (-(YTHUMBFLEX*S1312[2][1]) + XTHUMBFLEX*S1312[2][2])*Si1213[2][1]*T11213[4][5] + Si1213[2][2]*T11213[5][3] + (-(YTHUMBFLEX*S1312[1][1]) + XTHUMBFLEX*S1312[1][2])*Si1213[2][2]*T11213[5][4];
T1213[5][4]=S1312[2][1]*Si1213[2][1]*T11213[4][5] + S1312[1][1]*Si1213[2][2]*T11213[5][4];
T1213[5][5]=S1312[2][2]*Si1213[2][1]*T11213[4][5] + S1312[1][2]*Si1213[2][2]*T11213[5][4];
T1213[5][6]=-(XTHUMBFLEX*T11213[3][6]) + Si1213[2][1]*T11213[4][6] + Si1213[2][2]*T11213[5][6];

T1213[6][1]=S1312[1][1]*((-(YTHUMBFLEX*Si1213[1][2]) + XTHUMBFLEX*Si1213[2][2])*T11213[2][1] + T11213[6][1]) + S1312[2][1]*((-(YTHUMBFLEX*Si1213[1][1]) + XTHUMBFLEX*Si1213[2][1])*T11213[1][2] + T11213[6][2]);
T1213[6][2]=S1312[1][2]*((-(YTHUMBFLEX*Si1213[1][2]) + XTHUMBFLEX*Si1213[2][2])*T11213[2][1] + T11213[6][1]) + S1312[2][2]*((-(YTHUMBFLEX*Si1213[1][1]) + XTHUMBFLEX*Si1213[2][1])*T11213[1][2] + T11213[6][2]);
T1213[6][3]=(-(YTHUMBFLEX*Si1213[1][1]) + XTHUMBFLEX*Si1213[2][1])*T11213[1][3] + (-(YTHUMBFLEX*Si1213[1][2]) + XTHUMBFLEX*Si1213[2][2])*T11213[2][3] + T11213[6][3] + (-(YTHUMBFLEX*S1312[1][1]) + XTHUMBFLEX*S1312[1][2])*((-(YTHUMBFLEX*Si1213[1][1]) + XTHUMBFLEX*Si1213[2][1])*T11213[1][4] + T11213[6][4]) + (-(YTHUMBFLEX*S1312[2][1]) + XTHUMBFLEX*S1312[2][2])*((-(YTHUMBFLEX*Si1213[1][2]) + XTHUMBFLEX*Si1213[2][2])*T11213[2][5] + T11213[6][5]);
T1213[6][4]=S1312[1][1]*((-(YTHUMBFLEX*Si1213[1][1]) + XTHUMBFLEX*Si1213[2][1])*T11213[1][4] + T11213[6][4]) + S1312[2][1]*((-(YTHUMBFLEX*Si1213[1][2]) + XTHUMBFLEX*Si1213[2][2])*T11213[2][5] + T11213[6][5]);
T1213[6][5]=S1312[1][2]*((-(YTHUMBFLEX*Si1213[1][1]) + XTHUMBFLEX*Si1213[2][1])*T11213[1][4] + T11213[6][4]) + S1312[2][2]*((-(YTHUMBFLEX*Si1213[1][2]) + XTHUMBFLEX*Si1213[2][2])*T11213[2][5] + T11213[6][5]);



}


void
hermes_InvDynArtfunc92(void)
      {
JA12[1][1]=T1213[1][1];
JA12[1][2]=links[39].mcm[3] + T1213[1][2];
JA12[1][3]=-links[39].mcm[2] + T1213[1][3];
JA12[1][4]=links[39].m + T1213[1][4];
JA12[1][5]=T1213[1][5];

JA12[2][1]=-links[39].mcm[3] + T1213[2][1];
JA12[2][2]=T1213[2][2];
JA12[2][3]=links[39].mcm[1] + T1213[2][3];
JA12[2][4]=T1213[2][4];
JA12[2][5]=links[39].m + T1213[2][5];

JA12[3][1]=links[39].mcm[2] + T1213[3][1];
JA12[3][2]=-links[39].mcm[1] + T1213[3][2];
JA12[3][6]=links[39].m + T1213[3][6];

JA12[4][1]=links[39].inertia[1][1] + T1213[4][1];
JA12[4][2]=links[39].inertia[1][2] + T1213[4][2];
JA12[4][3]=links[39].inertia[1][3] + T1213[4][3];
JA12[4][4]=T1213[4][4];
JA12[4][5]=-links[39].mcm[3] + T1213[4][5];
JA12[4][6]=links[39].mcm[2] + T1213[4][6];

JA12[5][1]=links[39].inertia[1][2] + T1213[5][1];
JA12[5][2]=links[39].inertia[2][2] + T1213[5][2];
JA12[5][3]=links[39].inertia[2][3] + T1213[5][3];
JA12[5][4]=links[39].mcm[3] + T1213[5][4];
JA12[5][5]=T1213[5][5];
JA12[5][6]=-links[39].mcm[1] + T1213[5][6];

JA12[6][1]=links[39].inertia[1][3] + T1213[6][1];
JA12[6][2]=links[39].inertia[2][3] + T1213[6][2];
JA12[6][3]=links[39].inertia[3][3] + T1213[6][3];
JA12[6][4]=-links[39].mcm[2] + T1213[6][4];
JA12[6][5]=links[39].mcm[1] + T1213[6][5];


h12[1]=JA12[1][1];
h12[2]=JA12[2][1];
h12[3]=JA12[3][1];
h12[4]=JA12[4][1];
h12[5]=JA12[5][1];
h12[6]=JA12[6][1];

T11012[1][1]=JA12[1][1];
T11012[1][2]=JA12[1][2];
T11012[1][3]=JA12[1][3];
T11012[1][4]=JA12[1][4];
T11012[1][5]=JA12[1][5];

T11012[2][1]=JA12[2][1];
T11012[2][2]=JA12[2][2];
T11012[2][3]=JA12[2][3];
T11012[2][4]=JA12[2][4];
T11012[2][5]=JA12[2][5];

T11012[3][1]=JA12[3][1];
T11012[3][2]=JA12[3][2];
T11012[3][6]=JA12[3][6];

T11012[4][1]=JA12[4][1];
T11012[4][2]=JA12[4][2];
T11012[4][3]=JA12[4][3];
T11012[4][4]=JA12[4][4];
T11012[4][5]=JA12[4][5];
T11012[4][6]=JA12[4][6];

T11012[5][1]=JA12[5][1];
T11012[5][2]=JA12[5][2];
T11012[5][3]=JA12[5][3];
T11012[5][4]=JA12[5][4];
T11012[5][5]=JA12[5][5];
T11012[5][6]=JA12[5][6];

T11012[6][1]=JA12[6][1];
T11012[6][2]=JA12[6][2];
T11012[6][3]=JA12[6][3];
T11012[6][4]=JA12[6][4];
T11012[6][5]=JA12[6][5];


T1012[1][1]=S1210[3][1]*(Si1012[1][1]*T11012[1][3] + Si1012[1][2]*T11012[2][3]) + (ZTHUMB*S1210[1][2] + YTHUMB*S1210[1][3])*(Si1012[1][1]*T11012[1][4] + Si1012[1][2]*T11012[2][4]) + (ZTHUMB*S1210[2][2] + YTHUMB*S1210[2][3])*(Si1012[1][1]*T11012[1][5] + Si1012[1][2]*T11012[2][5]) + S1210[1][1]*(Si1012[1][1]*T11012[1][1] + Si1012[1][2]*T11012[2][1] + Si1012[1][3]*T11012[3][1]) + S1210[2][1]*(Si1012[1][1]*T11012[1][2] + Si1012[1][2]*T11012[2][2] + Si1012[1][3]*T11012[3][2]) + (ZTHUMB*S1210[3][2] + YTHUMB*S1210[3][3])*Si1012[1][3]*T11012[3][6];
T1012[1][2]=S1210[3][2]*(Si1012[1][1]*T11012[1][3] + Si1012[1][2]*T11012[2][3]) + (-(ZTHUMB*S1210[1][1]) - XTHUMB*S1210[1][3])*(Si1012[1][1]*T11012[1][4] + Si1012[1][2]*T11012[2][4]) + (-(ZTHUMB*S1210[2][1]) - XTHUMB*S1210[2][3])*(Si1012[1][1]*T11012[1][5] + Si1012[1][2]*T11012[2][5]) + S1210[1][2]*(Si1012[1][1]*T11012[1][1] + Si1012[1][2]*T11012[2][1] + Si1012[1][3]*T11012[3][1]) + S1210[2][2]*(Si1012[1][1]*T11012[1][2] + Si1012[1][2]*T11012[2][2] + Si1012[1][3]*T11012[3][2]) + (-(ZTHUMB*S1210[3][1]) - XTHUMB*S1210[3][3])*Si1012[1][3]*T11012[3][6];
T1012[1][3]=S1210[3][3]*(Si1012[1][1]*T11012[1][3] + Si1012[1][2]*T11012[2][3]) + (-(YTHUMB*S1210[1][1]) + XTHUMB*S1210[1][2])*(Si1012[1][1]*T11012[1][4] + Si1012[1][2]*T11012[2][4]) + (-(YTHUMB*S1210[2][1]) + XTHUMB*S1210[2][2])*(Si1012[1][1]*T11012[1][5] + Si1012[1][2]*T11012[2][5]) + S1210[1][3]*(Si1012[1][1]*T11012[1][1] + Si1012[1][2]*T11012[2][1] + Si1012[1][3]*T11012[3][1]) + S1210[2][3]*(Si1012[1][1]*T11012[1][2] + Si1012[1][2]*T11012[2][2] + Si1012[1][3]*T11012[3][2]) + (-(YTHUMB*S1210[3][1]) + XTHUMB*S1210[3][2])*Si1012[1][3]*T11012[3][6];
T1012[1][4]=S1210[1][1]*(Si1012[1][1]*T11012[1][4] + Si1012[1][2]*T11012[2][4]) + S1210[2][1]*(Si1012[1][1]*T11012[1][5] + Si1012[1][2]*T11012[2][5]) + S1210[3][1]*Si1012[1][3]*T11012[3][6];
T1012[1][5]=S1210[1][2]*(Si1012[1][1]*T11012[1][4] + Si1012[1][2]*T11012[2][4]) + S1210[2][2]*(Si1012[1][1]*T11012[1][5] + Si1012[1][2]*T11012[2][5]) + S1210[3][2]*Si1012[1][3]*T11012[3][6];
T1012[1][6]=S1210[1][3]*(Si1012[1][1]*T11012[1][4] + Si1012[1][2]*T11012[2][4]) + S1210[2][3]*(Si1012[1][1]*T11012[1][5] + Si1012[1][2]*T11012[2][5]) + S1210[3][3]*Si1012[1][3]*T11012[3][6];

T1012[2][1]=S1210[3][1]*(Si1012[2][1]*T11012[1][3] + Si1012[2][2]*T11012[2][3]) + (ZTHUMB*S1210[1][2] + YTHUMB*S1210[1][3])*(Si1012[2][1]*T11012[1][4] + Si1012[2][2]*T11012[2][4]) + (ZTHUMB*S1210[2][2] + YTHUMB*S1210[2][3])*(Si1012[2][1]*T11012[1][5] + Si1012[2][2]*T11012[2][5]) + S1210[1][1]*(Si1012[2][1]*T11012[1][1] + Si1012[2][2]*T11012[2][1] + Si1012[2][3]*T11012[3][1]) + S1210[2][1]*(Si1012[2][1]*T11012[1][2] + Si1012[2][2]*T11012[2][2] + Si1012[2][3]*T11012[3][2]) + (ZTHUMB*S1210[3][2] + YTHUMB*S1210[3][3])*Si1012[2][3]*T11012[3][6];
T1012[2][2]=S1210[3][2]*(Si1012[2][1]*T11012[1][3] + Si1012[2][2]*T11012[2][3]) + (-(ZTHUMB*S1210[1][1]) - XTHUMB*S1210[1][3])*(Si1012[2][1]*T11012[1][4] + Si1012[2][2]*T11012[2][4]) + (-(ZTHUMB*S1210[2][1]) - XTHUMB*S1210[2][3])*(Si1012[2][1]*T11012[1][5] + Si1012[2][2]*T11012[2][5]) + S1210[1][2]*(Si1012[2][1]*T11012[1][1] + Si1012[2][2]*T11012[2][1] + Si1012[2][3]*T11012[3][1]) + S1210[2][2]*(Si1012[2][1]*T11012[1][2] + Si1012[2][2]*T11012[2][2] + Si1012[2][3]*T11012[3][2]) + (-(ZTHUMB*S1210[3][1]) - XTHUMB*S1210[3][3])*Si1012[2][3]*T11012[3][6];
T1012[2][3]=S1210[3][3]*(Si1012[2][1]*T11012[1][3] + Si1012[2][2]*T11012[2][3]) + (-(YTHUMB*S1210[1][1]) + XTHUMB*S1210[1][2])*(Si1012[2][1]*T11012[1][4] + Si1012[2][2]*T11012[2][4]) + (-(YTHUMB*S1210[2][1]) + XTHUMB*S1210[2][2])*(Si1012[2][1]*T11012[1][5] + Si1012[2][2]*T11012[2][5]) + S1210[1][3]*(Si1012[2][1]*T11012[1][1] + Si1012[2][2]*T11012[2][1] + Si1012[2][3]*T11012[3][1]) + S1210[2][3]*(Si1012[2][1]*T11012[1][2] + Si1012[2][2]*T11012[2][2] + Si1012[2][3]*T11012[3][2]) + (-(YTHUMB*S1210[3][1]) + XTHUMB*S1210[3][2])*Si1012[2][3]*T11012[3][6];
T1012[2][4]=S1210[1][1]*(Si1012[2][1]*T11012[1][4] + Si1012[2][2]*T11012[2][4]) + S1210[2][1]*(Si1012[2][1]*T11012[1][5] + Si1012[2][2]*T11012[2][5]) + S1210[3][1]*Si1012[2][3]*T11012[3][6];
T1012[2][5]=S1210[1][2]*(Si1012[2][1]*T11012[1][4] + Si1012[2][2]*T11012[2][4]) + S1210[2][2]*(Si1012[2][1]*T11012[1][5] + Si1012[2][2]*T11012[2][5]) + S1210[3][2]*Si1012[2][3]*T11012[3][6];
T1012[2][6]=S1210[1][3]*(Si1012[2][1]*T11012[1][4] + Si1012[2][2]*T11012[2][4]) + S1210[2][3]*(Si1012[2][1]*T11012[1][5] + Si1012[2][2]*T11012[2][5]) + S1210[3][3]*Si1012[2][3]*T11012[3][6];

T1012[3][1]=S1210[3][1]*(Si1012[3][1]*T11012[1][3] + Si1012[3][2]*T11012[2][3]) + (ZTHUMB*S1210[1][2] + YTHUMB*S1210[1][3])*(Si1012[3][1]*T11012[1][4] + Si1012[3][2]*T11012[2][4]) + (ZTHUMB*S1210[2][2] + YTHUMB*S1210[2][3])*(Si1012[3][1]*T11012[1][5] + Si1012[3][2]*T11012[2][5]) + S1210[1][1]*(Si1012[3][1]*T11012[1][1] + Si1012[3][2]*T11012[2][1] + Si1012[3][3]*T11012[3][1]) + S1210[2][1]*(Si1012[3][1]*T11012[1][2] + Si1012[3][2]*T11012[2][2] + Si1012[3][3]*T11012[3][2]) + (ZTHUMB*S1210[3][2] + YTHUMB*S1210[3][3])*Si1012[3][3]*T11012[3][6];
T1012[3][2]=S1210[3][2]*(Si1012[3][1]*T11012[1][3] + Si1012[3][2]*T11012[2][3]) + (-(ZTHUMB*S1210[1][1]) - XTHUMB*S1210[1][3])*(Si1012[3][1]*T11012[1][4] + Si1012[3][2]*T11012[2][4]) + (-(ZTHUMB*S1210[2][1]) - XTHUMB*S1210[2][3])*(Si1012[3][1]*T11012[1][5] + Si1012[3][2]*T11012[2][5]) + S1210[1][2]*(Si1012[3][1]*T11012[1][1] + Si1012[3][2]*T11012[2][1] + Si1012[3][3]*T11012[3][1]) + S1210[2][2]*(Si1012[3][1]*T11012[1][2] + Si1012[3][2]*T11012[2][2] + Si1012[3][3]*T11012[3][2]) + (-(ZTHUMB*S1210[3][1]) - XTHUMB*S1210[3][3])*Si1012[3][3]*T11012[3][6];
T1012[3][3]=S1210[3][3]*(Si1012[3][1]*T11012[1][3] + Si1012[3][2]*T11012[2][3]) + (-(YTHUMB*S1210[1][1]) + XTHUMB*S1210[1][2])*(Si1012[3][1]*T11012[1][4] + Si1012[3][2]*T11012[2][4]) + (-(YTHUMB*S1210[2][1]) + XTHUMB*S1210[2][2])*(Si1012[3][1]*T11012[1][5] + Si1012[3][2]*T11012[2][5]) + S1210[1][3]*(Si1012[3][1]*T11012[1][1] + Si1012[3][2]*T11012[2][1] + Si1012[3][3]*T11012[3][1]) + S1210[2][3]*(Si1012[3][1]*T11012[1][2] + Si1012[3][2]*T11012[2][2] + Si1012[3][3]*T11012[3][2]) + (-(YTHUMB*S1210[3][1]) + XTHUMB*S1210[3][2])*Si1012[3][3]*T11012[3][6];
T1012[3][4]=S1210[1][1]*(Si1012[3][1]*T11012[1][4] + Si1012[3][2]*T11012[2][4]) + S1210[2][1]*(Si1012[3][1]*T11012[1][5] + Si1012[3][2]*T11012[2][5]) + S1210[3][1]*Si1012[3][3]*T11012[3][6];
T1012[3][5]=S1210[1][2]*(Si1012[3][1]*T11012[1][4] + Si1012[3][2]*T11012[2][4]) + S1210[2][2]*(Si1012[3][1]*T11012[1][5] + Si1012[3][2]*T11012[2][5]) + S1210[3][2]*Si1012[3][3]*T11012[3][6];
T1012[3][6]=S1210[1][3]*(Si1012[3][1]*T11012[1][4] + Si1012[3][2]*T11012[2][4]) + S1210[2][3]*(Si1012[3][1]*T11012[1][5] + Si1012[3][2]*T11012[2][5]) + S1210[3][3]*Si1012[3][3]*T11012[3][6];

T1012[4][1]=(ZTHUMB*S1210[3][2] + YTHUMB*S1210[3][3])*((ZTHUMB*Si1012[2][3] + YTHUMB*Si1012[3][3])*T11012[3][6] + Si1012[1][1]*T11012[4][6] + Si1012[1][2]*T11012[5][6]) + S1210[1][1]*((ZTHUMB*Si1012[2][1] + YTHUMB*Si1012[3][1])*T11012[1][1] + (ZTHUMB*Si1012[2][2] + YTHUMB*Si1012[3][2])*T11012[2][1] + (ZTHUMB*Si1012[2][3] + YTHUMB*Si1012[3][3])*T11012[3][1] + Si1012[1][1]*T11012[4][1] + Si1012[1][2]*T11012[5][1] + Si1012[1][3]*T11012[6][1]) + S1210[2][1]*((ZTHUMB*Si1012[2][1] + YTHUMB*Si1012[3][1])*T11012[1][2] + (ZTHUMB*Si1012[2][2] + YTHUMB*Si1012[3][2])*T11012[2][2] + (ZTHUMB*Si1012[2][3] + YTHUMB*Si1012[3][3])*T11012[3][2] + Si1012[1][1]*T11012[4][2] + Si1012[1][2]*T11012[5][2] + Si1012[1][3]*T11012[6][2]) + S1210[3][1]*((ZTHUMB*Si1012[2][1] + YTHUMB*Si1012[3][1])*T11012[1][3] + (ZTHUMB*Si1012[2][2] + YTHUMB*Si1012[3][2])*T11012[2][3] + Si1012[1][1]*T11012[4][3] + Si1012[1][2]*T11012[5][3] + Si1012[1][3]*T11012[6][3]) + (ZTHUMB*S1210[1][2] + YTHUMB*S1210[1][3])*((ZTHUMB*Si1012[2][1] + YTHUMB*Si1012[3][1])*T11012[1][4] + (ZTHUMB*Si1012[2][2] + YTHUMB*Si1012[3][2])*T11012[2][4] + Si1012[1][1]*T11012[4][4] + Si1012[1][2]*T11012[5][4] + Si1012[1][3]*T11012[6][4]) + (ZTHUMB*S1210[2][2] + YTHUMB*S1210[2][3])*((ZTHUMB*Si1012[2][1] + YTHUMB*Si1012[3][1])*T11012[1][5] + (ZTHUMB*Si1012[2][2] + YTHUMB*Si1012[3][2])*T11012[2][5] + Si1012[1][1]*T11012[4][5] + Si1012[1][2]*T11012[5][5] + Si1012[1][3]*T11012[6][5]);
T1012[4][2]=(-(ZTHUMB*S1210[3][1]) - XTHUMB*S1210[3][3])*((ZTHUMB*Si1012[2][3] + YTHUMB*Si1012[3][3])*T11012[3][6] + Si1012[1][1]*T11012[4][6] + Si1012[1][2]*T11012[5][6]) + S1210[1][2]*((ZTHUMB*Si1012[2][1] + YTHUMB*Si1012[3][1])*T11012[1][1] + (ZTHUMB*Si1012[2][2] + YTHUMB*Si1012[3][2])*T11012[2][1] + (ZTHUMB*Si1012[2][3] + YTHUMB*Si1012[3][3])*T11012[3][1] + Si1012[1][1]*T11012[4][1] + Si1012[1][2]*T11012[5][1] + Si1012[1][3]*T11012[6][1]) + S1210[2][2]*((ZTHUMB*Si1012[2][1] + YTHUMB*Si1012[3][1])*T11012[1][2] + (ZTHUMB*Si1012[2][2] + YTHUMB*Si1012[3][2])*T11012[2][2] + (ZTHUMB*Si1012[2][3] + YTHUMB*Si1012[3][3])*T11012[3][2] + Si1012[1][1]*T11012[4][2] + Si1012[1][2]*T11012[5][2] + Si1012[1][3]*T11012[6][2]) + S1210[3][2]*((ZTHUMB*Si1012[2][1] + YTHUMB*Si1012[3][1])*T11012[1][3] + (ZTHUMB*Si1012[2][2] + YTHUMB*Si1012[3][2])*T11012[2][3] + Si1012[1][1]*T11012[4][3] + Si1012[1][2]*T11012[5][3] + Si1012[1][3]*T11012[6][3]) + (-(ZTHUMB*S1210[1][1]) - XTHUMB*S1210[1][3])*((ZTHUMB*Si1012[2][1] + YTHUMB*Si1012[3][1])*T11012[1][4] + (ZTHUMB*Si1012[2][2] + YTHUMB*Si1012[3][2])*T11012[2][4] + Si1012[1][1]*T11012[4][4] + Si1012[1][2]*T11012[5][4] + Si1012[1][3]*T11012[6][4]) + (-(ZTHUMB*S1210[2][1]) - XTHUMB*S1210[2][3])*((ZTHUMB*Si1012[2][1] + YTHUMB*Si1012[3][1])*T11012[1][5] + (ZTHUMB*Si1012[2][2] + YTHUMB*Si1012[3][2])*T11012[2][5] + Si1012[1][1]*T11012[4][5] + Si1012[1][2]*T11012[5][5] + Si1012[1][3]*T11012[6][5]);
T1012[4][3]=(-(YTHUMB*S1210[3][1]) + XTHUMB*S1210[3][2])*((ZTHUMB*Si1012[2][3] + YTHUMB*Si1012[3][3])*T11012[3][6] + Si1012[1][1]*T11012[4][6] + Si1012[1][2]*T11012[5][6]) + S1210[1][3]*((ZTHUMB*Si1012[2][1] + YTHUMB*Si1012[3][1])*T11012[1][1] + (ZTHUMB*Si1012[2][2] + YTHUMB*Si1012[3][2])*T11012[2][1] + (ZTHUMB*Si1012[2][3] + YTHUMB*Si1012[3][3])*T11012[3][1] + Si1012[1][1]*T11012[4][1] + Si1012[1][2]*T11012[5][1] + Si1012[1][3]*T11012[6][1]) + S1210[2][3]*((ZTHUMB*Si1012[2][1] + YTHUMB*Si1012[3][1])*T11012[1][2] + (ZTHUMB*Si1012[2][2] + YTHUMB*Si1012[3][2])*T11012[2][2] + (ZTHUMB*Si1012[2][3] + YTHUMB*Si1012[3][3])*T11012[3][2] + Si1012[1][1]*T11012[4][2] + Si1012[1][2]*T11012[5][2] + Si1012[1][3]*T11012[6][2]) + S1210[3][3]*((ZTHUMB*Si1012[2][1] + YTHUMB*Si1012[3][1])*T11012[1][3] + (ZTHUMB*Si1012[2][2] + YTHUMB*Si1012[3][2])*T11012[2][3] + Si1012[1][1]*T11012[4][3] + Si1012[1][2]*T11012[5][3] + Si1012[1][3]*T11012[6][3]) + (-(YTHUMB*S1210[1][1]) + XTHUMB*S1210[1][2])*((ZTHUMB*Si1012[2][1] + YTHUMB*Si1012[3][1])*T11012[1][4] + (ZTHUMB*Si1012[2][2] + YTHUMB*Si1012[3][2])*T11012[2][4] + Si1012[1][1]*T11012[4][4] + Si1012[1][2]*T11012[5][4] + Si1012[1][3]*T11012[6][4]) + (-(YTHUMB*S1210[2][1]) + XTHUMB*S1210[2][2])*((ZTHUMB*Si1012[2][1] + YTHUMB*Si1012[3][1])*T11012[1][5] + (ZTHUMB*Si1012[2][2] + YTHUMB*Si1012[3][2])*T11012[2][5] + Si1012[1][1]*T11012[4][5] + Si1012[1][2]*T11012[5][5] + Si1012[1][3]*T11012[6][5]);
T1012[4][4]=S1210[3][1]*((ZTHUMB*Si1012[2][3] + YTHUMB*Si1012[3][3])*T11012[3][6] + Si1012[1][1]*T11012[4][6] + Si1012[1][2]*T11012[5][6]) + S1210[1][1]*((ZTHUMB*Si1012[2][1] + YTHUMB*Si1012[3][1])*T11012[1][4] + (ZTHUMB*Si1012[2][2] + YTHUMB*Si1012[3][2])*T11012[2][4] + Si1012[1][1]*T11012[4][4] + Si1012[1][2]*T11012[5][4] + Si1012[1][3]*T11012[6][4]) + S1210[2][1]*((ZTHUMB*Si1012[2][1] + YTHUMB*Si1012[3][1])*T11012[1][5] + (ZTHUMB*Si1012[2][2] + YTHUMB*Si1012[3][2])*T11012[2][5] + Si1012[1][1]*T11012[4][5] + Si1012[1][2]*T11012[5][5] + Si1012[1][3]*T11012[6][5]);
T1012[4][5]=S1210[3][2]*((ZTHUMB*Si1012[2][3] + YTHUMB*Si1012[3][3])*T11012[3][6] + Si1012[1][1]*T11012[4][6] + Si1012[1][2]*T11012[5][6]) + S1210[1][2]*((ZTHUMB*Si1012[2][1] + YTHUMB*Si1012[3][1])*T11012[1][4] + (ZTHUMB*Si1012[2][2] + YTHUMB*Si1012[3][2])*T11012[2][4] + Si1012[1][1]*T11012[4][4] + Si1012[1][2]*T11012[5][4] + Si1012[1][3]*T11012[6][4]) + S1210[2][2]*((ZTHUMB*Si1012[2][1] + YTHUMB*Si1012[3][1])*T11012[1][5] + (ZTHUMB*Si1012[2][2] + YTHUMB*Si1012[3][2])*T11012[2][5] + Si1012[1][1]*T11012[4][5] + Si1012[1][2]*T11012[5][5] + Si1012[1][3]*T11012[6][5]);
T1012[4][6]=S1210[3][3]*((ZTHUMB*Si1012[2][3] + YTHUMB*Si1012[3][3])*T11012[3][6] + Si1012[1][1]*T11012[4][6] + Si1012[1][2]*T11012[5][6]) + S1210[1][3]*((ZTHUMB*Si1012[2][1] + YTHUMB*Si1012[3][1])*T11012[1][4] + (ZTHUMB*Si1012[2][2] + YTHUMB*Si1012[3][2])*T11012[2][4] + Si1012[1][1]*T11012[4][4] + Si1012[1][2]*T11012[5][4] + Si1012[1][3]*T11012[6][4]) + S1210[2][3]*((ZTHUMB*Si1012[2][1] + YTHUMB*Si1012[3][1])*T11012[1][5] + (ZTHUMB*Si1012[2][2] + YTHUMB*Si1012[3][2])*T11012[2][5] + Si1012[1][1]*T11012[4][5] + Si1012[1][2]*T11012[5][5] + Si1012[1][3]*T11012[6][5]);

T1012[5][1]=(ZTHUMB*S1210[3][2] + YTHUMB*S1210[3][3])*((-(ZTHUMB*Si1012[1][3]) - XTHUMB*Si1012[3][3])*T11012[3][6] + Si1012[2][1]*T11012[4][6] + Si1012[2][2]*T11012[5][6]) + S1210[1][1]*((-(ZTHUMB*Si1012[1][1]) - XTHUMB*Si1012[3][1])*T11012[1][1] + (-(ZTHUMB*Si1012[1][2]) - XTHUMB*Si1012[3][2])*T11012[2][1] + (-(ZTHUMB*Si1012[1][3]) - XTHUMB*Si1012[3][3])*T11012[3][1] + Si1012[2][1]*T11012[4][1] + Si1012[2][2]*T11012[5][1] + Si1012[2][3]*T11012[6][1]) + S1210[2][1]*((-(ZTHUMB*Si1012[1][1]) - XTHUMB*Si1012[3][1])*T11012[1][2] + (-(ZTHUMB*Si1012[1][2]) - XTHUMB*Si1012[3][2])*T11012[2][2] + (-(ZTHUMB*Si1012[1][3]) - XTHUMB*Si1012[3][3])*T11012[3][2] + Si1012[2][1]*T11012[4][2] + Si1012[2][2]*T11012[5][2] + Si1012[2][3]*T11012[6][2]) + S1210[3][1]*((-(ZTHUMB*Si1012[1][1]) - XTHUMB*Si1012[3][1])*T11012[1][3] + (-(ZTHUMB*Si1012[1][2]) - XTHUMB*Si1012[3][2])*T11012[2][3] + Si1012[2][1]*T11012[4][3] + Si1012[2][2]*T11012[5][3] + Si1012[2][3]*T11012[6][3]) + (ZTHUMB*S1210[1][2] + YTHUMB*S1210[1][3])*((-(ZTHUMB*Si1012[1][1]) - XTHUMB*Si1012[3][1])*T11012[1][4] + (-(ZTHUMB*Si1012[1][2]) - XTHUMB*Si1012[3][2])*T11012[2][4] + Si1012[2][1]*T11012[4][4] + Si1012[2][2]*T11012[5][4] + Si1012[2][3]*T11012[6][4]) + (ZTHUMB*S1210[2][2] + YTHUMB*S1210[2][3])*((-(ZTHUMB*Si1012[1][1]) - XTHUMB*Si1012[3][1])*T11012[1][5] + (-(ZTHUMB*Si1012[1][2]) - XTHUMB*Si1012[3][2])*T11012[2][5] + Si1012[2][1]*T11012[4][5] + Si1012[2][2]*T11012[5][5] + Si1012[2][3]*T11012[6][5]);
T1012[5][2]=(-(ZTHUMB*S1210[3][1]) - XTHUMB*S1210[3][3])*((-(ZTHUMB*Si1012[1][3]) - XTHUMB*Si1012[3][3])*T11012[3][6] + Si1012[2][1]*T11012[4][6] + Si1012[2][2]*T11012[5][6]) + S1210[1][2]*((-(ZTHUMB*Si1012[1][1]) - XTHUMB*Si1012[3][1])*T11012[1][1] + (-(ZTHUMB*Si1012[1][2]) - XTHUMB*Si1012[3][2])*T11012[2][1] + (-(ZTHUMB*Si1012[1][3]) - XTHUMB*Si1012[3][3])*T11012[3][1] + Si1012[2][1]*T11012[4][1] + Si1012[2][2]*T11012[5][1] + Si1012[2][3]*T11012[6][1]) + S1210[2][2]*((-(ZTHUMB*Si1012[1][1]) - XTHUMB*Si1012[3][1])*T11012[1][2] + (-(ZTHUMB*Si1012[1][2]) - XTHUMB*Si1012[3][2])*T11012[2][2] + (-(ZTHUMB*Si1012[1][3]) - XTHUMB*Si1012[3][3])*T11012[3][2] + Si1012[2][1]*T11012[4][2] + Si1012[2][2]*T11012[5][2] + Si1012[2][3]*T11012[6][2]) + S1210[3][2]*((-(ZTHUMB*Si1012[1][1]) - XTHUMB*Si1012[3][1])*T11012[1][3] + (-(ZTHUMB*Si1012[1][2]) - XTHUMB*Si1012[3][2])*T11012[2][3] + Si1012[2][1]*T11012[4][3] + Si1012[2][2]*T11012[5][3] + Si1012[2][3]*T11012[6][3]) + (-(ZTHUMB*S1210[1][1]) - XTHUMB*S1210[1][3])*((-(ZTHUMB*Si1012[1][1]) - XTHUMB*Si1012[3][1])*T11012[1][4] + (-(ZTHUMB*Si1012[1][2]) - XTHUMB*Si1012[3][2])*T11012[2][4] + Si1012[2][1]*T11012[4][4] + Si1012[2][2]*T11012[5][4] + Si1012[2][3]*T11012[6][4]) + (-(ZTHUMB*S1210[2][1]) - XTHUMB*S1210[2][3])*((-(ZTHUMB*Si1012[1][1]) - XTHUMB*Si1012[3][1])*T11012[1][5] + (-(ZTHUMB*Si1012[1][2]) - XTHUMB*Si1012[3][2])*T11012[2][5] + Si1012[2][1]*T11012[4][5] + Si1012[2][2]*T11012[5][5] + Si1012[2][3]*T11012[6][5]);
T1012[5][3]=(-(YTHUMB*S1210[3][1]) + XTHUMB*S1210[3][2])*((-(ZTHUMB*Si1012[1][3]) - XTHUMB*Si1012[3][3])*T11012[3][6] + Si1012[2][1]*T11012[4][6] + Si1012[2][2]*T11012[5][6]) + S1210[1][3]*((-(ZTHUMB*Si1012[1][1]) - XTHUMB*Si1012[3][1])*T11012[1][1] + (-(ZTHUMB*Si1012[1][2]) - XTHUMB*Si1012[3][2])*T11012[2][1] + (-(ZTHUMB*Si1012[1][3]) - XTHUMB*Si1012[3][3])*T11012[3][1] + Si1012[2][1]*T11012[4][1] + Si1012[2][2]*T11012[5][1] + Si1012[2][3]*T11012[6][1]) + S1210[2][3]*((-(ZTHUMB*Si1012[1][1]) - XTHUMB*Si1012[3][1])*T11012[1][2] + (-(ZTHUMB*Si1012[1][2]) - XTHUMB*Si1012[3][2])*T11012[2][2] + (-(ZTHUMB*Si1012[1][3]) - XTHUMB*Si1012[3][3])*T11012[3][2] + Si1012[2][1]*T11012[4][2] + Si1012[2][2]*T11012[5][2] + Si1012[2][3]*T11012[6][2]) + S1210[3][3]*((-(ZTHUMB*Si1012[1][1]) - XTHUMB*Si1012[3][1])*T11012[1][3] + (-(ZTHUMB*Si1012[1][2]) - XTHUMB*Si1012[3][2])*T11012[2][3] + Si1012[2][1]*T11012[4][3] + Si1012[2][2]*T11012[5][3] + Si1012[2][3]*T11012[6][3]) + (-(YTHUMB*S1210[1][1]) + XTHUMB*S1210[1][2])*((-(ZTHUMB*Si1012[1][1]) - XTHUMB*Si1012[3][1])*T11012[1][4] + (-(ZTHUMB*Si1012[1][2]) - XTHUMB*Si1012[3][2])*T11012[2][4] + Si1012[2][1]*T11012[4][4] + Si1012[2][2]*T11012[5][4] + Si1012[2][3]*T11012[6][4]) + (-(YTHUMB*S1210[2][1]) + XTHUMB*S1210[2][2])*((-(ZTHUMB*Si1012[1][1]) - XTHUMB*Si1012[3][1])*T11012[1][5] + (-(ZTHUMB*Si1012[1][2]) - XTHUMB*Si1012[3][2])*T11012[2][5] + Si1012[2][1]*T11012[4][5] + Si1012[2][2]*T11012[5][5] + Si1012[2][3]*T11012[6][5]);
T1012[5][4]=S1210[3][1]*((-(ZTHUMB*Si1012[1][3]) - XTHUMB*Si1012[3][3])*T11012[3][6] + Si1012[2][1]*T11012[4][6] + Si1012[2][2]*T11012[5][6]) + S1210[1][1]*((-(ZTHUMB*Si1012[1][1]) - XTHUMB*Si1012[3][1])*T11012[1][4] + (-(ZTHUMB*Si1012[1][2]) - XTHUMB*Si1012[3][2])*T11012[2][4] + Si1012[2][1]*T11012[4][4] + Si1012[2][2]*T11012[5][4] + Si1012[2][3]*T11012[6][4]) + S1210[2][1]*((-(ZTHUMB*Si1012[1][1]) - XTHUMB*Si1012[3][1])*T11012[1][5] + (-(ZTHUMB*Si1012[1][2]) - XTHUMB*Si1012[3][2])*T11012[2][5] + Si1012[2][1]*T11012[4][5] + Si1012[2][2]*T11012[5][5] + Si1012[2][3]*T11012[6][5]);
T1012[5][5]=S1210[3][2]*((-(ZTHUMB*Si1012[1][3]) - XTHUMB*Si1012[3][3])*T11012[3][6] + Si1012[2][1]*T11012[4][6] + Si1012[2][2]*T11012[5][6]) + S1210[1][2]*((-(ZTHUMB*Si1012[1][1]) - XTHUMB*Si1012[3][1])*T11012[1][4] + (-(ZTHUMB*Si1012[1][2]) - XTHUMB*Si1012[3][2])*T11012[2][4] + Si1012[2][1]*T11012[4][4] + Si1012[2][2]*T11012[5][4] + Si1012[2][3]*T11012[6][4]) + S1210[2][2]*((-(ZTHUMB*Si1012[1][1]) - XTHUMB*Si1012[3][1])*T11012[1][5] + (-(ZTHUMB*Si1012[1][2]) - XTHUMB*Si1012[3][2])*T11012[2][5] + Si1012[2][1]*T11012[4][5] + Si1012[2][2]*T11012[5][5] + Si1012[2][3]*T11012[6][5]);
T1012[5][6]=S1210[3][3]*((-(ZTHUMB*Si1012[1][3]) - XTHUMB*Si1012[3][3])*T11012[3][6] + Si1012[2][1]*T11012[4][6] + Si1012[2][2]*T11012[5][6]) + S1210[1][3]*((-(ZTHUMB*Si1012[1][1]) - XTHUMB*Si1012[3][1])*T11012[1][4] + (-(ZTHUMB*Si1012[1][2]) - XTHUMB*Si1012[3][2])*T11012[2][4] + Si1012[2][1]*T11012[4][4] + Si1012[2][2]*T11012[5][4] + Si1012[2][3]*T11012[6][4]) + S1210[2][3]*((-(ZTHUMB*Si1012[1][1]) - XTHUMB*Si1012[3][1])*T11012[1][5] + (-(ZTHUMB*Si1012[1][2]) - XTHUMB*Si1012[3][2])*T11012[2][5] + Si1012[2][1]*T11012[4][5] + Si1012[2][2]*T11012[5][5] + Si1012[2][3]*T11012[6][5]);

T1012[6][1]=(ZTHUMB*S1210[3][2] + YTHUMB*S1210[3][3])*((-(YTHUMB*Si1012[1][3]) + XTHUMB*Si1012[2][3])*T11012[3][6] + Si1012[3][1]*T11012[4][6] + Si1012[3][2]*T11012[5][6]) + S1210[1][1]*((-(YTHUMB*Si1012[1][1]) + XTHUMB*Si1012[2][1])*T11012[1][1] + (-(YTHUMB*Si1012[1][2]) + XTHUMB*Si1012[2][2])*T11012[2][1] + (-(YTHUMB*Si1012[1][3]) + XTHUMB*Si1012[2][3])*T11012[3][1] + Si1012[3][1]*T11012[4][1] + Si1012[3][2]*T11012[5][1] + Si1012[3][3]*T11012[6][1]) + S1210[2][1]*((-(YTHUMB*Si1012[1][1]) + XTHUMB*Si1012[2][1])*T11012[1][2] + (-(YTHUMB*Si1012[1][2]) + XTHUMB*Si1012[2][2])*T11012[2][2] + (-(YTHUMB*Si1012[1][3]) + XTHUMB*Si1012[2][3])*T11012[3][2] + Si1012[3][1]*T11012[4][2] + Si1012[3][2]*T11012[5][2] + Si1012[3][3]*T11012[6][2]) + S1210[3][1]*((-(YTHUMB*Si1012[1][1]) + XTHUMB*Si1012[2][1])*T11012[1][3] + (-(YTHUMB*Si1012[1][2]) + XTHUMB*Si1012[2][2])*T11012[2][3] + Si1012[3][1]*T11012[4][3] + Si1012[3][2]*T11012[5][3] + Si1012[3][3]*T11012[6][3]) + (ZTHUMB*S1210[1][2] + YTHUMB*S1210[1][3])*((-(YTHUMB*Si1012[1][1]) + XTHUMB*Si1012[2][1])*T11012[1][4] + (-(YTHUMB*Si1012[1][2]) + XTHUMB*Si1012[2][2])*T11012[2][4] + Si1012[3][1]*T11012[4][4] + Si1012[3][2]*T11012[5][4] + Si1012[3][3]*T11012[6][4]) + (ZTHUMB*S1210[2][2] + YTHUMB*S1210[2][3])*((-(YTHUMB*Si1012[1][1]) + XTHUMB*Si1012[2][1])*T11012[1][5] + (-(YTHUMB*Si1012[1][2]) + XTHUMB*Si1012[2][2])*T11012[2][5] + Si1012[3][1]*T11012[4][5] + Si1012[3][2]*T11012[5][5] + Si1012[3][3]*T11012[6][5]);
T1012[6][2]=(-(ZTHUMB*S1210[3][1]) - XTHUMB*S1210[3][3])*((-(YTHUMB*Si1012[1][3]) + XTHUMB*Si1012[2][3])*T11012[3][6] + Si1012[3][1]*T11012[4][6] + Si1012[3][2]*T11012[5][6]) + S1210[1][2]*((-(YTHUMB*Si1012[1][1]) + XTHUMB*Si1012[2][1])*T11012[1][1] + (-(YTHUMB*Si1012[1][2]) + XTHUMB*Si1012[2][2])*T11012[2][1] + (-(YTHUMB*Si1012[1][3]) + XTHUMB*Si1012[2][3])*T11012[3][1] + Si1012[3][1]*T11012[4][1] + Si1012[3][2]*T11012[5][1] + Si1012[3][3]*T11012[6][1]) + S1210[2][2]*((-(YTHUMB*Si1012[1][1]) + XTHUMB*Si1012[2][1])*T11012[1][2] + (-(YTHUMB*Si1012[1][2]) + XTHUMB*Si1012[2][2])*T11012[2][2] + (-(YTHUMB*Si1012[1][3]) + XTHUMB*Si1012[2][3])*T11012[3][2] + Si1012[3][1]*T11012[4][2] + Si1012[3][2]*T11012[5][2] + Si1012[3][3]*T11012[6][2]) + S1210[3][2]*((-(YTHUMB*Si1012[1][1]) + XTHUMB*Si1012[2][1])*T11012[1][3] + (-(YTHUMB*Si1012[1][2]) + XTHUMB*Si1012[2][2])*T11012[2][3] + Si1012[3][1]*T11012[4][3] + Si1012[3][2]*T11012[5][3] + Si1012[3][3]*T11012[6][3]) + (-(ZTHUMB*S1210[1][1]) - XTHUMB*S1210[1][3])*((-(YTHUMB*Si1012[1][1]) + XTHUMB*Si1012[2][1])*T11012[1][4] + (-(YTHUMB*Si1012[1][2]) + XTHUMB*Si1012[2][2])*T11012[2][4] + Si1012[3][1]*T11012[4][4] + Si1012[3][2]*T11012[5][4] + Si1012[3][3]*T11012[6][4]) + (-(ZTHUMB*S1210[2][1]) - XTHUMB*S1210[2][3])*((-(YTHUMB*Si1012[1][1]) + XTHUMB*Si1012[2][1])*T11012[1][5] + (-(YTHUMB*Si1012[1][2]) + XTHUMB*Si1012[2][2])*T11012[2][5] + Si1012[3][1]*T11012[4][5] + Si1012[3][2]*T11012[5][5] + Si1012[3][3]*T11012[6][5]);
T1012[6][3]=(-(YTHUMB*S1210[3][1]) + XTHUMB*S1210[3][2])*((-(YTHUMB*Si1012[1][3]) + XTHUMB*Si1012[2][3])*T11012[3][6] + Si1012[3][1]*T11012[4][6] + Si1012[3][2]*T11012[5][6]) + S1210[1][3]*((-(YTHUMB*Si1012[1][1]) + XTHUMB*Si1012[2][1])*T11012[1][1] + (-(YTHUMB*Si1012[1][2]) + XTHUMB*Si1012[2][2])*T11012[2][1] + (-(YTHUMB*Si1012[1][3]) + XTHUMB*Si1012[2][3])*T11012[3][1] + Si1012[3][1]*T11012[4][1] + Si1012[3][2]*T11012[5][1] + Si1012[3][3]*T11012[6][1]) + S1210[2][3]*((-(YTHUMB*Si1012[1][1]) + XTHUMB*Si1012[2][1])*T11012[1][2] + (-(YTHUMB*Si1012[1][2]) + XTHUMB*Si1012[2][2])*T11012[2][2] + (-(YTHUMB*Si1012[1][3]) + XTHUMB*Si1012[2][3])*T11012[3][2] + Si1012[3][1]*T11012[4][2] + Si1012[3][2]*T11012[5][2] + Si1012[3][3]*T11012[6][2]) + S1210[3][3]*((-(YTHUMB*Si1012[1][1]) + XTHUMB*Si1012[2][1])*T11012[1][3] + (-(YTHUMB*Si1012[1][2]) + XTHUMB*Si1012[2][2])*T11012[2][3] + Si1012[3][1]*T11012[4][3] + Si1012[3][2]*T11012[5][3] + Si1012[3][3]*T11012[6][3]) + (-(YTHUMB*S1210[1][1]) + XTHUMB*S1210[1][2])*((-(YTHUMB*Si1012[1][1]) + XTHUMB*Si1012[2][1])*T11012[1][4] + (-(YTHUMB*Si1012[1][2]) + XTHUMB*Si1012[2][2])*T11012[2][4] + Si1012[3][1]*T11012[4][4] + Si1012[3][2]*T11012[5][4] + Si1012[3][3]*T11012[6][4]) + (-(YTHUMB*S1210[2][1]) + XTHUMB*S1210[2][2])*((-(YTHUMB*Si1012[1][1]) + XTHUMB*Si1012[2][1])*T11012[1][5] + (-(YTHUMB*Si1012[1][2]) + XTHUMB*Si1012[2][2])*T11012[2][5] + Si1012[3][1]*T11012[4][5] + Si1012[3][2]*T11012[5][5] + Si1012[3][3]*T11012[6][5]);
T1012[6][4]=S1210[3][1]*((-(YTHUMB*Si1012[1][3]) + XTHUMB*Si1012[2][3])*T11012[3][6] + Si1012[3][1]*T11012[4][6] + Si1012[3][2]*T11012[5][6]) + S1210[1][1]*((-(YTHUMB*Si1012[1][1]) + XTHUMB*Si1012[2][1])*T11012[1][4] + (-(YTHUMB*Si1012[1][2]) + XTHUMB*Si1012[2][2])*T11012[2][4] + Si1012[3][1]*T11012[4][4] + Si1012[3][2]*T11012[5][4] + Si1012[3][3]*T11012[6][4]) + S1210[2][1]*((-(YTHUMB*Si1012[1][1]) + XTHUMB*Si1012[2][1])*T11012[1][5] + (-(YTHUMB*Si1012[1][2]) + XTHUMB*Si1012[2][2])*T11012[2][5] + Si1012[3][1]*T11012[4][5] + Si1012[3][2]*T11012[5][5] + Si1012[3][3]*T11012[6][5]);
T1012[6][5]=S1210[3][2]*((-(YTHUMB*Si1012[1][3]) + XTHUMB*Si1012[2][3])*T11012[3][6] + Si1012[3][1]*T11012[4][6] + Si1012[3][2]*T11012[5][6]) + S1210[1][2]*((-(YTHUMB*Si1012[1][1]) + XTHUMB*Si1012[2][1])*T11012[1][4] + (-(YTHUMB*Si1012[1][2]) + XTHUMB*Si1012[2][2])*T11012[2][4] + Si1012[3][1]*T11012[4][4] + Si1012[3][2]*T11012[5][4] + Si1012[3][3]*T11012[6][4]) + S1210[2][2]*((-(YTHUMB*Si1012[1][1]) + XTHUMB*Si1012[2][1])*T11012[1][5] + (-(YTHUMB*Si1012[1][2]) + XTHUMB*Si1012[2][2])*T11012[2][5] + Si1012[3][1]*T11012[4][5] + Si1012[3][2]*T11012[5][5] + Si1012[3][3]*T11012[6][5]);
T1012[6][6]=S1210[3][3]*((-(YTHUMB*Si1012[1][3]) + XTHUMB*Si1012[2][3])*T11012[3][6] + Si1012[3][1]*T11012[4][6] + Si1012[3][2]*T11012[5][6]) + S1210[1][3]*((-(YTHUMB*Si1012[1][1]) + XTHUMB*Si1012[2][1])*T11012[1][4] + (-(YTHUMB*Si1012[1][2]) + XTHUMB*Si1012[2][2])*T11012[2][4] + Si1012[3][1]*T11012[4][4] + Si1012[3][2]*T11012[5][4] + Si1012[3][3]*T11012[6][4]) + S1210[2][3]*((-(YTHUMB*Si1012[1][1]) + XTHUMB*Si1012[2][1])*T11012[1][5] + (-(YTHUMB*Si1012[1][2]) + XTHUMB*Si1012[2][2])*T11012[2][5] + Si1012[3][1]*T11012[4][5] + Si1012[3][2]*T11012[5][5] + Si1012[3][3]*T11012[6][5]);



}


void
hermes_InvDynArtfunc93(void)
      {
JA11[1][2]=eff[2].mcm[3];
JA11[1][3]=-eff[2].mcm[2];
JA11[1][4]=eff[2].m;

JA11[2][1]=-eff[2].mcm[3];
JA11[2][3]=eff[2].mcm[1];
JA11[2][5]=eff[2].m;

JA11[3][1]=eff[2].mcm[2];
JA11[3][2]=-eff[2].mcm[1];
JA11[3][6]=eff[2].m;

JA11[4][5]=-eff[2].mcm[3];
JA11[4][6]=eff[2].mcm[2];

JA11[5][4]=eff[2].mcm[3];
JA11[5][6]=-eff[2].mcm[1];

JA11[6][4]=-eff[2].mcm[2];
JA11[6][5]=eff[2].mcm[1];


T11011[1][2]=JA11[1][2];
T11011[1][3]=JA11[1][3];
T11011[1][4]=JA11[1][4];

T11011[2][1]=JA11[2][1];
T11011[2][3]=JA11[2][3];
T11011[2][5]=JA11[2][5];

T11011[3][1]=JA11[3][1];
T11011[3][2]=JA11[3][2];
T11011[3][6]=JA11[3][6];

T11011[4][5]=JA11[4][5];
T11011[4][6]=JA11[4][6];

T11011[5][4]=JA11[5][4];
T11011[5][6]=JA11[5][6];

T11011[6][4]=JA11[6][4];
T11011[6][5]=JA11[6][5];


T1011[1][1]=(-(eff[2].x[3]*S1110[1][2]) + eff[2].x[2]*S1110[1][3])*Si1011[1][1]*T11011[1][4] + S1110[3][1]*(Si1011[1][1]*T11011[1][3] + Si1011[1][2]*T11011[2][3]) + (-(eff[2].x[3]*S1110[2][2]) + eff[2].x[2]*S1110[2][3])*Si1011[1][2]*T11011[2][5] + S1110[1][1]*(Si1011[1][2]*T11011[2][1] + Si1011[1][3]*T11011[3][1]) + S1110[2][1]*(Si1011[1][1]*T11011[1][2] + Si1011[1][3]*T11011[3][2]) + (-(eff[2].x[3]*S1110[3][2]) + eff[2].x[2]*S1110[3][3])*Si1011[1][3]*T11011[3][6];
T1011[1][2]=(eff[2].x[3]*S1110[1][1] - eff[2].x[1]*S1110[1][3])*Si1011[1][1]*T11011[1][4] + S1110[3][2]*(Si1011[1][1]*T11011[1][3] + Si1011[1][2]*T11011[2][3]) + (eff[2].x[3]*S1110[2][1] - eff[2].x[1]*S1110[2][3])*Si1011[1][2]*T11011[2][5] + S1110[1][2]*(Si1011[1][2]*T11011[2][1] + Si1011[1][3]*T11011[3][1]) + S1110[2][2]*(Si1011[1][1]*T11011[1][2] + Si1011[1][3]*T11011[3][2]) + (eff[2].x[3]*S1110[3][1] - eff[2].x[1]*S1110[3][3])*Si1011[1][3]*T11011[3][6];
T1011[1][3]=(-(eff[2].x[2]*S1110[1][1]) + eff[2].x[1]*S1110[1][2])*Si1011[1][1]*T11011[1][4] + S1110[3][3]*(Si1011[1][1]*T11011[1][3] + Si1011[1][2]*T11011[2][3]) + (-(eff[2].x[2]*S1110[2][1]) + eff[2].x[1]*S1110[2][2])*Si1011[1][2]*T11011[2][5] + S1110[1][3]*(Si1011[1][2]*T11011[2][1] + Si1011[1][3]*T11011[3][1]) + S1110[2][3]*(Si1011[1][1]*T11011[1][2] + Si1011[1][3]*T11011[3][2]) + (-(eff[2].x[2]*S1110[3][1]) + eff[2].x[1]*S1110[3][2])*Si1011[1][3]*T11011[3][6];
T1011[1][4]=S1110[1][1]*Si1011[1][1]*T11011[1][4] + S1110[2][1]*Si1011[1][2]*T11011[2][5] + S1110[3][1]*Si1011[1][3]*T11011[3][6];
T1011[1][5]=S1110[1][2]*Si1011[1][1]*T11011[1][4] + S1110[2][2]*Si1011[1][2]*T11011[2][5] + S1110[3][2]*Si1011[1][3]*T11011[3][6];
T1011[1][6]=S1110[1][3]*Si1011[1][1]*T11011[1][4] + S1110[2][3]*Si1011[1][2]*T11011[2][5] + S1110[3][3]*Si1011[1][3]*T11011[3][6];

T1011[2][1]=(-(eff[2].x[3]*S1110[1][2]) + eff[2].x[2]*S1110[1][3])*Si1011[2][1]*T11011[1][4] + S1110[3][1]*(Si1011[2][1]*T11011[1][3] + Si1011[2][2]*T11011[2][3]) + (-(eff[2].x[3]*S1110[2][2]) + eff[2].x[2]*S1110[2][3])*Si1011[2][2]*T11011[2][5] + S1110[1][1]*(Si1011[2][2]*T11011[2][1] + Si1011[2][3]*T11011[3][1]) + S1110[2][1]*(Si1011[2][1]*T11011[1][2] + Si1011[2][3]*T11011[3][2]) + (-(eff[2].x[3]*S1110[3][2]) + eff[2].x[2]*S1110[3][3])*Si1011[2][3]*T11011[3][6];
T1011[2][2]=(eff[2].x[3]*S1110[1][1] - eff[2].x[1]*S1110[1][3])*Si1011[2][1]*T11011[1][4] + S1110[3][2]*(Si1011[2][1]*T11011[1][3] + Si1011[2][2]*T11011[2][3]) + (eff[2].x[3]*S1110[2][1] - eff[2].x[1]*S1110[2][3])*Si1011[2][2]*T11011[2][5] + S1110[1][2]*(Si1011[2][2]*T11011[2][1] + Si1011[2][3]*T11011[3][1]) + S1110[2][2]*(Si1011[2][1]*T11011[1][2] + Si1011[2][3]*T11011[3][2]) + (eff[2].x[3]*S1110[3][1] - eff[2].x[1]*S1110[3][3])*Si1011[2][3]*T11011[3][6];
T1011[2][3]=(-(eff[2].x[2]*S1110[1][1]) + eff[2].x[1]*S1110[1][2])*Si1011[2][1]*T11011[1][4] + S1110[3][3]*(Si1011[2][1]*T11011[1][3] + Si1011[2][2]*T11011[2][3]) + (-(eff[2].x[2]*S1110[2][1]) + eff[2].x[1]*S1110[2][2])*Si1011[2][2]*T11011[2][5] + S1110[1][3]*(Si1011[2][2]*T11011[2][1] + Si1011[2][3]*T11011[3][1]) + S1110[2][3]*(Si1011[2][1]*T11011[1][2] + Si1011[2][3]*T11011[3][2]) + (-(eff[2].x[2]*S1110[3][1]) + eff[2].x[1]*S1110[3][2])*Si1011[2][3]*T11011[3][6];
T1011[2][4]=S1110[1][1]*Si1011[2][1]*T11011[1][4] + S1110[2][1]*Si1011[2][2]*T11011[2][5] + S1110[3][1]*Si1011[2][3]*T11011[3][6];
T1011[2][5]=S1110[1][2]*Si1011[2][1]*T11011[1][4] + S1110[2][2]*Si1011[2][2]*T11011[2][5] + S1110[3][2]*Si1011[2][3]*T11011[3][6];
T1011[2][6]=S1110[1][3]*Si1011[2][1]*T11011[1][4] + S1110[2][3]*Si1011[2][2]*T11011[2][5] + S1110[3][3]*Si1011[2][3]*T11011[3][6];

T1011[3][1]=(-(eff[2].x[3]*S1110[1][2]) + eff[2].x[2]*S1110[1][3])*Si1011[3][1]*T11011[1][4] + S1110[3][1]*(Si1011[3][1]*T11011[1][3] + Si1011[3][2]*T11011[2][3]) + (-(eff[2].x[3]*S1110[2][2]) + eff[2].x[2]*S1110[2][3])*Si1011[3][2]*T11011[2][5] + S1110[1][1]*(Si1011[3][2]*T11011[2][1] + Si1011[3][3]*T11011[3][1]) + S1110[2][1]*(Si1011[3][1]*T11011[1][2] + Si1011[3][3]*T11011[3][2]) + (-(eff[2].x[3]*S1110[3][2]) + eff[2].x[2]*S1110[3][3])*Si1011[3][3]*T11011[3][6];
T1011[3][2]=(eff[2].x[3]*S1110[1][1] - eff[2].x[1]*S1110[1][3])*Si1011[3][1]*T11011[1][4] + S1110[3][2]*(Si1011[3][1]*T11011[1][3] + Si1011[3][2]*T11011[2][3]) + (eff[2].x[3]*S1110[2][1] - eff[2].x[1]*S1110[2][3])*Si1011[3][2]*T11011[2][5] + S1110[1][2]*(Si1011[3][2]*T11011[2][1] + Si1011[3][3]*T11011[3][1]) + S1110[2][2]*(Si1011[3][1]*T11011[1][2] + Si1011[3][3]*T11011[3][2]) + (eff[2].x[3]*S1110[3][1] - eff[2].x[1]*S1110[3][3])*Si1011[3][3]*T11011[3][6];
T1011[3][3]=(-(eff[2].x[2]*S1110[1][1]) + eff[2].x[1]*S1110[1][2])*Si1011[3][1]*T11011[1][4] + S1110[3][3]*(Si1011[3][1]*T11011[1][3] + Si1011[3][2]*T11011[2][3]) + (-(eff[2].x[2]*S1110[2][1]) + eff[2].x[1]*S1110[2][2])*Si1011[3][2]*T11011[2][5] + S1110[1][3]*(Si1011[3][2]*T11011[2][1] + Si1011[3][3]*T11011[3][1]) + S1110[2][3]*(Si1011[3][1]*T11011[1][2] + Si1011[3][3]*T11011[3][2]) + (-(eff[2].x[2]*S1110[3][1]) + eff[2].x[1]*S1110[3][2])*Si1011[3][3]*T11011[3][6];
T1011[3][4]=S1110[1][1]*Si1011[3][1]*T11011[1][4] + S1110[2][1]*Si1011[3][2]*T11011[2][5] + S1110[3][1]*Si1011[3][3]*T11011[3][6];
T1011[3][5]=S1110[1][2]*Si1011[3][1]*T11011[1][4] + S1110[2][2]*Si1011[3][2]*T11011[2][5] + S1110[3][2]*Si1011[3][3]*T11011[3][6];
T1011[3][6]=S1110[1][3]*Si1011[3][1]*T11011[1][4] + S1110[2][3]*Si1011[3][2]*T11011[2][5] + S1110[3][3]*Si1011[3][3]*T11011[3][6];

T1011[4][1]=S1110[3][1]*((-(eff[2].x[3]*Si1011[2][1]) + eff[2].x[2]*Si1011[3][1])*T11011[1][3] + (-(eff[2].x[3]*Si1011[2][2]) + eff[2].x[2]*Si1011[3][2])*T11011[2][3]) + S1110[1][1]*((-(eff[2].x[3]*Si1011[2][2]) + eff[2].x[2]*Si1011[3][2])*T11011[2][1] + (-(eff[2].x[3]*Si1011[2][3]) + eff[2].x[2]*Si1011[3][3])*T11011[3][1]) + S1110[2][1]*((-(eff[2].x[3]*Si1011[2][1]) + eff[2].x[2]*Si1011[3][1])*T11011[1][2] + (-(eff[2].x[3]*Si1011[2][3]) + eff[2].x[2]*Si1011[3][3])*T11011[3][2]) + (-(eff[2].x[3]*S1110[3][2]) + eff[2].x[2]*S1110[3][3])*((-(eff[2].x[3]*Si1011[2][3]) + eff[2].x[2]*Si1011[3][3])*T11011[3][6] + Si1011[1][1]*T11011[4][6] + Si1011[1][2]*T11011[5][6]) + (-(eff[2].x[3]*S1110[1][2]) + eff[2].x[2]*S1110[1][3])*((-(eff[2].x[3]*Si1011[2][1]) + eff[2].x[2]*Si1011[3][1])*T11011[1][4] + Si1011[1][2]*T11011[5][4] + Si1011[1][3]*T11011[6][4]) + (-(eff[2].x[3]*S1110[2][2]) + eff[2].x[2]*S1110[2][3])*((-(eff[2].x[3]*Si1011[2][2]) + eff[2].x[2]*Si1011[3][2])*T11011[2][5] + Si1011[1][1]*T11011[4][5] + Si1011[1][3]*T11011[6][5]);
T1011[4][2]=S1110[3][2]*((-(eff[2].x[3]*Si1011[2][1]) + eff[2].x[2]*Si1011[3][1])*T11011[1][3] + (-(eff[2].x[3]*Si1011[2][2]) + eff[2].x[2]*Si1011[3][2])*T11011[2][3]) + S1110[1][2]*((-(eff[2].x[3]*Si1011[2][2]) + eff[2].x[2]*Si1011[3][2])*T11011[2][1] + (-(eff[2].x[3]*Si1011[2][3]) + eff[2].x[2]*Si1011[3][3])*T11011[3][1]) + S1110[2][2]*((-(eff[2].x[3]*Si1011[2][1]) + eff[2].x[2]*Si1011[3][1])*T11011[1][2] + (-(eff[2].x[3]*Si1011[2][3]) + eff[2].x[2]*Si1011[3][3])*T11011[3][2]) + (eff[2].x[3]*S1110[3][1] - eff[2].x[1]*S1110[3][3])*((-(eff[2].x[3]*Si1011[2][3]) + eff[2].x[2]*Si1011[3][3])*T11011[3][6] + Si1011[1][1]*T11011[4][6] + Si1011[1][2]*T11011[5][6]) + (eff[2].x[3]*S1110[1][1] - eff[2].x[1]*S1110[1][3])*((-(eff[2].x[3]*Si1011[2][1]) + eff[2].x[2]*Si1011[3][1])*T11011[1][4] + Si1011[1][2]*T11011[5][4] + Si1011[1][3]*T11011[6][4]) + (eff[2].x[3]*S1110[2][1] - eff[2].x[1]*S1110[2][3])*((-(eff[2].x[3]*Si1011[2][2]) + eff[2].x[2]*Si1011[3][2])*T11011[2][5] + Si1011[1][1]*T11011[4][5] + Si1011[1][3]*T11011[6][5]);
T1011[4][3]=S1110[3][3]*((-(eff[2].x[3]*Si1011[2][1]) + eff[2].x[2]*Si1011[3][1])*T11011[1][3] + (-(eff[2].x[3]*Si1011[2][2]) + eff[2].x[2]*Si1011[3][2])*T11011[2][3]) + S1110[1][3]*((-(eff[2].x[3]*Si1011[2][2]) + eff[2].x[2]*Si1011[3][2])*T11011[2][1] + (-(eff[2].x[3]*Si1011[2][3]) + eff[2].x[2]*Si1011[3][3])*T11011[3][1]) + S1110[2][3]*((-(eff[2].x[3]*Si1011[2][1]) + eff[2].x[2]*Si1011[3][1])*T11011[1][2] + (-(eff[2].x[3]*Si1011[2][3]) + eff[2].x[2]*Si1011[3][3])*T11011[3][2]) + (-(eff[2].x[2]*S1110[3][1]) + eff[2].x[1]*S1110[3][2])*((-(eff[2].x[3]*Si1011[2][3]) + eff[2].x[2]*Si1011[3][3])*T11011[3][6] + Si1011[1][1]*T11011[4][6] + Si1011[1][2]*T11011[5][6]) + (-(eff[2].x[2]*S1110[1][1]) + eff[2].x[1]*S1110[1][2])*((-(eff[2].x[3]*Si1011[2][1]) + eff[2].x[2]*Si1011[3][1])*T11011[1][4] + Si1011[1][2]*T11011[5][4] + Si1011[1][3]*T11011[6][4]) + (-(eff[2].x[2]*S1110[2][1]) + eff[2].x[1]*S1110[2][2])*((-(eff[2].x[3]*Si1011[2][2]) + eff[2].x[2]*Si1011[3][2])*T11011[2][5] + Si1011[1][1]*T11011[4][5] + Si1011[1][3]*T11011[6][5]);
T1011[4][4]=S1110[3][1]*((-(eff[2].x[3]*Si1011[2][3]) + eff[2].x[2]*Si1011[3][3])*T11011[3][6] + Si1011[1][1]*T11011[4][6] + Si1011[1][2]*T11011[5][6]) + S1110[1][1]*((-(eff[2].x[3]*Si1011[2][1]) + eff[2].x[2]*Si1011[3][1])*T11011[1][4] + Si1011[1][2]*T11011[5][4] + Si1011[1][3]*T11011[6][4]) + S1110[2][1]*((-(eff[2].x[3]*Si1011[2][2]) + eff[2].x[2]*Si1011[3][2])*T11011[2][5] + Si1011[1][1]*T11011[4][5] + Si1011[1][3]*T11011[6][5]);
T1011[4][5]=S1110[3][2]*((-(eff[2].x[3]*Si1011[2][3]) + eff[2].x[2]*Si1011[3][3])*T11011[3][6] + Si1011[1][1]*T11011[4][6] + Si1011[1][2]*T11011[5][6]) + S1110[1][2]*((-(eff[2].x[3]*Si1011[2][1]) + eff[2].x[2]*Si1011[3][1])*T11011[1][4] + Si1011[1][2]*T11011[5][4] + Si1011[1][3]*T11011[6][4]) + S1110[2][2]*((-(eff[2].x[3]*Si1011[2][2]) + eff[2].x[2]*Si1011[3][2])*T11011[2][5] + Si1011[1][1]*T11011[4][5] + Si1011[1][3]*T11011[6][5]);
T1011[4][6]=S1110[3][3]*((-(eff[2].x[3]*Si1011[2][3]) + eff[2].x[2]*Si1011[3][3])*T11011[3][6] + Si1011[1][1]*T11011[4][6] + Si1011[1][2]*T11011[5][6]) + S1110[1][3]*((-(eff[2].x[3]*Si1011[2][1]) + eff[2].x[2]*Si1011[3][1])*T11011[1][4] + Si1011[1][2]*T11011[5][4] + Si1011[1][3]*T11011[6][4]) + S1110[2][3]*((-(eff[2].x[3]*Si1011[2][2]) + eff[2].x[2]*Si1011[3][2])*T11011[2][5] + Si1011[1][1]*T11011[4][5] + Si1011[1][3]*T11011[6][5]);

T1011[5][1]=S1110[3][1]*((eff[2].x[3]*Si1011[1][1] - eff[2].x[1]*Si1011[3][1])*T11011[1][3] + (eff[2].x[3]*Si1011[1][2] - eff[2].x[1]*Si1011[3][2])*T11011[2][3]) + S1110[1][1]*((eff[2].x[3]*Si1011[1][2] - eff[2].x[1]*Si1011[3][2])*T11011[2][1] + (eff[2].x[3]*Si1011[1][3] - eff[2].x[1]*Si1011[3][3])*T11011[3][1]) + S1110[2][1]*((eff[2].x[3]*Si1011[1][1] - eff[2].x[1]*Si1011[3][1])*T11011[1][2] + (eff[2].x[3]*Si1011[1][3] - eff[2].x[1]*Si1011[3][3])*T11011[3][2]) + (-(eff[2].x[3]*S1110[3][2]) + eff[2].x[2]*S1110[3][3])*((eff[2].x[3]*Si1011[1][3] - eff[2].x[1]*Si1011[3][3])*T11011[3][6] + Si1011[2][1]*T11011[4][6] + Si1011[2][2]*T11011[5][6]) + (-(eff[2].x[3]*S1110[1][2]) + eff[2].x[2]*S1110[1][3])*((eff[2].x[3]*Si1011[1][1] - eff[2].x[1]*Si1011[3][1])*T11011[1][4] + Si1011[2][2]*T11011[5][4] + Si1011[2][3]*T11011[6][4]) + (-(eff[2].x[3]*S1110[2][2]) + eff[2].x[2]*S1110[2][3])*((eff[2].x[3]*Si1011[1][2] - eff[2].x[1]*Si1011[3][2])*T11011[2][5] + Si1011[2][1]*T11011[4][5] + Si1011[2][3]*T11011[6][5]);
T1011[5][2]=S1110[3][2]*((eff[2].x[3]*Si1011[1][1] - eff[2].x[1]*Si1011[3][1])*T11011[1][3] + (eff[2].x[3]*Si1011[1][2] - eff[2].x[1]*Si1011[3][2])*T11011[2][3]) + S1110[1][2]*((eff[2].x[3]*Si1011[1][2] - eff[2].x[1]*Si1011[3][2])*T11011[2][1] + (eff[2].x[3]*Si1011[1][3] - eff[2].x[1]*Si1011[3][3])*T11011[3][1]) + S1110[2][2]*((eff[2].x[3]*Si1011[1][1] - eff[2].x[1]*Si1011[3][1])*T11011[1][2] + (eff[2].x[3]*Si1011[1][3] - eff[2].x[1]*Si1011[3][3])*T11011[3][2]) + (eff[2].x[3]*S1110[3][1] - eff[2].x[1]*S1110[3][3])*((eff[2].x[3]*Si1011[1][3] - eff[2].x[1]*Si1011[3][3])*T11011[3][6] + Si1011[2][1]*T11011[4][6] + Si1011[2][2]*T11011[5][6]) + (eff[2].x[3]*S1110[1][1] - eff[2].x[1]*S1110[1][3])*((eff[2].x[3]*Si1011[1][1] - eff[2].x[1]*Si1011[3][1])*T11011[1][4] + Si1011[2][2]*T11011[5][4] + Si1011[2][3]*T11011[6][4]) + (eff[2].x[3]*S1110[2][1] - eff[2].x[1]*S1110[2][3])*((eff[2].x[3]*Si1011[1][2] - eff[2].x[1]*Si1011[3][2])*T11011[2][5] + Si1011[2][1]*T11011[4][5] + Si1011[2][3]*T11011[6][5]);
T1011[5][3]=S1110[3][3]*((eff[2].x[3]*Si1011[1][1] - eff[2].x[1]*Si1011[3][1])*T11011[1][3] + (eff[2].x[3]*Si1011[1][2] - eff[2].x[1]*Si1011[3][2])*T11011[2][3]) + S1110[1][3]*((eff[2].x[3]*Si1011[1][2] - eff[2].x[1]*Si1011[3][2])*T11011[2][1] + (eff[2].x[3]*Si1011[1][3] - eff[2].x[1]*Si1011[3][3])*T11011[3][1]) + S1110[2][3]*((eff[2].x[3]*Si1011[1][1] - eff[2].x[1]*Si1011[3][1])*T11011[1][2] + (eff[2].x[3]*Si1011[1][3] - eff[2].x[1]*Si1011[3][3])*T11011[3][2]) + (-(eff[2].x[2]*S1110[3][1]) + eff[2].x[1]*S1110[3][2])*((eff[2].x[3]*Si1011[1][3] - eff[2].x[1]*Si1011[3][3])*T11011[3][6] + Si1011[2][1]*T11011[4][6] + Si1011[2][2]*T11011[5][6]) + (-(eff[2].x[2]*S1110[1][1]) + eff[2].x[1]*S1110[1][2])*((eff[2].x[3]*Si1011[1][1] - eff[2].x[1]*Si1011[3][1])*T11011[1][4] + Si1011[2][2]*T11011[5][4] + Si1011[2][3]*T11011[6][4]) + (-(eff[2].x[2]*S1110[2][1]) + eff[2].x[1]*S1110[2][2])*((eff[2].x[3]*Si1011[1][2] - eff[2].x[1]*Si1011[3][2])*T11011[2][5] + Si1011[2][1]*T11011[4][5] + Si1011[2][3]*T11011[6][5]);
T1011[5][4]=S1110[3][1]*((eff[2].x[3]*Si1011[1][3] - eff[2].x[1]*Si1011[3][3])*T11011[3][6] + Si1011[2][1]*T11011[4][6] + Si1011[2][2]*T11011[5][6]) + S1110[1][1]*((eff[2].x[3]*Si1011[1][1] - eff[2].x[1]*Si1011[3][1])*T11011[1][4] + Si1011[2][2]*T11011[5][4] + Si1011[2][3]*T11011[6][4]) + S1110[2][1]*((eff[2].x[3]*Si1011[1][2] - eff[2].x[1]*Si1011[3][2])*T11011[2][5] + Si1011[2][1]*T11011[4][5] + Si1011[2][3]*T11011[6][5]);
T1011[5][5]=S1110[3][2]*((eff[2].x[3]*Si1011[1][3] - eff[2].x[1]*Si1011[3][3])*T11011[3][6] + Si1011[2][1]*T11011[4][6] + Si1011[2][2]*T11011[5][6]) + S1110[1][2]*((eff[2].x[3]*Si1011[1][1] - eff[2].x[1]*Si1011[3][1])*T11011[1][4] + Si1011[2][2]*T11011[5][4] + Si1011[2][3]*T11011[6][4]) + S1110[2][2]*((eff[2].x[3]*Si1011[1][2] - eff[2].x[1]*Si1011[3][2])*T11011[2][5] + Si1011[2][1]*T11011[4][5] + Si1011[2][3]*T11011[6][5]);
T1011[5][6]=S1110[3][3]*((eff[2].x[3]*Si1011[1][3] - eff[2].x[1]*Si1011[3][3])*T11011[3][6] + Si1011[2][1]*T11011[4][6] + Si1011[2][2]*T11011[5][6]) + S1110[1][3]*((eff[2].x[3]*Si1011[1][1] - eff[2].x[1]*Si1011[3][1])*T11011[1][4] + Si1011[2][2]*T11011[5][4] + Si1011[2][3]*T11011[6][4]) + S1110[2][3]*((eff[2].x[3]*Si1011[1][2] - eff[2].x[1]*Si1011[3][2])*T11011[2][5] + Si1011[2][1]*T11011[4][5] + Si1011[2][3]*T11011[6][5]);

T1011[6][1]=S1110[3][1]*((-(eff[2].x[2]*Si1011[1][1]) + eff[2].x[1]*Si1011[2][1])*T11011[1][3] + (-(eff[2].x[2]*Si1011[1][2]) + eff[2].x[1]*Si1011[2][2])*T11011[2][3]) + S1110[1][1]*((-(eff[2].x[2]*Si1011[1][2]) + eff[2].x[1]*Si1011[2][2])*T11011[2][1] + (-(eff[2].x[2]*Si1011[1][3]) + eff[2].x[1]*Si1011[2][3])*T11011[3][1]) + S1110[2][1]*((-(eff[2].x[2]*Si1011[1][1]) + eff[2].x[1]*Si1011[2][1])*T11011[1][2] + (-(eff[2].x[2]*Si1011[1][3]) + eff[2].x[1]*Si1011[2][3])*T11011[3][2]) + (-(eff[2].x[3]*S1110[3][2]) + eff[2].x[2]*S1110[3][3])*((-(eff[2].x[2]*Si1011[1][3]) + eff[2].x[1]*Si1011[2][3])*T11011[3][6] + Si1011[3][1]*T11011[4][6] + Si1011[3][2]*T11011[5][6]) + (-(eff[2].x[3]*S1110[1][2]) + eff[2].x[2]*S1110[1][3])*((-(eff[2].x[2]*Si1011[1][1]) + eff[2].x[1]*Si1011[2][1])*T11011[1][4] + Si1011[3][2]*T11011[5][4] + Si1011[3][3]*T11011[6][4]) + (-(eff[2].x[3]*S1110[2][2]) + eff[2].x[2]*S1110[2][3])*((-(eff[2].x[2]*Si1011[1][2]) + eff[2].x[1]*Si1011[2][2])*T11011[2][5] + Si1011[3][1]*T11011[4][5] + Si1011[3][3]*T11011[6][5]);
T1011[6][2]=S1110[3][2]*((-(eff[2].x[2]*Si1011[1][1]) + eff[2].x[1]*Si1011[2][1])*T11011[1][3] + (-(eff[2].x[2]*Si1011[1][2]) + eff[2].x[1]*Si1011[2][2])*T11011[2][3]) + S1110[1][2]*((-(eff[2].x[2]*Si1011[1][2]) + eff[2].x[1]*Si1011[2][2])*T11011[2][1] + (-(eff[2].x[2]*Si1011[1][3]) + eff[2].x[1]*Si1011[2][3])*T11011[3][1]) + S1110[2][2]*((-(eff[2].x[2]*Si1011[1][1]) + eff[2].x[1]*Si1011[2][1])*T11011[1][2] + (-(eff[2].x[2]*Si1011[1][3]) + eff[2].x[1]*Si1011[2][3])*T11011[3][2]) + (eff[2].x[3]*S1110[3][1] - eff[2].x[1]*S1110[3][3])*((-(eff[2].x[2]*Si1011[1][3]) + eff[2].x[1]*Si1011[2][3])*T11011[3][6] + Si1011[3][1]*T11011[4][6] + Si1011[3][2]*T11011[5][6]) + (eff[2].x[3]*S1110[1][1] - eff[2].x[1]*S1110[1][3])*((-(eff[2].x[2]*Si1011[1][1]) + eff[2].x[1]*Si1011[2][1])*T11011[1][4] + Si1011[3][2]*T11011[5][4] + Si1011[3][3]*T11011[6][4]) + (eff[2].x[3]*S1110[2][1] - eff[2].x[1]*S1110[2][3])*((-(eff[2].x[2]*Si1011[1][2]) + eff[2].x[1]*Si1011[2][2])*T11011[2][5] + Si1011[3][1]*T11011[4][5] + Si1011[3][3]*T11011[6][5]);
T1011[6][3]=S1110[3][3]*((-(eff[2].x[2]*Si1011[1][1]) + eff[2].x[1]*Si1011[2][1])*T11011[1][3] + (-(eff[2].x[2]*Si1011[1][2]) + eff[2].x[1]*Si1011[2][2])*T11011[2][3]) + S1110[1][3]*((-(eff[2].x[2]*Si1011[1][2]) + eff[2].x[1]*Si1011[2][2])*T11011[2][1] + (-(eff[2].x[2]*Si1011[1][3]) + eff[2].x[1]*Si1011[2][3])*T11011[3][1]) + S1110[2][3]*((-(eff[2].x[2]*Si1011[1][1]) + eff[2].x[1]*Si1011[2][1])*T11011[1][2] + (-(eff[2].x[2]*Si1011[1][3]) + eff[2].x[1]*Si1011[2][3])*T11011[3][2]) + (-(eff[2].x[2]*S1110[3][1]) + eff[2].x[1]*S1110[3][2])*((-(eff[2].x[2]*Si1011[1][3]) + eff[2].x[1]*Si1011[2][3])*T11011[3][6] + Si1011[3][1]*T11011[4][6] + Si1011[3][2]*T11011[5][6]) + (-(eff[2].x[2]*S1110[1][1]) + eff[2].x[1]*S1110[1][2])*((-(eff[2].x[2]*Si1011[1][1]) + eff[2].x[1]*Si1011[2][1])*T11011[1][4] + Si1011[3][2]*T11011[5][4] + Si1011[3][3]*T11011[6][4]) + (-(eff[2].x[2]*S1110[2][1]) + eff[2].x[1]*S1110[2][2])*((-(eff[2].x[2]*Si1011[1][2]) + eff[2].x[1]*Si1011[2][2])*T11011[2][5] + Si1011[3][1]*T11011[4][5] + Si1011[3][3]*T11011[6][5]);
T1011[6][4]=S1110[3][1]*((-(eff[2].x[2]*Si1011[1][3]) + eff[2].x[1]*Si1011[2][3])*T11011[3][6] + Si1011[3][1]*T11011[4][6] + Si1011[3][2]*T11011[5][6]) + S1110[1][1]*((-(eff[2].x[2]*Si1011[1][1]) + eff[2].x[1]*Si1011[2][1])*T11011[1][4] + Si1011[3][2]*T11011[5][4] + Si1011[3][3]*T11011[6][4]) + S1110[2][1]*((-(eff[2].x[2]*Si1011[1][2]) + eff[2].x[1]*Si1011[2][2])*T11011[2][5] + Si1011[3][1]*T11011[4][5] + Si1011[3][3]*T11011[6][5]);
T1011[6][5]=S1110[3][2]*((-(eff[2].x[2]*Si1011[1][3]) + eff[2].x[1]*Si1011[2][3])*T11011[3][6] + Si1011[3][1]*T11011[4][6] + Si1011[3][2]*T11011[5][6]) + S1110[1][2]*((-(eff[2].x[2]*Si1011[1][1]) + eff[2].x[1]*Si1011[2][1])*T11011[1][4] + Si1011[3][2]*T11011[5][4] + Si1011[3][3]*T11011[6][4]) + S1110[2][2]*((-(eff[2].x[2]*Si1011[1][2]) + eff[2].x[1]*Si1011[2][2])*T11011[2][5] + Si1011[3][1]*T11011[4][5] + Si1011[3][3]*T11011[6][5]);
T1011[6][6]=S1110[3][3]*((-(eff[2].x[2]*Si1011[1][3]) + eff[2].x[1]*Si1011[2][3])*T11011[3][6] + Si1011[3][1]*T11011[4][6] + Si1011[3][2]*T11011[5][6]) + S1110[1][3]*((-(eff[2].x[2]*Si1011[1][1]) + eff[2].x[1]*Si1011[2][1])*T11011[1][4] + Si1011[3][2]*T11011[5][4] + Si1011[3][3]*T11011[6][4]) + S1110[2][3]*((-(eff[2].x[2]*Si1011[1][2]) + eff[2].x[1]*Si1011[2][2])*T11011[2][5] + Si1011[3][1]*T11011[4][5] + Si1011[3][3]*T11011[6][5]);



}


void
hermes_InvDynArtfunc94(void)
      {
JA10[1][1]=T1011[1][1] + T1012[1][1] + T1016[1][1] + T1020[1][1] + T1024[1][1] + T1028[1][1];
JA10[1][2]=links[7].mcm[3] + T1011[1][2] + T1012[1][2] + T1016[1][2] + T1020[1][2] + T1024[1][2] + T1028[1][2];
JA10[1][3]=-links[7].mcm[2] + T1011[1][3] + T1012[1][3] + T1016[1][3] + T1020[1][3] + T1024[1][3] + T1028[1][3];
JA10[1][4]=links[7].m + T1011[1][4] + T1012[1][4] + T1016[1][4] + T1020[1][4] + T1024[1][4] + T1028[1][4];
JA10[1][5]=T1011[1][5] + T1012[1][5] + T1016[1][5] + T1020[1][5] + T1024[1][5] + T1028[1][5];
JA10[1][6]=T1011[1][6] + T1012[1][6] + T1016[1][6] + T1020[1][6] + T1024[1][6] + T1028[1][6];

JA10[2][1]=-links[7].mcm[3] + T1011[2][1] + T1012[2][1] + T1016[2][1] + T1020[2][1] + T1024[2][1] + T1028[2][1];
JA10[2][2]=T1011[2][2] + T1012[2][2] + T1016[2][2] + T1020[2][2] + T1024[2][2] + T1028[2][2];
JA10[2][3]=links[7].mcm[1] + T1011[2][3] + T1012[2][3] + T1016[2][3] + T1020[2][3] + T1024[2][3] + T1028[2][3];
JA10[2][4]=T1011[2][4] + T1012[2][4] + T1016[2][4] + T1020[2][4] + T1024[2][4] + T1028[2][4];
JA10[2][5]=links[7].m + T1011[2][5] + T1012[2][5] + T1016[2][5] + T1020[2][5] + T1024[2][5] + T1028[2][5];
JA10[2][6]=T1011[2][6] + T1012[2][6] + T1016[2][6] + T1020[2][6] + T1024[2][6] + T1028[2][6];

JA10[3][1]=links[7].mcm[2] + T1011[3][1] + T1012[3][1] + T1016[3][1] + T1020[3][1] + T1024[3][1] + T1028[3][1];
JA10[3][2]=-links[7].mcm[1] + T1011[3][2] + T1012[3][2] + T1016[3][2] + T1020[3][2] + T1024[3][2] + T1028[3][2];
JA10[3][3]=T1011[3][3] + T1012[3][3] + T1016[3][3] + T1020[3][3] + T1024[3][3] + T1028[3][3];
JA10[3][4]=T1011[3][4] + T1012[3][4] + T1016[3][4] + T1020[3][4] + T1024[3][4] + T1028[3][4];
JA10[3][5]=T1011[3][5] + T1012[3][5] + T1016[3][5] + T1020[3][5] + T1024[3][5] + T1028[3][5];
JA10[3][6]=links[7].m + T1011[3][6] + T1012[3][6] + T1016[3][6] + T1020[3][6] + T1024[3][6] + T1028[3][6];

JA10[4][1]=links[7].inertia[1][1] + T1011[4][1] + T1012[4][1] + T1016[4][1] + T1020[4][1] + T1024[4][1] + T1028[4][1];
JA10[4][2]=links[7].inertia[1][2] + T1011[4][2] + T1012[4][2] + T1016[4][2] + T1020[4][2] + T1024[4][2] + T1028[4][2];
JA10[4][3]=links[7].inertia[1][3] + T1011[4][3] + T1012[4][3] + T1016[4][3] + T1020[4][3] + T1024[4][3] + T1028[4][3];
JA10[4][4]=T1011[4][4] + T1012[4][4] + T1016[4][4] + T1020[4][4] + T1024[4][4] + T1028[4][4];
JA10[4][5]=-links[7].mcm[3] + T1011[4][5] + T1012[4][5] + T1016[4][5] + T1020[4][5] + T1024[4][5] + T1028[4][5];
JA10[4][6]=links[7].mcm[2] + T1011[4][6] + T1012[4][6] + T1016[4][6] + T1020[4][6] + T1024[4][6] + T1028[4][6];

JA10[5][1]=links[7].inertia[1][2] + T1011[5][1] + T1012[5][1] + T1016[5][1] + T1020[5][1] + T1024[5][1] + T1028[5][1];
JA10[5][2]=links[7].inertia[2][2] + T1011[5][2] + T1012[5][2] + T1016[5][2] + T1020[5][2] + T1024[5][2] + T1028[5][2];
JA10[5][3]=links[7].inertia[2][3] + T1011[5][3] + T1012[5][3] + T1016[5][3] + T1020[5][3] + T1024[5][3] + T1028[5][3];
JA10[5][4]=links[7].mcm[3] + T1011[5][4] + T1012[5][4] + T1016[5][4] + T1020[5][4] + T1024[5][4] + T1028[5][4];
JA10[5][5]=T1011[5][5] + T1012[5][5] + T1016[5][5] + T1020[5][5] + T1024[5][5] + T1028[5][5];
JA10[5][6]=-links[7].mcm[1] + T1011[5][6] + T1012[5][6] + T1016[5][6] + T1020[5][6] + T1024[5][6] + T1028[5][6];

JA10[6][1]=links[7].inertia[1][3] + T1011[6][1] + T1012[6][1] + T1016[6][1] + T1020[6][1] + T1024[6][1] + T1028[6][1];
JA10[6][2]=links[7].inertia[2][3] + T1011[6][2] + T1012[6][2] + T1016[6][2] + T1020[6][2] + T1024[6][2] + T1028[6][2];
JA10[6][3]=links[7].inertia[3][3] + T1011[6][3] + T1012[6][3] + T1016[6][3] + T1020[6][3] + T1024[6][3] + T1028[6][3];
JA10[6][4]=-links[7].mcm[2] + T1011[6][4] + T1012[6][4] + T1016[6][4] + T1020[6][4] + T1024[6][4] + T1028[6][4];
JA10[6][5]=links[7].mcm[1] + T1011[6][5] + T1012[6][5] + T1016[6][5] + T1020[6][5] + T1024[6][5] + T1028[6][5];
JA10[6][6]=T1011[6][6] + T1012[6][6] + T1016[6][6] + T1020[6][6] + T1024[6][6] + T1028[6][6];


h10[1]=JA10[1][3];
h10[2]=JA10[2][3];
h10[3]=JA10[3][3];
h10[4]=JA10[4][3];
h10[5]=JA10[5][3];
h10[6]=JA10[6][3];

T1910[1][1]=JA10[1][1];
T1910[1][2]=JA10[1][2];
T1910[1][3]=JA10[1][3];
T1910[1][4]=JA10[1][4];
T1910[1][5]=JA10[1][5];
T1910[1][6]=JA10[1][6];

T1910[2][1]=JA10[2][1];
T1910[2][2]=JA10[2][2];
T1910[2][3]=JA10[2][3];
T1910[2][4]=JA10[2][4];
T1910[2][5]=JA10[2][5];
T1910[2][6]=JA10[2][6];

T1910[3][1]=JA10[3][1];
T1910[3][2]=JA10[3][2];
T1910[3][3]=JA10[3][3];
T1910[3][4]=JA10[3][4];
T1910[3][5]=JA10[3][5];
T1910[3][6]=JA10[3][6];

T1910[4][1]=JA10[4][1];
T1910[4][2]=JA10[4][2];
T1910[4][3]=JA10[4][3];
T1910[4][4]=JA10[4][4];
T1910[4][5]=JA10[4][5];
T1910[4][6]=JA10[4][6];

T1910[5][1]=JA10[5][1];
T1910[5][2]=JA10[5][2];
T1910[5][3]=JA10[5][3];
T1910[5][4]=JA10[5][4];
T1910[5][5]=JA10[5][5];
T1910[5][6]=JA10[5][6];

T1910[6][1]=JA10[6][1];
T1910[6][2]=JA10[6][2];
T1910[6][3]=JA10[6][3];
T1910[6][4]=JA10[6][4];
T1910[6][5]=JA10[6][5];
T1910[6][6]=JA10[6][6];


T910[1][1]=S109[1][1]*(Si910[1][1]*T1910[1][1] + Si910[1][2]*T1910[2][1]) + S109[2][1]*(Si910[1][1]*T1910[1][2] + Si910[1][2]*T1910[2][2]);
T910[1][2]=-(Si910[1][1]*T1910[1][3]) - Si910[1][2]*T1910[2][3];
T910[1][3]=S109[1][3]*(Si910[1][1]*T1910[1][1] + Si910[1][2]*T1910[2][1]) + S109[2][3]*(Si910[1][1]*T1910[1][2] + Si910[1][2]*T1910[2][2]);
T910[1][4]=S109[1][1]*(Si910[1][1]*T1910[1][4] + Si910[1][2]*T1910[2][4]) + S109[2][1]*(Si910[1][1]*T1910[1][5] + Si910[1][2]*T1910[2][5]);
T910[1][5]=-(Si910[1][1]*T1910[1][6]) - Si910[1][2]*T1910[2][6];
T910[1][6]=S109[1][3]*(Si910[1][1]*T1910[1][4] + Si910[1][2]*T1910[2][4]) + S109[2][3]*(Si910[1][1]*T1910[1][5] + Si910[1][2]*T1910[2][5]);

T910[2][1]=-(S109[1][1]*T1910[3][1]) - S109[2][1]*T1910[3][2];
T910[2][2]=T1910[3][3];
T910[2][3]=-(S109[1][3]*T1910[3][1]) - S109[2][3]*T1910[3][2];
T910[2][4]=-(S109[1][1]*T1910[3][4]) - S109[2][1]*T1910[3][5];
T910[2][5]=T1910[3][6];
T910[2][6]=-(S109[1][3]*T1910[3][4]) - S109[2][3]*T1910[3][5];

T910[3][1]=S109[1][1]*(Si910[3][1]*T1910[1][1] + Si910[3][2]*T1910[2][1]) + S109[2][1]*(Si910[3][1]*T1910[1][2] + Si910[3][2]*T1910[2][2]);
T910[3][2]=-(Si910[3][1]*T1910[1][3]) - Si910[3][2]*T1910[2][3];
T910[3][3]=S109[1][3]*(Si910[3][1]*T1910[1][1] + Si910[3][2]*T1910[2][1]) + S109[2][3]*(Si910[3][1]*T1910[1][2] + Si910[3][2]*T1910[2][2]);
T910[3][4]=S109[1][1]*(Si910[3][1]*T1910[1][4] + Si910[3][2]*T1910[2][4]) + S109[2][1]*(Si910[3][1]*T1910[1][5] + Si910[3][2]*T1910[2][5]);
T910[3][5]=-(Si910[3][1]*T1910[1][6]) - Si910[3][2]*T1910[2][6];
T910[3][6]=S109[1][3]*(Si910[3][1]*T1910[1][4] + Si910[3][2]*T1910[2][4]) + S109[2][3]*(Si910[3][1]*T1910[1][5] + Si910[3][2]*T1910[2][5]);

T910[4][1]=S109[1][1]*(Si910[1][1]*T1910[4][1] + Si910[1][2]*T1910[5][1]) + S109[2][1]*(Si910[1][1]*T1910[4][2] + Si910[1][2]*T1910[5][2]);
T910[4][2]=-(Si910[1][1]*T1910[4][3]) - Si910[1][2]*T1910[5][3];
T910[4][3]=S109[1][3]*(Si910[1][1]*T1910[4][1] + Si910[1][2]*T1910[5][1]) + S109[2][3]*(Si910[1][1]*T1910[4][2] + Si910[1][2]*T1910[5][2]);
T910[4][4]=S109[1][1]*(Si910[1][1]*T1910[4][4] + Si910[1][2]*T1910[5][4]) + S109[2][1]*(Si910[1][1]*T1910[4][5] + Si910[1][2]*T1910[5][5]);
T910[4][5]=-(Si910[1][1]*T1910[4][6]) - Si910[1][2]*T1910[5][6];
T910[4][6]=S109[1][3]*(Si910[1][1]*T1910[4][4] + Si910[1][2]*T1910[5][4]) + S109[2][3]*(Si910[1][1]*T1910[4][5] + Si910[1][2]*T1910[5][5]);

T910[5][1]=-(S109[1][1]*T1910[6][1]) - S109[2][1]*T1910[6][2];
T910[5][2]=T1910[6][3];
T910[5][3]=-(S109[1][3]*T1910[6][1]) - S109[2][3]*T1910[6][2];
T910[5][4]=-(S109[1][1]*T1910[6][4]) - S109[2][1]*T1910[6][5];
T910[5][5]=T1910[6][6];
T910[5][6]=-(S109[1][3]*T1910[6][4]) - S109[2][3]*T1910[6][5];

T910[6][1]=S109[1][1]*(Si910[3][1]*T1910[4][1] + Si910[3][2]*T1910[5][1]) + S109[2][1]*(Si910[3][1]*T1910[4][2] + Si910[3][2]*T1910[5][2]);
T910[6][2]=-(Si910[3][1]*T1910[4][3]) - Si910[3][2]*T1910[5][3];
T910[6][3]=S109[1][3]*(Si910[3][1]*T1910[4][1] + Si910[3][2]*T1910[5][1]) + S109[2][3]*(Si910[3][1]*T1910[4][2] + Si910[3][2]*T1910[5][2]);
T910[6][4]=S109[1][1]*(Si910[3][1]*T1910[4][4] + Si910[3][2]*T1910[5][4]) + S109[2][1]*(Si910[3][1]*T1910[4][5] + Si910[3][2]*T1910[5][5]);
T910[6][5]=-(Si910[3][1]*T1910[4][6]) - Si910[3][2]*T1910[5][6];
T910[6][6]=S109[1][3]*(Si910[3][1]*T1910[4][4] + Si910[3][2]*T1910[5][4]) + S109[2][3]*(Si910[3][1]*T1910[4][5] + Si910[3][2]*T1910[5][5]);



}


void
hermes_InvDynArtfunc95(void)
      {
JA9[1][1]=T910[1][1];
JA9[1][2]=links[6].mcm[3] + T910[1][2];
JA9[1][3]=-links[6].mcm[2] + T910[1][3];
JA9[1][4]=links[6].m + T910[1][4];
JA9[1][5]=T910[1][5];
JA9[1][6]=T910[1][6];

JA9[2][1]=-links[6].mcm[3] + T910[2][1];
JA9[2][2]=T910[2][2];
JA9[2][3]=links[6].mcm[1] + T910[2][3];
JA9[2][4]=T910[2][4];
JA9[2][5]=links[6].m + T910[2][5];
JA9[2][6]=T910[2][6];

JA9[3][1]=links[6].mcm[2] + T910[3][1];
JA9[3][2]=-links[6].mcm[1] + T910[3][2];
JA9[3][3]=T910[3][3];
JA9[3][4]=T910[3][4];
JA9[3][5]=T910[3][5];
JA9[3][6]=links[6].m + T910[3][6];

JA9[4][1]=links[6].inertia[1][1] + T910[4][1];
JA9[4][2]=links[6].inertia[1][2] + T910[4][2];
JA9[4][3]=links[6].inertia[1][3] + T910[4][3];
JA9[4][4]=T910[4][4];
JA9[4][5]=-links[6].mcm[3] + T910[4][5];
JA9[4][6]=links[6].mcm[2] + T910[4][6];

JA9[5][1]=links[6].inertia[1][2] + T910[5][1];
JA9[5][2]=links[6].inertia[2][2] + T910[5][2];
JA9[5][3]=links[6].inertia[2][3] + T910[5][3];
JA9[5][4]=links[6].mcm[3] + T910[5][4];
JA9[5][5]=T910[5][5];
JA9[5][6]=-links[6].mcm[1] + T910[5][6];

JA9[6][1]=links[6].inertia[1][3] + T910[6][1];
JA9[6][2]=links[6].inertia[2][3] + T910[6][2];
JA9[6][3]=links[6].inertia[3][3] + T910[6][3];
JA9[6][4]=-links[6].mcm[2] + T910[6][4];
JA9[6][5]=links[6].mcm[1] + T910[6][5];
JA9[6][6]=T910[6][6];


h9[1]=JA9[1][3];
h9[2]=JA9[2][3];
h9[3]=JA9[3][3];
h9[4]=JA9[4][3];
h9[5]=JA9[5][3];
h9[6]=JA9[6][3];

T189[1][1]=JA9[1][1];
T189[1][2]=JA9[1][2];
T189[1][3]=JA9[1][3];
T189[1][4]=JA9[1][4];
T189[1][5]=JA9[1][5];
T189[1][6]=JA9[1][6];

T189[2][1]=JA9[2][1];
T189[2][2]=JA9[2][2];
T189[2][3]=JA9[2][3];
T189[2][4]=JA9[2][4];
T189[2][5]=JA9[2][5];
T189[2][6]=JA9[2][6];

T189[3][1]=JA9[3][1];
T189[3][2]=JA9[3][2];
T189[3][3]=JA9[3][3];
T189[3][4]=JA9[3][4];
T189[3][5]=JA9[3][5];
T189[3][6]=JA9[3][6];

T189[4][1]=JA9[4][1];
T189[4][2]=JA9[4][2];
T189[4][3]=JA9[4][3];
T189[4][4]=JA9[4][4];
T189[4][5]=JA9[4][5];
T189[4][6]=JA9[4][6];

T189[5][1]=JA9[5][1];
T189[5][2]=JA9[5][2];
T189[5][3]=JA9[5][3];
T189[5][4]=JA9[5][4];
T189[5][5]=JA9[5][5];
T189[5][6]=JA9[5][6];

T189[6][1]=JA9[6][1];
T189[6][2]=JA9[6][2];
T189[6][3]=JA9[6][3];
T189[6][4]=JA9[6][4];
T189[6][5]=JA9[6][5];
T189[6][6]=JA9[6][6];


T89[1][1]=T189[3][3] + (LOWERARM*S98[1][2] + WRISTY*S98[1][3])*T189[3][4] + (LOWERARM*S98[2][2] + WRISTY*S98[2][3])*T189[3][5];
T89[1][2]=S98[1][2]*T189[3][1] + S98[2][2]*T189[3][2] - LOWERARM*T189[3][6];
T89[1][3]=S98[1][3]*T189[3][1] + S98[2][3]*T189[3][2] - WRISTY*T189[3][6];
T89[1][4]=T189[3][6];
T89[1][5]=S98[1][2]*T189[3][4] + S98[2][2]*T189[3][5];
T89[1][6]=S98[1][3]*T189[3][4] + S98[2][3]*T189[3][5];

T89[2][1]=Si89[2][1]*T189[1][3] + Si89[2][2]*T189[2][3] + (LOWERARM*S98[1][2] + WRISTY*S98[1][3])*(Si89[2][1]*T189[1][4] + Si89[2][2]*T189[2][4]) + (LOWERARM*S98[2][2] + WRISTY*S98[2][3])*(Si89[2][1]*T189[1][5] + Si89[2][2]*T189[2][5]);
T89[2][2]=S98[1][2]*(Si89[2][1]*T189[1][1] + Si89[2][2]*T189[2][1]) + S98[2][2]*(Si89[2][1]*T189[1][2] + Si89[2][2]*T189[2][2]) - LOWERARM*(Si89[2][1]*T189[1][6] + Si89[2][2]*T189[2][6]);
T89[2][3]=S98[1][3]*(Si89[2][1]*T189[1][1] + Si89[2][2]*T189[2][1]) + S98[2][3]*(Si89[2][1]*T189[1][2] + Si89[2][2]*T189[2][2]) - WRISTY*(Si89[2][1]*T189[1][6] + Si89[2][2]*T189[2][6]);
T89[2][4]=Si89[2][1]*T189[1][6] + Si89[2][2]*T189[2][6];
T89[2][5]=S98[1][2]*(Si89[2][1]*T189[1][4] + Si89[2][2]*T189[2][4]) + S98[2][2]*(Si89[2][1]*T189[1][5] + Si89[2][2]*T189[2][5]);
T89[2][6]=S98[1][3]*(Si89[2][1]*T189[1][4] + Si89[2][2]*T189[2][4]) + S98[2][3]*(Si89[2][1]*T189[1][5] + Si89[2][2]*T189[2][5]);

T89[3][1]=Si89[3][1]*T189[1][3] + Si89[3][2]*T189[2][3] + (LOWERARM*S98[1][2] + WRISTY*S98[1][3])*(Si89[3][1]*T189[1][4] + Si89[3][2]*T189[2][4]) + (LOWERARM*S98[2][2] + WRISTY*S98[2][3])*(Si89[3][1]*T189[1][5] + Si89[3][2]*T189[2][5]);
T89[3][2]=S98[1][2]*(Si89[3][1]*T189[1][1] + Si89[3][2]*T189[2][1]) + S98[2][2]*(Si89[3][1]*T189[1][2] + Si89[3][2]*T189[2][2]) - LOWERARM*(Si89[3][1]*T189[1][6] + Si89[3][2]*T189[2][6]);
T89[3][3]=S98[1][3]*(Si89[3][1]*T189[1][1] + Si89[3][2]*T189[2][1]) + S98[2][3]*(Si89[3][1]*T189[1][2] + Si89[3][2]*T189[2][2]) - WRISTY*(Si89[3][1]*T189[1][6] + Si89[3][2]*T189[2][6]);
T89[3][4]=Si89[3][1]*T189[1][6] + Si89[3][2]*T189[2][6];
T89[3][5]=S98[1][2]*(Si89[3][1]*T189[1][4] + Si89[3][2]*T189[2][4]) + S98[2][2]*(Si89[3][1]*T189[1][5] + Si89[3][2]*T189[2][5]);
T89[3][6]=S98[1][3]*(Si89[3][1]*T189[1][4] + Si89[3][2]*T189[2][4]) + S98[2][3]*(Si89[3][1]*T189[1][5] + Si89[3][2]*T189[2][5]);

T89[4][1]=(LOWERARM*Si89[2][1] + WRISTY*Si89[3][1])*T189[1][3] + (LOWERARM*Si89[2][2] + WRISTY*Si89[3][2])*T189[2][3] + T189[6][3] + (LOWERARM*S98[1][2] + WRISTY*S98[1][3])*((LOWERARM*Si89[2][1] + WRISTY*Si89[3][1])*T189[1][4] + (LOWERARM*Si89[2][2] + WRISTY*Si89[3][2])*T189[2][4] + T189[6][4]) + (LOWERARM*S98[2][2] + WRISTY*S98[2][3])*((LOWERARM*Si89[2][1] + WRISTY*Si89[3][1])*T189[1][5] + (LOWERARM*Si89[2][2] + WRISTY*Si89[3][2])*T189[2][5] + T189[6][5]);
T89[4][2]=S98[1][2]*((LOWERARM*Si89[2][1] + WRISTY*Si89[3][1])*T189[1][1] + (LOWERARM*Si89[2][2] + WRISTY*Si89[3][2])*T189[2][1] + T189[6][1]) + S98[2][2]*((LOWERARM*Si89[2][1] + WRISTY*Si89[3][1])*T189[1][2] + (LOWERARM*Si89[2][2] + WRISTY*Si89[3][2])*T189[2][2] + T189[6][2]) - LOWERARM*((LOWERARM*Si89[2][1] + WRISTY*Si89[3][1])*T189[1][6] + (LOWERARM*Si89[2][2] + WRISTY*Si89[3][2])*T189[2][6] + T189[6][6]);
T89[4][3]=S98[1][3]*((LOWERARM*Si89[2][1] + WRISTY*Si89[3][1])*T189[1][1] + (LOWERARM*Si89[2][2] + WRISTY*Si89[3][2])*T189[2][1] + T189[6][1]) + S98[2][3]*((LOWERARM*Si89[2][1] + WRISTY*Si89[3][1])*T189[1][2] + (LOWERARM*Si89[2][2] + WRISTY*Si89[3][2])*T189[2][2] + T189[6][2]) - WRISTY*((LOWERARM*Si89[2][1] + WRISTY*Si89[3][1])*T189[1][6] + (LOWERARM*Si89[2][2] + WRISTY*Si89[3][2])*T189[2][6] + T189[6][6]);
T89[4][4]=(LOWERARM*Si89[2][1] + WRISTY*Si89[3][1])*T189[1][6] + (LOWERARM*Si89[2][2] + WRISTY*Si89[3][2])*T189[2][6] + T189[6][6];
T89[4][5]=S98[1][2]*((LOWERARM*Si89[2][1] + WRISTY*Si89[3][1])*T189[1][4] + (LOWERARM*Si89[2][2] + WRISTY*Si89[3][2])*T189[2][4] + T189[6][4]) + S98[2][2]*((LOWERARM*Si89[2][1] + WRISTY*Si89[3][1])*T189[1][5] + (LOWERARM*Si89[2][2] + WRISTY*Si89[3][2])*T189[2][5] + T189[6][5]);
T89[4][6]=S98[1][3]*((LOWERARM*Si89[2][1] + WRISTY*Si89[3][1])*T189[1][4] + (LOWERARM*Si89[2][2] + WRISTY*Si89[3][2])*T189[2][4] + T189[6][4]) + S98[2][3]*((LOWERARM*Si89[2][1] + WRISTY*Si89[3][1])*T189[1][5] + (LOWERARM*Si89[2][2] + WRISTY*Si89[3][2])*T189[2][5] + T189[6][5]);

T89[5][1]=-(LOWERARM*T189[3][3]) + Si89[2][1]*T189[4][3] + Si89[2][2]*T189[5][3] + (LOWERARM*S98[1][2] + WRISTY*S98[1][3])*(-(LOWERARM*T189[3][4]) + Si89[2][1]*T189[4][4] + Si89[2][2]*T189[5][4]) + (LOWERARM*S98[2][2] + WRISTY*S98[2][3])*(-(LOWERARM*T189[3][5]) + Si89[2][1]*T189[4][5] + Si89[2][2]*T189[5][5]);
T89[5][2]=S98[1][2]*(-(LOWERARM*T189[3][1]) + Si89[2][1]*T189[4][1] + Si89[2][2]*T189[5][1]) + S98[2][2]*(-(LOWERARM*T189[3][2]) + Si89[2][1]*T189[4][2] + Si89[2][2]*T189[5][2]) - LOWERARM*(-(LOWERARM*T189[3][6]) + Si89[2][1]*T189[4][6] + Si89[2][2]*T189[5][6]);
T89[5][3]=S98[1][3]*(-(LOWERARM*T189[3][1]) + Si89[2][1]*T189[4][1] + Si89[2][2]*T189[5][1]) + S98[2][3]*(-(LOWERARM*T189[3][2]) + Si89[2][1]*T189[4][2] + Si89[2][2]*T189[5][2]) - WRISTY*(-(LOWERARM*T189[3][6]) + Si89[2][1]*T189[4][6] + Si89[2][2]*T189[5][6]);
T89[5][4]=-(LOWERARM*T189[3][6]) + Si89[2][1]*T189[4][6] + Si89[2][2]*T189[5][6];
T89[5][5]=S98[1][2]*(-(LOWERARM*T189[3][4]) + Si89[2][1]*T189[4][4] + Si89[2][2]*T189[5][4]) + S98[2][2]*(-(LOWERARM*T189[3][5]) + Si89[2][1]*T189[4][5] + Si89[2][2]*T189[5][5]);
T89[5][6]=S98[1][3]*(-(LOWERARM*T189[3][4]) + Si89[2][1]*T189[4][4] + Si89[2][2]*T189[5][4]) + S98[2][3]*(-(LOWERARM*T189[3][5]) + Si89[2][1]*T189[4][5] + Si89[2][2]*T189[5][5]);

T89[6][1]=-(WRISTY*T189[3][3]) + Si89[3][1]*T189[4][3] + Si89[3][2]*T189[5][3] + (LOWERARM*S98[1][2] + WRISTY*S98[1][3])*(-(WRISTY*T189[3][4]) + Si89[3][1]*T189[4][4] + Si89[3][2]*T189[5][4]) + (LOWERARM*S98[2][2] + WRISTY*S98[2][3])*(-(WRISTY*T189[3][5]) + Si89[3][1]*T189[4][5] + Si89[3][2]*T189[5][5]);
T89[6][2]=S98[1][2]*(-(WRISTY*T189[3][1]) + Si89[3][1]*T189[4][1] + Si89[3][2]*T189[5][1]) + S98[2][2]*(-(WRISTY*T189[3][2]) + Si89[3][1]*T189[4][2] + Si89[3][2]*T189[5][2]) - LOWERARM*(-(WRISTY*T189[3][6]) + Si89[3][1]*T189[4][6] + Si89[3][2]*T189[5][6]);
T89[6][3]=S98[1][3]*(-(WRISTY*T189[3][1]) + Si89[3][1]*T189[4][1] + Si89[3][2]*T189[5][1]) + S98[2][3]*(-(WRISTY*T189[3][2]) + Si89[3][1]*T189[4][2] + Si89[3][2]*T189[5][2]) - WRISTY*(-(WRISTY*T189[3][6]) + Si89[3][1]*T189[4][6] + Si89[3][2]*T189[5][6]);
T89[6][4]=-(WRISTY*T189[3][6]) + Si89[3][1]*T189[4][6] + Si89[3][2]*T189[5][6];
T89[6][5]=S98[1][2]*(-(WRISTY*T189[3][4]) + Si89[3][1]*T189[4][4] + Si89[3][2]*T189[5][4]) + S98[2][2]*(-(WRISTY*T189[3][5]) + Si89[3][1]*T189[4][5] + Si89[3][2]*T189[5][5]);
T89[6][6]=S98[1][3]*(-(WRISTY*T189[3][4]) + Si89[3][1]*T189[4][4] + Si89[3][2]*T189[5][4]) + S98[2][3]*(-(WRISTY*T189[3][5]) + Si89[3][1]*T189[4][5] + Si89[3][2]*T189[5][5]);



}


void
hermes_InvDynArtfunc96(void)
      {
JA8[1][1]=T89[1][1];
JA8[1][2]=links[5].mcm[3] + T89[1][2];
JA8[1][3]=-links[5].mcm[2] + T89[1][3];
JA8[1][4]=links[5].m + T89[1][4];
JA8[1][5]=T89[1][5];
JA8[1][6]=T89[1][6];

JA8[2][1]=-links[5].mcm[3] + T89[2][1];
JA8[2][2]=T89[2][2];
JA8[2][3]=links[5].mcm[1] + T89[2][3];
JA8[2][4]=T89[2][4];
JA8[2][5]=links[5].m + T89[2][5];
JA8[2][6]=T89[2][6];

JA8[3][1]=links[5].mcm[2] + T89[3][1];
JA8[3][2]=-links[5].mcm[1] + T89[3][2];
JA8[3][3]=T89[3][3];
JA8[3][4]=T89[3][4];
JA8[3][5]=T89[3][5];
JA8[3][6]=links[5].m + T89[3][6];

JA8[4][1]=links[5].inertia[1][1] + T89[4][1];
JA8[4][2]=links[5].inertia[1][2] + T89[4][2];
JA8[4][3]=links[5].inertia[1][3] + T89[4][3];
JA8[4][4]=T89[4][4];
JA8[4][5]=-links[5].mcm[3] + T89[4][5];
JA8[4][6]=links[5].mcm[2] + T89[4][6];

JA8[5][1]=links[5].inertia[1][2] + T89[5][1];
JA8[5][2]=links[5].inertia[2][2] + T89[5][2];
JA8[5][3]=links[5].inertia[2][3] + T89[5][3];
JA8[5][4]=links[5].mcm[3] + T89[5][4];
JA8[5][5]=T89[5][5];
JA8[5][6]=-links[5].mcm[1] + T89[5][6];

JA8[6][1]=links[5].inertia[1][3] + T89[6][1];
JA8[6][2]=links[5].inertia[2][3] + T89[6][2];
JA8[6][3]=links[5].inertia[3][3] + T89[6][3];
JA8[6][4]=-links[5].mcm[2] + T89[6][4];
JA8[6][5]=links[5].mcm[1] + T89[6][5];
JA8[6][6]=T89[6][6];


h8[1]=JA8[1][3];
h8[2]=JA8[2][3];
h8[3]=JA8[3][3];
h8[4]=JA8[4][3];
h8[5]=JA8[5][3];
h8[6]=JA8[6][3];

T178[1][1]=JA8[1][1];
T178[1][2]=JA8[1][2];
T178[1][3]=JA8[1][3];
T178[1][4]=JA8[1][4];
T178[1][5]=JA8[1][5];
T178[1][6]=JA8[1][6];

T178[2][1]=JA8[2][1];
T178[2][2]=JA8[2][2];
T178[2][3]=JA8[2][3];
T178[2][4]=JA8[2][4];
T178[2][5]=JA8[2][5];
T178[2][6]=JA8[2][6];

T178[3][1]=JA8[3][1];
T178[3][2]=JA8[3][2];
T178[3][3]=JA8[3][3];
T178[3][4]=JA8[3][4];
T178[3][5]=JA8[3][5];
T178[3][6]=JA8[3][6];

T178[4][1]=JA8[4][1];
T178[4][2]=JA8[4][2];
T178[4][3]=JA8[4][3];
T178[4][4]=JA8[4][4];
T178[4][5]=JA8[4][5];
T178[4][6]=JA8[4][6];

T178[5][1]=JA8[5][1];
T178[5][2]=JA8[5][2];
T178[5][3]=JA8[5][3];
T178[5][4]=JA8[5][4];
T178[5][5]=JA8[5][5];
T178[5][6]=JA8[5][6];

T178[6][1]=JA8[6][1];
T178[6][2]=JA8[6][2];
T178[6][3]=JA8[6][3];
T178[6][4]=JA8[6][4];
T178[6][5]=JA8[6][5];
T178[6][6]=JA8[6][6];


T78[1][1]=S87[1][1]*(Si78[1][1]*T178[1][1] + Si78[1][2]*T178[2][1]) + S87[2][1]*(Si78[1][1]*T178[1][2] + Si78[1][2]*T178[2][2]);
T78[1][2]=Si78[1][1]*T178[1][3] + Si78[1][2]*T178[2][3];
T78[1][3]=S87[1][3]*(Si78[1][1]*T178[1][1] + Si78[1][2]*T178[2][1]) + S87[2][3]*(Si78[1][1]*T178[1][2] + Si78[1][2]*T178[2][2]);
T78[1][4]=S87[1][1]*(Si78[1][1]*T178[1][4] + Si78[1][2]*T178[2][4]) + S87[2][1]*(Si78[1][1]*T178[1][5] + Si78[1][2]*T178[2][5]);
T78[1][5]=Si78[1][1]*T178[1][6] + Si78[1][2]*T178[2][6];
T78[1][6]=S87[1][3]*(Si78[1][1]*T178[1][4] + Si78[1][2]*T178[2][4]) + S87[2][3]*(Si78[1][1]*T178[1][5] + Si78[1][2]*T178[2][5]);

T78[2][1]=S87[1][1]*T178[3][1] + S87[2][1]*T178[3][2];
T78[2][2]=T178[3][3];
T78[2][3]=S87[1][3]*T178[3][1] + S87[2][3]*T178[3][2];
T78[2][4]=S87[1][1]*T178[3][4] + S87[2][1]*T178[3][5];
T78[2][5]=T178[3][6];
T78[2][6]=S87[1][3]*T178[3][4] + S87[2][3]*T178[3][5];

T78[3][1]=S87[1][1]*(Si78[3][1]*T178[1][1] + Si78[3][2]*T178[2][1]) + S87[2][1]*(Si78[3][1]*T178[1][2] + Si78[3][2]*T178[2][2]);
T78[3][2]=Si78[3][1]*T178[1][3] + Si78[3][2]*T178[2][3];
T78[3][3]=S87[1][3]*(Si78[3][1]*T178[1][1] + Si78[3][2]*T178[2][1]) + S87[2][3]*(Si78[3][1]*T178[1][2] + Si78[3][2]*T178[2][2]);
T78[3][4]=S87[1][1]*(Si78[3][1]*T178[1][4] + Si78[3][2]*T178[2][4]) + S87[2][1]*(Si78[3][1]*T178[1][5] + Si78[3][2]*T178[2][5]);
T78[3][5]=Si78[3][1]*T178[1][6] + Si78[3][2]*T178[2][6];
T78[3][6]=S87[1][3]*(Si78[3][1]*T178[1][4] + Si78[3][2]*T178[2][4]) + S87[2][3]*(Si78[3][1]*T178[1][5] + Si78[3][2]*T178[2][5]);

T78[4][1]=S87[1][1]*(Si78[1][1]*T178[4][1] + Si78[1][2]*T178[5][1]) + S87[2][1]*(Si78[1][1]*T178[4][2] + Si78[1][2]*T178[5][2]);
T78[4][2]=Si78[1][1]*T178[4][3] + Si78[1][2]*T178[5][3];
T78[4][3]=S87[1][3]*(Si78[1][1]*T178[4][1] + Si78[1][2]*T178[5][1]) + S87[2][3]*(Si78[1][1]*T178[4][2] + Si78[1][2]*T178[5][2]);
T78[4][4]=S87[1][1]*(Si78[1][1]*T178[4][4] + Si78[1][2]*T178[5][4]) + S87[2][1]*(Si78[1][1]*T178[4][5] + Si78[1][2]*T178[5][5]);
T78[4][5]=Si78[1][1]*T178[4][6] + Si78[1][2]*T178[5][6];
T78[4][6]=S87[1][3]*(Si78[1][1]*T178[4][4] + Si78[1][2]*T178[5][4]) + S87[2][3]*(Si78[1][1]*T178[4][5] + Si78[1][2]*T178[5][5]);

T78[5][1]=S87[1][1]*T178[6][1] + S87[2][1]*T178[6][2];
T78[5][2]=T178[6][3];
T78[5][3]=S87[1][3]*T178[6][1] + S87[2][3]*T178[6][2];
T78[5][4]=S87[1][1]*T178[6][4] + S87[2][1]*T178[6][5];
T78[5][5]=T178[6][6];
T78[5][6]=S87[1][3]*T178[6][4] + S87[2][3]*T178[6][5];

T78[6][1]=S87[1][1]*(Si78[3][1]*T178[4][1] + Si78[3][2]*T178[5][1]) + S87[2][1]*(Si78[3][1]*T178[4][2] + Si78[3][2]*T178[5][2]);
T78[6][2]=Si78[3][1]*T178[4][3] + Si78[3][2]*T178[5][3];
T78[6][3]=S87[1][3]*(Si78[3][1]*T178[4][1] + Si78[3][2]*T178[5][1]) + S87[2][3]*(Si78[3][1]*T178[4][2] + Si78[3][2]*T178[5][2]);
T78[6][4]=S87[1][1]*(Si78[3][1]*T178[4][4] + Si78[3][2]*T178[5][4]) + S87[2][1]*(Si78[3][1]*T178[4][5] + Si78[3][2]*T178[5][5]);
T78[6][5]=Si78[3][1]*T178[4][6] + Si78[3][2]*T178[5][6];
T78[6][6]=S87[1][3]*(Si78[3][1]*T178[4][4] + Si78[3][2]*T178[5][4]) + S87[2][3]*(Si78[3][1]*T178[4][5] + Si78[3][2]*T178[5][5]);



}


void
hermes_InvDynArtfunc97(void)
      {
JA7[1][1]=T78[1][1];
JA7[1][2]=links[4].mcm[3] + T78[1][2];
JA7[1][3]=-links[4].mcm[2] + T78[1][3];
JA7[1][4]=links[4].m + T78[1][4];
JA7[1][5]=T78[1][5];
JA7[1][6]=T78[1][6];

JA7[2][1]=-links[4].mcm[3] + T78[2][1];
JA7[2][2]=T78[2][2];
JA7[2][3]=links[4].mcm[1] + T78[2][3];
JA7[2][4]=T78[2][4];
JA7[2][5]=links[4].m + T78[2][5];
JA7[2][6]=T78[2][6];

JA7[3][1]=links[4].mcm[2] + T78[3][1];
JA7[3][2]=-links[4].mcm[1] + T78[3][2];
JA7[3][3]=T78[3][3];
JA7[3][4]=T78[3][4];
JA7[3][5]=T78[3][5];
JA7[3][6]=links[4].m + T78[3][6];

JA7[4][1]=links[4].inertia[1][1] + T78[4][1];
JA7[4][2]=links[4].inertia[1][2] + T78[4][2];
JA7[4][3]=links[4].inertia[1][3] + T78[4][3];
JA7[4][4]=T78[4][4];
JA7[4][5]=-links[4].mcm[3] + T78[4][5];
JA7[4][6]=links[4].mcm[2] + T78[4][6];

JA7[5][1]=links[4].inertia[1][2] + T78[5][1];
JA7[5][2]=links[4].inertia[2][2] + T78[5][2];
JA7[5][3]=links[4].inertia[2][3] + T78[5][3];
JA7[5][4]=links[4].mcm[3] + T78[5][4];
JA7[5][5]=T78[5][5];
JA7[5][6]=-links[4].mcm[1] + T78[5][6];

JA7[6][1]=links[4].inertia[1][3] + T78[6][1];
JA7[6][2]=links[4].inertia[2][3] + T78[6][2];
JA7[6][3]=links[4].inertia[3][3] + T78[6][3];
JA7[6][4]=-links[4].mcm[2] + T78[6][4];
JA7[6][5]=links[4].mcm[1] + T78[6][5];
JA7[6][6]=T78[6][6];


h7[1]=JA7[1][3];
h7[2]=JA7[2][3];
h7[3]=JA7[3][3];
h7[4]=JA7[4][3];
h7[5]=JA7[5][3];
h7[6]=JA7[6][3];

T167[1][1]=JA7[1][1];
T167[1][2]=JA7[1][2];
T167[1][3]=JA7[1][3];
T167[1][4]=JA7[1][4];
T167[1][5]=JA7[1][5];
T167[1][6]=JA7[1][6];

T167[2][1]=JA7[2][1];
T167[2][2]=JA7[2][2];
T167[2][3]=JA7[2][3];
T167[2][4]=JA7[2][4];
T167[2][5]=JA7[2][5];
T167[2][6]=JA7[2][6];

T167[3][1]=JA7[3][1];
T167[3][2]=JA7[3][2];
T167[3][3]=JA7[3][3];
T167[3][4]=JA7[3][4];
T167[3][5]=JA7[3][5];
T167[3][6]=JA7[3][6];

T167[4][1]=JA7[4][1];
T167[4][2]=JA7[4][2];
T167[4][3]=JA7[4][3];
T167[4][4]=JA7[4][4];
T167[4][5]=JA7[4][5];
T167[4][6]=JA7[4][6];

T167[5][1]=JA7[5][1];
T167[5][2]=JA7[5][2];
T167[5][3]=JA7[5][3];
T167[5][4]=JA7[5][4];
T167[5][5]=JA7[5][5];
T167[5][6]=JA7[5][6];

T167[6][1]=JA7[6][1];
T167[6][2]=JA7[6][2];
T167[6][3]=JA7[6][3];
T167[6][4]=JA7[6][4];
T167[6][5]=JA7[6][5];
T167[6][6]=JA7[6][6];


T67[1][1]=T167[3][3] + UPPERARM*S76[1][2]*T167[3][4] + UPPERARM*S76[2][2]*T167[3][5];
T67[1][2]=S76[1][2]*T167[3][1] + S76[2][2]*T167[3][2] - UPPERARM*T167[3][6];
T67[1][3]=S76[1][3]*T167[3][1] + S76[2][3]*T167[3][2];
T67[1][4]=T167[3][6];
T67[1][5]=S76[1][2]*T167[3][4] + S76[2][2]*T167[3][5];
T67[1][6]=S76[1][3]*T167[3][4] + S76[2][3]*T167[3][5];

T67[2][1]=Si67[2][1]*T167[1][3] + Si67[2][2]*T167[2][3] + UPPERARM*S76[1][2]*(Si67[2][1]*T167[1][4] + Si67[2][2]*T167[2][4]) + UPPERARM*S76[2][2]*(Si67[2][1]*T167[1][5] + Si67[2][2]*T167[2][5]);
T67[2][2]=S76[1][2]*(Si67[2][1]*T167[1][1] + Si67[2][2]*T167[2][1]) + S76[2][2]*(Si67[2][1]*T167[1][2] + Si67[2][2]*T167[2][2]) - UPPERARM*(Si67[2][1]*T167[1][6] + Si67[2][2]*T167[2][6]);
T67[2][3]=S76[1][3]*(Si67[2][1]*T167[1][1] + Si67[2][2]*T167[2][1]) + S76[2][3]*(Si67[2][1]*T167[1][2] + Si67[2][2]*T167[2][2]);
T67[2][4]=Si67[2][1]*T167[1][6] + Si67[2][2]*T167[2][6];
T67[2][5]=S76[1][2]*(Si67[2][1]*T167[1][4] + Si67[2][2]*T167[2][4]) + S76[2][2]*(Si67[2][1]*T167[1][5] + Si67[2][2]*T167[2][5]);
T67[2][6]=S76[1][3]*(Si67[2][1]*T167[1][4] + Si67[2][2]*T167[2][4]) + S76[2][3]*(Si67[2][1]*T167[1][5] + Si67[2][2]*T167[2][5]);

T67[3][1]=Si67[3][1]*T167[1][3] + Si67[3][2]*T167[2][3] + UPPERARM*S76[1][2]*(Si67[3][1]*T167[1][4] + Si67[3][2]*T167[2][4]) + UPPERARM*S76[2][2]*(Si67[3][1]*T167[1][5] + Si67[3][2]*T167[2][5]);
T67[3][2]=S76[1][2]*(Si67[3][1]*T167[1][1] + Si67[3][2]*T167[2][1]) + S76[2][2]*(Si67[3][1]*T167[1][2] + Si67[3][2]*T167[2][2]) - UPPERARM*(Si67[3][1]*T167[1][6] + Si67[3][2]*T167[2][6]);
T67[3][3]=S76[1][3]*(Si67[3][1]*T167[1][1] + Si67[3][2]*T167[2][1]) + S76[2][3]*(Si67[3][1]*T167[1][2] + Si67[3][2]*T167[2][2]);
T67[3][4]=Si67[3][1]*T167[1][6] + Si67[3][2]*T167[2][6];
T67[3][5]=S76[1][2]*(Si67[3][1]*T167[1][4] + Si67[3][2]*T167[2][4]) + S76[2][2]*(Si67[3][1]*T167[1][5] + Si67[3][2]*T167[2][5]);
T67[3][6]=S76[1][3]*(Si67[3][1]*T167[1][4] + Si67[3][2]*T167[2][4]) + S76[2][3]*(Si67[3][1]*T167[1][5] + Si67[3][2]*T167[2][5]);

T67[4][1]=UPPERARM*Si67[2][1]*T167[1][3] + UPPERARM*Si67[2][2]*T167[2][3] + T167[6][3] + UPPERARM*S76[1][2]*(UPPERARM*Si67[2][1]*T167[1][4] + UPPERARM*Si67[2][2]*T167[2][4] + T167[6][4]) + UPPERARM*S76[2][2]*(UPPERARM*Si67[2][1]*T167[1][5] + UPPERARM*Si67[2][2]*T167[2][5] + T167[6][5]);
T67[4][2]=S76[1][2]*(UPPERARM*Si67[2][1]*T167[1][1] + UPPERARM*Si67[2][2]*T167[2][1] + T167[6][1]) + S76[2][2]*(UPPERARM*Si67[2][1]*T167[1][2] + UPPERARM*Si67[2][2]*T167[2][2] + T167[6][2]) - UPPERARM*(UPPERARM*Si67[2][1]*T167[1][6] + UPPERARM*Si67[2][2]*T167[2][6] + T167[6][6]);
T67[4][3]=S76[1][3]*(UPPERARM*Si67[2][1]*T167[1][1] + UPPERARM*Si67[2][2]*T167[2][1] + T167[6][1]) + S76[2][3]*(UPPERARM*Si67[2][1]*T167[1][2] + UPPERARM*Si67[2][2]*T167[2][2] + T167[6][2]);
T67[4][4]=UPPERARM*Si67[2][1]*T167[1][6] + UPPERARM*Si67[2][2]*T167[2][6] + T167[6][6];
T67[4][5]=S76[1][2]*(UPPERARM*Si67[2][1]*T167[1][4] + UPPERARM*Si67[2][2]*T167[2][4] + T167[6][4]) + S76[2][2]*(UPPERARM*Si67[2][1]*T167[1][5] + UPPERARM*Si67[2][2]*T167[2][5] + T167[6][5]);
T67[4][6]=S76[1][3]*(UPPERARM*Si67[2][1]*T167[1][4] + UPPERARM*Si67[2][2]*T167[2][4] + T167[6][4]) + S76[2][3]*(UPPERARM*Si67[2][1]*T167[1][5] + UPPERARM*Si67[2][2]*T167[2][5] + T167[6][5]);

T67[5][1]=-(UPPERARM*T167[3][3]) + Si67[2][1]*T167[4][3] + Si67[2][2]*T167[5][3] + UPPERARM*S76[1][2]*(-(UPPERARM*T167[3][4]) + Si67[2][1]*T167[4][4] + Si67[2][2]*T167[5][4]) + UPPERARM*S76[2][2]*(-(UPPERARM*T167[3][5]) + Si67[2][1]*T167[4][5] + Si67[2][2]*T167[5][5]);
T67[5][2]=S76[1][2]*(-(UPPERARM*T167[3][1]) + Si67[2][1]*T167[4][1] + Si67[2][2]*T167[5][1]) + S76[2][2]*(-(UPPERARM*T167[3][2]) + Si67[2][1]*T167[4][2] + Si67[2][2]*T167[5][2]) - UPPERARM*(-(UPPERARM*T167[3][6]) + Si67[2][1]*T167[4][6] + Si67[2][2]*T167[5][6]);
T67[5][3]=S76[1][3]*(-(UPPERARM*T167[3][1]) + Si67[2][1]*T167[4][1] + Si67[2][2]*T167[5][1]) + S76[2][3]*(-(UPPERARM*T167[3][2]) + Si67[2][1]*T167[4][2] + Si67[2][2]*T167[5][2]);
T67[5][4]=-(UPPERARM*T167[3][6]) + Si67[2][1]*T167[4][6] + Si67[2][2]*T167[5][6];
T67[5][5]=S76[1][2]*(-(UPPERARM*T167[3][4]) + Si67[2][1]*T167[4][4] + Si67[2][2]*T167[5][4]) + S76[2][2]*(-(UPPERARM*T167[3][5]) + Si67[2][1]*T167[4][5] + Si67[2][2]*T167[5][5]);
T67[5][6]=S76[1][3]*(-(UPPERARM*T167[3][4]) + Si67[2][1]*T167[4][4] + Si67[2][2]*T167[5][4]) + S76[2][3]*(-(UPPERARM*T167[3][5]) + Si67[2][1]*T167[4][5] + Si67[2][2]*T167[5][5]);

T67[6][1]=Si67[3][1]*T167[4][3] + Si67[3][2]*T167[5][3] + UPPERARM*S76[1][2]*(Si67[3][1]*T167[4][4] + Si67[3][2]*T167[5][4]) + UPPERARM*S76[2][2]*(Si67[3][1]*T167[4][5] + Si67[3][2]*T167[5][5]);
T67[6][2]=S76[1][2]*(Si67[3][1]*T167[4][1] + Si67[3][2]*T167[5][1]) + S76[2][2]*(Si67[3][1]*T167[4][2] + Si67[3][2]*T167[5][2]) - UPPERARM*(Si67[3][1]*T167[4][6] + Si67[3][2]*T167[5][6]);
T67[6][3]=S76[1][3]*(Si67[3][1]*T167[4][1] + Si67[3][2]*T167[5][1]) + S76[2][3]*(Si67[3][1]*T167[4][2] + Si67[3][2]*T167[5][2]);
T67[6][4]=Si67[3][1]*T167[4][6] + Si67[3][2]*T167[5][6];
T67[6][5]=S76[1][2]*(Si67[3][1]*T167[4][4] + Si67[3][2]*T167[5][4]) + S76[2][2]*(Si67[3][1]*T167[4][5] + Si67[3][2]*T167[5][5]);
T67[6][6]=S76[1][3]*(Si67[3][1]*T167[4][4] + Si67[3][2]*T167[5][4]) + S76[2][3]*(Si67[3][1]*T167[4][5] + Si67[3][2]*T167[5][5]);



}


void
hermes_InvDynArtfunc98(void)
      {
JA6[1][1]=T67[1][1];
JA6[1][2]=links[3].mcm[3] + T67[1][2];
JA6[1][3]=-links[3].mcm[2] + T67[1][3];
JA6[1][4]=links[3].m + T67[1][4];
JA6[1][5]=T67[1][5];
JA6[1][6]=T67[1][6];

JA6[2][1]=-links[3].mcm[3] + T67[2][1];
JA6[2][2]=T67[2][2];
JA6[2][3]=links[3].mcm[1] + T67[2][3];
JA6[2][4]=T67[2][4];
JA6[2][5]=links[3].m + T67[2][5];
JA6[2][6]=T67[2][6];

JA6[3][1]=links[3].mcm[2] + T67[3][1];
JA6[3][2]=-links[3].mcm[1] + T67[3][2];
JA6[3][3]=T67[3][3];
JA6[3][4]=T67[3][4];
JA6[3][5]=T67[3][5];
JA6[3][6]=links[3].m + T67[3][6];

JA6[4][1]=links[3].inertia[1][1] + T67[4][1];
JA6[4][2]=links[3].inertia[1][2] + T67[4][2];
JA6[4][3]=links[3].inertia[1][3] + T67[4][3];
JA6[4][4]=T67[4][4];
JA6[4][5]=-links[3].mcm[3] + T67[4][5];
JA6[4][6]=links[3].mcm[2] + T67[4][6];

JA6[5][1]=links[3].inertia[1][2] + T67[5][1];
JA6[5][2]=links[3].inertia[2][2] + T67[5][2];
JA6[5][3]=links[3].inertia[2][3] + T67[5][3];
JA6[5][4]=links[3].mcm[3] + T67[5][4];
JA6[5][5]=T67[5][5];
JA6[5][6]=-links[3].mcm[1] + T67[5][6];

JA6[6][1]=links[3].inertia[1][3] + T67[6][1];
JA6[6][2]=links[3].inertia[2][3] + T67[6][2];
JA6[6][3]=links[3].inertia[3][3] + T67[6][3];
JA6[6][4]=-links[3].mcm[2] + T67[6][4];
JA6[6][5]=links[3].mcm[1] + T67[6][5];
JA6[6][6]=T67[6][6];


h6[1]=JA6[1][3];
h6[2]=JA6[2][3];
h6[3]=JA6[3][3];
h6[4]=JA6[4][3];
h6[5]=JA6[5][3];
h6[6]=JA6[6][3];

T156[1][1]=JA6[1][1];
T156[1][2]=JA6[1][2];
T156[1][3]=JA6[1][3];
T156[1][4]=JA6[1][4];
T156[1][5]=JA6[1][5];
T156[1][6]=JA6[1][6];

T156[2][1]=JA6[2][1];
T156[2][2]=JA6[2][2];
T156[2][3]=JA6[2][3];
T156[2][4]=JA6[2][4];
T156[2][5]=JA6[2][5];
T156[2][6]=JA6[2][6];

T156[3][1]=JA6[3][1];
T156[3][2]=JA6[3][2];
T156[3][3]=JA6[3][3];
T156[3][4]=JA6[3][4];
T156[3][5]=JA6[3][5];
T156[3][6]=JA6[3][6];

T156[4][1]=JA6[4][1];
T156[4][2]=JA6[4][2];
T156[4][3]=JA6[4][3];
T156[4][4]=JA6[4][4];
T156[4][5]=JA6[4][5];
T156[4][6]=JA6[4][6];

T156[5][1]=JA6[5][1];
T156[5][2]=JA6[5][2];
T156[5][3]=JA6[5][3];
T156[5][4]=JA6[5][4];
T156[5][5]=JA6[5][5];
T156[5][6]=JA6[5][6];

T156[6][1]=JA6[6][1];
T156[6][2]=JA6[6][2];
T156[6][3]=JA6[6][3];
T156[6][4]=JA6[6][4];
T156[6][5]=JA6[6][5];
T156[6][6]=JA6[6][6];


T56[1][1]=S65[1][1]*(Si56[1][1]*T156[1][1] + Si56[1][2]*T156[2][1]) + S65[2][1]*(Si56[1][1]*T156[1][2] + Si56[1][2]*T156[2][2]);
T56[1][2]=Si56[1][1]*T156[1][3] + Si56[1][2]*T156[2][3] + SHOULDERY*S65[1][3]*(Si56[1][1]*T156[1][4] + Si56[1][2]*T156[2][4]) + SHOULDERY*S65[2][3]*(Si56[1][1]*T156[1][5] + Si56[1][2]*T156[2][5]);
T56[1][3]=S65[1][3]*(Si56[1][1]*T156[1][1] + Si56[1][2]*T156[2][1]) + S65[2][3]*(Si56[1][1]*T156[1][2] + Si56[1][2]*T156[2][2]) - SHOULDERY*(Si56[1][1]*T156[1][6] + Si56[1][2]*T156[2][6]);
T56[1][4]=S65[1][1]*(Si56[1][1]*T156[1][4] + Si56[1][2]*T156[2][4]) + S65[2][1]*(Si56[1][1]*T156[1][5] + Si56[1][2]*T156[2][5]);
T56[1][5]=Si56[1][1]*T156[1][6] + Si56[1][2]*T156[2][6];
T56[1][6]=S65[1][3]*(Si56[1][1]*T156[1][4] + Si56[1][2]*T156[2][4]) + S65[2][3]*(Si56[1][1]*T156[1][5] + Si56[1][2]*T156[2][5]);

T56[2][1]=S65[1][1]*T156[3][1] + S65[2][1]*T156[3][2];
T56[2][2]=T156[3][3] + SHOULDERY*S65[1][3]*T156[3][4] + SHOULDERY*S65[2][3]*T156[3][5];
T56[2][3]=S65[1][3]*T156[3][1] + S65[2][3]*T156[3][2] - SHOULDERY*T156[3][6];
T56[2][4]=S65[1][1]*T156[3][4] + S65[2][1]*T156[3][5];
T56[2][5]=T156[3][6];
T56[2][6]=S65[1][3]*T156[3][4] + S65[2][3]*T156[3][5];

T56[3][1]=S65[1][1]*(Si56[3][1]*T156[1][1] + Si56[3][2]*T156[2][1]) + S65[2][1]*(Si56[3][1]*T156[1][2] + Si56[3][2]*T156[2][2]);
T56[3][2]=Si56[3][1]*T156[1][3] + Si56[3][2]*T156[2][3] + SHOULDERY*S65[1][3]*(Si56[3][1]*T156[1][4] + Si56[3][2]*T156[2][4]) + SHOULDERY*S65[2][3]*(Si56[3][1]*T156[1][5] + Si56[3][2]*T156[2][5]);
T56[3][3]=S65[1][3]*(Si56[3][1]*T156[1][1] + Si56[3][2]*T156[2][1]) + S65[2][3]*(Si56[3][1]*T156[1][2] + Si56[3][2]*T156[2][2]) - SHOULDERY*(Si56[3][1]*T156[1][6] + Si56[3][2]*T156[2][6]);
T56[3][4]=S65[1][1]*(Si56[3][1]*T156[1][4] + Si56[3][2]*T156[2][4]) + S65[2][1]*(Si56[3][1]*T156[1][5] + Si56[3][2]*T156[2][5]);
T56[3][5]=Si56[3][1]*T156[1][6] + Si56[3][2]*T156[2][6];
T56[3][6]=S65[1][3]*(Si56[3][1]*T156[1][4] + Si56[3][2]*T156[2][4]) + S65[2][3]*(Si56[3][1]*T156[1][5] + Si56[3][2]*T156[2][5]);

T56[4][1]=S65[1][1]*(Si56[1][1]*T156[4][1] + Si56[1][2]*T156[5][1]) + S65[2][1]*(Si56[1][1]*T156[4][2] + Si56[1][2]*T156[5][2]);
T56[4][2]=Si56[1][1]*T156[4][3] + Si56[1][2]*T156[5][3] + SHOULDERY*S65[1][3]*(Si56[1][1]*T156[4][4] + Si56[1][2]*T156[5][4]) + SHOULDERY*S65[2][3]*(Si56[1][1]*T156[4][5] + Si56[1][2]*T156[5][5]);
T56[4][3]=S65[1][3]*(Si56[1][1]*T156[4][1] + Si56[1][2]*T156[5][1]) + S65[2][3]*(Si56[1][1]*T156[4][2] + Si56[1][2]*T156[5][2]) - SHOULDERY*(Si56[1][1]*T156[4][6] + Si56[1][2]*T156[5][6]);
T56[4][4]=S65[1][1]*(Si56[1][1]*T156[4][4] + Si56[1][2]*T156[5][4]) + S65[2][1]*(Si56[1][1]*T156[4][5] + Si56[1][2]*T156[5][5]);
T56[4][5]=Si56[1][1]*T156[4][6] + Si56[1][2]*T156[5][6];
T56[4][6]=S65[1][3]*(Si56[1][1]*T156[4][4] + Si56[1][2]*T156[5][4]) + S65[2][3]*(Si56[1][1]*T156[4][5] + Si56[1][2]*T156[5][5]);

T56[5][1]=S65[1][1]*(SHOULDERY*Si56[3][1]*T156[1][1] + SHOULDERY*Si56[3][2]*T156[2][1] + T156[6][1]) + S65[2][1]*(SHOULDERY*Si56[3][1]*T156[1][2] + SHOULDERY*Si56[3][2]*T156[2][2] + T156[6][2]);
T56[5][2]=SHOULDERY*Si56[3][1]*T156[1][3] + SHOULDERY*Si56[3][2]*T156[2][3] + T156[6][3] + SHOULDERY*S65[1][3]*(SHOULDERY*Si56[3][1]*T156[1][4] + SHOULDERY*Si56[3][2]*T156[2][4] + T156[6][4]) + SHOULDERY*S65[2][3]*(SHOULDERY*Si56[3][1]*T156[1][5] + SHOULDERY*Si56[3][2]*T156[2][5] + T156[6][5]);
T56[5][3]=S65[1][3]*(SHOULDERY*Si56[3][1]*T156[1][1] + SHOULDERY*Si56[3][2]*T156[2][1] + T156[6][1]) + S65[2][3]*(SHOULDERY*Si56[3][1]*T156[1][2] + SHOULDERY*Si56[3][2]*T156[2][2] + T156[6][2]) - SHOULDERY*(SHOULDERY*Si56[3][1]*T156[1][6] + SHOULDERY*Si56[3][2]*T156[2][6] + T156[6][6]);
T56[5][4]=S65[1][1]*(SHOULDERY*Si56[3][1]*T156[1][4] + SHOULDERY*Si56[3][2]*T156[2][4] + T156[6][4]) + S65[2][1]*(SHOULDERY*Si56[3][1]*T156[1][5] + SHOULDERY*Si56[3][2]*T156[2][5] + T156[6][5]);
T56[5][5]=SHOULDERY*Si56[3][1]*T156[1][6] + SHOULDERY*Si56[3][2]*T156[2][6] + T156[6][6];
T56[5][6]=S65[1][3]*(SHOULDERY*Si56[3][1]*T156[1][4] + SHOULDERY*Si56[3][2]*T156[2][4] + T156[6][4]) + S65[2][3]*(SHOULDERY*Si56[3][1]*T156[1][5] + SHOULDERY*Si56[3][2]*T156[2][5] + T156[6][5]);

T56[6][1]=S65[1][1]*(-(SHOULDERY*T156[3][1]) + Si56[3][1]*T156[4][1] + Si56[3][2]*T156[5][1]) + S65[2][1]*(-(SHOULDERY*T156[3][2]) + Si56[3][1]*T156[4][2] + Si56[3][2]*T156[5][2]);
T56[6][2]=-(SHOULDERY*T156[3][3]) + Si56[3][1]*T156[4][3] + Si56[3][2]*T156[5][3] + SHOULDERY*S65[1][3]*(-(SHOULDERY*T156[3][4]) + Si56[3][1]*T156[4][4] + Si56[3][2]*T156[5][4]) + SHOULDERY*S65[2][3]*(-(SHOULDERY*T156[3][5]) + Si56[3][1]*T156[4][5] + Si56[3][2]*T156[5][5]);
T56[6][3]=S65[1][3]*(-(SHOULDERY*T156[3][1]) + Si56[3][1]*T156[4][1] + Si56[3][2]*T156[5][1]) + S65[2][3]*(-(SHOULDERY*T156[3][2]) + Si56[3][1]*T156[4][2] + Si56[3][2]*T156[5][2]) - SHOULDERY*(-(SHOULDERY*T156[3][6]) + Si56[3][1]*T156[4][6] + Si56[3][2]*T156[5][6]);
T56[6][4]=S65[1][1]*(-(SHOULDERY*T156[3][4]) + Si56[3][1]*T156[4][4] + Si56[3][2]*T156[5][4]) + S65[2][1]*(-(SHOULDERY*T156[3][5]) + Si56[3][1]*T156[4][5] + Si56[3][2]*T156[5][5]);
T56[6][5]=-(SHOULDERY*T156[3][6]) + Si56[3][1]*T156[4][6] + Si56[3][2]*T156[5][6];
T56[6][6]=S65[1][3]*(-(SHOULDERY*T156[3][4]) + Si56[3][1]*T156[4][4] + Si56[3][2]*T156[5][4]) + S65[2][3]*(-(SHOULDERY*T156[3][5]) + Si56[3][1]*T156[4][5] + Si56[3][2]*T156[5][5]);



}


void
hermes_InvDynArtfunc99(void)
      {
JA5[1][1]=T56[1][1];
JA5[1][2]=links[2].mcm[3] + T56[1][2];
JA5[1][3]=-links[2].mcm[2] + T56[1][3];
JA5[1][4]=links[2].m + T56[1][4];
JA5[1][5]=T56[1][5];
JA5[1][6]=T56[1][6];

JA5[2][1]=-links[2].mcm[3] + T56[2][1];
JA5[2][2]=T56[2][2];
JA5[2][3]=links[2].mcm[1] + T56[2][3];
JA5[2][4]=T56[2][4];
JA5[2][5]=links[2].m + T56[2][5];
JA5[2][6]=T56[2][6];

JA5[3][1]=links[2].mcm[2] + T56[3][1];
JA5[3][2]=-links[2].mcm[1] + T56[3][2];
JA5[3][3]=T56[3][3];
JA5[3][4]=T56[3][4];
JA5[3][5]=T56[3][5];
JA5[3][6]=links[2].m + T56[3][6];

JA5[4][1]=links[2].inertia[1][1] + T56[4][1];
JA5[4][2]=links[2].inertia[1][2] + T56[4][2];
JA5[4][3]=links[2].inertia[1][3] + T56[4][3];
JA5[4][4]=T56[4][4];
JA5[4][5]=-links[2].mcm[3] + T56[4][5];
JA5[4][6]=links[2].mcm[2] + T56[4][6];

JA5[5][1]=links[2].inertia[1][2] + T56[5][1];
JA5[5][2]=links[2].inertia[2][2] + T56[5][2];
JA5[5][3]=links[2].inertia[2][3] + T56[5][3];
JA5[5][4]=links[2].mcm[3] + T56[5][4];
JA5[5][5]=T56[5][5];
JA5[5][6]=-links[2].mcm[1] + T56[5][6];

JA5[6][1]=links[2].inertia[1][3] + T56[6][1];
JA5[6][2]=links[2].inertia[2][3] + T56[6][2];
JA5[6][3]=links[2].inertia[3][3] + T56[6][3];
JA5[6][4]=-links[2].mcm[2] + T56[6][4];
JA5[6][5]=links[2].mcm[1] + T56[6][5];
JA5[6][6]=T56[6][6];


h5[1]=JA5[1][3];
h5[2]=JA5[2][3];
h5[3]=JA5[3][3];
h5[4]=JA5[4][3];
h5[5]=JA5[5][3];
h5[6]=JA5[6][3];

T145[1][1]=JA5[1][1];
T145[1][2]=JA5[1][2];
T145[1][3]=JA5[1][3];
T145[1][4]=JA5[1][4];
T145[1][5]=JA5[1][5];
T145[1][6]=JA5[1][6];

T145[2][1]=JA5[2][1];
T145[2][2]=JA5[2][2];
T145[2][3]=JA5[2][3];
T145[2][4]=JA5[2][4];
T145[2][5]=JA5[2][5];
T145[2][6]=JA5[2][6];

T145[3][1]=JA5[3][1];
T145[3][2]=JA5[3][2];
T145[3][3]=JA5[3][3];
T145[3][4]=JA5[3][4];
T145[3][5]=JA5[3][5];
T145[3][6]=JA5[3][6];

T145[4][1]=JA5[4][1];
T145[4][2]=JA5[4][2];
T145[4][3]=JA5[4][3];
T145[4][4]=JA5[4][4];
T145[4][5]=JA5[4][5];
T145[4][6]=JA5[4][6];

T145[5][1]=JA5[5][1];
T145[5][2]=JA5[5][2];
T145[5][3]=JA5[5][3];
T145[5][4]=JA5[5][4];
T145[5][5]=JA5[5][5];
T145[5][6]=JA5[5][6];

T145[6][1]=JA5[6][1];
T145[6][2]=JA5[6][2];
T145[6][3]=JA5[6][3];
T145[6][4]=JA5[6][4];
T145[6][5]=JA5[6][5];
T145[6][6]=JA5[6][6];


T45[1][1]=S54[1][1]*(Si45[1][1]*T145[1][1] + Si45[1][2]*T145[2][1]) + S54[2][1]*(Si45[1][1]*T145[1][2] + Si45[1][2]*T145[2][2]) - SHOULDERX*(Si45[1][1]*T145[1][6] + Si45[1][2]*T145[2][6]);
T45[1][2]=-(Si45[1][1]*T145[1][3]) - Si45[1][2]*T145[2][3] - SHOULDERX*S54[1][1]*(Si45[1][1]*T145[1][4] + Si45[1][2]*T145[2][4]) - SHOULDERX*S54[2][1]*(Si45[1][1]*T145[1][5] + Si45[1][2]*T145[2][5]);
T45[1][3]=S54[1][3]*(Si45[1][1]*T145[1][1] + Si45[1][2]*T145[2][1]) + S54[2][3]*(Si45[1][1]*T145[1][2] + Si45[1][2]*T145[2][2]);
T45[1][4]=S54[1][1]*(Si45[1][1]*T145[1][4] + Si45[1][2]*T145[2][4]) + S54[2][1]*(Si45[1][1]*T145[1][5] + Si45[1][2]*T145[2][5]);
T45[1][5]=-(Si45[1][1]*T145[1][6]) - Si45[1][2]*T145[2][6];
T45[1][6]=S54[1][3]*(Si45[1][1]*T145[1][4] + Si45[1][2]*T145[2][4]) + S54[2][3]*(Si45[1][1]*T145[1][5] + Si45[1][2]*T145[2][5]);

T45[2][1]=-(S54[1][1]*T145[3][1]) - S54[2][1]*T145[3][2] + SHOULDERX*T145[3][6];
T45[2][2]=T145[3][3] + SHOULDERX*S54[1][1]*T145[3][4] + SHOULDERX*S54[2][1]*T145[3][5];
T45[2][3]=-(S54[1][3]*T145[3][1]) - S54[2][3]*T145[3][2];
T45[2][4]=-(S54[1][1]*T145[3][4]) - S54[2][1]*T145[3][5];
T45[2][5]=T145[3][6];
T45[2][6]=-(S54[1][3]*T145[3][4]) - S54[2][3]*T145[3][5];

T45[3][1]=S54[1][1]*(Si45[3][1]*T145[1][1] + Si45[3][2]*T145[2][1]) + S54[2][1]*(Si45[3][1]*T145[1][2] + Si45[3][2]*T145[2][2]) - SHOULDERX*(Si45[3][1]*T145[1][6] + Si45[3][2]*T145[2][6]);
T45[3][2]=-(Si45[3][1]*T145[1][3]) - Si45[3][2]*T145[2][3] - SHOULDERX*S54[1][1]*(Si45[3][1]*T145[1][4] + Si45[3][2]*T145[2][4]) - SHOULDERX*S54[2][1]*(Si45[3][1]*T145[1][5] + Si45[3][2]*T145[2][5]);
T45[3][3]=S54[1][3]*(Si45[3][1]*T145[1][1] + Si45[3][2]*T145[2][1]) + S54[2][3]*(Si45[3][1]*T145[1][2] + Si45[3][2]*T145[2][2]);
T45[3][4]=S54[1][1]*(Si45[3][1]*T145[1][4] + Si45[3][2]*T145[2][4]) + S54[2][1]*(Si45[3][1]*T145[1][5] + Si45[3][2]*T145[2][5]);
T45[3][5]=-(Si45[3][1]*T145[1][6]) - Si45[3][2]*T145[2][6];
T45[3][6]=S54[1][3]*(Si45[3][1]*T145[1][4] + Si45[3][2]*T145[2][4]) + S54[2][3]*(Si45[3][1]*T145[1][5] + Si45[3][2]*T145[2][5]);

T45[4][1]=S54[1][1]*(-(SHOULDERX*T145[3][1]) + Si45[1][1]*T145[4][1] + Si45[1][2]*T145[5][1]) + S54[2][1]*(-(SHOULDERX*T145[3][2]) + Si45[1][1]*T145[4][2] + Si45[1][2]*T145[5][2]) - SHOULDERX*(-(SHOULDERX*T145[3][6]) + Si45[1][1]*T145[4][6] + Si45[1][2]*T145[5][6]);
T45[4][2]=SHOULDERX*T145[3][3] - Si45[1][1]*T145[4][3] - Si45[1][2]*T145[5][3] - SHOULDERX*S54[1][1]*(-(SHOULDERX*T145[3][4]) + Si45[1][1]*T145[4][4] + Si45[1][2]*T145[5][4]) - SHOULDERX*S54[2][1]*(-(SHOULDERX*T145[3][5]) + Si45[1][1]*T145[4][5] + Si45[1][2]*T145[5][5]);
T45[4][3]=S54[1][3]*(-(SHOULDERX*T145[3][1]) + Si45[1][1]*T145[4][1] + Si45[1][2]*T145[5][1]) + S54[2][3]*(-(SHOULDERX*T145[3][2]) + Si45[1][1]*T145[4][2] + Si45[1][2]*T145[5][2]);
T45[4][4]=S54[1][1]*(-(SHOULDERX*T145[3][4]) + Si45[1][1]*T145[4][4] + Si45[1][2]*T145[5][4]) + S54[2][1]*(-(SHOULDERX*T145[3][5]) + Si45[1][1]*T145[4][5] + Si45[1][2]*T145[5][5]);
T45[4][5]=SHOULDERX*T145[3][6] - Si45[1][1]*T145[4][6] - Si45[1][2]*T145[5][6];
T45[4][6]=S54[1][3]*(-(SHOULDERX*T145[3][4]) + Si45[1][1]*T145[4][4] + Si45[1][2]*T145[5][4]) + S54[2][3]*(-(SHOULDERX*T145[3][5]) + Si45[1][1]*T145[4][5] + Si45[1][2]*T145[5][5]);

T45[5][1]=S54[1][1]*(-(SHOULDERX*Si45[1][1]*T145[1][1]) - SHOULDERX*Si45[1][2]*T145[2][1] - T145[6][1]) + S54[2][1]*(-(SHOULDERX*Si45[1][1]*T145[1][2]) - SHOULDERX*Si45[1][2]*T145[2][2] - T145[6][2]) - SHOULDERX*(-(SHOULDERX*Si45[1][1]*T145[1][6]) - SHOULDERX*Si45[1][2]*T145[2][6] - T145[6][6]);
T45[5][2]=SHOULDERX*Si45[1][1]*T145[1][3] + SHOULDERX*Si45[1][2]*T145[2][3] + T145[6][3] - SHOULDERX*S54[1][1]*(-(SHOULDERX*Si45[1][1]*T145[1][4]) - SHOULDERX*Si45[1][2]*T145[2][4] - T145[6][4]) - SHOULDERX*S54[2][1]*(-(SHOULDERX*Si45[1][1]*T145[1][5]) - SHOULDERX*Si45[1][2]*T145[2][5] - T145[6][5]);
T45[5][3]=S54[1][3]*(-(SHOULDERX*Si45[1][1]*T145[1][1]) - SHOULDERX*Si45[1][2]*T145[2][1] - T145[6][1]) + S54[2][3]*(-(SHOULDERX*Si45[1][1]*T145[1][2]) - SHOULDERX*Si45[1][2]*T145[2][2] - T145[6][2]);
T45[5][4]=S54[1][1]*(-(SHOULDERX*Si45[1][1]*T145[1][4]) - SHOULDERX*Si45[1][2]*T145[2][4] - T145[6][4]) + S54[2][1]*(-(SHOULDERX*Si45[1][1]*T145[1][5]) - SHOULDERX*Si45[1][2]*T145[2][5] - T145[6][5]);
T45[5][5]=SHOULDERX*Si45[1][1]*T145[1][6] + SHOULDERX*Si45[1][2]*T145[2][6] + T145[6][6];
T45[5][6]=S54[1][3]*(-(SHOULDERX*Si45[1][1]*T145[1][4]) - SHOULDERX*Si45[1][2]*T145[2][4] - T145[6][4]) + S54[2][3]*(-(SHOULDERX*Si45[1][1]*T145[1][5]) - SHOULDERX*Si45[1][2]*T145[2][5] - T145[6][5]);

T45[6][1]=S54[1][1]*(Si45[3][1]*T145[4][1] + Si45[3][2]*T145[5][1]) + S54[2][1]*(Si45[3][1]*T145[4][2] + Si45[3][2]*T145[5][2]) - SHOULDERX*(Si45[3][1]*T145[4][6] + Si45[3][2]*T145[5][6]);
T45[6][2]=-(Si45[3][1]*T145[4][3]) - Si45[3][2]*T145[5][3] - SHOULDERX*S54[1][1]*(Si45[3][1]*T145[4][4] + Si45[3][2]*T145[5][4]) - SHOULDERX*S54[2][1]*(Si45[3][1]*T145[4][5] + Si45[3][2]*T145[5][5]);
T45[6][3]=S54[1][3]*(Si45[3][1]*T145[4][1] + Si45[3][2]*T145[5][1]) + S54[2][3]*(Si45[3][1]*T145[4][2] + Si45[3][2]*T145[5][2]);
T45[6][4]=S54[1][1]*(Si45[3][1]*T145[4][4] + Si45[3][2]*T145[5][4]) + S54[2][1]*(Si45[3][1]*T145[4][5] + Si45[3][2]*T145[5][5]);
T45[6][5]=-(Si45[3][1]*T145[4][6]) - Si45[3][2]*T145[5][6];
T45[6][6]=S54[1][3]*(Si45[3][1]*T145[4][4] + Si45[3][2]*T145[5][4]) + S54[2][3]*(Si45[3][1]*T145[4][5] + Si45[3][2]*T145[5][5]);



}


void
hermes_InvDynArtfunc100(void)
       {
JA4[1][1]=T45[1][1];
JA4[1][2]=links[1].mcm[3] + T45[1][2];
JA4[1][3]=-links[1].mcm[2] + T45[1][3];
JA4[1][4]=links[1].m + T45[1][4];
JA4[1][5]=T45[1][5];
JA4[1][6]=T45[1][6];

JA4[2][1]=-links[1].mcm[3] + T45[2][1];
JA4[2][2]=T45[2][2];
JA4[2][3]=links[1].mcm[1] + T45[2][3];
JA4[2][4]=T45[2][4];
JA4[2][5]=links[1].m + T45[2][5];
JA4[2][6]=T45[2][6];

JA4[3][1]=links[1].mcm[2] + T45[3][1];
JA4[3][2]=-links[1].mcm[1] + T45[3][2];
JA4[3][3]=T45[3][3];
JA4[3][4]=T45[3][4];
JA4[3][5]=T45[3][5];
JA4[3][6]=links[1].m + T45[3][6];

JA4[4][1]=links[1].inertia[1][1] + T45[4][1];
JA4[4][2]=links[1].inertia[1][2] + T45[4][2];
JA4[4][3]=links[1].inertia[1][3] + T45[4][3];
JA4[4][4]=T45[4][4];
JA4[4][5]=-links[1].mcm[3] + T45[4][5];
JA4[4][6]=links[1].mcm[2] + T45[4][6];

JA4[5][1]=links[1].inertia[1][2] + T45[5][1];
JA4[5][2]=links[1].inertia[2][2] + T45[5][2];
JA4[5][3]=links[1].inertia[2][3] + T45[5][3];
JA4[5][4]=links[1].mcm[3] + T45[5][4];
JA4[5][5]=T45[5][5];
JA4[5][6]=-links[1].mcm[1] + T45[5][6];

JA4[6][1]=links[1].inertia[1][3] + T45[6][1];
JA4[6][2]=links[1].inertia[2][3] + T45[6][2];
JA4[6][3]=links[1].inertia[3][3] + T45[6][3];
JA4[6][4]=-links[1].mcm[2] + T45[6][4];
JA4[6][5]=links[1].mcm[1] + T45[6][5];
JA4[6][6]=T45[6][6];


h4[1]=JA4[1][3];
h4[2]=JA4[2][3];
h4[3]=JA4[3][3];
h4[4]=JA4[4][3];
h4[5]=JA4[5][3];
h4[6]=JA4[6][3];

T134[1][1]=JA4[1][1];
T134[1][2]=JA4[1][2];
T134[1][3]=JA4[1][3];
T134[1][4]=JA4[1][4];
T134[1][5]=JA4[1][5];
T134[1][6]=JA4[1][6];

T134[2][1]=JA4[2][1];
T134[2][2]=JA4[2][2];
T134[2][3]=JA4[2][3];
T134[2][4]=JA4[2][4];
T134[2][5]=JA4[2][5];
T134[2][6]=JA4[2][6];

T134[3][1]=JA4[3][1];
T134[3][2]=JA4[3][2];
T134[3][3]=JA4[3][3];
T134[3][4]=JA4[3][4];
T134[3][5]=JA4[3][5];
T134[3][6]=JA4[3][6];

T134[4][1]=JA4[4][1];
T134[4][2]=JA4[4][2];
T134[4][3]=JA4[4][3];
T134[4][4]=JA4[4][4];
T134[4][5]=JA4[4][5];
T134[4][6]=JA4[4][6];

T134[5][1]=JA4[5][1];
T134[5][2]=JA4[5][2];
T134[5][3]=JA4[5][3];
T134[5][4]=JA4[5][4];
T134[5][5]=JA4[5][5];
T134[5][6]=JA4[5][6];

T134[6][1]=JA4[6][1];
T134[6][2]=JA4[6][2];
T134[6][3]=JA4[6][3];
T134[6][4]=JA4[6][4];
T134[6][5]=JA4[6][5];
T134[6][6]=JA4[6][6];


T34[1][1]=S43[1][1]*(Si34[1][1]*T134[1][1] + Si34[1][2]*T134[2][1] + 0.7071067811865475*T134[3][1]) + S43[2][1]*(Si34[1][1]*T134[1][2] + Si34[1][2]*T134[2][2] + 0.7071067811865475*T134[3][2]) + 0.7071067811865475*(Si34[1][1]*T134[1][3] + Si34[1][2]*T134[2][3] + 0.7071067811865475*T134[3][3]);
T34[1][2]=S43[1][2]*(Si34[1][1]*T134[1][1] + Si34[1][2]*T134[2][1] + 0.7071067811865475*T134[3][1]) + S43[2][2]*(Si34[1][1]*T134[1][2] + Si34[1][2]*T134[2][2] + 0.7071067811865475*T134[3][2]) + THORAX2SHOULDER*S43[1][3]*(Si34[1][1]*T134[1][4] + Si34[1][2]*T134[2][4] + 0.7071067811865475*T134[3][4]) + THORAX2SHOULDER*S43[2][3]*(Si34[1][1]*T134[1][5] + Si34[1][2]*T134[2][5] + 0.7071067811865475*T134[3][5]) - 0.7071067811865475*THORAX2SHOULDER*(Si34[1][1]*T134[1][6] + Si34[1][2]*T134[2][6] + 0.7071067811865475*T134[3][6]);
T34[1][3]=S43[1][3]*(Si34[1][1]*T134[1][1] + Si34[1][2]*T134[2][1] + 0.7071067811865475*T134[3][1]) + S43[2][3]*(Si34[1][1]*T134[1][2] + Si34[1][2]*T134[2][2] + 0.7071067811865475*T134[3][2]) - 0.7071067811865475*(Si34[1][1]*T134[1][3] + Si34[1][2]*T134[2][3] + 0.7071067811865475*T134[3][3]) - THORAX2SHOULDER*S43[1][2]*(Si34[1][1]*T134[1][4] + Si34[1][2]*T134[2][4] + 0.7071067811865475*T134[3][4]) - THORAX2SHOULDER*S43[2][2]*(Si34[1][1]*T134[1][5] + Si34[1][2]*T134[2][5] + 0.7071067811865475*T134[3][5]);
T34[1][4]=S43[1][1]*(Si34[1][1]*T134[1][4] + Si34[1][2]*T134[2][4] + 0.7071067811865475*T134[3][4]) + S43[2][1]*(Si34[1][1]*T134[1][5] + Si34[1][2]*T134[2][5] + 0.7071067811865475*T134[3][5]) + 0.7071067811865475*(Si34[1][1]*T134[1][6] + Si34[1][2]*T134[2][6] + 0.7071067811865475*T134[3][6]);
T34[1][5]=S43[1][2]*(Si34[1][1]*T134[1][4] + Si34[1][2]*T134[2][4] + 0.7071067811865475*T134[3][4]) + S43[2][2]*(Si34[1][1]*T134[1][5] + Si34[1][2]*T134[2][5] + 0.7071067811865475*T134[3][5]);
T34[1][6]=S43[1][3]*(Si34[1][1]*T134[1][4] + Si34[1][2]*T134[2][4] + 0.7071067811865475*T134[3][4]) + S43[2][3]*(Si34[1][1]*T134[1][5] + Si34[1][2]*T134[2][5] + 0.7071067811865475*T134[3][5]) - 0.7071067811865475*(Si34[1][1]*T134[1][6] + Si34[1][2]*T134[2][6] + 0.7071067811865475*T134[3][6]);

T34[2][1]=S43[1][1]*(Si34[2][1]*T134[1][1] + Si34[2][2]*T134[2][1]) + S43[2][1]*(Si34[2][1]*T134[1][2] + Si34[2][2]*T134[2][2]) + 0.7071067811865475*(Si34[2][1]*T134[1][3] + Si34[2][2]*T134[2][3]);
T34[2][2]=S43[1][2]*(Si34[2][1]*T134[1][1] + Si34[2][2]*T134[2][1]) + S43[2][2]*(Si34[2][1]*T134[1][2] + Si34[2][2]*T134[2][2]) + THORAX2SHOULDER*S43[1][3]*(Si34[2][1]*T134[1][4] + Si34[2][2]*T134[2][4]) + THORAX2SHOULDER*S43[2][3]*(Si34[2][1]*T134[1][5] + Si34[2][2]*T134[2][5]) - 0.7071067811865475*THORAX2SHOULDER*(Si34[2][1]*T134[1][6] + Si34[2][2]*T134[2][6]);
T34[2][3]=S43[1][3]*(Si34[2][1]*T134[1][1] + Si34[2][2]*T134[2][1]) + S43[2][3]*(Si34[2][1]*T134[1][2] + Si34[2][2]*T134[2][2]) - 0.7071067811865475*(Si34[2][1]*T134[1][3] + Si34[2][2]*T134[2][3]) - THORAX2SHOULDER*S43[1][2]*(Si34[2][1]*T134[1][4] + Si34[2][2]*T134[2][4]) - THORAX2SHOULDER*S43[2][2]*(Si34[2][1]*T134[1][5] + Si34[2][2]*T134[2][5]);
T34[2][4]=S43[1][1]*(Si34[2][1]*T134[1][4] + Si34[2][2]*T134[2][4]) + S43[2][1]*(Si34[2][1]*T134[1][5] + Si34[2][2]*T134[2][5]) + 0.7071067811865475*(Si34[2][1]*T134[1][6] + Si34[2][2]*T134[2][6]);
T34[2][5]=S43[1][2]*(Si34[2][1]*T134[1][4] + Si34[2][2]*T134[2][4]) + S43[2][2]*(Si34[2][1]*T134[1][5] + Si34[2][2]*T134[2][5]);
T34[2][6]=S43[1][3]*(Si34[2][1]*T134[1][4] + Si34[2][2]*T134[2][4]) + S43[2][3]*(Si34[2][1]*T134[1][5] + Si34[2][2]*T134[2][5]) - 0.7071067811865475*(Si34[2][1]*T134[1][6] + Si34[2][2]*T134[2][6]);

T34[3][1]=S43[1][1]*(Si34[3][1]*T134[1][1] + Si34[3][2]*T134[2][1] - 0.7071067811865475*T134[3][1]) + S43[2][1]*(Si34[3][1]*T134[1][2] + Si34[3][2]*T134[2][2] - 0.7071067811865475*T134[3][2]) + 0.7071067811865475*(Si34[3][1]*T134[1][3] + Si34[3][2]*T134[2][3] - 0.7071067811865475*T134[3][3]);
T34[3][2]=S43[1][2]*(Si34[3][1]*T134[1][1] + Si34[3][2]*T134[2][1] - 0.7071067811865475*T134[3][1]) + S43[2][2]*(Si34[3][1]*T134[1][2] + Si34[3][2]*T134[2][2] - 0.7071067811865475*T134[3][2]) + THORAX2SHOULDER*S43[1][3]*(Si34[3][1]*T134[1][4] + Si34[3][2]*T134[2][4] - 0.7071067811865475*T134[3][4]) + THORAX2SHOULDER*S43[2][3]*(Si34[3][1]*T134[1][5] + Si34[3][2]*T134[2][5] - 0.7071067811865475*T134[3][5]) - 0.7071067811865475*THORAX2SHOULDER*(Si34[3][1]*T134[1][6] + Si34[3][2]*T134[2][6] - 0.7071067811865475*T134[3][6]);
T34[3][3]=S43[1][3]*(Si34[3][1]*T134[1][1] + Si34[3][2]*T134[2][1] - 0.7071067811865475*T134[3][1]) + S43[2][3]*(Si34[3][1]*T134[1][2] + Si34[3][2]*T134[2][2] - 0.7071067811865475*T134[3][2]) - 0.7071067811865475*(Si34[3][1]*T134[1][3] + Si34[3][2]*T134[2][3] - 0.7071067811865475*T134[3][3]) - THORAX2SHOULDER*S43[1][2]*(Si34[3][1]*T134[1][4] + Si34[3][2]*T134[2][4] - 0.7071067811865475*T134[3][4]) - THORAX2SHOULDER*S43[2][2]*(Si34[3][1]*T134[1][5] + Si34[3][2]*T134[2][5] - 0.7071067811865475*T134[3][5]);
T34[3][4]=S43[1][1]*(Si34[3][1]*T134[1][4] + Si34[3][2]*T134[2][4] - 0.7071067811865475*T134[3][4]) + S43[2][1]*(Si34[3][1]*T134[1][5] + Si34[3][2]*T134[2][5] - 0.7071067811865475*T134[3][5]) + 0.7071067811865475*(Si34[3][1]*T134[1][6] + Si34[3][2]*T134[2][6] - 0.7071067811865475*T134[3][6]);
T34[3][5]=S43[1][2]*(Si34[3][1]*T134[1][4] + Si34[3][2]*T134[2][4] - 0.7071067811865475*T134[3][4]) + S43[2][2]*(Si34[3][1]*T134[1][5] + Si34[3][2]*T134[2][5] - 0.7071067811865475*T134[3][5]);
T34[3][6]=S43[1][3]*(Si34[3][1]*T134[1][4] + Si34[3][2]*T134[2][4] - 0.7071067811865475*T134[3][4]) + S43[2][3]*(Si34[3][1]*T134[1][5] + Si34[3][2]*T134[2][5] - 0.7071067811865475*T134[3][5]) - 0.7071067811865475*(Si34[3][1]*T134[1][6] + Si34[3][2]*T134[2][6] - 0.7071067811865475*T134[3][6]);

T34[4][1]=S43[1][1]*(Si34[1][1]*T134[4][1] + Si34[1][2]*T134[5][1] + 0.7071067811865475*T134[6][1]) + S43[2][1]*(Si34[1][1]*T134[4][2] + Si34[1][2]*T134[5][2] + 0.7071067811865475*T134[6][2]) + 0.7071067811865475*(Si34[1][1]*T134[4][3] + Si34[1][2]*T134[5][3] + 0.7071067811865475*T134[6][3]);
T34[4][2]=S43[1][2]*(Si34[1][1]*T134[4][1] + Si34[1][2]*T134[5][1] + 0.7071067811865475*T134[6][1]) + S43[2][2]*(Si34[1][1]*T134[4][2] + Si34[1][2]*T134[5][2] + 0.7071067811865475*T134[6][2]) + THORAX2SHOULDER*S43[1][3]*(Si34[1][1]*T134[4][4] + Si34[1][2]*T134[5][4] + 0.7071067811865475*T134[6][4]) + THORAX2SHOULDER*S43[2][3]*(Si34[1][1]*T134[4][5] + Si34[1][2]*T134[5][5] + 0.7071067811865475*T134[6][5]) - 0.7071067811865475*THORAX2SHOULDER*(Si34[1][1]*T134[4][6] + Si34[1][2]*T134[5][6] + 0.7071067811865475*T134[6][6]);
T34[4][3]=S43[1][3]*(Si34[1][1]*T134[4][1] + Si34[1][2]*T134[5][1] + 0.7071067811865475*T134[6][1]) + S43[2][3]*(Si34[1][1]*T134[4][2] + Si34[1][2]*T134[5][2] + 0.7071067811865475*T134[6][2]) - 0.7071067811865475*(Si34[1][1]*T134[4][3] + Si34[1][2]*T134[5][3] + 0.7071067811865475*T134[6][3]) - THORAX2SHOULDER*S43[1][2]*(Si34[1][1]*T134[4][4] + Si34[1][2]*T134[5][4] + 0.7071067811865475*T134[6][4]) - THORAX2SHOULDER*S43[2][2]*(Si34[1][1]*T134[4][5] + Si34[1][2]*T134[5][5] + 0.7071067811865475*T134[6][5]);
T34[4][4]=S43[1][1]*(Si34[1][1]*T134[4][4] + Si34[1][2]*T134[5][4] + 0.7071067811865475*T134[6][4]) + S43[2][1]*(Si34[1][1]*T134[4][5] + Si34[1][2]*T134[5][5] + 0.7071067811865475*T134[6][5]) + 0.7071067811865475*(Si34[1][1]*T134[4][6] + Si34[1][2]*T134[5][6] + 0.7071067811865475*T134[6][6]);
T34[4][5]=S43[1][2]*(Si34[1][1]*T134[4][4] + Si34[1][2]*T134[5][4] + 0.7071067811865475*T134[6][4]) + S43[2][2]*(Si34[1][1]*T134[4][5] + Si34[1][2]*T134[5][5] + 0.7071067811865475*T134[6][5]);
T34[4][6]=S43[1][3]*(Si34[1][1]*T134[4][4] + Si34[1][2]*T134[5][4] + 0.7071067811865475*T134[6][4]) + S43[2][3]*(Si34[1][1]*T134[4][5] + Si34[1][2]*T134[5][5] + 0.7071067811865475*T134[6][5]) - 0.7071067811865475*(Si34[1][1]*T134[4][6] + Si34[1][2]*T134[5][6] + 0.7071067811865475*T134[6][6]);

T34[5][1]=S43[1][1]*(THORAX2SHOULDER*Si34[3][1]*T134[1][1] + THORAX2SHOULDER*Si34[3][2]*T134[2][1] - 0.7071067811865475*THORAX2SHOULDER*T134[3][1] + Si34[2][1]*T134[4][1] + Si34[2][2]*T134[5][1]) + S43[2][1]*(THORAX2SHOULDER*Si34[3][1]*T134[1][2] + THORAX2SHOULDER*Si34[3][2]*T134[2][2] - 0.7071067811865475*THORAX2SHOULDER*T134[3][2] + Si34[2][1]*T134[4][2] + Si34[2][2]*T134[5][2]) + 0.7071067811865475*(THORAX2SHOULDER*Si34[3][1]*T134[1][3] + THORAX2SHOULDER*Si34[3][2]*T134[2][3] - 0.7071067811865475*THORAX2SHOULDER*T134[3][3] + Si34[2][1]*T134[4][3] + Si34[2][2]*T134[5][3]);
T34[5][2]=S43[1][2]*(THORAX2SHOULDER*Si34[3][1]*T134[1][1] + THORAX2SHOULDER*Si34[3][2]*T134[2][1] - 0.7071067811865475*THORAX2SHOULDER*T134[3][1] + Si34[2][1]*T134[4][1] + Si34[2][2]*T134[5][1]) + S43[2][2]*(THORAX2SHOULDER*Si34[3][1]*T134[1][2] + THORAX2SHOULDER*Si34[3][2]*T134[2][2] - 0.7071067811865475*THORAX2SHOULDER*T134[3][2] + Si34[2][1]*T134[4][2] + Si34[2][2]*T134[5][2]) + THORAX2SHOULDER*S43[1][3]*(THORAX2SHOULDER*Si34[3][1]*T134[1][4] + THORAX2SHOULDER*Si34[3][2]*T134[2][4] - 0.7071067811865475*THORAX2SHOULDER*T134[3][4] + Si34[2][1]*T134[4][4] + Si34[2][2]*T134[5][4]) + THORAX2SHOULDER*S43[2][3]*(THORAX2SHOULDER*Si34[3][1]*T134[1][5] + THORAX2SHOULDER*Si34[3][2]*T134[2][5] - 0.7071067811865475*THORAX2SHOULDER*T134[3][5] + Si34[2][1]*T134[4][5] + Si34[2][2]*T134[5][5]) - 0.7071067811865475*THORAX2SHOULDER*(THORAX2SHOULDER*Si34[3][1]*T134[1][6] + THORAX2SHOULDER*Si34[3][2]*T134[2][6] - 0.7071067811865475*THORAX2SHOULDER*T134[3][6] + Si34[2][1]*T134[4][6] + Si34[2][2]*T134[5][6]);
T34[5][3]=S43[1][3]*(THORAX2SHOULDER*Si34[3][1]*T134[1][1] + THORAX2SHOULDER*Si34[3][2]*T134[2][1] - 0.7071067811865475*THORAX2SHOULDER*T134[3][1] + Si34[2][1]*T134[4][1] + Si34[2][2]*T134[5][1]) + S43[2][3]*(THORAX2SHOULDER*Si34[3][1]*T134[1][2] + THORAX2SHOULDER*Si34[3][2]*T134[2][2] - 0.7071067811865475*THORAX2SHOULDER*T134[3][2] + Si34[2][1]*T134[4][2] + Si34[2][2]*T134[5][2]) - 0.7071067811865475*(THORAX2SHOULDER*Si34[3][1]*T134[1][3] + THORAX2SHOULDER*Si34[3][2]*T134[2][3] - 0.7071067811865475*THORAX2SHOULDER*T134[3][3] + Si34[2][1]*T134[4][3] + Si34[2][2]*T134[5][3]) - THORAX2SHOULDER*S43[1][2]*(THORAX2SHOULDER*Si34[3][1]*T134[1][4] + THORAX2SHOULDER*Si34[3][2]*T134[2][4] - 0.7071067811865475*THORAX2SHOULDER*T134[3][4] + Si34[2][1]*T134[4][4] + Si34[2][2]*T134[5][4]) - THORAX2SHOULDER*S43[2][2]*(THORAX2SHOULDER*Si34[3][1]*T134[1][5] + THORAX2SHOULDER*Si34[3][2]*T134[2][5] - 0.7071067811865475*THORAX2SHOULDER*T134[3][5] + Si34[2][1]*T134[4][5] + Si34[2][2]*T134[5][5]);
T34[5][4]=S43[1][1]*(THORAX2SHOULDER*Si34[3][1]*T134[1][4] + THORAX2SHOULDER*Si34[3][2]*T134[2][4] - 0.7071067811865475*THORAX2SHOULDER*T134[3][4] + Si34[2][1]*T134[4][4] + Si34[2][2]*T134[5][4]) + S43[2][1]*(THORAX2SHOULDER*Si34[3][1]*T134[1][5] + THORAX2SHOULDER*Si34[3][2]*T134[2][5] - 0.7071067811865475*THORAX2SHOULDER*T134[3][5] + Si34[2][1]*T134[4][5] + Si34[2][2]*T134[5][5]) + 0.7071067811865475*(THORAX2SHOULDER*Si34[3][1]*T134[1][6] + THORAX2SHOULDER*Si34[3][2]*T134[2][6] - 0.7071067811865475*THORAX2SHOULDER*T134[3][6] + Si34[2][1]*T134[4][6] + Si34[2][2]*T134[5][6]);
T34[5][5]=S43[1][2]*(THORAX2SHOULDER*Si34[3][1]*T134[1][4] + THORAX2SHOULDER*Si34[3][2]*T134[2][4] - 0.7071067811865475*THORAX2SHOULDER*T134[3][4] + Si34[2][1]*T134[4][4] + Si34[2][2]*T134[5][4]) + S43[2][2]*(THORAX2SHOULDER*Si34[3][1]*T134[1][5] + THORAX2SHOULDER*Si34[3][2]*T134[2][5] - 0.7071067811865475*THORAX2SHOULDER*T134[3][5] + Si34[2][1]*T134[4][5] + Si34[2][2]*T134[5][5]);
T34[5][6]=S43[1][3]*(THORAX2SHOULDER*Si34[3][1]*T134[1][4] + THORAX2SHOULDER*Si34[3][2]*T134[2][4] - 0.7071067811865475*THORAX2SHOULDER*T134[3][4] + Si34[2][1]*T134[4][4] + Si34[2][2]*T134[5][4]) + S43[2][3]*(THORAX2SHOULDER*Si34[3][1]*T134[1][5] + THORAX2SHOULDER*Si34[3][2]*T134[2][5] - 0.7071067811865475*THORAX2SHOULDER*T134[3][5] + Si34[2][1]*T134[4][5] + Si34[2][2]*T134[5][5]) - 0.7071067811865475*(THORAX2SHOULDER*Si34[3][1]*T134[1][6] + THORAX2SHOULDER*Si34[3][2]*T134[2][6] - 0.7071067811865475*THORAX2SHOULDER*T134[3][6] + Si34[2][1]*T134[4][6] + Si34[2][2]*T134[5][6]);

T34[6][1]=S43[1][1]*(-(THORAX2SHOULDER*Si34[2][1]*T134[1][1]) - THORAX2SHOULDER*Si34[2][2]*T134[2][1] + Si34[3][1]*T134[4][1] + Si34[3][2]*T134[5][1] - 0.7071067811865475*T134[6][1]) + S43[2][1]*(-(THORAX2SHOULDER*Si34[2][1]*T134[1][2]) - THORAX2SHOULDER*Si34[2][2]*T134[2][2] + Si34[3][1]*T134[4][2] + Si34[3][2]*T134[5][2] - 0.7071067811865475*T134[6][2]) + 0.7071067811865475*(-(THORAX2SHOULDER*Si34[2][1]*T134[1][3]) - THORAX2SHOULDER*Si34[2][2]*T134[2][3] + Si34[3][1]*T134[4][3] + Si34[3][2]*T134[5][3] - 0.7071067811865475*T134[6][3]);
T34[6][2]=S43[1][2]*(-(THORAX2SHOULDER*Si34[2][1]*T134[1][1]) - THORAX2SHOULDER*Si34[2][2]*T134[2][1] + Si34[3][1]*T134[4][1] + Si34[3][2]*T134[5][1] - 0.7071067811865475*T134[6][1]) + S43[2][2]*(-(THORAX2SHOULDER*Si34[2][1]*T134[1][2]) - THORAX2SHOULDER*Si34[2][2]*T134[2][2] + Si34[3][1]*T134[4][2] + Si34[3][2]*T134[5][2] - 0.7071067811865475*T134[6][2]) + THORAX2SHOULDER*S43[1][3]*(-(THORAX2SHOULDER*Si34[2][1]*T134[1][4]) - THORAX2SHOULDER*Si34[2][2]*T134[2][4] + Si34[3][1]*T134[4][4] + Si34[3][2]*T134[5][4] - 0.7071067811865475*T134[6][4]) + THORAX2SHOULDER*S43[2][3]*(-(THORAX2SHOULDER*Si34[2][1]*T134[1][5]) - THORAX2SHOULDER*Si34[2][2]*T134[2][5] + Si34[3][1]*T134[4][5] + Si34[3][2]*T134[5][5] - 0.7071067811865475*T134[6][5]) - 0.7071067811865475*THORAX2SHOULDER*(-(THORAX2SHOULDER*Si34[2][1]*T134[1][6]) - THORAX2SHOULDER*Si34[2][2]*T134[2][6] + Si34[3][1]*T134[4][6] + Si34[3][2]*T134[5][6] - 0.7071067811865475*T134[6][6]);
T34[6][3]=S43[1][3]*(-(THORAX2SHOULDER*Si34[2][1]*T134[1][1]) - THORAX2SHOULDER*Si34[2][2]*T134[2][1] + Si34[3][1]*T134[4][1] + Si34[3][2]*T134[5][1] - 0.7071067811865475*T134[6][1]) + S43[2][3]*(-(THORAX2SHOULDER*Si34[2][1]*T134[1][2]) - THORAX2SHOULDER*Si34[2][2]*T134[2][2] + Si34[3][1]*T134[4][2] + Si34[3][2]*T134[5][2] - 0.7071067811865475*T134[6][2]) - 0.7071067811865475*(-(THORAX2SHOULDER*Si34[2][1]*T134[1][3]) - THORAX2SHOULDER*Si34[2][2]*T134[2][3] + Si34[3][1]*T134[4][3] + Si34[3][2]*T134[5][3] - 0.7071067811865475*T134[6][3]) - THORAX2SHOULDER*S43[1][2]*(-(THORAX2SHOULDER*Si34[2][1]*T134[1][4]) - THORAX2SHOULDER*Si34[2][2]*T134[2][4] + Si34[3][1]*T134[4][4] + Si34[3][2]*T134[5][4] - 0.7071067811865475*T134[6][4]) - THORAX2SHOULDER*S43[2][2]*(-(THORAX2SHOULDER*Si34[2][1]*T134[1][5]) - THORAX2SHOULDER*Si34[2][2]*T134[2][5] + Si34[3][1]*T134[4][5] + Si34[3][2]*T134[5][5] - 0.7071067811865475*T134[6][5]);
T34[6][4]=S43[1][1]*(-(THORAX2SHOULDER*Si34[2][1]*T134[1][4]) - THORAX2SHOULDER*Si34[2][2]*T134[2][4] + Si34[3][1]*T134[4][4] + Si34[3][2]*T134[5][4] - 0.7071067811865475*T134[6][4]) + S43[2][1]*(-(THORAX2SHOULDER*Si34[2][1]*T134[1][5]) - THORAX2SHOULDER*Si34[2][2]*T134[2][5] + Si34[3][1]*T134[4][5] + Si34[3][2]*T134[5][5] - 0.7071067811865475*T134[6][5]) + 0.7071067811865475*(-(THORAX2SHOULDER*Si34[2][1]*T134[1][6]) - THORAX2SHOULDER*Si34[2][2]*T134[2][6] + Si34[3][1]*T134[4][6] + Si34[3][2]*T134[5][6] - 0.7071067811865475*T134[6][6]);
T34[6][5]=S43[1][2]*(-(THORAX2SHOULDER*Si34[2][1]*T134[1][4]) - THORAX2SHOULDER*Si34[2][2]*T134[2][4] + Si34[3][1]*T134[4][4] + Si34[3][2]*T134[5][4] - 0.7071067811865475*T134[6][4]) + S43[2][2]*(-(THORAX2SHOULDER*Si34[2][1]*T134[1][5]) - THORAX2SHOULDER*Si34[2][2]*T134[2][5] + Si34[3][1]*T134[4][5] + Si34[3][2]*T134[5][5] - 0.7071067811865475*T134[6][5]);
T34[6][6]=S43[1][3]*(-(THORAX2SHOULDER*Si34[2][1]*T134[1][4]) - THORAX2SHOULDER*Si34[2][2]*T134[2][4] + Si34[3][1]*T134[4][4] + Si34[3][2]*T134[5][4] - 0.7071067811865475*T134[6][4]) + S43[2][3]*(-(THORAX2SHOULDER*Si34[2][1]*T134[1][5]) - THORAX2SHOULDER*Si34[2][2]*T134[2][5] + Si34[3][1]*T134[4][5] + Si34[3][2]*T134[5][5] - 0.7071067811865475*T134[6][5]) - 0.7071067811865475*(-(THORAX2SHOULDER*Si34[2][1]*T134[1][6]) - THORAX2SHOULDER*Si34[2][2]*T134[2][6] + Si34[3][1]*T134[4][6] + Si34[3][2]*T134[5][6] - 0.7071067811865475*T134[6][6]);



}


void
hermes_InvDynArtfunc101(void)
       {
JA3[1][1]=T332[1][1] + T34[1][1] + T360[1][1];
JA3[1][2]=links[31].mcm[3] + T332[1][2] + T34[1][2] + T360[1][2];
JA3[1][3]=-links[31].mcm[2] + T332[1][3] + T34[1][3] + T360[1][3];
JA3[1][4]=links[31].m + T332[1][4] + T34[1][4] + T360[1][4];
JA3[1][5]=T332[1][5] + T34[1][5] + T360[1][5];
JA3[1][6]=T332[1][6] + T34[1][6] + T360[1][6];

JA3[2][1]=-links[31].mcm[3] + T332[2][1] + T34[2][1] + T360[2][1];
JA3[2][2]=T332[2][2] + T34[2][2] + T360[2][2];
JA3[2][3]=links[31].mcm[1] + T332[2][3] + T34[2][3] + T360[2][3];
JA3[2][4]=T332[2][4] + T34[2][4] + T360[2][4];
JA3[2][5]=links[31].m + T332[2][5] + T34[2][5] + T360[2][5];
JA3[2][6]=T332[2][6] + T34[2][6] + T360[2][6];

JA3[3][1]=links[31].mcm[2] + T332[3][1] + T34[3][1] + T360[3][1];
JA3[3][2]=-links[31].mcm[1] + T332[3][2] + T34[3][2] + T360[3][2];
JA3[3][3]=T332[3][3] + T34[3][3] + T360[3][3];
JA3[3][4]=T332[3][4] + T34[3][4] + T360[3][4];
JA3[3][5]=T332[3][5] + T34[3][5] + T360[3][5];
JA3[3][6]=links[31].m + T332[3][6] + T34[3][6] + T360[3][6];

JA3[4][1]=links[31].inertia[1][1] + T332[4][1] + T34[4][1] + T360[4][1];
JA3[4][2]=links[31].inertia[1][2] + T332[4][2] + T34[4][2] + T360[4][2];
JA3[4][3]=links[31].inertia[1][3] + T332[4][3] + T34[4][3] + T360[4][3];
JA3[4][4]=T332[4][4] + T34[4][4] + T360[4][4];
JA3[4][5]=-links[31].mcm[3] + T332[4][5] + T34[4][5] + T360[4][5];
JA3[4][6]=links[31].mcm[2] + T332[4][6] + T34[4][6] + T360[4][6];

JA3[5][1]=links[31].inertia[1][2] + T332[5][1] + T34[5][1] + T360[5][1];
JA3[5][2]=links[31].inertia[2][2] + T332[5][2] + T34[5][2] + T360[5][2];
JA3[5][3]=links[31].inertia[2][3] + T332[5][3] + T34[5][3] + T360[5][3];
JA3[5][4]=links[31].mcm[3] + T332[5][4] + T34[5][4] + T360[5][4];
JA3[5][5]=T332[5][5] + T34[5][5] + T360[5][5];
JA3[5][6]=-links[31].mcm[1] + T332[5][6] + T34[5][6] + T360[5][6];

JA3[6][1]=links[31].inertia[1][3] + T332[6][1] + T34[6][1] + T360[6][1];
JA3[6][2]=links[31].inertia[2][3] + T332[6][2] + T34[6][2] + T360[6][2];
JA3[6][3]=links[31].inertia[3][3] + T332[6][3] + T34[6][3] + T360[6][3];
JA3[6][4]=-links[31].mcm[2] + T332[6][4] + T34[6][4] + T360[6][4];
JA3[6][5]=links[31].mcm[1] + T332[6][5] + T34[6][5] + T360[6][5];
JA3[6][6]=T332[6][6] + T34[6][6] + T360[6][6];


h3[1]=JA3[1][3];
h3[2]=JA3[2][3];
h3[3]=JA3[3][3];
h3[4]=JA3[4][3];
h3[5]=JA3[5][3];
h3[6]=JA3[6][3];

T123[1][1]=JA3[1][1];
T123[1][2]=JA3[1][2];
T123[1][3]=JA3[1][3];
T123[1][4]=JA3[1][4];
T123[1][5]=JA3[1][5];
T123[1][6]=JA3[1][6];

T123[2][1]=JA3[2][1];
T123[2][2]=JA3[2][2];
T123[2][3]=JA3[2][3];
T123[2][4]=JA3[2][4];
T123[2][5]=JA3[2][5];
T123[2][6]=JA3[2][6];

T123[3][1]=JA3[3][1];
T123[3][2]=JA3[3][2];
T123[3][3]=JA3[3][3];
T123[3][4]=JA3[3][4];
T123[3][5]=JA3[3][5];
T123[3][6]=JA3[3][6];

T123[4][1]=JA3[4][1];
T123[4][2]=JA3[4][2];
T123[4][3]=JA3[4][3];
T123[4][4]=JA3[4][4];
T123[4][5]=JA3[4][5];
T123[4][6]=JA3[4][6];

T123[5][1]=JA3[5][1];
T123[5][2]=JA3[5][2];
T123[5][3]=JA3[5][3];
T123[5][4]=JA3[5][4];
T123[5][5]=JA3[5][5];
T123[5][6]=JA3[5][6];

T123[6][1]=JA3[6][1];
T123[6][2]=JA3[6][2];
T123[6][3]=JA3[6][3];
T123[6][4]=JA3[6][4];
T123[6][5]=JA3[6][5];
T123[6][6]=JA3[6][6];


T23[1][1]=S32[1][1]*(Si23[1][1]*T123[1][1] + Si23[1][2]*T123[2][1]) + S32[2][1]*(Si23[1][1]*T123[1][2] + Si23[1][2]*T123[2][2]);
T23[1][2]=Si23[1][1]*T123[1][3] + Si23[1][2]*T123[2][3];
T23[1][3]=S32[1][3]*(Si23[1][1]*T123[1][1] + Si23[1][2]*T123[2][1]) + S32[2][3]*(Si23[1][1]*T123[1][2] + Si23[1][2]*T123[2][2]);
T23[1][4]=S32[1][1]*(Si23[1][1]*T123[1][4] + Si23[1][2]*T123[2][4]) + S32[2][1]*(Si23[1][1]*T123[1][5] + Si23[1][2]*T123[2][5]);
T23[1][5]=Si23[1][1]*T123[1][6] + Si23[1][2]*T123[2][6];
T23[1][6]=S32[1][3]*(Si23[1][1]*T123[1][4] + Si23[1][2]*T123[2][4]) + S32[2][3]*(Si23[1][1]*T123[1][5] + Si23[1][2]*T123[2][5]);

T23[2][1]=S32[1][1]*T123[3][1] + S32[2][1]*T123[3][2];
T23[2][2]=T123[3][3];
T23[2][3]=S32[1][3]*T123[3][1] + S32[2][3]*T123[3][2];
T23[2][4]=S32[1][1]*T123[3][4] + S32[2][1]*T123[3][5];
T23[2][5]=T123[3][6];
T23[2][6]=S32[1][3]*T123[3][4] + S32[2][3]*T123[3][5];

T23[3][1]=S32[1][1]*(Si23[3][1]*T123[1][1] + Si23[3][2]*T123[2][1]) + S32[2][1]*(Si23[3][1]*T123[1][2] + Si23[3][2]*T123[2][2]);
T23[3][2]=Si23[3][1]*T123[1][3] + Si23[3][2]*T123[2][3];
T23[3][3]=S32[1][3]*(Si23[3][1]*T123[1][1] + Si23[3][2]*T123[2][1]) + S32[2][3]*(Si23[3][1]*T123[1][2] + Si23[3][2]*T123[2][2]);
T23[3][4]=S32[1][1]*(Si23[3][1]*T123[1][4] + Si23[3][2]*T123[2][4]) + S32[2][1]*(Si23[3][1]*T123[1][5] + Si23[3][2]*T123[2][5]);
T23[3][5]=Si23[3][1]*T123[1][6] + Si23[3][2]*T123[2][6];
T23[3][6]=S32[1][3]*(Si23[3][1]*T123[1][4] + Si23[3][2]*T123[2][4]) + S32[2][3]*(Si23[3][1]*T123[1][5] + Si23[3][2]*T123[2][5]);

T23[4][1]=S32[1][1]*(Si23[1][1]*T123[4][1] + Si23[1][2]*T123[5][1]) + S32[2][1]*(Si23[1][1]*T123[4][2] + Si23[1][2]*T123[5][2]);
T23[4][2]=Si23[1][1]*T123[4][3] + Si23[1][2]*T123[5][3];
T23[4][3]=S32[1][3]*(Si23[1][1]*T123[4][1] + Si23[1][2]*T123[5][1]) + S32[2][3]*(Si23[1][1]*T123[4][2] + Si23[1][2]*T123[5][2]);
T23[4][4]=S32[1][1]*(Si23[1][1]*T123[4][4] + Si23[1][2]*T123[5][4]) + S32[2][1]*(Si23[1][1]*T123[4][5] + Si23[1][2]*T123[5][5]);
T23[4][5]=Si23[1][1]*T123[4][6] + Si23[1][2]*T123[5][6];
T23[4][6]=S32[1][3]*(Si23[1][1]*T123[4][4] + Si23[1][2]*T123[5][4]) + S32[2][3]*(Si23[1][1]*T123[4][5] + Si23[1][2]*T123[5][5]);

T23[5][1]=S32[1][1]*T123[6][1] + S32[2][1]*T123[6][2];
T23[5][2]=T123[6][3];
T23[5][3]=S32[1][3]*T123[6][1] + S32[2][3]*T123[6][2];
T23[5][4]=S32[1][1]*T123[6][4] + S32[2][1]*T123[6][5];
T23[5][5]=T123[6][6];
T23[5][6]=S32[1][3]*T123[6][4] + S32[2][3]*T123[6][5];

T23[6][1]=S32[1][1]*(Si23[3][1]*T123[4][1] + Si23[3][2]*T123[5][1]) + S32[2][1]*(Si23[3][1]*T123[4][2] + Si23[3][2]*T123[5][2]);
T23[6][2]=Si23[3][1]*T123[4][3] + Si23[3][2]*T123[5][3];
T23[6][3]=S32[1][3]*(Si23[3][1]*T123[4][1] + Si23[3][2]*T123[5][1]) + S32[2][3]*(Si23[3][1]*T123[4][2] + Si23[3][2]*T123[5][2]);
T23[6][4]=S32[1][1]*(Si23[3][1]*T123[4][4] + Si23[3][2]*T123[5][4]) + S32[2][1]*(Si23[3][1]*T123[4][5] + Si23[3][2]*T123[5][5]);
T23[6][5]=Si23[3][1]*T123[4][6] + Si23[3][2]*T123[5][6];
T23[6][6]=S32[1][3]*(Si23[3][1]*T123[4][4] + Si23[3][2]*T123[5][4]) + S32[2][3]*(Si23[3][1]*T123[4][5] + Si23[3][2]*T123[5][5]);



}


void
hermes_InvDynArtfunc102(void)
       {
JA2[1][1]=T23[1][1];
JA2[1][2]=links[30].mcm[3] + T23[1][2];
JA2[1][3]=-links[30].mcm[2] + T23[1][3];
JA2[1][4]=links[30].m + T23[1][4];
JA2[1][5]=T23[1][5];
JA2[1][6]=T23[1][6];

JA2[2][1]=-links[30].mcm[3] + T23[2][1];
JA2[2][2]=T23[2][2];
JA2[2][3]=links[30].mcm[1] + T23[2][3];
JA2[2][4]=T23[2][4];
JA2[2][5]=links[30].m + T23[2][5];
JA2[2][6]=T23[2][6];

JA2[3][1]=links[30].mcm[2] + T23[3][1];
JA2[3][2]=-links[30].mcm[1] + T23[3][2];
JA2[3][3]=T23[3][3];
JA2[3][4]=T23[3][4];
JA2[3][5]=T23[3][5];
JA2[3][6]=links[30].m + T23[3][6];

JA2[4][1]=links[30].inertia[1][1] + T23[4][1];
JA2[4][2]=links[30].inertia[1][2] + T23[4][2];
JA2[4][3]=links[30].inertia[1][3] + T23[4][3];
JA2[4][4]=T23[4][4];
JA2[4][5]=-links[30].mcm[3] + T23[4][5];
JA2[4][6]=links[30].mcm[2] + T23[4][6];

JA2[5][1]=links[30].inertia[1][2] + T23[5][1];
JA2[5][2]=links[30].inertia[2][2] + T23[5][2];
JA2[5][3]=links[30].inertia[2][3] + T23[5][3];
JA2[5][4]=links[30].mcm[3] + T23[5][4];
JA2[5][5]=T23[5][5];
JA2[5][6]=-links[30].mcm[1] + T23[5][6];

JA2[6][1]=links[30].inertia[1][3] + T23[6][1];
JA2[6][2]=links[30].inertia[2][3] + T23[6][2];
JA2[6][3]=links[30].inertia[3][3] + T23[6][3];
JA2[6][4]=-links[30].mcm[2] + T23[6][4];
JA2[6][5]=links[30].mcm[1] + T23[6][5];
JA2[6][6]=T23[6][6];


h2[1]=JA2[1][3];
h2[2]=JA2[2][3];
h2[3]=JA2[3][3];
h2[4]=JA2[4][3];
h2[5]=JA2[5][3];
h2[6]=JA2[6][3];

T112[1][1]=JA2[1][1];
T112[1][2]=JA2[1][2];
T112[1][3]=JA2[1][3];
T112[1][4]=JA2[1][4];
T112[1][5]=JA2[1][5];
T112[1][6]=JA2[1][6];

T112[2][1]=JA2[2][1];
T112[2][2]=JA2[2][2];
T112[2][3]=JA2[2][3];
T112[2][4]=JA2[2][4];
T112[2][5]=JA2[2][5];
T112[2][6]=JA2[2][6];

T112[3][1]=JA2[3][1];
T112[3][2]=JA2[3][2];
T112[3][3]=JA2[3][3];
T112[3][4]=JA2[3][4];
T112[3][5]=JA2[3][5];
T112[3][6]=JA2[3][6];

T112[4][1]=JA2[4][1];
T112[4][2]=JA2[4][2];
T112[4][3]=JA2[4][3];
T112[4][4]=JA2[4][4];
T112[4][5]=JA2[4][5];
T112[4][6]=JA2[4][6];

T112[5][1]=JA2[5][1];
T112[5][2]=JA2[5][2];
T112[5][3]=JA2[5][3];
T112[5][4]=JA2[5][4];
T112[5][5]=JA2[5][5];
T112[5][6]=JA2[5][6];

T112[6][1]=JA2[6][1];
T112[6][2]=JA2[6][2];
T112[6][3]=JA2[6][3];
T112[6][4]=JA2[6][4];
T112[6][5]=JA2[6][5];
T112[6][6]=JA2[6][6];


T12[1][1]=S21[1][1]*(Si12[1][1]*T112[1][1] + Si12[1][2]*T112[2][1]) + S21[2][1]*(Si12[1][1]*T112[1][2] + Si12[1][2]*T112[2][2]);
T12[1][2]=-(Si12[1][1]*T112[1][3]) - Si12[1][2]*T112[2][3];
T12[1][3]=S21[1][3]*(Si12[1][1]*T112[1][1] + Si12[1][2]*T112[2][1]) + S21[2][3]*(Si12[1][1]*T112[1][2] + Si12[1][2]*T112[2][2]);
T12[1][4]=S21[1][1]*(Si12[1][1]*T112[1][4] + Si12[1][2]*T112[2][4]) + S21[2][1]*(Si12[1][1]*T112[1][5] + Si12[1][2]*T112[2][5]);
T12[1][5]=-(Si12[1][1]*T112[1][6]) - Si12[1][2]*T112[2][6];
T12[1][6]=S21[1][3]*(Si12[1][1]*T112[1][4] + Si12[1][2]*T112[2][4]) + S21[2][3]*(Si12[1][1]*T112[1][5] + Si12[1][2]*T112[2][5]);

T12[2][1]=-(S21[1][1]*T112[3][1]) - S21[2][1]*T112[3][2];
T12[2][2]=T112[3][3];
T12[2][3]=-(S21[1][3]*T112[3][1]) - S21[2][3]*T112[3][2];
T12[2][4]=-(S21[1][1]*T112[3][4]) - S21[2][1]*T112[3][5];
T12[2][5]=T112[3][6];
T12[2][6]=-(S21[1][3]*T112[3][4]) - S21[2][3]*T112[3][5];

T12[3][1]=S21[1][1]*(Si12[3][1]*T112[1][1] + Si12[3][2]*T112[2][1]) + S21[2][1]*(Si12[3][1]*T112[1][2] + Si12[3][2]*T112[2][2]);
T12[3][2]=-(Si12[3][1]*T112[1][3]) - Si12[3][2]*T112[2][3];
T12[3][3]=S21[1][3]*(Si12[3][1]*T112[1][1] + Si12[3][2]*T112[2][1]) + S21[2][3]*(Si12[3][1]*T112[1][2] + Si12[3][2]*T112[2][2]);
T12[3][4]=S21[1][1]*(Si12[3][1]*T112[1][4] + Si12[3][2]*T112[2][4]) + S21[2][1]*(Si12[3][1]*T112[1][5] + Si12[3][2]*T112[2][5]);
T12[3][5]=-(Si12[3][1]*T112[1][6]) - Si12[3][2]*T112[2][6];
T12[3][6]=S21[1][3]*(Si12[3][1]*T112[1][4] + Si12[3][2]*T112[2][4]) + S21[2][3]*(Si12[3][1]*T112[1][5] + Si12[3][2]*T112[2][5]);

T12[4][1]=S21[1][1]*(Si12[1][1]*T112[4][1] + Si12[1][2]*T112[5][1]) + S21[2][1]*(Si12[1][1]*T112[4][2] + Si12[1][2]*T112[5][2]);
T12[4][2]=-(Si12[1][1]*T112[4][3]) - Si12[1][2]*T112[5][3];
T12[4][3]=S21[1][3]*(Si12[1][1]*T112[4][1] + Si12[1][2]*T112[5][1]) + S21[2][3]*(Si12[1][1]*T112[4][2] + Si12[1][2]*T112[5][2]);
T12[4][4]=S21[1][1]*(Si12[1][1]*T112[4][4] + Si12[1][2]*T112[5][4]) + S21[2][1]*(Si12[1][1]*T112[4][5] + Si12[1][2]*T112[5][5]);
T12[4][5]=-(Si12[1][1]*T112[4][6]) - Si12[1][2]*T112[5][6];
T12[4][6]=S21[1][3]*(Si12[1][1]*T112[4][4] + Si12[1][2]*T112[5][4]) + S21[2][3]*(Si12[1][1]*T112[4][5] + Si12[1][2]*T112[5][5]);

T12[5][1]=-(S21[1][1]*T112[6][1]) - S21[2][1]*T112[6][2];
T12[5][2]=T112[6][3];
T12[5][3]=-(S21[1][3]*T112[6][1]) - S21[2][3]*T112[6][2];
T12[5][4]=-(S21[1][1]*T112[6][4]) - S21[2][1]*T112[6][5];
T12[5][5]=T112[6][6];
T12[5][6]=-(S21[1][3]*T112[6][4]) - S21[2][3]*T112[6][5];

T12[6][1]=S21[1][1]*(Si12[3][1]*T112[4][1] + Si12[3][2]*T112[5][1]) + S21[2][1]*(Si12[3][1]*T112[4][2] + Si12[3][2]*T112[5][2]);
T12[6][2]=-(Si12[3][1]*T112[4][3]) - Si12[3][2]*T112[5][3];
T12[6][3]=S21[1][3]*(Si12[3][1]*T112[4][1] + Si12[3][2]*T112[5][1]) + S21[2][3]*(Si12[3][1]*T112[4][2] + Si12[3][2]*T112[5][2]);
T12[6][4]=S21[1][1]*(Si12[3][1]*T112[4][4] + Si12[3][2]*T112[5][4]) + S21[2][1]*(Si12[3][1]*T112[4][5] + Si12[3][2]*T112[5][5]);
T12[6][5]=-(Si12[3][1]*T112[4][6]) - Si12[3][2]*T112[5][6];
T12[6][6]=S21[1][3]*(Si12[3][1]*T112[4][4] + Si12[3][2]*T112[5][4]) + S21[2][3]*(Si12[3][1]*T112[4][5] + Si12[3][2]*T112[5][5]);



}


void
hermes_InvDynArtfunc103(void)
       {
JA1[1][1]=T12[1][1];
JA1[1][2]=links[29].mcm[3] + T12[1][2];
JA1[1][3]=-links[29].mcm[2] + T12[1][3];
JA1[1][4]=links[29].m + T12[1][4];
JA1[1][5]=T12[1][5];
JA1[1][6]=T12[1][6];

JA1[2][1]=-links[29].mcm[3] + T12[2][1];
JA1[2][2]=T12[2][2];
JA1[2][3]=links[29].mcm[1] + T12[2][3];
JA1[2][4]=T12[2][4];
JA1[2][5]=links[29].m + T12[2][5];
JA1[2][6]=T12[2][6];

JA1[3][1]=links[29].mcm[2] + T12[3][1];
JA1[3][2]=-links[29].mcm[1] + T12[3][2];
JA1[3][3]=T12[3][3];
JA1[3][4]=T12[3][4];
JA1[3][5]=T12[3][5];
JA1[3][6]=links[29].m + T12[3][6];

JA1[4][1]=links[29].inertia[1][1] + T12[4][1];
JA1[4][2]=links[29].inertia[1][2] + T12[4][2];
JA1[4][3]=links[29].inertia[1][3] + T12[4][3];
JA1[4][4]=T12[4][4];
JA1[4][5]=-links[29].mcm[3] + T12[4][5];
JA1[4][6]=links[29].mcm[2] + T12[4][6];

JA1[5][1]=links[29].inertia[1][2] + T12[5][1];
JA1[5][2]=links[29].inertia[2][2] + T12[5][2];
JA1[5][3]=links[29].inertia[2][3] + T12[5][3];
JA1[5][4]=links[29].mcm[3] + T12[5][4];
JA1[5][5]=T12[5][5];
JA1[5][6]=-links[29].mcm[1] + T12[5][6];

JA1[6][1]=links[29].inertia[1][3] + T12[6][1];
JA1[6][2]=links[29].inertia[2][3] + T12[6][2];
JA1[6][3]=links[29].inertia[3][3] + T12[6][3];
JA1[6][4]=-links[29].mcm[2] + T12[6][4];
JA1[6][5]=links[29].mcm[1] + T12[6][5];
JA1[6][6]=T12[6][6];


h1[1]=JA1[1][3];
h1[2]=JA1[2][3];
h1[3]=JA1[3][3];
h1[4]=JA1[4][3];
h1[5]=JA1[5][3];
h1[6]=JA1[6][3];

T101[1][1]=JA1[1][1];
T101[1][2]=JA1[1][2];
T101[1][3]=JA1[1][3];
T101[1][4]=JA1[1][4];
T101[1][5]=JA1[1][5];
T101[1][6]=JA1[1][6];

T101[2][1]=JA1[2][1];
T101[2][2]=JA1[2][2];
T101[2][3]=JA1[2][3];
T101[2][4]=JA1[2][4];
T101[2][5]=JA1[2][5];
T101[2][6]=JA1[2][6];

T101[3][1]=JA1[3][1];
T101[3][2]=JA1[3][2];
T101[3][3]=JA1[3][3];
T101[3][4]=JA1[3][4];
T101[3][5]=JA1[3][5];
T101[3][6]=JA1[3][6];

T101[4][1]=JA1[4][1];
T101[4][2]=JA1[4][2];
T101[4][3]=JA1[4][3];
T101[4][4]=JA1[4][4];
T101[4][5]=JA1[4][5];
T101[4][6]=JA1[4][6];

T101[5][1]=JA1[5][1];
T101[5][2]=JA1[5][2];
T101[5][3]=JA1[5][3];
T101[5][4]=JA1[5][4];
T101[5][5]=JA1[5][5];
T101[5][6]=JA1[5][6];

T101[6][1]=JA1[6][1];
T101[6][2]=JA1[6][2];
T101[6][3]=JA1[6][3];
T101[6][4]=JA1[6][4];
T101[6][5]=JA1[6][5];
T101[6][6]=JA1[6][6];


T01[1][1]=S10[1][1]*(Si01[1][1]*T101[1][1] + Si01[1][2]*T101[2][1]) + S10[2][1]*(Si01[1][1]*T101[1][2] + Si01[1][2]*T101[2][2]) - PELVIS2THORAX*S10[1][2]*(Si01[1][1]*T101[1][4] + Si01[1][2]*T101[2][4]) - PELVIS2THORAX*S10[2][2]*(Si01[1][1]*T101[1][5] + Si01[1][2]*T101[2][5]) + PELVISOFFSET*(Si01[1][1]*T101[1][6] + Si01[1][2]*T101[2][6]);
T01[1][2]=S10[1][2]*(Si01[1][1]*T101[1][1] + Si01[1][2]*T101[2][1]) + S10[2][2]*(Si01[1][1]*T101[1][2] + Si01[1][2]*T101[2][2]) + PELVIS2THORAX*S10[1][1]*(Si01[1][1]*T101[1][4] + Si01[1][2]*T101[2][4]) + PELVIS2THORAX*S10[2][1]*(Si01[1][1]*T101[1][5] + Si01[1][2]*T101[2][5]);
T01[1][3]=-(Si01[1][1]*T101[1][3]) - Si01[1][2]*T101[2][3] + PELVISOFFSET*S10[1][1]*(Si01[1][1]*T101[1][4] + Si01[1][2]*T101[2][4]) + PELVISOFFSET*S10[2][1]*(Si01[1][1]*T101[1][5] + Si01[1][2]*T101[2][5]);
T01[1][4]=S10[1][1]*(Si01[1][1]*T101[1][4] + Si01[1][2]*T101[2][4]) + S10[2][1]*(Si01[1][1]*T101[1][5] + Si01[1][2]*T101[2][5]);
T01[1][5]=S10[1][2]*(Si01[1][1]*T101[1][4] + Si01[1][2]*T101[2][4]) + S10[2][2]*(Si01[1][1]*T101[1][5] + Si01[1][2]*T101[2][5]);
T01[1][6]=-(Si01[1][1]*T101[1][6]) - Si01[1][2]*T101[2][6];

T01[2][1]=S10[1][1]*(Si01[2][1]*T101[1][1] + Si01[2][2]*T101[2][1]) + S10[2][1]*(Si01[2][1]*T101[1][2] + Si01[2][2]*T101[2][2]) - PELVIS2THORAX*S10[1][2]*(Si01[2][1]*T101[1][4] + Si01[2][2]*T101[2][4]) - PELVIS2THORAX*S10[2][2]*(Si01[2][1]*T101[1][5] + Si01[2][2]*T101[2][5]) + PELVISOFFSET*(Si01[2][1]*T101[1][6] + Si01[2][2]*T101[2][6]);
T01[2][2]=S10[1][2]*(Si01[2][1]*T101[1][1] + Si01[2][2]*T101[2][1]) + S10[2][2]*(Si01[2][1]*T101[1][2] + Si01[2][2]*T101[2][2]) + PELVIS2THORAX*S10[1][1]*(Si01[2][1]*T101[1][4] + Si01[2][2]*T101[2][4]) + PELVIS2THORAX*S10[2][1]*(Si01[2][1]*T101[1][5] + Si01[2][2]*T101[2][5]);
T01[2][3]=-(Si01[2][1]*T101[1][3]) - Si01[2][2]*T101[2][3] + PELVISOFFSET*S10[1][1]*(Si01[2][1]*T101[1][4] + Si01[2][2]*T101[2][4]) + PELVISOFFSET*S10[2][1]*(Si01[2][1]*T101[1][5] + Si01[2][2]*T101[2][5]);
T01[2][4]=S10[1][1]*(Si01[2][1]*T101[1][4] + Si01[2][2]*T101[2][4]) + S10[2][1]*(Si01[2][1]*T101[1][5] + Si01[2][2]*T101[2][5]);
T01[2][5]=S10[1][2]*(Si01[2][1]*T101[1][4] + Si01[2][2]*T101[2][4]) + S10[2][2]*(Si01[2][1]*T101[1][5] + Si01[2][2]*T101[2][5]);
T01[2][6]=-(Si01[2][1]*T101[1][6]) - Si01[2][2]*T101[2][6];

T01[3][1]=-(S10[1][1]*T101[3][1]) - S10[2][1]*T101[3][2] + PELVIS2THORAX*S10[1][2]*T101[3][4] + PELVIS2THORAX*S10[2][2]*T101[3][5] - PELVISOFFSET*T101[3][6];
T01[3][2]=-(S10[1][2]*T101[3][1]) - S10[2][2]*T101[3][2] - PELVIS2THORAX*S10[1][1]*T101[3][4] - PELVIS2THORAX*S10[2][1]*T101[3][5];
T01[3][3]=T101[3][3] - PELVISOFFSET*S10[1][1]*T101[3][4] - PELVISOFFSET*S10[2][1]*T101[3][5];
T01[3][4]=-(S10[1][1]*T101[3][4]) - S10[2][1]*T101[3][5];
T01[3][5]=-(S10[1][2]*T101[3][4]) - S10[2][2]*T101[3][5];
T01[3][6]=T101[3][6];

T01[4][1]=S10[1][1]*(-(PELVIS2THORAX*Si01[2][1]*T101[1][1]) - PELVIS2THORAX*Si01[2][2]*T101[2][1] + PELVISOFFSET*T101[3][1] + Si01[1][1]*T101[4][1] + Si01[1][2]*T101[5][1]) + S10[2][1]*(-(PELVIS2THORAX*Si01[2][1]*T101[1][2]) - PELVIS2THORAX*Si01[2][2]*T101[2][2] + PELVISOFFSET*T101[3][2] + Si01[1][1]*T101[4][2] + Si01[1][2]*T101[5][2]) - PELVIS2THORAX*S10[1][2]*(-(PELVIS2THORAX*Si01[2][1]*T101[1][4]) - PELVIS2THORAX*Si01[2][2]*T101[2][4] + PELVISOFFSET*T101[3][4] + Si01[1][1]*T101[4][4] + Si01[1][2]*T101[5][4]) - PELVIS2THORAX*S10[2][2]*(-(PELVIS2THORAX*Si01[2][1]*T101[1][5]) - PELVIS2THORAX*Si01[2][2]*T101[2][5] + PELVISOFFSET*T101[3][5] + Si01[1][1]*T101[4][5] + Si01[1][2]*T101[5][5]) + PELVISOFFSET*(-(PELVIS2THORAX*Si01[2][1]*T101[1][6]) - PELVIS2THORAX*Si01[2][2]*T101[2][6] + PELVISOFFSET*T101[3][6] + Si01[1][1]*T101[4][6] + Si01[1][2]*T101[5][6]);
T01[4][2]=S10[1][2]*(-(PELVIS2THORAX*Si01[2][1]*T101[1][1]) - PELVIS2THORAX*Si01[2][2]*T101[2][1] + PELVISOFFSET*T101[3][1] + Si01[1][1]*T101[4][1] + Si01[1][2]*T101[5][1]) + S10[2][2]*(-(PELVIS2THORAX*Si01[2][1]*T101[1][2]) - PELVIS2THORAX*Si01[2][2]*T101[2][2] + PELVISOFFSET*T101[3][2] + Si01[1][1]*T101[4][2] + Si01[1][2]*T101[5][2]) + PELVIS2THORAX*S10[1][1]*(-(PELVIS2THORAX*Si01[2][1]*T101[1][4]) - PELVIS2THORAX*Si01[2][2]*T101[2][4] + PELVISOFFSET*T101[3][4] + Si01[1][1]*T101[4][4] + Si01[1][2]*T101[5][4]) + PELVIS2THORAX*S10[2][1]*(-(PELVIS2THORAX*Si01[2][1]*T101[1][5]) - PELVIS2THORAX*Si01[2][2]*T101[2][5] + PELVISOFFSET*T101[3][5] + Si01[1][1]*T101[4][5] + Si01[1][2]*T101[5][5]);
T01[4][3]=PELVIS2THORAX*Si01[2][1]*T101[1][3] + PELVIS2THORAX*Si01[2][2]*T101[2][3] - PELVISOFFSET*T101[3][3] - Si01[1][1]*T101[4][3] - Si01[1][2]*T101[5][3] + PELVISOFFSET*S10[1][1]*(-(PELVIS2THORAX*Si01[2][1]*T101[1][4]) - PELVIS2THORAX*Si01[2][2]*T101[2][4] + PELVISOFFSET*T101[3][4] + Si01[1][1]*T101[4][4] + Si01[1][2]*T101[5][4]) + PELVISOFFSET*S10[2][1]*(-(PELVIS2THORAX*Si01[2][1]*T101[1][5]) - PELVIS2THORAX*Si01[2][2]*T101[2][5] + PELVISOFFSET*T101[3][5] + Si01[1][1]*T101[4][5] + Si01[1][2]*T101[5][5]);
T01[4][4]=S10[1][1]*(-(PELVIS2THORAX*Si01[2][1]*T101[1][4]) - PELVIS2THORAX*Si01[2][2]*T101[2][4] + PELVISOFFSET*T101[3][4] + Si01[1][1]*T101[4][4] + Si01[1][2]*T101[5][4]) + S10[2][1]*(-(PELVIS2THORAX*Si01[2][1]*T101[1][5]) - PELVIS2THORAX*Si01[2][2]*T101[2][5] + PELVISOFFSET*T101[3][5] + Si01[1][1]*T101[4][5] + Si01[1][2]*T101[5][5]);
T01[4][5]=S10[1][2]*(-(PELVIS2THORAX*Si01[2][1]*T101[1][4]) - PELVIS2THORAX*Si01[2][2]*T101[2][4] + PELVISOFFSET*T101[3][4] + Si01[1][1]*T101[4][4] + Si01[1][2]*T101[5][4]) + S10[2][2]*(-(PELVIS2THORAX*Si01[2][1]*T101[1][5]) - PELVIS2THORAX*Si01[2][2]*T101[2][5] + PELVISOFFSET*T101[3][5] + Si01[1][1]*T101[4][5] + Si01[1][2]*T101[5][5]);
T01[4][6]=PELVIS2THORAX*Si01[2][1]*T101[1][6] + PELVIS2THORAX*Si01[2][2]*T101[2][6] - PELVISOFFSET*T101[3][6] - Si01[1][1]*T101[4][6] - Si01[1][2]*T101[5][6];

T01[5][1]=S10[1][1]*(PELVIS2THORAX*Si01[1][1]*T101[1][1] + PELVIS2THORAX*Si01[1][2]*T101[2][1] + Si01[2][1]*T101[4][1] + Si01[2][2]*T101[5][1]) + S10[2][1]*(PELVIS2THORAX*Si01[1][1]*T101[1][2] + PELVIS2THORAX*Si01[1][2]*T101[2][2] + Si01[2][1]*T101[4][2] + Si01[2][2]*T101[5][2]) - PELVIS2THORAX*S10[1][2]*(PELVIS2THORAX*Si01[1][1]*T101[1][4] + PELVIS2THORAX*Si01[1][2]*T101[2][4] + Si01[2][1]*T101[4][4] + Si01[2][2]*T101[5][4]) - PELVIS2THORAX*S10[2][2]*(PELVIS2THORAX*Si01[1][1]*T101[1][5] + PELVIS2THORAX*Si01[1][2]*T101[2][5] + Si01[2][1]*T101[4][5] + Si01[2][2]*T101[5][5]) + PELVISOFFSET*(PELVIS2THORAX*Si01[1][1]*T101[1][6] + PELVIS2THORAX*Si01[1][2]*T101[2][6] + Si01[2][1]*T101[4][6] + Si01[2][2]*T101[5][6]);
T01[5][2]=S10[1][2]*(PELVIS2THORAX*Si01[1][1]*T101[1][1] + PELVIS2THORAX*Si01[1][2]*T101[2][1] + Si01[2][1]*T101[4][1] + Si01[2][2]*T101[5][1]) + S10[2][2]*(PELVIS2THORAX*Si01[1][1]*T101[1][2] + PELVIS2THORAX*Si01[1][2]*T101[2][2] + Si01[2][1]*T101[4][2] + Si01[2][2]*T101[5][2]) + PELVIS2THORAX*S10[1][1]*(PELVIS2THORAX*Si01[1][1]*T101[1][4] + PELVIS2THORAX*Si01[1][2]*T101[2][4] + Si01[2][1]*T101[4][4] + Si01[2][2]*T101[5][4]) + PELVIS2THORAX*S10[2][1]*(PELVIS2THORAX*Si01[1][1]*T101[1][5] + PELVIS2THORAX*Si01[1][2]*T101[2][5] + Si01[2][1]*T101[4][5] + Si01[2][2]*T101[5][5]);
T01[5][3]=-(PELVIS2THORAX*Si01[1][1]*T101[1][3]) - PELVIS2THORAX*Si01[1][2]*T101[2][3] - Si01[2][1]*T101[4][3] - Si01[2][2]*T101[5][3] + PELVISOFFSET*S10[1][1]*(PELVIS2THORAX*Si01[1][1]*T101[1][4] + PELVIS2THORAX*Si01[1][2]*T101[2][4] + Si01[2][1]*T101[4][4] + Si01[2][2]*T101[5][4]) + PELVISOFFSET*S10[2][1]*(PELVIS2THORAX*Si01[1][1]*T101[1][5] + PELVIS2THORAX*Si01[1][2]*T101[2][5] + Si01[2][1]*T101[4][5] + Si01[2][2]*T101[5][5]);
T01[5][4]=S10[1][1]*(PELVIS2THORAX*Si01[1][1]*T101[1][4] + PELVIS2THORAX*Si01[1][2]*T101[2][4] + Si01[2][1]*T101[4][4] + Si01[2][2]*T101[5][4]) + S10[2][1]*(PELVIS2THORAX*Si01[1][1]*T101[1][5] + PELVIS2THORAX*Si01[1][2]*T101[2][5] + Si01[2][1]*T101[4][5] + Si01[2][2]*T101[5][5]);
T01[5][5]=S10[1][2]*(PELVIS2THORAX*Si01[1][1]*T101[1][4] + PELVIS2THORAX*Si01[1][2]*T101[2][4] + Si01[2][1]*T101[4][4] + Si01[2][2]*T101[5][4]) + S10[2][2]*(PELVIS2THORAX*Si01[1][1]*T101[1][5] + PELVIS2THORAX*Si01[1][2]*T101[2][5] + Si01[2][1]*T101[4][5] + Si01[2][2]*T101[5][5]);
T01[5][6]=-(PELVIS2THORAX*Si01[1][1]*T101[1][6]) - PELVIS2THORAX*Si01[1][2]*T101[2][6] - Si01[2][1]*T101[4][6] - Si01[2][2]*T101[5][6];

T01[6][1]=S10[1][1]*(PELVISOFFSET*Si01[1][1]*T101[1][1] + PELVISOFFSET*Si01[1][2]*T101[2][1] - T101[6][1]) + S10[2][1]*(PELVISOFFSET*Si01[1][1]*T101[1][2] + PELVISOFFSET*Si01[1][2]*T101[2][2] - T101[6][2]) - PELVIS2THORAX*S10[1][2]*(PELVISOFFSET*Si01[1][1]*T101[1][4] + PELVISOFFSET*Si01[1][2]*T101[2][4] - T101[6][4]) - PELVIS2THORAX*S10[2][2]*(PELVISOFFSET*Si01[1][1]*T101[1][5] + PELVISOFFSET*Si01[1][2]*T101[2][5] - T101[6][5]) + PELVISOFFSET*(PELVISOFFSET*Si01[1][1]*T101[1][6] + PELVISOFFSET*Si01[1][2]*T101[2][6] - T101[6][6]);
T01[6][2]=S10[1][2]*(PELVISOFFSET*Si01[1][1]*T101[1][1] + PELVISOFFSET*Si01[1][2]*T101[2][1] - T101[6][1]) + S10[2][2]*(PELVISOFFSET*Si01[1][1]*T101[1][2] + PELVISOFFSET*Si01[1][2]*T101[2][2] - T101[6][2]) + PELVIS2THORAX*S10[1][1]*(PELVISOFFSET*Si01[1][1]*T101[1][4] + PELVISOFFSET*Si01[1][2]*T101[2][4] - T101[6][4]) + PELVIS2THORAX*S10[2][1]*(PELVISOFFSET*Si01[1][1]*T101[1][5] + PELVISOFFSET*Si01[1][2]*T101[2][5] - T101[6][5]);
T01[6][3]=-(PELVISOFFSET*Si01[1][1]*T101[1][3]) - PELVISOFFSET*Si01[1][2]*T101[2][3] + T101[6][3] + PELVISOFFSET*S10[1][1]*(PELVISOFFSET*Si01[1][1]*T101[1][4] + PELVISOFFSET*Si01[1][2]*T101[2][4] - T101[6][4]) + PELVISOFFSET*S10[2][1]*(PELVISOFFSET*Si01[1][1]*T101[1][5] + PELVISOFFSET*Si01[1][2]*T101[2][5] - T101[6][5]);
T01[6][4]=S10[1][1]*(PELVISOFFSET*Si01[1][1]*T101[1][4] + PELVISOFFSET*Si01[1][2]*T101[2][4] - T101[6][4]) + S10[2][1]*(PELVISOFFSET*Si01[1][1]*T101[1][5] + PELVISOFFSET*Si01[1][2]*T101[2][5] - T101[6][5]);
T01[6][5]=S10[1][2]*(PELVISOFFSET*Si01[1][1]*T101[1][4] + PELVISOFFSET*Si01[1][2]*T101[2][4] - T101[6][4]) + S10[2][2]*(PELVISOFFSET*Si01[1][1]*T101[1][5] + PELVISOFFSET*Si01[1][2]*T101[2][5] - T101[6][5]);
T01[6][6]=-(PELVISOFFSET*Si01[1][1]*T101[1][6]) - PELVISOFFSET*Si01[1][2]*T101[2][6] + T101[6][6];



}


void
hermes_InvDynArtfunc104(void)
       {
JA0[1][1]=T01[1][1] + T070[1][1] + T084[1][1];
JA0[1][2]=links[0].mcm[3] + T01[1][2] + T070[1][2] + T084[1][2];
JA0[1][3]=-links[0].mcm[2] + T01[1][3] + T070[1][3] + T084[1][3];
JA0[1][4]=links[0].m + T01[1][4] + T070[1][4] + T084[1][4];
JA0[1][5]=T01[1][5] + T070[1][5] + T084[1][5];
JA0[1][6]=T01[1][6] + T070[1][6] + T084[1][6];

JA0[2][1]=-links[0].mcm[3] + T01[2][1] + T070[2][1] + T084[2][1];
JA0[2][2]=T01[2][2] + T070[2][2] + T084[2][2];
JA0[2][3]=links[0].mcm[1] + T01[2][3] + T070[2][3] + T084[2][3];
JA0[2][4]=T01[2][4] + T070[2][4] + T084[2][4];
JA0[2][5]=links[0].m + T01[2][5] + T070[2][5] + T084[2][5];
JA0[2][6]=T01[2][6] + T070[2][6] + T084[2][6];

JA0[3][1]=links[0].mcm[2] + T01[3][1] + T070[3][1] + T084[3][1];
JA0[3][2]=-links[0].mcm[1] + T01[3][2] + T070[3][2] + T084[3][2];
JA0[3][3]=T01[3][3] + T070[3][3] + T084[3][3];
JA0[3][4]=T01[3][4] + T070[3][4] + T084[3][4];
JA0[3][5]=T01[3][5] + T070[3][5] + T084[3][5];
JA0[3][6]=links[0].m + T01[3][6] + T070[3][6] + T084[3][6];

JA0[4][1]=links[0].inertia[1][1] + T01[4][1] + T070[4][1] + T084[4][1];
JA0[4][2]=links[0].inertia[1][2] + T01[4][2] + T070[4][2] + T084[4][2];
JA0[4][3]=links[0].inertia[1][3] + T01[4][3] + T070[4][3] + T084[4][3];
JA0[4][4]=T01[4][4] + T070[4][4] + T084[4][4];
JA0[4][5]=-links[0].mcm[3] + T01[4][5] + T070[4][5] + T084[4][5];
JA0[4][6]=links[0].mcm[2] + T01[4][6] + T070[4][6] + T084[4][6];

JA0[5][1]=links[0].inertia[1][2] + T01[5][1] + T070[5][1] + T084[5][1];
JA0[5][2]=links[0].inertia[2][2] + T01[5][2] + T070[5][2] + T084[5][2];
JA0[5][3]=links[0].inertia[2][3] + T01[5][3] + T070[5][3] + T084[5][3];
JA0[5][4]=links[0].mcm[3] + T01[5][4] + T070[5][4] + T084[5][4];
JA0[5][5]=T01[5][5] + T070[5][5] + T084[5][5];
JA0[5][6]=-links[0].mcm[1] + T01[5][6] + T070[5][6] + T084[5][6];

JA0[6][1]=links[0].inertia[1][3] + T01[6][1] + T070[6][1] + T084[6][1];
JA0[6][2]=links[0].inertia[2][3] + T01[6][2] + T070[6][2] + T084[6][2];
JA0[6][3]=links[0].inertia[3][3] + T01[6][3] + T070[6][3] + T084[6][3];
JA0[6][4]=-links[0].mcm[2] + T01[6][4] + T070[6][4] + T084[6][4];
JA0[6][5]=links[0].mcm[1] + T01[6][5] + T070[6][5] + T084[6][5];
JA0[6][6]=T01[6][6] + T070[6][6] + T084[6][6];



}


void
hermes_InvDynArtfunc105(void)
       {
/* bias forces */
p97[1]=pv97[1];
p97[2]=pv97[2];
p97[3]=pv97[3];
p97[4]=pv97[4];
p97[5]=pv97[5];
p97[6]=pv97[6];

pmm97[1]=p97[1];
pmm97[2]=p97[2];
pmm97[3]=p97[3];
pmm97[4]=p97[4];
pmm97[5]=p97[5];
pmm97[6]=p97[6];

pm97[1]=pmm97[1]*Si9097[1][1] + pmm97[2]*Si9097[1][2] + pmm97[3]*Si9097[1][3];
pm97[2]=pmm97[1]*Si9097[2][1] + pmm97[2]*Si9097[2][2] + pmm97[3]*Si9097[2][3];
pm97[3]=pmm97[1]*Si9097[3][1] + pmm97[2]*Si9097[3][2] + pmm97[3]*Si9097[3][3];
pm97[4]=pmm97[4]*Si9097[1][1] + pmm97[5]*Si9097[1][2] + pmm97[6]*Si9097[1][3] + pmm97[1]*(-(eff[4].x[3]*Si9097[2][1]) + eff[4].x[2]*Si9097[3][1]) + pmm97[2]*(-(eff[4].x[3]*Si9097[2][2]) + eff[4].x[2]*Si9097[3][2]) + pmm97[3]*(-(eff[4].x[3]*Si9097[2][3]) + eff[4].x[2]*Si9097[3][3]);
pm97[5]=pmm97[4]*Si9097[2][1] + pmm97[5]*Si9097[2][2] + pmm97[6]*Si9097[2][3] + pmm97[1]*(eff[4].x[3]*Si9097[1][1] - eff[4].x[1]*Si9097[3][1]) + pmm97[2]*(eff[4].x[3]*Si9097[1][2] - eff[4].x[1]*Si9097[3][2]) + pmm97[3]*(eff[4].x[3]*Si9097[1][3] - eff[4].x[1]*Si9097[3][3]);
pm97[6]=pmm97[1]*(-(eff[4].x[2]*Si9097[1][1]) + eff[4].x[1]*Si9097[2][1]) + pmm97[2]*(-(eff[4].x[2]*Si9097[1][2]) + eff[4].x[1]*Si9097[2][2]) + pmm97[3]*(-(eff[4].x[2]*Si9097[1][3]) + eff[4].x[1]*Si9097[2][3]) + pmm97[4]*Si9097[3][1] + pmm97[5]*Si9097[3][2] + pmm97[6]*Si9097[3][3];

p90[1]=0. + pm97[1] + pv90[1];
p90[2]=0. + pm97[2] + pv90[2];
p90[3]=0. + pm97[3] + pv90[3];
p90[4]=0. + pm97[4] + pv90[4];
p90[5]=0. + pm97[5] + pv90[5];
p90[6]=0. + pm97[6] + pv90[6];

pmm90[1]=state[21].thdd*h90[1] + p90[1] + c90[1]*JA90[1][1] + c90[2]*JA90[1][2] + c90[4]*JA90[1][4] + c90[5]*JA90[1][5];
pmm90[2]=state[21].thdd*h90[2] + p90[2] + c90[1]*JA90[2][1] + c90[2]*JA90[2][2] + c90[4]*JA90[2][4] + c90[5]*JA90[2][5];
pmm90[3]=state[21].thdd*h90[3] + p90[3] + c90[1]*JA90[3][1] + c90[2]*JA90[3][2] + c90[4]*JA90[3][4] + c90[5]*JA90[3][5];
pmm90[4]=state[21].thdd*h90[4] + p90[4] + c90[1]*JA90[4][1] + c90[2]*JA90[4][2] + c90[4]*JA90[4][4] + c90[5]*JA90[4][5];
pmm90[5]=state[21].thdd*h90[5] + p90[5] + c90[1]*JA90[5][1] + c90[2]*JA90[5][2] + c90[4]*JA90[5][4] + c90[5]*JA90[5][5];
pmm90[6]=state[21].thdd*h90[6] + p90[6] + c90[1]*JA90[6][1] + c90[2]*JA90[6][2] + c90[4]*JA90[6][4] + c90[5]*JA90[6][5];

pm90[1]=pmm90[1]*Si8990[1][1] + pmm90[2]*Si8990[1][2];
pm90[2]=-pmm90[3];
pm90[3]=pmm90[1]*Si8990[3][1] + pmm90[2]*Si8990[3][2];
pm90[4]=pmm90[4]*Si8990[1][1] + pmm90[5]*Si8990[1][2];
pm90[5]=-pmm90[6];
pm90[6]=pmm90[4]*Si8990[3][1] + pmm90[5]*Si8990[3][2];

p89[1]=pm90[1] + pv89[1];
p89[2]=pm90[2] + pv89[2];
p89[3]=pm90[3] + pv89[3];
p89[4]=pm90[4] + pv89[4];
p89[5]=pm90[5] + pv89[5];
p89[6]=pm90[6] + pv89[6];

pmm89[1]=state[20].thdd*h89[1] + p89[1] + c89[1]*JA89[1][1] + c89[2]*JA89[1][2] + c89[4]*JA89[1][4] + c89[5]*JA89[1][5];
pmm89[2]=state[20].thdd*h89[2] + p89[2] + c89[1]*JA89[2][1] + c89[2]*JA89[2][2] + c89[4]*JA89[2][4] + c89[5]*JA89[2][5];
pmm89[3]=state[20].thdd*h89[3] + p89[3] + c89[1]*JA89[3][1] + c89[2]*JA89[3][2] + c89[4]*JA89[3][4] + c89[5]*JA89[3][5];
pmm89[4]=state[20].thdd*h89[4] + p89[4] + c89[1]*JA89[4][1] + c89[2]*JA89[4][2] + c89[4]*JA89[4][4] + c89[5]*JA89[4][5];
pmm89[5]=state[20].thdd*h89[5] + p89[5] + c89[1]*JA89[5][1] + c89[2]*JA89[5][2] + c89[4]*JA89[5][4] + c89[5]*JA89[5][5];
pmm89[6]=state[20].thdd*h89[6] + p89[6] + c89[1]*JA89[6][1] + c89[2]*JA89[6][2] + c89[4]*JA89[6][4] + c89[5]*JA89[6][5];

pm89[1]=pmm89[1]*Si8889[1][1] + pmm89[2]*Si8889[1][2];
pm89[2]=pmm89[3];
pm89[3]=pmm89[1]*Si8889[3][1] + pmm89[2]*Si8889[3][2];
pm89[4]=LOWERLEG*pmm89[3] + pmm89[4]*Si8889[1][1] + pmm89[5]*Si8889[1][2];
pm89[5]=pmm89[6] - LOWERLEG*pmm89[1]*Si8889[1][1] - LOWERLEG*pmm89[2]*Si8889[1][2];
pm89[6]=pmm89[4]*Si8889[3][1] + pmm89[5]*Si8889[3][2];

p88[1]=pm89[1] + pv88[1];
p88[2]=pm89[2] + pv88[2];
p88[3]=pm89[3] + pv88[3];
p88[4]=pm89[4] + pv88[4];
p88[5]=pm89[5] + pv88[5];
p88[6]=pm89[6] + pv88[6];

pmm88[1]=state[19].thdd*h88[1] + p88[1] + c88[1]*JA88[1][1] + c88[2]*JA88[1][2] + c88[4]*JA88[1][4] + c88[5]*JA88[1][5];
pmm88[2]=state[19].thdd*h88[2] + p88[2] + c88[1]*JA88[2][1] + c88[2]*JA88[2][2] + c88[4]*JA88[2][4] + c88[5]*JA88[2][5];
pmm88[3]=state[19].thdd*h88[3] + p88[3] + c88[1]*JA88[3][1] + c88[2]*JA88[3][2] + c88[4]*JA88[3][4] + c88[5]*JA88[3][5];
pmm88[4]=state[19].thdd*h88[4] + p88[4] + c88[1]*JA88[4][1] + c88[2]*JA88[4][2] + c88[4]*JA88[4][4] + c88[5]*JA88[4][5];
pmm88[5]=state[19].thdd*h88[5] + p88[5] + c88[1]*JA88[5][1] + c88[2]*JA88[5][2] + c88[4]*JA88[5][4] + c88[5]*JA88[5][5];
pmm88[6]=state[19].thdd*h88[6] + p88[6] + c88[1]*JA88[6][1] + c88[2]*JA88[6][2] + c88[4]*JA88[6][4] + c88[5]*JA88[6][5];

pm88[1]=pmm88[1]*Si8788[1][1] + pmm88[2]*Si8788[1][2];
pm88[2]=pmm88[3];
pm88[3]=pmm88[1]*Si8788[3][1] + pmm88[2]*Si8788[3][2];
pm88[4]=pmm88[4]*Si8788[1][1] + pmm88[5]*Si8788[1][2];
pm88[5]=pmm88[6];
pm88[6]=pmm88[4]*Si8788[3][1] + pmm88[5]*Si8788[3][2];

p87[1]=pm88[1] + pv87[1];
p87[2]=pm88[2] + pv87[2];
p87[3]=pm88[3] + pv87[3];
p87[4]=pm88[4] + pv87[4];
p87[5]=pm88[5] + pv87[5];
p87[6]=pm88[6] + pv87[6];

pmm87[1]=state[18].thdd*h87[1] + p87[1] + c87[1]*JA87[1][1] + c87[2]*JA87[1][2] + c87[4]*JA87[1][4] + c87[5]*JA87[1][5];
pmm87[2]=state[18].thdd*h87[2] + p87[2] + c87[1]*JA87[2][1] + c87[2]*JA87[2][2] + c87[4]*JA87[2][4] + c87[5]*JA87[2][5];
pmm87[3]=state[18].thdd*h87[3] + p87[3] + c87[1]*JA87[3][1] + c87[2]*JA87[3][2] + c87[4]*JA87[3][4] + c87[5]*JA87[3][5];
pmm87[4]=state[18].thdd*h87[4] + p87[4] + c87[1]*JA87[4][1] + c87[2]*JA87[4][2] + c87[4]*JA87[4][4] + c87[5]*JA87[4][5];
pmm87[5]=state[18].thdd*h87[5] + p87[5] + c87[1]*JA87[5][1] + c87[2]*JA87[5][2] + c87[4]*JA87[5][4] + c87[5]*JA87[5][5];
pmm87[6]=state[18].thdd*h87[6] + p87[6] + c87[1]*JA87[6][1] + c87[2]*JA87[6][2] + c87[4]*JA87[6][4] + c87[5]*JA87[6][5];

pm87[1]=pmm87[1]*Si8687[1][1] + pmm87[2]*Si8687[1][2];
pm87[2]=-pmm87[3];
pm87[3]=pmm87[1]*Si8687[3][1] + pmm87[2]*Si8687[3][2];
pm87[4]=-(UPPERLEGMOD*pmm87[3]) + pmm87[4]*Si8687[1][1] + pmm87[5]*Si8687[1][2];
pm87[5]=-pmm87[6] + pmm87[1]*(-(UPPERLEGMOD*Si8687[1][1]) - YKNEE*Si8687[3][1]) + pmm87[2]*(-(UPPERLEGMOD*Si8687[1][2]) - YKNEE*Si8687[3][2]);
pm87[6]=-(YKNEE*pmm87[3]) + pmm87[4]*Si8687[3][1] + pmm87[5]*Si8687[3][2];

p86[1]=pm87[1] + pv86[1];
p86[2]=pm87[2] + pv86[2];
p86[3]=pm87[3] + pv86[3];
p86[4]=pm87[4] + pv86[4];
p86[5]=pm87[5] + pv86[5];
p86[6]=pm87[6] + pv86[6];

pmm86[1]=state[17].thdd*h86[1] + p86[1] + c86[1]*JA86[1][1] + c86[2]*JA86[1][2] + c86[4]*JA86[1][4] + c86[5]*JA86[1][5];
pmm86[2]=state[17].thdd*h86[2] + p86[2] + c86[1]*JA86[2][1] + c86[2]*JA86[2][2] + c86[4]*JA86[2][4] + c86[5]*JA86[2][5];
pmm86[3]=state[17].thdd*h86[3] + p86[3] + c86[1]*JA86[3][1] + c86[2]*JA86[3][2] + c86[4]*JA86[3][4] + c86[5]*JA86[3][5];
pmm86[4]=state[17].thdd*h86[4] + p86[4] + c86[1]*JA86[4][1] + c86[2]*JA86[4][2] + c86[4]*JA86[4][4] + c86[5]*JA86[4][5];
pmm86[5]=state[17].thdd*h86[5] + p86[5] + c86[1]*JA86[5][1] + c86[2]*JA86[5][2] + c86[4]*JA86[5][4] + c86[5]*JA86[5][5];
pmm86[6]=state[17].thdd*h86[6] + p86[6] + c86[1]*JA86[6][1] + c86[2]*JA86[6][2] + c86[4]*JA86[6][4] + c86[5]*JA86[6][5];

pm86[1]=pmm86[1]*Si8586[1][1] + pmm86[2]*Si8586[1][2] + pmm86[3]*Si8586[1][3];
pm86[2]=pmm86[1]*Si8586[2][1] + pmm86[2]*Si8586[2][2] + pmm86[3]*Si8586[2][3];
pm86[3]=pmm86[1]*Si8586[3][1] + pmm86[2]*Si8586[3][2];
pm86[4]=pmm86[4]*Si8586[1][1] + pmm86[5]*Si8586[1][2] + pmm86[6]*Si8586[1][3];
pm86[5]=pmm86[4]*Si8586[2][1] + pmm86[5]*Si8586[2][2] + pmm86[6]*Si8586[2][3] - YHIP*pmm86[1]*Si8586[3][1] - YHIP*pmm86[2]*Si8586[3][2];
pm86[6]=YHIP*pmm86[1]*Si8586[2][1] + YHIP*pmm86[2]*Si8586[2][2] + YHIP*pmm86[3]*Si8586[2][3] + pmm86[4]*Si8586[3][1] + pmm86[5]*Si8586[3][2];

p85[1]=pm86[1] + pv85[1];
p85[2]=pm86[2] + pv85[2];
p85[3]=pm86[3] + pv85[3];
p85[4]=pm86[4] + pv85[4];
p85[5]=pm86[5] + pv85[5];
p85[6]=pm86[6] + pv85[6];

pmm85[1]=state[15].thdd*h85[1] + p85[1] + c85[1]*JA85[1][1] + c85[2]*JA85[1][2] + c85[4]*JA85[1][4] + c85[5]*JA85[1][5];
pmm85[2]=state[15].thdd*h85[2] + p85[2] + c85[1]*JA85[2][1] + c85[2]*JA85[2][2] + c85[4]*JA85[2][4] + c85[5]*JA85[2][5];
pmm85[3]=state[15].thdd*h85[3] + p85[3] + c85[1]*JA85[3][1] + c85[2]*JA85[3][2] + c85[4]*JA85[3][4] + c85[5]*JA85[3][5];
pmm85[4]=state[15].thdd*h85[4] + p85[4] + c85[1]*JA85[4][1] + c85[2]*JA85[4][2] + c85[4]*JA85[4][4] + c85[5]*JA85[4][5];
pmm85[5]=state[15].thdd*h85[5] + p85[5] + c85[1]*JA85[5][1] + c85[2]*JA85[5][2] + c85[4]*JA85[5][4] + c85[5]*JA85[5][5];
pmm85[6]=state[15].thdd*h85[6] + p85[6] + c85[1]*JA85[6][1] + c85[2]*JA85[6][2] + c85[4]*JA85[6][4] + c85[5]*JA85[6][5];

pm85[1]=pmm85[1]*Si8485[1][1] + pmm85[2]*Si8485[1][2];
pm85[2]=pmm85[3];
pm85[3]=pmm85[1]*Si8485[3][1] + pmm85[2]*Si8485[3][2];
pm85[4]=pmm85[4]*Si8485[1][1] + pmm85[5]*Si8485[1][2];
pm85[5]=pmm85[6];
pm85[6]=pmm85[4]*Si8485[3][1] + pmm85[5]*Si8485[3][2];

p84[1]=pm85[1] + pv84[1];
p84[2]=pm85[2] + pv84[2];
p84[3]=pm85[3] + pv84[3];
p84[4]=pm85[4] + pv84[4];
p84[5]=pm85[5] + pv84[5];
p84[6]=pm85[6] + pv84[6];

pmm84[1]=state[16].thdd*h84[1] + p84[1] + c84[1]*JA84[1][1] + c84[2]*JA84[1][2] + c84[4]*JA84[1][4] + c84[5]*JA84[1][5];
pmm84[2]=state[16].thdd*h84[2] + p84[2] + c84[1]*JA84[2][1] + c84[2]*JA84[2][2] + c84[4]*JA84[2][4] + c84[5]*JA84[2][5];
pmm84[3]=state[16].thdd*h84[3] + p84[3] + c84[1]*JA84[3][1] + c84[2]*JA84[3][2] + c84[4]*JA84[3][4] + c84[5]*JA84[3][5];
pmm84[4]=state[16].thdd*h84[4] + p84[4] + c84[1]*JA84[4][1] + c84[2]*JA84[4][2] + c84[4]*JA84[4][4] + c84[5]*JA84[4][5];
pmm84[5]=state[16].thdd*h84[5] + p84[5] + c84[1]*JA84[5][1] + c84[2]*JA84[5][2] + c84[4]*JA84[5][4] + c84[5]*JA84[5][5];
pmm84[6]=state[16].thdd*h84[6] + p84[6] + c84[1]*JA84[6][1] + c84[2]*JA84[6][2] + c84[4]*JA84[6][4] + c84[5]*JA84[6][5];

pm84[1]=pmm84[1]*Si084[1][1] + pmm84[2]*Si084[1][2];
pm84[2]=-pmm84[3];
pm84[3]=pmm84[1]*Si084[3][1] + pmm84[2]*Si084[3][2];
pm84[4]=pmm84[4]*Si084[1][1] + pmm84[5]*Si084[1][2];
pm84[5]=-pmm84[6] + XHIP*pmm84[1]*Si084[3][1] + XHIP*pmm84[2]*Si084[3][2];
pm84[6]=XHIP*pmm84[3] + pmm84[4]*Si084[3][1] + pmm84[5]*Si084[3][2];

p83[1]=pv83[1];
p83[2]=pv83[2];
p83[3]=pv83[3];
p83[4]=pv83[4];
p83[5]=pv83[5];
p83[6]=pv83[6];

pmm83[1]=p83[1];
pmm83[2]=p83[2];
pmm83[3]=p83[3];
pmm83[4]=p83[4];
pmm83[5]=p83[5];
pmm83[6]=p83[6];

pm83[1]=pmm83[1]*Si7683[1][1] + pmm83[2]*Si7683[1][2] + pmm83[3]*Si7683[1][3];
pm83[2]=pmm83[1]*Si7683[2][1] + pmm83[2]*Si7683[2][2] + pmm83[3]*Si7683[2][3];
pm83[3]=pmm83[1]*Si7683[3][1] + pmm83[2]*Si7683[3][2] + pmm83[3]*Si7683[3][3];
pm83[4]=pmm83[4]*Si7683[1][1] + pmm83[5]*Si7683[1][2] + pmm83[6]*Si7683[1][3] + pmm83[1]*(-(eff[3].x[3]*Si7683[2][1]) + eff[3].x[2]*Si7683[3][1]) + pmm83[2]*(-(eff[3].x[3]*Si7683[2][2]) + eff[3].x[2]*Si7683[3][2]) + pmm83[3]*(-(eff[3].x[3]*Si7683[2][3]) + eff[3].x[2]*Si7683[3][3]);
pm83[5]=pmm83[4]*Si7683[2][1] + pmm83[5]*Si7683[2][2] + pmm83[6]*Si7683[2][3] + pmm83[1]*(eff[3].x[3]*Si7683[1][1] - eff[3].x[1]*Si7683[3][1]) + pmm83[2]*(eff[3].x[3]*Si7683[1][2] - eff[3].x[1]*Si7683[3][2]) + pmm83[3]*(eff[3].x[3]*Si7683[1][3] - eff[3].x[1]*Si7683[3][3]);
pm83[6]=pmm83[1]*(-(eff[3].x[2]*Si7683[1][1]) + eff[3].x[1]*Si7683[2][1]) + pmm83[2]*(-(eff[3].x[2]*Si7683[1][2]) + eff[3].x[1]*Si7683[2][2]) + pmm83[3]*(-(eff[3].x[2]*Si7683[1][3]) + eff[3].x[1]*Si7683[2][3]) + pmm83[4]*Si7683[3][1] + pmm83[5]*Si7683[3][2] + pmm83[6]*Si7683[3][3];

p76[1]=0. + pm83[1] + pv76[1];
p76[2]=0. + pm83[2] + pv76[2];
p76[3]=0. + pm83[3] + pv76[3];
p76[4]=0. + pm83[4] + pv76[4];
p76[5]=0. + pm83[5] + pv76[5];
p76[6]=0. + pm83[6] + pv76[6];

pmm76[1]=state[28].thdd*h76[1] + p76[1] + c76[1]*JA76[1][1] + c76[2]*JA76[1][2] + c76[4]*JA76[1][4] + c76[5]*JA76[1][5];
pmm76[2]=state[28].thdd*h76[2] + p76[2] + c76[1]*JA76[2][1] + c76[2]*JA76[2][2] + c76[4]*JA76[2][4] + c76[5]*JA76[2][5];
pmm76[3]=state[28].thdd*h76[3] + p76[3] + c76[1]*JA76[3][1] + c76[2]*JA76[3][2] + c76[4]*JA76[3][4] + c76[5]*JA76[3][5];
pmm76[4]=state[28].thdd*h76[4] + p76[4] + c76[1]*JA76[4][1] + c76[2]*JA76[4][2] + c76[4]*JA76[4][4] + c76[5]*JA76[4][5];
pmm76[5]=state[28].thdd*h76[5] + p76[5] + c76[1]*JA76[5][1] + c76[2]*JA76[5][2] + c76[4]*JA76[5][4] + c76[5]*JA76[5][5];
pmm76[6]=state[28].thdd*h76[6] + p76[6] + c76[1]*JA76[6][1] + c76[2]*JA76[6][2] + c76[4]*JA76[6][4] + c76[5]*JA76[6][5];

pm76[1]=pmm76[1]*Si7576[1][1] + pmm76[2]*Si7576[1][2];
pm76[2]=pmm76[3];
pm76[3]=pmm76[1]*Si7576[3][1] + pmm76[2]*Si7576[3][2];
pm76[4]=pmm76[4]*Si7576[1][1] + pmm76[5]*Si7576[1][2];
pm76[5]=pmm76[6];
pm76[6]=pmm76[4]*Si7576[3][1] + pmm76[5]*Si7576[3][2];

p75[1]=pm76[1] + pv75[1];
p75[2]=pm76[2] + pv75[2];
p75[3]=pm76[3] + pv75[3];
p75[4]=pm76[4] + pv75[4];
p75[5]=pm76[5] + pv75[5];
p75[6]=pm76[6] + pv75[6];

pmm75[1]=state[27].thdd*h75[1] + p75[1] + c75[1]*JA75[1][1] + c75[2]*JA75[1][2] + c75[4]*JA75[1][4] + c75[5]*JA75[1][5];
pmm75[2]=state[27].thdd*h75[2] + p75[2] + c75[1]*JA75[2][1] + c75[2]*JA75[2][2] + c75[4]*JA75[2][4] + c75[5]*JA75[2][5];
pmm75[3]=state[27].thdd*h75[3] + p75[3] + c75[1]*JA75[3][1] + c75[2]*JA75[3][2] + c75[4]*JA75[3][4] + c75[5]*JA75[3][5];
pmm75[4]=state[27].thdd*h75[4] + p75[4] + c75[1]*JA75[4][1] + c75[2]*JA75[4][2] + c75[4]*JA75[4][4] + c75[5]*JA75[4][5];
pmm75[5]=state[27].thdd*h75[5] + p75[5] + c75[1]*JA75[5][1] + c75[2]*JA75[5][2] + c75[4]*JA75[5][4] + c75[5]*JA75[5][5];
pmm75[6]=state[27].thdd*h75[6] + p75[6] + c75[1]*JA75[6][1] + c75[2]*JA75[6][2] + c75[4]*JA75[6][4] + c75[5]*JA75[6][5];

pm75[1]=pmm75[1]*Si7475[1][1] + pmm75[2]*Si7475[1][2];
pm75[2]=-pmm75[3];
pm75[3]=pmm75[1]*Si7475[3][1] + pmm75[2]*Si7475[3][2];
pm75[4]=LOWERLEG*pmm75[3] + pmm75[4]*Si7475[1][1] + pmm75[5]*Si7475[1][2];
pm75[5]=-pmm75[6] + LOWERLEG*pmm75[1]*Si7475[1][1] + LOWERLEG*pmm75[2]*Si7475[1][2];
pm75[6]=pmm75[4]*Si7475[3][1] + pmm75[5]*Si7475[3][2];

p74[1]=pm75[1] + pv74[1];
p74[2]=pm75[2] + pv74[2];
p74[3]=pm75[3] + pv74[3];
p74[4]=pm75[4] + pv74[4];
p74[5]=pm75[5] + pv74[5];
p74[6]=pm75[6] + pv74[6];

pmm74[1]=state[26].thdd*h74[1] + p74[1] + c74[1]*JA74[1][1] + c74[2]*JA74[1][2] + c74[4]*JA74[1][4] + c74[5]*JA74[1][5];
pmm74[2]=state[26].thdd*h74[2] + p74[2] + c74[1]*JA74[2][1] + c74[2]*JA74[2][2] + c74[4]*JA74[2][4] + c74[5]*JA74[2][5];
pmm74[3]=state[26].thdd*h74[3] + p74[3] + c74[1]*JA74[3][1] + c74[2]*JA74[3][2] + c74[4]*JA74[3][4] + c74[5]*JA74[3][5];
pmm74[4]=state[26].thdd*h74[4] + p74[4] + c74[1]*JA74[4][1] + c74[2]*JA74[4][2] + c74[4]*JA74[4][4] + c74[5]*JA74[4][5];
pmm74[5]=state[26].thdd*h74[5] + p74[5] + c74[1]*JA74[5][1] + c74[2]*JA74[5][2] + c74[4]*JA74[5][4] + c74[5]*JA74[5][5];
pmm74[6]=state[26].thdd*h74[6] + p74[6] + c74[1]*JA74[6][1] + c74[2]*JA74[6][2] + c74[4]*JA74[6][4] + c74[5]*JA74[6][5];

pm74[1]=pmm74[1]*Si7374[1][1] + pmm74[2]*Si7374[1][2];
pm74[2]=-pmm74[3];
pm74[3]=pmm74[1]*Si7374[3][1] + pmm74[2]*Si7374[3][2];
pm74[4]=pmm74[4]*Si7374[1][1] + pmm74[5]*Si7374[1][2];
pm74[5]=-pmm74[6];
pm74[6]=pmm74[4]*Si7374[3][1] + pmm74[5]*Si7374[3][2];

p73[1]=pm74[1] + pv73[1];
p73[2]=pm74[2] + pv73[2];
p73[3]=pm74[3] + pv73[3];
p73[4]=pm74[4] + pv73[4];
p73[5]=pm74[5] + pv73[5];
p73[6]=pm74[6] + pv73[6];

pmm73[1]=state[25].thdd*h73[1] + p73[1] + c73[1]*JA73[1][1] + c73[2]*JA73[1][2] + c73[4]*JA73[1][4] + c73[5]*JA73[1][5];
pmm73[2]=state[25].thdd*h73[2] + p73[2] + c73[1]*JA73[2][1] + c73[2]*JA73[2][2] + c73[4]*JA73[2][4] + c73[5]*JA73[2][5];
pmm73[3]=state[25].thdd*h73[3] + p73[3] + c73[1]*JA73[3][1] + c73[2]*JA73[3][2] + c73[4]*JA73[3][4] + c73[5]*JA73[3][5];
pmm73[4]=state[25].thdd*h73[4] + p73[4] + c73[1]*JA73[4][1] + c73[2]*JA73[4][2] + c73[4]*JA73[4][4] + c73[5]*JA73[4][5];
pmm73[5]=state[25].thdd*h73[5] + p73[5] + c73[1]*JA73[5][1] + c73[2]*JA73[5][2] + c73[4]*JA73[5][4] + c73[5]*JA73[5][5];
pmm73[6]=state[25].thdd*h73[6] + p73[6] + c73[1]*JA73[6][1] + c73[2]*JA73[6][2] + c73[4]*JA73[6][4] + c73[5]*JA73[6][5];

pm73[1]=pmm73[1]*Si7273[1][1] + pmm73[2]*Si7273[1][2];
pm73[2]=pmm73[3];
pm73[3]=pmm73[1]*Si7273[3][1] + pmm73[2]*Si7273[3][2];
pm73[4]=-(UPPERLEGMOD*pmm73[3]) + pmm73[4]*Si7273[1][1] + pmm73[5]*Si7273[1][2];
pm73[5]=pmm73[6] + pmm73[1]*(UPPERLEGMOD*Si7273[1][1] - YKNEE*Si7273[3][1]) + pmm73[2]*(UPPERLEGMOD*Si7273[1][2] - YKNEE*Si7273[3][2]);
pm73[6]=YKNEE*pmm73[3] + pmm73[4]*Si7273[3][1] + pmm73[5]*Si7273[3][2];

p72[1]=pm73[1] + pv72[1];
p72[2]=pm73[2] + pv72[2];
p72[3]=pm73[3] + pv72[3];
p72[4]=pm73[4] + pv72[4];
p72[5]=pm73[5] + pv72[5];
p72[6]=pm73[6] + pv72[6];

pmm72[1]=state[24].thdd*h72[1] + p72[1] + c72[1]*JA72[1][1] + c72[2]*JA72[1][2] + c72[4]*JA72[1][4] + c72[5]*JA72[1][5];
pmm72[2]=state[24].thdd*h72[2] + p72[2] + c72[1]*JA72[2][1] + c72[2]*JA72[2][2] + c72[4]*JA72[2][4] + c72[5]*JA72[2][5];
pmm72[3]=state[24].thdd*h72[3] + p72[3] + c72[1]*JA72[3][1] + c72[2]*JA72[3][2] + c72[4]*JA72[3][4] + c72[5]*JA72[3][5];
pmm72[4]=state[24].thdd*h72[4] + p72[4] + c72[1]*JA72[4][1] + c72[2]*JA72[4][2] + c72[4]*JA72[4][4] + c72[5]*JA72[4][5];
pmm72[5]=state[24].thdd*h72[5] + p72[5] + c72[1]*JA72[5][1] + c72[2]*JA72[5][2] + c72[4]*JA72[5][4] + c72[5]*JA72[5][5];
pmm72[6]=state[24].thdd*h72[6] + p72[6] + c72[1]*JA72[6][1] + c72[2]*JA72[6][2] + c72[4]*JA72[6][4] + c72[5]*JA72[6][5];

pm72[1]=pmm72[1]*Si7172[1][1] + pmm72[2]*Si7172[1][2] + pmm72[3]*Si7172[1][3];
pm72[2]=pmm72[1]*Si7172[2][1] + pmm72[2]*Si7172[2][2] + pmm72[3]*Si7172[2][3];
pm72[3]=pmm72[1]*Si7172[3][1] + pmm72[2]*Si7172[3][2];
pm72[4]=pmm72[4]*Si7172[1][1] + pmm72[5]*Si7172[1][2] + pmm72[6]*Si7172[1][3];
pm72[5]=pmm72[4]*Si7172[2][1] + pmm72[5]*Si7172[2][2] + pmm72[6]*Si7172[2][3] - YHIP*pmm72[1]*Si7172[3][1] - YHIP*pmm72[2]*Si7172[3][2];
pm72[6]=YHIP*pmm72[1]*Si7172[2][1] + YHIP*pmm72[2]*Si7172[2][2] + YHIP*pmm72[3]*Si7172[2][3] + pmm72[4]*Si7172[3][1] + pmm72[5]*Si7172[3][2];

p71[1]=pm72[1] + pv71[1];
p71[2]=pm72[2] + pv71[2];
p71[3]=pm72[3] + pv71[3];
p71[4]=pm72[4] + pv71[4];
p71[5]=pm72[5] + pv71[5];
p71[6]=pm72[6] + pv71[6];

pmm71[1]=state[22].thdd*h71[1] + p71[1] + c71[1]*JA71[1][1] + c71[2]*JA71[1][2] + c71[4]*JA71[1][4] + c71[5]*JA71[1][5];
pmm71[2]=state[22].thdd*h71[2] + p71[2] + c71[1]*JA71[2][1] + c71[2]*JA71[2][2] + c71[4]*JA71[2][4] + c71[5]*JA71[2][5];
pmm71[3]=state[22].thdd*h71[3] + p71[3] + c71[1]*JA71[3][1] + c71[2]*JA71[3][2] + c71[4]*JA71[3][4] + c71[5]*JA71[3][5];
pmm71[4]=state[22].thdd*h71[4] + p71[4] + c71[1]*JA71[4][1] + c71[2]*JA71[4][2] + c71[4]*JA71[4][4] + c71[5]*JA71[4][5];
pmm71[5]=state[22].thdd*h71[5] + p71[5] + c71[1]*JA71[5][1] + c71[2]*JA71[5][2] + c71[4]*JA71[5][4] + c71[5]*JA71[5][5];
pmm71[6]=state[22].thdd*h71[6] + p71[6] + c71[1]*JA71[6][1] + c71[2]*JA71[6][2] + c71[4]*JA71[6][4] + c71[5]*JA71[6][5];

pm71[1]=pmm71[1]*Si7071[1][1] + pmm71[2]*Si7071[1][2];
pm71[2]=-pmm71[3];
pm71[3]=pmm71[1]*Si7071[3][1] + pmm71[2]*Si7071[3][2];
pm71[4]=pmm71[4]*Si7071[1][1] + pmm71[5]*Si7071[1][2];
pm71[5]=-pmm71[6];
pm71[6]=pmm71[4]*Si7071[3][1] + pmm71[5]*Si7071[3][2];

p70[1]=pm71[1] + pv70[1];
p70[2]=pm71[2] + pv70[2];
p70[3]=pm71[3] + pv70[3];
p70[4]=pm71[4] + pv70[4];
p70[5]=pm71[5] + pv70[5];
p70[6]=pm71[6] + pv70[6];

pmm70[1]=state[23].thdd*h70[1] + p70[1] + c70[1]*JA70[1][1] + c70[2]*JA70[1][2] + c70[4]*JA70[1][4] + c70[5]*JA70[1][5];
pmm70[2]=state[23].thdd*h70[2] + p70[2] + c70[1]*JA70[2][1] + c70[2]*JA70[2][2] + c70[4]*JA70[2][4] + c70[5]*JA70[2][5];
pmm70[3]=state[23].thdd*h70[3] + p70[3] + c70[1]*JA70[3][1] + c70[2]*JA70[3][2] + c70[4]*JA70[3][4] + c70[5]*JA70[3][5];
pmm70[4]=state[23].thdd*h70[4] + p70[4] + c70[1]*JA70[4][1] + c70[2]*JA70[4][2] + c70[4]*JA70[4][4] + c70[5]*JA70[4][5];
pmm70[5]=state[23].thdd*h70[5] + p70[5] + c70[1]*JA70[5][1] + c70[2]*JA70[5][2] + c70[4]*JA70[5][4] + c70[5]*JA70[5][5];
pmm70[6]=state[23].thdd*h70[6] + p70[6] + c70[1]*JA70[6][1] + c70[2]*JA70[6][2] + c70[4]*JA70[6][4] + c70[5]*JA70[6][5];

pm70[1]=pmm70[1]*Si070[1][1] + pmm70[2]*Si070[1][2];
pm70[2]=pmm70[3];
pm70[3]=pmm70[1]*Si070[3][1] + pmm70[2]*Si070[3][2];
pm70[4]=pmm70[4]*Si070[1][1] + pmm70[5]*Si070[1][2];
pm70[5]=pmm70[6] - XHIP*pmm70[1]*Si070[3][1] - XHIP*pmm70[2]*Si070[3][2];
pm70[6]=XHIP*pmm70[3] + pmm70[4]*Si070[3][1] + pmm70[5]*Si070[3][2];

p67[1]=0. + pv67[1];
p67[2]=0. + pv67[2];
p67[3]=0. + pv67[3];
p67[4]=0. + pv67[4];
p67[5]=0. + pv67[5];
p67[6]=0. + pv67[6];

pmm67[1]=state[38].thdd*h67[1] + p67[1] + c67[2]*JA67[1][2] + c67[4]*JA67[1][4];
pmm67[2]=state[38].thdd*h67[2] + p67[2] + c67[1]*JA67[2][1] + c67[5]*JA67[2][5];
pmm67[3]=p67[3] + c67[1]*JA67[3][1] + c67[2]*JA67[3][2];
pmm67[4]=state[38].thdd*h67[4] + p67[4] + c67[1]*JA67[4][1] + c67[2]*JA67[4][2] + c67[5]*JA67[4][5];
pmm67[5]=state[38].thdd*h67[5] + p67[5] + c67[1]*JA67[5][1] + c67[2]*JA67[5][2] + c67[4]*JA67[5][4];
pmm67[6]=state[38].thdd*h67[6] + p67[6] + c67[1]*JA67[6][1] + c67[2]*JA67[6][2] + c67[4]*JA67[6][4] + c67[5]*JA67[6][5];

pm67[1]=-pmm67[3];
pm67[2]=pmm67[1]*Si6667[2][1] + pmm67[2]*Si6667[2][2];
pm67[3]=pmm67[1]*Si6667[3][1] + pmm67[2]*Si6667[3][2];
pm67[4]=-pmm67[6];
pm67[5]=pmm67[4]*Si6667[2][1] + pmm67[5]*Si6667[2][2];
pm67[6]=pmm67[4]*Si6667[3][1] + pmm67[5]*Si6667[3][2];

p66[1]=pm67[1] + pv66[1];
p66[2]=pm67[2] + pv66[2];
p66[3]=pm67[3] + pv66[3];
p66[4]=pm67[4] + pv66[4];
p66[5]=pm67[5] + pv66[5];
p66[6]=pm67[6] + pv66[6];

pmm66[1]=state[37].thdd*h66[1] + p66[1] + c66[2]*JA66[1][2] + c66[4]*JA66[1][4];
pmm66[2]=state[37].thdd*h66[2] + p66[2] + c66[1]*JA66[2][1] + c66[2]*JA66[2][2] + c66[5]*JA66[2][5];
pmm66[3]=state[37].thdd*h66[3] + p66[3] + c66[1]*JA66[3][1] + c66[2]*JA66[3][2] + c66[5]*JA66[3][5];
pmm66[4]=state[37].thdd*h66[4] + p66[4] + c66[1]*JA66[4][1] + c66[2]*JA66[4][2] + c66[5]*JA66[4][5];
pmm66[5]=state[37].thdd*h66[5] + p66[5] + c66[1]*JA66[5][1] + c66[2]*JA66[5][2] + c66[4]*JA66[5][4] + c66[5]*JA66[5][5];
pmm66[6]=state[37].thdd*h66[6] + p66[6] + c66[1]*JA66[6][1] + c66[2]*JA66[6][2] + c66[4]*JA66[6][4] + c66[5]*JA66[6][5];

pm66[1]=pmm66[1]*Si6266[1][1] + pmm66[2]*Si6266[1][2];
pm66[2]=pmm66[1]*Si6266[2][1] + pmm66[2]*Si6266[2][2];
pm66[3]=pmm66[3];
pm66[4]=-(EYEYOFF*pmm66[3]) + pmm66[4]*Si6266[1][1] + pmm66[5]*Si6266[1][2] + HEAD*pmm66[1]*Si6266[2][1] + HEAD*pmm66[2]*Si6266[2][2];
pm66[5]=EYEXOFF*pmm66[3] - HEAD*pmm66[1]*Si6266[1][1] - HEAD*pmm66[2]*Si6266[1][2] + pmm66[4]*Si6266[2][1] + pmm66[5]*Si6266[2][2];
pm66[6]=pmm66[6] + pmm66[1]*(EYEYOFF*Si6266[1][1] - EYEXOFF*Si6266[2][1]) + pmm66[2]*(EYEYOFF*Si6266[1][2] - EYEXOFF*Si6266[2][2]);

p64[1]=0. + pv64[1];
p64[2]=0. + pv64[2];
p64[3]=0. + pv64[3];
p64[4]=0. + pv64[4];
p64[5]=0. + pv64[5];
p64[6]=0. + pv64[6];

pmm64[1]=state[36].thdd*h64[1] + p64[1] + c64[2]*JA64[1][2] + c64[4]*JA64[1][4];
pmm64[2]=state[36].thdd*h64[2] + p64[2] + c64[1]*JA64[2][1] + c64[5]*JA64[2][5];
pmm64[3]=p64[3] + c64[1]*JA64[3][1] + c64[2]*JA64[3][2];
pmm64[4]=state[36].thdd*h64[4] + p64[4] + c64[1]*JA64[4][1] + c64[2]*JA64[4][2] + c64[5]*JA64[4][5];
pmm64[5]=state[36].thdd*h64[5] + p64[5] + c64[1]*JA64[5][1] + c64[2]*JA64[5][2] + c64[4]*JA64[5][4];
pmm64[6]=state[36].thdd*h64[6] + p64[6] + c64[1]*JA64[6][1] + c64[2]*JA64[6][2] + c64[4]*JA64[6][4] + c64[5]*JA64[6][5];

pm64[1]=-pmm64[3];
pm64[2]=pmm64[1]*Si6364[2][1] + pmm64[2]*Si6364[2][2];
pm64[3]=pmm64[1]*Si6364[3][1] + pmm64[2]*Si6364[3][2];
pm64[4]=-pmm64[6];
pm64[5]=pmm64[4]*Si6364[2][1] + pmm64[5]*Si6364[2][2];
pm64[6]=pmm64[4]*Si6364[3][1] + pmm64[5]*Si6364[3][2];

p63[1]=pm64[1] + pv63[1];
p63[2]=pm64[2] + pv63[2];
p63[3]=pm64[3] + pv63[3];
p63[4]=pm64[4] + pv63[4];
p63[5]=pm64[5] + pv63[5];
p63[6]=pm64[6] + pv63[6];

pmm63[1]=state[35].thdd*h63[1] + p63[1] + c63[2]*JA63[1][2] + c63[4]*JA63[1][4];
pmm63[2]=state[35].thdd*h63[2] + p63[2] + c63[1]*JA63[2][1] + c63[2]*JA63[2][2] + c63[5]*JA63[2][5];
pmm63[3]=state[35].thdd*h63[3] + p63[3] + c63[1]*JA63[3][1] + c63[2]*JA63[3][2] + c63[5]*JA63[3][5];
pmm63[4]=state[35].thdd*h63[4] + p63[4] + c63[1]*JA63[4][1] + c63[2]*JA63[4][2] + c63[5]*JA63[4][5];
pmm63[5]=state[35].thdd*h63[5] + p63[5] + c63[1]*JA63[5][1] + c63[2]*JA63[5][2] + c63[4]*JA63[5][4] + c63[5]*JA63[5][5];
pmm63[6]=state[35].thdd*h63[6] + p63[6] + c63[1]*JA63[6][1] + c63[2]*JA63[6][2] + c63[4]*JA63[6][4] + c63[5]*JA63[6][5];

pm63[1]=pmm63[1]*Si6263[1][1] + pmm63[2]*Si6263[1][2];
pm63[2]=pmm63[1]*Si6263[2][1] + pmm63[2]*Si6263[2][2];
pm63[3]=pmm63[3];
pm63[4]=-(EYEYOFF*pmm63[3]) + pmm63[4]*Si6263[1][1] + pmm63[5]*Si6263[1][2] + HEAD*pmm63[1]*Si6263[2][1] + HEAD*pmm63[2]*Si6263[2][2];
pm63[5]=-(EYEXOFF*pmm63[3]) - HEAD*pmm63[1]*Si6263[1][1] - HEAD*pmm63[2]*Si6263[1][2] + pmm63[4]*Si6263[2][1] + pmm63[5]*Si6263[2][2];
pm63[6]=pmm63[6] + pmm63[1]*(EYEYOFF*Si6263[1][1] + EYEXOFF*Si6263[2][1]) + pmm63[2]*(EYEYOFF*Si6263[1][2] + EYEXOFF*Si6263[2][2]);

p62[1]=0. + pm63[1] + pm66[1] + pv62[1];
p62[2]=0. + pm63[2] + pm66[2] + pv62[2];
p62[3]=0. + pm63[3] + pm66[3] + pv62[3];
p62[4]=0. + pm63[4] + pm66[4] + pv62[4];
p62[5]=0. + pm63[5] + pm66[5] + pv62[5];
p62[6]=0. + pm63[6] + pm66[6] + pv62[6];

pmm62[1]=state[34].thdd*h62[1] + p62[1] + c62[1]*JA62[1][1] + c62[2]*JA62[1][2] + c62[4]*JA62[1][4] + c62[5]*JA62[1][5];
pmm62[2]=state[34].thdd*h62[2] + p62[2] + c62[1]*JA62[2][1] + c62[2]*JA62[2][2] + c62[4]*JA62[2][4] + c62[5]*JA62[2][5];
pmm62[3]=state[34].thdd*h62[3] + p62[3] + c62[1]*JA62[3][1] + c62[2]*JA62[3][2] + c62[4]*JA62[3][4] + c62[5]*JA62[3][5];
pmm62[4]=state[34].thdd*h62[4] + p62[4] + c62[1]*JA62[4][1] + c62[2]*JA62[4][2] + c62[4]*JA62[4][4] + c62[5]*JA62[4][5];
pmm62[5]=state[34].thdd*h62[5] + p62[5] + c62[1]*JA62[5][1] + c62[2]*JA62[5][2] + c62[4]*JA62[5][4] + c62[5]*JA62[5][5];
pmm62[6]=state[34].thdd*h62[6] + p62[6] + c62[1]*JA62[6][1] + c62[2]*JA62[6][2] + c62[4]*JA62[6][4] + c62[5]*JA62[6][5];

pm62[1]=pmm62[1]*Si6162[1][1] + pmm62[2]*Si6162[1][2];
pm62[2]=pmm62[3];
pm62[3]=pmm62[1]*Si6162[3][1] + pmm62[2]*Si6162[3][2];
pm62[4]=pmm62[4]*Si6162[1][1] + pmm62[5]*Si6162[1][2];
pm62[5]=pmm62[6];
pm62[6]=pmm62[4]*Si6162[3][1] + pmm62[5]*Si6162[3][2];

p61[1]=pm62[1] + pv61[1];
p61[2]=pm62[2] + pv61[2];
p61[3]=pm62[3] + pv61[3];
p61[4]=pm62[4] + pv61[4];
p61[5]=pm62[5] + pv61[5];
p61[6]=pm62[6] + pv61[6];

pmm61[1]=state[33].thdd*h61[1] + p61[1] + c61[1]*JA61[1][1] + c61[2]*JA61[1][2] + c61[4]*JA61[1][4] + c61[5]*JA61[1][5];
pmm61[2]=state[33].thdd*h61[2] + p61[2] + c61[1]*JA61[2][1] + c61[2]*JA61[2][2] + c61[4]*JA61[2][4] + c61[5]*JA61[2][5];
pmm61[3]=state[33].thdd*h61[3] + p61[3] + c61[1]*JA61[3][1] + c61[2]*JA61[3][2] + c61[4]*JA61[3][4] + c61[5]*JA61[3][5];
pmm61[4]=state[33].thdd*h61[4] + p61[4] + c61[1]*JA61[4][1] + c61[2]*JA61[4][2] + c61[4]*JA61[4][4] + c61[5]*JA61[4][5];
pmm61[5]=state[33].thdd*h61[5] + p61[5] + c61[1]*JA61[5][1] + c61[2]*JA61[5][2] + c61[4]*JA61[5][4] + c61[5]*JA61[5][5];
pmm61[6]=state[33].thdd*h61[6] + p61[6] + c61[1]*JA61[6][1] + c61[2]*JA61[6][2] + c61[4]*JA61[6][4] + c61[5]*JA61[6][5];

pm61[1]=pmm61[3];
pm61[2]=pmm61[1]*Si6061[2][1] + pmm61[2]*Si6061[2][2];
pm61[3]=pmm61[1]*Si6061[3][1] + pmm61[2]*Si6061[3][2];
pm61[4]=pmm61[6] - CERVICAL*pmm61[1]*Si6061[3][1] - CERVICAL*pmm61[2]*Si6061[3][2];
pm61[5]=pmm61[4]*Si6061[2][1] + pmm61[5]*Si6061[2][2];
pm61[6]=CERVICAL*pmm61[3] + pmm61[4]*Si6061[3][1] + pmm61[5]*Si6061[3][2];

p60[1]=pm61[1] + pv60[1];
p60[2]=pm61[2] + pv60[2];
p60[3]=pm61[3] + pv60[3];
p60[4]=pm61[4] + pv60[4];
p60[5]=pm61[5] + pv60[5];
p60[6]=pm61[6] + pv60[6];

pmm60[1]=state[32].thdd*h60[1] + p60[1] + c60[1]*JA60[1][1] + c60[2]*JA60[1][2] + c60[4]*JA60[1][4] + c60[5]*JA60[1][5];
pmm60[2]=state[32].thdd*h60[2] + p60[2] + c60[1]*JA60[2][1] + c60[2]*JA60[2][2] + c60[4]*JA60[2][4] + c60[5]*JA60[2][5];
pmm60[3]=state[32].thdd*h60[3] + p60[3] + c60[1]*JA60[3][1] + c60[2]*JA60[3][2] + c60[4]*JA60[3][4] + c60[5]*JA60[3][5];
pmm60[4]=state[32].thdd*h60[4] + p60[4] + c60[1]*JA60[4][1] + c60[2]*JA60[4][2] + c60[4]*JA60[4][4] + c60[5]*JA60[4][5];
pmm60[5]=state[32].thdd*h60[5] + p60[5] + c60[1]*JA60[5][1] + c60[2]*JA60[5][2] + c60[4]*JA60[5][4] + c60[5]*JA60[5][5];
pmm60[6]=state[32].thdd*h60[6] + p60[6] + c60[1]*JA60[6][1] + c60[2]*JA60[6][2] + c60[4]*JA60[6][4] + c60[5]*JA60[6][5];

pm60[1]=pmm60[1]*Si360[1][1] + pmm60[2]*Si360[1][2];
pm60[2]=pmm60[1]*Si360[2][1] + pmm60[2]*Si360[2][2];
pm60[3]=pmm60[3];
pm60[4]=pmm60[4]*Si360[1][1] + pmm60[5]*Si360[1][2];
pm60[5]=THORAX2NECK*pmm60[3] + pmm60[4]*Si360[2][1] + pmm60[5]*Si360[2][2];
pm60[6]=pmm60[6] - THORAX2NECK*pmm60[1]*Si360[2][1] - THORAX2NECK*pmm60[2]*Si360[2][2];

p56[1]=0. + pv56[1];
p56[2]=0. + pv56[2];
p56[3]=0. + pv56[3];
p56[4]=0. + pv56[4];
p56[5]=0. + pv56[5];
p56[6]=0. + pv56[6];

pmm56[1]=state[50].thdd*h56[1] + p56[1] + c56[2]*JA56[1][2] + c56[4]*JA56[1][4];
pmm56[2]=state[50].thdd*h56[2] + p56[2] + c56[1]*JA56[2][1] + c56[5]*JA56[2][5];
pmm56[3]=p56[3] + c56[1]*JA56[3][1] + c56[2]*JA56[3][2];
pmm56[4]=state[50].thdd*h56[4] + p56[4] + c56[1]*JA56[4][1] + c56[2]*JA56[4][2] + c56[5]*JA56[4][5];
pmm56[5]=state[50].thdd*h56[5] + p56[5] + c56[1]*JA56[5][1] + c56[2]*JA56[5][2] + c56[4]*JA56[5][4];
pmm56[6]=state[50].thdd*h56[6] + p56[6] + c56[1]*JA56[6][1] + c56[2]*JA56[6][2] + c56[4]*JA56[6][4] + c56[5]*JA56[6][5];

pm56[1]=pmm56[1]*Si3856[1][1] + pmm56[2]*Si3856[1][2];
pm56[2]=pmm56[1]*Si3856[2][1] + pmm56[2]*Si3856[2][2] + pmm56[3]*Si3856[2][3];
pm56[3]=pmm56[1]*Si3856[3][1] + pmm56[2]*Si3856[3][2] + pmm56[3]*Si3856[3][3];
pm56[4]=pmm56[4]*Si3856[1][1] + pmm56[5]*Si3856[1][2] + pmm56[1]*(-(ZLF*Si3856[2][1]) + YLF*Si3856[3][1]) + pmm56[2]*(-(ZLF*Si3856[2][2]) + YLF*Si3856[3][2]) + pmm56[3]*(-(ZLF*Si3856[2][3]) + YLF*Si3856[3][3]);
pm56[5]=pmm56[4]*Si3856[2][1] + pmm56[5]*Si3856[2][2] + pmm56[6]*Si3856[2][3] + pmm56[1]*(ZLF*Si3856[1][1] - XLF*Si3856[3][1]) + pmm56[2]*(ZLF*Si3856[1][2] - XLF*Si3856[3][2]) - XLF*pmm56[3]*Si3856[3][3];
pm56[6]=pmm56[1]*(-(YLF*Si3856[1][1]) + XLF*Si3856[2][1]) + pmm56[2]*(-(YLF*Si3856[1][2]) + XLF*Si3856[2][2]) + XLF*pmm56[3]*Si3856[2][3] + pmm56[4]*Si3856[3][1] + pmm56[5]*Si3856[3][2] + pmm56[6]*Si3856[3][3];

p52[1]=0. + pv52[1];
p52[2]=0. + pv52[2];
p52[3]=0. + pv52[3];
p52[4]=0. + pv52[4];
p52[5]=0. + pv52[5];
p52[6]=0. + pv52[6];

pmm52[1]=state[49].thdd*h52[1] + p52[1] + c52[2]*JA52[1][2] + c52[4]*JA52[1][4];
pmm52[2]=state[49].thdd*h52[2] + p52[2] + c52[1]*JA52[2][1] + c52[5]*JA52[2][5];
pmm52[3]=p52[3] + c52[1]*JA52[3][1] + c52[2]*JA52[3][2];
pmm52[4]=state[49].thdd*h52[4] + p52[4] + c52[1]*JA52[4][1] + c52[2]*JA52[4][2] + c52[5]*JA52[4][5];
pmm52[5]=state[49].thdd*h52[5] + p52[5] + c52[1]*JA52[5][1] + c52[2]*JA52[5][2] + c52[4]*JA52[5][4];
pmm52[6]=state[49].thdd*h52[6] + p52[6] + c52[1]*JA52[6][1] + c52[2]*JA52[6][2] + c52[4]*JA52[6][4] + c52[5]*JA52[6][5];

pm52[1]=pmm52[1]*Si3852[1][1] + pmm52[2]*Si3852[1][2];
pm52[2]=pmm52[1]*Si3852[2][1] + pmm52[2]*Si3852[2][2] + pmm52[3]*Si3852[2][3];
pm52[3]=pmm52[1]*Si3852[3][1] + pmm52[2]*Si3852[3][2] + pmm52[3]*Si3852[3][3];
pm52[4]=pmm52[4]*Si3852[1][1] + pmm52[5]*Si3852[1][2] + pmm52[1]*(-(ZRF*Si3852[2][1]) + YRF*Si3852[3][1]) + pmm52[2]*(-(ZRF*Si3852[2][2]) + YRF*Si3852[3][2]) + pmm52[3]*(-(ZRF*Si3852[2][3]) + YRF*Si3852[3][3]);
pm52[5]=pmm52[4]*Si3852[2][1] + pmm52[5]*Si3852[2][2] + pmm52[6]*Si3852[2][3] + pmm52[1]*(ZRF*Si3852[1][1] - XRF*Si3852[3][1]) + pmm52[2]*(ZRF*Si3852[1][2] - XRF*Si3852[3][2]) - XRF*pmm52[3]*Si3852[3][3];
pm52[6]=pmm52[1]*(-(YRF*Si3852[1][1]) + XRF*Si3852[2][1]) + pmm52[2]*(-(YRF*Si3852[1][2]) + XRF*Si3852[2][2]) + XRF*pmm52[3]*Si3852[2][3] + pmm52[4]*Si3852[3][1] + pmm52[5]*Si3852[3][2] + pmm52[6]*Si3852[3][3];

p48[1]=0. + pv48[1];
p48[2]=0. + pv48[2];
p48[3]=0. + pv48[3];
p48[4]=0. + pv48[4];
p48[5]=0. + pv48[5];
p48[6]=0. + pv48[6];

pmm48[1]=state[48].thdd*h48[1] + p48[1] + c48[2]*JA48[1][2] + c48[4]*JA48[1][4];
pmm48[2]=state[48].thdd*h48[2] + p48[2] + c48[1]*JA48[2][1] + c48[5]*JA48[2][5];
pmm48[3]=p48[3] + c48[1]*JA48[3][1] + c48[2]*JA48[3][2];
pmm48[4]=state[48].thdd*h48[4] + p48[4] + c48[1]*JA48[4][1] + c48[2]*JA48[4][2] + c48[5]*JA48[4][5];
pmm48[5]=state[48].thdd*h48[5] + p48[5] + c48[1]*JA48[5][1] + c48[2]*JA48[5][2] + c48[4]*JA48[5][4];
pmm48[6]=state[48].thdd*h48[6] + p48[6] + c48[1]*JA48[6][1] + c48[2]*JA48[6][2] + c48[4]*JA48[6][4] + c48[5]*JA48[6][5];

pm48[1]=pmm48[1]*Si3848[1][1] + pmm48[2]*Si3848[1][2];
pm48[2]=pmm48[1]*Si3848[2][1] + pmm48[2]*Si3848[2][2] + pmm48[3]*Si3848[2][3];
pm48[3]=pmm48[1]*Si3848[3][1] + pmm48[2]*Si3848[3][2] + pmm48[3]*Si3848[3][3];
pm48[4]=pmm48[4]*Si3848[1][1] + pmm48[5]*Si3848[1][2] + pmm48[1]*(-(ZMF*Si3848[2][1]) + YMF*Si3848[3][1]) + pmm48[2]*(-(ZMF*Si3848[2][2]) + YMF*Si3848[3][2]) + pmm48[3]*(-(ZMF*Si3848[2][3]) + YMF*Si3848[3][3]);
pm48[5]=pmm48[4]*Si3848[2][1] + pmm48[5]*Si3848[2][2] + pmm48[6]*Si3848[2][3] + pmm48[1]*(ZMF*Si3848[1][1] - XMF*Si3848[3][1]) + pmm48[2]*(ZMF*Si3848[1][2] - XMF*Si3848[3][2]) - XMF*pmm48[3]*Si3848[3][3];
pm48[6]=pmm48[1]*(-(YMF*Si3848[1][1]) + XMF*Si3848[2][1]) + pmm48[2]*(-(YMF*Si3848[1][2]) + XMF*Si3848[2][2]) + XMF*pmm48[3]*Si3848[2][3] + pmm48[4]*Si3848[3][1] + pmm48[5]*Si3848[3][2] + pmm48[6]*Si3848[3][3];

p44[1]=0. + pv44[1];
p44[2]=0. + pv44[2];
p44[3]=0. + pv44[3];
p44[4]=0. + pv44[4];
p44[5]=0. + pv44[5];
p44[6]=0. + pv44[6];

pmm44[1]=state[47].thdd*h44[1] + p44[1] + c44[2]*JA44[1][2] + c44[4]*JA44[1][4];
pmm44[2]=state[47].thdd*h44[2] + p44[2] + c44[1]*JA44[2][1] + c44[5]*JA44[2][5];
pmm44[3]=p44[3] + c44[1]*JA44[3][1] + c44[2]*JA44[3][2];
pmm44[4]=state[47].thdd*h44[4] + p44[4] + c44[1]*JA44[4][1] + c44[2]*JA44[4][2] + c44[5]*JA44[4][5];
pmm44[5]=state[47].thdd*h44[5] + p44[5] + c44[1]*JA44[5][1] + c44[2]*JA44[5][2] + c44[4]*JA44[5][4];
pmm44[6]=state[47].thdd*h44[6] + p44[6] + c44[1]*JA44[6][1] + c44[2]*JA44[6][2] + c44[4]*JA44[6][4] + c44[5]*JA44[6][5];

pm44[1]=pmm44[1]*Si3844[1][1] + pmm44[2]*Si3844[1][2];
pm44[2]=pmm44[1]*Si3844[2][1] + pmm44[2]*Si3844[2][2] + pmm44[3]*Si3844[2][3];
pm44[3]=pmm44[1]*Si3844[3][1] + pmm44[2]*Si3844[3][2] + pmm44[3]*Si3844[3][3];
pm44[4]=pmm44[4]*Si3844[1][1] + pmm44[5]*Si3844[1][2] + pmm44[1]*(-(ZIF*Si3844[2][1]) + YIF*Si3844[3][1]) + pmm44[2]*(-(ZIF*Si3844[2][2]) + YIF*Si3844[3][2]) + pmm44[3]*(-(ZIF*Si3844[2][3]) + YIF*Si3844[3][3]);
pm44[5]=pmm44[4]*Si3844[2][1] + pmm44[5]*Si3844[2][2] + pmm44[6]*Si3844[2][3] + pmm44[1]*(ZIF*Si3844[1][1] - XIF*Si3844[3][1]) + pmm44[2]*(ZIF*Si3844[1][2] - XIF*Si3844[3][2]) - XIF*pmm44[3]*Si3844[3][3];
pm44[6]=pmm44[1]*(-(YIF*Si3844[1][1]) + XIF*Si3844[2][1]) + pmm44[2]*(-(YIF*Si3844[1][2]) + XIF*Si3844[2][2]) + XIF*pmm44[3]*Si3844[2][3] + pmm44[4]*Si3844[3][1] + pmm44[5]*Si3844[3][2] + pmm44[6]*Si3844[3][3];

p41[1]=0. + pv41[1];
p41[2]=0. + pv41[2];
p41[3]=0. + pv41[3];
p41[4]=0. + pv41[4];
p41[5]=0. + pv41[5];
p41[6]=0. + pv41[6];

pmm41[1]=state[46].thdd*h41[1] + p41[1] + c41[2]*JA41[1][2] + c41[4]*JA41[1][4];
pmm41[2]=state[46].thdd*h41[2] + p41[2] + c41[1]*JA41[2][1] + c41[5]*JA41[2][5];
pmm41[3]=p41[3] + c41[1]*JA41[3][1] + c41[2]*JA41[3][2];
pmm41[4]=state[46].thdd*h41[4] + p41[4] + c41[1]*JA41[4][1] + c41[2]*JA41[4][2] + c41[5]*JA41[4][5];
pmm41[5]=state[46].thdd*h41[5] + p41[5] + c41[1]*JA41[5][1] + c41[2]*JA41[5][2] + c41[4]*JA41[5][4];
pmm41[6]=state[46].thdd*h41[6] + p41[6] + c41[1]*JA41[6][1] + c41[2]*JA41[6][2] + c41[4]*JA41[6][4] + c41[5]*JA41[6][5];

pm41[1]=pmm41[1]*Si4041[1][1] + pmm41[2]*Si4041[1][2];
pm41[2]=pmm41[1]*Si4041[2][1] + pmm41[2]*Si4041[2][2];
pm41[3]=pmm41[3];
pm41[4]=YTHUMBFLEX*pmm41[3] + pmm41[4]*Si4041[1][1] + pmm41[5]*Si4041[1][2];
pm41[5]=-(XTHUMBFLEX*pmm41[3]) + pmm41[4]*Si4041[2][1] + pmm41[5]*Si4041[2][2];
pm41[6]=pmm41[6] + pmm41[1]*(-(YTHUMBFLEX*Si4041[1][1]) + XTHUMBFLEX*Si4041[2][1]) + pmm41[2]*(-(YTHUMBFLEX*Si4041[1][2]) + XTHUMBFLEX*Si4041[2][2]);

p40[1]=pm41[1] + pv40[1];
p40[2]=pm41[2] + pv40[2];
p40[3]=pm41[3] + pv40[3];
p40[4]=pm41[4] + pv40[4];
p40[5]=pm41[5] + pv40[5];
p40[6]=pm41[6] + pv40[6];

pmm40[1]=state[45].thdd*h40[1] + p40[1] + c40[2]*JA40[1][2] + c40[3]*JA40[1][3] + c40[5]*JA40[1][5];
pmm40[2]=state[45].thdd*h40[2] + p40[2] + c40[2]*JA40[2][2] + c40[3]*JA40[2][3] + c40[5]*JA40[2][5];
pmm40[3]=state[45].thdd*h40[3] + p40[3] + c40[2]*JA40[3][2] + c40[6]*JA40[3][6];
pmm40[4]=state[45].thdd*h40[4] + p40[4] + c40[2]*JA40[4][2] + c40[3]*JA40[4][3] + c40[5]*JA40[4][5] + c40[6]*JA40[4][6];
pmm40[5]=state[45].thdd*h40[5] + p40[5] + c40[2]*JA40[5][2] + c40[3]*JA40[5][3] + c40[5]*JA40[5][5] + c40[6]*JA40[5][6];
pmm40[6]=state[45].thdd*h40[6] + p40[6] + c40[2]*JA40[6][2] + c40[3]*JA40[6][3] + c40[5]*JA40[6][5];

pm40[1]=pmm40[1]*Si3840[1][1] + pmm40[2]*Si3840[1][2] + pmm40[3]*Si3840[1][3];
pm40[2]=pmm40[1]*Si3840[2][1] + pmm40[2]*Si3840[2][2] + pmm40[3]*Si3840[2][3];
pm40[3]=pmm40[1]*Si3840[3][1] + pmm40[2]*Si3840[3][2] + pmm40[3]*Si3840[3][3];
pm40[4]=pmm40[4]*Si3840[1][1] + pmm40[5]*Si3840[1][2] + pmm40[6]*Si3840[1][3] + pmm40[1]*(-(ZTHUMB*Si3840[2][1]) + YTHUMB*Si3840[3][1]) + pmm40[2]*(-(ZTHUMB*Si3840[2][2]) + YTHUMB*Si3840[3][2]) + pmm40[3]*(-(ZTHUMB*Si3840[2][3]) + YTHUMB*Si3840[3][3]);
pm40[5]=pmm40[4]*Si3840[2][1] + pmm40[5]*Si3840[2][2] + pmm40[6]*Si3840[2][3] + pmm40[1]*(ZTHUMB*Si3840[1][1] - XTHUMB*Si3840[3][1]) + pmm40[2]*(ZTHUMB*Si3840[1][2] - XTHUMB*Si3840[3][2]) + pmm40[3]*(ZTHUMB*Si3840[1][3] - XTHUMB*Si3840[3][3]);
pm40[6]=pmm40[1]*(-(YTHUMB*Si3840[1][1]) + XTHUMB*Si3840[2][1]) + pmm40[2]*(-(YTHUMB*Si3840[1][2]) + XTHUMB*Si3840[2][2]) + pmm40[3]*(-(YTHUMB*Si3840[1][3]) + XTHUMB*Si3840[2][3]) + pmm40[4]*Si3840[3][1] + pmm40[5]*Si3840[3][2] + pmm40[6]*Si3840[3][3];

p39[1]=pv39[1];
p39[2]=pv39[2];
p39[3]=pv39[3];
p39[4]=pv39[4];
p39[5]=pv39[5];
p39[6]=pv39[6];

pmm39[1]=p39[1];
pmm39[2]=p39[2];
pmm39[3]=p39[3];
pmm39[4]=p39[4];
pmm39[5]=p39[5];
pmm39[6]=p39[6];

pm39[1]=pmm39[1]*Si3839[1][1] + pmm39[2]*Si3839[1][2] + pmm39[3]*Si3839[1][3];
pm39[2]=pmm39[1]*Si3839[2][1] + pmm39[2]*Si3839[2][2] + pmm39[3]*Si3839[2][3];
pm39[3]=pmm39[1]*Si3839[3][1] + pmm39[2]*Si3839[3][2] + pmm39[3]*Si3839[3][3];
pm39[4]=pmm39[4]*Si3839[1][1] + pmm39[5]*Si3839[1][2] + pmm39[6]*Si3839[1][3] + pmm39[1]*(-(eff[1].x[3]*Si3839[2][1]) + eff[1].x[2]*Si3839[3][1]) + pmm39[2]*(-(eff[1].x[3]*Si3839[2][2]) + eff[1].x[2]*Si3839[3][2]) + pmm39[3]*(-(eff[1].x[3]*Si3839[2][3]) + eff[1].x[2]*Si3839[3][3]);
pm39[5]=pmm39[4]*Si3839[2][1] + pmm39[5]*Si3839[2][2] + pmm39[6]*Si3839[2][3] + pmm39[1]*(eff[1].x[3]*Si3839[1][1] - eff[1].x[1]*Si3839[3][1]) + pmm39[2]*(eff[1].x[3]*Si3839[1][2] - eff[1].x[1]*Si3839[3][2]) + pmm39[3]*(eff[1].x[3]*Si3839[1][3] - eff[1].x[1]*Si3839[3][3]);
pm39[6]=pmm39[1]*(-(eff[1].x[2]*Si3839[1][1]) + eff[1].x[1]*Si3839[2][1]) + pmm39[2]*(-(eff[1].x[2]*Si3839[1][2]) + eff[1].x[1]*Si3839[2][2]) + pmm39[3]*(-(eff[1].x[2]*Si3839[1][3]) + eff[1].x[1]*Si3839[2][3]) + pmm39[4]*Si3839[3][1] + pmm39[5]*Si3839[3][2] + pmm39[6]*Si3839[3][3];

p38[1]=pm39[1] + pm40[1] + pm44[1] + pm48[1] + pm52[1] + pm56[1] + pv38[1];
p38[2]=pm39[2] + pm40[2] + pm44[2] + pm48[2] + pm52[2] + pm56[2] + pv38[2];
p38[3]=pm39[3] + pm40[3] + pm44[3] + pm48[3] + pm52[3] + pm56[3] + pv38[3];
p38[4]=pm39[4] + pm40[4] + pm44[4] + pm48[4] + pm52[4] + pm56[4] + pv38[4];
p38[5]=pm39[5] + pm40[5] + pm44[5] + pm48[5] + pm52[5] + pm56[5] + pv38[5];
p38[6]=pm39[6] + pm40[6] + pm44[6] + pm48[6] + pm52[6] + pm56[6] + pv38[6];

pmm38[1]=state[14].thdd*h38[1] + p38[1] + c38[1]*JA38[1][1] + c38[2]*JA38[1][2] + c38[4]*JA38[1][4] + c38[5]*JA38[1][5];
pmm38[2]=state[14].thdd*h38[2] + p38[2] + c38[1]*JA38[2][1] + c38[2]*JA38[2][2] + c38[4]*JA38[2][4] + c38[5]*JA38[2][5];
pmm38[3]=state[14].thdd*h38[3] + p38[3] + c38[1]*JA38[3][1] + c38[2]*JA38[3][2] + c38[4]*JA38[3][4] + c38[5]*JA38[3][5];
pmm38[4]=state[14].thdd*h38[4] + p38[4] + c38[1]*JA38[4][1] + c38[2]*JA38[4][2] + c38[4]*JA38[4][4] + c38[5]*JA38[4][5];
pmm38[5]=state[14].thdd*h38[5] + p38[5] + c38[1]*JA38[5][1] + c38[2]*JA38[5][2] + c38[4]*JA38[5][4] + c38[5]*JA38[5][5];
pmm38[6]=state[14].thdd*h38[6] + p38[6] + c38[1]*JA38[6][1] + c38[2]*JA38[6][2] + c38[4]*JA38[6][4] + c38[5]*JA38[6][5];

pm38[1]=pmm38[1]*Si3738[1][1] + pmm38[2]*Si3738[1][2];
pm38[2]=pmm38[3];
pm38[3]=pmm38[1]*Si3738[3][1] + pmm38[2]*Si3738[3][2];
pm38[4]=pmm38[4]*Si3738[1][1] + pmm38[5]*Si3738[1][2];
pm38[5]=pmm38[6];
pm38[6]=pmm38[4]*Si3738[3][1] + pmm38[5]*Si3738[3][2];

p37[1]=pm38[1] + pv37[1];
p37[2]=pm38[2] + pv37[2];
p37[3]=pm38[3] + pv37[3];
p37[4]=pm38[4] + pv37[4];
p37[5]=pm38[5] + pv37[5];
p37[6]=pm38[6] + pv37[6];

pmm37[1]=state[13].thdd*h37[1] + p37[1] + c37[1]*JA37[1][1] + c37[2]*JA37[1][2] + c37[4]*JA37[1][4] + c37[5]*JA37[1][5];
pmm37[2]=state[13].thdd*h37[2] + p37[2] + c37[1]*JA37[2][1] + c37[2]*JA37[2][2] + c37[4]*JA37[2][4] + c37[5]*JA37[2][5];
pmm37[3]=state[13].thdd*h37[3] + p37[3] + c37[1]*JA37[3][1] + c37[2]*JA37[3][2] + c37[4]*JA37[3][4] + c37[5]*JA37[3][5];
pmm37[4]=state[13].thdd*h37[4] + p37[4] + c37[1]*JA37[4][1] + c37[2]*JA37[4][2] + c37[4]*JA37[4][4] + c37[5]*JA37[4][5];
pmm37[5]=state[13].thdd*h37[5] + p37[5] + c37[1]*JA37[5][1] + c37[2]*JA37[5][2] + c37[4]*JA37[5][4] + c37[5]*JA37[5][5];
pmm37[6]=state[13].thdd*h37[6] + p37[6] + c37[1]*JA37[6][1] + c37[2]*JA37[6][2] + c37[4]*JA37[6][4] + c37[5]*JA37[6][5];

pm37[1]=-pmm37[3];
pm37[2]=pmm37[1]*Si3637[2][1] + pmm37[2]*Si3637[2][2];
pm37[3]=pmm37[1]*Si3637[3][1] + pmm37[2]*Si3637[3][2];
pm37[4]=-pmm37[6] + pmm37[1]*(-(LOWERARM*Si3637[2][1]) + WRISTY*Si3637[3][1]) + pmm37[2]*(-(LOWERARM*Si3637[2][2]) + WRISTY*Si3637[3][2]);
pm37[5]=-(LOWERARM*pmm37[3]) + pmm37[4]*Si3637[2][1] + pmm37[5]*Si3637[2][2];
pm37[6]=WRISTY*pmm37[3] + pmm37[4]*Si3637[3][1] + pmm37[5]*Si3637[3][2];

p36[1]=pm37[1] + pv36[1];
p36[2]=pm37[2] + pv36[2];
p36[3]=pm37[3] + pv36[3];
p36[4]=pm37[4] + pv36[4];
p36[5]=pm37[5] + pv36[5];
p36[6]=pm37[6] + pv36[6];

pmm36[1]=state[12].thdd*h36[1] + p36[1] + c36[1]*JA36[1][1] + c36[2]*JA36[1][2] + c36[4]*JA36[1][4] + c36[5]*JA36[1][5];
pmm36[2]=state[12].thdd*h36[2] + p36[2] + c36[1]*JA36[2][1] + c36[2]*JA36[2][2] + c36[4]*JA36[2][4] + c36[5]*JA36[2][5];
pmm36[3]=state[12].thdd*h36[3] + p36[3] + c36[1]*JA36[3][1] + c36[2]*JA36[3][2] + c36[4]*JA36[3][4] + c36[5]*JA36[3][5];
pmm36[4]=state[12].thdd*h36[4] + p36[4] + c36[1]*JA36[4][1] + c36[2]*JA36[4][2] + c36[4]*JA36[4][4] + c36[5]*JA36[4][5];
pmm36[5]=state[12].thdd*h36[5] + p36[5] + c36[1]*JA36[5][1] + c36[2]*JA36[5][2] + c36[4]*JA36[5][4] + c36[5]*JA36[5][5];
pmm36[6]=state[12].thdd*h36[6] + p36[6] + c36[1]*JA36[6][1] + c36[2]*JA36[6][2] + c36[4]*JA36[6][4] + c36[5]*JA36[6][5];

pm36[1]=pmm36[1]*Si3536[1][1] + pmm36[2]*Si3536[1][2];
pm36[2]=-pmm36[3];
pm36[3]=pmm36[1]*Si3536[3][1] + pmm36[2]*Si3536[3][2];
pm36[4]=pmm36[4]*Si3536[1][1] + pmm36[5]*Si3536[1][2];
pm36[5]=-pmm36[6];
pm36[6]=pmm36[4]*Si3536[3][1] + pmm36[5]*Si3536[3][2];

p35[1]=pm36[1] + pv35[1];
p35[2]=pm36[2] + pv35[2];
p35[3]=pm36[3] + pv35[3];
p35[4]=pm36[4] + pv35[4];
p35[5]=pm36[5] + pv35[5];
p35[6]=pm36[6] + pv35[6];

pmm35[1]=state[11].thdd*h35[1] + p35[1] + c35[1]*JA35[1][1] + c35[2]*JA35[1][2] + c35[4]*JA35[1][4] + c35[5]*JA35[1][5];
pmm35[2]=state[11].thdd*h35[2] + p35[2] + c35[1]*JA35[2][1] + c35[2]*JA35[2][2] + c35[4]*JA35[2][4] + c35[5]*JA35[2][5];
pmm35[3]=state[11].thdd*h35[3] + p35[3] + c35[1]*JA35[3][1] + c35[2]*JA35[3][2] + c35[4]*JA35[3][4] + c35[5]*JA35[3][5];
pmm35[4]=state[11].thdd*h35[4] + p35[4] + c35[1]*JA35[4][1] + c35[2]*JA35[4][2] + c35[4]*JA35[4][4] + c35[5]*JA35[4][5];
pmm35[5]=state[11].thdd*h35[5] + p35[5] + c35[1]*JA35[5][1] + c35[2]*JA35[5][2] + c35[4]*JA35[5][4] + c35[5]*JA35[5][5];
pmm35[6]=state[11].thdd*h35[6] + p35[6] + c35[1]*JA35[6][1] + c35[2]*JA35[6][2] + c35[4]*JA35[6][4] + c35[5]*JA35[6][5];

pm35[1]=-pmm35[3];
pm35[2]=pmm35[1]*Si3435[2][1] + pmm35[2]*Si3435[2][2];
pm35[3]=pmm35[1]*Si3435[3][1] + pmm35[2]*Si3435[3][2];
pm35[4]=-pmm35[6] - UPPERARM*pmm35[1]*Si3435[2][1] - UPPERARM*pmm35[2]*Si3435[2][2];
pm35[5]=-(UPPERARM*pmm35[3]) + pmm35[4]*Si3435[2][1] + pmm35[5]*Si3435[2][2];
pm35[6]=pmm35[4]*Si3435[3][1] + pmm35[5]*Si3435[3][2];

p34[1]=pm35[1] + pv34[1];
p34[2]=pm35[2] + pv34[2];
p34[3]=pm35[3] + pv34[3];
p34[4]=pm35[4] + pv34[4];
p34[5]=pm35[5] + pv34[5];
p34[6]=pm35[6] + pv34[6];

pmm34[1]=state[10].thdd*h34[1] + p34[1] + c34[1]*JA34[1][1] + c34[2]*JA34[1][2] + c34[4]*JA34[1][4] + c34[5]*JA34[1][5];
pmm34[2]=state[10].thdd*h34[2] + p34[2] + c34[1]*JA34[2][1] + c34[2]*JA34[2][2] + c34[4]*JA34[2][4] + c34[5]*JA34[2][5];
pmm34[3]=state[10].thdd*h34[3] + p34[3] + c34[1]*JA34[3][1] + c34[2]*JA34[3][2] + c34[4]*JA34[3][4] + c34[5]*JA34[3][5];
pmm34[4]=state[10].thdd*h34[4] + p34[4] + c34[1]*JA34[4][1] + c34[2]*JA34[4][2] + c34[4]*JA34[4][4] + c34[5]*JA34[4][5];
pmm34[5]=state[10].thdd*h34[5] + p34[5] + c34[1]*JA34[5][1] + c34[2]*JA34[5][2] + c34[4]*JA34[5][4] + c34[5]*JA34[5][5];
pmm34[6]=state[10].thdd*h34[6] + p34[6] + c34[1]*JA34[6][1] + c34[2]*JA34[6][2] + c34[4]*JA34[6][4] + c34[5]*JA34[6][5];

pm34[1]=pmm34[1]*Si3334[1][1] + pmm34[2]*Si3334[1][2];
pm34[2]=-pmm34[3];
pm34[3]=pmm34[1]*Si3334[3][1] + pmm34[2]*Si3334[3][2];
pm34[4]=pmm34[4]*Si3334[1][1] + pmm34[5]*Si3334[1][2];
pm34[5]=-pmm34[6] + SHOULDERY*pmm34[1]*Si3334[3][1] + SHOULDERY*pmm34[2]*Si3334[3][2];
pm34[6]=SHOULDERY*pmm34[3] + pmm34[4]*Si3334[3][1] + pmm34[5]*Si3334[3][2];

p33[1]=pm34[1] + pv33[1];
p33[2]=pm34[2] + pv33[2];
p33[3]=pm34[3] + pv33[3];
p33[4]=pm34[4] + pv33[4];
p33[5]=pm34[5] + pv33[5];
p33[6]=pm34[6] + pv33[6];

pmm33[1]=state[9].thdd*h33[1] + p33[1] + c33[1]*JA33[1][1] + c33[2]*JA33[1][2] + c33[4]*JA33[1][4] + c33[5]*JA33[1][5];
pmm33[2]=state[9].thdd*h33[2] + p33[2] + c33[1]*JA33[2][1] + c33[2]*JA33[2][2] + c33[4]*JA33[2][4] + c33[5]*JA33[2][5];
pmm33[3]=state[9].thdd*h33[3] + p33[3] + c33[1]*JA33[3][1] + c33[2]*JA33[3][2] + c33[4]*JA33[3][4] + c33[5]*JA33[3][5];
pmm33[4]=state[9].thdd*h33[4] + p33[4] + c33[1]*JA33[4][1] + c33[2]*JA33[4][2] + c33[4]*JA33[4][4] + c33[5]*JA33[4][5];
pmm33[5]=state[9].thdd*h33[5] + p33[5] + c33[1]*JA33[5][1] + c33[2]*JA33[5][2] + c33[4]*JA33[5][4] + c33[5]*JA33[5][5];
pmm33[6]=state[9].thdd*h33[6] + p33[6] + c33[1]*JA33[6][1] + c33[2]*JA33[6][2] + c33[4]*JA33[6][4] + c33[5]*JA33[6][5];

pm33[1]=pmm33[1]*Si3233[1][1] + pmm33[2]*Si3233[1][2];
pm33[2]=pmm33[3];
pm33[3]=pmm33[1]*Si3233[3][1] + pmm33[2]*Si3233[3][2];
pm33[4]=-(SHOULDERX*pmm33[3]) + pmm33[4]*Si3233[1][1] + pmm33[5]*Si3233[1][2];
pm33[5]=pmm33[6] + SHOULDERX*pmm33[1]*Si3233[1][1] + SHOULDERX*pmm33[2]*Si3233[1][2];
pm33[6]=pmm33[4]*Si3233[3][1] + pmm33[5]*Si3233[3][2];

p32[1]=pm33[1] + pv32[1];
p32[2]=pm33[2] + pv32[2];
p32[3]=pm33[3] + pv32[3];
p32[4]=pm33[4] + pv32[4];
p32[5]=pm33[5] + pv32[5];
p32[6]=pm33[6] + pv32[6];

pmm32[1]=state[8].thdd*h32[1] + p32[1] + c32[1]*JA32[1][1] + c32[2]*JA32[1][2] + c32[4]*JA32[1][4] + c32[5]*JA32[1][5];
pmm32[2]=state[8].thdd*h32[2] + p32[2] + c32[1]*JA32[2][1] + c32[2]*JA32[2][2] + c32[4]*JA32[2][4] + c32[5]*JA32[2][5];
pmm32[3]=state[8].thdd*h32[3] + p32[3] + c32[1]*JA32[3][1] + c32[2]*JA32[3][2] + c32[4]*JA32[3][4] + c32[5]*JA32[3][5];
pmm32[4]=state[8].thdd*h32[4] + p32[4] + c32[1]*JA32[4][1] + c32[2]*JA32[4][2] + c32[4]*JA32[4][4] + c32[5]*JA32[4][5];
pmm32[5]=state[8].thdd*h32[5] + p32[5] + c32[1]*JA32[5][1] + c32[2]*JA32[5][2] + c32[4]*JA32[5][4] + c32[5]*JA32[5][5];
pmm32[6]=state[8].thdd*h32[6] + p32[6] + c32[1]*JA32[6][1] + c32[2]*JA32[6][2] + c32[4]*JA32[6][4] + c32[5]*JA32[6][5];

pm32[1]=-0.7071067811865475*pmm32[3] + pmm32[1]*Si332[1][1] + pmm32[2]*Si332[1][2];
pm32[2]=pmm32[1]*Si332[2][1] + pmm32[2]*Si332[2][2];
pm32[3]=-0.7071067811865475*pmm32[3] + pmm32[1]*Si332[3][1] + pmm32[2]*Si332[3][2];
pm32[4]=-0.7071067811865475*pmm32[6] + pmm32[4]*Si332[1][1] + pmm32[5]*Si332[1][2];
pm32[5]=-0.7071067811865475*THORAX2SHOULDER*pmm32[3] + pmm32[4]*Si332[2][1] + pmm32[5]*Si332[2][2] + THORAX2SHOULDER*pmm32[1]*Si332[3][1] + THORAX2SHOULDER*pmm32[2]*Si332[3][2];
pm32[6]=-0.7071067811865475*pmm32[6] - THORAX2SHOULDER*pmm32[1]*Si332[2][1] - THORAX2SHOULDER*pmm32[2]*Si332[2][2] + pmm32[4]*Si332[3][1] + pmm32[5]*Si332[3][2];

p28[1]=0. + pv28[1];
p28[2]=0. + pv28[2];
p28[3]=0. + pv28[3];
p28[4]=0. + pv28[4];
p28[5]=0. + pv28[5];
p28[6]=0. + pv28[6];

pmm28[1]=state[44].thdd*h28[1] + p28[1] + c28[2]*JA28[1][2] + c28[4]*JA28[1][4];
pmm28[2]=state[44].thdd*h28[2] + p28[2] + c28[1]*JA28[2][1] + c28[5]*JA28[2][5];
pmm28[3]=p28[3] + c28[1]*JA28[3][1] + c28[2]*JA28[3][2];
pmm28[4]=state[44].thdd*h28[4] + p28[4] + c28[1]*JA28[4][1] + c28[2]*JA28[4][2] + c28[5]*JA28[4][5];
pmm28[5]=state[44].thdd*h28[5] + p28[5] + c28[1]*JA28[5][1] + c28[2]*JA28[5][2] + c28[4]*JA28[5][4];
pmm28[6]=state[44].thdd*h28[6] + p28[6] + c28[1]*JA28[6][1] + c28[2]*JA28[6][2] + c28[4]*JA28[6][4] + c28[5]*JA28[6][5];

pm28[1]=pmm28[1]*Si1028[1][1] + pmm28[2]*Si1028[1][2];
pm28[2]=pmm28[1]*Si1028[2][1] + pmm28[2]*Si1028[2][2] + pmm28[3]*Si1028[2][3];
pm28[3]=pmm28[1]*Si1028[3][1] + pmm28[2]*Si1028[3][2] + pmm28[3]*Si1028[3][3];
pm28[4]=pmm28[4]*Si1028[1][1] + pmm28[5]*Si1028[1][2] + pmm28[1]*(ZLF*Si1028[2][1] + YLF*Si1028[3][1]) + pmm28[2]*(ZLF*Si1028[2][2] + YLF*Si1028[3][2]) + pmm28[3]*(ZLF*Si1028[2][3] + YLF*Si1028[3][3]);
pm28[5]=pmm28[4]*Si1028[2][1] + pmm28[5]*Si1028[2][2] + pmm28[6]*Si1028[2][3] + pmm28[1]*(-(ZLF*Si1028[1][1]) - XLF*Si1028[3][1]) + pmm28[2]*(-(ZLF*Si1028[1][2]) - XLF*Si1028[3][2]) - XLF*pmm28[3]*Si1028[3][3];
pm28[6]=pmm28[1]*(-(YLF*Si1028[1][1]) + XLF*Si1028[2][1]) + pmm28[2]*(-(YLF*Si1028[1][2]) + XLF*Si1028[2][2]) + XLF*pmm28[3]*Si1028[2][3] + pmm28[4]*Si1028[3][1] + pmm28[5]*Si1028[3][2] + pmm28[6]*Si1028[3][3];

p24[1]=0. + pv24[1];
p24[2]=0. + pv24[2];
p24[3]=0. + pv24[3];
p24[4]=0. + pv24[4];
p24[5]=0. + pv24[5];
p24[6]=0. + pv24[6];

pmm24[1]=state[43].thdd*h24[1] + p24[1] + c24[2]*JA24[1][2] + c24[4]*JA24[1][4];
pmm24[2]=state[43].thdd*h24[2] + p24[2] + c24[1]*JA24[2][1] + c24[5]*JA24[2][5];
pmm24[3]=p24[3] + c24[1]*JA24[3][1] + c24[2]*JA24[3][2];
pmm24[4]=state[43].thdd*h24[4] + p24[4] + c24[1]*JA24[4][1] + c24[2]*JA24[4][2] + c24[5]*JA24[4][5];
pmm24[5]=state[43].thdd*h24[5] + p24[5] + c24[1]*JA24[5][1] + c24[2]*JA24[5][2] + c24[4]*JA24[5][4];
pmm24[6]=state[43].thdd*h24[6] + p24[6] + c24[1]*JA24[6][1] + c24[2]*JA24[6][2] + c24[4]*JA24[6][4] + c24[5]*JA24[6][5];

pm24[1]=pmm24[1]*Si1024[1][1] + pmm24[2]*Si1024[1][2];
pm24[2]=pmm24[1]*Si1024[2][1] + pmm24[2]*Si1024[2][2] + pmm24[3]*Si1024[2][3];
pm24[3]=pmm24[1]*Si1024[3][1] + pmm24[2]*Si1024[3][2] + pmm24[3]*Si1024[3][3];
pm24[4]=pmm24[4]*Si1024[1][1] + pmm24[5]*Si1024[1][2] + pmm24[1]*(ZRF*Si1024[2][1] + YRF*Si1024[3][1]) + pmm24[2]*(ZRF*Si1024[2][2] + YRF*Si1024[3][2]) + pmm24[3]*(ZRF*Si1024[2][3] + YRF*Si1024[3][3]);
pm24[5]=pmm24[4]*Si1024[2][1] + pmm24[5]*Si1024[2][2] + pmm24[6]*Si1024[2][3] + pmm24[1]*(-(ZRF*Si1024[1][1]) - XRF*Si1024[3][1]) + pmm24[2]*(-(ZRF*Si1024[1][2]) - XRF*Si1024[3][2]) - XRF*pmm24[3]*Si1024[3][3];
pm24[6]=pmm24[1]*(-(YRF*Si1024[1][1]) + XRF*Si1024[2][1]) + pmm24[2]*(-(YRF*Si1024[1][2]) + XRF*Si1024[2][2]) + XRF*pmm24[3]*Si1024[2][3] + pmm24[4]*Si1024[3][1] + pmm24[5]*Si1024[3][2] + pmm24[6]*Si1024[3][3];

p20[1]=0. + pv20[1];
p20[2]=0. + pv20[2];
p20[3]=0. + pv20[3];
p20[4]=0. + pv20[4];
p20[5]=0. + pv20[5];
p20[6]=0. + pv20[6];

pmm20[1]=state[42].thdd*h20[1] + p20[1] + c20[2]*JA20[1][2] + c20[4]*JA20[1][4];
pmm20[2]=state[42].thdd*h20[2] + p20[2] + c20[1]*JA20[2][1] + c20[5]*JA20[2][5];
pmm20[3]=p20[3] + c20[1]*JA20[3][1] + c20[2]*JA20[3][2];
pmm20[4]=state[42].thdd*h20[4] + p20[4] + c20[1]*JA20[4][1] + c20[2]*JA20[4][2] + c20[5]*JA20[4][5];
pmm20[5]=state[42].thdd*h20[5] + p20[5] + c20[1]*JA20[5][1] + c20[2]*JA20[5][2] + c20[4]*JA20[5][4];
pmm20[6]=state[42].thdd*h20[6] + p20[6] + c20[1]*JA20[6][1] + c20[2]*JA20[6][2] + c20[4]*JA20[6][4] + c20[5]*JA20[6][5];

pm20[1]=pmm20[1]*Si1020[1][1] + pmm20[2]*Si1020[1][2];
pm20[2]=pmm20[1]*Si1020[2][1] + pmm20[2]*Si1020[2][2] + pmm20[3]*Si1020[2][3];
pm20[3]=pmm20[1]*Si1020[3][1] + pmm20[2]*Si1020[3][2] + pmm20[3]*Si1020[3][3];
pm20[4]=pmm20[4]*Si1020[1][1] + pmm20[5]*Si1020[1][2] + pmm20[1]*(ZMF*Si1020[2][1] + YMF*Si1020[3][1]) + pmm20[2]*(ZMF*Si1020[2][2] + YMF*Si1020[3][2]) + pmm20[3]*(ZMF*Si1020[2][3] + YMF*Si1020[3][3]);
pm20[5]=pmm20[4]*Si1020[2][1] + pmm20[5]*Si1020[2][2] + pmm20[6]*Si1020[2][3] + pmm20[1]*(-(ZMF*Si1020[1][1]) - XMF*Si1020[3][1]) + pmm20[2]*(-(ZMF*Si1020[1][2]) - XMF*Si1020[3][2]) - XMF*pmm20[3]*Si1020[3][3];
pm20[6]=pmm20[1]*(-(YMF*Si1020[1][1]) + XMF*Si1020[2][1]) + pmm20[2]*(-(YMF*Si1020[1][2]) + XMF*Si1020[2][2]) + XMF*pmm20[3]*Si1020[2][3] + pmm20[4]*Si1020[3][1] + pmm20[5]*Si1020[3][2] + pmm20[6]*Si1020[3][3];

p16[1]=0. + pv16[1];
p16[2]=0. + pv16[2];
p16[3]=0. + pv16[3];
p16[4]=0. + pv16[4];
p16[5]=0. + pv16[5];
p16[6]=0. + pv16[6];

pmm16[1]=state[41].thdd*h16[1] + p16[1] + c16[2]*JA16[1][2] + c16[4]*JA16[1][4];
pmm16[2]=state[41].thdd*h16[2] + p16[2] + c16[1]*JA16[2][1] + c16[5]*JA16[2][5];
pmm16[3]=p16[3] + c16[1]*JA16[3][1] + c16[2]*JA16[3][2];
pmm16[4]=state[41].thdd*h16[4] + p16[4] + c16[1]*JA16[4][1] + c16[2]*JA16[4][2] + c16[5]*JA16[4][5];
pmm16[5]=state[41].thdd*h16[5] + p16[5] + c16[1]*JA16[5][1] + c16[2]*JA16[5][2] + c16[4]*JA16[5][4];
pmm16[6]=state[41].thdd*h16[6] + p16[6] + c16[1]*JA16[6][1] + c16[2]*JA16[6][2] + c16[4]*JA16[6][4] + c16[5]*JA16[6][5];

pm16[1]=pmm16[1]*Si1016[1][1] + pmm16[2]*Si1016[1][2];
pm16[2]=pmm16[1]*Si1016[2][1] + pmm16[2]*Si1016[2][2] + pmm16[3]*Si1016[2][3];
pm16[3]=pmm16[1]*Si1016[3][1] + pmm16[2]*Si1016[3][2] + pmm16[3]*Si1016[3][3];
pm16[4]=pmm16[4]*Si1016[1][1] + pmm16[5]*Si1016[1][2] + pmm16[1]*(ZIF*Si1016[2][1] + YIF*Si1016[3][1]) + pmm16[2]*(ZIF*Si1016[2][2] + YIF*Si1016[3][2]) + pmm16[3]*(ZIF*Si1016[2][3] + YIF*Si1016[3][3]);
pm16[5]=pmm16[4]*Si1016[2][1] + pmm16[5]*Si1016[2][2] + pmm16[6]*Si1016[2][3] + pmm16[1]*(-(ZIF*Si1016[1][1]) - XIF*Si1016[3][1]) + pmm16[2]*(-(ZIF*Si1016[1][2]) - XIF*Si1016[3][2]) - XIF*pmm16[3]*Si1016[3][3];
pm16[6]=pmm16[1]*(-(YIF*Si1016[1][1]) + XIF*Si1016[2][1]) + pmm16[2]*(-(YIF*Si1016[1][2]) + XIF*Si1016[2][2]) + XIF*pmm16[3]*Si1016[2][3] + pmm16[4]*Si1016[3][1] + pmm16[5]*Si1016[3][2] + pmm16[6]*Si1016[3][3];

p13[1]=0. + pv13[1];
p13[2]=0. + pv13[2];
p13[3]=0. + pv13[3];
p13[4]=0. + pv13[4];
p13[5]=0. + pv13[5];
p13[6]=0. + pv13[6];

pmm13[1]=state[40].thdd*h13[1] + p13[1] + c13[2]*JA13[1][2] + c13[4]*JA13[1][4];
pmm13[2]=state[40].thdd*h13[2] + p13[2] + c13[1]*JA13[2][1] + c13[5]*JA13[2][5];
pmm13[3]=p13[3] + c13[1]*JA13[3][1] + c13[2]*JA13[3][2];
pmm13[4]=state[40].thdd*h13[4] + p13[4] + c13[1]*JA13[4][1] + c13[2]*JA13[4][2] + c13[5]*JA13[4][5];
pmm13[5]=state[40].thdd*h13[5] + p13[5] + c13[1]*JA13[5][1] + c13[2]*JA13[5][2] + c13[4]*JA13[5][4];
pmm13[6]=state[40].thdd*h13[6] + p13[6] + c13[1]*JA13[6][1] + c13[2]*JA13[6][2] + c13[4]*JA13[6][4] + c13[5]*JA13[6][5];

pm13[1]=pmm13[1]*Si1213[1][1] + pmm13[2]*Si1213[1][2];
pm13[2]=pmm13[1]*Si1213[2][1] + pmm13[2]*Si1213[2][2];
pm13[3]=pmm13[3];
pm13[4]=YTHUMBFLEX*pmm13[3] + pmm13[4]*Si1213[1][1] + pmm13[5]*Si1213[1][2];
pm13[5]=-(XTHUMBFLEX*pmm13[3]) + pmm13[4]*Si1213[2][1] + pmm13[5]*Si1213[2][2];
pm13[6]=pmm13[6] + pmm13[1]*(-(YTHUMBFLEX*Si1213[1][1]) + XTHUMBFLEX*Si1213[2][1]) + pmm13[2]*(-(YTHUMBFLEX*Si1213[1][2]) + XTHUMBFLEX*Si1213[2][2]);

p12[1]=pm13[1] + pv12[1];
p12[2]=pm13[2] + pv12[2];
p12[3]=pm13[3] + pv12[3];
p12[4]=pm13[4] + pv12[4];
p12[5]=pm13[5] + pv12[5];
p12[6]=pm13[6] + pv12[6];

pmm12[1]=state[39].thdd*h12[1] + p12[1] + c12[2]*JA12[1][2] + c12[3]*JA12[1][3] + c12[5]*JA12[1][5];
pmm12[2]=state[39].thdd*h12[2] + p12[2] + c12[2]*JA12[2][2] + c12[3]*JA12[2][3] + c12[5]*JA12[2][5];
pmm12[3]=state[39].thdd*h12[3] + p12[3] + c12[2]*JA12[3][2] + c12[6]*JA12[3][6];
pmm12[4]=state[39].thdd*h12[4] + p12[4] + c12[2]*JA12[4][2] + c12[3]*JA12[4][3] + c12[5]*JA12[4][5] + c12[6]*JA12[4][6];
pmm12[5]=state[39].thdd*h12[5] + p12[5] + c12[2]*JA12[5][2] + c12[3]*JA12[5][3] + c12[5]*JA12[5][5] + c12[6]*JA12[5][6];
pmm12[6]=state[39].thdd*h12[6] + p12[6] + c12[2]*JA12[6][2] + c12[3]*JA12[6][3] + c12[5]*JA12[6][5];

pm12[1]=pmm12[1]*Si1012[1][1] + pmm12[2]*Si1012[1][2] + pmm12[3]*Si1012[1][3];
pm12[2]=pmm12[1]*Si1012[2][1] + pmm12[2]*Si1012[2][2] + pmm12[3]*Si1012[2][3];
pm12[3]=pmm12[1]*Si1012[3][1] + pmm12[2]*Si1012[3][2] + pmm12[3]*Si1012[3][3];
pm12[4]=pmm12[4]*Si1012[1][1] + pmm12[5]*Si1012[1][2] + pmm12[6]*Si1012[1][3] + pmm12[1]*(ZTHUMB*Si1012[2][1] + YTHUMB*Si1012[3][1]) + pmm12[2]*(ZTHUMB*Si1012[2][2] + YTHUMB*Si1012[3][2]) + pmm12[3]*(ZTHUMB*Si1012[2][3] + YTHUMB*Si1012[3][3]);
pm12[5]=pmm12[4]*Si1012[2][1] + pmm12[5]*Si1012[2][2] + pmm12[6]*Si1012[2][3] + pmm12[1]*(-(ZTHUMB*Si1012[1][1]) - XTHUMB*Si1012[3][1]) + pmm12[2]*(-(ZTHUMB*Si1012[1][2]) - XTHUMB*Si1012[3][2]) + pmm12[3]*(-(ZTHUMB*Si1012[1][3]) - XTHUMB*Si1012[3][3]);
pm12[6]=pmm12[1]*(-(YTHUMB*Si1012[1][1]) + XTHUMB*Si1012[2][1]) + pmm12[2]*(-(YTHUMB*Si1012[1][2]) + XTHUMB*Si1012[2][2]) + pmm12[3]*(-(YTHUMB*Si1012[1][3]) + XTHUMB*Si1012[2][3]) + pmm12[4]*Si1012[3][1] + pmm12[5]*Si1012[3][2] + pmm12[6]*Si1012[3][3];

p11[1]=pv11[1];
p11[2]=pv11[2];
p11[3]=pv11[3];
p11[4]=pv11[4];
p11[5]=pv11[5];
p11[6]=pv11[6];

pmm11[1]=p11[1];
pmm11[2]=p11[2];
pmm11[3]=p11[3];
pmm11[4]=p11[4];
pmm11[5]=p11[5];
pmm11[6]=p11[6];

pm11[1]=pmm11[1]*Si1011[1][1] + pmm11[2]*Si1011[1][2] + pmm11[3]*Si1011[1][3];
pm11[2]=pmm11[1]*Si1011[2][1] + pmm11[2]*Si1011[2][2] + pmm11[3]*Si1011[2][3];
pm11[3]=pmm11[1]*Si1011[3][1] + pmm11[2]*Si1011[3][2] + pmm11[3]*Si1011[3][3];
pm11[4]=pmm11[4]*Si1011[1][1] + pmm11[5]*Si1011[1][2] + pmm11[6]*Si1011[1][3] + pmm11[1]*(-(eff[2].x[3]*Si1011[2][1]) + eff[2].x[2]*Si1011[3][1]) + pmm11[2]*(-(eff[2].x[3]*Si1011[2][2]) + eff[2].x[2]*Si1011[3][2]) + pmm11[3]*(-(eff[2].x[3]*Si1011[2][3]) + eff[2].x[2]*Si1011[3][3]);
pm11[5]=pmm11[4]*Si1011[2][1] + pmm11[5]*Si1011[2][2] + pmm11[6]*Si1011[2][3] + pmm11[1]*(eff[2].x[3]*Si1011[1][1] - eff[2].x[1]*Si1011[3][1]) + pmm11[2]*(eff[2].x[3]*Si1011[1][2] - eff[2].x[1]*Si1011[3][2]) + pmm11[3]*(eff[2].x[3]*Si1011[1][3] - eff[2].x[1]*Si1011[3][3]);
pm11[6]=pmm11[1]*(-(eff[2].x[2]*Si1011[1][1]) + eff[2].x[1]*Si1011[2][1]) + pmm11[2]*(-(eff[2].x[2]*Si1011[1][2]) + eff[2].x[1]*Si1011[2][2]) + pmm11[3]*(-(eff[2].x[2]*Si1011[1][3]) + eff[2].x[1]*Si1011[2][3]) + pmm11[4]*Si1011[3][1] + pmm11[5]*Si1011[3][2] + pmm11[6]*Si1011[3][3];

p10[1]=pm11[1] + pm12[1] + pm16[1] + pm20[1] + pm24[1] + pm28[1] + pv10[1];
p10[2]=pm11[2] + pm12[2] + pm16[2] + pm20[2] + pm24[2] + pm28[2] + pv10[2];
p10[3]=pm11[3] + pm12[3] + pm16[3] + pm20[3] + pm24[3] + pm28[3] + pv10[3];
p10[4]=pm11[4] + pm12[4] + pm16[4] + pm20[4] + pm24[4] + pm28[4] + pv10[4];
p10[5]=pm11[5] + pm12[5] + pm16[5] + pm20[5] + pm24[5] + pm28[5] + pv10[5];
p10[6]=pm11[6] + pm12[6] + pm16[6] + pm20[6] + pm24[6] + pm28[6] + pv10[6];

pmm10[1]=state[7].thdd*h10[1] + p10[1] + c10[1]*JA10[1][1] + c10[2]*JA10[1][2] + c10[4]*JA10[1][4] + c10[5]*JA10[1][5];
pmm10[2]=state[7].thdd*h10[2] + p10[2] + c10[1]*JA10[2][1] + c10[2]*JA10[2][2] + c10[4]*JA10[2][4] + c10[5]*JA10[2][5];
pmm10[3]=state[7].thdd*h10[3] + p10[3] + c10[1]*JA10[3][1] + c10[2]*JA10[3][2] + c10[4]*JA10[3][4] + c10[5]*JA10[3][5];
pmm10[4]=state[7].thdd*h10[4] + p10[4] + c10[1]*JA10[4][1] + c10[2]*JA10[4][2] + c10[4]*JA10[4][4] + c10[5]*JA10[4][5];
pmm10[5]=state[7].thdd*h10[5] + p10[5] + c10[1]*JA10[5][1] + c10[2]*JA10[5][2] + c10[4]*JA10[5][4] + c10[5]*JA10[5][5];
pmm10[6]=state[7].thdd*h10[6] + p10[6] + c10[1]*JA10[6][1] + c10[2]*JA10[6][2] + c10[4]*JA10[6][4] + c10[5]*JA10[6][5];

pm10[1]=pmm10[1]*Si910[1][1] + pmm10[2]*Si910[1][2];
pm10[2]=-pmm10[3];
pm10[3]=pmm10[1]*Si910[3][1] + pmm10[2]*Si910[3][2];
pm10[4]=pmm10[4]*Si910[1][1] + pmm10[5]*Si910[1][2];
pm10[5]=-pmm10[6];
pm10[6]=pmm10[4]*Si910[3][1] + pmm10[5]*Si910[3][2];

p9[1]=pm10[1] + pv9[1];
p9[2]=pm10[2] + pv9[2];
p9[3]=pm10[3] + pv9[3];
p9[4]=pm10[4] + pv9[4];
p9[5]=pm10[5] + pv9[5];
p9[6]=pm10[6] + pv9[6];

pmm9[1]=state[6].thdd*h9[1] + p9[1] + c9[1]*JA9[1][1] + c9[2]*JA9[1][2] + c9[4]*JA9[1][4] + c9[5]*JA9[1][5];
pmm9[2]=state[6].thdd*h9[2] + p9[2] + c9[1]*JA9[2][1] + c9[2]*JA9[2][2] + c9[4]*JA9[2][4] + c9[5]*JA9[2][5];
pmm9[3]=state[6].thdd*h9[3] + p9[3] + c9[1]*JA9[3][1] + c9[2]*JA9[3][2] + c9[4]*JA9[3][4] + c9[5]*JA9[3][5];
pmm9[4]=state[6].thdd*h9[4] + p9[4] + c9[1]*JA9[4][1] + c9[2]*JA9[4][2] + c9[4]*JA9[4][4] + c9[5]*JA9[4][5];
pmm9[5]=state[6].thdd*h9[5] + p9[5] + c9[1]*JA9[5][1] + c9[2]*JA9[5][2] + c9[4]*JA9[5][4] + c9[5]*JA9[5][5];
pmm9[6]=state[6].thdd*h9[6] + p9[6] + c9[1]*JA9[6][1] + c9[2]*JA9[6][2] + c9[4]*JA9[6][4] + c9[5]*JA9[6][5];

pm9[1]=pmm9[3];
pm9[2]=pmm9[1]*Si89[2][1] + pmm9[2]*Si89[2][2];
pm9[3]=pmm9[1]*Si89[3][1] + pmm9[2]*Si89[3][2];
pm9[4]=pmm9[6] + pmm9[1]*(LOWERARM*Si89[2][1] + WRISTY*Si89[3][1]) + pmm9[2]*(LOWERARM*Si89[2][2] + WRISTY*Si89[3][2]);
pm9[5]=-(LOWERARM*pmm9[3]) + pmm9[4]*Si89[2][1] + pmm9[5]*Si89[2][2];
pm9[6]=-(WRISTY*pmm9[3]) + pmm9[4]*Si89[3][1] + pmm9[5]*Si89[3][2];

p8[1]=pm9[1] + pv8[1];
p8[2]=pm9[2] + pv8[2];
p8[3]=pm9[3] + pv8[3];
p8[4]=pm9[4] + pv8[4];
p8[5]=pm9[5] + pv8[5];
p8[6]=pm9[6] + pv8[6];

pmm8[1]=state[5].thdd*h8[1] + p8[1] + c8[1]*JA8[1][1] + c8[2]*JA8[1][2] + c8[4]*JA8[1][4] + c8[5]*JA8[1][5];
pmm8[2]=state[5].thdd*h8[2] + p8[2] + c8[1]*JA8[2][1] + c8[2]*JA8[2][2] + c8[4]*JA8[2][4] + c8[5]*JA8[2][5];
pmm8[3]=state[5].thdd*h8[3] + p8[3] + c8[1]*JA8[3][1] + c8[2]*JA8[3][2] + c8[4]*JA8[3][4] + c8[5]*JA8[3][5];
pmm8[4]=state[5].thdd*h8[4] + p8[4] + c8[1]*JA8[4][1] + c8[2]*JA8[4][2] + c8[4]*JA8[4][4] + c8[5]*JA8[4][5];
pmm8[5]=state[5].thdd*h8[5] + p8[5] + c8[1]*JA8[5][1] + c8[2]*JA8[5][2] + c8[4]*JA8[5][4] + c8[5]*JA8[5][5];
pmm8[6]=state[5].thdd*h8[6] + p8[6] + c8[1]*JA8[6][1] + c8[2]*JA8[6][2] + c8[4]*JA8[6][4] + c8[5]*JA8[6][5];

pm8[1]=pmm8[1]*Si78[1][1] + pmm8[2]*Si78[1][2];
pm8[2]=pmm8[3];
pm8[3]=pmm8[1]*Si78[3][1] + pmm8[2]*Si78[3][2];
pm8[4]=pmm8[4]*Si78[1][1] + pmm8[5]*Si78[1][2];
pm8[5]=pmm8[6];
pm8[6]=pmm8[4]*Si78[3][1] + pmm8[5]*Si78[3][2];

p7[1]=pm8[1] + pv7[1];
p7[2]=pm8[2] + pv7[2];
p7[3]=pm8[3] + pv7[3];
p7[4]=pm8[4] + pv7[4];
p7[5]=pm8[5] + pv7[5];
p7[6]=pm8[6] + pv7[6];

pmm7[1]=state[4].thdd*h7[1] + p7[1] + c7[1]*JA7[1][1] + c7[2]*JA7[1][2] + c7[4]*JA7[1][4] + c7[5]*JA7[1][5];
pmm7[2]=state[4].thdd*h7[2] + p7[2] + c7[1]*JA7[2][1] + c7[2]*JA7[2][2] + c7[4]*JA7[2][4] + c7[5]*JA7[2][5];
pmm7[3]=state[4].thdd*h7[3] + p7[3] + c7[1]*JA7[3][1] + c7[2]*JA7[3][2] + c7[4]*JA7[3][4] + c7[5]*JA7[3][5];
pmm7[4]=state[4].thdd*h7[4] + p7[4] + c7[1]*JA7[4][1] + c7[2]*JA7[4][2] + c7[4]*JA7[4][4] + c7[5]*JA7[4][5];
pmm7[5]=state[4].thdd*h7[5] + p7[5] + c7[1]*JA7[5][1] + c7[2]*JA7[5][2] + c7[4]*JA7[5][4] + c7[5]*JA7[5][5];
pmm7[6]=state[4].thdd*h7[6] + p7[6] + c7[1]*JA7[6][1] + c7[2]*JA7[6][2] + c7[4]*JA7[6][4] + c7[5]*JA7[6][5];

pm7[1]=pmm7[3];
pm7[2]=pmm7[1]*Si67[2][1] + pmm7[2]*Si67[2][2];
pm7[3]=pmm7[1]*Si67[3][1] + pmm7[2]*Si67[3][2];
pm7[4]=pmm7[6] + UPPERARM*pmm7[1]*Si67[2][1] + UPPERARM*pmm7[2]*Si67[2][2];
pm7[5]=-(UPPERARM*pmm7[3]) + pmm7[4]*Si67[2][1] + pmm7[5]*Si67[2][2];
pm7[6]=pmm7[4]*Si67[3][1] + pmm7[5]*Si67[3][2];

p6[1]=pm7[1] + pv6[1];
p6[2]=pm7[2] + pv6[2];
p6[3]=pm7[3] + pv6[3];
p6[4]=pm7[4] + pv6[4];
p6[5]=pm7[5] + pv6[5];
p6[6]=pm7[6] + pv6[6];

pmm6[1]=state[3].thdd*h6[1] + p6[1] + c6[1]*JA6[1][1] + c6[2]*JA6[1][2] + c6[4]*JA6[1][4] + c6[5]*JA6[1][5];
pmm6[2]=state[3].thdd*h6[2] + p6[2] + c6[1]*JA6[2][1] + c6[2]*JA6[2][2] + c6[4]*JA6[2][4] + c6[5]*JA6[2][5];
pmm6[3]=state[3].thdd*h6[3] + p6[3] + c6[1]*JA6[3][1] + c6[2]*JA6[3][2] + c6[4]*JA6[3][4] + c6[5]*JA6[3][5];
pmm6[4]=state[3].thdd*h6[4] + p6[4] + c6[1]*JA6[4][1] + c6[2]*JA6[4][2] + c6[4]*JA6[4][4] + c6[5]*JA6[4][5];
pmm6[5]=state[3].thdd*h6[5] + p6[5] + c6[1]*JA6[5][1] + c6[2]*JA6[5][2] + c6[4]*JA6[5][4] + c6[5]*JA6[5][5];
pmm6[6]=state[3].thdd*h6[6] + p6[6] + c6[1]*JA6[6][1] + c6[2]*JA6[6][2] + c6[4]*JA6[6][4] + c6[5]*JA6[6][5];

pm6[1]=pmm6[1]*Si56[1][1] + pmm6[2]*Si56[1][2];
pm6[2]=pmm6[3];
pm6[3]=pmm6[1]*Si56[3][1] + pmm6[2]*Si56[3][2];
pm6[4]=pmm6[4]*Si56[1][1] + pmm6[5]*Si56[1][2];
pm6[5]=pmm6[6] + SHOULDERY*pmm6[1]*Si56[3][1] + SHOULDERY*pmm6[2]*Si56[3][2];
pm6[6]=-(SHOULDERY*pmm6[3]) + pmm6[4]*Si56[3][1] + pmm6[5]*Si56[3][2];

p5[1]=pm6[1] + pv5[1];
p5[2]=pm6[2] + pv5[2];
p5[3]=pm6[3] + pv5[3];
p5[4]=pm6[4] + pv5[4];
p5[5]=pm6[5] + pv5[5];
p5[6]=pm6[6] + pv5[6];

pmm5[1]=state[2].thdd*h5[1] + p5[1] + c5[1]*JA5[1][1] + c5[2]*JA5[1][2] + c5[4]*JA5[1][4] + c5[5]*JA5[1][5];
pmm5[2]=state[2].thdd*h5[2] + p5[2] + c5[1]*JA5[2][1] + c5[2]*JA5[2][2] + c5[4]*JA5[2][4] + c5[5]*JA5[2][5];
pmm5[3]=state[2].thdd*h5[3] + p5[3] + c5[1]*JA5[3][1] + c5[2]*JA5[3][2] + c5[4]*JA5[3][4] + c5[5]*JA5[3][5];
pmm5[4]=state[2].thdd*h5[4] + p5[4] + c5[1]*JA5[4][1] + c5[2]*JA5[4][2] + c5[4]*JA5[4][4] + c5[5]*JA5[4][5];
pmm5[5]=state[2].thdd*h5[5] + p5[5] + c5[1]*JA5[5][1] + c5[2]*JA5[5][2] + c5[4]*JA5[5][4] + c5[5]*JA5[5][5];
pmm5[6]=state[2].thdd*h5[6] + p5[6] + c5[1]*JA5[6][1] + c5[2]*JA5[6][2] + c5[4]*JA5[6][4] + c5[5]*JA5[6][5];

pm5[1]=pmm5[1]*Si45[1][1] + pmm5[2]*Si45[1][2];
pm5[2]=-pmm5[3];
pm5[3]=pmm5[1]*Si45[3][1] + pmm5[2]*Si45[3][2];
pm5[4]=-(SHOULDERX*pmm5[3]) + pmm5[4]*Si45[1][1] + pmm5[5]*Si45[1][2];
pm5[5]=-pmm5[6] - SHOULDERX*pmm5[1]*Si45[1][1] - SHOULDERX*pmm5[2]*Si45[1][2];
pm5[6]=pmm5[4]*Si45[3][1] + pmm5[5]*Si45[3][2];

p4[1]=pm5[1] + pv4[1];
p4[2]=pm5[2] + pv4[2];
p4[3]=pm5[3] + pv4[3];
p4[4]=pm5[4] + pv4[4];
p4[5]=pm5[5] + pv4[5];
p4[6]=pm5[6] + pv4[6];

pmm4[1]=state[1].thdd*h4[1] + p4[1] + c4[1]*JA4[1][1] + c4[2]*JA4[1][2] + c4[4]*JA4[1][4] + c4[5]*JA4[1][5];
pmm4[2]=state[1].thdd*h4[2] + p4[2] + c4[1]*JA4[2][1] + c4[2]*JA4[2][2] + c4[4]*JA4[2][4] + c4[5]*JA4[2][5];
pmm4[3]=state[1].thdd*h4[3] + p4[3] + c4[1]*JA4[3][1] + c4[2]*JA4[3][2] + c4[4]*JA4[3][4] + c4[5]*JA4[3][5];
pmm4[4]=state[1].thdd*h4[4] + p4[4] + c4[1]*JA4[4][1] + c4[2]*JA4[4][2] + c4[4]*JA4[4][4] + c4[5]*JA4[4][5];
pmm4[5]=state[1].thdd*h4[5] + p4[5] + c4[1]*JA4[5][1] + c4[2]*JA4[5][2] + c4[4]*JA4[5][4] + c4[5]*JA4[5][5];
pmm4[6]=state[1].thdd*h4[6] + p4[6] + c4[1]*JA4[6][1] + c4[2]*JA4[6][2] + c4[4]*JA4[6][4] + c4[5]*JA4[6][5];

pm4[1]=0.7071067811865475*pmm4[3] + pmm4[1]*Si34[1][1] + pmm4[2]*Si34[1][2];
pm4[2]=pmm4[1]*Si34[2][1] + pmm4[2]*Si34[2][2];
pm4[3]=-0.7071067811865475*pmm4[3] + pmm4[1]*Si34[3][1] + pmm4[2]*Si34[3][2];
pm4[4]=0.7071067811865475*pmm4[6] + pmm4[4]*Si34[1][1] + pmm4[5]*Si34[1][2];
pm4[5]=-0.7071067811865475*THORAX2SHOULDER*pmm4[3] + pmm4[4]*Si34[2][1] + pmm4[5]*Si34[2][2] + THORAX2SHOULDER*pmm4[1]*Si34[3][1] + THORAX2SHOULDER*pmm4[2]*Si34[3][2];
pm4[6]=-0.7071067811865475*pmm4[6] - THORAX2SHOULDER*pmm4[1]*Si34[2][1] - THORAX2SHOULDER*pmm4[2]*Si34[2][2] + pmm4[4]*Si34[3][1] + pmm4[5]*Si34[3][2];

p3[1]=pm32[1] + pm4[1] + pm60[1] + pv3[1];
p3[2]=pm32[2] + pm4[2] + pm60[2] + pv3[2];
p3[3]=pm32[3] + pm4[3] + pm60[3] + pv3[3];
p3[4]=pm32[4] + pm4[4] + pm60[4] + pv3[4];
p3[5]=pm32[5] + pm4[5] + pm60[5] + pv3[5];
p3[6]=pm32[6] + pm4[6] + pm60[6] + pv3[6];

pmm3[1]=state[31].thdd*h3[1] + p3[1] + c3[1]*JA3[1][1] + c3[2]*JA3[1][2] + c3[4]*JA3[1][4] + c3[5]*JA3[1][5];
pmm3[2]=state[31].thdd*h3[2] + p3[2] + c3[1]*JA3[2][1] + c3[2]*JA3[2][2] + c3[4]*JA3[2][4] + c3[5]*JA3[2][5];
pmm3[3]=state[31].thdd*h3[3] + p3[3] + c3[1]*JA3[3][1] + c3[2]*JA3[3][2] + c3[4]*JA3[3][4] + c3[5]*JA3[3][5];
pmm3[4]=state[31].thdd*h3[4] + p3[4] + c3[1]*JA3[4][1] + c3[2]*JA3[4][2] + c3[4]*JA3[4][4] + c3[5]*JA3[4][5];
pmm3[5]=state[31].thdd*h3[5] + p3[5] + c3[1]*JA3[5][1] + c3[2]*JA3[5][2] + c3[4]*JA3[5][4] + c3[5]*JA3[5][5];
pmm3[6]=state[31].thdd*h3[6] + p3[6] + c3[1]*JA3[6][1] + c3[2]*JA3[6][2] + c3[4]*JA3[6][4] + c3[5]*JA3[6][5];

pm3[1]=pmm3[1]*Si23[1][1] + pmm3[2]*Si23[1][2];
pm3[2]=pmm3[3];
pm3[3]=pmm3[1]*Si23[3][1] + pmm3[2]*Si23[3][2];
pm3[4]=pmm3[4]*Si23[1][1] + pmm3[5]*Si23[1][2];
pm3[5]=pmm3[6];
pm3[6]=pmm3[4]*Si23[3][1] + pmm3[5]*Si23[3][2];

p2[1]=pm3[1] + pv2[1];
p2[2]=pm3[2] + pv2[2];
p2[3]=pm3[3] + pv2[3];
p2[4]=pm3[4] + pv2[4];
p2[5]=pm3[5] + pv2[5];
p2[6]=pm3[6] + pv2[6];

pmm2[1]=state[30].thdd*h2[1] + p2[1] + c2[1]*JA2[1][1] + c2[2]*JA2[1][2] + c2[4]*JA2[1][4] + c2[5]*JA2[1][5];
pmm2[2]=state[30].thdd*h2[2] + p2[2] + c2[1]*JA2[2][1] + c2[2]*JA2[2][2] + c2[4]*JA2[2][4] + c2[5]*JA2[2][5];
pmm2[3]=state[30].thdd*h2[3] + p2[3] + c2[1]*JA2[3][1] + c2[2]*JA2[3][2] + c2[4]*JA2[3][4] + c2[5]*JA2[3][5];
pmm2[4]=state[30].thdd*h2[4] + p2[4] + c2[1]*JA2[4][1] + c2[2]*JA2[4][2] + c2[4]*JA2[4][4] + c2[5]*JA2[4][5];
pmm2[5]=state[30].thdd*h2[5] + p2[5] + c2[1]*JA2[5][1] + c2[2]*JA2[5][2] + c2[4]*JA2[5][4] + c2[5]*JA2[5][5];
pmm2[6]=state[30].thdd*h2[6] + p2[6] + c2[1]*JA2[6][1] + c2[2]*JA2[6][2] + c2[4]*JA2[6][4] + c2[5]*JA2[6][5];

pm2[1]=pmm2[1]*Si12[1][1] + pmm2[2]*Si12[1][2];
pm2[2]=-pmm2[3];
pm2[3]=pmm2[1]*Si12[3][1] + pmm2[2]*Si12[3][2];
pm2[4]=pmm2[4]*Si12[1][1] + pmm2[5]*Si12[1][2];
pm2[5]=-pmm2[6];
pm2[6]=pmm2[4]*Si12[3][1] + pmm2[5]*Si12[3][2];

p1[1]=pm2[1] + pv1[1];
p1[2]=pm2[2] + pv1[2];
p1[3]=pm2[3] + pv1[3];
p1[4]=pm2[4] + pv1[4];
p1[5]=pm2[5] + pv1[5];
p1[6]=pm2[6] + pv1[6];

pmm1[1]=state[29].thdd*h1[1] + p1[1] + c1[1]*JA1[1][1] + c1[2]*JA1[1][2] + c1[4]*JA1[1][4] + c1[5]*JA1[1][5];
pmm1[2]=state[29].thdd*h1[2] + p1[2] + c1[1]*JA1[2][1] + c1[2]*JA1[2][2] + c1[4]*JA1[2][4] + c1[5]*JA1[2][5];
pmm1[3]=state[29].thdd*h1[3] + p1[3] + c1[1]*JA1[3][1] + c1[2]*JA1[3][2] + c1[4]*JA1[3][4] + c1[5]*JA1[3][5];
pmm1[4]=state[29].thdd*h1[4] + p1[4] + c1[1]*JA1[4][1] + c1[2]*JA1[4][2] + c1[4]*JA1[4][4] + c1[5]*JA1[4][5];
pmm1[5]=state[29].thdd*h1[5] + p1[5] + c1[1]*JA1[5][1] + c1[2]*JA1[5][2] + c1[4]*JA1[5][4] + c1[5]*JA1[5][5];
pmm1[6]=state[29].thdd*h1[6] + p1[6] + c1[1]*JA1[6][1] + c1[2]*JA1[6][2] + c1[4]*JA1[6][4] + c1[5]*JA1[6][5];

pm1[1]=pmm1[1]*Si01[1][1] + pmm1[2]*Si01[1][2];
pm1[2]=pmm1[1]*Si01[2][1] + pmm1[2]*Si01[2][2];
pm1[3]=-pmm1[3];
pm1[4]=PELVISOFFSET*pmm1[3] + pmm1[4]*Si01[1][1] + pmm1[5]*Si01[1][2] - PELVIS2THORAX*pmm1[1]*Si01[2][1] - PELVIS2THORAX*pmm1[2]*Si01[2][2];
pm1[5]=PELVIS2THORAX*pmm1[1]*Si01[1][1] + PELVIS2THORAX*pmm1[2]*Si01[1][2] + pmm1[4]*Si01[2][1] + pmm1[5]*Si01[2][2];
pm1[6]=-pmm1[6] + PELVISOFFSET*pmm1[1]*Si01[1][1] + PELVISOFFSET*pmm1[2]*Si01[1][2];

p0[1]=pm1[1] + pm70[1] + pm84[1] + pv0[1];
p0[2]=pm1[2] + pm70[2] + pm84[2] + pv0[2];
p0[3]=pm1[3] + pm70[3] + pm84[3] + pv0[3];
p0[4]=pm1[4] + pm70[4] + pm84[4] + pv0[4];
p0[5]=pm1[5] + pm70[5] + pm84[5] + pv0[5];
p0[6]=pm1[6] + pm70[6] + pm84[6] + pv0[6];


}

