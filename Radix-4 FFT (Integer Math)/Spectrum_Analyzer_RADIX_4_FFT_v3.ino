#include <avr/pgmspace.h>
/*
 ***********     RADIX 4 FFT    Demo Application.   ***********
 * 
 * http://coolarduino.wordpress.com/2012/03/26/radix-4-fft-integer-math/
 *
 * Created for  Arduino UNO board: Anatoly Kuzmenko 24 Mar 2012 
 *                                 k_anatoly@hotmail.com
 *
 * SOFTWARE COMPILES USING Arduino 0022 or 1.0 IDE (Tested on Linux OS only).
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute copies of the Software, 
 * and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY,  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 *
 * Copyright (C) 2012 Anatoly Kuzmenko.
 * All Rights Reserved.
 ********************************************************************
 */

//**************************************************************************************************
#define FFT_SIZE   	256
#define LOG_2_FFT       8  	

#define NWAVE           256    /* full length of Sinewave[] */
#define VERTICAL        30

const int Sinewave[NWAVE] = {
/* 12-bits */
/*
    +0,    +100,    +201,    +301,    +401,    +501,    +601,    +700,    +799,    +897,    +995,   +1092,   +1189,   +1285,   +1380,   +1474,
 +1567,   +1659,   +1751,   +1841,   +1930,   +2018,   +2105,   +2191,   +2275,   +2358,   +2439,   +2519,   +2598,   +2675,   +2750,   +2824,
 +2896,   +2966,   +3034,   +3101,   +3165,   +3228,   +3289,   +3348,   +3405,   +3460,   +3512,   +3563,   +3611,   +3658,   +3702,   +3744,
 +3783,   +3821,   +3856,   +3888,   +3919,   +3947,   +3972,   +3996,   +4016,   +4035,   +4051,   +4064,   +4075,   +4084,   +4090,   +4094,
 +4095,   +4094,   +4090,   +4084,   +4075,   +4064,   +4051,   +4035,   +4016,   +3996,   +3972,   +3947,   +3919,   +3888,   +3856,   +3821,
 +3783,   +3744,   +3702,   +3658,   +3611,   +3563,   +3512,   +3460,   +3405,   +3348,   +3289,   +3228,   +3165,   +3101,   +3034,   +2966,
 +2896,   +2824,   +2750,   +2675,   +2598,   +2519,   +2439,   +2358,   +2275,   +2191,   +2105,   +2018,   +1930,   +1841,   +1751,   +1659,
 +1567,   +1474,   +1380,   +1285,   +1189,   +1092,    +995,    +897,    +799,    +700,    +601,    +501,    +401,    +301,    +201,    +100,
    +0,    -101,    -201,    -301,    -401,    -501,    -601,    -700,    -799,    -897,    -995,   -1092,   -1189,   -1285,   -1380,   -1474,
 -1567,   -1660,   -1751,   -1842,   -1931,   -2019,   -2106,   -2191,   -2276,   -2359,   -2440,   -2520,   -2598,   -2675,   -2751,   -2824,
 -2896,   -2967,   -3035,   -3102,   -3166,   -3229,   -3290,   -3349,   -3406,   -3461,   -3513,   -3564,   -3612,   -3659,   -3703,   -3745,
 -3784,   -3822,   -3857,   -3889,   -3920,   -3948,   -3973,   -3996,   -4017,   -4036,   -4052,   -4065,   -4076,   -4085,   -4091,   -4095,
 -4096,   -4095,   -4091,   -4085,   -4076,   -4065,   -4052,   -4036,   -4017,   -3996,   -3973,   -3948,   -3920,   -3889,   -3857,   -3822,
 -3784,   -3745,   -3703,   -3659,   -3612,   -3564,   -3513,   -3461,   -3406,   -3349,   -3290,   -3229,   -3166,   -3102,   -3035,   -2967,
 -2896,   -2824,   -2751,   -2675,   -2598,   -2520,   -2440,   -2359,   -2276,   -2191,   -2106,   -2019,   -1931,   -1842,   -1751,   -1660,
 -1567,   -1474,   -1380,   -1285,   -1189,   -1092,    -995,    -897,    -799,    -700,    -601,    -501,    -401,    -301,    -201,    -101
*/
/* 15-bits */
/*
    +0,    +804,   +1608,   +2410,   +3212,   +4011,   +4808,   +5602,   +6393,   +7179,   +7962,   +8739,   +9512,  +10278,  +11039,  +11793,
+12539,  +13279,  +14010,  +14732,  +15446,  +16151,  +16846,  +17530,  +18204,  +18868,  +19519,  +20159,  +20787,  +21403,  +22005,  +22594,
+23170,  +23731,  +24279,  +24811,  +25329,  +25832,  +26319,  +26790,  +27245,  +27683,  +28105,  +28510,  +28898,  +29268,  +29621,  +29956,
+30273,  +30571,  +30852,  +31113,  +31356,  +31580,  +31785,  +31971,  +32137,  +32285,  +32412,  +32521,  +32609,  +32678,  +32728,  +32757,
+32767,  +32757,  +32728,  +32678,  +32609,  +32521,  +32412,  +32285,  +32137,  +31971,  +31785,  +31580,  +31356,  +31113,  +30852,  +30571,
+30273,  +29956,  +29621,  +29268,  +28898,  +28510,  +28105,  +27683,  +27245,  +26790,  +26319,  +25832,  +25329,  +24811,  +24279,  +23731,
+23170,  +22594,  +22005,  +21403,  +20787,  +20159,  +19519,  +18868,  +18204,  +17530,  +16846,  +16151,  +15446,  +14732,  +14010,  +13279,
+12539,  +11793,  +11039,  +10278,   +9512,   +8739,   +7962,   +7179,   +6393,   +5602,   +4808,   +4011,   +3212,   +2410,   +1608,    +804,
    +0,    -804,   -1608,   -2410,   -3212,   -4011,   -4808,   -5602,   -6393,   -7179,   -7962,   -8739,   -9512,  -10278,  -11039,  -11793,
-12539,  -13279,  -14010,  -14732,  -15446,  -16151,  -16846,  -17530,  -18204,  -18868,  -19519,  -20159,  -20787,  -21403,  -22005,  -22594,
-23170,  -23731,  -24279,  -24811,  -25329,  -25832,  -26319,  -26790,  -27245,  -27683,  -28105,  -28510,  -28898,  -29268,  -29621,  -29956,
-30273,  -30571,  -30852,  -31113,  -31356,  -31580,  -31785,  -31971,  -32137,  -32285,  -32412,  -32521,  -32609,  -32678,  -32728,  -32757,
-32767,  -32757,  -32728,  -32678,  -32609,  -32521,  -32412,  -32285,  -32137,  -31971,  -31785,  -31580,  -31356,  -31113,  -30852,  -30571,
-30273,  -29956,  -29621,  -29268,  -28898,  -28510,  -28105,  -27683,  -27245,  -26790,  -26319,  -25832,  -25329,  -24811,  -24279,  -23731,
-23170,  -22594,  -22005,  -21403,  -20787,  -20159,  -19519,  -18868,  -18204,  -17530,  -16846,  -16151,  -15446,  -14732,  -14010,  -13279,
-12539,  -11793,  -11039,  -10278,   -9512,   -8739,   -7962,   -7179,   -6393,   -5602,   -4808,   -4011,   -3212,   -2410,   -1608,    -804
*/
/* 8-bits */
      +0,      +6,     +13,     +19,     +25,     +31,     +37,     +44,     +50,     +56,     +62,     +68,     +74,     +80,     +86,     +92,
     +98,    +103,    +109,    +115,    +120,    +126,    +131,    +136,    +142,    +147,    +152,    +157,    +162,    +167,    +171,    +176,
    +180,    +185,    +189,    +193,    +197,    +201,    +205,    +208,    +212,    +215,    +219,    +222,    +225,    +228,    +231,    +233,
    +236,    +238,    +240,    +242,    +244,    +246,    +247,    +249,    +250,    +251,    +252,    +253,    +254,    +254,    +255,    +255,
    +255,    +255,    +255,    +254,    +254,    +253,    +252,    +251,    +250,    +249,    +247,    +246,    +244,    +242,    +240,    +238,
    +236,    +233,    +231,    +228,    +225,    +222,    +219,    +215,    +212,    +208,    +205,    +201,    +197,    +193,    +189,    +185,
    +180,    +176,    +171,    +167,    +162,    +157,    +152,    +147,    +142,    +136,    +131,    +126,    +120,    +115,    +109,    +103,
     +98,     +92,     +86,     +80,     +74,     +68,     +62,     +56,     +50,     +44,     +37,     +31,     +25,     +19,     +13,      +6,
      +0,      -6,     -13,     -19,     -25,     -31,     -38,     -44,     -50,     -56,     -62,     -68,     -74,     -80,     -86,     -92,
     -98,    -104,    -109,    -115,    -121,    -126,    -132,    -137,    -142,    -147,    -152,    -157,    -162,    -167,    -172,    -177,
    -181,    -185,    -190,    -194,    -198,    -202,    -206,    -209,    -213,    -216,    -220,    -223,    -226,    -229,    -231,    -234,
    -237,    -239,    -241,    -243,    -245,    -247,    -248,    -250,    -251,    -252,    -253,    -254,    -255,    -255,    -256,    -256,
    -256,    -256,    -256,    -255,    -255,    -254,    -253,    -252,    -251,    -250,    -248,    -247,    -245,    -243,    -241,    -239,
    -237,    -234,    -231,    -229,    -226,    -223,    -220,    -216,    -213,    -209,    -206,    -202,    -198,    -194,    -190,    -185,
    -181,    -177,    -172,    -167,    -162,    -157,    -152,    -147,    -142,    -137,    -132,    -126,    -121,    -115,    -109,    -104,
     -98,     -92,     -86,     -80,     -74,     -68,     -62,     -56,     -50,     -44,     -38,     -31,     -25,     -19,     -13,      -6
};

#define mult_shf_s16x16( a, b)    \
({                        \
int prod, val1=a, val2=b; \
__asm__ __volatile__ (    \ 
"muls %B1, %B2	\n\t"     \
"mov %B0, r0    \n\t"	  \ 
"mul %A1, %A2   \n\t"	  \ 
"mov %A0, r1    \n\t"     \ 
"mulsu %B1, %A2	\n\t"     \ 
"add %A0, r0    \n\t"     \ 
"adc %B0, r1    \n\t"     \ 
"mulsu %B2, %A1	\n\t"     \ 
"add %A0, r0    \n\t"     \ 
"adc %B0, r1    \n\t"     \ 
"clr r1         \n\t"     \ 
: "=&d" (prod)            \
: "a" (val1), "a" (val2)  \
);                        \
prod;                     \
})

static inline void mult_shf_I( int c, int s, int x, int y, int &u, int &v)  __attribute__((always_inline));
static inline void mult_shf_I( int c, int s, int x, int y, int &u, int &v)
{
//  u = ((long)x * c - (long)y * s) >> 12;      // Optimizer macro-mulriplier OFF, 24 millisec
//  v = ((long)y * c + (long)x * s) >> 12; 

  u = (mult_shf_s16x16(x, c) - mult_shf_s16x16(y, s));    // Optimizer macro-mulriplier ON, 10.1 millisec
  v = (mult_shf_s16x16(y, c) + mult_shf_s16x16(x, s));    // Hardcoded >>8 bits, use with 8-bits Sinewave ONLY.
}

static inline void sum_dif_I(int a, int b, int &s, int &d)  __attribute__((always_inline));
static inline void sum_dif_I(int a, int b, int &s, int &d)
{
  s = (a+b);// >> 1; // Right Shift Limiter: OFF   
  d = (a-b);// >> 1; // Performance with RSL 25.5 millisec, w/o - 25.1 millisec.
}

void rev_bin( int *fr, int fft_n)
{
    int m, mr, nn, l;
    int tr;

    mr = 0;
    nn = fft_n - 1;

    for (m=1; m<=nn; ++m) {
            l = fft_n;
         do {
             l >>= 1;
            } while (mr+l > nn);

            mr = (mr & (l-1)) + l;

        if (mr <= m)
             continue;
            tr = fr[m];
            fr[m] = fr[mr];
            fr[mr] = tr;
    }
}

void fft_radix4_I( int *fr, int *fi, int ldn)
{
    const int n = (1UL<<ldn);
    int ldm = 0, rdx = 2;

    for (int i0 = 0; i0 < n; i0 += 4)
        {
            int xr,yr,ur,vr, xi,yi,ui,vi;

            int i1 = i0 + 1;
            int i2 = i1 + 1;
            int i3 = i2 + 1;

            sum_dif_I(fr[i0], fr[i1], xr, ur);
            sum_dif_I(fr[i2], fr[i3], yr, vi);
            sum_dif_I(fi[i0], fi[i1], xi, ui);
            sum_dif_I(fi[i3], fi[i2], yi, vr);

            sum_dif_I(ui, vi, fi[i1], fi[i3]);
            sum_dif_I(xi, yi, fi[i0], fi[i2]);
            sum_dif_I(ur, vr, fr[i1], fr[i3]);
            sum_dif_I(xr, yr, fr[i0], fr[i2]);
        }
    
    for (ldm = 2 * rdx; ldm <= ldn; ldm += rdx)
    {
        int m = (1UL<<ldm);
        int m4 = (m>>rdx);

        int phI0 =  NWAVE / m;                            
        int phI  = 0;

        for (int j = 0; j < m4; j++)
        {
        int c,s,c2,s2,c3,s3;

         s  = Sinewave[   phI];
         s2 = Sinewave[ 2*phI];
         s3 = Sinewave[ 3*phI];

         c  = Sinewave[   phI + NWAVE/4];
         c2 = Sinewave[ 2*phI + NWAVE/4];
         c3 = Sinewave[ 3*phI + NWAVE/4];

       for (int r = 0; r < n; r += m)    
       {
                int i0 = j + r;
                int i1 = i0 + m4;
                int i2 = i1 + m4;
                int i3 = i2 + m4;

           int xr,yr,ur,vr, xi,yi,ui,vi;

             mult_shf_I( c2, s2, fr[i1], fi[i1], xr, xi);
             mult_shf_I(  c,  s, fr[i2], fi[i2], yr, vr);
             mult_shf_I( c3, s3, fr[i3], fi[i3], vi, yi);

             int t = yi - vr;
                yi += vr;
                vr = t;

                ur = fr[i0] - xr;
                xr += fr[i0];

              sum_dif_I(ur, vr, fr[i1], fr[i3]);

                 t = yr - vi;
                yr += vi;
                vi = t;

                ui = fi[i0] - xi;
                xi += fi[i0];

              sum_dif_I(ui, vi, fi[i1], fi[i3]);
              sum_dif_I(xr, yr, fr[i0], fr[i2]);
              sum_dif_I(xi, yi, fi[i0], fi[i2]);
            }
          phI += phI0;
        }
    }
}

void print_charta( int data[], int dlina )
{
  int peak = 0, nbr;
  const char mark = '*', nomk = '-';

  for ( int i = 0; i < FFT_SIZE/dlina; i++)
   { if ( (data[i]) > peak ){
        peak = data[i];
        nbr = i;
       }
    }
  for ( int j = VERTICAL; j > 0; j-- )
            { Serial.print('|');
           for ( int i = 0; i < FFT_SIZE/dlina; i++)
            { if ((data[i]) >= j * (peak/VERTICAL)) {
                 Serial.print( mark);
                 delay(1);
            }
              else{
                 Serial.print( nomk);
                 delay(1);
              }
            }
           Serial.println('|');
        }
   Serial.print("\tPeak: ");
   Serial.print(peak);
   Serial.print("\t N: ");
   Serial.println(nbr);
}

//**************************************************************************************************

int   f_r[FFT_SIZE];
int   f_i[FFT_SIZE];
int    incomingByte; 

unsigned long time_start;
unsigned int  time_sampl, time_revbn, time_radix, time_sqrtl;

const int sdvig = 512; //DC bias of the ADC, approxim +2.5V.

void setup() {
  Serial.begin(115200);  
}

void sampling() {
  //  ADCSRA = 0x87; // turn on adc, freq = 1/128 ,  125 kHz/ 13.5 =~  9 kHz sampling rate
  ADCSRA = 0x85;   // turn on adc, freq  = 1/32,   500 kHz/ 13.5 =~ 36   kHz sampling rate
                   // Freq. spectrum analysis up to 36 kHz/  2   = 18 kHz
 
  ADMUX = 0x40;    //Bit 5 – ADLAR: NO ADC Left Adjust Result
  ADCSRA |= (1<<ADSC);
  //	while((ADCSRA&(1<<ADIF)) == 0); //Discard first conversion result.
  while(!(ADCSRA & 0x10));

  int sum2b;
  for(int i = 0; i < FFT_SIZE; i++ ) {

    while(!(ADCSRA & 0x10));
    ADCSRA |= (1<<ADSC);

    sum2b =  ADCL; 
    sum2b += (ADCH << 8); 
    
    f_r[i] = sum2b - sdvig;
    f_i[i] = 0;
  }  
  ADCSRA = 0x00;
}

void loop()
{
//Debugging monitor, to check processing data on each stage.
// x command - printout data received from ADC (input raw data).
  if (Serial.available() > 0) {
    incomingByte = Serial.read();
    if (incomingByte == 'x') {
    
    sampling();
    
      for (int i = 0; i < FFT_SIZE; i++){
        Serial.print("\t");
        Serial.print(f_r[i], DEC);
        if ((i+1)%10 == 0) Serial.print("\n");
      } 
      Serial.print("\n");
      delay(200);
    }
// f command - printout data after FFT. Clear view of each bin in the spectrum.
// ---->>>>>   Both format, as a table and chart representation   <<<<<-------
// Chart is AUTOSCALE mode, pay attention to the magnitude printed at the bottom.
  if (incomingByte == 'f') {

  time_start = micros();
 sampling(); 
  time_sampl  = micros() - time_start;

  time_start = micros();
 rev_bin( f_r, FFT_SIZE);
  time_revbn  = micros() - time_start;

  time_start = micros();

 fft_radix4_I( f_r, f_i, LOG_2_FFT);

  time_radix  = micros() - time_start;


      time_start = micros();
   for (int i = 0; i < (FFT_SIZE / 2); i++){
      f_r[i] = sqrt((long)f_r[i] * f_r[i] + (long)f_i[i] * f_i[i]);
      }
   time_sqrtl  = micros() - time_start;

      for (int i=0; i < (FFT_SIZE / 2); i++){
        Serial.print("\t");
        Serial.print(f_r[i], DEC);     
        if ((i+1)%16 == 0) Serial.print("\n");
      } // Prosmotr dannuh massiva polychennogo posle FFT
      delay(200);
      Serial.print("\n");

     Serial.print("\tSmpl ");
     Serial.print(time_sampl);
     Serial.print("\tRevb ");
     Serial.print(time_revbn);
     Serial.print("\tRDX4 ");
     Serial.print(time_radix);
     Serial.print("\tSqrt ");
     Serial.println(time_sqrtl);
  
     print_charta( f_r, 2 );
  
      for (int i = 0; i < (FFT_SIZE / 2); i++){
      f_r[i] = 20 * log10(f_r[i] +1);  // Floor -60 dBV ( limit 10-bits ADC)
      }

      for (int i=0; i < (FFT_SIZE / 2); i++){
        Serial.print("\t");
        Serial.print((f_r[i] -90), DEC);     
        if ((i+1)%16 == 0) Serial.print("\n");
      } // Prosmotr dannuh massiva polychennogo posle FFT
      delay(200);
      Serial.print("\n");
  
     print_charta( f_r, 2 );
  }
 }
}
