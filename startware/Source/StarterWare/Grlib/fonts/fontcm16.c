//*****************************************************************************
//
// fontcm16.c - Font definition for the 16 point Cm font.
//
// Copyright (c) 2008-2010 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 6288 of the Stellaris Graphics Library.
//
//*****************************************************************************

//*****************************************************************************
//
// This file is generated by ftrasterize; DO NOT EDIT BY HAND!
//
//*****************************************************************************

#include "grlib.h"

//*****************************************************************************
//
// Details of this font:
//     Style: cm
//     Size: 16 point
//     Bold: no
//     Italic: no
//     Memory usage: 2084 bytes
//
//*****************************************************************************

//*****************************************************************************
//
// The compressed data for the 16 point Cm font.
//
//*****************************************************************************
static const unsigned char g_pucCm16Data[1881] =
{
      5,   6,   0,  12,  96,  13,   2,  33,  17,  17,  17,  17,
     17,  17,  17,  17,  81, 144,  15,   6, 194,  18,  33,  33,
     33,  33,  17,  33,  33,  33,   0,   7,  96,  29,  12, 240,
    225, 161,  49, 113,  49, 113,  33, 129,  33,  75,  65,  49,
    113,  33,  91,  49,  49, 113,  49, 113,  33, 129,  33, 177,
    240,  48,  31,   7,  49,  83,  49,  17,  17,  17,  33,  17,
     17,  33,  17,  17,  33,  65,  17,  83,  83,  65,  17,  18,
     17,  17,  17,  33,  17,  36,  81, 240, 144,  43,  12,  18,
     81,  65,  17,  50,  49,  52,  65,  49,  33,  65,  49,  17,
     81,  49,  17,  82,  17,  17, 115,  17,  19,  97,  17,  34,
     65,  33,  49,  65,  33,  49,  49,  49,  49,  49,  50,  18,
    131, 240, 240, 128,  32,  13, 240,  18, 161,  33, 145,  33,
    145,  33, 145,  17, 162,  68,  65,  81,  68,  49,  65,  49,
     33,  81,  66,  98,  49,  17,  49,  36,  51,   0,   6,  96,
     10,   3,  98,  33,  33,  17,  33, 240, 240,  32,  18,   4,
     97,  33,  49,  33,  49,  49,  49,  49,  49,  49,  49,  49,
     65,  49,  65,  49,  18,   4,  65,  65,  49,  65,  49,  49,
     49,  49,  49,  49,  49,  33,  49,  49,  33, 112,  14,   7,
    145,  65,  17,  33,  36,  66,  51,  18,  49,   0,   9,  32,
     18,  12, 240, 240, 177, 177, 177, 177, 177, 107,  97, 177,
    177, 177, 177, 240, 240, 192,   9,   3, 240, 240,  98,  33,
     33,  17,  80,   8,   5, 240, 240, 244, 240, 240,  96,   5,
      2, 240, 145, 144,  20,   7,  81,  97,  81,  97,  97,  81,
     97,  97,  81,  97,  97,  81,  97,  97,  81,  97,  97,  96,
     25,   7, 240,  18,  65,  33,  33,  65,  17,  65,  17,  65,
     17,  65,  17,  65,  17,  65,  17,  65,  33,  33,  67, 240,
    240,  15,   6, 225,  51,  81,  81,  81,  81,  81,  81,  81,
     81,  53, 240, 160,  18,   6, 211,  33,  34,  81,  17,  49,
     81,  65,  81,  65,  65,  65,  49,  21, 240, 160,  19,   7,
    244,  33,  65,  17,  65,  81,  51, 113, 113,  97,  17,  65,
     17,  50,  36, 240, 240,  21,   8, 240,  81, 113,  83,  81,
     17,  65,  33,  49,  49,  49,  49,  55,  81, 113,  99, 240,
    240,  64,  19,   7, 241, 100,  49,  97,  18,  50,  17, 113,
     97,  97,  18,  49,  17,  65,  36, 240, 240,  24,   7, 240,
     20,  33,  49,  33,  81,  19,  34,  33,  33,  65,  17,  65,
     17,  65,  17,  65,  33,  33,  67, 240, 240,  18,   7, 113,
    102,  17,  65,  81,  81,  97,  81,  97,  97,  97,  97,  97,
    240, 240,  32,  25,   8, 240,  52,  34,  50,  17,  81,  18,
     65,  34,  18,  67,  65,  50,  17,  81,  17,  81,  18,  49,
     67, 240, 240,  80,  23,   7, 243,  50,  33,  33,  65,  17,
     65,  17,  65,  18,  34,  35,  17,  97,  81,  33,  49,  36,
    240, 240,  16,   5,   2, 193, 177, 144,   7,   2, 193, 177,
     17,  17,  80,  13,   2, 161,  49,  17,  17,  17,  17,  17,
     17,  17,  17,  48,  10,  11,   0,   9,  90, 240, 138,   0,
      8,  48,  16,   6, 240, 240,  33, 177,  81,  81,  65,  81,
     65,  81,  81,  49,  35, 128,  17,   6, 115,  33,  49,  17,
     49,  81,  65,  81,  65,  81,  81, 240,  33, 240, 192,  41,
     12, 244, 113,  66,  65,  34,  49,  34,  17,  33,  33,  33,
     17,  65,  33,  17,  17,  65,  33,  17,  17,  65,  33,  17,
     17,  65,  33,  18,  17,  34,  33,  33,  34,  34,  65, 113,
     71,   0,   6,  32,  27,  12, 240,  33, 177, 161,  17, 129,
     33, 129,  33, 129,  49,  97,  65, 117,  97,  66,  65,  97,
     65,  97,  51,  68,   0,   6,  16,  25,   9, 240,  55,  49,
     66,  33,  81,  33,  65,  54,  49,  66,  33,  81,  33,  81,
     33,  81,  33,  65,  39, 240, 240, 128,  25,  11, 240, 164,
     33,  49,  65,  17,  33,  98,  17, 129,  17, 161, 161, 161,
    129,  33, 113,  49,  66,  85,   0,   5, 112,  26,  10, 240,
     86,  81,  66,  49,  81,  49,  97,  33,  97,  33,  97,  33,
     97,  33,  97,  33,  81,  49,  66,  38, 240, 240, 224,  25,
     10, 240,  88,  49,  97,  33,  97,  33,  49,  85,  81,  49,
     81,  49,  81,  97,  33,  97,  33,  82,  25, 240, 240, 176,
     22,  10, 240,  88,  49,  81,  49,  97,  33,  49,  85,  81,
     49,  81,  49,  81, 145, 145, 132,   0,   5,  96,  23,  11,
    240, 164,  82,  66,  49,  97,  33, 161, 161, 161,  84,  17,
    113,  49,  97,  65,  81,  85,   0,   5, 112,  27,  10, 240,
     83,  51,  33,  81,  49,  81,  49,  81,  55,  49,  81,  49,
     81,  49,  81,  49,  81,  49,  81,  35,  51, 240, 240, 176,
     15,   4, 131,  33,  49,  49,  49,  49,  49,  49,  49,  49,
     35, 240,  32,  18,   7, 240,  20,  81,  97,  97,  97,  97,
     97,  97,  97,  33,  49,  51, 240, 240,  16,  27,  11, 240,
    115,  67,  33,  81,  65,  65,  81,  49,  97,  33, 117,  97,
     49,  97,  65,  81,  81,  65,  82,  35,  67, 240, 240, 240,
     19,   8, 240,  20,  81, 113, 113, 113, 113, 113, 113, 113,
     65,  33,  65,  23, 240, 240,  48,  42,  12, 240, 147,  98,
     34,  97,  49,  17,  51,  49,  17,  49,  17,  49,  17,  49,
     17,  49,  33,  17,  33,  49,  33,  17,  33,  49,  33,  17,
     33,  49,  49,  49,  49,  49,  49,  35,  33,  35,   0,   6,
     16,  33,  11, 240, 114,  83,  34,  81,  51,  65,  49,  17,
     65,  49,  18,  49,  49,  34,  33,  49,  49,  33,  49,  65,
     17,  49,  67,  49,  82,  34,  97,   0,   5,  96,  26,  10,
    240, 131,  82,  50,  49,  81,  33, 113,  17, 113,  17, 113,
     17, 113,  17, 113,  33,  81,  50,  50,  83, 240, 240, 224,
     21,   9, 240,  54,  65,  66,  33,  81,  33,  81,  33,  66,
     37,  65, 129, 129, 129, 115, 240, 240, 192,  30,  11, 240,
    165,  81,  66,  49,  97,  33, 129,  17, 129,  17, 129,  17,
    129,  17, 129,  33,  35,  17,  67,  18, 100, 177,  17, 129,
     17, 145, 208,  27,  12, 240, 150, 113,  66,  81,  81,  81,
     66,  85, 113,  50,  97,  65,  97,  66,  81,  66,  81,  66,
     33,  19,  67,   0,   6,  32,  26,   8, 240,  36,  17,  18,
     33,  17,  17,  66,  17,  81,  18, 132, 114,  17,  81,  17,
     81,  18,  50,  17,  20, 240, 240,  64,  21,  11, 240, 137,
     33,  49,  49,  17,  65,  49,  97, 161, 161, 161, 161, 161,
    161, 133,   0,   5, 112,  26,  11, 240, 115,  67,  33,  97,
     49,  97,  49,  97,  49,  97,  49,  97,  49,  97,  49,  97,
     49,  97,  65,  65, 100,   0,   6,  26,  12, 240, 147,  83,
     33, 113,  65,  81,  81,  81,  97,  49, 113,  49, 114,  33,
    129,  17, 145,  17, 161, 177,   0,   6,  96,  42,  16, 240,
    240,  35,  51,  51,  33,  81,  81,  65,  66,  49,  81,  49,
     17,  49,  81,  49,  17,  49,  97,  33,  18,  17, 113,  17,
     49,  17, 113,  17,  49,  17, 130,  50, 145,  81, 145,  81,
      0,   8,  80,  26,  12, 240, 148,  51,  65,  81,  82,  49,
    114,  17, 146, 177, 161,  17, 129,  49,  97,  66,  65,  97,
     51,  68,   0,   6,  16,  22,  12, 240, 147,  83,  49,  97,
     65,  81,  97,  49, 129,  17, 147, 161, 177, 177, 177, 163,
      0,   6,  80,  20,   9, 240,  56,  17,  81, 129, 113, 113,
    113, 129, 113, 113,  81,  17,  97,  24, 240, 240, 112,  18,
      4,  67,  17,  49,  49,  49,  49,  49,  49,  49,  49,  49,
     49,  49,  49,  49,  51,  13,   6, 209,  33,  17,  33,  33,
     33,  34,  18,   0,   8,  48,  18,   4,  67,  49,  49,  49,
     49,  49,  49,  49,  49,  49,  49,  49,  49,  49,  49,  19,
      8,   5, 177,  49,  18,   0,   8,  32,   5,   2,  97, 240,
    192,   9,   3, 113,  17,  33,  34, 240, 240,  64,  20,   8,
      0,   6,  19,  66,  33,  83,  50,  33,  49,  49,  17,  17,
     49,  17,  37, 240, 240,  64,  24,   9, 240,  50, 129, 129,
    129, 129,  19,  66,  49,  49,  81,  33,  81,  33,  81,  34,
     49,  49,  19, 240, 240, 144,  15,   7, 240, 240, 227,  34,
     49,  17,  97,  97, 113,  49,  51, 240, 240,  24,   9, 240,
    145, 129, 129, 129,  67,  17,  49,  50,  33,  81,  33,  81,
     33,  81,  49,  50,  67,  18, 240, 240, 112,  15,   7, 240,
    240, 212,  33,  65,  22,  17,  97, 113,  49,  51, 240, 240,
     15,   5, 179,  33,  65,  65,  51,  49,  65,  65,  65,  65,
     51, 240, 112,  19,   8,   0,   6,  37,  33,  49,  49,  49,
     52,  65, 113, 132,  34,  65,  17,  81,  37, 160,  24,   8,
    240,  18, 113, 113, 113, 113,  19,  50,  33,  49,  49,  49,
     49,  49,  49,  49,  49,  35,  19, 240, 240,  48,  12,   4,
    209, 177,  49,  49,  49,  49,  49,  35, 240,  32,  16,   5,
    240,  34, 210,  65,  65,  65,  65,  65,  65,  65,  17,  33,
     19, 112,  24,   8, 240,  18, 113, 113, 113, 113,  49,  49,
     33,  65,  17,  82,  17,  65,  33,  65,  49,  35,  34, 240,
    240,  48,  15,   4, 130,  49,  49,  49,  49,  49,  49,  49,
     49,  49,  35, 240,  32,  28,  12,   0,   9,   2,  18,  35,
     50,  34,  33,  49,  49,  49,  49,  49,  49,  49,  49,  49,
     49,  49,  49,  35,  19,  19,   0,   6,  16,  21,   8,   0,
      6,   2,  19,  50,  33,  49,  49,  49,  49,  49,  49,  49,
     49,  35,  19, 240, 240,  48,  19,   8,   0,   6,  35,  65,
     49,  33,  81,  17,  81,  17,  81,  33,  49,  67, 240, 240,
     80,  21,   8,   0,   6,   2,  19,  50,  34,  33,  65,  33,
     65,  33,  65,  33,  49,  52,  65, 113,  99, 208,  21,   9,
      0,   7,   3,  81,  50,  33,  81,  33,  81,  33,  81,  49,
     50,  67,  17, 129, 129, 115, 160,  15,   6, 240, 240,  98,
     18,  34,  17,  33,  81,  81,  81,  67, 240, 192,  19,   7,
    240, 240, 211,  17,  17,  50,  17,  65,  36,  33,  65,  18,
     49,  17,  19, 240, 240,  17,   7, 240, 113,  97,  97,  86,
     33,  97,  97,  97,  49,  33,  49,  51, 240, 240,  21,   8,
      0,   6,   2,  34,  49,  49,  49,  49,  49,  49,  49,  49,
     49,  34,  66,  18, 240, 240,  48,  19,   8,   0,   6,   3,
     34,  33,  49,  49,  49,  65,  17,  81,  17,  97, 113, 240,
    240,  96,  28,  12,   0,   9,   3,  19,  34,  33,  49,  49,
     65,  33,  33,  81,  17,  17,  17,  81,  17,  17,  17,  97,
     49, 113,  49,   0,   6,  64,  18,   9,   0,   6,  99,  35,
     49,  33,  98, 129,  99,  81,  49,  50,  51, 240, 240, 112,
     21,   8,   0,   6,   3,  34,  33,  49,  49,  49,  65,  17,
     81,  17,  97, 113, 113,  65,  17,  82, 224,  15,   7, 240,
    240, 198,  17,  49,  81,  81,  81,  81,  65,  22, 240, 224,
      8,   9,   0,   9,   8,   0,   9,  16,   8,  16,   0,  16,
     15,   0,  16,  16,   9,   5, 177,  17,  17,  17,   0,   8,
     48,   8,   6, 240,  51,  17,   0,   9, 112,
};

//*****************************************************************************
//
// The font definition for the 16 point Cm font.
//
//*****************************************************************************
const tFont g_sFontCm16 =
{
    //
    // The format of the font.
    //
    FONT_FMT_PIXEL_RLE,

    //
    // The maximum width of the font.
    //
    14,

    //
    // The height of the font.
    //
    17,

    //
    // The baseline of the font.
    //
    13,

    //
    // The offset to each character in the font.
    //
    {
           0,    5,   18,   33,   62,   93,  136,  168,
         178,  196,  214,  228,  246,  255,  263,  268,
         288,  313,  328,  346,  365,  386,  405,  429,
         447,  472,  495,  500,  507,  520,  530,  546,
         563,  604,  631,  656,  681,  707,  732,  754,
         777,  804,  819,  837,  864,  883,  925,  958,
         984, 1005, 1035, 1062, 1088, 1109, 1135, 1161,
        1203, 1229, 1251, 1271, 1289, 1302, 1320, 1328,
        1333, 1342, 1362, 1386, 1401, 1425, 1440, 1455,
        1474, 1498, 1510, 1526, 1550, 1565, 1593, 1614,
        1633, 1654, 1675, 1690, 1709, 1726, 1747, 1766,
        1794, 1812, 1833, 1848, 1856, 1864, 1873,
    },

    //
    // A pointer to the actual font data
    //
    g_pucCm16Data
};
