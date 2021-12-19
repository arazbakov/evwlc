#ifndef EVWLC_ANIMATION_GRAPH_H
#define EVWLC_ANIMATION_GRAPH_H

#include "stm32g0xx_hal.h"

#define ANIMATION_STEPS_NUMBER 256

#define ANIMATION_LINEAR 0
#define ANIMATION_FAST_OUT_SLOW_IN 1
#define ANIMATION_IN_OUT_SINE 2

typedef struct {
    char label[16];
    char title[32];
    uint8_t chart[ANIMATION_STEPS_NUMBER];
} AnimationType_t;

static AnimationType_t animations[] = {
        {
            .label = "linear",
            .title = "Linear",
            .chart = {
                    0, 1, 2, 3, 4, 5, 6, 7,
                    8, 9, 10, 11, 12, 13, 14, 15,
                    16, 17, 18, 19, 20, 21, 22, 23,
                    24, 25, 26, 27, 28, 29, 30, 31,
                    32, 33, 34, 35, 36, 37, 38, 39,
                    40, 41, 42, 43, 44, 45, 46, 47,
                    48, 49, 50, 51, 52, 53, 54, 55,
                    56, 57, 58, 59, 60, 61, 62, 63,
                    64, 65, 66, 67, 68, 69, 70, 71,
                    72, 73, 74, 75, 76, 77, 78, 79,
                    80, 81, 82, 83, 84, 85, 86, 87,
                    88, 89, 90, 91, 92, 93, 94, 95,
                    96, 97, 98, 99, 100, 101, 102, 103,
                    104, 105, 106, 107, 108, 109, 110, 111,
                    112, 113, 114, 115, 116, 117, 118, 119,
                    120, 121, 122, 123, 124, 125, 126, 127,
                    128, 129, 130, 131, 132, 133, 134, 135,
                    136, 137, 138, 139, 140, 141, 142, 143,
                    144, 145, 146, 147, 148, 149, 150, 151,
                    152, 153, 154, 155, 156, 157, 158, 159,
                    160, 161, 162, 163, 164, 165, 166, 167,
                    168, 169, 170, 171, 172, 173, 174, 175,
                    176, 177, 178, 179, 180, 181, 182, 183,
                    184, 185, 186, 187, 188, 189, 190, 191,
                    192, 193, 194, 195, 196, 197, 198, 199,
                    200, 201, 202, 203, 204, 205, 206, 207,
                    208, 209, 210, 211, 212, 213, 214, 215,
                    216, 217, 218, 219, 220, 221, 222, 223,
                    224, 225, 226, 227, 228, 229, 230, 231,
                    232, 233, 234, 235, 236, 237, 238, 239,
                    240, 241, 242, 243, 244, 245, 246, 247,
                    248, 249, 250, 251, 252, 253, 254, 255
            }
        },{
            .label = "fastOutSlowIn",
            .title = "Fast Out Slow In",
            .chart = {
                    0, 0, 0, 0, 0, 0, 0, 0,
                    1, 1, 1, 1, 1, 2, 2, 2,
                    2, 3, 3, 3, 4, 4, 5, 5,
                    6, 6, 7, 7, 8, 9, 10, 10,
                    11, 12, 13, 14, 15, 16, 17, 18,
                    19, 20, 21, 22, 24, 25, 26, 28,
                    29, 31, 32, 34, 36, 38, 39, 41,
                    43, 45, 47, 50, 52, 54, 56, 59,
                    61, 63, 66, 68, 71, 73, 76, 79,
                    81, 84, 87, 90, 92, 95, 98, 100,
                    103, 106, 109, 111, 114, 117, 119, 122,
                    124, 127, 129, 132, 134, 137, 139, 141,
                    144, 146, 148, 150, 152, 154, 156, 158,
                    160, 162, 164, 166, 168, 170, 172, 173,
                    175, 177, 178, 180, 182, 183, 185, 186,
                    188, 189, 190, 192, 193, 195, 196, 197,
                    198, 200, 201, 202, 203, 204, 205, 207,
                    208, 209, 210, 211, 212, 213, 214, 215,
                    216, 217, 218, 218, 219, 220, 221, 222,
                    223, 223, 224, 225, 226, 226, 227, 228,
                    229, 229, 230, 231, 231, 232, 232, 233,
                    234, 234, 235, 235, 236, 236, 237, 238,
                    238, 239, 239, 239, 240, 240, 241, 241,
                    242, 242, 243, 243, 243, 244, 244, 245,
                    245, 245, 246, 246, 246, 247, 247, 247,
                    248, 248, 248, 248, 249, 249, 249, 249,
                    250, 250, 250, 250, 251, 251, 251, 251,
                    251, 252, 252, 252, 252, 252, 253, 253,
                    253, 253, 253, 253, 253, 254, 254, 254,
                    254, 254, 254, 254, 254, 254, 254, 254,
                    255, 255, 255, 255, 255, 255, 255, 255,
                    255, 255, 255, 255, 255, 255, 255, 255
            }
        },{
            .label = "inOutSine",
            .title = "In Out Sine",
            .chart = {
                    0, 0, 0, 0, 1, 1, 1, 1,
                    1, 1, 2, 2, 2, 2, 3, 3,
                    3, 4, 4, 4, 5, 5, 5, 6,
                    6, 7, 7, 7, 8, 8, 9, 9,
                    10, 10, 11, 12, 12, 13, 13, 14,
                    15, 15, 16, 17, 17, 18, 19, 19,
                    20, 21, 22, 23, 23, 24, 25, 26,
                    27, 28, 29, 30, 31, 32, 33, 34,
                    35, 36, 37, 38, 39, 40, 41, 42,
                    43, 44, 46, 47, 48, 49, 51, 52,
                    53, 54, 56, 57, 58, 60, 61, 62,
                    64, 65, 67, 68, 70, 71, 72, 74,
                    75, 77, 78, 80, 82, 83, 85, 86,
                    88, 90, 91, 93, 94, 96, 98, 99,
                    101, 103, 104, 106, 108, 109, 111, 113,
                    115, 116, 118, 120, 121, 123, 125, 127,
                    128, 130, 132, 134, 135, 137, 139, 140,
                    142, 144, 146, 147, 149, 151, 152, 154,
                    156, 157, 159, 161, 162, 164, 165, 167,
                    169, 170, 172, 173, 175, 177, 178, 180,
                    181, 183, 184, 185, 187, 188, 190, 191,
                    193, 194, 195, 197, 198, 199, 201, 202,
                    203, 204, 206, 207, 208, 209, 211, 212,
                    213, 214, 215, 216, 217, 218, 219, 220,
                    221, 222, 223, 224, 225, 226, 227, 228,
                    229, 230, 231, 232, 232, 233, 234, 235,
                    236, 236, 237, 238, 238, 239, 240, 240,
                    241, 242, 242, 243, 243, 244, 245, 245,
                    246, 246, 247, 247, 248, 248, 248, 249,
                    249, 250, 250, 250, 251, 251, 251, 252,
                    252, 252, 253, 253, 253, 253, 254, 254,
                    254, 254, 254, 254, 255, 255, 255, 255
            }
        },{
            .label = "inCubic",
            .title = "In Cubic",
            .chart = {
                    0, 0, 0, 0, 0, 1, 1, 1,
                    1, 1, 1, 1, 1, 2, 2, 2,
                    2, 2, 2, 2, 2, 3, 3, 3,
                    3, 3, 3, 3, 4, 4, 4, 4,
                    4, 4, 4, 5, 5, 5, 5, 5,
                    5, 6, 6, 6, 6, 6, 7, 7,
                    7, 7, 7, 7, 8, 8, 8, 8,
                    9, 9, 9, 9, 9, 10, 10, 10,
                    10, 11, 11, 11, 11, 12, 12, 12,
                    12, 13, 13, 13, 14, 14, 14, 14,
                    15, 15, 15, 16, 16, 16, 17, 17,
                    17, 18, 18, 19, 19, 19, 20, 20,
                    20, 21, 21, 22, 22, 23, 23, 23,
                    24, 24, 25, 25, 26, 26, 27, 27,
                    28, 28, 29, 30, 30, 31, 31, 32,
                    32, 33, 34, 34, 35, 36, 36, 37,
                    38, 38, 39, 40, 41, 41, 42, 43,
                    44, 44, 45, 46, 47, 48, 49, 50,
                    51, 51, 52, 53, 54, 55, 56, 57,
                    58, 59, 60, 62, 63, 64, 65, 66,
                    67, 68, 70, 71, 72, 73, 75, 76,
                    77, 79, 80, 81, 83, 84, 85, 87,
                    88, 90, 91, 93, 94, 96, 98, 99,
                    101, 102, 104, 106, 108, 109, 111, 113,
                    114, 116, 118, 120, 122, 124, 125, 127,
                    129, 131, 133, 135, 137, 139, 141, 143,
                    145, 147, 149, 151, 154, 156, 158, 160,
                    162, 164, 167, 169, 171, 173, 175, 178,
                    180, 182, 185, 187, 189, 191, 194, 196,
                    199, 201, 203, 206, 208, 210, 213, 215,
                    218, 220, 223, 225, 228, 230, 232, 235,
                    237, 240, 242, 245, 247, 250, 252, 255
            }
        },
};


#endif //EVWLC_ANIMATION_GRAPH_H