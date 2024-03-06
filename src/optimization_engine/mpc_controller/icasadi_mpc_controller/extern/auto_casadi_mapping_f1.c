/* This file was automatically generated by CasADi 3.6.3.
 *  It consists of: 
 *   1) content generated by CasADi runtime: not copyrighted
 *   2) template code copied from CasADi source: permissively licensed (MIT-0)
 *   3) user code: owned by the user
 *
 */
#ifdef __cplusplus
extern "C" {
#endif

/* How to prefix internal symbols */
#ifdef CASADI_CODEGEN_PREFIX
  #define CASADI_NAMESPACE_CONCAT(NS, ID) _CASADI_NAMESPACE_CONCAT(NS, ID)
  #define _CASADI_NAMESPACE_CONCAT(NS, ID) NS ## ID
  #define CASADI_PREFIX(ID) CASADI_NAMESPACE_CONCAT(CODEGEN_PREFIX, ID)
#else
  #define CASADI_PREFIX(ID) open_mapping_f1_mpc_controller_ ## ID
#endif

#include <math.h>

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int long long int
#endif

/* Add prefix to internal symbols */
#define casadi_copy CASADI_PREFIX(copy)
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)
#define casadi_s2 CASADI_PREFIX(s2)

/* Symbol visibility in DLLs */
#ifndef CASADI_SYMBOL_EXPORT
  #if defined(_WIN32) || defined(__WIN32__) || defined(__CYGWIN__)
    #if defined(STATIC_LINKED)
      #define CASADI_SYMBOL_EXPORT
    #else
      #define CASADI_SYMBOL_EXPORT __declspec(dllexport)
    #endif
  #elif defined(__GNUC__) && defined(GCC_HASCLASSVISIBILITY)
    #define CASADI_SYMBOL_EXPORT __attribute__ ((visibility ("default")))
  #else
    #define CASADI_SYMBOL_EXPORT
  #endif
#endif

void casadi_copy(const casadi_real* x, casadi_int n, casadi_real* y) {
  casadi_int i;
  if (y) {
    if (x) {
      for (i=0; i<n; ++i) *y++ = *x++;
    } else {
      for (i=0; i<n; ++i) *y++ = 0.;
    }
  }
}

static const casadi_int casadi_s0[24] = {20, 1, 0, 20, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19};
static const casadi_int casadi_s1[1252] = {1248, 1, 0, 1248, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 251, 252, 253, 254, 255, 256, 257, 258, 259, 260, 261, 262, 263, 264, 265, 266, 267, 268, 269, 270, 271, 272, 273, 274, 275, 276, 277, 278, 279, 280, 281, 282, 283, 284, 285, 286, 287, 288, 289, 290, 291, 292, 293, 294, 295, 296, 297, 298, 299, 300, 301, 302, 303, 304, 305, 306, 307, 308, 309, 310, 311, 312, 313, 314, 315, 316, 317, 318, 319, 320, 321, 322, 323, 324, 325, 326, 327, 328, 329, 330, 331, 332, 333, 334, 335, 336, 337, 338, 339, 340, 341, 342, 343, 344, 345, 346, 347, 348, 349, 350, 351, 352, 353, 354, 355, 356, 357, 358, 359, 360, 361, 362, 363, 364, 365, 366, 367, 368, 369, 370, 371, 372, 373, 374, 375, 376, 377, 378, 379, 380, 381, 382, 383, 384, 385, 386, 387, 388, 389, 390, 391, 392, 393, 394, 395, 396, 397, 398, 399, 400, 401, 402, 403, 404, 405, 406, 407, 408, 409, 410, 411, 412, 413, 414, 415, 416, 417, 418, 419, 420, 421, 422, 423, 424, 425, 426, 427, 428, 429, 430, 431, 432, 433, 434, 435, 436, 437, 438, 439, 440, 441, 442, 443, 444, 445, 446, 447, 448, 449, 450, 451, 452, 453, 454, 455, 456, 457, 458, 459, 460, 461, 462, 463, 464, 465, 466, 467, 468, 469, 470, 471, 472, 473, 474, 475, 476, 477, 478, 479, 480, 481, 482, 483, 484, 485, 486, 487, 488, 489, 490, 491, 492, 493, 494, 495, 496, 497, 498, 499, 500, 501, 502, 503, 504, 505, 506, 507, 508, 509, 510, 511, 512, 513, 514, 515, 516, 517, 518, 519, 520, 521, 522, 523, 524, 525, 526, 527, 528, 529, 530, 531, 532, 533, 534, 535, 536, 537, 538, 539, 540, 541, 542, 543, 544, 545, 546, 547, 548, 549, 550, 551, 552, 553, 554, 555, 556, 557, 558, 559, 560, 561, 562, 563, 564, 565, 566, 567, 568, 569, 570, 571, 572, 573, 574, 575, 576, 577, 578, 579, 580, 581, 582, 583, 584, 585, 586, 587, 588, 589, 590, 591, 592, 593, 594, 595, 596, 597, 598, 599, 600, 601, 602, 603, 604, 605, 606, 607, 608, 609, 610, 611, 612, 613, 614, 615, 616, 617, 618, 619, 620, 621, 622, 623, 624, 625, 626, 627, 628, 629, 630, 631, 632, 633, 634, 635, 636, 637, 638, 639, 640, 641, 642, 643, 644, 645, 646, 647, 648, 649, 650, 651, 652, 653, 654, 655, 656, 657, 658, 659, 660, 661, 662, 663, 664, 665, 666, 667, 668, 669, 670, 671, 672, 673, 674, 675, 676, 677, 678, 679, 680, 681, 682, 683, 684, 685, 686, 687, 688, 689, 690, 691, 692, 693, 694, 695, 696, 697, 698, 699, 700, 701, 702, 703, 704, 705, 706, 707, 708, 709, 710, 711, 712, 713, 714, 715, 716, 717, 718, 719, 720, 721, 722, 723, 724, 725, 726, 727, 728, 729, 730, 731, 732, 733, 734, 735, 736, 737, 738, 739, 740, 741, 742, 743, 744, 745, 746, 747, 748, 749, 750, 751, 752, 753, 754, 755, 756, 757, 758, 759, 760, 761, 762, 763, 764, 765, 766, 767, 768, 769, 770, 771, 772, 773, 774, 775, 776, 777, 778, 779, 780, 781, 782, 783, 784, 785, 786, 787, 788, 789, 790, 791, 792, 793, 794, 795, 796, 797, 798, 799, 800, 801, 802, 803, 804, 805, 806, 807, 808, 809, 810, 811, 812, 813, 814, 815, 816, 817, 818, 819, 820, 821, 822, 823, 824, 825, 826, 827, 828, 829, 830, 831, 832, 833, 834, 835, 836, 837, 838, 839, 840, 841, 842, 843, 844, 845, 846, 847, 848, 849, 850, 851, 852, 853, 854, 855, 856, 857, 858, 859, 860, 861, 862, 863, 864, 865, 866, 867, 868, 869, 870, 871, 872, 873, 874, 875, 876, 877, 878, 879, 880, 881, 882, 883, 884, 885, 886, 887, 888, 889, 890, 891, 892, 893, 894, 895, 896, 897, 898, 899, 900, 901, 902, 903, 904, 905, 906, 907, 908, 909, 910, 911, 912, 913, 914, 915, 916, 917, 918, 919, 920, 921, 922, 923, 924, 925, 926, 927, 928, 929, 930, 931, 932, 933, 934, 935, 936, 937, 938, 939, 940, 941, 942, 943, 944, 945, 946, 947, 948, 949, 950, 951, 952, 953, 954, 955, 956, 957, 958, 959, 960, 961, 962, 963, 964, 965, 966, 967, 968, 969, 970, 971, 972, 973, 974, 975, 976, 977, 978, 979, 980, 981, 982, 983, 984, 985, 986, 987, 988, 989, 990, 991, 992, 993, 994, 995, 996, 997, 998, 999, 1000, 1001, 1002, 1003, 1004, 1005, 1006, 1007, 1008, 1009, 1010, 1011, 1012, 1013, 1014, 1015, 1016, 1017, 1018, 1019, 1020, 1021, 1022, 1023, 1024, 1025, 1026, 1027, 1028, 1029, 1030, 1031, 1032, 1033, 1034, 1035, 1036, 1037, 1038, 1039, 1040, 1041, 1042, 1043, 1044, 1045, 1046, 1047, 1048, 1049, 1050, 1051, 1052, 1053, 1054, 1055, 1056, 1057, 1058, 1059, 1060, 1061, 1062, 1063, 1064, 1065, 1066, 1067, 1068, 1069, 1070, 1071, 1072, 1073, 1074, 1075, 1076, 1077, 1078, 1079, 1080, 1081, 1082, 1083, 1084, 1085, 1086, 1087, 1088, 1089, 1090, 1091, 1092, 1093, 1094, 1095, 1096, 1097, 1098, 1099, 1100, 1101, 1102, 1103, 1104, 1105, 1106, 1107, 1108, 1109, 1110, 1111, 1112, 1113, 1114, 1115, 1116, 1117, 1118, 1119, 1120, 1121, 1122, 1123, 1124, 1125, 1126, 1127, 1128, 1129, 1130, 1131, 1132, 1133, 1134, 1135, 1136, 1137, 1138, 1139, 1140, 1141, 1142, 1143, 1144, 1145, 1146, 1147, 1148, 1149, 1150, 1151, 1152, 1153, 1154, 1155, 1156, 1157, 1158, 1159, 1160, 1161, 1162, 1163, 1164, 1165, 1166, 1167, 1168, 1169, 1170, 1171, 1172, 1173, 1174, 1175, 1176, 1177, 1178, 1179, 1180, 1181, 1182, 1183, 1184, 1185, 1186, 1187, 1188, 1189, 1190, 1191, 1192, 1193, 1194, 1195, 1196, 1197, 1198, 1199, 1200, 1201, 1202, 1203, 1204, 1205, 1206, 1207, 1208, 1209, 1210, 1211, 1212, 1213, 1214, 1215, 1216, 1217, 1218, 1219, 1220, 1221, 1222, 1223, 1224, 1225, 1226, 1227, 1228, 1229, 1230, 1231, 1232, 1233, 1234, 1235, 1236, 1237, 1238, 1239, 1240, 1241, 1242, 1243, 1244, 1245, 1246, 1247};
static const casadi_int casadi_s2[19] = {15, 1, 0, 15, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14};

/* open_mapping_f1_mpc_controller:(i0[20],i1[1248])->(o0[15]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_int i, j, k;
  casadi_real *rr, *ss, *tt;
  const casadi_real *cr, *cs;
  casadi_real *w0=w+1, w1, *w2=w+17, w3, w4, *w5=w+25, *w6=w+27, *w7=w+29, w8, w9, w10, w11, w12, w13, *w14=w+37, w15, w16, w17, w18, w19, w20, w21, w22, w23, w24, w25, *w26=w+50;
  /* #0: @0 = input[1][157] */
  casadi_copy(arg[1] ? arg[1]+333 : 0, 15, w0);
  /* #1: @1 = 0 */
  w1 = 0.;
  /* #2: @2 = input[1][0] */
  casadi_copy(arg[1], 6, w2);
  /* #3: @3 = @2[0] */
  for (rr=(&w3), ss=w2+0; ss!=w2+1; ss+=1) *rr++ = *ss;
  /* #4: @4 = @2[1] */
  for (rr=(&w4), ss=w2+1; ss!=w2+2; ss+=1) *rr++ = *ss;
  /* #5: @5 = vertcat(@3, @4) */
  rr=w5;
  *rr++ = w3;
  *rr++ = w4;
  /* #6: @6 = input[1][35] */
  casadi_copy(arg[1] ? arg[1]+91 : 0, 2, w6);
  /* #7: @6 = (@5-@6) */
  for (i=0, rr=w6, cr=w5, cs=w6; i<2; ++i) (*rr++)  = ((*cr++)-(*cs++));
  /* #8: @7 = @6' */
  casadi_copy(w6, 2, w7);
  /* #9: @1 = mac(@7,@6,@1) */
  for (i=0, rr=(&w1); i<1; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w7+j, tt=w6+i*2; k<2; ++k) *rr += ss[k*1]**tt++;
  /* #10: @3 = input[1][65] */
  w3 = arg[1] ? arg[1][151] : 0;
  /* #11: @1 = (@1-@3) */
  w1 -= w3;
  /* #12: @4 = 0 */
  w4 = 0.;
  /* #13: @7 = input[1][36] */
  casadi_copy(arg[1] ? arg[1]+93 : 0, 2, w7);
  /* #14: @7 = (@5-@7) */
  for (i=0, rr=w7, cr=w5, cs=w7; i<2; ++i) (*rr++)  = ((*cr++)-(*cs++));
  /* #15: @6 = @7' */
  casadi_copy(w7, 2, w6);
  /* #16: @4 = mac(@6,@7,@4) */
  for (i=0, rr=(&w4); i<1; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w6+j, tt=w7+i*2; k<2; ++k) *rr += ss[k*1]**tt++;
  /* #17: @4 = (@4-@3) */
  w4 -= w3;
  /* #18: @8 = 0 */
  w8 = 0.;
  /* #19: @6 = input[1][37] */
  casadi_copy(arg[1] ? arg[1]+95 : 0, 2, w6);
  /* #20: @5 = (@5-@6) */
  for (i=0, rr=w5, cs=w6; i<2; ++i) (*rr++) -= (*cs++);
  /* #21: @6 = @5' */
  casadi_copy(w5, 2, w6);
  /* #22: @8 = mac(@6,@5,@8) */
  for (i=0, rr=(&w8); i<1; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w6+j, tt=w5+i*2; k<2; ++k) *rr += ss[k*1]**tt++;
  /* #23: @8 = (@8-@3) */
  w8 -= w3;
  /* #24: @9 = 0 */
  w9 = 0.;
  /* #25: @10 = @2[0] */
  for (rr=(&w10), ss=w2+0; ss!=w2+1; ss+=1) *rr++ = *ss;
  /* #26: @11 = 0.1 */
  w11 = 1.0000000000000001e-01;
  /* #27: @6 = input[0][0] */
  casadi_copy(arg[0], 2, w6);
  /* #28: @12 = @6[0] */
  for (rr=(&w12), ss=w6+0; ss!=w6+1; ss+=1) *rr++ = *ss;
  /* #29: @11 = (@11*@12) */
  w11 *= w12;
  /* #30: @12 = @2[2] */
  for (rr=(&w12), ss=w2+2; ss!=w2+3; ss+=1) *rr++ = *ss;
  /* #31: @12 = cos(@12) */
  w12 = cos( w12 );
  /* #32: @11 = (@11*@12) */
  w11 *= w12;
  /* #33: @10 = (@10+@11) */
  w10 += w11;
  /* #34: @11 = @2[1] */
  for (rr=(&w11), ss=w2+1; ss!=w2+2; ss+=1) *rr++ = *ss;
  /* #35: @12 = 0.1 */
  w12 = 1.0000000000000001e-01;
  /* #36: @13 = @6[0] */
  for (rr=(&w13), ss=w6+0; ss!=w6+1; ss+=1) *rr++ = *ss;
  /* #37: @12 = (@12*@13) */
  w12 *= w13;
  /* #38: @13 = @2[2] */
  for (rr=(&w13), ss=w2+2; ss!=w2+3; ss+=1) *rr++ = *ss;
  /* #39: @13 = sin(@13) */
  w13 = sin( w13 );
  /* #40: @12 = (@12*@13) */
  w12 *= w13;
  /* #41: @11 = (@11+@12) */
  w11 += w12;
  /* #42: @5 = vertcat(@10, @11) */
  rr=w5;
  *rr++ = w10;
  *rr++ = w11;
  /* #43: @7 = input[1][38] */
  casadi_copy(arg[1] ? arg[1]+97 : 0, 2, w7);
  /* #44: @7 = (@5-@7) */
  for (i=0, rr=w7, cr=w5, cs=w7; i<2; ++i) (*rr++)  = ((*cr++)-(*cs++));
  /* #45: @14 = @7' */
  casadi_copy(w7, 2, w14);
  /* #46: @9 = mac(@14,@7,@9) */
  for (i=0, rr=(&w9); i<1; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w14+j, tt=w7+i*2; k<2; ++k) *rr += ss[k*1]**tt++;
  /* #47: @9 = (@9-@3) */
  w9 -= w3;
  /* #48: @12 = 0 */
  w12 = 0.;
  /* #49: @14 = input[1][39] */
  casadi_copy(arg[1] ? arg[1]+99 : 0, 2, w14);
  /* #50: @14 = (@5-@14) */
  for (i=0, rr=w14, cr=w5, cs=w14; i<2; ++i) (*rr++)  = ((*cr++)-(*cs++));
  /* #51: @7 = @14' */
  casadi_copy(w14, 2, w7);
  /* #52: @12 = mac(@7,@14,@12) */
  for (i=0, rr=(&w12); i<1; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w7+j, tt=w14+i*2; k<2; ++k) *rr += ss[k*1]**tt++;
  /* #53: @12 = (@12-@3) */
  w12 -= w3;
  /* #54: @13 = 0 */
  w13 = 0.;
  /* #55: @7 = input[1][40] */
  casadi_copy(arg[1] ? arg[1]+101 : 0, 2, w7);
  /* #56: @5 = (@5-@7) */
  for (i=0, rr=w5, cs=w7; i<2; ++i) (*rr++) -= (*cs++);
  /* #57: @7 = @5' */
  casadi_copy(w5, 2, w7);
  /* #58: @13 = mac(@7,@5,@13) */
  for (i=0, rr=(&w13); i<1; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w7+j, tt=w5+i*2; k<2; ++k) *rr += ss[k*1]**tt++;
  /* #59: @13 = (@13-@3) */
  w13 -= w3;
  /* #60: @15 = 0 */
  w15 = 0.;
  /* #61: @16 = 0.1 */
  w16 = 1.0000000000000001e-01;
  /* #62: @7 = input[0][1] */
  casadi_copy(arg[0] ? arg[0]+2 : 0, 2, w7);
  /* #63: @17 = @7[0] */
  for (rr=(&w17), ss=w7+0; ss!=w7+1; ss+=1) *rr++ = *ss;
  /* #64: @16 = (@16*@17) */
  w16 *= w17;
  /* #65: @17 = @2[2] */
  for (rr=(&w17), ss=w2+2; ss!=w2+3; ss+=1) *rr++ = *ss;
  /* #66: @18 = 0.1 */
  w18 = 1.0000000000000001e-01;
  /* #67: @19 = @6[1] */
  for (rr=(&w19), ss=w6+1; ss!=w6+2; ss+=1) *rr++ = *ss;
  /* #68: @18 = (@18*@19) */
  w18 *= w19;
  /* #69: @17 = (@17+@18) */
  w17 += w18;
  /* #70: @18 = cos(@17) */
  w18 = cos( w17 );
  /* #71: @16 = (@16*@18) */
  w16 *= w18;
  /* #72: @10 = (@10+@16) */
  w10 += w16;
  /* #73: @16 = 0.1 */
  w16 = 1.0000000000000001e-01;
  /* #74: @18 = @7[0] */
  for (rr=(&w18), ss=w7+0; ss!=w7+1; ss+=1) *rr++ = *ss;
  /* #75: @16 = (@16*@18) */
  w16 *= w18;
  /* #76: @18 = sin(@17) */
  w18 = sin( w17 );
  /* #77: @16 = (@16*@18) */
  w16 *= w18;
  /* #78: @11 = (@11+@16) */
  w11 += w16;
  /* #79: @6 = vertcat(@10, @11) */
  rr=w6;
  *rr++ = w10;
  *rr++ = w11;
  /* #80: @5 = input[1][41] */
  casadi_copy(arg[1] ? arg[1]+103 : 0, 2, w5);
  /* #81: @5 = (@6-@5) */
  for (i=0, rr=w5, cr=w6, cs=w5; i<2; ++i) (*rr++)  = ((*cr++)-(*cs++));
  /* #82: @14 = @5' */
  casadi_copy(w5, 2, w14);
  /* #83: @15 = mac(@14,@5,@15) */
  for (i=0, rr=(&w15); i<1; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w14+j, tt=w5+i*2; k<2; ++k) *rr += ss[k*1]**tt++;
  /* #84: @15 = (@15-@3) */
  w15 -= w3;
  /* #85: @16 = 0 */
  w16 = 0.;
  /* #86: @14 = input[1][42] */
  casadi_copy(arg[1] ? arg[1]+105 : 0, 2, w14);
  /* #87: @14 = (@6-@14) */
  for (i=0, rr=w14, cr=w6, cs=w14; i<2; ++i) (*rr++)  = ((*cr++)-(*cs++));
  /* #88: @5 = @14' */
  casadi_copy(w14, 2, w5);
  /* #89: @16 = mac(@5,@14,@16) */
  for (i=0, rr=(&w16); i<1; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w5+j, tt=w14+i*2; k<2; ++k) *rr += ss[k*1]**tt++;
  /* #90: @16 = (@16-@3) */
  w16 -= w3;
  /* #91: @18 = 0 */
  w18 = 0.;
  /* #92: @5 = input[1][43] */
  casadi_copy(arg[1] ? arg[1]+107 : 0, 2, w5);
  /* #93: @6 = (@6-@5) */
  for (i=0, rr=w6, cs=w5; i<2; ++i) (*rr++) -= (*cs++);
  /* #94: @5 = @6' */
  casadi_copy(w6, 2, w5);
  /* #95: @18 = mac(@5,@6,@18) */
  for (i=0, rr=(&w18); i<1; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w5+j, tt=w6+i*2; k<2; ++k) *rr += ss[k*1]**tt++;
  /* #96: @18 = (@18-@3) */
  w18 -= w3;
  /* #97: @19 = 0 */
  w19 = 0.;
  /* #98: @20 = 0.1 */
  w20 = 1.0000000000000001e-01;
  /* #99: @5 = input[0][2] */
  casadi_copy(arg[0] ? arg[0]+4 : 0, 2, w5);
  /* #100: @21 = @5[0] */
  for (rr=(&w21), ss=w5+0; ss!=w5+1; ss+=1) *rr++ = *ss;
  /* #101: @20 = (@20*@21) */
  w20 *= w21;
  /* #102: @21 = 0.1 */
  w21 = 1.0000000000000001e-01;
  /* #103: @22 = @7[1] */
  for (rr=(&w22), ss=w7+1; ss!=w7+2; ss+=1) *rr++ = *ss;
  /* #104: @21 = (@21*@22) */
  w21 *= w22;
  /* #105: @17 = (@17+@21) */
  w17 += w21;
  /* #106: @21 = cos(@17) */
  w21 = cos( w17 );
  /* #107: @20 = (@20*@21) */
  w20 *= w21;
  /* #108: @10 = (@10+@20) */
  w10 += w20;
  /* #109: @20 = 0.1 */
  w20 = 1.0000000000000001e-01;
  /* #110: @21 = @5[0] */
  for (rr=(&w21), ss=w5+0; ss!=w5+1; ss+=1) *rr++ = *ss;
  /* #111: @20 = (@20*@21) */
  w20 *= w21;
  /* #112: @21 = sin(@17) */
  w21 = sin( w17 );
  /* #113: @20 = (@20*@21) */
  w20 *= w21;
  /* #114: @11 = (@11+@20) */
  w11 += w20;
  /* #115: @7 = vertcat(@10, @11) */
  rr=w7;
  *rr++ = w10;
  *rr++ = w11;
  /* #116: @6 = input[1][44] */
  casadi_copy(arg[1] ? arg[1]+109 : 0, 2, w6);
  /* #117: @6 = (@7-@6) */
  for (i=0, rr=w6, cr=w7, cs=w6; i<2; ++i) (*rr++)  = ((*cr++)-(*cs++));
  /* #118: @14 = @6' */
  casadi_copy(w6, 2, w14);
  /* #119: @19 = mac(@14,@6,@19) */
  for (i=0, rr=(&w19); i<1; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w14+j, tt=w6+i*2; k<2; ++k) *rr += ss[k*1]**tt++;
  /* #120: @19 = (@19-@3) */
  w19 -= w3;
  /* #121: @20 = 0 */
  w20 = 0.;
  /* #122: @14 = input[1][45] */
  casadi_copy(arg[1] ? arg[1]+111 : 0, 2, w14);
  /* #123: @14 = (@7-@14) */
  for (i=0, rr=w14, cr=w7, cs=w14; i<2; ++i) (*rr++)  = ((*cr++)-(*cs++));
  /* #124: @6 = @14' */
  casadi_copy(w14, 2, w6);
  /* #125: @20 = mac(@6,@14,@20) */
  for (i=0, rr=(&w20); i<1; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w6+j, tt=w14+i*2; k<2; ++k) *rr += ss[k*1]**tt++;
  /* #126: @20 = (@20-@3) */
  w20 -= w3;
  /* #127: @21 = 0 */
  w21 = 0.;
  /* #128: @6 = input[1][46] */
  casadi_copy(arg[1] ? arg[1]+113 : 0, 2, w6);
  /* #129: @7 = (@7-@6) */
  for (i=0, rr=w7, cs=w6; i<2; ++i) (*rr++) -= (*cs++);
  /* #130: @6 = @7' */
  casadi_copy(w7, 2, w6);
  /* #131: @21 = mac(@6,@7,@21) */
  for (i=0, rr=(&w21); i<1; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w6+j, tt=w7+i*2; k<2; ++k) *rr += ss[k*1]**tt++;
  /* #132: @21 = (@21-@3) */
  w21 -= w3;
  /* #133: @22 = 0 */
  w22 = 0.;
  /* #134: @23 = 0.1 */
  w23 = 1.0000000000000001e-01;
  /* #135: @6 = input[0][3] */
  casadi_copy(arg[0] ? arg[0]+6 : 0, 2, w6);
  /* #136: @24 = @6[0] */
  for (rr=(&w24), ss=w6+0; ss!=w6+1; ss+=1) *rr++ = *ss;
  /* #137: @23 = (@23*@24) */
  w23 *= w24;
  /* #138: @24 = 0.1 */
  w24 = 1.0000000000000001e-01;
  /* #139: @25 = @5[1] */
  for (rr=(&w25), ss=w5+1; ss!=w5+2; ss+=1) *rr++ = *ss;
  /* #140: @24 = (@24*@25) */
  w24 *= w25;
  /* #141: @17 = (@17+@24) */
  w17 += w24;
  /* #142: @24 = cos(@17) */
  w24 = cos( w17 );
  /* #143: @23 = (@23*@24) */
  w23 *= w24;
  /* #144: @10 = (@10+@23) */
  w10 += w23;
  /* #145: @23 = 0.1 */
  w23 = 1.0000000000000001e-01;
  /* #146: @24 = @6[0] */
  for (rr=(&w24), ss=w6+0; ss!=w6+1; ss+=1) *rr++ = *ss;
  /* #147: @23 = (@23*@24) */
  w23 *= w24;
  /* #148: @17 = sin(@17) */
  w17 = sin( w17 );
  /* #149: @23 = (@23*@17) */
  w23 *= w17;
  /* #150: @11 = (@11+@23) */
  w11 += w23;
  /* #151: @6 = vertcat(@10, @11) */
  rr=w6;
  *rr++ = w10;
  *rr++ = w11;
  /* #152: @5 = input[1][47] */
  casadi_copy(arg[1] ? arg[1]+115 : 0, 2, w5);
  /* #153: @5 = (@6-@5) */
  for (i=0, rr=w5, cr=w6, cs=w5; i<2; ++i) (*rr++)  = ((*cr++)-(*cs++));
  /* #154: @7 = @5' */
  casadi_copy(w5, 2, w7);
  /* #155: @22 = mac(@7,@5,@22) */
  for (i=0, rr=(&w22); i<1; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w7+j, tt=w5+i*2; k<2; ++k) *rr += ss[k*1]**tt++;
  /* #156: @22 = (@22-@3) */
  w22 -= w3;
  /* #157: @10 = 0 */
  w10 = 0.;
  /* #158: @7 = input[1][48] */
  casadi_copy(arg[1] ? arg[1]+117 : 0, 2, w7);
  /* #159: @7 = (@6-@7) */
  for (i=0, rr=w7, cr=w6, cs=w7; i<2; ++i) (*rr++)  = ((*cr++)-(*cs++));
  /* #160: @5 = @7' */
  casadi_copy(w7, 2, w5);
  /* #161: @10 = mac(@5,@7,@10) */
  for (i=0, rr=(&w10); i<1; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w5+j, tt=w7+i*2; k<2; ++k) *rr += ss[k*1]**tt++;
  /* #162: @10 = (@10-@3) */
  w10 -= w3;
  /* #163: @11 = 0 */
  w11 = 0.;
  /* #164: @5 = input[1][49] */
  casadi_copy(arg[1] ? arg[1]+119 : 0, 2, w5);
  /* #165: @6 = (@6-@5) */
  for (i=0, rr=w6, cs=w5; i<2; ++i) (*rr++) -= (*cs++);
  /* #166: @5 = @6' */
  casadi_copy(w6, 2, w5);
  /* #167: @11 = mac(@5,@6,@11) */
  for (i=0, rr=(&w11); i<1; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w5+j, tt=w6+i*2; k<2; ++k) *rr += ss[k*1]**tt++;
  /* #168: @11 = (@11-@3) */
  w11 -= w3;
  /* #169: @26 = vertcat(@1, @4, @8, @9, @12, @13, @15, @16, @18, @19, @20, @21, @22, @10, @11) */
  rr=w26;
  *rr++ = w1;
  *rr++ = w4;
  *rr++ = w8;
  *rr++ = w9;
  *rr++ = w12;
  *rr++ = w13;
  *rr++ = w15;
  *rr++ = w16;
  *rr++ = w18;
  *rr++ = w19;
  *rr++ = w20;
  *rr++ = w21;
  *rr++ = w22;
  *rr++ = w10;
  *rr++ = w11;
  /* #170: @0 = (@0*@26) */
  for (i=0, rr=w0, cs=w26; i<15; ++i) (*rr++) *= (*cs++);
  /* #171: output[0][0] = @0 */
  casadi_copy(w0, 15, res[0]);
  return 0;
}

CASADI_SYMBOL_EXPORT int open_mapping_f1_mpc_controller(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int open_mapping_f1_mpc_controller_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int open_mapping_f1_mpc_controller_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void open_mapping_f1_mpc_controller_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int open_mapping_f1_mpc_controller_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void open_mapping_f1_mpc_controller_release(int mem) {
}

CASADI_SYMBOL_EXPORT void open_mapping_f1_mpc_controller_incref(void) {
}

CASADI_SYMBOL_EXPORT void open_mapping_f1_mpc_controller_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int open_mapping_f1_mpc_controller_n_in(void) { return 2;}

CASADI_SYMBOL_EXPORT casadi_int open_mapping_f1_mpc_controller_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real open_mapping_f1_mpc_controller_default_in(casadi_int i) {
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* open_mapping_f1_mpc_controller_name_in(casadi_int i) {
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* open_mapping_f1_mpc_controller_name_out(casadi_int i) {
  switch (i) {
    case 0: return "o0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* open_mapping_f1_mpc_controller_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* open_mapping_f1_mpc_controller_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s2;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int open_mapping_f1_mpc_controller_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 17;
  if (sz_res) *sz_res = 2;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 65;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
