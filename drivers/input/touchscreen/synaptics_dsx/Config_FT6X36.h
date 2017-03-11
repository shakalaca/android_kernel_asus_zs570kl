/************************************************************************
* Copyright (C) 2012-2015, Focaltech Systems (R)£¬All Rights Reserved.
*
* File Name: Config_FT6X36.h
*
* Author: Software Development Team, AE
*
* Created: 2015-10-08
*
* Abstract: Set Config for FT6X36/FT3X07/FT6416/FT6426
*
************************************************************************/
#ifndef _CONFIG_FT6X36_H
#define _CONFIG_FT6X36_H

#include "test_lib.h"

/*-----------------------------------------------
FT6X36
------------------------------------------------*/
struct stCfg_FT6X36_TestItem
{
	unsigned char FW_VERSION_TEST;
	unsigned char FACTORY_ID_TEST;
	unsigned char PROJECT_CODE_TEST;
	unsigned char IC_VERSION_TEST;
	unsigned char RAWDATA_TEST;
	unsigned char CHANNEL_NUM_TEST;
	unsigned char CHANNEL_SHORT_TEST;
	unsigned char INT_PIN_TEST;
	unsigned char RESET_PIN_TEST;
	unsigned char NOISE_TEST;
	unsigned char CB_TEST;
	unsigned char DELTA_CB_TEST;
	unsigned char CHANNELS_DEVIATION_TEST;
	unsigned char TWO_SIDES_DEVIATION_TEST;
	unsigned char FPC_SHORT_TEST;
	unsigned char FPC_OPEN_TEST;
	unsigned char SREF_OPEN_TEST;
	unsigned char TE_TEST;
	unsigned char CB_DEVIATION_TEST;
	unsigned char DIFFER_TEST;
	unsigned char WEAK_SHORT_TEST;
	unsigned char DIFFER_TEST2;
	unsigned char K1_DIFFER_TEST;
};

struct stCfg_FT6X36_BasicThreshold
{
	BYTE FW_VER_VALUE;
	BYTE Factory_ID_Number;
	char Project_Code[32];
	BYTE IC_Version;
	int RawDataTest_Min;
	int RawDataTest_Max;
	BYTE ChannelNumTest_ChannelNum;
	BYTE ChannelNumTest_KeyNum;
	int ChannelShortTest_K1;
	int ChannelShortTest_K2;
	int ChannelShortTest_CB;
	BYTE ResetPinTest_RegAddr;
	BYTE IntPinTest_RegAddr;
	int WeakShortThreshold;
	int NoiseTest_Max;
	int NoiseTest_Frames;
	int NoiseTest_Time;
	BYTE NoiseTest_SampeMode;
	BYTE NoiseTest_NoiseMode;
	BYTE NoiseTest_ShowTip;
	int FPCShort_CB_Min;
	int FPCShort_CB_Max;
	int FPCShort_RawData_Min;
	int FPCShort_RawData_Max;
	int FPCOpen_CB_Min;
	int FPCOpen_CB_Max;
	int FPCOpen_RawData_Min;
	int FPCOpen_RawData_Max;
	int SREFOpen_Hole_Base1;
	int SREFOpen_Hole_Base2;
	int SREFOpen_Hole;
	int CBDeviationTest_Hole;
	int Differ_Ave_Hole;
	int Differ_Max_Hole;
	int CbTest_Min;
	int CbTest_Max;
	int DeltaCbTest_Base;
	int DeltaCbTest_Differ_Max;
	unsigned char DeltaCbTest_Include_Key_Test;
	int DeltaCbTest_Key_Differ_Max;
	int DeltaCbTest_Deviation_S1;
	int DeltaCbTest_Deviation_S2;
	int DeltaCbTest_Deviation_S3;
	int DeltaCbTest_Deviation_S4;
	int DeltaCbTest_Deviation_S5;
	int DeltaCbTest_Deviation_S6;
	unsigned char DeltaCbTest_Set_Critical;
	int DeltaCbTest_Critical_S1;
	int DeltaCbTest_Critical_S2;
	int DeltaCbTest_Critical_S3;
	int DeltaCbTest_Critical_S4;
	int DeltaCbTest_Critical_S5;
	int DeltaCbTest_Critical_S6;

	int ChannelsDeviationTest_Deviation_S1;
	int ChannelsDeviationTest_Deviation_S2;
	int ChannelsDeviationTest_Deviation_S3;
	int ChannelsDeviationTest_Deviation_S4;
	int ChannelsDeviationTest_Deviation_S5;
	int ChannelsDeviationTest_Deviation_S6;
	unsigned char ChannelsDeviationTest_Set_Critical;
	int ChannelsDeviationTest_Critical_S1;
	int ChannelsDeviationTest_Critical_S2;
	int ChannelsDeviationTest_Critical_S3;
	int ChannelsDeviationTest_Critical_S4;
	int ChannelsDeviationTest_Critical_S5;
	int ChannelsDeviationTest_Critical_S6;

	int TwoSidesDeviationTest_Deviation_S1;
	int TwoSidesDeviationTest_Deviation_S2;
	int TwoSidesDeviationTest_Deviation_S3;
	int TwoSidesDeviationTest_Deviation_S4;
	int TwoSidesDeviationTest_Deviation_S5;
	int TwoSidesDeviationTest_Deviation_S6;
	unsigned char TwoSidesDeviationTest_Set_Critical;
	int TwoSidesDeviationTest_Critical_S1;
	int TwoSidesDeviationTest_Critical_S2;
	int TwoSidesDeviationTest_Critical_S3;
	int TwoSidesDeviationTest_Critical_S4;
	int TwoSidesDeviationTest_Critical_S5;
	int TwoSidesDeviationTest_Critical_S6;

	int DifferTest2_Data_H_Min;
	int DifferTest2_Data_H_Max;
	int DifferTest2_Data_M_Min;
	int DifferTest2_Data_M_Max;
	int DifferTest2_Data_L_Min;
	int DifferTest2_Data_L_Max;
	unsigned char bDifferTest2_Data_H;
	unsigned char bDifferTest2_Data_M;
	unsigned char bDifferTest2_Data_L;
	int  K1DifferTest_StartK1;
	int  K1DifferTest_EndK1;
	int  K1DifferTest_MinHold2;
	int  K1DifferTest_MaxHold2;
	int  K1DifferTest_MinHold4;
	int  K1DifferTest_MaxHold4;
	int  K1DifferTest_Deviation2;
	int  K1DifferTest_Deviation4;
};

enum enumTestItem_FT6X36
{
	Code_FT6X36_ENTER_FACTORY_MODE,//All IC have to process test item
	Code_FT6X36_DOWNLOAD,//All IC have to process test item
	Code_FT6X36_UPGRADE,//All IC have to process test item
	Code_FT6X36_FACTORY_ID_TEST,
	Code_FT6X36_PROJECT_CODE_TEST,
	Code_FT6X36_FW_VERSION_TEST,
	Code_FT6X36_IC_VERSION_TEST,
	Code_FT6X36_RAWDATA_TEST,
	Code_FT6X36_CHANNEL_NUM_TEST,
	Code_FT6X36_CHANNEL_SHORT_TEST,
	Code_FT6X36_INT_PIN_TEST,
	Code_FT6X36_RESET_PIN_TEST,
	Code_FT6X36_NOISE_TEST,
	Code_FT6X36_CB_TEST,
	Code_FT6X36_DELTA_CB_TEST,
	Code_FT6X36_CHANNELS_DEVIATION_TEST,
	Code_FT6X36_TWO_SIDES_DEVIATION_TEST,
	Code_FT6X36_FPC_SHORT_TEST,
	Code_FT6X36_FPC_OPEN_TEST,
	Code_FT6X36_SREF_OPEN_TEST,
	Code_FT6X36_TE_TEST,
	Code_FT6X36_CB_DEVIATION_TEST,
	Code_FT6X36_WRITE_CONFIG,//All IC have to process test item
	Code_FT6X36_DIFFER_TEST,
	Code_FT6X36_WEAK_SHORT_TEST,
	Code_FT6X36_DIFFER_TEST2,
	Code_FT6X36_K1_DIFFER_TEST,
};

extern struct stCfg_FT6X36_TestItem g_stCfg_FT6X36_TestItem;
extern struct stCfg_FT6X36_BasicThreshold g_stCfg_FT6X36_BasicThreshold;

void OnInit_FT6X36_TestItem(char *strIniFile);
void OnInit_FT6X36_BasicThreshold(char *strIniFile);
void SetTestItem_FT6X36(void);

#endif
