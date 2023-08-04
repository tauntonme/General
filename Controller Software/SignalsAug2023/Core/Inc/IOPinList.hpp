/*
 * IOPinList.hpp
 *
 *  Created on: Jul 3, 2023
 *      Author: Jon Freeman
 *
 *      Here is where circuit board input and output connections get assigned
 *
 *      Input and Output polarity is selected using INV (inverted) or !INV (not inverted)
 *
 */
#ifndef INC_IOPINLIST_HPP_
#define INC_IOPINLIST_HPP_

enum	InputNumbers	{Inp00, Inp01, Inp02, Inp03, Inp04, Inp05, Inp06, Inp07, Inp08, Inp09,
						 Inp10, Inp11, Inp12, Inp13, Inp14, Inp15, Inp16, Inp17, Inp18, Inp19,
						 Inp20, Inp21, Inp22, Inp23, Inp24, Inp25, Inp26, Inp27, Inp28, Inp29,
						 Inp30, Inp31, Inp32, Inp33, Inp34, Inp35, Inp36, Inp37, Inp38, Inp39,
						 Inp40, Inp41, Inp42, Inp43, Inp44, Inp45, Inp46, Inp47, Inp48, Inp49,
						 Inp50, Inp51, Inp52, Inp53, Inp54, Inp55, Inp56, Inp57, Inp58, Inp59,
						 Inp60, Inp61, Inp62, Inp63, Inp64, Inp65, Inp66, Inp67, Inp68, Inp69,
						}	;

enum	OutputNumbers	{Out00 /*= 1000*/, Out01, Out02, Out03, Out04, Out05, Out06, Out07, Out08, Out09,
						 Out10, Out11, Out12, Out13, Out14, Out15, Out16, Out17, Out18, Out19,
						 Out20, Out21, Out22, Out23, Out24, Out25, Out26, Out27, Out28, Out29,
						 Out30, Out31, Out32, Out33, Out34, Out35, Out36, Out37, Out38, Out39,
						 Out40, Out41, Out42, Out43, Out44, Out45, Out46, Out47, Out48, Out49,
						 Out50, Out51, Out52, Out53, Out54, Out55, Out56, Out57, Out58, Out59,
						 Out60, Out61, Out62, Out63, Out64, Out65, Out66, Out67, Out68, Out69,
						}	;


#define	LEVEL_CROSSING_OUT1			Out01, INV
#define	LEVEL_CROSSING_OUT2			Out02, INV
#define	LEVEL_CROSSING_GATE1_INS	Inp01, !INV, Inp02, !INV
#define	LEVEL_CROSSING_GATE2_INS	Inp03, !INV, Inp04, !INV
#define	LEVEL_CROSSING_GATE3_INS	Inp05, !INV, Inp06, !INV
#define	LEVEL_CROSSING_GATE4_INS	Inp07, !INV, Inp08, !INV
#define	LC_TRAFFIC_LIGHT_INS_OUTS	Inp08, Inp09, Out03, Out04		//	inputs 8,9, outputs 3,4
#define	LC_LOCK1_INS				Inp10, !INV, Inp11, !INV
#define	LC_LOCK2_INS				Inp12, !INV, Inp13, !INV
#define	LC_BUTTON1_IN				Inp14, !INV
#define	LC_BUTTON2_IN				Inp15, !INV
#define	LC_ALARM					Out28, !INV


//  RailwaySignal
#define	SIG_DN_MAIN_DIST_PINLIST			Inp01, Inp02,	Out01, Out02
#define	SIG_DN_MAIN_OUTER_HOME_PINLIST		Inp03, Inp04,	Out03, Out04
#define	SIG_DN_MAIN_HOME_PINLIST			Inp05, Inp06,	Out05, Out06
#define	SIG_DN_MAIN_PLATFORM_HOME_PINLIST	Inp07, Inp08,	Out07, Out08
#define	SIG_DISC_DN_MAIN_BACKING_PINLIST	Inp09, Inp10,	Out09, Out10
#define	SIG_STEAM_BAY_EXIT_PINLIST			Inp11, Inp12,	Out11, Out12
#define	SIG_DISC_UP_MAIN_BACKING_PINLIST	Inp13, Inp14,	Out13, Out14
#define	SIG_UP_LOOP_TO_SHED_PINLIST			Inp15, Inp16,	Out15, Out16
#define	SIG_EXIT_FROM_SHED_PINLIST			Inp17, Inp18,	Out17, Out18
#define	SIG_UP_LOOP_UP_MAIN_START_PINLIST	Inp19, Inp20,	Out19, Out20
#define	SIG_UP_LOOP_HOME_PINLIST			Inp21, Inp22,	Out21, Out22
#define	SIG_UP_MAIN_START_PINLIST			Inp23, Inp24,	Out23, Out24
#define	SIG_UP_MAIN_HOME_PINLIST			Inp25, Inp26,	Out25, Out26

#define	POINTS_FROM_TURNTABLE_PINLIST		Inp27,	true,	Inp28,	true, Out27, true, Out28, true
#define	POINTS_DN_MAIN_TO_PLATFORM_PINLIST	Inp29,	true,	Inp30,	true, Out29, true, Out30, true
#define	XOVR_STEAMING_BAYS_PINLIST			Inp31,	Inp32,true,true,Inp33,	Inp34,	true,true,	Out31, Out32, true,true
#define	XOVR_UP_LOOP_UP_MAIN_PINLIST		Inp35,	Inp36,true,true,Inp37,	Inp38,	true,true,	Out33, Out34, true,true





enum	Ground_Lev_Railway_Kit_Of_Parts	{
	//	List of all Railway items of interest to the signalling system for the ground level railway at West Buckland.
	//		Level Crossing, 1 off
	//		Signals, 13 off
	//		Turnouts a.k.a Points, 6 off including two paired 'Crossover' sets
	//		Track Sections, many.
	//		Push Buttons, ?
	//		Alarms, ?
	//		Timers		often e.g. a useful alarm component
	//		Registers	(bit like SR flip flops)

//	Each item is modelled as an "Object" in C++ code.
//	This makes possible a clear 'Plain English'-like coding style.

//	Note Level Crossing has multiple parts, all dealt with elsewhere in dedicated 'Level_Crossing' controller.

	West_Buck_Level_Crossing,	//	One of these

//	Lineside railway signals (for non-confusion with electrical or any other kind of 'signal')
//	Names of Lineside Railway Signals :
//enum	LinesideRailwaySignals	{
/*	Sig_Down_Main_Distant,
	Sig_Down_Main_Outer_Home,
	Sig_Down_Main_Home,
	Sig_Down_Main_To_Platform_Home,
	Sig_Disc_Signal_Down_Main_Backing,
	Sig_Steaming_Bays_Exit,
	Sig_Disc_Up_Main_Backing,
	Sig_Up_Loop_To_Carriage_Shed,
	Sig_Exit_From_Carriage_Shed,
	Sig_Up_Loop_To_Up_Main_Start,
	Sig_Up_Loop_Home,
	Sig_Up_Main_Start,
	Sig_Up_Main_Home,
//}	;
*/
//	'Turnouts' (as D.H. calls them) or 'Points' to everyone else
//	We have two single and two paired sets.
//	Paired sets are called 'Crossovers' ? Is this correct ?

//enum	RailwayTurnouts	{
/*	Turnout_From_Turntable,		//	Manually op, but include in case signalling system needs to know setting
	Turnout_Down_Main_To_Platform,
	Crossover_To_Steaming_Bays,
	Crossover_Up_Loop_To_Up_Main,
//}	;
*/

	//enum	PushButtons	{	//	Note Level Crossing has four of these
		Push_Button_Platform_Whatever,
		Push_Button_Something_Else,

	//enum	Alarm	{	//	Note Level Crossing has one of these

		//	Railway Track sections have two ends - 'A' and 'B'
//	AA.A is one end of section 'AA'. AA.B is the other end of section 'AA'
//	Presumably we fit motion / train detectors where any track section connects to anything else ?
//enum	Railway_Track_Sections	{
	AA, AB, AC, AD, AE, AF,	//	Surely we can come up with something more meaningful than this!
};

#endif /* INC_IOPINLIST_HPP_ */
