/*
 * ForeverLoop.cpp
 *
 *  Created on: Jun 6, 2023
 *      Author: Jon Freeman B Eng Hons
 */
#include 	"main.h"

#include	"Serial.hpp"
#include	"RailwayHardware.hpp"
#include	"settings.hpp"
#include	"CmdLine.hpp"
#include	"IOPinList.hpp"

extern	void	check_commands	()	;	//	Looks after all command line input

//extern	"C"	bool	set_output_bit	(uint32_t which_out, bool hiorlo)	;
//extern	"C"	bool	get_ins_history_majority	(uint32_t which_in, uint32_t how_many)	;
extern	"C"	bool	get_raw_input_bytes	(char * dest)	;

using namespace std;

extern	uint32_t	millisecs;

	DualInput	Freda	(LEVEL_CROSSING_GATE1_INS)	;
//	DualInput	Freda	(2 ,false, 3, true)	;

//MovingPart	gate97	(6, INVERTED, 7, !INVERTED);


/**
 * 		ClassLevelCrossing	Level_Crossing	(
 *
 * 		This is where the Level Crossing object is created
 */
ClassLevelCrossing	Level_Crossing	(
		LEVEL_CROSSING_GATE1_INS,	//	gate1 input numbers, 'true' for switch normally open contacts
		LEVEL_CROSSING_GATE2_INS, 	//	likewise gate 2
		LEVEL_CROSSING_GATE3_INS, 	//	and gate 3
		LEVEL_CROSSING_GATE4_INS,	//	and gate 4
		LC_LOCK1_INS, 	//	lock1
		LC_LOCK2_INS,	//	lock2
		LC_TRAFFIC_LIGHT_INS_OUTS,	//	traffic light IO numbers
		LC_BUTTON1_IN, 			//	button1
		LC_BUTTON1_IN,			//	button2
		LC_ALARM,				//		Out23,	true,				//	alarm
		Out03, Out08, false			//	actuators outputs
		);					//	I make that 18 wires connected to 'LevelCrossing', 4 outs, 14 ins


//	Create the set of 'LinesideSignal' objects, all wired up as specified in IOPinList.hpp
  LinesideSignal
  	  Sig_Dn_Main_Distant				(SIG_DN_MAIN_DIST_PINLIST),
	  Sig_Dn_Main_Outer_Home			(SIG_DN_MAIN_OUTER_HOME_PINLIST),
	  Sig_Dn_Main_Home					(SIG_DN_MAIN_HOME_PINLIST),
	  Sig_Dn_Main_To_Platform_Home		(SIG_DN_MAIN_PLATFORM_HOME_PINLIST),
	  Sig_Disc_Dn_Main_Backing			(SIG_DISC_DN_MAIN_BACKING_PINLIST),
	  Sig_Steaming_Bays_Exit			(SIG_STEAM_BAY_EXIT_PINLIST),
	  Sig_Disc_Up_Main_Backing			(SIG_DISC_UP_MAIN_BACKING_PINLIST),
	  Sig_Up_Loop_To_Carriage_Shed		(SIG_UP_LOOP_TO_SHED_PINLIST),
	  Sig_Exit_From_Carriage_Shed		(SIG_EXIT_FROM_SHED_PINLIST),
	  Sig_Up_Loop_To_Up_Main_Start		(SIG_UP_LOOP_UP_MAIN_START_PINLIST),
	  Sig_Up_Loop_Home					(SIG_UP_LOOP_HOME_PINLIST),
	  Sig_Up_Main_Start					(SIG_UP_MAIN_START_PINLIST),
	  Sig_Up_Main_Home					(SIG_UP_MAIN_HOME_PINLIST);

//	Create the set of 'Points', or 'Turnout' objects, all wired up as specified in IOPinList.hpp
  Points
	Turnout_From_Turntable				(POINTS_FROM_TURNTABLE_PINLIST),
	//	Above Manually op, but include in case signalling system needs to know setting
	Turnout_Dn_Main_To_Platform			(POINTS_DN_MAIN_TO_PLATFORM_PINLIST);
  DualPoints
	Xover_To_Steaming_Bays			(XOVR_STEAMING_BAYS_PINLIST),
	Xover_Up_Loop_To_Up_Main		(XOVR_UP_LOOP_UP_MAIN_PINLIST);





	PulseAlarm	Al1	(Out07, true);
	PulseAlarm	Al2	(Out08, true);
int i = 0, k = -10, ln;

bool toggle = true, oncelist = true;
char	rx[12];

extern	UartComChan	pc;
extern	bool	can_flag;
extern	void	ce_show	()	;

CAN_TxHeaderTypeDef   CANTxHeader;
uint8_t               CANTxData[8] = {0};
uint32_t              CANTxMailbox,	can_errors = 0, testindex = 0;
uint32_t	test_addrs[] = {0x0123, 0x0466, 0x0700, 0x0704, 0x755, 0x7ff, 0};

extern	CAN_HandleTypeDef hcan1;

void	can_try	()	{	//	Test CAN bus
	static int tog = 0;
	tog++;
//	if	(!can_flag)
//		return;
	can_flag = false;
	CANTxHeader.IDE = CAN_ID_STD;	//	CAN_ID_STD means that we are using the Standard ID (not extended)
//	CANTxHeader.IDE = CAN_ID_EXT;	//	CAN_ID_STD means that we are using the Standard ID (not extended)

	//	CANTxHeader.StdId = 0x446;		//	0x446 is the Identifier. This is the ID of the Transmitter, and it should be maximum 11 bit wide
	CANTxHeader.StdId = test_addrs[testindex++];		//	0x446 is the Identifier. This is the ID of the Transmitter, and it should be maximum 11 bit wide
	if	(testindex >= sizeof(test_addrs) / sizeof(uint32_t))	testindex = 0;
	if	(tog & 1)
		CANTxHeader.RTR = CAN_RTR_DATA;	//	CAN_RTR_DATA indicates that we are sending a data frame
	else
		CANTxHeader.RTR = CAN_RTR_REMOTE;	//	CAN_RTR_REMOTE indicates that we are requesting remote frame ?
	CANTxHeader.DLC = 3;			//	DLC is the Length of data bytes, and here we are sending 2now3 data Bytes

	CANTxData[0] = 50;  			//
	CANTxData[1] = 0xAA;			//	Two bytes to send
	CANTxData[2]++;
	if (HAL_CAN_AddTxMessage(&hcan1, &CANTxHeader, CANTxData, &CANTxMailbox) != HAL_OK)
	{
		can_errors++;
//	   Error_Handler ();
	}
}

void	showme	()	{
	char	txt[255];
	int	len;
	pc.write	("Lev X data\r\n", 12);
	len = sprintf	(txt, "gate1 %d %c, %d %c,\tgate2 %d %c, %d %c,\tgate3 %d %c, %d %c,\tgate4 %d %c, %d %c\r\n",
			Level_Crossing.gate1.A.get_pin_num(), Level_Crossing.gate1.A.active() ? 'T' : 'F', Level_Crossing.gate1.B.get_pin_num(), Level_Crossing.gate1.B.active() ? 'T' : 'F',
			Level_Crossing.gate2.A.get_pin_num(), Level_Crossing.gate2.A.active() ? 'T' : 'F', Level_Crossing.gate2.B.get_pin_num(), Level_Crossing.gate2.B.active() ? 'T' : 'F',
			Level_Crossing.gate3.A.get_pin_num(), Level_Crossing.gate3.A.active() ? 'T' : 'F', Level_Crossing.gate3.B.get_pin_num(), Level_Crossing.gate3.B.active() ? 'T' : 'F',
			Level_Crossing.gate4.A.get_pin_num(), Level_Crossing.gate4.A.active() ? 'T' : 'F', Level_Crossing.gate4.B.get_pin_num(), Level_Crossing.gate4.B.active() ? 'T' : 'F'
			);
	pc.write(txt, len);
	len = sprintf(txt, "alarm %d\r\n", Level_Crossing.alarm.get_pin_num());
	pc.write(txt, len);
}
//Sig_Dn_Main_Distant.set_occulting();

extern	"C"	{

void	ForeverLoop	()	{	//	Loop repeat rate 50Hz (June 2023)
							//	Arrives here asap after spi completes and inputs have been  read and updated
//	can_try();
	if	(can_flag)	{
		can_flag = false;
		ce_show	();
	}

//	Every object has a 'read_update' function.  These do any input switch debouncing etc

	Level_Crossing	.read_update	();
	Al1				.read_update	();
	Al2				.read_update	();

	Sig_Dn_Main_Distant			.read_update()	;
	Sig_Dn_Main_Outer_Home		.read_update()	;
	Sig_Dn_Main_Home			.read_update()	;
	Sig_Dn_Main_To_Platform_Home.read_update()	;
	Sig_Disc_Dn_Main_Backing	.read_update()	;
	Sig_Steaming_Bays_Exit		.read_update()	;
	Sig_Disc_Up_Main_Backing	.read_update()	;
	Sig_Up_Loop_To_Carriage_Shed.read_update()	;
	Sig_Exit_From_Carriage_Shed	.read_update()	;
	Sig_Up_Loop_To_Up_Main_Start.read_update()	;
	Sig_Up_Loop_Home			.read_update()	;
	Sig_Up_Main_Start			.read_update()	;
	Sig_Up_Main_Home			.read_update()	;

	Turnout_From_Turntable		.read_update()	;
	Turnout_Dn_Main_To_Platform	.read_update()	;
	Xover_To_Steaming_Bays		.read_update()	;
	Xover_Up_Loop_To_Up_Main	.read_update()	;


/*	if	(false)
		Sig_Dn_Main_Distant.set_clear();
	else
		Sig_Dn_Main_Distant.set_danger();
*/
	Sig_Dn_Main_Distant.set_occulting();

	if	(false)
		Sig_Dn_Main_Outer_Home.set_clear();
	else
		Sig_Dn_Main_Outer_Home.set_danger();


	if	(Turnout_Dn_Main_To_Platform.reverse() && Level_Crossing.safe_for_rail() && Xover_To_Steaming_Bays.normal())
		Sig_Dn_Main_Home.set_clear();
	else
		Sig_Dn_Main_Home.set_danger();


	if	(Turnout_Dn_Main_To_Platform.reverse() && Level_Crossing.safe_for_rail() && Xover_To_Steaming_Bays.normal())
		Sig_Dn_Main_To_Platform_Home.set_clear();
	else
		Sig_Dn_Main_To_Platform_Home.set_danger();


	if	(Level_Crossing.safe_for_rail())
		Sig_Disc_Dn_Main_Backing.set_clear();
	else
		Sig_Disc_Dn_Main_Backing.set_danger();


	if	(Xover_To_Steaming_Bays.reverse() && Level_Crossing.safe_for_rail())
		Sig_Steaming_Bays_Exit.set_clear();
	else
		Sig_Steaming_Bays_Exit.set_danger();

	if	(!Xover_Up_Loop_To_Up_Main.wrong())	//	Detect No.11B switchblades in either position
		Sig_Disc_Up_Main_Backing.set_clear();
	else
		Sig_Disc_Up_Main_Backing.set_danger();

	if	(false)
		Sig_Up_Loop_To_Carriage_Shed.set_clear();
	else
		Sig_Up_Loop_To_Carriage_Shed.set_danger();

	if	(false)
		Sig_Exit_From_Carriage_Shed.set_clear();
	else
		Sig_Exit_From_Carriage_Shed.set_danger();


	if	(false)
		Sig_Up_Loop_To_Up_Main_Start.set_clear();
	else
		Sig_Up_Loop_To_Up_Main_Start.set_danger();


	if	(Level_Crossing.safe_for_rail())
		Sig_Up_Loop_Home.set_clear();
	else
		Sig_Up_Loop_Home.set_danger();


	if	(false)
		Sig_Up_Main_Start.set_clear();
	else
		Sig_Up_Main_Start.set_danger();


	if	(Level_Crossing.safe_for_rail())
		Sig_Up_Main_Home.set_clear();
	else
		Sig_Up_Main_Home.set_danger();


	if	(oncelist)	{
		Al2.mark_space(100,500);
		Level_Crossing.alarm.mark_space(950, 50);
		Level_Crossing.alarm.set();					//	start alarm
		Al1.set();
		Al2.set();
		showme();
		oncelist = false;
	}
//	int	fred = Level_Crossing.gate1.A.get_pin_num();
/*	set_output_bit	(12, true);
	if	(Level_Crossing.button1.pressed())	//	It either is, or it is not
		fred--;
	if	(Level_Crossing.button2.pressed())	//	It either is, or it is not
		fred--;
	if	(Level_Crossing.gates_are_locked())	//	They either are, or they are not
		fred--;
	if	(Level_Crossing.safe_for_road())	//	It either is, or it is not
		fred--;
	if	(Level_Crossing.safe_for_rail())	//	It either is, or it is not
		fred--;*/
//	bool	tmp = Level_Crossing.gate2.input_A();
	check_commands	()	;	//	Looks after all command line input
	i++;
	if	(i >= 50)	{
		i = 0;
		can_try();
		get_raw_input_bytes(rx);
//		if	(set_output_bit(k, toggle))
//		le = sprintf	(t, "Set out bit %d %d %s %2x %2x %2x %2x %2x %2x %2x %2x\r\n", k, toggle,
//					set_output_bit(k, toggle) ? "good" : "bad", rx[0], rx[1], rx[2], rx[3],
//					rx[4], rx[5], rx[6], rx[7]);
//		else
//			le = sprintf	(t, "Set out bit %d %d bad\r\n", k, toggle);
//		pc.write	(t, le);
			k++;
		if(k > 67)	{
			k = -3;
			toggle = !toggle;
		}	//	00 -> 56, 08 -> 48
		  HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);	//	as good a place as any to toggle green Nucleo led
		  //	itoa	(num, buf, 10);
//		  freda	(tc, indx);
//		  len = strlen(tc);
//		  len = sprintf(tc, "This is the sprintf %d test %d with growing pi %.3f\r\n", indx, fred++, pi);
	}
}	//	End of ForeverLoop function
}	//	End of extern "C" wrapper


