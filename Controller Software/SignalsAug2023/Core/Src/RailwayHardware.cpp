/*
 * RailwayHardware.cpp
 *
 *  Created on: Jun 13, 2023
 *      Author: Jon Freeman  B Eng Hons
 */
#include 	"main.h"
#include	"RailwayHardware.hpp"


//	Need to know how to set outputs and read inputs. Two external functions provide these services : -

/**
 * bool	set_output_bit	(uint32_t which_out, bool hiorlo)	{
 * Function in "main.c"
 * Number of output BYTES defined as 'IO_DAISY_CHAIN_BYTE_LEN' defined in "main.c", (probably 8)
 * Number of outputs = BYTES * 8 (numbered 0 to 63 when IO_DAISY_CHAIN_BYTE_LEN == 8)
 *
 * Function returns 'false' if 'which_output' contains invalid output number, returns 'true' otherwise
 * Using hiorlo 'true' turns output on, 'false' off.
 */
extern	"C"	bool	set_output_bit	(uint32_t which_output, bool hiorlo)	;

uint32_t		ins_count[66] = {0}, outs_count[66] = {0},	iosetuperrors = 0	;	//	Used to help find ins or outs used more than once

void	inpincounter	(INPIN x)	{	//	Keeps record of what inputs used and how many times !
	if	((x < 0) || (x > 63))
		iosetuperrors++;
	else
		ins_count[x]++;
}

void	outpincounter	(OUTPIN x)	{	//	Useful for checking not used same output pins more than once
	if	((x < 0) || (x > 63))
		iosetuperrors++;
	else
		outs_count[x]++;
}

void	RegisterBit::set(bool b)	{	Q = b;		}	//	set Q = b;
void	RegisterBit::set()			{	Q = true;	}	//	set Q = true;
void	RegisterBit::clr()			{	Q = false;	}	//	clr Q = false;
bool	RegisterBit::read()			{	return	Q;	}
//void	RegisterBit::read_update()	{}					//	There is nothing to update here

void	SingleOutput::set	(bool torf)	{
	Q.set(torf);
	set_output_bit	(output_ionum, (torf ^ polarity));
}

void	SingleOutput::set	()	{				//	Turn output 'on'
	Q.set();
	set_output_bit	(output_ionum, polarity);
}

void	SingleOutput::clr	()	{				//	Turn output 'off'
	Q.clr();
	set_output_bit	(output_ionum, !polarity);
}

/*void	SingleOutput::on	()	{				//	Turn output 'on', same as set
	Q.set();
	set_output_bit	(output_ionum, polarity);
}

void	SingleOutput::off	()	{				//	Turn output 'off', same as clr
	Q.clr();
	set_output_bit	(output_ionum, !polarity);
}*/

bool	SingleOutput::read()	{
	return	Q.read();
}

OUTPIN	SingleOutput::get_pin_num	()	{
	return	output_ionum;	//
}



/**
 * bool	get_input_bit_debounced
 * A 'De-Bounce' implementation
 * 			This is the function to call to access inputs.
 *
 * 	Inputs are scanned (and outputs updated) regularly, probably 50 times per second.
 *
 * Most recent 31 input[n] bit reads stored in uint32_t ins_history[n], newest in bit 31, oldest bit 0
 * (chose 31 over 32 so that number positive regardless of 'int' ot 'uint')
 * function takes odd number for samples and tests this many from most recent back in time
 * returns 'true' for more '1's than '0's, false for more '0's than '1's
 * On error sets bits in 'input_test_error_flags' and returns false
 */
extern	"C"	bool	get_input_bit_debounced	(uint32_t which_in, uint32_t how_many)	;	//	from main.c

/*bool	SingleInput::input	()	{	//	Returns debounced state corrected for polarity
	return (debounced ^ polarity);
}*/

bool	SingleInput::rising	()	{	//	Returns true when rising edge detected
	return	(present && !previous)	;	//	Found __--
}

bool	SingleInput::falling	()	{	//	Returns true when falling edge detected
	return	(!present && previous)	;	//	Found --__
}

bool	SingleInput::active	()	{	//	Returns debounced state corrected for polarity
	return	present;
}

void	SingleInput::read_update	()	{
	previous = present;		//	Save for possible edge detection
	present = ((get_input_bit_debounced	(input_ionum ,DEFAULT_DEBOUNCE_VALUES)) ^ polarity);
}

INPIN	SingleInput::get_pin_num	()	{
	return	input_ionum;	//
}


void	DualInput::read_update	()	{	//	For reading Moving Parts in accordance with
	A.read_update()	;					//	the Moving Part Rule
	B.read_update()	;
}

//enum	SignalStates	{SIGNAL_BLACK, SIGNAL_DANGER, SIGNAL_CAUTION, SIGNAL_CLEAR}	;

void	LinesideSignal::set_danger	()	{
	outs.A.clr();	//	set_output_bit	(ClearOutputNum, false);
	outs.B.set();	//	set_output_bit	(DangerOutputNum, true);
	commanded_state = SIGNAL_DANGER;
}

void	LinesideSignal::set_caution	()	{
	outs.A.set();	//	set_output_bit	(ClearOutputNum, true);
	outs.B.set();	//	set_output_bit	(DangerOutputNum, true);
	commanded_state = SIGNAL_CAUTION;
}

void	LinesideSignal::set_occulting	()	{	//	OCCULTING_YELLOW
	//	Requires yellow flash with on time > off time
//	outs.A.set();	//	set_output_bit	(ClearOutputNum, true);
//	outs.B.set();	//	set_output_bit	(DangerOutputNum, true);
	commanded_state = SIGNAL_OCCULTING_YELLOW;
}

void	LinesideSignal::set_clear	()	{
	outs.A.set();	//	set_output_bit	(ClearOutputNum, true);
	outs.B.clr();	//	set_output_bit	(DangerOutputNum, false);
	commanded_state = SIGNAL_CLEAR;
}

void	LinesideSignal::set_black	()	{
	outs.A.clr();	//	set_output_bit	(ClearOutputNum, false);
	outs.B.clr();	//	set_output_bit	(DangerOutputNum, false);
	commanded_state = SIGNAL_BLACK;
}

void	LinesideSignal::set_to	(int state_to_set)	{	//	Affects two outputs only
	commanded_state = state_to_set;	//	record most recent setting
	switch	(state_to_set)	{
		case	SIGNAL_DANGER:
			set_danger	()	;
			break;
		case	SIGNAL_CAUTION:
			set_caution	()	;
			break;
		case	SIGNAL_OCCULTING_YELLOW:
			set_occulting	()	;
			break;
		case	SIGNAL_CLEAR:
			set_clear	()	;
			break;
		case	SIGNAL_BLACK:
			set_black	()	;
			break;
		default:
//			pc.write("Bad", 3);
			break;
	}
}

void	LinesideSignal::read_update	()	{	//	Affects two outputs only
	const	int	OCCULT_ON_MS	=	200;
	const	int	OCCULT_OFF_MS	=	100;

	ins.A.read_update();	//	There are few if any reasons to read inputs back, except
	ins.B.read_update();	//	to determine signal head type, head presence or absence.
	if	(commanded_state == SIGNAL_OCCULTING_YELLOW)	{
		if	(flash_timer < millisecs)	{
			if	(flash_state)	{
				flash_state = false;
				flash_timer = millisecs + OCCULT_ON_MS;
				outs.A.set();
				outs.B.set();
			}
			else	{
				flash_state = true;
				flash_timer = millisecs + OCCULT_OFF_MS;
				outs.A.clr();
				outs.B.clr();
			}
		}
	}
}

int		LinesideSignal::returned	()	{	//	Affects two outputs only
	int	rv = 0;
	if	(ins.A.active())	rv |= 1;
	if	(ins.B.active())	rv |= 2;
	returned_state = rv;
	return	rv;							//	Returns 0, 1, 2 or 3
}


void	Points::set_to	(int state_to_set)	{	//	Affects two outputs only
//	set_state = state_to_set;
	switch	(state_to_set)	{
		case	UNPOWERED:
			outs.A.clr();	//	set_output_bit	(ClearOutputNum, false);
			outs.B.clr();	//	set_output_bit	(DangerOutputNum, true);
			break;
		case	NORMAAL:	//	Dutch spelling prevents potential keyword conflict with 'NORMAL'
			set_state = state_to_set;
			outs.A.set();	//	set_output_bit	(ClearOutputNum, true);
			outs.B.clr();	//	set_output_bit	(DangerOutputNum, true);
			break;
		case	OMKEREN:	//	Dutch for 'reverse', avoids potential keyword conflict with 'REVERSE'
			set_state = state_to_set;
			outs.A.clr();	//	set_output_bit	(ClearOutputNum, true);
			outs.B.set();	//	set_output_bit	(DangerOutputNum, false);
			break;
//		case	FAULTY:
//			outs.A.clr();	//	set_output_bit	(ClearOutputNum, false);
//			outs.B.clr();	//	set_output_bit	(DangerOutputNum, false);
//			break;
		default:
//			pc.write("Bad", 3);
			break;
	}
}

void	Points::read_update	()	{	//	Affects two outputs only
	ins.A.read_update();
	ins.B.read_update();
}

int		Points::returned	()	{	//	Affects two outputs only
	int	rv = 0;
	if	(ins.A.active())	rv |= 1;
	if	(ins.B.active())	rv |= 2;
	returned_state = rv;
	return	rv;
}

bool	Points::wrong	()	{	//	returns true while readback state does not match set state
	if	((set_state != NORMAAL) && (set_state != OMKEREN))
		return	true;	//	Points have not been set
	returned	()	;	//	updates returned_state
	return	!(returned_state == set_state);
}

bool	Points::normal	()	{
	return	(!wrong() && ins.A.active() && !ins.B.active());
}

bool	Points::reverse	()	{
	return	(!wrong() && !ins.A.active() && ins.B.active());
}


bool	DualPoints::wrong	()	{	//	returns true while readback state does not match set state
	return	(Points1.wrong() && Points2.wrong());
}

bool	DualPoints::normal	()	{	//	returns true while readback state does not match set state
	return	(Points1.normal() && Points2.normal());
}

bool	DualPoints::reverse	()	{	//	returns true while readback state does not match set state
	return	(Points1.reverse() && Points2.reverse());
}

void	DualPoints::set_to	(int settostate)	{
	Points1.set_to(settostate);	//	Uses only 1 pair of outputs as point pair work together
//	Points2.set_to(settostate);
}

void	DualPoints::read_update()	{
	Points1.read_update();
	Points2.read_update();
}

int	DualPoints::returned()	{
	int	rv1, rv2;
	rv1	=	Points1.returned();
	rv2	=	Points2.returned();
	if	(rv1 != rv2)
		return	FAULTY;	//	readings same to pass here
	return	rv1;	//	Could be any of 4 states
}



bool	ClassLevelCrossing::set	(int new_state)	{	//	new_state will be in range 0 to 3
	if	((new_state < 0) || (new_state > 3))
		return	false;
	actuators.A.set((new_state & 1) != 0);
	actuators.B.set((new_state & 2) != 0);
	return	true;
}

bool	ClassLevelCrossing::gates_are_locked	()	{
	return	(lock1.A.active() && lock2.A.active());
}

bool	ClassLevelCrossing::safe_for_road	()	{
	return	(gate1.A.active() && gate2.A.active() && gate3.A.active() && gate4.A.active()
			&& gates_are_locked());
}

bool	ClassLevelCrossing::safe_for_rail	()	{
	return	(gate1.B.active() && gate2.B.active() && gate3.B.active() && gate4.B.active()
			&& gates_are_locked());
}

void	ClassLevelCrossing::read_update()	{	//	Reads and debounces all inputs once per loop pass here.
	gate1.read_update	()	;					//	Debouncing uses processor time,
	gate2.read_update	()	;					//	therfore quicker to debounce once and reuse result
	gate3.read_update	()	;
	gate4.read_update	()	;
	lock1.read_update	()	;
	lock2.read_update	()	;
	button1.read_update	()	;
	button2.read_update	()	;
	alarm.read_update	();
	switch	(state)	{			//	This is where any gate action is initiated
	case	0:					//	Reset state, item positions unknown
		break;
	default:
		break;
	}
}

/*class	PulseAlarm	{
	SingleOutput	OutBit	;
	int	time_on, time_off, temp, up_count	;
	bool	alarm_on	;
public:
	void	mark_space	(int, int)	;
	void	set	()	;
	void	clr	()	;
	void	read_update	()	;
};*/

//extern	uint32_t	millisecs	;	now in .hpp
void	TimerClass::set(uint32_t t)	{
	timer = t + millisecs;
}

bool	TimerClass::timeout()	{
	return	(timer < millisecs);
}

void	PulseAlarm::mark_space(int mark_ms, int space_ms)	{
	mark = mark_ms;
	space = space_ms;
}

void	PulseAlarm::set	()	{
	alarm_on = true;
	alarm_timer = millisecs + mark;
	OutPin.set();
}

void	PulseAlarm::clr	()	{
	alarm_on = false;
	OutPin.clr();
}

void	PulseAlarm::read_update	()	{
	uint32_t	current_time = millisecs;
	if	(alarm_on && (current_time > alarm_timer))	{
		if	(OutPin.read())	{
			alarm_timer = current_time + space;
			OutPin.clr();
		}
		else	{
			alarm_timer = current_time + mark;
			OutPin.set();
		}
	}
}

OUTPIN	PulseAlarm::get_pin_num	()	{
	return	OutPin.get_pin_num();
}



