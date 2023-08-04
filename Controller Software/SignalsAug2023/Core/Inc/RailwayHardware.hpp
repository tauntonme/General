/*
 * RailwayHardware.hpp
 *
 *  Created on: Jun 13, 2023
 *      Author: Jon Freeman  B Eng Hons
 *
 *      This file contains Object Class declarations for items of Railway Signalling Hardware.
 *      These are the 'entity' type declarations which record how the object to be created connects to the wider system.
 *      Electrical connections will be made using Inputs and Outputs.
 *      Class member functions are listed here, indicating how objects may be interfaced to other system entities.
 *
 *      Code forming the inner workings of member functions is in file 'RailwayHardware.cpp'
 *
 *      Read here to learn how to use signals, buttons, moving parts and level crossings
 *
 *      Items List -	doubtless incomplete	:
 *      	Building blocks :
 *      		RegisterBit		a 1 bit memory
 *		      	SingleInput		use this for e.g. PushButton
 *  	    	DualInput		use this for e.g. MovingPart
 *      		SingleOutput
 *      		DualOutput		use this for signals, traffic lights, points actuators
 *      	Compound items :
 *		      	LinesideSignal	incorporates one DualInput, one DualOutput
 *      		LevelCrossing	-	a large object of many parts
 *     		 	Points			or Turnouts
 *      		DualPoints		or Crossovers
 */

#ifndef INC_RAILWAYHARDWARE_HPP_
#define INC_RAILWAYHARDWARE_HPP_

const	bool	INV		=	false;
const	bool	NONINV 	=	!INV;
const 	bool	INVERTED = false;	//	used to select input polarity

#define	DEFAULT_DEBOUNCE_VALUES	5	//	Switch readings will be the majority out of this number of consecutive reads (one read per ForeverLoop pass)

typedef	unsigned int	INPIN;
typedef	unsigned int	OUTPIN;
typedef	bool			POLARITY;

enum	SignalStates	{	SIGNAL_BLACK,
							SIGNAL_DANGER,
							SIGNAL_CAUTION,
							SIGNAL_CLEAR,
							SIGNAL_OCCULTING_YELLOW}	;//	States to which LED signals may be set. Semaphores to show DANGER when not set to CLEAR
	//	Note Occulting is flash where on_time > off_time. See https://www.railforums.co.uk/threads/flashing-signals.186537/


enum	PointsStates	{UNPOWERED, NORMAAL, OMKEREN, FAULTY}	;	//	Position of points, levers and whatever

void	inpincounter	(INPIN)	;	//	Keeps record of what inputs used and how many times !
void	outpincounter	(OUTPIN);	//	Useful for checking not used same pins more than once

/*class	D_FlipFLop	{
	bool	Q, lastD, lastCLK;
};

class	SR_FlipFlop	{
	bool	Q;
  public:
	SR_FlipFlop	()	{
		Q = false;
	}
	void	set();
	void	clr();
};*/
extern	uint32_t	millisecs	;

class	TimerClass	{	//	Set timeout for some millisecs hence using e.g. MyTimer.set(1000);	. Test using e.g. if(MyTimer.timeout())
	uint32_t	timer;
  public:
	TimerClass	()	{
		timer = 0L;
	}
	void	set	(uint32_t);	//	Use to load timer with desired timeout delay, in ms
	bool	timeout	();		//	Returns true if timed out, false if full time not yet elapsed
}	;


/**
 * class	LinkedListNode	{
 * 		Of no immediate interest in building the signalling system, this comes into play
 * 		holding information about what physical items connect to which others.
 * 		This will be useful in constructing display panels, layout diagrams etc.
 */
class	LinkedListNode	{
  public:
	LinkedListNode * prev, * next;	//	Pointers to previous and next nodes
}	;

class	TrackSection	{	//	Useful when modelling the whole railway, joins things together
  public:					//	Becomes a vital element once train detection is in place
	LinkedListNode	node;
	TrackSection	()	{
		node.next = node.prev = nullptr;	//	Use 'nullptr' to indicate disconnection
	}
}	;

/**	class	SingleInput	{	This is a fundamental base unit reused in most of what follows
 *
 * 							There are two things we need to know about every Input connection : -
 * 							1.	Which input terminal the item is wired to
 * 							2.	Polarity, switch contacts are N.O. or N.C.
 */
class	SingleInput	{
	INPIN	input_ionum;	//	Which physical numbered input terminal on PCB the wire is screwed into
	bool	polarity, present, previous;
  public:
	SingleInput	(INPIN n, POLARITY p)	{	//	Constructor
		input_ionum = n;
		polarity = p;
		present = previous = false;	//
		inpincounter	(input_ionum);	//	Record fact that pin 'inpin_ionum' has been used
	}
	void	read_update	()	;	//	Copies present into previous, updates present with latest read of input pin
	bool	active		()	;	//	Same as 'input', e.g. use if(button1.active()).	Returns true for active, false for inactive
	bool	rising		()	;	//	Returns true when rising edge detected	i.e.	(present && !previous)
	bool	falling		()	;	//	Returns true when falling edge detected	i.e.	(!present && previous)
	INPIN	get_pin_num	()	;	//
}	;

class	DualInput	{	//	Group of two SingleInputs, A and B
  public:
	SingleInput	A, B;	//	DualInput is a pair of SingleInput s
	DualInput	(INPIN A_input_ionum, POLARITY A_polarity, INPIN B_input_ionum, POLARITY B_polarity)	:
	A	{A_input_ionum, A_polarity},
	B	{B_input_ionum, B_polarity}
	{	/*	Nothing to initialise in constructor here	*/	}
	void	read_update	()	;	//	Calls read_update for inputs A and B
}	;

class	RegisterBit	{	//	A one-bit memory
	bool	Q;
public:
	RegisterBit	()	{	Q = false;	}	;	//	Default at power-on
	void	set			(bool b)	;	//	set Q = b;
	void	set			()	;			//	set Q = true;
	void	clr			()	;			//	clr Q = false;
	bool	read		()	;			//	return Q
//	void	read_update	()	;			//	There is nothing to update here
}	;

class	SingleOutput	{	//	Normal use is to drive output pin. Also usable as 1 bit register with invalid output_ionum
	OUTPIN	output_ionum;
	RegisterBit	Q;			//	This is needed to enable read back of how the output was last set
	POLARITY	polarity;	//	Outputs normally 'active on', allows for choice of 'active off'.
  public:
	SingleOutput	(OUTPIN f, POLARITY p)	{	//	Constructor
		output_ionum = f;
		polarity = p;
		Q.clr();
		outpincounter	(output_ionum);	//	Record fact that pin 'inpin_ionum' has been used
	}
	void	set			(bool b)	;	//	value 'b ^ polarity' is output
	void	set			()	;	//	value 'polarity' is output
	void	clr			()	;	//	value '!polarity' is output
	bool	read		()	;	//	return last value output
	OUTPIN	get_pin_num	()	;
	//	void	read_update	()	;			//	There is nothing to update here
}	;

class	DualOutput	{	//	Group of two SingleOutputs, A and B
  public:
	SingleOutput	A, B;	//	Two single outputs
	DualOutput	(OUTPIN A_output_ionum, POLARITY A_polarity, OUTPIN B_output_ionum, POLARITY B_polarity)	:
	A	{A_output_ionum, A_polarity},	//	Instantiation of single output 'A'
	B	{B_output_ionum, B_polarity}
	{	/*	Nothing to initialise in constructor here	*/	}
	//	No member functions, merely gateway down to SingleOutput objects A and B
}	;



/**	class	LinesideSignal	{
 *
 * Lineside Railway Signals, coloured light or semaphore, connect to two output lines from the controller,
 * 		and connect two 'status' output lines back to controller inputs.
 * 	That is, the physical hardware signal has 4I/O (Input Outputs) :
 * 		Two status Outputs connected to controller Inputs informing the controller of signal status
 * 		Two command Inputs driven by controller Outputs to set signal to 'Danger', 'Clear', 'Caution', 'Black'.
 * 		Semaphores are to show 'Clear' only for the 'Clear' state, 'Danger' otherwise.
 */
class	LinesideSignal	{	//	Lineside railway signal, 3 aspect colour light or 2 aspect semaphore
	int			commanded_state, returned_state;
	uint32_t	flash_timer;
	bool		flash_state;	//	Used in on/off toggle when 'occulting' (flashing)
	DualInput	ins;	//	Two inputs to conform to the Moving Part Rule
	DualOutput	outs;	//	Two outputs to define states 'Danger', 'Caution', 'Clear' and 'Black'
	LinkedListNode	node;
public:
	void	set_clear	();
	void	set_danger	();	//	default at power-up
	void	set_caution	();
	void	set_occulting	();	//	Do not EVER call this flashing! (Mustn't upset the gricers)
	void	set_black	();
	LinesideSignal	(INPIN ClearInputNum, INPIN DangerInputNum, OUTPIN ClearOutputNum, OUTPIN DangerOutputNum)	:
		ins		{ClearInputNum, true, DangerInputNum, true},	//	Two inputs
		outs	{ClearOutputNum, true, DangerOutputNum, true}	//	Two outputs
	{	//	This is the "constructor"	//
		set_danger	()	;	//	also sets commanded_state
		returned_state = 0;	//	Actual state read back on input lines
		flash_timer = 0;
		node.next = nullptr;
		node.prev = nullptr;
	}	;
	void	set_to		(int);	//	set state to one of DANGER, CAUTION, CLEAR, BLACK, OCCULTING_Y
	void	read_update	();	//	Read state read back from input lines
	int		returned	();	//	The state read back from input lines
//	bool	is_semaphore();	//	Returns 'true' for semaphore detected, false otherwise (colour light assumed)
}	;

class	Points	{	//	Points - a.k.a. Turnouts
	int			set_state, returned_state;
	DualInput	ins;	//	Two inputs to conform to the Moving Part Rule, Points set to 'A', 'B', neither
	DualOutput	outs;	//	Two outputs for actuators to move A->B, or B->A
	LinkedListNode	node;
  public:
	Points	(INPIN inA, POLARITY polAin, INPIN inB, POLARITY polBin, OUTPIN outA, POLARITY polAout, OUTPIN outB, POLARITY polBout):
		ins		{inA,  polAin, inB, polBin},		//	2 inputs from points travel limit switches
		outs	{outA, polAout, outB, polBout}		//	2 outputs for points actuators , 1 out, 1 back
		{	//	Constructor start
			/*	*/
			set_state = UNPOWERED;	//
			outs.A.clr();
			outs.B.clr();
			node.next = nullptr;
			node.prev = nullptr;
		};	//	Constructor end
		void	set_to		(int);	//	set state to one of NORMAAL, NIET_NORMAAL, UNPOWERED
		void	read_update	();		//	Read state read back from input lines
		int		returned	();		//	The state read back from input lines
		bool	wrong		()	;	//	returns true while readback state does not match set state
		bool	normal		()	;
		bool	reverse		()	;
}	;

class	DualPoints	{	//	Uses only 1 pair of outputs as point pair work together
	Points	Points1, Points2;
public:
	DualPoints	(INPIN in1A, INPIN in1B, POLARITY pol1Ain, POLARITY pol1Bin, INPIN in2A, INPIN in2B, POLARITY pol2Ain, POLARITY pol2Bin,
			OUTPIN outA, OUTPIN outB, POLARITY polAout, POLARITY polBout):
		Points1	{in1A, pol1Ain, in1B, pol1Bin, outA, polAout, outB, polBout},
		Points2	{in2A, pol2Ain, in2B, pol2Bin, outA, polAout, outB, polBout}
		{	//	consructor start
			//	constructor action
		}	//	constructor end
		void	set_to		(int);	//	set state to one of NORMAAL, NIET_NORMAAL, UNPOWERED
		void	read_update	();		//	Read state read back from input lines
		int		returned	();		//	The state read back from input lines
		bool	wrong		()	;	//	returns true while readback state does not match set state
		bool	normal		()	;
		bool	reverse		()	;
}	;

class	PulseAlarm	{
	SingleOutput	OutPin	;
	uint32_t	mark, space	;
	uint32_t	alarm_timer	;
	bool	alarm_on	;
public:
	PulseAlarm	(OUTPIN a, POLARITY b) : OutPin{a, b}	{
		alarm_on = false;
		mark = 250;			//	times in ms
		space = 250;
		alarm_timer = 0;
		OutPin.clr();
	}
	void	mark_space	(int mark_ms, int space_ms)	;	//	set mark and space times, each in millisecs
	void	set	()	;									//	Start alarm
	void	clr	()	;									//	Stop alarm
	void	read_update	()	;							//	Call at forever loop repetition rate
	OUTPIN	get_pin_num	()	;
}	;

/**
 * class	ClassLevelCrossing	{
 *
 * As defined here, a Level Crossing has 4 gates, 2 locks, 2 push buttons and 1 traffic light
 * 4 controller outputs are provided
 */
class	ClassLevelCrossing	{
 		int	state;		//	state of the level crossing (not used to 9th June 2023 J.F.)
 		int	timer1, timer2, timer3;
  public:
 		DualOutput		actuators;
 		PulseAlarm		alarm;
 	    DualInput	 	gate1, gate2, gate3, gate4;	//	Level crossing has 4 gates, MovingParts
		DualInput		lock1, lock2;				//	2 gate locks, MovingParts
		LinesideSignal	trafficlight;				//	LinesideSignal used here as road traffic lights
 	    SingleInput		button1, button2;			//	and 2 push buttons, SingleInput s
//  public:

    ClassLevelCrossing (	//
			INPIN ain1, 	POLARITY pa1, 	INPIN bin1, 	POLARITY pb1,	//	gate1 info			//	gate is 'MovingPart' with 2 position detectors.
			INPIN ain2, 	POLARITY pa2, 	INPIN bin2, 	POLARITY pb2,	//	gate2 info			//	bool pa1 etc are polarity selectors for N.O. or N.C. switch types
			INPIN ain3, 	POLARITY pa3, 	INPIN bin3, 	POLARITY pb3,	//	gate3 info
			INPIN ain4, 	POLARITY pa4, 	INPIN bin4, 	POLARITY pb4,	//	gate4 info
			INPIN la1, 		POLARITY lap1, 		INPIN lb1, 	POLARITY lbp1,	//	lock1 info
			INPIN la2, 		POLARITY lap2, 		INPIN lb2, 	POLARITY lbp2,	//	lock2 info
			INPIN tli1,		INPIN	tli2,	OUTPIN	tlo1,	OUTPIN	tlo2,	//	traffic light I/O numbers
			INPIN but1N,	POLARITY but1P,							//	push button 1 info
			INPIN but2N,	POLARITY but2P,							//	push button 2 info
			OUTPIN	alarm_pin, POLARITY	al_pol,						//	alarm
			OUTPIN	out_NORM,	OUTPIN out_NIET, POLARITY out_pol
			) :
			actuators	{out_NORM, out_pol, out_NIET, out_pol},
			alarm	{alarm_pin, al_pol},
 	        gate1	{ain1, pa1, bin1, pb1},				//	two input port numbers, two boolean polarity selectors
 	        gate2	{ain2, pa2, bin2, pb2},
 	        gate3	{ain3, pa3, bin3, pb3},
			gate4	{ain4, pa4, bin4, pb4},

			lock1	{la1, lap1, lb1, lbp1},
    		lock2	{la1, lap1, lb1, lbp1},
			trafficlight	{tli1, tli2, tlo1, tlo2},	//	two inputs, two outputs
			button1	{but1N, but1P},
			button2	{but2N, but2P}
			{
    	state = 0;	//	Reset state, we don't know the position of any parts yet.
    	timer1 = 0;
    	timer2 = 0;
    	timer3 = 0;
    	alarm.clr();
    }
    //	public member functions for LevelCrossing
	bool	set				(int);	//	energise actuators
	void	read_update		();	//	Updates input info in all included MovingParts
	bool	gates_are_locked();	//	Returns 'true' if gates are locked !
	bool	safe_for_road	();	//	Returns 'true' if gates are locked AND all 4 gates safe for road
	bool	safe_for_rail	();	//	Returns 'true' if gates are locked AND all 4 gates safe for rail
}	;


#endif /* INC_RAILWAYHARDWARE_HPP_ */
