/*
 * settings.cpp
 *
 *  Created on: Jun 13, 2023
 *      Author: jon34
 */

/*
 * settings.cpp
 *
 *  Created on: Oct 26, 2022
 *      Author: Jon Freeman
 */
#include	"settings.hpp"
#include	"Serial.hpp"
#include	<stdio.h>
#include	<stdbool.h>
#include	<string.h>
#include 	<stdlib.h>
#include 	<ctype.h>
#include 	<math.h>

i2eeprom_settings	j_settings		;

extern	UartComChan	pc,	ctrlr	;
extern	I2C_HandleTypeDef hi2c1;

#define	LM75_ADDR	0x90
#define	LC64_ADDR	0xa0
#define	EEPROM_PAGE_SIZE	32
#define	SETTINGS_BASE_ADDRESS	1024


struct  usettings_optpar  {
    const int32_t min, max, de_fault, style;  //  min, max, default    changed up to int32_t Oct 2022
    const char * txt;     //  description
//    int32_t i;      //  Oct 22, noted is possible to mix const and variables here, no use found yet.
}   ;


struct  usettings_optpar    user_settings_list[] = {    //  Position in this list is very touchy.
    {0, 0, 0, 'z',     "0 not valid setting"}, //  BOARD_ID    defaults to '0' before eerom setup for first time 12
    {'1', '9', '0', 'C',     "Alternative ID ascii '1' to '9'"}, //  BOARD_ID    defaults to '0' before eerom setup for first time 12
    {0, 1, 1, 'I',     "Motor direction [0 or 1]"},       //  MOTADIR               0
    {4, 8, 8, 'I',     "Motor poles 4 or 6 or 8"},      //  MOTAPOLES               2
    {1, 4, 1, 'I',     "Motor 0R05 shunt Rs 1 to 4"},        //  ISHUNTA            4
    {10, 50, 20, 'I',  "POT_REGBRAKE_RANGE percentage 10 to 50"},     //  POT_REGBRAKE_RANGE         6
    {2, 6, 2, 'I',     "Command source 2= COM2 (Touch Screen), 3= Pot, 4= RCIn1, 5= RCIn2, 6=RCin_both"},     //  COMM_SRC 11
    {10, 250, 60, 'I',       "Top speed MPH times 10, range 10 to 250"},    //  New Jan 2019 TOP_SPEED 13
    {50, 253, 98, 'I',       "Wheel diameter mm"},   //  New 01/06/2018 14
    {10, 253, 27, 'I',       "Motor pinion"},   //  New 01/06/2018 15
    {20, 253, 85, 'I',       "Wheel gear"},   //  New 01/06/2018     ** NOTE this and above two used to calculate RPM to MPH **
    {15, 60, 48, 'I',    "Regen brake backoff voltage, used to limit regen voltage to safe level"},    //   NOM_SYSTEM_VOLTS
    {5, 91, 90, 'I',    "Brake Effectiveness percentage"},   //
    {25, 100, 50, 'I',    "Limit max motor current to this percentage"}, //
    {20, 50, 43, 'I',    "Pot Input Voltage Range times 10, range 20 to 43"},   //     POT_V_RANGE New 28th May 2021 in V2.0.1
    {0, 100, 0, 'I',     "Watchdog [1] Enable, [0] Disable"},   //
    {10, 60, 10, 'I',    "Low Volt Tailoff zero power volts P1 "},   //
    {12, 62, 12, 'I',    "Full power volts P2 "},
}   ;

const   int    numof_eeprom_options    = sizeof(user_settings_list) / sizeof (struct usettings_optpar);

void	i2eeprom_settings::edit   (float * flt, uint32_t numof_floats, const char * cl)    {
	    int32_t    temps[MAX_CLI_PARAMS+1];  //  max num of parameters entered from pc terminal
	    char    txt[200];
	    int     len;        //  Used to catch strlen as return value of sprintf
	    uint32_t i = 0;
	    float  topspeed;   //  New Jan 2019 - set max loco speed
	    len = sprintf     (txt, "\r\nus - User Settings data in EEPROM\r\nSyntax 'us' with no parameters lists current state.\r\n");
	    pc.write    ((uint8_t*)txt, len);
	    if  (numof_floats > 0)  {           //  If more than 0 parameters supplied
	        if  (numof_floats > MAX_CLI_PARAMS)
	            numof_floats = MAX_CLI_PARAMS;
	        for (i = 0; i < numof_floats; i++)
	            temps[i] = (int32_t)flt[i];          //  recast doubles to int32_t Oct 2022
	        while   (MAX_CLI_PARAMS > i)
	            temps[i++] = 0;
	        i = temps[0];
	        if	(strncmp	(cl, "us defaults", 11) == 0)
	        	i = 99876;
	        switch  (i) {   //  First number read from user response after "mode"
	            case    MOTOR_DIR:      //  MotorA_dir [0 or 1], MotorB_dir [0 or 1]
	                wr  (temps[1], MOTOR_DIR);	//	One motor
	                break;
	            case    MOTOR_POLES:      //  MotorA_poles [4,6,8], MotorB_poles [4,6,8]
	                if  (temps[1] == 4 || temps[1] == 6 || temps[1] == 8)
	                    wr    (temps[1], MOTOR_POLES);	//	One motor
	                break;
	            case    ISHUNTS:      //  MotorA_ current sense resistors [1 to 4], MotorB_ current sense resistors [1 to 4]
	                wr  (temps[1], ISHUNTS);     //  One motor
	                break;
	            case    BRAKE_EFFECTIVENESS:
	                wr  (temps[1], BRAKE_EFFECTIVENESS);
	                break;
	            case    POT_REGBRAKE_RANGE:
	                len = sprintf   (txt, "POT_REGBRAKE_RANGE value entered = %ld\r\n", temps[1]);
	                pc.write((uint8_t*)txt, len);
//	                if  (!in_range    (temps[1], POT_REGBRAKE_RANGE))
//	                    temps[1] = def(POT_REGBRAKE_RANGE);
//	                wr    (temps[1], POT_REGBRAKE_RANGE);
	                break;
	            case    BOARD_ID:      //  Board ID '0' to '9'
	                if  (temps[1] <= 9)    //  pointless to compare unsigned integer with zero
	                    wr  ('0' | temps[1], BOARD_ID);
	                break;
	            case    TOP_SPEED_MPH:      //  TOP_SPEED
	                topspeed = flt[1];
	                if  (topspeed > 25.0)   topspeed = 25.0;
	                if  (topspeed < 1.0)    topspeed = 1.0;
	                wr  ((char)(topspeed * 10.0), TOP_SPEED_MPH);
	                break;
	            case    WHEELDIA:      //  Wheel dia mm, Motor pinion teeth, Wheel gear teeth
	                wr  (temps[1], WHEELDIA);     //  These must be correct for speed reading to be correct
	                break;
	            case	MOTPIN:
	                wr  (temps[1], MOTPIN);
	            	break;
	            case	WHEELGEAR:
	                wr  (temps[1], WHEELGEAR);
	            	break;
	            case    COMM_SRC:      //    {2, 5, 2, "Command source 2= COM2 (Touch Screen), 3= Pot, 4= RC Input1, 5= RC Input2, 6=RC1+2 Robot"},
	                if  (temps[1] > 1 && temps[1] <= 6)
	                    wr  (temps[1], COMM_SRC);
	                break;
	            case    BRAKE_BACKOFF_VOLTS: //  Nominal System Voltage  ** Sept 2022 * Repurposed BRAKE_BACKOFF_VOLTS
	//                wr    (temps[1], NOM_SYSTEM_VOLTS);
	                wr    (temps[1], BRAKE_BACKOFF_VOLTS);
	                break;
	            case    I_SCALER: //  Current Scaler  **NEW** Work in progress May 2021
	                wr  (temps[1], I_SCALER);
	                len = sprintf(txt, "In us got [%s], %.0f, %.0f, %.0f, numof_dbls %ld\r\n", cl, flt[0], flt[1], flt[2], numof_floats);
	                pc.write((uint8_t*)txt, len);
	                break;
	            case    POT_V_RANGE: //  Pot FSD now selectable 2.0 to 5.0 volts  **NEW** 28th May 2021
	                wr  (temps[1], POT_V_RANGE);
	                break;
	//            case    32:
	//                wr(temps[1], DIR_ESTOP);
	//                break;
//	            case    32: //  Watchdog Enable [1], Disable [0]
//	                break;
	            case    36: //  Low voltage Tailoff P1, P2
	//                len = sprintf (txt, "In us, got min %d, max %d\r\n", (int)dbl[0], (int)dbl[1]) ;
	//                len = sprintf (txt, "In us, got min %d %s, max %d %s\r\n", temps[1],
	//                            in_range(temps[1], LV_TAILOFF_MIN) ? "good":"bad", temps[2],
	//                            in_range(temps[2], LV_TAILOFF_MAX) ? "good":"bad" );
	//                pc.write(txt, len);
	                if  (temps[2] == temps[1])  temps[2]++; //  Avoid div 0 error
	                if  (temps[2] < temps[1]) {   //  swap. Saves remembering which comes first on command line
	                    temps[3] = temps[1];
	                    temps[1] = temps[2];
	                    temps[2] = temps[3];
	                }
	                if  (in_range(temps[1], LV_TAILOFF_MIN) && in_range(temps[2], LV_TAILOFF_MAX))   {
	                    wr (temps[1], LV_TAILOFF_MIN);
	                    wr (temps[2], LV_TAILOFF_MAX);
	                    len = sprintf (txt, "Set good voltage tailoff values %ld, %ld\r\n", temps[1], temps[2]);
	                }
	                else {
	                    len = sprintf (txt, "NOT set voltage tailoff values %ld, %ld\r\n", temps[1], temps[2]);
	                }
	                pc.write ((uint8_t*)txt, len);
	                break;
	            case    99876: //  set to defaults
	                set_defaults   ();
	                pc.write	((uint8_t*)"Got case -99 set defaults\r\n", 27);
	                break;
	/*            case    9:      //  9 Save settings
	                save   ();
	                pc.printf   ("Saving settings to EEPROM\r\n");
	                break;*/
	            default:
	                len = sprintf     (txt, "Not found - user setting %ld\r\n", i);
	                pc.write    ((uint8_t*)txt, len);
//	                i = 0;
	                break;
	        }       //  endof   switch
	        if  (i) {
	            save    ();
	            pc.write   ((uint8_t*)"Saving settings to EEPROM\r\n", 27);
	        }
	    }           //  endof   //  If more than 0 parameters supplied
	    else    {   //  command was just "us" on its own
	        pc.write   ((uint8_t*)"\r\nNo Changes. List of User Settings : -\r\n", 41);
	    }
	    pc.write	((uint8_t*)"us defaults\t\tSet default values\r\n", 33);
	    for	(int j = 1; j < numof_eeprom_options; j++)	{
	    	if	(user_settings_list[j].style == 'I')
	    		len = sprintf	(txt, "us %d\t%s = %ld\r\n", j, t(j), values.i[j]);
	    	if	(user_settings_list[j].style == 'C')
	    		len = sprintf	(txt, "us %d\t%s = \'%c\'\r\n", j, t(j), (int)values.i[j]);
    	    pc.write    ((uint8_t*)txt, len);
	    }

/*
	//                  WHEELDIA, MOTPIN, WHEELGEAR, used in converting RPM to MPH
	//    pc.write    (t, strlen(t));
	//  5"      100dia, 27, 85      (3.15)
	//  7.25"   146dia, 17, 76      (4.47)
	// Julie's Baby Deltic 16, 69   (4.31)	*L*** Now 15:70 (4.6667)
	//    pc.printf   ("us 9\tSave settings\r\r\n");*/
}


bool        i2eeprom_settings::set_defaults    () {         //  Put default settings into EEPROM and local buffer
	bool	rv;
    for (int i = 0; i < numof_eeprom_options; i++)  {
        values.i[i] = user_settings_list[i].de_fault;       //  Load defaults and 'Save Settings'
    }
    rv =  save    ();
    if	(rv)
    	pc.write	((uint8_t*)"save GOOD\r\n", 11);
    else
    	pc.write	((uint8_t*)"save BAD!", 11);
    return	rv;
}

int32_t     i2eeprom_settings::rd  (uint32_t addr)	{	//  ;           //  Read one setup char value from private buffer 'settings'
	return	values.i[addr];
}

//    bool        wr  (char, uint32_t)  ;     //  Write one setup char value to private buffer 'settings'
bool        i2eeprom_settings::wr  (int32_t val, uint32_t addr)	{	//  ;     //  Write one setup char value to private buffer 'settings'
	if	(addr > MAX_SETTINGS)
		return	false;
	values.i[addr] = val;
	return	true;
}

float       i2eeprom_settings::get_top_MPH ()  {
    return  values.f[TOP_SPEED_MPH];
}

float       i2eeprom_settings::get_top_RPM ()  {
    return  values.f[TOP_SPEED_MPH] / rpm2mph();
}

float   i2eeprom_settings::rpm2mph    ()  {
    return  frpm2mph;
}

float   i2eeprom_settings::rpm2mph    (float rpm)  {
    return  frpm2mph * rpm;
}

int32_t    i2eeprom_settings::min (uint32_t i)    {
    if  (i >= numof_eeprom_options)
        i = numof_eeprom_options - 1;
    return  user_settings_list[i].min;
}

int32_t    i2eeprom_settings::max (uint32_t i)    {
    if  (i >= numof_eeprom_options)
        i = numof_eeprom_options - 1;
    return  user_settings_list[i].max;
}

int32_t    i2eeprom_settings::def (uint32_t i)    {
    if  (i >= numof_eeprom_options)
        i = numof_eeprom_options - 1;
    return  user_settings_list[i].de_fault;
}


const char *  i2eeprom_settings::t  (uint32_t    i)  {
    if  (i >= numof_eeprom_options)
        i = numof_eeprom_options - 1;
    return  user_settings_list[i].txt;
}

/**
    *   bool    eeprom_settings::in_range   (int32_t val, uint32_t p)  {   //  true if val is in range
    *   Oct 22 changed char to int32_t
*/
bool    i2eeprom_settings::in_range   (int32_t val, uint32_t p)  {   //  true if val is in range
    return  ((val >= user_settings_list[p].min) && (val <= user_settings_list[p].max))  ;
}


void	set_speed_limit	(float mph)	{
    int32_t    sp = (int32_t) (mph * 10.0);    //  Upgraded Oct 2022
    if  (sp > user_settings_list[TOP_SPEED_MPH].max)   sp = user_settings_list[TOP_SPEED_MPH].max;
    if  (sp < user_settings_list[TOP_SPEED_MPH].min)   sp = user_settings_list[TOP_SPEED_MPH].min;
    j_settings.wr    (sp, TOP_SPEED_MPH);
    j_settings.save   ();

}


bool	i2eeprom_settings::read_ee	(uint32_t address, const char *buffer, uint32_t size)	{
	HAL_StatusTypeDef	ret;	//	Used to test return results of HAL functions
	uint8_t	addr[4];
	addr[0] = address >> 8;
	addr[1] = address & 0xff;
	ret = HAL_I2C_Master_Transmit	(&hi2c1, LC64_ADDR, addr, 2, 10);	//	write 2 byte address
	if	(ret != HAL_OK)
		return	false;
	HAL_Delay(1);
  	ret = HAL_I2C_Master_Receive	(&hi2c1, LC64_ADDR, (uint8_t*)buffer, size, 100);
	if	(ret != HAL_OK)
		return	false;
	return	true;
}


bool	i2eeprom_settings::write_ee	(uint32_t address, const char *buffer, uint32_t size)	{
		HAL_StatusTypeDef	ret;	//	Used to test return results of HAL functions
		uint8_t	values[40];
		// Check the address and size fit onto the chip.
		if	((address + size) >= 8192)
			return	false;
	    const char *page = buffer;
	    uint32_t 	left = size;
	    // While we have some more data to write.
	    while (left != 0) {
	        // Calculate the number of bytes we can write in the current page.
	        // If the address is not page aligned then write enough to page
	        // align it.
	        uint32_t toWrite;
	        if ((address % EEPROM_PAGE_SIZE) != 0) {
	            toWrite = (((address / EEPROM_PAGE_SIZE) + 1) * EEPROM_PAGE_SIZE) - address;
	            if (toWrite > size) {
	                toWrite = size;
	            }
	        } else {
	            if (left <= EEPROM_PAGE_SIZE) {
	                toWrite = left;
	            } else {
	                toWrite = EEPROM_PAGE_SIZE;
	            }
	        }
	        //printf("Writing [%.*s] at %d size %d\n\r", toWrite, page, address, toWrite);
	        // Start the page write with the address in one write call.
	        values[0] = (char)(address >> 8);
	        values[1] = (char)(address & 0xFF);

	        for (uint32_t count = 0; count != toWrite; ++count)
	        	values[count + 2] = *page++;
	        ret = HAL_I2C_Master_Transmit	(&hi2c1, LC64_ADDR, values, toWrite + 2, 100);	//	write 2 byte address followed by n data
	        if	(ret != HAL_OK)
	        	return	false;

	        HAL_Delay(5);			//        waitForWrite();
	        left -= toWrite;        // Update the counters with the amount we've just written
	        address += toWrite;
	    }
		return	true;
}


const   float   RPM_TO_MPH_FACTOR = 0.0001171254;  //  Needs ratio and wheel size

bool	i2eeprom_settings::load	()	{
	eeprom_status = true;
	tmp[0] = (SETTINGS_BASE_ADDRESS >> 8);
	tmp[1] = (SETTINGS_BASE_ADDRESS & 0xff);
	ret = HAL_I2C_Master_Transmit	(&hi2c1, LC64_ADDR, tmp, 2, 10);	//	write 2 byte address
	if	(ret != HAL_OK)
		eeprom_status = false;
	HAL_Delay(1);
	ret = HAL_I2C_Master_Receive	(&hi2c1, LC64_ADDR, (uint8_t*)values.i, MAX_SETTINGS * 4, 100);
	if	(ret != HAL_OK)	{
		eeprom_status = false;
	}

	frpm2mph =	RPM_TO_MPH_FACTOR * (float)values.i[WHEELDIA] * (float)values.i[MOTPIN] / (float)values.i[WHEELGEAR];
	update_floats	()	;
	return	eeprom_status;
}

void    i2eeprom_settings::update_floats ()  {    //  All floats derived from chars settings.c[n]
    for(int j = 0; j < MAX_SETTINGS; j++)
        values.f[j] = (float)values.i[j];

    values.f[POT_REGBRAKE_RANGE]      /= 100.0;    //  Scale percentage to 0.0 to 1.0
    values.f[BRAKE_EFFECTIVENESS]     /= 100.0;
    values.f[TOP_SPEED_MPH]           /= 10.0;
    values.f[POT_V_RANGE]             *= 1524.0;
    values.f[I_SCALER]                /= 100.0;
}

bool	i2eeprom_settings::save	()	{
	char * buff = (char *)values.i;
	return	write_ee	(SETTINGS_BASE_ADDRESS, buff, MAX_SETTINGS * 4);
}

/*bool	i2eeprom_settings::get_status	()	{
	return	eeprom_status;
}*/


//bool	read_temperature	(float * temperature)	{
bool	read_temperature	(float & temperature)	{
	HAL_StatusTypeDef	ret;	//	Used to test return results of HAL functions
	uint8_t	i2c_tx_buff[10], i2c_rx_buff[10];
	  i2c_tx_buff[0] = i2c_tx_buff[1] = 0;	//	pointer to temperature data
	  ret = HAL_I2C_Master_Transmit	(&hi2c1, LM75_ADDR, i2c_tx_buff, 1, 10);
	  if	(ret != HAL_OK)	{
		  return	false;
	  }
	  else	{	//	i2c send addr to LM75 worked
		  int16_t tmp = 0;
		  ret = HAL_I2C_Master_Receive	(&hi2c1, LM75_ADDR, i2c_rx_buff, 2, 10);
		  if	(ret != HAL_OK)	{
			  return	false;
		  }
		  else	{	//	read two temperature bytes
			  tmp = ((int16_t)i2c_rx_buff[0] << 3) | (i2c_rx_buff[1] >> 5);
			  if(tmp > 0x3ff)	{	//	temperature is below 0
				  tmp |= 0xf800;	//	sign extend
			  }
//			  *temperature = ((float) tmp) / 8.0;	//
			  temperature = ((float) tmp) / 8.0;	//
		  }
	  }
	return	true;
}




























/*struct  usettings_optpar    ouser_settings_list[] = {    //  Position in this list is very touchy.
    {0, 1, 1,     "MotorA direction [0 or 1]"},       //  MOTADIR               0
    {0, 1, 0,     "MotorB direction [0 or 1]"},       //  MOTBDIR               1
    {4, 8, 8,     "MotorA poles 4 or 6 or 8"},      //  MOTAPOLES               2
    {4, 8, 8,     "MotorB poles 4 or 6 or 8"},      //  MOTBPOLES               3
    {1, 4, 1,     "MotorA 0R05 shunt Rs 1 to 4"},        //  ISHUNTA            4
    {1, 4, 1,     "MotorB 0R05 shunt Rs 1 to 4"},        //  ISHUNTB            5
    {10, 50, 20,  "POT_REGBRAKE_RANGE percentage 10 to 50"},     //  POT_REGBRAKE_RANGE         6
    {0, 1, 0,     "Servo1 out 0 = Disabled, 1= Output enabled"},        //  SVO1                7
    {0, 1, 0,     "Servo2 out 0 = Disabled, 1= Output enabled"},        //  SVO2                8
    {0, 2, 0,     "RC Input1 0 = Not used, 1= Uni_dir, 2= Bi_dir"},     //  RCIN1               9

    {0, 2, 0,     "RC Input2 0 = Not used, 1= Uni_dir, 2= Bi_dir"},     //  RCIN2               10
    {2, 6, 2,     "Command source 2= COM2 (Touch Screen), 3= Pot, 4= RCIn1, 5= RCIn2, 6=RCin_both"},     //  COMM_SRC 11
    {'1', '9', '0',     "Alternative ID ascii '1' to '9'"}, //  BOARD_ID    defaults to '0' before eerom setup for first time 12
    {10, 250, 60,       "Top speed MPH, range 1.0 to 25.0"},    //  New Jan 2019 TOP_SPEED 13
    {50, 253, 98,       "Wheel diameter mm"},   //  New 01/06/2018 14
    {10, 253, 27,       "Motor pinion"},   //  New 01/06/2018 15
    {20, 253, 85,       "Wheel gear"},   //  New 01/06/2018     ** NOTE this and above two used to calculate RPM to MPH ** 16
    {0, 255, 12,     "RC in 1 trim"},    //  New Dec 2019 RCI1_TRIM read as 2's complement (-128 to +127), user enters -128 to +127
    {0, 255, 12,     "RC in 2 trim"},    //  New Dec 2019 RCI2_TRIM read as 2's complement (-128 to +127), user enters -128 to +127
    {10, 50, 20,     "RCIN_REGBRAKE_RANGE stick range percent 10 to 50"},     //  RCIN_REGBRAKE_RANGE 19

    {10, 90, 50,     "RCIN_STICK_ATTACK rate percent 10 to 90"},     //  RCIN_STICK_ATTACK 20
    {0, 1, 0,     "RCIN1 direction swap 0 normal, 1 reverse"},     //  RCIN1REVERSE 21
    {0, 1, 0,     "RCIN2 direction swap 0 normal, 1 reverse"},     //  RCIN2REVERSE 22
    {15, 60, 48,    "Regen brake backoff voltage, used to limit regen voltage to safe level"},    //   NOM_SYSTEM_VOLTS 23
    {5, 91, 90,    "Brake Effectiveness percentage"},   //  24
//    {1, 5, 1,       "Baud rate, [1=9k6, 2=19k2, 3=38k4, 4=78k6, 5=115k2] = "},   //  BAUD 1=9k6, 2=19k2, 3=38k4, 4=78k6, 5=115k2
//  baud rate now fixed at 115200
    {25, 100, 50,    "Limit max motor current to this percentage"}, //  25
    {20, 50, 43,    "Pot Input Voltage Range times 10, range 20 to 43"},   //     POT_V_RANGE New 28th May 2021 in V2.0.1
//NO    {0, 1, 0,       "Direction Input is [0] Direction, [1] ESTOP"},   //  DIR_ESTOP Repurpose 'direction' input as ESTOP
    {0, 100, 0,     "Watchdog [1] Enable, [0] Disable"},   //us32
    {0, 100, 0,     "Future 28"},
    {0, 100, 0,     "Future 29"},
    {0, 100, 0,     "Future 30"},
    {0, 100, 0,     "Future 31"},
    {0, 100, 0,     "Future 32"},
    {10, 60, 10,    "Low Volt Tailoff zero power volts P1 "},   //  us36
    {12, 62, 12,    "Full power volts P2 "},

//    {0, 100, 0,     "Future 35"},
//    {0, 100, 0,     "Future 36"},
//    {0, 100, 0,     "Future 37"},
}   ;

const   int    numof_eeprom_options    = sizeof(ouser_settings_list) / sizeof (struct usettings_optpar);
*/
/*
float      eeprom_settings::get_float  (uint8_t which_setting)  {   //  New Sep 2022
    if(which_setting >= MAX_NUMOF_SETTINGS)
        return -1.0;
    return settings.f[which_setting];
}
*/
/*float      eeprom_settings::get_pot_fsd_scaler  ()  {   //  New May 2021. In user_settings, can scale pot fsd from 2.0 to 5.0 volt
    return settings.f[POT_V_RANGE];
}*/
/*
float      eeprom_settings::get_tailoff_factor (float v_link)  {  //  Takes link voltage as param, returns 0.0 <= limit <= 1.0
    float   rv = (v_link - settings.f[LV_TAILOFF_MIN]) / (settings.f[LV_TAILOFF_MAX] - settings.f[LV_TAILOFF_MIN]) ;
    if  (rv > 1.0)    rv = 1.0    ;
    if  (rv < 0.0)    rv = 0.0    ;
    return rv;
}
*/
/*float       eeprom_settings::get_brake_backoff_volts () {   //  New Sept 2022
    return settings.f[BRAKE_BACKOFF_VOLTS];
}*/

/**
    *   get_current_scaler()
    *   returns value from user settings us 30 "Limit max motor current to this percentage, 25 to 100"
*/
/*float      eeprom_settings::get_current_scaler  ()  {
    return  settings.f[I_SCALER];
}*/
/*
float       eeprom_settings::get_top_MPH ()  {
    return  settings.f[TOP_SPEED_MPH];
}

float       eeprom_settings::get_top_RPM ()  {
    return  settings.f[TOP_SPEED_MPH] / rpm2mph();
}
*/
/*float   eeprom_settings::get_brake_effectiveness    ()  {
    return  settings.f[BRAKE_EFFECTIVENESS];
}*/

/*float   eeprom_settings::get_user_brake_range    ()  {
    return  settings.f[POT_REGBRAKE_RANGE];
}*/
/*
float   eeprom_settings::rpm2mph    ()  {
    return  frpm2mph;
}

float   i2eeprom_settings::rpm2mph    (float rpm)  {
    return  frpm2mph * rpm;
}
*/
/*void    eeprom_settings::edit   (float * dbl, uint32_t numof_dbls, const char * cl)    {
extern  void    set_RCIN_offsets    ()  ;
//    const char* labs[3] = {"Disabled","Uni_directional","Bi_directional"};
//    char    temps[MAX_CLI_PARAMS+1];  //  max num of parameters entered from pc terminal
    int32_t    temps[MAX_CLI_PARAMS+1];  //  max num of parameters entered from pc terminal
    char    txt[100];
    int     len;        //  Used to catch strlen as return value of sprintf
    uint32_t i;
//    double  user_scratch;
    double  topspeed;   //  New Jan 2019 - set max loco speed
    len = sprintf     (txt, "\r\nus - User Settings data in EEPROM\r\nSyntax 'us' with no parameters lists current state.\r\n");
    pc.write    ((uint8_t*)txt, len);
    if  (numof_dbls > 0)  {           //  If more than 0 parameters supplied
        if  (numof_dbls > MAX_CLI_PARAMS)
            numof_dbls = MAX_CLI_PARAMS;
        for (i = 0; i < numof_dbls; i++)
//            temps[i] = (char)dbl[i];          //  recast doubles to char
            temps[i] = (int32_t)dbl[i];          //  recast doubles to int32_t Oct 2022
        while   (MAX_CLI_PARAMS > i)
            temps[i++] = 0;
        i = temps[0];
        switch  (i) {   //  First number read from user response after "mode"
            case    11:      //  MotorA_dir [0 or 1], MotorB_dir [0 or 1]
                wr  (temps[1], MOTADIR);
                wr  (temps[2], MOTBDIR);
                break;
            case    12:      //  MotorA_poles [4,6,8], MotorB_poles [4,6,8]
                if  (temps[1] == 4 || temps[1] == 6 || temps[1] == 8)
                    wr    (temps[1], MOTAPOLES);
                if  (temps[2] == 4 || temps[2] == 6 || temps[2] == 8)
                    wr    (temps[2], MOTBPOLES);
                break;
            case    13:      //  MotorA_ current sense resistors [1 to 4], MotorB_ current sense resistors [1 to 4]
                wr  (temps[1], ISHUNTA);     //  Corrected since published
                wr  (temps[2], ISHUNTB);
                break;
            case    14:
                wr  (temps[1], BRAKE_EFFECTIVENESS);
                break;
            case    15:
                len = sprintf   (txt, "POT_REGBRAKE_RANGE value entered = %ld\r\n", temps[1]);
                pc.write((uint8_t*)txt, len);
                if  (!in_range    (temps[1], POT_REGBRAKE_RANGE))
                    temps[1] = def(POT_REGBRAKE_RANGE);
                wr    (temps[1], POT_REGBRAKE_RANGE);
                break;
#ifdef SERVO_ENABLE
            case    16:      //  2 Servo1 [0 or 1], Servo2 [0 or 1]
                wr  (temps[1], SVO1);
                wr  (temps[2], SVO2);
                break;
#endif
#ifdef RADIO_CONTROL_ENABLE
            case    17:      //  3 RCIn1 [0 or 1], RCIn2 [0 or 1]
                wr  (temps[1], RCIN1);
                wr  (temps[2], RCIN2);
                break;
            case    18:
                wr  (temps[1], RCIN1REVERSE);
                break;
            case    19:
                wr  (temps[1], RCIN2REVERSE);
                break;


            case    21:      //  3 RCIn1 trim [-128 to +127]
            case    22:      //  3 RCIn2 trim [-128 to +127]
                user_scratch = dbl[1];
                if  (user_scratch > 127.0)  user_scratch = 127.0;
                if  (user_scratch < -128.0) user_scratch = -128.0;
//                wr    (((signed char) user_scratch), i == 21 ? RCI1_TRIM : RCI2_TRIM);
                wr    (((int32_t) user_scratch), i == 21 ? RCI1_TRIM : RCI2_TRIM);
                set_RCIN_offsets    ()  ;
                break;
            case    23:     //  RCIN_REGBRAKE_RANGE
                wr    (temps[1], RCIN_REGBRAKE_RANGE);
                break;
            case    24:      //  RCIN_STICK_ATTACK
                wr    (temps[1], RCIN_STICK_ATTACK);
                break;
#endif
            case    25:      //  Board ID '0' to '9'
                if  (temps[1] <= 9)    //  pointless to compare unsigned integer with zero
                    wr  ('0' | temps[1], BOARD_ID);
                break;
            case    26:      //  TOP_SPEED
                topspeed = dbl[1];
                if  (topspeed > 25.0)   topspeed = 25.0;
                if  (topspeed < 1.0)    topspeed = 1.0;
                wr  ((char)(topspeed * 10.0), TOP_SPEED_MPH);
                break;
            case    27:      //  Wheel dia mm, Motor pinion teeth, Wheel gear teeth
                wr  (temps[1], WHEELDIA);     //  These must be correct for speed reading to be correct
                wr  (temps[2], MOTPIN);
                wr  (temps[3], WHEELGEAR);
                break;
            case    28:      //    {2, 5, 2, "Command source 2= COM2 (Touch Screen), 3= Pot, 4= RC Input1, 5= RC Input2, 6=RC1+2 Robot"},
                if  (temps[1] > 1 && temps[1] <= 6)
                    wr  (temps[1], COMM_SRC);
                break;
            case    29: //  Nominal System Voltage  ** Sept 2022 * Repurposed BRAKE_BACKOFF_VOLTS
//                wr    (temps[1], NOM_SYSTEM_VOLTS);
                wr    (temps[1], BRAKE_BACKOFF_VOLTS);
                break;
            case    30: //  Current Scaler  **NEW** Work in progress May 2021
                wr  (temps[1], I_SCALER);
                len = sprintf(txt, "In us got [%s], %.0f, %.0f, %.0f, numof_dbls %ld\r\n", cl, dbl[0], dbl[1], dbl[2], numof_dbls);
                pc.write((uint8_t*)txt, len);
                break;
            case    31: //  Pot FSD now selectable 2.0 to 5.0 volts  **NEW** 28th May 2021
                wr  (temps[1], POT_V_RANGE);
                break;
//            case    32:
//                wr(temps[1], DIR_ESTOP);
//                break;
            case    32: //  Watchdog Enable [1], Disable [0]
                break;
            case    36: //  Low voltage Tailoff P1, P2
//                len = sprintf (txt, "In us, got min %d, max %d\r\n", (int)dbl[0], (int)dbl[1]) ;
//                len = sprintf (txt, "In us, got min %d %s, max %d %s\r\n", temps[1],
//                            in_range(temps[1], LV_TAILOFF_MIN) ? "good":"bad", temps[2],
//                            in_range(temps[2], LV_TAILOFF_MAX) ? "good":"bad" );
//                pc.write(txt, len);
                if  (temps[2] == temps[1])  temps[2]++; //  Avoid div 0 error
                if  (temps[2] < temps[1]) {   //  swap. Saves remembering which comes first on command line
                    temps[3] = temps[1];
                    temps[1] = temps[2];
                    temps[2] = temps[3];
                }
                if  (in_range(temps[1], LV_TAILOFF_MIN) && in_range(temps[2], LV_TAILOFF_MAX))   {
                    wr (temps[1], LV_TAILOFF_MIN);
                    wr (temps[2], LV_TAILOFF_MAX);
                    len = sprintf (txt, "Set good voltage tailoff values %ld, %ld\r\n", temps[1], temps[2]);
                }
                else {
                    len = sprintf (txt, "NOT set voltage tailoff values %ld, %ld\r\n", temps[1], temps[2]);
                }
                pc.write ((uint8_t*)txt, len);
                break;
            case    83: //  set to defaults
                set_defaults   ();
                break;*/
//            case    30: //  BAUD now fixed 115200
//                wr  (temps[1], BAUD);
//                break;
/*            case    9:      //  9 Save settings
                save   ();
                pc.printf   ("Saving settings to EEPROM\r\n");
                break;*/
/*            default:
                len = sprintf     (txt, "Not found - user setting %ld\r\n", i);
                pc.write    ((uint8_t*)txt, len);
                i = 0;
                break;
        }       //  endof   switch
        if  (i) {
            save    ();
            pc.write   ((uint8_t*)"Saving settings to EEPROM\r\n", 27);
        }
    }           //  endof   //  If more than 0 parameters supplied
    else    {   //  command was just "mode" on its own
        pc.write   ((uint8_t*)"No Changes\r\n", 12);
    }
    len = sprintf     (txt, "us 11\t%s = %ld, %s = %ld\r\n",
                        t(MOTADIR), settings.i[MOTADIR], t(MOTBDIR), settings.i[MOTBDIR]);
    pc.write    ((uint8_t*)txt, len);
    len = sprintf     (txt, "us 12\t%s = %ld, %s = %ld\r\n",
                        t(MOTAPOLES), settings.i[MOTAPOLES], t(MOTBPOLES), settings.i[MOTBPOLES]);
    pc.write    ((uint8_t*)txt, len);
    len = sprintf     (txt, "us 13\tNumof motor current shunt resistors [%ld to %ld], MotorA = %ld, MotorB = %ld\r\n",
                        min(ISHUNTA), max(ISHUNTA), settings.i[ISHUNTA], settings.i[ISHUNTB]);
    pc.write    ((uint8_t*)txt, len);
    len = sprintf     (txt, "us 14\t%s [min %ld, max %ld] = %ld\r\n",
                        t(BRAKE_EFFECTIVENESS), min(BRAKE_EFFECTIVENESS), max(BRAKE_EFFECTIVENESS), settings.i[BRAKE_EFFECTIVENESS]);
    pc.write    ((uint8_t*)txt, len);
    len = sprintf     (txt, "us 15\t%s[%ld to %ld] = %ld\r\n",
                        t(POT_REGBRAKE_RANGE), min(POT_REGBRAKE_RANGE), max(POT_REGBRAKE_RANGE), settings.i[POT_REGBRAKE_RANGE]);
    pc.write    ((uint8_t*)txt, len);
#ifdef SERVO_ENABLE
    len = sprintf     (txt, "us 16\tServo1 [0 or 1] = %d %s, Servo2 [0 or 1] = %d %s, Stepper Mot drv %s\r\n",
                        settings.i[SVO1], settings.i[SVO1] == 0 ? "Disabled":"Enabled", settings[SVO2], settings[SVO2] == 0 ? "Disabled":"Enabled", (settings[SVO1] == 0 && settings[SVO2] == 0) ? "Enabled":"Disabled");
    pc.write    ((uint8_t*)txt, len);
#endif
#ifdef RADIO_CONTROL_ENABLE
    len = sprintf     (txt, "us 17\tRCIn1 [0 disable, 1 Uni_dir, 2 Bi_dir] = %d, %s\r\n\tRCIn2 [0 disable, 1 Uni_dir, 2 Bi_dir] = %d, %s\r\n",
                        settings.i[RCIN1], labs[settings.i[RCIN1]], settings.c[RCIN2], labs[rd(RCIN2)]);
    pc.write    ((uint8_t*)txt, len);
    len = sprintf     (txt, "us 18\t%s RCIN1 = %d, %s\r\n", t(RCIN1REVERSE), settings.i[RCIN1REVERSE], settings.i[RCIN1REVERSE] == 0 ? "NORMAL":"REVERSE");
    pc.write    ((uint8_t*)txt, len);
    len = sprintf     (txt, "us 19\t%s RCIN2 = %d, %s\r\n", t(RCIN2REVERSE), settings.i[RCIN2REVERSE], settings.i[RCIN2REVERSE] == 0 ? "NORMAL":"REVERSE");
    pc.write    ((uint8_t*)txt, len);
    len = sprintf     (txt, "us 21\tRCIn1 two's comp trim, [-128 to +127] = %d\r\n", (signed char) settings.i[RCI1_TRIM]);
    pc.write    ((uint8_t*)txt, len);
    len = sprintf     (txt, "us 22\tRCIn2 two's comp trim, [-128 to +127] = %d\r\n", (signed char) settings.i[RCI2_TRIM]);
    pc.write    ((uint8_t*)txt, len);
    len = sprintf     (txt, "us 23\tRCIn Regen braking uses this pcntage of movement range, [%d to %d] = %d\r\n",
                        min(RCIN_REGBRAKE_RANGE), max(RCIN_REGBRAKE_RANGE), settings.i[RCIN_REGBRAKE_RANGE]);
    pc.write    ((uint8_t*)txt, len);
    len = sprintf     (txt, "us 24\tRCIn Stick move Attack rate, [%d to %d] = %d\r\n",
                        min(RCIN_STICK_ATTACK), max(RCIN_STICK_ATTACK), settings.i[RCIN_STICK_ATTACK]);
    pc.write    ((uint8_t*)txt, len);
#endif
    len = sprintf     (txt, "us 25\tBoard ID ['0' to '9'] = '%c'\r\n", (int)settings.i[BOARD_ID]);
    pc.write    ((uint8_t*)txt, len);
    len = sprintf     (txt, "us 26\t%s = %.1f\r\n", t(TOP_SPEED_MPH), double(settings.i[TOP_SPEED_MPH]) / 10.0);
    pc.write    ((uint8_t*)txt, len);
//                  WHEELDIA, MOTPIN, WHEELGEAR, used in converting RPM to MPH
//    pc.write    (t, strlen(t));
//  5"      100dia, 27, 85      (3.15)
//  7.25"   146dia, 17, 76      (4.47)
// Julie's Baby Deltic 16, 69   (4.31)
    len = sprintf     (txt, "us 27\t%s = %ld, %s = %ld, %s = %ld\r\n",
                        t(WHEELDIA), settings.i[WHEELDIA], t(MOTPIN), settings.i[MOTPIN], t(WHEELGEAR), settings.i[WHEELGEAR]);
    pc.write    ((uint8_t*)txt, len);
    len = sprintf     (txt, "us 28\tCommand Src [%ld] - 2=COM2 (Touch Screen), 3=Pot, 4=RC In1, 5=RC In2, 6=RC1+2\r\n", settings.i[COMM_SRC]);
    pc.write    ((uint8_t*)txt, len);
    len = sprintf     (txt, "us 29\t%s = %ld\r\n", t(BRAKE_BACKOFF_VOLTS), settings.i[BRAKE_BACKOFF_VOLTS]);
    pc.write    ((uint8_t*)txt, len);
    len = sprintf     (txt, "us 30\t%s = %ld\r\n", t(I_SCALER), settings.i[I_SCALER]);
    pc.write    ((uint8_t*)txt, len);
    len = sprintf     (txt, "us 31\t%s = %ld\r\n", t(POT_V_RANGE), settings.i[POT_V_RANGE]);
    pc.write    ((uint8_t*)txt, len);
//    sprintf     (txt, "us 32\t%s = %d\r\n", t(DIR_ESTOP), settings[DIR_ESTOP]);
//    pc.write    (txt, strlen(txt));
    len = sprintf     (txt, "us 32\t%s = %ld\r\n", t(WD_EN), settings.i[WD_EN]); //  31 Aug 2021
    pc.write    ((uint8_t*)txt, len);
    len = sprintf   (txt, "us 36\t%s = %ld, %s = %ld\r\n", t(LV_TAILOFF_MIN), settings.i[LV_TAILOFF_MIN],
                                                         t(LV_TAILOFF_MAX), settings.i[LV_TAILOFF_MAX]);
    pc.write    ((uint8_t*)txt, len);
    len = sprintf     (txt, "us 83\tSet to defaults\r\n");
    pc.write    ((uint8_t*)txt, len);
//    pc.printf   ("us 9\tSave settings\r\r\n");
}
*/
/**
    *   bool    eeprom_settings::in_range   (int32_t val, uint32_t p)  {   //  true if val is in range
    *   Oct 22 changed char to int32_t
*/
/*bool    eeprom_settings::in_range   (int32_t val, uint32_t p)  {   //  true if val is in range
    return  ((val >= ouser_settings_list[p].min) && (val <= ouser_settings_list[p].max))  ;
}
*/
/*
uint32_t    eeprom_settings::min (uint32_t i)    {
    if  (i >= numof_eeprom_options)
        i = numof_eeprom_options - 1;
    return  user_settings_list[i].min;
}

uint32_t    eeprom_settings::max (uint32_t i)    {
    if  (i >= numof_eeprom_options)
        i = numof_eeprom_options - 1;
    return  user_settings_list[i].max;
}

uint32_t    eeprom_settings::def (uint32_t i)    {
    if  (i >= numof_eeprom_options)
        i = numof_eeprom_options - 1;
    return  user_settings_list[i].de_fault;
}
*/
/*
int32_t    eeprom_settings::min (uint32_t i)    {
    if  (i >= numof_eeprom_options)
        i = numof_eeprom_options - 1;
    return  ouser_settings_list[i].min;
}

int32_t    eeprom_settings::max (uint32_t i)    {
    if  (i >= numof_eeprom_options)
        i = numof_eeprom_options - 1;
    return  ouser_settings_list[i].max;
}

int32_t    eeprom_settings::def (uint32_t i)    {
    if  (i >= numof_eeprom_options)
        i = numof_eeprom_options - 1;
    return  ouser_settings_list[i].de_fault;
}


const char *  eeprom_settings::t  (uint32_t    i)  {
    if  (i >= numof_eeprom_options)
        i = numof_eeprom_options - 1;
    return  ouser_settings_list[i].txt;
}

bool        eeprom_settings::set_defaults    () {         //  Put default settings into EEPROM and local buffer
    for (int i = 0; i < numof_eeprom_options; i++)  {
//        settings.c[i] = (char)user_settings_list[i].de_fault;       //  Load defaults and 'Save Settings'
        settings.i[i] = ouser_settings_list[i].de_fault;       //  Load defaults and 'Save Settings'
    }
    return  save    ();
}

bool        eeprom_settings::good    () {         //
    return  romgood;
}
*/
//  size_t I2C_Eeprom::write(size_t address, char value) {
//  size_t I2C_Eeprom::write(size_t address, const char *buffer, size_t size) {
//const   float   RPM_TO_MPH_FACTOR = (60.0 * PI * 39.37 / (1760.0 * 36));
//const   float   RPM_TO_MPH_FACTOR = (PI * 39.37 / 1056.0);
//const   float   RPM_TO_MPH_FACTOR = 0.0001171254;  //  Needs ratio and wheel size

/*eeprom_settings::eeprom_settings    ()  {   //  Constructor
    char    t[40], tst[40];
    //  Test for presence of eeprom first and set bool accordingly
    romgood = false;  //  Set true if eeprom found present and working correctly
    const char eetst[]={"I found the man sir\r\n"};     //  Secret message hidden in eeprom.
    for(int i = 1; i < 3; i++){                         //  If message read correctly have proved eeprom present
//        eeprom.read(64, settings.c, strlen (eetst));       //  If not, try to write it and confirm good write
//??        eeprom.read(64, tst, strlen (eetst));       //  If not, try to write it and confirm good write
//        if(strncmp(eetst, (char*)settings.i, strlen (eetst)) != 0){
        if(strncmp(eetst, tst, strlen (eetst)) != 0){
            int len = sprintf     (t, "Failed eeprom read %d   \r\n", i);
            pc.write    ((uint8_t*)t, len);
//??            eeprom.write(64, eetst, strlen (eetst));     //  Try to write max 3 times. Fail after 3 failed attempts
            wait_us(5000);
        }
        else  {
            i = 99;
            romgood = true;       //  eeprom verified
        }
    }
    if(good())  {   //  Confirmed have found working eeprom
        bool changes = false;
//        eeprom.read(128, settings.c, MAX_NUMOF_SETTINGS);     //  eeprom addr, dest buffer, length
//??        eeprom.read(SETTINGS_START_EE_ADDR, (char*)settings.i, (MAX_NUMOF_SETTINGS * 4));     //  eeprom addr, dest buffer, length
        for(uint j = 0; j < MAX_NUMOF_SETTINGS; j++)     {
            if(!in_range    (settings.i[j], j))   {
                settings.i[j] = def(j);   //  Set any out of range readings to default
                changes = true;
            }
        }
        if(changes) save();
        frpm2mph =
//                60.0                                                          //  to Motor Revs per hour;
//                * ((float)settings.i[MOTPIN] / (float)settings.i[WHEELGEAR])  //  Wheel revs per hour
//                * PI * ((float)settings.i[WHEELDIA] / 1000.0)                  //  metres per hour
//                * 39.37 / (1760.0 * 36.0);                                      //  miles per hour
                RPM_TO_MPH_FACTOR *
                (float)settings.i[WHEELDIA] * (float)settings.i[MOTPIN] / (float)settings.i[WHEELGEAR];

        update_floats ();
    }
}       //  endof constructor
*/
//const   float   RPM_TO_MPH_FACTOR = (60.0 * PI * 39.37 / (1760.0 * 36));

/*void    eeprom_settings::update_floats ()  {    //  All floats derived from chars settings.c[n]
    for(uint j = 0; j < MAX_NUMOF_SETTINGS; j++)
        settings.f[j] = (float)settings.i[j];

    settings.f[POT_REGBRAKE_RANGE]      /= 100.0;    //  Scale percentage to 0.0 to 1.0
    settings.f[BRAKE_EFFECTIVENESS]     /= 100.0;
    settings.f[TOP_SPEED_MPH]           /= 10.0;
    settings.f[POT_V_RANGE]             *= 1524.0;
    settings.f[I_SCALER]                /= 100.0;
//    char txt[64];
//    int len = sprintf     (txt, "rpm2mph factor %7.6f, frpm2mph %7.6f\r\n", RPM_TO_MPH_FACTOR, frpm2mph);
//    pc.write    (txt, len);
}
*/
//char    eeprom_settings::rd  (uint32_t i)  {           //  Read one setup char value from private buffer 'settings'
/*int32_t    eeprom_settings::rd  (uint32_t j)  {           //  Read one setup char value from private buffer 'settings'
    if  (j >= MAX_NUMOF_SETTINGS)    {
        char    t[40];
        int len = sprintf   (t, "ERROR Attempt to read setting %ld\r\n", j);
        pc.write    ((uint8_t*)t, len);
        return  0;
    }
    return  settings.i[j];
}

//bool    eeprom_settings::wr  (char val, uint32_t p)  {           //  Write one setup char value to private buffer 'settings'
bool    eeprom_settings::wr  (int32_t val, uint32_t p)  {           //  Write one setup char value to private buffer 'settings'
    if  (p >= MAX_NUMOF_SETTINGS)
        return  false;
    if  (in_range (val, p))    {
        settings.i[p] = val;
        return  true;
    }
    settings.i[p] = def(p);
    return  false;
}

//  size_t I2C_Eeprom::write(size_t address, char value) {
//  size_t I2C_Eeprom::write(size_t address, const char *buffer, size_t size) {
bool    eeprom_settings::save    ()  {               //  Write 'settings' buffer to EEPROM
    update_floats ();
//    eeprom.write(32*4, settings.c, MAX_NUMOF_SETTINGS);   //  original char array
//??    eeprom.write(SETTINGS_START_EE_ADDR, (char*)settings.i, (MAX_NUMOF_SETTINGS * 4));   //  Oct 2022 migrate from char to int32_t
//    eeprom.write(32*6, (const char * )&settings, sizeof(settings));    //  Do not save floats, these recalc'd from chars
    return true;
}
*/






