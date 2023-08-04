/*
 * settings.hpp
 *
 *  Created on: Jun 13, 2023
 *      Author: Jon Freeman  B Eng Hons
 */

#ifndef INC_SETTINGS_HPP_
#define INC_SETTINGS_HPP_


/*
 * settings.hpp
 *
 *  Created on: Oct 26, 2022
 *      Author: Jon Freeman
 */


//#include "stm32f4xx_hal.h"
#include "stm32l4xx_hal.h"

#define	MAX_SETTINGS	60
#define	MAX_CLI_PARAMS	20

class	i2eeprom_settings	{
private:
    struct  i_and_f {
        float   f[MAX_SETTINGS + 2];
        int32_t i[MAX_SETTINGS + 2];  //  Upgraded from char and into struct Oct 2022
    }   values ;

	bool		eeprom_status;
	uint8_t		tmp[4];
	HAL_StatusTypeDef	ret;	//	Used to test return results of HAL functions
	bool	write_ee	(uint32_t address, const char *buffer, uint32_t size)	;
    int32_t    min (uint32_t)   ;   //  Return int32_t, changed from uint_32_t Oct 22
    int32_t    max (uint32_t)   ;
    int32_t    def (uint32_t)   ;
    const char *      t   (uint32_t);   //  Returns -> option_list text entry
    bool        in_range (int32_t val, uint32_t p)  ;
    float       frpm2mph;   //, ftop_RPM; //, fcurrent_scaler;
    void        update_floats ()  ;
public:
	i2eeprom_settings	()	{
		eeprom_status = false;	//	until load has worked perfectly
	}	;
    int32_t     rd  (uint32_t)  ;           //  Read one setup char value from private buffer 'settings'
    bool        wr  (int32_t, uint32_t)  ;     //  Write one setup char value to private buffer 'settings'
	bool	read_ee	(uint32_t address, const char *buffer, uint32_t size)	;
	bool	save	()	;
	bool	load	()	;
	bool	set_defaults	()	;
    void	edit   (float * flt, uint32_t numof_floats, const char * cl)    ;

    float       get_top_MPH ()  ;
    float       get_top_RPM ()  ;
    float       get_tailoff_factor (float)  ;  //  Takes link voltage as param, returns 0.0 <= limit <= 1.0
    float       get_float  (uint8_t which_setting)  ;   //  New Sep 2022
    float       rpm2mph ()  ;
    float       rpm2mph (float)  ;
}	;

//  List user settable firmware bytes in EEROM
enum  User_Setting_Options
        {NOCMD, BOARD_ID, MOTOR_DIR, MOTOR_POLES, ISHUNTS, POT_REGBRAKE_RANGE,
        COMM_SRC, TOP_SPEED_MPH, WHEELDIA, MOTPIN, WHEELGEAR,
        BRAKE_BACKOFF_VOLTS, BRAKE_EFFECTIVENESS, I_SCALER,
        POT_V_RANGE, WD_EN,
        LV_TAILOFF_MIN, LV_TAILOFF_MAX, FUT35,}  ;
//  These represent uint32_t address offsets in 24LC64 rom user settable firmware settings





















/*
    //  List user settable firmware bytes in EEROM	TA<E THIS OUT
enum  User_Setting_Options
            {MOTADIR, MOTBDIR, MOTAPOLES, MOTBPOLES, ISHUNTA, ISHUNTB, POT_REGBRAKE_RANGE,
            SVO1, SVO2, RCIN1, RCIN2,
            COMM_SRC, BOARD_ID, TOP_SPEED_MPH, WHEELDIA, MOTPIN, WHEELGEAR,
            RCI1_TRIM, RCI2_TRIM, RCIN_REGBRAKE_RANGE, RCIN_STICK_ATTACK,     //  RC in trims new Dec 2019
            RCIN1REVERSE, RCIN2REVERSE,
            BRAKE_BACKOFF_VOLTS, BRAKE_EFFECTIVENESS, I_SCALER,
            POT_V_RANGE, DIR_ESTOP, EB28, EB29, EB30, EB31, WD_EN,
            LV_TAILOFF_MIN, LV_TAILOFF_MAX, FUT35,}  ;  //  These represent byte address offsets in 24LC64 rom user settable firmware settings

//const   int MAX_CLI_PARAMS = 22;	//	This is in wrong place
*/
//const   uint32_t    MAX_NUMOF_SETTINGS = 64;
/*
class   eeprom_settings {
    struct  char_and_float {
        float   f[MAX_NUMOF_SETTINGS];
        int32_t i[MAX_NUMOF_SETTINGS];  //  Upgraded from char and into struct Oct 2022
    }   settings ;

    bool        romgood;
    float       frpm2mph;   //, ftop_RPM; //, fcurrent_scaler;

    void        update_floats ()  ;
    bool        in_range (int32_t val, uint32_t p)  ;
    bool        set_defaults    ();         //  Put default settings into EEPROM and local buffer
    const char *      t   (uint32_t);   //  Returns -> option_list text entry
    int32_t    min (uint32_t)   ;   //  Return int32_t, changed from uint_32_t Oct 22
    int32_t    max (uint32_t)   ;
    int32_t    def (uint32_t)   ;
  public:
////    eeprom_settings (PinName sda, PinName scl); //  Constructor
    eeprom_settings (); //  Constructor
//    char        rd  (uint32_t)  ;           //  Read one setup char value from private buffer 'settings'
    int32_t     rd  (uint32_t)  ;           //  Read one setup char value from private buffer 'settings'
//    bool        wr  (char, uint32_t)  ;     //  Write one setup char value to private buffer 'settings'
    bool        wr  (int32_t, uint32_t)  ;     //  Write one setup char value to private buffer 'settings'
    bool        save    ()  ;               //  Write 'settings' buffer to EEPROM
    bool        good    ();     //  returns romgood, i.e. whether eeprom detected and found in good order
    void        edit   (float * flt, uint32_t numof_floats, const char * cl)    ;
//    float       get_user_brake_range ()  ;
//    float       get_brake_effectiveness ()  ;
//    float       get_current_scaler  ()  ;   //  New May 2021. In user_settings, can scale current down to 5% of max
//    float       get_pot_fsd_scaler  ()  ;   //  New May 2021. In user_settings, can scale pot fsd from 2.0 to 5.0 volt
//    float       get_brake_backoff_volts ();
    float       get_top_MPH ()  ;
    float       get_top_RPM ()  ;
    float       get_tailoff_factor (float)  ;  //  Takes link voltage as param, returns 0.0 <= limit <= 1.0
    float       get_float  (uint8_t which_setting)  ;   //  New Sep 2022
    float       rpm2mph ()  ;
    float       rpm2mph (float)  ;
//    uint32_t    uh();
//    uint32_t    baud    ();
}   ;

struct  usettings_optpar  {
    const int32_t min, max, de_fault;  //  min, max, default    changed up to int32_t Oct 2022
    const char * txt;     //  description
//    int32_t i;      //  Oct 22, noted is possible to mix const and variables here, no use found yet.
//    float   f;
}   ;

*/





#endif /* INC_SETTINGS_HPP_ */
