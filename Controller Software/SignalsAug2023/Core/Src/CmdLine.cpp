/*
 * CmdLine.cpp
 *
 *  Created on: Jun 30, 2023
 *      Author: Jon Freeman
 *
 */
using namespace std;
#include	"settings.hpp"
#include	"CmdLine.hpp"
#include	"Serial.hpp"


const	char	version_str[] = "West Buck Sigs Controller v1, 20230804";
const 	char * 	get_version	()	{	return	version_str;	}

bool	eeprom_valid_flag = false;
extern	i2eeprom_settings	j_settings		;

extern	uint32_t	can_errors;

UartComChan	pc		(huart2),
			ctrl	(huart1);

extern	"C"	{
void	one_ms_update_stuff	()	{	//	Called from enclosing while(1) loop in main. Not in interrupt context.
	pc.tx_any_buffered	()	;		//	Works well !
	ctrl.tx_any_buffered	()	;
}

void	Signal_sys_Setup	()	{	//	Called from main immediately before start of forever loop
	pc.write("\r\n\n\n\n", 5);
	pc.write(version_str, strlen(version_str));
	pc.write("\r\n\n", 3);
	eeprom_valid_flag = j_settings.load	();
	if	(eeprom_valid_flag)	{	//	Read 24LC64 eeprom for any config data
		pc.write("Loaded eeprom settings ok\r\n", 27);
	}
	else	{
		pc.write("eeprom settings load FAILED\r\n", 29);
	}
}
}	//	End of extern "C" wrapper

void    null_cmd (struct parameters & a)     {
	const char t[] = "null command - does nothing!\r\n";
    pc.write(t, strlen(t));
}

extern	uint32_t	cancount;
extern	CAN_RxHeaderTypeDef	RxHeader;

void	ce_show	()	{
	char	t[96];
	int	len;
	len = sprintf(t, "Sid 0x%lx, Eid 0x%lx, IDE 0c%lx, DLC 0x%lx, FMI 0x%lx, RTR 0x%ld\r\n",
			/*can_errors, cancount, */
			RxHeader.StdId, RxHeader.ExtId, RxHeader.IDE, RxHeader.DLC, RxHeader.FilterMatchIndex, RxHeader.RTR);
	pc.write(t, len);
}

void	ce_cmd (struct parameters & a)     {
	ce_show();
}

/**
*   void    menucmd (struct parameters & a)
*
*   List available terminal commands to pc terminal. No sense in touch screen using this
*/
void    menucmd (struct parameters & a)     {
    char    txt[240];
//    if  (a.respond) {
        int len = sprintf     (txt, "\r\n\n%s\r\nListing Commands:-\r\n", get_version());
        pc.write    (txt, len);
        int i = 0;
        while	(a.command_list[i].cmd_word)	{
            int len = sprintf     (txt, "[%s]\t\t%s\r\n", a.command_list[i].cmd_word, a.command_list[i].explan);
            pc.write    (txt, len);
            i++;
        }	//	Endof while	()
        pc.write("End of List of Commands\r\n", 25);
//    }	//	Endof if (a.respond)
}

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
extern	"C"	bool	get_input_bit_debounced	(uint32_t which_input, uint32_t how_many)	;

void    i_cmd (struct parameters & a)	{	//	read an input
	char	t[55];
	int b = (int)a.flt[0];
	int	len = sprintf(t, "In%d = %c\r\n", b, get_input_bit_debounced(b,1) ? 'T' : 'F');
	pc.write(t, len);
}

void    seton_cmd (struct parameters & a)	{	//	set one or more outputs ON
	int	j, k = a.numof_floats;
	while	(k > 0)	{
		j = (int)a.flt[--k];
		set_output_bit	(j, true);
	}
}

void    clroff_cmd (struct parameters & a)	{	//	set one or more outputs OFF
	int	j, k = a.numof_floats;
	while	(k > 0)	{
		j = (int)a.flt[--k];
		set_output_bit	(j, false);
	}
}

void    vi_cmd (struct parameters & a)	{	//
	pc.write	("Nothing for vi command to do!\r\n", 31);
}

  /**
  struct  cli_menu_entry_trio      const loco_command_list[] = {
  List of commands accepted from external pc through non-opto isolated com port 115200, 8,n,1
  */
  struct  cli_menu_entry_trio       const pc_command_list[] = {
      {"?", "Lists available commands", menucmd},
      {"vi", "Fifth, do nothing very much at all really", vi_cmd},
      {"set", "set one or  more output on", seton_cmd},
      {"clr", "clr one or more output off", clroff_cmd},
	  {"i", "read an input", i_cmd},
	  {"ce", "can errors", ce_cmd},
      {"nu", "do nothing", null_cmd},
      {NULL, NULL, NULL},	//	June 2023 new end of list delimiter. No need to pass sizeof
  }   ;

  CommandLineHandler	command_line_handler	(pc_command_list);	//	Nice and clean

void	check_commands	()	{	//	Called from ForeverLoop
/**
 * bool	UartComChan::test_for_message	()	{
 *
 * Called from ForeverLoop at repetition rate
 * Returns true when "\r\n" terminated command line has been assembled in lin_inbuff
 */
	if	(pc.test_for_message())	{	//	bool return true found msg, false not found
		command_line_handler.CommandExec(pc.lin_inbuff);
	}
}


bool	CommandLineHandler::CommandExec(char * inbuff)	{
	int	cnt, cmd_wrd_len, cmd_lin_len, k;
	bool	found_cmd, got_numerics;
	cmd_lin_len = strlen	(inbuff);
	cnt = 0;
	for	(int j = 0; j < MAX_CLI_PARAMS; j++)
		par.flt[j] = 0.0;
	uint8_t * p = (uint8_t*)inbuff, * pEnd;
	found_cmd = false;
	while	(!found_cmd && pcmdlist[cnt].cmd_word)	{
		cmd_wrd_len = strlen(pcmdlist[cnt].cmd_word);
		if	((strncmp(inbuff, pcmdlist[cnt].cmd_word, cmd_wrd_len) == 0) && (!isalpha(inbuff[cmd_wrd_len])))	{	//	Don't find 'fre' in 'fred'
//			pc.write("Found! ", 7);
			found_cmd = true;
			got_numerics = false;
			par.numof_floats = 0;
			k = cmd_wrd_len;
			pEnd = p + cmd_wrd_len;
			while	(!got_numerics && (k < cmd_lin_len))	//	Test for digits present in command line
				got_numerics = isdigit(inbuff[k++]);				//	if digits found, have 1 or more numeric parameters to read

			if	(got_numerics)	{
				while   (*pEnd && (par.numof_floats < MAX_CLI_PARAMS))  {          //  Assemble all numerics as doubles
					par.flt[par.numof_floats++] = strtof    ((const char *)pEnd, (char **)&pEnd);
					while   (*pEnd && (pEnd < p + strlen(inbuff)) && *pEnd && !isdigit(*pEnd) && ('.' != *pEnd) && ('-' != *pEnd) && ('+' != *pEnd))  {   //  Can
						pEnd++;
					}   //
					if  (((*pEnd == '-') || (*pEnd == '+')) && (!isdigit(*(pEnd+1))) && ('.' !=*(pEnd+1)))
						pEnd = (uint8_t*)inbuff + strlen(inbuff);   //  fixed by aborting remainder of line
				}		//	Endof while	(*pEnd ...
			}			//	Endof if	(got_numerics)

		    pcmdlist[cnt].f(par);

		}
		else	//	Command word not matched at list position 'cnt'
			cnt++;
	}			//	End of while	(!found_cmd ...
	return	true;
}



