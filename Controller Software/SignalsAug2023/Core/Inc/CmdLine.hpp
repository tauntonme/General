/*
 * CmdLine.hpp
 *
 *  Created on: Jun 30, 2023
 *      Author: Jon Freeman
 *
 *      Handles commands received from serial ports.
 */

#ifndef INC_CMDLINE_HPP_
#define INC_CMDLINE_HPP_

#include 	"main.h"
#include	<cstdio>
#include	<cctype>
#include	<cstdbool>
#include	<cstdlib>
#include	<string.h>

struct  parameters  {   //  Used in serial comms with pc and other controller (e.g. touch-screen)
    struct  cli_menu_entry_trio   const * command_list;
    int32_t position_in_list, numof_floats;//, target_unit, numof_menu_items;
    float  flt[MAX_CLI_PARAMS];
    const char * command_line;
    bool    respond;
}   ;

struct cli_menu_entry_trio  {    //  Commands tabulated as list of these structures as seen below
  const char * cmd_word;              //  points to command text e.g. "command_name"
  const char * explan;                //  very brief explanation or clue as to purpose of function
  void (*f)(struct parameters &);     //  points to function code for this menu choice
}  ;                                    //

class	CommandLineHandler	{
	  int	state;
	  struct parameters par;
	  struct cli_menu_entry_trio const * pcmdlist;
//		struct  cli_menu_entry_trio   const * command_list;
public:
	  CommandLineHandler	(struct cli_menu_entry_trio const * pcml)	{
//		  par.numof_menu_items = (sizeof(pc_command_list) / sizeof(cli_menu_entry_trio));
//		  par.command_list = pc_command_list;
		  par.command_list = pcml;
		  pcmdlist = pcml;
	  }	;
	  bool	CommandExec	(char * t)	;
}	;





#endif /* INC_CMDLINE_HPP_ */
