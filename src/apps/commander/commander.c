//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

#include "mos.h"
#include "msched.h"  // MANTIS scheduler (gives us start)
#include "command_daemon.h"
#include "printf.h"

void start(void)
{
   printf("Please type a command. [help] lists all commands.\n");
   mos_thread_new(mos_command_daemon, MOS_COMMANDER_STACK_SIZE,
		  PRIORITY_NORMAL);
   
#if defined(PLATFORM_MICA_ANY)
   register_mica_functions();
#endif
}
