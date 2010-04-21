//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

#define RTS_BYTE 0xef
#define CTS_BYTE 0xfe

int8_t rts_send (comBuf *pkt, va_list ap);
boolean rts_recv (comBuf *pkt, uint8_t **footer, uint8_t port);
