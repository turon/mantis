#ifndef _morse_h_
#define _morse_h_
static uint8_t morse_buff[16]; //buffer for reversing itoa converisions
static uint8_t buffsize;

#define MORSE_DOT_TIME 120
static uint32_t MorseCodeData[] = { // ***  Clean me up please!  ***
   0x0000010, 0x0000000, 0x0045D5D, 0x0000000, // [' ', '!', '"', '#']
   0x00475D5, 0x0000000, 0x0000000, 0x045DDDD, // ['$', '%', '&', "'"]
   0x0045DD7, 0x0475DD7, 0x0000000, 0x001175D, // ['(', ')', '*', '+']
   0x0477577, 0x0047557, 0x011D75D, 0x0011757, // [',', '-', '.', '/']
   0x0477777, 0x011DDDD, 0x0047775, 0x0011DD5, // ['0', '1', '2', '3']
   0x0004755, 0x0001155, 0x0004557, 0x0011577, // ['4', '5', '6', '7']
   0x0045777, 0x0117777, 0x0115777, 0x01175D7, // ['8', '9', ':', ';']
   0x0000000, 0x0011D57, 0x0000000, 0x0045775, // ['<', '=', '>', '?']
   0x01175DD, 0x000011D, 0x0001157, 0x00045D7, // ['@', 'A', 'B', 'C']
   0x0000457, 0x0000011, 0x0001175, 0x0001177, // ['D', 'E', 'F', 'G']
   0x0000455, 0x0000045, 0x0011DDD, 0x00011D7, // ['H', 'I', 'J', 'K']
   0x000115D, 0x0000477, 0x0000117, 0x0004777, // ['L', 'M', 'N', 'O']
   0x00045DD, 0x0011D77, 0x000045D, 0x0000115, // ['P', 'Q', 'R', 'S']
   0x0000047, 0x0000475, 0x00011D5, 0x00011DD, // ['T', 'U', 'V', 'W']
   0x0004757, 0x0011DD7, 0x0004577            // ['X', 'Y', 'Z']
};

void morse_dot(uint8_t l)
{
    mos_led_on(l);
    mos_thread_sleep(MORSE_DOT_TIME);
    mos_led_off(l);
    mos_thread_sleep(MORSE_DOT_TIME);
}

void morse_dash(uint8_t l)
{
    mos_led_on(l);
    mos_thread_sleep(MORSE_DOT_TIME*3);
    mos_led_off(l);
    mos_thread_sleep(MORSE_DOT_TIME);
}

void morse_letter_end(uint8_t l)
{
    mos_led_off(l);
    mos_thread_sleep(MORSE_DOT_TIME*3);}

void morse_word_end(uint8_t l)
{
    mos_led_off(l);
    mos_thread_sleep(MORSE_DOT_TIME*5);
}

void morse_send_char(uint8_t letter, uint8_t led)
{
    uint32_t morse;
    if('a' <= letter && letter <= 'z')
        letter -= 'a' - 'A';

    if(' ' <= letter && letter <= 'Z')
        morse = MorseCodeData[letter - ' '];
    else
        return;
    do
    {
        if ((morse & 3) == 1)
        {
            morse_dot(led);
            morse >>=2;
        }
        else if ((morse & 15) == 7)
        {
            morse_dash(led);
            morse >>=4;
        }
    }
    while ((morse & 3) != 0);
    morse_letter_end(led);    
}


/* Convert a number to a certain radix and send over uart.
 * 16-bit version */
static void morse_norecurse_16(uint16_t input, uint8_t radix, uint8_t led)
{
    
    uint8_t remainder = input % radix;
    buffsize = 0;

    if (input == 0)
        morse_buff[buffsize++] = '0';
   
    while(input >= radix) {
        if(remainder <= 9)
            morse_buff[buffsize++] = remainder + '0';
        else
            morse_buff[buffsize++] = remainder - 10 + 'A';

        input /= radix;
        remainder = input % radix;
    }
    
    //last place
    if(remainder > 0) {
        if(remainder <= 9)
            morse_buff[buffsize++] = remainder + '0';
        else
            morse_buff[buffsize++] = remainder - 10 + 'A';
    }

    while(buffsize > 0) 
    {
        morse_send_char(morse_buff[--buffsize],led);
    }
    
}

/* Send a NULL terminated string. */
static void morse_send_string(const char *s,uint8_t led)
{
   /*while(fieldwidth--) {
      printf_packet.data[printf_packet.size++] = padchar;
      check_size();
   }*/
   while(*s) {
      morse_send_char(*s++,led);
   }
}

/** @brief printf Prints the formatted string over the serial line
 * after which a shell must display the text to the screen. Note that
 * %C is for 8 bit numbers, %d for 16-bit and %l for 32 bit. 
 * @param format Format to print
 */
int16_t morse_printf(uint8_t led, const char *format, ...)
{
    va_list arg;
    const char *p = format;

    if(format == NULL)
        return -1;
    va_start(arg, format);

    while(*p) {
        if (*p == ' ') 
        {
            p++;
            morse_word_end(led);
        }// If its not a conversion specifier we just send it
        else if(*p != '%') {
            morse_send_char(*p++, led);
        } else {
            // Otherwise we need to substitute in a variable
            p++;
            
            // TODO the padding options are not implemented for non-numeric stuff
            // Pad with zeros or spaces?
            if (*p == '0') {
//              padchar = '0';
                p++;
            }// else
//              padchar = ' ';
	 
            // Width of field
//          fieldwidth = 0;
            while(*p >= '0' && *p <= '9') {
//              fieldwidth = fieldwidth * 10 + *p - '0';
                p++;
            }
	 
            switch(*p++) {
                case 's': {
                    char *s;
                    s = va_arg(arg, char *);
                    morse_send_string(s,led);
                    break;
                }
                case 'd': {
                    uint16_t num16;
                    num16 = va_arg(arg, uint16_t);
                    morse_norecurse_16(num16, 10, led);
                    break;
                }
                case 'c': {
                    morse_send_char((uint8_t)va_arg(arg, uint16_t),led);
                    break;
                }
                case 'C': {
                    uint16_t num8 = 0;
                    num8 = (uint8_t) va_arg(arg, uint16_t);
                    morse_norecurse_16((uint16_t)num8, 10,led);
                    break;
                }
                case 'x': 
                    morse_send_char('0',led);
                    morse_send_char('X',led);
                case 'h': {
                    uint16_t num16;
                    num16 = va_arg(arg, uint16_t);
                    morse_norecurse_16(num16, 16,led);
                    break;
                }
                case 'o': {
                    uint16_t num16;
                    num16 = va_arg(arg, uint16_t);
                    morse_norecurse_16(num16, 8,led);
                    break;
                }
                case '%':
                    morse_send_char('%',led);
                    break;
                case 'b': {
                    uint16_t num16;
                    num16 = va_arg(arg, uint16_t);
                    morse_norecurse_16(num16, 2,led);
                    break;
                }
                default:
                    morse_send_char('?',led);
                    morse_send_char('?',led);
                    morse_send_char('?',led);
                    break;
            }
        }
    }
    va_end(arg);
    return 0;
}
#endif
