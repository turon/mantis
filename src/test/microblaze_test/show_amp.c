
#include "printf.h"

#define TOTAL_CHARS 20
#define AMPLITUDE 512
#define TOTAL_CHARS 20
void show_amp(int index, int i, unsigned int amp)
{
    unsigned int ui = (i < 0) ? -1*i : i;
    unsigned int chars = ui * 20 / amp;

    unsigned int j;

    printf("%d\t%d\t", index, i);

    char output[42];
    char *p = output;
    if(i < 0)
    {
        if(chars > TOTAL_CHARS)
        {
            *p++ = '<';
            for(j=0; j < TOTAL_CHARS - 1; j++)
                *p++ = '-';
            *p++ = '|';
            *p++ = '\0';
        } else {
            for(j = 0; j < TOTAL_CHARS - chars; j++)
                *p++ = ' ';
            for(j = 0; j < chars; j++)

                *p++ = '#';
            *p++='|';
            *p++='\0';
        }
    } else {
        if(chars > TOTAL_CHARS)
        {
            for(j = 0; j < TOTAL_CHARS; j++)
                *p++ = ' ';

            *p++ = '|';
            for(j = 0; j < TOTAL_CHARS - 1; j++)
                *p++ = '-';
            *p++ = '>';
            *p++ = '\0';
        } else {
            for(j = 0; j < TOTAL_CHARS; j++)
                *p++ = ' ';
            *p++ = '|';
            for(j = 0; j < chars; j++)
                *p++ = '#';		      
            *p++ = '\0';
        }
    }
    printf("%s\r\n",output);
}

