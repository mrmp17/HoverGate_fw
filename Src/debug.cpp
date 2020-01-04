//
// Created by Nejc on 2020-01-02.
//

#include "debug.h"

/*
 * use like printf to print debug messages. Printing floats does not work. Send MAX 127 characters!!!L
 */
void debug_print(const char *format, ...){
    char formatedString[128] = {0};
    va_list arglist;
    va_start(arglist, format);
    int len = vsnprintf(formatedString, sizeof(formatedString), format, arglist);
    uint16_t n = 0;
    //comment out this block if carriage return not needed (change serial_01.write len!!!)
    while(formatedString[n] != 0) n++;
    formatedString[n] = 0xD;
    //####
    va_end(arglist);
    while(serial_01.txOngoing);
    serial_01.write(reinterpret_cast<uint8_t*>(formatedString), len+1);
}