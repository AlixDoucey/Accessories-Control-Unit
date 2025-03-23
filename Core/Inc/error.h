#ifndef ERROR_H
#define ERROR_H

#include <stdint.h>

extern char *error_throw();
extern uint8_t error_catch(char *dest);

#endif // ERROR_H
