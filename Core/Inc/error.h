#ifndef ERROR_H
#define ERROR_H

#include <stdint.h>

#define MSG_MAX_LEN 60
#define QUEUE_LEN 20

extern char *error_throw();
extern uint8_t error_catch(char *dest);

#endif // ERROR_H
