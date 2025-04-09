#include <stdint.h>
#include <string.h>
#include <sys/types.h>
#include "error.h"

char _error_queue[QUEUE_LEN][MSG_MAX_LEN];
int8_t head = -1;
int8_t tail = -1;
int8_t slots_left = QUEUE_LEN;

// the message should be <  MSG_MAX_LEN
char *error_throw() {
    if (slots_left == 0)
        return _error_queue[head];

    if (head == QUEUE_LEN - 1)
        head = -1;

    slots_left--;
    head++;

    return _error_queue[head];
}

uint8_t error_catch(char *dest) {
    if (slots_left == QUEUE_LEN)
        return 0;
    if (tail == QUEUE_LEN - 1)
        tail = -1;
    tail++;
    slots_left++;
    strcpy(dest, _error_queue[tail]);
    return strlen(_error_queue[tail]) + 1;
}

