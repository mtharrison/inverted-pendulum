#ifndef SERDE_H
#define SERDE_H

#include "string.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

typedef enum {
    CMD_RESET,
    CMD_MOVE,
    CMD_SENSE,
    CMD_UNKNOWN
} CommandType;

typedef struct {
    int id;
    CommandType command;
    float argument;
    int hasArgument;  // 1 if an argument was parsed, 0 otherwise
} ParsedMessage;

ParsedMessage parseMessage(const char *message);

#endif // SERDE_H