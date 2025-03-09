#include "serde.h"
#include "string.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

ParsedMessage parseMessage(const char *message) {
    ParsedMessage parsed;
    parsed.id = 0;
    parsed.command = CMD_UNKNOWN;
    parsed.argument = 0.0f;
    parsed.hasArgument = 0;

    // We will tokenize the message, so make a copy to avoid altering the original:
    char buffer[128];
    strncpy(buffer, message, sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = '\0';

    // Tokenize on '|'
    char *token = strtok(buffer, "|");
    if (!token) {
        return parsed; // No tokens, not a valid message
    }
    // 1) Parse the ID:
    parsed.id = atoi(token);

    // 2) Parse the command:
    token = strtok(NULL, "|");
    if (!token) {
        return parsed; // No command found
    }
    if (strcmp(token, "reset") == 0) {
        parsed.command = CMD_RESET;
    } else if (strcmp(token, "move") == 0) {
        parsed.command = CMD_MOVE;
    } else if (strcmp(token, "sense") == 0) {
        parsed.command = CMD_SENSE;
    } else {
        parsed.command = CMD_UNKNOWN;
    }

    // 3) Optionally parse the argument (if present):
    token = strtok(NULL, "|");
    if (token) {
        parsed.argument = (float)atof(token);
        parsed.hasArgument = 1;
    }

    return parsed;
}
