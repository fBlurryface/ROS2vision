#include "stepper_device.h"
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

// UNO D8 D9 D10 D11 -> ULN2003 IN1 IN2 IN3 IN4
StepperDevice stepper(8, 9, 10, 11);

static const unsigned long SERIAL_BAUD = 115200;
static const size_t CMD_BUF_LEN = 64;

char cmdBuffer[CMD_BUF_LEN];
size_t cmdIndex = 0;

void printHelp() {
    Serial.println(F("Commands:"));
    Serial.println(F("  HELP"));
    Serial.println(F("  PING"));
    Serial.println(F("  STEP <n>          relative steps, overrides current motion"));
    Serial.println(F("  ANG <deg>         relative angle, overrides current motion"));
    Serial.println(F("  REV <r>           relative revolutions, overrides current motion"));
    Serial.println(F("  STOP              stop current motion immediately"));
    Serial.println(F("  ZERO              set current logical position to zero (idle only)"));
    Serial.println(F("  POS?              print current position"));
    Serial.println(F("  STATE?            print busy/idle + target + remaining"));
    Serial.println(F("  SPR <value>       set steps per revolution"));
    Serial.println(F("  DELAY <ms>        set step delay"));
    Serial.println(F("  HOLD ON|OFF       hold coils after stop/finish"));
    Serial.println(F("  DIRINV ON|OFF     invert logical direction mapping"));
    Serial.println(F("  RELEASE           de-energize coils"));
}

void printPosition() {
    Serial.print(F("POS STEPS "));
    Serial.print(stepper.getCurrentSteps());
    Serial.print(F(" ANGLE "));
    Serial.println(stepper.getCurrentAngle(), 4);
}

void printState() {
    Serial.print(F("STATE "));
    Serial.print(stepper.isBusy() ? F("BUSY ") : F("IDLE "));
    Serial.print(F("CUR_STEPS "));
    Serial.print(stepper.getCurrentSteps());
    Serial.print(F(" CUR_ANGLE "));
    Serial.print(stepper.getCurrentAngle(), 4);
    Serial.print(F(" TARGET_STEPS "));
    Serial.print(stepper.getTargetSteps());
    Serial.print(F(" TARGET_ANGLE "));
    Serial.print(stepper.getTargetAngle(), 4);
    Serial.print(F(" REMAIN "));
    Serial.print(stepper.getRemainingSteps());
    Serial.print(F(" SPR "));
    Serial.print(stepper.getStepsPerRevolution(), 4);
    Serial.print(F(" DELAY "));
    Serial.print(stepper.getStepDelayMs());
    Serial.print(F(" HOLD "));
    Serial.print(stepper.getHoldAfterStop() ? F("ON") : F("OFF"));
    Serial.print(F(" DIRINV "));
    Serial.println(stepper.getDirectionInverted() ? F("ON") : F("OFF"));
}

void trimInPlace(char* s) {
    char* start = s;
    while (*start && isspace((unsigned char)*start)) {
        start++;
    }

    if (start != s) {
        memmove(s, start, strlen(start) + 1);
    }

    int len = strlen(s);
    while (len > 0 && isspace((unsigned char)s[len - 1])) {
        s[len - 1] = '\0';
        len--;
    }
}

void toUpperInPlace(char* s) {
    while (*s) {
        *s = toupper((unsigned char)*s);
        s++;
    }
}

bool isMotionCommand(const char* upper) {
    return (strncmp(upper, "STEP ", 5) == 0) ||
           (strncmp(upper, "ANG ", 4) == 0) ||
           (strncmp(upper, "REV ", 4) == 0);
}

void processCommand(char* line) {
    trimInPlace(line);
    if (strlen(line) == 0) return;

    char original[CMD_BUF_LEN];
    strncpy(original, line, CMD_BUF_LEN - 1);
    original[CMD_BUF_LEN - 1] = '\0';

    char upper[CMD_BUF_LEN];
    strncpy(upper, line, CMD_BUF_LEN - 1);
    upper[CMD_BUF_LEN - 1] = '\0';
    toUpperInPlace(upper);

    if (strcmp(upper, "HELP") == 0) {
        printHelp();
        return;
    }

    if (strcmp(upper, "PING") == 0) {
        Serial.println(F("OK PONG"));
        return;
    }

    if (strcmp(upper, "POS?") == 0) {
        printPosition();
        return;
    }

    if (strcmp(upper, "STATE?") == 0) {
        printState();
        return;
    }

    if (strcmp(upper, "STOP") == 0) {
        stepper.stop();
        Serial.println(F("OK STOP"));
        return;
    }

    if (strcmp(upper, "RELEASE") == 0) {
        stepper.stop();
        stepper.release();
        Serial.println(F("OK RELEASE"));
        return;
    }

    if (strcmp(upper, "ZERO") == 0) {
        if (stepper.isBusy()) {
            Serial.println(F("ERR BUSY"));
            return;
        }
        stepper.setZero();
        Serial.println(F("OK ZERO"));
        return;
    }

    if (strncmp(upper, "HOLD ", 5) == 0) {
        if (stepper.isBusy()) {
            Serial.println(F("ERR BUSY"));
            return;
        }

        if (strcmp(upper + 5, "ON") == 0) {
            stepper.setHoldAfterStop(true);
            Serial.println(F("OK HOLD ON"));
            return;
        }
        if (strcmp(upper + 5, "OFF") == 0) {
            stepper.setHoldAfterStop(false);
            Serial.println(F("OK HOLD OFF"));
            return;
        }

        Serial.println(F("ERR HOLD"));
        return;
    }

    if (strncmp(upper, "DIRINV ", 7) == 0) {
        if (stepper.isBusy()) {
            Serial.println(F("ERR BUSY"));
            return;
        }

        if (strcmp(upper + 7, "ON") == 0) {
            stepper.setDirectionInverted(true);
            Serial.println(F("OK DIRINV ON"));
            return;
        }
        if (strcmp(upper + 7, "OFF") == 0) {
            stepper.setDirectionInverted(false);
            Serial.println(F("OK DIRINV OFF"));
            return;
        }

        Serial.println(F("ERR DIRINV"));
        return;
    }

    if (strncmp(upper, "SPR ", 4) == 0) {
        if (stepper.isBusy()) {
            Serial.println(F("ERR BUSY"));
            return;
        }

        float spr = atof(original + 4);
        if (spr > 0.0f) {
            stepper.setStepsPerRevolution(spr);
            Serial.println(F("OK SPR"));
        } else {
            Serial.println(F("ERR SPR"));
        }
        return;
    }

    if (strncmp(upper, "DELAY ", 6) == 0) {
        if (stepper.isBusy()) {
            Serial.println(F("ERR BUSY"));
            return;
        }

        long delayMs = atol(original + 6);
        if (delayMs >= 1) {
            stepper.setStepDelayMs((unsigned long)delayMs);
            Serial.println(F("OK DELAY"));
        } else {
            Serial.println(F("ERR DELAY"));
        }
        return;
    }

    if (strncmp(upper, "STEP ", 5) == 0) {
        bool override = stepper.isBusy();
        long steps = atol(original + 5);
        stepper.commandMoveSteps(steps);
        Serial.println(override ? F("OK STEP OVERRIDE") : F("OK STEP"));
        return;
    }

    if (strncmp(upper, "ANG ", 4) == 0) {
        bool override = stepper.isBusy();
        float deg = atof(original + 4);
        stepper.commandMoveAngle(deg);
        Serial.println(override ? F("OK ANG OVERRIDE") : F("OK ANG"));
        return;
    }

    if (strncmp(upper, "REV ", 4) == 0) {
        bool override = stepper.isBusy();
        float rev = atof(original + 4);
        stepper.commandMoveRevolutions(rev);
        Serial.println(override ? F("OK REV OVERRIDE") : F("OK REV"));
        return;
    }

    Serial.println(F("ERR UNKNOWN_COMMAND"));
}

void readSerialCommands() {
    while (Serial.available() > 0) {
        char c = (char)Serial.read();

        if (c == '\r' || c == '\n') {
            if (cmdIndex > 0) {
                cmdBuffer[cmdIndex] = '\0';
                processCommand(cmdBuffer);
                cmdIndex = 0;
            }
            continue;
        }

        if (cmdIndex < CMD_BUF_LEN - 1) {
            cmdBuffer[cmdIndex++] = c;
        } else {
            cmdIndex = 0;
            Serial.println(F("ERR CMD_TOO_LONG"));
        }
    }
}

void publishStepperEvents() {
    if (stepper.consumeStoppedEvent()) {
        Serial.print(F("STOPPED STEPS "));
        Serial.print(stepper.getCurrentSteps());
        Serial.print(F(" ANGLE "));
        Serial.println(stepper.getCurrentAngle(), 4);
    }

    if (stepper.consumeDoneEvent()) {
        Serial.print(F("DONE STEPS "));
        Serial.print(stepper.getCurrentSteps());
        Serial.print(F(" ANGLE "));
        Serial.println(stepper.getCurrentAngle(), 4);
    }
}

void setup() {
    Serial.begin(SERIAL_BAUD);
    stepper.begin();
    delay(500);

    Serial.println(F("UNO Controller Ready"));
    Serial.println(F("Type HELP"));
}

void loop() {
    readSerialCommands();
    stepper.update();
    publishStepperEvents();
}