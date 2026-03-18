#pragma once
#include <Arduino.h>

class StepperDevice {
private:
    uint8_t pin1;
    uint8_t pin2;
    uint8_t pin3;
    uint8_t pin4;

    int stepIndex = 0;

    long currentSteps = 0;          // 逻辑位置（相对零点）
    long targetSteps = 0;           // 逻辑目标位置
    float stepsPerRevolution = 4096.0f;
    unsigned long stepDelayMs = 2;
    unsigned long lastStepTimeMs = 0;

    bool busy = false;
    bool holdAfterStop = false;
    bool directionInverted = false;

    bool doneEvent = false;
    bool stoppedEvent = false;

    // 28BYJ-48 + ULN2003 半步 8 拍序列
    const uint8_t sequence[8][4] = {
        {1, 0, 0, 0},
        {1, 1, 0, 0},
        {0, 1, 0, 0},
        {0, 1, 1, 0},
        {0, 0, 1, 0},
        {0, 0, 1, 1},
        {0, 0, 0, 1},
        {1, 0, 0, 1}
    };

    void applyStep(int index) {
        digitalWrite(pin1, sequence[index][0]);
        digitalWrite(pin2, sequence[index][1]);
        digitalWrite(pin3, sequence[index][2]);
        digitalWrite(pin4, sequence[index][3]);
    }

    void logicalStepOnce(int logicalDir) {
        // logicalDir 是命令空间里的方向
        // electricalDir 是实际输出相序方向
        int electricalDir = directionInverted ? -logicalDir : logicalDir;

        stepIndex += electricalDir;
        if (stepIndex > 7) stepIndex = 0;
        if (stepIndex < 0) stepIndex = 7;

        applyStep(stepIndex);

        // 位置反馈仍然按“逻辑方向”累计
        currentSteps += logicalDir;
    }

public:
    StepperDevice(uint8_t p1, uint8_t p2, uint8_t p3, uint8_t p4)
        : pin1(p1), pin2(p2), pin3(p3), pin4(p4) {}

    void begin() {
        pinMode(pin1, OUTPUT);
        pinMode(pin2, OUTPUT);
        pinMode(pin3, OUTPUT);
        pinMode(pin4, OUTPUT);
        release();
    }

    void release() {
        digitalWrite(pin1, LOW);
        digitalWrite(pin2, LOW);
        digitalWrite(pin3, LOW);
        digitalWrite(pin4, LOW);
    }

    void setZero() {
        if (!busy) {
            currentSteps = 0;
            targetSteps = 0;
        }
    }

    void setStepsPerRevolution(float spr) {
        if (spr > 0.0f) {
            stepsPerRevolution = spr;
        }
    }

    float getStepsPerRevolution() const {
        return stepsPerRevolution;
    }

    void setStepDelayMs(unsigned long delayMs) {
        if (delayMs >= 1) {
            stepDelayMs = delayMs;
        }
    }

    unsigned long getStepDelayMs() const {
        return stepDelayMs;
    }

    void setHoldAfterStop(bool hold) {
        holdAfterStop = hold;
        if (!holdAfterStop && !busy) {
            release();
        }
    }

    bool getHoldAfterStop() const {
        return holdAfterStop;
    }

    void setDirectionInverted(bool inverted) {
        directionInverted = inverted;
    }

    bool getDirectionInverted() const {
        return directionInverted;
    }

    long getCurrentSteps() const {
        return currentSteps;
    }

    long getTargetSteps() const {
        return targetSteps;
    }

    long getRemainingSteps() const {
        return targetSteps - currentSteps;
    }

    float getCurrentAngle() const {
        return (currentSteps * 360.0f) / stepsPerRevolution;
    }

    float getTargetAngle() const {
        return (targetSteps * 360.0f) / stepsPerRevolution;
    }

    bool isBusy() const {
        return busy;
    }

    // 停止型打断
    void stop() {
        if (busy) {
            busy = false;
            targetSteps = currentSteps;
            stoppedEvent = true;
        }

        if (!holdAfterStop) {
            release();
        }
    }

    // 覆盖型打断：新的运动命令直接改写目标
    void commandMoveSteps(long relativeSteps) {
        if (relativeSteps == 0) {
            return;
        }

        targetSteps = currentSteps + relativeSteps;
        busy = true;
        doneEvent = false;
        stoppedEvent = false;
    }

    void commandMoveAngle(float degrees) {
        long steps = lround((degrees / 360.0f) * stepsPerRevolution);
        commandMoveSteps(steps);
    }

    void commandMoveRevolutions(float revolutions) {
        long steps = lround(revolutions * stepsPerRevolution);
        commandMoveSteps(steps);
    }

    void update() {
        if (!busy) {
            return;
        }

        if (currentSteps == targetSteps) {
            busy = false;
            doneEvent = true;

            if (!holdAfterStop) {
                release();
            }
            return;
        }

        unsigned long now = millis();
        if (now - lastStepTimeMs < stepDelayMs) {
            return;
        }
        lastStepTimeMs = now;

        int logicalDir = (targetSteps > currentSteps) ? 1 : -1;
        logicalStepOnce(logicalDir);

        if (currentSteps == targetSteps) {
            busy = false;
            doneEvent = true;

            if (!holdAfterStop) {
                release();
            }
        }
    }

    bool consumeDoneEvent() {
        if (doneEvent) {
            doneEvent = false;
            return true;
        }
        return false;
    }

    bool consumeStoppedEvent() {
        if (stoppedEvent) {
            stoppedEvent = false;
            return true;
        }
        return false;
    }
};