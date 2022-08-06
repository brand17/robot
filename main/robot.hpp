#ifndef ARDUINO
#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#endif
#define PACKETSIZE 42 // expected DMP packet size (default is 42 bytes)

template <typename T>
int sgn(T val)
{
    return (T(0) < val) - (val < T(0));
}

class Dynamics
{
public: // protected:
    volatile float pos = 0, velocity = 0, acc = 0;

public:
    float getPos() { return pos; }
    float getVelocity() { return velocity; }
    float getAcc() { return acc; }
    Dynamics(float p = 0, float v = 0, float a = 0) : pos(p), velocity(v), acc(a){};
};

class Sensor : public Dynamics
{
public:
    float angle();
    void update()
    {
        auto newPos = angle();
        if (newPos != 1000)
        {
            auto newVelocity = newPos - pos;
            acc = newVelocity - velocity;
            velocity = newVelocity;
            pos = newPos;
        }
    };
};

#include <float.h>

class DynamicWithTimer : public Dynamics
{
    float _revAcc;
    int64_t _prevTime;

    int64_t getTime();

    float _getTimerPeriod(float dx){
        if (acc == 0 and velocity != 0)
        {
            return dx / velocity;
        }
        auto D = velocity * velocity + 2 * acc * dx;
        if (D >= 0){
            D = sqrtf(D);
            auto tp1 = (-velocity + D) * _revAcc;
            auto tp2 = (-velocity - D) * _revAcc;
            if (tp1 <= 0)
                return tp2;
            if (tp2 <= 0)
                return tp1;
            return std::fmin(tp1, tp2);
        }
        return FLT_MAX;
    }

    void stopTimer();

protected:
    void setTimerPeriod(float timerPeriod);

    void _updatePosAndVelocity()
    {
        auto time_since_boot = getTime();
        auto dt = (time_since_boot - _prevTime) / 1000000.f;
        _prevTime = time_since_boot;
        auto dv = acc * dt;
        pos += dt * (velocity + dv * 0.5);
        velocity += dv;
        // ESP_LOGI("_updatePosAndVelocity", "pos %f velocity %f", pos, velocity);
    }

    float _getTimerPeriod()
    {
        float timerPeriod = FLT_MAX;
        float dx = 0;
        if (acc > 0 or velocity > 0)
        { // the second option
            dx = floorf(pos) + 1 - pos;
            // ESP_LOGI("_getTimerPeriod", "velocity %f acc %f dx %f pos %f", velocity, acc, dx, pos);
            timerPeriod = _getTimerPeriod(dx);
        }
        if (acc < 0 or velocity < 0)
        { // the first option
            auto dx2 = ceilf(pos) - 1 - pos;
            // ESP_LOGI("_getTimerPeriod", "velocity %f acc %f dx2 %f pos %f", velocity, acc, dx2, pos);
            auto tp = _getTimerPeriod(dx2);
            if (timerPeriod > tp){
                timerPeriod = tp;
                dx = dx2;
            }
        }
        // ESP_LOGI("_getTimerPeriod", "velocity %f acc %f dx %f pos %f", velocity, acc, dx, pos);
        return timerPeriod;
    }

public:
    DynamicWithTimer(float p = 0, float v = 0, float a = 0) : Dynamics(p, v, a)
    {
        _revAcc = 1;
    };

    void initTimer();

    void setAcc(float a)
    {
        _updatePosAndVelocity();
        acc = a;
        _revAcc = 1 / acc;
        auto timerPeriod = _getTimerPeriod();
        stopTimer();
        setTimerPeriod(timerPeriod);
    }
};

class Engine : public DynamicWithTimer
{
    Sensor _sensor;
    void writeServo(int pos);
    void initServo();

public:
    Engine(DynamicWithTimer initial, Sensor &sensor) : DynamicWithTimer(initial.pos, initial.velocity, initial.acc)
    {
        _sensor = sensor;
        initServo();
    };

    void moveEngine()
    {
        // printf("move Engine started... ");
        _updatePosAndVelocity();
        pos = roundf(pos);
        auto timerPeriod = _getTimerPeriod();
        setTimerPeriod(timerPeriod);

        // if (acc == 0 && velocity == 0 && std::abs(_sensor.getVelocity()) < 0.01 && std::abs(_sensor.getPos()) > 0.01)
        // {
        //     acc = sgn(_sensor.getPos());
        // }
        if (abs(pos) > 90){
            pos = constrain(pos, -90, 90);
            velocity = 0;
            acc = 0;
        }
        if (abs(velocity) > 200)
        {
            velocity = constrain(velocity, -200, 200);
            acc = 0;
        }
        // ESP_LOGI("moveEngine", "velocity %f acc %f pos %f", velocity, acc, pos);
        writeServo(pos + 90);
    }
};
