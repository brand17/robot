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

// class MyServo
// {
// public:
//     MyServo();
//     void write(int pos);
// };
#include <float.h>

class DynamicWithTimer : public Dynamics
{
    float _revAcc;
    int64_t _prevTime;

    int64_t getTime();
    void setTimerPeriod(float timerPeriod);

    float _getTimerPeriod(float dx){
        auto D = velocity * velocity + 2 * acc * dx;
        if (D >= 0){
            D = sqrtf(D);
            auto tp = (-velocity + D) * _revAcc;
            if (tp > 0) 
                return tp;
            return (-velocity - D) * _revAcc;
        }
        return FLT_MAX;
    }

protected:
    int _nextPos; 
    float _nextVelocity;

public:
    DynamicWithTimer(float p = 0, float v = 0, float a = 0) : Dynamics(p, v, a)
    {
        _nextPos = pos + sgn(velocity);
        _revAcc = 1;
        _prevTime = getTime();
    };

    void initTimer();

    void setAcc(float a)
    {
        auto time_since_boot = getTime();
        auto dt = (time_since_boot - _prevTime) / 1000000.f;
        auto dv = acc * dt;
        pos += dv * dt * 0.5;
        velocity += dv;
        acc = a;
        _revAcc = 1 / acc;
        float timerPeriod = FLT_MAX;
        float dx = 0;
        if (acc > 0 or velocity > 0)
        { // the second option
            dx = floorf(pos) + 1 - pos;
            timerPeriod = _getTimerPeriod(dx);
        }
        if (acc < 0 or velocity < 0)
        { // the first option
            auto dx2 = ceilf(pos) - 1 + pos;
            auto tp = _getTimerPeriod(dx2);
            if (timerPeriod < tp){
                timerPeriod = tp;
                dx = dx2;
            }
        }
        _nextPos = pos + dx;
        _nextVelocity = velocity + timerPeriod * acc;
        setTimerPeriod(timerPeriod);
    }
};

class Engine : public DynamicWithTimer
{
    Sensor _sensor;
    // MyServo _servo;
    void writeServo(int pos);
    void initServo();

public:
    Engine(DynamicWithTimer initial, Sensor &sensor) : DynamicWithTimer(initial.pos, initial.velocity, initial.acc)
    {
        _sensor = sensor;
        initServo();
        // _servo = servo;
    };

    void moveEngine()
    {
        if (acc == 0 && velocity == 0 && std::abs(_sensor.getVelocity()) < 0.01 && std::abs(_sensor.getPos()) > 0.01)
        {
            // Serial << "Setting acc" << "\n";
            acc = sgn(_sensor.getPos());
        }
        pos = constrain(_nextPos, -90, 90);
        // int sum;
        // if (!__builtin_add_overflow(velocity, acc, &sum))
        //   velocity = sum;
        velocity = constrain(_nextVelocity, -200, 200);
        writeServo(pos + 90);
        // _servo.write(_nextPos + 90);
        // Serial.println(pos);
        // Serial << pos << " " << velocity << " " << acc << " " << counter << "\t\n";
    }
};
