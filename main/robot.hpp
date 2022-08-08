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

protected:
    int64_t _prevTime = 0;
    int64_t getTime();

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
    float update()
    {
        auto newPos = angle();
        if (newPos != 1000)
        {
            auto t = getTime();
            if (_prevTime != 0)
            {
                auto dt = 1000000.f / (t - _prevTime);
                auto newVelocity = (newPos - pos) * dt;
                acc = (newVelocity - velocity) * dt;
                velocity = newVelocity;
                pos = newPos;
            }
            else 
            {
                pos = newPos;
                newPos = 1000;
            }
            _prevTime = t;
        }
        return newPos;
    };
};

#include <float.h>

class DynamicWithTimer : public Dynamics
{
    float _revAcc;
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
    Engine(){}

    Engine(DynamicWithTimer initial, Sensor &sensor) : DynamicWithTimer(initial.pos, initial.velocity, initial.acc)
    {
        _sensor = sensor;
        initServo();
    };

    void moveEngine()
    {
        // printf("move Engine started... ");
        _updatePosAndVelocity();
        // ESP_LOGI("moveEngine", "velocity %f acc %f pos %f sv %f sp %f", velocity, acc, pos, _sensor.getVelocity(), _sensor.getPos());
        pos = roundf(pos);
        auto timerPeriod = _getTimerPeriod();
        setTimerPeriod(timerPeriod);

        if (abs(pos) > 90){
            pos = constrain(pos, -90, 90);
            velocity = 0;
            acc = 0;
        }
        if (abs(velocity) > 400)
        {
            velocity = constrain(velocity, -400, 400);
            acc = 0;
        }
        // ESP_LOGI("moveEngine", "velocity %f acc %f pos %f", velocity, acc, pos);
        writeServo(pos + 90);
    }
};

#include <BasicLinearAlgebra.h>
using namespace BLA;
#define ZERO_LIMIT 1e-4
#define MAT_SIZE 6

bool isZero(Matrix<MAT_SIZE> col){
  for (int i = 0; i < MAT_SIZE; i++){
    if (col(i) != 0) return false;
  }
  return true;
}

class Solver
{
    Sensor _sensor;
    Engine _engine;
public:
    Solver(Sensor &s, Engine &e) 
    {
        _sensor = s;
        _engine = e;
        observations.Fill(0);
    }

    void changeEngineAcc()
    {
        if (_sensor.update() == 1000) 
            return;
        Matrix<MAT_SIZE> obs = {_sensor.getPos(), _sensor.getVelocity(), _sensor.getAcc(),
                                _engine.getPos(), _engine.getVelocity(), _engine.getAcc()};
        if (isZero(obs))
            return;
        float newAcc = NAN;
        int ri;
        // observations = Identity<6, 6>();
        if (height == MAT_SIZE)
        {
            auto A = observations;
            auto decomp = LUDecompose(A);
            // Serial << "observations " << observations << "\n";
            Serial << "obs " << obs << "\n";
            if (decomp.singular)
            {
                Serial << "singular " << observations << "\n";
            }
            for (int i = 0; i < 3; i++)
            {
                auto sensData = nextObservations(obs, i);
                ratios.Submatrix<MAT_SIZE, 1>(0, i) = LUSolve(decomp, sensData);
            }
            Matrix<3, 1> b = (~ratios).Submatrix<3, MAT_SIZE - 1>(0, 0) * obs.Submatrix<MAT_SIZE - 1, 1>(0, 0);
            auto a = (~ratios).Submatrix<3, 1>(MAT_SIZE - 1, 0);
            newAcc = (~b * a)(0, 0) / (~a * a)(0, 0);
            Serial << newAcc << "\n";
            Serial << "observations before: " << observations << endl;
            ri = replacedIndexWithDet(obs);
            // Serial << "observations after: " << observations << endl;
        }
        else
        {
            ri = replacedIndexGauss(obs);
            Serial << "height=" << height << "\n";
            Serial << "observations: " << observations << endl;
        }
        for (int i = ri; i < MAT_SIZE - 1; i++)
        {
            observations.Submatrix<1, MAT_SIZE>(i, 0) =
                Matrix<1, MAT_SIZE>(observations.Submatrix<1, MAT_SIZE>(i + 1, 0));
        }
        observations.Submatrix<1, MAT_SIZE>(MAT_SIZE - 1, 0) = ~obs;
        if (height < MAT_SIZE && ri < MAT_SIZE - height)
            height++;
        // Serial << observations << "\n"; delay(1000);
        if (!isnan(newAcc))
        {
            _engine.setAcc(newAcc);
        }
        else{
            if (_engine.getAcc() == 0 && _engine.getVelocity() == 0 && std::abs(_sensor.getVelocity()) < 0.01 && std::abs(_sensor.getPos()) > 0.01)
            {
                // printf("acc changed\n");
                _engine.acc = sgn(_sensor.getPos());
            }
        }
    }

private:
    int replacedIndexWithDet(Matrix<MAT_SIZE> &obs)
    {
        for (int i = 0; i < MAT_SIZE; i++)
        {
            Matrix<MAT_SIZE, MAT_SIZE> rowReplaced = observations;
            rowReplaced.Submatrix<1, MAT_SIZE>(i, 0) = ~obs;
            // float det = Determinant(rowReplaced);
            // if (abs(det) > ZERO_LIMIT)
            //   return i;
            // auto m1 = rowReplaced;
            auto m = LUDecompose(rowReplaced);
            if (!m.singular)
            {
                // Serial << "non-singular " << m1 << "\n";
                return i;
            }
        }
        return 0;
    }

    int replacedIndexGauss(Matrix<MAT_SIZE> &obs)
    {
        Matrix<MAT_SIZE, MAT_SIZE + 1> rowReduce = ~observations || obs;
        // Serial << rowReduce << "\n"; delay(1000);
        for (int i = 0; i < MAT_SIZE; i++)
        {
            if (abs(rowReduce(i, MAT_SIZE - i)) < ZERO_LIMIT)
            {
                bool allZeros = true;
                for (int j = i + 1; j < MAT_SIZE; j++)
                {
                    if (abs(rowReduce(j, MAT_SIZE - i)) > ZERO_LIMIT)
                    {
                        auto r = Matrix<1, MAT_SIZE + 1>(rowReduce.Submatrix<1, MAT_SIZE + 1>(i, 0));
                        rowReduce.Submatrix<1, MAT_SIZE + 1>(i, 0) = Matrix<1, MAT_SIZE + 1>(rowReduce.Submatrix<1, MAT_SIZE + 1>(j, 0));
                        rowReduce.Submatrix<1, MAT_SIZE + 1>(j, 0) = r;
                        allZeros = false;
                        break;
                    }
                }
                if (allZeros)
                    return MAT_SIZE - i;
            }
            rowReduce.Submatrix<1, MAT_SIZE + 1>(i, 0) *= 1 / rowReduce(i, MAT_SIZE - i);
            for (int j = i + 1; j < MAT_SIZE; j++)
                rowReduce.Submatrix<1, MAT_SIZE + 1>(j, 0) -= rowReduce.Submatrix<1, MAT_SIZE + 1>(i, 0) * rowReduce(j, MAT_SIZE - i);
        }
        return 0;
    }

    Matrix<MAT_SIZE, MAT_SIZE> observations;
    Matrix<MAT_SIZE, 3> ratios;
    int height = 0;

    Matrix<MAT_SIZE> nextObservations(const Matrix<MAT_SIZE> &obs, const int index)
    {
        auto next_values = Matrix<MAT_SIZE>(observations.Column(index));
        for (int i = 0; i < MAT_SIZE - 1; i++)
        {
            next_values(i) = next_values(i + 1);
        }
        next_values(MAT_SIZE - 1) = obs(index);
        return next_values;
    }
};
