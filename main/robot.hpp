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
        _revAcc = 1 / a;
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
    void writeServo(int pos);
    void initServo();

public:
    Engine(){}

    Engine(DynamicWithTimer initial) : DynamicWithTimer(initial.pos, initial.velocity, initial.acc)
    {
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

#include <Eigen/Dense>
using Eigen::seq;
template <int Size>
using Vector = Eigen::Vector<float, Size>;
template <int rowSize, int colSize>
using Matrix = Eigen::Matrix<float, rowSize, colSize>;
#include <vector>
#include <iostream>

#define ZERO_LIMIT 1e-4
#define MAT_SIZE 6

bool isZero(Vector<MAT_SIZE> col){
    return (col.array() == 0).all();
}

class Solver
{
    Sensor _sensor;
    Engine _engine;
    Eigen::PartialPivLU<Matrix<MAT_SIZE, MAT_SIZE>> _partialSolver;

    Matrix<MAT_SIZE, MAT_SIZE> _observations = Matrix<MAT_SIZE, MAT_SIZE>::Zero();
    int _height = 0;

    std::vector<int> _getBasisIndices()
    {
        Eigen::Matrix<float, Eigen::Dynamic, MAT_SIZE, Eigen::RowMajor, MAT_SIZE, MAT_SIZE> m;  
        m = _observations.bottomRows(_height);
        std::vector<int> res;
        auto i = 0;
        for (int col = 0; col < MAT_SIZE; col++)
        {
            if (abs(m(i, col)) < ZERO_LIMIT)
            {
                for (int j = i + 1; j < _height; j++)
                {
                    if (abs(m(j, col)) > ZERO_LIMIT)
                    {
                        auto r = m.row(i).eval();
                        m.row(i) = m.row(j);
                        m.row(j) = r;
                        break;
                    }
                }
            }
            if (abs(m(i, col)) >= ZERO_LIMIT)
            {
                res.push_back(col);
                if (res.size() == _height)
                    return res;
                m.row(i) *= 1 / m(i, col);
                for (int j = i + 1; j < _height; j++)
                    m.row(j) -= m.row(i) * m(j, col);
                i++;
            }
        }
        return res;
    }

    int _replacedIndex(Vector<MAT_SIZE> &obs)
    {
        Eigen::Matrix<float, MAT_SIZE, MAT_SIZE + 1, Eigen::RowMajor> rowReduce;
        // Matrix<MAT_SIZE, MAT_SIZE + 1> rowReduce; 
        rowReduce << _observations.transpose(), obs;
        auto subMatSize = std::min(MAT_SIZE, _height + 1);
        for (int i = 0; i < subMatSize; i++)
        {
            auto col = MAT_SIZE - i;
            if (abs(rowReduce(i, col)) < ZERO_LIMIT)
            {
                for (int j = i + 1; j < MAT_SIZE; j++)
                {
                    if (abs(rowReduce(j, col)) > ZERO_LIMIT)
                    {
                        auto r = rowReduce.row(i).eval();
                        rowReduce.row(i) = rowReduce.row(j);
                        rowReduce.row(j) = r;
                        break;
                    }
                }
                if (abs(rowReduce(i, col)) < ZERO_LIMIT)
                    return col;
            }
            rowReduce.row(i) *= 1 / rowReduce(i, col);
            for (int j = i + 1; j < MAT_SIZE; j++)
                rowReduce.row(j) -= rowReduce.row(i) * rowReduce(j, col);
        }
        return MAT_SIZE - subMatSize;
    }

    Vector<MAT_SIZE> _nextObservations(const Vector<MAT_SIZE> &obs, const int index)
    {
        Vector<MAT_SIZE> next_values;
        int l = _height - 1;
        next_values.segment(MAT_SIZE - _height, l) = _observations.col(index).tail(l);
        next_values(MAT_SIZE - 1) = obs(index);
        return next_values;
    }

public:
    Solver()
    {
        _sensor = Sensor();
        _engine = Engine(DynamicWithTimer(0, 0, 1));
    }

    void initEngineTimer()
    {
        _engine.initTimer();
    }

    void setEngineAcc(float a)
    {
        _engine.setAcc(a);
    }

    void moveEngine()
    {
        _engine.moveEngine();
    }

    void changeEngineAcc()
    {
        if (_sensor.update() == 1000) 
            return;
        Vector<MAT_SIZE> obs = {_engine.getAcc(), _sensor.getAcc(), _sensor.getVelocity(), 
                                _sensor.getPos(), _engine.getVelocity(), _engine.getPos()};
        if (isZero(obs))
            return;
        float newAcc = NAN;
        int ri;
        if (_height == MAT_SIZE)
        {
            _partialSolver.compute(_observations);
            Matrix<MAT_SIZE, 3> ratios;
            for (int i = 0; i < 3; i++)
            {
                auto sensData = _nextObservations(obs, i + 1);
                ratios.col(i) = _partialSolver.solve(sensData);
            }
            int l = MAT_SIZE - 1;
            Vector<3> b; b.noalias() = ratios.transpose().rightCols(l) * obs.tail(l);
            Vector<3> a = ratios.row(0);
            newAcc = - b.dot(a) / a.dot(a);
            std::cout << obs(3) << "\n";
            // ri = _replacedIndex(obs);
        }
        else
        {
            if (_height > 1)
            {
                auto ind = _getBasisIndices(); // basis indices
                auto obsBasis = _observations.bottomRows(_height)(Eigen::all, ind);
                // auto m = Matrix<2, 2>({{1, 2, 3}, {3, 4}});
                // auto obsBasis = m(Eigen::all, std::vector<int>({0, 1}));
                auto dynSolver = obsBasis.partialPivLu();
                // if (_fullSolver.rank() < 2)
                //     return;
                Eigen::Matrix<float, Eigen::Dynamic, 3, 0, MAT_SIZE> ratios(_height, 3);
                for (int i = 0; i < 3; i++)
                {
                    auto sensData = _nextObservations(obs, i + 1).bottomRows(_height);
                    // Vector<3> s {1, 1, 1};
                    // auto sensData = s.bottomRows(_height);
                    ratios.col(i) = dynSolver.solve(sensData);
                }
                int l = _height - 1;
                Vector<3> b; b.noalias() = ratios.transpose().rightCols(l) * obs(ind).tail(l);
                Vector<3> a = ratios.row(0);
                newAcc = - b.dot(a) / a.dot(a);
                // ri = _replacedIndex(obs);
            }
            // else
            //     ri = _replacedIndex(obs);
        }
        ri = _replacedIndex(obs);
        for (int i = ri; i < MAT_SIZE - 1; i++)
        {
            _observations.row(i) = _observations.row(i + 1);
        }
        _observations.row(MAT_SIZE - 1) = obs.transpose();
        if (_height < MAT_SIZE && ri < MAT_SIZE - _height)
            _height++;
        if (!isnan(newAcc))
        {
            _engine.setAcc(newAcc); // printf("height=%i acc=%f\n", _height, newAcc);
        }
    }

    void test()
    {
        _height = 2;
        _observations(5, 5) = 2;
        auto res = _getBasisIndices();
        assert(res.size() == 1 && res[0] == 5);

        _observations(4, 5) = 2;
        res = _getBasisIndices();
        assert(res.size() == 1 && res[0] == 5);

        _observations(5, 4) = 2;
        res = _getBasisIndices();
        assert(res.size() == 2 && res == std::vector<int>({4, 5}));

        _observations(5, 3) = 2;
        res = _getBasisIndices();
        assert(res.size() == 2 && res == std::vector<int>({3, 5}));

        _observations(4, 4) = 2;
        res = _getBasisIndices();
        assert(res.size() == 2 && res == std::vector<int>({3, 4}));
        ESP_LOGI("tests", "done!!!");
    }
};
