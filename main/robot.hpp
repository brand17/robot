#include <iostream>
#include <math.h>
#include <array>
#ifndef ARDUINO
#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#endif
#define SENSOR_OUTPUT_DIM 1

template <typename T>
int sgn(T val)
{
    return (T(0) < val) - (val < T(0));
}

extern int64_t getTime();

class Dynamics
{
public:
    float pos = 0, velocity = 0, acc = 0;
    Dynamics(float p = 0, float v = 0, float a = 0) : pos(p), velocity(v), acc(a){};
};

#include <Eigen/Dense>

class KalmanFilter
{

public:
    KalmanFilter()
    {
        R = 1.3;
        x_hat.setZero();
        P = Eigen::DiagonalMatrix<float, 3>(0.00113, 0.00113, 0.00113);
        I.setIdentity();
    };

    float get_pos(const float y, float dt)
    {
        F << 1, dt, dt * 0.5f,
            0, 1, dt,
            0, 0, 1;
        update(y);
        return x_hat(0);
    }

private:
    void update(const float y)
    {
        x_hat_new = F * x_hat;
        P = F * P * F.transpose();
        K = P.col(0) / (P(0, 0) + R);
        // auto P_ = P; auto x_ = x_hat_new;
        x_hat_new += K * (y - x_hat_new(0));
        auto m_ = I;
        m_.col(0) -= K;
        P = m_ * P;
        // Eigen::MatrixXf m(3, 16); m << F, x_, P_, K, x_hat_new, P, Eigen::Vector3f(y, 0, 0), m_;
        // std::string sep = "\n----------------------------------------\n";
        // Eigen::IOFormat HeavyFmt(Eigen::FullPrecision, Eigen::DontAlignCols, " ", "\n", "", "", "", "");
        // std::cout << m.format(HeavyFmt) << sep;
        x_hat = x_hat_new;
        // ESP_LOGI("", "K: %2.5f P: %2.5f", K(0, 0), P(0, 0));
    }
    Eigen::Matrix3f F, P;
    float R;
    Eigen::Matrix3f I;
    Eigen::Vector3f K, x_hat, x_hat_new;
};

class Sensor
{
    long long __prevTime = 0;
    int64_t _prevTime = 0;
    KalmanFilter kf[SENSOR_OUTPUT_DIM];
public:
    Dynamics observations[SENSOR_OUTPUT_DIM];
    std::array<float, SENSOR_OUTPUT_DIM> angles();
    float update()
    {
        auto angs = angles();
        auto t = getTime();
        float pos = 1000000;
        if (_prevTime != 0)
        {
            auto dt = t - _prevTime;
            auto rev_dt = 1000000.f / (dt);
            for (int i = 0; i < SENSOR_OUTPUT_DIM; i++)
            {
                pos = angs[i]; auto &obs = observations[i];
                auto t2 = t / 1000000;
                if (t2 != __prevTime)
                {
                    // printf("%2.0f\n", pos);
                    __prevTime = t2;
                }
                printf("pos: %2.0f \n", pos);
                pos = kf[i].get_pos(pos, dt);
                // printf("pos: %2.0f\n", pos);
                auto newVelocity = (pos - obs.pos) * rev_dt;
                obs.acc = (newVelocity - obs.velocity) * rev_dt;
                obs.velocity = newVelocity;
                obs.pos = pos;
                // printf("pos: %2.0f, velocity: %4.0f, acc: %6.0f\n", obs.pos, obs.velocity, obs.acc);
            }
        }
        else 
        {
            for (int i = 0; i < SENSOR_OUTPUT_DIM; i++)
            {
                observations[i].pos = angs[i];
                // newPos = 1000;
            }
        }
        _prevTime = t;
        return pos;
    };
};

#include <float.h>

class DynamicWithTimer : public Dynamics
{
    int64_t _prevTime = 0;
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
        // auto timerPeriod = _getTimerPeriod();
        // stopTimer();
        // setTimerPeriod(timerPeriod);
    }
};

class Engine : public DynamicWithTimer
{
    void writeServo(int pos);
    void initEngine();

public:
    Engine(){}

    Engine(DynamicWithTimer initial) : DynamicWithTimer(initial.pos, initial.velocity, initial.acc)
    {
        initEngine();
    };

    void moveEngine()
    {
        // printf("move Engine started... ");
        _updatePosAndVelocity();
        // ESP_LOGI("moveEngine", "velocity %f acc %f pos %f sv %f sp %f", velocity, acc, pos, _sensor.getVelocity(), _sensor.getPos());
        pos = roundf(pos);
        auto timerPeriod = _getTimerPeriod();
        setTimerPeriod(timerPeriod);

        if (abs(pos) > 90){ // to do: move to _updatePosAndVelocity to improve acc calculation
            pos = constrain(pos, -90, 90);
            velocity = 0;
            acc = 0;
        }
        if (abs(velocity) > 400)
        {
            velocity = constrain(velocity, -400, 400);
            acc = 0;
        }
        // std::cout << velocity << " " << acc << " " << pos << "\n";
        writeServo(pos + 90);
    }
};

using Eigen::seq;
template <int Size>
using Vector = Eigen::Vector<float, Size>;
template <int rowSize, int colSize>
using Matrix = Eigen::Matrix<float, rowSize, colSize>;
#include <vector>

#define ZERO_LIMIT 1e-3
#define MAT_SIZE 3 + SENSOR_OUTPUT_DIM * 3

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
        // printf("Solver constructor called\n");
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

    // int counter = 0;

    void changeEngineAcc()
    {
        if (_sensor.update() == 1000000) 
            return;
        // Vector<MAT_SIZE> obs = {_engine.getAcc(), _sensor.getAcc(), _sensor.getVelocity(), 
        //                         _sensor.getPos(), _engine.getVelocity(), _engine.getPos()};
        // auto t = getTime();
        Vector<MAT_SIZE> obs; 
        obs[0] = _engine.acc;
        int i = 1;
        for (auto &&o: _sensor.observations)
        {
            obs.segment<3>(i) << o.pos, o.velocity, o.acc; 
            i += 3;
        }
        obs.tail<2>() << _engine.velocity, _engine.pos;
        if (isZero(obs))
            return;
        float newAcc = NAN;
        int ri;
        if (_height == MAT_SIZE)
        {
            _partialSolver.compute(_observations);
            Matrix<MAT_SIZE, 3 * SENSOR_OUTPUT_DIM> ratios;
            for (int i = 0; i < 3 * SENSOR_OUTPUT_DIM; i++)
            {
                auto sensData = _nextObservations(obs, i + 1);
                ratios.col(i) = _partialSolver.solve(sensData);
            }
            int l = MAT_SIZE - 1;
            Vector<3 * SENSOR_OUTPUT_DIM> b; b.noalias() = ratios.transpose().rightCols(l) * obs.tail(l);
            Vector<3 * SENSOR_OUTPUT_DIM> a = ratios.row(0);
            newAcc = - b.dot(a) / a.dot(a);
            Eigen::IOFormat HeavyFmt(Eigen::FullPrecision, Eigen::DontAlignCols, " ", "\n", "", "", "\n", "");
            // std::cout << ratios.transpose().format(HeavyFmt);
            // std::cout << obs.transpose() << " " << newAcc << "\n";
            // counter ++;
            // // std::cout << counter << "\n";
            // if (counter == 100){
            //     std::cout << _observations << "\n"
            //               << obs.transpose() << "\n"
            //               << newAcc << "\n";
            // }
        }
        else
        {
            if (_height > 1)
            {
                auto ind = _getBasisIndices(); // basis indices
                auto obsBasis = _observations.bottomRows(_height)(Eigen::all, ind);
                // auto m = Matrix<2, 2>({{1, 2, 3}, {3, 4}});
                // auto obsBasis = m(Eigen::all, std::vector<int>({0, 1}));
                // std::cout << obsBasis.rows() << obsBasis.cols() << _height << "\n";
                auto dynSolver = obsBasis.partialPivLu();
                // if (_fullSolver.rank() < 2)
                //     return;
                Eigen::Matrix<float, Eigen::Dynamic, 3 * SENSOR_OUTPUT_DIM, 0, MAT_SIZE> ratios(_height, 3 * SENSOR_OUTPUT_DIM);
                for (int i = 0; i < 3 * SENSOR_OUTPUT_DIM; i++)
                {
                    auto sensData = _nextObservations(obs, i + 1).bottomRows(_height);
                    // Vector<3> s {1, 1, 1};
                    // auto sensData = s.bottomRows(_height);
                    ratios.col(i) = dynSolver.solve(sensData);
                }
                int l = _height - 1;
                Vector<3 * SENSOR_OUTPUT_DIM> b; b.noalias() = ratios.transpose().rightCols(l) * obs(ind).tail(l);
                Vector<3 * SENSOR_OUTPUT_DIM> a = ratios.row(0);
                newAcc = - b.dot(a) / a.dot(a);
                // ri = _replacedIndex(obs);
            }
            // else
            //     ri = _replacedIndex(obs);
        }
        ri = _replacedIndex(obs);
        // if (_height < MAT_SIZE && ri < MAT_SIZE - _height && _height == 7)
        //     ri = _replacedIndex(obs);
        if (_height < MAT_SIZE && ri < MAT_SIZE - _height)
            _height++;
        // std::cout << ri << " ";
        // if (_height == 8)
        //     std::cout << _observations << "\n";
        for (int i = ri; i < MAT_SIZE - 1; i++)
        {
            _observations.row(i) = _observations.row(i + 1);
        }
        _observations.row(MAT_SIZE - 1) = obs.transpose();
        // if (_height == 8)
        //     std::cout << _observations << "\n";
        if (!isnan(newAcc))
        {
            _engine.setAcc(newAcc); // printf("height=%i acc=%f\n", _height, newAcc);
        }
        // std::cout << getTime() - t << " ";
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
