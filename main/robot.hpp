#include <iostream>
#include <math.h>
#include <array>
#include <vector>
#ifndef ARDUINO
#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#endif
// #define SENSOR_OUTPUT_DIM 1
#define MIN_ENGINE_PWM 30
#include <gsl/gsl_vector.h>
#include <gsl/gsl_multiroots.h>

int64_t getTime();

void print_state (size_t iter, gsl_multiroot_fsolver * s)
{
  printf ("iter = %3u x = % .3f % .3f "
          "f(x) = % .3e % .3e %i\n",
          iter,
          gsl_vector_get (s->x, 0),
          gsl_vector_get (s->x, 1),
          gsl_vector_get (s->f, 0),
          gsl_vector_get (s->f, 1),
          (int) getTime());
}

struct rparams
  {
    double a;
    double b;
  };

int rosenbrock_f (const gsl_vector * x, void *params,
              gsl_vector * f)
{
  double a = ((struct rparams *) params)->a;
  double b = ((struct rparams *) params)->b;

  const double x0 = gsl_vector_get (x, 0);
  const double x1 = gsl_vector_get (x, 1);

  const double y0 = a * (1 - x0);
  const double y1 = b * (x1 - x0 * x0);

  gsl_vector_set (f, 0, y0);
  gsl_vector_set (f, 1, y1);

  return GSL_SUCCESS;
}

int rosenbrock_df (const gsl_vector * x, void *params,
               gsl_matrix * J)
{
  const double a = ((struct rparams *) params)->a;
  const double b = ((struct rparams *) params)->b;

  const double x0 = gsl_vector_get (x, 0);

  const double df00 = -a;
  const double df01 = 0;
  const double df10 = -2 * b  * x0;
  const double df11 = b;

  gsl_matrix_set (J, 0, 0, df00);
  gsl_matrix_set (J, 0, 1, df01);
  gsl_matrix_set (J, 1, 0, df10);
  gsl_matrix_set (J, 1, 1, df11);

  return GSL_SUCCESS;
}

int rosenbrock_fdf (const gsl_vector * x, void *params,
                gsl_vector * f, gsl_matrix * J)
{
  rosenbrock_f (x, params, f);
  rosenbrock_df (x, params, J);

  return GSL_SUCCESS;
}

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
        // Eigen::IOFormat _HeavyFmt(Eigen::FullPrecision, Eigen::DontAlignCols, " ", "\n", "", "", "", "");
        // std::cout << m.format(_HeavyFmt) << sep;
        x_hat = x_hat_new;
        // ESP_LOGI("", "K: %2.5f P: %2.5f", K(0, 0), P(0, 0));
    }
    Eigen::Matrix3f F, P;
    float R;
    Eigen::Matrix3f I;
    Eigen::Vector3f K, x_hat, x_hat_new;
};

class Multisample{
    std::vector<float> measures;
    uint8_t index = 0;
    float total = 0;
    uint8_t size = 0;
    float multiplier = 0;
public:
    Multisample(uint8_t s){
        size = s;
        measures.reserve(size);
        // std::fill(measures.begin(), measures.end(), 0);
        multiplier = 1. / size;
        std::cout << "multiplier: " << multiplier << "\n";
    }
    float get(float value){
        if (measures.size() < size){
            measures.push_back(value);
            total += value;
            return total / measures.size();
        }
        else{
            total += value - measures[index];
            measures[index] = value;
            index ++;
            if (index == measures.size())
                index = 0;
            return total * multiplier;
        }
    }
};

class Sensor
{
    int64_t _prevTime = 0;
    KalmanFilter _kf;
    uint8_t _sampleSize = 1;
    bool _useKalman = false;
    uint8_t _counter = 0;
    float _accum_angles = 0;
public:
    Sensor(){}
    Sensor(uint8_t sampleSize, bool useKalman) : _sampleSize(sampleSize), _useKalman(useKalman){}
    Dynamics observations;
    virtual float angle();
    float update()
    {
        float pos = 1000000;
        _accum_angles += angle();
        _counter ++;
        if (_counter == _sampleSize){
            auto ang = _accum_angles / _sampleSize;
            // std::cout << "ang=" << ang << "\n";
            _counter = 0; _accum_angles = 0;
            // auto ang = angle();
            auto t = getTime();
            if (_prevTime != 0)
            {
                auto dt = t - _prevTime;
                auto rev_dt = 1000000.f / (dt);
                pos = ang; auto &obs = observations;
                // printf("pos: %2.0f \n", pos);
                if (_useKalman){
                    // printf("using Kalman\n");
                    pos = _kf.get_pos(pos, dt);
                }
                // printf("pos: %2.0f\n", pos);
                auto newVelocity = (pos - obs.pos) * rev_dt;
                obs.acc = (newVelocity - obs.velocity) * rev_dt;
                obs.velocity = newVelocity;
                obs.pos = pos;
                // printf("pos: %2.0f, velocity: %4.0f, acc: %6.0f\n", obs.pos, obs.velocity, obs.acc);
            }
            else 
            {
                observations.pos = ang;
            }
            _prevTime = t;
        }
        return pos;
    };
};

#include <float.h>

class Engine : public Sensor
{
public:
    Engine();
    void setDuty(float duty);
    virtual float angle();
};

using Eigen::seq;
template <int Size>
using Vector = Eigen::Vector<float, Size>;
template <int rowSize, int colSize>
using Matrix = Eigen::Matrix<float, rowSize, colSize>;
#include <vector>
Eigen::IOFormat _HeavyFmt(Eigen::FullPrecision, Eigen::DontAlignCols, " ", "\n", "", "", "\n", "");

#define ZERO_LIMIT 1e-3
#define MAT_SIZE 3 + 3 + 1

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
    float _duty = 0;

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
        _sensor = Sensor(8, false);
        // _engine = Engine();
    }

    void changeEngineAcc()
    {
        auto s = _sensor.update();
        if (s == 1000000) 
            return;
        auto e = _engine.update();
        if (e == 1000000) 
            return;
        Vector<MAT_SIZE> obs; 
        obs.segment<3>(1) << _sensor.observations.pos, _sensor.observations.velocity, _sensor.observations.acc; 
        auto engObs = &_engine.observations;
        obs.tail<3>() << engObs->pos, engObs->velocity, engObs->acc;
        auto eng_cos = sqrtf(1 - engObs->pos * engObs->pos);
        // if (eng_cos == 0 or isnan(eng_cos)) 
            // std::cout  << "eng_cos is " << eng_cos << " " << engObs->pos << "\n";
        obs[0] = (_duty - MIN_ENGINE_PWM) * eng_cos;
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
        }
        else
        {
            if (_height > 1)
            {
                auto ind = _getBasisIndices(); // basis indices
                auto obsBasis = _observations.bottomRows(_height)(Eigen::all, ind);
                auto dynSolver = obsBasis.partialPivLu();
                Eigen::Matrix<float, Eigen::Dynamic, 3, 0, MAT_SIZE> ratios(_height, 3);
                for (int i = 0; i < 3; i++)
                {
                    auto sensData = _nextObservations(obs, i + 1).bottomRows(_height);
                    ratios.col(i) = dynSolver.solve(sensData);
                }
                int l = _height - 1;
                Vector<3> b; b.noalias() = ratios.transpose().rightCols(l) * obs(ind).tail(l);
                Vector<3> a = ratios.row(0);
                newAcc = - b.dot(a) / a.dot(a);
            }
        }
        ri = _replacedIndex(obs);
        if (_height < MAT_SIZE && ri < MAT_SIZE - _height)
            _height++;
        for (int i = ri; i < MAT_SIZE - 1; i++)
        {
            _observations.row(i) = _observations.row(i + 1);
        }
        _observations.row(MAT_SIZE - 1) = obs.transpose();
        if (!isnan(newAcc))
        {
            _duty = constrain((newAcc + MIN_ENGINE_PWM) / eng_cos, -100, 100);
            std::cout << obs.transpose().format(_HeavyFmt) << " " << newAcc << " " << _duty << " " << eng_cos;// << "\n";
            // std::cout << ratios.transpose().format(_HeavyFmt);
            _engine.setDuty(_duty); // printf("height=%i acc=%f\n", _height, newAcc);
        }
        else 
        {
            std::cout  << " newAcc is Nan!!!\n";
            _engine.setDuty(0);
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
