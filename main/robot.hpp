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
#include <algorithm>
using std::vector;

float getTime();

void print_state (size_t iter, gsl_multiroot_fsolver * s)
{
  printf ("iter = %3u x = % .3f % .3f % .3f "
          "f(x) = % .3e % .3e % .3e %f\n",
          iter,
          gsl_vector_get (s->x, 0),
          gsl_vector_get (s->x, 1),
          gsl_vector_get (s->x, 2),
          gsl_vector_get (s->f, 0),
          gsl_vector_get (s->f, 1),
          gsl_vector_get (s->f, 2),
          getTime());
}

struct duty2pos_params_multy
{
    double pos[4], t[4], duty[3], v[4], acc[3];
};

int duty2pos_f (const gsl_vector * x, void *params, gsl_vector * f)
{
    auto pos = ((struct duty2pos_params_multy *) params)->pos;
    auto t = ((struct duty2pos_params_multy *) params)->t;
    auto duty = ((struct duty2pos_params_multy *) params)->duty;
    double x_[4];
    auto v = ((struct duty2pos_params_multy *) params)->v;

    const double b = gsl_vector_get (x, 0);
    const double h = gsl_vector_get (x, 1);
    const double i = gsl_vector_get (x, 2);
    const double revb = 1. / b;

    x_[0] = pos[0];
    for (uint8_t j = 0; j < 3; j++){
        const uint8_t k = j + 1;
        const double d = duty[j];
        const double g = h + i / d;
        const double dv = v[j] - g;
        const double dt = t[k] - t[j];
        const double ebdt = exp(b * dt);
        x_[k] = dv * revb * (ebdt - 1) + g * dt + x_[j];
        gsl_vector_set (f, j, x_[k] - pos[k]);
        v[k] = dv * ebdt + g;
    }

    return GSL_SUCCESS;
}

int duty2pos_df (const gsl_vector * x, void *params, gsl_matrix * J)
{
    auto t = ((struct duty2pos_params_multy *) params)->t;
    auto duty = ((struct duty2pos_params_multy *) params)->duty;
    auto acc = ((struct duty2pos_params_multy *) params)->acc;

    const double b = gsl_vector_get (x, 0);
    const double h = gsl_vector_get (x, 1);
    const double i = gsl_vector_get (x, 2);

    double dvdb[3], dvdi[3], dvdh[3], dfdb[4], dfdh[4], dfdi[4];
    auto v = ((struct duty2pos_params_multy *) params)->v;
    dvdb[0] = 0; dvdi[0] = 0; dvdh[0] = 0;
    dfdb[0] = 0; dfdi[0] = 0; dfdh[0] = 0;
    const double revb = 1. / b;

    for (uint8_t j = 0; j < 3; j++){
        const uint8_t k = j + 1;
        const double d = duty[j];
        const double dgdi = 1 / d;
        const double g = (h * d + i) * dgdi;
        // const double ln_duty = log(d);
        const double dv = v[j] - g;
        const double dt = t[k] - t[j];
        const double ebdt = exp(b * dt);
        const double edt_less_1 = ebdt - 1;
        const double edt_less_1_by_b = edt_less_1 * revb;
        const double dt_ebdt = dt * ebdt;
        dfdb[k] = revb * (dv * (dt_ebdt - edt_less_1_by_b) + dvdb[j] * edt_less_1) + dfdb[j];
        dfdi[k] = edt_less_1_by_b * (dvdi[j] - dgdi) + dt * dgdi + dfdi[j];
        // const double dgdh = 1;
        dfdh[k] = edt_less_1_by_b * (dvdh[j] - 1) + dt + dfdh[j];
        gsl_matrix_set (J, j, 0, dfdb[k]);
        gsl_matrix_set (J, j, 1, dfdh[k]);
        gsl_matrix_set (J, j, 2, dfdi[k]);
        acc[j] = dv * b;
        if (k < 3)
        {
            v[k] = dv * ebdt + g;
            dvdb[k] = ebdt * (dvdb[j] + dt * dv);
            dvdi[k] = ebdt * (dvdi[j] - dgdi) + dgdi;
            dvdh[k] = ebdt * (dvdh[j] - 1) + 1;
        }
    }

    return GSL_SUCCESS;
}

int duty2pos_fdf (const gsl_vector * x, void *params, gsl_vector * f, gsl_matrix * J)
{
    duty2pos_f (x, params, f);
    duty2pos_df (x, params, J);
    return GSL_SUCCESS;
}

struct DutyReturnMulty
{
    vector<double> x_init;
    double duty;
}; 

DutyReturnMulty get_duty_multy(duty2pos_params_multy &p, double x_init[3], double a4)
{
    int status;
    size_t iter = 0;
    const size_t n = 3;

    gsl_vector *x = gsl_vector_alloc(n);

    gsl_vector_set(x, 0, x_init[0]);
    gsl_vector_set(x, 1, x_init[1]);
    gsl_vector_set(x, 2, x_init[2]);

    gsl_multiroot_function_fdf fdf = {&duty2pos_f,
                                    &duty2pos_df,
                                    &duty2pos_fdf,
                                    n, &p};

    auto s = gsl_multiroot_fdfsolver_alloc (gsl_multiroot_fdfsolver_gnewton, n);
    gsl_multiroot_fdfsolver_set (s, &fdf, x);

    print_state (iter, (gsl_multiroot_fsolver*)s);

    do
        {
        iter++;

        status = gsl_multiroot_fdfsolver_iterate (s);

        print_state (iter, (gsl_multiroot_fsolver*)s);

        if (status)
            break;

        status = gsl_multiroot_test_residual (s->f, 1e-7);
        }
    while (status == GSL_CONTINUE && iter < 50);

    printf ("status = %s\n", gsl_strerror (status));

    // std::cout << p.v[0] << " " << p.v[1] << " " << p.v[2] << " "<< p.v[3] << "\n";
    // std::cout << p.acc[0] << " " << p.acc[1] << " " << p.acc[2] << "\n";
    auto b = gsl_vector_get(s->x, 0);
    auto h = gsl_vector_get(s->x, 1);
    auto i = gsl_vector_get(s->x, 2);

    DutyReturnMulty res = {vector<double>{b, h, i}, i / (p.v[3] - a4 / b - h)};

    // res.duty = exp((pow(p.v[3] - a4 / b, 3) - i)/h);
    // std::cout << res.duty << "\n";

    gsl_multiroot_fdfsolver_free (s);

    gsl_vector_free(x);
    // sleep(10);
    return res;
}

#include <gsl/gsl_roots.h>

struct duty2pos_params
{
    vector<double> pos, t;
    double v, g, v1;
};

double quadratic (double b, void *params)
{
    struct duty2pos_params *p = (struct duty2pos_params *) params;
    auto pos = p->pos;
    auto t = p->t;
    auto g = p->g;
    const double dv = p->v - g;
    const double dt = t[1] - t[0];
    const double ebdt = exp(b * dt);
    p->v1 = dv * ebdt + g;
    return dv / b * (ebdt - 1) + g * dt + pos[0] - pos[1];
}

double quadratic_deriv (double b, void *params)
{
    struct duty2pos_params *p = (struct duty2pos_params *) params;
    auto t = p->t;
    auto v = p->v;
    const double revb = 1. / b;
    const double dt = t[1] - t[0];
    const double ebdt = exp(b * dt);
    return revb * ((v - p->g) * (dt * ebdt - (ebdt - 1) * revb));
}

void quadratic_fdf (double x, void *params, double *y, double *dy)
{
    struct duty2pos_params *p
        = (struct duty2pos_params *) params;

    *y = quadratic(x, p);
    *dy = quadratic_deriv(x, p);
}

struct DutyReturn
{
    double x_init, duty, v1, a2;
};

inline double duty2g(const double d, const double h, const double i)
{
    if (abs(d) < -i / h) return 0;
    auto g = std::max(0., h + i / abs(d));
    if (d < 0) return -g;
    return g;
}

inline double g2duty(const double g, const double h, const double i)
{
    // assert(abs(g) < abs(h) && h > 0 && i < 0);
    auto abs_g = abs(g);
    double d;
    if (abs_g < h + i) d = i / (abs_g - h);
    else d = 1;
    if (g < 0) return -d;
    return d;
}

DutyReturn get_duty(
    const vector<double> pos,
    const vector<double> t, 
    double v, 
    const double d, 
    const double r_expected, 
    const double a4, 
    const double h, 
    const double i)
{
    printf("{%f, %f},\n{%f, %f},\n%f,\n%f,\n%f,\n%f,\n%f,\n%f\n", pos[0], pos[1], t[0], t[1], v, d, r_expected, a4, h, i);
    // const auto v_lim = h + i;
    // v = constrain(v, -v_lim, v_lim);
    // printf("a4 = %f\n", a4);
    int status;
    int iter = 0, max_iter = 100;
    const gsl_root_fdfsolver_type *T;
    gsl_root_fdfsolver *s;
    double x0, x = r_expected;
    gsl_function_fdf FDF;

    FDF.f = &quadratic;
    FDF.df = &quadratic_deriv;
    FDF.fdf = &quadratic_fdf;
    auto g = duty2g(d, h, i);
    duty2pos_params p = {pos, t, v, g};
    FDF.params = &p;

    T = gsl_root_fdfsolver_newton;
    s = gsl_root_fdfsolver_alloc(T);
    gsl_root_fdfsolver_set(s, &FDF, x);

    // printf ("using %s method\n",
    //         gsl_root_fdfsolver_name (s));

    // printf ("%-5s %10s %10s %10s %10i\n",
    //         "iter", "root", "err", "err(est)", (int) getTime());
    do
    {
        iter++;
        status = gsl_root_fdfsolver_iterate(s);
        x0 = x;
        x = gsl_root_fdfsolver_root(s);
        status = gsl_root_test_delta(x, x0, 0, 1e-3);

        // if (status == GSL_SUCCESS)
        //     printf ("Converged:\n");

        printf("%5d %10.7f %+10.7f %10.7f %f\n",
               iter, x, x - r_expected, x - x0, getTime());
    } while (status == GSL_CONTINUE && iter < max_iter);

    gsl_root_fdfsolver_free(s);
    auto g1 = p.v1 - a4 / x;
    DutyReturn res = {x, g2duty(g1, h, i), p.v1, x * (p.v1 - g)};
    // auto a2 = x * (p.v1 - g);
    printf("g = %f v1 = %f duty = %f a2=%f\n", g, p.v1, res.duty, res.a2);

    // sleep(10);
    return res;
}

template <typename T>
int sgn(T val)
{
    return (T(0) < val) - (val < T(0));
}

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
    // float _prevTime = 0;
    KalmanFilter _kf;
    uint8_t _sampleSize = 1;
    bool _useKalman = false;
    uint8_t _counter = 0;
    float _accum_angles = 0;
public:
    Sensor(){}
    Sensor(uint8_t sampleSize, bool useKalman) : _sampleSize(sampleSize), _useKalman(useKalman){}
    // Dynamics observations;
    virtual float angle();
    float update(float dt)
    {
        float pos = 1000000;
        _accum_angles += angle();
        _counter ++;
        if (_counter == _sampleSize){
            auto ang = _accum_angles / _sampleSize;
            // std::cout << "ang=" << ang << "\n";
            _counter = 0; _accum_angles = 0;
            // auto ang = angle();
            // float t = getTime();
            // if (_prevTime != 0)
            {
                // auto dt = t - _prevTime;
                pos = ang; 
                //auto &obs = observations;
                // printf("obs.pos=%f pos=%f ", obs.pos, pos);
                // if (pos - obs.pos < -1) pos += 2;
                // else if (pos - obs.pos > 1) pos -= 2;
                // printf("%f\n", pos);
                // printf("pos: %2.0f \n", pos);
                if (_useKalman){
                    // printf("using Kalman\n");
                    pos = _kf.get_pos(pos, dt);
                }
                // printf("pos: %2.0f\n", pos);
                // printf("pos: %2.4f, velocity: %4.4f, acc: %6.4f, dt: %1.4f t: %1.4f ", obs.pos, obs.velocity, obs.acc, dt, t);
                // auto newAcc = (2 * (pos - observations.pos) - observations.velocity * dt) / (dt * dt);
                // observations.velocity += newAcc * dt;
                // observations.acc = newAcc;
                // observations.pos = pos;

                // auto rev_dt = 1000000.f / (dt);
                // auto newVelocity = (pos - obs.pos) * rev_dt;
                // obs.acc = (newVelocity - obs.velocity) * rev_dt;
                // obs.velocity = newVelocity;
                // printf("pos: %2.4f, velocity: %4.4f, acc: %6.4f\n", obs.pos, obs.velocity, obs.acc);
            }
            // else 
            // {
            //     observations.pos = ang;
            // }
            // _prevTime = t;
        }
        return pos;
    };
};

#include <float.h>

class Engine : public Sensor
{
    float _duty = 0;
public:
    Engine();
    void setDuty(float duty);
    float duty(){ return _duty;}
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
#define MAT_SIZE 3 + 3

bool isZero(Vector<MAT_SIZE> col){
    return (col.array() == 0).all();
}

Dynamics observation(float pos0, float pos1, float vel0, float dt)
{
    // printf("pos: %2.0f\n", pos0);
    // printf("pos: %2.4f, velocity: %4.4f, acc: %6.4f, dt: %1.4f t: %1.4f ", pos0, vel0, dt);
    auto newAcc = (2 * (pos1 - pos0) - vel0 * dt) / (dt * dt);
    return {pos1, newAcc * dt, newAcc};
}

class Solver
{
    Sensor _sensor;
    Engine _engine;
    Eigen::PartialPivLU<Matrix<MAT_SIZE, MAT_SIZE>> _partialSolver;

    Matrix<MAT_SIZE, MAT_SIZE + 2> _observations = Matrix<MAT_SIZE, MAT_SIZE + 2>::Zero();
    int _height = 0;
    double _duty = 100;

    std::vector<int> _getBasisIndices()
    {
        Eigen::Matrix<float, Eigen::Dynamic, MAT_SIZE, Eigen::RowMajor, MAT_SIZE, MAT_SIZE> m;  
        m = _observations.bottomLeftCorner(_height, MAT_SIZE);
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

    int _replacedIndex(Vector<MAT_SIZE + 2> &obs)
    {
        Eigen::Matrix<float, MAT_SIZE, MAT_SIZE + 1, Eigen::RowMajor> rowReduce;
        rowReduce << _observations.leftCols(MAT_SIZE).transpose(), obs.head(MAT_SIZE);
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

    Vector<MAT_SIZE> _nextObservations(const Vector<MAT_SIZE + 2> &obs, const int index)
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
        // _engine.setDuty(100);
    }

    struct changeEngineAccType
    {
        double x_init;
        float time;
    };

    double changeEngineAcc(const double x_init)
    {
        auto curr_time = getTime();
        auto dt = curr_time - _observations(MAT_SIZE - 1, 6);
        auto s = _sensor.update(dt);
        if (s == 1000000) 
            return x_init;
        auto e = _engine.update(dt);
        if (e == 1000000) 
            return x_init;
        _engine.setDuty(_duty); // printf("height=%i acc=%f\n", _height, newAcc);
        Vector<MAT_SIZE + 2> obs;
        if (_observations(MAT_SIZE - 1, 6) != 0)
        {
            auto s_obs = observation(_observations(MAT_SIZE - 1, 1), s, _observations(MAT_SIZE - 1, 2), dt);
            auto e_obs = observation(_observations(MAT_SIZE - 1, 4), e, _observations(MAT_SIZE - 1, 5), dt);
            obs << e_obs.acc, s, s_obs.velocity, s_obs.acc, e, e_obs.velocity, curr_time, _engine.duty() / 100.; 
        }
        else 
        {
            obs << 0, s, 0, 0, e, 0, curr_time, _engine.duty() / 100.; 
        }
        // auto eng_cos = sqrtf(1 - engObs->pos * engObs->pos);
        // if (eng_cos == 0 or isnan(eng_cos)) 
            // std::cout  << "eng_cos is " << eng_cos << " " << engObs->pos << "\n";
        if (isZero(obs.head(MAT_SIZE)))
            return x_init;
        float newAcc = NAN;
        int ri;
        if (_height == MAT_SIZE)
        {
            _partialSolver.compute(_observations.leftCols(MAT_SIZE));
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
                auto obsBasis = _observations.bottomLeftCorner(_height, MAT_SIZE)(Eigen::all, ind);
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
            if(_height > 3)
            {
                auto pos = _observations.col(4).tail(2);
                auto t = _observations.col(6).tail(2);
                auto duties = _observations.col(7).tail(2);
                // duty2pos_params_multy p = {
                //     {pos[0], pos[1], pos[2], pos[3]},
                //     {t[0], t[1], t[2]},
                //     {duties[0], duties[1], duties[2]},
                //     {_observations.col(5).tail(4)[0], 0, 0, 0}, // velocity
                //     {0, 0, 0} // acceleration
                // };
                // auto duty_pos = get_duty_multy(p, x_init.data(), newAcc);

                // struct duty2pos_params p = {
                //     {pos[0], pos[1]},                // positions
                //     {t[0], t[1]},                    // time
                //     _observations.col(5).tail(2)[0], // velocity
                //     0.73-.13*duties[0],                            // h
                //     0 // v1
                // };
                const Eigen::IOFormat _LightFmt(5, 0, " ", "\n", "", "", "\n", "");
                std::cout << _observations.format(_LightFmt) << "\n";
                auto duty_pos = get_duty(
                    {pos[0], pos[1]}, 
                    {t[0], t[1]}, 
                    _observations.col(5).tail(2)[0], 
                    duties[0], 
                    x_init, 
                    newAcc, 
                    10.96, 
                    -1.93);

                _duty = duty_pos.duty * 100;
                _duty = constrain(_duty, -100, 100);
                // std::cout << obs.transpose().format(_HeavyFmt) << " " << newAcc << " " << _duty << "\n";
                // std::cout << ratios.transpose().format(_HeavyFmt);
                // _engine.setDuty(_duty); // printf("height=%i acc=%f\n", _height, newAcc);
                _observations(MAT_SIZE - 1, MAT_SIZE - 1) = duty_pos.v1;
                _observations(MAT_SIZE - 1, 0) = duty_pos.a2;
                return duty_pos.x_init;
            }
            return x_init;
        }
        else 
        {
            std::cout  << " newAcc is Nan!!!\n";
            // _engine.setDuty(0);
            return x_init;
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

        auto d = get_duty(
            {1, 1.392402504}, // position
            {1, 1.738686685}, // time
            1,                // velocity
            -1,                // duty
            -2,               // expected b
            -0.764385485,      // acceleration
            0.73,             // h
            -0.13             // i
        );
        assert(abs(d.duty + 1.) < 0.001);

        d = get_duty(
            {1, 1.420538094}, // position
            {1, 1.738686685}, // time
            1,                // velocity
            -0.5,                // duty
            -2,               // expected b
            -0.832279164,      // acceleration
            0.73,             // h
            -0.13             // i
        );
        assert(abs(d.duty + 1.) < 0.001);

        d = get_duty(
            {1, 1.522259072}, // position
            {1, 1.738686685}, // time
            1,                // velocity
            -0.1,                // duty
            -2,               // expected b
            -1.077740928,      // acceleration
            0.73,             // h
            -0.13             // i
        );
        assert(abs(d.duty + 1.) < 0.001);

        d = get_duty(
            {1, 1.522259072}, // position
            {1, 1.738686685}, // time
            1,                // velocity
            0,                // duty
            -2,               // expected b
            -1.077740928,      // acceleration
            0.73,             // h
            -0.13             // i
        );
        assert(abs(d.duty + 1.) < 0.001);

        d = get_duty(
            {1, 1.522259072}, // position
            {1, 1.738686685}, // time
            1,                // velocity
            0.1,                // duty
            -2,               // expected b
            -1.077740928,      // acceleration
            0.73,             // h
            -0.13             // i
        );
        assert(abs(d.duty + 1.) < 0.001);

        d = get_duty(
            {1, 1.62398005}, // position
            {1, 1.738686685}, // time
            1,                // velocity
            0.5,                // duty
            -2,               // expected b
            -1.323202692,      // acceleration
            0.73,             // h
            -0.13             // i
        );
        assert(abs(d.duty + 1.) < 0.001);

        d = get_duty(
            {1, 1.65211564}, // position
            {1, 1.738686685}, // time
            1,                // velocity
            1,                // duty
            -2,               // expected b
            -1.391096371,      // acceleration
            0.73,             // h
            -0.13             // i
        );
        assert(abs(d.duty + 1.) < 0.001);

        ESP_LOGI("tests", "done!!!");
    }
};
