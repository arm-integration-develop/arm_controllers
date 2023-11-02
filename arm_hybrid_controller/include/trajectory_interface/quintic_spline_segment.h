//
// Created by lsy on 23-11-2.
//
#pragma once

#include <array>
#include <iterator>
#include <stdexcept>
#include <trajectory_interface/pos_vel_acc_state.h>

namespace trajectory_interface
{
class QuinticSplineSegment
{
public:
    QuinticSplineSegment(const double &  start_time,
                         const PosVelAccState<double>& start_state,
                         const double&  end_time,
                         const PosVelAccState<double>& end_state)
    {
        init(start_time, start_state, end_time, end_state);
    }
private:
    typedef std::array<double, 6> SplineCoefficients;
    std::vector<SplineCoefficients> coefs_;
    double duration_,start_time_,time_from_start_;

    void init(const double &  start_time,
              const PosVelAccState<double>& start_state,
              const double&  end_time,
              const PosVelAccState<double>& end_state);

    double startTime() const {return start_time_;}
    double endTime() const {return start_time_ + duration_;}
    double timeFromStart() const {return time_from_start_;}
    unsigned int size() const {return coefs_.size();}

    static void generatePowers(int n, const double& x, double* powers);

    static void computeCoefficients(const double& start_pos,
                                    const double& end_pos,
                                    const double& time,
                                    SplineCoefficients& coefficients);

    static void computeCoefficients(const double& start_pos, const double& start_vel,
                                    const double& end_pos,   const double& end_vel,
                                    const double& time,
                                    SplineCoefficients& coefficients);

    static void computeCoefficients(const double& start_pos, const double& start_vel, const double& start_acc,
                                    const double& end_pos,   const double& end_vel,   const double& end_acc,
                                    const double& time,
                                    SplineCoefficients& coefficients);
};

void QuinticSplineSegment::init(const double &  start_time,
                                            const PosVelAccState<double>& start_state,
                                            const double &  end_time,
                                            const PosVelAccState<double>& end_state)
{
    // Preconditions
    const unsigned int dim = start_state.position.size();
    const bool has_velocity     = !start_state.velocity.empty()     && !end_state.velocity.empty();
    const bool has_acceleration = !start_state.acceleration.empty() && !end_state.acceleration.empty();

    // Time data
    start_time_ = start_time;
    duration_   = end_time - start_time;
    time_from_start_ = start_state.time_from_start;

    // Spline coefficients
    coefs_.resize(dim);

    typedef typename std::vector<SplineCoefficients>::iterator Iterator;
    if (!has_velocity)
    {
        // Linear interpolation
        for(Iterator coefs_it = coefs_.begin(); coefs_it != coefs_.end(); ++coefs_it)
        {
            const typename std::vector<double>::size_type id = std::distance(coefs_.begin(), coefs_it);

            computeCoefficients(start_state.position[id],
                                end_state.position[id],
                                duration_,
                                *coefs_it);
        }
    }
    else if (!has_acceleration)
    {
        // Cubic interpolation
        for(Iterator coefs_it = coefs_.begin(); coefs_it != coefs_.end(); ++coefs_it)
        {
            const typename std::vector<double>::size_type id = std::distance(coefs_.begin(), coefs_it);

            computeCoefficients(start_state.position[id], start_state.velocity[id],
                                end_state.position[id],   end_state.velocity[id],
                                duration_,
                                *coefs_it);
        }
    }
    else
    {
        // Quintic interpolation
        for(Iterator coefs_it = coefs_.begin(); coefs_it != coefs_.end(); ++coefs_it)
        {
            const typename std::vector<double>::size_type id = std::distance(coefs_.begin(), coefs_it);

            computeCoefficients(start_state.position[id], start_state.velocity[id], start_state.acceleration[id],
                                end_state.position[id],   end_state.velocity[id],   end_state.acceleration[id],
                                duration_,
                                *coefs_it);
        }
    }
}

inline void QuinticSplineSegment::generatePowers(int n, const double & x, double * powers)
{
    powers[0] = 1.0;
    for (int i=1; i<=n; ++i)
    {
        powers[i] = powers[i-1]*x;
    }
}

void QuinticSplineSegment::computeCoefficients(const double& start_pos,
                    const double& end_pos,
                    const double& time,
                    SplineCoefficients& coefficients)
{
    coefficients[0] = start_pos;
    coefficients[1] = (time == 0.0) ? 0.0 : (end_pos - start_pos) / time;
    coefficients[2] = 0.0;
    coefficients[3] = 0.0;
    coefficients[4] = 0.0;
    coefficients[5] = 0.0;
}

void QuinticSplineSegment::computeCoefficients(const double& start_pos, const double& start_vel,
                    const double& end_pos,   const double& end_vel,
                    const double& time,
                    SplineCoefficients& coefficients)
{
    if (time == 0.0)
    {
        coefficients[0] = start_pos;
        coefficients[1] = start_vel;
        coefficients[2] = 0.0;
        coefficients[3] = 0.0;
    }
    else
    {
        double T[4];
        generatePowers(3, time, T);

        coefficients[0] = start_pos;
        coefficients[1] = start_vel;
        coefficients[2] = (-3.0*start_pos + 3.0*end_pos - 2.0*start_vel*T[1] - end_vel*T[1]) / T[2];
        coefficients[3] = (2.0*start_pos - 2.0*end_pos + start_vel*T[1] + end_vel*T[1]) / T[3];
    }
    coefficients[4] = 0.0;
    coefficients[5] = 0.0;
}

void QuinticSplineSegment::computeCoefficients(const double& start_pos, const double& start_vel, const double& start_acc,
                    const double& end_pos,   const double& end_vel,   const double& end_acc,
                    const double& time,
                    SplineCoefficients& coefficients)
{
    if (time == 0.0)
    {
        coefficients[0] = start_pos;
        coefficients[1] = start_vel;
        coefficients[2] = 0.5*start_acc;
        coefficients[3] = 0.0;
        coefficients[4] = 0.0;
        coefficients[5] = 0.0;
    }
    else
    {
        double T[6];
        generatePowers(5, time, T);

        coefficients[0] = start_pos;
        coefficients[1] = start_vel;
        coefficients[2] = 0.5*start_acc;
        coefficients[3] = (-20.0*start_pos + 20.0*end_pos - 3.0*start_acc*T[2] + end_acc*T[2] -
                           12.0*start_vel*T[1] - 8.0*end_vel*T[1]) / (2.0*T[3]);
        coefficients[4] = (30.0*start_pos - 30.0*end_pos + 3.0*start_acc*T[2] - 2.0*end_acc*T[2] +
                           16.0*start_vel*T[1] + 14.0*end_vel*T[1]) / (2.0*T[4]);
        coefficients[5] = (-12.0*start_pos + 12.0*end_pos - start_acc*T[2] + end_acc*T[2] -
                           6.0*start_vel*T[1] - 6.0*end_vel*T[1]) / (2.0*T[5]);
    }
}

}