#include "PID.h"

using namespace std;

PID::PID() : tau_p_(0.0), tau_d_(0.0), tau_i_(0.0), p_error_(0.0), d_error_(0.0), i_error_(0.0)
{
}

PID::~PID()
{
}

void PID::Init(double k_p, double k_i, double k_d)
{
    this->tau_p_ = k_p;
    this->tau_d_ = k_d;
    this->tau_i_ = k_i;
}

void PID::UpdateError(double cte)
{
    d_error_ = cte - p_error_;
    i_error_ += cte;
    p_error_ = cte;
}

double PID::TotalError()
{
    return -tau_p_ * p_error_  - tau_d_ * d_error_  - tau_i_* i_error_;
}
