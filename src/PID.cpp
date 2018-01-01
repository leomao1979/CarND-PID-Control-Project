#include "PID.h"

using namespace std;
// Number of recent CTEs to keep
static const int number_of_recent_cte = 10;

PID::PID() {
}

PID::~PID() {
}

void PID::Init(double Kp, double Ki, double Kd) {
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    
    this->p_error = 0;
    this->d_error = 0;
    this->i_error = 0;
    
    this->recent_ctes.clear();
    this->isFirstStep = true;
}

void PID::UpdateError(double cte) {
    if (isFirstStep) {
        d_error = 0;
        isFirstStep = false;
    } else {
        d_error = cte - p_error;
    }
    p_error = cte;
    i_error += cte;
    recent_ctes.push_back(cte);
    if (recent_ctes.size() > number_of_recent_cte) {
        double retired_cte = recent_ctes.front();
        i_error -= retired_cte;
        recent_ctes.erase(recent_ctes.begin());
    }
}

double PID::TotalError() {
    return Kp * p_error + Kd * d_error + Ki * i_error;
}

