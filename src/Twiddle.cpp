#include "Twiddle.h"
#include <math.h>
#include <iostream>
#include <limits>

using namespace std;

static const int eval_steps     = 2000;
static const int minimum_steps   = 700;
static const double tolerence   = 0.2;
static const int number_of_recent_speeds = 10;

// If pid.i_error exceeds the threshold, give up this round running
static const double cte_threshold        = 40;

Twiddle::Twiddle() {
    p[0] = 0.1;
    p[1] = 0.005;
    p[2] = 4.0;
   
    dp[0] = 0.1;
    dp[1] = 0.005;
    dp[2] = 0.2;

    for (int i=0; i < number_of_params; i++) {
        best_p[i] = p[i];
    }
    p_index = 0;
    best_error = std::numeric_limits<double>::max();
    shallReset = false;
    is_first_round = true;
    iteration = 0;
}

Twiddle::~Twiddle() {
}

void Twiddle::Start() {
    pid.Init(p[0], p[1], p[2]);
    current_error = 0;
    steps = 0;
    ++ iteration;
    isStuck = false;
    recent_speeds.clear();
    if (shallReset) {
        shallReset = false;
    }

    cout << "Iteration #" << iteration << ". ";
    cout << "Tuning p: [" << p[0] << " " << p[1] << " " << p[2] << "]" << ", dp: [" << dp[0] << " " << dp[1] << " " << dp[2] << "]" << endl;
}

void Twiddle::Update(double cte, double speed) {
    if (shallReset) {
        // Do nothing when in reset process
        return;
    }
    ++ steps;
    if (steps <= eval_steps) {
        pid.UpdateError(cte);
        current_error += cte * cte;
        UpdateSpeed(speed);
        CheckCTEs();
        if (isStuck || !cteInRange) {
            AdjustParam();
        }
    } else {
        AdjustParam();
    }
}

double Twiddle::TotalError() {
    return pid.TotalError();
}

bool Twiddle::ShallReset() {
    return shallReset;
}

bool Twiddle::IsCompleted() {
    double sum = 0.0;
    for (int i=0; i < number_of_params; i++) {
        sum += dp[i];
    }
    return sum <= tolerence;
}

void Twiddle::UpdateSpeed(double speed) {
    isStuck = false;
    recent_speeds.push_back(speed);
    if (recent_speeds.size() > number_of_recent_speeds) {
        recent_speeds.erase(recent_speeds.begin());
    }
    if (recent_speeds.size() >= number_of_recent_speeds) {
        double sum = 0;
        for (int i=0; i<recent_speeds.size(); i++) {
            sum += recent_speeds[i];
        }
        if (fabs(sum) <= 1) {
            // Vehicle is stuck if the average speed < 0.1
            isStuck = true;
        }
    }
}

void Twiddle::CheckCTEs() {
    cteInRange = (fabs(pid.i_error) <= cte_threshold);
}

void Twiddle::AdjustParam() {
    if (isStuck) {
        cout << "Vehicle is stuck. Skip.";
    }
    if (!cteInRange) {
        cout << "CTE is out of range. Skip.";
    }
    cout << " pid.i_error: " << pid.i_error << ", Steps: " << steps << endl;
    if (steps < minimum_steps) {
        // Ignore the result if it doesn't run the minium steps
        current_error = std::numeric_limits<double>::max();
        cout << "Insufficent steps taken, result ignored. Prev Best Error: " << best_error << endl;
    } else {
        current_error = sqrt(current_error / steps);
        cout << "Tuned p: [" << p[0] << " " << p[1] << " " << p[2] << "]. Current Error: " << current_error << ", Prev Best Error: " << best_error << endl;
    }
    if (is_first_round) {
        best_error = current_error;
        for (int i=0; i < number_of_params; i++) {
            best_p[i] = p[i];
        }
        p[p_index] += dp[p_index];
        is_first_round = false;
        p_ascending = true;
    } else {
        if (current_error < best_error) {
            best_error = current_error;
            for (int i=0; i < number_of_params; i++) {
                best_p[i] = p[i];
            }
            dp[p_index] *= 1.1;
            // Tune next parameter
            p_index = (p_index + 1) % number_of_params;
            p[p_index] += dp[p_index];
            p_ascending = true;
        } else {
            if (p_ascending) {
                // Descend parameter
                p[p_index] -= 2 * dp[p_index];
                p_ascending = false;
            } else {
                p[p_index] += dp[p_index];
                dp[p_index] *= 0.9;
                p_ascending = true;
                // Tune next parameter. Ascend first
                p_index = (p_index + 1) % number_of_params;
                p[p_index] += dp[p_index];
            }
        }
    }
    shallReset = true;
    // dp[] might be changed, check completion
    if (IsCompleted()) {
        logResult();
        exit(0);
    }
}

void Twiddle::logResult() {
    cout << endl;
    cout << "================= Result ============= " << endl;
    cout << "Total iterations: " << iteration << endl;
    cout << "Best Error: " << best_error << endl;
    cout << "Best P:  [" << best_p[0] << " " << best_p[1] << " " << best_p[2] << "]" << endl;
    cout << "dp: [" << dp[0] << " " << dp[1] << " " << dp[2] << "]" << endl;
    cout << "====================================== " << endl;
}
