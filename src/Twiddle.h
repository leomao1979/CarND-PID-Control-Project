#ifndef TWIDDLE_H
#define TWIDDLE_H

#include <vector>
#include "PID.h"

class Twiddle {
public:
    /*
     * Constructor
     */
    Twiddle();
    
    /*
     * Destructor.
     */
    virtual ~Twiddle();
    
    void Start();
    
    /*
     * Update the PID error variables given cross track error.
     */
    void Update(double cte, double speed);
    
    /*
     * Calculate the total PID error.
     */
    double TotalError();
    
    /*
     * Tells whether the simulator shall be reset (to prepare for the new round running)
     */
    bool ShallReset();
    
    /*
     * Is the parameter tuning completed
     */
    bool IsCompleted();
    
private:
    PID pid;
    static const int number_of_params = 3;
    double p[number_of_params];	    // Parameters [Kp, Ki, Kd] for PID controller
    double dp[number_of_params];    // Delta P
    int p_index;                    // Index of parameter being tuned
    bool p_ascending;               // Is parameter ascending?
    bool is_first_round;            // First round running?
    
    double best_error;
    double best_p[number_of_params];
    double current_error;

    int steps;		        // The steps taken for this round of running
    bool shallReset;
    int iteration;
    
    bool isStuck;           // Is the vehicle stuck when running
    bool cteInRange;        // Are the recent CTEs still in range
    std::vector<double> recent_speeds;
    void AdjustParam();
    void logResult();
    void UpdateSpeed(double speed);
    void CheckCTEs();
};

#endif /* TWIDDLE_H */
