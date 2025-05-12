#ifndef myPID_H
#define myPID_H

class myPID
{
  public:
    float P      = 0.0;
    float I      = 0.0;
    float D      = 0.0;
    float output = 0.0;

    // Constructor
    myPID(float kp, float ki, float kd) : Kp(kp), Ki(ki), Kd(kd) { reset(); }

    // Compute PID output
    float compute(float target, float current, float dt)
    {
        error           = target - current;
        float error_dot = (error - error_prev) / dt;
        error_sum += error * dt;

        P = Kp * error;
        I += Ki * error * dt;
        D = Kd * error_dot;

        output     = P + I + D;
        error_prev = error;

        return output;
    }

    // Reset internal state
    void reset()
    {
        error      = 0.0;
        error_prev = 0.0;
        error_sum  = 0.0;
        I          = 0.0;
    }

    // Get the current error value
    float getError() const { return error; }

  private:
    float Kp         = 0.0;
    float Ki         = 0.0;
    float Kd         = 0.0;
    float error      = 0.0;
    float error_prev = 0.0;
    float error_sum  = 0.0;
};

#endif
