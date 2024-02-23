#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "../filters/pid_filter.hpp"
#include "../utils/timing.hpp"
#include "../comms/rm_can.hpp"
#include "../sensors/RefSystem.hpp"
#include "state.hpp"

#define NUM_GAINS 12
#define NUM_CONTROLLER_LEVELS 3
struct Controller
{

protected:
    float gains[NUM_GAINS];
    Timer timer;
    /// @brief defines controller inputs and outputs (0 means Macro_state input, micro_state output)
    /// (1 means Micro_state input, motor_current output) (2 means Macro state input, motor_current output)
    int controller_level;
    float gear_ratio = 0;
public:
    Controller(){};

    void set_gains(float _gains[NUM_GAINS])
    {
        for (int i = 0; i < NUM_GAINS; i++)
            gains[i] = _gains[i];
    }

    /// @brief Generates an output from a state reference and estimation
    /// @returns motor_current or Micro_state depending on controller_level
    virtual float step(float reference[3], float estimate[3]);
    /// @brief Generates an output from a state reference and estimation
    /// @returns motor_current 
    virtual float step(float reference, float estimate[MICRO_STATE_LEN]);

    /// @brief Resets integrators/timers
    virtual void reset() { timer.start_timer(); }
};

struct NullController : public Controller
{
public:
    float step(float reference[3], float estimate[3]) { return 0; }

    float step(float reference, float estimate[MICRO_STATE_LEN]) { return 0; }
};

struct PIDPositionController : public Controller
{
private:
    PIDFilter pid;

public:
    PIDPositionController(int _controller_level) {
        controller_level = _controller_level;
    }
    /// @brief step for macro_state input
    /// @param reference macro_reference
    /// @param estimate macro_estimate
    /// @return Motor output or micro_reference
    float step(float reference[3], float estimate[3])
    {
        float dt = timer.delta();

        pid.setpoint = reference[0]; // 0th index = position
        pid.measurement = estimate[0];
        pid.K[0] = gains[0];
        pid.K[1] = gains[1];
        pid.K[2] = gains[2];
        bool bounded = (controller_level == 2);
        float output = pid.filter(dt, bounded);
        return output;
    }
    /// @brief step for micro_state input
    /// @param reference micro_reference
    /// @param estimate micro_estimate
    /// @return Motor output
    float step(float reference, float estimate[MICRO_STATE_LEN])
    {
        float dt = timer.delta();

        pid.setpoint = reference; // 0th index = position
        pid.measurement = estimate[0];
        pid.K[0] = gains[0];
        pid.K[1] = gains[1];
        pid.K[2] = gains[2];

        float output = pid.filter(dt, true);
        return output;
    }

    void reset()
    {
        Controller::reset();
        pid.sumError = 0.0;
    }
};

struct PIDVelocityController : public Controller
{
private:
    PIDFilter pid;

public:
    PIDVelocityController(int _controller_level) {
        controller_level = _controller_level;
    }
    /// @brief step for macro_state input
    /// @param reference macro_reference
    /// @param estimate macro_estimate
    /// @return Motor output or micro_reference
    float step(float reference[3], float estimate[3])
    {
        float dt = timer.delta();

        pid.setpoint = reference[1]; // 1st index = position
        pid.measurement = estimate[1];
        pid.K[0] = gains[0];
        pid.K[1] = gains[1];
        pid.K[2] = gains[2];

        bool bounded = (controller_level == 2);
        float output = pid.filter(dt, bounded);
        return output;
    }
    /// @brief step for micro_state input
    /// @param reference micro_reference
    /// @param estimate micro_estimate
    /// @return Motor output
    float step(float reference, float estimate[MICRO_STATE_LEN])
    {
        float dt = timer.delta();

        pid.setpoint = reference; // 0th index = position
        pid.measurement = estimate[0];
        pid.K[0] = gains[0];
        pid.K[1] = gains[1];
        pid.K[2] = gains[2];

        float output = pid.filter(dt, true);
        return output;
    }

    void reset()
    {
        Controller::reset();
        pid.sumError = 0.0;
    }
};

struct FullStateFeedbackController : public Controller
{
private:
    PIDFilter pid1, pid2;

public:
    FullStateFeedbackController(int _controller_level)
    {
        controller_level = _controller_level;
        if (controller_level == 1)
            Serial.println("FullStateFeedbackController can't be a low level controller");
    }

    float step(float reference[3], float estimate[3])
    {
        float dt = timer.delta();
        float output = 0.0;

        pid1.K[0] = gains[0];
        pid1.K[1] = gains[1];
        pid1.K[2] = gains[2];
        pid2.K[0] = gains[3];
        pid2.K[1] = gains[4];
        pid2.K[2] = gains[5];

        pid1.setpoint = reference[0];
        pid1.measurement = estimate[0];

        pid2.setpoint = reference[1];
        pid2.measurement = estimate[1];

        output += pid1.filter(dt, true);
        output += pid2.filter(dt, true);

        return output;
    }

    float step(float reference, float estimate[MICRO_STATE_LEN]) { return 0; }

    void reset()
    {
        Controller::reset();
        pid1.sumError = 0.0;
        pid2.sumError = 0.0;
    }
};

struct ChassisPIDVelocityController : public Controller
{
private:
    PIDFilter pid;

public:
    ChassisPIDVelocityController(int _controller_level)
    {
        controller_level = _controller_level;
        if (controller_level != 1)
            Serial.println("chassisPIDVelocityController must be a low level controller");
    }
    /// @brief dont brick the program if its initialized at the wrong controller level
    float step(float reference[3], float estimate[3]) { return 0; }
    /// @brief takes in a micro_reference of wheel velocity
    /// @param reference reference
    /// @param estimate estimate
    /// @return outputs motor current
    float step(float reference, float estimate[MICRO_STATE_LEN])
    {
        float dt = timer.delta();
        pid.setpoint = reference; // 1st index = position
        pid.measurement = estimate[0];
        pid.K[0] = gains[0];
        pid.K[1] = gains[1];
        pid.K[2] = gains[2];

        // Power limiting
        float power_buffer = ref.ref_data.power_heat.buffer_energy;
        float power_limit_ratio = 1.0;
        float power_buffer_limit_thresh = gains[3];
        float power_buffer_critical_thresh = gains[4];
        if (power_buffer < power_buffer_limit_thresh) {
            power_limit_ratio = constrain(((power_buffer - power_buffer_critical_thresh) / power_buffer_limit_thresh), 0.0, 1.0);
        }
        float output = pid.filter(dt, true) * power_limit_ratio;
        return output;
    }

    void reset()
    {
        Controller::reset();
        pid.sumError = 0.0;
    }
};

#endif // CONTROLLER_H