
float vertPower = 0;

float integral = 0;

float derivative;

float prevError = 0;

float error = 0;
// use the same dt as the slew loop
if (joystick == 0)
{
    error = abs(currentDepth - lastDepth);

    integral += error * dt;

    derivative = (error - prevError)/ dt

    vertPower = error * kP + integral * kI + derivative * kD;

    prevError = error;
}
Motor set (vertPower)