class slewSlop {
    private:
        // update output starting value? or maybe not if we assume 0 is no power here
        float output = 0;
    public:
        float accelRate, decelRate;

        float slewVal(float accel, float decel): accelRate(accel), decelRate(decel) {}

        float update (float input, float dt)
        {
            float error = input - output;

            // the maximum acceleration allowed
            float rate;

            // checking direction to determine acceleration or deceleartion
            bool sameDirection = (input >= 0 && output >= 0) || (input <= 0 && output <= 0);

            if (!sameDirection) {
                rate = decelRate;
            }
            else if (abs(input) >= abs(output)) {
                rate = accelRate;
            }
            else {
                rate = decelRate;
            }

            // maximum change in velocity around
            float maxChange = rate * dt;

            if (error > maxChange)
            {
                output += maxChange;
            }
            else if (error < maxChange) {
                output -= maxChange;
            }
            else {
                output = input;
            }

            return output;
        }
}

slewSlop forward.slewVal (0,1);