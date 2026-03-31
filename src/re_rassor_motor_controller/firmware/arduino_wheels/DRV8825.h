// Copyright 2025 UCF RE-RASSOR
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
#include <Arduino.h>
class DRV8825
{
private:
    unsigned short _step_pin;
    unsigned short _dir_pin;
    unsigned short _enable_pin;

    long _position;
    bool _direction;
    long _steps_per_revolution;
    bool _enabled;
    bool _invert_dir;

    bool _current_state = LOW;

    unsigned long _step_delay = 1000;
    unsigned long _last_step_time = 0;

    long _microstep_mode = 32;

public:
    void set_speed(long rpm);
    void update();
    DRV8825(unsigned short step_pin, unsigned short dir_pin, unsigned short enable_pin, long steps_per_revolution, bool invert_dir=false);
    void set_direction(bool direction);
    bool get_direction();
    long get_position();
    void set_enabled(bool enabled);
    bool get_enabled();
};