https://stackoverflow.com/questions/18705363/arduino-uno-pwm-pins-conflict

So by writing 255, the analogWrite code ignores the whole PWM and output compare thing, and just writes the pin high.

Finally, as to solving your problem, I would personally go the route of not using pins 11 and 3 (timer2). Yes it will require a small rewiring, but that way you can free up timer2 for the IR library to use.

Alternatively, you could poke around the IR library and try to make it work without resetting the count.
