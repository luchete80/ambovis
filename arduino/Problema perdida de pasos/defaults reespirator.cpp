// Stepper
#define STEPPER_MICROSTEPS                  4       // microsteps per step
#define STEPPER_STEPS_PER_REVOLUTION        200     // steps
#define STEPPER_DRIVER_INVERTED_LOGIC       true    /** @warning check your stepper driver specs! */

#define STEPPER_MICROSTEPS_PER_REVOLUTION   (STEPPER_STEPS_PER_REVOLUTION * STEPPER_MICROSTEPS)
#define STEPPER_DIR                         1
#define STEPPER_HOMING_DIRECTION            (-1)
#define STEPPER_HOMING_SPEED                (STEPPER_MICROSTEPS * 100)    // steps/s
#define STEPPER_LOWEST_POSITION             (STEPPER_MICROSTEPS *  85)    // steps
#define STEPPER_HIGHEST_POSITION            (STEPPER_MICROSTEPS * -100)   // steps
#define STEPPER_SPEED_DEFAULT               (STEPPER_MICROSTEPS *  800)   // steps/s
#define STEPPER_ACC_EXSUFFLATION            (STEPPER_MICROSTEPS *  600)   // steps/s2
#define STEPPER_ACC_INSUFFLATION            (STEPPER_MICROSTEPS *  450)   // steps/s2
