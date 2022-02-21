# Makerverse Motor Driver 2 Channel MicroPython Module

This is the firmware repo for the Core Electronics [Makerverse Motor Driver 2 Channel](https://core-electronics.com.au/catalog/product/view/sku/ce08038).

See the [Application Guide](https://core-electronics.com.au/tutorials/raspberry-pi-pico/makerverse-motor-driver-2-channel-application-guide.html) for more documentation.

# Usage

## Examples

The minimum requirements to drive a single motor at full speed, connected to the default pins (GP0 for PWM, GP1 for DIR):

```python
from Makerverse_Motor_2ch import motor

m1 = motor()
m1.go()
```

A more advanced example which drives a motor at 80% speed, with the DIR pin on the default of GP1 and PWM pin set to GP2.

The motor is driven until GP3 becomes high (simulating, say, an infrared sensor detecting an obstacle) and is then switched off.

```python
from Makerverse_Motor_2ch import motor
from machine import Pin

sensor = Pin(3, Pin.IN, Pin.PULL_DOWN)

m1 = motor(pwmPin = 2, speed = 80)
m1.go()

# Wait for the sensor to go high
while sensor.value() is 0:
	continue
	
# If we get here the the sensor detected something, so we'll stop the motor.
m1.stop()
```

Controlling a bipolar stepper motor connected to the default pins (GP0 to GP3 for PWMA, DIRA, PWMB, and DIRB).

This basic example moves the motor 100 steps forward, 45 degrees backward, then returns home (ie: to where it started).

```python
from Makerverse_Motor_2ch import bipolarStepper

stepper = bipolarStepper()

stepper.rotate(50)
stepper.rotate(angle=-180)
stepper.returnHome()
```

## Details - motor Class

### Constructor: motor(pwmPin = 0, dirPin = 1, speed = 100, pwmFreq = 200)

Returns a motor object given the microcontroller pins which are connected to the PWM and DIR inputs on the motor driver.

Parameter | Type | Range | Default | Description
--- | --- | --- | --- | ---
pwmPin | int or Pin | 0 to 28 | 0 | The Raspberry Pi Pico pin which is connected to the motor driver channel's PWM pin. This can be an integer or Pin object.
dirPin | int or Pin | 0 to 28 | 1 | The Raspberry Pi Pico pin which is connected to the motor driver channel's DIR pin. This can be an integer or Pin object.
speed | int | -100 to +100 | 100 | The initial relative speed value. Negative values imply reverse.
pwmFreq | int | 8 to 400000 | 200 | The pulse width modulation frequency (in Hz) when the motor is driven at a speed below 100%. If unsure, use the 200Hz default. Lower values (50-100Hz) tend to increase DC motor efficiency, especially at low duty cycles. The 400kHz maximum is the limit of the TC78H660 motor driver chip while the 8Hz minimum is set by the RP2040's PWM hardware.

### motor.speed(speed)

Sets the motor's speed given a percentage value from -100 to 100. Negative values imply a reverse direction while positive values drive the motor forward. A value of 0 causes the motor to stop.

If the speed argument exceeds the +/- 100 range the setting saturates at +100 or -100.

Any changes to the motor speed will apply immediately.

Note that a given speed value doesn't guarantee that the motor will move - there will always be a minimum speed value
required to sufficiently "drive" the motor to overcome static friction, especially if it is under any static load.

Parameter | Type | Range | Default | Description
--- | --- | --- | --- | ---
speed | int | -100 to +100 | - | The relative motor speed. Negative values drive the motor in reverse while positive drive forward. This argument is required.

### motor.stop()

Stops the motor.

A subsequent call to motor.go() will cause the motor to return to the previously set speed and direction.

### motor.go()

Turns the motor on with the previously set speed and direction.

### motor.forward()

Sets the motor direction to forward. This has immediate effect on the physical motor.

### motor.reverse()

Sets the motor direction to reverse. This has immediate effect on the physical motor.

## Details - twoMotorRobot Class

### Constructor: twoMotorRobot(pwmPinLeft = 0, dirPinLeft = 1, pwmPinRight = 2, dirPinRight = 3)

Returns a twoMotorRobot object given the microcontroller pins which are connected to the PWM and DIR inputs on each motor's driver.

This class is intended to drive pairs of motors fitted to a robot chassis with two driven wheels at the front and a castor wheel at the rear.

This class assumes that "forward" is the same motor driver output polarity on each motor. This typically requires the motors to be physically connected with opposite polarity.

### twoMotorRobot.speed(speed = 100)

Sets the speed of both motors given a percentage value from -100 to 100. Negative values imply a reverse direction while positive values drive the motor forward. A value of 0 causes the motor to stop.

This method will physically apply the new speed before returning.

Note that a given speed value doesn't guarantee that the motors will move - there will always be a minimum speed value required to sufficiently "drive" the motor to overcome static friction.

If the speed of each motor needs to be set independently twoMotorRobot.motorLeft.speed() and twoMotorRobot.motorRight.speed() can be called.

Parameter | Type | Range | Default | Description
--- | --- | --- | --- | ---
speed | int | -100 to +100 | 0 | The relative motor speed. Negative values drive the motors in reverse while positive values drive forward.

### twoMotorRobot.turnLeft()

Stops the left motor while continuing the right motor forward, causing the robot to turn left.

### twoMotorRobot.turnRight()

Stops the right motor while continuing the left motor forward, causing the robot to turn right.

### twoMotorRobot.rotateLeft()

Drives the left motor forward and the right motor in reverse, causing the robot to rotate left on the spot.

### twoMotorRobot.rotateRight()

Drives the right motor forward and the left motor in reverse, causing the robot to rotate right on the spot.

### twoMotorRobot.driveForward()

Sets both motors to forward, causing the robot to drive forward.

### twoMotorRobot.driveReverse()

Sets both motors to reverse, causing the robot to drive in reverse.

### twoMotorRobot.stop()

Stops both motors.

## Details - bipolarStepper Class

### Constructor: bipolarStepper(pwmPinA = 0, dirPinA = 1, pwmPinB = 2, dirPinB = 3, RPM = 10, stepsPerRotation = 200)

Returns a bipolarStepper object given the Raspberry Pi Pico pins which are connected to the PWM and DIR inputs driving each of the two stepper motor coils.

This class is only designed for driving stepper motors through whole steps - half steps and microstepping are not supported.

It is assumed that the motor is in the "home" position - the internal step count is initialised at zero.

From the perspective of this class the definition of "forward" and "reverse" is arbitrary. Swapping the polarity of one stepper motor phase will swap the physical forward/reverse directions.

Parameter | Type | Range | Default | Description
--- | --- | --- | --- | ---
pwmPinA | int or Pin | 0 to 28 | 0 | The Raspberry Pi Pico pin which is connected to the motor driver channel's PWM A pin. This can be an integer or Pin object.
dirPinA | int or Pin | 0 to 28 | 1 | The Raspberry Pi Pico pin which is connected to the motor driver channel's DIR A pin. This can be an integer or Pin object.
pwmPinB | int or Pin | 0 to 28 | 0 | The Raspberry Pi Pico pin which is connected to the motor driver channel's PWM B pin. This can be an integer or Pin object.
dirPinB | int or Pin | 0 to 28 | 1 | The Raspberry Pi Pico pin which is connected to the motor driver channel's DIR B pin. This can be an integer or Pin object.
RPM | int or float | -100 to +100 | 0 to 100, motor dependent | 10 | The RPM of the stepper motor when moved with bipolarStepper.rotate() or bipolarStepper.returnHome(). Higher values reduce motor torque and risk skipping steps.
stepsPerRotation | int | - | 200 | The number of steps in a full rotation.

### bipolarStepper.setRPM(RPM)

Changes the stepper motor's RPM when moving with bipolarStepper.rotate() or bipolarStepper.returnHome().

Note that the RPM argument is required.

Parameter | Type | Range | Default | Description
--- | --- | --- | --- | ---
RPM | int or float | -100 to +100 | 0 to 100, motor dependent | Unspecified | The RPM of the stepper motor when moved with bipolarStepper.rotate() or bipolarStepper.returnHome(). Higher values reduce motor torque and risk skipping steps.

### bipolarStepper.setHome()

Resets the internal step count to zero. The current physical position of the stepper motor becomes the new "home" position.

### bipolarStepper.getSteps()

Returns the number of steps rotated, relative to the home position. A positive number indicates "forward" steps while a negative number indicates a net "backward" rotation.

This value does not "wrap" at a full rotation. If the motor rotates 10 times in one direction it will return 10*stepsPerRotation.

eg: If there have been 20 steps forward and 5 steps backward this function will return 15.

### bipolarStepper.returnHome()

Rotates the stepper motor to the "home" position by reversing the currently accumulated step count. The motor will rotate at the RPM given by the constructor (default 10 RPM) or the last call to bipolarStepper.setRPM(RPM).

Note that this is an "absolute" position - the step count does not reset after a full rotation but rather accumulates.

eg: If the motor has undergone 3 full "forward" rotations calling bipolarStepper.returnHome() will result in 3 full rotations "reverse".

### bipolarStepper.forwardStep()

Perform a single step in the "forward" direction.

The number of steps taken since initialisation (or a call to bipolarStepper.setHome()) is remembered internally.

### bipolarStepper.backwardStep()

Perform a single step in the "backward" direction.

The number of steps taken since initialisation (or a call to bipolarStepper.setHome()) is remembered internally.

### bipolarStepper.rotate(steps = 0, angle = None)

Rotates the stepper motor through a given number of steps or, if the angle argument is specified, through a given angle (rounded to the nearest step).

If both steps and angle are given then the angle argument is used and the steps argument ignored.

A postive argument results in a "forward" rotation while a negative one rotates the stepper "backward".

The speed of rotation is controlled by the RPM argument passed at initialisation or the last call to bipolarStepper.setRPM().

Parameter | Type | Range | Default | Description
--- | --- | --- | --- | ---
steps | int | -inf to +inf | 0 | The number of steps to rotate. Positve values rotate "forward" while negative values rotate "backward".
angle | float or int | -inf to +inf | None | An angle to rotate. If this keyword argument is provided the steps argument is ignored (and not required).