# Mindscape 2 - FUTUR ENGINEERS - WRO 25 Lebanon
## Team members 
1- Jean-Paul Eghnatios <br>
2- Salim Gergess <br>
3- Camil Ouneissy <br>

## Table of content
| Content | Folder 
| - | - |
| Team members official picture | Team-photo 
| Team members funny picture | Team-photo 
| Vehicule different angles pictures | Vehicule-photo 
| vehicule video on the map | Video
| Self driving car code | src
| the electromechanical circuits blueprints | schemes
| The car base model build | Models

# The Mission
## The map
<img src="Other\WRO-2025_FutureEngineers_Playfield_page-0001 (1).jpg" width=600></img>

## The objectif
**There is a total of 2 mission with 2 rounds each** <br>
#### 0- General rules
The self driving car must drive around the map for 3 complete Laps. <br>
 The car is not allowed to touch or move any of the walls that are placed around the map. <br>
 The participant are not allowed to touch the car during a round.

#### 1- The Open chalenge
The Open challenge is where the car must complete three full laps around the field without touching a wall. The size of each side of the field is determined by judges randomly of either 100 cm or 60 cm. The direction that the car drives in is also completly randomized (clockwize or counterclockwize).

#### 2- The Obstacle Chalenge
In the obstacle challenge the car must complete three full laps around the field, without touching the different coloured traffic signs (Red or green pillars). If the pillar is red, the car should pass on its right side, and if the pillar is green, the car should pass on its left side. The direction that the car drives in is also completly randomized (clockwize or counterclockwize).
After the third lap, depending on the last pillar, the car must continue in the same direction or change directions to find the parking lot. The car must then back into the parking lot without touching either of the ends. The size of each side of the field remains constant, 1 metre for each side. 

## The Engineering Materials
- The car [base](https://www.amazon.com/Click-Play-Control-Crawler-Vehicle/dp/B01MD1SYFZ).<br>
<img src="Other\The car base.jpg">
<br>
<br>
<li> MG995 servo motor. <br>
<img src="Other\Mg995 servo.jpg" width=200>
<br>
<br>
<li>Raspberry pi 5 4GB. <br>
<img src="Other\raspberry-pi-54gb.webp" width=200>
<br>
<br>
<li>Ingco 12V 4A rechargeble drilling machine battery. <br>
<img src="Other\Ingco 20V battery.jpg" width=200>
<br>
<br>
<li>Step Down Converter <br>
<img src="Other\Stepdown conveter for the raspberrry.jpg" width=200>
<br>
<br>
<li>A 12V DC motor <br>
<img src="Other\12v DC motor.jpg" width=200>
<br>
<br>
<li>A L298N motor driver <br>
<img src="Other\L298N motor driver.jpeg" width=200>
<br>
<br>
<li>Positive and negative lines of a breadboard <br>
<img src="Other\Positive &Negative BB lines.jpg" width=200>
<br>
<br>
<li>Jumper cables (MM & MF & FF) <br>
<img src="Other\MM MF FF Wires.jpg" width=200>
<br>
<br>
<li>Rocker switch <br>
<img src="Other\Rocker switch.jpeg" width=200>
<br>
<br>
<li>Mini cooling fan for raspberry pi 5 <br>
<img src="Other\rsp cooler.jpeg" width=200>
<br>
<br>
<li>Generic Webcam FHD <br>
<img src="Other\Generic Webcam FHD.jpg" width=200>
<br>
<br>
<li> HC-SR04 Ultrasonic sensors <br>
<img src="Other\ultra sonic sensor.jpeg" width=200></img>

# Car Assembling
- The servo motor in placed horizontaly on the front of the robot dteering the front wheels to make the car steer to the direction it rotates to.
- The Webcam is place on the servo motor at the base level fixed with double tape.
- The Ingco bbattery  is place dehinf the servo motor fized with double tape also and the ON/OFF switch is glued on the front of it.
- The raspberry pi 5 is fized on top of the battery with its cooler on top of it.
- The step down converter is placed on the Left side of the left side of the battery.
- Behinf the battry, at the wheels llevel is placed a motor (note: we removed the car default DC motor and added a stronger one).
- On the motor box is placed the L298N motor driver, that controls the motor.
- At the very end of the base is place the positive & negative lines of the bread board part we used.
### Notes
- All the GPIO pins are connected perfectly to the raspberry pi 5.
- The electricity management is done with the positive line of the breadboard.
- The same is done with the GND.




