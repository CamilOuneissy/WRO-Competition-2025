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
<img src="Other\WRO-2025_FutureEngineers_Playfield_page-0001.jpg"></img>

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
<img src="Other\Step Down Converter.webp" width=200>
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
<li>The Webcam was screwed on the front of the car at the base level to detect the upcomming obstacle. <li> The MG995 servo motor is placed on the front wheels allowing the  car to turn using ackerman steering to the direction the servo motor turns to. <li>The INGCO 20V battery is placed on the servo motor connected to the step down converter ti insure the perfect voltage for each extension added to our Raspberry pi 5. <li> For the raspberry pi 5, it's placed behind the servo motor on in middle of the car base covered by its 12V cooler. <li> The bread board positive and negative lane are place next the raspberry pi. <li> The Dc motor is connected to the rear  wheels using gears to ensure more torque to the car, and it's connected the the L298N motor driver. <li> The L298N motor driver is place on the back of the car at the truck level. <li> Two ultrasonic sensors are place on each left and right sdes of the car to ensure the car steering around the map.





