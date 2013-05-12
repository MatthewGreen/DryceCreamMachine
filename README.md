#The Dryce Cream Machine
---
INFO 4320: Rapid Prototyping & Physical Computing (Spring 2013)
Professor: François Guimbretière
Students: Alan Jenson, Yu Li & Matthew Green
Due Date: May 11 2013

Final Project - The Dryce Cream Machine (An Autonomous Dry Ice Ice Cream Making Robot)

##Project Description
---
Our final project consisted of designing and building an autonomous dry ice ice cream making prototype robot. Our design consisted of an amalgamation of five different componets split into two units - the base and the canopy. The base unit had a motorized turntable that had a magnet attached to it to trigger reed switches on the flat base. The canopy unit was the crux of the design as it consisted of the final four components which were a liquid dispenser, a dry ice dispenser, a whisking/stirring mechanism and finally a topping dispenser. The 15" high canopy was made out of wood just like the base and was formed by drilling holes into it as well as attaching parts to it in orer to integrate everything. The liquid dispenser sytem used an air pump to push air into two airtight 64oz containers with vanilla and chocolate ice cream base. The liquid was then forced up through a food safe silicone tube and crimped shut using servo motors and a 3D printed component that acted as a holster and valve handle. When the servos were set to 180 degrees the liquid would dispense and when at 0 the liquid would become suspended in the tube. The next component, the dry ice dispenser, consisted of a flat "puck" with six holes that would hold dry ice that we loaded manually. When a cup was in place the stepper motor and gear mechanism would turn from beneath the puck and position a "slot" over a hole in the canopy, the ice would then fall into the cup. The whisk/stirring mechanism follows the dry ice dispenser. This system was the most complex as it involved both lowering a shaft and actuating a DC motor whisk (a Norpro handheld mixer). The shaft was made from clear acrylic in a box type pattern and this held the motor inside of it. When ready to stir the shaft would we lowered using a stepper motor and gear system and then the DC motor which was hardwired would then be turned on, after 10seconds of mixing the shaft would retract. The final mechanism is the topping dispenser. This system is very much akin to the dry ice dispenser but instead of six holes we use one hole in the puck. This puck is also encapsulated by a true hopper mechanism whereby a container with sprinkles is placed in a holster above the puck. When toppings needed to be dispensed the stepper motor would move the puck underneath the container's mouth to collect the topping and then move towards the hole in the canopy to deliver the topping to the waiting cup.

Overall this system was very involved as it had a lot of free standing parts that needed to work together to make the final product - dry ice ice cream.

##GUI Link
---
https://github.com/MatthewGreen/DryceCreamMachine_GUI