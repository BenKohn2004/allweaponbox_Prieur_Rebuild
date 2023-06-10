#Prieur Rebuild

A rebuild of a Prieur Foil/Epee Scorebox from the 1980s? The internal circuitry was replaced with an Arduino Nano and code based on [Wnew's Implimentation](https://github.com/wnew/fencing_scoring_box).

The lights were replaced with LEDs and the two left pushbuttons were altered to choose between Foil/Epee/Saber/Foil Classic. Where foil classic is foil with the pre-2005 timing.

The original coding also displayed the lights after the lockout time elapsed. The code was altered to display the lights immediately after hitting instead of waiting the lockout timing to expire. This is useful for refereeing foil so that the referee can more easily differentiate between a mal-parry amd remise. This may alter the accuracy of the timing, but it still "feels" correct and makes refereeing easier.

Foil/Epee work great, saber is does not work as well.

<img src="Prieur Bottom of Circuit Board.jpg">

