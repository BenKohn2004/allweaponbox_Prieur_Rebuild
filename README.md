#Prieur Rebuild

A rebuild of a Prieur Foil/Epee Scorebox from the 1980s? The internal circuitry was replaced with an [Arduino Nano](https://github.com/BenKohn2004/allweaponbox_Prieur_Rebuild/blob/main/Schematic_Prieur_Sports_MP90_Arduino_Rebuild.pdf) and code based on [Wnew's Implimentation](https://github.com/wnew/fencing_scoring_box).

The lights were replaced with LEDs and the two left pushbuttons were altered to choose between Foil/Epee/Saber/Foil Classic. Where Foil Classic is foil with the pre-2005 timing. A RGB LED replaced the power light and now indicates power, but also which weapon is selected.

The original coding also displayed the lights after the lockout time elapsed. The code was altered to display the lights immediately after hitting instead of waiting for the lockout timing to expire. This is useful for refereeing foil so that the referee can more easily differentiate between a mal-parry amd remise. This may alter the accuracy of the timing, but it still "feels" correct and makes refereeing easier.

Foil/Epee work great, saber not as well.

<img src="Prieur Bottom of Circuit Board.jpg">

<img src="Prieur Top of Circuit Board.jpg">

<img src="Prieur Scoring Box.jpg">

<img src="prieur box thumbnail.jpg">


