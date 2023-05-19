# FreeRTOS
Small project in FreeRTOS targeting the ST Bluepill microcontroller 

Project developed for Embedded Software Programming class. To the microcontroler's GPIO four LEDs and three push-button are attached. The program has four different states:

1. All the LEDs are off
2. The LEDs blink on after the other, going forom left to right
3. The Leds blink one after the other, going from right to left
4. All the LEDs blink at the same time

Each button is attached to an interrupt. The first interrupt triggers state 2, the second interrupt triggers state 3, the third interruption triggers state 4 and a 5 seconds timer. When the timer reaches 0, if the button is still pressed, it will trigger state 1.
