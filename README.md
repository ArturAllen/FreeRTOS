# FreeRTOS
Small project in FreeRTOS targeting the ST Bluepill microcontroller 

Project developed for Embedded Software Programming class. To the microcontroler's GPIO four LEDs and one push-button are attached. The program has four different states:

1. All the LEDs are off
2. The LEDs blink on after the other, going forom left to right
3. The Leds blink one after the other, going from right to left
4. All the LEDs blink at the same time

The change in state is triggered by puthing the button, which is attached to an interrupt. The states change in a cyclic manner.
