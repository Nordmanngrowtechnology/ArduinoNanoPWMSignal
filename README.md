# Arduino Nano: Eine 25kHz Frequenz für ein PWM Signal erzeugen

Mit dem Arduino Nano eine 25kHz Frequenz zu erzeugen ist kein Problem. Dieses Beispiel bezieht sich auf einen ATmega328P Microchip.

### Datenblatt Atmel ATmega328P
https://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-7810-Automotive-Microcontrollers-ATmega328P_Datasheet.pdf

# TIMER1 verändern, um eine 25kHz Frequenz zu erzeugen

Das PWM Signal sollte mit TIMER1 verändert werden da TIMER0 zum Beispiel im Systemprogramm gebraucht wird für die Funktion von delay() oder millsec(). Nach der änderung des Timers haben wir in den dazugehörigen PIN Blöcken die möglichkeit ein Duty Cycle zu nutzen.


# TIMER1 manipulieren beim Booten vom Arduino Nano

```

// definition pins
struct Pin
{
    uint8_t pwmPin;
};

constexpr Pin FAN_PWM_PIN_TOP_FAN{10};         // 10
constexpr Pin FAN_PWM_PIN_BOTTOM_FAN{9};       // 9



void setup()
{
 cli(); // disable interrupts

    /*
     *  FIND MORE INFORMATION HOW IT WORKS
     *
     *  DEFINITION TIMER1 FOR ATMEL ATmega328P
     *  https://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-7810-Automotive-Microcontrollers-ATmega328P_Datasheet.pdf
     *  https://www.mikrocontroller.net/articles/AVR-Tutorial:_Timer
     *
     *
     *  CHANGE TIMER1 SETTINGS FOR 25kHz SIGNAL CALCULATION
     */

    // RESET  Clear Timer/Counter control registers
    TCNT1 = 0;
    TCCR1A = 0;
    TCCR1B = 0;

    // Set TOP value for 25kHz frequency (ICR1)
    ICR1 = 320; // Set TOP limiter to 320 >>> update OCR1A at TOP

    // REGISTER DEFINITION
    TCCR1A |= 1 << WGM11; // PWM, Phase Correct
    TCCR1B |= 1 << WGM13; // The phase correct Pulse Width Modulation or phase
                          // correct PWM mode (WGM13:0 = 1
    TCCR1B |= 1 << CS10;  // clkI/O/1 (No prescaling)

    // Need COM1A1 and COM1B1 for tow pin output . Clear OC1A/OC1B on compare
    // match
    TCCR1A |= 1 << COM1A1 | 1 << COM1B1;
    // Clear OC1A/OC1B on Compare match when down counting set output to low
    // level

    // Set Arduino Nano pins to output with Data Direction Registers B (Arduino
    // Pin 9 is PB1), (Arduino Pin 10 is PB2) is a part of Port B
    DDRB |= 1 << 1 | 1 << 2; // set as OUTPUT

   
    sei(); // allow interrupts
    // timer configuration finish
    /**************************************************************************/
}
```

# Function analogWrite() überschreiben

```
/*++++++++++++++++++++++++ analogeWrite() overwrite +++++++++++++++++++++++++++++++++++++++
 *
 * Yes we overwrite the default function of the arduino framework
 * so we can use percent 0.100 for duty cycle and write it to the register OCR1x
 * in this software we use tow register A and B:
 *
 * So we had different duty cycle on the same frequency 25kHz
 *
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 */
void analogWrite(const Pin &pin, const uint8_t percent)
{
    if (pin.pwmPin == FAN_PWM_PIN_TOP_FAN.pwmPin)
    {
        OCR1A = map(percent, 0, 100, 0, ICR1);
    } // ICR1 320
    if (pin.pwmPin == FAN_PWM_PIN_BOTTOM_FAN.pwmPin)
    {
        OCR1B = map(percent, 0, 100, 0, ICR1);
    } // ICR1 320
}
```


# Beispiel wie man die neue Funktion in loop() nutzen kann

```
void loop()
{
// run with 25% duty cicle
analogWrite(FAN_PWM_PIN_TOP_FAN, 25);
}
```

