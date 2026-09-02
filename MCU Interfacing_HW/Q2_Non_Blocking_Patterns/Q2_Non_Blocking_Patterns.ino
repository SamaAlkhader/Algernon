#include <avr/io.h>

// ----------------------------------------------------
// Bit masks
// ----------------------------------------------------

#define LED_A_MASK (1 << PB0)
#define LED_B_MASK (1 << PB1)
#define LED_C_MASK (1 << PB2)
#define LED_D_MASK (1 << PB3)
#define LED_E_MASK (1 << PB4)

#define BUTTON_MASK (1 << PD2)

#define ABC_MASK (LED_A_MASK | LED_B_MASK | LED_C_MASK)


// ----------------------------------------------------
// Timing constants
// ----------------------------------------------------

// LED A
const unsigned long A_ON_TIME  = 200;
const unsigned long A_OFF_TIME = 800;

// LED B
const unsigned long B_TOGGLE_TIME = 350;

// LED C
const unsigned long C_ON_TIME  = 150;
const unsigned long C_OFF_TIME = 650;

// Button
const unsigned long DEBOUNCE_TIME = 40;
const unsigned long LONG_PRESS_TIME = 2000;

// LED D safety flash
const unsigned long SAFETY_FLASH_TIME = 50;

// Serial status
const unsigned long SERIAL_TIME = 2500;


// ----------------------------------------------------
// LED states
// ----------------------------------------------------

bool ledAState = false;
bool ledBState = false;

// LED C must start ON
bool ledCState = true;

// Normal state of LED D.
// This is the state produced by normal button presses.
// During a long hold, the physical LED flashes temporarily.
bool ledDNormalState = false;


// ----------------------------------------------------
// LED timing variables
// ----------------------------------------------------

unsigned long previousATime = 0;
unsigned long previousBTime = 0;
unsigned long previousCTime = 0;

unsigned long previousSerialTime = 0;


// ----------------------------------------------------
// Button debounce variables
// ----------------------------------------------------

// HIGH = released because internal pull-up is used
// LOW  = pressed

bool lastRawButtonState = true;
bool stableButtonState = true;

unsigned long debounceStartTime = 0;


// ----------------------------------------------------
// Long-press variables
// ----------------------------------------------------

unsigned long pressStartTime = 0;
unsigned long holdDuration = 0;

bool safetyFlashActive = false;

unsigned long previousSafetyFlashTime = 0;


// ----------------------------------------------------
// Setup
// ----------------------------------------------------

void setup()
{
    // ------------------------------------------
    // Configure PB0-PB4 as outputs.
    //
    // |= is used so other DDRB bits are not
    // changed.
    // ------------------------------------------

    DDRB |= LED_A_MASK |
            LED_B_MASK |
            LED_C_MASK |
            LED_D_MASK |
            LED_E_MASK;


    // ------------------------------------------
    // Configure PD2 as input.
    //
    // &= with inverted mask clears only PD2.
    // ------------------------------------------

    DDRD &= ~BUTTON_MASK;


    // ------------------------------------------
    // Enable the internal pull-up resistor
    // on PD2.
    // ------------------------------------------

    PORTD |= BUTTON_MASK;


    // ------------------------------------------
    // Initial LED states
    // ------------------------------------------

    // LED A starts OFF
    PORTB &= ~LED_A_MASK;

    // LED B starts OFF
    PORTB &= ~LED_B_MASK;

    // LED C MUST start ON
    PORTB |= LED_C_MASK;

    // LED D starts OFF
    PORTB &= ~LED_D_MASK;

    // Bonus LED E starts OFF
    PORTB &= ~LED_E_MASK;


    // ------------------------------------------
    // Start serial communication
    // ------------------------------------------

    Serial.begin(9600);


    // ------------------------------------------
    // Get initial time
    // ------------------------------------------

    unsigned long currentTime = millis();

    previousATime = currentTime;
    previousBTime = currentTime;
    previousCTime = currentTime;
    previousSerialTime = currentTime;
}


// ----------------------------------------------------
// Main loop
// ----------------------------------------------------

void loop()
{
    unsigned long currentTime = millis();


    // ==================================================
    // TASK 1: LED A
    //
    // 200 ms ON
    // 800 ms OFF
    // ==================================================

    if (ledAState == true)
    {
        // LED A is currently ON

        if (currentTime - previousATime >= A_ON_TIME)
        {
            // Turn LED A OFF

            PORTB &= ~LED_A_MASK;

            ledAState = false;

            previousATime = currentTime;
        }
    }
    else
    {
        // LED A is currently OFF

        if (currentTime - previousATime >= A_OFF_TIME)
        {
            // Turn LED A ON

            PORTB |= LED_A_MASK;

            ledAState = true;

            previousATime = currentTime;
        }
    }


    // ==================================================
    // TASK 2: LED B
    //
    // Toggle every 350 ms
    //
    // ON  350 ms
    // OFF 350 ms
    // ==================================================

    if (currentTime - previousBTime >= B_TOGGLE_TIME)
    {
        // XOR toggles only PB1

        PORTB ^= LED_B_MASK;

        ledBState = !ledBState;

        previousBTime = currentTime;
    }


    // ==================================================
    // TASK 3: LED C
    //
    // Starts ON
    // ON  = 150 ms
    // OFF = 650 ms
    // ==================================================

    if (ledCState == true)
    {
        // LED C is currently ON

        if (currentTime - previousCTime >= C_ON_TIME)
        {
            // Turn LED C OFF

            PORTB &= ~LED_C_MASK;

            ledCState = false;

            previousCTime = currentTime;
        }
    }
    else
    {
        // LED C is currently OFF

        if (currentTime - previousCTime >= C_OFF_TIME)
        {
            // Turn LED C ON

            PORTB |= LED_C_MASK;

            ledCState = true;

            previousCTime = currentTime;
        }
    }


    // ==================================================
    // TASK 4: Read Button Directly From PIND
    // ==================================================

    bool rawButtonState;

    if (PIND & BUTTON_MASK)
    {
        rawButtonState = true;      // HIGH = released
    }
    else
    {
        rawButtonState = false;     // LOW = pressed
    }


    // ==================================================
    // TASK 5: Non-blocking debounce
    //
    // A raw change must remain unchanged for at
    // least 40 ms before it is accepted.
    // ==================================================

    if (rawButtonState != lastRawButtonState)
    {
        // Raw input changed.
        // Restart the debounce timer.

        debounceStartTime = currentTime;

        lastRawButtonState = rawButtonState;
    }


    // Has the raw signal stayed stable for 40 ms?

    if ((currentTime - debounceStartTime >= DEBOUNCE_TIME) &&
        (rawButtonState != stableButtonState))
    {
        // Accept the new button state

        stableButtonState = rawButtonState;


        // ----------------------------------------------
        // Button has become PRESSED
        // ----------------------------------------------

        if (stableButtonState == false)
        {
            // Save the starting time of this press

            pressStartTime = currentTime;

            holdDuration = 0;

            safetyFlashActive = false;


            // Toggle the NORMAL LED D state exactly
            // once for this press.

            ledDNormalState = !ledDNormalState;


            if (ledDNormalState == true)
            {
                PORTB |= LED_D_MASK;
            }
            else
            {
                PORTB &= ~LED_D_MASK;
            }
        }


        // ----------------------------------------------
        // Button has become RELEASED
        // ----------------------------------------------

        else
        {
            // Stop safety flashing

            safetyFlashActive = false;

            holdDuration = 0;


            // Restore LED D to the normal state that
            // resulted from the original button press.

            if (ledDNormalState == true)
            {
                PORTB |= LED_D_MASK;
            }
            else
            {
                PORTB &= ~LED_D_MASK;
            }
        }
    }


    // ==================================================
    // TASK 6: Measure hold duration
    // ==================================================

    if (stableButtonState == false)
    {
        // Button is currently held

        holdDuration = currentTime - pressStartTime;


        // ----------------------------------------------
        // Has button been held for more than 2000 ms?
        // ----------------------------------------------

        if ((holdDuration > LONG_PRESS_TIME) &&
            (safetyFlashActive == false))
        {
            // Enter safety flash mode

            safetyFlashActive = true;

            previousSafetyFlashTime = currentTime;

            // Immediately toggle LED D once when
            // safety mode begins.

            PORTB ^= LED_D_MASK;
        }
    }
    else
    {
        // Button is not held

        holdDuration = 0;
    }


    // ==================================================
    // TASK 7: LED D fast safety flash
    //
    // Toggle every 50 ms while long press remains active
    // ==================================================

    if (safetyFlashActive == true)
    {
        if (currentTime - previousSafetyFlashTime >= SAFETY_FLASH_TIME)
        {
            PORTB ^= LED_D_MASK;

            previousSafetyFlashTime = currentTime;
        }
    }


    // ==================================================
    // BONUS: LED E
    //
    // PB4 ON only when A, B and C are ALL HIGH.
    //
    // The condition examines A/B/C using ONE
    // bitwise masking operation on PORTB.
    // ==================================================

    if ((PORTB & ABC_MASK) == ABC_MASK)
    {
        PORTB |= LED_E_MASK;
    }
    else
    {
        PORTB &= ~LED_E_MASK;
    }


    // ==================================================
    // TASK 8: Serial status every 2.5 seconds
    // ==================================================

    if (currentTime - previousSerialTime >= SERIAL_TIME)
    {
        previousSerialTime = currentTime;


        Serial.print("A:");

        if (PORTB & LED_A_MASK)
        {
            Serial.print(1);
        }
        else
        {
            Serial.print(0);
        }


        Serial.print(" B:");

        if (PORTB & LED_B_MASK)
        {
            Serial.print(1);
        }
        else
        {
            Serial.print(0);
        }


        Serial.print(" C:");

        if (PORTB & LED_C_MASK)
        {
            Serial.print(1);
        }
        else
        {
            Serial.print(0);
        }


        Serial.print(" D:");

        if (PORTB & LED_D_MASK)
        {
            Serial.print(1);
        }
        else
        {
            Serial.print(0);
        }


        Serial.print(" Hold:");

        Serial.print(holdDuration);

        Serial.println("ms");
    }
}