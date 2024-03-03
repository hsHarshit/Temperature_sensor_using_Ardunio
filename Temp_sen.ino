// Include the necessary libraries
#include <avr/io.h>
#include <avr/interrupt.h>

// Define the pin for the LM35 temperature sensor
#define LM35_PIN A0

// Define the pins for the onboard LED
#define LED_PIN 13

// Define constants for temperature thresholds
#define TEMP_THRESHOLD_LOW 30
#define TEMP_THRESHOLD_HIGH 31

// Global variables
volatile int temperature = 0;
volatile unsigned long previousMillis = 0;
volatile unsigned long interval = 250; // Default interval 250 ms
volatile bool ledState = LOW;

// Function prototypes
void setupTimer();
void setup();
void loop();
void readTemperature();

// Timer1 overflow interrupt service routine
ISR(TIMER1_OVF_vect) {
  // Update the timer
  TCNT1 = 64911; // Reset Timer1 count to achieve 1ms interrupt
  // Update temperature reading
  readTemperature();
}

// Function to setup Timer1 for generating interrupts
void setupTimer() {
  // Set Timer1 in normal mode
  TCCR1A = 0;
  // Set prescaler to 64
  TCCR1B |= (1 << CS11) | (1 << CS10);
  // Enable Timer1 overflow interrupt
  TIMSK1 |= (1 << TOIE1);
  // Initialize Timer1 count to generate 1ms interrupt
  TCNT1 = 64911; // Initial count value for 1ms overflow with 16MHz clock
  // Enable interrupts
  sei();
}

void setup() {
  // Set LED pin as output
  pinMode(LED_PIN, OUTPUT);
  // Start the timer
  setupTimer();
}

void loop() {
  // Check temperature and control LED blinking
  if (temperature < TEMP_THRESHOLD_LOW) {
    interval = 250; // If temperature is below 30°C, blink every 250ms
  } else if (temperature >= TEMP_THRESHOLD_HIGH) {
    interval = 500; // If temperature is above or equal to 31°C, blink every 500ms
  }
  
  // Blink LED
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    // Save the last time the LED blinked
    previousMillis = currentMillis;
    // Toggle LED state
    ledState = !ledState;
    // Update LED
    digitalWrite(LED_PIN, ledState);
  }
}

// Function to read temperature from LM35 sensor
void readTemperature() {
  // Read the analog value from LM35 sensor
  int sensorValue = analogRead(LM35_PIN);
  // Convert the analog value to temperature in Celsius
  temperature = (sensorValue * 500) / 1024;
}
