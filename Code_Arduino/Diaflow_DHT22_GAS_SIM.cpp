#include <Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <SoftwareSerial.h>

// MQ2 sensor and alarm setup
int mq2SensorPin = A0;    // Analog pin for MQ2 sensor connected to A0 of Arduino
int buzzerPin = 8;        // Pin connected to the buzzer
int ledPin = 10;           // Pin connected to the LED
int servoPin = 9;         // Pin connected to the servo
int gasSensorValue = 0;   // Variable to store the sensor value
int gasThreshold = 400;   // Threshold for gas concentration to trigger the alarm

Servo gasServo;

// DHT22 sensor and LCD setup
#define DHTPIN A1           // Pin for DHT22 sensor
#define DHTTYPE DHT22       // Type of DHT sensor (DHT22 or DHT11)
#define BUTTON_PIN 2        // Pin for button
#define FAN_PIN 3           // Pin for fan control relay
#define DEBOUNCE_DELAY 500  // Debounce delay in milliseconds

DHT dht(DHTPIN, DHTTYPE);

byte doC[] = {
  B00111,
  B00101,
  B00111,
  B00000,
  B00000,
  B00000,
  B00000,
  B00000
};

#define I2C_ADDR 0x27
LiquidCrystal_I2C lcd(I2C_ADDR, 16, 2);

bool isCelsius = true; // Variable to track temperature unit
unsigned long lastDebounceTime = 0; // Variable to store the last debounce time

// SIM900A setup
SoftwareSerial SIM900A(6, 7); // RX, TX
bool isCalling = false; // Biến để theo dõi trạng thái cuộc gọi

void setup() {
  // Setup for MQ2 and alarm
  Serial.begin(9600);           // Initialize Serial Monitor
  SIM900A.begin(9600);          // Initialize SIM900A Serial Communication
  pinMode(mq2SensorPin, INPUT); // Set mq2SensorPin as input
  pinMode(buzzerPin, OUTPUT);   // Set buzzerPin as output
  pinMode(ledPin, OUTPUT);      // Set ledPin as output
  gasServo.attach(servoPin);    // Attach servo to servoPin
  gasServo.write(0);            // Initialize servo to 0 degrees

  // Setup for DHT22, LCD, and fan control
  pinMode(BUTTON_PIN, INPUT_PULLUP); // Set button pin as input with pull-up resistor
  pinMode(FAN_PIN, OUTPUT);          // Set fan pin as output
  digitalWrite(FAN_PIN, LOW);        // Turn off fan initially
  lcd.init();
  lcd.backlight();
  dht.begin();
  
  lcd.setCursor(0, 0);
  lcd.print("Temp:");
  lcd.setCursor(0, 1);
  lcd.print("Hum:");
  lcd.createChar(0, doC);
}

void loop() {
  // MQ2 sensor reading and alarm control
  gasSensorValue = analogRead(mq2SensorPin); // Read value from MQ2 sensor

  if (gasSensorValue > gasThreshold) {
    digitalWrite(buzzerPin, HIGH); // Activate buzzer
    digitalWrite(ledPin, HIGH);    // Activate LED
    gasServo.write(100);           // Open door (servo to 100 degrees)
    makeCall();                    // Make a call using SIM900A
  } else {
    digitalWrite(buzzerPin, LOW);  // Deactivate buzzer
    digitalWrite(ledPin, LOW);     // Deactivate LED
    gasServo.write(0);             // Close door (servo to 0 degrees)
    isCalling = false;             // Đặt lại trạng thái cuộc gọi khi không còn nguy cơ
  
  }

  // DHT22 sensor reading and fan control
  bool currentButtonState = digitalRead(BUTTON_PIN); // Read the current button state
  unsigned long currentTime = millis(); // Get the current time
  bool lastButtonState = HIGH;
  
  if (currentButtonState != lastButtonState) {
    if (currentButtonState == LOW && (currentTime - lastDebounceTime) > DEBOUNCE_DELAY) {
      isCelsius = !isCelsius; // Toggle temperature unit
      lastDebounceTime = currentTime; // Update debounce time
    }
  }

  lastButtonState = currentButtonState; // Update last button state
  
  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();

  if (temperature > 36.5) {
    digitalWrite(FAN_PIN, HIGH); // Turn on fan
  } else {
    digitalWrite(FAN_PIN, LOW); // Turn off fan
  }
  
  displayTemperatureAndHumidity(temperature, humidity);
  handleSerialCommands();
}

void displayTemperatureAndHumidity(float temp, float hum) {
  if (isnan(hum) || isnan(temp)) {
    lcd.setCursor(0, 1);
    lcd.print("Sensor error");
    delay(2000);
    return;
  }

  float displayTemperature = temp;
  if (!isCelsius) {
    displayTemperature = temp * 9 / 5 + 32;
  }

  lcd.setCursor(6, 0);
  lcd.print("     ");
  lcd.setCursor(6, 0);
  lcd.print(displayTemperature);
  lcd.setCursor(11, 0);
  lcd.write(byte(0));
  lcd.setCursor(12, 0);
  lcd.print(isCelsius ? "C" : "F");

  lcd.setCursor(5, 1);
  lcd.print("     ");
  lcd.setCursor(5, 1);  
  lcd.print(hum);
  lcd.print(" %");
}

#define GET_GAS_COMMAND "GET_GAS"

void handleSerialCommands() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim(); // Xóa khoảng trắng ở đầu và cuối chuỗi

    if (command == "GET_TEMP_C") {
      float temperatureC = dht.readTemperature(false); 
      Serial.println(temperatureC);
    } else if (command == "GET_TEMP_F") {
      float temperatureF = dht.readTemperature(true); 
      Serial.println(temperatureF);
    } else if (command == "GET_HUM") {
      float humidity = dht.readHumidity();
      Serial.println(humidity);
    } else if (command == GET_GAS_COMMAND) {
      for (int i = 0; i < 10; i++) {
        gasSensorValue = analogRead(mq2SensorPin); // Read gas sensor value
        Serial.println(gasSensorValue);
        Serial.println(gasSensorValue); // Send gas sensor value to Raspberry Pi   
      }
    } else {
      Serial.println("Unknown command");
    }
  }
}

void makeCall() {
  if (!isCalling) {
    SIM900A.println("ATD0913660211;"); // Command to dial the number
    isCalling = true; // Đánh dấu đang trong cuộc gọi
  }
}
