// Código de Funcionamento para Sistema de Monitoramento e Irrigação com Arduino

#include <LiquidCrystal_I2C.h>
#include <Wire.h>

#define pinoAnalog A0
#define pinoRele 8

#define MIN_HUMIDITY_PERCENTAGE 48

byte ledState = LOW;

unsigned long previousMillis = 0;
unsigned long intervalOff = 60000;  //minute; //1800000;  //30m
int intervalOffInMinutes = (int)((intervalOff / 1000) / 60);

const int dry = 518;  // value for dry sensor
const int wet = 215;  // value for wet sensor

unsigned long irrigationCounter = 0;

LiquidCrystal_I2C lcd(0x27, 16, 2);  // Criando um LCD de 16x2 no endereço 0x27

byte waterDropChar[8] = {
  0b00100,
  0b01110,
  0b01110,
  0b11111,
  0b11111,
  0b11111,
  0b01110,
  0b00000
};

void setup() {
  Serial.begin(9600);
  // bluetooth.begin(9600);
  Serial1.begin(9600);

  pinMode(pinoRele, OUTPUT);

  lcd.init();
  lcd.setBacklight(HIGH);

  lcd.createChar(0, waterDropChar);

  lcd.setCursor(5, 0);
  lcd.print("Prox.");
  lcd.setCursor(0, 1);
  lcd.print("Qtd Irr. ");

  previousMillis = millis() - intervalOff;

  Serial.println("System is running...");
}

int readSensor() {
  int percentage = 0;
  int valAnalogIn = analogRead(pinoAnalog);

  lcd.setCursor(0, 0);
  lcd.write((byte)0);
  lcd.print("..");
  lcd.print("%");

  percentage = map(valAnalogIn, wet, dry, 100, 0);

  Serial.print(percentage);
  Serial.println("%");

  lcd.setCursor(0, 0);
  lcd.write((byte)0);
  lcd.print(percentage);
  lcd.print("%");

  return percentage;
}

void doIrrigacao() {
  Serial.println("Irrigando a planta ...");
  digitalWrite(pinoRele, HIGH);

  delay(12000);

  digitalWrite(pinoRele, LOW);
  irrigationCounter++;
}

void loop() {
  unsigned long currentMillis = millis();
  unsigned long lastUpdateDiff = currentMillis - previousMillis;

  int lastUpdateDiffInMinutes = (int)((lastUpdateDiff / 1000) / 60);
  int nextSoilAnalysis = intervalOffInMinutes - lastUpdateDiffInMinutes;

  lcd.setCursor(10, 0);
  lcd.print(nextSoilAnalysis);

  if (nextSoilAnalysis < 100) {
    lcd.print("m ");
  } else {
    lcd.print("m");
  }

  lcd.setCursor(9, 1);
  lcd.print(irrigationCounter);

  if (Serial1.available()) {
    int dadoBluetooth = Serial1.read();

    if (dadoBluetooth == '1') {
      doIrrigacao();
    }
    if (dadoBluetooth == '0') {
      Serial1.println(readSensor());
    }
  }

  if (lastUpdateDiff >= intervalOff) {
    int humidityPercentage = readSensor();
    if (humidityPercentage < MIN_HUMIDITY_PERCENTAGE) {
      doIrrigacao();
    }

    previousMillis = millis();
  }
  
  delay(200);
}