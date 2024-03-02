#include <LiquidCrystal_I2C.h>
#include <Wire.h>

// Código de Funcionamento para Sistema de Monitoramento e Irrigação com Arduino

#define pinoAnalog A0
#define pinoRele 8

byte ledState = LOW;

unsigned long previousMillis = 0;
unsigned long intervalOff = 7200000;  //desligado por 2h
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

  pinMode(pinoRele, OUTPUT);

  lcd.init();
  lcd.setBacklight(HIGH);

  lcd.createChar(0, waterDropChar);

  lcd.setCursor(5, 0);
  lcd.print("Prox.");
  lcd.setCursor(0, 1);
  lcd.print("Qtd Irr. ");

  previousMillis = millis() - intervalOff;
}

int readSensor() {
  int mesuresCount = 0;
  int percentage = 0;

  lcd.setCursor(0, 0);
  lcd.write((byte)0);
  lcd.print("..");
  lcd.print("%");

  while (mesuresCount < 5) {
    int valAnalogIn = analogRead(pinoAnalog);
    int currentValue = map(valAnalogIn, wet, dry, 100, 0);

    percentage = (currentValue + percentage) / 2;

    mesuresCount++;
    delay(500);
  }

  Serial.print(percentage);
  Serial.println("%");

  lcd.setCursor(0, 0);
  lcd.write((byte)0);
  lcd.print(percentage);
  lcd.print("%");

  return percentage;
}

void doIrrigacao() {
  int porcento = readSensor();

  while (porcento <= 45) {
    Serial.println("Irrigando a planta ...");
    digitalWrite(pinoRele, HIGH);

    delay(4000);

    digitalWrite(pinoRele, LOW);
    irrigationCounter++;
    porcento = readSensor();
  }
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

  if (lastUpdateDiff >= intervalOff) {
    doIrrigacao();
    Serial.println("Planta Irrigada ...");
    previousMillis = millis();
  }
  delay(2000);
}