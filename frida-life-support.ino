#include <LiquidCrystal_I2C.h>
#include <Wire.h>

// Código de Funcionamento para Sistema de Monitoramento e Irrigação com Arduino

#define pinoAnalog A0
#define pinoRele 8
#define pinoSensor 7

int ValAnalogIn;

byte ledState = LOW;

unsigned long previousMillis = 0;
unsigned long intervalOff = 7200000;  //desligado por 2h
int intervalOffInMinutes = (int)((intervalOff / 1000) / 60);

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


int readSensor() {
  digitalWrite(pinoSensor, HIGH);

  ValAnalogIn = analogRead(pinoAnalog);
  int porcento = map(ValAnalogIn, 1023, 0, 0, 100);

  Serial.print(porcento);
  Serial.println("%");

  lcd.setCursor(0, 0);
  lcd.print(porcento);
  lcd.print("% ");
  lcd.write((byte)0);

  digitalWrite(pinoSensor, LOW);

  return porcento;
}


void setup() {
  Serial.begin(9600);

  pinMode(pinoRele, OUTPUT);
  pinMode(pinoSensor, OUTPUT);

  lcd.init();
  lcd.createChar(0, waterDropChar);
  
  doIrrigacao();

  lcd.setBacklight(HIGH);
}

void doIrrigacao() {
  int porcento = readSensor();

  while (porcento <= 45) {
    Serial.println("Irrigando a planta ...");
    lcd.setCursor(0, 1);
    lcd.print("Irrigando...");
    digitalWrite(pinoRele, HIGH);

    delay(2500);
    irrigationCounter++;
    porcento = readSensor();
  }

  digitalWrite(pinoRele, LOW);
}

void loop() {
  unsigned long currentMillis = millis();

  unsigned long lastUpdateDiff = currentMillis - previousMillis;
  int lastUpdateDiffInMinutes = (int)((lastUpdateDiff / 1000) / 60);
  int nextSoilAnalysis = intervalOffInMinutes - lastUpdateDiffInMinutes;

  lcd.setCursor(6, 0);
  lcd.print("Prx. ");
  lcd.print(nextSoilAnalysis);
  lcd.print("m");

  lcd.setCursor(0, 1);
  lcd.print("Qtd Irr. ");
  lcd.print(irrigationCounter);

  if (lastUpdateDiff >= intervalOff) {
    doIrrigacao();
    Serial.println("Planta Irrigada ...");
    previousMillis = millis();
  }
}