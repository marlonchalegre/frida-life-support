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
  int mesuresCount = 0;
  int percentage = 0;

  digitalWrite(pinoSensor, HIGH);

  lcd.setCursor(0, 0);
  lcd.write((byte)0);
  lcd.print("..");
  lcd.print("%");

  while (mesuresCount < 30) {
    ValAnalogIn = analogRead(pinoAnalog);
    int currentValue = map(ValAnalogIn, 1023, 0, 0, 100);

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

  digitalWrite(pinoSensor, LOW);

  return percentage;
}


void setup() {
  Serial.begin(9600);

  pinMode(pinoRele, OUTPUT);
  pinMode(pinoSensor, OUTPUT);

  lcd.init();
  lcd.setBacklight(HIGH);

  lcd.createChar(0, waterDropChar);

  doIrrigacao();

  lcd.setCursor(5, 0);
  lcd.print("Prox.");
  lcd.setCursor(0, 1);
  lcd.print("Qtd Irr. ");
}

void doIrrigacao() {
  int porcento = readSensor();

  while (porcento <= 47) {
    Serial.println("Irrigando a planta ...");
    digitalWrite(pinoRele, HIGH);

    delay(2500);
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