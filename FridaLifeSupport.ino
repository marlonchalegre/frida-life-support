#include <LiquidCrystal_I2C.h>
#include <Wire.h>

// Código de Funcionamento para Sistema de Monitoramento e Irrigação com Arduino

#define pinoAnalog A0  // Define o pino A0 como "pinoAnalog"
#define pinoRele 8     // Define o pino 8 como "pinoRele"
#define pinoSensor 7   // Define o pino 7 como "pinoSensor"
#define pinoDisplay 9  // Define o pino 7 como "pinoSensor"

int ValAnalogIn;  // Introduz o valor analógico ao código

byte ledState = LOW;

unsigned long previousMillis = 0;
unsigned long intervalOff = 7200000;  //21600000;  //desligado por 6h

LiquidCrystal_I2C lcd(0x27, 16, 2);  // Criando um LCD de 16x2 no endereço 0x20


int readSensor() {
  digitalWrite(pinoSensor, HIGH);

  ValAnalogIn = analogRead(pinoAnalog);              // Relaciona o valor analógico com o recebido do sensor
  int porcento = map(ValAnalogIn, 1023, 0, 0, 100);  // Relaciona o valor analógico à porcentagem

  Serial.print(porcento);  // Imprime o valor em Porcento no monitor Serial
  Serial.println("%");     // Imprime o símbolo junto ao valor encontrado

  lcd.setCursor(0, 0);
  lcd.print("Umidade ");
  lcd.print(porcento);
  lcd.print("%");

  digitalWrite(pinoSensor, LOW);

  return porcento;
}


void setup() {
  Serial.begin(9600);  // Declara o BaundRate em 9600

  pinMode(pinoRele, OUTPUT);        // Declara o pinoRele como Saída
  pinMode(pinoSensor, OUTPUT);      // Declara o pinoSensor como Saída
  digitalWrite(pinoSensor, HIGH);   // Põem o pinoSensor em estado Alto = 5V
  digitalWrite(pinoDisplay, HIGH);  // Põem o pinoSensor em estado Alto = 5V //not working

  lcd.init();  // Inicializando o LCD

  doIrrigacao(); //Inicia testando a umidade
}

void doIrrigacao() {
  int porcento = readSensor();

  while (porcento <= 50) {
    Serial.println("Irrigando a planta ...");
    lcd.setCursor(0, 1);
    lcd.print("Irrigando...");
    digitalWrite(pinoRele, HIGH);

    delay(2000);
    porcento = readSensor();
  }
}

void loop() {
  unsigned long currentMillis = millis();
  lcd.setBacklight(LOW);

  unsigned long lastUpdateDiff = currentMillis - previousMillis;
  float lastUpdateDiffInMinutes = (lastUpdateDiff / 1000) / 60;

  lcd.setCursor(0, 1);
  lcd.print(lastUpdateDiffInMinutes);
  lcd.print(" m atras");

  if (lastUpdateDiff >= intervalOff) {
    digitalWrite(pinoSensor, HIGH);

    doIrrigacao();

    Serial.println("Planta Irrigada ...");
    digitalWrite(pinoRele, LOW);
    digitalWrite(pinoSensor, LOW);

    previousMillis = millis();
  }
}