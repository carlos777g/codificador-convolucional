//RECEPTOR

#include <SPI.h>
#include <RF24.h>

// CE en pin 7, CSN en pin 53 (recomendado en Arduino Mega)
RF24 radio(7, 53);

const byte identificacion[6] = "00001";

void setup() {
  Serial.begin(9600);  // Monitor Serial

  radio.begin();
  radio.openReadingPipe(0, identificacion);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();  // Modo receptor
}

void loop() {
  if (radio.available()) {
    char texto[32] = "";
    radio.read(&texto, sizeof(texto));
    Serial.println(texto);  // Mostrar mensaje recibido
  }
}
