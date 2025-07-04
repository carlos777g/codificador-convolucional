//tRANSMISOR
#include <SPI.h>
#include <RF24.h>

// CE en pin 7, CSN en pin 53 (recomendado en Mega usar 53 como CS)
RF24 radio(7, 53);

const byte identificacion[6] = "00001";

void setup() {
  Serial.begin(9600);  // Opcional para debug

  radio.begin();
  radio.openWritingPipe(identificacion);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();  // Modo transmisor
}

void loop() {
  const char texto[] = "";
  bool exito = radio.write(&texto, sizeof(texto));

  if (exito) {
    Serial.println("Mensaje enviado correctamente.");
  } else {
    Serial.println("Error al enviar.");
  }

  delay(1000);  // Espera 1 segundo
}
