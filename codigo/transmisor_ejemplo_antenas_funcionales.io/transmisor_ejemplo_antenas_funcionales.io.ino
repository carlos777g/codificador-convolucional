// TRANSMISOR RF24 - Codificador Convolucional (n=2, k=1, K=4)
// g1 = 1111 (binario) = 15 (decimal)
// g2 = 1011 (binario) = 11 (decimal)

#include <SPI.h>
#include <RF24.h>

// Configuración del módulo RF24
RF24 radio(7, 53); // CE, CSN
const byte direccion[6] = "00001";

// Pines del sensor HC-SR04
#define TRIGGER_PIN 4
#define ECHO_PIN 5
#define LED_STATUS_PIN 13

// Parámetros del código convolucional
#define CONSTRAINT_LENGTH 4
#define G1 0b1111  // Polinomio g1 = 1111
#define G2 0b1011  // Polinomio g2 = 1011

// Variables globales
uint8_t shiftRegister = 0; // Registro de desplazamiento de 4 bits
uint32_t packetsSent = 0;
uint32_t lastDistanceReading = 0;

// Estructura para el paquete de datos RF24
struct DataPacket {
  uint32_t packetId;
  uint8_t sensorData;     // Distancia mapeada a 8 bits
  uint16_t rawDistance;   // Distancia real en cm
  uint16_t checksum;
  uint8_t encodedBits[22]; // 8 bits datos * 2 + 3 bits terminación * 2 = 22 bits
  uint8_t padding[5];      // Relleno para completar 32 bytes
};

void setup() {
  Serial.begin(9600);
  
  // Configurar RF24
  radio.begin();
  radio.setAutoAck(false);
  radio.setRetries(0, 0);
  radio.setCRCLength(RF24_CRC_DISABLED);
  radio.setPayloadSize(sizeof(DataPacket));
  radio.openWritingPipe(direccion);
  radio.stopListening();
  
  // Configurar pines
  pinMode(LED_STATUS_PIN, OUTPUT);
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  Serial.println("=== TRANSMISOR RF24 CONVOLUCIONAL INICIADO ===");
  Serial.println("Codigo: n=2, k=1, K=4");
  Serial.println("g1=1111, g2=1011");
  Serial.println("=====================================");
  
  resetEncoder();
  calibrateUltrasonicSensor();
}

void loop() {
  uint16_t distance = readUltrasonicDistance();
  uint8_t sensorData = map(constrain(distance, 0, 400), 0, 400, 0, 255);
  
  if (abs(distance - lastDistanceReading) > 5 || packetsSent % 10 == 0) {
    DataPacket packet;
    memset(&packet, 0, sizeof(packet));
    
    packet.packetId   = packetsSent++;
    packet.sensorData = sensorData;
    packet.rawDistance = distance;
    packet.checksum   = calculateChecksum(packet.packetId, packet.sensorData);
    
    convolutionalEncode(sensorData, packet.encodedBits);
    
    bool result = radio.write(&packet, sizeof(packet));
    
    // Debug por serial
    Serial.print("Paquete #"); Serial.print(packet.packetId);
    Serial.print(" - Distancia: "); Serial.print(distance);
    Serial.print("cm -> Datos: "); Serial.print(sensorData, BIN);
    Serial.print(" ("); Serial.print(sensorData); Serial.print(")");
    Serial.print(" RF24 Status: "); Serial.println(result ? "OK" : "FAIL");
    
    Serial.print("Bits originales: ");
    for (int b = 7; b >= 0; b--) Serial.print((sensorData >> b) & 1);
    Serial.println();
    
    printEncodedBits(packet.encodedBits, 22);
    
    lastDistanceReading = distance;
    
    digitalWrite(LED_STATUS_PIN, HIGH);
    delay(100);
    digitalWrite(LED_STATUS_PIN, LOW);
  }
  delay(1000);
}


// Reiniciar el codificador
void resetEncoder() {
  shiftRegister = 0;
}

// Codificar un solo bit, insertando el nuevo en MSB y desplazando a la derecha
void encodeOneBit(uint8_t inputBit, uint8_t* output1, uint8_t* output2) {
  shiftRegister = ((inputBit << (CONSTRAINT_LENGTH - 1)) | (shiftRegister >> 1)) & 0x0F;
  *output1 = parityCheck(shiftRegister & G1);
  *output2 = parityCheck(shiftRegister & G2);
}

// Codificar un byte completo + flush
void convolutionalEncode(uint8_t inputByte, uint8_t* outputBits) {
  int bitIndex = 0;
  // Procesar de MSB a LSB
  for (int i = 7; i >= 0; i--) {
    uint8_t b = (inputByte >> i) & 1;
    uint8_t o1, o2;
    encodeOneBit(b, &o1, &o2);
    outputBits[bitIndex++] = o1;
    outputBits[bitIndex++] = o2;
  }
  // Flush: meter K-1 ceros
  for (int i = 0; i < CONSTRAINT_LENGTH - 1; i++) {
    uint8_t o1, o2;
    encodeOneBit(0, &o1, &o2);
    outputBits[bitIndex++] = o1;
    outputBits[bitIndex++] = o2;
  }
}

// Paridad (XOR de todos los bits)
uint8_t parityCheck(uint8_t value) {
  uint8_t p = 0;
  while (value) {
    p ^= (value & 1);
    value >>= 1;
  }
  return p;
}

// Checksum simple
uint16_t calculateChecksum(uint32_t id, uint8_t data) {
  return (id + data) & 0xFFFF;
}

// Imprimir bits codificados
void printEncodedBits(uint8_t* bits, int len) {
  Serial.print("Codificado: ");
  for (int i = 0; i < len; i++) {
    Serial.print(bits[i]);
    if ((i+1)%2 == 0) Serial.print(' ');
  }
  Serial.print("  Total bits: "); Serial.print(len);
  Serial.print(" ("); Serial.print(len/2); Serial.println(" símbolos)");
}

// Lectura HC-SR04
uint16_t readUltrasonicDistance() {
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);
  unsigned long dur = pulseIn(ECHO_PIN, HIGH, 30000);
  uint16_t dist = (dur * 0.034) / 2;
  return (dur==0 || dist>400) ? 400 : dist;
}

// Calibración HC-SR04
void calibrateUltrasonicSensor() {
  Serial.print("Calibrando sensor ");
  unsigned long start = millis();
  while (millis() - start < 5000) {
    readUltrasonicDistance();
    delay(100);
  }
  Serial.println(" OK");
}
