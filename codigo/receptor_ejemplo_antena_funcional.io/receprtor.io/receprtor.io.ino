// RECEPTOR RF24 - Decodificador Viterbi (n=2, k=1, K=4)
// g1 = 1111 (binario) = 15 (decimal)
// g2 = 1011 (binario) = 11 (decimal)
// YA BIEN
#include <SPI.h>
#include <RF24.h>

// Configuración del módulo RF24
RF24 radio(7, 53);       // CE, CSN
const byte direccion[6] = "00001";

// Pines de indicadores
#define LED_ERROR_PIN 13
#define LED_OK_PIN     12
#define BUZZER_PIN     11

// Parámetros del código convolucional
#define CONSTRAINT_LENGTH 4
#define NUM_STATES       (1 << (CONSTRAINT_LENGTH-1))  // = 8 estados
#define G1 0b1111
#define G2 0b1011
#define MAX_PATH_METRIC 9999

// Cada paquete trae 22 bits codificados (11 pares)
#define SYMBOLS_PER_PACKET 11
#define BITS_PER_SYMBOL    2
#define TOTAL_CODED_BITS   (SYMBOLS_PER_PACKET * BITS_PER_SYMBOL)

// Estructura para el paquete de datos RF24 (coincide con TX)
struct DataPacket {
  uint32_t packetId;
  uint8_t  sensorData;                // 8 bits originales
  uint16_t rawDistance;               // Distancia real en cm
  uint16_t checksum;
  uint8_t  encodedBits[TOTAL_CODED_BITS]; // 22 bits codificados
  uint8_t  padding[5];                // Relleno hasta 32 bytes
};

// Métricas y backpointers para cada etapa del trellis
struct ViterbiState {
  uint16_t pathMetric;
  uint8_t  prevState;
  uint8_t  decodedBit;
};

// Estadísticas de recepción
struct Statistics {
  uint32_t packetsReceived;
  uint32_t packetsDetected;
  uint32_t packetsCorrected;
  uint32_t bitsReceived;
  uint32_t bitsErrorDetected;
  uint32_t bitsErrorCorrected;
} stats = {0};

// Tablas de transición y salida precalculadas
uint8_t nextState[NUM_STATES][2];
uint8_t outputBits[NUM_STATES][2];

// Trellis: índices 0..SYMBOLS_PER_PACKET
ViterbiState trellis[SYMBOLS_PER_PACKET+1][NUM_STATES];

void setup() {
  Serial.begin(9600);

  // RF24
  radio.begin();
  radio.setAutoAck(false);
  radio.setRetries(0, 0);
  radio.setCRCLength(RF24_CRC_DISABLED);
  radio.setPayloadSize(sizeof(DataPacket));
  radio.openReadingPipe(0, direccion);
  radio.startListening();

  // Pines
  pinMode(LED_ERROR_PIN, OUTPUT);
  pinMode(LED_OK_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  Serial.println("=== RECEPTOR RF24 VITERBI INICIADO ===");
  Serial.println("Esperando 22 bits codificados por trama...");
  Serial.println("========================================");

  initializeTrellisTable();
  initializeTrellis();
}

void loop() {
  if (radio.available()) {
    DataPacket packet;
    radio.read(&packet, sizeof(packet));
    processReceivedPacket(packet);
  }
  // Estadísticas cada 30 s
  static unsigned long lastStats = 0;
  if (millis() - lastStats > 30000) {
    printStatistics();
    lastStats = millis();
  }
}

void processReceivedPacket(const DataPacket& packet) {
  stats.packetsReceived++;

  // Paquete válido si packetId cambió
  if (packet.packetId == 0 && stats.packetsReceived > 1) return;

  stats.packetsDetected++;
  stats.bitsReceived += 8;  // contamos bits de datos originales

  // Copiar bits codificados
  uint8_t receivedBits[TOTAL_CODED_BITS];
  memcpy(receivedBits, packet.encodedBits, TOTAL_CODED_BITS);

  // Simular ruido del canal (opcional)
  simulateChannelNoise(receivedBits, TOTAL_CODED_BITS);

  // Decodificar Viterbi
  uint8_t decodedByte = viterbiDecode(receivedBits);

  bool correctionMade = (decodedByte != packet.sensorData);
  if (correctionMade) stats.packetsCorrected++;

  // Verificar checksum
  uint16_t calcChk = (packet.packetId + decodedByte) & 0xFFFF;
  bool chkValid = (calcChk == packet.checksum);

  // Mostrar resultados
  Serial.println("---- PAQUETE RECIBIDO ----");
  Serial.print("ID: ");             Serial.println(packet.packetId);
  Serial.print("Dist real: ");      Serial.print(packet.rawDistance);
  Serial.println(" cm");
  Serial.print("Orig bits: ");      Serial.print(packet.sensorData, BIN);
  Serial.print(" ("); Serial.print(packet.sensorData); Serial.println(")");
  //Serial.print("Decodificado: ");   Serial.print(decodedByte, BIN);
  Serial.print(" ("); Serial.print(decodedByte);   Serial.println(")");
  Serial.print("Corrección: ");     Serial.println(correctionMade ? "SÍ" : "NO");
  Serial.print("Checksum: ");       Serial.println(chkValid ? "VÁLIDO" : "INVÁLIDO");

  Serial.print("Bits recv: ");
  for (int i = 0; i < TOTAL_CODED_BITS; i++) {
    Serial.print(receivedBits[i]);
    if ((i + 1) % 2 == 0) Serial.print(' ');
  }
  Serial.println("\n-------------------------");

  // Indicadores
  if (chkValid) {
    digitalWrite(LED_OK_PIN, HIGH);
    if (correctionMade) tone(BUZZER_PIN, 1000, 200);
    delay(300);
    digitalWrite(LED_OK_PIN, LOW);
  } else {
    digitalWrite(LED_ERROR_PIN, HIGH);
    tone(BUZZER_PIN, 300, 500);
    delay(500);
    digitalWrite(LED_ERROR_PIN, LOW);
  }
}

// Precalcula nextState y outputBits
void initializeTrellisTable() {
  for (int st = 0; st < NUM_STATES; st++) {
    for (int input = 0; input < 2; input++) {
      nextState[st][input] = ((st << 1) | input) & (NUM_STATES - 1);
      uint8_t reg = ((st << 1) | input) & ((1 << CONSTRAINT_LENGTH) - 1);
      uint8_t o1  = parityCheck(reg & G1);
      uint8_t o2  = parityCheck(reg & G2);
      outputBits[st][input] = (o1 << 1) | o2;
    }
  }
}

// Inicializa todas las métricas
void initializeTrellis() {
  for (int t = 0; t <= SYMBOLS_PER_PACKET; t++)
    for (int st = 0; st < NUM_STATES; st++)
      trellis[t][st] = { MAX_PATH_METRIC, 0, 0 };
  trellis[0][0].pathMetric = 0;  // estado inicial
}

// Viterbi (22 bits → 11 símbolos → extraer 8 datos)
uint8_t viterbiDecode(const uint8_t* recvBits) {
  initializeTrellis();

  // Forward pass
  for (int t = 0; t < SYMBOLS_PER_PACKET; t++) {
    uint8_t pair = (recvBits[2*t] << 1) | recvBits[2*t + 1];
    for (int st = 0; st < NUM_STATES; st++) {
      uint16_t pm = trellis[t][st].pathMetric;
      if (pm == MAX_PATH_METRIC) continue;
      for (int input = 0; input < 2; input++) {
        int ns       = nextState[st][input];
        uint8_t expo = outputBits[st][input];
        uint8_t hd   = __builtin_popcount(pair ^ expo);
        uint16_t newM = pm + hd;
        if (newM < trellis[t+1][ns].pathMetric) {
          trellis[t+1][ns] = { newM, (uint8_t)st, (uint8_t)input };
        }
      }
    }
  }

  // Selección de estado final
  int bestSt = 0;
  uint16_t bestM = trellis[SYMBOLS_PER_PACKET][0].pathMetric;
  for (int st = 1; st < NUM_STATES; st++) {
    if (trellis[SYMBOLS_PER_PACKET][st].pathMetric < bestM) {
      bestM = trellis[SYMBOLS_PER_PACKET][st].pathMetric;
      bestSt = st;
    }
  }

  // Backward pass: extraer solo 8 bits de datos (saltando 3 flush)
  uint8_t decoded = 0;
  int curSt = bestSt;
  for (int t = SYMBOLS_PER_PACKET - 1; t >= (CONSTRAINT_LENGTH - 1); t--) {
    uint8_t bit = trellis[t+1][curSt].decodedBit;
    decoded = (decoded << 1) | bit;
    curSt    = trellis[t+1][curSt].prevState;
  }

  // Estadísticas de error
  if (bestM > 0) {
    stats.bitsErrorDetected  += bestM;
    stats.bitsErrorCorrected += bestM;
  }

  return decoded;
}

// Simula ruido de canal (opcional)
void simulateChannelNoise(uint8_t* bits, int len) {
  float ber = 0.01;  // tasa de error de bit ejemplo
  for (int i = 0; i < len; i++) {
    if (random(10000) < ber * 10000) bits[i] ^= 1;
  }
}

// Paridad (XOR de bits)
uint8_t parityCheck(uint8_t v) {
  uint8_t p = 0;
  while (v) { p ^= (v & 1); v >>= 1; }
  return p;
}

// Estadísticas periódicas
void printStatistics() {
  Serial.println("\n===== ESTADÍSTICAS =====");
  Serial.print("Recibidos: ");  Serial.println(stats.packetsReceived);
  Serial.print("Detectados: "); Serial.println(stats.packetsDetected);
  Serial.print("Corregidos: "); Serial.println(stats.packetsCorrected);
  if (stats.packetsDetected) {
    float per = (float)stats.packetsCorrected / stats.packetsDetected * 100;
    Serial.print("PER: "); Serial.print(per); Serial.println("%");
  }
  if (stats.bitsReceived) {
    float ber_before = (float)stats.bitsErrorDetected  / stats.bitsReceived;
    float ber_after  = (float)(stats.bitsErrorDetected - stats.bitsErrorCorrected) / stats.bitsReceived;
    Serial.print("BER antes: "); Serial.println(ber_before, 5);
    Serial.print("BER después: ");Serial.println(ber_after, 5);
  }
  Serial.println("========================\n");
}
