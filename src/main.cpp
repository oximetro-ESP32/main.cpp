
#include <Arduino.h>
#include <Wire.h>
#include <MAX30105.h>
#include <spo2_algorithm.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// OLED
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET -1
#define OLED_ADDRESS 0x3C

// I2C ESP32/////////
#define SDA_PIN 21//este pin define el pin SDA para la comunicacion I2C con el sensor MAX30102, permitiendo la transferencia de datos entre el microcontrolador y el sensor de manera eficiente y confiable
#define SCL_PIN 22//este pin define el pin SCL para la comunicacion I2C con el sensor MAX30102, permitiendo la sincronizacion de la transferencia de datos entre el microcontrolador y el sensor, asegurando una comunicacion precisa y estable
// Sensor
#define FINGER_THRESHOLD 5000 //esta constante define el umbral de deteccion del dedo, permitiendo determinar si el dedo esta colocado sobre el sensor o no, asegurando que las mediciones se realicen solo cuando el dedo este presente
#define BUFFER_SIZE 100///esta constante define el tamaño del buffer para almacenar las muestras de la senal infrarroja y roja, permitiendo un analisis adecuado de los datos y garantizando que se cuente con suficientes mediciones para calcular BPM y SpO2 de manera precisa
#define STEP_SIZE 25///esta constante define el tamaño del paso para desplazar las muestras en el buffer, permitiendo actualizar los datos de manera eficiente y mantener un flujo continuo de mediciones sin perder información relevante
////// Configuracion de diagnosticos/////////
#define DEBUG_MODE 1 // esta linea permite activar o desactivar el modo de depuracion, mostrando informacion adicional en el monitor serial
#define GRAPH_MODE 0 // esta linea permite activar o desactivar el modo grafico, mostrando la senal infrarroja en el Serial Plotter

Adafruit_SSD1306 display(
    SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET
);

MAX30105 sensor;

// Buffers para el algoritmo SparkFun
uint32_t irBuffer[BUFFER_SIZE];
uint32_t redBuffer[BUFFER_SIZE];

int32_t spo2 = 0;
int32_t heartRate = 0;
int8_t validSPO2 = 0;
int8_t validHeartRate = 0;

bool fingerPresent = false;
bool bufferReady = false;

int samplesCollected = 0;
int newSamples = 0;

unsigned long lastDisplay = 0;
unsigned long lastDiagnostic = 0;
unsigned long lastSample = 0;

// Diagnostico de frecuencia de muestreo
unsigned long lastSampleRateCheck = 0;
unsigned long samplesInInterval = 0;

// Resultados mostrados
int displayedBPM = 0;
int displayedSpO2 = 0;
bool bpmOK = false;
bool spo2OK = false;

int previousBPM = 0;
int consecutiveStable = 0;
int candidateBPM = 0;
int candidateCount = 0;

unsigned long lastValidBPM = 0;
const unsigned long BPM_TIMEOUT = 5000;

// Filtro de estabilidad para SpO2 /// esto permite exigir tres resultados consistentes antes de aceptar un valor de SpO2
int previousSpO2 = 0;
int consecutiveStableSpO2 = 0;

unsigned long lastValidSpO2 = 0;

const unsigned long SPO2_TIMEOUT = 5000;
const int SPO2_MAX_DIFFERENCE = 3;
const int SPO2_REQUIRED_STABLE = 3;

uint32_t lastIR = 0;
uint32_t lastRED = 0;

// Mostrar mensaje en OLED
void showMessage(const char *message) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 12);
    display.println(message);
    display.display();
}

// Reiniciar mediciones

void resetMeasurements() {
    samplesCollected = 0;
    newSamples = 0;
    bufferReady = false;

    previousBPM = 0;
    consecutiveStable = 0;
    candidateBPM = 0;
candidateCount = 0;
    lastValidBPM = 0;

    displayedBPM = 0;
    displayedSpO2 = 0;

    bpmOK = false;
    spo2OK = false;

    validHeartRate = 0;
    validSPO2 = 0;

    // Reiniciar filtro SpO2 /// esta funcion permite reiniciar el filtro de estabilidad para SpO2, asegurando que se exijan tres resultados consistentes antes de aceptar un valor de SpO2
previousSpO2 = 0;
consecutiveStableSpO2 = 0;
lastValidSpO2 = 0;
}

 // Calcular BPM y SpO2
void calculateMeasurements() {

    maxim_heart_rate_and_oxygen_saturation(
        irBuffer,
        BUFFER_SIZE,
        redBuffer,
        &spo2,
        &validSPO2,
        &heartRate,
        &validHeartRate
    );

    // Diagnostico de la senal infrarroja
    uint32_t irMin = irBuffer[0];
    uint32_t irMax = irBuffer[0];

    for (int i = 1; i < BUFFER_SIZE; i++) {
        if (irBuffer[i] < irMin) {
            irMin = irBuffer[i];
        }

        if (irBuffer[i] > irMax) {
            irMax = irBuffer[i];
        }
    }

    uint32_t irAmplitude = irMax - irMin;
    // Diagnostico relativo de la senal IR
uint64_t irSum = 0;

for (int i = 0; i < BUFFER_SIZE; i++) {
    irSum += irBuffer[i];
}

float irAverage = (float)irSum / BUFFER_SIZE;

float relativeAmplitude = 0;

if (irAverage > 0) {
    relativeAmplitude =
        ((float)irAmplitude / irAverage) * 100.0f;
}
#if DEBUG_MODE
Serial.print("IR promedio: ");
Serial.println(irAverage, 2);

Serial.print("Amplitud relativa IR (%): ");
Serial.println(relativeAmplitude, 3);


    Serial.print("IR minimo: ");
    Serial.println(irMin);

    Serial.print("IR maximo: ");
    Serial.println(irMax);

    Serial.print("Amplitud IR: ");
    Serial.println(irAmplitude);
    #endif
#if DEBUG_MODE
    // Valores originales del algoritmo
    Serial.print("BPM original: ");
    Serial.println(heartRate);

    Serial.print("ValidHR: ");
    Serial.println(validHeartRate);

    Serial.print("SpO2 original: ");
    Serial.println(spo2);

    Serial.print("ValidSpO2: ");
    Serial.println(validSPO2);
   #endif

    // Validacion inicial del algoritmo
    bool rawBpmValid =
        validHeartRate == 1 &&
        heartRate >= 40 &&
        heartRate <= 180;

    spo2OK =
        validSPO2 == 1 &&
        spo2 >= 70 &&
        spo2 <= 100;

    // Comprobar consistencia entre resultados
    bpmOK = false;

    

if (rawBpmValid) {

    if (previousBPM == 0) {

        previousBPM = heartRate;
        consecutiveStable = 1;

        candidateBPM = 0;
        candidateCount = 0;

    } else if (abs(heartRate - previousBPM) <= 12) {

        consecutiveStable++;
        previousBPM = heartRate;

        candidateBPM = 0;
        candidateCount = 0;

    } else {

        // La lectura no coincide con la referencia actual.
        // Buscar una nueva referencia consistente.

        consecutiveStable = 0;

        if (candidateBPM > 0 &&
            abs(heartRate - candidateBPM) <= 12) {

            candidateCount++;

        } else {

            candidateCount = 1;
        }

        candidateBPM = heartRate;

        if (candidateCount >= 2) {

            previousBPM = candidateBPM;
            consecutiveStable = 2;

            candidateBPM = 0;
            candidateCount = 0;
        }
    }

    // Aceptar BPM despues de tres lecturas consistentes
    if (consecutiveStable >= 3) {

        bpmOK = true;
        displayedBPM = heartRate;
        lastValidBPM = millis();
    }

} else {

    previousBPM = 0;
    consecutiveStable = 0;

    candidateBPM = 0;
    candidateCount = 0;
}
#if DEBUG_MODE
    // Diagnostico del filtro de BPM
    Serial.print("BPM aceptada: ");
    Serial.println(displayedBPM);

    Serial.print("Lecturas estables: ");
    Serial.println(consecutiveStable);
#endif
  
    // Conservar solo una BPM aceptada recientemente
    if (lastValidBPM != 0 &&
        displayedBPM > 0 &&
        millis() - lastValidBPM <= BPM_TIMEOUT) {

        bpmOK = true;

    } else {
        bpmOK = false;
        displayedBPM = 0;
    }

    
    // Filtro de estabilidad para SpO2
    bool rawSpO2Valid =
        validSPO2 == 1 &&
        spo2 >= 70 &&
        spo2 <= 100;

    if (rawSpO2Valid) {

        if (previousSpO2 > 0 &&
            abs(spo2 - previousSpO2) <= SPO2_MAX_DIFFERENCE) {

            consecutiveStableSpO2++;

        } else {
            consecutiveStableSpO2 = 1;
        }

        previousSpO2 = spo2;

        // Aceptar después de tres lecturas consistentes
        if (consecutiveStableSpO2 >= SPO2_REQUIRED_STABLE) {
            displayedSpO2 = spo2;
            lastValidSpO2 = millis();
        }

    } else {
        previousSpO2 = 0;
        consecutiveStableSpO2 = 0;
    }

    // Caducidad del valor mostrado
    if (lastValidSpO2 != 0 &&
        millis() - lastValidSpO2 <= SPO2_TIMEOUT &&
        displayedSpO2 > 0) {

        spo2OK = true;

    } else {
        spo2OK = false;
        displayedSpO2 = 0;
    }
#if DEBUG_MODE
    // Diagnostico del nuevo filtro
    Serial.print("SpO2 aceptado: ");

    if (spo2OK) Serial.println(displayedSpO2);
    else Serial.println("--");

    Serial.print("Lecturas estables SpO2: ");
    Serial.println(consecutiveStableSpO2);
    #endif
}



// Mostrar resultados
void updateOLED() {

    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);

    if (!fingerPresent) {
        display.setTextSize(1);
        display.setCursor(15, 12);
        display.print("COLOQUE DEDO");
    }
    else if (!bufferReady) {
        display.setTextSize(1);
        display.setCursor(20, 8);
        display.println("MIDIENDO...");
        display.setCursor(35, 20);
        display.print(samplesCollected);
        display.print("/100");
    }
    else {
        display.setTextSize(1);

        display.setCursor(0, 0);
        display.print("BPM");

        display.setCursor(78, 0);
        display.print("SpO2");

        display.setTextSize(2);

        display.setCursor(0, 13);
        if (bpmOK) display.print(displayedBPM);
        else display.print("--");

        display.setCursor(76, 13);
        if (spo2OK) {
            display.print(displayedSpO2);
            display.setTextSize(1);
            display.print("%");
        } else {
            display.print("--");
        }
    }

    display.display();
}

void setup() {

    Serial.begin(115200);
    delay(500);

    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(100000);

    if (!display.begin(
        SSD1306_SWITCHCAPVCC,
        OLED_ADDRESS
    )) {
        Serial.println("ERROR OLED");
        while (true) delay(1000);
    }

    display.setRotation(2);
    display.clearDisplay();
display.setTextSize(2);
display.setTextColor(SSD1306_WHITE);
display.setCursor(10, 8);
display.println("HOLA");
display.display();
delay(5000);

    showMessage("INICIANDO...");
    delay(1000);

    if (!sensor.begin(Wire, I2C_SPEED_STANDARD)) {
        Serial.println("ERROR MAX30102");
        showMessage("ERROR SENSOR");
        while (true) delay(1000);
    }

    // Configuracion de muestreo
    byte ledBrightness = 0x1F;
    byte sampleAverage = 4;
    byte ledMode = 2;
    int sampleRate = 100;
    int pulseWidth = 411;
    int adcRange = 16384;

    sensor.setup(
        ledBrightness,
        sampleAverage,
        ledMode,
        sampleRate,
        pulseWidth,
        adcRange
    );

    sensor.clearFIFO();

    Serial.println("MAX30102 INICIADO");
    Serial.println("Coloque el dedo");

    showMessage("SENSOR OK");
    delay(1000);
}

void loop() {


    
#if GRAPH_MODE

    // Modo exclusivo para observar la senal IR
    sensor.check();

    while (sensor.available()) {
        uint32_t ir = sensor.getFIFOIR();
        sensor.nextSample();

        Serial.print("IR:");
        Serial.println(ir);
    }

    delay(1);
    return;

#endif

 // Obtener muestras disponibles del FIFO
    sensor.check();

    while (sensor.available()) {

        uint32_t ir = sensor.getFIFOIR();
        uint32_t red = sensor.getFIFORed();

        sensor.nextSample();
        #if DEBUG_MODE
        // Contar cada muestra leida del FIFO
samplesInInterval++;
   #endif

        lastIR = ir;
        lastRED = red;
        lastSample = millis();
        // Enviar senal infrarroja para observarla
// en el Serial Plotter
// Serial.println(ir);


        // Detectar dedo
        if (ir < FINGER_THRESHOLD) {

            if (fingerPresent) {
                Serial.println("DEDO RETIRADO");
            }

            fingerPresent = false;
            resetMeasurements();
            continue;
        }

        if (!fingerPresent) {
            fingerPresent = true;
            resetMeasurements();
            Serial.println("DEDO DETECTADO");
        }

        // Llenar el buffer inicial
        if (!bufferReady) {

            irBuffer[samplesCollected] = ir;
            redBuffer[samplesCollected] = red;

            samplesCollected++;

            if (samplesCollected >= BUFFER_SIZE) {
                bufferReady = true;
                calculateMeasurements();
            }
        }
        else {

            // Desplazar las muestras
            if (newSamples == 0) {

                for (int i = 0;
                     i < BUFFER_SIZE - STEP_SIZE;
                     i++) {

                    irBuffer[i] =
                        irBuffer[i + STEP_SIZE];

                    redBuffer[i] =
                        redBuffer[i + STEP_SIZE];
                }
            }

            int index =
                BUFFER_SIZE - STEP_SIZE + newSamples;

            irBuffer[index] = ir;
            redBuffer[index] = red;

            newSamples++;

            if (newSamples >= STEP_SIZE) {
                newSamples = 0;
                calculateMeasurements();
            }
        }
    }
    #if DEBUG_MODE
    // Diagnostico de muestras recibidas por segundo
if (millis() - lastSampleRateCheck >= 5000) {

    unsigned long elapsed =
        millis() - lastSampleRateCheck;

    float samplesPerSecond =
        (samplesInInterval * 1000.0f) / elapsed;

    Serial.print("FRECUENCIA REAL: ");
    Serial.print(samplesPerSecond, 2);
    Serial.println(" muestras/segundo");

    samplesInInterval = 0;
    lastSampleRateCheck = millis();
}
#endif
#if DEBUG_MODE // Activar o desactivar diagnosticos
    // Diagnostico serial cada segundo
    if (millis() - lastDiagnostic >= 1000) {

        lastDiagnostic = millis();

        Serial.print("IR: ");
        Serial.print(lastIR);

        Serial.print(" | RED: ");
        Serial.print(lastRED);

        Serial.print(" | Dedo: ");
        Serial.print(fingerPresent ? "SI" : "NO");

        Serial.print(" | Muestras: ");
        Serial.print(samplesCollected);

        Serial.print(" | BPM: ");
        if (bpmOK) Serial.print(displayedBPM);
        else Serial.print("--");

        Serial.print(" | SpO2: ");
        if (spo2OK) Serial.println(displayedSpO2);
        else Serial.println("--");
    }
    #endif // Activar o desactivar diagnosticos

    // Detectar interrupcion de muestras
    if (millis() - lastSample > 3000 &&
        fingerPresent) {

        Serial.println("SIN MUESTRAS");
        fingerPresent = false;
        resetMeasurements();
    }
    
    // Comprobar si la ultima BPM aceptada ha caducado
    if (bpmOK &&
        millis() - lastValidBPM > BPM_TIMEOUT) {

        bpmOK = false;
        displayedBPM = 0;
#if DEBUG_MODE
        Serial.println("BPM CADUCADAS");
      #endif  
    }
    // Verificar caducidad del SpO2 // esto permite que el valor de SpO2 caduque si no se reciben nuevas lecturas válidas dentro del tiempo especificado, asegurando que los resultados mostrados sean recientes y confiables
if (spo2OK &&
    millis() - lastValidSpO2 > SPO2_TIMEOUT) {

    spo2OK = false;
    displayedSpO2 = 0;
#if DEBUG_MODE
    Serial.println("SpO2 CADUCADO");
    #endif
}
#if DEBUG_MODE

// Diagnostico temporal del tiempo de BPM
static unsigned long lastTimeoutDiagnostic = 0;

if (fingerPresent &&
    bufferReady &&
    millis() - lastTimeoutDiagnostic >= 1000) {

    lastTimeoutDiagnostic = millis();

    Serial.print("Tiempo sin nueva BPM aceptada: ");

    if (lastValidBPM == 0) {
        Serial.println("SIN BPM ACEPTADA");
    } else {
        Serial.print(millis() - lastValidBPM);
        Serial.println(" ms");
    }
}
#endif

    
   // Actualizar pantalla OLED cada segundo ///esta linea asegura que la pantalla OLED se actualice cada segundo, proporcionando al usuario información visual sobre las mediciones de BPM y SpO2 de manera oportuna
if (millis() - lastDisplay >= 1000) {
    lastDisplay = millis();
    updateOLED();
}
    
    delay(1);
}






