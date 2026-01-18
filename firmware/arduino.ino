#include <Servo.h>
#include <dht.h>
//#include <Adafruit_VL53L0X.h>
#include <Wire.h>
#include <MPU6050.h>

// =====================================================
// SERVOS
// =====================================================
Servo servo1;  // Codo brazo (0-180°)
Servo servo2;  // Sensor brazo (0-180°)
Servo servo3;  // Cámara frontal (0-30°)
//Servo servo4;  // Cámara superior (0-180°)

// =====================================================
// SENSORES
// =====================================================
dht DHT;
//Adafruit_VL53L0X lox = Adafruit_VL53L0X();
MPU6050 mpu;

const int PIN_TEMP = 8;
const int PIN_HUMEDAD = A0;
const int PIN_LUZ = A1;

// =====================================================
// MOTORES DC (PWM)
// =====================================================
// Motor A = Izquierdo
const int MOTOR_A1 = 3;   // Adelante
const int MOTOR_A2 = 5;   // Atrás
// Motor B = Derecho
const int MOTOR_B1 = 11;  // Adelante
const int MOTOR_B2 = 6;  // Atrás

// =====================================================
// CALIBRACIÓN DE MOTORES
// =====================================================
// Factor de corrección para compensar el drift
// Si el rover va a la izquierda, el motor derecho es más débil
// Ajusta estos valores según tu robot:
// - Si va mucho a la izquierda: aumenta FACTOR_DERECHO o disminuye FACTOR_IZQUIERDO
// - Si va mucho a la derecha: aumenta FACTOR_IZQUIERDO o disminuye FACTOR_DERECHO
float FACTOR_IZQUIERDO = 0.95;  // Reduce ligeramente el motor izquierdo
float FACTOR_DERECHO = 1.0;     // Motor derecho al 100%

// Velocidad mínima para que los motores se muevan
const int VEL_MINIMA = 50;

// =====================================================
// VARIABLES MPU6050
// =====================================================
int16_t ax, ay, az;
int16_t gx, gy, gz;

// =====================================================
// SETUP
// =====================================================
void setup() {
  Serial.begin(9600);
  
  // Inicializar servos
  servo1.attach(4);
  servo2.attach(2);
  servo3.attach(7);
  //servo4.attach(9);
  
  // Posiciones iniciales
  servo1.write(0);
  servo2.write(0);
  servo3.write(15);
  //servo4.write(90);
  
  // Configurar pines de motores
  pinMode(MOTOR_A1, OUTPUT);
  pinMode(MOTOR_A2, OUTPUT);
  pinMode(MOTOR_B1, OUTPUT);
  pinMode(MOTOR_B2, OUTPUT);
  
  // Detener motores
  detenerMotores();
  
  // Inicializar MPU6050
  Wire.begin();
  mpu.initialize();
  
  if (mpu.testConnection()) {
    Serial.println("MPU6050: OK");
  } else {
    Serial.println("MPU6050: ERROR");
  }
  
  // Inicializar VL53L0X
  //if (!lox.begin()) {
  //  Serial.println("VL53L0X: ERROR");
  //} else {
  //  Serial.println("VL53L0X: OK");
  //}
  
  Serial.println("ARDUINO LISTO");
  Serial.print("Calibracion: IZQ=");
  Serial.print(FACTOR_IZQUIERDO);
  Serial.print(" DER=");
  Serial.println(FACTOR_DERECHO);
  delay(500);
}

// =====================================================
// FUNCIONES DE MOTORES - MEJORADAS CON CALIBRACIÓN
// =====================================================

void detenerMotores() {
  // Apagar todos los pines
  analogWrite(MOTOR_A1, 0);
  analogWrite(MOTOR_A2, 0);
  analogWrite(MOTOR_B1, 0);
  analogWrite(MOTOR_B2, 0);
}

void avanzar(int vel) {
  vel = constrain(vel, VEL_MINIMA, 255);
  
  // Aplicar factores de calibración
  int vel_izq = (int)(vel * FACTOR_IZQUIERDO);
  int vel_der = (int)(vel * FACTOR_DERECHO);
  
  // Asegurar que no excedan límites
  vel_izq = constrain(vel_izq, VEL_MINIMA, 255);
  vel_der = constrain(vel_der, VEL_MINIMA, 255);
  
  // Ambos motores adelante
  analogWrite(MOTOR_A1, vel_izq);  // Izquierdo adelante
  analogWrite(MOTOR_A2, 0);
  analogWrite(MOTOR_B1, vel_der);  // Derecho adelante
  analogWrite(MOTOR_B2, 0);
}

void retroceder(int vel) {
  vel = constrain(vel, VEL_MINIMA, 255);
  
  // Aplicar factores de calibración
  int vel_izq = (int)(vel * FACTOR_IZQUIERDO);
  int vel_der = (int)(vel * FACTOR_DERECHO);
  
  vel_izq = constrain(vel_izq, VEL_MINIMA, 255);
  vel_der = constrain(vel_der, VEL_MINIMA, 255);
  
  // Ambos motores atrás
  analogWrite(MOTOR_A1, 0);
  analogWrite(MOTOR_A2, vel_izq);  // Izquierdo atrás
  analogWrite(MOTOR_B1, 0);
  analogWrite(MOTOR_B2, vel_der);  // Derecho atrás
}

void girarIzquierda(int vel) {
  vel = constrain(vel, VEL_MINIMA, 255);
  
  // En giros, usar velocidad completa sin calibración
  // porque queremos máximo torque de giro
  
  // Izquierdo atrás, Derecho adelante (giro en el lugar)
  analogWrite(MOTOR_A1, 0);
  analogWrite(MOTOR_A2, vel);  // Izquierdo atrás
  analogWrite(MOTOR_B1, vel);  // Derecho adelante
  analogWrite(MOTOR_B2, 0);
}

void girarDerecha(int vel) {
  vel = constrain(vel, VEL_MINIMA, 255);
  
  // En giros, usar velocidad completa sin calibración
  
  // Izquierdo adelante, Derecho atrás (giro en el lugar)
  analogWrite(MOTOR_A1, vel);  // Izquierdo adelante
  analogWrite(MOTOR_A2, 0);
  analogWrite(MOTOR_B1, 0);
  analogWrite(MOTOR_B2, vel);  // Derecho atrás
}

// =====================================================
// FUNCIÓN PARA AJUSTAR CALIBRACIÓN EN TIEMPO REAL
// =====================================================
void ajustarCalibracion(float factor_izq, float factor_der) {
  FACTOR_IZQUIERDO = constrain(factor_izq, 0.5, 1.5);
  FACTOR_DERECHO = constrain(factor_der, 0.5, 1.5);
  
  Serial.print("Nueva calibracion: IZQ=");
  Serial.print(FACTOR_IZQUIERDO);
  Serial.print(" DER=");
  Serial.println(FACTOR_DERECHO);
}

// =====================================================
// FUNCIONES DE SENSORES
// =====================================================

float leerTemperatura() {
  int chk = DHT.read11(PIN_TEMP);
  if (chk == DHTLIB_OK) {
    return DHT.temperature;
  }
  return -999;
}

int leerHumedad() {
  int lectura = analogRead(PIN_HUMEDAD);
  // Calibración: 588 = 0%, 308 = 100%
  int humedad = map(lectura, 588, 308, 0, 100);
  return constrain(humedad, 0, 100);
}

int leerLuz() {
  int lectura = analogRead(PIN_LUZ);
  // 1023 = oscuro, 0 = brillante
  int luz = map(lectura, 1023, 0, 0, 100);
  return constrain(luz, 0, 100);
}

float leerDistancia() {
  //VL53L0X_RangingMeasurementData_t medida;
  //lox.rangingTest(&medida, false);
  
  //if (medida.RangeStatus != 4) {
  //  return medida.RangeMilliMeter;
  //}
  return -1;
}

void leerMPU() {
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
}

// =====================================================
// PROCESAMIENTO DE COMANDOS
// =====================================================

void procesarComando(String comando) {
  comando.trim();
  
  int espacio = comando.indexOf(' ');
  String accion;
  int valor = 0;
  
  if (espacio > 0) {
    accion = comando.substring(0, espacio);
    valor = comando.substring(espacio + 1).toInt();
  } else {
    accion = comando;
  }
  
  // ========== MOVIMIENTO ==========
  if (accion == "avanzar") {
    avanzar(valor > 0 ? valor : 200);
    Serial.println("OK: Avanzando");
  }
  else if (accion == "retroceder") {
    retroceder(valor > 0 ? valor : 200);
    Serial.println("OK: Retrocediendo");
  }
  else if (accion == "izquierda") {
    girarIzquierda(valor > 0 ? valor : 200);
    Serial.println("OK: Girando izquierda");
  }
  else if (accion == "derecha") {
    girarDerecha(valor > 0 ? valor : 200);
    Serial.println("OK: Girando derecha");
  }
  else if (accion == "stop") {
    detenerMotores();
    Serial.println("OK: Detenido");
  }
  
  // ========== CALIBRACIÓN DE MOTORES ==========
  else if (accion == "calibrar") {
    // Comando: calibrar 0.95 1.0
    // Formato: calibrar <factor_izq> <factor_der>
    int segundoEspacio = comando.indexOf(' ', espacio + 1);
    if (segundoEspacio > 0) {
      float factor_izq = comando.substring(espacio + 1, segundoEspacio).toFloat();
      float factor_der = comando.substring(segundoEspacio + 1).toFloat();
      ajustarCalibracion(factor_izq, factor_der);
    } else {
      Serial.println("Uso: calibrar <factor_izq> <factor_der>");
      Serial.println("Ejemplo: calibrar 0.95 1.0");
    }
  }
  else if (accion == "get_calibracion") {
    Serial.print("Calibracion actual: IZQ=");
    Serial.print(FACTOR_IZQUIERDO);
    Serial.print(" DER=");
    Serial.println(FACTOR_DERECHO);
  }
  
  // ========== SERVOS ==========
  else if (accion == "servo1") {
    valor = constrain(valor, 0, 180);
    servo1.write(valor);
    Serial.print("Servo1: ");
    Serial.print(valor);
    Serial.println("°");
  }
  else if (accion == "servo2") {
    valor = constrain(valor, 0, 180);
    servo2.write(valor);
    Serial.print("Servo2: ");
    Serial.print(valor);
    Serial.println("°");
  }
  else if (accion == "servo3") {
    valor = constrain(valor, 0, 30);
    servo3.write(valor);
    Serial.print("Servo3: ");
    Serial.print(0);
    Serial.println("°");
  }
  else if (accion == "servo4") {
  //  valor = constrain(valor, 0, 180);
  //  servo4.write(valor);
    Serial.print("Servo4: ");
    Serial.print(0);
    Serial.println("°");
  }
  
  // ========== SENSORES ==========
  else if (accion == "temp") {
    float temp = leerTemperatura();
    if (temp > -999) {
      Serial.print(temp, 1);
      Serial.println(" °C");
    } else {
      Serial.println("Error DHT");
    }
  }
  else if (accion == "humedad") {
    int hum = leerHumedad();
    Serial.print(hum);
    Serial.println(" %");
  }
  else if (accion == "luz") {
    int luz = leerLuz();
    Serial.print(luz);
    Serial.println(" V");
  }
  else if (accion == "dist") {
    float dist = leerDistancia();
    if (dist > 0) {
      Serial.print(dist, 0);
      Serial.println(" mm");
    } else {
      Serial.println("Fuera de rango");
    }
  }
  else if (accion == "mpu") {
    leerMPU();
    Serial.print("AX: "); Serial.print(ax);
    Serial.print(" AY: "); Serial.print(ay);
    Serial.print(" AZ: "); Serial.print(az);
    Serial.print(" GX: "); Serial.print(gx);
    Serial.print(" GY: "); Serial.print(gy);
    Serial.print(" GZ: "); Serial.println(gz);
  }
  
  // ========== DIAGNÓSTICO ==========
  else if (accion == "status") {
    Serial.println("=== STATUS ===");
    Serial.print("Temp: "); Serial.print(leerTemperatura()); Serial.println(" C");
    Serial.print("Luz: "); Serial.print(leerLuz()); Serial.println(" V");
    Serial.print("Humedad: "); Serial.print(leerHumedad()); Serial.println(" %");
    Serial.print("Dist: "); Serial.print(leerDistancia()); Serial.println(" mm");
    Serial.print("Calibracion: IZQ="); Serial.print(FACTOR_IZQUIERDO);
    Serial.print(" DER="); Serial.println(FACTOR_DERECHO);
  }
  
  // ========== TEST MOTORES CON CALIBRACIÓN ==========
  else if (accion == "test_motores") {
    Serial.println("=== TEST DE MOTORES ===");
    
    Serial.println("Avanzar 3s...");
    avanzar(150);
    delay(3000);
    detenerMotores();
    delay(1000);
    
    Serial.println("Retroceder 3s...");
    retroceder(150);
    delay(3000);
    detenerMotores();
    delay(1000);
    
    Serial.println("Izquierda 2s...");
    girarIzquierda(150);
    delay(2000);
    detenerMotores();
    delay(1000);
    
    Serial.println("Derecha 2s...");
    girarDerecha(150);
    delay(2000);
    detenerMotores();
    
    Serial.println("Test completado");
    Serial.println("Si el robot se desvio, ajusta calibracion con:");
    Serial.println("  calibrar <factor_izq> <factor_der>");
  }
  
  // ========== COMANDO DESCONOCIDO ==========
  else {
    Serial.print("ERROR: Comando desconocido: ");
    Serial.println(comando);
  }
}

// =====================================================
// LOOP PRINCIPAL
// =====================================================
void loop() {
  if (Serial.available() > 0) {
    String comando = Serial.readStringUntil('\n');
    procesarComando(comando);
  }
  
  delay(10);
}