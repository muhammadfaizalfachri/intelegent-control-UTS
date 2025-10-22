// =============================================================  
//  SISTEM KONTROL MOTOR KONVEYOR DENGAN INPUT PWM DARI PYTHON
//  Komunikasi Serial (115200 bps) — Kompatibel dengan Fuzzy Control Python
//  Dilengkapi Encoder untuk Monitoring Kecepatan + Servo Penahan
//  Dibuat oleh: Lui — Revisi stabil servo + motor nyala bareng
// =============================================================

#include <Arduino.h>
#include <Servo.h>

volatile long encCount = 0;

// ---------------- PIN KONFIGURASI ----------------
const int pinEncA = 2;   // Encoder Channel A (Interrupt)
const int pinEncB = 3;   // Encoder Channel B
const int pinIN1  = 7;   // Motor Driver IN1
const int pinIN2  = 8;   // Motor Driver IN2
const int pinENA  = 5;   // PWM (Enable Motor A) — PIN 5 biar tidak bentrok dengan servo
const int pinServo = 10; // Servo pin (Timer1)

// ---------------- VARIABEL MOTOR ----------------
int pwmValue = 100;       // PWM awal motor (default jalan sedang)
unsigned long lastPrint = 0;

// ---------------- VARIABEL SERVO ----------------
Servo myServo;
bool servoActive = false;
unsigned long servoStart = 0;

// ---------------- ISR ENCODER ----------------
void isrEncA() {
  int a = digitalRead(pinEncA);
  int b = digitalRead(pinEncB);
  if (a == b)
    encCount++;
  else
    encCount--;
}

// ---------------- SETUP ----------------
void setup() {
  Serial.begin(115200);
  pinMode(pinEncA, INPUT_PULLUP);
  pinMode(pinEncB, INPUT_PULLUP);
  pinMode(pinIN1, OUTPUT);
  pinMode(pinIN2, OUTPUT);
  pinMode(pinENA, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(pinEncA), isrEncA, CHANGE);

  // Jalankan motor default ON
  digitalWrite(pinIN1, HIGH);
  digitalWrite(pinIN2, LOW);
  analogWrite(pinENA, pwmValue);

  // Inisialisasi servo
  myServo.attach(pinServo);
  myServo.write(0); // posisi normal (tidak menahan)

  Serial.println("🚀 Arduino Ready — Mode: Fuzzy Adaptive PWM + Servo aktif");
  Serial.println("Kirim format: PWM:<nilai> (0–255), STOP, atau SERVO:HOLD");
}

// ---------------- LOOP ----------------
void loop() {
  // === 1. BACA DATA DARI PYTHON ===
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    data.trim();

    // ---- Perintah PWM: ----
    if (data.startsWith("PWM:")) {
      int newPWM = data.substring(4).toInt();
      newPWM = constrain(newPWM, 0, 255);

      if (newPWM != pwmValue) {
        pwmValue = newPWM;
        digitalWrite(pinIN1, HIGH);
        digitalWrite(pinIN2, LOW);
        analogWrite(pinENA, pwmValue);

        Serial.print("✅ PWM updated => ");
        Serial.println(pwmValue);
      }
    }

    // ---- Perintah STOP: ----
    else if (data.equalsIgnoreCase("STOP")) {
      pwmValue = 0;
      digitalWrite(pinIN1, LOW);
      digitalWrite(pinIN2, LOW);
      analogWrite(pinENA, 0);
      Serial.println("🛑 Motor STOP");
    }

    // ---- Perintah SERVO:HOLD (tambahan) ----
  else if (data.equalsIgnoreCase("SERVO:HOLD")) {
  myServo.write(90);
  servoActive = true;
  servoStart = millis();
  Serial.println("⚙️ Servo aktif — Menahan benda 2 detik (jarak dekat)");
}

  }

  // === 2. MONITOR ENCODER SETIAP 1 DETIK ===
  if (millis() - lastPrint >= 1000) {
    lastPrint = millis();

    long count;
    noInterrupts();
    count = encCount;
    encCount = 0; // reset tiap 1 detik
    interrupts();

    Serial.print("ENC Count/s: ");
    Serial.print(count);
    Serial.print(" | PWM: ");
    Serial.println(pwmValue);
  }

  // === 3. LOGIKA TIMER SERVO ===
  if (servoActive && millis() - servoStart >= 2000) { // tahan 2 detik
    myServo.write(0);          // kembalikan servo ke posisi awal
    servoActive = false;
    Serial.println("✅ Servo kembali ke posisi awal");
  }

  delay(5);
}
