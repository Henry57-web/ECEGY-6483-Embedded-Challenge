#include <Arduino.h>
#include <Wire.h>
#include <arduinoFFT.h>
#include <math.h>

// -----------------------------
// I2C2 for LSM6DSL
// -----------------------------
TwoWire Wire2(PB11, PB10); // SDA = PB11, SCL = PB10

#define LSM6DSL_ADDR 0x6A
#define REG_CTRL1_XL 0x10
#define REG_OUTX_L_XL 0x28
#define REG_WHOAMI 0x0F

// -----------------------------
// LED pins
// -----------------------------
#define LED_TREMOR LED1
#define LED_DYSK LED2
#define LED_FOG LED3

// -----------------------------
// FFT settings
// -----------------------------
constexpr int SAMPLE_RATE = 342;
constexpr int BUF_LEN = 1024;

float buf[BUF_LEN];
float mag[BUF_LEN / 2];

double vReal[BUF_LEN];
double vImag[BUF_LEN];

ArduinoFFT<double> FFT(vReal, vImag, BUF_LEN, SAMPLE_RATE);

int idx = 0;

// -----------------------------
// FOG detection parameters
// -----------------------------
float previousVar = 0.0;
bool wasMoving = false;

#define MOVEMENT_VAR_THR 0.0008f // Variance threshold for movement detection
#define STILL_VAR_THR 0.00015f   // Variance threshold for stillness detection
#define VAR_DROP_RATIO 0.6f      // Required variance drop ratio for FOG
#define FOG_ENERGY_THR 8.0f      // Energy threshold for 1–3 Hz band

// -----------------------------
// Read multiple registers
// -----------------------------
bool readRegisters(uint8_t reg, uint8_t *buf, int len)
{
  Wire2.beginTransmission(LSM6DSL_ADDR);
  Wire2.write(reg);
  if (Wire2.endTransmission(false) != 0)
    return false;

  int n = Wire2.requestFrom(LSM6DSL_ADDR, len);
  if (n != len)
    return false;

  for (int i = 0; i < len; i++)
    buf[i] = Wire2.read();

  return true;
}

// -----------------------------
// Write single register
// -----------------------------
bool writeRegister(uint8_t reg, uint8_t val)
{
  Wire2.beginTransmission(LSM6DSL_ADDR);
  Wire2.write(reg);
  Wire2.write(val);
  return Wire2.endTransmission() == 0;
}

// -----------------------------
// Compute variance of data buffer
// -----------------------------
float computeVariance(float *data, int len)
{
  float mean = 0;
  for (int i = 0; i < len; i++)
    mean += data[i];
  mean /= len;

  float var = 0;
  for (int i = 0; i < len; i++)
  {
    float d = data[i] - mean;
    var += d * d;
  }
  return var / len;
}

// -----------------------------
// FFT with DC offset removal
// -----------------------------
void computeFFT()
{
  // Compute mean for DC removal
  double mean = 0;
  for (int i = 0; i < BUF_LEN; i++)
    mean += buf[i];
  mean /= BUF_LEN;

  // Remove DC component
  for (int i = 0; i < BUF_LEN; i++)
  {
    vReal[i] = buf[i] - mean;
    vImag[i] = 0;
  }

  FFT.windowing(vReal, BUF_LEN, FFTWindow::Hamming, FFTDirection::Forward);
  FFT.compute(vReal, vImag, BUF_LEN, FFTDirection::Forward);
  FFT.complexToMagnitude(vReal, vImag, BUF_LEN);

  for (int i = 0; i < BUF_LEN / 2; i++)
    mag[i] = vReal[i];
}

// -----------------------------
// Find max magnitude index in frequency band
// -----------------------------
int argmax(int f1, int f2)
{
  int k1 = (f1 * BUF_LEN) / SAMPLE_RATE;
  int k2 = (f2 * BUF_LEN) / SAMPLE_RATE;

  int best = k1;
  float val = mag[k1];

  for (int k = k1; k <= k2; k++)
  {
    if (mag[k] > val)
    {
      val = mag[k];
      best = k;
    }
  }
  return best;
}

// -----------------------------
// Map frequency to blink delay
// -----------------------------
int mapBlink(int f, int fmin, int fmax)
{
  int maxD = 1000;
  int minD = 100;

  if (f <= fmin)
    return maxD;
  if (f >= fmax)
    return minD;

  float r = (float)(f - fmin) / (float)(fmax - fmin);
  return maxD - r * (maxD - minD);
}

// -----------------------------
// Compute energy in frequency band
// -----------------------------
float computeBandEnergy(int f1, int f2)
{
  int k1 = (f1 * BUF_LEN) / SAMPLE_RATE;
  int k2 = (f2 * BUF_LEN) / SAMPLE_RATE;

  float sum = 0;
  for (int k = k1; k <= k2; k++)
    sum += mag[k];

  return sum;
}

// -----------------------------
// Flash LED with given delay
// -----------------------------
void flashLED(int ledPin, int delayMs)
{
  unsigned long start = millis();
  while (millis() - start < 5000)
  {
    digitalWrite(ledPin, HIGH);
    delay(delayMs);
    digitalWrite(ledPin, LOW);
    delay(delayMs);
  }
}

// -----------------------------
// Setup
// -----------------------------
void setup()
{
  Serial.begin(115200);
  delay(200);

  Serial.println("=== Tremor / Dyskinesia / FOG Detection Start ===");

  pinMode(LED_TREMOR, OUTPUT);
  pinMode(LED_DYSK, OUTPUT);
  pinMode(LED_FOG, OUTPUT);

  Wire2.begin();
  Wire2.setClock(400000);

  writeRegister(REG_CTRL1_XL, 0x60);
  delay(10);

  uint8_t id = 0;
  readRegisters(REG_WHOAMI, &id, 1);
  Serial.print("WHO_AM_I = 0x");
  Serial.println(id, HEX);
}

// -----------------------------
// Loop
// -----------------------------
void loop()
{
  uint8_t raw[6];

  if (readRegisters(REG_OUTX_L_XL, raw, 6))
  {
    int16_t ax = (raw[1] << 8) | raw[0];
    int16_t ay = (raw[3] << 8) | raw[2];
    int16_t az = (raw[5] << 8) | raw[4];

    float g = sqrtf(ax * ax + ay * ay + az * az) * (2.0f / 32768.0f);
    if (!isfinite(g))
      g = 0;

    buf[idx++] = g;

    if (idx >= BUF_LEN)
    {
      Serial.println("\n--- Window full, FFT processing ---");

      float variance = computeVariance(buf, BUF_LEN);
      bool isMoving = variance > MOVEMENT_VAR_THR;

      computeFFT();

      // --- Tremor & Dysk frequency peaks ---
      int kt = argmax(3, 5);
      int kd = argmax(5, 7);

      float ft = (float)kt * SAMPLE_RATE / BUF_LEN;
      float fd = (float)kd * SAMPLE_RATE / BUF_LEN;

      float mt = mag[kt];
      float md = mag[kd];

      Serial.print("Tremor F=");
      Serial.print(ft, 2);
      Serial.print(" Mag=");
      Serial.println(mt, 2);

      Serial.print("Dysk   F=");
      Serial.print(fd, 2);
      Serial.print(" Mag=");
      Serial.println(md, 2);

      Serial.print("Var=");
      Serial.print(variance, 6);
      Serial.print(" PrevVar=");
      Serial.print(previousVar, 6);
      Serial.print(" Moving=");
      Serial.println(isMoving);

      // --- FOG peak in 1–3 Hz band ---
      int fogK = argmax(1, 3);
      float fogF = (float)fogK * SAMPLE_RATE / BUF_LEN;
      float fogMag = mag[fogK];

      Serial.print("FOG peak F=");
      Serial.print(fogF, 2);
      Serial.print(" Mag=");
      Serial.println(fogMag, 2);

      float fogEnergy = computeBandEnergy(1, 3);
      Serial.print("FOG Energy=");
      Serial.println(fogEnergy, 2);

      // --- Simplified FOG rule: movement → sudden stillness ---
      bool trem = (mt > md && mt > 25.0f);
      bool dysk = (md > 25.0f);
      bool fog = false;

      if (wasMoving && !isMoving)
      {
        fog = true; // Trigger FOG when movement stops suddenly
      }

      // -----------------------------
      // LED Output
      // -----------------------------
      if (fog)
      {
        Serial.println(">>> FOG detected — LED3 FAST blink");
        flashLED(LED_FOG, 80);
      }
      else if (trem)
      {
        int d = mapBlink((int)ft, 3, 5);
        Serial.println(">> Tremor detected — LED1 blink");
        flashLED(LED_TREMOR, d);
      }
      else if (dysk)
      {
        // Amplitude-controlled blinking for dyskinesia
        float md_min = 10.0f;
        float md_max = 40.0f;

        int maxD = 1000;
        int minD = 100;
        int d;

        if (md <= md_min)
          d = maxD;
        else if (md >= md_max)
          d = minD;
        else
        {
          float r = (md - md_min) / (md_max - md_min);
          d = maxD - r * (maxD - minD);
        }

        Serial.print(">> Dysk detected — LED2 blink (Amp-based) delay=");
        Serial.println(d);

        flashLED(LED_DYSK, d);
      }
      else
      {
        Serial.println(">> No abnormal motion");
      }

      previousVar = variance;
      wasMoving = isMoving;
      idx = 0;
    }
  }

  delayMicroseconds((int)(1000000.0 / SAMPLE_RATE));
}
