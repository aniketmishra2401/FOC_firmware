#include <Arduino.h>
#include <Wire.h>

//pins
const int PWM_A=25, PWM_B=33, PWM_C=32;
const int Driver_off=26;
const int ADC_Ia=34, ADC_Ib=35, ADC_Vbus=36;
const int Encoder=0x36;

const int Pole_pairs=7;
const float KV=1000.0f;
const float Rph=0.18f;
const float L=100e-6f;
const float J=2e-5f, fric=1e-5f;
const float Imax=6.0f;
const float Current_BW=400.0f;
const float Kp=25.0f, Kd=10.0f;
const unsigned long Loop=50;
const unsigned long Max_gap=250;

//sensors
const float zero_Va=1.652f, zero_Vb=1.648f;
const float V_per_Ia=0.200f, V_per_Ib=0.200f;
const float Bus_divider=4.3f;
const float Rotor_zero=0.52f;
const int Encoder_DIR=1;

const bool Motor_en=false;

const float Flux= (60.0f/(2.0f*PI*KV))/(1.5f*Pole_pairs);
const float T_per_Amp= 1.5f * Pole_pairs * Flux;
const float Tmax= T_per_Amp * Imax;
const float Kp_I= L * 2.0f * PI * Current_BW;
const float Ki_I= Rph * 2.0f * PI * Current_BW;

//initialisation
bool armed=false, positionMode=false, firstAngle=true;
float torqueCmd=0.0f, posTarget=0.0f;
float position = 0.0f, speed = 0.0f;  
float angle=0.0f, lastAngle=0.0f;
float Ia=0.0f, Ib=0.0f, Ic=0.0f, busV=0.0f;
float rawA=0.0f, rawB=0.0f, rawBus=0.0f;
float integralD=0.0f, integralQ=0.0f;
unsigned long lastLoop=0, lastPrint=0;


bool ReadEncoder(){
  Wire.beginTransmission(Encoder);
  Wire.write(0x0C);
  if (Wire.endTransmission(false) !=0) return false;
  if (Wire.requestFrom(Encoder,2) !=2) return false;
  int high = Wire.read();
  int low = Wire.read();
  int raw = ((high<<8) | low) & 0x0FFF;
  angle = raw * (2.0f * PI /4096.0f);
  return true;
}

//calibration
bool ReadSensors(float dt){
  rawA = analogReadMilliVolts(ADC_Ia) / 1000.0f;
  rawB = analogReadMilliVolts(ADC_Ib) / 1000.0f;
  rawBus = analogReadMilliVolts(ADC_Vbus) / 1000.0f;

  bool calibrated = V_per_Ia >0 && V_per_Ib>0 && Bus_divider>0;
  if (calibrated){
    Ia = (rawA - zero_Va)/ V_per_Ia;
    Ib = (rawB - zero_Vb)/ V_per_Ib;
    busV= rawBus * Bus_divider;
  }

  if (!ReadEncoder()) return false;
  float a= angle*Encoder_DIR;
  if (firstAngle){
    lastAngle=a;
    firstAngle=false;
  }
  else{
    float step=a-lastAngle;
    if (step>PI) step -= 2.0f *PI;
    if (step< -PI) step +=2.0f * PI;
    position +=step;
    speed = 0.8f *speed +0.2f * step /dt;
    lastAngle=a;
  }
  return calibrated;
}

//outputs
void writeDuty(float a, float b, float c){
  ledcWrite(PWM_A, constrain(a,0.0f,1.0f)*4095);
  ledcWrite(PWM_B, constrain(b, 0.0f, 1.0f) * 4095);
  ledcWrite(PWM_C, constrain(c, 0.0f, 1.0f) * 4095);
}

void disarm(){
  armed=false;
  digitalWrite(Driver_off, HIGH);
  writeDuty(0,0,0);
  integralD=integralQ=0.0f;
}

//check
bool limits(){
  float Ic=-Ia-Ib;
  return busV >=9.0f && busV<=12.6f &&
          fabsf(Ia)<Imax && fabsf(Ib) <= Imax &&
          fabsf(Ic)<=Imax;
}


//FOC
void runFOC(float dt){
  
  float torque=torqueCmd;
  if(positionMode){
    float accel= Kp * (posTarget - position) -Kd * speed;
    torque = J* accel + fric * speed;
  }
  torque = constrain(torque, -Tmax, Tmax);
  float iqWanted= torque/T_per_Amp;

  float elec= fmodf(Pole_pairs *angle * Encoder_DIR - Rotor_zero, 2.0f*PI);
  if (elec<0) elec += 2.0f*PI;
  float c= cosf(elec), s=sinf(elec);

  //clarke park
  float iAlpha = Ia;
  float iBeta= (Ia +2.0f*Ib) * 0.5773503f;
  float id= iAlpha * c + iBeta * s;
  float iq= -iAlpha * s + iBeta * c;

  //PI
  float errD = -id;
  float errQ = iqWanted - iq;
  float wElec = Pole_pairs * speed;
  float vdRaw = Kp_I * errD + integralD - wElec * L * iq;
  float vqRaw = Kp_I * errQ + integralQ + wElec * (L * id + Flux);

  float Vmax= busV * 0.5773503f;
  float Mag= hypotf(vdRaw, vqRaw);
  float Scale = (Mag>Vmax) ? Vmax/Mag :1.0f;
  float Vd= vdRaw * Scale, Vq= vqRaw * Scale;

  if (Scale > 0.999f || vdRaw * errD < 0) integralD += Ki_I * errD * dt;
  if (Scale > 0.999f || vqRaw * errQ < 0) integralQ += Ki_I * errQ * dt;

  //inverse clarke park
  float vAlpha = Vd * c - Vq * s;
  float vBeta = Vd * s - Vq *c;
  float Va = vAlpha;
  float Vb = -0.5f * vAlpha + 0.8660254f * vBeta;
  float Vc = -0.5f * vAlpha - 0.8660254f * vBeta;

  //svpwm
  float shift = -0.5f * (max(Va, max(Vb, Vc)) + min(Va, min(Vb, Vc)));
  float dutyA = constrain(0.5f + (Va + shift) / busV, 0.10f, 0.90f);
  float dutyB = constrain(0.5f + (Vb + shift) / busV, 0.10f, 0.90f);
  float dutyC = constrain(0.5f + (Vc + shift) / busV, 0.10f, 0.90f);
  writeDuty(dutyA, dutyB, dutyC);
}

void handleSerial(bool sensorsReady) {
  while (Serial.available()) {
    char key = tolower(Serial.read());
    switch (key) {
      case 'd':
        disarm();
        Serial.println("DISARMED");
        break;
      case 'a':
        if (Motor_en && sensorsReady && limits()) {
          writeDuty(0.5f, 0.5f, 0.5f);        // zero net voltage
          digitalWrite(Driver_off, LOW);
          armed = true;
          Serial.println("ARMED");
        } else {
          disarm();
          Serial.println("ARM REJECTED: outputs/calibration not ready");
        }
        break;
      case 'p':
        positionMode = !positionMode;
        Serial.println(positionMode ? "POSITION MODE" : "TORQUE MODE");
        break;
      case '+':
      case '-': {
        float dir = (key == '+') ? 1.0f : -1.0f;
        if (positionMode) posTarget = constrain(posTarget + dir * 5.0f * DEG_TO_RAD, -PI, PI);
        else torqueCmd = constrain(torqueCmd + dir * 0.001f, -Tmax, Tmax);
        break;
      }
      case '0':
        torqueCmd = 0.0f;
        posTarget = position;
        break;
    }
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(Driver_off, OUTPUT);
  digitalWrite(Driver_off, HIGH);             
  analogReadResolution(12);
  Wire.begin();                               
  Wire.setClock(400000);
  ledcAttach(PWM_A, 20000, 12);               
  ledcAttach(PWM_B, 20000, 12);
  ledcAttach(PWM_C, 20000, 12);
  writeDuty(0, 0, 0);
  lastLoop = micros();
  Serial.println("Simple ESP32 FOC demo; outputs disabled by default.");
  Serial.println("A arm | D disarm | P torque/position | +/- change | 0 reset command");
}

void loop() {
  unsigned long now = micros();
  unsigned long elapsed = now - lastLoop;
  if (elapsed < Loop) return;              
  lastLoop = now;
  float dt = elapsed * 1e-6f;

  bool ready = ReadSensors(dt);

  // Safety check
  if (armed && elapsed > Max_gap) {
    disarm();
    Serial.println("FAULT: loop timing");
  }
  if (armed && (!ready || !limits())) {
    disarm();
    Serial.println("FAULT: sensor, bus voltage, or current limit");
  }

  if (armed) runFOC(dt);
  handleSerial(ready);

  // Slow telemetry, only while disarmed so it can't disturb the fast loop.
  if (!armed && micros() - lastPrint >= 500000UL) {  // 500 ms
  lastPrint = micros();
  Serial.printf("raw mV A/B/bus: %.0f,%.0f,%.0f | angle: %.1f deg\n",
                rawA * 1000.0f, rawB * 1000.0f, rawBus * 1000.0f,
                angle * RAD_TO_DEG); 
}
}