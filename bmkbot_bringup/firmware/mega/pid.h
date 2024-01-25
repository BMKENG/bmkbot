// 기어비 * 채배 * 엔코더 개수 * 틱수 
// 30 * 4 * 11
#define rotation 1404 

//for Encoder pin define
#define ENC1_CHA    2  //INT.5
#define ENC1_CHB    3

#define ENC2_CHA    18
#define ENC2_CHB    19


//for Motor I/O pin define
#define M1_DIR   8
#define M2_DIR   9
 
#define M1_PWM      6
#define M2_PWM      7

#define Kp1 3.// P-gain 
#define Ki1 .5// i-gain  
#define Kd1 .03// d-gain  

#define Kp2 3.// P-gain 
#define Ki2 .5// i-gain  
#define Kd2 .03// d-gain 

//Variables
//Encoder value
long    e1cnt = 0;
long    e1cnt_k = 0, e1cnt_k_1 = 0, d_e1cnt = 0;
long    e2cnt = 0;
long    e2cnt_k = 0, e2cnt_k_1 = 0, d_e2cnt = 0;

//motor value
float   m1_speed = 0;
float   m1turn = 0;
float   m2_speed = 0;
float   m2turn = 0;

//for motor control variable
//motor1
float   m1_ref_spd = 0;
float   m1_err_spd = 0;
int     m1_ctrl_u = 0;
int     m1_ipwm_u = 0;
//motor2
float   m2_ref_spd = 0;
float   m2_err_spd = 0;
int     m2_ctrl_u = 0;
int     m2_ipwm_u = 0;

//-----------------------------------//
float   M1err_P = 0;
float   M1err_I = 0;
float   M1err_D = 0;
float   M1err_B = 0;
long    M1PID_val = 0;

float   M2err_P = 0;
float   M2err_I = 0;
float   M2err_D = 0;
float   M2err_B = 0;
long    M2PID_val = 0;

unsigned long M1t_k = 0, M1t_k_1 = 0, M1dt = 0;
unsigned long M2t_k = 0, M2t_k_1 = 0, M2dt = 0;

void Enc1chA_ISR();
void Enc2chA_ISR();
void Enc1chB_ISR();
void Enc2chB_ISR();


void M1vel_PID(int m1_ref_spd);
void M2vel_PID(int m2_ref_spd);

void M1RPM();
void M2RPM();

void M1doMotor(bool dir, long vel);
void M2doMotor(bool dir, long vel);

void setting(){
    // put your setup code here, to run once:
    pinMode(ENC1_CHA, INPUT_PULLUP);
    pinMode(ENC1_CHB, INPUT_PULLUP);
    pinMode(ENC2_CHA, INPUT_PULLUP);
    pinMode(ENC2_CHB, INPUT_PULLUP);

    pinMode(M1_DIR, OUTPUT);
    pinMode(M2_DIR, OUTPUT);

    pinMode(M1_PWM, OUTPUT);
    pinMode(M2_PWM, OUTPUT);
    //encoder 값을 받기 위한 외부 인터럽트 설정
    attachInterrupt(digitalPinToInterrupt(ENC1_CHA), Enc1chA_ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC2_CHA), Enc2chA_ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC1_CHB), Enc1chB_ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC2_CHB), Enc2chB_ISR, CHANGE);

    analogWrite(M1_PWM, 0);
    analogWrite(M2_PWM, 0);
}

void t2_ISR(){
    M1vel_PID(m1_ref_spd);
    M2vel_PID(m2_ref_spd);
}


void M1vel_PID(int m1_ref_spd){
    M1RPM();
    m1_err_spd = m1_speed - m1_ref_spd;
    M1err_P = m1_err_spd;
    M1err_I += M1err_P;
    M1err_D = M1err_B - M1err_P;
    M1err_B = M1err_P;

    M1PID_val=((M1err_P * Kp1) + (M1err_I * Ki1) + (M1err_D * Kd1));
    m1_ctrl_u = M1PID_val;
    // 속도 오차가 생기면  
    M1doMotor(m1_ctrl_u >= 0 ? HIGH : LOW, m1_ctrl_u);
    Serial.print("(");
    Serial.print(m1_speed);
    Serial.print(", ");
}


void M2vel_PID(int m2_ref_spd){

    // if(Serial.available()){
    //   m2_ref_spd = Serial.parseFloat();
    // }

    M2RPM();
    m2_err_spd = m2_speed - m2_ref_spd;
    M2err_P = m2_err_spd;
    M2err_I += M2err_P;
    M2err_D = M2err_B - M2err_P;
    M2err_B = M2err_P;

    M2PID_val=((M2err_P * Kp2) + (M2err_I * Ki2) + (M2err_D * Kd2));
    m2_ctrl_u = M2PID_val;
    // 속도 오차가 생기면  
    M2doMotor(m2_ctrl_u >= 0 ? HIGH : LOW, m2_ctrl_u);
    // Serial.print("m2 speed : ");
    Serial.print(m2_speed);
    Serial.println(")");   
}

void M1RPM(){
  // if(Serial.available()){
  // m1_ref_spd = Serial.parseFloat();
  // }
  
  M1t_k = millis();//시간측정 
  M1dt = M1t_k - M1t_k_1;//시간변화율
  if(M1dt >= 10){
    e1cnt_k = e1cnt; 
    d_e1cnt = e1cnt_k - e1cnt_k_1;// 엔코더 값
    m1_speed = d_e1cnt * 60000/rotation;
    m1_speed /= (signed long)M1dt;// 거 = 속 시 RPM 값
    M1t_k_1 = M1t_k;//시간 초기화
    e1cnt_k_1 = e1cnt_k;//엔코더 초기화
  }
}

void M2RPM(){
  
  M2t_k = millis();//시간측정 
  M2dt = M2t_k - M2t_k_1;//시간변화율
  if(M2dt >= 10){
    e2cnt_k = e2cnt; 
    d_e2cnt = e2cnt_k - e2cnt_k_1;// 엔코더 값
    // ms -> min 변환
    m2_speed = d_e2cnt * 60000/rotation;
    m2_speed /= (signed long)M2dt;// 거 = 속 시 RPM 값
    M2t_k_1 = M2t_k;//시간 초기화
    e2cnt_k_1 = e2cnt_k;//엔코더 초기화
  }
}

void M1doMotor(bool dir, long vel){
  // dir = 1 : cw, dir = 0 : ccw
  digitalWrite(M1_DIR, dir);

  vel = abs(vel);
  m1_ipwm_u = vel > 255 ? 255 : vel; 
  analogWrite(M1_PWM, m1_ipwm_u);
}

void M2doMotor(bool dir, long vel){
  // dir = 1 : cw, dir = 0 : ccw
  digitalWrite(M2_DIR, dir);
  vel = abs(vel);
  m2_ipwm_u = vel > 255 ? 255 : vel; 
  analogWrite(M2_PWM, m2_ipwm_u);
}


void Enc1chA_ISR(){  e1cnt += (digitalRead(ENC1_CHA)==digitalRead(ENC1_CHB))?-1:1;}
void Enc2chA_ISR(){  e2cnt += (digitalRead(ENC2_CHA)==digitalRead(ENC2_CHB))?1:-1;}
void Enc1chB_ISR(){  e1cnt += (digitalRead(ENC1_CHA)==digitalRead(ENC1_CHB))?1:-1;}
void Enc2chB_ISR(){  e2cnt += (digitalRead(ENC2_CHA)==digitalRead(ENC2_CHB))?-1:1;}
