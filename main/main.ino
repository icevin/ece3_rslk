#include <ECE3.h>

const int left_nslp_pin=31; // nslp ==> awake & ready for PWM
const int left_dir_pin=29;
const int left_pwm_pin=40;

const int right_nslp_pin=11;
const int right_dir_pin=30;
const int right_pwm_pin=39;


uint16_t sensorValues[8];

void setup() {
  
  ECE3_Init();
  
  pinMode(P5_2, INPUT);
  pinMode(P5_0, INPUT);
  pinMode(P10_4, INPUT);
  pinMode(P10_5, INPUT);
  attachInterrupt(P5_2, ENC_L, FALLING);
  attachInterrupt(P5_0, ENC_R, FALLING);

  pinMode(PUSH2, INPUT_PULLUP);
  attachInterrupt(PUSH2, l_button, FALLING);

  pinMode(PUSH1, INPUT_PULLUP);
  attachInterrupt(PUSH1, r_button, FALLING);
  
  Serial.begin(9600);
  Serial.println("-----STARTUP-----");
  delay(2500);

  pinMode(left_nslp_pin,OUTPUT);
  pinMode(left_dir_pin,OUTPUT);
  pinMode(left_pwm_pin,OUTPUT);

  pinMode(right_nslp_pin,OUTPUT);
  pinMode(right_dir_pin,OUTPUT);
  pinMode(right_pwm_pin,OUTPUT);


  digitalWrite(left_dir_pin,LOW);
  digitalWrite(right_dir_pin,LOW);
  
  digitalWrite(left_nslp_pin,HIGH);
  digitalWrite(right_nslp_pin,HIGH);
}

void setL(int speed) {
  if (speed > 127)
    speed = 127;
  if (speed < -127)
    speed = -127;
  if (speed == 0) {
    analogWrite(left_pwm_pin, 0);
    return;
  }
  
  if (speed > 0)
      digitalWrite(left_dir_pin,LOW);
  else
      digitalWrite(left_dir_pin,HIGH);
  analogWrite(left_pwm_pin, abs(speed));
}

void setR(int speed) {
  if (speed > 127)
    speed = 127;
  if (speed < -127)
    speed = -127;
  if (speed == 0) {
    analogWrite(right_pwm_pin, 0);
    return;
  }
  
  if (speed > 0)
      digitalWrite(right_dir_pin,LOW);
  else
      digitalWrite(right_dir_pin,HIGH);
  analogWrite(right_pwm_pin, abs(speed));
}

uint8_t flag = 0;


#define L_PIN1 P5_2
#define L_PIN2 P10_5

bool l_newState1 = 0;
bool l_newState2 = 0;

bool l_oldState1 = 0;
bool l_oldState2 = 0;


long l_tracker = 0;
long l_dir = 0;

void ENC_L() {
    // Serial.println("---ENC_L---");
    l_newState1 = digitalRead(L_PIN1);
    l_newState2 = digitalRead(L_PIN2);
    if (l_newState1 == l_oldState1 && l_newState2 == l_oldState2) {
      goto end;
    }
    if (l_newState2 == 0 && l_newState1 == 0 && l_oldState2 == 0 && l_oldState1 == 1) {
      l_dir++; goto end;
    }
    if (l_newState2 == 0 && l_newState1 == 0 && l_oldState2 == 1 && l_oldState1 == 0) {
      l_dir--; goto end;
    }    
    if (l_newState2 == 0 && l_newState1 == 0 && l_oldState2 == 1 && l_oldState1 == 1) {
      l_dir+= 2; goto end;
    }
    if (l_newState2 == 0 && l_newState1 == 1 && l_oldState2 == 0 && l_oldState1 == 0) {
      l_dir--; goto end;
    }
    if (l_newState2 == 0 && l_newState1 == 1 && l_oldState2 == 1 && l_oldState1 == 0) {
      l_dir+= 2; goto end;
    }
    if (l_newState2 == 0 && l_newState1 == 1 && l_oldState2 == 1 && l_oldState1 == 1) {
      l_dir++; goto end;
    }
    if (l_newState2 == 1 && l_newState1 == 0 && l_oldState2 == 0 && l_oldState1 == 0) {
      l_dir++; goto end;
    }
    if (l_newState2 == 1 && l_newState1 == 0 && l_oldState2 == 0 && l_oldState1 == 1) {
      l_dir-= 2; goto end;
    }
    if (l_newState2 == 1 && l_newState1 == 0 && l_oldState2 == 1 && l_oldState1 == 1) {
      l_dir--; goto end;
    }   
    if (l_newState2 == 1 && l_newState1 == 1 && l_oldState2 == 0 && l_oldState1 == 0) {
      l_dir+=2; goto end;
    }
    if (l_newState2 == 1 && l_newState1 == 1 && l_oldState2 == 0 && l_oldState1 == 1) {
      l_dir--; goto end;
    }
    if (l_newState2 == 1 && l_newState1 == 1 && l_oldState2 == 1 && l_oldState1 == 0) {
      l_dir++; goto end;
    }
    end:
    l_oldState1 = l_newState1;
    l_oldState2 = l_newState2;

    if(l_dir == 1)
      l_tracker++;
    else
      l_tracker--;
}


#define R_PIN1 P5_0
#define R_PIN2 P10_4

bool r_newState1 = 0;
bool r_newState2 = 0;

bool r_oldState1 = 0;
bool r_oldState2 = 0;


long r_tracker = 0;
long r_dir = 0;
void ENC_R() {
    // Serial.println("---ENC_R---");
    r_newState1 = digitalRead(R_PIN1);
    r_newState2 = digitalRead(R_PIN2);
    if (r_newState1 == r_oldState1 && r_newState2 == r_oldState2) {
      goto end;
    }
    if (r_newState2 == 0 && r_newState1 == 0 && r_oldState2 == 0 && r_oldState1 == 1) {
      r_dir++; goto end;
    }
    if (r_newState2 == 0 && r_newState1 == 0 && r_oldState2 == 1 && r_oldState1 == 0) {
      r_dir--; goto end;
    }    
    if (r_newState2 == 0 && r_newState1 == 0 && r_oldState2 == 1 && r_oldState1 == 1) {
      r_dir+= 2; goto end;
    }
    if (r_newState2 == 0 && r_newState1 == 1 && r_oldState2 == 0 && r_oldState1 == 0) {
      r_dir--; goto end;
    }
    if (r_newState2 == 0 && r_newState1 == 1 && r_oldState2 == 1 && r_oldState1 == 0) {
      r_dir+= 2; goto end;
    }
    if (r_newState2 == 0 && r_newState1 == 1 && r_oldState2 == 1 && r_oldState1 == 1) {
      r_dir++; goto end;
    }
    if (r_newState2 == 1 && r_newState1 == 0 && r_oldState2 == 0 && r_oldState1 == 0) {
      r_dir++; goto end;
    }
    if (r_newState2 == 1 && r_newState1 == 0 && r_oldState2 == 0 && r_oldState1 == 1) {
      r_dir-= 2; goto end;
    }
    if (r_newState2 == 1 && r_newState1 == 0 && r_oldState2 == 1 && r_oldState1 == 1) {
      r_dir--; goto end;
    }   
    if (r_newState2 == 1 && r_newState1 == 1 && r_oldState2 == 0 && r_oldState1 == 0) {
      r_dir+=2; goto end;
    }
    if (r_newState2 == 1 && r_newState1 == 1 && r_oldState2 == 0 && r_oldState1 == 1) {
      r_dir--; goto end;
    }
    if (r_newState2 == 1 && r_newState1 == 1 && r_oldState2 == 1 && r_oldState1 == 0) {
      r_dir++; goto end;
    }
    end:
    r_oldState1 = r_newState1;
    r_oldState2 = r_newState2;

    if(r_dir == 1)
      r_tracker++;
    else
      r_tracker--;
}


const int weights[] = {8, 4, 2, 1, -1, -2, -4, -8};
const int offset[] = {483, 460, 553, 437, 483, 529, 506, 530};

int lines = 0;
int nextEnd = 0;

int calc_error() {
  long error = 0;
  uint8_t counter = 0;
  for (unsigned char i = 0; i < 8; i++) {
    if (sensorValues[i] > 2100) {
      counter++;
    }
    error += (sensorValues[i] - offset[i]) * weights[i];
  }
//  Serial.print("Counter: ");
//  Serial.print(counter);
//  Serial.println();
  if (counter >= 5) {
    if(lines < 4 && nextEnd == 0) {
      turnCCW();
      nextEnd = 1;
    } else if (nextEnd == 1 || lines > 40) {
      flag = 1;
    }
    lines++;
  }
  return error/4;
}



const int max_drive = 100;
const int max_motor = 100;
#define PID_INTEGRAL_LIMIT 40

float pid_Kp = 0.2;
float pid_Ki = 0.001;
float pid_Kd = 0.6;

float pidCurrError = 0;
float pidLastError = 0;
float pidIntegral = 0;
float pidDerivative = 0;
float pidDrive = 0;
void pidController(int pidCurrError)
{
    pidCurrError = calc_error();
    
    if(pid_Ki!=0)
    {
      if(fabs(pidCurrError) < PID_INTEGRAL_LIMIT)
        pidIntegral = pidIntegral + pidCurrError;
      else
        pidIntegral = 0;
    }
    else
      pidIntegral = 0;
      
    if(pidCurrError !=0)
    {
      pidDerivative = pidCurrError - pidLastError;
      pidLastError = pidCurrError;
    }
    else
    {
      pidDerivative = 0;
    }
    
    if(pidCurrError < 4.0)
    {
      pidIntegral = 0;
    }
    pidDrive+= (pid_Kp*pidCurrError) + (pid_Ki*pidIntegral) + (pid_Kd * pidDerivative);

    if(pidDrive > max_drive)
      pidDrive = max_drive;
    if(pidDrive < -max_drive)
      pidDrive = -max_drive;

    pidLastError = pidCurrError;
}

bool turn_done = true;
int l_dest = 380;
int r_dest = -380;

long l_err = 0;
long r_err = 0;

long l_lastErr = 0;
long r_lastErr = 0;

long l_pid = 0;
long r_pid = 0;

long l_d = 0;
long r_d = 0;

long l_int = 0;
long r_int = 0;

float d_kp = 0.4;
float d_ki = 0.1;
float d_kd = 0.2;

void spin180() {
    if(!turn_done) {
      Serial.print("l_pos: ");
      Serial.print(l_tracker);
      Serial.print(" r_pos: ");
      Serial.print(r_tracker);
      Serial.print(" lerr: ");
      Serial.print(l_err);
      Serial.println();

      l_err = l_dest - l_tracker;
      r_err = r_dest - r_tracker;

      if (l_err != 0) {
        l_d = l_err - l_lastErr;
      } else {
        l_d = 0;
      }
      
      if (r_err != 0) {
        r_d = r_err - r_lastErr;
      } else {
        r_d = 0;
      }

      if(abs(l_err) < 80)
        l_int += l_err;
      else
        l_int = 0;
        
      if(abs(r_err) < 80)
        r_int += r_err;
      else
        r_int = 0;
      l_pid = (d_kp * l_err) + (d_kd * l_d) + (d_ki * l_int);
      r_pid = (d_kp * r_err) + (d_kd * r_d) + (d_ki * r_int);

      setL(l_pid);
      setR(r_pid);

      if(abs(l_err) < 20 && abs(r_err) < 20) {
         turn_done = 1;
         setL(80);
         setR(80);
         delay(200);
         setL(0);
         setR(0);
         flag = 0;
      }
      l_lastErr = l_err;
      r_lastErr = r_err;
      delay(5);
    }
}

void turnCW() {
    l_tracker = 0;
    r_tracker = 0;
    turn_done = false;
    l_dest = 380;
    r_dest = -380;
    spin180();
}

void turnCCW() {
    l_tracker = 0;
    r_tracker = 0;
    turn_done = false;
    l_dest = -380;
    r_dest = 380;
}

int allstop = 0;
int turns_done = 0;

void loop() {
    ECE3_read_IR(sensorValues);
    int error = calc_error();

    if(allstop == 1) {
      setL(0);
      setR(0);
      return;
    }

    if(flag == 1) {
      if(lines >= 5) {
        allstop = 1;
        flag = 0;
      }
    }

    if(!turn_done) {
      spin180();
      Serial.println("///////////////////////////////////////////////");
    } else if(!flag && !allstop) {
      pidController(error);
      if(pidDrive < -10) {
        setL(max_motor + pidDrive);
        setR(max_motor);
      } else if (pidDrive > 10) {
        setL(max_motor);
        setR(max_motor - pidDrive);
      } else {
        setL(max_motor);
        setR(max_motor);
      }
    } else {
      setL(0);
      setR(0);
    }
//
//    Serial.print("lines: ");
//    Serial.print(lines);
//    Serial.print("flag: ");
//    Serial.print(flag);
//    Serial.print("turn done?");
//    Serial.print(turn_done);
//    Serial.println();
    Serial.println();
    Serial.print("error: ");
    Serial.print(error);
    Serial.print(" drive: ");
    Serial.print(pidDrive);
    Serial.println();  
    Serial.println();

  
//  Serial.print("l_pos: ");
//  Serial.print(l_tracker);
//  Serial.print(" r_pos: ");
//  Serial.print(r_tracker);
//  Serial.println();

//  for (unsigned char i = 0; i < 8; i++)
//  {
//    Serial.print(sensorValues[i]);
//    Serial.print('\t'); // tab to format the raw data into columns in the Serial monitor
//  }

    delay(50);
}

void l_button() {
    flag = 0;
    allstop = 0;
    lines = 0;
    turns_done == 0;
    turn_done == true;
    nextEnd = 0;
}

void r_button() {
//    int rightSpd = 127;
//    if (Serial.available() > 0) {
//      // read the incoming byte:
//      rightSpd = Serial.read();
//  
//      // say what you got:
//      Serial.print("I received: ");
//      Serial.println(rightSpd, DEC);
//    }
//    analogWrite(right_pwm_pin, rightSpd);
}
