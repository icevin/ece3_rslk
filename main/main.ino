#include <ECE3.h>

const int left_nslp_pin=31; // nslp ==> awake & ready for PWM
const int left_dir_pin=29;
const int left_pwm_pin=40;


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
  
  Serial.begin(9600);
  Serial.println("Basic Encoder Test:");

  pinMode(left_nslp_pin,OUTPUT);
  pinMode(left_dir_pin,OUTPUT);
  pinMode(left_pwm_pin,OUTPUT);

  digitalWrite(left_dir_pin,LOW);
  digitalWrite(left_nslp_pin,HIGH);

}

void l_button() {
    int leftSpd = 70;
    analogWrite(left_pwm_pin, leftSpd);
}


#define L_PIN1 P5_2
#define L_PIN2 P10_5

bool l_newState1 = 0;
bool l_newState2 = 0;

bool l_oldState1 = 0;
bool l_oldState2 = 0;


long l_tracker = 0;
long l_dir = 0;

void ENC_L() {
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

uint8_t flag = 0;

int calc_error() {
  long error = 0;
  uint8_t counter = 0;
  for (unsigned char i = 0; i < 8; i++) {
    if (sensorValues[i] > 2000) {
      counter++;
    }
    error += (sensorValues[i] - offset[i]) * weights[i];
  }
  if (counter >= 6) {
    flag = 1;
  }
  return error/4;
}

void loop() {
//  Serial.print("l_pos: ");
//  Serial.print(l_tracker);
//  Serial.print(" r_pos: ");
//  Serial.print(r_tracker);
  ECE3_read_IR(sensorValues);
  for (unsigned char i = 0; i < 8; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t'); // tab to format the raw data into columns in the Serial monitor
  }
  Serial.println();
  Serial.print("error: ");
  Serial.print(calc_error());
  Serial.println();  
  Serial.println();

  delay(50);
}
