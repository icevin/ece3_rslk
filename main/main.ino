#include <ECE3.h>

const int left_nslp_pin=31; // nslp ==> awake & ready for PWM
const int left_dir_pin=29;
const int left_pwm_pin=40;

void setup() {
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


  
  IR.setSensorPins((const uint8_t[]) {65, 48, 64, 47, 52, 68, 53, 69}, 8);
  IR.setEmitterPins(45, 61);
  IR.setTimeout(2500);

}

void ECE3_read_IR(uint16_t * sensorValues){
  return IR.read(sensorValues);
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

void loop() {
  Serial.print("l_pos: ");
  Serial.print(l_tracker);
  Serial.print(" r_pos: ");
  Serial.print(r_tracker);
  Serial.println();
  delay(300);
}
