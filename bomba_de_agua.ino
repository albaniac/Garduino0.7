//PINB |= 0b00000001

int bomba;
int output[] = {3, 4, 5, 6};
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  /* 
  pinMode(output[0], OUTPUT); //S0
  pinMode(output[1], OUTPUT); //S1
  pinMode(output[2], OUTPUT); //S2
  pinMode(output[3], OUTPUT); //S3
  */
}


void relevador(byte pin){
  for (int i =0;i<4;i++) { 
      if (bitRead(0, 0b00000000)==1){
          digitalWrite(output[i], HIGH);
          //digitalWrite(pin del relevador, HIGH)
          Serial.println("HIGH");
      }
      else{
          digitalWrite(output[i], LOW);
          //digitalWrite(pin del relevador, HIGH)
          Serial.println("LOW");
      }
  }
}

void loop() {
  // put your main code here, to run repeatedly: 
  for (byte binary =0;binary<=15; binary++) { 
       relevador(binary); 
       delay(2000);
  } 
}
