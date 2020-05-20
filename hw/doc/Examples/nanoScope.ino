void setup() {
  
  //set ADC prescaler to 128, 9.6kHz sampling rate
  ADCSRA &= ~(bit (ADPS0) | bit (ADPS1) | bit (ADPS2)); // clear prescaler bits
  ADCSRA |= bit (ADPS0) | bit (ADPS1) | bit (ADPS2);   //   1 1 1 -> 128
  
  //initialize serial
  Serial.begin(115200);
}

// this gives a sampling rate of 300Hz
#define AVG_SAMPLES 32 

void loop() {
  int val = 0;
  for (int i = 0; i < AVG_SAMPLES; ++i){
    val += analogRead(A0);
  }
  val /= AVG_SAMPLES;
  
  Serial.println(val);
}
