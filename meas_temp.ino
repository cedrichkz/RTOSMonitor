/*
Atmega32u4 Datasheet
ADC conversion registers:
ADC Multiplexer Selection Register – ADMUX
ADC Control and Status Register A – ADCSRA
The ADC Data Register – ADCL and ADCH - contains temp result
The sensor initial tolerance is large (±10°C), but its characteristic is linear. 
But an accuracy of about 2 degrees Celcius is possible if the gain and offset is measured.
This internal temperature can not be used to read the ambient temperature.
TODO: noise reduction mode
Before calibration room temperatur outputs - 17°C 
*/

void setup() 
{
  Serial.begin(9600);
  ADMUX = 0xC7; // 11 0 00111 ; turn on internal reference 2.56V, right-shift ADC buffer, ADC channel = internal temp sensor 100111 (bit 5 in ADCSRB)
  delay(10);  // wait a sec for the analog reference to stabilize
  ADCSRB = 0x20; // 0 0 1 0 0000 ; High speed mode off, ADMUX extra bit set temperature, n/a, Free Running mode.
  delay(10);  // wait for stabilization
}

void loop() 
{
  
  Serial.print("Temperature: ");
  Serial.println(readTemperature());
  /* T1 = 25
  // T2 = 27 we assume lineair working, so no gain error
  // ADC_T1 = 17.00 °C
  // ADC_T2 = 19°C after 6min 
  // calculateOffsetGain(25, 27, 17, 19);
  // offset : -8
  // gain: 1
  */
  SMCR = 0x01; // sleep mode = ADC Noise Reduction 001, Power Down 010, Power-Save 011
  SCMR |= _BV(SE); // sleep enable
  
  //ADC Noise Reduction
  delay(5000); // 5s
}

float readTemperature() 
{
  unsigned int wADC; //store initial temp
  float temp; //temp after correction
  ADCSRA |= _BV(ADSC); // start the conversion; _BV = 1<<(bit); Bit 6 – ADSC; 
  while (bit_is_set(ADCSRA, ADSC)); // ADSC is cleared when the conversion finishes, wait till conversion is done; first conversion 25 clk cycles after 13
  wADC = ADCL | (ADCH << 8); //result in ADCH & ADCL; shift ADCH & sum registers; ADC register; test ADCW
  temp = wADC;
  float T_off = -8; //offset
  float k = 1; //one point callibation, offset compensation
  temp = (wADC-273 - T_off)*k; 
  return temp; 
}

// improve accuracy
float averageTemperature()
{
  readTemperature(); // discard first sample (never hurts to be safe)
  float averageTemp; // create a float to hold running average
  for (int i = 1; i < 1000; i++) // start at 1 so we dont divide by 0
    averageTemp += ((readTemperature() - averageTemp)/(float)i); // get next sample, calculate running average

  return averageTemp; // return average temperature reading
}

float calculateOffsetGain(float T1, float T2, float ADC_T1, float ADC_T2){

    //ADC_T in °Cs
    float m = (ADC_T2 - ADC_T1) / (T2 - T1);
    float T_off = m*(-T1) + ADC_T1; // OFSETT
    float k = T2 / (ADC_T2 - T_off); // GAIN
    Serial.print("offset: ");
    Serial.println(T_off);
    Serial.print("gain: ");
    Serial.println(k);
}