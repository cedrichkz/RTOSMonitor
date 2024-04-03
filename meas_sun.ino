void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

/*
Sun module: am-5608
Vopen = 3.3V
Vmin = 0.1V

Iopen = 36.0mA 
Pmax = 125mW （3.9V-32.0mA）
analogRead: 0-1023 range
*/
void loop() {
  // put your main code here, to run repeatedly:
  int rawValue = analogRead(A0);
  float voltage = rawValue * (3.3/ 1023); //scale voltage to range  of arduino, Vmin = 0.1
  Serial.print("Voltage solar panel: ");
  Serial.print(voltage); // 0.02 V delay
  Serial.println(" V");
  delay(1000);
}
