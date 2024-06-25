void setup() {
 Serial.begin(9600);
 
 pinMode(8,INPUT);

}

void loop() {

  int wartosc_cyfrowa = 0; 
  int wartosc_analogowa = 0;
 
  wartosc_cyfrowa = digitalRead(8);
  wartosc_analogowa = analogRead(A0);
 
  if (wartosc_cyfrowa== LOW)
  {
    Serial.println("Wykryto magnes!");
    
    
  }
 
  Serial.print("Wartosc analogowa: ");
  Serial.println(wartosc_analogowa);
 
  delay(300);
}
