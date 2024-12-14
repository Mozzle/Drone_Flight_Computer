
void setup() {
  BMP280_Begin();
}

void loop() {
  BMP280_ReadAltitude(1018.3); // Will likely need some way to load hPa for each flight

  delay(25);
}