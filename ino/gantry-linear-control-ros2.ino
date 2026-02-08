
// Works on ESP8266/ESP32 with Serial

bool busy = false;
float x=0, y=0, z=0;
String lastErr = "";

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("OK");
}

void loop() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) return;

    if (line == "PING") {
      Serial.println("OK");
    } else if (line == "HOME") {
      // start homing routine
      busy = true;
      Serial.println("OK");
      // ... do homing ...
      x=0; y=0; z=0;
      busy = false;
    } else if (line == "STOP") {
      // emergency stop
      busy = false;
      Serial.println("OK");
    } else if (line.startsWith("G0")) {
      // parse: G0 X.. Y.. Z.. F..
      float nx=x, ny=y, nz=z, f=10;

      // very simple parsing:
      int ix = line.indexOf('X');
      int iy = line.indexOf('Y');
      int iz = line.indexOf('Z');
      int iF = line.indexOf('F');
      if (ix>=0) nx = line.substring(ix+1).toFloat();
      if (iy>=0) ny = line.substring(iy+1).toFloat();
      if (iz>=0) nz = line.substring(iz+1).toFloat();
      if (iF>=0) f  = line.substring(iF+1).toFloat();

      Serial.println("OK");
      busy = true;

      // ... start motion to nx,ny,nz at feed f ...
      // In real code, stepper ISR updates position

      x = nx; y = ny; z = nz;
      busy = false;
    } else {
      lastErr = "unknown_cmd";
      Serial.print("ERR ");
      Serial.println(lastErr);
    }
  }

  // Periodic status
  static unsigned long t0 = 0;
  if (millis() - t0 > 50) {
    t0 = millis();
    Serial.print("BUSY "); Serial.println(busy ? "1" : "0");
    Serial.print("POS X"); Serial.print(x,3);
    Serial.print(" Y"); Serial.print(y,3);
    Serial.print(" Z"); Serial.println(z,3);
  }
}
