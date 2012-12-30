
void convertGpsCoordinate(int32_t coord, char* p_direction, int* p_degMin, int* p_sek) {
    float coordFloat = abs(coord / 10000000.0); // Convert to decimal

    int degMin = coordFloat; // Degrees
    coordFloat -= degMin;
    coordFloat *= 60; // minutes
    int min = coordFloat;
    degMin = degMin * 100 + min;
    
    coordFloat -= min;
    
    *p_direction = (coord < 0.0);
    *p_degMin = degMin;
    *p_sek = coordFloat * 10000;
}


void setup() {
   Serial.begin(115200); 
   
   Serial.println("Starting");
   
   uint32_t coord = 480937416;
   char direction;
   int degMin;
   int sek;
   
   convertGpsCoordinate(coord, &direction, &degMin, &sek);
   
   Serial.print("Direction: ");
   Serial.println(direction);
   
   Serial.print("DegMin: ");
   Serial.println(degMin);
   
   Serial.print("Sek: ");
   Serial.println(sek);
}

void loop() {
    
}
