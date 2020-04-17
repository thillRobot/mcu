// Sample program to run stepper motor at full step
// Stephen Canfield Mar 14 2016
// Stepper connected to PB4-7

void setup() {
  // put your setup code here, to run once:
  DDRB = 0b11110111; // all PPRTB outputs
  DDRD = 0b11111111; // all PPRTB outputs
  Serial.begin(9600);
  Serial.println("program starting");

  PORTB=0b11111111;
  PORTD=0b11111111;

}

void loop() {
  // put your main code here, to run repeatedly:
int step_array[4] = {0b00101000,0b00100100,0b00010100,0b00011000};
int step_idx, step_cnt;

while(1){
  
  for(step_cnt = 0;step_cnt<=100; step_cnt++){
    step_idx = step_cnt%4;
    Serial.println(step_idx);
    PORTD |= step_array[step_idx];
    delay(20);
  }
   for(step_cnt = 100;step_cnt>=0; step_cnt--){
    step_idx = step_cnt%4;
    Serial.println(step_idx);
    PORTD |= step_array[step_idx];
    delay(20);
  }
 
}
      
 
}
