// Sample program to run stepper motor at full step
// Stephen Canfield Mar 14 2016
// Stepper connected to PB4-7
// Updated by TWH - 04-24-2018

void setup() {
  // put your setup code here, to run once:
  DDRA = 0xFF; // all PPRTB outputs
  Serial.begin(9600);
  Serial.println("program starting");

}

void loop() {
  // put your main code here, to run repeatedly:
int step_array[4] = {0b00111010,0b00111001,0b00110101,0b00110110};
int step_idx, step_cnt;

while(1){
  for(step_cnt = 0;step_cnt<=100; step_cnt++){
    step_idx = step_cnt%4;
    Serial.println(step_idx);
    PORTA = step_array[step_idx];
    delay(20);
  }
   for(step_cnt = 100;step_cnt>=0; step_cnt--){
    step_idx = step_cnt%4;
    Serial.println(step_idx);
    PORTA = step_array[step_idx];
    delay(20);
  }
 
}
      
 
}
