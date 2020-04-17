

Using the analog to digital converter with atmel 2560 (arduino mega)
Tristan Hill

 1) choose voltage ref -> hardwire pin AREF to 5v 
 
 2) set reference and channels (initialize)
        
        
        ADMUX – ADC Multiplexer Selection Register
        bit     REFS1       REFS0       ADLAR       MUX4    MUX3    MUX2    MUX1    MUX0
        val     0           0           0           0       0       0       0       0   
        means   |----ext VREF---|       R adj       |----------channel ADC0------------|
        >> ADMUX = B00000000;
        
        ADCSRB – ADC Control and Status Register B
        bit     ----        ACME        ----    ----    MUX5    ADTS2 ADTS1 ADTS0 
        val     ----        0           ----    ----    0       0     0     0 
        means   ----  analog compare    ----    ----    ADC0    |---auto trig---|
        >> ADCSRB = B00000000;
        
        ADCSRA – ADC Control and Status Register A
        bit     ADEN        ADSC        ADATE           ADIF           ADIE         ADPS2   ADPS1   ADPS0
        val     1           1           0               -               -             0     0       0
        mean    enable      start       auto trig   interrupt flag  interrupt en.  |-- clock Prescale =2 --|
        >>ADSCRA=B11000000;
        
 3) read conversion 
    
    
        >>val=ADCH*256+ADCL;