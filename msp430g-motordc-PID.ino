// Controle de velocidade de motor DC com Controlador PID 

#define sensor  10  //p2.2
#define pwm_in     14  //p1.6


int pulsos=0;
int rpm=0;
long PWM=255;
int rpm_set=0;
unsigned long deltaTempo=0;
unsigned long tempo=0;

// variaveis controlador PID
float Kp =0.44;
float Ki =0.00128;
float Kd = 39.43;
float erro, int_erro = 0;
float rpm_ant=0;
float dif_rpm=0;


float alfa = 0.973292;
int rpm_med=0;


void contador() {
  pulsos++;   //incrementa contador
   
}

void setup() {
  pinMode(sensor,INPUT);
  pinMode(pwm_in, OUTPUT);
  Serial.begin(115200);

  analogWrite(pwm_in,PWM);
  delay(1000);
  attachInterrupt(digitalPinToInterrupt(sensor), contador, RISING);   
  tempo=micros();
  
}

void loop() {
    
    if(Serial.available()>0){
        rpm_set = Serial.parseInt(); // permite valores ate 32767
        if (rpm_set>3400){
          rpm_set=3400;
          Serial.println("Insira valores RPM <= 3800 !!!!");
        }
        if (rpm_set<0){
          rpm_set=0;
        }

    }

    
    if (pulsos>=23) { // a cada 3 voltas
      detachInterrupt(digitalPinToInterrupt(sensor)); //desabilita interrupcao durante o calculo    
       deltaTempo=micros()-tempo;      
       rpm = 180000000.0/(deltaTempo); // 1rotação*1000*60/deltaTempo, sendo deltaTempo o intervalo de tempo entre o inicio e o fim da aquisição de dados      
      
      Serial.print(rpm_set);
      Serial.print("\t");      
      Serial.print(PWM);      
      Serial.print("\t");
      Serial.println(rpm_med, DEC);     
       
      pulsos=0;
      tempo= micros();
      
    }
    rpm_med=((alfa*rpm_med)+(0.026708)*rpm); // media movel exponencial de 36 periodos  
   
    controle();     // chama funcao de controle
      
    attachInterrupt(digitalPinToInterrupt(sensor), contador, RISING); 
        
}

int   controle(){

  erro = rpm_set-rpm_med; // calculo do erro entre SP e PV
  if(erro>(rpm_set*0.2) || erro<(-rpm_set*0.2)){
    int_erro=0;            //reset do erro integrativo (anti-windup)
    
  }else{
     int_erro +=erro;    ////habilita o erro integral se o erro for menor que 10% de SetP 
                         // (em modulo) 
    
  }
  dif_rpm=rpm_med-rpm_ant;   // calcula derivada do PV
  
  PWM = erro*Kp + int_erro*Ki - dif_rpm*Kd;
  
    if(PWM > 255){
          PWM=255;
          
    }if(PWM < 0){
         PWM=0;
         
    }
     
  
  rpm_ant= rpm_med;  // armazena em PV_ant o valor atual de PV
  analogWrite(pwm_in,255-PWM);

}
