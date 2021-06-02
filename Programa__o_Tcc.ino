// Controle de Motor de Passo com Modulo driver A4988
//
// Modulo A4988 / Motor de Passo Bipolar / Arduino Nano / IDE 1.8.5
// Gustavo Murta 29/mar/2018

// Definições das Portas Digitais do Arduino
#include<Wire.h>
#define A_R 16384.0 // 32768/2
#define G_R 131.0 // 32768/250
//Conversion de radianes a grados 180/PI
#define RAD_A_DEG = 57.295779


int RST = 8;              // Porta digital D08 - reset do A4988
int SLP = 9;              // Porta digital D09 - dormir (sleep) A4988
int ENA = 7;              // Porta digital D07 - ativa (enable) A4988
int MS1 = 4;              // Porta digital D04 - MS1 do A4988
int MS2 = 5;              // Porta digital D05 - MS2 do A4988
int MS3 = 6;              // Porta digital D06 - MS3 do A4988
int DIR = 3;              // Porta digital D03 - direção (direction) do A4988
int STP = 2;              // Porta digital D02 - passo(step) do A4988

// Definições de variaveis para controle do motor de passo

int MeioPeriodo = 1000;   // MeioPeriodo do pulso STEP em microsegundos F= 1/T = 1/2000 uS = 500 Hz
float PPS = 0;            // Pulsos por segundo
boolean sentido = true;   // Variavel de sentido
long PPR = 200;           // Número de passos por volta
long Pulsos;              // Pulsos para o driver do motor
float Voltas;             // voltas do motor
float RPM;                // Rotacoes por minuto

// Definições de Variaveis para o controle do sistema

int lLDRPin = A0;         // Ldr no pino A0
int oLDRPin = A1;         // Ldr no pino A1
int lesteLDR = 0;         // Variavel para armazenar as leituras dos LDRs
int oesteLDR = 0;         // Variavel para armazenar as leituras dos LDRs
int differenca = 0;       // Variavel para comparação dos dois LDRs
int error = 10;           // Variavel para calibração dos dois LDRs
int lmt_inicial = 11;     // Fim de curso inicial
int lmt_final = 10;       // Fim de Curso Final

////////////////////////Variables///////////////////////
int Read = 0;
float distance = 0.0;
float elapsedTime, time, timePrev;        //Variables for time control
float distance_previous_error, distance_error;
int period = 50;  //Refresh rate period of the loop is 50ms
///////////////////////////////////////////////////////

//Endereco I2C do MPU6050
const int MPU=0x68;  
//Variaveis para armazenar valores dos sensores
int AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
//Angulos
float Acc[2];
float Gy[3];
float Angle[3];
String valores;

long tiempo_prev;
float dt;


///////////////////PID constants///////////////////////
float kp=0.01; //
float ki=0; //
float kd=0; //
float distance_setpoint = 0;           //Should be the distance from sensor to the middle of the bar in mm
float PID_p, PID_i, PID_d, PID_total;
///////////////////////////////////////////////////////
void setup()
{
  Serial.begin(9600);

  DDRD = DDRD | B11111100;  // Configura Portas D02 até D07 como saída
  disa_A4988();             // Desativa o chip A4988

  DDRB = 0x0F;              // Configura Portas D08,D09,D10 e D11 como saída
  digitalWrite(SLP, HIGH);  // Desativa modo sleep do A4988
  rst_A4988();              // Reseta o chip A4988
  ena_A4988();              // Ativa o chip A4988
  pinMode(lmt_inicial, INPUT);
  pinMode(lmt_final, INPUT);
  
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B); 
   
  //Inicializa o MPU-6050
  Wire.write(0); 
  Wire.endTransmission(true);
  Serial.println("CLEARDATA");
  Serial.println("LABEL, Data, Hora, Angulo x, Angulo y");
  Serial.println("RESETTIMER");
}

void rst_A4988()
{
  digitalWrite(RST, LOW);     // Realiza o reset do A4988
  delay (10);                 // Atraso de 10 milisegundos
  digitalWrite(RST, HIGH);    // Libera o reset do A4988
  delay (10);                 // Atraso de 10 milisegundos
}

void disa_A4988()
{
  digitalWrite(ENA, HIGH);    // Desativa o chip A4988
  delay (10);                 // Atraso de 10 milisegundos
}

void ena_A4988()
{
  digitalWrite(ENA, LOW);     // Ativa o chip A4988
  delay (10);                 // Atraso de 10 milisegundos
}

void HOR()                      // Configura o sentido de rotação do Motor
{
  Serial.println(" Sentido Horario ");
  digitalWrite(DIR, HIGH);      // Configura o sentido HORÁRIO
}

void AHR()                      // Configura o sentido de rotação do Motor
{
  Serial.println(" Sentido anti-Horario ");
  digitalWrite(DIR, LOW);       // Configura o sentido ANTI-HORÁRIO
}

void PASSO()                         // Pulso do passo do Motor
{
  digitalWrite(STP, LOW);            // Pulso nível baixo
  delayMicroseconds (MeioPeriodo);   // MeioPeriodo de X microsegundos
  digitalWrite(STP, HIGH);           // Pulso nível alto
  delayMicroseconds (MeioPeriodo);   // MeioPeriodo de X microsegundos
}

void FREQUENCIA()                    // calcula Pulsos, PPS e RPM
{
  Pulsos = PPR * Voltas;             // Quantidade total de Pulsos (PPR = pulsos por volta)
  PPS = 1000000/ (2 * MeioPeriodo); // Frequencia Pulsos por segundo
  RPM = (PPS * 60) / PPR;            // Calculo do RPM
}

void FULL()
{
  Serial.println(" Passo Completo  PPR = 200 ");
  PPR = 200;                 // PPR = pulsos por volta
  digitalWrite(MS1, LOW);    // Configura modo Passo completo (Full step)
  digitalWrite(MS2, LOW);
  digitalWrite(MS3, LOW);
}

void HALF()
{
  Serial.println(" Meio Passo  PPR = 400 ");
  PPR = 400;                  // PPR = pulsos por volta
  digitalWrite(MS1, HIGH);    // Configura modo Meio Passo (Half step)
  digitalWrite(MS2, LOW);
  digitalWrite(MS3, LOW);
}

void P1_4()
{
  Serial.println(" Micro-passo 1/4  PPR = 800 ");
  PPR = 800;                 // PPR = pulsos por volta
  digitalWrite(MS1, LOW);    // Configura modo Micro Passo 1/4
  digitalWrite(MS2, HIGH);
  digitalWrite(MS3, LOW);
}

void P1_8()
{
  Serial.println(" Micro-passo 1/8  PPR = 1600 ");
  PPR = 1600;                 // PPR = pulsos por volta
  digitalWrite(MS1, HIGH);    // Configura modo Micro Passo 1/8
  digitalWrite(MS2, HIGH);
  digitalWrite(MS3, LOW);
}

void P1_16()
{
  Serial.println(" Micro-passo 1/16  PPR = 3200 ");
  PPR = 3200;                 // PPR = pulsos por volta
  digitalWrite(MS1, HIGH);    // Configura modo Micro Passo 1/16
  digitalWrite(MS2, HIGH);
  digitalWrite(MS3, HIGH);
}

void TesteMotor()
{
  Print_RPM ();                           // Print Voltas, PPS e  RPM

  HOR();                                  // Gira sentido Horario
  for (int i = 0; i <= Pulsos; i++)       // Incrementa o Contador
  {
    PASSO();                              // Avança um passo no Motor
  }
  disa_A4988();                           // Desativa o chip A4988
    delay (1000) ;                          // Atraso de 1 segundo
    ena_A4988();                            // Ativa o chip A4988

    AHR();                                  // Gira sentido anti-Horario
    for (int i = 0; i <= Pulsos; i++)       // Incrementa o Contador
    {
    PASSO();                              // Avança um passo no Motor
    }
    disa_A4988();                           // Desativa o chip A4988
    delay (1000) ;                          // Atraso de 1 segundo
    ena_A4988();                           // Ativa o chip A4988
}
void Motor_HOR()
{
  Print_RPM ();
  HOR();                                  // Gira sentido Horario
  for (int i = 0; i <= Pulsos; i++)       // Incrementa o Contador
  {
    PASSO();                              // Avança um passo no Motor
  }
  
}

void Motor_AHR()
{
  Print_RPM ();
  AHR();                                  // Gira sentido anti-Horario
    for (int i = 0; i <= Pulsos; i++)       // Incrementa o Contador
    {
    PASSO();                              // Avança um passo no Motor
    }
  
}

void Print_RPM ()
{
  FREQUENCIA();                           // calcula Pulsos, PPS e RPM
  Serial.print(" Voltas= ");
  Serial.print(Voltas);
  Serial.print(" Pulsos= ");
  Serial.print(Pulsos);
  Serial.print(" PPS= ");
  Serial.print(PPS, 2);
  Serial.print(" RPM= ");
  Serial.println(RPM, 2);
}
void Leitura_LDRs()                       // Faz a leitura dos LDRs para aquisição dos valores
{
  lesteLDR = analogRead(lLDRPin);
  oesteLDR = analogRead(oLDRPin);
  differenca = lesteLDR - oesteLDR ;      //Checa a diferenca entre as leituras
}
void Posicao_Inicial()                    // Coloca o painel na posicção incial, sempre apontado para o nascer do sol.
{
  if (lesteLDR < 100 && oesteLDR < 100)   // Verifica se está com luz baixa
  {
    
      Voltas = 1;
       Motor_HOR();
      Serial.println(" Voltaando para posição inicial ");
      
    
  }
}
void Rastreador()
{
  Serial.println(differenca);
  if (differenca > 20) //Send the panel towards the LDR with a higher reading
  {
   // if (lmt_inicial == LOW)
    //{
    PID_total=PID_total*-1;
      Voltas = PID_total;
      Motor_HOR();
   // }
  } else if (differenca < -20)
  {
    //if (lmt_final == LOW)
    //{
    
      Voltas = PID_total;
      Motor_AHR();
   // }
  }
}

void P_I_D()
{
  if (millis() > time+period)
  {
    time = millis();    
    distance = differenca;   
    distance_error = distance_setpoint - distance;   
    PID_p = kp * distance_error;
    float dist_diference = distance_error - distance_previous_error;     
    PID_d = kd*((distance_error - distance_previous_error)/period);
      
    if(-30 < distance_error && distance_error < 30)
    {
      PID_i = PID_i + (ki * distance_error);
    }
    else
    {
      PID_i = 0;
    }
  
    PID_total = PID_p + PID_i + PID_d;  
    Serial.println(PID_total);
    //PID_total = map(PID_total, -150, 150, 0, 150);
  
    //if(PID_total < 20){PID_total = 20;}
    //if(PID_total > 160) {PID_total = 160; } 
  
   
    distance_previous_error = distance_error;
  }
  
}
void giroscopio()
  {
    Wire.beginTransmission(MPU);
   Wire.write(0x3B); //Pedir el registro 0x3B - corresponde al AcX
   Wire.endTransmission(false);
   Wire.requestFrom(MPU,6,true);   //A partir del 0x3B, se piden 6 registros
   AcX=Wire.read()<<8|Wire.read(); //Cada valor ocupa 2 registros
   AcY=Wire.read()<<8|Wire.read();
   AcZ=Wire.read()<<8|Wire.read();
 
   //A partir de los valores del acelerometro, se calculan los angulos Y, X
   //respectivamente, con la formula de la tangente.
   Acc[1] = atan(-1*(AcX/A_R)/sqrt(pow((AcY/A_R),2) + pow((AcZ/A_R),2)))*RAD_TO_DEG;
   Acc[0] = atan((AcY/A_R)/sqrt(pow((AcX/A_R),2) + pow((AcZ/A_R),2)))*RAD_TO_DEG;
 
   //Leer los valores del Giroscopio
   Wire.beginTransmission(MPU);
   Wire.write(0x43);
   Wire.endTransmission(false);
   Wire.requestFrom(MPU,6,true);   //A partir del 0x43, se piden 6 registros
   GyX=Wire.read()<<8|Wire.read(); //Cada valor ocupa 2 registros
   GyY=Wire.read()<<8|Wire.read();
   GyZ=Wire.read()<<8|Wire.read();
 
   //Calculo del angulo del Giroscopio
   Gy[0] = GyX/G_R;
   Gy[1] = GyY/G_R;
   Gy[2] = GyZ/G_R;

   dt = (millis() - tiempo_prev) / 1000.0;
   tiempo_prev = millis();
 
   //Aplicar el Filtro Complementario
   Angle[0] = 0.98 *(Angle[0]+Gy[0]*dt) + 0.02*Acc[0];
   Angle[1] = 0.98 *(Angle[1]+Gy[1]*dt) + 0.02*Acc[1];

   //Integración respecto del tiempo paras calcular el YAW
   Angle[2] = Angle[2]+Gy[2]*dt;

   }
    void escrita()
    {
      Serial.print("DATA,DATE,TIME,"); //display date and time
   Serial.print(Angle[0]);
   Serial.print(",");
   Serial.println(Angle[1]);
   
   
   delay(10);
      }

void loop()
{
  
  giroscopio();
  //FULL();          // Selecione aqui o modo de passo
  HALF();        // desmarque o comentario somente da opcao desejada
  //P1_4();
  //P1_8();
  //P1_16();
  Leitura_LDRs();
  P_I_D();
  Posicao_Inicial();
  Rastreador();
  //giroscopio();
  //escrita();

  //Voltas = 100;        // Selecione o numero de Voltas
  //TesteMotor();      // Inicia teste do motor
}
