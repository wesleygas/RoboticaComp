// ---------------------------------------------------------------------------
// Example NewPing library sketch that pings 3 sensors 20 times a second.
// ---------------------------------------------------------------------------

#include <NewPing.h>
#include <Wire.h>

#define SONAR_NUM 3     // Number of sensors.
#define MAX_DISTANCE 200 // Maximum distance (in cm) to ping.

#define MDATA 5
#define MCLK 6

unsigned long currentMillis = 0;
unsigned long sonarMillis = 0;

#define RMO1 10
#define RMO2 7
#define LMO1 9
#define LMO2 8

byte LO1 = 0;
byte LO2 = 0;
byte RO1 = 0;
byte RO2 = 0;

byte goal = 40;

String toOut = "";
int dist = 0;

NewPing sonar[SONAR_NUM] = {   // Sensor object array.
  NewPing(3, 2, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping. 
  NewPing(11, 4, MAX_DISTANCE), 
  NewPing(13, 12, MAX_DISTANCE)
};

float postUltrass(){
  toOut = "U:";
  for (uint8_t i = 0; i < SONAR_NUM; i++) { // Loop through each sensor and post results.   
    dist = 0;
    dist = sonar[i].ping_cm();
    delay(40);
    if(dist != 0){
      toOut += String(dist)+ ",";
    }
  }
  toOut += "\n";
  Serial.print(toOut);

  
}

void gohi(int pin)
{
  pinMode(pin, INPUT);
  digitalWrite(pin, HIGH);
}

void golo(int pin)
{
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
}

void mouse_write(char data)
{
  char i;
  char parity = 1;

  //  Serial.print("Sending ");
  //  Serial.print(data, HEX);
  //  Serial.print(" to mouse\n");
  //  Serial.print("RTS");
  /* put pins in output mode */
  gohi(MDATA);
  gohi(MCLK);
  delayMicroseconds(300);
  golo(MCLK);
  delayMicroseconds(300);
  golo(MDATA);
  delayMicroseconds(10);
  /* start bit */
  gohi(MCLK);
  /* wait for mouse to take control of clock); */
  while (digitalRead(MCLK) == HIGH)
    ;
  /* clock is low, and we are clear to send data */
  for (i=0; i < 8; i++) {
    if (data & 0x01) {
      gohi(MDATA);
    } 
    else {
      golo(MDATA);
    }
    /* wait for clock cycle */
    while (digitalRead(MCLK) == LOW)
      ;
    while (digitalRead(MCLK) == HIGH)
      ;
    parity = parity ^ (data & 0x01);
    data = data >> 1;
  }  
  /* parity */
  if (parity) {
    gohi(MDATA);
  } 
  else {
    golo(MDATA);
  }
  while (digitalRead(MCLK) == LOW)
    ;
  while (digitalRead(MCLK) == HIGH)
    ;
  /* stop bit */
  gohi(MDATA);
  delayMicroseconds(50);
  while (digitalRead(MCLK) == HIGH)
    ;
  /* wait for mouse to switch modes */
  while ((digitalRead(MCLK) == LOW) || (digitalRead(MDATA) == LOW))
    ;
  /* put a hold on the incoming data. */
  golo(MCLK);
  //  Serial.print("done.\n");
}

/*
 * Get a byte of data from the mouse
 */
char mouse_read(void)
{
  char data = 0x00;
  int i;
  char bit = 0x01;

  //  Serial.print("reading byte from mouse\n");
  /* start the clock */
  gohi(MCLK);
  gohi(MDATA);
  delayMicroseconds(50);
  while (digitalRead(MCLK) == HIGH)
    ;
  delayMicroseconds(5);  /* not sure why */
  while (digitalRead(MCLK) == LOW) /* eat start bit */
    ;
  for (i=0; i < 8; i++) {
    while (digitalRead(MCLK) == HIGH)
      ;
    if (digitalRead(MDATA) == HIGH) {
      data = data | bit;
    }
    while (digitalRead(MCLK) == LOW)
      ;
    bit = bit << 1;
  }
  /* eat parity bit, which we ignore */
  while (digitalRead(MCLK) == HIGH)
    ;
  while (digitalRead(MCLK) == LOW)
    ;
  /* eat stop bit */
  while (digitalRead(MCLK) == HIGH)
    ;
  while (digitalRead(MCLK) == LOW)
    ;

  /* put a hold on the incoming data. */
  golo(MCLK);
  //  Serial.print("Recvd data ");
  //  Serial.print(data, HEX);
  //  Serial.print(" from mouse\n");
  return data;
}

void mouse_init()
{
  gohi(MCLK);
  gohi(MDATA);
  //  Serial.print("Sending reset to mouse\n");
  mouse_write(0xff);
  mouse_read();  /* ack byte */
  //  Serial.print("Read ack byte1\n");
  mouse_read();  /* blank */
  mouse_read();  /* blank */
  //  Serial.print("Sending remote mode code\n");
  mouse_write(0xf0);  /* remote mode */
  mouse_read();  /* ack */
  //  Serial.print("Read ack byte2\n");
  delayMicroseconds(100);
}


char cmd[40]; //Inicia um array de caracteres onde vamos guardar os comandos recebidos pela serial
int cmdIndex; //Placeholder para a index que vamos ler dos comandos
String serialOut = "";
int linearVel = 0;
int angularVel = 0;

bool negative = false;

void exeCmd(){
  //Podemos usar o metodo strcmp(cmd,"str"), que retorna 0  se as duas strings forem iguais, mas isso é muito lerdo
  if((cmd[0] == 'L' || cmd[0] == 'A') && cmd[1] == ' '){ //Se o comando tiver inicio com L(linear) ou A(angular)
    
    int val = 0; //Iniciando o parsing do comando
    for(int i = 2; cmd[i] != 0; i++){ //O for irá rodar até encontrar o valor 0(null) que colocaremos na hora do recebimento
      if(cmd[i] == '-'){ //Se o valor for negativo, levanta a flag e pula o sinal
        negative = true;
        i++;
      }
      val = val*10 + (cmd[i] - '0'); //para cada rodada, atualizamos qual casa decimal estamos escrevendo
      //a subtracao por zero e pra pegar o valor inteiro do caractere (o zero eh 48 e assim vai ate o 9 = 57
    
    }
    if(negative){ //Se a flag estiver ativa, inverte o valor final e abaixa a flag
      val = -val;
      negative = false;    
    }
    
    if(cmd[0] == 'L'){
      linearVel = val;

      
    }
    else{ //Se nao eh L e entrou no if, é A
      angularVel = val;
      
      
    }   
  }
}

//Os comandos seguirao a sintaxe "(char)TYPE (VALUE
void receiveCmd(){
  if(Serial.available()){
    char c = (char)Serial.read();
    if(c == '\n'){ 
      cmd[cmdIndex] = 0; //Adiciona o tail (byte null)
      exeCmd();
      cmdIndex = 0;
    }
    else{
      cmd[cmdIndex] = c;
      if(cmdIndex < 39) cmdIndex++;
    }
  }
}


void setup() {
  Serial.begin(115200); // Open serial monitor at 115200 baud to see ping results.
  pinMode(RMO1,OUTPUT);
  pinMode(RMO2,OUTPUT);
  pinMode(LMO1,OUTPUT);
  pinMode(LMO2,OUTPUT);
  mouse_init();
  cmdIndex = 0;
  
}

void loop() { 
  currentMillis = millis();
    if(currentMillis - sonarMillis > 300){
      sonarMillis = currentMillis;
      postUltrass();
    }
  receiveCmd();
  char mstat;
  int mx;
  int my;

  /* get a reading from the mouse */
  mouse_write(0xeb);  /* give me data! */
  mouse_read();      /* ignore ack */
  mstat = mouse_read();
  mx = mouse_read();
  my = - mouse_read();

  

 
}


