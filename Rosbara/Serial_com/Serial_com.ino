//Codigo para receber valores inteiros pela serial junto com alguma label 
#include <Wire.h>


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
   serialOut = "U:";
   serialOut += String(linearVel)+ ",";
   serialOut += String(angularVel) + ",";
   serialOut += String(linearVel+angularVel) + ",";
   serialOut += "\n";
   Serial.print(serialOut);
   
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

  cmdIndex = 0; //A cada inicio volta o cmdIndex para 0
  

}

void loop() {
  receiveCmd();
}


