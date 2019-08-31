// 
// INTERFACE 48 CONTROLES CMRI SMINI - Versao 2.1
// By Clederson T. Przybysz - clederson_p@yahoo.com.br
// expressoarduino.blogspot.com
// Criação: Fevereiro/2019 - Revisão: Agosto/2019
//
// Release Note: 
// 2.0: Modulos MCP2307 para Entradas e Saidas I2C
//      Controle AMV por Servos com modulo I2C
//      Modo Configuração automatico quando pino 12 em nivel Baixo;
//      Novo menu de configuração detalhado;
// 2.1: Inversao dos Valores Enviados das Entradas;
//
//Copyright Notes Interface 48 Controles:
// O SOFTWARE É FORNECIDO "NO ESTADO EM QUE SE ENCONTRAM", SEM GARANTIA DE QUALQUER TIPO, EXPRESSA OU IMPLÍCITA, MAS NÃO SE LIMITANDO ÀS GARANTIAS DE COMERCIALIZAÇÃO.  
// EM NENHUMA CIRCUNSTÂNCIA, O AUTOR/TITULAR DE DIREITOS AUTORAIS SE RESPONSABILIZA POR QUALQUER RECLAMAÇÃO, DANOS OU OUTRA RESPONSABILIDADE, 
// SEJA EM AÇÃO DE CONTRATO, DELITO OU DE OUTRA FORMA, DECORRENDO DESTE SOFTWARE OU RELACIONADO AO SEU  USO.
//
//Copyright Notes CRMI Library AND AutoRS485 Library: 
//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
//OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
//HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
//FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
// Copyright Notes Adafruit PWMServoDriver Library:
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
// THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
// HOLDER BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT 
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
// POSSIBILITY OF SUCH DAMAGE.


#include <Auto485.h>
#include <CMRI.h>
#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>
#include <EEPROM.h>
#include "Adafruit_MCP23017.h"

//Instacia RS485
Auto485 SerialRs485(2);
CMRI cmrinode(0,24, 48, SerialRs485); //0 - Endereco Base, 24 Entradas(Sensores), 48 Saidas(Controles), ControleSerial

//Instacia i2C Servo
Adafruit_PWMServoDriver modServo = Adafruit_PWMServoDriver(0x40);

//Instacia I2C MCP23017
Adafruit_MCP23017 mcp0;
Adafruit_MCP23017 mcp1;
Adafruit_MCP23017 mcp2;

#define VERSAOATUAL 2.0
#define MIN_PULSE_WIDTH       650
#define MAX_PULSE_WIDTH       2350
#define DEFAULT_PULSE_WIDTH   1500
#define FREQUENCY             50

unsigned long TimerAtualizaServos;
unsigned long TimerCarregaEntradas;

#define TempoPulso 100
#define TempoLeituraEntradas 500

// Pino Modo Config - Pino deve fica em nivel baixo para funcionamento em modo Node e ligado ao nivel Baixo para exibir o menu configuração
#define configPinOff 11
#define configPin   12 

#define pinPowerDelay 7

byte AtualizaValorSaida=1;
byte EtapaConfig;
byte ItemConfig;

byte EnderecoNode=0;

byte ValorSaidas[6];   //Cada byte tem 8 bit - 8x4 32 bits
byte ValorEntradas[3]; //Cada byte tem 8 bit - 8x3 24 bits 
byte PosServo[16];     //Posicao Atual do Servo
byte NovaPosServo[16]; //Nova Posicao do Servo

void setup() 
{
  SerialRs485.begin(19200);
  
  //Pino Config habilitado em nivel alto
  pinMode(configPinOff, OUTPUT);
  digitalWrite(configPinOff,LOW);
  pinMode(configPin, INPUT_PULLUP);

  //Pino Power Delay Servos
  pinMode(pinPowerDelay, OUTPUT);
  digitalWrite(pinPowerDelay,LOW);

  //Inicia MCP2307
  mcp0.begin(0);
  mcp1.begin(1);
  mcp2.begin(2);
  
  IniciaPortasMCP23017();


  //Carrega Configuraces EEPROM
  CarregaConfiguracoes();

  //Se pino Config está em nivel Baixo habilita configuracao
  if (digitalRead(configPin)==0) {
    //Ativa Alimentacao dos Servos
    digitalWrite(pinPowerDelay,HIGH);
    
    SerialRs485.println("Modo Configuracao...");
    EtapaConfig=0;
    LoopModoConfig();
  }
  
     
  //Configura Posicao Inicial dos Servos
  for(int i = 0; i < 16; i++) {
    ValorSaidas[i]=0;
    NovaPosServo[i] = PosicaoZeroServo(i);
    PosServo[i] = NovaPosServo[i];
    AtualizaPosicaoServo(i, PosServo[i]);
  }
    
  //Endereco Node
  cmrinode.set_address(EnderecoNode);

  //Power Servos
  delay(1000+500*EnderecoNode);
  digitalWrite(pinPowerDelay,HIGH);
  
  SerialRs485.flush();
 
}



void loop() {
  
  cmrinode.process();
   
  //Carrega Valores de Entrada
  if (millis()-TimerCarregaEntradas>500) {
    CarregaValoresEntradas();
    TimerCarregaEntradas = millis();
  }

  //Verifica Mudança Nos Valores das Saidas
  for(int i = 0; i < 6; i++) {
    byte NovoValor;
    NovoValor = cmrinode.get_byte(i);
    if (NovoValor != ValorSaidas[i]) {
      ComparaBitsSaidas(NovoValor, i);
    }
  }

  
  //Verifica Posicao dos Servos - Intervalo 0,1 segundos
  if (millis()-TimerAtualizaServos>TempoPulso) {
    for(int i = 0; i < 16; i++) {
      //Incrementa Posicao do Servo
      if (NovaPosServo[i] > PosServo[i]) {
        PosServo[i]++;
        AtualizaPosicaoServo(i, PosServo[i]);
      }
      if (NovaPosServo[i] < PosServo[i]) {
        PosServo[i]--;
        AtualizaPosicaoServo(i, PosServo[i]);
      }
    }
    TimerAtualizaServos = millis();
  }

   
  //Atualizacao das Portas de Saidas
  if (AtualizaValorSaida>0) AtualizaSaida();
}

void ComparaBitsSaidas(byte tmpValor, byte bloco) {
  //Compara os bit de Novo Valor
  for(byte i = 0; i < 8; i++) {
    if (bitRead(tmpValor, i) != bitRead(ValorSaidas[bloco],i)) {
      if (bloco<2) {
        NovoValorServo(bloco * 8 + i, bitRead(tmpValor, i));  
      }
      else
      { 
        AtualizaValorSaida=1; 
      }
    }
  }
  ValorSaidas[bloco] = tmpValor;
}

void NovoValorServo(byte nServo, byte NovoValor) {
  if (NovoValor==1) {
    NovaPosServo[nServo] = PosicaoUmServo(nServo); 
  }
  else
  {
    NovaPosServo[nServo] = PosicaoZeroServo(nServo); 
  }
}

byte PosicaoZeroServo(byte nServo) {
  byte Angulo;
  Angulo = EEPROM.read(20 + nServo);
  if (Angulo>180) Angulo=5;
  return Angulo;
}

byte PosicaoUmServo(byte nServo) {
  byte Angulo;
  Angulo = EEPROM.read(52 + nServo);
  if (Angulo>180) Angulo=35;
  return Angulo;
}


void AtualizaPosicaoServo(byte nServo, int Angulo) {
  modServo.setPWM(nServo, 0, ConverteAnguloPulso(Angulo)); 
}

int ConverteAnguloPulso(int angle){
  int pulse_wide, analog_value;
  pulse_wide   = map(angle, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  analog_value = int(float(pulse_wide) / 1000000 * FREQUENCY * 4096);
  //Serial.println(analog_value);
  return analog_value;
}

void AtualizaSaida() {
  int tmpAB;
  
  //Saidas 2 a 3 (17 a 32) - MCP23017 1 
  tmpAB = ValorSaidas[3];
  tmpAB <<= 8;
  tmpAB |= ValorSaidas[2];
  mcp1.writeGPIOAB(tmpAB);
  
  //PortaSaida 4 e 5 (32 a 48)- MCP23017 2 
  tmpAB = ValorSaidas[5];
  tmpAB <<= 8;
  tmpAB |= ValorSaidas[4];
  mcp2.writeGPIOAB(tmpAB);
}

void CarregaValoresEntradas() {
  for (byte i = 0; i < 2; i++) {
    //Carrega Valor MCP23017
    byte NovoValor = RetornaValorEntradaMCP23017(i);
        
    //Verifica Mudanca no Valor
    if (NovoValor != ValorEntradas[i]) {
      cmrinode.set_byte(i, ~NovoValor);
      ValorEntradas[i] = NovoValor;
    }
  }
}

byte RetornaValorEntradaMCP23017(byte Entrada) {
  byte Valor;
  //PortaEntrada 0 - MCP23017 0 / Porta A
  if (Entrada==0) Valor=mcp0.readGPIO(0);
  //PortaEntrada 1 - MCP23017 0 / Porta B
  if (Entrada==1) Valor=mcp0.readGPIO(1);
  //Retorna Valor
  return Valor;
}

void IniciaPortasMCP23017() {
  //Modulo 0 - Input
  for (int p=0;p<16;p++) {
    mcp0.pinMode(p, INPUT);
    mcp0.pullUp(p, HIGH);  
  }
  //Modulo 1 - Output
  for (int p=0;p<16;p++) {
    mcp1.pinMode(p, OUTPUT);
  }
  //Modulo 2 - Output
  for (int p=0;p<16;p++) {
    mcp2.pinMode(p, OUTPUT);
  }
}


void LoopModoConfig() {
  //Executa Loop Menu Configuração Enquanto Etapa Diferente de 99
  while (EtapaConfig<99) {
    MenuConfiguracao();
  }
  //Fim Loop Configuracao
  SerialRs485.println("Interface Ativa");
}

void MenuConfiguracao() {
  byte vRecebidoFimLinha=0;
  byte vRecebido=0;
  //Exibe Menu de Configuracao
  switch (EtapaConfig) {
    case 0:
      // Menu Inicial
      SerialRs485.println("");
      SerialRs485.println("MENU INICIAL:");
      SerialRs485.println("[1] Endereco Node");
      SerialRs485.println("[2] Configura Servos");
      SerialRs485.println("[3] Lista Configuracoes");
      SerialRs485.println("[9] Fim");
      SerialRs485.println("Digite a opcao [1 a 3 ou 9]:");
      break;
    case 10:
      // Endereco Node
      SerialRs485.println("");
      SerialRs485.println("End.Node [0-125]:");
      break;
    case 20:
      // Configura Servo (1/3)
      SerialRs485.println("");
      SerialRs485.println("Configurar Servo [1-16] ou [88] Todos ou [99] Voltar:");
      ItemConfig=0;
      break; 
    case 21:
      // Configura Angulo Minimo Servo (2/3)
      SerialRs485.print("Angulo Min Servo ");
      if (ItemConfig==88) SerialRs485.print("1 a 16"); else SerialRs485.print(ItemConfig);
      SerialRs485.println(":");
      break;
    case 22:
      // Configura Angulo Maximo Servo (2/3)
      SerialRs485.print("Angulo Max Servo ");
      if (ItemConfig==88) SerialRs485.print("1 a 16"); else SerialRs485.print(ItemConfig);
      SerialRs485.println(":");
      break;
    default:
      break;
  }

  //Le Porta Serial Ate Receber Fim de Linha
  SerialRs485.flush();
  while (vRecebidoFimLinha == 0) {
    if (SerialRs485.available() > 0) {
      byte inChar = SerialRs485.read();
      //Aguarda Caracter de Fim de Linha (LF ou CR)
      if (inChar==10||inChar==13) {
        vRecebidoFimLinha=1;
      }
      else
      {
        vRecebido = vRecebido * 10 + (inChar-48);
      }
    }
  }
  
  //Exibe Valor Recebido Porta Serial
  SerialRs485.println(vRecebido);

  //Trata Valor Recebido conforme Etapa Configuracao

  // 0. Menu Inicial
  if (EtapaConfig==0) {
    // Valor entre 1 e 6 Muda para Menu Selecionado ou 9 Sair
    if (vRecebido>0&&vRecebido<3) EtapaConfig = vRecebido*10;
    if (vRecebido==3) ListaConfiguracoes();
    if (vRecebido==9) EtapaConfig = 99;
  }
  
  // 10.Endereco Node
  else if (EtapaConfig==10&&vRecebido>=0&&vRecebido<2) {
    //Grava Endereco Node na EEPROM 1 e Volta Menu Inicial
    EEPROM.write(1, vRecebido);  
    EnderecoNode=vRecebido;
    EtapaConfig = 0;
  }

  // 20.Configura Servos - Numero do Servo (Menu 1/3) 
  else if (EtapaConfig==20){
    if ((vRecebido>0&&vRecebido<17)||vRecebido==88) {
      //Armazena Item Seleciona e Vai para Menu 51
      ItemConfig=vRecebido;
      EtapaConfig=21;
    }
    if (vRecebido==99) {
      //Volta Menu Inicial
      EtapaConfig=0;
    }
  }

  // 21 Configura Servos - Angulo Minimo (Menu 2/3)
  else if (EtapaConfig==21&&vRecebido>=0&&vRecebido<=180) {
    //Grava Angulo Minimo Servo EEPROM 20 a 35 e vai para Menu 22
    if (ItemConfig<88) {
      EEPROM.write(19+ItemConfig, vRecebido);
      AtualizaPosicaoServo(ItemConfig-1, vRecebido);
    }
    else
    {
      for (byte i = 1; i < 17; i++) {
        EEPROM.write(19+i, vRecebido);
        AtualizaPosicaoServo(i-1, vRecebido);
      }
    }
    EtapaConfig=22;
  }

  // 22 Configura Servos - Angulo Maximo (Menu 3/3)
  else if (EtapaConfig==22&&vRecebido>=0&&vRecebido<=180) {
    //Grava Angulo Maximo Servo EEPROM 52 a 67 e volta para Menu 20
    if (ItemConfig<88) {
      EEPROM.write(51+ItemConfig, vRecebido);
      AtualizaPosicaoServo(ItemConfig-1, vRecebido);
    }
    else
    {
      for (byte i = 1; i < 17; i++) {
        EEPROM.write(51+i, vRecebido);
        AtualizaPosicaoServo(i-1, vRecebido);
      }
    }
    EtapaConfig=20;
  } 
  
}



void CarregaConfiguracoes() {
  
  //Carrega Endereco Node 
  EnderecoNode = EEPROM.read(1);
  
  //Inicia I2C
  modServo.begin();
  modServo.setPWMFreq(FREQUENCY);
}


void ListaConfiguracoes() {
    SerialRs485.println(" ");
    SerialRs485.print("Configuracoes Interface CMRI/SMIN - Versao ");
    SerialRs485.println(VERSAOATUAL);
    SerialRs485.println("");
    SerialRs485.print("Node CMRI - Endereco: ");
    SerialRs485.println(EnderecoNode);
    SerialRs485.println("");
    SerialRs485.println("Angulo Servos:");
    for (byte i = 0; i < 16; i++) {
      SerialRs485.print(" - Servo "); 
      if (i+1<10) SerialRs485.print("0");
      SerialRs485.print(i+1);  
      SerialRs485.print(" - Pos0: ");
      SerialRs485.print(PosicaoZeroServo(i));
      SerialRs485.print(" - Pos1: ");
      SerialRs485.println(PosicaoUmServo(i));
    }
}

