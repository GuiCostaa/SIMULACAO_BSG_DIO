// CODIGO DESENVOLVIDO PARA SIMULAÇÃO DE BSG NO BARRAMENTO CAN
// CAN ID E PAYLOADS FORAM RETIRADOS DE UMA COLETA DE LOG EM UM CAMINHÃO DA MAN
// SÃO 4 LOGS: KL15 DESLIGADA PORTA ABERTA, KL15 DESLIGADA PORTA FECHADA, KL15 LIGADA, KL15 MOTOR LIGADO.

#include <Arduino.h>
#include <SPI.h>
#include "timer.h"
#include <mcp_can.h>

#define PIN_ACC_IGN 9
#define PIN_DRV_DOOR 5
#define PIN_PSG_DOOR 8
#define PIN_LED 6
//#define PIN_READ_SPEED


MCP_CAN CAN0(10);     // Set CS to pin 10


#define MODO_DEBUG "DEBUG,ENABLE"
#define HELP "HELP"
#define COMUNICAN 1
#define SIGNALIGN 2
#define DRVDOOR 3
#define PSGDOOR 4
#define DEBUGOFF "DEBUG,DISABLE"
#define VELOCIDADE "VELOCIDADE"

bool PERIOD_PSG = false;
bool PERIOD_DRV = false;
bool PERIOD_IGN = false;
bool PERIOD_CAN = false;

bool status_led = false;
bool Status_CAN_DRV = false;
bool Status_CAN_PSG = false;
bool STATUS_ACC_IGN = false;
bool DEBUG = false;
bool Start_Led = false;

byte *Payload_open_Drv_door = NULL;
byte *Payload_open_Psg_door = NULL;
int last_read_drv = 0;
int last_read_psg = 0;
int Drv_Door = 1;
int Psg_Door = 1;

const int tensao_acc_ign = 0;


//Funções
byte SendCanMessage(unsigned long CANID, byte Payload);
void ACC_IGN();
void DRV_DOOR();
void PSG_DOOR();
void teste();
void DEBUGGING();
void OPEN_DOOR();
void Blink_led(byte PARAM_CAN);


// Variavel de tempo
ulong timer_100ms = 0;
ulong timer_200ms = 0;
ulong timer_1000ms = 0;
ulong timer_led = 0;
ulong timer_drv_door = 0;
ulong timer_psg_door = 0;
ulong DEBOUNCE_DRV_DOOR = 0;
ulong DEBOUNCE_PSG_DOOR = 0;
ulong DEBOUNCE_ACC = 0;
ulong DEBOUNCE_LED = 0;
ulong DEBOUNCE_DEBUG_IGN = 0;
ulong DEBOUNCE_DEBUG_CAN = 0;
ulong DEBOUNCE_DEBUG_DRV = 0;
ulong DEBOUNCE_DEBUG_PSG = 0;

String Read_Serial;
int TypeDebug;

// CAN ID DOOR Status
#define CANID_DOOR 0x18FDA521 // CAN ID responsavel pelo status de porta aberta e porta fechada. Com informação no Byte 1 (motorista) e Byte 2 (Passageiro) {0x04; 0x01}
#define CANID_CFE6CEE 0xCFE6CEE // CAN ID Velocidade
#define CANID_18f00503 0x18F00503
#define CANID_18fda421 0x18FDA421
#define CANID_18f00029 0x18F00029
#define CANID_18febfob 0x18FEBF0B
#define CANID_18FDCD21 0x18FDCD21
#define CANID_18fdd121 0x18FDD121
#define CANID_18fe4021 0x18FE4021
#define CANID_18fe700b 0x18FE700B



// Payload Door Status
byte Closed_Door_Payload[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // PORTA FECHADA
//byte Open_Drv_Door_Payload[8] = {0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // PORTA DO MOTORISTA ABERTA
//byte Open_Psg_Door_Payload[8] = {0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x000}; // PORTA DO PASSAGEIRO ABERTA


byte Payload_canid_18f00503 [8] = {0x7D, 0xFF, 0xFF, 0x7D, 0x7D, 0x7D, 0X7e, 0xF7}; // Payload identificado nos logs de KL15 Desl, ligado e motor ligado porem não identificado no log de porta aberta
byte Payload_canid_18fda421 [8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // Payload identificado em todos os logs!!
byte Payload_canid_18febfob [8] = {0x00, 0x7D, 0x7D, 0x7D, 0x7D, 0x7D, 0x7D, 0x7D}; // Paylod identificado nos demais logs e não aparece no log KL15 desligado e porta aberta.
byte Payload_canid_18fdcd21 [8] = {0x0F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // Payload identificado em todos os logs!!
byte Payload_canid_18fdd121 [8] = {0xF0, 0x34, 0x30, 0x34, 0x30, 0xFF, 0xFF, 0xFF}; // Payload identificado em todos os logs!!
byte Payload_canid_18fe4021 [8] = {0xFC, 0x0F, 0xFF, 0xCF, 0xFF, 0xFF, 0xFF, 0xFF}; // Payload identificado em todos os logs!!
byte Payload_canid_18fe700b [8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // Payload identificado em todos os logs, menos no log de porta aberta.
byte Payload_canid_18f00029_KL15_Desl [8] = {0xFF, 0x7D, 0x7D, 0xF0, 0xF0, 0xFF, 0xC0, 0x7D}; // Payload identificado em todos menos porta aberta, com payload diferente do log do motor ligado {1}
byte Payload_canid_18f00029_Start_engine [8] = {0xFF, 0x7D, 0x7D, 0xF0, 0xFF, 0xFF, 0xC0, 0x7D}; // Payload identificado em todos, mas com motor ligado altera o payload {2}
byte Payload_velo [8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16};

void setup()
{
  // Inicialização comunicação Serial
  Serial.begin(115200);

  // Inicialização dos pinos de entrada
  pinMode(PIN_ACC_IGN, INPUT);
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_DRV_DOOR, INPUT_PULLUP);
  pinMode(PIN_PSG_DOOR, INPUT_PULLUP);
  
  Payload_open_Drv_door = Closed_Door_Payload;
  Payload_open_Psg_door = Closed_Door_Payload;

  Start_Led = true;

  
    if(timerElapsed(&DEBOUNCE_LED, 500))
    {
      digitalWrite(PIN_LED, status_led);
      status_led = !status_led;
      digitalWrite(PIN_LED, status_led);
    }



  

  // Initialize MCP2515 running at 8MHz with a baudrate of 500kb/s and the masks and filters disabled.
  if(CAN0.begin(MCP_ANY, CAN_250KBPS, MCP_8MHZ) == CAN_OK) Serial.println("MCP2515 Initialized Successfully!");
  else Serial.println("Error Initializing MCP2515...");

  CAN0.setMode(MCP_NORMAL);   // Change to normal mode to allow messages to be transmitted
}



void loop()
{

  ACC_IGN();
  DRV_DOOR();
  PSG_DOOR();
  DEBUGGING();
 
}


byte SendCanMessage(unsigned long CANID, byte *Payload)
{
  byte sendstatus = CAN0.sendMsgBuf(CANID, 1, 8, Payload);

  return sendstatus;
}


void ACC_IGN()
{
  int Read_Pin_Acc = 0;
  if (timerElapsed(&DEBOUNCE_ACC, 30))
  {
    Read_Pin_Acc = digitalRead(PIN_ACC_IGN); 
  }

  // Envio de mensagem a cada 100ms no barramento CAN.
  if(Read_Pin_Acc > 0)
  {

    if(timerElapsed(&timer_100ms, 100))
    {
      //Serial.println("IGN LIGADA");
      byte statussend = SendCanMessage(CANID_18f00503, Payload_canid_18f00503);
      Blink_led(statussend);
      SendCanMessage(CANID_18fda421, Payload_canid_18fda421);
      SendCanMessage(CANID_18f00029, Payload_canid_18f00029_KL15_Desl);
      
    }
  

    // Envio de mensagem a cada 200ms no barramento CAN.
    if(timerElapsed(&timer_200ms, 200))
    {

      byte statussend = SendCanMessage(CANID_18febfob, Payload_canid_18febfob);
      Blink_led(statussend);
    }
  
    if(timerElapsed(&timer_1000ms, 1000))
    {
    
      byte statussend = SendCanMessage(CANID_18FDCD21, Payload_canid_18fdcd21);
      Blink_led(statussend);
      SendCanMessage(CANID_18fdd121, Payload_canid_18fdd121);
      SendCanMessage(CANID_18fe4021, Payload_canid_18fe4021);
      SendCanMessage(CANID_18fe700b, Payload_canid_18fe700b);

    }
  }

}

/*void teste()
{
  byte status = SendCanMessage(CANID_DOOR, Closed_Door_Payload);
  if( status == CAN_OK)
  {
    digitalWrite(PIN_LED, HIGH);
    Serial.println("CAN OK!");
  }
  else
  {
    Serial.println("ERROR!");
    if(timerElapsed(&timer_led, 500))
    {
      digitalWrite(PIN_LED, status_led);
      status_led = !status_led;
    }
  }

}*/

void DRV_DOOR()
{
  if (timerElapsed(&DEBOUNCE_DRV_DOOR, 30))
  {
    Drv_Door = digitalRead(PIN_DRV_DOOR);

  }

  if(timerElapsed(&timer_drv_door, 100))
  {
    
    if (Drv_Door == 0)
    { 
      
      //Serial.println("OPEN DRV DOOR");

      Payload_open_Drv_door [0] = 0x04;

      //Serial.print("Payload Closed: ");
      /*for(int i = 0; i < 8; i++)
      {
        Serial.print(Closed_Door_Payload[i], HEX);
        
      }
      Serial.println(" ");
      Serial.print("Payload Ponteiro: ");
      for(int i = 0; i < 8; i++)
      {
        Serial.print(Payload_open_Drv_door[i], HEX);
      }*/

      byte St_Open_Drv_Door = SendCanMessage(CANID_DOOR, Closed_Door_Payload);
      Blink_led(St_Open_Drv_Door);
    }
    else
    {
      //Serial.println("CLOSE DOOR");
      Payload_open_Drv_door [0] = 0x00; 
      SendCanMessage(CANID_DOOR, Closed_Door_Payload);
    }
  }
}

void PSG_DOOR()
{
  if (timerElapsed(&DEBOUNCE_PSG_DOOR, 30))
  {
    Psg_Door = digitalRead(PIN_PSG_DOOR);

  }

  OPEN_DOOR();

  if(timerElapsed(&timer_psg_door, 100))
  {
    
    if (Psg_Door == 0)
    { 

      Payload_open_Psg_door [1] = 0x01;
      SendCanMessage(CANID_DOOR, Closed_Door_Payload);
    }
    else
    {
      //Serial.println("CLOSE DOOR");
      Payload_open_Psg_door [1] = 0x00;
      SendCanMessage(CANID_DOOR, Closed_Door_Payload);
    }
  }
}


void OPEN_DOOR()
{

}


void DEBUGGING()
{
  if(Serial.available() > 0)
  {
    Read_Serial = Serial.readStringUntil('\n');
    Read_Serial.toUpperCase();
    int PositionIndex = Read_Serial.indexOf(',');
    String Parse_Read_Serial = Read_Serial.substring(0, PositionIndex);
    PositionIndex++;
    int lengthReadSerial = Read_Serial.length();
    String Read_Serial_Position = Read_Serial.substring(PositionIndex, lengthReadSerial);
    TypeDebug = Read_Serial.toInt();

    Serial.println(TypeDebug);
    if (Read_Serial == MODO_DEBUG)
    {
      DEBUG = true;
      Serial.println("Modo Depuracao ativado!");
      Serial.println("Digite Help para verificar os comandos");
    }
    else if (DEBUG == true && Read_Serial == HELP)
    {
      Serial.println("Depuracao: "); 
      Serial.println("1- Comunicacao CAN "); 
      Serial.println("2- Leitura de Sinal Ignicao");
      Serial.println("3- Leitura de Sinal Porta Motorista");
      Serial.println("4- Leitura de Sinal Porta Passageiro");
      Serial.println("5- Para encerracao do modo Depuracao");
    }


    //Serial.println(PERIOD_CAN);
    //Serial.println(PERIOD_DRV);
    //Serial.println(PERIOD_IGN);
    //Serial.println(PERIOD_PSG);
  }
  if(DEBUG == true)
  {
    /*if ((TypeDebug == COMUNICAN) || ( PERIOD_CAN ))
    {

    }*/
    if (Read_Serial == DEBUGOFF)
    {
      PERIOD_CAN = false;
      PERIOD_DRV = false;
      PERIOD_IGN = false;
      PERIOD_PSG = false;
      DEBUG = false;

    }
    else if ((TypeDebug == SIGNALIGN) || ( PERIOD_IGN ) )
    {
      PERIOD_IGN = true;
      if(timerElapsed(&DEBOUNCE_DEBUG_IGN, 1000))
      {
      Serial.print("Valor do Pino de entrada da IGN: ");
      Serial.println(PIN_ACC_IGN);
      }
    }
    else if ((TypeDebug == DRVDOOR ) || ( PERIOD_DRV ))
    {
      PERIOD_DRV = true;
      if(timerElapsed(&DEBOUNCE_DEBUG_DRV, 1000))
      {
        Serial.print("Valor do Pino de entrada da Porta do Motorista: ");
        Serial.println(Drv_Door);
      }
    }
    else if((TypeDebug == PSGDOOR) || ( PERIOD_PSG ))
    {
      PERIOD_PSG = true;
      if(timerElapsed(&DEBOUNCE_DEBUG_PSG, 1000))
      {
        Serial.print("Valor do Pino de entrada da Porta do Passageiro: ");
        Serial.println(Psg_Door);
      }
    }

    else if (Read_Serial == VELOCIDADE)
    {
      byte St_send_Velo = SendCanMessage(CANID_CFE6CEE, Payload_velo);
      Blink_led(St_send_Velo);
    }
  }

}

void Blink_led(byte PARAM_CAN)
{
  if(PARAM_CAN == CAN_OK)
  {
  digitalWrite(PIN_LED, HIGH);
  }
}
/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
