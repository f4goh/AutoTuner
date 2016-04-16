/*
  Boite de couplage automatique
  version boitier étanche avec com RS485
 
  V0.4 : recherche a partir de l'ATU balise

Commande rs485 boitier ou USB (PC)

Par la commande rs485
- le mode balun ou asymétrique
- le seuil max du swr avant de déclencher une recherche et seuil acceptable
- permettre une commande de C,L,HZ, Balun
- avoir un compte rendu de la recherche swr
- un bilan des capas et selfs et hzlz
- avoir un affichage de P et du SWR


la boite auto devra :

Mémoriser dans l'eeprom
- le mode balun,swr,offsetdbmpont
- la dernière recherche L C HZ balun
- Dialoguer avec le boitier de commande si il existe

Etat des leds verte et bleue

rouge on , verte off  : swr > seuil ou mise sous tension
rouge et vert clignote en alternatif : recherche
rouge off, verte fixe : swr <= seuil

Protocole : 

chaque commande suivi de nouvelle ligne

commande vers boite auto :
b 00/01 : balun ou asymétrique
w22 : seuil swr x10
u 00/01 L/C UP
d 00/01 L/C Down
h 00/01 HzLz
c 00 record config
m 47 dbm pont
n 00 effectue une recherche
v 00/01 save auto
i 00 init eeprom
t 30 trig level Power
r 00 read config


boite auto vers commande :
S45 :  swr x10
R00 : démarrage d'une recherche large
F00 : démarrage d'une recherche fine
L(byte)  : valeur 8 bits de la commande des relais des selfs
C(byte)  : valeur 8 bits de la commande des relais des selfs
H(byte) : hzlz
P 000/999 : x10  power level
B 00/01 balun
W 22   seuil swr
M 47   dbm pont
V 01   save auto
T 30   trig power level

*/

// include the library code:

#include <EEPROM.h>
#include <Wire.h> 
//#include <LiquidCrystal_I2C.h>

//#define DEBUG_CMD
//#define DEBUG_SWR
//#define DEBUG_DBM

#define measureInterval 100
#define searchDelay 20


// initialize the lcd 
//LiquidCrystal_I2C lcd(0x27, 20, 4);

// init keypad
int lcd_key     = 0;
int adc_key_in  = 0;
#define balun_pin 2
#define hzlz_pin  3
#define rede_pin  4
#define tx_soft_pin  7
#define rx_soft_pin  8

#define led_red 12
#define led_green 13

#define vf A0
#define vr A1



typedef struct  {
  byte id;            //indentification match box default 0xfd
  byte balun;         // Balun active or not
  float swrSet;          // set swr max for searching (exemple 32 for 3.2)
  byte OffsetdBmBridge;  //set offset compesation bridge in dbm
  byte L;             // L number (0-255)
  byte C;             // L number (0-255)
  byte Hz;            // hzlz relais
  byte saveAuto;      //save automaticly last search into eeprom
  float Pset;         //set power level to enable seraching  (exemple 05 for 0.5W)
  }AtuStruct;
AtuStruct Atu;    //declare la structure 1+1+4+1+1+1+1+1+4=15 bytes

#define AtuSize 15

#define LRAdr 0x20    //adresse i2c mcp23008
#define CRAdr 0x21

byte cval=0;
byte lval=0;
byte hzlz=0;

byte cmdOrder=0;
byte cmdValue=0;
byte cmdState=0;


float P;
byte Research=0;


void setup() {
  // set up the LCD's number of columns and rows: 
  Wire.begin(); 
  //  lcd.begin();
  //  lcd.backlight();
  Serial.begin(9600);
  // Print a message to the LCD.
  //  lcd.print("BOITE AUTO");
  //  lcd.print("F4GOH/F1BJD");
  //  pinMode(wakeup, INPUT);     
  analogReference(EXTERNAL);

  pinMode(balun_pin, OUTPUT);     
  pinMode(hzlz_pin, OUTPUT);     
  pinMode(rede_pin, OUTPUT);
  pinMode(led_red, OUTPUT);
  pinMode(led_green, OUTPUT);

  digitalWrite(balun_pin, LOW);
  digitalWrite(hzlz_pin, LOW);
  digitalWrite(rede_pin, LOW);
  digitalWrite(led_red, HIGH);
  digitalWrite(led_green, LOW);


  send_gpio(LRAdr, 0x00, 0x00);  //init gpio L to output
  send_gpio(CRAdr, 0x00, 0x00);  //init gpio C to output

  send_gpio(LRAdr, 0x09, 0x00);  //all gpio L at level 0
  send_gpio(CRAdr, 0x09, 0x00);  //all gpio C at level 0


    check_eeprom();
  #ifdef DEBUG_CMD
    commandDebug();
  #endif
}



void loop() {

  checkSwr();
  commandInput();
}

//--------------------------------------- gestion de la commande de déclenchement du mode recherche

void checkSwr()
{
  int s=mesureSWR();
  if (P>Atu.Pset) {
    digitalWrite(rede_pin, HIGH);
    if ((s>Atu.swrSet*10) && (Research==0)) {
      search_atu();
      Research=1;
    }
    SendSwr(s);           
    SendPwr();
    //delay(10);
    digitalWrite(rede_pin, LOW);
  }
  else Research=0;
  delay(measureInterval);
}

//--------------------------------------- gestion de l'EEPROM

void check_eeprom()
{
  if (EEPROM.read(0)!=0xfd) {
    Atu.id=0xfd;
    Atu.balun=0;
    Atu.swrSet=3;
    Atu.OffsetdBmBridge=47;
    Atu.L=0;
    Atu.C=0;
    Atu.Hz=0;
    Atu.saveAuto=0;
    Atu.Pset=0.5;
    SaveEeprom();
  }
  ReadEeprom();
  setAtu(Atu.L, Atu.C, Atu.Hz, Atu.balun);
}

void SaveEeprom()
{
  for (int i = 0; i < AtuSize; i++) {
    EEPROM.write(i, *((char*)&Atu + i));
  }
}

void ReadEeprom()
{
  for (int i = 0; i < AtuSize; i++) {
    *((char*)&Atu + i) = EEPROM.read(i);
  }
}


//--------------------------------------- routines i2c des drivers MCP23008

void setAtu(byte L, byte C, byte hz, byte balun)
{
  send_gpio(LRAdr, 0x09, L); 
  send_gpio(CRAdr, 0x09, swapp(C)); 
  digitalWrite(hzlz_pin, hz);
  digitalWrite(balun_pin, balun);
}



void LCUpDown(byte lu, int pas)
//Exemple : LCUpDown (1,-1) décemente C
//Exemple : LCUpDown (0,10) incrémente L de 10
{
  if (lu==0) {
    lval+=pas;
  }
  else 
  {
    cval+=pas;
  }
  send_gpio(LRAdr, 0x09, lval); 
  send_gpio(CRAdr, 0x09, swapp(cval));
}


byte swapp(byte cval)         //valable que pour platine L+C
{
  byte c=0;
  bitWrite(c, 0, bitRead(cval, 7));
  bitWrite(c, 1, bitRead(cval, 6));
  bitWrite(c, 2, bitRead(cval, 5));
  bitWrite(c, 3, bitRead(cval, 4));
  bitWrite(c, 4, bitRead(cval, 3));
  bitWrite(c, 5, bitRead(cval, 2));
  bitWrite(c, 6, bitRead(cval, 1));
  bitWrite(c, 7, bitRead(cval, 0));
  return c;
}

void send_gpio(byte adresse, byte registre, byte valeur)
{
  Wire.beginTransmission(adresse);      // start talking to the device
  Wire.write(registre);                   // select the IODIR register
  Wire.write(valeur);                   // set register value-all high, sets all pins as outputs on MCP23008
  Wire.endTransmission();            // stop talking to the devicevice
}


//--------------------------------------- recherche rapide et fine avec sortie immediate

#define delta_Lfine 8
#define delta_Cfine 16

void search_atu()
{
  int swr_min=100;
  int current_swr;

  int hz=0;
  int Lvalue;
  int Cvalue;

  
  Serial.write('R');  //démarrage d'une recherche large
  Serial.write('0'); 
  Serial.write('0'); 
  Serial.write(0x0a);
  waitTxComplete();
  
  do
  {
    Lvalue=0;
    do
    {
      Cvalue=0;
      do
      {
        digitalWrite(hzlz_pin, hzlz);
        send_gpio(LRAdr, 0x09, Lvalue); 
        send_gpio(CRAdr, 0x09, swapp(Cvalue));
        delay(searchDelay);
        current_swr=mesureSWR();
        SendSwr(current_swr);
        if (current_swr<swr_min) {
          cval=Cvalue;
          lval=Lvalue;
          hzlz=hz;
          swr_min=current_swr;
        }
        Cvalue=Cvalue*2+1;
      }
      while ( (Cvalue<=127)&&(swr_min>10));
      Lvalue=Lvalue*2+1;
    }
    while ( (Lvalue<=255)&&(swr_min>10)); 
    hz++;
  } 
  while ( (hz<=1)&&(swr_min>10));

  int Lstart;
  int Lstop;
  int Cstart;
  int Cstop;

  if (lval<delta_Lfine) {
    Lstart=lval;
    Lstop=lval+delta_Lfine;
  }
  else
    if ((lval>=delta_Lfine) && (lval<255)) {
      Lstart=lval-delta_Lfine;
      Lstop=lval+delta_Lfine;
    }
    else
    {
      Lstart=lval-delta_Lfine;
      Lstop=lval;
    }

  if (cval<delta_Cfine) {
    Cstart=cval;
    Cstop=cval+delta_Cfine;
  }
  else
    if ((cval>=delta_Cfine) && (cval<127)) {
      Cstart=cval-delta_Cfine;
      Cstop=cval+delta_Cfine;
    }
    else
    {
      Cstart=cval-delta_Cfine;
      Cstop=cval;
    }

  Serial.write('F');  //démarrage d'une recherche fine
  Serial.write('0'); 
  Serial.write('0'); 
  Serial.write(0x0a);
  waitTxComplete();

  Lvalue= Lstart;
  do
  {
    Cvalue=Cstart;
    do
    {
      digitalWrite(hzlz_pin, hzlz);
      send_gpio(LRAdr, 0x09, Lvalue); 
      send_gpio(CRAdr, 0x09, swapp(Cvalue));
      delay(searchDelay);
      current_swr=mesureSWR();
      SendSwr(current_swr);
      if (current_swr<swr_min) {
        cval=Cvalue;
        lval=Lvalue;
        swr_min=current_swr;
      }
      Cvalue++;
    }
    while ( (Cvalue<=Cstop)&&(swr_min>10));  
    Lvalue++;
  }
  while ( (Lvalue<=Lstop)&&(swr_min>10)); 

  digitalWrite(hzlz_pin, hzlz);
  send_gpio(LRAdr, 0x09, lval); 
  send_gpio(CRAdr, 0x09, swapp(cval));

  Atu.L=lval;
  Atu.C=cval;
  Atu.Hz=hzlz;
  if (Atu.saveAuto==1) SaveEeprom();

  SendLC(lval, cval, hzlz);
  SendSwr(swr_min);
  
}

// ----------------------------------------  mesure swr
int mesureSWR(void)
{

  int fwd_num =  analogRead(vf);  // puissance directe numérique
  int rev_num =  analogRead(vr);  // puissance reflechie numérique

  float fwd_dbm =   NtodBm(fwd_num)+Atu.OffsetdBmBridge;  // puissance directe dbm
  float rev_dbm =   NtodBm(rev_num)+Atu.OffsetdBmBridge;  // puissance reflechie dbm

  float Rho=0;
  float swr=0; 

  P=pow(10, fwd_dbm/10)/1000;    //calcul en W

  #ifdef DEBUG_DBM
   Serial.println("num---");
  Serial.println(fwd_num);
  Serial.println(rev_num);
  Serial.println("dbm---");
  Serial.println(fwd_dbm);
  Serial.println(rev_dbm);
  waitTxComplete();
  #endif


    if (rev_dbm < fwd_dbm) {
    Rho = (rev_dbm-fwd_dbm)/20;
    Rho = pow(10, Rho);
    swr = (1 + Rho)/(1 - Rho);
    #ifdef DEBUG_SWR
      Serial.print("rho ");
    Serial.println(Rho);
    Serial.print("swr ");
    Serial.print(swr);
    Serial.print(" / ");
    Serial.print(P);
    Serial.println(" W");
    waitTxComplete();         
    #endif
  }
  else {
    swr = 10; // pas de charge ou pas d'antenne
  }

  if (P>Atu.Pset) {
    if (swr > Atu.swrSet) {
      digitalWrite(led_green, LOW);    
      digitalWrite(led_red, HIGH);
    }
    else {
      digitalWrite(led_green, HIGH);
      digitalWrite(led_red, LOW);
    }
  }
  return (int) (swr * 10);
}

float NtodBm(int N)
{
  return  (100*(float) N)/1023-90;
}


/***********************************************************
 * protocole d'émission
 ********************************************************** */

void SendLC(byte L, byte C, byte hz)
{
  /*
   L(byte)  : valeur 8 bits de la commande des relais des selfs
   C(byte)  : valeur 8 bits de la commande des relais des selfs
   H(byte) : hzlz
   */
   Serial.write('L');
  if (L<10) Serial.print(0);
  Serial.print(L);
  Serial.write(0x0a);
  Serial.write('C');
  if (C<10) Serial.print(0);
  Serial.print(C);
  Serial.write(0x0a);
  Serial.write('H');
  Serial.print(0);
  Serial.print(hz);
  Serial.write(0x0a);
  waitTxComplete();
}


void SendSwr(int swr)
{
  /*
S45 : compte rendu de la recherche swr x10 => 4.5
   */
  Serial.write('S');
  if (swr>99) Serial.print("99"); 
  else Serial.print(swr);
  Serial.write(0x0a);
  waitTxComplete();
}

void SendPwr()
{
  /*
P000/999 : x10 -> 99.9W
   */
  if (P>Atu.Pset) {
    Serial.write('P');
    if (P<10) Serial.write('0');
    if (P<1) Serial.write('0');
    Serial.print((int)(P*10));
    Serial.write(0x0a);
    waitTxComplete();
  }
}

void commandInput()
{
  if (Serial.available() > 0) {
    readCommand(Serial.read());
  }
}

void readCommand(char car)
{
  switch (cmdState)
  {
  case 0 : 
    if (car==0xa) cmdState=0;
    else {
      cmdOrder=car;
      cmdState=1;
      // Serial.print("cmd: ");
      // Serial.println(cmdOrder);
    }
    break;
  case 1 : 
    if (car==0xa) cmdState=0;
    else {
      cmdValue=(car-0x30)*10;
      cmdState=2;
      // Serial.print("diz: ");
      // Serial.println(cmdValue);
    }
    break;
  case 2 : 
    if (car==0xa) cmdState=0;
    else {
      cmdValue=cmdValue+(car-0x30);
      cmdState=3;
      // Serial.print("uni :");
      // Serial.println(cmdValue);
    }
    break;
  case 3 : 
    if (car==0xa) commandExecute();
    cmdState=0;
    // Serial.print("exe :");
    // Serial.println(car,HEX);
    break;
  default : 
    cmdState=0;
  }
}

/*
chaque commande suivi de nouvelle ligne
 
 commande vers boite auto :
 b 00/01 : balun ou asymétrique
 w22 : seuil swr x10
 u 00/01 L/C UP
 d 00/01 L/C Down
 h 00/01 HzLz
 c 00 record config
 m 47 dbm pont
 n 00 effectue une recherche
 v 00/01 save auto
 i 00 init eeprom
 t 30 trig level Power
 r 00 read config
 
 
 */
//Exemple : LCUpDown (1,-1) décemente C
//Exemple : LCUpDown (0,10) incrémente L de 10

void commandExecute()
{
  switch (cmdOrder)
  {
  case 'b' : 
    Atu.balun=cmdValue;
    digitalWrite(balun_pin, Atu.balun);
    break;
  case 'w' : 
    Atu.swrSet=((float)cmdValue)/10;
    break;
  case 'u' : 
    if (cmdValue==0) {
      Atu.L++;
      LCUpDown (0, 1);
    }
    else
    {
      Atu.C++;
      LCUpDown (1, 1);
    }
    break;
  case 'd' : 
    if (cmdValue==0) {
      Atu.L--;
      LCUpDown (0, -1);
    }
    else
    {
      Atu.C--;
      LCUpDown (1, -1);
    }
    break;
  case 'h' : 
    Atu.Hz=cmdValue;
    digitalWrite(hzlz_pin, Atu.Hz);
    break; 
  case 'c' : 
    SaveEeprom();
    break;  
  case 'm' : 
    Atu.OffsetdBmBridge=cmdValue;
    break; 
  case 'n' : 
    search_atu();
    break; 
  case 'v' : 
    Atu.saveAuto=cmdValue;
    break;
  case 'i' : 
    EEPROM.write(0, 0xff);
    check_eeprom();
    break;      
  case 't' : 
    Atu.Pset=((float)cmdValue)/10;
    break;                                                          
  case 'r' : 
    digitalWrite(rede_pin, HIGH);
    SendLC(Atu.L, Atu.C, Atu.Hz);
    Serial.write('B');             //B 00/01 balun
    Serial.print(0);
    Serial.print(Atu.balun);
    Serial.write(0x0a);

    Serial.write('W');             //W 22   seuil swr
    Serial.print((int)(Atu.swrSet*10));   
    Serial.write(0x0a);

    Serial.write('M');             //M 47   dbm pont           
    Serial.print(Atu.OffsetdBmBridge);
    Serial.write(0x0a);

    Serial.write('V');             //S 01   save auto
    Serial.print(0);
    Serial.print(Atu.balun);
    Serial.write(0x0a);

    Serial.write('T');             //T 30   trig power level
    if (Atu.Pset<1)  Serial.print(0);
    Serial.print((int)(Atu.Pset*10));
    Serial.write(0x0a);
    waitTxComplete();
    digitalWrite(rede_pin, LOW);
    break;
  }
  #ifdef DEBUG_CMD
    commandDebug();
  #endif
}


void commandDebug()
{
  digitalWrite(rede_pin, HIGH);
  Serial.println("-----------");  
  Serial.print("balun: ");
  Serial.println(Atu.balun);
  Serial.print("swr set:");
  Serial.println(Atu.swrSet);
  Serial.print("offset bridge: ");
  Serial.println(Atu.OffsetdBmBridge);
  Serial.print("L: ");
  Serial.println(Atu.L);
  Serial.print("C: ");
  Serial.println(Atu.C);
  Serial.print("Hz: ");
  Serial.println(Atu.Hz);
  Serial.print("save auto: ");
  Serial.println(Atu.saveAuto);
  Serial.print("Pset: ");
  Serial.println(Atu.Pset);
  waitTxComplete();
  digitalWrite(rede_pin, LOW);
}

void waitTxComplete()
{
  while (!(UCSR0A & (1 << UDRE0)))  // Wait for empty transmit buffer
     UCSR0A |= 1 << TXC0;  // mark transmission not complete
 while (!(UCSR0A & (1 << TXC0)));   // Wait for the transmission to complete
 //delay(20);
}



