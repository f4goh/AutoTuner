/*
  Boite de couplage automatique
  version boitier étanche avec com RS485
 
  V0.8 : recherche a partir de l'ATU balise
  2x 8 relais attention actifs au NL1
  2x 2 relais attention actifs au NL0 (en fonction du modèle)
  debug rs485 en cours
  
  
  

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

//#define AFFLCD

#include <EEPROM.h>
#include <Wire.h> 


#ifdef AFFLCD
 #include <LiquidCrystal_I2C.h>
#endif


//#define DEBUG_CMD
//#define DEBUG_SWR
//#define DEBUG_DBM

#define measureInterval 100
#define searchDelay 20
//#define searchDelay 1

// initialize the lcd 
//LiquidCrystal_I2C lcd(0x27, 20, 4);

// init keypad
int lcd_key     = 0;
int adc_key_in  = 0;
#define balun_pin 3
#define hzlz_pin  2
#define rede_pin  4
#define tx_soft_pin  7
#define rx_soft_pin  8

#define led_red 12
#define led_green 13

#define vf A0
#define vr A1

#define relaylEnable HIGH
#define relaylDisable LOW

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

#define McpPcf 1        //0 MCP, 1 PCF

#define mcpLRAdr 0x20    //adresse i2c mcp23008
#define mcpCRAdr 0x21

// PCF8574  Address range is 0x20-0x27
// PCF8574A Address range is 0x38-0x3F

#define pcfLRAdr 0x38    //adresse i2c PCF8574AT
#define pcfCRAdr 0x20    //adresse i2c PCF8574T

#ifdef AFFLCD
  LiquidCrystal_I2C lcd(0x27, 20, 4);

  byte car0[8] = {B00000,B00000,B00000,B00000,B00000,B00000,B00000,B00000};  //declare en global pour allez plus vite
  byte car1[8] = {B10000,B10000,B10000,B10000,B10000,B10000,B10000,B10000};
  byte car2[8] = {B11000,B11000,B11000,B11000,B11000,B11000,B11000,B11000};
  byte car3[8] = {B11100,B11100,B11100,B11100,B11100,B11100,B11100,B11100};
  byte car4[8] = {B11110,B11110,B11110,B11110,B11110,B11110,B11110,B11110};
  byte car5[8] = {B11111,B11111,B11111,B11111,B11111,B11111,B11111,B11111};

typedef struct  {
  byte swrPrec;
  byte swr;            //read swr from match box  
  int pPrec;              //read power from box
  int p;              //read power from box
  }AtuLcdStruct;
AtuLcdStruct AtuLcd;    //declare la structure


#endif



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
  Serial.begin(9600);
  analogReference(EXTERNAL);

  pinMode(balun_pin, OUTPUT);     
  pinMode(hzlz_pin, OUTPUT);     
  pinMode(rede_pin, OUTPUT);
  pinMode(led_red, OUTPUT);
  pinMode(led_green, OUTPUT);

  digitalWrite(balun_pin, relaylDisable);
  digitalWrite(hzlz_pin, relaylDisable);
  digitalWrite(rede_pin, LOW);
  digitalWrite(led_red, HIGH);
  digitalWrite(led_green, LOW);

  #ifdef AFFLCD
    lcd.begin();
    lcd.setBacklight(HIGH);    
    lcd.clear();
    lcd.print(F(" AUTOMATIC ATU v1.0"));    //intro
    lcd.setCursor(0, 2);
    lcd.print(F("     F4GOH 2016"));
    delay(4000);
    lcd.clear();
    lcd.createChar(0, car0);
    lcd.createChar(1, car1);
    lcd.createChar(2, car2);
    lcd.createChar(3, car3);
    lcd.createChar(4, car4);
    lcd.createChar(5, car5);
  #endif
  

if (McpPcf==0){
  sendGpioMcp(mcpLRAdr, 0x00, 0x00);  //init gpio L to output
  sendGpioMcp(mcpCRAdr, 0x00, 0x00);  //init gpio C to output

  sendGpioMcp(mcpLRAdr, 0x09, 0xff);  //all gpio L at level 0
  sendGpioMcp(mcpCRAdr, 0x09, 0xff);  //all gpio C at level 0
}
else
{
  sendGpioPCF(pcfLRAdr,0xff);
  sendGpioPCF(pcfCRAdr,0xff);
}


    check_eeprom();
  #ifdef DEBUG_CMD
    commandDebug();
  #endif

   #ifdef AFFLCD
  printParameters();
  delay(2000);
  printSearchmenu();
  #endif
}



void loop() {

  checkSwr(0);
  commandInput();

}

//--------------------------------------- gestion de l'affichage
#ifdef AFFLCD
void printParameters()
{
lcd.setCursor(0,0); 
lcd.print(F("Balun:    Save:"));
lcd.setCursor(0,1); 
lcd.print(F("Pont:  dbm"));
lcd.setCursor(0,2); 
lcd.print(F("Trig power:"));
lcd.setCursor(0,3); 
lcd.print(F("Swr max level:"));

lcd.setCursor(6,0);
if (Atu.balun==1) lcd.print(F("on")); else lcd.print(F("off"));
lcd.setCursor(15,0);
if (Atu.saveAuto==1) lcd.print(F("auto")); else lcd.print(F("none"));
lcd.setCursor(5,1);
lcd.print(Atu.OffsetdBmBridge);
lcd.setCursor(11,2);
lcd.print((int)Atu.Pset/10);
lcd.print(F("."));
lcd.print((int) Atu.Pset%10);
lcd.print(F("W"));
lcd.setCursor(14,3);
lcd.print((int)Atu.swrSet/10);
lcd.print(F("."));
lcd.print((int)Atu.swrSet%10);
delay(2000);
printSearchmenu();
}

void printSearchmenu()
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("SWR"));
  lcd.setCursor(0, 2);
  lcd.print(F("Power:"));
}

//--------------------------------------- gestion de la commande d'affichage du swr

void PrintSwr()
{

if (AtuLcd.swrPrec!=AtuLcd.swr){
lcd.setCursor(5,0);
lcd.print((int) AtuLcd.swr/10);
lcd.print(F("."));
lcd.print((int) AtuLcd.swr%10);
//lcd.print("  ");
//Atu.swrPrec=Atu.swr;
Bargraph(AtuLcd.swr,1,1);
}
}


void PrintPwr()
{
if (AtuLcd.pPrec!=AtuLcd.p){
lcd.setCursor(7,2);
lcd.print(AtuLcd.p/10);
lcd.print(F("."));
lcd.print(AtuLcd.p%10);
lcd.print("W  ");
AtuLcd.pPrec=AtuLcd.p;
}

}



void Bargraph(int swr, int ligne_curseur, byte large_or_fine)
{
int curseur_map;

 // curseur swr depart 6 fin 19 (1ong car 19-6+1=14) 5*14=70 avec -1 marge pour pas déborder
if (large_or_fine==1){
        if (swr>=70) swr=70;                //changement d'échelle pour recherche large (swr max 10)
        curseur_map=map(swr,10,100,1,100-1);    
      }
      else
      {
        if (swr>=40) swr=40;                  //changement d'échelle pour recherche fine (swr max 4)
        curseur_map=map(swr,10,40,1,70-1);
      }
int curseur_entier=curseur_map/5;        //un caractère 5 de large
int curseur_modulo=curseur_map%5;        //modulo
int start_curseur=6;                     //position de départ en colonne sur l'afficheur
int end_curseur=6+curseur_entier;        //position de fin en caractère plein
while(start_curseur!=end_curseur)        //remplissage
{
  lcd.setCursor(start_curseur, ligne_curseur);
  lcd.write(byte(5));
  start_curseur++;
}
  lcd.setCursor(start_curseur, ligne_curseur);  //positionne le caractère modulo 5
  lcd.write(byte(curseur_modulo));
  start_curseur++;
  while(start_curseur!=20)                      //complète jusqu'a 19 en caractère vide
  {
  lcd.setCursor(start_curseur, ligne_curseur); 
  lcd.write(byte(0));
  start_curseur++;
  }
/*  lcd.setCursor(8,3);    //affiche le résulat courant
  lcd.print(swr / 10);
  lcd.print(F("."));
  lcd.print(swr % 10);
  lcd.print(F("  "));
  */
}


 #endif

//--------------------------------------- gestion de la commande de déclenchement du mode recherche

void checkSwr(byte force)
{
  int s=mesureSWR();
   if ((P>Atu.Pset) || (force==1)) {
    digitalWrite(rede_pin, HIGH);
    if (((s>Atu.swrSet*10) && (Research==0)) || (force==1)) {
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
  sendGpio(~L,~C);
  
 // sendGpioMcp(mcpLRAdr, 0x09, ~L); 
 // sendGpioMcp(mcpCRAdr, 0x09, ~C); 
  //digitalWrite(hzlz_pin, hz^1);
  //digitalWrite(balun_pin, balun^1);

  digitalWrite(hzlz_pin, hz);
  digitalWrite(balun_pin, balun);

}




/*
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
*/


void sendGpio(byte Lvalue,byte Cvalue)
{
  if (McpPcf==0){
      sendGpioMcp(mcpLRAdr, 0x09, Lvalue); 
      sendGpioMcp(mcpCRAdr, 0x09, Cvalue);
  }
  else
  {
    sendGpioPCF(pcfLRAdr,Lvalue);
    sendGpioPCF(pcfCRAdr,Cvalue);
  }
}

void sendGpioMcp(byte adresse, byte registre, byte valeur)
{
  Wire.beginTransmission(adresse);      // start talking to the device
  Wire.write(registre);                   // select the IODIR register
  Wire.write(valeur);                   // set register value-all high, sets all pins as outputs on MCP23008
  Wire.endTransmission();            // stop talking to the devicevice
}

void sendGpioPCF(byte adresse, byte valeur)
{
  Wire.beginTransmission(adresse);      // start talking to the device
  Wire.write(valeur);                   // select the IODIR register
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
        digitalWrite(hzlz_pin, hz^1);
        sendGpio(~Lvalue,~Cvalue);
        //sendGpioMcp(mcpLRAdr, 0x09, ~Lvalue); 
        //sendGpioMcp(mcpCRAdr, 0x09, ~Cvalue);
        delay(searchDelay);
        SendSwr(current_swr);
        current_swr=mesureSWR();
        //SendSwr(current_swr);
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

digitalWrite(hzlz_pin, hzlz^1);

  Lvalue= Lstart;
  do
  {
    Cvalue=Cstart;
    do
    {
      sendGpio(~Lvalue,~Cvalue);
      //sendGpioMcp(mcpLRAdr, 0x09, ~Lvalue); 
      //sendGpioMcp(mcpCRAdr, 0x09, ~Lvalue);
      delay(searchDelay);
       SendSwr(current_swr);
      current_swr=mesureSWR();
      //SendSwr(current_swr);
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

  //digitalWrite(hzlz_pin, hzlz^1);
  sendGpio(~lval,~cval);
  //sendGpioMcp(mcpLRAdr, 0x09, ~lval); 
  //sendGpioMcp(mcpCRAdr, 0x09, ~cval);

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
#ifdef AFFLCD
  AtuLcd.swr=swr;         //testale
  PrintSwr();
 #endif
  
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
    #ifdef AFFLCD
     AtuLcd.p=P*10; 
     PrintPwr();  
    #endif
    
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
    //digitalWrite(balun_pin, Atu.balun^1);
    digitalWrite(balun_pin, Atu.balun);
    break;
  case 'w' : 
    Atu.swrSet=((float)cmdValue)/10;
    break;
  case 'u' : 
    if (cmdValue==0) {
      Atu.L++;
      //LCUpDown (0, 1);      
    }
    else
    {
      Atu.C++;
      //LCUpDown (1, 1);
    }
     sendGpio(~Atu.L,~Atu.C);
    break;
  case 'd' : 
    if (cmdValue==0) {
      Atu.L--;
     // LCUpDown (0, -1);
    }
    else
    {
      Atu.C--;
     // LCUpDown (1, -1);
    }
    sendGpio(~Atu.L,~Atu.C);
    break;
  case 'h' : 
    Atu.Hz=cmdValue;
    //digitalWrite(hzlz_pin, Atu.Hz^1);
    digitalWrite(hzlz_pin, Atu.Hz);
    break; 
  case 'c' : 
    SaveEeprom();
    break;  
  case 'm' : 
    Atu.OffsetdBmBridge=cmdValue;
    break; 
  case 'n' : 
    //search_atu();  //hs car pas de cde rs485
    checkSwr(1);    // test force swr
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

void scanI2C()
{
Serial.println ();
  Serial.println ("I2C scanner. Scanning ...");
  byte count = 0;
 
  Wire.begin();
  for (byte i = 8; i < 120; i++)
  {
    Wire.beginTransmission (i);
    if (Wire.endTransmission () == 0)
      {
      Serial.print ("Found address: ");
      Serial.print (i, DEC);
      Serial.print (" (0x");
      Serial.print (i, HEX);
      Serial.println (")");
      count++;
      delay (1);  // maybe unneeded?
      } // end of good response
  } // end of for loop
  Serial.println ("Done.");
  Serial.print ("Found ");
  Serial.print (count, DEC);
  Serial.println (" device(s).");
}


