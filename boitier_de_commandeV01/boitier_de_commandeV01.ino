/*
  Boitier de commande pour la boite de couplage automatique
  version boitier étanche avec com RS485
 
  V0.1 : affichage de la puissance et swr

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
P999 : x10  power level
B 00/01 balun
W 22   seuil swr
M 47   dbm pont
V 01   save auto
T 30   trig power level

*/

// include the library code:

#include <EEPROM.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>

//#define DEBUG_CMD
//#define DEBUG_SWR
//#define DEBUG_DBM

#define SOFTSERIAL



// initialize the lcd 
LiquidCrystal_I2C lcd(0x27, 20, 4);



// init keypad
int adc_key_in  = 0;

#define rede_pin  4
#define tx_soft_pin  7
#define rx_soft_pin  8

SoftwareSerial atuSerial(rx_soft_pin, tx_soft_pin); // RX, TX

#define key_pin A0


typedef struct  {
  byte id;            //indentification match box default 0xfd
  byte balun;         // Balun active or not
  int swrSet;          // set swr max for searching (exemple 32 for 3.2)
  byte OffsetdBmBridge;  //set offset compesation bridge in dbm
  byte L;             // L number (0-255)
  byte C;             // L number (0-255)
  byte Hz;            // hzlz relais
  byte saveAuto;      //save automaticly last search into eeprom
  int Pset;         //set power level to enable seraching  (exemple 05 for 0.5W)
  byte swrPrec;
  byte swr;            //read swr from match box  
  int pPrec;              //read power from box
  int p;              //read power from box
  }AtuStruct;
AtuStruct Atu;    //declare la structure



byte cmdOrder=0;
int cmdValue=0;
byte cmdState=0;

byte car0[8] = {B00000,B00000,B00000,B00000,B00000,B00000,B00000,B00000};  //declare en global pour allez plus vite
byte car1[8] = {B10000,B10000,B10000,B10000,B10000,B10000,B10000,B10000};
byte car2[8] = {B11000,B11000,B11000,B11000,B11000,B11000,B11000,B11000};
byte car3[8] = {B11100,B11100,B11100,B11100,B11100,B11100,B11100,B11100};
byte car4[8] = {B11110,B11110,B11110,B11110,B11110,B11110,B11110,B11110};
byte car5[8] = {B11111,B11111,B11111,B11111,B11111,B11111,B11111,B11111};


void setup() {
  // set up the LCD's number of columns and rows: 
  Wire.begin(); 
  Wire.setClock(400000);
  lcd.begin();
  lcd.backlight();

  lcd.createChar(0, car0);
  lcd.createChar(1, car1);
  lcd.createChar(2, car2);
  lcd.createChar(3, car3);
  lcd.createChar(4, car4);
  lcd.createChar(5, car5);
  
  Serial.begin(115200);
  atuSerial.begin(9600);
  // Print a message to the LCD.
  //  lcd.print("BOITE AUTO");
  //  lcd.print("F4GOH/F1BJD");


  pinMode(rede_pin, OUTPUT);
  digitalWrite(rede_pin, LOW);

  Atu.swrPrec=0;
  Atu.swr=0;
  Atu.pPrec=0;
  Atu.p=0;



 CommandSend('r',0);

/*
    check_eeprom();
  #ifdef DEBUG_CMD
    commandDebug();
  #endif
  */
}

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
lcd.print(Atu.Pset/10);
lcd.print(F("."));
lcd.print(Atu.Pset%10);
lcd.print(F("W"));
lcd.setCursor(14,3);
lcd.print(Atu.swrSet/10);
lcd.print(F("."));
lcd.print(Atu.swrSet%10);
delay(2000);
printSearchmenu();
}

void printSearchmenu()
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("SWR"));
  lcd.setCursor(0, 1);
  lcd.print(F("Power:"));
  lcd.setCursor(0, 2);
  lcd.print(F("Large:"));
  lcd.setCursor(0, 3);
  lcd.print(F("Fine :"));
}


void loop() {

  PrintSwr();
  commandInput();
}

//--------------------------------------- gestion de la commande d'affichage du swr

void PrintSwr()
{
if (Atu.swrPrec!=Atu.swr){
lcd.setCursor(5,0);
lcd.print(Atu.swr/10);
lcd.print(F("."));
lcd.print(Atu.swr%10);
//lcd.print("  ");
Atu.swrPrec=Atu.swr;
Bargraph(Atu.swr,2,1);
}
if (Atu.pPrec!=Atu.p){
lcd.setCursor(7,1);
lcd.print(Atu.p/10);
lcd.print(F("."));
lcd.print(Atu.p%10);
lcd.print("W  ");
Atu.pPrec=Atu.p;
}

}


void Bargraph(int swr, int ligne_curseur, byte large_or_fine)
{
int curseur_map;

 // curseur swr depart 6 fin 19 (1ong car 19-6+1=14) 5*14=70 avec -1 marge pour pas déborder
if (large_or_fine==1){
        if (swr>=100) swr=100;                //changement d'échelle pour recherche large (swr max 10)
        curseur_map=map(swr,10,100,1,70-1);    
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

//--------------------------------------- gestion de l'EEPROM pour memorisation du call

void callSignEeprom()
{
  //EEPROM.read(0);
  // EEPROM.read(i);
}




/***********************************************************
 * protocole d'émission
 ********************************************************** */


void commandInput()
{
  char car;

if (atuSerial.overflow())  Serial.println("SoftwareSerial overflow!");
while (atuSerial.available()>0)
{
    car=atuSerial.read();
    Serial.write(car);    
    readCommand(car);
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
      cmdValue=(car-0x30);
      cmdState=2;
      // Serial.print("diz: ");
      // Serial.println(cmdValue);
    }
    break;
  case 2 : 
    if (car==0xa) cmdState=0;
    else {
      cmdValue=(cmdValue*10)+(car-0x30);
      if (cmdOrder=='P') cmdState=3; else cmdState=4;
      // Serial.print("uni :");
      // Serial.println(cmdValue);
    }
    break;
  case 3 : 
    if (car==0xa) cmdState=0;
    else {
      cmdValue=(cmdValue*10)+(car-0x30);
      cmdState=4;
      // Serial.print("uni :");
      // Serial.println(cmdValue);
    }
    break;
  case 4 : 
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
 
boite auto vers commande :
- S45 :  swr x10
R00 : démarrage d'une recherche large
F00 : démarrage d'une recherche fine
- L(byte)  : valeur 8 bits de la commande des relais des selfs
- C(byte)  : valeur 8 bits de la commande des relais des selfs
- H(byte) : hzlz
- P999 : x10  power level
- B 00/01 balun
- W 22   seuil swr
- M 47   dbm pont
- V 01   save auto
- T 30   trig power level
 */

void commandExecute()
{
  switch (cmdOrder)
  {
  case 'B' : 
    Atu.balun=cmdValue;    
    break;
  case 'W' : 
    Atu.swrSet=cmdValue;
    break;
  case 'T' : 
    Atu.Pset=cmdValue;
    printParameters();
  #ifdef DEBUG_CMD
    commandDebug();
  #endif
    break;
  case 'H' : 
    Atu.Hz=cmdValue;
    break; 
   case 'M' : 
    Atu.OffsetdBmBridge=cmdValue;
    break; 
   case 'V' : 
    Atu.saveAuto=cmdValue;
    break;
    case 'L' : 
    Atu.L=cmdValue;
    break;                                                          
   case 'C' : 
    Atu.L=cmdValue;
    break;                                                          
   case 'S' : 
    Atu.swr=cmdValue;
    break;  
    case 'P' : 
    Atu.p=cmdValue;
    break;                                                          
  }
  //Serial.println(cmdValue);
}


void commandDebug()
{
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
  Serial.print("swr mes: ");
  Serial.println(Atu.swr);
  Serial.print("Pw mes: ");
  Serial.println(Atu.p);
  
}


void CommandSend(char cmdOrder, byte cmdValue)
{
 digitalWrite(rede_pin, HIGH);
 atuSerial.write(cmdOrder);
 atuSerial.write((cmdValue%10)+0x30);
 atuSerial.write((cmdValue/10)+0x30);
 atuSerial.write(0x0a);
 waitTxComplete();
 digitalWrite(rede_pin, LOW); 
  
}

void waitTxComplete()
{
 #ifdef SOFTSERIAL
 atuSerial.flush();
 delay(5);
 #else
 while (!(UCSR0A & (1 << UDRE0)))  // Wait for empty transmit buffer
 UCSR0A |= 1 << TXC0;  // mark transmission not complete
 while (!(UCSR0A & (1 << TXC0)));   // Wait for the transmission to complete
 #endif
}

