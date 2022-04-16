#include <dht.h>
#include <LiquidCrystal_I2C.h>
#include <PID_v1.h>
#define _uno
#include <PWM.h>
#include "Plotter.h"
//Plotter pvsp;
//#include <LcdBarGraphX.h>
//LiquidCrystal_I2C lcd(0x3F, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE); Alte lib!
LiquidCrystal_I2C lcd(0x27,20,4);
dht DHT;
#define DHT11_PIN 7                   //Raumtemp
double setpoint;                      //Der PID brauch alles als double
double processvalue;                  // PV von temp1
double output;
double outputPercent;                 //OP %
double outputrange;
double LASTSP;                        // Letzten SP merken fuer Abgleich


const int ntc = A0;                   // wassertemp pin
const int raumtemp = A1;                   // raumtemp pin
const int ntcNominal = 10000;         // Widerstand des NTC bei Nominaltemperatur //War 10000
const int tempNominal = 25;           // Temperatur bei der der NTC den angegebenen Widerstand hat
const int bCoefficient = 3977;        // Beta Coefficient(B25 aus Datenblatt des NTC)

const int serienWiderstand = 6800;   // Wert des Widerstands der mit dem NTC in Serie geschalten ist
const int abfrageZahl = 10;            // Je mehr abfragen, desto stabiler isr das Ergebnis, dauert aber länger
const int tachoPin = 2;               // Pin des Tachosignals des Lüfters
const int pwm = 9 ;  //initializing pin 9 as pwm
float rps = 0;                        // Variable mit Kommastelle für die Berechnung der Umdrehungen pro Sekunde
int rpm = 0;                          // Variable für die gemittelte Drehzahl
float umdrZeit = 0;                   // Variable mit Kommastelle für die Zeit pro Umdrehung des Lüftes
float flankenZeit = 0;                // Variable mit Kommastelle für die Zeit pro Puls des Lüfters
int abfrZeit = 2000;                  // Zeitabstand für die Abfragen des Tachosignals
long tachoMillis = abfrZeit;          // Zeitabstand für Pulse Stretching Funktion
long displayMillis = abfrZeit;        // Zeitabstand für Pulse Stretching Funktion
int fanSpeed = 0;                    // Variable für die Lüftergeschwindigkeit
int fanMin = 100;                    // Kleinster PWM Wert für den Lüfter befor er abschält
int fanOut = 1;                      // Variable zum pürfen ob der Lüfter aus war
int tMin = 20;                       // Untere Grenze des Temperaturbereichs
int tMax = 60;                      // Obere Grenze des Temperaturbereichs
int abfrage[abfrageZahl];            // Array Variable für das Mitteln der Temperatur
float durchschnitt = 0;            // Variable für das Mitteln der Temperatur
PID luefterPID(&processvalue, &output, &setpoint, 5, 0.2, 0.5, REVERSE);
int32_t frequency = 25000; // Freqzenz fuer Luefter, bei den jetzigen egal, wenn gute verbaut sind: wichtigwichtig!
static unsigned long previousMillis = 1000;//serial Verzoegerung
void setup()
{
  InitTimersSafe();
  setpoint = 36,000000000;
  Serial.begin(9600);
  luefterPID.SetMode(AUTOMATIC);
  //luefterPID.SetSampleTime(200);
  pinMode(ntc, processvalue);            // Setzt den Pin des NTC Widerstands als Eingang
  pinMode(tachoPin, INPUT);
  pinMode(pwm, output);
  lcd.init();
  lcd.begin(20, 4);
  lcd.setCursor(4, 1);
  lcd.print("booting");
    lcd.backlight();
  delay(5000);
  lcd.clear();
  // pvsp.Begin(); // start plotter
  // pvsp.AddTimeGraph( "PV", 200, "PV", processvalue, "SP", setpoint);
SetPinFrequencySafe(output, frequency);
}

void loop()
{

  Temperaturberechnung();      // Startet die Temperaturerfassungsroutine
  //Setpoint();
  LuefterPID();      // PID
  Display();        //Displayausgabe

  // if((millis() - tachoMillis) >= abfrZeit)
  // {
  Drehzahl();        //Displayausgabe
  //  }
  //  pvsp.Plot(); // Plot abrufbar mit arduino-plotter/listener  https://github.com/devinaconley/arduino-plotter/blob/master/README.md
  serial();
}

void Setpoint() //Setpointberechnung
{
int raumtemp = DHT.temperature;
setpoint = 36;

  //setpointbegrenzung
    if (setpoint >= 45) {
      setpoint = 45;
    }
  }

void Temperaturberechnung() //PV
{
  // Nimmt N Abfragen in einer Reihe, mit einem kurzen delay
  for (int i = 0; i < abfrageZahl; i++)
  {
    abfrage[i] = analogRead(ntc);
    delay(10);
  }

  // Mittelt alle Abfragen
  durchschnitt = 0;
  for (int i = 0; i < abfrageZahl; i++)
  {
    durchschnitt += abfrage[i];
  }
  durchschnitt /= abfrageZahl;

  // Umwandlung des Wertes in Widerstand
  durchschnitt = 1023 / durchschnitt - 1;
  durchschnitt = serienWiderstand / durchschnitt;

  // Umrechnung aller Ergebnisse in die Temperatur mittels einer Steinhard Berechnung
  processvalue = durchschnitt / ntcNominal;     // (R/Ro)
  processvalue = log(processvalue);                     // ln(R/Ro)
  processvalue /= bCoefficient;                 // 1/B * ln(R/Ro)
  processvalue += 1.0 / (tempNominal + 273.15); // + (1/To)
  processvalue = 1.0 / processvalue;                    // Invertieren
  processvalue -= 273.15;                       // Umwandeln in °C
}


void LuefterPID()
{

  luefterPID.Compute();
  luefterPID.SetControllerDirection(REVERSE);

  luefterPID.SetTunings(4, 1, 1) ;  // SetTunings(P, I, D)
  //luefterPID.SetoutputLimits(57, 255);


  outputrange = map(output, 0, 255, 26, 255);




  pwmWrite(9,outputrange) ; 
 outputPercent = map(outputrange, 0, 255, 0, 100);   //Mapping EU nach OP, nur zur Displayanzeige
}
 
void Display() {

  lcd.setCursor ( 0, 0 );
  lcd.print("PV : ");
  lcd.print(processvalue);
  lcd.print((char)223);
  lcd.print("C");
  if (LASTSP != SP) { //An der Stelle flackert es wenn man das nicht tut.
    lcd.setCursor ( 0, 1 );
    lcd.print ("SP : ");
    lcd.print (setpoint);
  }
  lcd.setCursor ( 0, 2 );

  lcd.print ("OP : ");
  lcd.print (outputPercent);
  lcd.setCursor(0, 3);


  lcd.print("RPM: ");
  lcd.print(rpm);
  lcd.print(" ");

  //raumtemp
  int chk = DHT.read11(DHT11_PIN);
  lcd.setCursor(14, 0);
  lcd.print("Raum: ");
  lcd.setCursor(14, 1);
  lcd.print(DHT.temperature);
  lcd.print((char)223);
  lcd.setCursor(14, 2);
  lcd.print("rF: ");
  lcd.setCursor(14, 3);
  lcd.print(DHT.humidity);
  lcd.print("%");
  //ende raumtemp

  /*  Serial.print("PV--> ");
    Serial.println(processvalue);



    Serial.print("SP --> ");
    Serial.println(setpoint);

    Serial.print("OP --> ");
    Serial.println(output);
    Serial.println("------");

      Serial.print("rpm --> ");
    Serial.println(rpm);
    Serial.println("------");

          Serial.print("Raum --> ");
    Serial.println(DHT.temperature);
    Serial.println("------");*/
  delay (10);
  LASTSP = SP;
  displayMillis = millis();      // Die displayMillis werden aktualisiert um die nächsten 2000ms zählen zu können
}

void Drehzahl(void) {
  flankenZeit = pulseIn(tachoPin, HIGH);    // Abfrage der Zeit pro Puls in Mikrosekunden
  umdrZeit = ((flankenZeit * 4) / 1000);    // Berechnung der Zeit pro Umdrehung in Millisekunden
  rps = (1000 / umdrZeit);                  // Umrechnung auf Umdrehungen pro Sekunde
  rpm = (rps * 60);                         // Umrechnung auf Umdrehungen pro Minute
  tachoMillis = millis();                  // Die TachoMillis werden aktualisiert um die nächsten 2000ms zählen zu können
}

void serial(void) {
  unsigned long currentMillis = millis();
  if ((currentMillis - previousMillis) >= 4000) {
    Serial.println ();
    Serial.print ("SP: ");
    Serial.print (setpoint, 0);
    Serial.print (" | PV: ");
    Serial.print (processvalue, 1);
    Serial.print (" | OP: ");
    Serial.print (outputPercent, 0);
    Serial.print (" | RPM: ");
    Serial.print (rpm);
    Serial.print (" | Raum: ");
    Serial.print (DHT.temperature, 0);
    Serial.print (" | rF: ");
    Serial.print (DHT.humidity, 0);
    previousMillis = currentMillis;
  }
}
