
/***
Permission is hereby granted, free of charge, to any person
obtaining a copy of this software and associated documentation
files (the "Software"), to deal in the Software without
restriction, including without limitation the rights to use,
copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the
Software is furnished to do so, subject to the following
conditions:

The above copyright notice and this permission notice shall be
included in all copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
OTHER DEALINGS IN THE SOFTWARE.
***/

#include "Arduino.h"
#include "bitlash.h"
#include "src/bitlash.h"
#include "PID_v1.h"

#include "Adafruit_MAX31855.h"


#define RelayPin 6
double gSetPoint = 0.0, gInput = 0.0, gOutput = 0.0;
PID gPID(&gInput, &gOutput, &gSetPoint, 2, 5, 1, DIRECT);
int gWindowSize = 1000;
unsigned long gWindowStartTime;

int thermoDO = 5;
int thermoCS = 4;
int thermoCLK = 3;

Adafruit_MAX31855 gThermocouple(thermoCLK, thermoCS, thermoDO);


void printDouble(double val, byte precision) {
  
  printInteger ( int(val), 0, '0');  //prints the int part
  if ( precision > 0) {
    sp(".");
    unsigned long frac;
    unsigned long mult = 1;
    byte padding = precision -1;
    while ( precision-- )
       mult *=10;
       
    if(val >= 0)
      frac = (val - int(val)) * mult;
    else
      frac = (int(val)- val ) * mult;
    unsigned long frac1 = frac;
    while( frac1 /= 10 )
      padding--;
    while(  padding--)
      sp("0");
    printInteger(frac, 0, '0') ;
  }

}

numvar debug_cmdline(void)
{
  for (int i=1; i<=getarg(0); i++)
  {
    sp("  ");
    if (isstringarg(i))
    {
      sp("s ");
      sp((char *)getstringarg(i));
    }
    else
    {
      sp("i ");
      printInteger(getarg(i), 0, ' ');
    }
    speol();
  }
}


//------------------------------
// PID interface functions begin
//------------------------------

numvar pidinfo(void)
{
  char buf[16];
  sp("PID info:");
  sp(" Kp(");  printDouble( gPID.GetKp(), 2 ); sp(")");
  sp(" Ki(");  printDouble( gPID.GetKi(), 2 ); sp(")");
  sp(" Kd(");  printDouble( gPID.GetKd(), 2 ); sp(")");
  sp(" SampleTime("); printInteger( gPID.SampleTime , 0, ' '); sp(")");
  sp(" outMin("); printDouble( gPID.outMin, 2); sp(")");
  sp(" outMax("); printDouble( gPID.outMax, 2); sp(")");
  speol();
  
  sp("  mode: ");
  if (gPID.GetMode() == AUTOMATIC)
    sp("automatic");
  else if (gPID.GetMode() == MANUAL)
    sp("manual");
  else sp("unknown");
  speol();
  
  sp("  direction: ");
  if (gPID.GetDirection() == DIRECT)
    sp("direct");
  else if (gPID.GetDirection() == REVERSE)
    sp("reverse");
  else sp("uknown");
  speol();
  
  sp("  SetPoint: ");  printDouble( gSetPoint, 3);  speol();
  sp("  Input: ");     printDouble( gInput, 3);     speol();
  sp("  Output: ");    printDouble( gOutput, 3);    speol();
}

numvar pidmode(void)
{
  char *s;
  
  if (getarg(0) != 1) return -1;

  if (!isstringarg(1)) return -1;
  s = (char *)getstringarg(1);
  if ( (s[0]=='a') && (s[1] =='u') &&
       (s[2]=='t') && (s[3] =='o') )
    gPID.SetMode(AUTOMATIC);
  else if ( (s[0] == 'm') && (s[1] == 'a') && (s[2] == 'n' ) )
    gPID.SetMode(MANUAL);
  else
    return -1;
  return 0;
}

numvar pidlimit(void)
{
  if (getarg(0) != 2) return -1;
  if (!isstringarg(1) || !isstringarg(2)) return -2;
  gPID.SetOutputLimits( atof((char *)getstringarg(1)), atof((char *)getstringarg(2)) );
  return 0;
  
}

numvar pidtune(void)
{
  if (getarg(0) != 3) return -1;
  if (!isstringarg(1) || !isstringarg(2) || !isstringarg(3)) return -2;
  gPID.SetTunings( atof((char *)getstringarg(1)), atof((char *)getstringarg(2)), atof((char *)getstringarg(3)));
  return 0;
}

numvar piddir(void)
{
  char *s;
  if (getarg(0) != 1) return -1;
  if (!isstringarg(1)) return -2;
  
  s = (char *)getstringarg(1);
  if ( (s[0] == 'd') && (s[1] == 'i') && (s[2] == 'r') )
    gPID.SetControllerDirection(DIRECT);
  else if ( (s[0] == 'r') && (s[1] == 'e') && (s[2] == 'v') )
    gPID.SetControllerDirection(REVERSE);
  else return -3;
  
  return 0;

}

numvar pidsample(void)
{
  if (getarg(0) != 1) {
    sp("invalid argument (expects 1)"); speol();
    return -1;
  }
  if (isstringarg(1)) {
    sp("argument is string"); speol();
    return -2;
  }
  gPID.SetSampleTime( getarg(1) );
  return 0;
}

numvar pidsetpoint(void)
{
  if (getarg(0) != 1)
  {
    sp("invalid argument (expects 1)"); speol();
    return -1;
  }
  if (!isstringarg(1)) {
    sp("invalid argument expects double as string"); speol();
    return -2;
  }
  gSetPoint = atof( (char *)getarg(1));
  return 0;
}

//----------------------------
// PID interface functions end
//----------------------------



//-----------------------------------
// Max31855 interface functions begin
//-----------------------------------

numvar temperature(void)
{
  double c;
  c = gThermocouple.readCelsius();
  printDouble(c, 3);
  sp(" C");
  speol();
}

//---------------------------------
// Max31855 interface functions end
//---------------------------------



void setup(void) {
  initBitlash(57600);    // must be first to initialize serial port


  // Setup functions
  addBitlashFunction("debug", (bitlash_function) debug_cmdline);
  addBitlashFunction("pidinfo", (bitlash_function) pidinfo);
  addBitlashFunction("pidlimit", (bitlash_function) pidlimit);
  addBitlashFunction("pidmode", (bitlash_function) pidmode);
  addBitlashFunction("piddir", (bitlash_function) piddir);
  addBitlashFunction("pidtune", (bitlash_function) pidtune);
  addBitlashFunction("pidsample", (bitlash_function) pidsample);
  addBitlashFunction("pidsetpoint", (bitlash_function) pidsetpoint);
  
  
  addBitlashFunction("temp", (bitlash_function) temperature);


  // setup PID
  gWindowStartTime = millis();
  gSetPoint = 100;
  gPID.SetMode(AUTOMATIC);
  gPID.SetOutputLimits(0, gWindowSize);
  
}

void loop(void)
{
  unsigned long now;

  gInput = gThermocouple.readCelsius();

  runBitlash();

  now = millis();
  //gInput = analogRead(0);
  gPID.Compute();
  if ( (now - gWindowStartTime) > gWindowSize)
  {
    gWindowStartTime += gWindowSize;
  }
  if (gOutput > now - gWindowStartTime)
    digitalWrite(RelayPin, HIGH);
  else
    digitalWrite(RelayPin, LOW);

}
