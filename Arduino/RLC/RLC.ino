#include <Adafruit_GFX.h>    
#include <Adafruit_ST7735.h> 
#include <SPI.h>

#define TFT_CS        10
#define TFT_RST        -1 //brak korzystamy wyprowadzenia rst
#define TFT_DC         12
#define TFT_MOSI 11  
#define TFT_SCLK 13  

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);

#define pulseO  3
#define pulseI  9

#define ModePin A7
#define Rref    A0
#define Rread   A1

#define Cread         A4
#define Cdischarge    A3
#define Cout          A2
#define Ccharge       A5

#define S0      4
#define S1      5
#define S2      6

#define CR    9910


float tryb = 0;
int flag = 0;
int ref = 0;  //A0
int rx[7];    //A1
float r7[]={507.4, 2173.0, 5053.0, 10020.0, 100900.0, 468600.0, 1003000.0};
int ind = 0;
float val = 0;
char result[9];

unsigned long startTime;
unsigned long elapsedTime;

double c = 0.995E-6;  
double puls ;
double freq ;


int controlPin[] = {S0, S1, S2};
int muxChannel[8][3]={
  {0,0,0}, //kanał 0
  {1,0,0}, //kanał 1
  {0,1,0}, //kanał 2
  {1,1,0}, //kanał 3
  {0,0,1}, //kanał 4
  {1,0,1}, //kanał 5
  {0,1,1}, //kanał 6
  {1,1,1}, //kanał 7
};
 
void drawtext(char *text, uint16_t *x, uint16_t *Stxt, uint16_t *s);


void setup() {
  
  tft.initR(INITR_BLACKTAB);
  tft.fillScreen(ST7735_BLACK);
  Serial.begin(9600);
  pinMode(S0, OUTPUT); 
  pinMode(S1, OUTPUT); 
  pinMode(S2, OUTPUT);
  digitalWrite(S0, LOW);
  digitalWrite(S1, LOW);
  digitalWrite(S2, LOW);

  pinMode(pulseO, OUTPUT);
  pinMode(pulseI, INPUT);
  tft.setTextColor(ST77XX_WHITE,ST77XX_BLACK);
  tft.setTextWrap(true);
  Serial.begin(9600);
  
}

void loop() {
    
  tryb = (analogRead(ModePin) * 5) / 1023;
  
  if(tryb <0.2){
    if(flag != 1){
      tft.fillScreen(ST77XX_BLACK);
      flag = 1;
    }
 
    drawtext("MIERNIK",35,sizeof("MIERNI"),3);
    drawtext("RLC",80,sizeof("RLC"),3); 
  }
  
  else if(tryb < 2){
    
    if(flag != 2){    
      tft.fillScreen(ST77XX_BLACK);
      flag = 2;  
    }
    
    drawtext("REZYSTANCJA",10,sizeof("REZYSTANCJA"),1);

    for(int j = 0; j < 3; j ++){
      digitalWrite(controlPin[j], muxChannel[0][j]);
    }
    
    ref = analogRead(A0);
    
    float Uwe = (ref* (5.0 / 1023.0))*((9990.0+10060.0)/10060.0);

    for(int i = 1; i<8; i++){
      
      for(int j = 0; j < 3; j ++){
        
        digitalWrite(controlPin[j], muxChannel[i][j]);
        
      }
      delay(50);
      rx[i-1] = analogRead(A1);
      
    }
    
    ind = 6;
    for(int i = 7; i >= 0; i--){
      if(rx[i]>512){
        ind=i;
      }
    }
    
    if(ind == 0){
      if(rx[0] < 900){
        ind = 1;
        }
    }
    
    else if(ind == 1){
      if(rx[1] < 800){
        ind = 2;
      }
    }
    
    else if(ind == 2){
      if(rx[2] < 650){
        ind = 3;
      }
    }

    val = ((Uwe*r7[ind])/(rx[ind]* (5.0 / 1023.0)))-r7[ind];

    if(val >2000000){  
      drawtext("  ----  ",50,sizeof("  ----  "),2);
      drawtext("  Ohm  ",95,sizeof("  Ohm  "),2);     
    }
    
    else if (val<1000){        
      dtostrf(val,6,1,result);
      drawtext(result,50,6,2);
      drawtext("  Ohm  ",95,sizeof("  Ohm  "),2); 
    }    
    
    else if (val<100000){      
      dtostrf(val,6,0,result);
      drawtext(result,50,6,2);
      drawtext("  Ohm  ",95,sizeof("  Ohm  "),2);
    }
    
    else if (val<1000000){
      val/=1000;  
      dtostrf(val,6,1,result);
      drawtext(result,50,6,2);
      drawtext("  KOhm  ",95,sizeof("  KOhm  "),2);
    }
    
    else{
      val/=1000000;  
      dtostrf(val,6,3,result);
      drawtext(result,50,6,2);
      drawtext("  MOhm  ",95,sizeof("  MOhm  "),2);
    }
    
  }
  
  else if(tryb < 3.8){
    
    if(flag != 3){
      tft.fillScreen(ST77XX_BLACK);
      flag = 3;
    }
    
    drawtext("POJEMNOSC",10,sizeof("POJEMNOSC"),1);

    // duże wartości
    pinMode(Cout, OUTPUT);    
    pinMode(Ccharge, OUTPUT);
    pinMode(Cdischarge, OUTPUT);
    pinMode(Cread, INPUT);

    digitalWrite(Cdischarge,LOW);
    digitalWrite(Cout,LOW);
    digitalWrite(Ccharge,LOW);

    while(analogRead(Cread)> 0);
    
    digitalWrite(Ccharge, HIGH);    
    
    startTime = micros();
    while(analogRead(Cread)< 648);
    
    elapsedTime= micros() - startTime;
    
    val = ((float)elapsedTime / CR);

    if(val<1){
      
      val *= 1000; //wartość w nF
      
      if(val <99){  //male wartości
        
        pinMode(Ccharge,INPUT);
        pinMode(Cdischarge,INPUT);
        pinMode(Cread, OUTPUT);
        delay(1);
        pinMode(Cout, INPUT_PULLUP);
        unsigned long u1 = micros();
        unsigned long t;
        int digVal;

        do{
          digVal = digitalRead(Cout);
          unsigned long u2 = micros();
          t = u2 > u1 ? u2 - u1 : u1 - u2;
        } 
      
        while ((digVal < 1) && (t < 400000L));

        pinMode(Cout, INPUT);
        val = analogRead(Cout);
        digitalWrite(Cread, HIGH);
        int dischargeTime = (int)(t / 1000L) * 5;
        delay(dischargeTime); 
        pinMode(Cout, OUTPUT);  
        digitalWrite(Cout, LOW);
        digitalWrite(Cread, LOW);
    
        val = (-(float)t / 34.8) / (log(1.0 - (float)val / (float)1023));
        
        if(val>=0.15){
          dtostrf(val,6,3,result);
          drawtext(result,50,6,2);
          drawtext("  nF  ",95,sizeof("  nF  "),2);
        }
        
        else{          
          drawtext("  ----  ",50,sizeof("  ----  "),2);
          drawtext("  F  ",95,sizeof("  F  "),2);
        }
        
      }//koniec małe watości
      
      else{
        dtostrf(val,6,2,result);
        drawtext(result,50,6,2);
        drawtext("  nF  ",95,sizeof("  nF  "),2);
      }
      
    }
    
    else if(val<10){
      dtostrf(val,6,2,result);
      drawtext(result,50,6,2);
      drawtext("  uF  ",95,sizeof("  uF  "),2);
    }
    
    else if(val<100){
      dtostrf(val,6,1,result);
      drawtext(result,50,6,2);
      drawtext("  uF  ",95,sizeof("  uF  "),2);
    }
    
    else{
      dtostrf(val,6,0,result);
      drawtext(result,50,6,2);
      drawtext("  uF  ",95,sizeof("  uF  "),2);
    }
    while (micros() % 1000 != 0);
  }
  
  else if(tryb < 5){
    
    if(flag != 4){
      tft.fillScreen(ST77XX_BLACK);
      flag = 4;
    }
    
    drawtext("INDUKCYJNOSC",10,sizeof("INDUKCYJNOSC"),1);

    digitalWrite(pulseO, HIGH);     //ładowanie induktora
    delay(5); //czas na naładowanie
    digitalWrite(pulseO, LOW);
    delayMicroseconds(100);
    puls = pulseIn(pulseI, HIGH, 5000); //pomiar
    
    if(puls > 0){
      
      freq = 1.E6/(2*puls);  //zamiana na częstotliwość
      val = 1/(4*c*freq*freq*PI*PI);
      val *= 1.0E6;
      
      if(val<1000){    
        dtostrf(val,6,2,result);
        drawtext(result,50,6,2);
        drawtext("  uH  ",95,sizeof("  uH  "),2);    
      }
      
      else{  
        dtostrf(val/1000,6,2,result);   
        drawtext(result,50,6,2);
        drawtext("  mH  ",95,sizeof("  mH  "),2); 
      }
      
    }
    
    else{
      drawtext("  ----  ",50,sizeof("  ----  "),2);
      drawtext("  H  ",95,sizeof("  H  "),2);
    }  
  }
}

void drawtext(char *text, uint16_t *y, uint16_t *Stxt, uint16_t *s) {

  uint16_t x = (128-((uint16_t)Stxt*5*(uint16_t)s+(uint16_t)Stxt*(uint16_t)s-1*(uint16_t)s))/2;
  tft.setCursor(x, y);
  tft.setTextSize(s);
  tft.print(text);
  tft.print("  ");
  
}
