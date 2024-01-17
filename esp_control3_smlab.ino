#include <WiFi.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

//----------Pin Definitions----------//
#define j0_x 36
#define j0_y 39
#define p0 34

#define j1_x 35
#define j1_y 32
#define p1 33

#define s0 25
#define s1 26
#define s2 27
#define s3 14

#define lcd_heading(heading) lcd.setCursor(3,0);lcd.print(heading)
#define lcd_line1(line1) lcd.setCursor(3,1);lcd.print(line1)
#define lcd_line2(line2) lcd.setCursor(3,2);lcd.print(line2)
#define lcd_line3(line3) lcd.setCursor(9,3);lcd.print(line3)

LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x27 for a 16 chars and 2 line display

int col = 20;
int row = 4;
const char* ssid = "smlab";
const char* password =  "igcsm123";
 
const uint16_t port = 8090;
const char * host = "192.168.1.236";

int buttonPushCounter = 0;   // counter for the number of button presses
int buttonState = 0;         // current state of the button
int lastButtonState = 0;     // previous state of the button

int secButtonPushCounter = 0;   // counter for the number of button presses
int secButtonState = 0;         // current state of the button
int secLastButtonState = 0;     // previous state of the button
//
//int 3ButtonPushCounter = 0;   // counter for the number of button presses
int ButtonState3 = 0;         // current state of the button
int LastButtonState3 = 0; 
//
//int 4ButtonPushCounter = 0;   // counter for the number of button presses
int buttonState4 = 0;         // current state of the button
int LastButtonState4 = 0; 
char message = 'K';

int menuOption=0;
 
void setup()
{
  lcd.init();                      // initialize the lcd 
  lcd.backlight();
  Serial.begin(115200);

  pinMode(s0,INPUT);
  pinMode(s1,INPUT);

  pinMode(s2,INPUT);
  pinMode(s3,INPUT);

  lcd.clear();
  
 
  
 
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("...");
    lcd_line1("Connecting..");
  }
  lcd.clear();
  Serial.print("WiFi connected with IP: ");
  Serial.println(WiFi.localIP());
  lcd_line1("Wifi");
 
}

void loop()
{
    WiFiClient client;
 
    if (!client.connect(host, port)) {
 
        Serial.println("Connection to host failed");
        lcd_line2("Server_error");
        
        delay(500);
        return;
    }
    lcd.clear();
    message='K';
    buttonState = digitalRead(s0);
    secButtonState = digitalRead(s3);
    ButtonState3=digitalRead(s2);
    buttonState4=digitalRead(s1);
    lcd_line1("Wifi");
    lcd_line2("Server");

    if (buttonState != lastButtonState) {
    
      message='W';
      lcd_line3("W");
      }// Delay a little bit to avoid bouncing
    delay(50);
    lastButtonState = buttonState;

    if (secButtonState != secLastButtonState) {
    
    message='S';
    lcd_line3("S");
    }// Delay a little bit to avoid bouncing
    delay(50);
    
    secButtonState=secLastButtonState;

    if(ButtonState3!=LastButtonState3){
    
    message='A';
    lcd_line3("A");
    }// Delay a little bit to avoid bouncing
    delay(50);
    
    ButtonState3=LastButtonState3;

    if (buttonState4!=LastButtonState4){
    
    message='D';
    lcd_line3("D");
    }// Delay a little bit to avoid bouncing
    delay(50);
    buttonState4=LastButtonState4;

    
    
  
    Serial.println("Connected to server successful!");
 
    client.print(message);
 
    //Serial.println("Disconnecting...");
    client.stop();
 
    delay(500);
}
