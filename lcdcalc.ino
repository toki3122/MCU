#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27,16,2);
void setup() {
  lcd.init();
  lcd.backlight();
  Serial.begin(9600);
  int a=0, sum;
  Serial.print("enter number: ");
  while(!Serial.available()){}
  delay(1000);
  a=Serial.parseInt();
  Serial.println(a);
  if(a%10000!=0){
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("invalid input");
  lcd.setCursor(0,1);
  lcd.print("enter again");
  }
  int b=a;
  int digit[4];
  for(int i=3;i>=0;i--){
    digit[i]=a%10;
    a/=10;
  }
  if(digit[1]>digit[2]){
  sum=digit[3]+digit[0];
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("input:");
  lcd.setCursor(6,0);
  lcd.print(b);
  lcd.setCursor(0,1);
  lcd.print("addition:");
  lcd.setCursor(9,1);
  lcd.print(sum);
  }
  else if(digit[1]<digit[2]){
  sum=digit[3]-digit[0];
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("input:");
  lcd.setCursor(6,0);
  lcd.print(b);
  lcd.setCursor(0,1);
  lcd.print("subtraction:");
  lcd.setCursor(11,1);
  lcd.print(sum);
  }
  else if(digit[1]=digit[2]){
  sum=digit[1]+10*digit[0];
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("input:");
  lcd.setCursor(6,0);
  lcd.print(b);
  lcd.setCursor(0,1);
  lcd.print("concatenation:");
  lcd.setCursor(14,1);
  lcd.print(sum);
  }
}
void loop() {}
