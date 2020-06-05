#include <LiquidCrystal.h>
/* Create object named lcd of the class LiquidCrystal */
// Creates an LCD object. Parameters: (rs, enable, d0,d1,d2,d3,d4, d5, d6, d7)
LiquidCrystal lcd(13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3);  /* For 8-bit mode */
//LiquidCrystal lcd(13, 12, 11, 6, 5, 4, 3);  /* For 4-bit mode */

// make some custom characters:
byte Heart[8] = {
0b00000,
0b01010,
0b11111,
0b11111,
0b01110,
0b00100,
0b00000,
0b00000
};

byte Bell[8] = {
0b00100,
0b01110,
0b01110,
0b01110,
0b11111,
0b00000,
0b00100,
0b00000
};


byte Alien[8] = {
0b11111,
0b10101,
0b11111,
0b11111,
0b01110,
0b01010,
0b11011,
0b00000
};

byte Check[8] = {
0b00000,
0b00001,
0b00011,
0b10110,
0b11100,
0b01000,
0b00000,
0b00000
};

byte Speaker[8] = {
0b00001,
0b00011,
0b01111,
0b01111,
0b01111,
0b00011,
0b00001,
0b00000
};


byte Sound[8] = {
0b00001,
0b00011,
0b00101,
0b01001,
0b01001,
0b01011,
0b11011,
0b11000
};


byte Skull[8] = {
0b00000,
0b01110,
0b10101,
0b11011,
0b01110,
0b01110,
0b00000,
0b00000
};

byte Lock[8] = {
0b01110,
0b10001,
0b10001,
0b11111,
0b11011,
0b11011,
0b11111,
0b00000
};
byte Plus[8] =
{
0b00000,
0b00000,
0b00100,
0b00100,
0b11111,
0b00100,
0b00100,
0b00000
};
byte Symbol[9] =
{
0b00100,
0b01010,
0b10101,
0b00100,
0b01010,
0b00100,
0b00100,
0b00100
};



void setup() {
  lcd.begin(16,2);  /* Initialize 16x2 LCD */
  lcd.clear();  /* Clear the LCD */
  // create a new character
  lcd.createChar(0, Plus);
  // create a new character
  lcd.createChar(1, Symbol);
  // create a new character
  lcd.createChar(2, Alien);
  // create a new character
  lcd.createChar(3, Check);
  // create a new character
  lcd.createChar(4, Speaker);
  // create a new character
  lcd.createChar(5, Sound);
  // create a new character
  lcd.createChar(6, Skull);
  // create a new character
  lcd.createChar(7, Lock);

  // Clears the LCD screen
  lcd.clear();

  // Print a message to the lcd.
  lcd.print("Custom Character");
//  lcd.createChar(0, Character1);  /* Generate custom character */
//  lcd.createChar(1, Character2);
}

void loop() {
//  lcd.setCursor(0,0); /* Set cursor to column 0 row 0 */
//  lcd.print("Welcome ARDUINO!!!!"); /* Print data on display */
//  lcd.setCursor(0,1); 
//  
//  lcd.write(byte(4)); /* Write a character to display */
 
  
  lcd.setCursor(0, 1);
  lcd.write(byte(8));

  lcd.setCursor(2, 1);
  lcd.write(byte(9));

  lcd.setCursor(4, 1);
  lcd.write(byte(2));

  lcd.setCursor(6, 1);
  lcd.write(byte(3));

  lcd.setCursor(8, 1);
  lcd.write(byte(4));

  lcd.setCursor(10, 1);
  lcd.write(byte(5));

  lcd.setCursor(12, 1);
  lcd.write(byte(6));

  lcd.setCursor(14, 1);
  lcd.write(byte(7));
}
