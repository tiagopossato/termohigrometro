#include <U8glib.h>  // U8glib library
#include <OneWire.h>
#include <DallasTemperature.h>

#define AMOSTRAS 20
U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_NONE | U8G_I2C_OPT_DEV_0); // I2C / TWI

// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS 9

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

float temperatura;
float bateria;

void draw(void)
{
  u8g.setFont(u8g_font_fub25r);   // select font
  u8g.setPrintPos(0, 36);        // set position
  u8g.print(temperatura, 2);  // display temperature from DHT11 in Celsius
  u8g.println(" C");
  u8g.setFont(u8g_font_fub11r);   // select font
  u8g.setPrintPos(0, 62);        // set position
  u8g.print(bateria, 1);  // display temperature from DHT11 in Celsius
  u8g.println(" V");
}

void setup(void)
{
  // Start up the library
  sensors.begin();
  Serial.begin(9600);
  analogReference(INTERNAL);
  DeviceAddress deviceAddress; // We'll use this variable to store a found device address
  sensors.getAddress(deviceAddress, 0);

  char id[4][6];
  sprintf(id[0], "");
  sprintf(id[1], "");
  sprintf(id[2], "");
  sprintf(id[3], "");

  uint8_t i = 0;
  for (; i < 2; i++)
  {
    sprintf(id[0], "%s%02X", id[0], deviceAddress[i]);
  }
  for (; i < 4; i++)
  {
    sprintf(id[1], "%s%02X", id[1], deviceAddress[i]);
  }
  for (; i < 6; i++)
  {
    sprintf(id[2], "%s%02X", id[2], deviceAddress[i]);
  }

  for (; i < 8; i++)
  {
    sprintf(id[3], "%s%02X", id[3], deviceAddress[i]);
  }

  sprintf(id[0], "%s-", id[0]);
  sprintf(id[1], "%s-", id[1]);
  sprintf(id[2], "%s-", id[2]);

  u8g.firstPage();
  do
  {
    u8g.setFont(u8g_font_fub14r);   // select font
    u8g.setPrintPos(0, 25);        // set position
    u8g.print("SENSOR ID:"); 
     u8g.setFont(u8g_font_fub11r);   // select font
    u8g.setPrintPos(0, 45);        // set position
    u8g.print(id[0]);
    //u8g.setPrintPos(0, 38);        // set position
    u8g.print(id[1]);
    u8g.setPrintPos(0, 62);        // set position
    u8g.print(id[2]);
    //u8g.setPrintPos(0, 64);        // set position
    u8g.print(id[3]);
  }  while ( u8g.nextPage() );
  delay(2000);
}

void loop(void)
{
  //http://www.arduinoecia.com.br/p/calculador-divisor-de-tensao-function.html
  //Usando divisor resistivo com 820k / 220k
  uint32_t tmp = 0;
  for (char i = 0; i < AMOSTRAS; i++) {
    tmp += analogRead(A3);
    delay(2);
  }

  bateria = converte(tmp, 0, 1023 * AMOSTRAS, 0, 1.1 * 4.674545455);

  sensors.requestTemperatures();
  temperatura = sensors.getTempCByIndex(0);
  Serial.println(temperatura);
  u8g.firstPage();
  do
  {
    draw();
  }  while ( u8g.nextPage() );
  delay(200);  // Delay of 2 sec before accessing DHT11 (min - 2sec)
}

/*
   Funcao de regressao linear com com valores em ponto flutuante
*/
float converte(float x, float in_min, float in_max, float out_min,
               float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
