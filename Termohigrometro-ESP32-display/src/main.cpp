#include <Arduino.h>
#include <Wire.h>           // Para comunicação I2C com o SHT31
#include <Adafruit_SHT31.h> // Biblioteca do sensor SHT31
#include <TFT_eSPI.h>       // Biblioteca do display TFT
#include <esp_sleep.h>      // Para Deep Sleep no ESP32

#define SERIAL_ENABLED // Habilita a serial para depuração

// Pino ADC para leitura da tensão da bateria (geralmente GPIO34 no T-Display)
#define BATT_ADC_PIN 34
// Fator de correção do divisor de tensão para a bateria (CALIBRE ESTE VALOR!)
#define BATT_VOLTAGE_DIVIDER_FACTOR 2.0
// Tensão de referência interna do ADC (geralmente 3.3V para ESP32)
#define ADC_REF_VOLTAGE 3.3

// Pino do backlight do display (definido nos build_flags do platformio.ini como TFT_BL)
#define TFT_BACKLIGHT_PIN 4

/**
 * Função para verificar a tensão da bateria e entrar em modo de economia de energia
 * se a tensão estiver abaixo de um certo limite.
 * @param battery_voltage A tensão da bateria medida.
 */
void checkBatteryAndSleep(float battery_voltage);

// Instância do display TFT
TFT_eSPI tft = TFT_eSPI();

// Instância do sensor SHT31
Adafruit_SHT31 sht31 = Adafruit_SHT31(&Wire1); // Usando Wire1 para evitar conflito com o TFT

void setup()
{
  // Inicializa a comunicação Serial para depuração
#ifdef SERIAL_ENABLED
  Serial.begin(115200);
  Serial.println("\nIniciando Estacao ...");
#endif

  // Configura o pino do backlight como OUTPUT
  pinMode(TFT_BACKLIGHT_PIN, OUTPUT);
  digitalWrite(TFT_BACKLIGHT_PIN, HIGH); // Liga o backlight

  // Inicializa o display TFT
  tft.init();
  tft.setRotation(1);          // 0 = portrait, 1 = landscape
  tft.fillScreen(TFT_BLACK);   // Limpa a tela
  tft.setTextFont(2);          // Fonte padrão
  tft.setTextColor(TFT_WHITE); // Cor do texto

  tft.setCursor(10, 10);
  tft.println("Iniciando...");

  Wire.begin(21, 22); // Inicializa I2C nos pinos GPIO 21 (SDA) e 22 (SCL)

  // Inicializa o sensor SHT31
  Wire1.begin(15, 13); // Inicializa I2C nos pinos GPIO 15 (SDA) e 13 (SCL) -----CONFERIR!---------
  if (!sht31.begin(0x44))
  {
    tft.setCursor(10, 30);
    tft.setTextColor(TFT_RED);
    tft.println("Erro no SENSOR!");
    delay(2000); // Exibe o erro por um tempo
    // Não entramos em Deep Sleep imediatamente se o sensor falhar,
    // mas ainda assim podemos desligar o backlight e tentar novamente no próximo ciclo.
  }
  else
  {
    tft.setCursor(10, 30);
    tft.setTextColor(TFT_GREEN);
    tft.println("Sensor OK!");
  }

  // Configura a resolução do ADC para leitura da bateria
  analogReadResolution(12);       // 12 bits = 0-4095
  analogSetAttenuation(ADC_11db); // Atuação de 11dB para ler até 3.3V (ou 3.9V)

  delay(1000); // Pequena pausa para a mensagem de inicialização
}

void loop()
{
  static uint32_t startTime = millis(); // Marca o tempo de início do loop

  if (millis() - startTime > 1000)
  {
    startTime = millis(); // Reseta o tempo de início a cada 1s

    // --- Leitura do SHT31 ---
    float temperature = sht31.readTemperature();
    float humidity = sht31.readHumidity();

    // --- Leitura da Tensão da Bateria ---
    int raw_adc_value = analogRead(BATT_ADC_PIN);
    float adc_voltage = raw_adc_value * (ADC_REF_VOLTAGE / 4095.0);
    float battery_voltage = adc_voltage * BATT_VOLTAGE_DIVIDER_FACTOR;
    // calcula o percentual de bateria
    float battery_percentage = (battery_voltage - 3.0) / (4.2 - 3.0) * 100.0;
    if (battery_percentage < 0)
      battery_percentage = 0; // Garante que não fique negativo
    if (battery_percentage > 100)
      battery_percentage = 100; // Garante que não ultrapasse 100%

    // --- Exibição no Display ---
    tft.fillScreen(TFT_BLACK); // Limpa a tela para atualizar os valores

    // Umidade
    tft.setCursor(20, 5);
    tft.setTextSize(4);
    tft.setTextColor(TFT_GREEN);
    if (!isnan(humidity))
    {
      if (humidity < 10)
      {
        tft.print(" "); // Adiciona um espaço à esquerda se a umidade for menor que 10%
      }
      else if (humidity >= 100)
      {
        tft.print(""); // Não adiciona nada se a umidade for maior ou igual a 100%
      }
      // Exibe a umidade com uma casa decimal
      if (humidity < 100)
      {
        tft.print(humidity, 1);
      }
      else
      {
        tft.print(humidity, 0);
      }
      tft.println("%rh");
    }
    else
    {
      tft.setTextColor(TFT_RED);
      tft.print(" -----");
    }

    // Temperatura
    tft.setCursor(40, 55);
    tft.setTextSize(4);
    tft.setTextColor(TFT_GREEN);
    if (!isnan(temperature))
    {
      tft.print(temperature, 1);
      tft.println("C");
    }
    else
    {
      tft.setTextColor(TFT_RED);
      tft.print("-----");
    }

    // Tensão da Bateria
    // escohe a cor da fonte com base na porcentagem da bateria
    if (battery_percentage < 20)
    {
      tft.setTextColor(TFT_RED);
    }
    else if (battery_percentage < 50)
    {
      tft.setTextColor(TFT_ORANGE);
    }
    else
    {
      tft.setTextColor(TFT_GREEN);
    }

    tft.setTextSize(1);
    tft.setCursor(10, 120);

    tft.print("Bateria: ");
    tft.print(battery_percentage, 2);
    tft.println(" %");

// --- Saída no Monitor Serial (para depuração) ---
#ifdef SERIAL_ENABLED
    Serial.print("Temperatura: ");
    Serial.print(temperature, 1);
    Serial.print(" C | Umidade: ");
    Serial.print(humidity, 1);
    Serial.print(" % | Bateria: ");
    Serial.print(battery_voltage, 2);
    Serial.println(" V");
#endif
    checkBatteryAndSleep(battery_voltage);
  }
}

void checkBatteryAndSleep(float battery_voltage)
{
  // --- Verifica a Tensão da Bateria ---
  // Se a tensão da bateria for menor que 3.0V, entra em modo de economia de energia
  if (battery_voltage < 3.0)
  {
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_RED);
    tft.setCursor(10, 20);
    tft.setTextSize(1);

    tft.println("Bateria fraca!");
    tft.println("desligando...");
    delay(2000); // Exibe a mensagem por 2 segundos
    // --- Modo de Economia de Energia ---
    // Desliga o backlight da tela
    digitalWrite(TFT_BACKLIGHT_PIN, LOW);
    tft.fillScreen(TFT_BLACK); // Opcional: limpa a tela para garantir que nada seja mostrado

    // Como não há pino de controle de energia direto no SHT31 via software,
    // o desligamento do I2C do ESP32 pelo Deep Sleep já o desliga efetivamente.
    Wire.end(); // Desliga a interface I2C

    esp_deep_sleep_start(); // Entra em Deep Sleep
  }
}
