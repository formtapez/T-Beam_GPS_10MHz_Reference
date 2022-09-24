#include <WiFi.h>
#include <Wire.h>
#include <axp20x.h>
#include <math.h>

#include "gps.h"
#include "gpsicon.h"
#include "Adafruit_GFX.h"
#include "Adafruit_SSD1306.h"

// T-Beam specific hardware
#define SELECT_BTN 38

// PPS
#define PPS_PIN 37
#define HF_PIN 0  // solder some wire to GPIO 0 as antenna

#define OLED_RESET 4 // not used
Adafruit_SSD1306 display(OLED_RESET);

// Powermanagement chip AXP192
AXP20X_Class axp;
#define AXP192_SLAVE_ADDRESS 0x34
const uint8_t axp_irq_pin = 35;

unsigned int blinkGps = 0;
boolean noFix = true;
boolean redraw = false;
boolean PowerOff = false;
axp_chgled_mode_t ledMode = AXP20X_LED_LOW_LEVEL;

uint8_t txBuffer[9];
uint16_t txBuffer2[5];
gps gps;

unsigned long lastMillis = 0;
unsigned long lastMillis2 = 0;

// For battery mesurement
float VBAT;  // battery voltage from ESP32 ADC read

unsigned clk_old = 0;
unsigned clk;
unsigned cnt = 0;
double pwm_freq;
double real_freq;
unsigned char secs = 0;
bool FirstFill = true;
double Freq_Array[60];
char status[100];

// triggered by PPS pulse from GPS
// sample cpu-cycles every 10 times, than average last 10 measurings (100 secs average total)
void pps()
{
  cnt++;
  if (cnt == 10)
  {
    clk = xthal_get_ccount(); // 240 mio. CPU-Cycles per sec

    if (gps.checkGpsFix() && clk_old > 0 && clk > clk_old)
    {
      unsigned diff = clk - clk_old;
      if (diff > (240000000.0 * cnt * 0.95) && diff < (240000000.0 * cnt * 1.05))
      {
        Freq_Array[secs++] = diff;

        double tmp = 0;
        for (unsigned char i=0; i<60; i++) tmp += Freq_Array[i];
        if (FirstFill) real_freq = tmp / (cnt * secs * 24.0);   // reduced averaging while ringbuffer not full
        else real_freq = tmp / (cnt * 10.0 * 24.0);             // full averaging with full ringbuffer

        if (secs >= 10)
        {
          secs = 0;
          FirstFill = false;
        }
      }
      char buffer[100];
      sprintf(buffer, "%.6lf // %.6lf // %.6lf MHz", real_freq / 1000000, real_freq * 43.0 / 1000000, real_freq * 128.0 / 1000000);
      Serial.println(buffer);

      if (FirstFill) sprintf(status, "Averaging [%i/10] ...", secs);
      else sprintf(status, "Averaging [10/10]");
    }
    clk_old = clk;
    cnt = 0;
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println(F("T-Beam 10 MHz source"));

  pinMode(PPS_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PPS_PIN), pps, RISING);

  pinMode(HF_PIN, OUTPUT);
  digitalWrite(HF_PIN, LOW); 

  // HF-PWM
  #define LEDC_CHANNEL 0
  #define LEDC_TIMER_BIT 1
  #define LEDC_BASE_FREQ 10000000
  ledcSetup(LEDC_CHANNEL, LEDC_BASE_FREQ, LEDC_TIMER_BIT);
  ledcAttachPin(HF_PIN, LEDC_CHANNEL);
  ledcWrite(LEDC_CHANNEL, LEDC_TIMER_BIT);
  pwm_freq = ledcReadFreq(LEDC_CHANNEL);

  Wire.begin(21, 22);
  if (!axp.begin(Wire, AXP192_SLAVE_ADDRESS)) Serial.println("AXP192 Begin PASS");
  else Serial.println("AXP192 Begin FAIL");

  // enable all irq channel
  axp.enableIRQ(0xFFFFFFFF, true);

  // attachInterrupt to gpio 35 -> Power-Off!
  pinMode(axp_irq_pin, INPUT_PULLUP);
  attachInterrupt(axp_irq_pin, [] {PowerOff = true;}, FALLING);
  axp.clearIRQ();

  // setup AXP192
  axp.setDCDC1Voltage(3300);              // OLED display at full 3.3v

  // activate ADC
  axp.adc1Enable(AXP202_BATT_VOL_ADC1, true);
  axp.adc1Enable(AXP202_BATT_CUR_ADC1, true);
  axp.adc1Enable(AXP202_VBUS_VOL_ADC1, true);
  axp.adc1Enable(AXP202_VBUS_CUR_ADC1, true);

  // activate power rails
  axp.setPowerOutPut(AXP192_LDO2, AXP202_ON);
  axp.setPowerOutPut(AXP192_LDO3, AXP202_ON);
  axp.setPowerOutPut(AXP192_DCDC2, AXP202_ON);
  axp.setPowerOutPut(AXP192_EXTEN, AXP202_ON);
  axp.setPowerOutPut(AXP192_DCDC1, AXP202_ON);
  

  pinMode(SELECT_BTN, INPUT);// UI Button
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C, 0, 21, 22, 800000);
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(1,0);
  display.print("T-Beam 10 MHz source");
  display.setTextSize(2);
  display.setCursor(0,32);
  display.print("\\/\\/\\/\\/\\/");
  display.setTextSize(1);
  display.setCursor(48,48);
  display.print("v1.0");
  display.drawLine(0, 10, display.width(), 10, WHITE);
  display.display();
  
  //Turn off WiFi and Bluetooth
  WiFi.mode(WIFI_OFF);
  btStop();
  gps.init();
  
  axp.setChgLEDMode(AXP20X_LED_OFF);
  display.clearDisplay();

  Serial.println(F("Waiting for GPS-Fix..."));

  lastMillis2 = -1000;
  delay(2000);

  sprintf(status, "Averaging [0/10] ...");

  // Enable passthrough to control GPS via serial (no exit possible)
//  gps.passthrough();
}

void loop()
{
  gps.encode();
  if (lastMillis + 1000 < millis())
  {
    lastMillis = millis();
    VBAT = axp.getBattVoltage()/1000;
    
    if (gps.checkGpsFix())
    { 
      noFix = false;

      gps.gdisplay(txBuffer2);
      float hdop = txBuffer2[4] / 10.0;
      display.clearDisplay();
      display.setTextColor(WHITE);
      display.setTextSize(1);
      display.setCursor(0,0);
      display.print("SAT: " + String(txBuffer2[0]));

      if (gps.tGps.time.isValid())
      {
        display.setCursor(48,0);
        if (gps.tGps.time.hour() < 10) display.print("0");
        display.print(gps.tGps.time.hour());
        display.print(":");
        if (gps.tGps.time.minute() < 10) display.print("0");
        display.print(gps.tGps.time.minute());
        display.print(":");
        if (gps.tGps.time.second() < 10) display.print("0");
        display.print(gps.tGps.time.second());
      }
      
      display.setCursor(104,0);
      display.print(VBAT,1);
      display.setCursor(122,0);
      // display charging-state
      if (axp.isChargeing()) display.print("V");
      else display.print("v");

      display.drawLine(0, 10, display.width(), 10, WHITE);
      
      // Freqs
      char buffer[100];
      display.setCursor(0,18);
      sprintf(buffer, "%.6lf MHz", real_freq / 1000000);
      display.print(buffer);

      display.setCursor(0,28);
      sprintf(buffer, "%.6lf MHz", real_freq * 43.0 / 1000000);
      display.print(buffer);

      display.setCursor(0,38);
      sprintf(buffer, "%.6lf MHz", real_freq * 128.0 / 1000000);
      display.print(buffer);

      display.drawLine(0, 52, display.width(), 52, WHITE);
      display.setCursor(0,56);
      display.print(status);
    }
    else
    {
      noFix = true;
      display.clearDisplay();
      display.setTextColor(WHITE);

      display.setTextSize(2);
      display.setCursor(0,16);
      display.print("No GPS");
      display.setTextSize(1);
      display.setCursor(104,0);
      display.print(VBAT,1);
      display.setCursor(122,0);
      // display charging-state
      if (axp.isChargeing())
      {
        display.print("V");
      } else {
        display.print("v");
      }
    }
    redraw = true;
  }
  if ((lastMillis2 + 500 < millis()) || redraw)
  {
    lastMillis2 = millis();
    if (noFix)
    {
      blinkGps = 1-blinkGps;
      if (blinkGps == 1)
      {
        display.fillRect((display.width()  - imageWidthGpsIcon ), (display.height() - imageHeightGpsIcon), imageWidthGpsIcon, imageHeightGpsIcon, 0);
        display.drawBitmap(
                            (display.width()  - imageWidthGpsIcon ),
                            (display.height() - imageHeightGpsIcon),
                            gpsIcon, imageWidthGpsIcon, imageHeightGpsIcon, 1);
      } else {
        display.fillRect((display.width()  - imageWidthGpsIcon ), (display.height() - imageHeightGpsIcon), imageWidthGpsIcon, imageHeightGpsIcon, 0);
        display.drawBitmap(
                            (display.width()  - imageWidthGpsIcon ),
                            (display.height() - imageHeightGpsIcon),
                            gpsIcon2, imageWidthGpsIcon, imageHeightGpsIcon, 1);
      }
    }
    redraw = true;
  }

  if (redraw)
  {
    redraw = false;
    display.display();
  }

  if (PowerOff)
  {
    PowerOff = false;
    Serial.println("Power-OFF!");
    display.clearDisplay();
    display.setTextColor(WHITE);
    display.setTextSize(2);
    display.setCursor(1,16);
    display.print("Power-OFF!");
    display.display();
    delay(1000);
    axp.setChgLEDMode(AXP20X_LED_OFF);
    axp.shutdown();
  }
}
