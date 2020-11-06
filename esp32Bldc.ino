#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_attr.h"
#include "soc/rtc.h"
#include "driver/mcpwm.h"
//#include "soc/mcpwm_periph.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"
#include <SPI.h>
#include "driver/i2s.h"
#include "driver/adc.h"
#include <soc/syscon_reg.h>
#include "DHTesp.h"
#include "esp_adc_cal.h"
#include "esp_err.h"

// Prototypes
static void IRAM_ATTR callback(void *arg);
static const void Sampling(void);

int dhtPin = 32;    // Digital pin connected to the DHT sensor
ComfortState cf;
DHTesp dht;


#define CAP_SIG_NUM 3        //Three capture signals from HALL-A, HALL-B, HALL-C
#define CAP0_INT_EN BIT(27)  //Capture 0 interrupt bit
#define CAP1_INT_EN BIT(28)  //Capture 1 interrupt bit
#define CAP2_INT_EN BIT(29)  //Capture 2 interrupt bit

typedef struct {
  uint32_t capture_signal;
  mcpwm_capture_signal_t sel_cap_signal;
} capture;

static mcpwm_dev_t *MCPWM[2] = {&MCPWM0, &MCPWM1};
xQueueHandle cap_queue;

/*
#define GPIO_PWM0A_OUT 15   //Set GPIO 15 as PWM0A (U HS)
#define GPIO_PWM0B_OUT 02   //Set GPIO 02 as PWM0B (U LS)
#define GPIO_PWM1A_OUT 00   //Set GPIO 00 as PWM1A (V HS)
#define GPIO_PWM1B_OUT 04   //Set GPIO 04 as PWM1B (V LS)
#define GPIO_PWM2A_OUT 16   //Set GPIO 16 as PWM2A (W HS)
#define GPIO_PWM2B_OUT 17   //Set GPIO 17 as PWM2B (W LS)
*/

#define GPIO_PWM0A_OUT 16   //Set GPIO 15 as PWM0A (U HS)
#define GPIO_PWM0B_OUT 17   //Set GPIO 02 as PWM0B (U LS)
#define GPIO_PWM1A_OUT 00   //Set GPIO 00 as PWM1A (V HS)
#define GPIO_PWM1B_OUT 04   //Set GPIO 04 as PWM1B (V LS)
#define GPIO_PWM2A_OUT 15   //Set GPIO 16 as PWM2A (W HS)
#define GPIO_PWM2B_OUT 02   //Set GPIO 17 as PWM2B (W LS)

#define GPIO_CAP0_IN   25   //Set GPIO 25 as CAP0
#define GPIO_CAP1_IN   26   //Set GPIO 26 as CAP1
#define GPIO_CAP2_IN   27   //Set GPIO 27 as CAP2

#define GPIO_XOR_IN    13   // XOR

#define GPIO_FAULT   22     //Set GPIO 22 as  FAULT
#define GPIO_PWRGD   21     //Set GPIO 21 as  PWRGD
#define GPIO_WAKE    33     //Set GPIO 13 as  WAKE
#define GPIO_ENGATE  12     //Set GPIO 12 as  ENGATE

// SPI
#define VSPI_MISO   MISO
#define VSPI_MOSI   MOSI
#define VSPI_SCLK   SCK
#define VSPI_SS     SS
static const int spiClk = 1000000; // 1 MHz
SPIClass * vspi = NULL; // Uninitalised pointers to SPI objects



// Variabls for blinking an LED with Millis
const int led = 2; // ESP32 Pin to which onboard LED is connected
unsigned long previousMillis = 0;  // will store last time LED was updated
const long interval = 3000;  // interval at which to blink (milliseconds)
int ledState = LOW;  // ledState used to set the LED


bool FLAG_ALLOW_MOTORSTART = false;
static uint32_t n_ActualRpm = 0;
static uint32_t hall_sensor_value = 0;
static uint32_t hall_sensor_previous = 0;
float dcBusVoltage = 0;
int adcVal_dcBusVoltage = 0;
volatile uint64_t  time1 = 0;
volatile uint64_t  time2 = 0;
volatile uint64_t  timediff = 0;

uint32_t adcVal_sin = 0;
float sin_voltage = 0;
uint32_t adcVal_cos = 0;
float cos_voltage = 0;



TaskHandle_t Task_Print2Console;
TaskHandle_t Task_SerialReceiveData;
TaskHandle_t Task_Commutation;
TaskHandle_t Task_SPI;

hw_timer_t * timer = NULL;

void setup() {

  configure_i2s();
  
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("begin setup");
  
  dht.setup(dhtPin, DHTesp::DHT22);
  vspi = new SPIClass(VSPI);
  //initialise vspi with default pins
  //SCLK = 18, MISO = 19, MOSI = 23, SS = 5
  
  vspi->begin();

  /*
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 200, true); // value in µs (1000000µs = 1s)
  timerAlarmEnable(timer);
  */
  
  pinMode(led, OUTPUT);
  pinMode(GPIO_WAKE, OUTPUT);
  pinMode(GPIO_ENGATE, OUTPUT);
  pinMode(VSPI_SS, OUTPUT);

  pinMode(GPIO_CAP0_IN, INPUT_PULLUP);
  pinMode(GPIO_CAP1_IN, INPUT_PULLUP);
  pinMode(GPIO_CAP2_IN, INPUT_PULLUP);
  pinMode(GPIO_XOR_IN, INPUT_PULLUP);
  pinMode(GPIO_FAULT, INPUT_PULLUP);

  analogSetAttenuation(ADC_11db);
  
  digitalWrite(GPIO_WAKE, HIGH);
  delay(5);
  digitalWrite(GPIO_ENGATE, HIGH);
  delay(5);
  digitalWrite(GPIO_ENGATE, LOW);
  delay(5);
  digitalWrite(GPIO_ENGATE, HIGH);

  static mcpwm_dev_t *MCPWM[2] = {&MCPWM0, &MCPWM1};

  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_PWM0A_OUT);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, GPIO_PWM0B_OUT);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, GPIO_PWM1A_OUT);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1B, GPIO_PWM1B_OUT);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM2A, GPIO_PWM2A_OUT);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM2B, GPIO_PWM2B_OUT);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_CAP_0, GPIO_CAP0_IN);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_CAP_1, GPIO_CAP1_IN);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_CAP_2, GPIO_CAP2_IN);

  mcpwm_config_t pwm_config;
  pwm_config.frequency = 16000; // f  = 16khz
  pwm_config.cmpr_a = 50.0;     // duty cycle of PWMxA = 50.0%
  pwm_config.cmpr_b = 50.0;     // duty cycle of PWMxb = 50.0%
  pwm_config.counter_mode = MCPWM_UP_COUNTER;
  pwm_config.duty_mode = MCPWM_DUTY_MODE_1;
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);    //Configure PWM0A & PWM0B with above settings
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config);    //Configure PWM1A & PWM1B with above settings
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_2, &pwm_config);    //Configure PWM2A & PWM2B with above settings

  //configure CAP0, CAP1 and CAP2 signal to start capture counter on rising edge
  //we generate a gpio_test_signal of 20ms on GPIO 12 and connect it to one of the capture signal, the disp_captured_function displays the time between rising edge
  //In general practice you can connect Capture  to external signal, measure time between rising edge or falling edge and take action accordingly
  mcpwm_capture_enable(MCPWM_UNIT_0, MCPWM_SELECT_CAP0, MCPWM_POS_EDGE, 0);  //capture signal on rising edge, pulse num = 0 i.e. 800,000,000 counts is equal to one second
  mcpwm_capture_enable(MCPWM_UNIT_0, MCPWM_SELECT_CAP1, MCPWM_POS_EDGE, 0);  //capture signal on rising edge, pulse num = 0 i.e. 800,000,000 counts is equal to one second
  mcpwm_capture_enable(MCPWM_UNIT_0, MCPWM_SELECT_CAP2, MCPWM_POS_EDGE, 0);  //capture signal on rising edge, pulse num = 0 i.e. 800,000,000 counts is equal to one second
  //enable interrupt, so each this a rising edge occurs interrupt is triggered
  MCPWM[MCPWM_UNIT_0]->int_ena.val = (CAP0_INT_EN | CAP1_INT_EN | CAP2_INT_EN);  //Enable interrupt on  CAP0, CAP1 and CAP2 signal
  mcpwm_isr_register(MCPWM_UNIT_0, callback, NULL, ESP_INTR_FLAG_IRAM, NULL);  //Set ISR Handler

/*
  attachInterrupt(GPIO_XOR_IN, callback, CHANGE);
  attachInterrupt(GPIO_CAP0_IN, callback, CHANGE);
  attachInterrupt(GPIO_CAP1_IN, callback, CHANGE);
  attachInterrupt(GPIO_CAP2_IN, callback, CHANGE);
*/
  
  xTaskCreatePinnedToCore(Print2Console, "Print2Console", 4096, NULL, 5, &Task_Print2Console, 0);               // Core 0
  xTaskCreatePinnedToCore(SerialReceiveData, "SerialReceiveData", 4096, NULL, 6, &Task_SerialReceiveData, 0);   // Core 0
  cap_queue = xQueueCreate(1, sizeof(capture));
  xTaskCreate(disp_captured_signal, "mcpwm_config", 4096, NULL, 2, NULL);
  //xTaskCreatePinnedToCore(disp_captured_signal, "mcpwm_config", 4096, NULL, 2, NULL, 0);                        // Core 0                    
                      
//  xTaskCreatePinnedToCore(
//                      Commutation,   /* Task function. */
//                      "Commutation",     /* name of task. */
//                      4096,       /* Stack size of task */
//                      NULL,        /* parameter of the task */
//                      1,           /* priority of the task */
//                      &Task_Commutation,      /* Task handle to keep track of created task */
//                      1); // Core 1


  Serial.println("setup finished");
}

/*
void IRAM_ATTR onTimer() {

  //adcVal_sin = analogRead(GPIO_NUM_32);
  //sin_voltage = ((3.6/4096)*adcVal_sin) - 2.5;
  //adcVal_cos = analogRead(GPIO_NUM_35);
  //cos_voltage = ((3.6/4096)*adcVal_cos) - 2.5;
}
*/

static void IRAM_ATTR callback(void *arg) {
  
  uint32_t mcpwm_intr_status;
  capture evt;

  
  mcpwm_intr_status = MCPWM[MCPWM_UNIT_0]->int_st.val; //Read interrupt status
  if (mcpwm_intr_status & CAP0_INT_EN) { //Check for interrupt on rising edge on CAP0 signal
      evt.capture_signal = mcpwm_capture_signal_get_value(MCPWM_UNIT_0, MCPWM_SELECT_CAP0); //get capture signal counter value
      evt.sel_cap_signal = MCPWM_SELECT_CAP0;
      xQueueSendFromISR(cap_queue, &evt, NULL);
      //Serial.println("CAP0");
  }
  if (mcpwm_intr_status & CAP1_INT_EN) { //Check for interrupt on rising edge on CAP1 signal
      evt.capture_signal = mcpwm_capture_signal_get_value(MCPWM_UNIT_0, MCPWM_SELECT_CAP1); //get capture signal counter value
      evt.sel_cap_signal = MCPWM_SELECT_CAP1;
      xQueueSendFromISR(cap_queue, &evt, NULL);
      //Serial.println("CAP1");
  }
  if (mcpwm_intr_status & CAP2_INT_EN) { //Check for interrupt on rising edge on CAP2 signal
      evt.capture_signal = mcpwm_capture_signal_get_value(MCPWM_UNIT_0, MCPWM_SELECT_CAP2); //get capture signal counter value
      evt.sel_cap_signal = MCPWM_SELECT_CAP2;
      xQueueSendFromISR(cap_queue, &evt, NULL);
      //Serial.println("CAP2");
  }
  MCPWM[MCPWM_UNIT_0]->int_clr.val = mcpwm_intr_status;


  /*
  time1 = esp_timer_get_time();
  timediff = time1 - time2;
  if (timediff< 300) { // 300µs
  // do nothing because it was a glitch!
  }
  else {
    n_ActualRpm = (((1000000/(timediff * 6 * 8))*60)); // 1000000 µs = 1s; DurationTimeOneSector[µs]; 6 = Number of Sectors; 4 = Nbr of Pole Pairs (China Motor) and 8 for Nanotec LF45024048 and 2 = Nbr of Pole Pairs Linix Motor
  }
  time2 = time1;
  Serial.println(n_ActualRpm);
  */
}

static void disp_captured_signal(void *arg) {
    uint32_t *current_cap_value = (uint32_t *)malloc(sizeof(CAP_SIG_NUM));
    uint32_t *previous_cap_value = (uint32_t *)malloc(sizeof(CAP_SIG_NUM));
    capture evt;
    while (1) {
        xQueueReceive(cap_queue, &evt, portMAX_DELAY);
        if (evt.sel_cap_signal == MCPWM_SELECT_CAP0) {
            current_cap_value[0] = evt.capture_signal - previous_cap_value[0];
            previous_cap_value[0] = evt.capture_signal;
            current_cap_value[0] = (current_cap_value[0] / 10000) * (10000000000 / rtc_clk_apb_freq_get());
            //printf("CAP0 : %d us\n", current_cap_value[0]);
            //Serial.println(current_cap_value[0]);
            //Serial.println(mcpwm_capture_signal_get_edge(MCPWM_UNIT_0, MCPWM_SELECT_CAP0));
        }
        if (evt.sel_cap_signal == MCPWM_SELECT_CAP1) {
            current_cap_value[1] = evt.capture_signal - previous_cap_value[1];
            previous_cap_value[1] = evt.capture_signal;
            current_cap_value[1] = (current_cap_value[1] / 10000) * (10000000000 / rtc_clk_apb_freq_get());
            //printf("CAP1 : %d us\n", current_cap_value[1]);
            //Serial.println(current_cap_value[1]);
        }
        if (evt.sel_cap_signal == MCPWM_SELECT_CAP2) {
            current_cap_value[2] = evt.capture_signal -  previous_cap_value[2];
            previous_cap_value[2] = evt.capture_signal;
            current_cap_value[2] = (current_cap_value[2] / 10000) * (10000000000 / rtc_clk_apb_freq_get());
            //printf("CAP2 : %d us\n", current_cap_value[2]);
            //Serial.println(current_cap_value[2]);
        }

        n_ActualRpm = (((1000000/(((current_cap_value[0]+current_cap_value[1]+current_cap_value[2])/3) * 2 * 2))*60));
        Serial.println(n_ActualRpm);
    }
}




/*
//Task_Commutation
void Commutation(void * pvParameters ){
  
  for(;;){

    hall_sensor_value = (gpio_get_level(GPIO_NUM_27) * 1) + (gpio_get_level(GPIO_NUM_26) * 2) + (gpio_get_level(GPIO_NUM_25) * 4);
    //hall_sensor_value = (gpio_get_level(GPIO_NUM_25) * 4) + (gpio_get_level(GPIO_NUM_26) * 2) + (gpio_get_level(GPIO_NUM_27) * 1);
    
  
    if (FLAG_ALLOW_MOTORSTART == true) {
      if (hall_sensor_value != hall_sensor_previous) {
        //printf("hall_sen val: %d\n", hall_sensor_value);       
      if (hall_sensor_value == 2) {
          mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A);
          mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_B);
          mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A);
          mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B);
          //MCPWMXA to duty mode 1 and MCPWMXB to duty mode 0 or vice versa will generate MCPWM compliment signal of each other, there are also other ways to do it
          mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_1); //Set PWM0A to duty mode one
          mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, MCPWM_DUTY_MODE_0); //Set PWM0B back to duty mode zero
          mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_BYPASS_FED, 20, 20);   //Deadtime of 2us
          //mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_BYPASS_FED, 100, 100);   //Deadtime of 10us
      }
      if (hall_sensor_value == 6) {
          mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);
          mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B);
          mcpwm_deadtime_disable(MCPWM_UNIT_0, MCPWM_TIMER_0);
          mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A);
          mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B);
          //MCPWMXA to duty mode 1 and MCPWMXB to duty mode 0 or vice versa will generate MCPWM compliment signal of each other, there are also other ways to do it
          mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, MCPWM_DUTY_MODE_1); //Set PWM2A to duty mode one
          mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_B, MCPWM_DUTY_MODE_0); //Set PWM2B back to duty mode zero
          mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_BYPASS_FED, 20, 20);   //Deadtime of 2us
      }
      if (hall_sensor_value == 4) {
          mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A);
          mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B);
          mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);
          mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B);
          //MCPWMXA to duty mode 1 and MCPWMXB to duty mode 0 or vice versa will generate MCPWM compliment signal of each other, there are also other ways to do it
          mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, MCPWM_DUTY_MODE_1); //Set PWM2A to duty mode one
          mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_B, MCPWM_DUTY_MODE_0); //Set PWM2B back to duty mode zero
          mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_BYPASS_FED, 20, 20);   //Deadtime of 2us
      }
      if (hall_sensor_value == 5) {
          mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A);
          mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_B);
          mcpwm_deadtime_disable(MCPWM_UNIT_0, MCPWM_TIMER_2);
          mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);
          mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B);
          //MCPWMXA to duty mode 1 and MCPWMXB to duty mode 0 or vice versa will generate MCPWM compliment signal of each other, there are also other ways to do it
          mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, MCPWM_DUTY_MODE_1); //Set PWM1A to duty mode one
          mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, MCPWM_DUTY_MODE_0); //Set PWM1B back to duty mode zero
          mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_BYPASS_FED, 20, 20);   //Deadtime of 2us
      }
      if (hall_sensor_value == 1) {
          mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);
          mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B);
          mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A);
          mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_B);
          //MCPWMXA to duty mode 1 and MCPWMXB to duty mode 0 or vice versa will generate MCPWM compliment signal of each other, there are also other ways to do it
          mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, MCPWM_DUTY_MODE_1); //Set PWM1A to duty mode one
          mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, MCPWM_DUTY_MODE_0); //Set PWM1B back to duty mode zero
          mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_BYPASS_FED, 20, 20);   //Deadtime of 2us
      }
      if (hall_sensor_value == 3) {
          mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A);
          mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B);
          mcpwm_deadtime_disable(MCPWM_UNIT_0, MCPWM_TIMER_1);
          mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A);
          mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_B);
          //MCPWMXA to duty mode 1 and MCPWMXB to duty mode 0 or vice versa will generate MCPWM compliment signal of each other, there are also other ways to do it
          mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_1); //Set PWM0A to duty mode one
          mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, MCPWM_DUTY_MODE_0); //Set PWM0B back to duty mode zero
          mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_BYPASS_FED, 20, 20);   //Deadtime of 2us
      }
        
        hall_sensor_previous = hall_sensor_value;    
      }
    }
  
    else {
      mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A);
      mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_B);
      mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A);
      mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B);
      mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);
      mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B);
    }
  }
}

*/

//Task_Print2Console
void Print2Console(void * pvParameters ){
  
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 1000; // 1000ms

  //Initialise the xLastWakeTime variable with the current time.
  xLastWakeTime = xTaskGetTickCount();

  //Serial.print("Task_Print2Console running on core ");
  //Serial.println(xPortGetCoreID());
  
  for(;;){
    // Wait for the next cycle.
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(xFrequency));

    // Perfrom action here

    Sampling();
    //Serial.write(3000);
    vspiCommand();

/*
    while (Serial.available() > 0){
      Serial.print(char(Serial2.read()));
    }
*/


    //Serial.print("DC BUS VOLTAGE [V]: " );
    //Serial.println(dcBusVoltage);
    
    //Serial.print("SIN/COS VOLTAGE [V]: ");
    //Serial.print(sin_voltage);
    //Serial.print(" / ");
    //Serial.println(cos_voltage);

    //Serial.println((asin(sin_voltage*2))*180/3.14);
    
    //Serial.print("RPM: ");
    //Serial.println(n_ActualRpm);
  }
}

//Task_SerialReceiveData
void SerialReceiveData(void * pvParameters ) {
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 100; // 100ms
  // Initialise the xLastWakeTime variable with the current time.
  xLastWakeTime = xTaskGetTickCount();
  for(;;){
    // Wait for the next cycle.
    vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS(xFrequency) );
    
    bool stringComplete = false;
    String IncomingString = "";
    IncomingString.reserve(10);
    
    while (Serial.available() > 0) {
      // get the new byte:
      char inChar = (char)Serial.read();
      // add it to the inputString:
      IncomingString += inChar;
      // if the incoming character is a newline, set a flag so the main loop can
      // do something about it:
      //Serial.println("Es kommt rein: ");
      //Serial.println(IncomingString);
      if (inChar == '\n') {
        stringComplete = true;
      }
    }
  
    if (stringComplete == true) {     
      if (IncomingString[0] == '1') {
        digitalWrite(GPIO_WAKE, HIGH);
        delay(5);
        digitalWrite(GPIO_ENGATE, HIGH);
        delay(5);
        digitalWrite(GPIO_ENGATE, LOW);
        delay(5);
        digitalWrite(GPIO_ENGATE, HIGH);
        FLAG_ALLOW_MOTORSTART = true;
      }
      if ((IncomingString[0] == 'd') and (IncomingString[1] == 'c')) {
        static int dc = 0;
        static char tempStr[3];
        tempStr[0] = IncomingString[3];
        tempStr[1] = IncomingString[4];
        tempStr[2] = '\0';
        dc = atoi(tempStr);
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, dc);
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, dc);
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, dc);
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, dc);
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, dc);
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_B, dc);
      }
      if (IncomingString[0] == '4') {
        FLAG_ALLOW_MOTORSTART = false;
      }

      IncomingString = "";
      stringComplete = false;
    }
  }
}



void loop() {

/*
  unsigned long currentMillis = millis();
  
  if (currentMillis - previousMillis >= interval) {
    adcVal_dcBusVoltage = analogRead(36);
    dcBusVoltage = ( (3.6/4096)*adcVal_dcBusVoltage ) / ( 4.99/(62+4.99) );

    // save the last time you blinked the LED
    previousMillis = currentMillis;
    // if the LED is off turn it on and vice-versa:
    ledState = not(ledState);
    // set the LED with the ledState of the variable:
    digitalWrite(led,  ledState);
    //Serial.println(adcVal_dcBusVoltage);
    //Serial.println(dcBusVoltage);

    int FaultPin = digitalRead(GPIO_FAULT);
    //Serial.print("Fault: ");
    //Serial.println(FaultPin);

    //Serial.print("HALL_1: ");
    //Serial.println(digitalRead(GPIO_CAP0_IN));
    //Serial.print("HALL_2: ");
    //Serial.println(digitalRead(GPIO_CAP1_IN));
    //Serial.print("HALL_3: ");
    //Serial.println(digitalRead(GPIO_CAP2_IN));

    TempAndHumidity newValues = dht.getTempAndHumidity();

    if (dht.getStatus() != 0) {
      //Serial.println("DHT22 error status: " + String(dht.getStatusString()));
    }

    float heatIndex = dht.computeHeatIndex(newValues.temperature, newValues.humidity);
    float dewPoint = dht.computeDewPoint(newValues.temperature, newValues.humidity);
    float cr = dht.getComfortRatio(cf, newValues.temperature, newValues.humidity);

    String comfortStatus;
    switch(cf) {
      case Comfort_OK:
        comfortStatus = "Comfort_OK";
        break;
      case Comfort_TooHot:
        comfortStatus = "Comfort_TooHot";
        break;
      case Comfort_TooCold:
        comfortStatus = "Comfort_TooCold";
        break;
      case Comfort_TooDry:
        comfortStatus = "Comfort_TooDry";
        break;
      case Comfort_TooHumid:
        comfortStatus = "Comfort_TooHumid";
        break;
      case Comfort_HotAndHumid:
        comfortStatus = "Comfort_HotAndHumid";
        break;
      case Comfort_HotAndDry:
        comfortStatus = "Comfort_HotAndDry";
        break;
      case Comfort_ColdAndHumid:
        comfortStatus = "Comfort_ColdAndHumid";
        break;
      case Comfort_ColdAndDry:
        comfortStatus = "Comfort_ColdAndDry";
        break;
      default:
        comfortStatus = "Unknown:";
        break;
    };
  
    //Serial.println(" T:" + String(newValues.temperature) + " H:" + String(newValues.humidity) + " I:" + String(heatIndex) + " D:" + String(dewPoint) + " " + comfortStatus);
    vspiCommand();
  }
*/


  //hall_sensor_value = (gpio_get_level(GPIO_NUM_27) * 1) + (gpio_get_level(GPIO_NUM_26) * 2) + (gpio_get_level(GPIO_NUM_25) * 4);
  hall_sensor_value = (gpio_get_level(GPIO_NUM_25) * 4) + (gpio_get_level(GPIO_NUM_26) * 2) + (gpio_get_level(GPIO_NUM_27) * 1);


  if (FLAG_ALLOW_MOTORSTART == true) {
    if (hall_sensor_value != hall_sensor_previous) {
      //printf("hall_sen val: %d\n", hall_sensor_value);
      if (hall_sensor_value == 2) {
          mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A);
          mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_B);
          mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A);
          mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B);
          //MCPWMXA to duty mode 1 and MCPWMXB to duty mode 0 or vice versa will generate MCPWM compliment signal of each other, there are also other ways to do it
          mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_1); //Set PWM0A to duty mode one
          mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, MCPWM_DUTY_MODE_0); //Set PWM0B back to duty mode zero
          //mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_BYPASS_FED, 20, 20);   //Deadtime of 2us
      }
      if (hall_sensor_value == 6) {
          mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);
          mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B);
          mcpwm_deadtime_disable(MCPWM_UNIT_0, MCPWM_TIMER_0);
          mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A);
          mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B);
          //MCPWMXA to duty mode 1 and MCPWMXB to duty mode 0 or vice versa will generate MCPWM compliment signal of each other, there are also other ways to do it
          mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, MCPWM_DUTY_MODE_1); //Set PWM2A to duty mode one
          mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_B, MCPWM_DUTY_MODE_0); //Set PWM2B back to duty mode zero
          //mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_BYPASS_FED, 20, 20);   //Deadtime of 2us
      }
      if (hall_sensor_value == 4) {
          mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A);
          mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B);
          mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);
          mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B);
          //MCPWMXA to duty mode 1 and MCPWMXB to duty mode 0 or vice versa will generate MCPWM compliment signal of each other, there are also other ways to do it
          mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, MCPWM_DUTY_MODE_1); //Set PWM2A to duty mode one
          mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_B, MCPWM_DUTY_MODE_0); //Set PWM2B back to duty mode zero
          //mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_BYPASS_FED, 20, 20);   //Deadtime of 2us
      }
      if (hall_sensor_value == 5) {
          mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A);
          mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_B);
          mcpwm_deadtime_disable(MCPWM_UNIT_0, MCPWM_TIMER_2);
          mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);
          mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B);
          //MCPWMXA to duty mode 1 and MCPWMXB to duty mode 0 or vice versa will generate MCPWM compliment signal of each other, there are also other ways to do it
          mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, MCPWM_DUTY_MODE_1); //Set PWM1A to duty mode one
          mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, MCPWM_DUTY_MODE_0); //Set PWM1B back to duty mode zero
          //mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_BYPASS_FED, 20, 20);   //Deadtime of 2us
      }
      if (hall_sensor_value == 1) {
          mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);
          mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B);
          mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A);
          mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_B);
          //MCPWMXA to duty mode 1 and MCPWMXB to duty mode 0 or vice versa will generate MCPWM compliment signal of each other, there are also other ways to do it
          mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, MCPWM_DUTY_MODE_1); //Set PWM1A to duty mode one
          mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, MCPWM_DUTY_MODE_0); //Set PWM1B back to duty mode zero
          //mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_BYPASS_FED, 20, 20);   //Deadtime of 2us
      }
      if (hall_sensor_value == 3) {
          mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A);
          mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B);
          mcpwm_deadtime_disable(MCPWM_UNIT_0, MCPWM_TIMER_1);
          mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A);
          mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_B);
          //MCPWMXA to duty mode 1 and MCPWMXB to duty mode 0 or vice versa will generate MCPWM compliment signal of each other, there are also other ways to do it
          mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_1); //Set PWM0A to duty mode one
          mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, MCPWM_DUTY_MODE_0); //Set PWM0B back to duty mode zero
          //mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_BYPASS_FED, 20, 20);   //Deadtime of 2us
      }
      
      hall_sensor_previous = hall_sensor_value;    
    }
  }

  else {
    mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A);
    mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_B);
    mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A);
    mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B);
    mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);
    mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B);
  } 
  
}

void vspiCommand() {
  //byte datablock = 0b01010101; // junk data to illustrate usage
  //unsigned int datablock = 0b1011101000000000;
  unsigned int datablock = 0b1000100000000000;
  unsigned int mydata = 0;
  //use it as you would the regular arduino SPI API
  vspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE1));
  digitalWrite(VSPI_SS, LOW); //pull SS slow to prep other end for transfer
  mydata = vspi->transfer16(datablock);  
  digitalWrite(VSPI_SS, HIGH); //pull ss high to signify end of data transfer
  vspi->endTransaction();
  Serial.print("SPI_ANSWER: ");
  Serial.println(mydata);
}

void configure_i2s(){
  i2s_config_t i2s_config = 
    {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN),  // I2S receive mode with ADC
    .sample_rate = 144000,                                                        // 144000
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,                                 // 16 bit I2S
    .channel_format = I2S_CHANNEL_FMT_ALL_LEFT,                                   // all the left channel
    .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),   // I2S format
    .intr_alloc_flags = 0,                                                        // none
    .dma_buf_count = 2,                                                           // number of DMA buffers 2 for fastness
    .dma_buf_len = 1024,                                                          // number of samples
    .use_apll = 0,                                                                // no Audio PLL
  };
  /*
  static const adc_i2s_pattern_t adc_i2s_pattern[] = {
        {.atten = ADC_ATTEN_DB_11, .bits = ADC_WIDTH_BIT_12, .channel = ADC1_CHANNEL_0}, // GPIO 36
        {.atten = ADC_ATTEN_DB_11, .bits = ADC_WIDTH_BIT_12, .channel = ADC1_CHANNEL_3} // GPIO 39
        };
*/

  adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_11db); //ADC Channel 0 is GPIO 36
  adc1_config_width(ADC_WIDTH_12Bit);
  
  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
 
  i2s_set_adc_mode(ADC_UNIT_1, ADC1_CHANNEL_0);
  //i2s_set_adc_mode(ADC_UNIT_1, adc_i2s_pattern, sizeof(adc_i2s_pattern));
  
  SET_PERI_REG_MASK(SYSCON_SARADC_CTRL2_REG, SYSCON_SARADC_SAR1_INV);
  i2s_adc_enable(I2S_NUM_0);
  vTaskDelay(5000);
}

static const inline void Sampling() {
  uint16_t DcBusVoltage = 0;
  uint16_t i2s_read_buff[100];
  size_t bytes_read;
  i2s_read(I2S_NUM_0, (char*)i2s_read_buff,100 * sizeof(uint16_t), &bytes_read, portMAX_DELAY); // 100 samples
  i2s_adc_disable(I2S_NUM_0);
  //Serial.println(i2s_read_buff[0]);
  DcBusVoltage = ((3.55/4096)*i2s_read_buff[0]) / (4.99/(62+4.99));
  Serial.println(DcBusVoltage);
  i2s_zero_dma_buffer(I2S_NUM_0);
  i2s_adc_enable(I2S_NUM_0);
  
  //esp_err_t err_str = adc2_vref_to_gpio(GPIO_NUM_25); // only GPIO 25, 26, 27 supported (1,095V measured)
  //Serial.println(err_str);
}
