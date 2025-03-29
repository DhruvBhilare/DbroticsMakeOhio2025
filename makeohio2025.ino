#include "driver/i2s.h"
#include "Arduino.h"
#include "DFRobot_DF2301Q.h"

// I2S Configuration
#define MIC_DATA_IN      32
#define MIC_CLK          14
#define MIC_LRCLK        15
#define SPK_DATA_OUT     25
#define SPK_CLK          26
#define SPK_LRCLK        27
#define SAMPLE_RATE     16000
#define BUFFER_SIZE     1024

// Sensitivity control
#define GAIN_FACTOR     5
#define NOISE_GATE      100

// Voice recognition module
DFRobot_DF2301Q_I2C asr;

// Global variables
bool isActive = false;

void setup() {
  Serial.begin(115200);
  
  // Initialize voice recognition module
  while (!(asr.begin())) {
    Serial.println("Communication with voice module failed, please check connection");
    delay(3000);
  }
  Serial.println("Voice module initialized!");
  asr.setVolume(4);
  asr.setMuteMode(0);
  asr.setWakeTime(20);

  // I2S Microphone Configuration
  i2s_config_t mic_i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = BUFFER_SIZE,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };

  i2s_pin_config_t mic_pin_config = {
    I2S_PIN_NO_CHANGE, // mck_io_num
    MIC_CLK,           // bck_io_num
    MIC_LRCLK,         // ws_io_num
    I2S_PIN_NO_CHANGE, // data_out_num
    MIC_DATA_IN        // data_in_num
  };

  i2s_driver_install(I2S_NUM_0, &mic_i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &mic_pin_config);

  // I2S Speaker Configuration
  i2s_config_t spk_i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = BUFFER_SIZE,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };

  i2s_pin_config_t spk_pin_config = {
    I2S_PIN_NO_CHANGE, // mck_io_num
    SPK_CLK,           // bck_io_num
    SPK_LRCLK,         // ws_io_num
    SPK_DATA_OUT,      // data_out_num
    I2S_PIN_NO_CHANGE  // data_in_num
  };

  i2s_driver_install(I2S_NUM_1, &spk_i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_1, &spk_pin_config);

  Serial.println("Megaphone ready! Say 'ON' or 'OFF' to control.");
}

void loop() {
  uint8_t CMDID = asr.getCMDID();
  switch (CMDID) {
    case 5:  // "Turn on the megaphone"
      isActive = true;
      Serial.println("Megaphone ON");
      break;
    case 93:  // "Turn off the megaphone"
      isActive = false;
      Serial.println("Megaphone OFF");
      break;
    default:
      if (CMDID != 0) {
        Serial.print("Unknown CMDID = ");
        Serial.println(CMDID);
      }
  }

  if (isActive) {
    int16_t audio_buffer[BUFFER_SIZE];
    size_t bytes_read;

    // Read and process audio
    i2s_read(I2S_NUM_0, &audio_buffer, sizeof(audio_buffer), &bytes_read, portMAX_DELAY);
    for (int i = 0; i < bytes_read / 2; i++) {
      if (abs(audio_buffer[i]) > NOISE_GATE) {
        int32_t amplified = audio_buffer[i] * GAIN_FACTOR;
        audio_buffer[i] = (int16_t)constrain(amplified, -32768, 32767);
      } else {
        audio_buffer[i] = 0;
      }
    }

    // Output audio
    size_t bytes_written;
    i2s_write(I2S_NUM_1, &audio_buffer, bytes_read, &bytes_written, portMAX_DELAY);
  } 
  else {
    // Ensure no residual sound by flushing with silence
    int16_t silence_buffer[BUFFER_SIZE] = {0};
    size_t bytes_written;
    i2s_write(I2S_NUM_1, &silence_buffer, sizeof(silence_buffer), &bytes_written, portMAX_DELAY);
    i2s_zero_dma_buffer(I2S_NUM_1);
  }
}
