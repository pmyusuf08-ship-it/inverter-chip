#define ADS1115_DEBUG

#include "wokwi-api.h"
#include <stdio.h>
#include <stdlib.h>

#define BASE_ADDRESS 0x48

typedef struct {
  i2c_dev_t i2c_dev;
  pin_t pin_alrt;
  pin_t pin_addr;
  pin_t pin_vcc;   // supply pin, named "VCC" in the chip.json
  pin_t pin_a[4];  // A0, A1, A2, A3

  uint8_t pointer_reg;
  uint16_t config_reg;
  uint16_t conversion_reg;
  uint16_t lo_thresh;
  uint16_t hi_thresh;

  uint8_t rx_byte_count;
  uint8_t rx_buffer[3];
} chip_state_t;

// Full scale range per PGA setting (config bits 11:9)
static const float FSR_TABLE[8] = {
  6.144f, 4.096f, 2.048f, 1.024f, 0.512f, 0.256f, 0.256f, 0.256f
};

// Determine the I2C address from the ADDR pin connection
static uint8_t get_i2c_address(pin_t addr_pin, pin_t sda_pin, pin_t scl_pin) {
  // 1. ADDR -> SDA ?
  pin_mode(addr_pin, OUTPUT);
  pin_write(addr_pin, LOW);
  if (pin_read(sda_pin) == LOW) {
    pin_mode(addr_pin, INPUT_PULLDOWN);
    return BASE_ADDRESS + 2; // 0x4A
  }

  // 2. ADDR -> SCL ?
  if (pin_read(scl_pin) == LOW) {
    pin_mode(addr_pin, INPUT_PULLDOWN);
    return BASE_ADDRESS + 3; // 0x4B
  }

  // Back to input for the remaining checks
  pin_mode(addr_pin, INPUT);

  // 3. ADDR -> VCC ?
  if (pin_read(addr_pin) == HIGH) {
    return BASE_ADDRESS + 1; // 0x49
  }

  // 4. Default: ADDR -> GND (or floating)
  return BASE_ADDRESS; // 0x48
}

// Sample the selected input, update the conversion register and the ALRT pin
static void update_conversion(chip_state_t *chip) {
  uint8_t mux = (chip->config_reg >> 12) & 0x07;
  uint8_t channel = (mux >= 4) ? (mux - 4) : 0; // single-ended A0-A3

  // Wokwi returns the real voltage of the net (0..5V)
  float voltage = pin_adc_read(chip->pin_a[channel]);

  // Supply voltage: the real ADS1115 cannot read above ~VCC
  float vcc = pin_adc_read(chip->pin_vcc);
  if (vcc < 1.0f) vcc = 5.0f;   // fallback if VCC is not connected
  if (vcc > 5.5f) vcc = 5.5f;

  if (voltage > vcc)  voltage = vcc;
  if (voltage < 0.0f) voltage = 0.0f;

  // PGA / full scale range
  float fsr = FSR_TABLE[(chip->config_reg >> 9) & 0x07];

  // Raw = (V / FSR) * 32768, saturated to the signed 16-bit range
  float scaled = (voltage / fsr) * 32768.0f;
  if (scaled > 32767.0f) scaled = 32767.0f;
  if (scaled < 0.0f)     scaled = 0.0f;
  int16_t raw_adc = (int16_t)scaled;

  chip->conversion_reg = (uint16_t)raw_adc;

  // ALRT pin (comparator)
  int16_t lo = (int16_t)chip->lo_thresh;
  int16_t hi = (int16_t)chip->hi_thresh;
  bool comp_disabled = ((chip->config_reg & 0x0003) == 0x0003);
  bool window_mode   = (chip->config_reg & 0x0010) != 0;
  bool active_high   = (chip->config_reg & 0x0008) != 0;

  bool alert = false;
  if (!comp_disabled) {
    if (window_mode) {
      alert = (raw_adc > hi) || (raw_adc < lo);
    } else {
      alert = (raw_adc > hi);
    }
  }

  bool level = active_high ? alert : !alert;
  pin_write(chip->pin_alrt, level ? HIGH : LOW);

#ifdef ADS1115_DEBUG
  printf("A%d V=%.3f vcc=%.3f fsr=%.3f adc=%d\n", channel, voltage, vcc, fsr, raw_adc);
#endif
}

static uint8_t on_i2c_read(void *user_data) {
  chip_state_t *chip = (chip_state_t *)user_data;

  update_conversion(chip);

  uint16_t reg_val = 0;
  switch (chip->pointer_reg) {
    case 0x00: reg_val = chip->conversion_reg; break;
    case 0x01: reg_val = chip->config_reg; break;
    case 0x02: reg_val = chip->lo_thresh; break;
    case 0x03: reg_val = chip->hi_thresh; break;
    default:   reg_val = 0; break;
  }

  uint8_t res;
  if (chip->rx_byte_count == 0) {
    res = (reg_val >> 8) & 0xFF; // MSB
    chip->rx_byte_count = 1;
  } else {
    res = reg_val & 0xFF;        // LSB
    chip->rx_byte_count = 0;
  }
  return res;
}

static bool on_i2c_write(void *user_data, uint8_t data) {
  chip_state_t *chip = (chip_state_t *)user_data;

  if (chip->rx_byte_count == 0) {
    chip->pointer_reg = data & 0x03;
    chip->rx_buffer[0] = data;
    chip->rx_byte_count = 1;
  } else if (chip->rx_byte_count == 1) {
    chip->rx_buffer[1] = data;
    chip->rx_byte_count = 2;
  } else if (chip->rx_byte_count == 2) {
    chip->rx_buffer[2] = data;
    uint16_t value = ((uint16_t)chip->rx_buffer[1] << 8) | chip->rx_buffer[2];

    switch (chip->pointer_reg) {
      case 0x01: chip->config_reg = value | 0x8000; break; // OS bit always reads 1 (conversion ready)
      case 0x02: chip->lo_thresh = value; break;
      case 0x03: chip->hi_thresh = value; break;
    }
    chip->rx_byte_count = 0;
  }
  return true;
}

static bool on_i2c_connect(void *user_data, uint32_t address, bool connect) {
  chip_state_t *chip = (chip_state_t *)user_data;
  if (connect) {
    chip->rx_byte_count = 0;
  }
  return true;
}

void chip_init() {
  chip_state_t *chip = malloc(sizeof(chip_state_t));

  // ALRT & ADDR
  chip->pin_alrt = pin_init("ALRT", OUTPUT);
  pin_write(chip->pin_alrt, HIGH);

  chip->pin_addr = pin_init("ADDR", INPUT_PULLDOWN);

  // Supply pin
  chip->pin_vcc = pin_init("VCC", ANALOG);

  // I2C & analog inputs
  pin_t pin_scl = pin_init("SCL", INPUT);
  pin_t pin_sda = pin_init("SDA", INPUT);

  chip->pin_a[0] = pin_init("A0", ANALOG);
  chip->pin_a[1] = pin_init("A1", ANALOG);
  chip->pin_a[2] = pin_init("A2", ANALOG);
  chip->pin_a[3] = pin_init("A3", ANALOG);

  // Default registers (ADS1115 power-on defaults)
  chip->pointer_reg = 0x00;
  chip->config_reg = 0x8583;
  chip->conversion_reg = 0x0000;
  chip->lo_thresh = 0x8000;
  chip->hi_thresh = 0x7FFF;
  chip->rx_byte_count = 0;

  // I2C address (0x48, 0x49, 0x4A, 0x4B)
  uint8_t i2c_addr = get_i2c_address(chip->pin_addr, pin_sda, pin_scl);

  const i2c_config_t i2c_config = {
    .user_data = chip,
    .address = i2c_addr,
    .scl = pin_scl,
    .sda = pin_sda,
    .connect = on_i2c_connect,
    .read = on_i2c_read,
    .write = on_i2c_write,
  };

  chip->i2c_dev = i2c_init(&i2c_config);
  printf("ADS1115 Custom Chip Initialized at I2C Address: 0x%02X\n", i2c_addr);
}
