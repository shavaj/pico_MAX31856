#include "MAX31856.hpp"

MAX31856::MAX31856(uint pin_miso, uint pin_sck, uint pin_mosi, uint pin_cs)
{

  this->miso = pin_miso;
  this->sck = pin_sck;
  this->mosi = pin_mosi;
  this->cs = pin_cs;

  gpio_init(pin_miso);
  gpio_set_dir(pin_miso, GPIO_OUT);
  gpio_put(pin_miso, 0);

  gpio_init(pin_sck);
  gpio_set_dir(pin_sck, GPIO_OUT);
  gpio_put(pin_sck, 0);

  gpio_init(pin_mosi);
  gpio_set_dir(pin_mosi, GPIO_OUT);
  gpio_put(pin_mosi, 0);

  gpio_init(pin_cs);
  gpio_set_dir(pin_cs, GPIO_OUT);
  gpio_put(pin_cs, 1);

  // assert on any fault
  uint8_t writeValue = 0;
  write_registers(MAX31856_MASK_REG, &writeValue, 1);

  // enable open circuit fault detection
  writeValue = MAX31856_CR0_OCFAULT0;
  write_registers(MAX31856_CR0_REG, &writeValue, 1);

  // set cold junction temperature offset to zero
  writeValue = 0;
  write_registers(MAX31856_MASK_REG, &writeValue, 1);

  // set Type K by default
  setThermocoupleType(MAX31856_TCTYPE_K);

  // set One-Shot conversion mode
  setConversionMode(MAX31856_ONESHOT);
}

void MAX31856::cs_select(void)
{
  asm volatile("nop \n nop \n nop");
  gpio_put(this->cs, 0); // Active low
  asm volatile("nop \n nop \n nop");
}

void MAX31856::cs_deselect(void)
{
  asm volatile("nop \n nop \n nop");
  gpio_put(this->cs, 1);
  asm volatile("nop \n nop \n nop");
}

void MAX31856::read_registers(uint8_t reg, uint8_t *buf, uint16_t len)
{
  // For this particular device, we send the device the register we want to read
  // first, then subsequently read from the device. The register is auto incrementing
  // so we don't need to keep sending the register we want, just the first.

  reg &= 0x7F; // MSB=0 for read, make sure top bit is not set

  cs_select();

  int8_t i = 0;
  uint8_t j = 0;

  for (i = 7; i >= 0; i--)
  {
    if (reg & (1 << i))
      gpio_put(this->mosi, 1);
    else
      gpio_put(this->mosi, 0);

    gpio_put(this->sck, 1);
    sleep_us(100);
    gpio_put(this->sck, 0);
  }

  sleep_ms(10);

  for (j = 0; j < len; j++)
  {
    buf[j] = 0;

    for (i = 7; i >= 0; i--)
    {
      gpio_put(this->sck, 1);
      sleep_us(100);
      gpio_put(this->sck, 0);

      if (gpio_get(this->miso))
        buf[j] |= (1 << i);
    }
  }

  cs_deselect();
}

void MAX31856::write_registers(uint8_t reg, uint8_t *buf, uint16_t len)
{
  // For this particular device, we send the device the register we want to read
  // first, then subsequently read from the device. The register is auto incrementing
  // so we don't need to keep sending the register we want, just the first.

  reg |= 0x80; // MSB=1 for write, make sure top bit is set

  cs_select();

  int8_t i = 0;
  uint8_t j = 0;

  for (i = 7; i >= 0; i--)
  {
    if (reg & (1 << i))
      gpio_put(this->mosi, 1);
    else
      gpio_put(this->mosi, 0);

    gpio_put(this->sck, 1);
    sleep_us(100);
    gpio_put(this->sck, 0);
  }

  for (j = 0; j < len; j++)
  {
    for (i = 7; i >= 0; i--)
    {
      if (buf[j] & (1 << i))
        gpio_put(this->mosi, 1);
      else
        gpio_put(this->mosi, 0);

      gpio_put(this->sck, 1);
      sleep_us(100);
      gpio_put(this->sck, 0);
    }
  }

  cs_deselect();
  sleep_ms(10);
}

/**************************************************************************/
/*!
    @brief  Set temperature conversion mode
    @param mode The conversion mode
*/
/**************************************************************************/
void MAX31856::setConversionMode(max31856_conversion_mode_t mode)
{
  conversionMode = mode;
  uint8_t t = 0;

  read_registers(MAX31856_CR0_REG, &t, 1); // get current register value

  if (conversionMode == MAX31856_CONTINUOUS)
  {
    t |= MAX31856_CR0_AUTOCONVERT; // turn on automatic
    t &= ~MAX31856_CR0_1SHOT;      // turn off one-shot
  }
  else
  {
    t &= ~MAX31856_CR0_AUTOCONVERT; // turn off automatic
    t |= MAX31856_CR0_1SHOT;        // turn on one-shot
  }

  write_registers(MAX31856_CR0_REG, &t, 1); // write value back to register
}

/**************************************************************************/
/*!
    @brief  Get temperature conversion mode
    @returns The conversion mode
*/
/**************************************************************************/
max31856_conversion_mode_t MAX31856::getConversionMode(void)
{
  return conversionMode;
}

/**************************************************************************/
/*!
    @brief  Set which kind of Thermocouple (K, J, T, etc) to detect & decode
    @param type The enumeration type of the thermocouple
*/
/**************************************************************************/
void MAX31856::setThermocoupleType(max31856_thermocoupletype_t type)
{
  uint8_t t = 0;

  read_registers(MAX31856_CR1_REG, &t, 1);

  t &= 0xF0; // mask off bottom 4 bits
  t |= (uint8_t)type & 0x0F;

  write_registers(MAX31856_CR1_REG, &t, 1);
}

max31856_thermocoupletype_t MAX31856::getThermocoupleType(void)
{
  uint8_t t = 0;

  read_registers(MAX31856_CR1_REG, &t, 1);
  t &= 0x0F;

  return (max31856_thermocoupletype_t)(t);
}

/**************************************************************************/
/*!
    @brief  Read the fault register (8 bits)
    @returns 8 bits of fault register data
*/
/**************************************************************************/
uint8_t MAX31856::readFault(void)
{
  uint8_t t = 0;
  read_registers(MAX31856_SR_REG, &t, 1);

  return t;
}

/**************************************************************************/
/*!
    @brief  Sets the threshhold for internal chip temperature range
    for fault detection. NOT the thermocouple temperature range!
    @param  low Low (min) temperature, signed 8 bit so -128 to 127 degrees C
    @param  high High (max) temperature, signed 8 bit so -128 to 127 degrees C
*/
/**************************************************************************/
void MAX31856::setColdJunctionFaultThreshholds(int8_t low, int8_t high)
{
  uint8_t t = low;
  write_registers(MAX31856_CJLF_REG, &t, 1);
  t = high;
  write_registers(MAX31856_CJHF_REG, &t, 1);
}

/**************************************************************************/
/*!
    @brief  Sets the mains noise filter. Can be set to 50 or 60hz.
    Defaults to 60hz. You need to call this if you live in a 50hz country.
    @param  noiseFilter One of MAX31856_NOISE_FILTER_50HZ or
   MAX31856_NOISE_FILTER_60HZ
*/
/**************************************************************************/
void MAX31856::setNoiseFilter(max31856_noise_filter_t noiseFilter)
{

  uint8_t t = 0;

  read_registers(MAX31856_CR0_REG, &t, 1);

  if (noiseFilter == MAX31856_NOISE_FILTER_50HZ)
  {
    t |= 0x01;
  }
  else
  {
    t &= 0xfe;
  }

  write_registers(MAX31856_CR0_REG, &t, 1);
}

/**************************************************************************/
/*!
    @brief  Sets the threshhold for thermocouple temperature range
    for fault detection. NOT the internal chip temperature range!
    @param  flow Low (min) temperature, floating point
    @param  fhigh High (max) temperature, floating point
*/
/**************************************************************************/
void MAX31856::setTempFaultThreshholds(float flow, float fhigh)
{
  int16_t low, high;

  flow *= 16;
  low = flow;

  fhigh *= 16;
  high = fhigh;

  uint8_t t = high >> 8;
  write_registers(MAX31856_LTHFTH_REG, &t, 1);
  t = high;
  write_registers(MAX31856_LTHFTL_REG, &t, 1);

  t = low >> 8;
  write_registers(MAX31856_LTLFTH_REG, &t, 1);
  t = low;
  write_registers(MAX31856_LTLFTL_REG, &t, 1);
}

/**************************************************************************/
/*!
    @brief  Begin a one-shot (read temperature only upon request) measurement.
    Value must be read later, not returned here!
*/
/**************************************************************************/
void MAX31856::triggerOneShot(void)
{

  if (conversionMode == MAX31856_CONTINUOUS)
    return;

  uint8_t t = 0;
  read_registers(MAX31856_CR0_REG, &t, 1); // get current register value

  t &= ~MAX31856_CR0_AUTOCONVERT; // turn off autoconvert
  t |= MAX31856_CR0_1SHOT;        // turn on one-shot

  write_registers(MAX31856_CR0_REG, &t, 1); // write value back to register
                                            // conversion starts when CS goes high
}

/**************************************************************************/
/*!
    @brief  Return status of temperature conversion.
    @returns true if conversion complete, otherwise false
*/
/**************************************************************************/
bool MAX31856::conversionComplete(void)
{

  if (conversionMode == MAX31856_CONTINUOUS)
    return true;

  uint8_t t = 0;
  read_registers(MAX31856_CR0_REG, &t, 1);
  return !(t & MAX31856_CR0_1SHOT);
}

/**************************************************************************/
/*!
    @brief  Return cold-junction (internal chip) temperature
    @returns Floating point temperature in Celsius
*/
/**************************************************************************/
float MAX31856::readCJTemperature(void)
{

  uint8_t buffer[2] = {0, 0};
  read_registers(MAX31856_CJTH_REG, buffer, 2);

  uint16_t ret = buffer[0];
  ret <<= 8;
  ret |= buffer[1];

  return ret / 256.0;
}

/**************************************************************************/
/*!
    @brief  Return hot-junction (thermocouple) temperature
    @returns Floating point temperature in Celsius
*/
/**************************************************************************/
float MAX31856::readThermocoupleTemperature(void)
{

  // for one-shot, make it happen
  if (conversionMode == MAX31856_ONESHOT)
  {

    uint16_t timeoutCounter = 0;

    triggerOneShot();

    while (!conversionComplete())
    {
      timeoutCounter++;
      if (timeoutCounter > 250)
        return 0;

      sleep_ms(10);
    }
  }

  // read the thermocouple temperature registers (3 bytes)
  uint8_t buffer[3] = {0, 0, 0};
  read_registers(MAX31856_LTCBH_REG, buffer, 3);

  uint32_t temp24 = buffer[0];
  temp24 <<= 8;
  temp24 |= buffer[1];
  temp24 <<= 8;
  temp24 |= buffer[2];

  // and compute temperature
  if (temp24 & 0x800000)
  {
    temp24 |= 0xFF000000; // fix sign
  }

  temp24 >>= 5; // bottom 5 bits are unused

  return temp24 * 0.0078125;
}