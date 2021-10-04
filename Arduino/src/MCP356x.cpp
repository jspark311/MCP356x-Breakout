/*
File:   MCP356x.cpp
Author: J. Ian Lindsay
*/

#include "MCP356x.h"

/*******************************************************************************
*      _______.___________.    ___   .___________. __    ______     _______.
*     /       |           |   /   \  |           ||  |  /      |   /       |
*    |   (----`---|  |----`  /  ^  \ `---|  |----`|  | |  ,----'  |   (----`
*     \   \       |  |      /  /_\  \    |  |     |  | |  |        \   \
* .----)   |      |  |     /  _____  \   |  |     |  | |  `----.----)   |
* |_______/       |__|    /__/     \__\  |__|     |__|  \______|_______/
*
* Static members and initializers should be located here.
*******************************************************************************/
// We can have up to four of these in a given system.
#define MCP356X_MAX_INSTANCES    2
volatile static MCP356x* INSTANCES[MCP356X_MAX_INSTANCES] = {0, };

static SPISettings spi_settings(12000000, MSBFIRST, SPI_MODE0);  // Max is 20MHz

/* Register widths in bytes. Index corresponds directly to register address. */
static const uint8_t MCP356x_reg_width[16] = {
  1, 1, 1, 1, 1, 1, 1, 3, 3, 3, 3, 3, 1, 1, 2, 2
};

static const uint16_t OSR1_VALUES[16] = {
  1, 1, 1, 1, 1, 2, 4, 8, 16, 32, 40, 48, 80, 96, 160, 192
};

static const uint16_t OSR3_VALUES[16] = {
  32, 64, 128, 256, 512, 512, 512, 512, 512, 512, 512, 512, 512, 512, 512, 512
};

static const char* CHAN_NAMES[16] = {
  "SE_0", "SE_1", "SE_2", "SE_3", "SE_4", "SE_5", "SE_6", "SE_7",
  "DIFF_A", "DIFF_B", "DIFF_C", "DIFF_D", "TEMP", "AVDD", "VCM", "OFFSET"
};

/*
* This is an ISR.
*/
void mcp356x_isr0() {
  ((MCP356x*) INSTANCES[0])->isr_fired = true;
}

/*
* This is an ISR.
*/
void mcp356x_isr1() {
  ((MCP356x*) INSTANCES[1])->isr_fired = true;
}



/*******************************************************************************
*   ___ _              ___      _ _              _      _
*  / __| |__ _ ______ | _ ) ___(_) |___ _ _ _ __| |__ _| |_ ___
* | (__| / _` (_-<_-< | _ \/ _ \ | / -_) '_| '_ \ / _` |  _/ -_)
*  \___|_\__,_/__/__/ |___/\___/_|_\___|_| | .__/_\__,_|\__\___|
*                                          |_|
* Constructors/destructors, class initialization functions and so-forth...
*******************************************************************************/

/*
* Constructor. Delegated.
*/
MCP356x::MCP356x(uint8_t irq_pin, uint8_t cs_pin, uint8_t mclk_pin) :
  MCP356x(irq_pin, cs_pin, mclk_pin, 0x01) {}

/*
* Constructor specifying device address.
*/
MCP356x::MCP356x(uint8_t irq_pin, uint8_t cs_pin, uint8_t mclk_pin, uint8_t addr) :
  _IRQ_PIN(irq_pin), _CS_PIN(cs_pin), _MCLK_PIN(mclk_pin), _DEV_ADDR(addr)
{
  bool unslotted = true;
  for (uint8_t i = 0; i < MCP356X_MAX_INSTANCES; i++) {
    if (unslotted) {
      if (nullptr == INSTANCES[i]) {
        _slot_number = i;
        INSTANCES[_slot_number] = this;
        unslotted = false;
      }
    }
  }
}


/*
* Destructor
*/
MCP356x::~MCP356x() {
  if (255 != _IRQ_PIN) {
    detachInterrupt(digitalPinToInterrupt(_IRQ_PIN));
  }
  INSTANCES[_slot_number] = nullptr;
}


/*
* Send the fastcommand to bounce the ADC.
* Returns...
*   -2 if refresh found unexpected values.
*   -1 if there was a problem writing the reset command or refreshing the registers.
*   0  if reset command was sent successfully.
*/
int8_t MCP356x::reset() {
  int8_t ret = _send_fast_command(0x38);
  if (0 == ret) {
    delay(75);   // <--- Arbitrary delay value
    digitalWrite(_CS_PIN, 0);
    digitalWrite(_CS_PIN, 1);
    digitalWrite(_CS_PIN, 0);
    digitalWrite(_CS_PIN, 1);   // Twiddle the CS line to ensure SPI reset.
    ret = _clear_registers();
    if (0 == ret) {
      delay(75);   // <--- Arbitrary delay value
      ret = refresh();
    }
  }
  return ret;
}


/*
* Resets the chip and writes our default configuration.
* Returns...
*   -7 if offset cal failed.
*   -6 if clock detection failed.
*   -5 if register refresh failed.
*   -4 if pin setup failed.
*   -3 if bad bus reference.
*   -2 if reset failed.
*   -1 if clock detection or calibration failed for some reason.
*   0  on success.
*/
int8_t MCP356x::init(SPIClass* b) {
  int8_t pin_setup_ret = _ll_pin_init();  // Configure the pins if they are not already.
  int8_t ret = -4;
  if (pin_setup_ret >= 0) {
    ret = -3;
    if (nullptr != b) {
      ret = -2;
      _bus = b;
      if (0 == reset()) {
        ret = -5;
        if (0 == _post_reset_fxn()) {
          if (!_mclk_in_bounds()) {   // Need to detect MCLK...
            ret = -6;
            int8_t det_ret = _detect_adc_clock();
            switch (det_ret) {
              case -3:    break;  // -3 if the class isn't ready for this measurement.
              case -2:    break;  // -2 if measurement timed out.
              case -1:    break;  // -1 if there was some mechanical problem communicating with the ADC
              case 0:     break;  // 0  if a clock signal within expected range was determined
              case 1:     break;  // 1  if we got a clock rate that was out of bounds or appears nonsensical
              default:    break;
            }
            //Serial.print("_detect_adc_clock returned ");
            //Serial.println(det_ret);
          }
          if (_mclk_in_bounds()) {
            switch (_calibrate_offset()) {
              case 0:   ret = 0;    break;
              default:  ret = -7;   break;
            }
          }
        }
      }
    }
  }
  return ret;
}


/*
* Sets some of the more specialized features of the chip.
* Pass as an argument a select combination of flags defined in the
*   header file.
* NOTE: This fxn must be called ahead of first init(), since our
*   pin configuration might be in conflict with ADC configuration
*   otherwise.
* Returns...
*   -1 if called too late for a provided option.
*   0  on success.
*/
int8_t MCP356x::setOption(uint32_t flgs) {
  int8_t ret = 0;
  if (flgs & MCP356X_FLAG_USE_INTERNAL_CLK) {
    if (!(_mcp356x_flag(MCP356X_FLAG_PINS_CONFIGURED))) {
      // Only allow this change if the pins are not yet configured.
      _mcp356x_set_flag(MCP356X_FLAG_USE_INTERNAL_CLK);
      _mcp356x_clear_flag(MCP356X_FLAG_GENERATE_MCLK);
    }
    else {
      ret = -1;
    }
  }
  if (flgs & MCP356X_FLAG_GENERATE_MCLK) {
    if (!(_mcp356x_flag(MCP356X_FLAG_PINS_CONFIGURED))) {
      // Only allow this change if the pins are not yet configured.
      _mcp356x_set_flag(MCP356X_FLAG_GENERATE_MCLK);
      _mcp356x_clear_flag(MCP356X_FLAG_USE_INTERNAL_CLK);
    }
    else {
      ret = -1;
    }
  }
  if (flgs & MCP356X_FLAG_3RD_ORDER_TEMP) {
    _mcp356x_set_flag(MCP356X_FLAG_3RD_ORDER_TEMP);
  }
  return ret;
}


/*
* Handles our configurations after reset.
* Returns...
*   -1 on failure to write a register.
*   0  on success.
*/
int8_t MCP356x::_post_reset_fxn() {
  int8_t ret = -1;
  uint32_t c0_val = 0x000000C3;
  //uint32_t c1_val = 0x00000000;

  // Enable fast command, disable IRQ on conversion start, IRQ pin is open-drain.
  ret = _write_register(MCP356xRegister::IRQ, 0x00000002);
  if (0 == ret) {
    if (_mcp356x_flag(MCP356X_FLAG_USE_INTERNAL_CLK)) {
      c0_val &= 0xFFFFFFCF;   // Set CLK_SEL to use internal clock with no pin output.
      c0_val |= 0x00000020;
    }
    ret = _write_register(MCP356xRegister::CONFIG0, c0_val);
    if (0 == ret) {
      if (_mcp356x_flag(MCP356X_FLAG_USE_INTERNAL_CLK)) {
        _mcp356x_set_flag(MCP356X_FLAG_MCLK_RUNNING);
      }
      // For simplicity, we select a 32-bit sign-extended data representation with
      //   channel identifiers.
      ret = _write_register(MCP356xRegister::CONFIG3, 0x000000F0);
      if (0 == ret) {
        _mcp356x_set_flag(MCP356X_FLAG_INITIALIZED);
      }
    }
  }
  return ret;
}


/*
* Reads the part's output register and performs the needed manipulation on the
*   data, according to the current configuration.
* NOTE: If the application recently called discardUnsettledSamples(), the register
*   shadow will not be updated unless the timeout has expired. But no matter what,
*   the true value of the register will be returned from this function. So if
*   the application wants to know if _valid_ data was read, it should call
*   newValue() or scanComplete() ahead of using it.
* Returns...
*   -1 on error reading from registers.
*   0  on successful read, but nothing to do.
*   1  if data was ready and read, but discarded due to being within the
*        declared settling time.
*   2  if the read resulted in a full set of data for all channels being scanned.
*/
int8_t MCP356x::read() {
  int8_t ret = _proc_irq_register();
  switch (ret) {
    case -1:
      break;
    case 0:
      break;
    case 1:   // Read the data register.
      _read_register(MCP356xRegister::ADCDATA);
      millis_last_read = millis();
      if (_discard_until_millis <= millis_last_read) {
        _normalize_data_register();
        if (scanComplete()) {
          ret = 2;
        }
      }
      read_count++;
      read_accumulator++;
      if (millis_last_read - millis_last_window >= 1000) {
        millis_last_window = millis_last_read;
        reads_per_second = read_accumulator;
        read_accumulator = 0;
      }
      break;
    default:
      break;
  }
  return ret;
}


/*
* Read the IRQ register and respond to its contents.
* Returns...
*   -1 if reading the IRQ register failed.
*   0  if there is nothing to do after this.
*   1  if we need to read the data register.
*/
int8_t MCP356x::_proc_irq_register() {
  int8_t ret = -1;
  if (0 == _read_register(MCP356xRegister::IRQ)) {
    ret = 0;
    uint8_t irq_reg_data = (uint8_t) reg_shadows[(uint8_t) MCP356xRegister::IRQ];
    _mcp356x_set_flag(MCP356X_FLAG_CRC_ERROR, (0 == (0x20 & irq_reg_data)));
    if (0 == (0x40 & irq_reg_data)) {   // Conversion is finished.
      ret = 1;
    }
    if (0 == (0x08 & irq_reg_data)) { // Power-on-Reset has happened.
      // Not sure why this happened, but reset the class.
      //_post_reset_fxn();
      digitalWrite(_CS_PIN, 0);
      digitalWrite(_CS_PIN, 1);
      digitalWrite(_CS_PIN, 0);
      digitalWrite(_CS_PIN, 1);
    }
    if (0 == (0x20 & irq_reg_data)) { // CRC config error.
      // Something is sideways in the configuration.
      // Send start/restart conversion. Might also write 0xA5 to the LOCK register.
      _write_register(MCP356xRegister::LOCK, 0x000000A5);
      _send_fast_command(0x28);
    }
    if (0x01 & irq_reg_data) {   // Conversion started
      // We don't configure the class this way, and don't observe the IRQ.
    }
  }
  isr_fired = !digitalRead(_IRQ_PIN);
  return ret;
}


/*
* Returns the number of channels this part supports. Should be (1, 2, 4). Any
*   other value is invalid and indicates a need for a register sync (if 0) or
*   the wrong device entirely.
*/
uint8_t MCP356x::_channel_count() {
  switch ((uint16_t) reg_shadows[(uint8_t) MCP356xRegister::RESERVED2]) {
    case 0x000C:   return 2;  // MCP3561
    case 0x000D:   return 4;  // MCP3562
    case 0x000F:   return 8;  // MCP3564
  }
  return 0;
}


/*
* Walk the value backward through the ADC transfer function to arrive at the
*   voltage on that channel.
* If the application didn't set a reference voltage, we assume it is equal to
*   AVdd. If that channel has never been read, the value will default to 3.3v.
*/
double MCP356x::valueAsVoltage(MCP356xChannel chan) {
  float  vrp  = _vref_plus;
  float  vrm  = _vref_minus;
  double result = 0.0;
  switch (chan) {
    case MCP356xChannel::SE_0:   // Single-ended channels.
    case MCP356xChannel::SE_1:
    case MCP356xChannel::SE_2:
    case MCP356xChannel::SE_3:
    case MCP356xChannel::SE_4:
    case MCP356xChannel::SE_5:
    case MCP356xChannel::SE_6:
    case MCP356xChannel::SE_7:
    case MCP356xChannel::DIFF_A:   // Differential channels.
    case MCP356xChannel::DIFF_B:
    case MCP356xChannel::DIFF_C:
    case MCP356xChannel::DIFF_D:
    case MCP356xChannel::OFFSET:
      result = (value(chan) * (vrp - vrm)) / (8388608.0 * _gain_value());
      break;
    case MCP356xChannel::TEMP:
      // TODO: voltage transfer fxn for temperature diode.
      break;
    case MCP356xChannel::AVDD:
      result = value(chan) / (0.33 * 8388608.0);   // Gain on this chan is always 0.33.
      break;
    case MCP356xChannel::VCM:
      result = (value(chan) * (vrp - vrm)) / 8388608.0;
      break;
  }
  return result;
}


/*
* Given a channel, return the last value.
*/
int32_t MCP356x::value(MCP356xChannel chan) {
  _channel_clear_new_flag(chan);
  return channel_vals[(uint8_t) chan & 0x0F];
}


/*
* Sets the offset calibration for the ADC.
* Enables the offset feature if the value is non-zero.
*/
int8_t MCP356x::setOffsetCalibration(int32_t offset) {
  int8_t ret = _write_register(MCP356xRegister::OFFSETCAL, (uint32_t) offset);
  if (0 == ret) {
    uint32_t c_val = reg_shadows[(uint8_t) MCP356xRegister::CONFIG3];
    c_val = (0 != offset) ? (c_val | 0x00000002) : (c_val & 0xFFFFFFFD);
    ret = _write_register(MCP356xRegister::CONFIG3, c_val);
  }
  return ret;
}


/*
* Sets the scale calibration for the ADC.
* Enables the scale feature if the value is non-zero.
*/
int8_t MCP356x::setGainCalibration(int32_t multiplier) {
  _write_register(MCP356xRegister::GAINCAL, (uint32_t) multiplier);
  uint32_t c_val = reg_shadows[(uint8_t) MCP356xRegister::CONFIG3];
  c_val = (0 != multiplier) ? (c_val | 0x00000001) : (c_val & 0xFFFFFFFE);
  return _write_register(MCP356xRegister::CONFIG3, c_val);
}


/*
* Changes the gain setting.
*/
int8_t MCP356x::setGain(MCP356xGain g) {
  uint32_t c_val = reg_shadows[(uint8_t) MCP356xRegister::CONFIG2];
  uint32_t gain_val = (c_val & 0xFFFFFFC7) | ((uint32_t) g << 3);
  return _write_register(MCP356xRegister::CONFIG2, gain_val);
}

/*
* Returns the current gain setting.
*/
MCP356xGain MCP356x::getGain() {
  return (MCP356xGain) ((reg_shadows[(uint8_t) MCP356xRegister::CONFIG2] >> 3) & 0x07);
}

/*
* Changes the BOOST setting.
*/
int8_t MCP356x::setBiasCurrent(MCP356xBiasCurrent d) {
  uint32_t c0_val = reg_shadows[(uint8_t) MCP356xRegister::CONFIG0] & 0x00F3FFFF;
  c0_val += ((((uint8_t) d) & 0x03) << 18);
  return _write_register(MCP356xRegister::CONFIG0, c0_val);
}


/*
* Changes the AMCLK prescaler.
* Returns...
*   -2 if MCLK frequency is unknown, but prescalar setting succeeded.
*   -1 if reconfiguration of prescalar failed.
*   0  on success.
*/
int8_t MCP356x::setAMCLKPrescaler(MCP356xAMCLKPrescaler d) {
  uint32_t c1_val = reg_shadows[(uint8_t) MCP356xRegister::CONFIG1] & 0x00FFFF3F;
  c1_val |= ((((uint8_t) d) & 0x03) << 6);
  int8_t ret = _write_register(MCP356xRegister::CONFIG1, c1_val);
  if (0 == ret) {
    ret = _recalculate_clk_tree();
  }
  return ret;
}


/*
* Changes the oversampling ratio.
*/
int8_t MCP356x::setOversamplingRatio(MCP356xOversamplingRatio d) {
  uint32_t c1_val = reg_shadows[(uint8_t) MCP356xRegister::CONFIG1] & 0x00FFFFC3;
  c1_val |= ((((uint8_t) d) & 0x0F) << 2);
  int8_t ret = _write_register(MCP356xRegister::CONFIG1, c1_val);
  if (0 == ret) {
    ret = _recalculate_settling_time();
  }
  return ret;
}


/*
* Gets the current oversampling ratio.
*/
MCP356xOversamplingRatio MCP356x::getOversamplingRatio() {
  return (MCP356xOversamplingRatio) ((reg_shadows[(uint8_t) MCP356xRegister::CONFIG1] & 0x0000003C) >> 2);
}


/**
* Application-facing accessor for VREF selection, if available.
*
* @return true if Vref is using the internally generated value.
*/
bool MCP356x::usingInternalVref() {
  bool ret = false;
  if (_mcp356x_flag(MCP356X_FLAG_HAS_INTRNL_VREF)) {
    ret = (0 != (reg_shadows[(uint8_t) MCP356xRegister::CONFIG0] & 0x00000040));
  }
  return ret;
}


/**
* Application-facing accessor for VREF selection, if available.
*
* @param x true will enable the internal Vref, if available.
* @return 0 on success, -1 on "not supported", or -2 on I/O failure.
*/
int8_t MCP356x::useInternalVref(bool x) {
  int8_t ret = -1;
  if (_mcp356x_flag(MCP356X_FLAG_HAS_INTRNL_VREF)) {
    uint32_t c0_val = reg_shadows[(uint8_t) MCP356xRegister::CONFIG0] & 0x00FFFFBF;
    if (x) {
      c0_val |= 0x00000040;
      _vref_plus  = 2.4;
      _vref_minus = 0;
    }
    ret = (0 == _write_register(MCP356xRegister::CONFIG0, c0_val)) ? 0 : -2;
  }
  return ret;
}



/*******************************************************************************
* Internal functions
*******************************************************************************/

/*
* Setup the low-level pin details. Execution is idempotent.
* Returns...
*   -1 if the pin setup is wrong. Class must halt.
*   0  if the pin setup is complete.
*   1  if the pin setup is complete, and the clock needs measurement.
*/
int8_t MCP356x::_ll_pin_init() {
  int8_t ret = -1;
  if (_mcp356x_flag(MCP356X_FLAG_PINS_CONFIGURED)) {
    ret = 0;
  }
  else if (255 != _CS_PIN) {
    ret = 1;
    pinMode(_CS_PIN, OUTPUT);
    digitalWrite(_CS_PIN, 1);
    if (255 != _IRQ_PIN) {
      pinMode(_IRQ_PIN, INPUT_PULLUP);
      switch (_slot_number) {  // TODO: This is terrible. You know better.
        case 0:
          attachInterrupt(digitalPinToInterrupt(_IRQ_PIN), mcp356x_isr0, FALLING);
          break;
        case 1:
          attachInterrupt(digitalPinToInterrupt(_IRQ_PIN), mcp356x_isr1, FALLING);
          break;
      }
    }
    if (255 != _MCLK_PIN) {
      // If we have MCLK, we need to generate a squarewave on that pin.
      // Otherwise, we hope that the board has an XTAL attached.
      if (_mcp356x_flag(MCP356X_FLAG_USE_INTERNAL_CLK)) {
        // TODO: We presently do nothing with this signal. But we might tap it
        //   for frequency measurement of the internal OSC.
        pinMode(_MCLK_PIN, INPUT);
      }
      else {
        pinMode(_MCLK_PIN, OUTPUT);
        if (_mcp356x_flag(MCP356X_FLAG_GENERATE_MCLK)) {
          // NOTE: Not all pin support this. Works for some pins on some MCUs.
          analogWriteFrequency(_MCLK_PIN, 4915200);
          analogWrite(_MCLK_PIN, 128);
          _mclk_freq = 4915200.0;
        }
        else {
          // There is a hardware oscillator whose enable pin we control with
          //   the MCLK pin. Set the pin high (enabled) and measure the clock.
          digitalWrite(_MCLK_PIN, 1);
        }
        _mcp356x_set_flag(MCP356X_FLAG_MCLK_RUNNING);
        ret = _recalculate_clk_tree();
      }
    }
    _mcp356x_set_flag(MCP356X_FLAG_PINS_CONFIGURED);
  }
  return ret;
}


/*
* Resets the internal register shadows to their default values.
* Also resets internal second-order data to avoid corrupted results.
*/
int8_t MCP356x::_clear_registers() {
  uint32_t flg_mask = MCP356X_FLAG_RESET_MASK;
  if (!(_mcp356x_flag(MCP356X_FLAG_USE_INTERNAL_CLK))) {
    // The only way the clock isn't running is if it is running internally.
    flg_mask |= MCP356X_FLAG_MCLK_RUNNING;
  }
  _flags = _flags & flg_mask;  // Reset the flags.
  for (uint8_t i = 0; i < 16; i++) {
    // We decline to revert RESERVED2, since we need it for device identity.
    // RESERVED2 (0x000F for 3564, 0xD for 3562, 0xC for 3561)
    reg_shadows[i]   = (14 != i) ? 0 : reg_shadows[i];
    channel_vals[i]  = 0;
  }
  _channel_flags        = 0;
  _discard_until_millis = 0;
  _settling_ms          = 0;
  read_count            = 0;
  read_accumulator      = 0;
  reads_per_second      = 0;
  millis_last_read      = 0;
  millis_last_window    = 0;
  return 0;
}


/*
* Does safety checks on the given value/register combo and writes to the part.
*/
int8_t MCP356x::_write_register(MCP356xRegister r, uint32_t val) {
  uint32_t safe_val = 0;
  int8_t ret = -1;
  uint8_t register_size = MCP356x_reg_width[(uint8_t) r];
  switch (r) {
    // Filter out the unimplemented bits.
    case MCP356xRegister::CONFIG1:
      safe_val = val & 0xFFFFFFFC;
      break;
    case MCP356xRegister::CONFIG2:
      safe_val = val | (_mcp356x_flag(MCP356X_FLAG_HAS_INTRNL_VREF) ? 0x00000001 : 0x00000003);
      break;
    case MCP356xRegister::SCAN:
      safe_val = val & 0xFFE0FFFF;
      break;
    case MCP356xRegister::RESERVED0:
      safe_val = 0x00900000;
      break;
    case MCP356xRegister::RESERVED1:
      safe_val = (_mcp356x_flag(MCP356X_FLAG_HAS_INTRNL_VREF) ? 0x00000030 : 0x00000050);
      break;
    case MCP356xRegister::RESERVED2:
      safe_val = val & 0x0000000F;
      break;
    // No safety required.
    case MCP356xRegister::CONFIG0:
    case MCP356xRegister::CONFIG3:
    case MCP356xRegister::IRQ:
    case MCP356xRegister::MUX:
    case MCP356xRegister::TIMER:
    case MCP356xRegister::OFFSETCAL:
    case MCP356xRegister::GAINCAL:
    case MCP356xRegister::LOCK:
      safe_val = val;
      break;
    // Not writable.
    case MCP356xRegister::ADCDATA:
    case MCP356xRegister::CRCCFG:
      return -3;
  }

  _bus->beginTransaction(spi_settings);
  digitalWrite(_CS_PIN, 0);
  _bus->transfer(_get_reg_addr(r));
  switch (register_size) {
    case 3:    _bus->transfer((uint8_t) (safe_val >> 16) & 0xFF);   // MSB-first
    case 2:    _bus->transfer((uint8_t) (safe_val >> 8)  & 0xFF);
    case 1:    _bus->transfer((uint8_t) safe_val & 0xFF);
      ret = 0;
      reg_shadows[(uint8_t) r] = safe_val;
      break;
    default:
      ret = -2;   // Error on unexpected width.
  }
  digitalWrite(_CS_PIN, 1);
  _bus->endTransaction();
  return ret;
}


/*
* Reads the given register.
*/
int8_t MCP356x::_read_register(MCP356xRegister r) {
  uint8_t bytes_to_read = MCP356x_reg_width[(uint8_t) r];
  if (MCP356xRegister::ADCDATA == r) {
    bytes_to_read = _output_coding_bytes();
  }

  _bus->beginTransaction(spi_settings);
  digitalWrite(_CS_PIN, 0);
  _bus->transfer((uint8_t) _get_reg_addr(r) | 0x01);
  uint32_t temp_val = 0;
  for (int i = 0; i < bytes_to_read; i++) {
    temp_val = (temp_val << 8) + _bus->transfer(0);
  }
  digitalWrite(_CS_PIN, 1);
  _bus->endTransaction();
  reg_shadows[(uint8_t) r] = temp_val;
  return 0;
}


/*
* Returns the number of bytes to be read from the data register.
*/
uint8_t MCP356x::_output_coding_bytes() {
  return (0 == reg_shadows[(uint8_t) MCP356xRegister::CONFIG3]) ? 3 : 4;
}


/*
* Output coding correction and observation.
* TODO: We rely on (assume) the output coding having chan-id and SGN.
*/
int8_t MCP356x::_normalize_data_register() {
  uint32_t rval = reg_shadows[(uint8_t) MCP356xRegister::ADCDATA];
  MCP356xChannel chan = (MCP356xChannel) ((rval >> 28) & 0x0F);

  // Sign extend, if needed.
  int32_t nval = (int32_t) (rval & 0x01000000) ? (rval | 0xFE000000) : (rval & 0x01FFFFFF);

  // Update the over-range marker...
  channel_vals[(uint8_t) chan]  = nval;   // Store the decoded ADC reading.
  _channel_set_ovr_flag(chan, ((nval > 8388609) | (nval < -8388609)));
  _channel_set_new_flag(chan);            // Mark the channel as updated.

  switch (chan) {
    // Different channels are interpreted differently...
    case MCP356xChannel::SE_0:   // Single-ended channels.
    case MCP356xChannel::SE_1:
    case MCP356xChannel::SE_2:
    case MCP356xChannel::SE_3:
    case MCP356xChannel::SE_4:
    case MCP356xChannel::SE_5:
    case MCP356xChannel::SE_6:
    case MCP356xChannel::SE_7:
    case MCP356xChannel::DIFF_A:   // Differential channels.
    case MCP356xChannel::DIFF_B:
    case MCP356xChannel::DIFF_C:
    case MCP356xChannel::DIFF_D:
      break;
    case MCP356xChannel::TEMP:
      break;
    case MCP356xChannel::AVDD:
      _mcp356x_set_flag(MCP356X_FLAG_SAMPLED_AVDD);
      if (!_mcp356x_flag(MCP356X_FLAG_VREF_DECLARED)) {
        // If we are scanning the AVDD channel, we use that instead of the
        //   assumed 3.3v.
        //_vref_plus = nval / (8388608.0 * 0.33);
      }
      if (MCP356X_FLAG_ALL_CAL_MASK == (_mcp356x_flags() & MCP356X_FLAG_ALL_CAL_MASK)) {
        _mark_calibrated();
      }
      break;
    case MCP356xChannel::VCM:
      // Nothing done here yet. Value should always be near 1.2v.
      _mcp356x_set_flag(MCP356X_FLAG_SAMPLED_VCM);
      if (MCP356X_FLAG_ALL_CAL_MASK == (_mcp356x_flags() & MCP356X_FLAG_ALL_CAL_MASK)) {
        _mark_calibrated();
      }
      break;
    case MCP356xChannel::OFFSET:
      if (0 == setOffsetCalibration(nval)) {
        _mcp356x_set_flag(MCP356X_FLAG_SAMPLED_OFFSET);
      }
      if (MCP356X_FLAG_ALL_CAL_MASK == (_mcp356x_flags() & MCP356X_FLAG_ALL_CAL_MASK)) {
        _mark_calibrated();
      }
      break;
  }
  return 0;
}


/*
* Returns the gain of the ADC as a float, or zero on enum out-of-bounds.
*/
float MCP356x::_gain_value() {
  switch (getGain()) {
    case MCP356xGain::GAIN_ONETHIRD: return 0.33;
    case MCP356xGain::GAIN_1:        return 1.0;
    case MCP356xGain::GAIN_2:        return 2.0;
    case MCP356xGain::GAIN_4:        return 4.0;
    case MCP356xGain::GAIN_8:        return 8.0;
    case MCP356xGain::GAIN_16:       return 16.0;
    case MCP356xGain::GAIN_32:       return 32.0;
    case MCP356xGain::GAIN_64:       return 64.0;
  }
  return 0.0;
}


/*
* Applies the device address and properly shifts the register address into
*   a control byte. Always sets up for incremental read/write.
* This never fails and always returns a byte to be used as the first byte
*   in an SPI transaction with the ADC.
*/
uint8_t MCP356x::_get_reg_addr(MCP356xRegister r) {
  return (((_DEV_ADDR & 0x03) << 6) | (((uint8_t) r) << 2) | 0x02);
}


/*
* Cause a full refresh. Update our shadows with the state of the hardware registers.
* Returns...
*   -2 if reads worked, but register values don't match defaults.
*   -1 if a register read failed.
*   0  if we found an MCP356x.
*/
int8_t MCP356x::refresh() {
  uint8_t i   = 0;
  int8_t  ret = 0;
  while ((0 == ret) & (i < 16)) {
    ret = _read_register((MCP356xRegister) i++);
  }
  if (0 == ret) {
    ret = -2;
    if (0x00900000 == reg_shadows[(uint8_t) MCP356xRegister::RESERVED0]) {
      uint8_t res1_val = (uint8_t) reg_shadows[(uint8_t) MCP356xRegister::RESERVED1];
      switch (res1_val) {
        case 0x30:
        case 0x50:
          // If the chip has an internal Vref, it will start up running and
          //   connected.
          _mcp356x_set_flag(MCP356X_FLAG_HAS_INTRNL_VREF, (res1_val == 0x30));
          switch (reg_shadows[(uint8_t) MCP356xRegister::RESERVED2]) {
            case 0x0C:
            case 0x0D:
            case 0x0F:
              _mcp356x_set_flag(MCP356X_FLAG_DEVICE_PRESENT);
              ret = 0;
              break;
            default:
              //Serial.println("bad RESERVED2 value\n");
              break;
          }
          break;
        default:
          //Serial.println("bad RESERVED1 value\n");
          break;
      }
    }
    //else Serial.println("bad RESERVED0 value.");
  }
  return ret;
}


/*
* Causes the ADC to throw away samples after reading them. This should be
*   invoked when it is known that the analog value is changing, and we only want
*   reports of fully-settled values.
*/
void MCP356x::discardUnsettledSamples() {
  _discard_until_millis = getSettlingTime() + _circuit_settle_ms + millis();
}


/*
* Sends a fast command that is not register access.
* Returns...
*    -1 on failure to write SPI.
*    0  on success.
*/
int8_t MCP356x::_send_fast_command(uint8_t cmd) {
  int8_t ret = 0;
  _bus->beginTransaction(spi_settings);
  digitalWrite(_CS_PIN, 0);
  _bus->transfer((uint8_t) ((_DEV_ADDR & 0x03) << 6) | cmd);
  digitalWrite(_CS_PIN, 1);
  _bus->endTransaction();
  return ret;
}


/*
* Sets the list of channels to read in SCAN mode.
* Returns...
*   -4 if a channel was requested that the driver doesn't support.
*   -3 if a channel was requested that the hardware doesn't support.
*   -2 if no channels were given.
*   -1 on failure to write the SCAN register.
*   0  on success.
*/
int8_t MCP356x::setScanChannels(int count, ...) {
  int8_t ret = (count > 0) ? 0 : -2;
  uint8_t chan_count = _channel_count();
  uint32_t existing_scan = reg_shadows[(uint8_t) MCP356xRegister::SCAN];
  uint32_t chans = 0;
  va_list args;
  va_start(args, count);
  for (int i = 0; i < count; i++) {
    MCP356xChannel chan = va_arg(args, MCP356xChannel);
    switch (chan) {
      case MCP356xChannel::SE_0:
      case MCP356xChannel::SE_1:
      case MCP356xChannel::DIFF_A:
      case MCP356xChannel::TEMP:
      case MCP356xChannel::AVDD:
      case MCP356xChannel::VCM:
      case MCP356xChannel::OFFSET:
        if (2 > chan_count) {    ret = -3;    }
        break;
      case MCP356xChannel::SE_2:
      case MCP356xChannel::SE_3:
      case MCP356xChannel::DIFF_B:
        if (4 > chan_count) {    ret = -3;    }
        break;
      case MCP356xChannel::SE_4:
      case MCP356xChannel::SE_5:
      case MCP356xChannel::SE_6:
      case MCP356xChannel::SE_7:
      case MCP356xChannel::DIFF_C:
      case MCP356xChannel::DIFF_D:
        if (8 != chan_count) {   ret = -3;    }
        break;
      default:
        ret = -4;
        break;
    }
    if (0 == ret) {
      chans = chans | (1 << ((uint8_t) chan));
    }
  }
  va_end(args);
  if (0 == ret) {   // If there were no foul ups, we can write the registers.
    chans = chans | (existing_scan & 0xFFFF0000);
    _channel_backup = chans;
    if (_mcp356x_flag(MCP356X_FLAG_CALIBRATED)) {
      ret = _set_scan_channels(chans);
    }
    else {
      _channel_backup = chans;
      ret = 0;
    }
  }
  return ret;
}


int8_t MCP356x::_set_scan_channels(uint32_t rval) {
  if (!_mcp356x_flag(MCP356X_FLAG_CALIBRATED)) {
    _channel_backup = reg_shadows[(uint8_t) MCP356xRegister::SCAN];
  }
  return _write_register(MCP356xRegister::SCAN, (uint32_t) rval);
}


/*
* Do we have data for all the channels we asked for?
*/
bool MCP356x::scanComplete() {
  uint32_t scan_chans = reg_shadows[(uint8_t) MCP356xRegister::SCAN] & 0x0000FFFF;
  return (scan_chans == (_channel_flags & scan_chans));
};

/*
* Some hardware arrangements don't use a rail-to-rail Vref. So this function
*   can be called from the application to manually define it.
*/
int8_t MCP356x::setReferenceRange(float plus, float minus) {
  _vref_plus  = plus;
  _vref_minus = minus;
  _mcp356x_set_flag(MCP356X_FLAG_VREF_DECLARED);
  return 0;
}


/*
* Runs the current temperature value through the temperature transfer function
*   to arrive at the die temperature. Uses the high-accuracy third-order fit.
*/
float MCP356x::getTemperature() {
  int32_t t_lsb = value(MCP356xChannel::TEMP);
  float ret = 0.0;
  if (_mcp356x_flag(MCP356X_FLAG_3RD_ORDER_TEMP)) {
    const double k1 = 0.0000000000000271 * (t_lsb * t_lsb * t_lsb);
    const double k2 = -0.000000018 * (t_lsb * t_lsb);
    const double k3 = 0.0055 * t_lsb;
    const double k4 = -604.22;
    ret = k1 + k2 + k3 + k4;
  }
  else {
    ret = 0.001581 * t_lsb - 324.27;
  }
  return ret;
}



/*******************************************************************************
* Hardware discovery functions
*******************************************************************************/

/*
* The maximum ADC input clock is between 1MHz and 20MHz.
*
* Returns true if MCLK (as measured) if within operational boundaries.
*/
bool MCP356x::_mclk_in_bounds() {
  return ((_mclk_freq > 1000000.0) && (_mclk_freq < 20000000.0));
}


/*
* Some designs drive the ADC from an on-board high-Q oscillator. But there is
*   no direct firmware means to discover the setting.
* This function discovers the frequency by timing ADC reads with known clocking parameters
*   and reports the result; storing the answer in the class variable _mclk_freq.
* Returns...
*   -3 if the class isn't ready for this measurement.
*   -2 if measurement timed out.
*   -1 if there was some mechanical problem communicating with the ADC
*   0  if a clock signal within expected range was determined
*   1  if we got a clock rate that was out of bounds or appears nonsensical
*/
int8_t MCP356x::_detect_adc_clock() {
  const uint32_t SAMPLE_TIME_MAX = 200000;
  const uint32_t SAMPLE_TIME_MIN = 50000;
  int8_t ret = -3;
  if (_mcp356x_flag(MCP356X_FLAG_PINS_CONFIGURED)) {
    // The timing parameters of the ADC must be known to arrive at a linear model of
    //   the interrupt rate with respect to input clock. Then, we use the model to determine
    //   clock rate by watching the IRQ rate.
    ret = -1;
    if (0 == _write_register(MCP356xRegister::SCAN, 0)) {
      if (0 == _write_register(MCP356xRegister::MUX, 0xDE)) {
        unsigned long micros_passed     = 0;
        unsigned long micros_adc_time_0 = micros();
        uint16_t rcount = 0;
        while (((1000 > rcount) | (micros_passed < SAMPLE_TIME_MIN)) && (micros_passed < SAMPLE_TIME_MAX)) {
          // We sample for at least 50ms, but no more than 200ms.
          if (isr_fired) {
            // If data is ready...
            if (0 < read()) {
              if (0 == rcount) {
                // The first time through, we reset the read count so that we don't
                //   bake the ADC startup time into our clock calculation.
                resetReadCount();
              }
              rcount++;
            }
          }
          micros_passed = micros() - micros_adc_time_0;
        }
        ret = -2;
        if (micros_passed < SAMPLE_TIME_MAX) {
          ret = 1;
          _mcp356x_set_flag(MCP356X_FLAG_MCLK_RUNNING);  // This must be reality.
          //StringBuilder temp_str;
          //temp_str.concatf("Took %u samples in %luus.\n", rcount, micros_passed);
          //Serial.print((char*) temp_str.string());
          _mclk_freq = _calculate_input_clock(micros_passed);
          if (_mclk_in_bounds()) {
            _recalculate_clk_tree();
            ret = 0;
          }
        }
      }
    }
  }
  return ret;
}


/*
* After the ADC has been running for awhile, we can calculate the true rate of
*   the input clock if we don't know it already.
* NOTE: since the sample count doesn't reset when timing parameters are altered,
*   this only gives accurate results if the settings are unchanged from init, or
*   the caller has pinged resetReadCount() before taking the measurement. See
*   the example file for usage pattern.
* Returns The calculated MCLK frequency
*/
double MCP356x::_calculate_input_clock(unsigned long elapsed_us) {
  uint32_t osr_idx = (reg_shadows[(uint8_t) MCP356xRegister::CONFIG1] & 0x0000003C) >> 2;
  uint16_t osr1 = OSR1_VALUES[osr_idx];
  uint16_t osr3 = OSR3_VALUES[osr_idx];
  uint32_t pre_val = (reg_shadows[(uint8_t) MCP356xRegister::CONFIG1] & 0x000000C0) >> 6;
  double  _drclk = ((double) read_count) / ((double) elapsed_us) * 1000000.0;
  return (4 * (osr3 * osr1) * (1 << pre_val) * _drclk);
}


/*
* Given MCLK, calculate DRCLK and store it locally.
* Returns...
*   -2 if MCLK frequency is out-of-bounds.
*   0  if the calculation completed.
*/
int8_t MCP356x::_recalculate_clk_tree() {
  if (_mclk_in_bounds()) {
    uint32_t pre_val = (reg_shadows[(uint8_t) MCP356xRegister::CONFIG1] & 0x000000C0) >> 6;
    _dmclk_freq = _mclk_freq / (4 * (1 << pre_val));
    return _recalculate_settling_time();
  }
  return -2;
}


/*
* Given our existing parameters, get the theoretical maximum settling time for
*   the data. This does not account for the circuit, obviously. It is only the
*   analytical guess based on the datasheet. It represents the minimum time
*   that the program should delay for an accurate reading.
*/
int8_t MCP356x::_recalculate_settling_time() {
  uint32_t osr_idx = (reg_shadows[(uint8_t) MCP356xRegister::CONFIG1] & 0x0000003C) >> 2;
  uint16_t osr1 = OSR1_VALUES[osr_idx];
  uint16_t osr3 = OSR3_VALUES[osr_idx];
  uint32_t dmclks = (3 * osr3) + ((osr1 - 1) * osr3);
  _settling_ms = ((uint32_t) (1000 / _dmclk_freq)) * dmclks;
  return 0;
}


/*
* Reads the ADC channels that assist us with calibration.
* Returns...
*   -1 if some register I/O failed.
*   0  if an offset calibration value was found and set.
*/
int8_t MCP356x::_calibrate_offset() {
  _channel_backup = reg_shadows[(uint8_t) MCP356xRegister::SCAN];
  int8_t ret = _set_scan_channels(0x0000E000);
  if (0 == ret) {
    _mcp356x_clear_flag(MCP356X_FLAG_CALIBRATED);
  }
  return ret;
}


int8_t MCP356x::_mark_calibrated() {
  int8_t ret = _set_scan_channels(_channel_backup);
  if (0 == ret) {
    _mcp356x_set_flag(MCP356X_FLAG_CALIBRATED);
  }
  return ret;
}


void MCP356x::printRegs(StringBuilder* output) {
  output->concatf("reg_shadows[0] (ADCDATA)     = 0x%08x\n", reg_shadows[0]);
  output->concatf("reg_shadows[1] (CONFIG0)     = 0x%02x\n", reg_shadows[1]);
  output->concatf("reg_shadows[2] (CONFIG1)     = 0x%02x\n", reg_shadows[2]);
  output->concatf("reg_shadows[3] (CONFIG2)     = 0x%02x\n", reg_shadows[3]);
  output->concatf("reg_shadows[4] (CONFIG3)     = 0x%02x\n", reg_shadows[4]);
  output->concatf("reg_shadows[5] (IRQ)         = 0x%02x\n", reg_shadows[5]);
  output->concatf("reg_shadows[6] (MUX)         = 0x%02x\n", reg_shadows[6]);
  output->concatf("reg_shadows[7] (SCAN)        = 0x%06x\n", reg_shadows[7]);
  output->concatf("reg_shadows[8] (TIMER)       = 0x%06x\n", reg_shadows[8]);
  output->concatf("reg_shadows[9] (OFFSETCAL)   = 0x%06x\n", reg_shadows[9]);
  output->concatf("reg_shadows[10] (GAINCAL)    = 0x%06x\n", reg_shadows[10]);
  output->concatf("reg_shadows[11] (RESERVED0)  = 0x%06x\n", reg_shadows[11]);
  output->concatf("reg_shadows[12] (RESERVED1)  = 0x%02x\n", reg_shadows[12]);
  output->concatf("reg_shadows[13] (LOCK)       = 0x%02x\n", reg_shadows[13]);
  output->concatf("reg_shadows[14] (RESERVED2)  = 0x%04x\n", reg_shadows[14]);
  output->concatf("reg_shadows[15] (CRCCFG)     = 0x%04x\n", reg_shadows[15]);
}


void MCP356x::printPins(StringBuilder* output) {
  output->concatf("IRQ:   %u\n", _IRQ_PIN);
  output->concatf("CS:    %u\n", _CS_PIN);
  output->concatf("MCLK:  %u\n", _MCLK_PIN);
}


void MCP356x::printTimings(StringBuilder* output) {
  output->concatf("\tMCLK                = %.4f MHz\n", _mclk_freq / 1000000.0);
  output->concatf("\tDMCLK               = %.4f MHz\n", _dmclk_freq / 1000000.0);
  output->concatf("\tData rate           = %.4f KHz\n", _drclk_freq / 1000.0);
  output->concatf("\tReal sample rate    = %u\n", reads_per_second);
  output->concatf("\tADC settling time   = %u\n", getSettlingTime());
  output->concatf("\tTotal settling time = %u\n", _circuit_settle_ms);
  output->concatf("\tLast read (millis)  = %u\n", millis_last_read);
}


void MCP356x::printData(StringBuilder* output) {
  StringBuilder prod_str("MCP356");
  if (adcFound()) {
    prod_str.concatf("%d", _channel_count() >> 1);
    if (hasInternalVref()) prod_str.concat('R');
  }
  else prod_str.concat("x (not found)");

  StringBuilder::styleHeader2(output, (const char*) prod_str.string());
  if (adcFound()) {
    output->concatf("\tChannels:       %u\n", _channel_count());
    output->concatf("\tClock running:  %c\n", (_mcp356x_flag(MCP356X_FLAG_MCLK_RUNNING) ? 'y' : 'n'));
    output->concatf("\tConfigured:     %c\n", (adcConfigured() ? 'y' : 'n'));
    output->concatf("\tCalibrated:     %c\n", (adcCalibrated() ? 'y' : 'n'));
    if (adcCalibrated()) {
      output->concat("\t");
      printChannel(MCP356xChannel::OFFSET, output);
      output->concat("\t");
      printChannel(MCP356xChannel::VCM, output);
      output->concat("\t");
      printChannel(MCP356xChannel::AVDD, output);
    }
    else {
      output->concatf("\t  SAMPLED_OFFSET: %c\n", (_mcp356x_flag(MCP356X_FLAG_SAMPLED_OFFSET) ? 'y' : 'n'));
      output->concatf("\t  SAMPLED_VCM:    %c\n", (_mcp356x_flag(MCP356X_FLAG_SAMPLED_VCM) ? 'y' : 'n'));
      output->concatf("\t  SAMPLED_AVDD:   %c\n", (_mcp356x_flag(MCP356X_FLAG_SAMPLED_AVDD) ? 'y' : 'n'));
    }
    output->concatf("\tCRC Error:      %c\n", (_mcp356x_flag(MCP356X_FLAG_CRC_ERROR) ? 'y' : 'n'));
    output->concatf("\tisr_fired:      %c\n", (isr_fired ? 'y' : 'n'));
    output->concatf("\tRead count:     %u\n", read_count);
    output->concatf("\tGain:           x%.2f\n", _gain_value());
    uint8_t _osr_idx = (uint8_t) getOversamplingRatio();
    output->concatf("\tOversampling:   x%u\n", OSR1_VALUES[_osr_idx] * OSR3_VALUES[_osr_idx]);
    output->concatf("\tVref source:    %sternal\n", (usingInternalVref() ? "In": "Ex"));
    output->concatf("\tVref declared:  %c\n", (_vref_declared() ? 'y' : 'n'));
    output->concatf("\tVref range:     %.3f / %.3f\n", _vref_minus, _vref_plus);
    output->concatf("\tClock SRC:      %sternal\n", (_mcp356x_flag(MCP356X_FLAG_USE_INTERNAL_CLK) ? "In" : "Ex"));
    if (_scan_covers_channel(MCP356xChannel::TEMP)) {
      output->concatf("\tTemperature:    %.2fC\n", getTemperature());
      output->concatf("\tThermo fitting: %s\n", (_mcp356x_flag(MCP356X_FLAG_3RD_ORDER_TEMP) ? "3rd-order" : "Linear"));
    }
  }
}


/*
* Prints a single channel.
*/
void MCP356x::printChannel(MCP356xChannel chan, StringBuilder* output) {
  output->concatf(
    "%s:\t%.6fv\t%s\n",
    CHAN_NAMES[((uint8_t) chan) & 0x0F],
    valueAsVoltage(chan),
    _channel_over_range(chan) ? "OvR" : " "
  );
}


/*
* Prints the values of all enabled channels.
*/
void MCP356x::printChannelValues(StringBuilder* output) {
  for (uint8_t i = 0; i < 16; i++) {
    MCP356xChannel chan = (MCP356xChannel) i;
    if (_scan_covers_channel(chan)) {
      switch (chan) {
        case MCP356xChannel::TEMP:
          output->concatf("Die temperature     = %.2fC\n", getTemperature());
          break;
        default:
          printChannel(MCP356xChannel::VCM, output);
          break;
      }
    }
  }
}
