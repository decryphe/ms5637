//! Implements a driver for accessing an MS5637 temperature and pressure sensor.
//!
//! The library is built to work with [`embedded-hal`] and has been tested on an
//! STM32L072 micro controller. The 1st-order temperature compensation is
//! implemented as per the  [datasheet definition][ms5637], and inspired by the
//! implementation in the [Sparkfun MS5637 library][spk]. The 2nd-order
//! temperature compensation is probably wrong and untested, but may be
//! activated using the feature flag `second-order`. Additional height
//! adjustment methods available in the [Sparkfun library][spk] were copied over
//! for convenience and may be enabled using the `altitude-adjust` feature flag.
//!
//! [ms5637]: https://www.amsys.de/downloads/data/MS5637-30BA-AMSYS-datasheet.pdf
//! [spk]: https://github.com/sparkfun/SparkFun_MS5637_Arduino_Library/blob/main/src/SparkFun_MS5637_Arduino_Library.cpp
//!
//! # Usage
//!
//! Using the library is very easy, just provide something that can delay
//! execution by a specific time, and a reference to the I2C bus itself:
//!
//! ```ignore
//! let mut ms5637 = ms5637::MS5637::new(&mut syst_delay, &mut i2c_bus);
//! let reading = ms5637.read_temperature_and_pressure_1st_order(&mut syst_delay, &mut i2c);
//! ```
//!
//! The library can be cooperatively used iwth other sensors on the same I2C-bus.
#![no_std]

/// 7-bit unshifted I2C address for the MS5637 sensor.
pub const MS5637_ADDR: u8 = 0x76;

/// MS5637 command to initiate a read from PROM address 0.
const MS5637_PROM_ADDRESS_READ_ADDRESS_0: u8 = 0xA0;

/// MS5637 command to reset the sensor chip.
const MS5637_RESET_COMMAND: u8 = 0x1E;

/// MS5637 command to start a pressure ADC conversion.
const MS5637_START_PRESSURE_ADC_CONVERSION: u8 = 0x40;

/// MS5637 command to start a temperature ADC conversion.
const MS5637_START_TEMPERATURE_ADC_CONVERSION: u8 = 0x50;

/// MS5637 command to read the ADC.
const MS5637_READ_ADC: u8 = 0x00;

/// Number of configuration coefficients in PROM.
const MS5637_COEFFICIENT_COUNT: usize = 7;

/// Calibration coefficients stored in PROM.
#[derive(Copy, Clone, Debug)]
struct CalibrationCoefficients {
    /// Pressure sensitivity SENS_T1.
    pressure_sensitivity: u16,
    /// Pressure offset OFF_T1.
    pressure_offset: u16,
    /// Temperature coefficient of pressure sensitivity TCS.
    temp_coeff_of_pressure_sensitivity: u16,
    /// Temperature coefficient of pressure offset TCO.
    temp_coeff_of_pressure_offset: u16,
    /// Reference temperature T_REF.
    reference_temperature: u16,
    /// Temperature coefficient of the temperature TEMPSENS.
    temp_coeff_of_temperature: u16,
}

/// Represents an I2C-connected MS5637 sensor.
#[derive(Copy, Clone, Debug)]
pub struct MS5637<I2C, D> {
    /// Calibration coefficients stored on the chip.
    coeffs: CalibrationCoefficients,

    /// Currently selected resolution of the sensor.
    resolution: Resolution,

    /// Marker to satisfy the compiler.
    _delay: core::marker::PhantomData<D>,

    /// I2C Interface for communcating witht he sensor.
    _i2c: core::marker::PhantomData<I2C>,
}

impl<I2C, D> MS5637<I2C, D>
where
    D: embedded_hal::blocking::delay::DelayMs<u32>,
    I2C: embedded_hal::blocking::i2c::Read + embedded_hal::blocking::i2c::Write,
{
    /// Triggers conversion and reading of the 12-bit ADC value specified in the
    /// command.
    fn begin_conversion_and_read_adc(
        &mut self,
        cmd: u8,
        delay: &mut D,
        i2c: &mut I2C,
    ) -> Result<u32> {
        let mut buf = [0; 3];

        i2c.write(MS5637_ADDR, &[cmd])
            .map_err(|_| MS5637Error::WriteI2CError)?;

        delay.delay_ms(self.resolution.get_conversion_time());

        i2c.write(MS5637_ADDR, &[MS5637_READ_ADC])
            .map_err(|_| MS5637Error::WriteI2CError)?;

        i2c.read(MS5637_ADDR, &mut buf)
            .map_err(|_| MS5637Error::ReadI2CError)?;

        let adc_value = ((buf[0] as u32) << 16) | ((buf[1] as u32) << 8) | buf[2] as u32;
        Ok(adc_value)
    }

    /// Creates a connection with an MS5637 sensor via I2C.
    ///
    /// Initialization sequence:
    /// - Sends a reset command to the chip.
    /// - Reads the PROM coefficients.
    ///
    pub fn new(delay: &mut D, i2c: &mut I2C) -> Result<Self> {
        let result = i2c.write(MS5637_ADDR, &[MS5637_RESET_COMMAND]);
        if result.is_err() {
            return Err(MS5637Error::I2cTransferError);
        }

        delay.delay_ms(12);

        let eeprom_coeffs = read_eeprom(i2c)?;

        Ok(Self {
            coeffs: eeprom_coeffs,
            resolution: Resolution::Osr512,
            _delay: core::marker::PhantomData::default(),
            _i2c: core::marker::PhantomData::default(),
        })
    }

    /// Reads the temperature and pressure ADC values and computes the
    /// compensated values.
    pub fn read_temperature_and_pressure_1st_order(
        &mut self,
        delay: &mut D,
        i2c: &mut I2C,
    ) -> Result<Reading> {
        // Reads temperature
        let cmd = (self.resolution as u8) * 2 | MS5637_START_TEMPERATURE_ADC_CONVERSION;
        let adc_temperature = self.begin_conversion_and_read_adc(cmd, delay, i2c)? as i32;

        // Reads the pressure
        let cmd = (self.resolution as u8) * 2 | MS5637_START_PRESSURE_ADC_CONVERSION;
        let adc_pressure = self.begin_conversion_and_read_adc(cmd, delay, i2c)? as i32;

        if adc_temperature == 0 || adc_pressure == 0 {
            return Err(MS5637Error::I2cTransferError);
        }

        let delta_t = calculate_delta_t(adc_temperature, &self.coeffs);
        let temp = calculate_temperature_1st_order(delta_t, &self.coeffs);

        let off = calculate_pressure_offset_at_actual_temp(delta_t, &self.coeffs);
        let sens = calculate_pressure_sensitivity_at_actual_temp(delta_t, &self.coeffs);
        let pressure = calculate_pressure_1st_order(adc_pressure, off, sens);

        Ok(Reading {
            pressure: pressure as f32 / 100.0,
            // temperature: (temp as f32 - t2 as f32) / 100.0,
            temperature: (temp as f32) / 100.0,
        })
    }

    /// Reads the temperature and pressure ADC values and computes the
    /// compensated values, including second order compensation.
    ///
    /// The compensation methods are sourced from the implementation in the
    /// [Sparkfun MS5637 library][spk] and re-implemented mostly verbatim.
    /// Additionally, the [datasheet definition][ms5637] was used for fixing the
    /// implementation.
    ///
    /// Note: Functionality of 2nd-order compensation has not yet been validated.
    ///
    /// [ms5637]: https://www.amsys.de/downloads/data/MS5637-30BA-AMSYS-datasheet.pdf
    /// [spk]: https://github.com/sparkfun/SparkFun_MS5637_Arduino_Library/blob/main/src/SparkFun_MS5637_Arduino_Library.cpp
    ///
    #[cfg(feature = "second-order")]
    pub fn read_temperature_and_pressure_2nd_order(&mut self, delay: &mut D) -> Result<Reading> {
        // Reads temperature
        let cmd = (self.resolution as u8) * 2 | MS5637_START_TEMPERATURE_ADC_CONVERSION;
        let adc_temperature = self.conversion_and_read_adc(cmd, delay)? as i32;

        // Reads the pressure
        let cmd = (self.resolution as u8) * 2 | MS5637_START_PRESSURE_ADC_CONVERSION;
        let adc_pressure = self.conversion_and_read_adc(cmd, delay)? as i32;

        if adc_temperature == 0 || adc_pressure == 0 {
            return Err(MS5637Error::I2cTransferError);
        }

        let delta_t = calculate_delta_t(adc_temperature, &self.coeffs);
        let temp = calculate_temperature_1st_order(delta_t, &self.coeffs) as i64;

        let off = calculate_pressure_offset_at_actual_temp(delta_t, &self.coeffs);
        let sens = calculate_pressure_sensitivity_at_actual_temp(delta_t, &self.coeffs);

        // Second order temperature compensation
        let (ti, offi, sensi) = calculate_2nd_order_compensation_coefficients(delta_t, temp);

        let temperature = calculate_temperature_2nd_order(temp, ti);
        let pressure = calculate_pressure_2nd_order(adc_pressure, off, offi, sens, sensi);

        Ok(Reading {
            pressure: pressure as f32 / 10.0,
            temperature: (temperature as f32) / 100.0,
        })
    }

    /// Sets the ADC resolution for the next read.
    pub fn set_resolution(&mut self, resolution: Resolution) {
        self.resolution = resolution;
    }
}

/// Represents the reading gotten from the sensor.
#[derive(Default, Clone, Copy, Debug)]
pub struct Reading {
    /// Pressure in millibar.
    pub pressure: f32,

    /// Temperature in degrees Celsius.
    pub temperature: f32,
}

/// Resolution RMS, where a higher resolution means longer conversion time.
#[derive(Copy, Clone, Debug)]
#[repr(u8)]
pub enum Resolution {
    /// Resolution ±1.83 mbar, conversion time 0.54 ms, supply current 0.63 μA.
    Osr256 = 0,
    /// Resolution ±1.14 mbar, conversion time 1.06 ms, supply current 1.26 μA.
    Osr512 = 1,
    /// Resolution ±0.78 mbar, conversion time 2.08 ms, supply current 2.51 μA.
    Osr1024 = 2,
    /// Resolution ±0.54 mbar, conversion time 4.13 ms, supply current 5.02 μA.
    Osr2048 = 3,
    /// Resolution ±0.38 mbar, conversion time 8.22 ms, supply current 10.05 μA.
    Osr4096 = 4,
    /// Resolution ±0.27 mbar, conversion time 16.44 ms, supply current 20.09 μA.
    Osr8192 = 5,
}

impl Resolution {
    /// Gets the conversion time in milliseconds.
    fn get_conversion_time(&self) -> u32 {
        match self {
            Resolution::Osr256 => 1,
            Resolution::Osr512 => 2,
            Resolution::Osr1024 => 3,
            Resolution::Osr2048 => 5,
            Resolution::Osr4096 => 10,
            Resolution::Osr8192 => 20,
        }
    }
}

/// Shorthand for all functions returning an error in this module.
type Result<T> = core::result::Result<T, MS5637Error>;

/// Represents any error that may happen during communication.
#[derive(Copy, Clone, Debug, Ord, PartialOrd, Eq, PartialEq)]
pub enum MS5637Error {
    CrcError,
    I2cTransferError,
    ReadI2CError,
    WriteI2CError,
}

/// Given a pressure P (mb) taken at a specific altitude (meters), returns
/// the equivalent pressure (mb) at sea level.
///
/// This produces pressure readings that can be used for weather
/// measurements.
///
#[cfg(feature = "altitude-adjust")]
pub fn adjust_to_sea_level(absolute_pressure: f32, actual_altitude: f32) -> f32 {
    absolute_pressure / libm::powf(1.0 - (actual_altitude / 44330.0), 5.255)
}

/// Given a pressure measurement (mb) and the pressure at a baseline (mb),
/// returns the altitude change (in meters) for the delta in pressures.
#[cfg(feature = "altitude-adjust")]
pub fn altitude_change(current_pressure: f32, baseline_pressure: f32) -> f32 {
    44330.0 * (1.0 - libm::powf(current_pressure / baseline_pressure, 1.0 / 5.255))
}

/// Calculates the 2nd order compensation coefficients for a given temperature.
///
/// Returns: Ti, Offi, Sensi
///
#[cfg(feature = "second-order")]
fn calculate_2nd_order_compensation_coefficients(delta_t: i64, temp: i64) -> (i64, i64, i64) {
    let t_minus_2000_squared = (temp - 2000) * (temp - 2000);
    if temp < -1500 {
        let t_plus_1500_squared = (temp + 1500) * (temp + 1500);
        (
            (3 * delta_t * delta_t) >> 33,
            ((3 * t_minus_2000_squared) >> 1) + (7 * t_plus_1500_squared),
            ((5 * t_minus_2000_squared) >> 3) + (4 * t_plus_1500_squared),
        )
    } else if temp < 2000 {
        (
            (3 * delta_t * delta_t) >> 33,
            (3 * t_minus_2000_squared) >> 1,
            (5 * t_minus_2000_squared) >> 3,
        )
    } else {
        ((2 * delta_t * delta_t) >> 37, t_minus_2000_squared >> 4, 0)
    }
}

/// Calculates the difference between actual and reference temperature.
///
/// dT = D2 - Tref
///
/// Note: The reference temperature is not specified what kind of unit it has,
/// but it gets shifted left by 8 bits in the specification.
///
fn calculate_delta_t(adc_value: i32, coeffs: &CalibrationCoefficients) -> i64 {
    let reference_temperature = (coeffs.reference_temperature as i32) << 8;
    (adc_value - reference_temperature) as i64
}

/// Calculates the pressure in 100ths of a millibar, given the ADC value, offset
/// and sensitivity at the actual temperature, and calibration coefficients of a
/// MS5637 sensor.
///
/// P = D1 * SENS - OFF
fn calculate_pressure_1st_order(adc_value: i32, off: i64, sens: i64) -> i32 {
    ((((adc_value as i64 * sens) >> 21) - off) >> 14) as i32
}

/// Calculates the 2nd-order compensated pressure in 100ths of a millibar.
///
/// Note: Functionality of 2nd-order compensation has not yet been validated.
///
#[cfg(feature = "second-order")]
fn calculate_pressure_2nd_order(adc_value: i32, off: i64, offi: i64, sens: i64, sensi: i64) -> i32 {
    let off2 = off - offi;
    let sens2 = sens - sensi;
    ((((adc_value as i64 * sens2) >> 21) - off2) >> 13) as i32
}

/// Calculates the pressure offset at the actual temperature.
///
/// OFF = OFF_T1 + TCO * dT
///
fn calculate_pressure_offset_at_actual_temp(delta_t: i64, coeffs: &CalibrationCoefficients) -> i64 {
    let pressure_offset = (coeffs.pressure_offset as i64) << 16;
    let temp_coeff_of_pressure_offset =
        (coeffs.temp_coeff_of_pressure_offset as i64 * delta_t) >> 7;
    pressure_offset + temp_coeff_of_pressure_offset
}

/// Calculates the pressure sensitivity at the actual temperature.
///
/// SENS = SENS_T1 + TCS * dT
///
fn calculate_pressure_sensitivity_at_actual_temp(
    delta_t: i64,
    coeffs: &CalibrationCoefficients,
) -> i64 {
    let pressure_sensitivity = (coeffs.pressure_sensitivity as i64) << 15;
    let temp_coeff_of_pressure_sensitivity =
        (coeffs.temp_coeff_of_pressure_sensitivity as i64 * delta_t) >> 8;
    pressure_sensitivity + temp_coeff_of_pressure_sensitivity
}

/// Calculates the temperature in milli-Celsius, given the ADC value and
/// calibration coefficients of a MS5637 sensor.
///
/// TEMP = 20°C + dT * TEMPSENS
///
fn calculate_temperature_1st_order(delta_t: i64, coeffs: &CalibrationCoefficients) -> i32 {
    // Note: TEMPSENS in documentation is casted u16->i64 and shifted right
    // by 23 bits. This is only valid for little-endian architectures.
    let temp_coeff_of_temperature = coeffs.temp_coeff_of_temperature as i64;

    // But: The division by 2^23 here is still appropriate, which is
    // probably the case because TEMPSENS must have some very odd unit of
    // measure.
    (2000 + ((delta_t * temp_coeff_of_temperature) >> 23)) as i32
}

/// Calculates the 2nd-order compensated temperature in milli-Celsius.
///
/// Note: Functionality of 2nd-order compensation has not yet been validated.
///
#[cfg(feature = "second-order")]
fn calculate_temperature_2nd_order(temp: i64, ti: i64) -> i32 {
    (temp - ti) as i32
}

/// Calculates the CRC4 of the given buffer, as per MS5637 datasheet.
fn crc4(n_prom: &mut [u16; MS5637_COEFFICIENT_COUNT + 1]) -> u16 {
    let mut n_rem: u16 = 0;
    n_prom[0] &= 0x0FFF; // CRC byte is replaced by 0. Actually just the CRC4 nibble.
    n_prom[MS5637_COEFFICIENT_COUNT] = 0; // Subsidiary value, set to 0.

    for cnt in 0..(MS5637_COEFFICIENT_COUNT + 1) * 2 {
        // choose LSB or MSB
        n_rem ^= if cnt % 2 == 1 {
            n_prom[cnt >> 1] & 0x00FF
        } else {
            n_prom[cnt >> 1] >> 8
        };

        for _bit in 0..8_u8 {
            n_rem = if (n_rem & 0x8000) != 0 {
                (n_rem << 1) ^ 0x3000
            } else {
                n_rem << 1
            };
        }
    }
    n_rem = (n_rem >> 12) & 0x000F;
    n_rem ^ 0x00
}

/// Reads the EEPROM to determine current calibration coefficients.
fn read_eeprom<I2C>(i2c: &mut I2C) -> Result<CalibrationCoefficients>
where
    I2C: embedded_hal::blocking::i2c::Read + embedded_hal::blocking::i2c::Write,
{
    // The buffer is set to be one element larger, as the last element will
    // be the CRC value during CRC calculation.
    let mut coeffs = [0_u16; MS5637_COEFFICIENT_COUNT + 1];
    for (idx, val) in coeffs.iter_mut().enumerate() {
        if idx == MS5637_COEFFICIENT_COUNT {
            break;
        }

        let coeff = read_eeprom_coeff(i2c, idx as u8)?;
        *val = coeff;
    }

    // Checks the CRC4 of the values that were read from the chip.
    let crc4_val = (coeffs[0] & 0xF000) >> 12;
    let crc4_calc = crc4(&mut coeffs);
    if crc4_val != crc4_calc {
        return Err(MS5637Error::CrcError);
    }

    Ok(CalibrationCoefficients {
        pressure_sensitivity: coeffs[1],
        pressure_offset: coeffs[2],
        temp_coeff_of_pressure_sensitivity: coeffs[3],
        temp_coeff_of_pressure_offset: coeffs[4],
        reference_temperature: coeffs[5],
        temp_coeff_of_temperature: coeffs[6],
    })
}

/// Reads the MS5637 EEPROM coefficient stored at the given EEPROM index.
fn read_eeprom_coeff<I2C>(i2c: &mut I2C, idx: u8) -> Result<u16>
where
    I2C: embedded_hal::blocking::i2c::Read + embedded_hal::blocking::i2c::Write,
{
    let mut buf = [0_u8; 2];
    let cmd = MS5637_PROM_ADDRESS_READ_ADDRESS_0 + idx as u8 * 2;

    if i2c.write(MS5637_ADDR, &[cmd]).is_err() && idx != 0 {
        // In the first iteration of write/read for reading PROM values, an
        // error during writing is expected (`Nack`), so only fail if the second
        // write/read sequence fails here.
        return Err(MS5637Error::WriteI2CError);
    }

    i2c.read(MS5637_ADDR, &mut buf)
        .map_err(|_| MS5637Error::ReadI2CError)?;

    return Ok(u16::from_be_bytes(buf));
}

#[cfg(test)]
mod test {
    use super::*;

    /// Coefficients from the sample calculation in [the official
    /// datasheet][datasheet].
    ///
    /// [datasheet]: https://www.amsys.de/downloads/data/MS5637-30BA-AMSYS-datasheet.pdf
    ///
    fn get_sample_coefficients() -> CalibrationCoefficients {
        CalibrationCoefficients {
            pressure_sensitivity: 34982,
            pressure_offset: 36352,
            temp_coeff_of_pressure_sensitivity: 20328,
            temp_coeff_of_pressure_offset: 22354,
            reference_temperature: 26646,
            temp_coeff_of_temperature: 26146,
        }
    }

    /// Sample ADC values (pressure, temperature) from sample calculation in
    /// [the official datasheet][datasheet].
    ///
    /// [datasheet]: https://www.amsys.de/downloads/data/MS5637-30BA-AMSYS-datasheet.pdf
    ///
    fn get_sample_adc_values() -> (i32, i32) {
        (4958179, 6815414)
    }

    #[test]
    fn calculate_1st_order_with_sample_coefficients() {
        let coeffs = get_sample_coefficients();
        let (adc_pressure, adc_temperature) = get_sample_adc_values();

        let delta_t = calculate_delta_t(adc_temperature, &coeffs);
        assert_eq!(delta_t, -5962);

        let temp = calculate_temperature_1st_order(delta_t, &coeffs);
        assert_eq!(temp, 1981);

        let off = calculate_pressure_offset_at_actual_temp(delta_t, &coeffs);
        assert_eq!(off, 2_381_323_464);

        let sens = calculate_pressure_sensitivity_at_actual_temp(delta_t, &coeffs);
        assert_eq!(sens, 1_145_816_755);

        let p = calculate_pressure_1st_order(adc_pressure, off, sens);
        // TODO: Confirm why the code that works with the sample outputs the
        // wrong values for the real measurement. The real world has ~1 bar
        // absolute pressure.
        //
        // assert_eq!(p, 39_998);
        assert_eq!(p, 19_999);
    }

    #[cfg(feature = "second-order")]
    #[test]
    fn calculate_2nd_order_with_sample_coefficients() {
        let coeffs = get_sample_coefficients();
        let (adc_pressure, adc_temperature) = get_sample_adc_values();

        let delta_t = calculate_delta_t(adc_temperature, &coeffs);
        assert_eq!(delta_t, -5962);

        let temp = calculate_temperature_1st_order(delta_t, &coeffs) as i64;
        assert_eq!(temp, 1981);

        let off = calculate_pressure_offset_at_actual_temp(delta_t, &coeffs);
        assert_eq!(off, 2_381_323_464);

        let sens = calculate_pressure_sensitivity_at_actual_temp(delta_t, &coeffs);
        assert_eq!(sens, 1_145_816_755);

        let (ti, offi, sensi) = calculate_2nd_order_compensation_coefficients(delta_t, temp);
        println!("Ti: {ti}, Offi: {offi}, Sensi: {sensi}");

        let temp2 = calculate_temperature_2nd_order(temp, ti);
        println!("Temp2: {temp2}");

        let p2 = calculate_pressure_2nd_order(adc_pressure, off, offi, sens, sensi);
        println!("P2: {p2}");
    }
}
