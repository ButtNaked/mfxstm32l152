#![no_std]
#![allow(clippy::upper_case_acronyms)]

use embedded_hal::blocking::delay::DelayUs;
use embedded_hal::blocking::i2c::{Write, WriteRead};
use embedded_hal::digital::v2::OutputPin;

use core::fmt;

pub struct Ampere {
    value: u32,
    exponent: u8,
}

impl Ampere {
    pub fn new(value: u32, exponent: u8) -> Self {
        Self { value, exponent }
    }
}

impl fmt::Display for Ampere {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        let (unit, div) = if self.exponent < 3 {
            ("m", 10u32.pow(3 - self.exponent as u32))
        } else if self.exponent < 6 {
            ("u", 10u32.pow(6 - self.exponent as u32))
        } else {
            ("n", 10u32.pow(9 - self.exponent as u32))
        };
        write!(f, "{}{}A", self.value * div, unit)
    }
}

#[allow(dead_code)]
#[allow(non_camel_case_types)]
#[derive(Copy, Clone)]
pub enum RoRegister {
    // Chip ID
    ADR_ID = 0x00,

    // Firmware version
    ADR_FW_VERSION = 0x01,

    // Read error code 0 ok, 1 timeout, 2 no value
    ERROR_MSG = 0x04,

    // Read the number of shunt being used in the last idd read
    IDD_SHUNT_USED = 0x1A,

    // Value in 24 bits MSB MID LSB
    IDD_VALUE = 0x14,
}

#[allow(dead_code)]
#[allow(non_camel_case_types)]
#[derive(Copy, Clone)]
pub enum DelayUnit {
    TIME_5_MS = 0x00,
    TIME_20_MS = 0x80,
}

#[allow(dead_code)]
#[allow(non_camel_case_types)]
#[derive(Copy, Clone)]
pub enum Register {
    // System control register
    SYS_CTRL = 0x40,

    // Idd control register (R/W)
    IDD_CTRL = 0x80,

    IDD_PRE_DELAY = 0x81,
    // Shunt resistor configuration
    // Lets do one incremental write
    IDD_SHUNT0 = 0x82, // MSB 0x83
    IDD_SHUNT1 = 0x84, // MSB 0x85,
    IDD_SHUNT2 = 0x86, // MSB 0x87,
    IDD_SHUNT3 = 0x88, // MSB 0x89,
    IDD_SHUNT4 = 0x8A, // MSB 0x8B,

    // Ampli gain
    IDD_GAIN = 0x8C, // 0x8B is the LSB
    // Vdd Min value u16
    IDD_VDD_MIN = 0x8E, // 0x8F is the MSB

    // Shunt stabilization in millisecond
    IDD_SH0_STABILIZATION = 0x90,
    IDD_SH1_STABILIZATION = 0x91,
    IDD_SH2_STABILIZATION = 0x92,
    IDD_SH3_STABILIZATION = 0x93,
    IDD_SH4_STABILIZATION = 0x94,

    IDD_NBR_OF_MEAS = 0x96,

    // Delay between each measurment
    IDD_MEAS_DELTA_DELAY = 0x97,

    // Shunt on board
    IDD_SHUNTS_ON_BOARD = 0x98,
}

#[allow(dead_code)]
#[allow(non_camel_case_types)]
#[derive(Copy, Clone)]
pub enum NbShunt {
    SHUNT_NB_1 = 0x01,
    SHUNT_NB_2 = 0x02,
    SHUNT_NB_3 = 0x03,
    SHUNT_NB_4 = 0x04,
    SHUNT_NB_5 = 0x05,
}

#[allow(dead_code)]
#[allow(non_camel_case_types)]
#[derive(Copy, Clone)]
enum SysCtrl {
    SWRST = 0x80,
    STANDBY = 0x40,
    ALTERNATE_GPIO_EN = 0x08, //* by the way if IDD and TS are enabled they take automatically the AF pins*/
    IDD_EN = 0x04,
    TS_EN = 0x02,
    GPIO_EN = 0x01,
}

pub struct MFX<I2C, GPIO, Delay> {
    i2c: I2C,
    wakup: GPIO,
    delay: Delay,
    address: u8,
}

impl<I2C, GPIO, Delay, E> MFX<I2C, GPIO, Delay>
where
    I2C: WriteRead<Error = E> + Write<Error = E>,
    GPIO: OutputPin,
    Delay: DelayUs<u32>,
{
    pub fn new(i2c: I2C, wakup: GPIO, delay: Delay, address: u8) -> Result<Self, E> {
        let mut mfx = Self {
            i2c,
            wakup,
            delay,
            address,
        };
        mfx.wakup()?;
        Ok(mfx)
    }

    fn wakup(&mut self) -> Result<(), E> {
        let _ = self.wakup.set_high();
        self.delay.delay_us(1000);
        let _ = self.wakup.set_low();
        let mut mode = self.read_u8(Register::SYS_CTRL as u8)?;
        mode |= SysCtrl::IDD_EN as u8;
        self.write_u8(Register::SYS_CTRL as u8, mode)?;
        Ok(())
    }

    pub fn set_idd_ctrl(
        &mut self,
        calibration_disabled: bool,
        vref_disabled: bool,
        nb_shunt: NbShunt,
    ) -> Result<(), E> {
        let cal = if calibration_disabled { 0x80 } else { 0x00 };

        let vref = if vref_disabled { 0x40 } else { 0x00 };
        let nb_shunt = nb_shunt as u8;
        let value = (nb_shunt << 1) | cal | vref;
        self.write_u8(Register::IDD_CTRL as u8, value)?;
        self.write_u8(Register::IDD_SHUNTS_ON_BOARD as u8, nb_shunt)
    }

    pub fn set_idd_nb_measurment(&mut self, nb: u8) -> Result<(), E> {
        self.write_u8(Register::IDD_NBR_OF_MEAS as u8, nb)
    }

    pub fn set_idd_shunt0(&mut self, data: u16, stab_delay: u8) -> Result<(), E> {
        self.set_idd_shunt(
            Register::IDD_SHUNT0,
            data,
            Register::IDD_SH0_STABILIZATION,
            stab_delay,
        )
    }

    pub fn set_idd_shunt1(&mut self, data: u16, stab_delay: u8) -> Result<(), E> {
        self.set_idd_shunt(
            Register::IDD_SHUNT1,
            data,
            Register::IDD_SH1_STABILIZATION,
            stab_delay,
        )
    }

    pub fn set_idd_shunt2(&mut self, data: u16, stab_delay: u8) -> Result<(), E> {
        self.set_idd_shunt(
            Register::IDD_SHUNT2,
            data,
            Register::IDD_SH2_STABILIZATION,
            stab_delay,
        )
    }

    pub fn set_idd_shunt3(&mut self, data: u16, stab_delay: u8) -> Result<(), E> {
        self.set_idd_shunt(
            Register::IDD_SHUNT3,
            data,
            Register::IDD_SH3_STABILIZATION,
            stab_delay,
        )
    }

    pub fn set_idd_shunt4(&mut self, data: u16, stab_delay: u8) -> Result<(), E> {
        self.set_idd_shunt(
            Register::IDD_SHUNT4,
            data,
            Register::IDD_SH4_STABILIZATION,
            stab_delay,
        )
    }

    fn set_idd_shunt(
        &mut self,
        reg_shunt: Register,
        data: u16,
        reg_delay: Register,
        stab_delay: u8,
    ) -> Result<(), E> {
        self.noinc_write_be_u16(reg_shunt as u8, data)?;
        self.write_u8(reg_delay as u8, stab_delay)
    }

    pub fn set_idd_gain(&mut self, value: u16) -> Result<(), E> {
        self.noinc_write_be_u16(Register::IDD_GAIN as u8, value)
    }

    pub fn set_idd_pre_delay(&mut self, unit: DelayUnit, value: u8) -> Result<(), E> {
        let value = self.cap_delay_value(value);
        let unit = unit as u8;
        self.write_u8(Register::IDD_PRE_DELAY as u8, unit & value)
    }

    pub fn set_idd_meas_delta_delay(&mut self, unit: DelayUnit, value: u8) -> Result<(), E> {
        let value = self.cap_delay_value(value);
        let unit = unit as u8;
        self.write_u8(Register::IDD_MEAS_DELTA_DELAY as u8, unit & value)
    }

    pub fn set_idd_vdd_min(&mut self, value: u16) -> Result<(), E> {
        self.noinc_write_be_u16(Register::IDD_VDD_MIN as u8, value)
    }

    pub fn idd_start(&mut self) -> Result<(), E> {
        let mut mode = self.read_u8(Register::IDD_CTRL as u8)?;
        mode |= 1;
        self.write_u8(Register::IDD_CTRL as u8, mode)
    }

    pub fn idd_get_value(&mut self) -> Result<Ampere, E> {
        // TODO: Fix delay, maybe use IT.
        self.delay.delay_us(500_000);
        self.delay.delay_us(500_000);
        self.delay.delay_us(500_000);
        self.delay.delay_us(500_000);
        // read_be_u24
        //let value = self.i2c.read_be_u24(self.address, RoRegister::IDD_VALUE)?;
        let v = self.read_be_u24(RoRegister::IDD_VALUE as u8)?;
        Ok(Ampere::new(v, 8))
    }

    pub fn idd_ctrl(&mut self) -> Result<u8, E> {
        self.read_u8(Register::IDD_CTRL as u8)
    }

    pub fn idd_last_shunt_used(&mut self) -> Result<u8, E> {
        //self.i2c.read_u8(self.address, RoRegister::IDD_SHUNT_USED)
        self.read_u8(RoRegister::IDD_SHUNT_USED as u8)
    }

    pub fn idd_shunts_on_board(&mut self) -> Result<u8, E> {
        self.read_u8(Register::IDD_SHUNTS_ON_BOARD as u8)
    }

    pub fn error_code(&mut self) -> Result<u8, E> {
        //self.i2c.read_u8(self.address, RoRegister::ERROR_MSG)
        let mut buf = [0; 1];
        self.i2c
            .write_read(self.address, &[RoRegister::ERROR_MSG as u8], &mut buf)?;
        Ok(buf[0])
    }

    pub fn chip_id(&mut self) -> Result<u8, E> {
        //self.i2c.read_u8(self.address, RoRegister::ADR_ID)
        self.read_u8(RoRegister::ADR_ID as u8)
    }

    pub fn firmware_version(&mut self) -> Result<u16, E> {
        //self.i2c.read_be_u16(self.address, RoRegister::ADR_FW_VERSION)
        self.read_be_u16(RoRegister::ADR_FW_VERSION as u8)
    }

    fn cap_delay_value(&mut self, value: u8) -> u8 {
        if value > 0x80 {
            0x7F
        } else {
            value
        }
    }

    fn write_u8(&mut self, reg: u8, v: u8) -> Result<(), E> {
        self.i2c.write(self.address, &[reg, v])
    }

    fn read_u8(&mut self, reg: u8) -> Result<u8, E> {
        let mut buf = [0; 1];
        self.i2c.write_read(self.address, &[reg], &mut buf)?;
        Ok(buf[0])
    }

    fn read_be_u24(&mut self, reg: u8) -> Result<u32, E> {
        let mut buf = [0; 3];
        self.i2c.write_read(self.address, &[reg as u8], &mut buf)?;
        Ok(u32::from_be_bytes([0, buf[0], buf[1], buf[2]]))
    }

    fn read_be_u16(&mut self, reg: u8) -> Result<u16, E> {
        let mut buf = [0; 2];
        self.i2c.write_read(self.address, &[reg], &mut buf)?;
        Ok(u16::from_be_bytes(buf))
    }

    fn noinc_write_be_u16(&mut self, reg: u8, v: u16) -> Result<(), E> {
        let buffer: [u8; 2] = v.to_be_bytes();
        self.i2c.write(self.address, &[reg, buffer[0]])?;
        self.i2c.write(self.address, &[reg + 1, buffer[1]])
    }
}
