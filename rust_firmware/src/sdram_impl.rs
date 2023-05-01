use stm32_fmc::FmcPeripheral;
use stm32_fmc::{
    AddressPinSet, Nand, NandChip, PinsNand, PinsSdram, Sdram, SdramChip,
    SdramPinSet, SdramTargetBank,
};

use stm32f4xx_hal::time::Hertz;

use stm32f4xx_hal::gpio::{self, Alternate};

use stm32_fmc::*;

struct stm32_f446_fmc;

unsafe impl FmcPeripheral for stm32_f446_fmc {
    const REGISTERS: *const () = 0 as *const ();
    fn enable(&mut self) {}
    fn source_clock_hz(&self) -> u32 {
        168_000_000
    }
}
