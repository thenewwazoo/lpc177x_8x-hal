
use lpc;
use lpc::SYSCON;
use common::Constrain;

//const FCCO_MIN_FREQ: u32 = 156_000_000_u32;
//const FCCO_MAX_FREQ: u32 = 320_000_000_u32;

impl Constrain<Syscon> for SYSCON {
    fn constrain(self) -> Syscon {
        Syscon {
            pconp: PCONP(()),
            clock_cfg: ClockConfig {
                sys_clk: SysclkSrc::IrcClk,
                cpu_clk: CpuClkSrc::Sysclk,
                cclk: 12_000_000,
                pclk: 12_000_000,
                emc_clk: None,
                usb_clk: None,
                spifi_clk: None,
                pll0: None,
                power_boost: BoostSel::FastFlash,
            },
        }
    }
}

pub struct Syscon {
    pub pconp: PCONP,
    pub clock_cfg: ClockConfig,
}

pub struct ClockConfig {
    pub sys_clk: SysclkSrc,
    pub cpu_clk: CpuClkSrc,
    pub cclk: u32,
    pub pclk: u32,
    pub emc_clk: Option<u32>,
    pub usb_clk: Option<UsbClkSrc>,
    pub spifi_clk: Option<SpiFiClkSrc>,
    pub pll0: Option<u32>,
    //pub pll1: Option<u32>,
    pub power_boost: BoostSel,
}

pub struct Clocks {
    pub cclk: u32,
    pub pclk: u32,
    pub pll: u32,
}

impl ClockConfig {

    pub fn freeze(self) -> Clocks {
        let syscon = unsafe { &(*SYSCON::ptr()) };

        // first, drop back to the internal RC at full speed and stop using the PLL anywhere
        syscon.cclksel.modify(|_,w| unsafe { w.cclkdiv().bits(1) });
        syscon.cclksel.modify(|_,w| w.cclksel().sysclk());
        syscon.usbclksel.modify(|_,w| w.usbsel().sysclk());
        syscon.spificlksel.modify(|_,w| w.spifisel().se_sysclk());
        syscon.clksrcsel.modify(|_,w| w.clksrc().internal_rc_osc());

        match self.power_boost {
            BoostSel::LowPower => {
                syscon.pboost.modify(|_,w| unsafe { w.boost().bits(0b00) });
            },
            BoostSel::FastFlash => {
                syscon.pboost.modify(|_,w| unsafe { w.boost().bits(0b11) });
            },
        };

        // main_osc can drive PLL1 directly, or be selected as the sysclk src
        if let SysclkSrc::OscClk(f) = self.sys_clk {
            if f < 15_000_000 {
                syscon.scs.modify(|_,w| w.oscrs()._1_to_20_mhz());
            } else {
                syscon.scs.modify(|_,w| w.oscrs()._15_to_25_mhz());
            }
            syscon.scs.modify(|_,w| w.oscen().set_bit());
            while syscon.scs.read().oscstat().bit_is_clear() {}
            syscon.clksrcsel.modify(|_,w| w.clksrc().main_osc());
        }

        let pll_clk = if let Some(f) = self.pll0.as_ref() {
            // turn off the PLL while we configure it
            syscon.pll0con.modify(|_,w| w.plle().clear_bit());
            syscon.pll0feed.write(|w| unsafe { w.bits(0xAA) });
            syscon.pll0feed.write(|w| unsafe { w.bits(0x55) });

            if self.sys_clk.freq() < 10_000_000 && self.sys_clk.freq() > 25_000_000 {
                panic!("pll0 input clk out of range");
            }

            // uh, very rough finder
            let pllm = f / self.sys_clk.freq();
            let pll_out_clk = pllm * self.sys_clk.freq();
            /*
            let mut pllp: u8 = 1;
            for p in [1, 2, 4, 8].iter() {
                let fcco = pll_out_clk * 2 * p;
                if fcco > FCCO_MIN_FREQ && fcco < FCCO_MAX_FREQ {
                    pllp = *p;
                    break;
                }
            };
            let pllm = pllm * pllp;
            */
            if pllm < 1 || pllm > 32 {
                panic!("bad pllm value {}", pllm);
            }

            //syscon.pll0cfg.modify(|_,w| unsafe { w.msel().bits(pllm - 1).psel().bits(pllp - 1) });
            syscon.pll0cfg.modify(|_,w| unsafe { w.msel().bits((pllm as u8) - 1) });
            syscon.pll0feed.write(|w| unsafe { w.bits(0xAA) });
            syscon.pll0feed.write(|w| unsafe { w.bits(0x55) });

            syscon.pll0con.modify(|_,w| w.plle().set_bit());
            syscon.pll0feed.write(|w| unsafe { w.bits(0xAA) });
            syscon.pll0feed.write(|w| unsafe { w.bits(0x55) });

            while syscon.pll0stat.read().plock().bit_is_clear() {}

            Some(pll_out_clk) /* / pllp */
        } else { None };

        // TODO
        if self.usb_clk.is_some() || self.emc_clk.is_some() || self.spifi_clk.is_some() {
            unimplemented!();
        }

        let cclk_pclk_in = match self.cpu_clk {
            CpuClkSrc::Sysclk => self.sys_clk.freq(),
            CpuClkSrc::PllClk => *pll_clk.as_ref().unwrap(),
        };

        match cclk_pclk_in {
            100_000_001 ... 120_000_000 => match self.power_boost {
                BoostSel::LowPower => {
                    syscon.flashcfg.modify(|_,w| w.flashtim().variant(lpc::syscon::flashcfg::FLASHTIMW::_6_CLK));
                },
                BoostSel::FastFlash => {
                    syscon.flashcfg.modify(|_,w| w.flashtim().variant(lpc::syscon::flashcfg::FLASHTIMW::_5_CLK));
                },
            },
            80_000_001 ... 100_000_000 => syscon.flashcfg.modify(|_,w| w.flashtim().variant(lpc::syscon::flashcfg::FLASHTIMW::_5_CLK)),
            60_000_001 ... 80_000_000 => syscon.flashcfg.modify(|_,w| w.flashtim().variant(lpc::syscon::flashcfg::FLASHTIMW::_4_CLK)),
            40_000_001 ... 60_000_000 => syscon.flashcfg.modify(|_,w| w.flashtim().variant(lpc::syscon::flashcfg::FLASHTIMW::_3_CLK)),
            20_000_001 ... 40_000_000 => syscon.flashcfg.modify(|_,w| w.flashtim().variant(lpc::syscon::flashcfg::FLASHTIMW::_2_CLK)),
            1 ... 20_000_000 => syscon.flashcfg.modify(|_,w| w.flashtim().variant(lpc::syscon::flashcfg::FLASHTIMW::_1_CLK)),
            _ => panic!("bad cpu clk frequency {}", self.cclk),
        };

        let cclk_pre = cclk_pclk_in / self.cclk;
        syscon.cclksel.modify(|_,w| unsafe { w.cclkdiv().bits(cclk_pre as u8) });

        let pclk_pre = cclk_pclk_in / self.pclk;
        syscon.pclksel.modify(|_,w| unsafe { w.pclkdiv().bits(pclk_pre as u8) });

        match self.cpu_clk {
            CpuClkSrc::PllClk => syscon.cclksel.modify(|_,w| w.cclksel().main_pll()),
            _ => {}
        }

        Clocks {
            cclk: cclk_pclk_in / cclk_pre,
            pclk: cclk_pclk_in / pclk_pre,
            pll: *pll_clk.as_ref().unwrap(),
        }
    }

}

pub enum SysclkSrc {
    IrcClk,
    OscClk(u32),
}

impl SysclkSrc {
    fn freq(&self) -> u32 {
        match self {
            SysclkSrc::IrcClk => 12_000_000u32,
            SysclkSrc::OscClk(f) => *f,
        }
    }
}

pub enum CpuClkSrc {
    Sysclk,
    PllClk,
}

pub enum UsbClkSrc {
    Sysclk,
    PllClk,
    AltPllClk,
}

pub enum SpiFiClkSrc {
    Sysclk,
    PllClk,
    AltPllClk,
}

pub struct PCONP(());

impl PCONP {
    pub fn power_up<B: PoweredBlock>(&self) {
        let syscon = unsafe { &(*SYSCON::ptr()) };
        syscon.pconp.modify(|_,w| B::power_up(w));
    }

    pub fn power_down<B: PoweredBlock>(&self) {
        let syscon = unsafe { &(*SYSCON::ptr()) };
        syscon.pconp.modify(|_,w| B::power_down(w));
    }
}

pub enum BoostSel {
    LowPower,
    FastFlash,
}

pub trait PoweredBlock {
    fn power_up<'w>(w: &'w mut lpc::syscon::pconp::W) -> &'w mut lpc::syscon::pconp::W;
    fn power_down<'w>(w: &'w mut lpc::syscon::pconp::W) -> &'w mut lpc::syscon::pconp::W;
}
