use common::Constrain;
use core::cmp::{min, max};
use core::mem::replace;
use lpc;
use lpc::SYSCON;

impl Constrain<Syscon> for SYSCON {
    fn constrain(self) -> Syscon {
        Syscon {
            pconp: PCONP(()),
            clock_cfg: ClockConfig::Open(Cfgr {
                sys_clk: SysclkSrc::IrcClk,
                cpu_clk: CpuClkSrc::Sysclk,
                cclk: 12_000_000,
                pclk: 12_000_000,
                emc_clk: None,
                usb_clk: None,
                spifi_clk: None,
                pll0: None,
                power_boost: BoostSel::FastFlash,
            }),
        }
    }
}

pub struct Syscon {
    pub pconp: PCONP,
    pub clock_cfg: ClockConfig,
}

pub enum ClockConfig {
    Open(Cfgr),
    Frozen(Clocks),
}


#[derive(Debug)]
/// Possible errors raised when trying to access the SYSCON clock configuration
pub enum CfgErr {
    /// The clock configuration is not frozen
    Configurable,
    /// The clock configuration cannot be configured
    Frozen,
}

impl ClockConfig {
    /// Retrieve the configurable clock settings, if not yet frozen
    pub fn config(&mut self) -> Result<&mut Cfgr, CfgErr> {
        match self {
            ClockConfig::Open(cfgr) => Ok(cfgr),
            _ => Err(CfgErr::Frozen),
        }
    }

    /// Retrieve the frozen clock settings, if frozen
    pub fn context(&self) -> Result<&Clocks, CfgErr> {
        match self {
            ClockConfig::Frozen(ctx) => Ok(ctx),
            _ => Err(CfgErr::Configurable),
        }
    }
}

pub struct Cfgr {
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
    pub sysclk: u32,
}

impl Syscon {
    pub fn freeze(&mut self) {
        let syscon = unsafe { &(*SYSCON::ptr()) };

        // first, drop back to the internal RC at full speed and stop using the PLL anywhere
        syscon.cclksel.modify(|_, w| unsafe { w.cclkdiv().bits(1) });
        syscon.cclksel.modify(|_, w| w.cclksel().sysclk());
        syscon.usbclksel.modify(|_, w| w.usbsel().sysclk());
        syscon.spificlksel.modify(|_, w| w.spifisel().se_sysclk());
        syscon.clksrcsel.modify(|_, w| w.clksrc().internal_rc_osc());

        let clocks = {
            let cfgr = self.clock_cfg.config().unwrap();

            match cfgr.power_boost {
                BoostSel::LowPower => {
                    syscon.pboost.modify(|_, w| unsafe { w.boost().bits(0b00) });
                }
                BoostSel::FastFlash => {
                    syscon.pboost.modify(|_, w| unsafe { w.boost().bits(0b11) });
                }
            };

            // main_osc can drive PLL1 directly, or be selected as the sysclk src
            if let SysclkSrc::OscClk(f) = cfgr.sys_clk {
                if f < 15_000_000 {
                    syscon.scs.modify(|_, w| w.oscrs()._1_to_20_mhz());
                } else {
                    syscon.scs.modify(|_, w| w.oscrs()._15_to_25_mhz());
                }
                syscon.scs.modify(|_, w| w.oscen().set_bit());
                while syscon.scs.read().oscstat().bit_is_clear() {}
                syscon.clksrcsel.modify(|_, w| w.clksrc().main_osc());
            }

            let pll_clk = if let Some(f) = cfgr.pll0.as_ref() {
                // turn off the PLL while we configure it
                syscon.pll0con.modify(|_, w| w.plle().clear_bit());
                syscon.pll0feed.write(|w| unsafe { w.bits(0xAA) });
                syscon.pll0feed.write(|w| unsafe { w.bits(0x55) });

                if cfgr.sys_clk.freq() < 10_000_000 && cfgr.sys_clk.freq() > 25_000_000 {
                    panic!("pll0 input clk out of range");
                }

                // uh, very rough finder
                let pllm = f / cfgr.sys_clk.freq();
                let pll_out_clk = pllm * cfgr.sys_clk.freq();
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
                syscon
                    .pll0cfg
                    .modify(|_, w| unsafe { w.msel().bits((pllm as u8) - 1) });
                syscon.pll0feed.write(|w| unsafe { w.bits(0xAA) });
                syscon.pll0feed.write(|w| unsafe { w.bits(0x55) });

                syscon.pll0con.modify(|_, w| w.plle().set_bit());
                syscon.pll0feed.write(|w| unsafe { w.bits(0xAA) });
                syscon.pll0feed.write(|w| unsafe { w.bits(0x55) });

                while syscon.pll0stat.read().plock().bit_is_clear() {}

                Some(pll_out_clk) /* / pllp */
            } else {
                None
            };

            // TODO
            if cfgr.usb_clk.is_some() || cfgr.emc_clk.is_some() || cfgr.spifi_clk.is_some() {
                unimplemented!();
            }

            let cclk_pclk_in = match cfgr.cpu_clk {
                CpuClkSrc::Sysclk => cfgr.sys_clk.freq(),
                CpuClkSrc::PllClk => *pll_clk.as_ref().unwrap(),
            };

            match cclk_pclk_in {
                100_000_001...120_000_000 => match cfgr.power_boost {
                    BoostSel::LowPower => {
                        syscon.flashcfg.modify(|_, w| {
                            w.flashtim()
                                .variant(lpc::syscon::flashcfg::FLASHTIMW::_6_CLK)
                        });
                    }
                    BoostSel::FastFlash => {
                        syscon.flashcfg.modify(|_, w| {
                            w.flashtim()
                                .variant(lpc::syscon::flashcfg::FLASHTIMW::_5_CLK)
                        });
                    }
                },
                80_000_001...100_000_000 => syscon.flashcfg.modify(|_, w| {
                    w.flashtim()
                        .variant(lpc::syscon::flashcfg::FLASHTIMW::_5_CLK)
                }),
                60_000_001...80_000_000 => syscon.flashcfg.modify(|_, w| {
                    w.flashtim()
                        .variant(lpc::syscon::flashcfg::FLASHTIMW::_4_CLK)
                }),
                40_000_001...60_000_000 => syscon.flashcfg.modify(|_, w| {
                    w.flashtim()
                        .variant(lpc::syscon::flashcfg::FLASHTIMW::_3_CLK)
                }),
                20_000_001...40_000_000 => syscon.flashcfg.modify(|_, w| {
                    w.flashtim()
                        .variant(lpc::syscon::flashcfg::FLASHTIMW::_2_CLK)
                }),
                1...20_000_000 => syscon.flashcfg.modify(|_, w| {
                    w.flashtim()
                        .variant(lpc::syscon::flashcfg::FLASHTIMW::_1_CLK)
                }),
                _ => panic!("bad cpu clk frequency {}", cfgr.cclk),
            };

            let cclk_pre = min(1, max(cclk_pclk_in / cfgr.cclk, 4));
            syscon
                .cclksel
                .modify(|_, w| unsafe { w.cclkdiv().bits(cclk_pre as u8) });

            let pclk_pre = cclk_pclk_in / cfgr.pclk;
            syscon
                .pclksel
                .modify(|_, w| unsafe { w.pclkdiv().bits(pclk_pre as u8) });

            match cfgr.cpu_clk {
                CpuClkSrc::PllClk => syscon.cclksel.modify(|_, w| w.cclksel().main_pll()),
                _ => {}
            }

            ClockConfig::Frozen(Clocks {
                cclk: cclk_pclk_in / cclk_pre,
                pclk: cclk_pclk_in / pclk_pre,
                pll: *pll_clk.as_ref().unwrap(),
                sysclk: cfgr.sys_clk.freq(),
            })
        };

        let _ = replace(&mut self.clock_cfg, clocks);
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
        syscon.pconp.modify(|_, w| B::power_up(w));
    }

    pub fn power_down<B: PoweredBlock>(&self) {
        let syscon = unsafe { &(*SYSCON::ptr()) };
        syscon.pconp.modify(|_, w| B::power_down(w));
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
