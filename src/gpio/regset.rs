use lpc::{gpio, GPIO};

pub trait GpioRegSet {
    fn dir(&mut self) -> &gpio::DIR;
    fn clr(&mut self) -> &gpio::CLR;
    fn mask(&mut self) -> &gpio::MASK;
    fn pin(&mut self) -> &gpio::PIN;
    fn set(&mut self) -> &gpio::SET;
}

pub struct Zero(pub(crate) ());
impl GpioRegSet for Zero {
    fn dir(&mut self) -> &gpio::DIR {
        unsafe { &(*GPIO::ptr()).dir0 }
    }

    fn clr(&mut self) -> &gpio::CLR {
        unsafe { &(*GPIO::ptr()).clr0 }
    }

    fn mask(&mut self) -> &gpio::MASK {
        unsafe { &(*GPIO::ptr()).mask0 }
    }

    fn pin(&mut self) -> &gpio::PIN {
        unsafe { &(*GPIO::ptr()).pin0 }
    }

    fn set(&mut self) -> &gpio::SET {
        unsafe { &(*GPIO::ptr()).set0 }
    }
}

pub struct One(pub(crate) ());
impl GpioRegSet for One {
    fn dir(&mut self) -> &gpio::DIR {
        unsafe { &(*GPIO::ptr()).dir1 }
    }

    fn clr(&mut self) -> &gpio::CLR {
        unsafe { &(*GPIO::ptr()).clr1 }
    }

    fn mask(&mut self) -> &gpio::MASK {
        unsafe { &(*GPIO::ptr()).mask1 }
    }

    fn pin(&mut self) -> &gpio::PIN {
        unsafe { &(*GPIO::ptr()).pin1 }
    }

    fn set(&mut self) -> &gpio::SET {
        unsafe { &(*GPIO::ptr()).set1 }
    }
}

pub struct Two(pub(crate) ());
impl GpioRegSet for Two {
    fn dir(&mut self) -> &gpio::DIR {
        unsafe { &(*GPIO::ptr()).dir2 }
    }

    fn clr(&mut self) -> &gpio::CLR {
        unsafe { &(*GPIO::ptr()).clr2 }
    }

    fn mask(&mut self) -> &gpio::MASK {
        unsafe { &(*GPIO::ptr()).mask2 }
    }

    fn pin(&mut self) -> &gpio::PIN {
        unsafe { &(*GPIO::ptr()).pin2 }
    }

    fn set(&mut self) -> &gpio::SET {
        unsafe { &(*GPIO::ptr()).set2 }
    }
}

pub struct Three(pub(crate) ());
impl GpioRegSet for Three {
    fn dir(&mut self) -> &gpio::DIR {
        unsafe { &(*GPIO::ptr()).dir3 }
    }

    fn clr(&mut self) -> &gpio::CLR {
        unsafe { &(*GPIO::ptr()).clr3 }
    }

    fn mask(&mut self) -> &gpio::MASK {
        unsafe { &(*GPIO::ptr()).mask3 }
    }

    fn pin(&mut self) -> &gpio::PIN {
        unsafe { &(*GPIO::ptr()).pin3 }
    }

    fn set(&mut self) -> &gpio::SET {
        unsafe { &(*GPIO::ptr()).set3 }
    }
}

pub struct Four(pub(crate) ());
impl GpioRegSet for Four {
    fn dir(&mut self) -> &gpio::DIR {
        unsafe { &(*GPIO::ptr()).dir4 }
    }

    fn clr(&mut self) -> &gpio::CLR {
        unsafe { &(*GPIO::ptr()).clr4 }
    }

    fn mask(&mut self) -> &gpio::MASK {
        unsafe { &(*GPIO::ptr()).mask4 }
    }

    fn pin(&mut self) -> &gpio::PIN {
        unsafe { &(*GPIO::ptr()).pin4 }
    }

    fn set(&mut self) -> &gpio::SET {
        unsafe { &(*GPIO::ptr()).set4 }
    }
}

pub struct Five(pub(crate) ());
impl GpioRegSet for Five {
    fn dir(&mut self) -> &gpio::DIR {
        unsafe { &(*GPIO::ptr()).dir5 }
    }

    fn clr(&mut self) -> &gpio::CLR {
        unsafe { &(*GPIO::ptr()).clr5 }
    }

    fn mask(&mut self) -> &gpio::MASK {
        unsafe { &(*GPIO::ptr()).mask5 }
    }

    fn pin(&mut self) -> &gpio::PIN {
        unsafe { &(*GPIO::ptr()).pin5 }
    }

    fn set(&mut self) -> &gpio::SET {
        unsafe { &(*GPIO::ptr()).set5 }
    }
}
