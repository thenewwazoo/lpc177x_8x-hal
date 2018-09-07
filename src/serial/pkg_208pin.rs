use lpc::{UART0, UART1, UART2, UART3, UART4};

use super::*;
use gpio::*;

unsafe impl TxPin<UART0> for P0_0<AF4> {}
unsafe impl TxPin<UART0> for P0_2<AF1> {}

unsafe impl RxPin<UART0> for P0_1<AF4> {}
unsafe impl RxPin<UART0> for P0_3<AF1> {}

unsafe impl TxPin<UART1> for P0_15<AF1> {}
unsafe impl TxPin<UART1> for P2_0<AF2> {}
unsafe impl TxPin<UART1> for P3_16<AF3> {}

unsafe impl RxPin<UART1> for P0_16<AF1> {}
unsafe impl RxPin<UART1> for P2_1<AF2> {}
unsafe impl RxPin<UART1> for P3_17<AF3> {}

unsafe impl TxPin<UART2> for P0_10<AF1> {}
unsafe impl TxPin<UART2> for P2_8<AF2> {}
unsafe impl TxPin<UART2> for P4_22<AF2> {}

unsafe impl RxPin<UART2> for P0_11<AF1> {}
unsafe impl RxPin<UART2> for P2_9<AF2> {}
unsafe impl RxPin<UART2> for P4_23<AF2> {}

unsafe impl TxPin<UART3> for P0_0<AF2> {}
unsafe impl TxPin<UART3> for P0_2<AF2> {}
unsafe impl TxPin<UART3> for P0_25<AF3> {}
unsafe impl TxPin<UART3> for P4_28<AF2> {}

unsafe impl RxPin<UART3> for P0_1<AF2> {}
unsafe impl RxPin<UART3> for P0_3<AF2> {}
unsafe impl RxPin<UART3> for P0_26<AF3> {}
unsafe impl RxPin<UART3> for P4_29<AF2> {}

unsafe impl TxPin<UART4> for P0_22<AF3> {}
unsafe impl TxPin<UART4> for P1_29<AF5> {}
unsafe impl TxPin<UART4> for P5_4<AF4> {}

unsafe impl RxPin<UART4> for P2_9<AF3> {}
unsafe impl RxPin<UART4> for P5_3<AF4> {}
