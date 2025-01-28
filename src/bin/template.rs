#![cfg_attr(not(test), no_std)]
#![cfg_attr(not(test), no_main)]

#[cfg(test)]
extern crate std;

use teensy4_panic as _;

#[rtic::app(device = teensy4_bsp, peripherals = true, dispatchers = [KPP])]
mod app {
    use bsp::board;
    use teensy4_bsp as bsp;
    use board::t40 as my_board;
    use rtic_monotonics::systick::{Systick, *};

    #[shared]
    struct Shared {}

    #[local]
    struct Local {}

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        Systick::start(
            cx.core.SYST,
            board::ARM_FREQUENCY,
            rtic_monotonics::create_systick_token!(),
        );

        main_task::spawn().unwrap();
        (Shared {}, Local {})
    }

    #[task]
    async fn main_task(_: main_task::Context) {
        loop {
            Systick::delay(1000.millis()).await;
            // Your code here
        }
    }
}
