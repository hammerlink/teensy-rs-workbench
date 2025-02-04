//! Demonstrates a USB serial device in RTIC.
//!
//! Flash your board with this example. Then, connect a serial interface to the USB device.
//! You should see all inputs echoed back to you. Every loopback toggles the LED.
//!
//! The USB peripheral varies by board. If there's more than one port on your board, check
//! you board's documentation to understand which port is used here.
//!
//! This example may not work on Windows. Read below for more information.
//!
//! # Known issues
//!
//! `usbd_serial::SerialPort` always allocates a bulk endpoint max packet size of 64 bytes. But
//! when the USB bus is configured for high speed operation, your host may warn about this not being
//! compliant. As of this writing, we cannot configure the `usbd_serial::SerialPort` object, so
//! we're ignoring these warnings (even if that means this example doesn't work on your host).
//! A workaround is to use `usbd_serial::CdcAcmClass` directly; we can configure the max packet size
//! in this API. But, that adds complexity to the example.
//!
//! Similarly, `usbd_serial::CdcAcmClass` believes `bInterval` for the interrupt endpoint should
//! be 255 (ms). This is invalid for high speed operation. We're again ignoring any host warnings
//! for this condition. There is no known workaround besides a patch.
//!
//! I think both of these could be fixed in a backwards-compatible manner in the upstream project.

#![no_std]
#![no_main]

use teensy4_panic as _;

#[rtic::app(device = teensy4_bsp, peripherals = true)]
mod app {
    use bsp::board;
    use hal::usbd::{BusAdapter, EndpointMemory, EndpointState, Speed};
    use imxrt_hal as hal;
    use imxrt_iomuxc::imxrt1060::Pads;
    use imxrt_log::Poller;
    use teensy4_bsp as bsp;

    // If you're using a Teensy 4.1 or MicroMod, you should eventually
    // change 't40' to 't41' or micromod, respectively.
    use board::t40 as my_board;

    use teensy_rs_workbench::{
        clock_tree::{perclk_frequency, uart_frequency, RunMode},
        logging,
        usb_utils::UsbBuffer,
    };
    //use imxrt_iomuxc as iomuxc;
    use usb_device::{
        bus::UsbBusAllocator,
        device::{UsbDevice, UsbDeviceBuilder, UsbDeviceState, UsbVidPid},
    };
    use usbd_serial::SerialPort;

    /// Change me if you want to play with a full-speed USB device.
    const SPEED: Speed = Speed::High;
    /// Matches whatever is in imxrt-log.
    const VID_PID: UsbVidPid = UsbVidPid(0x5824, 0x27dd);
    const PRODUCT: &str = "imxrt-hal-example";
    /// How frequently should we poll the logger?
    const PIT_FREQUENCY: u32 = perclk_frequency(RunMode::Overdrive);
    const LPUART_POLL_INTERVAL_MS: u32 = PIT_FREQUENCY / 1_000 * 100; // resolves to 7_500_000
    /// Change me to change how log messages are serialized.
    ///
    /// If changing to `Defmt`, you'll need to update the logging macros in
    /// this example. You'll also need to make sure the USB device you're debugging
    /// uses `defmt`.
    const FRONTEND: logging::Frontend = logging::Frontend::Log;

    /// Use this symbol to access the 'DMA_A' channel in the
    /// collection of all DMA channels.
    const BOARD_DMA_A_INDEX: usize = 7;

    pub const RUN_MODE: RunMode = RunMode::Overdrive;
    /// The UART clock frequency (Hz).
    const UART_CLK_FREQUENCY: u32 = uart_frequency(RUN_MODE);

    /// The console baud rate: 115200bps.
    const CONSOLE_BAUD: hal::lpuart::Baud = hal::lpuart::Baud::compute(UART_CLK_FREQUENCY, 115200);

    /// This allocation is shared across all USB endpoints. It needs to be large
    /// enough to hold the maximum packet size for *all* endpoints. If you start
    /// noticing panics, check to make sure that this is large enough for all endpoints.
    static EP_MEMORY: EndpointMemory<2048> = EndpointMemory::new();
    /// This manages the endpoints. It's large enough to hold the maximum number
    /// of endpoints; we're not using all the endpoints in this example.
    static EP_STATE: EndpointState = EndpointState::max_endpoints();

    type Bus = BusAdapter;

    #[local]
    struct Local {
        class: SerialPort<'static, Bus>,
        device: UsbDevice<'static, Bus>,
        led: board::Led,
        poller: Poller,
        timer: hal::pit::Pit<0>,
    }

    #[shared]
    struct Shared {}

    #[init(local = [bus: Option<UsbBusAllocator<Bus>> = None])]
    fn init(ctx: init::Context) -> (Shared, Local) {
        let board::Resources {
            mut gpio2,
            pins,
            usb,
            pit: (mut timer, _, _, _),
            mut dma,
            ..
        } = my_board(ctx.device);

        let led = board::led(&mut gpio2, pins.p13);

        let pads = unsafe { Pads::new() };

        let lpuart2 = unsafe { bsp::ral::lpuart::LPUART2::instance() };
        let mut console = hal::lpuart::Lpuart::new(
            lpuart2,
            hal::lpuart::Pins {
                tx: pads.gpio_ad_b1.p02,
                rx: pads.gpio_ad_b1.p03,
            },
        );
        console.disable(|console| {
            console.set_baud(&CONSOLE_BAUD);
            console.set_parity(None);
        });
        timer.set_load_timer_value(LPUART_POLL_INTERVAL_MS);
        timer.set_interrupt_enable(true);
        timer.enable();

        let dma_a = dma[BOARD_DMA_A_INDEX].take().unwrap();
        let poller = logging::lpuart(FRONTEND, console, dma_a);

        let bus = BusAdapter::with_speed(usb, &EP_MEMORY, &EP_STATE, SPEED);
        bus.set_interrupts(true);

        let bus = ctx.local.bus.insert(UsbBusAllocator::new(bus));
        let class = SerialPort::new(bus);
        let device = UsbDeviceBuilder::new(bus, VID_PID)
            .strings(&[usb_device::device::StringDescriptors::default().product(PRODUCT)])
            .unwrap()
            .device_class(usbd_serial::USB_CLASS_CDC)
            .max_packet_size_0(64)
            .unwrap()
            .build();

        (
            Shared {},
            Local {
                class,
                device,
                led,
                poller,
                timer,
            },
        )
    }

    /// Occasionally try to poll the logger.
    #[task(binds = PIT, local = [poller, timer], priority = 1)]
    fn pit_interrupt(ctx: pit_interrupt::Context) {
        while ctx.local.timer.is_elapsed() {
            ctx.local.timer.clear_elapsed();
        }
        ctx.local.poller.poll();
    }

    #[task(binds = USB_OTG1, local = [class, device, led, configured: bool = false, ctr: usize = 0], priority = 2)]
    fn usb1(ctx: usb1::Context) {
        let usb1::LocalResources {
            class,
            device,
            configured,
            led,
            ctr,
            ..
        } = ctx.local;
        *ctr += 1;

        if device.poll(&mut [class]) {
            if device.state() == UsbDeviceState::Configured {
                if !*configured {
                    device.bus().configure();
                }
                *configured = true;

                let mut buffer = [0; 64];
                match class.read(&mut buffer) {
                    Ok(count) => {
                        led.toggle();
                        class.write(&buffer[..count]).ok();

                        let mut buffer = UsbBuffer::<64>::new();
                        let formatted = buffer.format(format_args!("the counter {}\r\n", ctr));
                        class.write(formatted).ok();
                    }
                    Err(usb_device::UsbError::WouldBlock) => {}
                    Err(err) => log::error!("{:?}", err),
                }
            } else {
                *configured = false;
            }
        }
    }
}
