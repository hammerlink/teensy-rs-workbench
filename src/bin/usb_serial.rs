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
        clock_tree::{uart_frequency, RunMode},
        logging,
        usb_utils::{SerialBuffer, UsbBuffer},
        usb_write,
    };
    //use imxrt_iomuxc as iomuxc;
    use usb_device::{
        bus::UsbBusAllocator,
        device::{StringDescriptors, UsbDevice, UsbDeviceBuilder, UsbDeviceState, UsbVidPid},
        LangID,
    };
    use usbd_audio::{AudioClass, AudioClassBuilder, Format, StreamConfig, TerminalType};
    use usbd_serial::SerialPort;

    /// Change me if you want to play with a full-speed USB device.
    const SPEED: Speed = Speed::High;
    /// Matches whatever is in imxrt-log.
    const VID_PID: UsbVidPid = UsbVidPid(0x5824, 0x27dd);
    const PRODUCT: &str = "imxrt-hal-example";
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

    const TIMER_MICRO_SECONDS: u32 = 1_000;

    type Bus = BusAdapter;

    #[local]
    struct Local {
        device: UsbDevice<'static, Bus>,
        led: board::Led,
        poller: Poller,
        timer: hal::pit::Pit<0>,
    }

    #[shared]
    struct Shared {
        serial_buffer: SerialBuffer<2048>,
        configured: bool,
        usb_serial: SerialPort<'static, Bus>,
        usb_audio: AudioClass<'static, Bus>,
        usb_interrupt_counter: usize,
    }

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
        timer.set_load_timer_value(TIMER_MICRO_SECONDS);
        timer.set_interrupt_enable(true);
        timer.enable();

        let dma_a = dma[BOARD_DMA_A_INDEX].take().unwrap();
        let poller = logging::lpuart(FRONTEND, console, dma_a);

        let bus = BusAdapter::with_speed(usb, &EP_MEMORY, &EP_STATE, SPEED);
        bus.set_interrupts(true);

        let bus = ctx.local.bus.insert(UsbBusAllocator::new(bus));
        let class = SerialPort::new(bus);

        let string_descriptor = StringDescriptors::new(LangID::EN_US)
            .manufacturer("Hammernet")
            .product("Audio Port - Teensy 4")
            .serial_number("42");
        // Configure USB Audio
        let audio_class = AudioClassBuilder::new()
            .input(
                StreamConfig::new_discrete(Format::S16le, 1, &[48_000], TerminalType::InMicrophone)
                    .unwrap(),
            )
            .output(
                StreamConfig::new_discrete(
                    Format::S24le,
                    2,
                    &[44_100, 48_000],
                    TerminalType::OutSpeaker,
                )
                .unwrap(),
            )
            .build(bus)
            .unwrap();
        let device = UsbDeviceBuilder::new(bus, VID_PID)
            .max_packet_size_0(64)
            .unwrap()
            .strings(&[string_descriptor])
            .unwrap()
            .device_class(0x01) // Audio class
            .device_sub_class(0x00)
            .device_protocol(0x00)
            .max_packet_size_0(64)
            .unwrap()
            .build();

        (
            Shared {
                serial_buffer: SerialBuffer::<2048>::new(),
                configured: false,
                usb_serial: class,
                usb_audio: audio_class,
                usb_interrupt_counter: 0,
            },
            Local {
                device,
                led,
                poller,
                timer,
            },
        )
    }

    /// Occasionally try to poll the logger.
    #[task(binds = PIT, local = [poller, timer, ctr: usize = 0], shared = [serial_buffer, configured, usb_serial], priority = 1)]
    fn pit_interrupt(mut ctx: pit_interrupt::Context) {
        while ctx.local.timer.is_elapsed() {
            ctx.local.timer.clear_elapsed();
        }
        *ctx.local.ctr += 1;

        // triggers every second
        if *ctx.local.ctr % 1000 == 0 {
            // Add some test data to the buffer
            ctx.shared.serial_buffer.lock(|buffer| {
                buffer.write(format_args!("pit timer ctr {} \r\n", ctx.local.ctr));
            });
            ctx.local.poller.poll();
            if *ctx.local.ctr >= 10_000usize {
                *ctx.local.ctr = 0usize;
            }
        }
        let is_configured = ctx.shared.configured.lock(|configured| *configured);
        if is_configured {
            ctx.shared.serial_buffer.lock(|buffer| {
                if !buffer.has_data() {
                    return;
                }
                let mut read_cursor: usize = 0;
                while let Some(chunk) = buffer.read_chunk(64, read_cursor) {
                    if ctx
                        .shared
                        .usb_serial
                        .lock(|serial| serial.write(chunk))
                        .is_err()
                    {
                        break;
                    }
                    read_cursor += chunk.len();
                    if read_cursor >= buffer.get_buffer().len() {
                        break;
                    }
                }
                buffer.clear();
            });
        }
    }

    #[task(binds = USB_OTG1, local = [device, led, configured: bool = false], shared = [serial_buffer, configured, usb_serial, usb_audio, usb_interrupt_counter], priority = 2)]
    fn usb1(mut ctx: usb1::Context) {
        let usb1::LocalResources {
            device,
            led,
            configured,
            ..
        } = ctx.local;
        let counter = ctx.shared.usb_interrupt_counter.lock(|ctr| {
            *ctr += 1;
            *ctr
        });

        ctx.shared.usb_serial.lock(|usb_serial| {
            if counter % 10 == 0 {
                usb_write!(usb_serial, "usb1 interrupt counter {} \r\n", counter);
                if counter >= 1_000_000 {
                    ctx.shared.usb_interrupt_counter.lock(|ctr| *ctr = 0);
                }
            }
            ctx.shared.usb_audio.lock(|usb_audio| {
                if device.poll(&mut [usb_serial, usb_audio]) {
                    if device.state() == UsbDeviceState::Configured {
                        if !*configured {
                            device.bus().configure();
                            ctx.shared.configured.lock(|x| *x = true);
                            *configured = true;
                        }
                    }
                    let mut buffer = [0; 64];
                    match usb_serial.read(&mut buffer) {
                        Ok(count) => {
                            led.toggle();
                            usb_serial.write(&buffer[..count]).ok();
                            usb_write!(usb_serial, "the counter {} \r\n", counter);
                        }
                        Err(usb_device::UsbError::WouldBlock) => {}
                        Err(err) => log::error!("{:?}", err),
                    }
                }
            });
        });
    }
}
