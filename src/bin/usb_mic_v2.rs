//! Demonstrates a USB audio microphone using RTIC.
#![no_std]
#![no_main]

use teensy4_panic as _;

#[rtic::app(device = teensy4_bsp, peripherals = true)]
mod app {
    use bsp::board;
    use hal::usbd::{BusAdapter, EndpointMemory, EndpointState, Speed};
    use imxrt_hal as hal;
    use teensy4_bsp as bsp;

    use usb_device::{
        bus::UsbBusAllocator,
        device::{UsbDeviceBuilder, UsbVidPid},
        prelude::*,
        LangID,
    };
    use usbd_audio::{AudioClass, AudioClassBuilder, Format, StreamConfig, TerminalType};

    // If you're using a Teensy 4.1 or MicroMod, you should eventually
    // change 't40' to 't41' or micromod, respectively.
    use board::t40 as my_board;

    /// Change me if you want to play with a full-speed USB device.
    const SPEED: Speed = Speed::High;
    /// Matches whatever is in imxrt-log.
    const VID_PID: UsbVidPid = UsbVidPid(0x5824, 0x27dd);

    const SINETAB: [u8; 96] = [
    0,   0, 180,  16,  32,  33, 251,  48, 255,  63, 235,  77,
  129,  90, 139, 101, 217, 110,  64, 118, 162, 123, 230, 126,
  255, 127, 230, 126, 162, 123,  64, 118, 217, 110, 139, 101,
  129,  90, 235,  77, 255,  63, 251,  48,  32,  33, 180,  16,
    0,   0,  76, 239, 224, 222,   5, 207,   1, 192,  21, 178,
  127, 165, 117, 154,  39, 145, 192, 137,  94, 132,  26, 129,
    1, 128,  26, 129,  94, 132, 192, 137,  39, 145, 117, 154,
  127, 165,  21, 178,   1, 192,   5, 207, 224, 222,  76, 239
];
    const TIMER_MICRO_SECONDS: u32 = 125;

    /// This allocation is shared across all USB endpoints. It needs to be large
    /// enough to hold the maximum packet size for *all* endpoints. If you start
    /// noticing panics, check to make sure that this is large enough for all endpoints.
    static EP_MEMORY: EndpointMemory<1024> = EndpointMemory::new();
    /// This manages the endpoints. It's large enough to hold the maximum number
    /// of endpoints; we're not using all the endpoints in this example.
    static EP_STATE: EndpointState = EndpointState::max_endpoints();

    type Bus = BusAdapter;

    #[local]
    struct Local {
        usb_device: UsbDevice<'static, Bus>,
        led: board::Led,
        timer: hal::pit::Pit<0>,
    }

    #[shared]
    struct Shared {
        usb_audio_on: bool,
        usb_audio: AudioClass<'static, Bus>,
    }

    #[init(local = [bus: Option<UsbBusAllocator<Bus>> = None])]
    fn init(ctx: init::Context) -> (Shared, Local) {
        let board::Resources {
            mut gpio2,
            pins,
            usb,
            pit: (mut timer, _, _, _),
            ..
        } = my_board(ctx.device);

        timer.set_load_timer_value(TIMER_MICRO_SECONDS);
        timer.set_interrupt_enable(true);
        timer.enable();
        let led = board::led(&mut gpio2, pins.p13);

        let bus = BusAdapter::with_speed(usb, &EP_MEMORY, &EP_STATE, SPEED);
        bus.set_interrupts(true);

        let usb_bus = ctx.local.bus.insert(UsbBusAllocator::new(bus));
        //let bus = ctx.local.bus.insert(UsbBusAllocator::new(bus));

        // Configure USB Audio
        let class = AudioClassBuilder::new()
            .input(
                StreamConfig::new_discrete(Format::S16le, 1, &[48_000], TerminalType::InMicrophone)
                    .unwrap(),
            )
            .build(usb_bus)
            .unwrap();

        // Create USB device
        let string_descriptor = StringDescriptors::new(LangID::EN_US)
            .manufacturer("Hammernet")
            .product("Audio Port - Teensy 4 - Pure micro")
            .serial_number("42");

        let usb_device = UsbDeviceBuilder::new(usb_bus, VID_PID)
            .max_packet_size_0(64)
            .unwrap()
            .strings(&[string_descriptor])
            .unwrap()
            .build();

        (
            Shared {
                usb_audio: class,
                usb_audio_on: false,
            },
            Local {
                led,
                usb_device,
                timer,
            },
        )
    }

    #[task(binds = PIT, local = [timer, read_count: usize = 0], shared = [usb_audio, usb_audio_on])]
    fn pit_interrupt(mut ctx: pit_interrupt::Context) {
        let pit_interrupt::LocalResources {
            read_count,
            timer,
            ..
        } = ctx.local;
        while timer.is_elapsed() {
            timer.clear_elapsed();
        }
        let is_audio_on = ctx.shared.usb_audio_on.lock(|x| *x);
        if !is_audio_on {
            return;
        }
        *read_count += 1;
        if *read_count >= 8 {
            *read_count = 0;
        }
        // take one 8th of the sine wave
        let start = *read_count * 12;
        let slice = &SINETAB[start..start + 12];
        ctx.shared.usb_audio.lock(|audio| audio.write(slice).ok());

    }

    #[task(
        binds = USB_OTG1, 
        local = [usb_device, led, configured: bool = false], 
        shared = [usb_audio, usb_audio_on], 
        priority = 1
    )]
    fn usb1(mut ctx: usb1::Context) {
        let usb1::LocalResources {
            usb_device,
            led,
            configured,
            ..
        } = ctx.local;
        ctx.shared.usb_audio.lock(|usb_audio| {
            let (_, _, request_result) =  usb_device.poll_specific(&mut [usb_audio]);
            if *configured {
                if let Some(request) = request_result  {
                // volume is turned on or off
                    if *configured && request.request == 11u8 {
                        let value = request.value;
                        let is_on = value == 1u16;
                        ctx.shared.usb_audio_on.lock(|x| *x = is_on);
                    }
                }
            }
            if usb_device.state() == UsbDeviceState::Configured {
                if !*configured {
                    usb_device.bus().configure();
                    led.toggle();
                }
                *configured = true;
            } else {
                *configured = false;
            }
        });
    }
}
