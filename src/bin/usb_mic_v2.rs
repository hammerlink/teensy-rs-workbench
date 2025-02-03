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

    const SINETAB: [i16; 48] = [
        0i16, 4276, 8480, 12539, 16383, 19947, 23169, 25995, 28377, 30272, 31650, 32486, 32767,
        32486, 31650, 30272, 28377, 25995, 23169, 19947, 16383, 12539, 8480, 4276, 0, -4276, -8480,
        -12539, -16383, -19947, -23169, -25995, -28377, -30272, -31650, -32486, -32767, -32486,
        -31650, -30272, -28377, -25995, -23169, -19947, -16383, -12539, -8480, -4276,
    ];

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
        class: AudioClass<'static, Bus>,
        usb_device: UsbDevice<'static, Bus>,
        led: board::Led,
        sine_data: [u8; 96],
    }

    #[shared]
    struct Shared {}

    #[init(local = [bus: Option<UsbBusAllocator<Bus>> = None])]
    fn init(ctx: init::Context) -> (Shared, Local) {
        let board::Resources {
            mut gpio2,
            pins,
            usb,
            ..
        } = my_board(ctx.device);

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
            .output(
                StreamConfig::new_discrete(
                    Format::S24le,
                    2,
                    &[44100, 48000, 96000],
                    TerminalType::OutSpeaker,
                )
                .unwrap(),
            )
            .build(usb_bus)
            .unwrap();

        // Create USB device
        let string_descriptor = StringDescriptors::new(LangID::EN_US)
            .manufacturer("Hammernet")
            .product("Audio Port - Teensy 4")
            .serial_number("42");

        let usb_device = UsbDeviceBuilder::new(usb_bus, VID_PID)
            .max_packet_size_0(64)
            .unwrap()
            .strings(&[string_descriptor])
            .unwrap()
            .build();

        let sine_data = unsafe { *(&SINETAB as *const _ as *const [u8; 96]) };
        (
            Shared {},
            Local {
                class,
                led,
                usb_device,
                sine_data,
            },
        )
    }

    #[task(binds = USB_OTG1, local = [usb_device, class, led, sine_data, configured: bool = false, write_count: usize = 0], priority = 2)]
    fn usb1(ctx: usb1::Context) {
        let usb1::LocalResources {
            class,
            usb_device,
            led,
            configured,
            sine_data,
            write_count,
            ..
        } = ctx.local;
        *write_count += 1;
        if *write_count >= 5 {
            *write_count = 0;
            led.toggle();
        }
        let input_alt = class.input_alt_setting().unwrap();
        let output_alt = class.output_alt_setting().unwrap();
        //if device.state() == UsbDeviceState::Configured {}
        if usb_device.poll(&mut [class as &mut dyn usb_device::class::UsbClass<Bus>]) {
            let mut buf = [0u8; 1024];
            if output_alt > 0 && class.read(&mut buf).is_ok() {}
        }

        if usb_device.state() == UsbDeviceState::Configured {
            if !*configured {
                usb_device.bus().configure();
            }
            *configured = true;
            //led.toggle();
        } else {
            *configured = false;
        }
        if input_alt > 0 {
            class.write(sine_data).ok();
        }

        //// Handle USB configuration state
        //if device.state() == UsbDeviceState::Configured {
        //    if !*configured {
        //        device.bus().configure();
        //        *configured = true;
        //        led.toggle(); // Visual indicator of configuration
        //    }
        //    // Only write data when configured
        //    class.write(sine_data).ok();
        //} else {
        //    *configured = false;
        //}
    }
}
