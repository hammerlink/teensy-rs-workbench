#![cfg_attr(not(test), no_std)]
#![cfg_attr(not(test), no_main)]

#[cfg(test)]
extern crate std;

use teensy4_panic as _;

#[rtic::app(device = teensy4_bsp, peripherals = true, dispatchers = [KPP])]
mod app {
    use board::t40 as my_board;
    use bsp::board;
    use rtic_monotonics::systick::{Systick, *};
    use teensy4_bsp as bsp;
    use usb_device::{bus::UsbBusAllocator, class_prelude::UsbClass, prelude::*};
    use usbd_audio::{AudioClassBuilder, Format, StreamConfig, TerminalType};

    #[shared]
    struct Shared {
        // store the buffer for sine samples
    }

    #[local]
    struct Local {
        /// The LED on pin 13.
        led: board::Led,
        usb_dev: UsbDevice<'static, bsp::hal::usbd::BusAdapter>,
        usb_audio: usbd_audio::AudioClass<'static, bsp::hal::usbd::BusAdapter>,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        let board::Resources {
            mut gpio2,
            pins,
            usb,
            ..
        } = my_board(cx.device);

        let led = board::led(&mut gpio2, pins.p13);
        Systick::start(
            cx.core.SYST,
            board::ARM_FREQUENCY,
            rtic_monotonics::create_systick_token!(),
        );

        // Static allocations for USB
        static mut EP_MEMORY: bsp::hal::usbd::EndpointMemory<1024> =
            bsp::hal::usbd::EndpointMemory::new();
        static mut EP_STATE: bsp::hal::usbd::EndpointState = bsp::hal::usbd::EndpointState::new();
        static mut USB_BUS: Option<UsbBusAllocator<bsp::hal::usbd::BusAdapter>> = None;

        // Initialize USB
        unsafe {
            USB_BUS = Some(UsbBusAllocator::new(bsp::hal::usbd::BusAdapter::new(
                usb,
                &mut EP_MEMORY,
                &mut EP_STATE,
            )));
        }
        let usb_bus = unsafe { USB_BUS.as_ref().unwrap() };

        // Configure USB Audio
        let usb_audio = AudioClassBuilder::new()
            .input(
                StreamConfig::new_discrete(Format::S16le, 1, &[44_100], TerminalType::InMicrophone)
                    .unwrap(),
            )
            .build(usb_bus)
            .unwrap();

        // Create USB device
        let usb_dev = UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x16c0, 0x27dd))
            .manufacturer("Hammernet")
            .product("USB audio test")
            .serial_number("42")
            .max_packet_size_0(64)
            .device_class(0x01) // Audio class
            .device_sub_class(0x00)
            .device_protocol(0x00)
            .composite_with_iads()
            .build();

        main_task::spawn().unwrap();
        blink::spawn().unwrap();
        (
            Shared {},
            Local {
                led,
                usb_dev,
                usb_audio,
            },
        )
    }

    #[task(local = [led])]
    async fn blink(cx: blink::Context) {
        loop {
            cx.local.led.toggle();
            Systick::delay(200.millis()).await;
        }
    }

    #[task(local =[usb_dev, usb_audio])]
    async fn main_task(mut cx: main_task::Context) {
        let usb_dev = &mut cx.local.usb_dev;
        let usb_audio = cx.local.usb_audio;

        loop {
            Systick::delay(1000.micros()).await;
            if usb_dev.poll(&mut [usb_audio as &mut dyn UsbClass<bsp::hal::usbd::BusAdapter>]) {
                match usb_audio.input_alt_setting() {
                    Ok(alt) => {}
                    Err(_) => {}
                }
            }
        }
    }
}
