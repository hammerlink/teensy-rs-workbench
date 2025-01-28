#![cfg_attr(not(test), no_std)]
#![cfg_attr(not(test), no_main)]

#[cfg(test)]
extern crate std;

use teensy4_panic as _;

#[rtic::app(device = teensy4_bsp, peripherals = true, dispatchers = [KPP])]
mod app {
    use board::t40 as my_board;
    use bsp::board;
    use core::f32::consts::PI;
    use libm::sinf;
    use rtic_monotonics::systick::{Systick, *};
    use teensy4_bsp as bsp;
    use usb_device::{bus::UsbBusAllocator, class_prelude::UsbClass, prelude::*, UsbError};
    use usbd_audio::{AudioClassBuilder, Format, StreamConfig, TerminalType};

    #[shared]
    struct Shared {
        sample_index: u32,
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
            Shared { sample_index: 0 },
            Local {
                led,
                usb_dev,
                usb_audio,
            },
        )
    }

    fn generate_sine_sample(sample_index: u32, frequency: f32, sample_rate: f32) -> i16 {
        let time = sample_index as f32 / sample_rate;
        let angle = 2.0 * PI * frequency * time;
        let amplitude = 0.5; // 50% of max volume
        (amplitude * sinf(angle) * 32767.0) as i16
    }

    #[task(local = [led])]
    async fn blink(cx: blink::Context) {
        loop {
            cx.local.led.toggle();
            Systick::delay(200.millis()).await;
        }
    }

    #[task(local =[usb_dev, usb_audio], shared = [sample_index])]
    async fn main_task(mut cx: main_task::Context) {
        let usb_dev = &mut cx.local.usb_dev;
        let usb_audio = cx.local.usb_audio;

        // Calculate samples per transfer:
        // At 44.1kHz, we send 44.1 samples per millisecond
        // USB full-speed isochronous transfers happen every 1ms frame
        const SAMPLE_RATE: f32 = 44_100.0;
        const USB_FRAME_MS: u32 = 1;
        const SAMPLES_PER_TRANSFER: usize = 44; // ~44 samples per transfer
        const SINE_FREQ: f32 = 550.0;

        let mut sample_buffer = [0i16; SAMPLES_PER_TRANSFER];

        loop {
            Systick::delay(1000.micros()).await;
            if usb_dev.poll(&mut [usb_audio as &mut dyn UsbClass<bsp::hal::usbd::BusAdapter>]) {
                match usb_audio.input_alt_setting() {
                    Ok(alt) if alt > 0 => {
                        // Fill the buffer with sine wave samples
                        for i in 0..SAMPLES_PER_TRANSFER {
                            let sample_idx = cx.shared.sample_index.lock(|idx| {
                                *idx += 1;
                                *idx
                            });
                            sample_buffer[i] =
                                generate_sine_sample(sample_idx, SINE_FREQ, SAMPLE_RATE);
                        }

                        // Convert i16 samples to bytes (2 bytes per sample in little-endian format)
                        let bytes: [u8; SAMPLES_PER_TRANSFER * 2] = unsafe {
                            core::mem::transmute::<
                                [i16; SAMPLES_PER_TRANSFER],
                                [u8; SAMPLES_PER_TRANSFER * 2],
                            >(sample_buffer)
                        };
                        // Send the samples
                        usb_audio.write(&bytes);
                    }
                    _ => {}
                }
            }
        }
    }
}
