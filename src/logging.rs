/// Select the logging front-end.
#[derive(Debug, defmt::Format, PartialEq, Eq)]
pub enum Frontend {
    /// Use the `log` crate.
    Log,
    /// Use `defmt`.
    Defmt,
}

/// Select the logging back-end.
#[derive(Debug, defmt::Format, PartialEq, Eq)]
pub enum Backend {
    /// Use a USB peripheral.
    Usbd,
    /// Use LPUART and DMA.
    Lpuart,
}
use hal::dma::channel::Channel;
use hal::lpuart::Lpuart;
use hal::usbd::Instances;
use imxrt_hal as hal;
pub use imxrt_log::Poller;

pub const BACKEND: Backend = Backend::Usbd;

/// Initialize the logger.
pub fn init<P, const LPUART: u8, const USBD: u8>(
    frontend: Frontend,
    backend: Backend,
    lpuart: Lpuart<P, LPUART>,
    dma: Channel,
    usbd: Instances<USBD>,
) -> imxrt_log::Poller {
    // Always enable interrupts. If you don't want them to activate, don't unmask them.
    match (frontend, backend) {
        // Logging frontends...
        (Frontend::Log, Backend::Lpuart) => {
            imxrt_log::log::lpuart(lpuart, dma, imxrt_log::Interrupts::Enabled).unwrap()
        }
        (Frontend::Log, Backend::Usbd) => {
            imxrt_log::log::usbd(usbd, imxrt_log::Interrupts::Enabled).unwrap()
        }
        (Frontend::Defmt, Backend::Lpuart) => {
            imxrt_log::defmt::lpuart(lpuart, dma, imxrt_log::Interrupts::Enabled).unwrap()
        }
        (Frontend::Defmt, Backend::Usbd) => {
            imxrt_log::defmt::usbd(usbd, imxrt_log::Interrupts::Enabled).unwrap()
        }
    }
}

/// Initialize the LPUART logger.
///
/// Useful when you're debugging USB devices, and you want to get
/// log messages out of the device some other way.
///
/// This always enables interrupts. If you don't want interrupts to active,
/// then don't unmask them.
pub fn lpuart<P, const LPUART: u8>(
    frontend: Frontend,
    lpuart: Lpuart<P, LPUART>,
    dma_channel: Channel,
) -> imxrt_log::Poller {
    match frontend {
        Frontend::Log => {
            imxrt_log::log::lpuart(lpuart, dma_channel, imxrt_log::Interrupts::Enabled).unwrap()
        }
        Frontend::Defmt => {
            imxrt_log::defmt::lpuart(lpuart, dma_channel, imxrt_log::Interrupts::Enabled).unwrap()
        }
    }
}
