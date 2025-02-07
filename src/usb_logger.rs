use crate::usb_utils::SerialBuffer;
use core::sync::atomic::{AtomicBool, Ordering};
use log::{Level, LevelFilter, Metadata, Record};

static LOGGER: UsbLogger = UsbLogger::new();
static LOGGER_INITIALIZED: AtomicBool = AtomicBool::new(false);

pub struct UsbLogger {
    level: LevelFilter,
}

impl UsbLogger {
    const fn new() -> Self {
        UsbLogger {
            level: LevelFilter::Debug,
        }
    }
}

impl log::Log for UsbLogger {
    fn enabled(&self, metadata: &Metadata) -> bool {
        metadata.level() <= self.level
    }

    fn log(&self, record: &Record) {
        if !self.enabled(record.metadata()) {
            return;
        }

        if let Some(buffer) = unsafe { LOG_SERIAL_BUFFER.as_mut() } {
            // Format the log message with timestamp from PIT if available
            buffer.write(format_args!("[{:?}] {}\r\n", record.level(), record.args()));
        }
    }

    fn flush(&self) {}
}

pub static mut LOG_SERIAL_BUFFER: Option<SerialBuffer<2048>> = None;

pub fn init_logger() -> Result<(), log::SetLoggerError> {
    unsafe {
        LOG_SERIAL_BUFFER = Some(SerialBuffer::<2048>::new());
    }
    if LOGGER_INITIALIZED.load(Ordering::Relaxed) {
        return Ok(());
    }

    log::set_logger(&LOGGER).map(|()| {
        log::set_max_level(LevelFilter::Debug);
        LOGGER_INITIALIZED.store(true, Ordering::Relaxed);
    })
}
