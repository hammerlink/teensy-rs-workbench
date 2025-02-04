use core::fmt::Write;

/// A fixed-size buffer for formatting strings
pub struct UsbBuffer<const N: usize> {
    buffer: [u8; N],
    cursor: usize,
}

impl<const N: usize> UsbBuffer<N> {
    /// Creates a new empty buffer
    pub fn new() -> Self {
        Self {
            buffer: [0; N],
            cursor: 0,
        }
    }

    /// Format a string with arguments and return it as bytes
    pub fn format<'a>(&'a mut self, args: core::fmt::Arguments<'_>) -> &'a [u8] {
        self.cursor = 0;
        // Ignore error since we can't do much about it in no_std
        let _ = self.write_fmt(args);
        &self.buffer[..self.cursor]
    }

    // Add this new helper method
    pub fn format_fixed(&mut self, args: core::fmt::Arguments<'_>) -> ([u8; N], usize) {
        let slice = self.format(args);
        let mut result = [0u8; N];
        result[..slice.len()].copy_from_slice(slice);
        (result, slice.len())
    }
}

impl<const N: usize> core::fmt::Write for UsbBuffer<N> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let bytes = s.as_bytes();
        let remaining_space = self.buffer.len() - self.cursor;
        let count = remaining_space.min(bytes.len());

        self.buffer[self.cursor..self.cursor + count].copy_from_slice(&bytes[..count]);
        self.cursor += count;

        if count == bytes.len() {
            Ok(())
        } else {
            Err(core::fmt::Error)
        }
    }
}

pub struct SerialBuffer<const N: usize> {
    buffer: [u8; N],
    cursor: usize,
}

impl<const N: usize> SerialBuffer<N> {
    pub fn new() -> Self {
        Self {
            buffer: [0; N],
            cursor: 0,
        }
    }

    pub fn has_data(&self) -> bool {
        self.cursor > 0
    }

    pub fn write_str(&mut self, s: &str) -> &[u8] {
        // Ignore error since we can't do much about it in no_std
        let _ = Write::write_str(self, s);
        &self.buffer[..self.cursor]
    }

    pub fn write(&mut self, args: core::fmt::Arguments<'_>) -> &[u8] {
        // Ignore error since we can't do much about it in no_std
        let _ = self.write_fmt(args);
        &self.buffer[..self.cursor]
    }

    pub fn get_buffer(&self) -> &[u8] {
        &self.buffer[..self.cursor]
    }

    pub fn read_chunk(&self, chunk_size: usize, start_index: usize) -> Option<&[u8]> {
        // Return None if buffer is empty or start_index is beyond current data
        if self.cursor == 0 || start_index >= self.cursor {
            return None;
        }

        // Calculate how many bytes are available from the start_index
        let remaining = self.cursor - start_index;
        let available = remaining.min(chunk_size);

        // Ensure we don't exceed buffer bounds
        if start_index + available > self.buffer.len() {
            return None;
        }

        Some(&self.buffer[start_index..(start_index + available)])
    }

    pub fn clear(&mut self) {
        self.cursor = 0;
    }
}

impl<const N: usize> core::fmt::Write for SerialBuffer<N> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let bytes = s.as_bytes();
        let remaining_space = self.buffer.len() - self.cursor;
        let count = remaining_space.min(bytes.len());

        // only write the bytes that fit in the buffer
        self.buffer[self.cursor..self.cursor + count].copy_from_slice(&bytes[..count]);
        self.cursor += count;

        Ok(())
    }
}

#[macro_export]
macro_rules! u8_bytes {
    ($($arg:tt)*) => {
        UsbBuffer::<64>::new().format(format_args!($($arg)*))
    };
}
#[macro_export]
macro_rules! usb_write {
    ($class:expr, $($arg:tt)*) => {
        $class.write(UsbBuffer::<64>::new().format(format_args!($($arg)*))).ok()
    };
}
