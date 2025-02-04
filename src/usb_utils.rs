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
}

impl<const N: usize> core::fmt::Write for UsbBuffer<N> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let bytes = s.as_bytes();
        let remaining_space = self.buffer.len() - self.cursor;
        let count = remaining_space.min(bytes.len());
        
        self.buffer[self.cursor..self.cursor + count]
            .copy_from_slice(&bytes[..count]);
        self.cursor += count;
        
        if count == bytes.len() {
            Ok(())
        } else {
            Err(core::fmt::Error)
        }
    }
}
