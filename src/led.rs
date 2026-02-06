//! WS2812 LED driver using embassy-rp built-in PIO program

use embassy_rp::pio::{Common, Instance, StateMachine};
use embassy_rp::pio_programs::ws2812::{PioWs2812, PioWs2812Program};
use embassy_rp::Peri;
use smart_leds::RGB8;

/// RGB color wrapper
#[derive(Clone, Copy, Default)]
pub struct Color {
    pub r: u8,
    pub g: u8,
    pub b: u8,
}

impl Color {
    pub const fn new(r: u8, g: u8, b: u8) -> Self {
        Self { r, g, b }
    }

    pub const fn off() -> Self {
        Self { r: 0, g: 0, b: 0 }
    }

    pub const fn green() -> Self {
        Self {
            r: 0,
            g: 50,
            b: 0,
        }
    }

    pub const fn blue() -> Self {
        Self {
            r: 0,
            g: 0,
            b: 100,
        }
    }

    pub const fn red() -> Self {
        Self {
            r: 100,
            g: 0,
            b: 0,
        }
    }

    pub const fn yellow() -> Self {
        Self {
            r: 80,
            g: 40,
            b: 0,
        }
    }

    /// Convert to smart_leds RGB8
    pub fn to_rgb8(self) -> RGB8 {
        RGB8::new(self.r, self.g, self.b)
    }
}

/// WS2812 driver wrapper using embassy-rp built-in driver
pub struct Ws2812<'d, P: Instance, const S: usize> {
    inner: PioWs2812<'d, P, S, 1, embassy_rp::pio_programs::ws2812::Grb>,
}

impl<'d, P: Instance, const S: usize> Ws2812<'d, P, S> {
    /// Create new WS2812 driver using built-in PIO program
    pub fn new<PIN: embassy_rp::pio::PioPin, DMA: embassy_rp::dma::Channel>(
        common: &mut Common<'d, P>,
        sm: StateMachine<'d, P, S>,
        dma: Peri<'d, DMA>,
        pin: Peri<'d, PIN>,
    ) -> Self {
        let program = PioWs2812Program::new(common);
        let inner = PioWs2812::new(common, sm, dma, pin, &program);
        Self { inner }
    }

    /// Write a color to the LED
    pub async fn write_color(&mut self, color: Color) {
        let colors = [color.to_rgb8()];
        self.inner.write(&colors).await;
    }
}
