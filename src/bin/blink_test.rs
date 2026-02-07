//! Minimal blink test — just PIO WS2812 LED, nothing else
//! If this works, the problem is in the main code (UART/SD init)

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::PIO0;
use embassy_rp::pio::Pio;
use embassy_rp::pio_programs::ws2812::{PioWs2812, PioWs2812Program};
use embassy_time::{Duration, Timer};
use smart_leds::RGB8;

use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => embassy_rp::pio::InterruptHandler<PIO0>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let Pio {
        mut common, sm0, ..
    } = Pio::new(p.PIO0, Irqs);
    let program = PioWs2812Program::new(&mut common);
    let mut ws = PioWs2812::new(&mut common, sm0, p.DMA_CH0, p.PIN_25, &program);

    loop {
        // Blue
        ws.write(&[RGB8::new(0, 0, 100)]).await;
        Timer::after(Duration::from_millis(300)).await;
        // Off
        ws.write(&[RGB8::new(0, 0, 0)]).await;
        Timer::after(Duration::from_millis(300)).await;
    }
}
