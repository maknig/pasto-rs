use crate::channels::{CONTROL_CH, ControlEvent};
use crate::kalman::Kalman1D;
use embassy_stm32::Peri;
use embassy_stm32::adc::{Adc, AdcChannel, AnyAdcChannel, Resolution, SampleTime};
use embassy_stm32::peripherals::ADC1;

pub struct TempProbe {
    adc: Adc<'static, ADC1>,
    channel: AnyAdcChannel<ADC1>,
    kalman: Kalman1D,
}

impl TempProbe {
    pub fn new(adc: Peri<'static, ADC1>, channel: AnyAdcChannel<ADC1>) -> Self {
        Self {
            adc: Adc::new(adc),
            channel,
            kalman: Kalman1D::new(0.02, 0.5, 25.0),
        }
    }

    pub async fn read_celsius(&mut self) -> f32 {
        let raw = self.adc.blocking_read(&mut self.channel);
        let factor = 150.0 / 4096.0;
        let offset = -0.3;
        let temp = raw as f32 * factor + offset;

        self.kalman.update(temp)
    }
}

// #[embassy_executor::task]
// pub async fn temp_task(mut probe: TempProbe) {
//     loop {
//         let t = probe.read_celsius().await;
//
//         CONTROL_CH.send(ControlEvent::TempUpdate(t)).await;
//
//         Timer::after(Duration::from_millis(50)).await;
//     }
// }
