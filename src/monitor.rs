//! UART Telemetry Monitor Task
//!
//! Enabled via `--features monitor`.  Subscribes to the telemetry PubSub
//! channel and streams binary frames over USART1 (PA9 TX) for host-side
//! visualization with Rerun.
//!
//! # Frame format (19 bytes)
//!
//! | Offset | Size | Field        | Encoding         |
//! |--------|------|------------- |------------------|
//! | 0      | 1    | sync         | `0xAA`           |
//! | 1      | 4    | time_ms      | u32 LE           |
//! | 5      | 4    | temp         | f32 LE           |
//! | 9      | 4    | setpoint     | f32 LE           |
//! | 13     | 4    | power        | f32 LE           |
//! | 17     | 1    | flags        | bit0=en bit1=vlv bit2=pump |
//! | 18     | 1    | checksum     | XOR of bytes 1..17 |
//!
//! # Host usage
//!
//! ```sh
//! python sim/monitor_rerun.py /dev/ttyUSB0
//! ```

use crate::channels::TELEM_PUB;
use embassy_stm32::mode::Async;
use embassy_stm32::usart::UartTx;

const SYNC: u8 = 0xAA;
const FRAME_LEN: usize = 19;

#[embassy_executor::task]
pub async fn monitor_task(mut tx: UartTx<'static, Async>) {
    let mut sub = TELEM_PUB.subscriber().unwrap();

    loop {
        let frame = sub.next_message_pure().await;

        let mut buf = [0u8; FRAME_LEN];
        buf[0] = SYNC;
        buf[1..5].copy_from_slice(&frame.time_ms.to_le_bytes());
        buf[5..9].copy_from_slice(&frame.temp.to_le_bytes());
        buf[9..13].copy_from_slice(&frame.setpoint.to_le_bytes());
        buf[13..17].copy_from_slice(&frame.power.to_le_bytes());
        buf[17] = frame.flags;

        // XOR checksum over payload bytes (1..18)
        let mut chk: u8 = 0;
        for &b in &buf[1..18] {
            chk ^= b;
        }
        buf[18] = chk;

        // Ignore write errors — best-effort telemetry
        let _ = tx.write(&buf).await;
    }
}
