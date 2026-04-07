// #[embassy_executor::task]
// pub async fn watchdog_task(mut wdg: IndependentWatchdog<'static>) {
//     loop {
//         wdg.pet();
//
//         Timer::after(Duration::from_millis(500)).await;
//     }
// }
