#![no_std]
#![no_main]

use defmt::info;
use embassy_executor::Spawner;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::{Input, Level, Output, Pull, Speed};
use embassy_stm32::i2c::I2c;
use embassy_stm32::time::Hertz;
use embassy_stm32::{bind_interrupts, i2c, peripherals};
use mpu_6500_async::{calibration, raw_data::RawData, Mpu6500};
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    I2C1_EV => i2c::EventInterruptHandler<peripherals::I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<peripherals::I2C1>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());

    let mut led = Output::new(p.PC13, Level::High, Speed::Low);
    let mut imu_interrupt = ExtiInput::new(Input::new(p.PB3, Pull::Up), p.EXTI3);

    let i2c = I2c::new(
        p.I2C1,
        p.PA15,
        p.PB7,
        Irqs,
        p.DMA1_CH6,
        p.DMA1_CH7,
        Hertz(400_000),
        Default::default(),
    );

    let mut imu = Mpu6500::new(i2c);

    if !imu.detected().await.unwrap() {
        defmt::panic!("IMU not detected!");
    }

    info!("IMU detected");

    let gyro_bias = if cfg!(feature = "calibrate-gyro") {
        info!("Calculating gyro bias");
        calibration::get_gyro_bias(&mut imu, &mut embassy_time::Delay)
            .await
            .unwrap()
    } else {
        info!("Using hardcoded gyro bias");
        RawData::new(43, 41, 44)
    };

    imu.init(&mut embassy_time::Delay).await.unwrap();

    info!("IMU initialized");

    calibration::set_gyro_offset(&mut imu, gyro_bias)
        .await
        .unwrap();

    info!("Gyro offsets set");

    imu.enable_data_ready_interrupt().await.unwrap();

    info!("IMU interrupts enabled");

    loop {
        imu_interrupt.wait_for_low().await;

        led.set_high();

        let gyro = imu.read_gyro_data().await.unwrap();

        led.set_low();

        info!("Gyro raw data: {} {} {}", gyro.x(), gyro.y(), gyro.z());
    }
}
