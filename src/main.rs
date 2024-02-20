#![no_std]
#![no_main]

use defmt::{info, unwrap};
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_executor::Spawner;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::{Input, Level, Output, Pull, Speed};
use embassy_stm32::i2c::I2c;
use embassy_stm32::time::Hertz;
use embassy_stm32::{bind_interrupts, i2c, peripherals};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::Timer;
use mpu_6500_async::{calibration, raw_data::RawData, Mpu6500};
use static_cell::StaticCell;
use vl53l1x_async::register_map::{DistanceMode, TimingBudget};
use vl53l1x_async::Vl53l1x;
use {defmt_rtt as _, panic_probe as _};

type I2c1<'a> = I2c<'a, peripherals::I2C1, peripherals::DMA1_CH6, peripherals::DMA1_CH7>;

static I2C1_BUS: StaticCell<Mutex<NoopRawMutex, I2c1>> = StaticCell::new();

bind_interrupts!(struct Irqs {
    I2C1_EV => i2c::EventInterruptHandler<peripherals::I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<peripherals::I2C1>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());

    let mut led = Output::new(p.PC13, Level::High, Speed::Low);
    let mut fl_en = Output::new(p.PB8, Level::Low, Speed::Low);
    let mut fr_en = Output::new(p.PA1, Level::Low, Speed::Low);
    let mut dl_en = Output::new(p.PB9, Level::Low, Speed::Low);
    let mut dr_en = Output::new(p.PA0, Level::Low, Speed::Low);

    let imu_interrupt = ExtiInput::new(Input::new(p.PB3, Pull::Up), p.EXTI3);
    let ds_interrupt = ExtiInput::new(Input::new(p.PA4, Pull::Down), p.EXTI4);

    fl_en.set_low();
    fr_en.set_low();
    dl_en.set_low();
    dr_en.set_low();

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

    let i2c_bus = I2C1_BUS.init(Mutex::new(i2c));

    let imu = Mpu6500::new(I2cDevice::new(i2c_bus));

    let ds = Vl53l1x::new(I2cDevice::new(i2c_bus)).with_x_shut(fl_en);

    unwrap!(spawner.spawn(imu_task(imu, imu_interrupt)));

    unwrap!(spawner.spawn(distance_sensor_task(ds, ds_interrupt)));

    loop {
        led.set_high();

        Timer::after_millis(1000).await;

        led.set_low();

        Timer::after_millis(1000).await;
    }
}

#[embassy_executor::task]
async fn imu_task(
    mut imu: Mpu6500<I2cDevice<'static, NoopRawMutex, I2c1<'static>>>,
    mut imu_interrupt: ExtiInput<'static, peripherals::PB3>,
) {
    info!("IMU task started");

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

        let gyro = imu.read_gyro_data().await.unwrap();

        info!("Gyro raw data: {} {} {}", gyro.x(), gyro.y(), gyro.z());
    }
}

#[embassy_executor::task]
async fn distance_sensor_task(
    mut ds: Vl53l1x<
        I2cDevice<'static, NoopRawMutex, I2c1<'static>>,
        Output<'static, peripherals::PB8>,
    >,
    mut ds_interrupt: ExtiInput<'static, peripherals::PA4>,
) {
    info!("Sensor task started");

    ds.reset_until_detection(&mut embassy_time::Delay, 1000)
        .await
        .unwrap();

    info!("Sensor turned on");

    ds.wait_until_booted(&mut embassy_time::Delay, 1000)
        .await
        .unwrap();

    info!("Sensor booted");

    ds.init(&mut embassy_time::Delay, 1000).await.unwrap();

    info!("Sensor initialized");

    ds.set_interrupt_polarity(true).await.unwrap();
    ds.set_distance_mode(DistanceMode::Short).await.unwrap();
    ds.set_timing_budget(TimingBudget::Ms15).await.unwrap();
    ds.set_inter_measurement_period(15).await.unwrap();

    info!("Sensor configured");

    ds.start_ranging().await.unwrap();

    info!("Ranging started");

    loop {
        ds_interrupt.wait_for_high().await;

        let distance = ds.get_distance().await.unwrap();

        ds.clear_interrupt().await.unwrap();

        info!("Distance sensor data: {}", distance);
    }
}
