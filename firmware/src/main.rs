#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_futures::yield_now;
use embassy_net::{Stack, StackResources, DhcpConfig};
use embassy_net_wiznet::chip::W5500;
use embassy_net_wiznet::*;
use embassy_rp::clocks::RoscRng;
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::peripherals::SPI0;
use embassy_rp::peripherals;
use embassy_rp::spi::{Async, Config as SpiConfig, Spi};
use embassy_rp::watchdog::*;
use embassy_time::{Delay, Duration, Ticker, Instant};
use embedded_hal_bus::spi::ExclusiveDevice;
//use embedded_io_async::Write;
use rand::RngCore;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};
use assign_resources::assign_resources;

use core::cell::RefCell;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::blocking_mutex::Mutex;

use nom::IResult;
use nom::branch::alt;
use nom::bytes::complete::tag;
use nom::character::complete::space1;
use nom::sequence::separated_pair;
use nom::sequence::pair;
use nom::sequence::preceded;
use nom::number::complete::float;
use nom::Parser;

const FLASH_SIZE:usize = 2 * 1024 * 1024;
static FLASH_UUID: Mutex<ThreadModeRawMutex, RefCell<[u8;8]>> = Mutex::new(RefCell::new([0u8; 8]));

use git_version::git_version;
const PRODUCT_NAME:&str = "G-5500 HamLib Adaptor - Phil Crump M0DNY";
const GIT_VERSION:&str = git_version!(args = ["--dirty", "--always"], fallback = "nogit");

// Watchdog period needs to include max dhcp config duration
const WATCHDOG_PERIOD_MS:u64 = 8300; // Max is 8388ms
static WATCHDOG_RESET_SYSTEM: Mutex<ThreadModeRawMutex, RefCell<bool>> = Mutex::new(RefCell::new(false));

const DHCP_HOSTNAME:&str = "g5500-hamlib-adaptor";
const NUMBER_HAMLIB_SOCKETS:u16 = 4;
const SOCKET_TIMEOUT_S:u64 = 60;

static SOCKETS_CONNECTED: Mutex<ThreadModeRawMutex, RefCell<u16>> = Mutex::new(RefCell::new(0));

const PARK_AZ_DEGREES:f32 = 180.0;
const PARK_EL_DEGREES:f32 = 0.0;

const CONTROL_DEGREES_THRESHOLD:f32 = 3.0;
const CONTROL_AZ_DEGREES_MAXIMUM:f32 = 450.0;
const CONTROL_EL_DEGREES_MAXIMUM:f32 = 180.0;

static CURRENT_AZ_EL_RAW: Mutex<ThreadModeRawMutex, RefCell<(f32, f32)>> = Mutex::new(RefCell::new((0.0, 0.0)));
static CURRENT_AZ_EL_DEGREES: Mutex<ThreadModeRawMutex, RefCell<(f32, f32)>> = Mutex::new(RefCell::new((0.0, 0.0)));
static DEMAND_RUN_AZ_EL_DEGREES: Mutex<ThreadModeRawMutex, RefCell<(bool, f32, f32)>> = Mutex::new(RefCell::new((false, 0.0, 0.0)));

assign_resources! {
    azel_adc: AzElAdc {
        adc: ADC,
        dma: DMA_CH2,
        pin_az: PIN_26,
        pin_el: PIN_27
    },
    azel_control: AzElControl {
        pin_az_cw: PIN_2,
        pin_az_ccw: PIN_3,
        pin_el_up: PIN_4,
        pin_el_dn: PIN_5
    }
}

// W5500 Driver
#[embassy_executor::task]
async fn ethernet_task(
    runner: Runner<
        'static,
        W5500,
        ExclusiveDevice<Spi<'static, SPI0, Async>, Output<'static>, Delay>,
        Input<'static>,
        Output<'static>,
    >,
) -> ! {
    runner.run().await
}

// Network Driver
#[embassy_executor::task]
async fn net_task(mut runner: embassy_net::Runner<'static, Device<'static>>) -> ! {
    runner.run().await
}

// AzEL Control Driver
#[embassy_executor::task]
async fn control_task(_spawner: Spawner, r: AzElControl) {
    let mut az_cw = Output::new(r.pin_az_cw, Level::Low);
    let mut az_ccw = Output::new(r.pin_az_ccw, Level::Low);
    let mut el_up = Output::new(r.pin_el_up, Level::Low);
    let mut el_dn = Output::new(r.pin_el_dn, Level::Low);

    let mut local_current_az_degrees:f32;
    let mut local_current_el_degrees:f32;

    let mut local_demand_run:bool;
    let mut local_demand_az_degrees:f32;
    let mut local_demand_el_degrees:f32;

    let mut flag_ontarget:bool;

    let mut ticker = Ticker::every(Duration::from_millis(250));
    loop
    {
        (local_demand_run, local_demand_az_degrees, local_demand_el_degrees) = DEMAND_RUN_AZ_EL_DEGREES.lock(|f| f.clone().into_inner());

        if local_demand_run == true
        {
            (local_current_az_degrees, local_current_el_degrees) = CURRENT_AZ_EL_DEGREES.lock(|f| f.clone().into_inner());
            flag_ontarget = true;

            // Azimuth
            if local_current_az_degrees < (local_demand_az_degrees - CONTROL_DEGREES_THRESHOLD)
            {
                az_ccw.set_low();
                az_cw.set_high();
                flag_ontarget = false;
            }
            else if local_current_az_degrees > (local_demand_az_degrees + CONTROL_DEGREES_THRESHOLD)
            {
                az_cw.set_low();
                az_ccw.set_high();
                flag_ontarget = false;
            }
            else
            {
                az_cw.set_low();
                az_ccw.set_low();
            }

            // Elevation
            if local_current_el_degrees < (local_demand_el_degrees - CONTROL_DEGREES_THRESHOLD)
            {
                el_dn.set_low();
                el_up.set_high();
                flag_ontarget = false;
            }
            else if local_current_el_degrees > (local_demand_el_degrees + CONTROL_DEGREES_THRESHOLD)
            {
                el_up.set_low();
                el_dn.set_high();
                flag_ontarget = false;
            }
            else
            {
                el_up.set_low();
                el_dn.set_low();
            }

            if flag_ontarget == true
            {
                // Reset RUN to stopped to prevent more driving
                DEMAND_RUN_AZ_EL_DEGREES.lock(|f| {
                    let (_, _az, _el) = f.clone().into_inner();
                    f.replace((false, _az, _el));
                });
            }
        }
        else
        {
            // Stopped (by command, or on target)
            az_cw.set_low();
            az_ccw.set_low();
            el_up.set_low();
            el_dn.set_low();
        }

        ticker.next().await;
    }

}

use embassy_rp::adc::{Adc, Channel, Config, InterruptHandler};
use embassy_rp::bind_interrupts;

bind_interrupts!(struct Irqs {
    ADC_IRQ_FIFO => InterruptHandler;
});

// AzEl ADC Driver
#[embassy_executor::task]
async fn adc_task(_spawner: Spawner, r: AzElAdc) {

    // Either

    const UPPER_RESISTOR_K:f32 = 10.0;
    const LOWER_RESISTOR_K:f32 = 10.0;

    const LADDER_RATIO:f32 = LOWER_RESISTOR_K/(UPPER_RESISTOR_K+LOWER_RESISTOR_K);

    const VREF_V:f32 = 3.3;

    // Manuals say 2.0-4.5V
    //const G5500_VOLTAGE_LOW:f32 = 2.0;
    //const G5500_VOLTAGE_HIGH:f32 = 4.5;
    // Rotators are actually 0-5v
    const G5500_VOLTAGE_LOW:f32 = 0.0;
    const G5500_VOLTAGE_HIGH:f32 = 5.0;

    const ADC_RAW_AZ_LOW:f32 = G5500_VOLTAGE_LOW * LADDER_RATIO * (4096.0/VREF_V);
    const ADC_RAW_EL_LOW:f32 = G5500_VOLTAGE_LOW * LADDER_RATIO * (4096.0/VREF_V);

    const ADC_RAW_AZ_HIGH:f32 = G5500_VOLTAGE_HIGH * LADDER_RATIO * (4096.0/VREF_V);
    const ADC_RAW_EL_HIGH:f32 = G5500_VOLTAGE_HIGH * LADDER_RATIO * (4096.0/VREF_V);

    // or

    /*const ADC_RAW_AZ_0:u16 = ??;
    const ADC_RAW_AZ_360:u16 = ??;

    const ADC_RAW_EL_0:u16 = ??;
    const ADC_RAW_EL_180:u16 = ??;

    const ADC_RAW_AZ_LOW:f32 = ADC_RAW_AZ_0 as f32;
    const ADC_RAW_AZ_HIGH:f32 = ADC_RAW_AZ_360 as f32 * (450.0/360.0);

    const ADC_RAW_EL_LOW:f32 = ADC_RAW_EL_0 as f32;
    const ADC_RAW_EL_HIGH:f32 = ADC_RAW_EL_180 as f32;*/

    let mut adc = Adc::new(r.adc, Irqs, Config::default());
    let mut dma = r.dma;
    let mut pins = [
        Channel::new_pin(r.pin_az, Pull::None),
        Channel::new_pin(r.pin_el, Pull::None)
    ];

    const NUM_SAMPLES:usize = 512;
    // 512 samples * 2 to take approx 100ms, so 0.1ms/sample = 10kHz sample rate
    const DIV:u16 = 2399;
    let mut buf = [0_u16; { NUM_SAMPLES * 2 }];

    let mut az_sum:u32 ;
    let mut az_count:u32 ;
    let mut el_sum:u32 ;
    let mut el_count:u32;

    let mut candidate_az_raw:f32;
    let mut candidate_el_raw:f32;

    let mut candidate_az_degrees:f32;
    let mut candidate_el_degrees:f32;

    let mut ticker = Ticker::every(Duration::from_millis(100));
    loop {
        adc.read_many_multichannel(&mut pins, &mut buf, DIV, &mut dma)
            .await
            .unwrap();

        // Count is used to allow us to skip samples if we hit the documented DNL spikes.
        az_sum = 0;
        az_count = 0;
        el_sum = 0;
        el_count = 0;

        for i in 0..NUM_SAMPLES
        {
            // Azimuth
            if buf[2*i] != 512
                && buf[2*i] != 1536
                && buf[2*i] != 2560
                && buf[2*i] != 3584
            {
                az_sum += buf[2*i] as u32;
                az_count += 1;
            }

            // Elevation
            if buf[(2*i)+1] != 512
                && buf[(2*i)+1] != 1536
                && buf[(2*i)+1] != 2560
                && buf[(2*i)+1] != 3584
            {
                el_sum += buf[(2*i)+1] as u32;
                el_count += 1;
            }
        }

        // Azimuth
        candidate_az_raw = az_sum as f32 / az_count as f32;
        candidate_az_degrees = (candidate_az_raw - ADC_RAW_AZ_LOW) / ((ADC_RAW_AZ_HIGH-ADC_RAW_AZ_LOW)/450.0);

        if candidate_az_degrees < 0.0 { candidate_az_degrees = 0.0; }
        if candidate_az_degrees > 450.0 { candidate_az_degrees = 450.0; }

        // Elevation
        candidate_el_raw = el_sum as f32 / el_count as f32;
        candidate_el_degrees = (candidate_el_raw - ADC_RAW_EL_LOW) / ((ADC_RAW_EL_HIGH-ADC_RAW_EL_LOW)/180.0);

        if candidate_el_degrees < 0.0 { candidate_el_degrees = 0.0; }
        if candidate_el_degrees > 180.0 { candidate_el_degrees = 180.0; }

        // Update global current AzEl
        CURRENT_AZ_EL_DEGREES.lock(|f| {
            f.replace((candidate_az_degrees, candidate_el_degrees));
        });

        // Update global raw values
        CURRENT_AZ_EL_RAW.lock(|f| {
            f.replace((candidate_az_raw, candidate_el_raw));
        });

        ticker.next().await;
    }
}



#[derive(PartialEq, Eq)]
enum HamlibCommand {
    GetInfo,
    GetPos,
    Stop,
    Park,
    SetPos,
    Quit, // unofficial but used by gpredict?
    DumpState,
    Reset,
    _None
}

#[derive(PartialEq, Eq)]
struct Command {
    command_type: HamlibCommand
}
impl Command {
    // _, \get_info
    fn parse_get_info(input: &[u8]) -> IResult<&[u8], &[u8]> {
        alt((
            tag("_"), tag("\\get_info")
        )).parse(input)
    }
    // p, \get_pos
    fn parse_get_pos(input: &[u8]) -> IResult<&[u8], &[u8]> {
        alt((
            tag("p"), tag("\\get_pos")
        )).parse(input)
    }
    // S, \stop
    fn parse_stop(input: &[u8]) -> IResult<&[u8], &[u8]> {
        alt((
            tag("S"), tag("\\stop")
        )).parse(input)
    }
    // K, \park
    fn parse_park(input: &[u8]) -> IResult<&[u8], &[u8]> {
        alt((
            tag("K"), tag("\\park")
        )).parse(input)
    }
    // P 180.00 45.00, \set_pos 180.00 45.00
    fn parse_set_pos(input: &[u8]) -> IResult<&[u8], (f32, f32)> {
        preceded(
            pair(
                alt((tag("P"), tag("\\set_pos"))),
                space1
            ),
            separated_pair(float, space1, float)
        ).parse(input)
    }
    // q, \quit
    fn parse_quit(input: &[u8]) -> IResult<&[u8], &[u8]> {
        alt((
            tag("q"), tag("\\quit")
        )).parse(input)
    }
    // dump_state
    fn parse_dump_state(input: &[u8]) -> IResult<&[u8], &[u8]> {
        tag("\\dump_state")(input)
    }
    // R, \reset
    fn parse_reset(input: &[u8]) -> IResult<&[u8], &[u8]> {
        alt((
            tag("R"), tag("\\reset")
        )).parse(input)
    }
    fn parse(input: &[u8]) -> (HamlibCommand, f32, f32) {
        let _ = match Self::parse_get_info(input) {
            Ok(_) => return (HamlibCommand::GetInfo, 0.0, 0.0),
            Err(_) => {}
        };
        let _ = match Self::parse_get_pos(input) {
            Ok(_) => return (HamlibCommand::GetPos, 0.0, 0.0),
            Err(_) => {}
        };
        let _ = match Self::parse_stop(input) {
            Ok(_) => return (HamlibCommand::Stop, 0.0, 0.0),
            Err(_) => {}
        };
        let _ = match Self::parse_park(input) {
            Ok(_) => return (HamlibCommand::Park, 0.0, 0.0),
            Err(_) => {}
        };
        let _ = match Self::parse_set_pos(input) {
            Ok((_, (az, el))) => return (HamlibCommand::SetPos, az, el),
            Err(_) => {}
        };
        let _ = match Self::parse_quit(input) {
            Ok(_) => return (HamlibCommand::Quit, 0.0, 0.0),
            Err(_) => {}
        };
        let _ = match Self::parse_dump_state(input) {
            Ok(_) => return (HamlibCommand::DumpState, 0.0, 0.0),
            Err(_) => {}
        };
        let _ = match Self::parse_reset(input) {
            Ok(_) => return (HamlibCommand::Reset, 0.0, 0.0),
            Err(_) => {}
        };

        return (HamlibCommand::_None, 0.0, 0.0);
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    // System LED - Blinky
    let mut sys_led = Output::new(p.PIN_6, Level::Low);
    sys_led.set_high();

    // Watchdog
    let mut watchdog = Watchdog::new(p.WATCHDOG);
    //watchdog.pause_on_debug(true); // Doesn't appear to work?
    watchdog.start(Duration::from_millis(WATCHDOG_PERIOD_MS));

    // Sockets LED - ON if >0 clients connected
    let mut sockets_led = Output::new(p.PIN_7, Level::Low);

    // Flash Driver
    let mut flash = embassy_rp::flash::Flash::<_, embassy_rp::flash::Async, FLASH_SIZE>::new(p.FLASH, p.DMA_CH3);
    let mut uid = [0; 8];
    flash.blocking_unique_id(&mut uid).unwrap();
    FLASH_UUID.lock(|f| {
        f.replace(uid);
    });

    // Wiznet W5500 SPI Interface
    let mut spi_cfg = SpiConfig::default();
    spi_cfg.frequency = 50_000_000;
    let (miso, mosi, clk) = (p.PIN_16, p.PIN_19, p.PIN_18);
    let spi = Spi::new(p.SPI0, clk, mosi, miso, p.DMA_CH0, p.DMA_CH1, spi_cfg);
    let cs = Output::new(p.PIN_17, Level::High);
    let w5500_int = Input::new(p.PIN_21, Pull::Up);
    let w5500_reset = Output::new(p.PIN_20, Level::High);

    // Wiznet W5500 Driver
    let mac_addr = [0x02, 0x00, 0x00, 0x00, 0x00, 0x00];
    static STATE: StaticCell<State<8, 8>> = StaticCell::new();
    let state = STATE.init(State::<8, 8>::new());
    let (device, runner) = embassy_net_wiznet::new(
        mac_addr,
        state,
        ExclusiveDevice::new(spi, cs, Delay),
        w5500_int,
        w5500_reset,
    )
    .await
    .unwrap();
    unwrap!(spawner.spawn(ethernet_task(runner)));

    // Network Random Seed
    let mut rng = RoscRng;
    let seed = rng.next_u64();

    // DHCP Config
    let mut dhcp_config = DhcpConfig::default();
    dhcp_config.hostname = Some(heapless::String::try_from(DHCP_HOSTNAME).unwrap());
    let net_config = embassy_net::Config::dhcpv4(dhcp_config);

    // SmolTCP network driver
    static RESOURCES: StaticCell<StackResources<5>> = StaticCell::new();
    let (stack, runner) = embassy_net::new(
        device,
        net_config,
        RESOURCES.init(StackResources::new()),
        seed,
    );
    unwrap!(spawner.spawn(net_task(runner)));

    // Az & El ADC Driver
    let r = split_resources!(p);
    spawner.spawn(adc_task(spawner, r.azel_adc)).unwrap();

    // Az & El Control Driver
    spawner.spawn(control_task(spawner, r.azel_control)).unwrap();

    // Feed watchdog either side of DHCP task
    watchdog.feed();

    info!("Waiting for DHCP...");
    let cfg = wait_for_dhcp_config(stack).await;
    let local_addr = cfg.address.address();
    info!("IP address: {:?}", local_addr);

    // Feed watchdog either side of DHCP task
    watchdog.feed();

    // Spawn TCP Sockets
    for _ in 0..NUMBER_HAMLIB_SOCKETS {
        unwrap!(spawner.spawn(listen_task(stack, 4533)));
    }

    let mut local_sockets_connected;
    let mut ticker = Ticker::every(Duration::from_millis(250));
    loop {
        // Toggle System Heartbeat LED
        sys_led.toggle();

        // Set Socket-Connected LED
        local_sockets_connected = SOCKETS_CONNECTED.lock(|f| f.clone().into_inner());
        if local_sockets_connected > 0
        {
            sockets_led.set_high();
        }
        else
        {
            sockets_led.set_low();
        }

        // Reset system if requested, else feed watchdog
        if WATCHDOG_RESET_SYSTEM.lock(|f| f.clone().into_inner())
        {
            watchdog.trigger_reset();
        }
        watchdog.feed();

        ticker.next().await;
    }
}
    

#[embassy_executor::task(pool_size = 4)]
async fn listen_task(stack: Stack<'static>, port: u16) {
    let mut rx_buffer = [0; 4096];
    let mut tx_buffer = [0; 4096];
    let mut buf = [0; 4096];

    loop {
        let mut socket = embassy_net::tcp::TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
        socket.set_timeout(Some(Duration::from_secs(SOCKET_TIMEOUT_S)));

        if let Err(e) = socket.accept(port).await
        {
            warn!("TCP accept error: {:?}", e);
            continue;
        }

        // Connected.
        // Remote IP: socket.remote_endpoint()

        SOCKETS_CONNECTED.lock(|f| {
            let socket_count = f.clone().into_inner();
            f.replace(socket_count + 1);
        });

        loop {
            let n = match socket.read(&mut buf).await {
                Ok(0) => {
                    warn!("TCP listen error: read EOF");
                    break;
                }
                Ok(n) => n,
                Err(e) => {
                    warn!("TCP listen error: {:?}", e);
                    break;
                }
            };

            let (response, mut demand_az, mut demand_el) = Command::parse(&buf[..n]);

            match response {
                HamlibCommand::GetInfo => {
                    info!("Parsed get_info!");

                    let mut buf = [0u8; 128];

                    let _ = format_no_std::show(
                        &mut buf,
                        format_args!("Info \"{}, firmware: {}\"\n",
                            PRODUCT_NAME, GIT_VERSION
                        ),
                    ).unwrap();

                    let _ = socket.write(&buf).await;
                },
                HamlibCommand::GetPos => {
                    info!("Parsed get_pos!");

                    let mut buf = [0u8; 15];
                    let (local_az_degrees, local_el_degrees) = CURRENT_AZ_EL_DEGREES.lock(|f| f.clone().into_inner());
                    let _ = format_no_std::show(
                        &mut buf,
                        format_args!("{:.2}\n{:.2}\n", local_az_degrees, local_el_degrees),
                    ).unwrap();

                    let _ = socket.write(&buf).await;
                },
                HamlibCommand::Stop => {
                    info!("Parsed stop!");

                    DEMAND_RUN_AZ_EL_DEGREES.lock(|f| {
                        let (_, _az, _el) = f.clone().into_inner();
                        f.replace((false, _az, _el));
                    });

                    let _ = socket.write(b"RPRT 0\n").await;
                },
                HamlibCommand::Park => {
                    info!("Parsed park!");

                    DEMAND_RUN_AZ_EL_DEGREES.lock(|f| {
                        f.replace((true, PARK_AZ_DEGREES, PARK_EL_DEGREES));
                    });

                    let _ = socket.write(b"RPRT 0\n").await;
                },
                HamlibCommand::SetPos => {
                    info!("Parsed Set Pos! ({}, {})", demand_az, demand_el);

                    if demand_az < 0.0 { demand_az = 0.0; }
                    if demand_az > CONTROL_AZ_DEGREES_MAXIMUM { demand_az = CONTROL_AZ_DEGREES_MAXIMUM; }

                    if demand_el < 0.0 { demand_el = 0.0; }
                    if demand_el > CONTROL_EL_DEGREES_MAXIMUM { demand_el = CONTROL_EL_DEGREES_MAXIMUM; }

                    DEMAND_RUN_AZ_EL_DEGREES.lock(|f| {
                        f.replace((true, demand_az, demand_el));
                    });

                    let _ = socket.write(b"RPRT 0\n").await;
                },
                HamlibCommand::Quit => {
                    info!("Parsed quit!");

                    // Close socket
                    break;
                },
                HamlibCommand::DumpState => {
                    info!("Parsed dump state!");

                    let mut buf = [0u8; 768];

                    let local_flash_uuid = FLASH_UUID.lock(|f| f.clone().into_inner());
                    let local_sockets_connected = SOCKETS_CONNECTED.lock(|f| f.clone().into_inner());
                    let (local_az_degrees, local_el_degrees) = CURRENT_AZ_EL_DEGREES.lock(|f| f.clone().into_inner());
                    let (local_run, local_demand_az_degrees, local_demand_el_degrees) = DEMAND_RUN_AZ_EL_DEGREES.lock(|f| f.clone().into_inner());
                    let (local_az_raw, local_el_raw) = CURRENT_AZ_EL_RAW.lock(|f| f.clone().into_inner());

                    let _ = format_no_std::show(
                        &mut buf,
                        format_args!("\
                            Product Name: {}\n\
                            Firmware Version: {}\n\
                            Flash UUID: {:02x}{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}\n\
                            Uptime: {}s\n\
                            Connected Clients: {}/{}\n\
                            Current Azimuth: {:.2}° [raw: {:.1}/4096]\n\
                            Current Elevation: {:.2}° [raw: {:.1}/4096]\n\
                            Demand: RUN: {}, Az: {:.2}° El: {:.2}°\n",
                            PRODUCT_NAME,
                            GIT_VERSION,
                            local_flash_uuid[0], local_flash_uuid[1], local_flash_uuid[2], local_flash_uuid[3], local_flash_uuid[4], local_flash_uuid[5], local_flash_uuid[6], local_flash_uuid[7],
                            Instant::now().as_secs(),
                            local_sockets_connected, NUMBER_HAMLIB_SOCKETS,
                            local_az_degrees, local_az_raw,
                            local_el_degrees, local_el_raw,
                            local_run, local_demand_az_degrees, local_demand_el_degrees
                        ),
                    ).unwrap();

                    let _ = socket.write(&buf).await;
                },
                HamlibCommand::Reset => {
                    info!("Parsed reset!");

                    // Raise flag for main thread to starve the watchdog
                    WATCHDOG_RESET_SYSTEM.lock(|f| {
                        f.replace(true);
                    });

                    break;
                },
                HamlibCommand::_None => {
                    //info!("Failed to parse: {}", core::str::from_utf8(&buf[..n]).unwrap());

                    let _ = socket.write(b"RPRT 1\n").await;
                }
            }
        }

        // Disconnected.

        SOCKETS_CONNECTED.lock(|f| {
            let socket_count = f.clone().into_inner();
            f.replace(socket_count - 1);
        });
    }
}

async fn wait_for_dhcp_config(stack: Stack<'static>) -> embassy_net::StaticConfigV4 {
    loop {
        if let Some(config) = stack.config_v4() {
            return config.clone();
        }
        yield_now().await;
    }
}
