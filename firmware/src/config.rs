

use embassy_rp::flash::{Async, ERASE_SIZE, FLASH_BASE};

const FLASH_SIZE:usize = 2 * 1024 * 1024;
const ADDR_OFFSET: u32 = 0x100000;

const FLASH_CONFIG_OFFSET:usize = 1920_000; // 1920K
const FLASH_CONFIG_LENGTH:usize = 8;

const CONFIG_MAGIC_BYTE:u8 = 0xAE;

const DEFAULT_AZ_LOWER_RAW:u16 = 0;

mod crc8_ccitt;


#[allow(dead_code)]
pub fn multiwrite_bytes(flash: &mut embassy_rp::flash::Flash<'_, FLASH, Async, FLASH_SIZE>) {
    info!(">>>> [multiwrite_bytes]");
    let mut read_buf = [0u8; 16];

    flash.blocking_read(ADDR_OFFSET + FLASH_CONFIG_OFFSET, &mut read_buf);

    if read_buf[0] == CONFIG_MAGIC_BYTE && crc8_ccitt_validate_buffer(read_buf, 16)
    {
    	/* Valid config */

    }
    else
    {
    	/* Uninitialised or corrupted */
    	flash.blocking_erase(ADDR_OFFSET + FLASH_CONFIG_OFFSET, ADDR_OFFSET + FLASH_CONFIG_OFFSET + ERASE_SIZE as u32);

    	let mut write_buf:[u8] = [ CONFIG_MAGIC_BYTE, DEFAULT_AZ_LOWER_RAW,  0x00];
    	write_buf[2] = crc8_ccitt_buffer(write_buf, 2);

    	flash.blocking_write(ADDR_OFFSET + FLASH_CONFIG_OFFSET, &[CONFIG_MAGIC_BYTE]);
    	flash.blocking_write(ADDR_OFFSET + FLASH_CONFIG_OFFSET + 1, &[(DEFAULT_AZ_LOWER_RAW << 8) as u8]);
    	flash.blocking_write(ADDR_OFFSET + FLASH_CONFIG_OFFSET + 1, &[(DEFAULT_AZ_LOWER_RAW && 0xFF) as u8]);


    }

    info!("Addr of flash block is {:x}", ADDR_OFFSET + FLASH_CONFIG_OFFSET + FLASH_BASE as u32);
    info!("Contents start with {=[u8]}", read_buf[0..4]);

    defmt::unwrap!(flash.blocking_erase(ADDR_OFFSET + FLASH_CONFIG_OFFSET, ADDR_OFFSET + FLASH_CONFIG_OFFSET + ERASE_SIZE as u32));

    defmt::unwrap!(flash.blocking_read(ADDR_OFFSET + FLASH_CONFIG_OFFSET, &mut read_buf));
    info!("Contents after erase starts with {=[u8]}", read_buf[0..4]);
    if read_buf.iter().any(|x| *x != 0xFF) {
        defmt::panic!("unexpected");
    }

    defmt::unwrap!(flash.blocking_write(ADDR_OFFSET + FLASH_CONFIG_OFFSET, &[0x01]));
    defmt::unwrap!(flash.blocking_write(ADDR_OFFSET + FLASH_CONFIG_OFFSET + 1, &[0x02]));
    defmt::unwrap!(flash.blocking_write(ADDR_OFFSET + FLASH_CONFIG_OFFSET + 2, &[0x03]));
    defmt::unwrap!(flash.blocking_write(ADDR_OFFSET + FLASH_CONFIG_OFFSET + 3, &[0x04]));

    defmt::unwrap!(flash.blocking_read(ADDR_OFFSET + FLASH_CONFIG_OFFSET, &mut read_buf));
    info!("Contents after write starts with {=[u8]}", read_buf[0..4]);
    if &read_buf[0..4] != &[0x01, 0x02, 0x03, 0x04] {
        defmt::panic!("unexpected");
    }
}