## What the example does

It initializes SPI bus and the driver. While in a loop, show card number when
detected an RFID card.

```
I (29) boot: ESP-IDF v4.0-dev-1446-g6b169d473 2nd stage bootloader
I (29) boot: compile time 16:03:28
I (29) boot: Enabling RNG early entropy source...
I (35) boot: SPI Speed      : 40MHz
I (39) boot: SPI Mode       : DIO
I (43) boot: SPI Flash Size : 2MB
I (47) boot: Partition Table:
I (51) boot: ## Label            Usage          Type ST Offset   Length
I (58) boot:  0 nvs              WiFi data        01 02 00009000 00006000
I (65) boot:  1 phy_init         RF data          01 01 0000f000 00001000
I (73) boot:  2 factory          factory app      00 00 00010000 00100000
I (80) boot: End of partition table
I (85) esp_image: segment 0: paddr=0x00010020 vaddr=0x3f400020 size=0x0752c ( 29996) map
I (104) esp_image: segment 1: paddr=0x00017554 vaddr=0x3ffb0000 size=0x0201c (  8220) load
I (108) esp_image: segment 2: paddr=0x00019578 vaddr=0x40080000 size=0x00400 (  1024) load
I (112) esp_image: segment 3: paddr=0x00019980 vaddr=0x40080400 size=0x06690 ( 26256) load
I (131) esp_image: segment 4: paddr=0x00020018 vaddr=0x400d0018 size=0x15268 ( 86632) map
I (162) esp_image: segment 5: paddr=0x00035288 vaddr=0x40086a90 size=0x03e34 ( 15924) load
I (176) boot: Loaded app from partition at offset 0x10000
I (176) boot: Disabling RNG early entropy source...
I (177) cpu_start: Pro cpu up.
I (180) cpu_start: Application information:
I (185) cpu_start: Project name:     hello-world
I (190) cpu_start: App version:      V01-8-g30325fc
I (196) cpu_start: Compile time:     Aug 25 2019 16:03:33
I (202) cpu_start: ELF file SHA256:  9a58878941fe8ecd...
I (208) cpu_start: ESP-IDF:          v4.0-dev-1446-g6b169d473
I (214) cpu_start: Starting app cpu, entry point is 0x40081054
I (0) cpu_start: App cpu up.
I (225) heap_init: Initializing. RAM available for dynamic allocation:
I (232) heap_init: At 3FFAE6E0 len 00001920 (6 KiB): DRAM
I (238) heap_init: At 3FFB3188 len 0002CE78 (179 KiB): DRAM
I (244) heap_init: At 3FFE0440 len 00003AE0 (14 KiB): D/IRAM
I (250) heap_init: At 3FFE4350 len 0001BCB0 (111 KiB): D/IRAM
I (257) heap_init: At 4008A8C4 len 0001573C (85 KiB): IRAM
I (263) cpu_start: Pro cpu start user code
I (281) spi_flash: detected chip: generic
I (282) spi_flash: flash io: dio
W (282) spi_flash: Detected size(4096k) larger than the size in the binary image header(2048k). Using the size in the binary image header.
I (293) cpu_start: Starting scheduler on PRO CPU.
I (0) cpu_start: Starting scheduler on APP CPU.
Hello world!
This is ESP32 chip with 2 CPU cores, WiFi/BT/BLE, silicon revision 0, 2MB external flash
I (322) example_app: Initializing SPI
I (322) example_app: Initializing driver
I (432) example_app: Starting the loop
Card present
CardNo = [9421b1ab][2485236139]

Card present
CardNo = [9421b1ab][2485236139]
```

## Connecting

MFRC522 chip supports SPI and I2C. However, most of modules in the market
defaults to SPI (commonly used module name is `RFID-RC522`, printed on the
module).

| MFRC522 module | ESP32  |
|----------------|--------|
| SDA            | GPIO21 |
| SCK            | GPIO18 |
| MOSI           | GPIO23 |
| MISO           | GPIO19 |
| IRQ            | Not Connected (the example does not use IRQ) |
| RST            | GPIO22 |
| GND            | GND    |
| 3.3V           | 3.3V   |

Note that the library hard-coded GPIO numbers used for SPI.

