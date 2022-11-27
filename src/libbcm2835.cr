@[Link(ldflags: "-L ~/software/lib/ -lbcm2835")]
lib LibBcm2835
  alias Bool = LibC::Int
  alias Char = LibC::Char
  alias Double = LibC::Double
  alias Float = LibC::Float
  alias Int = LibC::Int
  alias UInt = LibC::UInt
  alias Long = LibC::Long
  alias SizeT = LibC::SizeT
  alias OffT = LibC::OffT
  alias UInt8T = LibC::UInt8T
  alias UInt16T = LibC::UInt16T
  alias UInt32T = LibC::UInt32T
  alias UInt64T = LibC::UInt64T

  VERSION = 10071

  # This means pin HIGH, true, 3.3volts on a pin.
  HIGH = 0x1
  # This means pin LOW, false, 0volts on a pin.
  LOW = 0x0

  # Speed of the core clock core_clk
  CORE_CLK_HZ = 250000000 #	250 MHz

  # On all recent OSs, the base of the peripherals is read from a /proc file #
  RPI2_DT_FILENAME = "/proc/device-tree/soc/ranges"

  # Physical addresses for various peripheral register sets
  #   Base Physical Address of the BCM 2835 peripheral registers
  #   Note this is different for the RPi2 BCM2836, where this is derived from /proc/device-tree/soc/ranges
  #   If /proc/device-tree/soc/ranges exists on a RPi 1 OS, it would be expected to contain the
  #   following numbers:

  # Peripherals block base address on RPi 1 #
  PERI_BASE = 0x20000000
  # Size of the peripherals block on RPi 1 #
  PERI_SIZE = 0x01000000
  # Alternate base address for RPI  2 / 3 #
  RPI2_PERI_BASE = 0x3F000000
  # Alternate base address for RPI  4 #
  RPI4_PERI_BASE = 0xFE000000
  # Alternate size for RPI  4 #
  RPI4_PERI_SIZE = 0x01800000

  # Offsets for the bases of various peripherals within the peripherals block
  #   Base Address of the System Timer registers

  ST_BASE = 0x3000
  # Base Address of the Pads registers #
  GPIO_PADS = 0x100000
  # Base Address of the Clock/timer registers #
  CLOCK_BASE = 0x101000
  # Base Address of the GPIO registers #
  GPIO_BASE = 0x200000
  # Base Address of the SPI0 registers #
  SPI0_BASE = 0x204000
  # Base Address of the BSC0 registers #
  BSC0_BASE = 0x205000
  # Base Address of the PWM registers #
  GPIO_PWM = 0x20C000
  # Base Address of the AUX registers #
  AUX_BASE = 0x215000
  # Base Address of the AUX_SPI1 registers #
  SPI1_BASE = 0x215080
  # Base Address of the AUX_SPI2 registers #
  SPI2_BASE = 0x2150C0
  # Base Address of the BSC1 registers #
  BSC1_BASE = 0x804000

  # BEB #
  # Base address of the SMI registers #
  SMI_BASE = 0x600000

  #  Register bases for bcm2835_regbase()
  #
  REGBASE_ST   = 1 # < Base of the ST (System Timer) registers. #
  REGBASE_GPIO = 2 # < Base of the GPIO registers. #
  REGBASE_PWM  = 3 # < Base of the PWM registers. #
  REGBASE_CLK  = 4 # < Base of the CLK registers. #
  REGBASE_PADS = 5 # < Base of the PADS registers. #
  REGBASE_SPI0 = 6 # < Base of the SPI0 registers. #
  REGBASE_BSC0 = 7 # < Base of the BSC0 registers. #
  REGBASE_BSC1 = 8 # < Base of the BSC1 registers. #
  REGBASE_AUX  = 9 # < Base of the AUX registers. #
  REGBASE_SPI1 = 1 # < Base of the SPI1 registers. #
  # BEB #
  REGBASE_SMI = 11 # < Base of the SMI registers. #

  # Size of memory page on RPi #
  PAGE_SIZE = (4*1024)
  # Size of memory block on RPi #
  BLOCK_SIZE = (4*1024)

  # Defines for GPIO
  #   The BCM2835 has 54 GPIO pins.
  #   BCM2835 data sheet, Page 90 onwards.

  # GPIO register offsets from GPIO_BASE.
  #  Offsets into the GPIO Peripheral block in bytes per 6.1 Register View

  GPFSEL0   = 0x0000 # < GPIO Function Select 0 #
  GPFSEL1   = 0x0004 # < GPIO Function Select 1 #
  GPFSEL2   = 0x0008 # < GPIO Function Select 2 #
  GPFSEL3   = 0x000c # < GPIO Function Select 3 #
  GPFSEL4   = 0x0010 # < GPIO Function Select 4 #
  GPFSEL5   = 0x0014 # < GPIO Function Select 5 #
  GPSET0    = 0x001c # < GPIO Pin Output Set 0 #
  GPSET1    = 0x0020 # < GPIO Pin Output Set 1 #
  GPCLR0    = 0x0028 # < GPIO Pin Output Clear 0 #
  GPCLR1    = 0x002c # < GPIO Pin Output Clear 1 #
  GPLEV0    = 0x0034 # < GPIO Pin Level 0 #
  GPLEV1    = 0x0038 # < GPIO Pin Level 1 #
  GPEDS0    = 0x0040 # < GPIO Pin Event Detect Status 0 #
  GPEDS1    = 0x0044 # < GPIO Pin Event Detect Status 1 #
  GPREN0    = 0x004c # < GPIO Pin Rising Edge Detect Enable 0 #
  GPREN1    = 0x0050 # < GPIO Pin Rising Edge Detect Enable 1 #
  GPFEN0    = 0x0058 # < GPIO Pin Falling Edge Detect Enable 0 #
  GPFEN1    = 0x005c # < GPIO Pin Falling Edge Detect Enable 1 #
  GPHEN0    = 0x0064 # < GPIO Pin High Detect Enable 0 #
  GPHEN1    = 0x0068 # < GPIO Pin High Detect Enable 1 #
  GPLEN0    = 0x0070 # < GPIO Pin Low Detect Enable 0 #
  GPLEN1    = 0x0074 # < GPIO Pin Low Detect Enable 1 #
  GPAREN0   = 0x007c # < GPIO Pin Async. Rising Edge Detect 0 #
  GPAREN1   = 0x0080 # < GPIO Pin Async. Rising Edge Detect 1 #
  GPAFEN0   = 0x0088 # < GPIO Pin Async. Falling Edge Detect 0 #
  GPAFEN1   = 0x008c # < GPIO Pin Async. Falling Edge Detect 1 #
  GPPUD     = 0x0094 # < GPIO Pin Pull-up/down Enable #
  GPPUDCLK0 = 0x0098 # < GPIO Pin Pull-up/down Enable Clock 0 #
  GPPUDCLK1 = 0x009c # < GPIO Pin Pull-up/down Enable Clock 1 #

  # 2711 has a different method for pin pull-up/down/enable  #
  GPPUPPDN0 = 0x00e4 # Pin pull-up/down for pins 15:0  #
  GPPUPPDN1 = 0x00e8 # Pin pull-up/down for pins 31:16 #
  GPPUPPDN2 = 0x00ec # Pin pull-up/down for pins 47:32 #
  GPPUPPDN3 = 0x00f0 # Pin pull-up/down for pins 57:48 #

  #  Port function select modes for bcm2835_gpio_fsel()
  #
  GPIO_FSEL_INPT = 0x00 # < Input 0b000 #
  GPIO_FSEL_OUTP = 0x01 # < Output 0b001 #
  GPIO_FSEL_ALT0 = 0x04 # < Alternate function 0 0b100 #
  GPIO_FSEL_ALT1 = 0x05 # < Alternate function 1 0b101 #
  GPIO_FSEL_ALT2 = 0x06 # < Alternate function 2 0b110, #
  GPIO_FSEL_ALT3 = 0x07 # < Alternate function 3 0b111 #
  GPIO_FSEL_ALT4 = 0x03 # < Alternate function 4 0b011 #
  GPIO_FSEL_ALT5 = 0x02 # < Alternate function 5 0b010 #
  GPIO_FSEL_MASK = 0x07 # < Function select bits mask 0b111 #

  # \brief bcm2835PUDControl
  #  Pullup/Pulldown defines for bcm2835_gpio_pud()
  #
  GPIO_PUD_OFF  = 0x00 # < Off ? disable pull-up/down 0b00 #
  GPIO_PUD_DOWN = 0x01 # < Enable Pull Down control 0b01 #
  GPIO_PUD_UP   = 0x02 # < Enable Pull Up control 0b10  #

  # need a value for pud functions that can't work unless RPI 4 #
  GPIO_PUD_ERROR = 0x08

  # Pad control register offsets from GPIO_PADS #
  PADS_GPIO_0_27  = 0x002c # < Pad control register for pads 0 to 27 #
  PADS_GPIO_28_45 = 0x0030 # < Pad control register for pads 28 to 45 #
  PADS_GPIO_46_53 = 0x0034 # < Pad control register for pads 46 to 53 #

  # Pad Control masks #
  PAD_PASSWRD             = (0x5A << 24) # < Password to enable setting pad mask #
  PAD_SLEW_RATE_UNLIMITED = 0x10         # < Slew rate unlimited #
  PAD_HYSTERESIS_ENABLED  = 0x08         # < Hysteresis enabled #
  PAD_DRIVE_2mA           = 0x00         # < 2mA drive current #
  PAD_DRIVE_4mA           = 0x01         # < 4mA drive current #
  PAD_DRIVE_6mA           = 0x02         # < 6mA drive current #
  PAD_DRIVE_8mA           = 0x03         # < 8mA drive current #
  PAD_DRIVE_10mA          = 0x04         # < 10mA drive current #
  PAD_DRIVE_12mA          = 0x05         # < 12mA drive current #
  PAD_DRIVE_14mA          = 0x06         # < 14mA drive current #
  PAD_DRIVE_16mA          = 0x07         # < 16mA drive current #

  # \brief bcm2835PadGroup
  #  Pad group specification for bcm2835_gpio_pad()
  #
  PAD_GROUP_GPIO_0_27  = 0 # < Pad group for GPIO pads 0 to 27 #
  PAD_GROUP_GPIO_28_45 = 1 # < Pad group for GPIO pads 28 to 45 #
  PAD_GROUP_GPIO_46_53 = 2 # < Pad group for GPIO pads 46 to 53 #

  #  Here we define Raspberry Pin GPIO pins on P1 in terms of the underlying BCM GPIO pin numbers.
  #  These can be passed as a pin number to any function requiring a pin.
  #  Not all pins on the RPi 26 bin IDE plug are connected to GPIO pins
  #  and some can adopt an alternate function.
  #  RPi version 2 has some slightly different pinouts, and these are values RPI_V2_*.
  #  RPi B+ has yet differnet pinouts and these are defined in RPI_BPLUS_*.
  #  At bootup, pins 8 and 10 are set to UART0_TXD, UART0_RXD (ie the alt0 function) respectively
  #  When SPI0 is in use (ie after bcm2835_spi_begin()), SPI0 pins are dedicated to SPI
  #  and cant be controlled independently.
  #  If you are using the RPi Compute Module, just use the GPIO number: there is no need to use one of these
  #  symbolic names
  #

  RPI_GPIO_P1_03 =  0 # < Version 1, Pin P1-03 #
  RPI_GPIO_P1_05 =  1 # < Version 1, Pin P1-05 #
  RPI_GPIO_P1_07 =  4 # < Version 1, Pin P1-07 #
  RPI_GPIO_P1_08 = 14 # < Version 1, Pin P1-08, defaults to alt function 0 UART0_TXD #
  RPI_GPIO_P1_10 = 15 # < Version 1, Pin P1-10, defaults to alt function 0 UART0_RXD #
  RPI_GPIO_P1_11 = 17 # < Version 1, Pin P1-11 #
  RPI_GPIO_P1_12 = 18 # < Version 1, Pin P1-12, can be PWM channel 0 in ALT FUN 5 #
  RPI_GPIO_P1_13 = 21 # < Version 1, Pin P1-13 #
  RPI_GPIO_P1_15 = 22 # < Version 1, Pin P1-15 #
  RPI_GPIO_P1_16 = 23 # < Version 1, Pin P1-16 #
  RPI_GPIO_P1_18 = 24 # < Version 1, Pin P1-18 #
  RPI_GPIO_P1_19 = 10 # < Version 1, Pin P1-19, MOSI when SPI0 in use #
  RPI_GPIO_P1_21 =  9 # < Version 1, Pin P1-21, MISO when SPI0 in use #
  RPI_GPIO_P1_22 = 25 # < Version 1, Pin P1-22 #
  RPI_GPIO_P1_23 = 11 # < Version 1, Pin P1-23, CLK when SPI0 in use #
  RPI_GPIO_P1_24 =  8 # < Version 1, Pin P1-24, CE0 when SPI0 in use #
  RPI_GPIO_P1_26 =  7 # < Version 1, Pin P1-26, CE1 when SPI0 in use #

  # RPi Version 2 #
  RPI_V2_GPIO_P1_03 =  2 # < Version 2, Pin P1-03 #
  RPI_V2_GPIO_P1_05 =  3 # < Version 2, Pin P1-05 #
  RPI_V2_GPIO_P1_07 =  4 # < Version 2, Pin P1-07 #
  RPI_V2_GPIO_P1_08 = 14 # < Version 2, Pin P1-08, defaults to alt function 0 UART0_TXD #
  RPI_V2_GPIO_P1_10 = 15 # < Version 2, Pin P1-10, defaults to alt function 0 UART0_RXD #
  RPI_V2_GPIO_P1_11 = 17 # < Version 2, Pin P1-11 #
  RPI_V2_GPIO_P1_12 = 18 # < Version 2, Pin P1-12, can be PWM channel 0 in ALT FUN 5 #
  RPI_V2_GPIO_P1_13 = 27 # < Version 2, Pin P1-13 #
  RPI_V2_GPIO_P1_15 = 22 # < Version 2, Pin P1-15 #
  RPI_V2_GPIO_P1_16 = 23 # < Version 2, Pin P1-16 #
  RPI_V2_GPIO_P1_18 = 24 # < Version 2, Pin P1-18 #
  RPI_V2_GPIO_P1_19 = 10 # < Version 2, Pin P1-19, MOSI when SPI0 in use #
  RPI_V2_GPIO_P1_21 =  9 # < Version 2, Pin P1-21, MISO when SPI0 in use #
  RPI_V2_GPIO_P1_22 = 25 # < Version 2, Pin P1-22 #
  RPI_V2_GPIO_P1_23 = 11 # < Version 2, Pin P1-23, CLK when SPI0 in use #
  RPI_V2_GPIO_P1_24 =  8 # < Version 2, Pin P1-24, CE0 when SPI0 in use #
  RPI_V2_GPIO_P1_26 =  7 # < Version 2, Pin P1-26, CE1 when SPI0 in use #
  RPI_V2_GPIO_P1_29 =  5 # < Version 2, Pin P1-29 #
  RPI_V2_GPIO_P1_31 =  6 # < Version 2, Pin P1-31 #
  RPI_V2_GPIO_P1_32 = 12 # < Version 2, Pin P1-32 #
  RPI_V2_GPIO_P1_33 = 13 # < Version 2, Pin P1-33 #
  RPI_V2_GPIO_P1_35 = 19 # < Version 2, Pin P1-35, can be PWM channel 1 in ALT FUN 5  #
  RPI_V2_GPIO_P1_36 = 16 # < Version 2, Pin P1-36 #
  RPI_V2_GPIO_P1_37 = 26 # < Version 2, Pin P1-37 #
  RPI_V2_GPIO_P1_38 = 20 # < Version 2, Pin P1-38 #
  RPI_V2_GPIO_P1_40 = 21 # < Version 2, Pin P1-40 #

  # RPi Version 2, new plug P5 #
  RPI_V2_GPIO_P5_03 = 28 # < Version 2, Pin P5-03 #
  RPI_V2_GPIO_P5_04 = 29 # < Version 2, Pin P5-04 #
  RPI_V2_GPIO_P5_05 = 30 # < Version 2, Pin P5-05 #
  RPI_V2_GPIO_P5_06 = 31 # < Version 2, Pin P5-06 #

  # RPi B+ J8 header, also RPi 2 40 pin GPIO header #
  RPI_BPLUS_GPIO_J8_03 =  2 # < B+, Pin J8-03 #
  RPI_BPLUS_GPIO_J8_05 =  3 # < B+, Pin J8-05 #
  RPI_BPLUS_GPIO_J8_07 =  4 # < B+, Pin J8-07 #
  RPI_BPLUS_GPIO_J8_08 = 14 # < B+, Pin J8-08, defaults to alt function 0 UART0_TXD #
  RPI_BPLUS_GPIO_J8_10 = 15 # < B+, Pin J8-10, defaults to alt function 0 UART0_RXD #
  RPI_BPLUS_GPIO_J8_11 = 17 # < B+, Pin J8-11 #
  RPI_BPLUS_GPIO_J8_12 = 18 # < B+, Pin J8-12, can be PWM channel 0 in ALT FUN 5 #
  RPI_BPLUS_GPIO_J8_13 = 27 # < B+, Pin J8-13 #
  RPI_BPLUS_GPIO_J8_15 = 22 # < B+, Pin J8-15 #
  RPI_BPLUS_GPIO_J8_16 = 23 # < B+, Pin J8-16 #
  RPI_BPLUS_GPIO_J8_18 = 24 # < B+, Pin J8-18 #
  RPI_BPLUS_GPIO_J8_19 = 10 # < B+, Pin J8-19, MOSI when SPI0 in use #
  RPI_BPLUS_GPIO_J8_21 =  9 # < B+, Pin J8-21, MISO when SPI0 in use #
  RPI_BPLUS_GPIO_J8_22 = 25 # < B+, Pin J8-22 #
  RPI_BPLUS_GPIO_J8_23 = 11 # < B+, Pin J8-23, CLK when SPI0 in use #
  RPI_BPLUS_GPIO_J8_24 =  8 # < B+, Pin J8-24, CE0 when SPI0 in use #
  RPI_BPLUS_GPIO_J8_26 =  7 # < B+, Pin J8-26, CE1 when SPI0 in use #
  RPI_BPLUS_GPIO_J8_29 =  5 # < B+, Pin J8-29,  #
  RPI_BPLUS_GPIO_J8_31 =  6 # < B+, Pin J8-31,  #
  RPI_BPLUS_GPIO_J8_32 = 12 # < B+, Pin J8-32,  #
  RPI_BPLUS_GPIO_J8_33 = 13 # < B+, Pin J8-33,  #
  RPI_BPLUS_GPIO_J8_35 = 19 # < B+, Pin J8-35, can be PWM channel 1 in ALT FUN 5 #
  RPI_BPLUS_GPIO_J8_36 = 16 # < B+, Pin J8-36,  #
  RPI_BPLUS_GPIO_J8_37 = 26 # < B+, Pin J8-37,  #
  RPI_BPLUS_GPIO_J8_38 = 20 # < B+, Pin J8-38,  #
  RPI_BPLUS_GPIO_J8_40 = 21 # < B+, Pin J8-40,  #

  # Defines for AUX
  #  GPIO register offsets from AUX_BASE.
  #
  AUX_IRQ    = 0x0000 # < xxx #
  AUX_ENABLE = 0x0004 # < #

  AUX_ENABLE_UART1 = 0x01 # <  #
  AUX_ENABLE_SPI0  = 0x02 # < SPI0 (SPI1 in the device) #
  AUX_ENABLE_SPI1  = 0x04 # < SPI1 (SPI2 in the device) #

  AUX_SPI_CNTL0  = 0x0000 # < #
  AUX_SPI_CNTL1  = 0x0004 # < #
  AUX_SPI_STAT   = 0x0008 # < #
  AUX_SPI_PEEK   = 0x000C # < Read but do not take from FF #
  AUX_SPI_IO     = 0x0020 # < Write = TX, read=RX #
  AUX_SPI_TXHOLD = 0x0030 # < Write = TX keep CS, read=RX #

  AUX_SPI_CLOCK_MIN =     30500 # < 30,5kHz #
  AUX_SPI_CLOCK_MAX = 125000000 # < 125Mhz #

  AUX_SPI_CNTL0_SPEED       = 0xFFF00000 # < #
  AUX_SPI_CNTL0_SPEED_MAX   =      0xFFF # < #
  AUX_SPI_CNTL0_SPEED_SHIFT =         20 # < #

  AUX_SPI_CNTL0_CS0_N = 0x000C0000 # < CS 0 low #
  AUX_SPI_CNTL0_CS1_N = 0x000A0000 # < CS 1 low #
  AUX_SPI_CNTL0_CS2_N = 0x00060000 # < CS 2 low #

  AUX_SPI_CNTL0_POSTINPUT = 0x00010000 # < #
  AUX_SPI_CNTL0_VAR_CS    = 0x00008000 # < #
  AUX_SPI_CNTL0_VAR_WIDTH = 0x00004000 # < #
  AUX_SPI_CNTL0_DOUTHOLD  = 0x00003000 # < #
  AUX_SPI_CNTL0_ENABLE    = 0x00000800 # < #
  AUX_SPI_CNTL0_CPHA_IN   = 0x00000400 # < #
  AUX_SPI_CNTL0_CLEARFIFO = 0x00000200 # < #
  AUX_SPI_CNTL0_CPHA_OUT  = 0x00000100 # < #
  AUX_SPI_CNTL0_CPOL      = 0x00000080 # < #
  AUX_SPI_CNTL0_MSBF_OUT  = 0x00000040 # < #
  AUX_SPI_CNTL0_SHIFTLEN  = 0x0000003F # < #

  AUX_SPI_CNTL1_CSHIGH  = 0x00000700 # < #
  AUX_SPI_CNTL1_IDLE    = 0x00000080 # < #
  AUX_SPI_CNTL1_TXEMPTY = 0x00000040 # < #
  AUX_SPI_CNTL1_MSBF_IN = 0x00000002 # < #
  AUX_SPI_CNTL1_KEEP_IN = 0x00000001 # < #

  AUX_SPI_STAT_TX_LVL   = 0xF0000000 # < #
  AUX_SPI_STAT_RX_LVL   = 0x00F00000 # < #
  AUX_SPI_STAT_TX_FULL  = 0x00000400 # < #
  AUX_SPI_STAT_TX_EMPTY = 0x00000200 # < #
  AUX_SPI_STAT_RX_FULL  = 0x00000100 # < #
  AUX_SPI_STAT_RX_EMPTY = 0x00000080 # < #
  AUX_SPI_STAT_BUSY     = 0x00000040 # < #
  AUX_SPI_STAT_BITCOUNT = 0x0000003F # < #

  # Defines for SPI
  #   GPIO register offsets from SPI0_BASE.
  #   Offsets into the SPI Peripheral block in bytes per 10.5 SPI Register Map
  #
  SPI0_CS   = 0x0000 # < SPI Master Control and Status #
  SPI0_FIFO = 0x0004 # < SPI Master TX and RX FIFOs #
  SPI0_CLK  = 0x0008 # < SPI Master Clock Divider #
  SPI0_DLEN = 0x000c # < SPI Master Data Length #
  SPI0_LTOH = 0x0010 # < SPI LOSSI mode TOH #
  SPI0_DC   = 0x0014 # < SPI DMA DREQ Controls #

  # Register masks for SPI0_CS #
  SPI0_CS_LEN_LONG = 0x02000000 # < Enable Long data word in Lossi mode if DMA_LEN is set #
  SPI0_CS_DMA_LEN  = 0x01000000 # < Enable DMA mode in Lossi mode #
  SPI0_CS_CSPOL2   = 0x00800000 # < Chip Select 2 Polarity #
  SPI0_CS_CSPOL1   = 0x00400000 # < Chip Select 1 Polarity #
  SPI0_CS_CSPOL0   = 0x00200000 # < Chip Select 0 Polarity #
  SPI0_CS_RXF      = 0x00100000 # < RXF - RX FIFO Full #
  SPI0_CS_RXR      = 0x00080000 # < RXR RX FIFO needs Reading (full) #
  SPI0_CS_TXD      = 0x00040000 # < TXD TX FIFO can accept Data #
  SPI0_CS_RXD      = 0x00020000 # < RXD RX FIFO contains Data #
  SPI0_CS_DONE     = 0x00010000 # < Done transfer Done #
  SPI0_CS_TE_EN    = 0x00008000 # < Unused #
  SPI0_CS_LMONO    = 0x00004000 # < Unused #
  SPI0_CS_LEN      = 0x00002000 # < LEN LoSSI enable #
  SPI0_CS_REN      = 0x00001000 # < REN Read Enable #
  SPI0_CS_ADCS     = 0x00000800 # < ADCS Automatically Deassert Chip Select #
  SPI0_CS_INTR     = 0x00000400 # < INTR Interrupt on RXR #
  SPI0_CS_INTD     = 0x00000200 # < INTD Interrupt on Done #
  SPI0_CS_DMAEN    = 0x00000100 # < DMAEN DMA Enable #
  SPI0_CS_TA       = 0x00000080 # < Transfer Active #
  SPI0_CS_CSPOL    = 0x00000040 # < Chip Select Polarity #
  SPI0_CS_CLEAR    = 0x00000030 # < Clear FIFO Clear RX and TX #
  SPI0_CS_CLEAR_RX = 0x00000020 # < Clear FIFO Clear RX  #
  SPI0_CS_CLEAR_TX = 0x00000010 # < Clear FIFO Clear TX  #
  SPI0_CS_CPOL     = 0x00000008 # < Clock Polarity #
  SPI0_CS_CPHA     = 0x00000004 # < Clock Phase #
  SPI0_CS_CS       = 0x00000003 # < Chip Select #

  #  Specifies the SPI data bit ordering for bcm2835_spi_setBitOrder()
  #
  SPI_BIT_ORDER_LSBFIRST = 0 # < LSB First #
  SPI_BIT_ORDER_MSBFIRST = 1 # < MSB First #

  #  Specify the SPI data mode to be passed to bcm2835_spi_setDataMode()
  #
  SPI_MODE0 = 0 # < CPOL = 0, CPHA = 0 #
  SPI_MODE1 = 1 # < CPOL = 0, CPHA = 1 #
  SPI_MODE2 = 2 # < CPOL = 1, CPHA = 0 #
  SPI_MODE3 = 3 # < CPOL = 1, CPHA = 1 #

  #  Specify the SPI chip select pin(s)
  #
  SPI_CS0     = 0 # < Chip Select 0 #
  SPI_CS1     = 1 # < Chip Select 1 #
  SPI_CS2     = 2 # < Chip Select 2 (ie pins CS1 and CS2 are asserted) #
  SPI_CS_NONE = 3 # < No CS, control it yourself #

  #  Specifies the divider used to generate the SPI clock from the system clock.
  #  Figures below give the divider, clock period and clock frequency.
  #  Clock divided is based on nominal core clock rate of 250MHz on RPi1 and RPi2, and 400MHz on RPi3.
  #  It is reported that (contrary to the documentation) any even divider may used.
  #  The frequencies shown for each divider have been confirmed by measurement on RPi1 and RPi2.
  #  The system clock frequency on RPi3 is different, so the frequency you get from a given divider will be different.
  #  See comments in 'SPI Pins' for information about reliable SPI speeds.
  #  Note: it is possible to change the core clock rate of the RPi 3 back to 250MHz, by putting
  #  \code
  #  core_freq=250
  #  \endcode
  #  in the config.txt
  #
  SPI_CLOCK_DIVIDER_65536 =     0 # < 65536 = 3.814697260kHz on Rpi2, 6.1035156kHz on RPI3 #
  SPI_CLOCK_DIVIDER_32768 = 32768 # < 32768 = 7.629394531kHz on Rpi2, 12.20703125kHz on RPI3 #
  SPI_CLOCK_DIVIDER_16384 = 16384 # < 16384 = 15.25878906kHz on Rpi2, 24.4140625kHz on RPI3 #
  SPI_CLOCK_DIVIDER_8192  =  8192 # < 8192 = 30.51757813kHz on Rpi2, 48.828125kHz on RPI3 #
  SPI_CLOCK_DIVIDER_4096  =  4096 # < 4096 = 61.03515625kHz on Rpi2, 97.65625kHz on RPI3 #
  SPI_CLOCK_DIVIDER_2048  =  2048 # < 2048 = 122.0703125kHz on Rpi2, 195.3125kHz on RPI3 #
  SPI_CLOCK_DIVIDER_1024  =  1024 # < 1024 = 244.140625kHz on Rpi2, 390.625kHz on RPI3 #
  SPI_CLOCK_DIVIDER_512   =   512 # < 512 = 488.28125kHz on Rpi2, 781.25kHz on RPI3 #
  SPI_CLOCK_DIVIDER_256   =   256 # < 256 = 976.5625kHz on Rpi2, 1.5625MHz on RPI3 #
  SPI_CLOCK_DIVIDER_128   =   128 # < 128 = 1.953125MHz on Rpi2, 3.125MHz on RPI3 #
  SPI_CLOCK_DIVIDER_64    =    64 # < 64 = 3.90625MHz on Rpi2, 6.250MHz on RPI3 #
  SPI_CLOCK_DIVIDER_32    =    32 # < 32 = 7.8125MHz on Rpi2, 12.5MHz on RPI3 #
  SPI_CLOCK_DIVIDER_16    =    16 # < 16 = 15.625MHz on Rpi2, 25MHz on RPI3 #
  SPI_CLOCK_DIVIDER_8     =     8 # < 8 = 31.25MHz on Rpi2, 50MHz on RPI3 #
  SPI_CLOCK_DIVIDER_4     =     4 # < 4 = 62.5MHz on Rpi2, 100MHz on RPI3. Dont expect this speed to work reliably. #
  SPI_CLOCK_DIVIDER_2     =     2 # < 2 = 125MHz on Rpi2, 200MHz on RPI3, fastest you can get. Dont expect this speed to work reliably.#
  SPI_CLOCK_DIVIDER_1     =     1 # < 1 = 3.814697260kHz on Rpi2, 6.1035156kHz on RPI3, same as 0/65536 #

  # Defines for I2C
  #   GPIO register offsets from BSC*_BASE.
  #   Offsets into the BSC Peripheral block in bytes per 3.1 BSC Register Map
  #
  BSC_C    = 0x0000 # < BSC Master Control #
  BSC_S    = 0x0004 # < BSC Master Status #
  BSC_DLEN = 0x0008 # < BSC Master Data Length #
  BSC_A    = 0x000c # < BSC Master Slave Address #
  BSC_FIFO = 0x0010 # < BSC Master Data FIFO #
  BSC_DIV  = 0x0014 # < BSC Master Clock Divider #
  BSC_DEL  = 0x0018 # < BSC Master Data Delay #
  BSC_CLKT = 0x001c # < BSC Master Clock Stretch Timeout #

  # Register masks for BSC_C #
  BSC_C_I2CEN   = 0x00008000 # < I2C Enable, 0 = disabled, 1 = enabled #
  BSC_C_INTR    = 0x00000400 # < Interrupt on RX #
  BSC_C_INTT    = 0x00000200 # < Interrupt on TX #
  BSC_C_INTD    = 0x00000100 # < Interrupt on DONE #
  BSC_C_ST      = 0x00000080 # < Start transfer, 1 = Start a new transfer #
  BSC_C_CLEAR_1 = 0x00000020 # < Clear FIFO Clear #
  BSC_C_CLEAR_2 = 0x00000010 # < Clear FIFO Clear #
  BSC_C_READ    = 0x00000001 # <	Read transfer #

  # Register masks for BSC_S #
  BSC_S_CLKT = 0x00000200 # < Clock stretch timeout #
  BSC_S_ERR  = 0x00000100 # < ACK error #
  BSC_S_RXF  = 0x00000080 # < RXF FIFO full, 0 = FIFO is not full, 1 = FIFO is full #
  BSC_S_TXE  = 0x00000040 # < TXE FIFO full, 0 = FIFO is not full, 1 = FIFO is full #
  BSC_S_RXD  = 0x00000020 # < RXD FIFO contains data #
  BSC_S_TXD  = 0x00000010 # < TXD FIFO can accept data #
  BSC_S_RXR  = 0x00000008 # < RXR FIFO needs reading (full) #
  BSC_S_TXW  = 0x00000004 # < TXW FIFO needs writing (full) #
  BSC_S_DONE = 0x00000002 # < Transfer DONE #
  BSC_S_TA   = 0x00000001 # < Transfer Active #

  BSC_FIFO_SIZE = 16 # < BSC FIFO size #

  #  Specifies the divider used to generate the I2C clock from the system clock.
  #  Clock divided is based on nominal base clock rate of 250MHz
  #
  I2C_CLOCK_DIVIDER_2500 = 2500 # < 2500 = 10us = 100 kHz #
  I2C_CLOCK_DIVIDER_626  =  626 # < 622 = 2.504us = 399.3610 kHz #
  I2C_CLOCK_DIVIDER_150  =  150 # < 150 = 60ns = 1.666 MHz (default at reset) #
  I2C_CLOCK_DIVIDER_148  =  148 # < 148 = 59ns = 1.689 MHz #

  #  Specifies the reason codes for the bcm2835_i2c_write and bcm2835_i2c_read functions.
  #
  I2C_REASON_OK         = 0x00 # < Success #
  I2C_REASON_ERROR_NACK = 0x01 # < Received a NACK #
  I2C_REASON_ERROR_CLKT = 0x02 # < Received Clock Stretch Timeout #
  I2C_REASON_ERROR_DATA = 0x04 # < Not all data is sent / received #

  # Registers offets from SMI_BASE #
  SMI_CS      =  0 # < Control and status register > #
  SMI_LENGTH  =  1 # < Transfer length register > #
  SMI_ADRS    =  2 # < Transfer address register > #
  SMI_DATA    =  3 # < Transfer data register > #
  SMI_READ0   =  4 # < Read  settings 0 register > #
  SMI_WRITE0  =  5 # < Write settings 0 register > #
  SMI_READ1   =  6 # < Read  settings 1 register > #
  SMI_WRITE1  =  7 # < Write settings 1 register > #
  SMI_READ2   =  8 # < Read  settings 2 register > #
  SMI_WRITE2  =  9 # < Write settings 2 register > #
  SMI_READ3   = 10 # < Read  settings 3 register > #
  SMI_WRITE3  = 11 # < Write settings 3 register > #
  SMI_DMAC    = 12 # < DMA control register > #
  SMI_DIRCS   = 13 # < Direct control register > #
  SMI_DIRADDR = 14 # < Direct access address register > #
  SMI_DIRDATA = 15 # < Direct access data register > #

  # Register masks for SMI_READ and SMI_WRITE #
  SMI_RW_WIDTH_MSK  = 0xC0000000 # < Data width mask > #
  SMI_RW_WID8       = 0x00000000 # < Data width 8 bits > #
  SMI_RW_WID16      = 0x40000000 # < Data width 16 bits > #
  SMI_RW_WID18      = 0x80000000 # < Data width 18 bits > #
  SMI_RW_WID9       = 0xC0000000 # < Data width 9 bits > #
  SMI_RW_SETUP_MSK  = 0x3F000000 # < Setup cycles (6 bits) > #
  SMI_RW_SETUP_LS   =         24 # < Shift for setup cycles > #
  SMI_RW_MODE68     = 0x00800000 # < Run cycle motorola mode > #
  SMI_RW_MODE80     = 0x00000000 # < Run cycle intel mode > #
  SMI_READ_FSETUP   = 0x00400000 # < Read : Setup only for first cycle > #
  SMI_WRITE_SWAP    = 0x00400000 # < Write : swap pixel data > #
  SMI_RW_HOLD_MSK   = 0x003F0000 # < Hold cycles (6 bits) > #
  SMI_RW_HOLD_LS    =         16 # < Shift for hold cycles > #
  SMI_RW_PACEALL    = 0x00008000 # < Apply pacing always > #
  SMI_RW_PACE_MSK   = 0x00007F00 # < Pace cycles (7 bits) > #
  SMI_RW_PACE_LS    =          8 # < Shift for pace cycles > #
  SMI_RW_DREQ       = 0x00000080 # < Use DMA req on read/write > #
  SMI_RW_STROBE_MSK = 0x0000007F # < Strobe cycles (7 bits) > #
  SMI_RW_STROBE_LS  =          0 # < Shift for strobe cycles > #

  # Registers masks for Direct Access control register #
  SMI_DIRCS_ENABLE = 0x00000001 # < Set to enable SMI. 0 = Read from ext. devices > #
  SMI_DIRCS_START  = 0x00000002 # < Initiate SMI transfer > #
  SMI_DIRCS_DONE   = 0x00000004 # < Set if transfer has finished / Write to clear flag > #
  SMI_DIRCS_WRITE  = 0x00000008 # < 1 = Write to ext. devices > #

  # Registers masks for Direct Access address register #
  SMI_DIRADRS_DEV_MSK = 0x00000300 # < Timing configuration slot >  #
  SMI_DIRADRS_DEV_LS  =          8 # < Shift for configuration slot > #
  SMI_DIRADRS_DEV0    = 0x00000000 # < Use timing config slot 0 > #
  SMI_DIRADRS_DEV1    = 0x00000100 # < Use timing config slot 1 > #
  SMI_DIRADRS_DEV2    = 0x00000200 # < Use timing config slot 2 > #
  SMI_DIRADRS_DEV3    = 0x00000300 # < Use timing config slot 3 > #
  SMI_DIRADRS_MSK     = 0x0000003F # < Adress bits SA5..SA0 > #
  SMI_DIRADRS_LS      =          0 # < Shift for address bits > #

  # SMI clock control registers : defined as offset from bcm2835_clk #
  SMICLK_CNTL = (44) # = 0xB0 #
  SMICLK_DIV  = (45) # = 0xB4 #

  # Defines for ST
  #   GPIO register offsets from ST_BASE.
  #   Offsets into the ST Peripheral block in bytes per 12.1 System Timer Registers
  #   The System Timer peripheral provides four 32-bit timer channels and a single 64-bit free running counter.
  #   ST_CLO is the System Timer Counter Lower bits register.
  #   The system timer free-running counter lower register is a read-only register that returns the current value
  #   of the lower 32-bits of the free running counter.
  #   ST_CHI is the System Timer Counter Upper bits register.
  #   The system timer free-running counter upper register is a read-only register that returns the current value
  #   of the upper 32-bits of the free running counter.
  #
  ST_CS  = 0x0000 # < System Timer Control/Status #
  ST_CLO = 0x0004 # < System Timer Counter Lower 32 bits #
  ST_CHI = 0x0008 # < System Timer Counter Upper 32 bits #

  # Defines for PWM, word offsets (ie 4 byte multiples) #
  PWM_CONTROL = 0
  PWM_STATUS  = 1
  PWM_DMAC    = 2
  PWM0_RANGE  = 4
  PWM0_DATA   = 5
  PWM_FIF1    = 6
  PWM1_RANGE  = 8
  PWM1_DATA   = 9

  # Defines for PWM Clock, word offsets (ie 4 byte multiples) #
  PWMCLK_CNTL = 40
  PWMCLK_DIV  = 41
  PWM_PASSWRD = (0x5A << 24) # < Password to enable setting PWM clock #

  PWM1_MS_MODE  = 0x8000 # < Run in Mark/Space mode #
  PWM1_USEFIFO  = 0x2000 # < Data from FIFO #
  PWM1_REVPOLAR = 0x1000 # < Reverse polarity #
  PWM1_OFFSTATE = 0x0800 # < Ouput Off state #
  PWM1_REPEATFF = 0x0400 # < Repeat last value if FIFO empty #
  PWM1_SERIAL   = 0x0200 # < Run in serial mode #
  PWM1_ENABLE   = 0x0100 # < Channel Enable #

  PWM0_MS_MODE   = 0x0080 # < Run in Mark/Space mode #
  PWM_CLEAR_FIFO = 0x0040 # < Clear FIFO #
  PWM0_USEFIFO   = 0x0020 # < Data from FIFO #
  PWM0_REVPOLAR  = 0x0010 # < Reverse polarity #
  PWM0_OFFSTATE  = 0x0008 # < Ouput Off state #
  PWM0_REPEATFF  = 0x0004 # < Repeat last value if FIFO empty #
  PWM0_SERIAL    = 0x0002 # < Run in serial mode #
  PWM0_ENABLE    = 0x0001 # < Channel Enable #

  # \brief bcm2835PWMClockDivider
  #  Specifies the divider used to generate the PWM clock from the system clock.
  #  Figures below give the divider, clock period and clock frequency.
  #  Clock divided is based on nominal PWM base clock rate of 19.2MHz
  #  The frequencies shown for each divider have been confirmed by measurement
  #
  PWM_CLOCK_DIVIDER_2048 = 2048 # < 2048 = 9.375kHz
  PWM_CLOCK_DIVIDER_1024 = 1024 # < 1024 = 18.75kHz
  PWM_CLOCK_DIVIDER_512  =  512 # < 512 = 37.5kHz
  PWM_CLOCK_DIVIDER_256  =  256 # < 256 = 75kHz
  PWM_CLOCK_DIVIDER_128  =  128 # < 128 = 150kHz
  PWM_CLOCK_DIVIDER_64   =   64 # < 64 = 300kHz
  PWM_CLOCK_DIVIDER_32   =   32 # < 32 = 600.0kHz
  PWM_CLOCK_DIVIDER_16   =   16 # < 16 = 1.2MHz
  PWM_CLOCK_DIVIDER_8    =    8 # < 8 = 2.4MHz
  PWM_CLOCK_DIVIDER_4    =    4 # < 4 = 4.8MHz
  PWM_CLOCK_DIVIDER_2    =    2 # < 2 = 9.6MHz, fastest you can get
  PWM_CLOCK_DIVIDER_1    =    1 # < 1 = 4.6875kHz, same as divider 4096

  fun init = bcm2835_init : Int
  fun close = bcm2835_close : Int
  fun set_debug = bcm2835_set_debug(debug : UInt8T) : Void
  fun version = bcm2835_version : UInt

  fun regbase = bcm2835_regbase(regbase : UInt8T) : UInt32T*
  fun peri_read = bcm2835_peri_read(paddr : UInt32T*) : UInt32T
  fun peri_read_nb = bcm2835_peri_read_nb(paddr : UInt32T*) : UInt32T
  fun peri_write = bcm2835_peri_write(paddr : UInt32T, value : UInt32T) : Void
  fun peri_write_nb = bcm2835_peri_write_nb(paddr : UInt32T*, value : UInt32T) : Void
  fun peri_set_bits = bcm2835_peri_set_bits(paddr : UInt32T*, value : UInt32T, mask : UInt32T) : Void

  fun gpio_fsel = bcm2835_gpio_fsel(pin : UInt8T, mode : UInt8T) : Void
  fun gpio_set = bcm2835_gpio_set(pin : UInt8T) : Void
  fun gpio_clr = bcm2835_gpio_clr(pin : UInt8T) : Void
  fun gpio_set_multi = bcm2835_gpio_set_multi(mask : UInt32T) : Void
  fun gpio_clr_multi = bcm2835_gpio_clr_multi(mask : UInt32T) : Void
  fun gpio_lev = bcm2835_gpio_lev(pin : UInt8T) : UInt8T
  fun gpio_eds = bcm2835_gpio_eds(pin : UInt8T) : UInt8T
  fun gpio_eds_multi = bcm2835_gpio_eds_multi(mask : UInt32T) : UInt32T
  fun gpio_set_eds = bcm2835_gpio_set_eds(pin : UInt8T) : Void
  fun gpio_set_eds_multi = bcm2835_gpio_set_eds_multi(mask : UInt32T) : Void
  fun gpio_ren = bcm2835_gpio_ren(pin : UInt8T) : Void
  fun gpio_clr_ren = bcm2835_gpio_clr_ren(pin : UInt8T) : Void
  fun gpio_fen = bcm2835_gpio_fen(pin : UInt8T) : Void
  fun gpio_clr_fen = bcm2835_gpio_clr_fen(pin : UInt8T) : Void
  fun gpio_hen = bcm2835_gpio_hen(pin : UInt8T) : Void
  fun gpio_clr_hen = bcm2835_gpio_clr_hen(pin : UInt8T) : Void
  fun gpio_len = bcm2835_gpio_len(pin : UInt8T) : Void
  fun gpio_clr_len = bcm2835_gpio_clr_len(pin : UInt8T) : Void
  fun gpio_aren = bcm2835_gpio_aren(pin : UInt8T) : Void
  fun gpio_clr_aren = bcm2835_gpio_clr_aren(pin : UInt8T) : Void
  fun gpio_afen = bcm2835_gpio_afen(pin : UInt8T) : Void
  fun gpio_clr_afen = bcm2835_gpio_clr_afen(pin : UInt8T) : Void
  fun gpio_pud = bcm2835_gpio_pud(pud : UInt8T) : Void
  fun gpio_pudclk = bcm2835_gpio_pudclk(pin : UInt8T, on : UInt8T) : Void
  fun gpio_pad = bcm2835_gpio_pad(group : UInt8T) : UInt32T
  fun gpio_set_pad = bcm2835_gpio_set_pad(group : UInt8T, control : UInt32T) : Void

  fun delay = bcm2835_delay(millis : UInt) : Void
  fun delay_microseconds = bcm2835_delayMicroseconds(micros : UInt64T) : Void

  fun gpio_write = bcm2835_gpio_write(pin : UInt8T, on : UInt8T) : Void
  fun gpio_write_multi = bcm2835_gpio_write_multi(mask : UInt32T, on : UInt8T) : Void
  fun gpio_write_mask = bcm2835_gpio_write_mask(value : UInt32T, mask : UInt32T) : Void
  fun gpio_set_pud = bcm2835_gpio_set_pud(pin : UInt8T, pud : UInt8T) : Void
  fun gpio_get_pud = bcm2835_gpio_get_pud(pin : UInt8T) : UInt8T

  fun spi_begin = bcm2835_spi_begin : Int
  fun spi_end = bcm2835_spi_end : Void
  fun spi_setBitOrder = bcm2835_spi_setBitOrder(order : UInt8T) : Void
  fun spi_setClockDivider = bcm2835_spi_setClockDivider(divider : UInt16T) : Void
  fun spi_set_speed_hz = bcm2835_spi_set_speed_hz(speed_hz : UInt32T) : Void
  fun spi_setDataMode = bcm2835_spi_setDataMode(mode : UInt8T) : Void
  fun spi_chipSelect = bcm2835_spi_chipSelect(cs : UInt8T) : Void
  fun spi_setChipSelectPolarity = bcm2835_spi_setChipSelectPolarity(cs : UInt8T, active : UInt8T) : Void
  fun spi_transfer = bcm2835_spi_transfer(value : UInt8T) : UInt8T
  fun spi_transfernb = bcm2835_spi_transfernb(tbuf : Char*, rbuf : Char*, len : UInt32T) : Void
  fun spi_transfern = bcm2835_spi_transfern(buf : Char*, len : UInt32T) : Void
  fun spi_writenb = bcm2835_spi_writenb(buf : Char*, len : UInt32T) : Void
  fun spi_write = bcm2835_spi_write(data : UInt16T) : Void

  fun aux_spi_begin = bcm2835_aux_spi_begin : Int
  fun aux_spi_end = bcm2835_aux_spi_end : Void
  fun aux_spi_setClockDivider = bcm2835_aux_spi_setClockDivider(divider : UInt16T) : Void
  fun aux_spi_CalcClockDivider = bcm2835_aux_spi_CalcClockDivider(speed_hz : UInt32T) : UInt16T
  fun aux_spi_write = bcm2835_aux_spi_write(data : UInt16T) : Void
  fun aux_spi_writenb = bcm2835_aux_spi_writenb(buf : Char*, len : UInt32T) : Void
  fun aux_spi_transfern = bcm2835_aux_spi_transfern(buf : Char*, len : UInt32T) : Void
  fun aux_spi_transfernb = bcm2835_aux_spi_transfernb(tbuf : Char*, rbuf : Char*, len : UInt32T) : Void
  fun aux_spi_transfer = bcm2835_aux_spi_transfer(value : UInt8T) : UInt8T

  fun i2c_begin = bcm2835_i2c_begin : Int
  fun i2c_end = bcm2835_i2c_end : Void
  fun i2c_setSlaveAddress = bcm2835_i2c_setSlaveAddress(addr : UInt8T) : Void
  fun i2c_setClockDivider = bcm2835_i2c_setClockDivider(divider : UInt16T) : Void
  fun i2c_set_baudrate = bcm2835_i2c_set_baudrate(baudrate : UInt32T) : Void
  fun i2c_write = bcm2835_i2c_write(buf : Char*, len : UInt32T) : UInt8T
  fun i2c_read = bcm2835_i2c_read(buf : Char*, len : UInt32T) : UInt8T
  fun i2c_read_register_rs = bcm2835_i2c_read_register_rs(regaddr : Char*, buf : Char*, len : UInt32T) : UInt8T
  fun i2c_write_read_rs = bcm2835_i2c_write_read_rs(cmds : Char*, cmds_len : UInt32T, buf : Char*, buf_len : UInt32T) : UInt8T

  fun smi_begin = bcm2835_smi_begin : Int
  fun smi_end = bcm2835_smi_end
  fun smi_set_timing = bcm2835_smi_set_timing(
    smichannel : UInt32T,
    readchannel : UInt32T,
    setupcycles : UInt32T,
    strobecycles : UInt32T,
    holdcycles : UInt32T,
    pacecycles : UInt32T
  ) : Void
  fun smi_write = bcm2835_smi_write(smichannel : UInt32T, data : UInt8T, address : UInt32T)
  fun smi_read = bcm2835_smi_read(smichannel : UInt32T, address : UInt32T) : UInt32T

  fun st_read = bcm2835_st_read : UInt64T
  fun st_delay = bcm2835_st_delay(offset_micros : UInt64T, micros : UInt64T) : Void

  fun pwm_set_clock = bcm2835_pwm_set_clock(divisor : UInt32T) : Void
  fun pwm_set_mode = bcm2835_pwm_set_mode(channel : UInt8T, markspace : UInt8T, enabled : UInt8T) : Void
  fun pwm_set_range = bcm2835_pwm_set_range(channel : UInt8T, range : UInt32T) : Void
  fun pwm_set_data = bcm2835_pwm_set_data(channel : UInt8T, data : UInt32T) : Void
end
