require "../src/bcm2835"

# If you call this, it will not actually access the GPIO
# Use for testing
#        LibBcm2835.set_debug(1)

if LibBcm2835.init == 0
  puts "bcm2835_init failed. Are you running as root??"
  exit 1
end

if LibBcm2835.spi_begin == 0
  puts "bcm2835_spi_begin failed. Are you running as root??"
  exit 1
end

LibBcm2835.spi_setBitOrder(LibBcm2835::SPI_BIT_ORDER_MSBFIRST)             # The default
LibBcm2835.spi_setDataMode(LibBcm2835::SPI_MODE0)                          # The default
LibBcm2835.spi_setClockDivider(LibBcm2835::SPI_CLOCK_DIVIDER_65536)        # The default
LibBcm2835.spi_chipSelect(LibBcm2835::SPI_CS0)                             # The default
LibBcm2835.spi_setChipSelectPolarity(LibBcm2835::SPI_CS0, LibBcm2835::LOW) # the default

# Send a byte to the slave and simultaneously read a byte back from the slave
# If you tie MISO to MOSI, you should read back what was sent

send_data = 0x23_u8
read_data = LibBcm2835.spi_transfer(send_data)

puts "Sent to SPI: 0x%02X. Read back from SPI: 0x%02X." % [send_data, read_data]

unless send_data == read_data
  puts "Do you have the loopback from MOSI to MISO connected?"
end

LibBcm2835.spi_end
LibBcm2835.close
