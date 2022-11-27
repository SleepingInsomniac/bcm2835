require "../src/bcm2835"

# Blinks on RPi Plug P1 pin 11 (which is GPIO pin 17)
PIN = LibBcm2835::RPI_GPIO_P1_11

# If you call this, it will not actually access the GPIO
# Used for testing
#    LibBcm2835.set_debug(1)

if LibBcm2835.init == 0
  exit 1
end

at_exit do
  LibBcm2835.close
end

# Set the pin to be an output
LibBcm2835.gpio_fsel(PIN, LibBcm2835::GPIO_FSEL_OUTP)

# Blink
loop do
  # Turn it on
  LibBcm2835.gpio_write(PIN, LibBcm2835::HIGH)

  # wait a bit
  sleep(0.5)

  # turn it off
  LibBcm2835.gpio_write(PIN, LibBcm2835::LOW)
  # wait a bit
  sleep(0.5)
end
