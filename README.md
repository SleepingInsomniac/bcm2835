# bcm2835

Crystal Bindings to [LibBcm2835](http://www.airspayce.com/mikem/bcm2835/index.html).

#### [From Raspberry Pi Documentation](https://www.raspberrypi.com/documentation/computers/processors.html):

The BCM2835 is the Broadcom chip used in the Raspberry Pi 1 Models A, A+, B, B+, the Raspberry Pi Zero, the Raspberry Pi Zero W, and the Raspberry Pi Compute Module 1. Some details of the chip can be found in the peripheral specification document. It contains a single-core ARM1176JZF-S processor.

## Installation

1. Add the dependency to your `shard.yml`:

   ```yaml
   dependencies:
     bcm2835:
       github: sleepinginsomniac/bcm2835
   ```

2. Run `shards install`
3. Run `./build_bcm2835.sh` to build the shared library

## Usage

```crystal
require "bcm2835"
```


## Contributing

1. Fork it (<https://github.com/sleepinginsomniac/bcm2835/fork>)
2. Create your feature branch (`git checkout -b my-new-feature`)
3. Commit your changes (`git commit -am 'Add some feature'`)
4. Push to the branch (`git push origin my-new-feature`)
5. Create a new Pull Request

## Contributors

- [Alex Clink](https://github.com/sleepinginsomniac) - creator and maintainer
