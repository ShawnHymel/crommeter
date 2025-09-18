# Crommeter SAO

> SHOW ME WHAT YOU GOT

Badge accessory (SAO) to measure general sound level from cheers and applause. Gives a (somewhat) precise measurement of sound pressure level in dB.

This was made in Rust for no other reason than to test my embedded Rust skills.

## Build the Project

To build the project, you'll need Rust on your computer. Follow the [instructions here to install Rust](https://www.rust-lang.org/tools/install).

Next, install the Thumbv8m target processor:

```sh
rustup target add thumbv8m.main-none-eabihf
```

Navigate into the firmware directory and build the project:

```sh
cd firmware/crommeter/
cargo build --release
```

You can either flash the output ELF file (in *target/thumbv8m.main-none-eabihf/release*) using the [Raspberry Pi Debug Probe](https://www.raspberrypi.com/documentation/microcontrollers/debug-probe.html) or you can convert the ELF file to a UF2 file to flash over the USB mass storage device bootloader. To do the latter, you'll need to install [picotool](https://github.com/raspberrypi/picotool).

## (Optional) Install picotool

I recommend downloading one of the pre-compiled releases of picotool from [here](https://github.com/raspberrypi/pico-sdk-tools/releases). Unzip the file somewhere on your computer.

### Windows

Copy the *picotool/* folder to a place like *C:\Program Files\picotool\*.

Update PATH:

 1. Press **Win + R**, type `sysdm.cpl`, press **Enter**
 2. Go to the **Advanced** tab and click **Environment Variables**
 3. Under *User variables* or *System variables*, find *Path*
 4. Click **Edit > New** and add: `C:\Program Files\picotool`
 5. Click **OK** on all dialogs
 6. Restart your terminal/command prompt

### Linux

Copy picotool to your user local bin folder:

```sh
mkdir -p ~/.local/bin
cp picotool ~/.local/bin/
chmod +x ~/.local/bin/picotool
```

Update PATH: add the following to *~/.bashrc*:

```sh
export PATH="$HOME/.local/bin:$PATH"
```

Reload:

```sh
source ~/.bashrc
```

### macOS

Copy picotool to your user local bin folder:

```sh
mkdir -p ~/.local/bin
cp picotool ~/.local/bin/
chmod +x ~/.local/bin/picotool
```

Update PATH: add the following to *~/.zshrc*:

```sh
export PATH="$HOME/.local/bin:$PATH"
```

Reload:

```sh
source ~/.zshrc
```

## Run picotool and Flash

Once you have picotool installed, you can convert the ELF file to a UF2 file. Navigate to the firmware's directory and run:

```sh
cd firmware/crommeter/
picotool uf2 convert target/thumbv8m.main-none-eabihf/release/crommeter -t elf firmware.uf2
```

Hold the BOOT button on the RP2350 stamp and press/release the RESET button while the board is plugged into your computer. That should put the RP2350 into bootloader mode, which enumerates as a mass storage device on your computer. Then, simply copy the *firmware.uf2* file you just created to the RP2350 drive. The board should automatically reset and start running the firmware!

## License

The hardware files are released under [Creative Commons Attribution 4.0 International (CC BY 4.0)](https://creativecommons.org/licenses/by/4.0/).

Code in the firmware directory, unless otherwise noted, is licensed under the [Zero-Clause BSD license](https://opensource.org/license/0bsd) (0BSD).

```
Zero-Clause BSD
=============

Permission to use, copy, modify, and/or distribute this software for
any purpose with or without fee is hereby granted.

THE SOFTWARE IS PROVIDED “AS IS” AND THE AUTHOR DISCLAIMS ALL
WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES
OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE
FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY
DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN
AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT
OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
```