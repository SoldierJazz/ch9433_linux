# CH9433 Linux Driver

## Description

This package provides the Linux kernel driver for the WCH CH9433 USB-to-CAN controller.
The driver registers CH9433 devices as SocketCAN network interfaces and also exposes a
character device named `/dev/ch9433_iodevN` for device-specific control transfers.

Supported USB device:

| Vendor ID | Product ID | Interface |
| --- | --- | --- |
| `0x1a86` | `0x5610` | `0` |

## Package Contents

```text
README.md
driver/
  Makefile
  ch9433.c
```

## Requirements

- Linux kernel headers for the target kernel
- GNU Make and GCC
- Kernel SocketCAN support (`can` and `can-dev`)
- Optional test tools from `can-utils`

On Debian or Ubuntu systems, the common build dependencies can be installed with:

```sh
sudo apt install build-essential linux-headers-$(uname -r)
```

## Build

```sh
cd driver
make
```

If the build succeeds, the kernel module `ch9433.ko` is generated in the `driver`
directory.

To build against a kernel other than the running kernel, pass the target kernel build
directory:

```sh
make KERNELDIR=/path/to/kernel/build KVER=<target-kernel-version>
```

## Build Into the Kernel Tree

For products that need menuconfig-managed or built-in driver support, add the driver to
the target Linux kernel source tree.

1. Copy the driver source into the kernel CAN USB driver directory:

```sh
cp driver/ch9433.c <kernel-source>/drivers/net/can/usb/ch9433.c
```

2. Add CH9433 to `<kernel-source>/drivers/net/can/usb/Makefile`:

```make
obj-$(CONFIG_CAN_CH9433) += ch9433.o
```

3. Add CH9433 to `<kernel-source>/drivers/net/can/usb/Kconfig`:

```kconfig
config CAN_CH9433
	tristate "WCH CH9433 USB-CAN adapter"
	depends on USB && CAN_DEV
	help
	  This driver supports the WCH CH9433 USB-to-CAN controller.
```

4. Enable the driver with `menuconfig`:

```text
Networking support
  CAN bus subsystem support
    CAN Device Drivers
      CAN USB interfaces
        WCH CH9433 USB-CAN adapter
```

Select `<*>` to build the driver into the kernel, or select `<M>` to build it as a
kernel-tree module.

5. Rebuild and install the kernel or modules according to the target platform's normal
kernel build process.

When the driver is built into the kernel, it is registered during kernel startup and
cannot be unloaded with `rmmod`. When built as `<M>`, load it with `modprobe ch9433`.

## Load Temporarily

Plug in the CH9433 device first, then load the driver:

```sh
cd driver
sudo make load
```

The same operation can be performed manually:

```sh
sudo modprobe can
sudo modprobe can-dev
sudo insmod ./ch9433.ko
```

Check whether the driver has been loaded and a CAN interface has been created:

```sh
lsmod | grep ch9433
dmesg | tail
ip link show type can
ls /dev/ch9433_iodev*
```

## Configure a CAN Interface

Replace `can0` with the interface name shown on your system:

```sh
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
```

With `can-utils` installed, basic traffic tests can be run with:

```sh
candump can0
cansend can0 123#1122334455667788
```

## Unload

```sh
cd driver
sudo make unload
```

Or manually:

```sh
sudo rmmod ch9433
```

## Install Permanently

Install the module into `/lib/modules/$(uname -r)/extra/` and refresh module
dependencies:

```sh
cd driver
sudo make install
sudo modprobe ch9433
```

After installation, the driver can be loaded by `modprobe ch9433` whenever the target
kernel is running.

## Uninstall

```sh
cd driver
sudo make uninstall
```

## Clean Build Files

```sh
cd driver
make clean
```

Run `make clean` before creating a release archive if generated kernel build files
should not be included in the package.

## Notes

- If Secure Boot or kernel module signature enforcement is enabled, the built module may
  need to be signed before it can be loaded.
- The module name is `ch9433`; the generated file is `ch9433.ko`.

For support, contact WCH: tech@wch.cn
