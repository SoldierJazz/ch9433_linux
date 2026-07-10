# CH9433 Linux 驱动程序

## 简介

本软件包提供 WCH CH9433 USB 转 CAN 控制器的 Linux 设备驱动程序。

该驱动将 CH9433 设备注册为 SocketCAN 网络接口，并暴露一个名为 `/dev/ch9433_iodevN` 的字符设备，用于设备特定的控制传输。

支持的 USB 设备：

| **供应商 ID (Vendor ID)** | **产品 ID (Product ID)** | **接口 (Interface)** |
| ------------------------- | ------------------------ | -------------------- |
| `0x1a86`                  | `0x5610`                 | `0`                  |

## 软件包内容

```
README.md
driver/
  Makefile
  ch9433.c
```

## 依赖要求

- 目标内核的 Linux 内核头文件 (Linux kernel headers)
- GNU Make 与 GCC
- 内核 SocketCAN 支持 (`can` 与 `can-dev`)
- 可选：`can-utils` 测试工具

在 Debian 或 Ubuntu 系统上，可通过以下命令安装常用的编译依赖项：

```
sudo apt install build-essential linux-headers-$(uname -r)
```

## 编译

```
cd driver
make
```

如果编译成功，将在 `driver` 目录中生成内核模块 `ch9433.ko`。

若要为非当前正在运行的内核进行编译，请传入目标内核的编译目录：

```
make KERNELDIR=/path/to/kernel/build KVER=<target-kernel-version>
```

## 编译进内核树

对于需要 menuconfig 管理或内置驱动支持的产品，请将驱动程序添加到目标 Linux 内核源码树中。

1. 将驱动源码复制到内核 CAN USB 驱动目录中：

```
cp driver/ch9433.c <kernel-source>/drivers/net/can/usb/ch9433.c
```

2. 在 `<kernel-source>/drivers/net/can/usb/Makefile` 中添加 CH9433：

```
obj-$(CONFIG_CAN_CH9433) += ch9433.o
```

3. 在 `<kernel-source>/drivers/net/can/usb/Kconfig` 中添加 CH9433：

```
config CAN_CH9433
	tristate "WCH CH9433 USB-CAN adapter"
	depends on USB && CAN_DEV
	help
	  This driver supports the WCH CH9433 USB-to-CAN controller.
```

4. 使用 `menuconfig` 启用该驱动：

```
Networking support
  CAN bus subsystem support
    CAN Device Drivers
      CAN USB interfaces
        WCH CH9433 USB-CAN adapter
```

选择 `<*>` 将驱动编译进内核，或选择 `<M>` 将其编译为内核树模块。

5. 按照目标平台常规的内核编译流程，重新编译并安装内核或模块。

当驱动被编译进内核时，它会在内核启动期间注册，且无法使用 `rmmod` 卸载。当编译为 `<M>`（模块）时，可使用 `modprobe ch9433` 进行加载。

## 动态加载模块

首先插入 CH9433 设备，然后加载驱动：

```
cd driver
sudo make load
```

也可以手动执行相同的操作：

```
sudo modprobe can
sudo modprobe can-dev
sudo insmod ./ch9433.ko
```

检查驱动是否已成功加载，并确认 CAN 接口是否已创建：

```
lsmod | grep ch9433
dmesg | tail
ip link show type can
ls /dev/ch9433_iodev*
```

## 配置 CAN 接口

请将 `can0` 替换为您系统中实际显示的接口名称：

```
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
```

在安装了 `can-utils` 的情况下，可以使用以下命令进行基本的数据流量测试：

```
candump can0
cansend can0 123#1122334455667788
```

## 卸载模块

```
cd driver
sudo make unload
```

或手动卸载：

```
sudo rmmod ch9433
```

## 永久安装

将模块安装到 `/lib/modules/$(uname -r)/extra/` 目录下，并刷新模块依赖关系：

```
cd driver
sudo make install
sudo modprobe ch9433
```

安装完成后，只要目标内核正在运行，系统就可以随时通过 `modprobe ch9433` 来加载驱动程序。

## 卸载安装

```
cd driver
sudo make uninstall
```

## 清理编译文件

```
cd driver
make clean
```

如果不希望在最终发版的源码包中包含生成的内核编译文件，请在创建发布压缩包之前运行 `make clean`。

## 注意事项

- 如果启用了安全启动 (Secure Boot) 或内核模块签名强制验证，编译生成的模块可能需要先进行签名才能被加载。
- 该模块名称为 `ch9433`；生成的文件为 `ch9433.ko`。

如需技术支持，请联系 WCH：tech@wch.cn