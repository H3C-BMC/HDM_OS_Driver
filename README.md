# Introduction 
**HDM_OS_Driver** is an operating system driver program, mainly used to facilitate information exchange between HDM (i.e. BMC) and the operating system.

# Requirements
Only H3C G6 and later series servers to provide services.

1. The HDM virtualizes itself into a USB device (Vendor ID:0x1472, Product ID:0x3013) through software.
2. Physically, the CPU and BMC are connected by a USB bus.
3. On the OS side, the USB device (Vendor ID:0x1472, Product ID:0x3013) can be identified on the USB bus Hdmdrv.ko is the driver of the device. Through this driver, you can communicate with USB devices (i.e. BMC) to achieve a variety of functions.
4. When the driver loads and identifies the corresponding USB device, a /dev/hdmdrv character device is generated, and the communication function to the BMC is realized through the read and write operation of the device.

# The hdmdrv function:
1. Identify the USB device (Vendor ID:0x1472, Product ID:0x3013), does not support other USB devices.
2. Use the customized data format for data transmission and communication with BMC. 
3. Provide read and write interfaces for upper-layer applications to call, and the upper-layer communication protocol is private.


# Source code usage:
1. Compile
  In the source directory, execute
  -make: Get the driver hdmdrv.ko.
  -make clean: Clean the product.
3. Install the driver
  Method 1:
    sp hdmdrv.ko /lib/modules/$(uname -r)/kernel/drivers/usb/misc/
    modprobe hdmdrv
  Method 2: insmod hdmdrv
4. Automatically load the driver (optional)
  echo "hdmdrv" > /etc/modules-load.d/hdmdrv.conf
  chmod +x /etc/modules-load.d/hdmdrv.conf
  depmod

