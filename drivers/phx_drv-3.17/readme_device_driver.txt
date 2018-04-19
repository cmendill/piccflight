Active Silicon CDA Phoenix Driver for Linux - Installation Guide
----------------------------------------------------------------

Version      : v3.15
Release Date : 27 April 2010

The Active Silicon Cached Driver Architecture (CDA) device driver
provides support for the high-performance Active Silicon frame grabber
range on Linux platforms.

This driver provides optimized control of analog and digital acquisition,
bus mastering and real-time control of Active Silicon's Phoenix and LFG
frame grabber cards.

The same basic CDA driver is used for all Active Silicon's capture cards,
but is tailored slightly during install so that device names appear as 
"lfg" for the LFG card, "phx" for the Phoenix card etc.


This file describes how to compile, install, and load the device
driver so that Phoenix cards can be used under Linux.



COPYRIGHT INFORMATION
---------------------

Active Silicon CDA device driver for linux.
Copyright (C) 1997-2010  Active Silicon Limited.

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>


We distribute the system library files required to control the device
driver in pre-compiled binary form. Use of these library files is subject
to the terms of the Active Silicon CDA Driver Software Licence & Warranty
Agreement. You will find this agreement in the file license.txt.

Active Silicon Limited developed the platform-portable CDA driver
architecture from the ground up, without using any GPL,BSD,MIT or any
other licensed source code. This CDA driver architecture is highly
optimised for Active Silicon's hardware, and can be rapidly ported
to new operating system versions or entirely new operating systems.



PRE-REQUISITE: KERNEL SOURCE or KERNEL HEADERS PACKAGE
------------------------------------------------------

Before the CDA device driver kernel module can be compiled, you MUST have
the source for your Linux kernel installed on your computer and it MUST
have been configured for your required kernel options.

If you are using a Linux distribution such as SuSE or Red Hat, and are
still using the Linux kernel that came with that distribution, you should
be able to install the kernel source for your installation from your
original installation media.

If you are using a Linux distribution such as SuSE or Red Hat, but have
updated your kernel to an 'errata' or 'update' version distributed by
your distribution vendor, you should ensure that you have also updated
your kernel source package.

If you are using a kernel that you have compiled yourself, you should
ensure that the directory /usr/src/linux exists, and that it contains the
source to the kernel you are running.


STAGE 1: KERNEL CONFIGURATION
-----------------------------
Note: Root priveleges are required for this stage and all subsequent stages.

The CDA device driver kernel module has to match the various options
selected in the kernel it will be loaded into. The compilation script
(the 'Makefile') for the CDA device driver will attempt to gather these
options from the kernel configuration file .config that it expects to
find in the directory /usr/src/linux

/usr/src/linux may well be a symbolic link to another directory.


Red Hat Linux
-------------
If you are using Red Hat Linux and using a Red Hat provided kernel,
perform the following steps as the root user:

1. Check that the /usr/src/linux directory or symbolic link exists.
   If it does not, check that you have installed the kernel source code.
   If a symbolic link /usr/src/linux-2.6 exists, make a new symbolic link
   /usr/src/linux pointing to the same destination:

      cd /usr/src
      ln -s linux-2.6 linux

2. Change to the /usr/src/linux directory:

      cd /usr/src/linux

3. Clear any current kernel configuration options:

      make mrproper

   (WARNING: This removes your current kernel source configuration. You
    may want to save a copy of this directory.)

4. If you have installed the Red Hat kernel source package, there will be
   a directory called "configs" in your /usr/src/linux directory. You
   will need to select the configuration file in that directory which
   matches your machine architecture and copy it to a file
   /usr/src/linux/.config

      e.g:  cp configs/kernel-2.6.18-i686.config .config

   Tip:  You can identify your hardware type with the command: uname -m
         (See also "uname -r" and "uname -a".)

5. Edit the file /usr/src/linux/Makefile, and remove the text "custom"
   from the line that starts "EXTRAVERSION ="

      e.g : Change the line EXTRAVERSION = -27.8.0custom
                   to read: EXTRAVERSION = -27.8.0

   You may need to add "smp" for multiprocessor kernels
      e.g :                 EXTRAVERSION = -27.8.0smp

6. Create a new kernel makefile from this .config file:

      make oldconfig


SuSE Linux
----------
If you are using SuSE Linux and using a SuSE provided kernel, perform the
following steps as the root user:

1. Check that the /usr/src/linux directory or symbolic link exists.
   If it does not, check that you have installed the kernel source code.
   If a symbolic link /usr/src/linux-2.6 exists, make a new symbolic link
   /usr/src/linux pointing to the same destination:

      cd /usr/src
      ln -s linux-2.6 linux

2. Change to the /usr/src/linux directory:

      cd /usr/src/linux

3. Clear any current kernel configuration options:

      make mrproper

4. Make a new configuration to match the running kernel:

      make cloneconfig


STAGE 2: INSTALL THE RPM
------------------------
1. Check for potential major device number clashes, by typing:

   cat /proc/devices

   and check that the number 243 is not listed under "Character
   devices".  (It is OK if the number appears under "Block devices".)

2. If Character Device 243 already exists and you do not know how to
   resolve this conflict, please contact Active Silicon Technical
   Support.

3. Install the rpm with the following command:

   rpm --install active_silicon-phx_drv-3.15-00.noarch.rpm



STAGE 3: BUILDING AND INSTALLING THE PHOENIX CDA DRIVER
-------------------------------------------------------

1. To build the device driver, change to the directory with the CDA
   source code (/usr/local/active_silicon/phx_drv-3.15/kernel-2.6/) and
   execute the command "make".
   This will create the binary kernel module phddrv.ko

2. To install the device driver, execute the command "make install". This
   will:
   a) copy the kernel module to [for 2.6 family kernels]
      /lib/modules/[kernel-version]/kernel/drivers/media/video/phddrv.ko
   b) call the "depmod" utility to recreate kernel dependencies to take
      account of Phoenix.
   c) Install an entry for the driver in /etc/modprobe.conf
   d) Create the required device special files in /dev

**** The Phoenix driver is now installed ****



STAGE 4: 32-BIT CPU MEMORY LIMITATIONS
--------------------------------------

On 32-bit systems (e.g. i386, i686 etc), this driver does not support the
use of 'high' memory. Therefore you should either
1. Set NOHIGHMEM: in your kernel configuration.
   or
2. add 'mem=768M' to your grub boot file (grub.conf) 

This restriction does not apply to 64-bit systems (e.g, x86_64).



PROCEDURE TO REMOVE THE PHOENIX CDA DRIVER
------------------------------------------
1. Type "cd /usr/local/active_silicon/phx_drv-3.15/kernel-2.6".

2. Type "make uninstall".

3. Type "cd /".

4. Type "rpm --erase active_silicon-phx_drv-3.15".

**** The driver and rpm package are now removed ****
