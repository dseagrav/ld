<!-- markdown-toc start - Don't edit this section. Run M-x markdown-toc-refresh-toc -->
**Table of Contents**

- [LAMBDADELTA](#lambdadelta)
    - [About the Lambda](#about-the-lambda)
    - [Contact](#contact)
- [Getting Started](#getting-started)
    - [Prerequisites](#prerequisites)
    - [Building LambdaDelta](#building-lambdadelta)
- [Configuring LambdaDelta](#configuring-lambdadelta)
- [Using LambdaDelta](#using-lambdadelta)
    - [Preparing ROM Images](#preparing-rom-images)
    - [Microcode Symbol Files](#microcode-symbol-files)
    - [Installing a New Machine](#installing-a-new-machine)

<!-- markdown-toc end -->
# LD-THREAD BRANCH WARNING

This is the HIGHLY EXPERIMENTAL multi-threaded branch of LambdaDelta.
It is not quite ready for public consumption yet. Expect misbehavior.
Back up your disk images frequently.

This version uses the Lambda's cache system to minimize contention
for bus mastership. Ensure that the Lambda's configuration profile
(set in the SDU "config" program) has cache enabled.

Two versions of the cache system are provided:

The default is the "Fast Cache" - If caching is permitted and the operation
is a word read, the bus cycle is bypassed and values read directly from
the memory array. All other cycles happen normally.

Optionally, the full Lambda cache system can be used, but benchmarks on it
indicate cache data maintenance costs more than the performance gained.

# LAMBDADELTA

LambdaDelta is an emulator of the LMI Lambda Lisp Machine. Its goal is to
simulate the hardware sufficiently to run original microcode and binaries
unmodified, with performance and capability comparable to the original
hardware.

LambdaDelta is written in C. It is intended to be portable to any reasonably
Unix-like operating system, but is developed primarily on Linux. The initial
versions will most likely fail to compile elsewhere, but attempts will be
made to rectify this as soon as practical.

SDL Version 1 or 2 is used for console IO, the Linux tuntap driver or
Berkeley Packet Filter is used for network IO, and standard Unix file
operations are used for disk and tape IO. LambdaDelta includes code from the
Fake86 project by Mike Chambers in its SDU implementation.

LambdaDelta is licensed under the GNU GPL Version 2 or later at your option.
Please see the file COPYING for details.

## About the Lambda

The LMI Lambda is a NuBus-based machine consisting of at least one Lambda
processor, a System Diagnostic Unit, and various NuBus and Multibus
peripherals. The System Diagnostic Unit has an Intel 8088 processor, and is
responsible for bootstrapping the Lambda processor. The Lambda processor is
made up of four cards in a specially wired area of the NuBus backplane.
The standard Multibus peripherals are a 3Com ethernet interface, an
Interphase SMD disk controller, a Tapemaster 9-track tape controller,
and a Quarterback QIC tape controller (not emulated). The SDU software 
mediates sharing of Multibus peripherals. The SDU hardware provides a
DMA path between Multibus space and NuBus space with mapping hardware
for routing pages between targets on both busses.

The NuBus peripherals are memory and the VCMEM console controller. There
must be one VCMEM for each Lambda processor. The standard configuration
has one pair. An optional configuration called LAMBDA-2x2 has two pairs.
This enables two simultaneous and separate Lisp environments to share one
backplane, memory, disk, and network interface. The result was a
considerable cost savings over buying a second machine. The 2x2 machine
was also used for operating system development.

There was also an optional 68000-based Unix processor which could run
V7 or SVR4. No attempt was made to emulate this. A configuration with a
Unix processor was given the suffix "plus", making a "LAMBDA-PLUS"
(or "LAMBDA+") a single-user Lambda with a Unix processor and a 
"LAMBDA-2X2-PLUS" a two-user Lambda with a Unix processor.

## Contact

Questions, ideas, problems, solutions, praise, complaints, or baseless
accusations may be directed to the LispM mailing list on tunes.org.
See http://lists.tunes.org/mailman/listinfo/lispm/ for more information.

All contributors to this project are listed in the AUTHORS file.

# Getting Started

## Prerequisites 

LambdaDelta requires a set of ROM images from a physical Lambda, and
either a disk image of an installed system or images of the installation
and source tapes. At the time of release, these items are available from
Bitsavers. See http://www.bitsavers.org/ for more information.

A prototype Release 5.0 system software distribution is available at 
https://s3.us-east-2.amazonaws.com/ds.storage.0000/Lambda-Release5.tar.gz
A Github repository has been set up to track issues and changes for this
release, see https://github.com/dseagrav/Lambda-system-software

## Building LambdaDelta

LambdaDelta uses the GNU auto* tools for configuration and compilation.
Most features and libraries should be configured automatically.

If the build fails at first, try `autoreconf -i` and re-try the build.
(See dseagrav/ld#6 for the issue.)

If your system has both SDL1 and SDL2, you will be required to explicitly disable
the one you do not want. (use --`without-SDLx`). SDL1 seems to perform
better over remote X11 connections, but SDL2 has better performance
when running locally.

If you wish to emulate the optional 2x2 configuration, run configure
with the option `--enable-config-2x2=yes`

After compilation, run-time options are controlled by a configuration
file. If you have the YAML library installed (and configure found it),
it will use YAML configuration. Otherwise, it will use the old
configuration file.

# Configuring LambdaDelta
This is the new configuration method that is intended to be used going
forward. The YAML configuration file is named `lam.yml` and is searched
for in the following places, in order: 

<!-- TODO: make this a link to the file -->
*Note:* If you do not have libyaml on your system (or do not wish to install it), please see doc/OLDCONF.md for the legacy configuration system..

1. The directory specified in `SYSCONFDIR` if it was defined during build.
2. The user's home directory, as determined by:
    1. The `$HOME` environment variable, or if that is not defined:
    2. If we are root and were run by sudo, from the sudoer's passwd entry.
    3. If we are root and were not run by sudo, root's passwd entry.
    4. If we are not root, the user's passwd entry.
3. The current working directory.

Any or all of these locations are valid; Configuration items in later
files supersede those specified in earlier files.

You may also specify a configuration file of any name on the command line
by using the `-c` argument. This file, if provided, is loaded last.

Single YAML key/value pairs may also be specified on the command line
by specifying them in the form  `--section-key value`. For example, the
argument `--log-ALL 10` will enable maximum logging of all types.

A full description of the YAML configuration file is beyond the scope
of this document; See the "lam.yml" file in the examples directory.

# Using LambdaDelta

While the program is running, the window title bar has the following form:

    LambdaDelta: VC N | Tape: FNAME | STATUS | DT X

N is the number of the active console, either 0 or 1. N is always zero when
the 2x2 configuration is not in use. `FNAME` is the name of the active tape
image file. `STATUS` is a string describing the state of the Lambda processor
associated with this console. The strings have the following meanings:

| Status string    | Meaning                                                                                                        |
|------------------|----------------------------------------------------------------------------------------------------------------|
| Cold Halted      | The processor has been powered on and has no state.                                                            |
| Cold Running     | The halt flag is clear but the state is unknown.                                                               |
| Cold Booting     | The bootstrap is running.                                                                                      |
| Lisp Boot Halted | The processor halted while Lisp was initializing. (This is an error condition.)                                |
| Lisp Booting     | Lisp is initializing or warm booting.                                                                          |
| Running          | Lisp is running.                                                                                               |
| Halted           | Lisp has stopped running. The processor has valid state. If you did not halt lisp, this is an error condition. |
           
After the status string is the time offset. X is a number in deciseconds
which indicates how far apart real time and the emulator's time are.
The emulator will try to hold this number at 0, but if your computer is
not fast enough to run the emulator in real time this number will become
negative and decrease further as the times diverge. If the number becomes
positive and increases, the throttle is not operating properly. This is a
bug which should be reported.

The following keys control emulator functions are cannot be remapped:

F9 switches the active console if the 2x2 configuration is enabled.
  If the standard configuration is in use, F9 may be remapped.
F10 changes mouse operation according to the configured mouse mode.
  In mode 0, F10 toggles capture and release of the mouse pointer. Clicking
  inside the LambdaDelta window while the mouse is not captured will
  recapture it.
  In mode 1, F10 toggles visibility of the host mouse pointer.

F11 simulates the "return to newboot" keyboard chord, which terminates Lisp
  and recalls the Newboot interface. Limitations of the keyboard emulation
  make typing the actual chord fail, so this directly sends the expected
  sequence of bytes to make things happen.

F12 causes the active tape image file to rotate to the next file in
  ASCIIbetical order. Pressing control and F12 at the same time causes
  the emulator to dump its state into a bunch of .DUMP files. These files
  are human-readable but not necessarily human-understandable. (It is our
  understanding that whether or not the developers are classified as human
  is a subject of ongoing debate.)

All other keys on the keyboard may be remapped using the map_key option
described above. The standard mapping preserves the printed key label
of the standard typewriter keys. The other keys are mapped as follows:

| HOST KEY   | LAMBDA KEY  | NOTES                                  |
|------------|-------------|----------------------------------------|
| [          | (           | [ is the shift state, ( is unshifted   |
| ]          | )           | ] is the shift state, ) is unshifted   |
| RETURN     | ENTER       |                                        |
| BACKSPACE  | RUBOUT      |                                        |
| UP         | HAND UP     |                                        |
| DOWN       | HAND DOWN   |                                        |
| LEFT       | HAND LEFT   |                                        |
| RIGHT      | HAND RIGHT  |                                        |
| PAGE UP    | ABORT       |                                        |
| PAGE DOWN  | RESUME      |                                        |
| F1         | SYSTEM      |                                        |
| F2         | NETWORK     |                                        |
| F3         | STATUS      |                                        |
| F4         | TERMINAL    |                                        |
| F5         | HELP        |                                        |
| F6         | CLEAR       |                                        |
| F7         | BREAK       |                                        |
| RIGHT CTRL | LEFT GREEK  |                                        |
| RIGHT ALT  | LEFT SUPER  |                                        |
| LEFT ALT   | LEFT META   |                                        |
| RIGHT FLAG | RIGHT SUPER | Does not always work on some platforms |
| LEFT FLAG  | LEFT SUPER  | Does not always work on some platforms |
| MENU       | RIGHT HYPER |                                        |

The default keymap is still under development and is subject to change. Feel
free to make suggestions or comments.

## Preparing ROM Images

The ROM images go in the "roms" subdirectory. The necessary files are:

- `SDU.ROM`: 
The SDU's 8088 program ROM. 64K, merged from two chips on the board. 
Bitsavers has one with an MD5 checksum of  `4795bf46168808d32012213c0b370f30`
- `MEM.ROM`:
The nubus-space configuration ROM for a memory card. 2K.
Bitsavers has two, and their MD5 checksums are `21089f3b4082548865f8cda6017f302e` or `1f898d018a2e2ab29279ecf00c7a4c82`.
Either one may be used, but the same one will be used for both cards simulated.
- `VCMEM.ROM` - The VCMEM's nubus-space configuration ROM. 2K. 
This contains 8088 program code that will be run by the SDU. 
The one on Bitsavers has an MD5 checksum of `0e53416a49294f02c7fd925c9de47f5a`.

These can be found in zip files on the PDFs side of Bitsavers.

## Microcode Symbol Files

In the lisp source tree there are files named `bootstrap.lam-uload` and 
`ulambda.lmc-sym` which correspond to the Lambda bootstrap code downloaded
by the SDU and the Lisp microcode. Place these in the LambdaDelta directory
to provide symbols for debugging. These are optional.

## Installing a New Machine

1. Prepare your config file, ROM images, tape images, and create the disk
   image file. Ensure the SDU switch setting is zero in your configuration.
2. Start the emulator. Telnet to port 3637 on the host. The emulator should
   start, and you should have a SDU prompt in your telnet session.

   Ignore the graphical console for the now - It is inoperative until
   the SDU firmware partition is loaded and the CMOS contents are valid.

3. Type `init` and push enter to initialize the busses and SDU. The SDU will
   reboot.

4. Ensure the install tape is the active tape.

5. Give the SDU command `/tar/setup clear eagle sp shell`. Your CMOS should
   be initialized.

6. Give the SDU command `/tar/load` and follow the prompts.

7. When the `load >` prompt appears, give the command `install` and follow
   the prompts.

8. Give the command `q` to return to the SDU

9. Close the emulator by closing its window

10. Edit your config file file and change the SDU switch setting to 1

11. Restart the emulator. You should get a SDU prompt on the console.

12. Give the `config` command to initialize the Lambda configuration.
    When asked if you want to change anything, type y and press enter.

13. At the `cmd:` prompt, give the command `lambda`.

14. At the `lambda cmd:` prompt, give the command `switches`

15. At the `->` prompt, give the command `set use_multiplier`

16. Push enter to return to the lambda command level.

17. Give the command `x` until config reboots the SDU.

18. At this point your machine is configured and you can save backups of your
    ld.conf, disk image, and CMOS image file.

19. At the SDU prompt, give the command `newboot` to bring up the
    bootstrap program.

20. At the `Command:` prompt, give the command `boot` to start lisp.

21. When the REPL arrives, evaluate `(si:set-pack-name "LAMBDA-A")` to set
    the hostname.

22. Evaluate `(si:%halt)` to halt Lisp.

23. When Lisp halts, press F11 to summon Newboot again

24. At the `Command:` prompt, give the boot command to cold-boot Lisp.

25. When the REPL arrives, evaluate `(fs:initialize-file-system)` to 
    format the LMFS. Answer `Yes` when prompted for confirmation.

26. Push F12 to switch tapes to your distribution tape, if you have one.

27. Type System-B to summon the Backup activity.

At this point you can follow the instructions included with the
distribution tape. If it is a backup tape, simply restore it. If it is
an actual distribution tape, it may have special loading procedures.
