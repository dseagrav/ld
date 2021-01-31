<!-- TODO: make more pretty, also see if we can't just use this file for both the YAML and old config's documentation-->
# Old Configuration File:
**Note: This file is provided for people who do not have libyaml on their system and wish to configure LambdaDelta without that dependency. However, it will not continue to be supported.**

  The old configuration file is named "ld.conf" and must be in the current
  working directory. An example configuration is provided in the examples
  directory. Each line of the configuration file has a keyword followed by
  a number of parameters. The following are the supported keywords
  and their parameters:

  ether_iface IFACE

  Causes the ethernet interface to be bound to the specified interface.
  For tuntap, this is the name of the tuntap device to use.
  For BPF, this is the interface the BPF interface will be bound to.
  As this is used for the passing of raw frames, it must be a wired
  Ethernet interface (or something emulating one) - A wireless interface
  will not work. The default is "ldtap".

  ether_addr MAC-ADDRESS

  Sets the MAC address used by the Lambda. This must be unique on your
  LAN, so do not make it the same as the host! It is written in the
  canonical colon-separated hex format.
  The default is 00:02:9C:55:89:C6.
  You can also specify LAA (case-insensitive) to generate a Locally Administered Address,
  which is saved persistently per interface in the file $IFNAME.LAA.

  disk N FNAME

  Sets the image file for disk unit N. The given FNAME must exist.
  The geometry of the disk is determined by settings in the SDU.
  The default is that unit 0 is disks/disk.img and all other units
  are absent.

  sdu_switch N

  Sets the position of the SDU's rotary switch. N is a single hex digit 0-F.
  Position 0 is used for initial software installation or recovery.
  Position 1 is the standard configuration. The other positions operate as
  determined by the SDU configuration. If the position is not 1, LambdaDelta
  will wait for a telnet connection on port 3637 after startup. This simulates
  a terminal connected to the SDU's serial port. 
  The default is 0.

  mouse_mode N
  
  Sets the operation mode of the mouse emulation. In mode 0, the mouse
  protocol is emulated directly using SDL relative mouse movement data.
  The lisp mouse pointer moves independently of the host mouse pointer,
  and movement may be difficult or unpredictable on some platforms.
  In mode 1, the position of the host's mouse pointer is written
  directly into the Lambda processor's memory, causing the mouse pointer
  to remain in sync with the host but requiring knowledge of the offsets
  at which the pointer data resides. For more information, see the usage
  instructions later in this document.
  The default is 1.

  mouse_x_loc_0 N
  mouse_y_loc_0 N
  mouse_wake_loc_0 N
  mouse_x_loc_1 N
  mouse_y_loc_1 N
  mouse_wake_loc_1 N

  Sets the memory offsets used for mouse operation mode 1. The 0 or 1
  indicates Lambda 0 or Lambda 1. N is an octal offset into A-memory. The
  offsets are for the X coordinate, the Y coordinate, and the mouse updation
  flag. You should only have to modify this if you modify the microcode.
  The defaults are 0156, 0157, and 0170 for both processors.

  pixel_on 0xVVVVVVVV
  pixel_off 0xVVVVVVVV

  Sets the pixel values used for on and off. The standard pixel format is
  AARRGGBB where AA is Alpha, RR is Red, GG is Green, BB is Blue. The default
  pixel values are 0xFFFFFFFF for on and 0x00000000 for off.

  input_fps N
  video_fps N

  Sets the framerate used for video output or user input. These are updated
  in the decisecond loop, so values less than ten will not work as expected,
  and will be set to ten. The default values are 60 input frames per second
  and 10 video frames per second.
  Note that the video frame rate does not affect updation of the title bar,
  only the rate at which the SDL framebuffer is refreshed.

  sdl_quit STATE

  Enables or disables handling of the platform-specific SDL_QUIT event.
  (Alt-F4,Command-Q,etc.) STATE is one of on,off,yes,no,true,or false.
  The default is enabled.

  map_key HV LV

  Maps host key HV to Lambda key LV.
  HV is a decimal SDL key code, LV is an octal Lambda key code.
  See the KEYCODES file in the doc directory for a list of Lambda key codes.
  See the SDL documentation for your SDL version to find the SDL key codes.
  SDL1 codes start with SDLK_ and SDL2 codes start with SDL_SCANCODE_.
  The default keymap is described later in this document.
