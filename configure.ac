#                                               -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.

AC_PREREQ([2.69])
AC_INIT([ld], [0.98.2], [dseagrav@lunar-tokyo.net])
AC_CONFIG_SRCDIR([src/kernel.c])
AM_INIT_AUTOMAKE
AC_CONFIG_SRCDIR([src/syms.h])
AC_CONFIG_HEADERS([src/config.h])

# Checks for programs.
AC_PROG_CC

# Conditionalize 2x2 support
AC_ARG_ENABLE([config_2x2],
	[AS_HELP_STRING([--enable-config-2x2], [Use 2x2 Lambda configuration @<:@default=no@:>@])])

AS_IF([test "x$enable_config_2x2" = "xyes"], [
	CFLAGS="$CFLAGS -DCONFIG_2X2"
	])

# Checks for libraries.
AC_ARG_WITH([SDL1],
	[AS_HELP_STRING([--with-SDL1],
		[Use SDL1 for console @<:@default=check@:>@])],
	[],
	[with_SDL1=check])

AC_ARG_WITH([SDL2],
	[AS_HELP_STRING([--with-SDL2],
		[Use SDL2 for console @<:@default=check@:>@])],
	[],
	[with_SDL2=check])

# Checks for SDL1
SDL_VERSION=1.2.0
LIBSDL1=AS_IF([test "x$with_SDL1" != xno],
		    [AM_PATH_SDL($SDL_VERSION,:,
			AS_IF([test "x$with_SDL1" == xyes],[AC_MSG_ERROR([*** SDL1 version $SDL_VERSION not found!])])
  			)
			GFX_SDL1=TRUE
			CFLAGS="$CFLAGS $SDL_CFLAGS -DSDL1"
			LIBS="$LIBS $SDL_LIBS"
			])

# Checks for SDL2
SDL2_VERSION=2.0.0
LIBSDL2=AS_IF([test "x$with_SDL2" != xno],
		    [AM_PATH_SDL2($SDL2_VERSION,:,
			AS_IF([test "x$with_SDL2" == xyes],[AC_MSG_ERROR([*** SDL2 version $SDL_VERSION not found!])])
			)
			GFX_SDL2=TRUE
			CFLAGS="$CFLAGS $SDL_CFLAGS -DSDL2"
			LIBS="$LIBS $SDL_LIBS"
			])

# Blow up if we ended up with both SDLs or neither SDL
AS_IF([test "$GFX_SDL1" = TRUE],
	    [AS_IF([test "$GFX_SDL2" = TRUE],
	    		 [AC_MSG_ERROR([*** You may use SDL1 or SDL2 but not both!])])],
	    [AS_IF([test "$GFX_SDL2" = TRUE],
	    		 [],
			 [AC_MSG_ERROR([*** You must have either SDL1 or SDL2!])])])

# Checks for header files.
AC_HEADER_STDC
AC_CHECK_HEADERS([fcntl.h netdb.h netinet/in.h stddef.h stdint.h stdlib.h string.h strings.h sys/ioctl.h sys/socket.h unistd.h])
AC_CHECK_HEADERS([linux/if.h], [], [],
[[#ifdef HAVE_SYS_SOCKET_H
#include <sys/socket.h>
#endif
]])
AC_CHECK_HEADERS([linux/if_tun.h], [], [],
[[#ifdef HAVE_SYS_SOCKET_H
#include <sys/socket.h>
#endif
]])
AC_CHECK_HEADERS([net/bpf.h])

# Checks for typedefs, structures, and compiler characteristics.
AC_CHECK_HEADER_STDBOOL
AC_TYPE_INT32_T
AC_TYPE_INT64_T
AC_TYPE_OFF_T
AC_TYPE_SIZE_T
AC_TYPE_SSIZE_T
AC_TYPE_UINT16_T
AC_TYPE_UINT32_T
AC_TYPE_UINT64_T
AC_TYPE_UINT8_T

# Checks for library functions.
AC_FUNC_MALLOC
AC_CHECK_FUNCS([atexit bzero gethostbyname memset socket strcasecmp strdup strtol])

AC_OUTPUT(Makefile src/Makefile)

