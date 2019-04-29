#                                               -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.

AC_PREREQ([2.69])
AC_INIT([ld], [0.98.2], [dseagrav@lunar-tokyo.net])
AC_CONFIG_SRCDIR([src/kernel.c])
AC_CONFIG_HEADERS([src/config.h])
AC_CONFIG_MACRO_DIR([m4])
AM_INIT_AUTOMAKE

# Checks for programs.
AC_PROG_CC

# Conditionalize XBEEP
AC_ARG_ENABLE([use_xbeep],
	[AS_HELP_STRING([--enable-use-xbeep], [Enable superior XBEEP hack @<:@default=yes@:>@])])

AS_IF([test "x$enable_use_xbeep" = "xno"], [], [
	CFLAGS="$CFLAGS -DXBEEP"
	])

# Conditionalize UTUN
AC_ARG_ENABLE([use_utun],
	[AS_HELP_STRING([--enable-use-utun], [Use UTUN frame diverter with BPF @<:@default=no@:>@])])

AS_IF([test "x$enable_use_utun" = "xyes"], [
	CFLAGS="$CFLAGS -DUSE_UTUN"
	])

# Conditionalize 2x2 support
AC_ARG_ENABLE([config_2x2],
	[AS_HELP_STRING([--enable-config-2x2], [Use 2x2 Lambda configuration @<:@default=no@:>@])])

AS_IF([test "x$enable_config_2x2" = "xyes"], [
	CFLAGS="$CFLAGS -DCONFIG_2X2"
	])

# Conditionalize physical keyboard support
AC_ARG_ENABLE([config_physkbd],
	[AS_HELP_STRING([--enable-config-physkbd], [Use physical Lambda keyboard @<:@default=no@:>@])])

AS_IF([test "x$enable_config_physkbd" = "xyes"], [
	CFLAGS="$CFLAGS -DCONFIG_PHYSKBD"
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

AC_ARG_WITH([FUSE],
	[AS_HELP_STRING([--with-FUSE],
		[support FUSE-based LMFS filesystem @<:@default=check@:>@])],
	[],
	[with_FUSE=check])

# Checks for SDL1
SDL_VERSION=1.2.0
LIBSDL1=
AS_IF([test "x$with_SDL1" != xno],
		    [AM_PATH_SDL($SDL_VERSION,:,
			AS_IF([test "x$with_SDL1" == xyes],[AC_MSG_ERROR([*** SDL1 version $SDL_VERSION not found!])])
  			)
			GFX_SDL1=TRUE
			CFLAGS="$CFLAGS $SDL_CFLAGS -DSDL1"
			LIBS="$LIBS $SDL_LIBS"
			])

# Checks for SDL2
SDL2_VERSION=2.0.0
LIBSDL2=
AS_IF([test "x$with_SDL2" != xno],
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

# Check for FUSE
LIBFUSE=
AS_IF([test "x$with_FUSE" != xno],
		    [PKG_CHECK_MODULES([fuse], [fuse],
		    [LIB_FUSE=TRUE],
		    [if test "x$with_FUSE" != xcheck; then
		      AC_MSG_FAILURE([--with-FUSE was given, but test for FUSE failed])
	 	     fi])])

# Checks for header files.
AC_HEADER_STDC
AC_CHECK_HEADERS([fcntl.h netdb.h netinet/in.h arpa/inet.h stddef.h stdint.h stdlib.h string.h strings.h])
AC_CHECK_HEADERS([ sys/ioctl.h sys/socket.h sys/time.h unistd.h termios.h utime.h])
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

# Check for structure members
AC_CHECK_MEMBERS([struct stat.st_blksize])

# Do we have libyaml?
AC_CHECK_HEADERS([yaml.h], [LIBS="$LIBS -lyaml"])

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
AC_TYPE_UID_T

AX_CHECK_COMPILE_FLAG([-Wno-packed-bitfield-compat], [CFLAGS="$CFLAGS -Wno-packed-bitfield-compat"], [], [-Werror])
AX_CHECK_COMPILE_FLAG([-Og], [CFLAGS="$CFLAGS -Og"], [], [-Werror])

# Checks for library functions.
AC_FUNC_MALLOC
AC_FUNC_REALLOC
AC_STRUCT_ST_BLOCKS
AC_CHECK_FUNCS([atexit bzero inet_ntoa gethostbyname memset socket strcasecmp strdup strtol strchr])
AC_CHECK_FUNCS([gettimeofday mkdir strerror strstr utime])

# Path propagation
CFLAGS="$CFLAGS -DSYSCONFDIR=${sysconfdir}"

# Done
AC_CONFIG_FILES([Makefile
                 src/Makefile])
# Also make tools
AC_CONFIG_FILES([tools/Makefile])
# If we have FUSE, make lmfuse
AM_CONDITIONAL([COND_LMFUSE],[test "$LIB_FUSE" = TRUE])
AC_CONFIG_FILES([tools/lmfuse/Makefile])
AC_OUTPUT
