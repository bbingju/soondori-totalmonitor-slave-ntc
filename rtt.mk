USE_RTT_FOR_DEBUG = 0

RTT_C_SOURCES = \
Lib/RTT/SEGGER_RTT.c \
Lib/RTT/SEGGER_RTT_printf.c \
Lib/Syscalls/SEGGER_RTT_Syscalls_GCC.c

RTT_C_INCLUDES = \
-ILib/RTT
