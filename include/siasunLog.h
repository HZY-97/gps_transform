#ifndef SIASUN_LOG_H
#define SIASUN_LOG_H
#include <stdio.h>
#include <sys/time.h>

// clang-format off
#define PRINT_RED       "\x1b[31m"
#define PRINT_GREEN     "\x1b[32m"
#define PRINT_YELLOW    "\x1b[33m"
#define PRINT_BLUE      "\x1b[34m"
#define PRINT_MAGENTA   "\x1b[35m"
#define PRINT_CYAN      "\x1b[36m"
#define PRINT_RESET     "\x1b[0m"

#ifdef OPEN_LOG
#define SIA_DEBUG(fmt, ...) printf("[DEBUG]: ");printf(fmt, ##__VA_ARGS__);printf ("\n");
#define SIA_INFO(fmt, ...) printf(PRINT_GREEN "[INFO]: ");printf(fmt, ##__VA_ARGS__);printf (PRINT_RESET "\n");
#define SIA_WARN(fmt, ...) printf(PRINT_YELLOW "[WARN]: ");printf(fmt, ##__VA_ARGS__);printf (PRINT_RESET "\n");
#define SIA_ERROR(fmt, ...) printf(PRINT_RED "[ERROR]: ");;printf(fmt, ##__VA_ARGS__);printf (PRINT_RESET "\n");
#else
#define SIA_DEBUG
#define SIA_INFO
#define SIA_WARN
#define SIA_ERROR
#endif


#endif