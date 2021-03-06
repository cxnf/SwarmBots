#pragma once

#include <cstdio>
#include <string>

#if defined(CONSOLE_COLOR)
#define BLACK "\033[22;30m"
#define RED "\033[22;31m"
#define GREEN "\033[22;32m"
#define YELLOW "\033[22;33m"
#define BLUE "\033[22;34m"
#define MAGENTA "\033[22;35m"
#define CYAN "\033[22;36m"
#define GRAY "\033[22;37m"
#define DARK_GRAY "\033[01;30m"
#define LIGHT_RED "\033[01;31m"
#define LIGHT_GREEN "\033[01;32m"
#define LIGHT_YELLOW "\033[01;33m"
#define LIGHT_BLUE "\033[01;34m"
#define LIGHT_MAGENTA "\033[01;35m"
#define LIGHT_CYAN "\033[01;36m"
#define WHITE "\033[01;37m"
#define COLOR_RESET "\033[39m"
#else
#define BLACK
#define RED
#define GREEN
#define YELLOW
#define BLUE
#define MAGENTA
#define CYAN
#define GRAY
#define DARK_GRAY
#define LIGHT_RED
#define LIGHT_GREEN
#define LIGHT_YELLOW
#define LIGHT_BLUE
#define LIGHT_MAGENTA
#define LIGHT_CYAN
#define WHITE
#define COLOR_RESET
#endif

#if defined(DEBUG) || defined(VERBOSE)
#define PRINT(...) { printf(__VA_ARGS__); printf(COLOR_RESET "\n"); }
#else
#define PRINT(...) { }
#endif
