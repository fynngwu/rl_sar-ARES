#include "keyboard_helper.hpp"

#include <termios.h>
#include <unistd.h>

#include <cstdlib>

static bool g_terminal_raw = false;
static struct termios g_original_termios;

static void restore_terminal()
{
    if (g_terminal_raw)
        tcsetattr(STDIN_FILENO, TCSANOW, &g_original_termios);
}

void restore_terminal()
{
    if (g_terminal_raw) {
        tcsetattr(STDIN_FILENO, TCSANOW, &g_original_termios);
        g_terminal_raw = false;
    }
}

int kbhit()
{
    if (!isatty(STDIN_FILENO))
        return -1;
    if (!g_terminal_raw) {
        tcgetattr(STDIN_FILENO, &g_original_termios);
        struct termios raw = g_original_termios;
        raw.c_lflag &= ~(ICANON | ECHO);
        raw.c_cc[VMIN] = 0;
        raw.c_cc[VTIME] = 0;
        tcsetattr(STDIN_FILENO, TCSANOW, &raw);
        g_terminal_raw = true;
        atexit(restore_terminal);
    }
    char c;
    if (read(STDIN_FILENO, &c, 1) == 1)
        return static_cast<unsigned char>(c);
    return -1;
}
