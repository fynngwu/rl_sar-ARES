#pragma once

// Returns ASCII key code (e.g. 's', 'r', 'd') or -1 if no key pressed.
// First call sets terminal to raw mode (non-canonical, no echo).
// atexit() handler restores terminal on process exit.
int kbhit();
