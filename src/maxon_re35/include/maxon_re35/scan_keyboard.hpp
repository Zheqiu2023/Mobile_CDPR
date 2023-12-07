#include <stdio.h>
#include <termio.h>

int scanKeyboard() {
    int in;
    struct termios new_settings;
    struct termios stored_settings;
    tcgetattr(STDIN_FILENO, &stored_settings);
    new_settings = stored_settings;
    new_settings.c_lflag &= (~ICANON);
    new_settings.c_cc[VTIME] = 0;
    tcgetattr(STDIN_FILENO, &stored_settings);
    new_settings.c_cc[VMIN] = 1;
    tcsetattr(STDIN_FILENO, TCSANOW, &new_settings);

    in = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &stored_settings);
    return in;
}
