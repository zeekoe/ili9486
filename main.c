
#include "ili9486.h"

#include <memory.h>
#include <stdio.h>
#include <unistd.h>


int main() {

    initDisplay();

    unsigned short row[480] = {0};

    for(int i = 0; i < 480; i++) {
        row[i] = (short) 0b1000000000000000;
    }
    for (int i = 0; i < 320; i++) {
        drawRowRaw(i, row);
    }

    return 0;
}
