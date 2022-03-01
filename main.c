
#include "ili9486.h"

#include <memory.h>
#include <stdio.h>
#include <unistd.h>


int main() {

    initDisplay();

    usleep(1000*2000);
    printf("en nu weer uit\n");

    deInitDisplay();
    return 0;
}
