
#include "ili9486.h"
#include "armbianio.h"

#include <memory.h>
#include <stdio.h>


int main() {
    int i, rc;
    const char *szBoardName;

    rc = AIOInit();
    if (rc == 0)
    {
        printf("Problem initializing ArmbianIO library\n");
        return 0;
    }
    szBoardName = AIOGetBoardName();
    printf("Running on a %s\n", szBoardName);


    InitILI9486();
    AIOShutdown();
    return 0;
}
