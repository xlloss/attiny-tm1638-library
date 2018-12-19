#include <stdio.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include "../tm1638.h"

#define DEVFILE "/dev/tm1638_dev"

int main(void)
{
    int fd;
    int ret;

    fd = open(DEVFILE, O_RDWR);
    if (fd == -1) {
            perror("open");
    }

    ret = ioctl(fd, IOCTL_DISPLAY_SEG, NULL);

    if (close(fd) != 0) {
            perror("close");
    }

    return 0;
}
