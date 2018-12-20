#include <stdio.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include "../tm1638.h"

#define DEVFILE "/dev/tm1638_dev"

int main(void)
{
    int fd;
    int ret;
    struct display_digit *dis_show;
    int i;

    dis_show = (struct display_digit *) malloc(sizeof(*dis_show));
    fd = open(DEVFILE, O_RDWR);
    if (fd == -1)
        perror("open");


    dis_show->position = 0;
    dis_show->dot = 1;

    for (i = 0; i < 10; i++) {
        dis_show->digit = i;
        ret = ioctl(fd, IOCTL_DISPLAY_SEG, dis_show);
        sleep(1);
    }

    getchar();
    if (close(fd))
        perror("close");

    if (dis_show)
        free(dis_show);

    return 0;
}
