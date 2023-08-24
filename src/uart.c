#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <linux/termios.h>
// #include <sys/ioctl.h>

int ioctl(int fd, unsigned long request, ...);

int change_baud(int fd, int baud)
{
	struct termios2 t;
	if (ioctl(fd, TCGETS2, &t))
	{
		return -1;
	}
	t.c_cflag &= ~CBAUD;
	t.c_cflag |= BOTHER;
	t.c_ispeed = baud;
	t.c_ospeed = baud;

	if (ioctl(fd, TCSETS2, &t))
	{
		return -2;
	}

#if 1
	if (ioctl(fd, TCGETS2, &t) == 0)
	{
		if (abs(t.c_ospeed - baud) <= 20000)
		{
			printf("reported %d\n", t.c_ospeed);
		}
		else
		{
			printf("reported err set:%d  result:%d\n", baud, t.c_ospeed);
			return -3;
		}
	}
#endif

	return 0;
}
