#include <string.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/loop.h> /* ioctl_list */

int main(int argc, char *argv[])
{
	char *path, cache_type;

	if (argc != 2) {
		fprintf(stderr, "usage: %s /dev/minibd0 \n", argv[0]);
		return 1;
	}

	path = argv[1];

	int fd = open(path, O_RDWR);

	printf("opened\n");

	if (fd < 0) {
		perror("Failed to open device");
		return 1;
	}


	for( int idx = 0 ; idx < 5 ; idx ++ )
	{
		const int buffer_size = 512;
		char buf[buffer_size +1];
		int n = read( fd, buf, buffer_size );
		printf(" read idx = %d, n = %d\n",idx,n);
	}

//	int cmd = LOOP_SET_FD;

    /* enable cache with type */
/*
	if (ioctl(fd, cmd, cache_type)) {
		perror("ioctl");
		return 1;
	}
*/

	printf("closed\n");

	if (close(fd) < 0) {
		perror("close");
		return 1;
	}

	return 0;
}
