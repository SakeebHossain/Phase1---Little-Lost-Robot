#include <stdio.h>

int main()
{
int ret;
char *bla;

ret = nxt_bluetooth_initialize("00:16:53:03:96:6B");
if (ret) {
	printf("Failed to initialize bluetooth\n");
        exit(1);
        }

bla = get_firmware_version();
printf("Return is >%s<\n", bla);
nxt_bluetooth_done();
return(0);
}
