#include <stdio.h>

void testthis(unsigned char *bla)
{
*bla=99;
}

main()
{
unsigned char x = 43;

testthis(&x);
printf("%d\n", x);
}
