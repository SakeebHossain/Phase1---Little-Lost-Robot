#include <stdio.h>
#include <gd.h>

main()
{
FILE *f;
gdImagePtr im;
int black;

f = fopen("castle.jpg", "rb");
im = gdImageCreateFromJpeg(f);
fclose(f);

black = gdImageColorAllocate(im, 0, 0, 0);
f = fopen("castle.wbmp", "wb");
gdImageWBMP(im, black, f);
fclose(f);
}
