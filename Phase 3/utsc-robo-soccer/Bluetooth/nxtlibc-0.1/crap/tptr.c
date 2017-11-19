#include <stdio.h>
#include <unistd.h>
#include <string.h>

char mybuf[1024];
char newbuf[1024];

void do_bla(char **buf, int *len)
{
int i;
for (i = 0; i < 1024; i++) {
	memcpy(*buf + i, &mybuf[i], 1);
	}
}

void setup_buf()
{
int i = 0;
unsigned char pos = 33;

/* 33 - 125 */
for (i = 0; i < 1024; i++) {
	mybuf[i] = (unsigned char) pos;
	if (pos == (unsigned char) 125) {
		pos = (unsigned char) 33;
		} else {
		pos += (unsigned char) 1;
		}
	}
}

void print_buf(char *buf, int len) {
int i;

printf("-- %d\n", strlen(buf));
for (i = 0; i <= len; i++) {
	printf("%c", (unsigned char)buf[i]);
	}
printf("\n");
}

void do_write_copy(char *nbuf, int cp, int cp2)
{
/*printf("do_write_copy: cp %d cp2 %d\n", cp, cp2);*/
memcpy(&newbuf[cp], nbuf, cp2);
}

void do_write(char *buf, int len)
{
int pos = len, cp = 0, cp2 = 0, mylen = len;
/*
while (cp2 < len) {
	printf("doing cp %d cp2 %d mylen %d\n", cp, cp2, mylen);
	memcpy(&newbuf[cp2], buf + cp2, cp);
	cp = mylen >= 56 ? 56 : mylen;
	cp2 += mylen >= 56 ? 56 : mylen;
	mylen -= 56;
	}
*/
for (cp = 0; cp < len; cp += 56) {
	cp2 = (len - cp > 56) ? 56 : (len - cp - 1);
	printf("doing cp %d cp2 %d\n", cp, cp2);
	do_write_copy(&buf[cp], cp, cp2);
	}
print_buf(&newbuf, 1024);
}

void testthis(int bla)
{
/*	bla=49;*/
}

main()
{
char *bla;
int *blanum;

setup_buf();
print_buf(&mybuf, 1024);
do_bla((char **)&bla, (int *)&blanum);
print_buf(bla, 1024);
/*
do_write(bla, 1024);
testthis(x);
printf("%d\n", x);*/
}
