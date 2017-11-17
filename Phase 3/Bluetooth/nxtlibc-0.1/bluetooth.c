/*
 * nxtlibc, the lego mindstorms nxt bluetooth api.
 * Copyright (C) 2007 Don Neumann
 * 
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

/*
 * http://people.csail.mit.edu/albert/bluez-intro/c401.html
 */

#include <stdio.h>
#include <errno.h>
#include <ctype.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <getopt.h>
#include <errno.h>
#include <sys/param.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/hci_lib.h>
#include <bluetooth/rfcomm.h>

#include "nxtlibc.h"


#ifdef DEBUG
extern int dodebug = 1;
#else
extern int dodebug = 0;
#endif

struct err {
	unsigned char val;
	char *text;
};

struct err nxterrors[] = {
	{ 0x20, "Pending communication transaction in progress"},
	{ 0x40, "Specified mailbox queue is empty"},
	{ 0xBD, "Request failed (i.e. specified file not found)"},
	{ 0xBE, "Unknown command opcode"},
	{ 0xBF, "Insane packet"},
	{ 0xC0, "Data contains out-of-range values"},
	{ 0xDD, "Communication bus error"},
	{ 0xDE, "No free memory in communication buffer"},
	{ 0xDF, "Specified channel/connection is not valid"},
	{ 0xE0, "Specified channel/connection not configured or busy"},
	{ 0xEC, "No active program"},
	{ 0xED, "Illegal size specified"},
	{ 0xEE, "Illegal mailbox queue ID specified"},
	{ 0xEF, "Attempted to access invalid field of a structure"},
	{ 0xF0, "Bad input or output specified"},
	{ 0xFB, "Insufficient memory available"},
	{ 0XFF, "Bad arguments 0xFF"},
	/* split here */
	{ 0x81, "No more handles"},
	{ 0x82, "No space"},
	{ 0x83, "No more files"},
	{ 0x84, "End of file expected"},
	{ 0x85, "End of file"},
	{ 0x86, "Not a linear file"},
	{ 0x87, "File not found"},
	{ 0x88, "Handle all ready closed"},
	{ 0x89, "No linear space"},
	{ 0x8A, "Undefined error"},
	{ 0x8B, "File is busy"},
	{ 0x8C, "No write buffers"},
	{ 0x8D, "Append not possible"},
	{ 0x8E, "File is full"},
	{ 0x8F, "File exists"},
	{ 0x90, "Module not found"},
	{ 0x91, "Out of boundary"},
	{ 0x92, "Illegal file name"},
	{ 0x93, "Illegal handle"},
	{ 0x0, NULL }
};

/*static void cmd_cc(int dev_id, int argc, char **argv)*/

int devcount = 0;
int nxtsock = 0;

char *MYADDR = NULL;

void debug_buf(char *buf, int len) {
	int i;

	for (i = 0; i < len; i++) {
		printf("%02x ", (unsigned char)buf[i]);
		}
	printf("\n");
}

char *_nxt_error(unsigned char val) {
	int i = 0;

	while (nxterrors[i].text != NULL) {
		if (val == nxterrors[i].val) return(nxterrors[i].text);
		i++;
		}
	return(NULL);
}

int _do_cmd(char *sbuf, int slen, char **rbuf, int *rlen, int errpos) { 

	fd_set set;
	int maxfd = nxtsock;
	struct timeval timeout = {0,2};
	int i;
	unsigned char buf[1024], lbuf[1024];
	ssize_t x, xr;
	unsigned short int si  = (short int) slen;
	unsigned short int sr;
	char *eptr = NULL;

	FD_ZERO(&set);
	FD_SET(nxtsock, &set);

	memcpy(&buf[0], &si, 2);
	memcpy(&buf[2], sbuf, slen);

	*rbuf = NULL;

	x = write(nxtsock, &buf, slen + 2);
	if (x < 0) {
		perror("write: ");
		return(-1);
	}
	if (dodebug) { 
//		printf("write x %d: ", slen+2);
		debug_buf((unsigned char *)&buf, slen + 2);
	}

	if ((unsigned char)sbuf[0] == NXT_NORET) {
		 return(0); 
	}

	while  (select(maxfd, &set, 0, 0, &timeout) < 0) {
		printf("got nothing\n");
	}

	while ((xr = read(nxtsock, &lbuf, 2)) == -1) {
		// EMPTY nothing by default?
	}

	if (xr != 2) {
		perror("read: ");
		return(-1);
	}

	memcpy(&sr, &lbuf[0], 2);

	while ((xr = read(nxtsock, &lbuf, (int)sr)) == -1) {
		// EMPTY nothing by default
	}

//	printf("read x %d\n", xr);
	if (xr != (int)sr) { printf("got mismatch of %d and %d\n", xr, sr); }

	if ((unsigned char)lbuf[errpos] != 0) {
		int ret = (unsigned char) lbuf[errpos];
		if (dodebug) {
			eptr = _nxt_error((unsigned char) lbuf[errpos]);
			fprintf(stderr, "Error %02x: %s\n", (unsigned char)lbuf[errpos], eptr ? eptr : "Unknown");
		}
		return(ret);
	}

//	printf("return buf size %d\n", sr);
	*rbuf = (unsigned char *) malloc((int)sr);
	memcpy(*rbuf, &lbuf, (int)sr);
	*rlen = (int)sr;

	if (dodebug) {
		debug_buf(*rbuf, *rlen);
	}
	return(0);
}

int _do_connect(char *btaddr) {

	int rv;
	struct sockaddr_rc addr = { 0 };
	int s, status;
	char dest[18];

	MYADDR = (char *) strdup(btaddr);
	memcpy(&dest, btaddr, 18);
	// allocate a socket
	nxtsock = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);

	// set the connection parameters (who to connect to)
	addr.rc_family = AF_BLUETOOTH;
	addr.rc_channel = (uint8_t) 1;
	str2ba( dest, &addr.rc_bdaddr );

	status = connect(nxtsock, (struct sockaddr *)&addr, sizeof(addr));
	if( status == 0 ) {
		if (dodebug) printf("rfcomm connection to %s good.\n", dest);
	}
	if( status < 0 ) {
		perror("rfcomm failed: ");
		return(1);
	}

	rv = fcntl(nxtsock, F_SETFL, O_NONBLOCK);
	if (rv == 0) {
		if (dodebug) { printf("nonblock set\n"); }
	}
	else {
		perror("nonblock failed: ");
		return(1);
	}

	return(0);
}

void _do_close() {

	close(nxtsock);
}


static int dev_info(int s, int dev_id, long arg) {
	/*struct hci_dev_list_req *dl;*/
	struct hci_dev_info di = { dev_id: dev_id };
	char addr[18];

/*	if (ioctl(s, HCIGETDEVLIST, (void *) dl) < 0) {
		printf("no adapter\n");
		return 0;
		}*/

	if (ioctl(s, HCIGETDEVINFO, (void *) &di)) {
		printf("no adapter\n");
		return 0;
		}

	devcount++;
	ba2str(&di.bdaddr, addr);
	/*printf("\t%s\t%s\n", di.name, addr);*/
	return 0;
}


int have_adapter() {

	int ret = hci_for_each_dev(HCI_UP, dev_info, 0);
	printf("devcount %d ret %d\n", devcount, ret);
	return(devcount);
}


static int find_conn(int s, int dev_id, long arg) {

        struct hci_conn_list_req *cl;
        struct hci_conn_info *ci;
        int i;

        if (!(cl = malloc(10 * sizeof(*ci) + sizeof(*cl)))) {
                perror("Can't allocate memory");
                return(1);
        }
        cl->dev_id = dev_id;
        cl->conn_num = 10;
        ci = cl->conn_info;

        if (ioctl(s, HCIGETCONNLIST, (void *) cl)) {
                perror("Can't get connection list");
                return(1);
        }

        for (i = 0; i < cl->conn_num; i++, ci++)
                if (!bacmp((bdaddr_t *) arg, &ci->bdaddr))
                        return 1;

        return 0;
}


int cmd_cc()
{
	int dev_id;
        bdaddr_t bdaddr;
        uint16_t handle;
        uint8_t role;
        unsigned int ptype;
        int dd;

	printf("in cmd_cc\n");
        role = 0x01;
        ptype = HCI_DM1 | HCI_DM3 | HCI_DM5 | HCI_DH1 | HCI_DH3 | HCI_DH5;

/*
        for_each_opt(opt, cc_options, NULL) {
                switch (opt) {
                case 'p':
                        hci_strtoptype(optarg, &ptype);
                        break;

                case 'r':
                        role = optarg[0] == 'm' ? 0 : 1;
                        break;

                default:
:
                        printf(cc_help);
                        return;
                }
        }
        argc -= optind;
        argv += optind;

        if (argc < 1) {
                printf(cc_help);
                return;
        }
*/

        /*str2ba(arg.., &bdaddr);*/
	printf("1\n");
        str2ba(MYADDR, &bdaddr);

        if (dev_id < 0) {
                dev_id = hci_get_route(&bdaddr);
                if (dev_id < 0) {
                        fprintf(stderr, "Device is not available.\n");
                        return(1);
                }
        }
	printf("2\n");
        dd = hci_open_dev(dev_id);
        if (dd < 0) {
                perror("HCI device open failed");
                return(1);
        }

	printf("3\n");
        if (hci_create_connection(dd, &bdaddr, htobs(ptype),
                                htobs(0x0000), role, &handle, 25000) < 0) {
                perror("Can't create connection");
		return(1);
		}

	printf("4\n");
        hci_close_dev(dd);
	return(0);
}

int cmd_auth()
{
        struct hci_conn_info_req *cr;
        bdaddr_t bdaddr;
        int dd;
	int dev_id;

/*
        for_each_opt(opt, auth_options, NULL) {
                switch (opt) {
                default:
                        printf(auth_help);
                        return;
                }
        }
        argc -= optind;
        argv += optind;

        if (argc < 1) {
                printf(auth_help);
                return;
        }
*/

        str2ba(MYADDR, &bdaddr);

        if (dev_id < 0) {
                dev_id = hci_for_each_dev(HCI_UP, find_conn, (long) &bdaddr);
                if (dev_id < 0) {
                        fprintf(stderr, "Not connected.\n");
                        return(1);
                }
        }

        dd = hci_open_dev(dev_id);
        if (dd < 0) {
                perror("HCI device open failed");
                return(1);
        }

        cr = malloc(sizeof(*cr) + sizeof(struct hci_conn_info));
        if (!cr) {
                perror("Can't allocate memory");
                return(1);
        }

        bacpy(&cr->bdaddr, &bdaddr);
        cr->type = ACL_LINK;
        if (ioctl(dd, HCIGETCONNINFO, (unsigned long) cr) < 0) {
                perror("Get connection info failed");
                return(1);
        }

        if (hci_authenticate_link(dd, htobs(cr->conn_info->handle), 25000) < 0) 
{
                perror("HCI authentication request failed");
                return(1);
        }

        free(cr);

        hci_close_dev(dd);
	return(0);
}


int scan() {

    inquiry_info *ii = NULL;
    int max_rsp, num_rsp;
    int dev_id, sock, len, flags;
    int i;
    char addr[19] = { 0 };
    char name[248] = { 0 };

    dev_id = hci_get_route(NULL);
    sock = hci_open_dev( dev_id );
    if (dev_id < 0 || sock < 0) {
        perror("opening socket");
        return(1);
    }

    len  = 8;
    max_rsp = 255;
    flags = IREQ_CACHE_FLUSH;
    ii = (inquiry_info*)malloc(max_rsp * sizeof(inquiry_info));

    num_rsp = hci_inquiry(dev_id, len, max_rsp, NULL, &ii, flags);
	printf("1\n");
    if( num_rsp < 0 ) perror("hci_inquiry");
	printf("2\n");

    for (i = 0; i < num_rsp; i++) {
	printf("3\n");
        ba2str(&(ii+i)->bdaddr, addr);
	printf("4\n");
        memset(name, 0, sizeof(name));
	printf("5\n");
        if (hci_read_remote_name(sock, &(ii+i)->bdaddr, sizeof(name), 
            name, 0) < 0)
        strcpy(name, "[unknown]");
	printf("6\n");
        printf("%s  %s\n", addr, name);
	printf("7\n");
    }

    free( ii );
    close( sock );
    return 0;
}


int cmd_dc(int dev_id, int argc, char **argv) {

	struct hci_conn_info_req *cr;
	bdaddr_t bdaddr;
	int dd;

	/*
	for_each_opt(opt, dc_options, NULL) {
		switch (opt) {
		default:
			printf(dc_help);
			return;
		}
	}
	argc -= optind;
	argv += optind;

	if (argc < 1) {
		printf(dc_help);
		return;
	}
*/
	str2ba(MYADDR, &bdaddr);

	if (dev_id < 0) {
		dev_id = hci_for_each_dev(HCI_UP, find_conn, (long) &bdaddr);
		if (dev_id < 0) {
			fprintf(stderr, "Not connected.\n");
			return(1);
		}
	}

	dd = hci_open_dev(dev_id);
	if (dd < 0) {
		perror("HCI device open failed");
		return(1);
	}

	cr = malloc(sizeof(*cr) + sizeof(struct hci_conn_info));
	if (!cr) {
		perror("Can't allocate memory");
		return(1);
	}

	bacpy(&cr->bdaddr, &bdaddr);
	cr->type = ACL_LINK;
	if (ioctl(dd, HCIGETCONNINFO, (unsigned long) cr) < 0) {
		perror("Get connection info failed");
		return(1);
	}

	if (hci_disconnect(dd, htobs(cr->conn_info->handle),
				HCI_OE_USER_ENDED_CONNECTION, 10000) < 0)
		perror("Disconnect failed");

	free(cr);

	hci_close_dev(dd);
	return(0);
}
