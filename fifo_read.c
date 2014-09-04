/* This program is distributed under the GPL, version 2 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <ftdi.h>

int main(int argc, char **argv)
{
    struct ftdi_context *ftdi;
    int f,i;
    unsigned char buf[1];
    int retval = 0;

    if ((ftdi = ftdi_new()) == 0)
    {
        fprintf(stderr, "ftdi_new failed\n");
        return EXIT_FAILURE;
    }

    f = ftdi_usb_open(ftdi, 0x0403, 0x6001);

    if (f < 0 && f != -5)
    {
        fprintf(stderr, "unable to open ftdi device: %d (%s)\n", f, ftdi_get_error_string(ftdi));
        retval = 1;
        goto done;
    }

    printf("ftdi open succeeded: %d\n",f);

    //printf("enabling bitbang mode. All Input !\n");
    //ftdi_set_bitmode(ftdi, 0x00, BITMODE_BITBANG);

    usleep(3 * 1000000);

    

    for (i = 0; i < 32; i++)
    {
        buf[0] =  0;
        f = ftdi_read_pins(ftdi, buf);
        if (f < 0)
        {
            fprintf(stderr,"read failed for 0x%02hhx, error %d (%s)\n",buf[0],f, ftdi_get_error_string(ftdi));
        }
        else
        {
        printf("read data: 0x%02hhx \n",buf[0]);
        }

        usleep(1 * 1000000);
    }

    printf("Close FTDI!\n");

    //printf("disabling bitbang mode\n");
    //ftdi_disable_bitbang(ftdi);

    ftdi_usb_close(ftdi);
done:
    ftdi_free(ftdi);

    return retval;
}
