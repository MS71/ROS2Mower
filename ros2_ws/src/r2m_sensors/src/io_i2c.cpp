#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#if 0 
char buf[10];
extern int com_serial;
extern int failcount;
 
 
//////////
// Set pointer address
//////////
void i2c_set_pointer(int address,int value,int file)
    {
    /*
    //printf("end: 0x%x%x\n", address,value);
    if (i2c_smbus_write_byte_data(file, address, value)<0)
        {
        fprintf(stderr, "Warning - write failed\n");
        } 
    */
    char buf[10];
    buf[0]=address;
    buf[1]=value;
 
    if (write(file, buf, 2) != 2)
        {
        fprintf(stderr, "Error setting pointer\n");
        } 
    else
        {
        }
 
    }
 
//////////
// Read n bytes
//////////
 
char * i2c_read(int add1, int add2, int nbytes,int file)
    {
    int n;
 
    i2c_set_pointer(add1,add2,file);
 
    if (read(file, buf, nbytes) != nbytes)
        {
        fprintf(stderr, "Error reading %i bytes\n",nbytes);
        com_serial=0;
        failcount++;
        } 
    else
        {
        for (n=0;n<nbytes;n++)
            {
            //printf("r_0x%0*x\n", 2, buf[n]);
            }
        com_serial=1;
        failcount=0;
        return buf;
        }
    }
#endif

#if 0
//////////
// Init I2Cdevice
//////////
int i2c_init(const char* filename, int addr)
{
    int file;

    if((file = open(filename, O_RDWR)) < 0) {
	printf("Failed to open the bus.\n");
	/* ERROR HANDLING; you can check errno to see what went wrong */
	exit(1);
    }

    if(ioctl(file, I2C_SLAVE, addr) < 0) {
	printf("Failed to acquire bus access and/or talk to slave.\n");
	/* ERROR HANDLING; you can check errno to see what went wrong */
	exit(1);
    }
    return file;
}

//////////
// Write n bytes
//////////
void i2c_write(int add1, int add2, int nbytes, char value[10], int file)
{
    int n;
    unsigned char buf[10];
    buf[0] = add1;
    buf[1] = add2;
    if(nbytes >= 1)
	buf[2] = value[0];
    if(nbytes >= 2)
	buf[3] = value[1];
    if(nbytes >= 3)
	buf[4] = value[2];
    if(nbytes >= 4)
	buf[5] = value[3];

    if(write(file, buf, nbytes) != nbytes) {
		fprintf(stderr, "Error writing %i bytes\n", nbytes);
    }
}
#endif

#define I2CDEV "/dev/i2c-ros"

int io_i2c_open(uint8_t addr)
{
    int fd;

    if((fd = open(I2CDEV, O_RDWR)) < 0) {
	printf("Failed to open the bus.\n");
	return -1;
    }

    if(ioctl(fd, I2C_SLAVE, addr) < 0) {
	printf("Failed to acquire bus access and/or talk to slave.\n");
	return -1;
    }
	
	return fd;
}

void io_i2c_close(int fd)
{
	if( fd != -1 )
	{
		close(fd);
	}
}

int io_i2c_write_bytes(int fd, uint8_t *buf, uint16_t nbytes)
{
	int rc;
	if( fd == -1 )
	{
		return -1;
	}

    if((rc = write(fd, buf, nbytes)) != nbytes) {
		fprintf(stderr, "Error writing %i bytes\n", nbytes);
    }
	return rc;
}

int io_i2c_read_bytes(int fd, uint8_t *buf, uint16_t nbytes)
{
	int rc;
	if( fd == -1 )
	{
		return -1;
	}

    if((rc = read(fd, buf, nbytes)) != nbytes) {
		fprintf(stderr, "Error writing %i bytes\n", nbytes);
    }
	return rc;
}
