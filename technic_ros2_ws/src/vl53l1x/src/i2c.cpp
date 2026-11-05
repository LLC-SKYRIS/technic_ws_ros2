#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <arpa/inet.h>

#include <vector>

#include "i2c.h"

static int fd = -1;

bool i2c_setup(uint8_t bus, uint8_t addr)
{
	char filename[50];
	snprintf(filename, sizeof(filename), "/dev/i2c-%d", bus);

	fd = open(filename, O_RDWR);
	if (fd < 0) {
		return false;
	}

	return ioctl(fd, I2C_SLAVE, addr) >= 0;
}

void i2c_release()
{
	if (fd >= 0) {
		close(fd);
		fd = -1;
	}
}

bool i2c_readRegisterMulti(uint16_t reg, size_t count, void *pdata)
{
	reg = htons(reg); // little endian to big endian

	// write register number
	if (write(fd, &reg, sizeof(reg)) < 0) {
		return false;
	}

	return read(fd, pdata, count);
}

bool i2c_readRegisterByte(int reg, uint8_t *pdata)
{
	return i2c_readRegisterMulti(reg, 1, pdata) > 0;
}

bool i2c_readRegisterWord(uint16_t reg, uint16_t *pdata)
{
	if (i2c_readRegisterMulti(reg, 2, pdata) < 0) {
		return false;
	}
	*pdata = ntohs(*pdata); // big endian to little endian
	return true;
}

bool i2c_writeRegisterMulti(uint16_t reg, size_t count, void *pdata)
{
	std::vector<uint8_t> buf(count + sizeof(reg));

	reg = htons(reg); // little endian to big endian

	memcpy(buf.data(), &reg, sizeof(reg));
	memcpy(buf.data() + sizeof(reg), pdata, count);

	return write(fd, buf.data(), buf.size()) > 0;
}

bool i2c_writeRegisterByte(uint16_t reg, uint8_t value)
{
	return i2c_writeRegisterMulti(reg, 1, &value);
}


bool i2c_writeRegisterWord(uint16_t reg, uint16_t value)
{
	value = htons(value); // little endian to big endian
	return i2c_writeRegisterMulti(reg, 2, &value);
}
