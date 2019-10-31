int io_i2c_open(uint8_t addr);
void io_i2c_close(int fd);
int io_i2c_write_bytes(int fd, uint8_t *buf, uint16_t nbytes);
int io_i2c_read_bytes(int fd, uint8_t *buf, uint16_t nbytes);
