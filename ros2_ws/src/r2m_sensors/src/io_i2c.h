int io_i2c_open(uint8_t addr);
void io_i2c_close(int fd);
int io_i2c_write_bytes(int fd, uint8_t *buf, uint16_t nbytes);
int io_i2c_read_bytes(int fd, uint8_t *buf, uint16_t nbytes);

#define I2C_SET_UINT16(_device_,_addr_,_val_) \
{ \
uint8_t data[3] = {_addr_,(_val_>>0)&0xff,(_val_>>8)&0xff}; \
if( io_i2c_write_bytes(_device_, (uint8_t*)&data, sizeof(data)) == sizeof(data) ) \
{ \
} \
}

#define READ_AND_PUB_UINT32(_device_,_addr_,_msg_,_pub_) \
	  { \
		uint8_t addr = _addr_; \
		uint8_t data[4] = {0}; \
		if( io_i2c_write_bytes(_device_, (uint8_t*)&addr, sizeof(addr)) == sizeof(addr) ) \
		{ \
			if( io_i2c_read_bytes(_device_, (uint8_t*)&data, sizeof(data)) == sizeof(data) ) \
			{ \
			  _msg_.data = ((data[0]<<0)|(data[1]<<8)|(data[2]<<16)|(data[3]<<24)); \
			  _pub_->publish(_msg_); \
			} \
		} \
	  }

		#define READ_AND_PUB_UINT64(_device_,_addr_,_msg_,_pub_) \
			  { \
				uint8_t addr = _addr_; \
				uint8_t data[8] = {0}; \
				if( io_i2c_write_bytes(_device_, (uint8_t*)&addr, sizeof(addr)) == sizeof(addr) ) \
				{ \
					if( io_i2c_read_bytes(_device_, (uint8_t*)&data, sizeof(data)) == sizeof(data) ) \
					{ \
					  _msg_.data = (((uint64_t)data[0]<<0)|((uint64_t)data[1]<<8)|((uint64_t)data[2]<<16)|((uint64_t)data[3]<<24)|((uint64_t)data[4]<<32)|((uint64_t)data[5]<<40)|((uint64_t)data[6]<<48)|((uint64_t)data[7]<<56)); \
					  _pub_->publish(_msg_); \
					} \
				} \
			  }

				#define READ_AND_PUB_INT16(_device_,_addr_,_msg_,_pub_) \
					  { \
						uint8_t addr = _addr_; \
						uint8_t data[2] = {0}; \
						if( io_i2c_write_bytes(_device_, (uint8_t*)&addr, sizeof(addr)) == sizeof(addr) ) \
						{ \
							if( io_i2c_read_bytes(_device_, (uint8_t*)&data, sizeof(data)) == sizeof(data) ) \
							{ \
							  _msg_.data = (((int16_t)data[0]<<0)|((int16_t)data[1]<<8)); \
							  _pub_->publish(_msg_); \
							} \
						} \
					  }

				#define READ_AND_PUB_INT64(_device_,_addr_,_msg_,_pub_) \
					  { \
						uint8_t addr = _addr_; \
						uint8_t data[8] = {0}; \
						if( io_i2c_write_bytes(_device_, (uint8_t*)&addr, sizeof(addr)) == sizeof(addr) ) \
						{ \
							if( io_i2c_read_bytes(_device_, (uint8_t*)&data, sizeof(data)) == sizeof(data) ) \
							{ \
							  _msg_.data = (((int64_t)data[0]<<0)|((int64_t)data[1]<<8)|((int64_t)data[2]<<16)|((int64_t)data[3]<<24)|((int64_t)data[4]<<32)|((int64_t)data[5]<<40)|((int64_t)data[6]<<48)|((int64_t)data[7]<<56)); \
							  _pub_->publish(_msg_); \
							} \
						} \
					  }

#define READ_AND_PUB_FLOAT32(_device_,_addr_,_msg_,_pub_) \
	  { \
		uint8_t addr = _addr_; \
		uint8_t data[2] = {0}; \
		if( io_i2c_write_bytes(_device_, (uint8_t*)&addr, sizeof(addr)) == sizeof(addr) ) \
		{ \
			if( io_i2c_read_bytes(_device_, (uint8_t*)&data, sizeof(data)) == sizeof(data) ) \
			{ \
			  _msg_.data = ((data[0]<<0)|(data[1]<<8))/1000.0; \
			  _pub_->publish(_msg_); \
			} \
		} \
	  }
