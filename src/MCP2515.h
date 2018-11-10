#include <stdint.h>
#include <stdlib.h>

// #define ENABLE_RTR

class MCP2515
{
	public:
		typedef enum
		{
			OK,
			NO_MESSAGE,
			HAS_MESSAGE,
			FAIL = 128,
			TIMEOUT,
			NO_BUFFER,
		} result_t;

		typedef enum : uint8_t
		{
			MODE_NORMAL = 0x00,
			MODE_SLEEP  = 0x20,
			MODE_LOOPBACK = 0x40,
			MODE_LISTENONLY = 0x60,
			MODE_CONFIG = 0x80,
		} mode_t;

		typedef enum : uint8_t
		{
			RATE_5KBPS = 1,
			RATE_10KBPS,
			RATE_20KBPS,
			RATE_40KBPS,
			RATE_50KBPS,
			RATE_80KBPS,
			RATE_100KBPS,
			RATE_125KBPS,
			RATE_200KBPS,
			RATE_250KBPS,
			RATE_500KBPS,
		} bitrate_t;

		// Filters
		typedef enum : uint8_t
		{
			FILTER0 =         0x00,
			FILTER1 =         0x04,
			FILTER2 =         0x08,
			FILTER3 =         0x10,
			FILTER4 =         0x14,
			FILTER5 =         0x18
		} rxfilter_t;

		// Masks
		typedef enum : uint8_t 
		{
			MASK0 =           0x20, // Mask 0 ID starts here
			MASK1 =           0x24  // Mask 1 ID starts here
		} rxmask_t;

		// Filter Types
		typedef enum : uint8_t
		{
			RECEIVE_NOFILTER = 0x60,         // Turn Filters/Masks off
			RECEIVE_STD =     0x20,          // Extenden messages only
			RECEIVE_EXT =     0x40,          // Standard Messages only
			RECEIVE_STDEXT =  0x00           // Std+Ext, Filters/Masks ON
		} messagetype_t;

		// config
		result_t        begin(const bitrate_t rate, const mode_t mode = MODE_LISTENONLY, const uint8_t cs_pin = 10);
		result_t        setRate(const bitrate_t rate);
		result_t        setMode(const mode_t newmode);
		void            setMessageType(const messagetype_t);
		const void      printStatus();
		const uint8_t   getReceiveErrorCounter();
		const uint8_t   getTransmitErrorCounter();

		// Standard
		result_t        setStdMask(const rxmask_t, const uint16_t mask, const uint8_t data0 = 0, const uint8_t data1 = 0);
		result_t       	setStdFilter(const rxfilter_t, const uint16_t filter, const uint8_t data0 = 0, const uint8_t data1 = 0);
		result_t       	sendStdMessage(const uint16_t id, const uint8_t len, const uint8_t *data);

		// Extended
		result_t		setExtMask(const rxmask_t, const uint32_t mask);
		result_t		setExtFilter(const rxfilter_t, const uint32_t filter);
		result_t       	sendExtMessage(const uint32_t id, const uint8_t len, const uint8_t *data);

		// Read Std/Ext Message
		const result_t hasMessage();
		const result_t readMessage(uint32_t* canid, uint8_t* is_ext, uint8_t* len = NULL, uint8_t* data = NULL);

	private:
		// SPI Instruction Set
		typedef enum
		{ 
			CMD_WRITE = 			0x02,
			CMD_READ  = 			0x03,
			CMD_BITMOD =     		0x05,
			CMD_LOAD_TX0 =   		0x40,
			CMD_LOAD_TX1 =   		0x42,
			CMD_LOAD_TX2 =   		0x44,
			CMD_READ_RX0_SIDH =		0x90,
			CMD_READ_RX0_D0 =   	0x92,
			CMD_READ_RX1_SIDH =		0x94,
			CMD_READ_RX1_D0 =   	0x96,
			CMD_READ_STATUS =	 	0xA0,
			CMD_RX_STATUS = 		0xB0,
			CMD_RESET =     		0xC0,
		} command_t;

		// Buffers
		typedef enum : uint8_t
		{
			RXBUFNONE = 			0,
			RXBUF0 = 				0x60,
			RXBUF1 = 				0x70
		} rxbuffer_t;

		typedef enum : uint8_t
		{
			TXBUFNONE = 			0,
			TXBUF0 =				0x30,
			TXBUF1 =				0x40,
			TXBUF2 =				0x50	
		} txbuffer_t;

		result_t       	init();
		void            reset(void);
		result_t       	set_mode(const mode_t newmode);
		result_t       	set_rate(const bitrate_t rate);
		void            write_std_id(const uint8_t addr, const uint16_t id, const uint8_t data0, const uint8_t data1);
		void            write_ext_id(const uint8_t addr, const uint32_t id);
		void            inner_write_ext_id(const uint32_t id);
		void            check_message();

		txbuffer_t      get_free_txbuffer();
		void            filters_enabled(const bool enabled);

		// obs
		void            _print_id(const uint8_t addr);

		// Low Level
		uint8_t         read_register(const uint8_t address);
		void            set_register(const uint8_t address, const uint8_t value);
		void            modify_register(const uint8_t address, const uint8_t mask, const uint8_t data);
		uint8_t         read_status();
		uint8_t         read_rxstatus();

		uint8_t         _cs_pin;
#ifdef ENABLE_RTR
		uint8_t         _rtr = false;
#endif
		mode_t       _mode = MODE_CONFIG;
		// RxBuffer _rx_waiting = RXBUFNONE;
};
