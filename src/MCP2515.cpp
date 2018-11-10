
#include "MCP2515.h"
#include "MCP2515_defs.h"

#include <Arduino.h>
#include <SPI.h>

#define SELECT()	    	digitalWrite(_cs_pin, LOW)
#define UNSELECT()	    	digitalWrite(_cs_pin, HIGH)
#define DEBUGMSG(x)			Serial.println(x)
#define TIMEOUTVALUE    	50

MCP2515::result_t MCP2515::begin(const bitrate_t rate, const mode_t mode, const uint8_t cs_pin)
{    
    _cs_pin = cs_pin;

    reset();

    if(init() != OK)
        return FAIL;

    if(set_rate(rate) != OK)
        return FAIL;

    if(setMode(mode) != OK)
        return FAIL;

    return OK;
};

void MCP2515::setMessageType(const messagetype_t type)
{
    set_mode(MODE_CONFIG);

    modify_register(REG_RXB0CTRL, RXB0CTRL_RXM_MASK, type);
    modify_register(REG_RXB1CTRL, RXB1CTRL_RXM_MASK, type);

    set_mode(_mode);
};

const uint8_t MCP2515::getReceiveErrorCounter()
{
    return read_register(REG_REC);
};

const uint8_t MCP2515::getTransmitErrorCounter()
{
    return read_register(REG_TEC);
};

const void MCP2515::printStatus()
{
    Serial.println("-- MCP2525 Status:");

    // Mode
    uint8_t canstat = read_register(REG_CANSTAT);
    Serial.print("Mode: ");
    switch(canstat & CANSTAT_OPMOD_MASK)
    {
        case MODE_CONFIG:       Serial.print("CONFIG"); break;
        case MODE_LISTENONLY:   Serial.print("LISTEN_ONLY"); break;
        case MODE_NORMAL:       Serial.print("NORMAL"); break;
        case MODE_SLEEP:        Serial.print("SLEEP"); break;
        case MODE_LOOPBACK:     Serial.print("LOOPBACK"); break;
        default:                    Serial.print("Error"); break;
    }
    Serial.println();

    // Receiving messages
    uint8_t ctrl0 = read_register(REG_RXB0CTRL);
    uint8_t ctrl1 = read_register(REG_RXB1CTRL);

    uint8_t status = read_rxstatus();
    Serial.print("SPI_RXSTATUS=");
    Serial.print(status, HEX);
    Serial.println();

    uint8_t canintf = read_register(REG_CANINTF);
    Serial.print("CANINTF=");
    Serial.print(canintf, HEX);
    Serial.println();

    // RxBuffer's Filled?
    Serial.print("RXBUF0: ");
    if(status & STATUS_RXBUF0)
    {
        if(status & STATUS_TYPE_EXT)
            Serial.print("FULL_EXT ");
        else
            Serial.print("FULL_STD ");
        if(status & STATUS_TYPE_RTR)
            Serial.print("RTR ");
        else
            Serial.print("DATA ");
        Serial.print("FILTER");
        Serial.print(status & STATUS_RXF_MASK, DEC);
        Serial.print(" ");
    } else
        Serial.print("FREE ");
    switch(ctrl0 & RXB0CTRL_RXM_MASK)
    {
        case RECEIVE_NOFILTER:  Serial.print("ACC_ANY "); break;
        case RECEIVE_STD:       Serial.print("ACC_STD "); break;
        case RECEIVE_EXT:       Serial.print("ACC_EXT "); break;
        case RECEIVE_STDEXT:    Serial.print("ACC_STDEXT "); break;
    }
    Serial.println();

    Serial.print("RXBUF1: ");
    if(status & STATUS_RXBUF1)
    {   
        if(status & STATUS_RXBUF0)
            Serial.print("FULL ");
        else
        {
            if(status & STATUS_TYPE_EXT)
                Serial.print("FULL_EXT ");
            else
                Serial.print("FULL_STD ");
            if(status & STATUS_TYPE_RTR)
                Serial.print("RTR ");
            else
                Serial.print("DATA ");
            Serial.print("FILTER");
            Serial.print(status & STATUS_RXF_MASK, DEC);
            Serial.print(" ");
        }
    } else
        Serial.print("FREE ");
    switch(ctrl1 & RXB1CTRL_RXM_MASK)
    {
        case RECEIVE_NOFILTER:  Serial.print("ACC_ANY "); break;
        case RECEIVE_STD:       Serial.print("ACC_STD "); break;
        case RECEIVE_EXT:       Serial.print("ACC_EXT "); break;
        case RECEIVE_STDEXT:    Serial.print("ACC_STDEXT "); break;
    }
    Serial.println();

    // // Which buffer is selected?
    // Serial.print("Selected buffer: ");
    // switch(_rx_waiting)
    // {
    //     case RXBUF0: Serial.print("RXBUF0"); break;
    //     case RXBUF1: Serial.print("RXBUF1"); break;
    //     case RXBUFNONE: Serial.print("NONE"); break;
    // };
    // Serial.println();

    // Error counters
    Serial.print("Errors R/T: ");
    Serial.print(getReceiveErrorCounter(), DEC);
    Serial.print(" ");
    Serial.println(getTransmitErrorCounter(), DEC);
    Serial.println();

    Serial.println("--");
    Serial.println();
};

MCP2515::result_t MCP2515::setStdMask(const rxmask_t mask_addr, const uint16_t mask, const uint8_t data0, const uint8_t data1)
{
    // TODO masks are enabled in begin(), but can be done here
    if(set_mode(MODE_CONFIG) != OK)
    {
        DEBUGMSG("Enter Config mode: failed!");
        return FAIL;
    }

    write_std_id(mask_addr, mask, data0, data1);

    if(set_mode(_mode) != OK)
    {
        DEBUGMSG("Enter mode: failed!");
        return FAIL;
    }
    
    return OK;
};

MCP2515::result_t MCP2515::setExtMask(const rxmask_t mask_addr, const uint32_t mask)
{
    // TODO masks are enabled in begin(), but can be done here
    if(set_mode(MODE_CONFIG) != OK)
    {
        DEBUGMSG("Enter Config mode: failed!");
        return FAIL;
    }

    write_ext_id(mask_addr, mask);

    if(set_mode(_mode) != OK)
    {
        DEBUGMSG("Enter mode: failed!");
        return FAIL;
    }
    
    return OK;
};

MCP2515::result_t MCP2515::setStdFilter(const rxfilter_t filter_addr, const uint16_t filter, const uint8_t data0, const uint8_t data1)
{
    if(set_mode(MODE_CONFIG) != OK)
    {
        DEBUGMSG("Enter setting mode: failed!");
        return FAIL;
    }

    write_std_id(filter_addr, filter, data0, data1);

    if(set_mode(_mode) != OK)
    {
        DEBUGMSG("Enter normal mode: failed!");
        return FAIL;
    }

    return OK;
};

MCP2515::result_t MCP2515::setExtFilter(const rxfilter_t filter_addr, const uint32_t filter)
{
    if(set_mode(MODE_CONFIG) != OK)
    {
        DEBUGMSG("Enter setting mode: failed!");
        return FAIL;
    }

    write_ext_id(filter_addr, filter);

    if(set_mode(_mode) != OK)
    {
        DEBUGMSG("Enter normal mode: failed!");
        return FAIL;
    }

    return OK;
};

MCP2515::result_t MCP2515::setRate(const bitrate_t rate)
{
    if(set_mode(MODE_CONFIG) != OK)
    {
        DEBUGMSG("Enter setting mode: failed!");
        return FAIL;
    }

    if(set_rate(rate) != OK)
    {
        DEBUGMSG("Set Rate Failed.");
        return FAIL;
    }

    if(set_mode(_mode) != OK)
    {
        DEBUGMSG("Enter normal mode: failed!");
        return FAIL;
    }
    return OK;
};

inline MCP2515::result_t MCP2515::setMode(const mode_t newmode)
{
    return set_mode(_mode = newmode);
};

MCP2515::result_t MCP2515::sendStdMessage(const uint16_t id, const uint8_t len, const uint8_t *buf)
{
    txbuffer_t tx = get_free_txbuffer();
    if(tx == TXBUFNONE)
        return NO_BUFFER;

    // Construct RTR+DLC
    uint8_t dlc = len & DLC_MASK;

    // Set RTR
#ifdef ENABLE_RTR
    if (_rtr)
        dlc |= RTR_MASK;
#endif

    // Write it all in one run
    SELECT();
    switch(tx)
    {
        case TXBUF0: SPI.transfer(CMD_LOAD_TX0); break;
        case TXBUF1: SPI.transfer(CMD_LOAD_TX1); break;
        case TXBUF2: SPI.transfer(CMD_LOAD_TX2); break;
		case TXBUFNONE: return NO_BUFFER;
    }
    SPI.transfer((uint8_t) (id >> 3));          // SIDH
    SPI.transfer((uint8_t) ((id & 0x07) << 5)); // SIDL
    SPI.transfer(0);        // EID8
    SPI.transfer(0);        // EID0
    SPI.transfer(dlc);      // DLC
    for(uint8_t i=0; i<len; i++)     // len*DATA
        SPI.transfer(buf[i]);
    UNSELECT();

    // Start Transmissionstart_transmit(txbuf_n);
    modify_register(tx + TXBnCTRL, TXBnCTRL_TXREQ, TXBnCTRL_TXREQ );

    return OK;
};

MCP2515::result_t MCP2515::sendExtMessage(const uint32_t id, const uint8_t len, const uint8_t *buf)
{
    txbuffer_t tx = get_free_txbuffer();
    if(tx == TXBUFNONE)
        return NO_BUFFER;
 
    // Construct RTR+DLC
    uint8_t dlc = len & DLC_MASK;

#ifdef ENABLE_RTR
    if (_rtr)
        dlc |= RTR_MASK;
#endif

    // Write it all in one run
    SELECT();
    switch(tx)
    {
        case TXBUF0: SPI.transfer(CMD_LOAD_TX0); break;
        case TXBUF1: SPI.transfer(CMD_LOAD_TX1); break;
        case TXBUF2: SPI.transfer(CMD_LOAD_TX2); break;
		case TXBUFNONE: return NO_BUFFER;
    }

    // SIDH, SIDL, EID8, EID0
    inner_write_ext_id(id);

    // DLC
    SPI.transfer(dlc);
    // DATA * len
    for(uint8_t i=0; i<len; i++)
        SPI.transfer(buf[i]);

    UNSELECT();

    // Start Transmissionstart_transmit(txbuf_n);
    modify_register(tx + TXBnCTRL, TXBnCTRL_TXREQ, TXBnCTRL_TXREQ );

    return OK;
};

const MCP2515::result_t MCP2515::hasMessage()
{
    uint8_t status = read_rxstatus();
    if(status & (STATUS_RXBUF0 | STATUS_RXBUF1))
        return HAS_MESSAGE;
    return NO_MESSAGE;
};

const MCP2515::result_t MCP2515::readMessage(uint32_t* canid, uint8_t* is_ext, uint8_t* len, uint8_t* data)
{
    // Check if, and which buffer has a message waiting
    command_t cmd;
    uint8_t status = read_rxstatus();
    if(status & STATUS_RXBUF0)
        cmd = CMD_READ_RX0_SIDH;
    else if(status & STATUS_RXBUF1)
        cmd = CMD_READ_RX1_SIDH;
    else
        return NO_MESSAGE;

    // Start SPI and send Read-RXBufX Command
    SELECT();
    SPI.transfer(cmd);

    // First the ID: SIDH, SIDL, EID8, EID0 are comming in
    *canid = SPI.transfer(0x00);                                // r:SIDH
    *canid <<= 3;
    uint8_t sidl = SPI.transfer(0x00);                          // r:SIDL
    *canid |= (sidl >> 5);

    // extended id?
    *is_ext = (sidl & EXIDE_MASK);
    uint8_t eid8 = SPI.transfer(0x00);                          // r:EID8
    uint8_t eid0 = SPI.transfer(0x00);                          // r:EID0
    if(*is_ext)
    {
        *canid = (*canid<<2);
        *canid |= sidl & 0x03;
        *canid <<= 16;
        *canid |= (uint16_t) eid8 << 8;           
        *canid |= (uint8_t) eid0;
    }

    if(len == NULL)
    {
        UNSELECT();
        return OK;
    };

    // DLC
    uint8_t dlc = SPI.transfer(0x00);                           // r:DLC
    *len = dlc & DLC_MASK;

    // Data bytes
    for(uint8_t i=0; i<*len; i++) 
        data[i] = SPI.transfer(0x00);                           // r:D0..D7

    // SPI done
    UNSELECT();

    // Command READ_RXBUFx clears RXxIF after CS

    return OK;
};

//**************************************************************************************//
void MCP2515::reset(void)                                      
{
    SELECT();
    SPI.transfer(CMD_RESET);
    UNSELECT();
    delay(10);
};

MCP2515::result_t MCP2515::init()
{
    // Stop external clock output to reduce EMI
    modify_register(REG_CANCTRL, CANCTRL_CLKEN | CANCTRL_CLKPRE_MASK, 0);

    //TODO: implement one-shot mode?

    // Clear Receive CTRL regs, receive std+ext, disable masks and filters
    set_register(REG_RXB0CTRL, RECEIVE_NOFILTER | RXB0CTRL_BUKT);
    set_register(REG_RXB1CTRL, RECEIVE_NOFILTER);

    // interrupt mode
    set_register(REG_CANINTE, CANINTE_RX0IE | CANINTE_RX1IE);

    return OK;
};

MCP2515::result_t MCP2515::set_rate(const bitrate_t rate)            
{
    // return OK; 17144 - 16884 = 260 

    uint8_t cfg1, cfg2, cfg3;
    switch (rate) {
        case RATE_5KBPS:
            cfg1 = SPD_16MHz_5kBPS_CFG1;
            cfg2 = SPD_16MHz_5kBPS_CFG2;
            cfg3 = SPD_16MHz_5kBPS_CFG3;
            break;

        case RATE_10KBPS:
            cfg1 = SPD_16MHz_10kBPS_CFG1;
            cfg2 = SPD_16MHz_10kBPS_CFG2;
            cfg3 = SPD_16MHz_10kBPS_CFG3;
            break;

        case RATE_20KBPS:
            cfg1 = SPD_16MHz_20kBPS_CFG1;
            cfg2 = SPD_16MHz_20kBPS_CFG2;
            cfg3 = SPD_16MHz_20kBPS_CFG3;
            break;

        case RATE_40KBPS:
            cfg1 = SPD_16MHz_40kBPS_CFG1;
            cfg2 = SPD_16MHz_40kBPS_CFG2;
            cfg3 = SPD_16MHz_40kBPS_CFG3;
            break;

        case RATE_50KBPS:
            cfg1 = SPD_16MHz_50kBPS_CFG1;
            cfg2 = SPD_16MHz_50kBPS_CFG2;
            cfg3 = SPD_16MHz_50kBPS_CFG3;
            break;

        case RATE_80KBPS:
            cfg1 = SPD_16MHz_80kBPS_CFG1;
            cfg2 = SPD_16MHz_80kBPS_CFG2;
            cfg3 = SPD_16MHz_80kBPS_CFG3;
            break;

        case RATE_100KBPS:
            cfg1 = SPD_16MHz_100kBPS_CFG1;
            cfg2 = SPD_16MHz_100kBPS_CFG2;
            cfg3 = SPD_16MHz_100kBPS_CFG3;
            break;

        case RATE_125KBPS:
            cfg1 = SPD_16MHz_125kBPS_CFG1;
            cfg2 = SPD_16MHz_125kBPS_CFG2;
            cfg3 = SPD_16MHz_125kBPS_CFG3;
            break;

        case RATE_200KBPS:
            cfg1 = SPD_16MHz_200kBPS_CFG1;
            cfg2 = SPD_16MHz_200kBPS_CFG2;
            cfg3 = SPD_16MHz_200kBPS_CFG3;
            break;

        case RATE_250KBPS:
            cfg1 = SPD_16MHz_250kBPS_CFG1;
            cfg2 = SPD_16MHz_250kBPS_CFG2;
            cfg3 = SPD_16MHz_250kBPS_CFG3;
            break;

        case RATE_500KBPS:
            cfg1 = SPD_16MHz_500kBPS_CFG1;
            cfg2 = SPD_16MHz_500kBPS_CFG2;
            cfg3 = SPD_16MHz_500kBPS_CFG3;
            break;

        default:
            return FAIL;
            break;
    }

    set_register(REG_CNF1, cfg1);
    set_register(REG_CNF2, cfg2);
    set_register(REG_CNF3, cfg3);
    return OK;
};


MCP2515::result_t MCP2515::set_mode(const mode_t newmode)
{
    modify_register(REG_CANCTRL, CANCTRL_REQOP_MASK, newmode);

// TODO:
// When changing modes, the mode will not actually
// change until all pending message transmissions are
// complete. The requested mode must be verified by
// reading the CANSTAT.OPMODE bits (see Register 10-2).
    uint16_t timeout = TIMEOUTVALUE;
    for(;;)
    {
        uint8_t mode = read_register(REG_CANSTAT) & CANSTAT_OPMOD_MASK;
        if(mode == newmode)
            return OK;
        if(!--timeout)
            return TIMEOUT;
    };
}

void MCP2515::_print_id(const uint8_t addr)
{
    for(uint8_t i=0; i<4; i++)
    {
        Serial.print(addr + i, HEX);
        Serial.print("=");
        Serial.print(read_register(addr + i), HEX);
        Serial.print(" ");
    };
};

void MCP2515::write_std_id(const uint8_t addr, const uint16_t id, const uint8_t data0, const uint8_t data1)
{
    SELECT();
    SPI.transfer(CMD_WRITE);
    SPI.transfer(addr);

    // SIDH
    SPI.transfer((uint8_t) (id >> 3));
    // SIDL
    SPI.transfer((uint8_t) ((id & 0x07) << 5));

    // data0 into EID8
    SPI.transfer(data0);

    // data1 into EID0
    SPI.transfer(data1);

    UNSELECT();
}

void MCP2515::write_ext_id(const uint8_t addr, const uint32_t id )
{
    
    SELECT();
    SPI.transfer(CMD_WRITE);
    SPI.transfer(addr);

    inner_write_ext_id(id);

    UNSELECT();
}

void MCP2515::inner_write_ext_id(const uint32_t id)
{
    uint16_t tmp  = (uint16_t)( id / 0x10000L );
    // SIDH
    SPI.transfer((uint8_t) (tmp >> 5)); // / 32
    // SIDL
    uint8_t sidl = (uint8_t) (tmp & 0x03);
    sidl += (uint8_t) ((tmp & 0x1C ) * 8);
    sidl |= EXIDE_MASK;
    SPI.transfer(sidl);

    tmp = (uint16_t)(id & 0x0FFFF);
    // EID8
    SPI.transfer((uint8_t) (tmp >> 8));
    // EID0
    SPI.transfer((uint8_t) (tmp & 0x0FF));
}

MCP2515::txbuffer_t MCP2515::get_free_txbuffer()
{
    // uint8_t ctrlval;
    // const TxBuffer buffers[3] = { TXBUF0, TXBUF1, TXBUF2 };

    uint16_t timeleft = TIMEOUTVALUE;
    for(;;)
    {
        if( (read_register( TXBUF0 + TXBnCTRL ) & TXBnCTRL_TXREQ) == 0 )
            return TXBUF0;
        if( (read_register( TXBUF1 + TXBnCTRL ) & TXBnCTRL_TXREQ) == 0 )
            return TXBUF1;
        if( (read_register( TXBUF2 + TXBnCTRL ) & TXBnCTRL_TXREQ) == 0 )
            return TXBUF2;
        if(!timeleft--) 
            return TXBUFNONE;
    };
}

// LOW LEVEL **********************************************************************
uint8_t MCP2515::read_register(const uint8_t address)                                                                     
{
    uint8_t ret;

    SELECT();
    SPI.transfer(CMD_READ);
    SPI.transfer(address);
    ret = SPI.transfer(0x00);
    UNSELECT();

    return ret;
}

void MCP2515::set_register(const uint8_t address,                  
                                  const uint8_t value)
{
    SELECT();
    SPI.transfer(CMD_WRITE);
    SPI.transfer(address);
    SPI.transfer(value);
    UNSELECT();
}

void MCP2515::modify_register(const uint8_t address,
                            const uint8_t mask,
                            const uint8_t data)
{
    SELECT();
    SPI.transfer(CMD_BITMOD);
    SPI.transfer(address);
    SPI.transfer(mask);
    SPI.transfer(data);
    UNSELECT();
}

uint8_t MCP2515::read_status()                             
{
	SELECT();
	SPI.transfer(CMD_READ_STATUS);
	uint8_t status = SPI.transfer(0x00);
	UNSELECT();
	
	return status;
}

uint8_t MCP2515::read_rxstatus()                             
{
    SELECT();
    SPI.transfer(CMD_RX_STATUS);
    uint8_t status = SPI.transfer(0x00);
    UNSELECT();
    
    return status;
}
