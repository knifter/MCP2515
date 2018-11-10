#ifndef _MCP2515_DEFS_H
#define _MCP2515_DEFS_H

// CAN Msgs
#define DLC_MASK        	0x0F  /* 4 LSBits                     */
#define RTR_MASK        	0x40  /* (1<<6) Bit 6                 */
#define EXIDE_MASK  		0x08  /* In TXBnSIDL                  */


// Status instruction result byte
#define STATUS_RXF_MASK 	0x07
#define STATUS_TYPE_MASK	0x18
	#define	STATUS_TYPE_EXT	0x10
	#define STATUS_TYPE_RTR 0x08
#define STATUS_RXBUF0		0x40
#define STATUS_RXBUF1		0x80

// REGISTERS
#define REG_BFPCTRL					0x0C
	#define BFPCTRL_B1BFS 			0x20
	#define BFPCTRL_B0BFS 			0x10
	#define BFPCTRL_B1BFE 			0x08
	#define BFPCTRL_B0BFE 			0x04
	#define BFPCTRL_B1BFM 			0x02
	#define	BFPCTRL_B0BFM			0x01
#define REG_TXRTSCTRL				0x0D
	#define TXRTSCTRL_B2RTS			0x20
	#define TXRTSCTRL_B1RTS			0x10
	#define TXRTSCTRL_B0RTS			0x08
	#define TXRTSCTRL_B2RTSM		0x04
	#define TXRTSCTRL_B1RTSM		0x02
	#define TXRTSCTRL_B0RTSM		0x01
#define REG_CANSTAT     			0x0E // Page 59
	#define CANSTAT_OPMOD_MASK		0xE0
	#define CANSTAT_ICOD_MASK		0x0E // Interrupt Cause/Code
#define REG_CANCTRL     			0x0F // Page 58
	#define CANCTRL_REQOP_MASK      0xE0
	// #define MODE_NORMAL   	0x00  
	// #define MODE_SLEEP     	0x20  
	// #define MODE_LOOPBACK  	0x40  
	// #define MODE_LISTENONLY  0x60  
	// #define MODE_CONFIG    	0x80  
	#define CANCTRL_ABAT			0x10
	#define CANCTRL_OSM				0x08
	#define CANCTRL_CLKEN 			0x04
	#define CANCTRL_CLKPRE_MASK 	0x03
#define REG_TEC         			0x1C
#define REG_REC         			0x1D
#define REG_CNF1        			0x2A // Page 42
	#define CNF1_SJW1            		0x00 // Sync Jump Width Length bits
	#define CNF1_SJW2            		0x40
	#define CNF1_SJW3            0x80
	#define CNF1_SJW4            0xC0
	#define CNF1_BRP1			0x01
	#define CNF1_BRP2			0x02
	#define CNF1_BRP4			0x04
	#define CNF1_BRP8			0x08
	#define CNF1_BRP16			0x10
	#define CNF1_BRP32			0x20
#define REG_CNF2        		0x29 // Page 42
	#define CNF2_PRSEG_MASK		0x07 // 0b00000111
	#define CNF2_PHSEG1_MASK		0x38 // 0b00111000
	#define CNF2_SAMPLE_1X       0x00
	#define CNF2_SAMPLE_3X       0x40
	#define CNF2_BTLMODE         0x80
#define REG_CNF3        		0x28 // Page 43
	#define CNF3_PHSEG2_MASK		0x07
	#define CNF3_WAKFIL			0x40 // Wake-up Filter enable bit
	#define	CNF3_SOF				0x80 // Start-of-Frame signal bit
#define REG_CANINTE    			0x2B
	#define CANINTE_RX0IE   		0x01
	#define CANINTE_RX1IE   		0x02
	#define CANINTE_TX0IE       	0x04
	#define CANINTE_TX1IE			0x08
	#define CANINTE_TX2IE    		0x10
	#define CANINTE_ERRIE    		0x20
	#define CANINTE_WAKIE    		0x40
	#define CANINTE_MERRE    		0x80
#define REG_CANINTF     			0x2C
	#define CANINTF_RX0IF   		0x01
	#define CANINTF_RX1IF   		0x02
	#define CANINTF_TX0IF       	0x04
	#define CANINTF_TX1IF			0x08
	#define CANINTF_TX2IF    		0x10
	#define CANINTF_ERRIF    		0x20
	#define CANINTF_WAKIF    		0x40
	#define CANINTF_MERRF    		0x80
#define REG_EFLG					0x2D
	#define	EFLG_RX1OVR				0x80
	#define	EFLG_RX0OVR				0x40
	#define	EFLG_TXBO				0x20
	#define	EFLG_TXEP				0x10
	#define EFLG_RXEP				0x08
	#define EFLG_TXWAR				0x04
	#define EFLG_RXWAR				0x02
	#define	EFLG_EWARN				0x01
#define REG_TXB0CTRL    			0x30
#define REG_TXB1CTRL    			0x40
#define REG_TXB2CTRL    			0x50
	#define TXBnCTRL_ABTF			0x40
	#define TXBnCTRL_MLOA			0x20
	#define TXBnCTRL_TXERR			0x10
	#define TXBnCTRL_TXREQ			0x08
	#define TXBnCTRL_TXIE			0x04
	#define TXBnCTRL_TXP10_MASK		0x03
#define REG_RXB0CTRL    			0x60	// Page 27
// FIXME: Check this!!:
	#define RXB0CTRL_RXM_MASK     	0x60	// Receive buffer operating mode bits
	#define RXB0CTRL_BUKT   		0x04	// Rollover bit (RXB0->RXB1)
	#define RXB0CTRL_FILHIT0_MASK	0x01	// Filter Hit bit in RX0
#define REG_RXB1CTRL    			0x70	// Page 28
	#define RXB1CTRL_RXM_MASK     	0x60	// Receive buffer operating mode bits
	#define RXB1CTRL_RXRTR			0x08	// Receive Remote Transfer Request Bit
	#define RXB1CTRL_FILHIT1_MASK	0x07	// Filter Hit Bit in RX1
#define TXBnCTRL 					0x00 // 

#define CANID 						0x01 // 31
	#define CANID_SIDH				0x00 // offsets of SIDL/H,EID0,8 in the CANID
	#define CANID_SIDL 				0x01 
	#define CANID_EID8            	0x02 
	#define CANID_EID0            	0x03 
#define DLC							0x05 // 35, 45, 55
#define DATA						0x06 // 36-3D, 46-4D, 56-5D

// END REGISTERS


// Speed Configs
#define SPD_16MHz_500kBPS_CFG1 (0x00)
#define SPD_16MHz_500kBPS_CFG2 (0xb0)
#define SPD_16MHz_500kBPS_CFG3 (0x06)

#define SPD_16MHz_250kBPS_CFG1 (0x01)
#define SPD_16MHz_250kBPS_CFG2 (0xb0)
#define SPD_16MHz_250kBPS_CFG3 (0x06)

#define SPD_16MHz_200kBPS_CFG1 (0x00)
#define SPD_16MHz_200kBPS_CFG2 (0xba)
#define SPD_16MHz_200kBPS_CFG3 (0x07)

#define SPD_16MHz_125kBPS_CFG1 (0x01)
#define SPD_16MHz_125kBPS_CFG2 (0xba)
#define SPD_16MHz_125kBPS_CFG3 (0x07)

#define SPD_16MHz_100kBPS_CFG1 (0x03)
#define SPD_16MHz_100kBPS_CFG2 (0xba)
#define SPD_16MHz_100kBPS_CFG3 (0x07)

#define SPD_16MHz_80kBPS_CFG1 (0x03)
#define SPD_16MHz_80kBPS_CFG2 (0xbf)
#define SPD_16MHz_80kBPS_CFG3 (0x07)

#define SPD_16MHz_50kBPS_CFG1 (0x07)
#define SPD_16MHz_50kBPS_CFG2 (0xba)
#define SPD_16MHz_50kBPS_CFG3 (0x07)

#define SPD_16MHz_40kBPS_CFG1 (0x07)
#define SPD_16MHz_40kBPS_CFG2 (0xbf)
#define SPD_16MHz_40kBPS_CFG3 (0x07)

#define SPD_16MHz_20kBPS_CFG1 (0x0f)
#define SPD_16MHz_20kBPS_CFG2 (0xbf)
#define SPD_16MHz_20kBPS_CFG3 (0x07)

#define SPD_16MHz_10kBPS_CFG1 (0x1f)
#define SPD_16MHz_10kBPS_CFG2 (0xbf)
#define SPD_16MHz_10kBPS_CFG3 (0x07)

#define SPD_16MHz_5kBPS_CFG1 (0x3f)
#define SPD_16MHz_5kBPS_CFG2 (0xbf)
#define SPD_16MHz_5kBPS_CFG3 (0x07)


#endif // _MCP2515_DEFS_H