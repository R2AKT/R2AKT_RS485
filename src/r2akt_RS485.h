 /******************************************************************************
 *
 *    FILE NAME : r2akt_RS485.h
 *       AUTHOR : Sergey Dorozhkin (R2AKT)
 *         DATE : 27-november-2024
 *      VERSION : 0.0.3
 * MODIFICATION : 4
 *      PURPOSE : Arduino library for RS485 packet data exchange
 *          URL : https://github.com/R2AKT/r2akt_RS485
 *
 ******************************************************************************/
#include <Arduino.h>

/*
Levels:
	PHY - KISS(SLIP)/COBS+MAC DATA+KISS(SLIP)/COBS - Done
	MAC - DstAdr+ScrAddr+APP DATA+CRC16 - Done
	APP - {User DATA}+CRC32 - Pending
	USER - User data (DstAddr/SrcAddr, [SrcAddr/-], Data) - Query
*/

 /******************************************************************************/
#ifndef r2akt_RS485_h
	#define r2akt_RS485_h
	#include <r2akt_crc.h>
	#include <r2akt_cobs.h>
	#include <r2akt_esc_deesc.h>
///
	#ifdef __cplusplus
		extern "C" {
	#endif

 /******************************************************************************/
#define error_num_no_error 0
#define error_num_oversize 1
#define error_num_crc_error 2
#define error_num_decode_error 3
#define error_num_encode_error 4
#define error_num_timeout 5
#define error_num_write_error 6
#define error_num_read_error 7
#define error_num_no_data 8
#define error_num_error_data 9
#define error_num_read_error_souce_broadcast 10
#define error_num_read_error_souce_self 11

#define error_num_unkw_error 99

 /******************************************************************************/
//#define DEBUG_ENABLE
#ifdef DEBUG_ENABLE
	#define DEBUG_PRINT(x) Serial.print(x)
	#define DEBUG_PRINTLN(x) Serial.println(x)
	#define DEBUG_PRINT_HEX(x) Serial.print(x, HEX)
	#define DEBUG_PRINTLN_HEX(x) Serial.println(x, HEX)
#else
	#define DEBUG_PRINT(x)
	#define DEBUG_PRINTLN(x)
	#define DEBUG_PRINT_HEX(x)
	#define DEBUG_PRINTLN_HEX(x)
#endif

 /******************************************************************************/
	#define packetTransmit HIGH
	#define packetReceive LOW
	#define TxDelay 10
	#define TxTail 10
 /******************************************************************************/
	class RS485 : public Stream {
		public:
			RS485 (Stream *Port, uint8_t SrcAddr, uint16_t BuffSize = 64, bool COBS_KISS = false, uint8_t ToglePin = 13, bool Blocking = false, uint16_t TimeOut = 1000);
			void begin (const uint8_t SrcAddr = 0x0);
			//
			int16_t send_phy (const uint8_t *Buff, const size_t size);
			int16_t receive_phy (uint8_t *Buff, bool Blocking = false, uint16_t TimeOut = 0);
			//
			int16_t send_mac (const uint8_t DstAddr, const uint8_t *Buff, const size_t size);
			int16_t receive_mac (uint8_t *Buff, uint8_t *SrcAddr, bool Blocking = false, uint16_t TimeOut = 0);
			//
			int16_t packet_send_to (const uint8_t DstAddr, const uint8_t *Buff, const uint16_t size);
			int16_t packet_receive_from (uint8_t *Buff, const uint8_t SrcAddr, bool Blocking = false, uint16_t TimeOut = 0);
			int16_t packet_receive (uint8_t *Buff, uint8_t *SrcAddr, bool Blocking = false, uint16_t TimeOut = 0);
			//
			int8_t PHY_Error_Num;
			int8_t MAC_Error_Num;
			int8_t PACKET_Error_Num;
//
		private:
			Stream *_Port;
			uint8_t _SrcAddr;
			bool _COBS;
			size_t _BuffSize;
			size_t _PacketBuffSize;
			uint8_t _ToglePin;
			bool _Blocking;
			uint16_t _TimeOut;
			//
			void setRXmode(void);
			void setTXmode(void);
			//
			uint8_t *PHY_Exchange_Rx;
			///
			#pragma pack (push, 1)
				struct phy_packet_status_struct {
					bool packet_Rx_Sync;
					size_t packet_Rx_Len;
				};
			#pragma pack (pop)
			//
			phy_packet_status_struct PHY_Packet_Status = {};

			// Stream interface
			int available();
			int read();
			int peek();
			void flush();
			int availableForWrite();
			//
			size_t write (const uint8_t c);
			size_t write (const char * Buff, size_t size);
			size_t write (const uint8_t * Buff, size_t size);
	};
//
	#ifdef __cplusplus
		};           /* closing brace for extern "C" */
	#endif
//
#endif
 /************************************************************** END OF FILE ***/