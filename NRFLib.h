/**
* Copyright 2016, Simon Harst
* This file is part of the NRFLib project.
*
* NRFLib is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* NRFLib is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with NRFLib.  If not, see <http://www.gnu.org/licenses/>.
*/

#define ADR_WIDTH		5

// Outcomes of sending a packet
#define TX_SUCCESS		1
#define TX_MAX_RETRIES	2
#define TX_SENDING		3

class NRFLib {
    public:
        NRFLib(uint8_t cepin, uint8_t csnpin);

        void init(void);
        bool wait_for_message(unsigned char buf[], int wait);
        bool receive_message(unsigned char buf[]);
        void set_RXADDR(unsigned char RXADDR[]);
        void set_TXADDR(unsigned char TXADDR[]);
        void flushRX(void);
        void flushTX(void);
        bool TX_Full(void);
        void set_payload(unsigned char payload);
        void send_message(unsigned char buf[]);
        unsigned char get_status(void);
        unsigned char get_link_quality(void);
        bool data_ready(void);
        bool is_sending(void);
        unsigned char wait_for_send(void);
        unsigned char send_success(void);
        unsigned char SPI_RW_Reg(unsigned char reg, unsigned char value);
        void power_down(void);
        void RXMode(void);
        void TXMode(void);
        unsigned char SPI_Read(unsigned char reg);
        unsigned char SPI_Read_Buf(unsigned char reg, unsigned char *pBuf, unsigned char bytes);
        unsigned char SPI_Write_Buf(unsigned char reg, unsigned char *pBuf, unsigned char bytes);
        unsigned char SPI_RW(unsigned char Byte);


        uint8_t _cepin;
        uint8_t _csnpin;
        uint8_t _payload;
        uint8_t _txmode;

        unsigned char _rx_addr[ADR_WIDTH];
        unsigned char _tx_addr[ADR_WIDTH];
    private:


};
