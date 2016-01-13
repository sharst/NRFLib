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

#include "Arduino.h"
#include "NRFLib.h"
#include <SPI85.h>
#include "API.h"



NRFLib::NRFLib(uint8_t cepin, uint8_t csnpin) {
	_cepin = cepin;
	_csnpin = csnpin;
	_payload = 32;
	_txmode = true;
}


void NRFLib::init(void) {
	pinMode(_cepin, OUTPUT);
	pinMode(_csnpin, OUTPUT);

	SPI85.begin();

	digitalWrite(_cepin, 0);		// chip enable
	digitalWrite(_csnpin, 1);		// Spi disable

	SPI_RW_Reg(WRITE_REG + EN_AA, 0x01);      // Enable Auto.Ack:Pipe0
	SPI_RW_Reg(WRITE_REG + EN_RXADDR, 0x01);  // Enable Pipe0
	SPI_RW_Reg(WRITE_REG + RF_CH, 120);        // Select RF channel 120
	SPI_RW_Reg(WRITE_REG + RX_PW_P0, _payload); // Select same RX payload width as TX Payload width
	SPI_RW_Reg(WRITE_REG + RF_SETUP, 0x27);   // TX_PWR:0dBm, Datarate:250kbps
	SPI_RW_Reg(WRITE_REG + SETUP_RETR, 0xff); // Up to 15retries with 4ms wait in between.
	SPI_Write_Buf(WRITE_REG + TX_ADDR, _tx_addr, ADR_WIDTH);    // Writes TX_Address to nRF24L01

}

void NRFLib::set_RXADDR(unsigned char RXADDR[]) {
	for (int i=0; i<ADR_WIDTH; i++)
		_rx_addr[i] = RXADDR[i];
}

void NRFLib::set_TXADDR(unsigne
		//RXMode();
		//flushRX();
		//flushTX();d char TXADDR[]) {
	for (int i=0; i<ADR_WIDTH; i++)
		_tx_addr[i] = TXADDR[i];
}

void NRFLib::set_payload(unsigned char payload) {
	_payload = payload;
}

unsigned char NRFLib::get_status(void) {
	unsigned char sstatus=SPI_Read(STATUS);
	return sstatus;
}

unsigned char NRFLib::get_link_quality(void) {
	unsigned char observe=SPI_Read(OBSERVE_TX)&0x0B;
	unsigned char retr=SPI_Read(SETUP_RETR)&0x0B;
	return (1-(observe/retr))*255;
}

bool NRFLib::TX_Full(void) {
	unsigned char status = SPI_Read(STATUS);
	return status & 1;
}

void NRFLib::flushRX(void) {
	digitalWrite(_csnpin, 0);
	SPI_RW(FLUSH_RX);
	digitalWrite(_csnpin, 1);
}

void NRFLib::flushTX(void) {
	digitalWrite(_csnpin, 0);
	SPI_RW(FLUSH_TX);
	digitalWrite(_csnpin, 1);
	//SPI_RW_Reg(WRITE_REG+FLUSH_TX, 0);
}


void NRFLib::send_message(unsigned char buf[]) {
  //flushTX();
  SPI_RW_Reg(WRITE_REG + STATUS, 0x30); // Clear MAX_RT and TX_DS bits.


  if (!_txmode)
	  TXMode();

  SPI_Write_Buf(WR_TX_PLOAD, buf, _payload);

  // Send it!
  digitalWrite(_cepin, 1);
  delay(1);
  digitalWrite(_cepin, 0);
}

bool NRFLib::is_sending(void) {
	unsigned char status = SPI_Read(STATUS);

	// if sending successful (TX_DS) or max retries exceeded (MAX_RT).
	if (status & TX_DS) {
		SPI_RW_Reg(WRITE_REG+STATUS, status);
		return false;
	} else if (status & MAX_RT) {
		SPI_RW_Reg(WRITE_REG+STATUS, status);
		return false;
	} else
		return true;
}

unsigned char NRFLib::wait_for_send(void) {
	unsigned char success = TX_SENDING;
	do {
		success = send_success();
	} while (success==TX_SENDING);
	return success;
}

unsigned char NRFLib::send_success(void) {
	unsigned char status = SPI_Read(STATUS);

	// if sending successful (TX_DS) or max retries exceeded (MAX_RT).
	if (status & TX_DS) {
		SPI_RW_Reg(WRITE_REG+STATUS, status);
		return TX_SUCCESS;
	} else if (status & MAX_RT) {
		SPI_RW_Reg(WRITE_REG+STATUS, status);
		flushTX();
		return TX_MAX_RETRIES;
	} else
		return TX_SENDING;
}

bool NRFLib::data_ready(void){
	if (_txmode)
		RXMode();
	unsigned char status = SPI_Read(STATUS);                         // read register STATUS's value
	return (status & RX_DR);
}

bool NRFLib::wait_for_message(unsigned char buf[], int wait) {
	if (_txmode)
		RXMode();

	unsigned char received;

	for (int i=0; i<wait; i++) {
		received = receive_message(buf);
		if (received) return true;
		delay(1);
	}
	return false;
}

bool NRFLib::receive_message(unsigned char buf[]) {
	if (_txmode)
		RXMode();
	unsigned char status = SPI_Read(STATUS);                         // read register STATUS's value
	unsigned char fifo = SPI_Read(0x17);
	//if(status&RX_DR){                                              // if receive data ready (TX_DS) interrupt
	if (!(fifo&1)) {
	  SPI_Read_Buf(RD_RX_PLOAD, buf, _payload);             // read playload to rx_buf
	  //flushRX();                                        // clear RX_FIFO
	  SPI_RW_Reg(WRITE_REG+STATUS,0x70); //0x70);                             // clear RX_DR or TX_DS or MAX_RT interrupt flag
	  return true;
	}
	return false;
}


void NRFLib::power_down(void) {
	digitalWrite(_cepin, 0);
	delayMicroseconds(75);
	SPI_RW_Reg(WRITE_REG + CONFIG, 0x00);   //pwr down
	digitalWrite(_cepin, 1);
}

void NRFLib::TXMode(void) {
	digitalWrite(_cepin, 0);
	SPI_Write_Buf(WRITE_REG + RX_ADDR_P0, _tx_addr, ADR_WIDTH); // RX_Addr0 same as TX_Adr for Auto.Ack
	SPI_RW_Reg(WRITE_REG + CONFIG, 0x0e);     // Set PWR_UP bit, enable CRC(2 unsigned chars) & Prim:TX. MAX_RT & TX_DS enabled..
	_txmode = true;
}

void NRFLib::RXMode(void) {
	digitalWrite(_cepin, 0);
	SPI_Write_Buf(WRITE_REG + RX_ADDR_P0, _rx_addr, ADR_WIDTH); // Use the same address on the RX device as the TX device
	SPI_RW_Reg(WRITE_REG + CONFIG, 0x0f);     // Set PWR_UP bit, enable CRC(2 unsigned chars) & Prim:RX.
	_txmode = false;
	digitalWrite(_cepin, 1);                  // Set CE pin high to enable RX device
}


/*
 * Transfer byte Byte via SPI
 */
unsigned char NRFLib::SPI_RW(unsigned char Byte) {
  return SPI85.transfer(Byte);
}

/*
 * Write value value to register reg
 */
unsigned char NRFLib::SPI_RW_Reg(unsigned char reg, unsigned char value) {
  unsigned char status;

  digitalWrite(_csnpin, 0);                   // CSN low, init SPI transaction
  SPI_RW(reg);                            // select register
  SPI_RW(value);                          // ..and write value to it..
  digitalWrite(_csnpin, 1);                   // CSN high again

  return(status);                   		// return nRF24L01 status unsigned char
}

/*
 * Read out value of register reg
 */
unsigned char NRFLib::SPI_Read(unsigned char reg) {
  unsigned char reg_val;

  digitalWrite(_csnpin, 0);                // CSN low, initialize SPI communication...
  SPI_RW(reg);                         // Select register to read from..
  reg_val = SPI_RW(0);                 // ..then read register value
  digitalWrite(_csnpin, 1);                // CSN high, terminate SPI communication

  return(reg_val);                     // return register value
}


unsigned char NRFLib::SPI_Read_Buf(unsigned char reg, unsigned char *pBuf, unsigned char bytes) {
  unsigned char sstatus,i;

  digitalWrite(_csnpin, 0);                   // Set CSN low, init SPI tranaction
  sstatus = SPI_RW(reg);       	    // Select register to write to and read status unsigned char

  for(i=0;i<bytes;i++)
  {
    pBuf[i] = SPI_RW(0);    // Perform SPI_RW to read unsigned char from nRF24L01
  }

  digitalWrite(_csnpin, 1);                   // Set CSN high again

  return(sstatus);                  // return nRF24L01 status unsigned char
}

unsigned char NRFLib::SPI_Write_Buf(unsigned char reg, unsigned char *pBuf, unsigned char bytes) {
  unsigned char sstatus,i;

  digitalWrite(_csnpin, 0);
  sstatus = SPI_RW(reg);             // Select register to write to and read status unsigned char
  for(i=0;i<bytes; i++){             // then write all unsigned char in buffer(*pBuf)
    SPI_RW(*pBuf++);
  }
  digitalWrite(_csnpin, 1);
  return(sstatus);                  // return nRF24L01 status unsigned char
}
