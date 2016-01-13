# NRFLib
An arduino library to interface with Mini - NRF24L01 modules 
Copyright 2016 Simon Harst

I have found that the libraries that are currently available on the internet don't work with the mini version of the NRF24L01 PCB with 1.27" SMD connectors - so I wrote my own library. It has only been tested on Attiny84 microcontrollers with the arduino-tiny core.

This software is licenced under the GNU Lesser General Public License (LGPL). 

Minimal working example:
```

#define CE        8               // CE-Pin of the Nrf24L01
#define CSN       7               // CSN-Pin of the Nrf24L01
#define payload   16              // Payload length. Dynamic payload not supported.

unsigned char tx_buf[payload] = {0};
unsigned char rx_buf[payload] = {0};

unsigned char RX_ADDRESS[5]  = { 0xaa,0xf0,0x21,0xe3,0x15 };
unsigned char TX_ADDRESS[5]  = { 0xaa,0xf0,0x21,0xe3,0x01 };

nrf.set_payload(payload);         // Set payload of message - dynamic payloads not supported yet.
nrf.set_TXADDR(TX_ADDRESS);       // Set address of device that messages should be sent to
nrf.set_RXADDR(RX_ADDRESS);       // Set address we are listening to
nrf.init();                       // Initialize module. This needs to happen after set_XXADDR and set_payload
nrf.TXMode();                     // Boot up in TXMode. This or RXMode() needs to be called after init.

nrf.send_message(tx_buf);         // Send the message
nrf.wait_for_send();              // Block until the ACK package was received or timeout

                                  // Returns a number from 0-100 representing the amount 
                                  // how often we needed to resend the message until it got through
signal_quality = nrf.get_link_quality();

                                  // Power up RX Mode and wait for 1s for a message to come in
bool received = nrf.wait_for_message(rx_buf, 1000);
if (received) {
  // rx_buf now holds the received message
  // do something with it.
} else {
  // Bummer. No message received.
}
```
