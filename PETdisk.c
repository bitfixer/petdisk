/*
    PETdisk.c
    Main program for the PETdisk storage device
    Copyright (C) 2011 Michael Hill

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    
    Contact the author at bitfixer@bitfixer.com
    http://bitfixer.com
*/

#define F_CPU 8000000UL		//freq 8 MHz
#define BAUD 9600
#define MYUBRR F_CPU/16/BAUD-1
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "bf-avr-sdlib/SPI_routines.h"
#include "bf-avr-sdlib/SD_routines.h"
#include "bf-avr-sdlib/UART_routines.h"
#include "bf-avr-sdlib/FAT32.h"
#include "IEEE488.h"

#define SPI_PORT PORTB
#define SPI_CTL  DDRB
#define MISO     0x10
#define CASSETTE_READ 0x40
#define CASSETTE_WRITE 0x80

#define FNAMELEN    39

void port_init(void)
{
    SPI_CTL = ~MISO & ~DATA0 & ~DATA1 & ~CASSETTE_READ & ~CASSETTE_WRITE;
    SPI_PORT = 0xff;

    // all IEEE lines input
    IEEE_CTL = 0x00;
    // activate pullups
    IEEE_PORT = 0xFF;

    // set data lines to input
    DATA_CTL = 0x00;
}


//call this routine to initialize all peripherals
void init_devices(void)
{
 cli();  //all interrupts disabled
 port_init();
 spi_init();
 uart0_init(MYUBRR);

 MCUCR = 0x00;
}

unsigned char get_device_address()
{
    DDRC = 0x00;
    DDRD = 0x00;
    
    // turn on pullup resistors
    PORTC = 0x20;
    PORTD = 0x01;
    
    _delay_ms(100);  //delay for VCC stabilization

    unsigned char addrlo,addrhi,addr;
    // read jumper settings
    addrlo = PIND;
    addrhi = PINC;
    
    // invert readings - jumper present should = 1 for that bit
    addrlo = ~addrlo;
    addrhi = ~addrhi;
    
    // bitmask and shift
    addrlo = addrlo & 0x01;
    addrhi = (addrhi & 0x20) >> 4;
    addr = addrhi | addrlo;
    
    addr = addr + 8;
    return addr;
    
}

/*
int main(void)
{
    unsigned char i;
    unsigned char progname[20];
    unsigned char error;
    init_devices();
    
    // initialize SD card
    for (i=0; i<10; i++)
    {
        error = SD_init();
        if(!error) break;
    }
    
    error = getBootSectorData (); //read boot sector and keep necessary data in global variables
    
    
    memset(progname, 0, 20);
    // look for firmware file
    progname[0] = 'T';
    progname[1] = 'H';
    progname[2] = 'I';
    progname[3] = 'S';
    progname[4] = ' ';
    progname[5] = 'I';
    progname[6] = 'S';
    progname[7] = ' ';
    progname[8] = 'C';
    progname[9] = 'O';
    progname[10] = 'O';
    progname[11] = 'L';
    progname[12] = '.';
    progname[13] = 'T';
    progname[14] = 'X';
    progname[15] = 'T';
    progname[16] = 0;
    
    openFileForWriting(progname, _rootCluster);
    
    memset(_buffer, 'A', 512);
    writeBufferToFile(512);
    
    closeFile();
    
}
*/

int main(void)
{
    unsigned char fileName[11];
    unsigned char progname[FNAMELEN];
    unsigned char rdchar,rdbus,ctl;
    unsigned char option, error, data, FAT32_active;
    unsigned char getting_filename;
    unsigned char filename_position;
    unsigned char address;
    unsigned int bytes_to_send;
    unsigned char i;
    unsigned long currentDirectoryCluster;
    
    address = get_device_address();
    
    init_devices();
    transmitString("Device ID: ");
    transmitHex(CHAR, address);
    TX_NEWLINE;
    
    _cardType = 0;

    struct dir_Structure *dir;
    unsigned long cluster, byteCounter = 0, fileSize, firstSector;
    unsigned int k;
    unsigned char j,sending;
    unsigned char response;
    unsigned char startline;
    unsigned int retry;
    unsigned int dir_start;
    unsigned int dirlength;
    unsigned char gotname;
    unsigned char savefile;
    unsigned char filenotfound;
    unsigned char initcard;
    unsigned char buscmd;
    //unsigned long startBlock;
    unsigned long cl;
    getting_filename = 0;
    filename_position = 0;
    // clear string
    for (i = 0; i < FNAMELEN; i++)
    {
        progname[i] = 0x00;
    }
    
    // initialize SD card
    for (i=0; i<10; i++)
    {
      error = SD_init();
      if(!error) break;
    }
    
    if (!error)
    {
        error = getBootSectorData(); //read boot sector and keep necessary data in global variables
        currentDirectoryCluster = _rootCluster;
    
		// look for firmware file
        progname[0] = 'F';
        progname[1] = 'I';
        progname[2] = 'R';
        progname[3] = 'M';
        progname[4] = '*';
        progname[5] = 0;
        dir = findFile(progname, _rootCluster);
        
        if (dir != 0)
        {
            // found firmware file
            transmitString("found firmware..");
            deleteFile();
        } 
        else 
        {
            transmitString("no firmware.");
        }
    }
     
    // start main loop
    while(1)
    {
        if (IEEE_CTL == 0x00)
        {
            // if we are in an unlisten state,
            // wait for my address
            //transmitString("waiting.");
            buscmd = wait_for_device_address(address);
            filenotfound = 0;
            if (buscmd == LISTEN)
            {
                initcard = 0;
            }
        }
    
        wait_for_dav_low();
        
        // lower NDAC and NRFD
        PORTC = NOT_NDAC & NOT_NRFD;
        
        // read byte
        recv_byte_IEEE(&rdchar);
        
        // read bus value
        rdbus = PINC;
        
        if (filenotfound == 1)
        {
            filenotfound = 0;
            unlisten();
        }
        else if ((rdchar == 0xf0 || rdchar == 0xf1) && (rdbus & ATN) == 0x00)
        {
            // we are retrieving a filename for a load or save
            getting_filename = 1;
            if (rdchar == 0xf1)
            {
                // this is a save
                savefile = 1;
            }
            else 
            {
                savefile = 0;
            }

        }
        else if (getting_filename == 1)
        {
            // add character to filename
            progname[filename_position] = rdchar;
            filename_position++;
            progname[filename_position] = 0;
            
            if ((rdbus & EOI) == 0)
            {
            
                // this is a directory request
                if (progname[0] == '$')
                {
                    getting_filename = 0;
                    filename_position = 0;
                }
                else 
                {
                    // check for DLOAD command, remove 0: from start of filename
                    if (progname[0] == '0' && progname[1] == ':')
                    {
                        for (i = 0; i < filename_position-2; i++)
                        {
                            progname[i] = progname[i+2];
                        }
                        
                        filename_position -= 2;
                    }
                
                
                    // add extension
                    progname[filename_position++] = '.';
                    progname[filename_position++] = 'P';
                    progname[filename_position++] = 'R';
                    progname[filename_position++] = 'G';
                    progname[filename_position] = 0;
                
                    getting_filename = 0;
                    filename_position = 0;
                    
                    transmitString(progname);
                    gotname = 1;
                }
            }
        }
        else if (rdchar == 0x60 && (rdbus & ATN) == 0x00)
        {
            // check for directory command
            if (progname[0] == '$')
            {
                _buffer[0] = 0x01;
                _buffer[1] = 0x04;
                _buffer[2] = 0x1F;
                _buffer[3] = 0x04;
                _buffer[4] = 0x00;
                _buffer[5] = 0x00;
                _buffer[6] = 0x12;
                
                // print directory title
                sprintf(&_buffer[7], "\"PETDISK V2.0    \"      ");
                _buffer[31] = 0x00;
            }
        }
        
        // raise NDAC
        PORTC = NOT_NRFD;
        wait_for_dav_high();
        // open file if needed
        if (initcard == 0)
        {
            // initialize card
            for (i=0; i<10; i++)
            {
              error = SD_init();
              if(!error)
                  break;
            }
            
            if (i == 10)
            {
                // reset current directory to root
                currentDirectoryCluster = 0;
            }
            
            error = getBootSectorData (); //read boot sector and keep necessary data in global variables
            if (currentDirectoryCluster == 0)
            {
                currentDirectoryCluster = _rootCluster;
            }
            
            initcard = 1;
        }
        if (gotname == 1)
        {
            if (savefile == 0)
            {
                if (!openFileForReading(progname, currentDirectoryCluster))
                {
                    // file not found
                    transmitString("file not found");
                    filenotfound = 1;
                }
                
                // clear string
                for (i = 0; i < FNAMELEN; i++) progname[i] = 0x00;
            }
            else 
            {
                // open file
                openFileForWriting(progname, currentDirectoryCluster);
            }
        }
        
        gotname = 0;
        
        if ((rdchar == 0x3f) || rdchar == 0x5f && (rdbus & ATN) == 0x00)
        {
            // unlisten or untalk command
            PORTC = NOT_NDAC;
            
            //transmitByte('*');
            //transmitHex(CHAR, rdchar);
            
            unlisten();
        }
        else 
        {
            // lower NRFD
            PORTC = NOT_NDAC;
        }
        
        // LOAD requested
        if (rdchar == 0x60 && (rdbus & ATN) == 0x00)
        {
            if (filenotfound == 0)
            {
                // this is a LOAD
                // release NRFD/NDAC
                DDRC = NDAC;
                
                // wait for atn high
                wait_for_atn_high();
            
                DDRC = DAV | EOI;
                PORTC = 0xFF;
                
                // change data bus to output
                DATA_CTL = 0xff;
                DDRB = DDRB | (DATA0 | DATA1);
                
                // get packet
                //if (strcmp(progname, "$") == 0)
                if (progname[0] == '$')
                {
                    transmitString("directory..");
                    for (i = 0; i < 32; i++)
                    {
                        send_byte(_buffer[i],0);
                    }
                    
                    // this is a change directory command
                    if (progname[1] == ':')
                    {
                        // get the cluster for the new directory
                        //transmitString(&progname[2]);
                        //TX_NEWLINE;
                        
                        dir = findFile(&progname[2], currentDirectoryCluster);
                        
                        if (dir != 0)
                        {
                            // get new directory cluster
                            currentDirectoryCluster = getFirstCluster(dir);
                            if (currentDirectoryCluster == 0)
                            {
                                currentDirectoryCluster = _rootCluster;
                            }
                        }
                        
                        //transmitHex(LONG, currentDirectoryCluster);
                        //TX_NEWLINE;
                    }
                    
                    // write directory entries
                    ListFilesIEEE(currentDirectoryCluster);
                }
                else
                {
                    sending = 1;
                    byteCounter = 1;
                    fileSize = _filePosition.fileSize;
                    while(sending == 1)
                    {
                        if (_filePosition.byteCounter < _filePosition.fileSize)
                        {
                            bytes_to_send = getNextFileBlock();
                            for (k = 0; k < bytes_to_send; k++)
                            {
                                if (byteCounter >= fileSize)
                                {
                                    send_byte(_buffer[k], 1);
                                }
                                else
                                {
                                    send_byte(_buffer[k], 0);
                                    byteCounter++;
                                }
                            }
                        }
                        else
                        {
                            sending = 0;
                        }
                    }
                }
            
                // raise DAV and EOI
                PORTC = 0xFF;
                
                // switch back to input mode
                DDRC = NRFD | NDAC;
                
                DATA_CTL = 0x00;
                DDRB = ~MISO & ~DATA0 & ~DATA1 & ~CASSETTE_READ & ~CASSETTE_WRITE;
                PORTC = NOT_NDAC;
            }
            
            unlisten();
            
        }
        else if (rdchar == 0x61 && (rdbus & ATN) == 0)
        {
            // save command
            writeFileFromIEEE();
            unlisten();
        }
    }
    
}