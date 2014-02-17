/*
    IEEE488.c
    IEEE Routines in the PETdisk storage device
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


#include "IEEE488.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/signal.h>
#include <inttypes.h>
#include <util/delay.h>
#include <string.h>
#include "bf-avr-sdlib/UART_routines.h"
#include "bf-avr-sdlib/FAT32.h"
#include "bf-avr-sdlib/sd_routines.h"

void wait_for_dav_high()
{
    unsigned char rdchar;
    rdchar = PINC;
    rdchar = rdchar & DAV;
    while (rdchar == 0x00)
    {
        rdchar = PINC;
        rdchar = rdchar & DAV;
    }
}

void wait_for_dav_low()
{
    unsigned char rdchar;
    rdchar = PINC;
    rdchar = rdchar & DAV;
    while (rdchar != 0x00)
    {
        rdchar = PINC;
        rdchar = rdchar & DAV;
    }
}

void wait_for_atn_high()
{
    unsigned char rdchar;
    rdchar = PINC;
    rdchar = rdchar & ATN;
    while (rdchar == 0x00)
    {
        rdchar = PINC;
        rdchar = rdchar & ATN;
    }
}

void wait_for_atn_low()
{
    unsigned char rdchar;
    rdchar = PINC;
    rdchar = rdchar & ATN;
    while (rdchar != 0x00)
    {
        rdchar = PINC;
        rdchar = rdchar & ATN;
    }
}



void wait_for_nrfd_high()
{
    unsigned char rdchar;
    rdchar = PINC;
    rdchar = rdchar & NRFD;
    while (rdchar == 0x00)
    {
        rdchar = PINC;
        rdchar = rdchar & NRFD;
    }
}

void wait_for_ndac_high()
{
    unsigned char rdchar;
    rdchar = PINC;
    rdchar = rdchar & NDAC;
    while (rdchar == 0x00)
    {
        rdchar = PINC;
        rdchar = rdchar & NDAC;
    }
}

void recv_byte_IEEE(unsigned char *byte)
{
    unsigned char hi, lo;
    hi = DATA_HI_IN;
    lo = DATA_LO_IN;
    
    hi = hi & ~DATALO;
    lo = lo & DATALO;
    
    *byte = ~(hi | lo);
    
}

void send_byte(unsigned char byte, int last)
{
    unsigned char temp,temp2;
    unsigned char lo,hi;
    // invert the byte
    temp = ~byte;
    lo = temp & DATALO;
    hi = temp & ~DATALO;
    
    
    // output the byte
    //PORTA = temp;
    DATA_PORT = hi;
    temp = PORTB;
    temp = temp & ~DATALO;
    temp = temp | lo;
    PORTB = temp;
    
    // wait for NRFD high
    wait_for_nrfd_high();
    
    // lower DAV and EOI
    
    if (last == 0)
    {
        temp = DAV;
        PORTC = ~temp;
    }
    else 
    {
        temp = DAV;
        temp2 = EOI;
        PORTC = (~temp) & (~temp2);
    }

    // wait for NDAC high
    wait_for_ndac_high();
    
    // raise DAV
    temp = DAV | EOI;
    // output to bus
    PORTC = temp;
    
}

void unlisten()
{
    // all bus lines input
    IEEE_CTL = 0x00;
    // set pullups
    IEEE_PORT = 0xff;
}

unsigned char wait_for_device_address(unsigned char my_address)
{
    unsigned char primary_address,dir;
    primary_address = 0;
    //transmitString("waiting..");
    while (primary_address != my_address)
    {
        
        // wait for atn signal
        wait_for_atn_low();
        
        // lower NDAC to respond
        IEEE_PORT = NOT_NDAC;
        // set output lines
        IEEE_CTL = NDAC | NRFD;
        
        // wait for primary address
        wait_for_dav_low();
        
        // lower NRFD
        IEEE_PORT = NOT_NDAC & NOT_NRFD;
        
        // read data
        recv_byte_IEEE(&primary_address);
        dir = primary_address & 0xf0;
        primary_address = primary_address & 0x0f;
        transmitHex(CHAR, primary_address);
        
        if (primary_address == my_address && (dir == TALK || dir == LISTEN))
        {
            // raise NDAC
            IEEE_PORT = NOT_NRFD;
            // wait for data to finish
            wait_for_dav_high();
            
            // release control and handle transaction
            IEEE_PORT = NOT_NDAC;
        }
        else 
        {
            // unlisten the bus
            unlisten();
            // wait for atn to release
            wait_for_atn_high();
        }
    }
    return dir;
}

struct dir_Structure* ListFilesIEEE(unsigned long firstCluster)
{
    struct dir_Structure *dir;
    unsigned char startline;
    unsigned char entry[32];
    unsigned char thisch;
    unsigned int dir_start;
    unsigned int f;
    unsigned int file;
    unsigned char hasLongEntry;
    int fname_length;
    
    dir_start = 0x041f;
    file = 0;
    
    // open the current directory
    openDirectory(firstCluster);
    
    do {
        // get next directory entry
        dir = getNextDirectoryEntry();
        
        if (dir == 0) // this is the end of the directory
        {
            // write ending bytes
            startline = 0;
            dir_start += 0x001e;
            
            entry[startline] = (unsigned char)(dir_start & 0x00ff);
            entry[startline+1] = (unsigned char)((dir_start & 0xff00) >> 8);
            entry[startline+2] = 0xff;
            entry[startline+3] = 0xff;
            sprintf(&entry[startline+4], "BLOCKS FREE.             ");
            entry[startline+29] = 0x00;
            entry[startline+30] = 0x00;
            entry[startline+31] = 0x00;
            
            for (f = 0; f < 32; f++)
            {
                if (f == 31)
                {
                    send_byte(entry[f], 1);
                }
                else
                {
                    send_byte(entry[f], 0);
                }
            }
            return 0;
        }
        else
        {
            if (dir->attrib != ATTR_VOLUME_ID)
            {
                dir_start += 0x0020;
                
                startline = 0;
                fname_length = 0;
                
                entry[startline] = (unsigned char)(dir_start & 0x00ff);
                entry[startline+1] = (unsigned char)((dir_start & 0xff00) >> 8);
                entry[startline+2] = file+1;
                entry[startline+3] = 0x00;
                entry[startline+4] = 0x20;
                entry[startline+5] = 0x20;
                entry[startline+6] = 0x22;
                
                hasLongEntry = _filePosition.isLongFilename;
                if (hasLongEntry)
                {
                    if (_filePosition.fileName[0] == 0)
                    {
                        hasLongEntry = 0;
                    }
                }
                
                if (hasLongEntry)
                {
                    while(_filePosition.fileName[fname_length] != '.' &&
                          _filePosition.fileName[fname_length] != 0 &&
                          fname_length < 17)
                    {
                        thisch = _filePosition.fileName[fname_length];
                        if (thisch >= 'a' && thisch <= 'z')
                        {
                            thisch -= 32;
                        }
                        entry[startline+7+fname_length] = thisch;
                        fname_length++;
                    }
                }
                else
                {
                    fname_length = 0;
                    for (f = 0; f < 8; f++)
                    {
                        if (dir->name[f] == ' ')
                            break;
                        
                        entry[startline+7+f] = dir->name[f];
                        fname_length++;
                    }
                }
                
                entry[startline+7+fname_length] = 0x22;
                for (f = 0; f < (17 - fname_length); f++)
                {
                    entry[startline+7+fname_length+f+1] = ' ';
                }
                
                
                if (dir->attrib == ATTR_DIRECTORY)
                {
                    entry[startline+25] = 'D';
                    entry[startline+26] = 'I';
                    entry[startline+27] = 'R';
                }
                else
                {
                    entry[startline+25] = dir->name[8];
                    entry[startline+26] = dir->name[9];
                    entry[startline+27] = dir->name[10];
                }
                
                entry[startline+28] = ' ';
                entry[startline+29] = ' ';
                entry[startline+30] = ' ';
                entry[startline+31] = 0x00;
                file++;
                
                for (f = 0; f < 32; f++)
                {
                    send_byte(entry[f], 0);
                }
            }
        }
    }
    while (dir != 0);
}


void writeFileFromIEEE ()
{
    unsigned int numBytes;
    unsigned char rdchar;
    unsigned char rdbus;
    
    numBytes = 0;
    do
    {
        wait_for_dav_low();
        PORTC = NOT_NDAC & NOT_NRFD;
        // read byte
        recv_byte_IEEE(&rdchar);
        rdbus = PINC;
        
        _buffer[numBytes++] = rdchar;
        
        if(numBytes >= 512)   //though 'i' will never become greater than 512, it's kept here to avoid
        {				//infinite loop in case it happens to be greater than 512 due to some data corruption
            numBytes = 0;
            writeBufferToFile(numBytes);
        }
        
        // raise NDAC
        PORTC = NOT_NRFD;
        wait_for_dav_high();
        PORTC = NOT_NDAC;
	}
    while((rdbus & EOI) != 0x00);
    
    if (numBytes > 0)
    {
        writeBufferToFile(numBytes);
    }
    
    closeFile();
}