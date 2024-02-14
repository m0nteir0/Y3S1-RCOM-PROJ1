// Aplication layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

int showStats = TRUE;

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
  
  LinkLayer connectionParameters = {
      .baudRate = baudRate,
      .timeout = timeout,
      .nRetransmissions = nTries,
  };
  strcpy(connectionParameters.serialPort, serialPort);               
  connectionParameters.role = strcmp(role, "tx") == 0 ? LlTx : LlRx; 

  printf("Opening connection...\n");
  int fd = llopen(connectionParameters);
  if (fd < 0)
  {
    printf("Error: opening connection\n");
    
    if (llclose(showStats) < 0)
      {
        printf("Error: closing serial port\n");
        exit(-1);
      }
  }
  printf("Connection opened sucessfully\n");
  printf("---------------------\n");

  switch (connectionParameters.role)
  {
  case LlTx:
  {

    FILE *file = fopen(filename, "r"); 
    if (file == NULL)
    {
      printf("Error: opening file\n");
      exit(-1);
    }

    long int pos1 = ftell(file); 
    fseek(file, 0L, SEEK_END);          
    long int size = ftell(file) - pos1; 
    printf("File size: %ld\n", size);
    fseek(file, 0L, SEEK_SET); 

    long int auxFileSize = size;
    unsigned char countBytes = 0;

    while (auxFileSize != 0)          
    {
      auxFileSize = auxFileSize >> 8; 
      countBytes++;
    }

    unsigned char c = 2;                        
    unsigned char t1 = 0, l1 = countBytes;
    unsigned char *v1 = malloc(countBytes);

    long int tempSize = size;
    for (int i = countBytes - 1; i >= 0; i--)
    {                                           
      v1[i] = (0xFF & tempSize);                
      tempSize = tempSize >> 8;                 
    }

    unsigned char t2 = 1, l2 = strlen(filename); 
    unsigned char *v2 = malloc(l2);

    for (int i = 0; i < l2; i++)
    {                                             
      v2[i] = filename[i];
    }


    unsigned char *controlPacket = malloc(5 + l1 + l2); 
    controlPacket[0] = c;
    controlPacket[1] = t1;
    controlPacket[2] = l1;

    for (int i = 0; i < l1; i++)
    {
      controlPacket[3 + i] = v1[i];
    }

    controlPacket[3 + l1] = t2;
    controlPacket[4 + l1] = l2;

    for (int i = 0; i < l2; i++)
    {
      controlPacket[5 + l1 + i] = v2[i];
    }

    if (llwrite(controlPacket, 5 + l1 + l2) < 0)
    {
      printf("Error: writing SOT Control Packet to serial port\n");
      if (llclose(showStats) < 0)
      {
        printf("Error: closing serial port\n");
        exit(-1);
      }
      exit(-1);
    }
    printf("SOT \n");

    unsigned char *content = malloc(size); 
    fread(content, 1, size, file);         
    long int sizeRemaining = size;
    printf("---------------------\n");
    while (sizeRemaining >= 0)
    {
      printf("Size Remaining: %ld\n", sizeRemaining);
      int dataSize = sizeRemaining > (MAX_PAYLOAD_SIZE - 3) ? (MAX_PAYLOAD_SIZE - 3) : sizeRemaining; 

      int datapacketSize = dataSize + 3;

      unsigned char *dataPacket = malloc(datapacketSize);
      dataPacket[0] = 1;                                                        

      dataPacket[1] = dataSize >> 8;   
      dataPacket[2] = dataSize & 0xFF; 

      
      memcpy(dataPacket + 3, content + (size - sizeRemaining), dataSize); 
      
      
      int res = 0;
      if ((res = llwrite(dataPacket, dataSize + 3)) < 0)
      {
        printf("Error: writing data packet to serial port\n");
        if (llclose(showStats) < 0)
      {
        printf("Error: closing serial port\n");
        exit(-1);
      }
        exit(-1);
      }
      printf("Bytes written: %d\n", res);
      if (res == 0) continue;
      

      sizeRemaining -= MAX_PAYLOAD_SIZE - 3;
      printf("---------------------\n");
    }

    c = 3;                                                  
    controlPacket[0] = c;

    if (llwrite(controlPacket, 5 + l1 + l2) < 0)
    {
      printf("Error: writing EOT control packet to serial port\n");
      if (llclose(showStats) < 0)
      {
        printf("Error: closing serial port\n");
        exit(-1);
      }
      exit(-1);
    }
    printf("End of Transmission\n");
    printf("---------------------\n");

    if (llclose(showStats) < 0)
    {
      
      printf("Error: closing serial port\n");
      exit(-1);
    }

    break;
  }

  case LlRx:
  {
 
    unsigned char *controlPacket = malloc(5);

    if (llread(controlPacket) < 0) 
    {
      printf("Error: reading SOT control packet from serial port\n");
      exit(-1);
    }

    if (controlPacket[0] != 2) 
    {
      printf("Error: control packet is not SOT\n");
      exit(-1);
    }

    printf("Received SOT Control Packet\n");

    long int fileSize = 0;                         
    for (int i = 0; i < controlPacket[2]; i++)
    {
      fileSize = fileSize << 8; 
      fileSize += controlPacket[3 + i];
    }

    FILE *newFile = fopen(filename, "wb"); 
    if (newFile == NULL)
    {
      printf("Error: opening file\n");
      exit(-1);
    }
    printf("---------------------\n");
    long int sizeRemaining = fileSize;      
    while (sizeRemaining >= 0)
    {
      int packetSize = sizeRemaining + 3 > MAX_PAYLOAD_SIZE ? MAX_PAYLOAD_SIZE : sizeRemaining + 3;
      
      unsigned char dataPacket[packetSize];
      int res;
      if ((res = llread(dataPacket)) < 0)
      {
        printf("Error: reading data packet from serial port\n");

        if (llclose(showStats) < 0)
        {
          printf("Error: closing serial port\n");
          exit(-1);
        }
        return;
      }
      if(res == 0){
        continue;
      }
      
      if (dataPacket[0] != 1)         
      {
        printf("dataPacket[0]=%0x", dataPacket[0]);
        printf("Error: data packet is not correct\n");
        exit(-1); 
      }

      
      int dataSize = 256 * dataPacket[1] + dataPacket[2]; 
      printf("Received Data Size: %d\n", dataSize);
      
      int bytesWritten = fwrite(dataPacket + 3, 1, dataSize, newFile); 
      printf("Bytes written to file: %d\n", bytesWritten);
      fseek(newFile, 0L, SEEK_END); 

      sizeRemaining -= MAX_PAYLOAD_SIZE - 3;
      printf("---------------------\n");
    }

    fclose(newFile); 

    if (llread(controlPacket) < 0)     
    {
      printf("Error: reading EOT control packet from serial port\n");
      exit(-1);
    }

    if (controlPacket[0] != 3)        
    {
      printf("Error: control packet is not EOT\n");
      exit(-1);
    }

    printf("Received EOT Control Packet\n");
    printf("---------------------\n");

    if (llclose(showStats) < 0)
    {
      printf("Error: closing serial port\n");
      exit(-1);
    }

    break;
  }

  default:
    exit(-1);
    break;
  }
}
