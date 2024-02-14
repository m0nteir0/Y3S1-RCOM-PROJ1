// Link layer protocol implementation

#include "link_layer.h"
#include <fcntl.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <termios.h>
#include <unistd.h>

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source
#define BAUDRATE 38400
#define BUF_SIZE 256

#define FLAG 0x7E
#define ESC 0x7D
#define A_TR 0x03   // commands by Transmitter and replies by Receiver
#define A_RT 0x01   // commands by Receiver and replies by Transmitter
#define C_SET 0x03  // sent by the transmitter to initiate a connection
#define C_UA 0x07   // confirmation to the reception of a valid supervision frame
#define C_RR0 0x05  // sent by the Receiver that it is ready to receive an information frame number 0
#define C_RR1 0x85  // sent by the Receiver that it is ready to receive an information frame number 1
#define C_REJ0 0x01 // sent by the Receiver that it rejects an information frame number 0 (detected an error)
#define C_REJ1 0x81 // sent by the Receiver that it rejects an information frame number 1 (detected an error)
#define C_DISC 0x0B // indicates the termination of a connection
#define C_N0 0x00   // info frame number 0
#define C_N1 0x40   // info frame number 1

#define TRUE 1
#define FALSE 0

struct timeval start_time, end_time; // start and end time of the transfer

int ns = 0, nr = 1; // info frame sequence number: ns -> sender number, nr -> receiver number
int expectedN = 0;  // expected info frame number
int fd;             // file descriptor for serial port
int alarmEnabled = FALSE;
int alarmCount = 0;
int timeout;
int nRetransmissions;
LinkLayerRole role;
struct termios oldtio, newtio;

typedef enum
{
  START,
  FLAG_RCV,
  A_RCV,
  C_RCV,
  BCC1_OK,
  READING,
  ESC_FOUND,
  STOP_State,
} State;

void alarmHandler(int signal)
{
  alarmEnabled = FALSE;
  alarmCount++;

  printf("Alarm #%d\n", alarmCount);
}

int sendFrame(int fd, unsigned char A, unsigned char C)
{
  unsigned char frame[5];
  frame[0] = FLAG;
  frame[1] = A;
  frame[2] = C;
  frame[3] = A ^ C;
  frame[4] = FLAG;

  int res = write(fd, frame, 5);
  if (res < 0)
  {
    printf("Error: problem writing to serial port\n");
    return -1;
  }
  return 0;
}

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{

  gettimeofday(&start_time, NULL);

  printf("serialPort - %s\n", connectionParameters.serialPort);

  fd = open(connectionParameters.serialPort, O_RDWR | O_NOCTTY);

  printf("fd - %d\n", fd);

  if (fd < 0)
  {
    perror(connectionParameters.serialPort);
    exit(-1);
  }

  if (tcgetattr(fd, &oldtio) == -1)
  {
    perror("tcgetattr");
    exit(-1);
  }

  memset(&newtio, 0, sizeof(newtio));

  newtio.c_cflag = connectionParameters.baudRate | CS8 | CLOCAL | CREAD;
  newtio.c_iflag = IGNPAR;
  newtio.c_oflag = 0;
  newtio.c_lflag = 0;
  newtio.c_cc[VTIME] = 1;
  newtio.c_cc[VMIN] = 0;

  tcflush(fd, TCIOFLUSH);

  if (tcsetattr(fd, TCSANOW, &newtio) == -1)
  {
    perror("tcsetattr");
    return -1;
  }

  State state = START;

  unsigned char byte;
  timeout = connectionParameters.timeout;
  nRetransmissions = connectionParameters.nRetransmissions;
  role = connectionParameters.role;
  switch (connectionParameters.role)
  {
  case LlTx:
  {
    (void)signal(SIGALRM, alarmHandler);

    while (state != STOP_State && alarmCount < nRetransmissions)
    {
      if (sendFrame(fd, (unsigned char)A_TR, (unsigned char)C_SET) < 0)
      {
        printf("Error: problem sending SET frame\n");
        return -1;
      }
      printf("SET sent\n");

      alarm(connectionParameters.timeout);
      alarmEnabled = TRUE;

      while (state != STOP_State && alarmEnabled)
      {
        if (read(fd, &byte, 1) < 0)
        {
          printf("Error: problem reading from serial port\n");
          return -1;
        }
        switch (state)
        {
        case START:
          if (byte == FLAG)
            state = FLAG_RCV;
          break;
        case FLAG_RCV:
          if (byte == A_TR)
            state = A_RCV;
          else if (byte != FLAG)
            state = START;
          break;
        case A_RCV:
          if (byte == C_UA)
            state = C_RCV;
          else if (byte == FLAG)
            state = FLAG_RCV;
          else
            state = START;
          break;
        case C_RCV:
          if (byte == (A_TR ^ C_UA))
            state = BCC1_OK;
          else if (byte == FLAG)
            state = FLAG_RCV;
          else
            state = START;
          break;
        case BCC1_OK:
          if (byte == FLAG)
          {
            state = STOP_State;
            alarm(0);
            printf("UA received\n");
          }
          else
            state = START;
          break;
        default:
          break;
        }
      }
    }
    if (alarmCount == nRetransmissions)
    {
      printf("Error: maximum number of retransmissions reached\n");
      return -1;
    }

    break;
  }
  case LlRx:
  {

    // state machine
    while (state != STOP_State)
    {
      if (read(fd, &byte, 1) < 0)
      {
        printf("Error: problem reading from serial port\n");
        return -1;
      }

      switch (state)
      {
      case START:
        if (byte == FLAG)
          state = FLAG_RCV;
        break;
      case FLAG_RCV:
        if (byte == A_TR)
          state = A_RCV;
        else if (byte != FLAG)
          state = START;
        break;
      case A_RCV:
        if (byte == C_SET)
          state = C_RCV;
        else if (byte == FLAG)
          state = FLAG_RCV;
        else
          state = START;
        break;
      case C_RCV:
        if (byte == (A_TR ^ C_SET))
          state = BCC1_OK;
        else if (byte == FLAG)
          state = FLAG_RCV;
        else
          state = START;
        break;
      case BCC1_OK:
        if (byte == FLAG)
        {
          state = STOP_State;
          printf("SET received\n");
        }
        else
          state = START;
        break;
      default:
        break;
      }
    }
    if (sendFrame(fd, A_TR, C_UA) < 0)
    {
      printf("Error: problem sending UA frame\n");
      return -1;
    }
    printf("UA sent\n");
    break;
  }

  default:
    return -1;
    break;
  }

  return fd;
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize)
{
  alarmCount = 0;

  int frameSize = 6 + bufSize;
  unsigned char *frame = (unsigned char *)malloc(frameSize);

  frame[0] = FLAG;
  frame[1] = A_TR;
  frame[2] = ns == 0 ? C_N0 : C_N1;
  frame[3] = frame[1] ^ frame[2];

  int j = 4; // data index in frame
  for (unsigned int i = 0; i < bufSize; i++)
  {
    if (buf[i] == FLAG || buf[i] == ESC)
    {
      frame = realloc(frame, ++frameSize);
      frame[j++] = ESC;
      frame[j++] = buf[i] ^ 0x20;
    }
    else
    {
      frame[j++] = buf[i];
    }
  }

  unsigned char BCC2 = buf[0];

  for (unsigned int i = 1; i < bufSize; i++)
    BCC2 ^= buf[i];

  if (BCC2 == FLAG || BCC2 == ESC)
  {
    frame = realloc(frame, ++frameSize);
    frame[j++] = ESC;
    frame[j++] = BCC2 ^ 0x20;
  }
  else
  {
    frame[j++] = BCC2;
  }
  frame[j++] = FLAG;

  State state = START;
  unsigned char byte;
  unsigned char c_byte;
  int rejected = 0, accepted = 0;

  while (alarmCount < nRetransmissions && state != STOP_State)
  {
    printf("I frame #%d sent\n", ns);
    write(fd, frame, j);
    alarm(timeout);
    alarmEnabled = TRUE;

    // state machine
    while (state != STOP_State && alarmEnabled)
    {
      if (read(fd, &byte, 1) < 0)
      {
        printf("Error: problem reading from serial port\n");
        return -1;
      }

      switch (state)
      {
      case START:
        if (byte == FLAG)
          state = FLAG_RCV;
        break;
      case FLAG_RCV:
        if (byte == A_TR)
          state = A_RCV;
        else if (byte != FLAG)
          state = START;
        break;
      case A_RCV:
        if (byte == C_RR0 || byte == C_RR1)
        {
          c_byte = byte;
          state = C_RCV;
        }
        else if (byte == C_REJ0 || byte == C_REJ1)
        {
          c_byte = byte;
          state = C_RCV;
        }
        else if (byte == FLAG)
          state = FLAG_RCV;
        else
          state = START;
        break;
      case C_RCV:
        if (byte == (A_TR ^ c_byte))
          state = BCC1_OK;
        else if (byte == FLAG)
          state = FLAG_RCV;
        else
          state = START;
        break;
      case BCC1_OK:
        if (byte == FLAG)
        {
          state = STOP_State;
          if (c_byte == C_RR0 || c_byte == C_RR1)
          {
            printf("RR%d received\n", c_byte == C_RR0 ? 0 : 1);
            ns = (ns + 1) % 2;
            rejected = 0;
            accepted = 1;
          }
          else if (c_byte == C_REJ0)
          {
            printf("REJ0 received\n");
            ns = 0;
            rejected = 1;
          }
          else if (c_byte == C_REJ1)
          {
            printf("REJ1 received\n");
            ns = 1;
            rejected = 1;
          }
          alarm(0);
        }

        else
          state = START;
        break;
      default:
        break;
      }
    }
  }

  free(frame);

  if (alarmCount == nRetransmissions)
  {
    printf("Error: maximum number of retransmissions reached\n");
    return -1;
  }

  if (accepted && !rejected)
    return frameSize;
  else if (rejected && !accepted)
    return 0;
  return -1;
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet)
{

  unsigned char byte;
  int i = 0;
  State state = START;
  char c;

  // state machine
  while (state != STOP_State)
  {
    if (read(fd, &byte, 1) < 0)
    {
      printf("Error: problem reading from serial port\n");
      return -1;
    }

    switch (state)
    {
    case START:
    {
      i = 0;
      if (byte == FLAG)
        state = FLAG_RCV;
      break;
    }
    case FLAG_RCV:
    {
      if (byte == A_TR)
        state = A_RCV;
      else if (byte != FLAG)
        state = START;
      else
        return 0;
      break;
    }
    case A_RCV:
    {
      if (byte == C_N0 || byte == C_N1)
      {
        c = byte;
        state = C_RCV;
      }
      else if (byte == C_DISC)
      {
        printf("DISC received\n");
        return -1;
      }
      else if (byte == FLAG)
        state = FLAG_RCV;
      else
        state = START;
      break;
    }
    case C_RCV:
    {
      if (byte == (A_TR ^ c))
      {
        if (c == C_N0)
        {
          if (expectedN == 0)
            state = READING;
          else if (expectedN == 1)
          {
            printf("Error: Wrong frame received. Expected %d\n", expectedN);
            if (sendFrame(fd, A_TR, C_RR1) < 0)
            {
              printf("Error: problem sending RR1 frame\n");
              return -1;
            }
            printf("RR1 sent\n");

            return 0;
          }
        }
        else if (c == C_N1)
        {
          if (expectedN == 1)
            state = READING;
          else if (expectedN == 0)
          {
            printf("Error: Wrong frame received. Expected %d\n", expectedN);
            if (sendFrame(fd, A_TR, C_RR0) < 0)
            {
              printf("Error: problem sending RR0 frame\n");
              return -1;
            }
            printf("RR0 sent\n");

            return 0;
          }
        }
      }
      else if (byte == FLAG)
        state = FLAG_RCV;
      else
        state = START;
      break;
    }
    case READING:
    {
      if (byte == ESC)
        state = ESC_FOUND;
      else if (byte == FLAG)
      {

        unsigned char bcc2 = packet[i - 1];
        i--;
        packet[i] = '\0';
        unsigned char verif = packet[0];

        for (int j = 1; j < i; j++)
        {
          verif = verif ^ packet[j];
        }

        if (bcc2 == verif)
        {
          state = STOP_State;
          printf("I frame #%d received\n", expectedN);

          if (expectedN == 0)
          {
            expectedN = 1;
            if (sendFrame(fd, A_TR, C_RR1) < 0)
            {
              printf("Error: problem sending RR1 frame\n");
              return -1;
            }
            printf("RR1 sent\n");
          }
          else
          {
            expectedN = 0;
            if (sendFrame(fd, A_TR, C_RR0) < 0)
            {
              printf("Error: problem sending RR0 frame\n");
              return -1;
            }
            printf("RR0 sent\n");
          }
          return i;
        }
        else
        {
          printf("BCC2 incorrect\n");
          if (expectedN == 0)
          {
            if (sendFrame(fd, A_TR, C_REJ0) < 0)
            {
              printf("Error: problem sending REJ0 frame\n");
              return -1;
            }
            printf("REJ0 sent\n");
          }
          else
          {
            if (sendFrame(fd, A_TR, C_REJ1) < 0)
            {
              printf("Error: problem sending REJ1 frame\n");
              return -1;
            }
            printf("REJ1 sent\n");
          }

          return 0;
        }
      }
      else
      {
        packet[i] = byte;
        i++;
      }
      break;
    }
    case ESC_FOUND:
    {
      state = READING;

      if (byte == 0x5e) // destuffing
      {
        packet[i] = FLAG;
        i++;
      }
      else if (byte == 0x5d)
      {
        packet[i] = ESC;
        i++;
      }
      else
      {
        packet[i] = ESC;
        i++;
        packet[i] = byte;
        i++;
      }
      break;
    }
    default:
      break;
    }
  }

  return 0;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose(int showStatistics)
{
  alarmCount = 0;

  State state = START;
  unsigned char byte;
  printf("Preparing to close the connection...\n");
  switch (role)
  {
  case LlTx:
  {
    (void)signal(SIGALRM, alarmHandler);

    while (alarmCount < nRetransmissions && state != STOP_State)
    {
      if (sendFrame(fd, A_TR, C_DISC) < 0)
      {
        printf("Error: problem sending DISC frame\n");
        return -1;
      }
      printf("DISC sent\n");
      alarm(timeout);
      alarmEnabled = TRUE;

      // state machine
      while (state != STOP_State && alarmEnabled)
      {
        if (read(fd, &byte, 1) < 0)
        {
          printf("Error: problem reading from serial port\n");
          return -1;
        }
        switch (state)
        {
        case START:
          if (byte == FLAG)
            state = FLAG_RCV;
          break;
        case FLAG_RCV:
          if (byte == A_TR)
            state = A_RCV;
          else if (byte != FLAG)
            state = START;
          break;
        case A_RCV:
          if (byte == C_DISC)
            state = C_RCV;
          else if (byte == FLAG)
            state = FLAG_RCV;
          else
            state = START;
          break;
        case C_RCV:
          if (byte == (A_TR ^ C_DISC))
            state = BCC1_OK;
          else if (byte == FLAG)
            state = FLAG_RCV;
          else
            state = START;
          break;
        case BCC1_OK:
          if (byte == FLAG)
          {
            state = STOP_State;
            alarm(0);
            printf("DISC received\n");
          }
          else
            state = START;
          break;
        default:
          break;
        }
      }
    }

    if (alarmCount == nRetransmissions)
    {
      printf("Error: maximum number of retransmissions reached\n");
      return -1;
    }

    if (sendFrame(fd, A_TR, C_UA) < 0)
    {
      printf("Error: problem sending UA frame\n");
      return -1;
    }
    printf("UA sent\n");
    break;
  }
  case LlRx:
  {
    while (state != STOP_State)
    {
      if (read(fd, &byte, 1) < 0)
      {
        printf("Error: problem reading from serial port\n");
        return -1;
      }
      switch (state)
      {
      case START:
        if (byte == FLAG)
          state = FLAG_RCV;
        break;
      case FLAG_RCV:
        if (byte == A_TR)
          state = A_RCV;
        else if (byte != FLAG)
          state = START;
        break;
      case A_RCV:
        if (byte == C_DISC)
          state = C_RCV;
        else if (byte == FLAG)
          state = FLAG_RCV;
        else
          state = START;
        break;
      case C_RCV:
        if (byte == (A_TR ^ C_DISC))
          state = BCC1_OK;
        else if (byte == FLAG)
          state = FLAG_RCV;
        else
          state = START;
        break;
      case BCC1_OK:
        if (byte == FLAG)
        {
          state = STOP_State;
          printf("DISC received\n");
        }
        else
          state = START;
        break;
      default:
        break;
      }
    }

    if (sendFrame(fd, (unsigned char)A_TR, (unsigned char)C_DISC) < 0)
    {
      printf("Error: problem sending DISC frame\n");
      return -1;
    }
    printf("DISC sent\n");

    state = START;

    // state machine
    while (state != STOP_State)
    {
      if (read(fd, &byte, 1) < 0)
      {
        printf("Error: problem reading from serial port\n");
        return -1;
      }
      switch (state)
      {
      case START:
        if (byte == FLAG)
          state = FLAG_RCV;
        break;
      case FLAG_RCV:
        if (byte == A_TR)
          state = A_RCV;
        else if (byte != FLAG)
          state = START;
        break;
      case A_RCV:
        if (byte == C_UA)
          state = C_RCV;
        else if (byte == FLAG)
          state = FLAG_RCV;
        else
          state = START;
        break;
      case C_RCV:
        if (byte == (A_TR ^ C_UA))
          state = BCC1_OK;
        else if (byte == FLAG)
          state = FLAG_RCV;
        else
          state = START;
        break;
      case BCC1_OK:
        if (byte == FLAG)
        {
          state = STOP_State;
          printf("UA received\n");
        }
        else
          state = START;
        break;
      default:
        break;
      }
    }

    break;
  }
  }

  if (tcsetattr(fd, TCSANOW, &oldtio) == -1)
  {
    perror("tcsetattr");
    exit(-1);
  }

  printf("Closing...\n");
  close(fd);
  if (showStatistics)
  {
    gettimeofday(&end_time, NULL);
    double elapsed_time = (end_time.tv_sec - start_time.tv_sec) + (end_time.tv_usec - start_time.tv_usec) / 1000000.0;
    printf("Total time: %f seconds\n", elapsed_time);
  }

  return 1;
}
