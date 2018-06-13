#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/poll.h>

#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <string.h> // C stuff
#include <stdlib.h>

#include <string> // C++ stuff
#include <cstring>
#include <chrono>
#include <iostream>
#include <fstream>
#include <regex>

#include "clara.hpp"


#define BUFFER_LENGTH 1024
#define BAUD 115200
#define NUMBER_OF_PINGS 100
#define DEBUG 0

auto PINGER = 0; // Are we sending the pings or recieving them
auto PORT = std::string("/dev/ttyUSB0");
auto ID = 1;
auto NUMBER_OF_PLANES = 2;

std::ofstream outputFile;
std::ofstream latencyFile;
std::ofstream pongFile;

typedef std::chrono::high_resolution_clock Clock;
auto start = Clock::now();
auto end = Clock::now();
auto timeoutStart = Clock::now();
auto Timestamp = Clock::now();

bool msg_ping = 1;
struct pollfd fds[1];
static int temp_tty_fd;

unsigned long ping_counter = 1;
int number_of_pings_sent = 0;
int number_of_pongs_sent = 0;
int number_of_pongs_recieved = 0;
int number_of_pings_recieved = 0;
int number_of_timeouts = 0;
int mismatched_regex = 0;
int corrupted_timestamp = 0;

void msg_in(std::string, int);

void msg_out(int tty_fd) {
  // Initialize stuff
  if (PINGER == 1) {
    int bytes_sent;
    // Check to see if we need to write data
    if (msg_ping || std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - timeoutStart).count() > 100) {
      if(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - timeoutStart).count() > 100) {
        ++number_of_timeouts;
      }
      msg_ping = 0;
      std::string temp = "PING " + std::to_string(ID) + " : " + std::to_string(ping_counter) + " : " + std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - Timestamp).count()) + "\n";
      bytes_sent = write(tty_fd, temp.c_str(), temp.size() + 1);
      std::cout << "Bytes Sent: " << bytes_sent << "\n";
      start = std::chrono::system_clock::now();
      timeoutStart = start;
      ++number_of_pings_sent;
      ++ping_counter;

    }
  }


  // Check to see if data needs to be read
  // Give timeout of 100 ms
  unsigned char buf[BUFFER_LENGTH];
  memset(buf, 0, BUFFER_LENGTH);
  int pollrc = poll(fds, 1, 100);
  if (pollrc < 0) {
    perror("Error when polling for input");
  }
  else if (pollrc > 0) {
    // If there is data to read
    if (fds[0].revents & POLLIN) {
      ssize_t rc = read(tty_fd, (void *)buf, BUFFER_LENGTH);
      std::cout << "Bytes Read: " << rc << "\n";
      if (rc > 0) {
        std::string message (reinterpret_cast<char*>(buf), sizeof(buf));
        msg_in(message, tty_fd);
      }
    }

  }
}

/****** Testing ******/
std::regex e ("([A-Z]+)[[:space:]]([[:digit:]]+)[[:space:]]:[[:space:]]([[:digit:]]+)[[:space:]]:[[:space:]]([[:digit:]]+)");
std::smatch match;
std::string PingOrPong;
std::string ID_tag;
std::string PingNumber;
std::string TimeStampFromSender;
std::string TimeStampFromSenderOld = "0";
/*********************/
void msg_in(std::string buf, int tty_fd) {

  try {

    static int pongCounter = 0;
    std::string message = buf.data();
    if (std::regex_search(message, match, e)) {
      PingOrPong = match.str(1);
      ID_tag = match.str(2);
      PingNumber = match.str(3);
      TimeStampFromSender = match.str(4);

      if (PingOrPong.compare("PONG") == 0 && PINGER == 1) { // If we got a PONG message

        end = Clock::now();

        // Check to see if the timestamp got corrupted in transmission
        uint64_t tempTime = std::stoull(TimeStampFromSender);
        uint64_t tempTimeOld = std::stoull(TimeStampFromSenderOld);
        if (tempTime < tempTimeOld) {
          // Corrupted timestamp
          ++corrupted_timestamp;
        }
        else {
          // Everything normal
          TimeStampFromSenderOld = TimeStampFromSender;
          printf("Pong %s recieved from %s with latency %lu ms\n", PingNumber.c_str(), ID_tag.c_str(), std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - Timestamp).count() - tempTime);//std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now().time_since_epoch()).count() - tempTime);
          if (PINGER == 1) {
            latencyFile << ID_tag.c_str() << "," << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - Timestamp).count() - tempTime << "," << PingNumber.c_str() << "," << std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - Timestamp).count() << "\n";
          }

          ++pongCounter;
          ++number_of_pongs_recieved;
          if (pongCounter == NUMBER_OF_PLANES) { // AND if all planes have responded
            msg_ping = 1;
            pongCounter = 0;
          }
        }
      }

      else if (PingOrPong.compare("PING") == 0) { // If we got a PING message record where it came from
        uint64_t tempTime = std::stoull(TimeStampFromSender);
        uint64_t tempTimeOld = std::stoull(TimeStampFromSenderOld);
        if (tempTime < tempTimeOld) {
          // Corrupted timestamp
          ++corrupted_timestamp;
        }
        else {
          // Everything normal
          TimeStampFromSenderOld = TimeStampFromSender;

          printf("Ping %d recieved from %s with send time %s\n", stoi(PingNumber), ID_tag.c_str(), TimeStampFromSender.c_str());
          ++number_of_pings_recieved;

          // Respond with pong
          std::string temp = "PONG " + std::to_string(ID) + " : " + PingNumber + " : " + TimeStampFromSender + "\n";
          int bytes_sent = write(tty_fd, temp.c_str(), temp.size() + 1);
          ++number_of_pongs_sent;
          if (PINGER == 0) {
            pongFile << ID_tag.c_str() << "," << PingNumber.c_str() << "," << TimeStampFromSender << "," << std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - Timestamp).count() << "\n";
          }
        }
      }
    }
    else {
      ++mismatched_regex;
    }
  }
  catch (int e) {
    printf("ERROR: Exception NBR %d\n", e);
  }
}

void handler(int signum) {
  // #if PINGER == 1
  if (PINGER == 1) {
    printf("Number of Pings Sent: %d\n", number_of_pings_sent);
    printf("Number of Pongs recieved: %d\n", number_of_pongs_recieved);
    printf("Number of Timeouts: %d\n", number_of_timeouts);
    printf("Number of Mismatched Regex: %d\n", mismatched_regex);
    printf("Number of Corrupted Timestamps: %d\n", corrupted_timestamp);
    outputFile << "Number of Pings Sent: \t" << number_of_pings_sent << "\n";
    outputFile << "Number of Pongs recieved: \t" << number_of_pongs_recieved << "\n";
    outputFile << "Number of Timeouts: \t" << number_of_timeouts << "\n";
    outputFile << "Number of Corrupted Timestamps: \t" << corrupted_timestamp << "\n";
    outputFile.close();
    latencyFile.close();
  }
  else {
    printf("Number of Pings recieved: %d\n", number_of_pings_recieved);
    printf("Number of Pongs Sent: %d\n", number_of_pongs_sent);
    printf("Number of Mismatched Regex: %d\n", mismatched_regex);
    printf("Number of Corrupted Timestamps: %d\n", corrupted_timestamp);
    outputFile << "Number of Pongs Sent: \t" << number_of_pongs_sent << "\n";
    outputFile << "Number of Pings recieved: \t" << number_of_pings_recieved << "\n";
    outputFile << "Number of Corrupted Timestamps: \t" << corrupted_timestamp << "\n";
    outputFile.close();
    pongFile.close();
  }
  close(temp_tty_fd);
  exit(1);
}

int main(int argc, char *argv[]) {

  auto PingOrPong = 0;
  auto Ident = 0;
  auto PortName = std::string{};
  auto NumberOfDevices = 0;
  auto showhelp = false;
  auto rate = 20.0f;
  int number_of_sends = 0;

  auto parser =
  clara::Arg(PingOrPong, "Ping (1) or Pong (0)")("Specify if you are sending the ping (1) or recieving them (0)") |
  clara::Arg(PortName, "Port Name")("Specify where the device is connected (default is /dev/ttyUSB0") |
  clara::Arg(Ident, "Identity Number")("Identification of the device sending") |
  clara::Arg(NumberOfDevices, "Number of other Devices")("Number of other devices connected") |
  clara::Arg(rate, "Loop Rate (ms)")("Rate at which the program will send data") |
  clara::Arg(number_of_sends, "Number of Messages to Send")("Number of messages to Send") |
  clara::Help(showhelp);

  try {
    auto result = parser.parse(clara::Args(argc, argv));
    if (!result) {
      std::cerr << "Error in command line: " << result.errorMessage() << "\n";
      return 1;
    }
    else if (showhelp) {
      parser.writeToStream(std::cout);
      return 0;
    }
    else {
      std::cout << "Ping or Pong:  " << PingOrPong << "\n";
      std::cout << "Port Name: " << PortName << "\n";
      std::cout << "Identity Number: " << Ident << "\n";
      std::cout << "Number of Devices:  " << NumberOfDevices << "\n";
      std::cout << "Loop Rate (ms):  " << rate << "\n";
      std::cout << "Number of Messages to Send:  " << number_of_sends << "\n";
    }
  }
  catch (std::exception const & e) {
    std::cout << e.what() << "\n";
  }
  PINGER  = PingOrPong;
  PORT = PortName;
  ID = Ident;
  NUMBER_OF_PLANES = NumberOfDevices;

  struct sigaction sa;
  sa.sa_handler = handler;
  sigemptyset(&sa.sa_mask);
  sa.sa_flags = SA_RESTART;
  if (sigaction(SIGINT, &sa, NULL) == -1) {
    perror("Error when handling SIGINT");
  }
  // Open serial port to device
  int tty_fd;
  if((tty_fd = open(PORT.c_str(), O_RDWR | O_SYNC)) < 0) {
    perror("Error while opening serial port\n");
    exit(-11);
  }
  temp_tty_fd = tty_fd;
  char buf[BUFFER_LENGTH];
  fds[0].fd = tty_fd;
  fds[0].events = POLLIN;

  struct termios tio;
  memset(&tio, 0, sizeof(tio)); // Set to 0
  tio.c_iflag = IGNPAR; // Ignore parity errors
  tio.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hw flow control
  tio.c_iflag &= ~(IXON | IXOFF | IXANY); // Disable flow control
  tio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // Raw mode
  tio.c_cflag = BAUD | CS8 | CREAD | CLOCAL; // 115200 BAUD, 8 bit Character Size mask, enable read, ignore control lines
  tio.c_oflag = 0; // Disable output flags
  tio.c_cc[VMIN] = 1; // Minimum # of characters for noncanonical read
  tio.c_cc[VTIME] = 0; // No timeout for noncanonical read
  tcflush(tty_fd, TCIFLUSH); // Flush output/input data not transmitted
  tcsetattr(tty_fd, TCSANOW, &tio); // Set paramaters

  if (PINGER == 1) {
    outputFile.open("Ping_output.txt");
    latencyFile.open("Latency_output.txt");
    latencyFile << "ID,Latency(ms),OrderNumber,TimeSinceStart(ms)\n";
  }
  else {
    pongFile.open("Pong_output.txt");
    pongFile << "ID,OrderNumber,TimestampRecieved,TimeSinceStart(ms)\n";
    outputFile.open("Debug_output.txt");
  }

  auto loop_end = std::chrono::system_clock::now();
  auto loop_start = loop_end;

  Timestamp = std::chrono::system_clock::now();

  while(number_of_pings_sent < 10000) {
    loop_start = std::chrono::system_clock::now();
    if (std::chrono::duration_cast<std::chrono::milliseconds>(loop_start - loop_end).count() > rate) {
      msg_out(tty_fd);
    }
  }

  if (PINGER == 1) {
    printf("Number of Pings Sent: %d\n", number_of_pings_sent);
    printf("Number of Pongs recieved: %d\n", number_of_pongs_recieved);
    printf("Number of Timeouts: %d\n", number_of_timeouts);
    printf("Number of Mismatched Regex: %d\n", mismatched_regex);
    printf("Number of Corrupted Timestamps: %d\n", corrupted_timestamp);
    outputFile << "Number of Pings Sent: \t" << number_of_pings_sent << "\n";
    outputFile << "Number of Pongs recieved: \t" << number_of_pongs_recieved << "\n";
    outputFile << "Number of Timeouts: \t" << number_of_timeouts << "\n";
    outputFile << "Number of Corrupted Timestamps: \t" << corrupted_timestamp << "\n";
    outputFile.close();
    latencyFile.close();
  }
  else {
    printf("Number of Pings recieved: %d\n", number_of_pings_recieved);
    printf("Number of Pongs Sent: %d\n", number_of_pongs_sent);
    printf("Number of Mismatched Regex: %d\n", mismatched_regex);
    printf("Number of Corrupted Timestamps: %d\n", corrupted_timestamp);
    outputFile << "Number of Pongs Sent: \t" << number_of_pongs_sent << "\n";
    outputFile << "Number of Pings recieved: \t" << number_of_pings_recieved << "\n";
    outputFile << "Number of Corrupted Timestamps: \t" << corrupted_timestamp << "\n";
    outputFile.close();
    pongFile.close();
  }
  close(temp_tty_fd);
  exit(1);

  return 0;
}
