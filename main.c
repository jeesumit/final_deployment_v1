#include <fcntl.h>   // For file control options
#include <string.h>
#include <termios.h> // For terminal interface control
#include <unistd.h>  // For read(), close()
#include <errno.h>   // For error numbers
#include <string.h>  // For strerror()
#include <stdio.h>   // For standard input/output
#include <stdlib.h>
#include "gnss.h"
#include <math.h>
#include <cjson/cJSON.h>
#include <time.h>
int serial_port,bytes_written;
extern double lat,lng,vel,head,hdop,hgt;
extern long int tim ,date;
extern int nsv,gpsqa; 
void port_init(const char *usbport);  
struct termios tty; 
int hr, sc, mn,day,mnt,yr;
long int tis;
int concatenate_integers_as_string(int a, int b, int c);

int main(){
    struct tm utc_time_struct;
    time_t epoch_time;
    char data[255];
    char read_buf[255];
    port_init("/dev/ttyS0");
    char filename[]="data/loggns.json";
     int fd = open(filename, O_WRONLY | O_CREAT | O_TRUNC, 0666);
     if(fd == -1){
           perror("open--File -- ERROR");
       }
    while(1){
     
     int num_bytes = read(serial_port, read_buf, sizeof(read_buf));
     if (num_bytes < 0) {
        fprintf(stderr, "Error reading: %s\n", strerror(errno));
     }
   
/*    int fd = open(filename, O_WRONLY | O_CREAT | O_TRUNC, 0666);
    if (fd == -1) {
        perror("openERROR");
    }*/
   
     
    
     id_filter(read_buf);
     
     if(tim!=0){
			hr = (tim / 10000);
			mn = (tim % 10000) / 100;
			sc = (tim % 10000) % 100;
			hr = hr+5;
			mn = mn+30;
			if (mn > 59)
			{
				mn = mn-60;
				hr = hr +1;
			}
			if (hr > 23)
			{
				hr = hr-24;
				 
			}
     tis = concatenate_integers_as_string(hr,mn,sc);
     }
					
   //sprintf(data,"%d-%02d-%02d %02d:%02d:%02d\n",yr,mnt,day,hr,mn,sc);
     
     if(lat != 0.0 && lng != 0.0 && hgt !=0.0){
   cJSON *json = cJSON_CreateObject();
   cJSON_AddNumberToObject(json, "Velocity", vel);
   cJSON_AddNumberToObject(json, "Latitude", lat);
   cJSON_AddNumberToObject(json, "Longitude", lng);
   cJSON_AddNumberToObject(json, "Head", head);
   cJSON_AddNumberToObject(json, "Altitude", hgt);
   cJSON_AddNumberToObject(json, "Hdop", hdop);
   cJSON_AddNumberToObject(json, "Time", tis);
   cJSON_AddNumberToObject(json, "Date", date);
   // convert the cJSON object to a JSON string
   char *json_str = cJSON_Print(json);  
   
   printf("%s\n", json_str);
    ssize_t bytes_written = write(fd,json_str, strlen(json_str));
    if (bytes_written == -1) {
        perror("writeERROR");
        close(fd);
        // Handle error
        }
        
   cJSON_free(json_str);
   cJSON_Delete(json);
        
    }
   /* if (close(fd) == -1) {
        perror("closeERROR");
        // Handle error
    }*/
     }
      if(close(fd) == -1){
          perror("closeERROR");
       }

    //memset(data,'\0',sizeof(data));
    close(serial_port);
    tcsetattr(serial_port,TCSANOW,&tty);
    return 0;
}

void port_init(const char *usbport){

///const char *portname = "/dev/ttyUSB0"; // Replace with your port
    serial_port = open(usbport, O_RDWR | O_NOCTTY);

    if (serial_port < 0) {
        fprintf(stderr, "Error opening serial port '%s': %s\n", usbport, strerror(errno));
        exit(1);
    }
    
        
    if (tcgetattr(serial_port, &tty) != 0) {
        fprintf(stderr, "Error from tcgetattr: %s\n", strerror(errno));
        close(serial_port);
        exit(1);
    }

    // Set baud rate
     cfsetospeed(&tty, B9600); 
     cfsetispeed(&tty, B9600);

    // Configure other settings (example: 8N1 - 8 data bits, no parity, 1 stop bit) 
    tty.c_cflag &= ~PARENB; // No parity
    tty.c_cflag &= ~CSTOPB; // 1 stop bit
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8; // 8 data bits
    tty.c_cflag &= ~CRTSCTS; // Disable hardware flow control
    tty.c_cflag |= CREAD | CLOCAL; // Enable reading, ignore control lines

    tty.c_lflag |= ICANON; // Disable canonical mode (line-by-line input)
    tty.c_lflag &= ~ECHO;   // Disable echo
    tty.c_lflag &= ~ECHOE;  // Disable erasure
    tty.c_lflag &= ~ECHONL;  // Disable new-line echo
    tty.c_lflag &= ~ISIG;   // Disable interpretation of signals

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Disable software flow control
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);

    tty.c_oflag &= ~OPOST; // Raw output

    tty.c_cc[VMIN] = 1;  // Read at least 1 character
    tty.c_cc[VTIME] = 0; // No timeout
    tcflush(serial_port,TCIFLUSH);
    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
       fprintf(stderr, "Error from tcsetattr: %s\n", strerror(errno));
        close(serial_port);
        exit(1);
    }
    
}

int concatenate_integers_as_string(int a, int b, int c) {
    char s1[20]; // Buffer for first integer as string
    char s2[20]; // Buffer for second integer as string
    char s3[20]; // Buffer for second integer as string
    char result_str[40]; // Buffer for concatenated string

    // Convert integers to strings
    sprintf(s1, "%d", a);
    sprintf(s2, "%d", b);
    sprintf(s3, "%d", c);

    // Concatenate the strings
    strcpy(result_str, s1); // Copy s1 to result_str
    strcat(result_str, s2); // Append s2 to result_str
    strcat(result_str, s3); // Append s2 to result_str

    // Convert the concatenated string back to an integer
    return atoi(result_str);
}


