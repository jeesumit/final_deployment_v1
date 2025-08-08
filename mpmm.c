
#include <stdlib.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <stdint.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <time.h>
#include "mm.h"
#include <cjson/cJSON.h>
double lat_p,lon_p,lat_pa,lon_pa,lat_pb,lon_pb;
double ang_ba=0.0, ang_ab=0.0, dis_bc=0.0, dis_ab=0.0, dis_ac=0.0, ang_ac=0.0, cte_ab=0.0, diffang_h=0.0;
int cte2can(double kval);
char filname[] = "./path_deployment_v2.0/data/path_points.txt";

/*----------------------KALMAN GAIN PARAMETERS --------------------------*/
double cur_lat =0.0,cur_lng=0.0,prev_lat=0.0,prev_lng=0.0;
double pred_lat,pred_lng,sum_lat,sum_lng,out_lat=0.0,out_lng=0.0;
int n =0,canbus=0;
float gain=0.5;
double prdstpt=0.0,prev_cte=0.0;
/*-----------------------------------------------------------------------*/

int main()
{
int fds;
const char *portname = "/dev/ttyUSB0";
struct termios tty;
char message[4]={'\0'};
ssize_t bytes_written;
/*-----------------------------------------------------------------------*/
    // Open the serial port
    fds = open(portname, O_RDWR | O_NOCTTY);
    if (fds == -1) {
        perror("Error opening serial port");
        return 1;
    }

    // Get current serial port settings
    if (tcgetattr(fds, &tty) != 0) {
        perror("Error getting termios settings");
        close(fds);
        return 1;
    }

    // Configure serial port settings (example: 9600 baud, 8N1)
    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);
    tty.c_cflag &= ~PARENB; // No parity
    tty.c_cflag &= ~CSTOPB; // 1 stop bit
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8; // 8 data bits
    tty.c_cflag &= ~CRTSCTS; // Disable hardware flow control
    tty.c_cflag |= CREAD | CLOCAL; // Enable read and ignore control lines
    tty.c_lflag &= ICANON; // Disable canonical mode
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG; // Disable interpretation of signals
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Disable software flow control
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable special handling of received bytes
    tty.c_oflag &= ~OPOST; // Disable output processing

    // Set the modified settings
    if (tcsetattr(fds, TCSANOW, &tty) != 0) {
        perror("Error setting termios settings");
        close(fds);
        return 1;
    }


int fd, res;
FILE * nf;
unsigned int *bn;
/*--------------------LOG FILE INIT___________________________________________*/
const char *logfile = "data/log.json";
//char *json_str_cte;
//cJSON *json_cte = cJSON_CreateObject();

/*------------------- Intialize GNSS FILE-------------------------------------*/
const char *filename = "inc/gnr.buf";
ssize_t bytes_read;
float spd,lat,lng,vel,head,hdop,hgt;
char buffer[1024];
char ch;
char data[255];
char line[255];
int count =0;
int line_count = 0;
double diff=0.0;
/*------------------- Intialize PATH File ------------------------------------*/
struct timeval str ,en;
gettimeofday(&str, NULL);


FILE *fp = fopen(filname,"r");
if (fp != NULL){
while(!feof(fp))
{
  ch = fgetc(fp);
  if(ch == '\n')
  {
    count++;
    
  }
}
 fclose(fp);    
}
printf("%d,",count);
nf = fopen(filname,"r");
if (nf == NULL){
      perror("An error occurred");
      return 0;
}

/*******************************************************************************/
while (!feof(nf) && line_count < count) { 
    gettimeofday(&en, NULL);
    /*--------------------------------------LINE PARSER ------------------------*/
    int k =0;
	char *token = strtok(line,",");
	while(token != NULL){
	if(k==0){
	lat_p = atof(token);
	}
	else if(k==1){
	lon_p = atof(token);
	}
	k+=1;
	token = strtok(NULL,",");
	}


    /*---------------------------------------------------------------------------*/
    FILE *fpk = fopen("data/gns.json", "r");
    if (fpk == NULL) {
        printf("Error: Unable to open the file.\n");
        return 1;
    }
    int len = fread(buffer, 1, sizeof(buffer), fpk);
    if(len < 0){
    fprintf(stderr, "Error reading: %s\n", strerror(errno));
    
    }	
	
    // parse the JSON data
    cJSON *json = cJSON_Parse(buffer);
    if (json == NULL) {
        const char *error_ptr = cJSON_GetErrorPtr();
        if (error_ptr != NULL) {
            printf("Error: %s\n", error_ptr);
        }
        cJSON_Delete(json);
    }

    // access the JSON data

cJSON *vel_item = cJSON_GetObjectItemCaseSensitive(json, "Velocity");
if (cJSON_IsNumber(vel_item)) {
        spd = vel_item->valuedouble;
    } 

cJSON *alt_item = cJSON_GetObjectItemCaseSensitive(json, "Altitude");
if (cJSON_IsNumber(alt_item)) {
        hgt = alt_item->valuedouble;
        
    } 

cJSON *lat_item = cJSON_GetObjectItemCaseSensitive(json, "Latitude");
if (cJSON_IsNumber(lat_item)) {
        lat = lat_item->valuedouble;
        
    } 

cJSON *lng_item = cJSON_GetObjectItemCaseSensitive(json, "Longitude");
if (cJSON_IsNumber(lng_item)) {
        lng = lng_item->valuedouble;
        
    } 


cJSON *head_item = cJSON_GetObjectItemCaseSensitive(json, "Head");
if (cJSON_IsNumber(head_item)) {
        head = head_item->valuedouble;
        
    } 
       // printf("%f %f \n",lat,lng);
	
     if(lat !=0.0 && lng !=0.0){
         if(en.tv_sec - str.tv_sec==1){
             lat_pa=lat;
             lon_pa=lng;
             
             lat_pb=lat_p;
             lon_pb=lon_p;
          }
          
          
	   n+=1;
	   cur_lat = lat;cur_lng = lng;
	   if(prev_lat==0.0 && prev_lng==0.0){
	   prev_lat = cur_lat;
	   prev_lng = cur_lng; 
	   }
	   
	   out_lat= update_filter(cur_lat, prev_lat, gain);
	   out_lng= update_filter(cur_lng, prev_lng, gain);
	   
	   sum_lat+=out_lat;
	   sum_lng+=out_lng;
	   
	   pred_lat= (sum_lat/n);
	   pred_lng= (sum_lng/n);
	   
	  
	   diff = distance(lat,lat_p,lng,lon_p);
	   dis_ab = distance(lat_pa,lat_pb,lon_pa,lon_pb);
	   //printf("Dist:%.2f  %.2f\n",diff,dis_ab);
	   
	   /*----------------------logic starts here---------------------------------------*/
	   ang_ab=initial_bearing(lat_pa,lat_pb,lon_pa,lon_pb);
	   ang_ba=initial_bearing(lat_pb,lat_pa,lon_pb,lon_pa);
	   dis_ac=distance(lat_pa,out_lat, lon_pa, out_lng);
	   dis_bc=distance(lat_pb,out_lat, lon_pb, out_lng);
	   ang_ac= initial_bearing(lat_pa,out_lat, lon_pa, out_lng);
	   diffang_h=head-ang_ab;
	   cte_ab = cross_track_error(dis_ac,ang_ab, ang_ac);
           
           if(cte_ab !=0.0){
           prev_cte = -1*0.6154*cte_ab;
           prdstpt =ConvertRadtoDeg(asin(prev_cte / dis_ac));
           }
           else{

            prev_cte = cte_ab;
            prdstpt = prev_cte;
            }
           canbus = pidHead(diffang_h,2*prdstpt,100.0,0.0,10.0);
           printf("cte:%.2f, estcte:%.2f, current_dist:%.2f, %d \n",cte_ab,prev_cte,diff,line_count);
	   /*------------------------ CANBUS SERIAL WRITE------------*/
	   sprintf(message,"%d\r\n",canbus);
	   bytes_written = write(fds,message, sizeof(message));
     	   if (bytes_written == -1) {
        	perror("Error writing to serial port");
        	close(fds);
    		} 
           /*---------------LOG FILE CREATION ------------------------*/
            int fdlg = open(logfile, O_WRONLY | O_CREAT | O_TRUNC, 0666);
             if (fdlg == -1) {
                perror("openERROR");
                }
           
           
            cJSON *json_cte = cJSON_CreateObject();
   	    cJSON_AddNumberToObject(json_cte, "Cross Track Error", cte_ab);
            cJSON_AddNumberToObject(json_cte, "Estimated CTE", prev_cte);
            cJSON_AddNumberToObject(json_cte, "Path Heading Error", diffang_h);
            cJSON_AddNumberToObject(json_cte, "estimated Latitude ", out_lat);
            cJSON_AddNumberToObject(json_cte, "estimated Longitude ", out_lng);
            cJSON_AddNumberToObject(json_cte, "Distance Path", dis_ab);
            cJSON_AddNumberToObject(json_cte, "Distance from Intial", dis_ac);
            cJSON_AddNumberToObject(json_cte, "Distance from Final", dis_bc);
            cJSON_AddNumberToObject(json_cte, "CAN", canbus);
            
   // convert the cJSON object to a JSON string
            char *json_str_cte = cJSON_Print(json_cte);  
           // printf("%s\n",json_str_cte);
            ssize_t bytes_write = write(fdlg,json_str_cte, strlen(json_str_cte));
            if (bytes_write == -1) {
                perror("writeERROR");
                close(fdlg);
        
                 }
                 
          // printf("%s\n", json_str_cte);
                 
           cJSON_free(json_str_cte);
           cJSON_Delete(json_cte);
           
           /*----------------- CHECKPOINTS ---------------------------*/
	   if(diff <5.0 && diff > 0.0){
		fgets(line, 255, nf);
		line_count++;
		gettimeofday(&str, NULL);	
		}
		
          if(diff > 10000 && line_count == 0){
	      fgets(line, 255, nf);
	      gettimeofday(&str, NULL);

                 }	
             
	  prev_lat=pred_lat;
	  prev_lng=pred_lng;
	  
	  if(n>=4){
	  n=0;
	  sum_lat=0;
	  sum_lng=0;
	  }
	  
	  
	  close(fdlg);
	}
	cJSON_Delete(json); 
        fclose(fpk); 
	
}
fclose(nf);
close(fds);
return 0;
/* restore the old port settings */
}

int cte2can(double kval){
int cte_steer, out;
cte_steer = (int)((1.76*(kval*kval))+(15.2*kval)+52.0); 
out=(int)Constrain(cte_steer, 19, 91);
return out;
}



/*
double update_filter(double measurement, double predict, double gain){
    double update = predict + gain*(measurement-predict);
    return update;
}



double distance(double lat1,double lat2,double lon1,double lon2){


	double  dlon = ConvertDegtoRad(lon2 - lon1) ;
	double dlat = ConvertDegtoRad(lat2 - lat1) ;
	double deg_lat1 = ConvertDegtoRad(lat1);
	double deg_lat2 = ConvertDegtoRad(lat2);
	double a= sin(dlat/2)*sin(dlat/2) + (cos(deg_lat1) * cos(deg_lat2))* (sin(dlon/2)*sin(dlon/2));
	double c=2 * atan2(sqrt(a),sqrt(1-a));
	double R= 6371000.0 ;
	double dist=c*R;
	return dist;
}


double ConvertDegtoRad(double degree) {
	double pi = 3.14159265359;
	return (degree * (pi /180));
}


double ConvertRadtoDeg(double radians) {
	double pi = 3.14159265359;
	return (radians * (180 /pi));
}*/
