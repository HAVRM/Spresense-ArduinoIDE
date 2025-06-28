#if (SUBCORE != 2)
#error "Core selection is wrong!!"
#endif

#include <Arduino.h>
#include <MP.h> 
#include <arch/board/board.h>

struct imuPacket{
  volatile int status; //0: ready, 1: busy, 2: done, -1: no update
  volatile float ave_temp;
  volatile float ave_gyro;
  volatile float ave_gx;
  volatile float ave_gy;
  volatile float ave_gz;
  volatile float ave_accel;
  volatile float ave_ax;
  volatile float ave_ay;
  volatile float ave_az;
  volatile float mdb_accel;
  volatile float mdb_ax;
  volatile float mdb_ay;
  volatile float mdb_az;
};

imuPacket *packet=NULL;

#ifdef SUBCORE
USER_HEAP_SIZE(80 * 1024); 
#endif

#include <nuttx/config.h>

#include <sys/ioctl.h>
#include <time.h>
#include <inttypes.h>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <signal.h>
#include <poll.h>
#include <errno.h>

#include <nuttx/sensors/cxd5602pwbimu.h>

#define CXD5602PWBIMU_DEVPATH      "/dev/imu0"

#define itemsof(a) (sizeof(a)/sizeof(a[0]))

static int start_sensing(int fd, int rate, int adrange, int gdrange,
                         int nfifos)
{
  cxd5602pwbimu_range_t range;
  int ret;

  ret = ioctl(fd, SNIOC_SSAMPRATE, rate);
  if (ret)
    {
      printf("ERROR: Set sampling rate failed. %d\n", errno);
      return 1;
    }

  range.accel = adrange;
  range.gyro = gdrange;
  ret = ioctl(fd, SNIOC_SDRANGE, (unsigned long)(uintptr_t)&range);
  if (ret)
    {
      printf("ERROR: Set dynamic range failed. %d\n", errno);
      return 1;
    }

  ret = ioctl(fd, SNIOC_SFIFOTHRESH, nfifos);
  if (ret)
    {
      printf("ERROR: Set sampling rate failed. %d\n", errno);
      return 1;
    }

  ret = ioctl(fd, SNIOC_ENABLE, 1);
  if (ret)
    {
      printf("ERROR: Enable failed. %d\n", errno);
      return 1;
    }

  return 0;
}

void getIMUDATA(int argc, FAR char *argv[])
{
  int fd;
  int ret;
  struct pollfd fds[1];
  struct timespec start, now, delta;
  cxd5602pwbimu_data_t *outbuf = NULL;
  cxd5602pwbimu_data_t *p = NULL;
  cxd5602pwbimu_data_t *last;

  const int samplerate = 1920;
  const int adrange = 2;
  const int gdrange = 125;
  const int nfifos = 1;

  fd = open(CXD5602PWBIMU_DEVPATH, O_RDONLY);
  if (fd < 0)
    {
      printf("ERROR: Device %s open failure. %d\n", CXD5602PWBIMU_DEVPATH, errno);
      return 1;
    }

  outbuf = (cxd5602pwbimu_data_t *)malloc(sizeof(cxd5602pwbimu_data_t) * samplerate);
  if (outbuf == NULL)
    {
      printf("ERROR: Output buffer allocation failed.\n");
      return 1;
    }
  last = outbuf + samplerate;

  fds[0].fd = fd;
  fds[0].events = POLLIN;

  ret = start_sensing(fd, samplerate, adrange, gdrange, nfifos);
  if (ret)
    {
      close(fd);
      return ret;
    }

  memset(&now, 0, sizeof(now));

  for (p = outbuf; p < last; p++)
    {
      ret = poll(fds, 1, 1000);
      if (ret < 0)
        {
          if (errno != EINTR)
            {
              printf("ERROR: poll failed. %d\n", errno);
            }
          break;
        }
      if (ret == 0)
        {
          printf("Timeout!\n");
        }
      if (p == outbuf)
        {
          clock_gettime(CLOCK_MONOTONIC, &start);
        }

      if (fds[0].revents & POLLIN)
        {
          ret = read(fd, p, sizeof(*p));
          if (ret != sizeof(*p))
            {
              printf("ERROR: read size mismatch! %d\n", ret);
            }
        }

      clock_gettime(CLOCK_MONOTONIC, &now);
      clock_timespec_subtract(&now, &start, &delta);
      if (delta.tv_sec >= 1)
        {
          break;
        }
    }
    
  last = p;

  close(fd);

  float Maxax=outbuf->ax;
  float minax=Maxax;
  float Maxay=outbuf->ay;
  float minay=Maxay;
  float Maxaz=outbuf->az;
  float minaz=Maxaz;
  float MaxAcc=sqrt(pow(outbuf->ax,2)+pow(outbuf->ay,2)+pow(outbuf->az,2));
  float MinAcc=MaxAcc;

  for (p = outbuf; p < last; p++)
    {
      packet->ave_temp+=p->temp;
      packet->ave_gx+=p->gx;
      packet->ave_gy+=p->gy;
      packet->ave_gz+=p->gz;
      packet->ave_gyro+=sqrt(pow(p->gx,2)+pow(p->gy,2)+pow(p->gz,2));
      packet->ave_ax+=p->ax;
      packet->ave_ay+=p->ay;
      packet->ave_az+=p->az;
      packet->ave_accel+=sqrt(pow(p->ax,2)+pow(p->ay,2)+pow(p->az,2));
      Maxax=max(Maxax,p->ax);
      minax=min(minax,p->ax);
      Maxay=max(Maxay,p->ay);
      minay=min(minay,p->ay);
      Maxaz=max(Maxaz,p->az);
      minaz=min(minaz,p->az);
      MaxAcc=max(MaxAcc, sqrt(pow(p->ax,2)+pow(p->ay,2)+pow(p->az,2)));
      MinAcc=min(MinAcc, sqrt(pow(p->ax,2)+pow(p->ay,2)+pow(p->az,2)));

    }
  packet->ave_temp=packet->ave_temp/(last - outbuf);
  packet->ave_gyro=packet->ave_gyro/(last - outbuf);
  packet->ave_gx=packet->ave_gx/(last - outbuf);
  packet->ave_gy=packet->ave_gy/(last - outbuf);
  packet->ave_gz=packet->ave_gz/(last - outbuf);
  packet->ave_accel=packet->ave_accel/(last - outbuf);
  packet->ave_ax=packet->ave_ax/(last - outbuf);
  packet->ave_ay=packet->ave_ay/(last - outbuf);
  packet->ave_az=packet->ave_az/(last - outbuf);
  packet->mdb_accel=max(MaxAcc-packet->ave_accel, packet->ave_accel-MinAcc);
  packet->mdb_ax=max(Maxax-packet->ave_ax, packet->ave_ax-minax);
  packet->mdb_ay=max(Maxay-packet->ave_ay, packet->ave_ay-minay);
  packet->mdb_az=max(Maxaz-packet->ave_az, packet->ave_az-minaz);
  packet->mdb_accel=20*log10(packet->mdb_accel/0.000001);
  packet->mdb_ax=20*log10(packet->mdb_ax/0.000001);
  packet->mdb_ay=20*log10(packet->mdb_ax/0.000001);
  packet->mdb_az=20*log10(packet->mdb_ax/0.000001);
   
  free(outbuf);
}

void setup(void) {
  int8_t msgid;
  MP.begin();

  int ret;
  ret = board_cxd5602pwbimu_initialize(5);
  if (ret < 0)
    {
      printf("ERROR: Failed to initialize CXD5602PWBIMU.\n");
    }
  else
    {
      printf("board_cxd5602pwbimu_initialize: OK\n");
    }
}

void loop()
{
  int8_t msgid;
  printf("Sub: Wait for memory.\n"); 
  MP.Recv(&msgid, &packet);
  int ret;
  switch(packet->status){
    case 0:
      packet->status=1;
      printf("Sub:get IMU data.\n");
      getIMUDATA(0,NULL);

    case -1:
      printf("Sub:send data.\n");
      packet->status=2;
      break;

    default:
      break;
  }  
  delay(100);
}
