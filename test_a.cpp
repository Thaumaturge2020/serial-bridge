#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <iostream>
#include <sys/types.h>  /*数据类型，比如一些XXX_t的那种*/
#include <sys/stat.h>   /*定义了一些返回值的结构，没看明白*/
#include <errno.h>      /*错误号定义*/

int main(int argc, char** argv) {
  int fd = open("/dev/ttyACM0", O_RDWR | O_NOCTTY);
  if (fd < 0) {
    //打开串口失败，退出
    return -1;
  } 

  struct termios oldtio = { 0 };
  struct termios newtio = { 0 };
  tcgetattr(fd, &oldtio);

  newtio.c_cflag = B115200 | CS8 | CLOCAL | CREAD;
  newtio.c_iflag = 0; // IGNPAR | ICRNL
  newtio.c_oflag = 0;
  newtio.c_lflag = 0; // ICANON
  newtio.c_cc[VTIME] = 0;
  newtio.c_cc[VMIN] = 1;
  tcflush(fd, TCIOFLUSH);
  tcsetattr(fd, TCSANOW, &newtio);
  //设置为非阻塞模式
  fcntl(fd, F_SETFL, O_NONBLOCK);

  while (true) {
    unsigned char buffer[1024] = {0};
    int ret = read(fd, buffer, sizeof(buffer));
    if (ret > 0) {
      //依次将读取到的数据输出到日志
      for (int i = 0; i < ret; ++i) {
        printf("%02x ", buffer[i]);
      }

	  printf("\n");

      //当收到数据时，再将收到的数据原样发送
      int n = write(fd, buffer, ret);
      if (n != ret) {
        printf("发送失败");
      }

      //当收到0xFF时，跳出循环
      if (buffer[0] == 0xFF) {
        break;
      }
    } else {
      //没收到数据时，休眠50ms，防止过度消耗cpu
      usleep(1000 * 50);
    }
  }

  close(fd);
  return 0;
}

