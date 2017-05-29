/*
 *  V4L2 video capture example
 *
 *  This program can be used and distributed without restrictions.
 *
 *      This program is provided with the V4L2 API
 * see http://linuxtv.org/docs.php for more information
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

//#include <getopt.h>             /* getopt_long() */

#include <fcntl.h>              /* low-level i/o */
#include <unistd.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>

#include <linux/videodev2.h>
#include "v4l2write.h"

#define CLEAR(x) memset(&(x), 0, sizeof(x))

enum io_method {
  //IO_METHOD_READ,
        IO_METHOD_MMAP,
	//    IO_METHOD_USERPTR,
};

struct buffer {
        void   *start;
        size_t  length;
};

 char            *dev_name;
static enum io_method   io = IO_METHOD_MMAP;
static int              fd = -1;
struct buffer          *buffers;
static unsigned int     n_buffers;
//static int              out_buf=0;
//static int              force_format;
//static int              frame_count = 10;
static struct timeval timeBegin;//, timeNow;




static void errno_exit(const char *s)
{
        fprintf(stderr, "%s error %d, %s\n", s, errno, strerror(errno));
        exit(EXIT_FAILURE);
}

static int xioctl(int fh, int request, void *arg)
{
        int r;

        do {
                r = ioctl(fh, request, arg);
        } while (-1 == r && EINTR == errno);

        return r;
}

static void controlExposureExtended(int fd) {
  struct v4l2_ext_control c;
  struct v4l2_ext_controls cp;

  cp.ctrl_class=V4L2_CTRL_CLASS_USER;
  cp.count=1;
  cp.controls=&c;

  c.id=V4L2_CID_EXPOSURE_AUTO;
  c.value=V4L2_EXPOSURE_MANUAL;

  if (-1==ioctl (fd,VIDIOC_S_EXT_CTRLS,&cp)) {
    printf("failed to set auto exposure to manual.\n");
  } else  {
    printf("set auto exposure to manual.\n");
  }

  cp.ctrl_class=V4L2_CTRL_CLASS_USER;//
  cp.count=1;
  cp.controls=&c;
  c.id=V4L2_CID_EXPOSURE_ABSOLUTE;
  c.value=800;
  if (-1==ioctl (fd,VIDIOC_S_EXT_CTRLS,&cp)) {
    printf("failed to set exposure.\n");
  } else  {
    printf("set exposure to %d.\n",c.value);
  }
   

}


//void process_image(const void *p, int size) {
//  printf("size: %d\n",size);
//  return;
//}
/*
FILE *writefile;
static void process_image(const void *p, int size)
{

  float elapsedTime;
  gettimeofday(&timeNow, NULL);
  elapsedTime=(timeNow.tv_sec-timeBegin.tv_sec)
      + (timeNow.tv_usec-timeBegin.tv_usec)*.000001;
  printf("Time elapsed: %8.4f \n", elapsedTime);

  //writefile=fopen("image.yuvs","w");
  //     if (out_buf)
  //             fwrite(p, size, 1, writefile);

  //        fflush(stderr);
        fprintf(stderr, ".");
	//      fflush(writefile);
	//	fclose(writefile);
}
*/
static int read_frame(void)
{
        struct v4l2_buffer buf;
        //unsigned int i;

        switch (io) {

        case IO_METHOD_MMAP:
                CLEAR(buf);

                buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                buf.memory = V4L2_MEMORY_MMAP;

                if (-1 == xioctl(fd, VIDIOC_DQBUF, &buf)) {
                        switch (errno) {
                        case EAGAIN:
                                return 0;

                        case EIO:
                                /* Could ignore EIO, see spec. */

                                /* fall through */

                        default:
			  printf("read_frame Errno: %d\n",errno);
			  errno_exit("VIDIOC_DQBUF");
                        }
                }

                //assert(buf.index < n_buffers);
		//printf("buffer: %d\n",buf.index);
                process_image(buffers[buf.index].start, buf.bytesused);

                if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
                        errno_exit("VIDIOC_QBUF");
                break;

        }

        return 1;
}



void* mainimageloop(void*)
{
        gettimeofday(&timeBegin, NULL);
        unsigned int count;
        count = 1;//frame_count;
	fd_set fds;
	struct timeval tv;
	int r;
	
        while (count > 0) {
                for (;;) {

		  FD_ZERO(&fds);
		  FD_SET(fd, &fds);
		  
		  /* Timeout. */
		  tv.tv_sec = 2;
		  tv.tv_usec = 0;	       
		  r = select(fd + 1, &fds, NULL, NULL, &tv);		  
		  if (-1 == r) {
		    if (EINTR == errno)
		      continue;
		    errno_exit("select");
		  }
		  
		  if (0 == r) {
		    fprintf(stderr, "select timeout\n");
		    exit(EXIT_FAILURE);
		  }
		  if (read_frame())
		    break;
                }
        }
	return NULL;
}



 void onesteploop(void)
{
  //gettimeofday(&timeBegin, NULL);
  //     unsigned int count;
  //     count = frame_count;
  fd_set fds;
  	struct timeval tv;
  	int r;
	
        //while (count-- > 0) {
	//for (;;) {

	FD_ZERO(&fds);
		  FD_SET(fd, &fds);
		  
		  /* Timeout. */
		  tv.tv_sec = 2;
			  tv.tv_usec = 0;	       
			  r = select(fd + 1, &fds, NULL, NULL, &tv);		//  
			  if (-1 == r) {
		    if (EINTR == errno)
		      return;
			    errno_exit("select");
			  }
		  
			  if (0 == r) {
			    fprintf(stderr, "select timeout\n");
			    exit(EXIT_FAILURE);
			  }
		  if (read_frame())
		    return;//break;
		  //      }
		//}
}

static void stop_capturing(void)
{
        enum v4l2_buf_type type;

        switch (io) {
	  //case IO_METHOD_READ:
          //      /* Nothing to do. */
          //      break;

        case IO_METHOD_MMAP:
	  //case IO_METHOD_USERPTR:
                type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                if (-1 == xioctl(fd, VIDIOC_STREAMOFF, &type))
                        errno_exit("VIDIOC_STREAMOFF");
                break;
        }
}

static void describe_capturing(void) {
  struct v4l2_fmtdesc description;
  int out=0;
  char *c;
  int q;
  for (q=0;q<15;q++) {
    description.index=q;
    description.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;
    out=xioctl(fd, VIDIOC_ENUM_FMT, &description);
    if (out==-1) {
      printf("%d Done. \n",q);
      break;
    }
    c=(char*)&description.pixelformat;
    printf("mode %d Flags %d Description: %s code:%c%c%c%c\n",q,description.flags,description.description,c[0],c[1],c[2],c[3]);
  }
  
}

static void start_capturing(void)
{
        unsigned int i;
        enum v4l2_buf_type type;

        switch (io) {

        case IO_METHOD_MMAP:
                for (i = 0; i < n_buffers; ++i) {
                        struct v4l2_buffer buf;

                        CLEAR(buf);
                        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                        buf.memory = V4L2_MEMORY_MMAP;
                        buf.index = i;

                        if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
                                errno_exit("VIDIOC_QBUF");
                }
                type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                if (-1 == xioctl(fd, VIDIOC_STREAMON, &type))
                        errno_exit("VIDIOC_STREAMON");
                break;

        }
}

static void uninit_device(void)
{
        unsigned int i;

        switch (io) {

        case IO_METHOD_MMAP:
                for (i = 0; i < n_buffers; ++i)
                        if (-1 == munmap(buffers[i].start, buffers[i].length))
                                errno_exit("munmap");
                break;
        }

        free(buffers);
}
/*
static void init_read(unsigned int buffer_size)
{
  buffers = (buffer* )calloc(1, sizeof(*buffers));

        if (!buffers) {
                fprintf(stderr, "Out of memory\n");
                exit(EXIT_FAILURE);
        }

        buffers[0].length = buffer_size;
        buffers[0].start = malloc(buffer_size);

        if (!buffers[0].start) {
                fprintf(stderr, "Out of memory\n");
                exit(EXIT_FAILURE);
        }
}
*/
static void init_mmap(void)
{
        struct v4l2_requestbuffers req;

        CLEAR(req);

        req.count = 4;
        req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_MMAP;

        if (-1 == xioctl(fd, VIDIOC_REQBUFS, &req)) {
                if (EINVAL == errno) {
                        fprintf(stderr, "%s does not support "
                                 "memory mapping\n", dev_name);
                        exit(EXIT_FAILURE);
                } else {
                        errno_exit("VIDIOC_REQBUFS");
                }
        }

        if (req.count < 2) {
                fprintf(stderr, "Insufficient buffer memory on %s\n",
                         dev_name);
                exit(EXIT_FAILURE);
        }

        buffers = (buffer*)calloc(req.count, sizeof(*buffers));

        if (!buffers) {
                fprintf(stderr, "Out of memory\n");
                exit(EXIT_FAILURE);
        }

        for (n_buffers = 0; n_buffers < req.count; ++n_buffers) {
                struct v4l2_buffer buf;

                CLEAR(buf);

                buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                buf.memory      = V4L2_MEMORY_MMAP;
                buf.index       = n_buffers;

                if (-1 == xioctl(fd, VIDIOC_QUERYBUF, &buf))
                        errno_exit("VIDIOC_QUERYBUF");

                buffers[n_buffers].length = buf.length;
                buffers[n_buffers].start =
                        mmap(NULL /* start anywhere */,
                              buf.length,
                              PROT_READ | PROT_WRITE /* required */,
                              MAP_SHARED /* recommended */,
                              fd, buf.m.offset);

                if (MAP_FAILED == buffers[n_buffers].start)
                        errno_exit("mmap");
        }
}

static void init_device(void)
{
        struct v4l2_capability cap;
        struct v4l2_cropcap cropcap;
        struct v4l2_crop crop;
        struct v4l2_format fmt;
        unsigned int min;

        if (-1 == xioctl(fd, VIDIOC_QUERYCAP, &cap)) {
                if (EINVAL == errno) {
                        fprintf(stderr, "%s is no V4L2 device\n",
                                 dev_name);
                        exit(EXIT_FAILURE);
                } else {
                        errno_exit("VIDIOC_QUERYCAP");
                }
        }

        if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
                fprintf(stderr, "%s is no video capture device\n",
                         dev_name);
                exit(EXIT_FAILURE);
        }

        switch (io) {
	  /* case IO_METHOD_READ:
                if (!(cap.capabilities & V4L2_CAP_READWRITE)) {
                        fprintf(stderr, "%s does not support read i/o\n",
                                 dev_name);
                        exit(EXIT_FAILURE);
                }
                break;
	  */
        case IO_METHOD_MMAP:
	  //case IO_METHOD_USERPTR:
                if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
                        fprintf(stderr, "%s does not support streaming i/o\n",
                                 dev_name);
                        exit(EXIT_FAILURE);
                }
                break;
        }


        /* Select video input, video standard and tune here. */


        CLEAR(cropcap);

        cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

        if (0 == xioctl(fd, VIDIOC_CROPCAP, &cropcap)) {
                crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                crop.c = cropcap.defrect; /* reset to default */

                if (-1 == xioctl(fd, VIDIOC_S_CROP, &crop)) {
                        switch (errno) {
                        case EINVAL:
                                /* Cropping not supported. */
                                break;
                        default:
                                /* Errors ignored. */
                                break;
                        }
                }
        } else {
                /* Errors ignored. */
        }
       
	//capture exposures set reliably at 30fps.
        controlExposureExtended(fd);
        
        struct v4l2_streamparm prm;
        
        CLEAR(prm);
        prm.type =  V4L2_BUF_TYPE_VIDEO_CAPTURE;
        prm.parm.capture.capability=V4L2_CAP_TIMEPERFRAME;
        prm.parm.capture.capturemode=0;
        prm.parm.capture.timeperframe.numerator=1;
        prm.parm.capture.timeperframe.denominator=30;
        prm.parm.capture.extendedmode=0;
        prm.parm.capture.readbuffers=10;
        
        if (-1==xioctl(fd,VIDIOC_S_PARM,&prm)) {
          printf("Failed to set parameters\n");
        } else {
          printf("Set parameters\n");
        }

        CLEAR(fmt);

        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (true) {
                fmt.fmt.pix.width       = 160;
                fmt.fmt.pix.height      = 120;
                fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
                fmt.fmt.pix.field       = V4L2_FIELD_NONE;

                if (-1 == xioctl(fd, VIDIOC_S_FMT, &fmt))
                        errno_exit("VIDIOC_S_FMT");

                /* Note VIDIOC_S_FMT may change width and height. */
        } else {
                /* Preserve original settings as set by v4l2-ctl for example */
                if (-1 == xioctl(fd, VIDIOC_G_FMT, &fmt))
                        errno_exit("VIDIOC_G_FMT");
        }

        /* Buggy driver paranoia. */
        min = fmt.fmt.pix.width * 2;
        if (fmt.fmt.pix.bytesperline < min)
                fmt.fmt.pix.bytesperline = min;
        min = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;
        if (fmt.fmt.pix.sizeimage < min)
                fmt.fmt.pix.sizeimage = min;

        switch (io) {
	  //case IO_METHOD_READ:
	  //    init_read(fmt.fmt.pix.sizeimage);
	  //    break;

        case IO_METHOD_MMAP:
                init_mmap();
                break;

		//case IO_METHOD_USERPTR:
	  //init_userp(fmt.fmt.pix.sizeimage);
                //break;
        }
}

static void close_device(void)
{
        if (-1 == close(fd))
                errno_exit("close");

        fd = -1;
}

static void open_device(void)
{
        struct stat st;

        if (-1 == stat(dev_name, &st)) {
                fprintf(stderr, "Cannot identify '%s': %d, %s\n",
                         dev_name, errno, strerror(errno));
                exit(EXIT_FAILURE);
        }

        if (!S_ISCHR(st.st_mode)) {
                fprintf(stderr, "%s is no device\n", dev_name);
                exit(EXIT_FAILURE);
        }

        fd = open(dev_name, O_RDWR /* required */ | O_NONBLOCK, 0);

        if (-1 == fd) {
                fprintf(stderr, "Cannot open '%s': %d, %s\n",
                         dev_name, errno, strerror(errno));
                exit(EXIT_FAILURE);
        }
}


//int main(int argc, char **argv)
//{
  //startup
void init_video() {
  char name[]="/dev/video0";
  dev_name = name;

	
        open_device();
	describe_capturing();
	printf("Init Video\n");
        init_device();
	printf("Starting capture\n");
        start_capturing();
	printf("Started capture\n");
}
	//redo
//        mainloop();
	//shutdown
void stop_video() {
	printf("Stopping capture\n");
        stop_capturing();
        uninit_device();
        close_device();
        fprintf(stderr, "\n");
}

/*int main(){
  init_video();
  onesteploop();
  stop_video();
}
*/
