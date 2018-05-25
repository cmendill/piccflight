//UVC Camera Software
//Edited by G.Howe Spring 2018

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <libusb.h>
#include <libuvc.h>


/* This callback function runs once per frame. Use it to perform any
 * quick processing you need, or have it put the frame into your application's
 * input queue. If this function takes too long, you'll start losing frames. */
void cb(uvc_frame_t *frame, void *ptr) {
  printf("Got frame: %d\n",frame->sequence);
}



int main(int argc, char **argv) {
  uvc_context_t *ctx;
  uvc_device_t *dev;
  uvc_device_handle_t *devh;
  uvc_stream_ctrl_t ctrl;
  uvc_error_t res;
  int imDimx = 1280;
  int imDimy = 960;
  int fps = 5;

  /* Initialize a UVC service context. Libuvc will set up its own libusb
   * context. Replace NULL with a libusb_context pointer to run libuvc
   * from an existing libusb context. */
  res = uvc_init(&ctx, NULL);

  if (res < 0) {
    uvc_perror(res, "uvc_init");
    return res;
  }

  printf("UVC initialized");

  /* Locates the first attached UVC device, stores in dev */
  res = uvc_find_device(
      ctx, &dev,
      0, 0, NULL); /* filter devices: vendor_id, product_id, "serial_num" */

  if (res < 0) {
    uvc_perror(res, "uvc_find_device"); /* no devices found */
  } else {
    printf("Device found");

    /* Try to open the device: requires exclusive access */
    res = uvc_open(dev, &devh);
    
    if (res < 0) {
      uvc_perror(res, "uvc_open"); /* unable to open device */
    } else {
      printf("Device opened");

      /* Print out a message containing all the information that libuvc
       * knows about the device */
      uvc_print_diag(devh, stderr);

      /* Setup stream profile */
      res = uvc_get_stream_ctrl_format_size(
					    devh, &ctrl, /* result stored in ctrl */
					    UVC_FRAME_FORMAT_ANY, /* YUV 422, aka YUV 4:2:2. try _COMPRESSED */
					    imDimx, imDimy, fps /* width, height, fps */
					    );

      /* Print out the result */
      uvc_print_stream_ctrl(&ctrl, stderr);

      if (res < 0) {
        uvc_perror(res, "get_mode"); /* device doesn't provide a matching stream */

      } else {
        //Start the video stream. The library will call user function cb:
	res = uvc_start_streaming(devh, &ctrl, cb, NULL, 0);
        if (res < 0) {
          uvc_perror(res, "start_streaming"); /* unable to start stream */
        } else {
          printf("Streaming...");
	  
          uvc_set_ae_mode(devh, 1); /* e.g., turn on auto exposure */

          sleep(5); /* stream for 5 seconds */

          /* End the stream. Blocks until last callback is serviced */
          uvc_stop_streaming(devh);
          printf("Done streaming.");
        }
      }

      /* Release our handle on the device */
      uvc_close(devh);
      printf("Device closed");
    }

    /* Release the device descriptor */
    uvc_unref_device(dev);
  }

  /* Close the UVC context. This closes and cleans up any existing device handles,
   * and it closes the libusb context if one was not provided. */
  uvc_exit(ctx);
  printf("UVC exited");

  return 0;
}
