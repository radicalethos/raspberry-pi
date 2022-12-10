# raspberry-pi
My home security project using compute module 4, it can do mp4 container conversion internally from raspivid without using any other program.
Audio tracks can also be added to mp4 container if you have a usb mic

### ffmpeg mp4 conversion instead of .h264 output
raspivid -v -o movie.mov -m4v

### ffmpeg along with usb mic 
raspivid -v -o movie.mov -m4v -m4a hw:1,0
here hw:1,0 is the device name (usb mic), do the arecord -l to find the list of attached audio capture devices on rpi

### compute module (stereo)
raspivid -v -o movie_tb.h264 -3d tb -w 1408 -h 1440
raspivid -v -o movie_sbs.h264 -3d sbs -w 1920 -h 768
