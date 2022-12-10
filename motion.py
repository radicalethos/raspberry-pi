import RPi.GPIO as GPIO #Import GPIO library
import subprocess
import threading

import datetime
import logging
import time
import signal
import glob
import sys
import os
import re

from gpiozero import Servo
from threading import Thread
from datetime import datetime

#sudo hwclock -r
#hwclock: ioctl(RTC_RD_TIME) to /dev/rtc0 to read the time failed: Connection timed out

#exit(0)

#raspivid -v -vf -hf -fps 24 -a 12 --qp 25 -t 10000 -o -\
# | ffmpeg -hide_banner -loglevel error -y -r 24 -i -\
# -vcodec copy -map 0 -segment_time 00:00:05 -f segment output%03d.mp4

class RaspiVid(Thread):

	m_buz = 16
	m_root = "./"
	m_cv = threading.Condition();

	def __init__(self, root_dir):
		print("Starting picam...")
		Thread.__init__(self)
		self.m_root = root_dir
		GPIO.setwarnings(False)
		GPIO.setmode(GPIO.BCM) #Set pin as GPIO in.
                #GPIO.setup(self.m_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
		GPIO.setup(self.m_buz, GPIO.OUT)
		signal.signal(signal.SIGTERM, self.sigterm_handler)
		logging.basicConfig(filename=os.path.expanduser("~/motion.log"),
			format='%(asctime)s %(message)s', filemode='w')
		self.m_log = logging.getLogger()
		self.m_log.setLevel(logging.DEBUG)
		self.log_debug("__init__")
		os.environ["LD_LIBRARY_PATH"] = "/usr/local/lib/arm-linux-gnueabihf/:/usr/local/lib/"

	def picam(self):

		self.free_space_auto()
		self.clean(self.m_root)

		#folder = os.path.join(self.m_root, datetime.now().strftime("%02Y-%02m-%02d"))
		self.log_debug(self.m_root)
		
		#if not os.path.exists(folder):
		#	os.mkdir(folder)

		if not os.path.exists(self.m_root):
			raise RuntimeError("video dir not found")

		if not os.path.exists("/dev/rtc0"):
			self.log_debug("module /dev/rtc0 not found")

		#found = False
		#cmd = "dmesg"
		#self.m_proc = subprocess.Popen(cmd.split(), shell=False,  stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
		#for line in iter(self.m_proc.stdout.readline, b''):
		#	uline = line.decode("utf-8")
		#	if uline.find("setting system clock to") != -1:
		#		found = True
			#	raise RuntimeError("rtc read error: RTC_RD_TIME")
		#if not found:
		#	self.beep(0.2, 5)

		#cmd = "raspivid -md 2 -o /dev/null -t 0"
		
		# /opt/vc/bin/raspivid -v -m4v -3d tb -w 1280 -h 1280 -t 20000 -fps 15 -o /mnt/usb/motion/2022-11-02/zzzz.mov

		# with manual awbg 
		cmd = "/opt/vc/bin/raspivid -v -mo 2 -md 2 -br {brt} -awb {agc} -ex {exp} -fps {fps} -a 12 -a 1024 -ae 32,0xfff,0x808000 --qp 25 -t 0 -sg 3600000 -m4v -m4a hw:1,0 -o {dir}/%02Y-%02m-%02d/%H-%M-%S.mov".format(brt=self.brightness(), agc=self.gaincontrol(), exp=self.exposure(), fps=self.framerate(),  dir=self.m_root)
		
		# with auto awb w=1280, h=1440
		# 1920 x 768
		cmd = "/opt/vc/bin/raspivid -v -md 2 -br {brt} -fps {fps} -a 12 -a 1024 -ae 32,0xfff,0x808000 --qp 25 -t 0 -sg 3600000 -m4v -m4a hw:1,0 -o {dir}/%02Y-%02m-%02d/%H-%M-%S.mov".format(brt=self.brightness(), fps=self.framerate(),  dir=self.m_root)

		cmd = "/opt/vc/bin/raspivid -v -mo 2 -w 1408 -h 1440 -3d tb -br {brt} -fps {fps} -a 12 -a 1024 -ae 32,0xfff,0x808000 --qp 25 -t 0 -sg 3600000 -m4v -m4a hw:1,0 -o {dir}/%02Y-%02m-%02d/%H-%M-%S.mov".format(brt=self.brightness(), fps=self.framerate(),  dir=self.m_root)


		self.log_debug(cmd)
		self.m_proc = subprocess.Popen(cmd.split(), shell=False,  stdout=subprocess.PIPE, stderr=subprocess.STDOUT)


		for line in iter(self.m_proc.stdout.readline, b''):
			uline = line.decode("utf-8")

			self.m_log.debug(uline.rstrip())
			if uline.find("SIGUSR2") != -1:
				self.log_debug("SIGUSR2 received")
				self.beep(0.2, 1)

			elif uline.find("SIGUSR1") != -1:
				if uline.find("pause") != -1:
					self.m_cv.acquire()
					self.m_cv.notifyAll()
					self.m_cv.release()

				elif uline.find("capture") != -1:
					self.m_cv.acquire()
					self.m_cv.notifyAll()
					self.m_cv.release()

			#elif re.match("[^\]]+[\]] Opening ['][^']+['] for writing", line):
			#	self.free_space_up_to(1024*1024*1024*5, self.m_root)
			#	self.log_debug("ffmpeg open outfile")
			#	self.beep(0.1, 1)

			elif uline.find("OBJECT_ENTER") != -1:
				print(uline.rstrip())
				self.beep(0.1, 0)

			elif uline.find("Opening output file") != -1:
				print (">>>", line.rstrip())
				self.free_space_up_to(1024*1024*1024*5, self.m_root)
				self.log_debug("Opening output file")
				self.beep(0.1, 1)

			elif uline.find("mmal: main: Error opening output file:") != -1:
				self.log_debug("mmal Error opening output file")
				self.beep(0.5, 2)

			elif uline.find("No data received from sensor") != -1:
				raise RuntimeError("No data received from sensor")

			elif uline.find("Failed to write buffer data") != -1:
				raise NotImplementedError("Failed to write buffer data")

			elif uline.find("mmal: Camera is not detected") != -1:
				self.log_debug("mmal: Camera is not detected")
				self.beep(0.5, 5)

			else:
				print(uline.rstrip())

	def getDayLight(self):
		hour = (int) (datetime.now().strftime("%02H")) 
		minute = (int) (datetime.now().strftime("%02M"))
		timeint = hour*100 + minute

		if timeint >= 1830:
			return 2
		elif timeint >= 1730:
			return 1
		elif timeint < 600:
			return 2
		else:
			return 0

	def brightness(self):
		if self.getDayLight() == 2:
			return 55
		elif self.getDayLight() == 1:
			return 52
		else:
			return 50

	def gaincontrol(self):
		if self.getDayLight() == 2:
			return "on"
		elif self.getDayLight() == 1:
			return "on"
		else:
			return "off -awbg 1.2,1.0"

	def exposure(self):
		if self.getDayLight() == 2:
			return "night"
		elif self.getDayLight() == 1:
			return "auto"
		else:
			return "auto"

	def framerate(self):
		if self.getDayLight() == 2:
			return 10
		elif self.getDayLight() == 1:
			return 10
		else:
			return 15

	def files_to_delete(self, rootfolder):
		return sorted(
			(os.path.join(dirname, filename)
			for dirname, dirnames, filenames in os.walk(rootfolder)
			for filename in filenames),
			key=lambda fn: -os.stat(fn).st_mtime)

	def free_space_up_to(self, free_bytes_required, rootfolder):
		file_list = self.files_to_delete(rootfolder)

		while file_list:
			statv=os.statvfs(rootfolder)
			if statv.f_bavail*statv.f_bsize >= free_bytes_required:
				break
			os.remove(file_list.pop())

	def free_space_auto(self):
		self.log_debug("free_space_auto...")

		statv=os.statvfs(self.m_root)
		if statv.f_bavail*statv.f_bsize < 1024*1024*1024:
			self.free_space_up_to(1024*1024*1024*5, self.m_root)
			return True

		return False

	def sigterm_handler(self, *args):
        	self.log_debug("sigterm received")
        	self.beep(0.05, 2)
        	self.stop()

	def stop(self):
		self.log_debug("stopping")
		self.m_proc.send_signal(signal.SIGINT)
		self.m_proc.wait()
	
	def clean(self, path):
		self.log_debug("Clean empty dirs...")	

		for f in os.listdir(path):
			filename = os.path.join(path,f)
			if os.path.isfile(filename):
				pass
#				thistime = datetime.now()
#				basetime = datetime.fromtimestamp(0)
#				filetime = datetime.fromtimestamp(os.stat(filename).st_mtime)
#				diff1 = thistime.date() - basetime.date()
#				diff2 = filetime.date() - basetime.date()

				#print diff1.days, diff2.days, filename
#				if diff1.days - diff2.days > 1:
#					print "deleting... ", filename
#					os.remove(filename)
			elif os.listdir(filename):
				self.clean(filename)
			else:
				print("deleting ", filename)
				self.m_log.info(filename)
				os.rmdir(filename)

	def  beep(self, dur, num):
		for i in range(0, num):
			GPIO.output(16, 1)
			time.sleep(dur)
			GPIO.output(16, 0)
			time.sleep(0.05)

	def log_debug(self, msg):
		self.m_log.debug(msg)

	def crontab(self, shutdown, wake_delay):
		self.log_debug("crontab entries")

		os.system("crontab -r")
		cmd = "echo \"{d} * * * echo 0 | sudo tee /sys/class/rtc/rtc0/wakealarm\" | crontab -".format(d=shutdown, w=wake_delay)
		os.system(cmd)
		cmd = "echo \"{d} * * * echo +{w} | sudo tee /sys/class/rtc/rtc0/wakealarm\" | crontab -".format(d=shutdown, w=wake_delay*60)
		os.system(cmd)
		cmd = "crontab -l | { cat; echo \""+shutdown+" * * * sudo poweroff\"; }  | crontab -"
		os.system(cmd)

	def run(self):
		proc = subprocess.Popen(["dmesg", "-wH"], stdout=subprocess.PIPE, stderr=subprocess.STDOUT)

		for line in iter(proc.stdout.readline, b''):
			uline = line.decode("utf-8")
			if uline.find("Firmware transaction timeout") != -1:
				self.m_log.debug("Firmware transaction timeout")
				self.beep(0.5, 10)


th = RaspiVid("/mnt/usb/motion")
#th.crontab("0 23", 8*60)
#exit(0)


try:
	#th.start()
	th.picam()
except OSError as err:
	th.log_debug("OSError: {0}".format(err))
	th.beep(1, 2)

except NotImplementedError as err:
	th.log_debug("NotImplementedError: {0}".format(err))
	th.beep(0.5, 3)
	os.system("sudo reboot")

except RuntimeError as err:
	th.log_debug("RuntimeError: {0}".format(err))
	th.beep(0.5, 2)

except KeyboardInterrupt:
	print("KeyboardInterrupt")
	th.stop()

