

Version History
---------------
0.6 	-Added ability to easily select if this driver should use timer0 or
	  timer1.
	-Timer1 is now the default timer used. Used to be timer0.
	-Made the timer init happen later to allow this driver to work when
	  used with an attiny arduino library.
	-Fixed bug where a 512 microsecond pulse would not be generated.
	
0.5 	-Initial public release




Short Guide On How To Compile
------------------------------
1) Make sure you have WinAvr installed (http://winavr.sourceforge.net/)
2) open a CMD window and navigate to the Servo8Bit folder
3) type in "make" in the CMD window and the code will be compiled
4) type in "make program" to program your Attiny. This assumes you are using a
   usb programmer (such as the AVRISP mkII).