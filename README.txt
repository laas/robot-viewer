=== Dependencies ===
 * Python 2.6

 * SimpleParse 2.1 
   http://simpleparse.sourceforge.net/
 
 * pyglet 1.1.4
   http://www.pyglet.org/

 * simplui 1.0.4 (an extension of pyglet)
   http://simplui.googlecode.com

 * omniORB 4.1.4 (optional)
   http://omniORB.sourceforge.net

=== INSTALLATION ===

 * Only simpleparse,omniORB requires installation, go to respective folder(s) and do "sudo python setup.py install"
 
 * MacOS X:
      On Snow Leopard, python is set to 64 bit by default, that can cause pyglet to crash, 
      if that happens on your system, try:
         defaults write com.apple.versioner.python Prefer-32-Bit -bool yes   

=== USAGE ===
 * Stand-alone mode
 * Server-client mode (requires omniORB (corbaserver))
