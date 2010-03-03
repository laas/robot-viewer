=== Dependencies ===

 * SimpleParse 2.1 
   http://simpleparse.sourceforge.net/

 * pyglet 1.1.4
   http://www.pyglet.org/

 * omniORB 4.1.4 (optional)
   http://omniORB.sourceforge.net

=== INSTALLATION ===

 * Download the tar balls and follow instructions. In general, 'sudo python setup.py install' will install the packages to the right path
 
 * MacOS X:
      On Snow Leopard, python is set to 64 bit by default, that can cause pyglet to crash, 
      if that happen on your system, try:
         defaults write com.apple.versioner.python Prefer-32-Bit -bool yes   

=== USAGE ===
 * Stand-alone mode
 * Server-client mode (requires omniORB (corbaserver))
