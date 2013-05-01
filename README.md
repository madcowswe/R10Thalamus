The way our code works is this: we produce a functions library, which abstracts all the register access stuff with easy-to-use functions, this is contained within thal.c.  Also included in this library are stuff like USB functions, Thalamus-specific functions incuding PWM output, and specific receiver functions, and a bunch of other stuff I can't remember.   We also include the startup code, and linker and Makefile designed for use with an ARM cross-compiler, we've been using YAGARTO, but we hear certain configurations of GCC also work.  (Instructions for setting stuff up are on our old website: http://universalair.co.uk/control/forebrain the procedure for Thalamus is the same as for Forebrain)

All of this stuff is in the ./build/ folder, and some configuration stuff is available in ./config.h which includes important stuff such as interrupt priorities and some of the configs for the peripherals .  The application code is in main.c.

So in summary:
- application code is in main.c,
- hardware-related functions are in thal.c (and also defined inline in thal.h), 
- all the defines are either in thal.h or config.h

 I hope you don't have too much trouble understanding what's going on there, it's not extensively commented.  If you have any questions, feel free to ask.

With best regards,
Yuan

