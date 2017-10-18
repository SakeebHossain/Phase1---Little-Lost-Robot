##############################################################################
#
# Simple makefile to compile CSC D18 programs on the machines of the CS
# labs. You may have to change options and/or add paths to libraries and
# header files if compiling on your own computer.
#
##############################################################################

# Define C compiler. On Linux this is typically gcc or g++,
# note that gcc is simply a wrapper for the GNU compiler collection,
# which selects an appropriate compiler for the input file.
# For an ANSI C program, gcc will call cc. For a C++ file it will
# call g++
CC            = gcc

# Define C++ compiler
CCC	          = /usr/bin/g++

# Define C compiler options
CFLAGS        = -c -g -O4

# Define C++ compiler options
CCCFLAGS      = -c -g -O4

# Define OpenGL and GLU library names - If the linker complains about not being
# able to find libraries, check where they are installed in your system, and
# add the appripriate -I and -L switches with the paths to include and lib 
# directories where OpenGL stuff can be found.
GL_LIBS       = -lGLU -lGL -lglut

# Define location of X Windows libraries,  and the X11 library names
#XLIBS         = -L/usr/X11R6/lib -L/usr/lib -lX11 
XLIBS         = -lX11 

# Define the location of the destination directory for the executable file
DEST	      = .

# Define flags that should be passed to the linker
LDFLAGS	      =

# Define libraries to be linked with
LIBS	      = Lander_Control.o $(GL_LIBS) $(GLUT_LIBS) -lm

# Define linker
LINKER	      = g++

# Define all object files to be the same as CPPSRCS but with all the .cpp and .c suffixes replaced with .o
OBJ           = $(CPPSRCS:.cpp=.o) $(CSRCS:.c=.o)

# Define name of target executable
PROGRAM	          = Lander_Control

# Define all C source files here
CSRCS         =

# Define all C++ source files here
CPPSRCS       = Lander.cpp

##############################################################################
# Define additional rules that make should know about in order to compile our
# files.                                        
##############################################################################

# Define default rule if Make is run without arguments
all : $(PROGRAM)

# Define rule for compiling all C++ files
%.o : %.cpp
	$(CCC) $(CCCFLAGS) $(CPPFLAGS) $*.cpp

# Define rule for compiling all C files
%.o : %.c
	$(CC) $(CFLAGS) $(CPPFLAGS) $*.c

# Define rule for creating executable
$(PROGRAM) :	$(OBJ)
		@echo -n "Loading $(PROGRAM) ... "
		$(LINKER) $(LDFLAGS) $(OBJ) $(LIBS) -o $(PROGRAM)
		@echo "done"

# Define rule to clean up directory by removing all object, temp and core
# files along with the executable
clean :
	@rm -f $(OBJ) *~ core $(PROGRAM)

