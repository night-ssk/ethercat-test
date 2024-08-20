
INCLUDE += -I /home/ssk/igh/_install/include
INCLUDE += -I ./ethercat 
INCLUDE += -I ./term 
CFLAGS  += -g -Wall -O2 $(DEFINES) $(INCLUDE)
LIBS    += -lethercat
LIBS    += -lpthread
LDFLAGS := -L /home/ssk/igh/_install/lib
CXXFLAGS:= $(CFLAGS)
SOURCE  := $(wildcard *.c) $(wildcard term/*.c)
OBJS    := $(patsubst %.c,%.o,$(patsubst %.cpp,%.o,$(SOURCE))) $(wildcard ethercat/*.c) 
TARGET  := igh_ethercat_dc_motor
CC=/home/ssk/t507-env/LinuxSDK/out/t507/tlt507-evm/longan/buildroot/host/usr/bin/aarch64-linux-gnu-gcc
.PHONY : everything objs clean distclean rebuild

all : $(TARGET)

objs : $(OBJS)

rebuild: distclean all

clean :
	rm -rf *~
	rm -rf *.o ethercat/*.o  term/*.o  


distclean : clean
	rm -rf $(TARGET)

$(TARGET) : $(OBJS)
	$(CC) $(CXXFLAGS) -g -o $@ $(OBJS) $(LDFLAGS) $(LIBS)
