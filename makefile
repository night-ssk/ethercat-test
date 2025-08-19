
INCLUDE += -I /home/ssk/ethercat/_install/include
INCLUDE += -I/home/ssk/t507-env/LinuxSDK/out/t507/tlt507-evm/longan/buildroot/target/include/cjson
INCLUDE += -I ./ethercat 
INCLUDE += -I ./term 
INCLUDE += -I ./dcsync 
CFLAGS  += -g -Wall -O2 $(DEFINES) $(INCLUDE)
LIBS    += -lethercat -lmosquitto -lpthread -lcjson -lm
LDFLAGS := -L/home/ssk/ethercat/_install/lib -L/home/ssk/t507-env/LinuxSDK/out/t507/tlt507-evm/longan/buildroot/target/lib
CXXFLAGS:= $(CFLAGS)
SOURCE  := $(wildcard *.c) $(wildcard term/*.c) $(wildcard ethercat/*.c) $(wildcard dcsync/*.c) 
OBJS    := $(patsubst %.c,%.o,$(patsubst %.cpp,%.o,$(SOURCE))) 
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
