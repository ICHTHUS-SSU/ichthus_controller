GG=g++
CC=c++
CFLAGS=-g -I. -fpermissive -std=c++11 -w -Wwrite-strings -lpthread
XML_FLAGS = -I/usr/include/libxml2 -lxml2
KBHIT_FLAGS = -lX11
ECAT_FLAGS = -I/opt/etherlab/include  /opt/etherlab/lib/libethercat.a
PROTO_FLAGS= `pkg-config --cflags --libs protobuf`
RPI_FLAGS = -lwiringPi
PCAN_FLAGS = -lpcanbasic
LDFLAGS_BOOST=-L/usr/local/lib -lboost_system -lboost_thread
DEBUG=0

pc :: VCFserver_pc.exe vcsmd.exe
rpi :: VCFserver_rpi.exe vcsmd.exe


vcsmd.exe : vcsmd.o
	$(GG) -o $@ $^ $(CFLAGS) 
VCFserver_pc.exe :  Controller_HA.o Ecat_HA.o ecat.o EthercatCyclic.o EthercatManager.o GlobalParam.o hybridautomata.o Log.o Main.o Motion_HA.o Obd_HA.o obd.o ObdManager.o regex.o xml.o replace.o packetparser.o
	$(GG) -o $@ $^ $(KBHIT_FLAGS) $(CFLAGS) $(XML_FLAGS) $(LDFLAGS) $(PROTO_FLAGS) $(ECAT_FLAGS) 
VCFserver_rpi.exe : Controller_HA.o Ecat_HA.o ecat.o EthercatCyclic.o EthercatManager.o GlobalParam.o hybridautomata.o Log.o Main.o Motion_HA.o Obd_HA.o obd.o ObdManager.o regex.o xml.o Can_HA.o CanManager.o can_decode.o packetparser.o hvi.o Hvi_HA.o HviManager.o
	$(GG) -o $@ $^ $(KBHIT_FLAGS) $(CFLAGS) $(XML_FLAGS) $(LDFLAGS) $(PROTO_FLAGS) $(ECAT_FLAGS) $(PCAN_FLAGS) $(RPI_FLAGS) 


%.o: %.c $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS) $(ECAT_FLAGS) 

%.o: %.cc $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

%.o: %.cpp $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS) $(XML_FLAGS)

clean: 
	rm -f *.exe *.o *~ *.stackdump *_middleman .*.swo .*.swp
