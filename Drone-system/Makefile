CC=g++ #Compiler
CFLAGS= -c -std=c++11 -fPIE -lrt -pthread #-g #Compiler Flags #-Ofast
DEFINES=-DENABLE_DELETE
INCPATH=

LDFLAGS=-lwiringPi #Linker options

SOURCES= main.cpp  $(MOCFILES) #cpp files

OBJECTS=$(SOURCES:.cpp=.o)  #Object files
EXECUTEABLE=Drone_system #Output name
all: $(HEADERS) $(SOURCES) $(EXECUTEABLE)

$(EXECUTEABLE): $(OBJECTS)
	$(CC)    $(OBJECTS) -o $(EXECUTEABLE) $(LDFLAGS)


.cpp.o:
	$(CC)  $(CFLAGS) $(INCPATH) $(DEFINES)   $< -o $@


clean:  ; rm *.o $(EXECUTEABLE) $(MOCFILES) $(HEADERS)


moc_%.cpp: %.h
		moc $(DEFINES) $(INCPATH) $< -o $@

ui_%.h: %.ui
		uic $< -o $@
