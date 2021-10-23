import serial
import time
from serial.serialutil import EIGHTBITS, PARITY_NONE, STOPBITS_ONE

SO = "Lnx"
#SO = "Win"


class SerialPort:

    # Constructor of Serial port class
    def __init__(self): 
        #Windows default port
        self.COMport   = 'COM4'
        #Linux default port
        self.ttyUSBport= '/dev/ttyUSB0'

        self.seqNo     = 0
        self.parity    = PARITY_NONE
        self.baudrate  = 115200
        self.timeout   = 1
    # Advance SeqNum for serial transfer enumeration [ Private method ]
    def __AdvanceSeqNum(self,seqNo=0):
        self.seqNo = seqNo

        self.seqNo = self.seqNo + 1

        return self.seqNo
    # Open serial port over windows SO   
    def OpenSerialWin(self,port,baudrate,timeout,parity,stopbits,bytesize):
        self.COMport = port
        self.Bdrate  = baudrate
        self.timeout = timeout
        self.parity  = parity
        self.stopbits= stopbits
        self.bytesize= bytesize

        port_obj = serial.Serial(self.COMport,
                                 self.baudrate,
                                 bytesize=      self.bytesize,
                                 parity=        self.parity,
                                 stopbits=      self.stopbits,
                                 timeout=       self.timeout,
                                 xonxoff=       False,
                                 rtscts=        False,
                                 write_timeout= None,
                                 dsrdtr=        False,
                                 inter_byte_timeout=None,
                                 #exclusive=         None
                                 )
        return port_obj
    # Open serial port over linux SO
    def OpenSerialLnx(self,port,baudrate,timeout,parity,stopbits,bytesize):
        self.COMport = port
        self.Bdrate  = baudrate
        self.timeout = timeout
        self.parity  = parity
        self.stopbits= stopbits
        self.bytesize= bytesize
        
        port_obj = serial.Serial(self.ttyUSBport,
                                 self.baudrate,
                                 bytesize=      self.bytesize,
                                 parity  =      self.parity,
                                 stopbits=      self.stopbits,
                                 timeout =      self.timeout,
                                 xonxoff =      False,
                                 rtscts  =      False,
                                 write_timeout= None,
                                 dsrdtr=        False,
                                 inter_byte_timeout=None,
                                 exclusive=None
                                )
        return port_obj
    # Read bytes over serial port
    def ReadBytes_OverSerial(self,port):
        
        self.port    = port

        if self.port.is_open == True:
            BytesRead = self.port.read()
            return BytesRead
        else:
            print("Error while reading serial port")
    # Write bytes over serial port
    def WriteBytes_OverSerial(self,port,frame):
        self.port = port
        self.frame = frame
        port.write(bytes(frame))
        #self.port(serial.to_bytes(frame))


    def SendStartManufactureMode(self,port):

        #Internals
        self.port       = port

        self.STX        = 0x02  #   STX

        self.DstId      = 0x03  #   DstID
        self.CmdId      = 0x21  #   CmdID
        self.seqNo      = self.__AdvanceSeqNum(self.seqNo)  # Increment seq number.

        #   No payload for StartMsg
        self.payload    = 0x0   #   Payload 

        self.headerSize = 0x03  #   CmdId + DstId+ SeqNo

        self.MSGLen     = self.payload + self.headerSize  # Define length of header and payload.

        self.ETX        = 0x03  #   ETX

        #build frame
        frame = [self.STX,self.MSGLen,self.seqNo,self.DstId,self.CmdId,self.payload,self.ETX]


        print("STX,Ln,Sq,Dst,Cmd,PLd,ETX")
        print(frame)

        return frame




class Main:
    def __init__(self):
        pass

    def main():

        SerialObj = SerialPort()    #Create serial port object
        
        if (SO == 'Win'):   #   Test if SO is windows.
            port_obj = SerialObj.OpenSerialWin( port     = 'COM4',             
                                                baudrate = 115200,
                                                timeout  = 1,
                                                parity   = PARITY_NONE,
                                                stopbits = serial.STOPBITS_ONE,
                                                bytesize = serial.EIGHTBITS,
                                            )  
        else:               #   Write over Linux port
            port_obj = SerialObj.OpenSerialLnx( port     = '/dev/ttyUSB0',             
                                                baudrate = 115200,
                                                timeout  = 1,
                                                parity   = PARITY_NONE,
                                                stopbits = serial.STOPBITS_ONE,
                                                bytesize = serial.EIGHTBITS,
                                               )

        while (True):
            try:
                if port_obj.isOpen():
                    StartMfFrame=SerialObj.SendStartManufactureMode(port_obj)
                    SerialObj.WriteBytes_OverSerial(port_obj,StartMfFrame)
                else:
                    print("Couldn't write over serial, check if the port is closed")
            
            except Exception as Ex: 
                print(Ex)
            finally:
                time.sleep(1)

Main.main()
