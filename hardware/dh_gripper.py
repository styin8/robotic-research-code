import serial
from time import sleep

class DH_Device:
    def __init__(self, port, baudrate,bytesize=8, parity='N',stopbits=1, set_output_flow_control='N',set_input_flow_control='N') -> None:
        self.serialPort = serial.Serial()
        self.port = port
        self.baudrate = baudrate
        self.bytesize = bytesize
        self.parity = parity
        self.stopbits = stopbits
        self.set_output_flow_control = set_output_flow_control
        self.set_input_flow_control = set_input_flow_control

    
    def connect(self) -> int:
        ret = -1
        self.serialPort.port= self.port
        self.serialPort.baudrate = self.baudrate
        self.serialPort.bytesize = self.bytesize
        self.serialPort.parity=self.parity
        self.serialPort.stopbits = self.stopbits
        self.serialPort.set_output_flow_control = self.set_output_flow_control
        self.serialPort.set_input_flow_control = self.set_input_flow_control

        self.serialPort.open()
        ret = -1
        if self.serialPort.isOpen():
            ret = 0
            print('Serial Open Success')
        else:
            ret = -1
            print('Serial Open Error')
        return ret
    
    def disconnect(self) -> int:
        ret = -1
        self.serialPort.close()
        if self.serialPort.isOpen():
            ret = -1
            print('Serial Close Error')
        else:
            ret = 0
            print('Serial Close Success')
        return ret
    
    def device_wrire(self, write_data) :
        write_lenght = 0
        if self.serialPort.isOpen() :
            write_lenght = self.serialPort.write(write_data)
            if(write_lenght == len(write_data)) :
                return write_lenght
            else :
                print('write error ! send_buff :',write_data)
                return 0
        else :
            return -1

    def device_read(self, wlen) :
        responseData = [0,0,0,0,0,0,0,0]
        if self.serialPort.isOpen() :
            responseData = self.serialPort.readline(wlen)
            #print('read_buff: ',responseData.hex())
            return responseData
        else :
            return -1
        
class DH_Gripper:
    def __init__(self, port, baudrate) -> None:
        self.Gripper_ID = 0x01
        self.port = port
        self.baudrate = baudrate

    def connect(self) -> int:
        self.device = DH_Device(self.port, self.baudrate)
        ret = self.device.connect()
        if ret < 0 :
            print('Gripper Connect failed!')
            return ret
        else :
            print('Gripper Connect successful!')
            return ret
        
    def disconnect(self) -> int:
        ret = self.device.disconnect()
        if ret < 0 :
            print('Gripper disconnect failed!')
            return ret
        else :
            print('Gripper disconnect successful!')
            return ret
    def CRC16(self,nData, wLength) :
        if nData==0x00:
            return 0x0000
        wCRCWord=0xFFFF
        poly=0xA001
        for num in range(wLength):
            date = nData[num]
            wCRCWord = (date & 0xFF)^ wCRCWord
            for _ in range(8) : 
                if(wCRCWord&0x01)!=0:
                    wCRCWord>>=1
                    wCRCWord^= poly
                else:
                    wCRCWord>>=1
        return wCRCWord
        
    def WriteRegisterFunc(self,index, value) :
        send_buf = [0,0,0,0,0,0,0,0]
        send_buf[0] = self.Gripper_ID
        send_buf[1] = 0x06
        send_buf[2] = (index >> 8) & 0xFF
        send_buf[3] = index & 0xFF
        send_buf[4] = (value >> 8) & 0xFF
        send_buf[5] = value & 0xFF

        crc = self.CRC16(send_buf,len(send_buf)-2)
        send_buf[6] = crc & 0xFF
        send_buf[7] = (crc >> 8) & 0xFF

        send_temp = send_buf
        ret = False
        retrycount = 3

        while ( ret == False ):
            ret = False

            if(retrycount < 0) :
                break
            retrycount = retrycount - 1

            wdlen = self.device.device_wrire(send_temp)
            if(len(send_temp) != wdlen) :
                print('write error ! write : ', send_temp)
                continue

            rev_buf = self.device.device_read(8)
            if(len(rev_buf) == wdlen) :
                ret = True
        return ret

    def ReadRegisterFunc(self,index) :
        send_buf = [0,0,0,0,0,0,0,0]
        send_buf[0] = self.Gripper_ID
        send_buf[1] = 0x03
        send_buf[2] = (index >> 8) & 0xFF
        send_buf[3] = index & 0xFF
        send_buf[4] = 0x00
        send_buf[5] = 0x01

        crc = self.CRC16(send_buf,len(send_buf)-2)
        send_buf[6] = crc & 0xFF
        send_buf[7] = (crc >> 8) & 0xFF

        send_temp = send_buf
        ret = False
        retrycount = 3

        while ( ret == False ):
            ret = False

            if(retrycount < 0) :
                break
            retrycount = retrycount - 1

            wdlen = self.device.device_wrire(send_temp)
            if(len(send_temp) != wdlen) :
                print('write error ! write : ', send_temp)
                continue

            rev_buf = self.device.device_read(7)
            if len(rev_buf) == 7 :
                value = ((rev_buf[4]&0xFF)|(rev_buf[3] << 8))
                ret = True
            #('read value : ', value)
        return value

    def Initialization(self) :
        try:
            self.WriteRegisterFunc(0x0100,0xA5)
            sleep(0.5)
            self.WriteRegisterFunc(0x0100,0x5A)
        except Exception as e:
            print(e)
            return
        
    def SetTargetPosition(self,refpos) :
        self.WriteRegisterFunc(0x0103,refpos)

    def SetTargetForce(self,force) :
        self.WriteRegisterFunc(0x0101,force)
        
    def SetTargetSpeed(self,speed) :
        self.WriteRegisterFunc(0x0104,speed)

    def GetCurrentPosition(self) :
        return self.ReadRegisterFunc(0x0202)

    def GetCurrentTargetForce(self) :
        return self.ReadRegisterFunc(0x0101)

    def GetCurrentTargetSpeed(self) :
        return self.ReadRegisterFunc(0x0104)

    def GetInitState(self) :
        return self.ReadRegisterFunc(0x0200)

    def GetGripState(self) :
        return self.ReadRegisterFunc(0x0201)

    """description of class"""


if __name__ == "__main__":
    """
    port: 串口号
    baudrate: 波特率
    initstate: 初始化状态
    g_state: 夹爪状态
    force: 夹爪力 (20-100)
    speed: 夹爪速度 (1-100)
    """
    port="com4"
    baudrate=115200
    initstate = 0
    g_state = 0
    force = 50
    speed = 20


    gripper = DH_Gripper(port,baudrate)

    gripper.connect()
    gripper.Initialization()
    print('Gripper init success!')
    while(initstate != 1) :
        initstate = gripper.GetInitState()
        sleep(0.2)
        
    gripper.SetTargetForce(force)
    gripper.SetTargetSpeed(speed)
    i = 0
    while True:
        g_state = 0
        gripper.SetTargetPosition(0)
        while(g_state == 0) :
            g_state = gripper.GetGripState()
            sleep(0.2)

        print(f"当前 {i} 力 ",gripper.GetCurrentTargetForce())
        print(f"当前 {i} 位置 ",gripper.GetCurrentPosition())
        print(f"当前 {i} 状态 ", gripper.GetGripState())
        print(f"当前 {i} 速度 ", gripper.GetCurrentTargetSpeed())
        # gripper.SetTargetPosition(0)
        # while(g_state == 0) :
        #     g_state = gripper.GetGripState()
        #     sleep(0.2)


        g_state = 0
        gripper.SetTargetPosition(1000)
        while(g_state == 0) :
            g_state = gripper.GetGripState()
            sleep(0.2)
        i += 1
    gripper.disconnect()