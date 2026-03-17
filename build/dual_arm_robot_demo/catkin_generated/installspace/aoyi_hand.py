import socket,time

import rospy


class AoYiHand():

    def __init__(self,ip = "169.254.128.19",port = 8080):
        
        self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client.connect((ip, port))


        self.get_power_ready() #modbus初始化


    def send_cmd(self,cmd_6axis):

        self.client.send(cmd_6axis.encode('utf-8'))


        return True
    
    def get_power_ready(self):
        """傲意灵巧手modbus初始化

        """

        point6_00 = '{"command":"set_modbus_mode","port":1,"baudrate":115200,"timeout ":2}\r\n'
        _ = self.send_cmd(cmd_6axis=point6_00)
        time.sleep(2)
        rospy.loginfo("配置通讯端口 ModbusRTU 模式")

    def open_hand(self):
        """傲意灵巧手打开

        """

        point6_00 = '{"command":"write_registers","port":1,"address":1135,"num":6,"data":[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 255, 0],"device":2}\r\n'
        _ = self.send_cmd(cmd_6axis=point6_00)
        time.sleep(2)
        rospy.loginfo(_)
        rospy.loginfo("  执行初始化成功")

    def catch_dumb(self):
        """傲意灵巧手抓东西

        """

        #握住瓶子哑铃
        point6_00 = '{"command":"write_registers","port":1,"address":1135,"num":6,"data":[255,255,255,255,255,255,255,255,255,255,0,0],"device":2}\r\n'
        _ = self.send_cmd(cmd_6axis=point6_00)
        time.sleep(2)
        rospy.loginfo(_)
        rospy.loginfo("  执行初始化成功")


if __name__ == "__main__":

    aoyihand = AoYiHand()
    aoyihand.open_hand()