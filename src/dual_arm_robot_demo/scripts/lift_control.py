
import rospy

from dual_arm_msgs.msg import Lift_Height,LiftState

from std_msgs.msg import Empty




def lift_example(height,speed = 25):
    """
    uint16 height #目标高度，单位 mm，范围：0~2600
    uint16 speed #速度百分比，1~100
    """

    #测试升降
    height_msg = Lift_Height()
    height_msg.height = height         #范围： 0-790 
    height_msg.speed = speed
    pub_height.publish(height_msg)
    rospy.loginfo(f"发布目标高度: {height_msg.height}")
    rospy.sleep(5)

def lift_state_callback(data):

    rospy.loginfo(f"Height: {data.height}")
    rospy.loginfo(f"Current: {data.current}")
    rospy.loginfo(f"Error Flag: {data.err_flag}")
    rospy.loginfo(f"Mode: {data.mode}")


def main():

    #获取当前升降的高度
    pub_height_status.publish()
    
    rospy.sleep(2)
    height = input("请输出你要到的高度(0-790)：")
    
    height = int(height)

    #改变到某个高度
    lift_example(height)

if __name__ == '__main__':

    
    rospy.init_node('lift_status_node', anonymous=True)  # 初始化ROS节点

    pub_height = rospy.Publisher("/l_arm/rm_driver/Lift_SetHeight", Lift_Height, queue_size=10)  
    pub_height_status = rospy.Publisher("/l_arm/rm_driver/Lift_GetState", Empty, queue_size=10)  
    sub = rospy.Subscriber('/l_arm/rm_driver/LiftState', LiftState, lift_state_callback)

    rospy.sleep(1.5)

    main()

    rospy.spin()