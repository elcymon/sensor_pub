#!/usr/bin/env python
import rospy
import roslib
from std_msgs.msg import String
from std_msgs.msg import Float64
from mavros_msgs.msg import OverrideRCIn
import serial
from mavros_msgs.srv import SetMode
from time import sleep

ser = serial.Serial('/dev/ttyACM0',115200,timeout=0.01)
serial_data = []
count = 0
msgg = OverrideRCIn()
def callback_read_serial(data):
    global serial_data
    v=''
   # v = ser.readline().rstrip('\r\n')
    #v = v.split(',')
    #serial_data = v
    #serial_data = [float(i) for i in v]
def read_serial():
    global count
    count = count + 1
    c = str(count)
    ser.write(c.encode('utf-8'))
    v = ''
    sleep(1)
    v = ser.readline().rstrip('\r\n')
    # while ser.in_waiting > 0:
    #     v = ser.readline().rstrip('\r\n')

    return (count,v)

def listener():
    global serial_data,count
    sub_serial = rospy.Subscriber('/read_serial',String,callback_read_serial,queue_size=1)
    pub_serial = rospy.Publisher('/read_serial',String,queue_size=1)
    pub_imu = rospy.Publisher('/pub_imu',String,queue_size=1)
    pub_rps = rospy.Publisher('/pub_rps',String,queue_size=1)
    pp = rospy.Publisher('/mavros/rc/override',OverrideRCIn,queue_size=1)
    #sub = rospy.Subscriber('/mavros/imu/data',Imu,imu_callback,queue_size=1)
    rate = rospy.Rate(10)
    start_serial = False
    imu_found = 'nan'
    start = False
    x = 1000
    while not rospy.is_shutdown():
        # msgg = OverrideRCIn()
        # x = x + 1
        # if x > 1900:
        #     x = 1000
        # msgg.channels[2] = x
        # pp.publish(msgg)

        v='nan'
        c = 'nan'
        count = count + 1
        while ser.in_waiting > 0:
            vtemp = ser.readline().rstrip('\r\n')
            # print vtemp
            if vtemp != '':
                v = vtemp
        
        if not start_serial:
            if 'Ready' in v:
                print('start ser')
                #ser.write('start'.encode('utf-8'))
                start = True
                start_serial = True
                imu_found = 'yes'
                count = 0
            if 'No IMU' in v:
                print('NO IMU FOUND')
                start_serial = True
                start = True
                imu_found = 'no'
                count = 0
                #ser.write('start'.encode('utf-8'))
                

        if start and 'Ready' not in v and 'No IMU' not in v and 'yes' in imu_found:
            # count = count + 1
            c = str(count)
            #ser.reset_input_buffer()
            ser.write(c.encode('utf-8'))
            #c,v = read_serial()
            pub_serial.publish(v)
           #v = ser.readline().rstrip('\r\n')
            #serial_data = v.split(',')
            print v
            vt = v.split(',')
            rps = vt[0]
            vt = vt[1:]
            v = ','.join(vt)
            serial_data = v#consider whether to change it to Imu type
            pub_imu.publish(serial_data)
            pub_rps.publish(rps)
            # print 'imu found',v
        elif 'no' in imu_found and 'No IMU' not in v and 'Ready' not in v:
            # print 'no imu'
            #count = count + 1
            #c = str(count)
            ser.write(c.encode('utf-8'))
            # c,v = read_serial()
            pub_serial.publish(v)
            rps = v
            pub_rps.publish(rps)
            print rps,'no imu'
        else:
            # c,v = read_serial()
            pub_serial.publish(v)
            print 'error',v
            #print(ser.in_waiting,count,serial_data)
        # print(ser.in_waiting,count,v)
    

        rate.sleep()
    print("Exiting")


if __name__=='__main__':
    node =     rospy.init_node('my_rover',anonymous=True)
    print('starting sim')
    rospy.wait_for_service('/mavros/set_mode')
    change_mode = rospy.ServiceProxy('/mavros/set_mode',SetMode)
    answer = change_mode(custom_mode='manual')
    print(answer)
    #answer = 'True'
    if 'True' in str(answer):
      try:
        listener()
      except rospy.ROSInterruptException:
        print('Something Went wrong')
        pass
