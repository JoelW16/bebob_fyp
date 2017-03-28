import rospy
import sys
import os
from ftplib import FTP
from std_msgs.msg import Empty, String


class guide:
    def __init__(self, latitude, longitiude):
        self.dir = os.path.dirname(__file__)
        self.fileName = os.path.join(self.dir,"guideFlight"+".mavlink")
        self.latitiude = float("{0:.6f}".format(latitude))
        self.longitiude = float("{0:.6f}".format(longitiude))
        self.pub_startFlightPlan = rospy.Publisher('/bebop/autoflight/start', String, queue_size=10)
        self.pub_stopFlightPlan = rospy.Publisher('/bebop/autoflight/stop', Empty, queue_size=10)
        self.createFlightPlan()
        self.uploadFlightPlan()



    def createFlightPlan(self):
        mavlink = open(self.fileName, "w")
        mavlink.write("QGC WPL 120")
        #                        cmd            radius                 yaw              lat                 long               Altiude
        mavlink.write("\n0\t0\t3\t16\t0.000000\t5.000000\t0.000000\t0.000000\t" + str(self.latitiude) + "\t" + str(self.longitiude) + "\t3.000000\t1")
        mavlink.write("\n1\t0\t3\t17\t0.000000\t0.000000\t0.000000\t0.000000\t" + str(self.latitiude) + "\t" + str(self.longitiude)  + "\t3.000000\t1")
        mavlink.close()
        print('.MavLink Created')

    def uploadFlightPlan(self):
        print('upload')
        ftp = FTP('192.168.42.1')
        ftp.login()
        print "FTP Connected"
        ftp.cwd('/internal_000/flightplans')
        print "DIR"
        file = open(self.fileName, 'rb')
        ftp.storbinary('STOR guideFlight.mavlink', file)
        file.close()
        ftp.quit()
        print "Uploaded"



    def startFlightPlan(self):
        pass

    def stopFlightPlan(self):
        pass
        #self.pub_stopFlightPlan.publish(Empty())





def main(args):
    try:
        rospy.init_node('bebop_guide')
        rospy.spin()
    except KeyboardInterrupt:
        rospy.signal_shutdown('Bye')

if __name__ == '__main__':
    main(sys.argv)
