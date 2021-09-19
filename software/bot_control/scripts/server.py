import flask
import rospy
from geometry_msgs.msg import Twist
msg="1.0,2.0,3"
l=0.095
r=0.034
s=1
def callback(data):
    global msg
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    v=data.linear.x
    w=data.angular.z
    print(v,w)
    wl=(2*v-l*w)/(2*r) 
    wr=(2*v+l*w)/(2*r) 
    if(wl<100 and wr<100):
        msg="{},{},{}".format(-1.1*wl,1*wr,s)
    if(wl<25 and wr<25):
        msg="0,0,1"
    print(msg)
rospy.init_node('listener', anonymous=True)
rospy.Subscriber("/bot1/cmd_vel", Twist, callback)

app = flask.Flask(__name__)
app.config["DEBUG"] = True

try:
    @app.route('/1', methods=['GET'])
    def home():
        global msg
        print(msg)
        return msg
except:
    pass

app.run()