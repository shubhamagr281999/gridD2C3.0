import flask
import rospy
from std_msgs.msg import String
msg="1.0,2.0,3"
def callback(data):
    global msg
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    msg=data.data   

rospy.init_node('listener', anonymous=True)
rospy.Subscriber("chatter", String, callback)

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