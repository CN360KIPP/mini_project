import cv2
from PIL import Image
import numpy as np
import paho.mqtt.client as mqtt
import math
import time

acked = ""

def connection(broker):
    client = mqtt.Client("Publisher_cammera")
    client.connect(broker, 1883, keepalive=60)
    return client
client = connection("192.168.1.141")
# def subscribe(client, topic='ack'):
#     def on_message(client, userdata, msg):
#         # received_message = msg.payload.decode()
#         # print(f"Received '{msg.payload.decode()}' from '{msg.topic}' topic")
#         print(f"Received '{msg.payload.decode()}' from '{msg.topic}' topic")
#         acked = msg.payload.decode()

#     client.subscribe(topic)
#     client.on_message = on_message
#     # # Start the network loop
#     # client.loop_forever()

# def on_connect(client, userdata, flags, rc):
#     print("Connected with result code "+str(rc))
#     client.subscribe("ack")

# def on_message(client, userdata, msg):
#     # print(msg.topic+" "+str(msg.payload))
#     global acked 
#     acked = str(msg.payload)[2]

# def subscribe(client, topic='ack'):
#     client.subscribe(topic)
#     client.on_connect = on_connect
#     client.on_message = on_message
#     # Set the on_message callback

pub_delay=0
def publish(client,msg,topic="joystick"):
    global pub_delay
    # client.loop_start()
    # subscribe(client, 'ack')
    # client.loop_stop()
    # # ack=client.subscribe(topic='ack')
    # # message = client.on_message()
    # print("ack", acked)
    # if acked=='1':
    if pub_delay==60:
        pubMsg = client.publish(topic, msg)
        pubMsg.wait_for_publish()
        pub_delay=0
    else:
        pub_delay+=1


def get_limits(color):
    c = np.uint8([[color]])  # BGR values
    hsvC = cv2.cvtColor(c, cv2.COLOR_BGR2HSV)
    hue = hsvC[0][0][0]  # Get the hue value
    # Handle red hue wrap-around
    if hue >= 165:  # Upper limit for divided red hue
        lowerLimit = np.array([hue - 10, 100, 100], dtype=np.uint8)
        upperLimit = np.array([180, 255, 255], dtype=np.uint8)
    elif hue <= 15:  # Lower limit for divided red hue
        lowerLimit = np.array([0, 100, 100], dtype=np.uint8)
        upperLimit = np.array([hue + 10, 255, 255], dtype=np.uint8)
    else:
        lowerLimit = np.array([hue - 10, 100, 100], dtype=np.uint8)
        upperLimit = np.array([hue + 10, 255, 255], dtype=np.uint8)

    return lowerLimit, upperLimit

blue = [255,0,0]
# orange = [0,165,255]
red = [0,0,255]
# green = [0,255,0]
# yellow=[0,255,255]

cap = cv2.VideoCapture(0)
time.sleep(1)
window_name = "Detection"

def center_object(x1,y1,x2,y2):
    return (int( (x2+x1)/2 ),int( (y2+y1)/2) )

def angle_distance(source_point,destination_point):
    # Calculate angle and distance
    angle = np.arctan2(destination_point[1] - source_point[1], destination_point[0] - source_point[0]) * 180 / np.pi
    distance = np.sqrt((destination_point[0] - source_point[0])**2 + (destination_point[1] - source_point[1])**2)
    return angle, distance

def draw_line(frame, source_point, destination_point, color=(255, 255, 255), thickness=2):
    '''
    Draw line and write info
    '''
    cv2.line(frame, source_point, destination_point, color, thickness)
    angle, distance = angle_distance(source_point,destination_point)
    cv2.putText(frame,f'{angle:.2f}d',source_point,cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,0,0),thickness=2)
    cv2.putText(frame,f'{distance:.2f}px',center_object(source_point[0],source_point[1],destination_point[0],destination_point[1]),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,0,0),thickness=2)
    return angle, distance

def draw_initial_angle_line(frame, box_center, angle, color=(0, 0, 255), thickness=2, dot_size=5):
    '''
    Draw line and write info
    '''
    # Calculate the length of the line based on the angle
    line_length = 50  # You can adjust this value based on your requirements
    delta_x = int(line_length * math.cos(math.radians(angle)))
    delta_y = int(line_length * math.sin(math.radians(angle)))

    # Calculate the starting and ending points of the line
    start_point = (box_center[0] - delta_x, box_center[1] - delta_y)
    end_point = (box_center[0] + delta_x, box_center[1] + delta_y)

    # Draw the main line
    cv2.line(frame, start_point, end_point, color, thickness)

    # Draw a big dot at the head of the line
    cv2.circle(frame, end_point, dot_size, color, -1)


def setup_detection():
    '''
    Setting for create object box
    '''
    _, frame = cap.read()
    hsvImage = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lowerS, upperS = get_limits(color=red)
    lowerD, upperD = get_limits(color=blue)
    maskS = cv2.inRange(hsvImage, lowerS, upperS)
    maskD = cv2.inRange(hsvImage, lowerD, upperD)
    maskS_ = Image.fromarray(maskS)
    maskD_ = Image.fromarray(maskD)
    bboxS = maskS_.getbbox()
    bboxD = maskD_.getbbox()
    return frame, bboxS, bboxD

def draw_object_source(frame, bboxS):
    '''
    Object Source
    '''
    if bboxS is not None:
        x1, y1, x2, y2 = bboxS
        centerS = center_object(x1,y1,x2,y2)
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 255), 2)
        cv2.putText(frame,f'.',centerS,cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,255,255),thickness=5)
        cv2.putText(frame,f'yellow: {centerS}',(x2,y2),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,255,255),thickness=2)
        return centerS
    return 0

def draw_object_destination(frame, bboxD):
    '''
    Object Destination
    '''
    if bboxD is not None:
        p1, q1, p2, q2 = bboxD
        centerD = center_object(p1,q1,p2,q2)
        cv2.rectangle(frame, (p1, q1), (p2, q2), (255, 0, 0), 2)
        cv2.putText(frame,f'.',centerD,cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,0,0),thickness=5)
        cv2.putText(frame,f'blue: {centerD}',(p2,q2),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,0,0),thickness=2)
        return centerD
    return 0

'''
Problem: Algorithm to find path from source to destination
Solution1:
    initial: find Source direction
        step1: keep initial point
        step2: move 0000 0010 0 (y+10)
        ? delay buffer
        step3: keep last point
        # calculate the angle
        * keep angle, and assign to direction field # meet the requirement
    main: find Joystick data to sent(0000 0000 0: x y center)
        step1: find new path
        # calculate: frame_angle - source_angle = new_path
        step2: sent Joystick data (0000 0000 0)
        sin(angle) = x / distance
        cos(angle) = y / distance
        set speed to 30 # no accelarate
        sent
            x: 30 * sin(angle)
            y: 30 * cos(angle)
    prove:
        case better: continue
            + dont start from initial
        case worse: fix
            + start from initial
'''
initial_delay = 0
INITIAL_MAX = 30
main_delay = 0
MAIN_BUFFER = 30
SPEED = 30
while True:
    '''
    info application
    '''
    frame, bboxS, bboxD = setup_detection()
    centerS = draw_object_source(frame, bboxS)
    centerD = draw_object_destination(frame, bboxD)
    '''
    If there are any of object between source and destination disappear, Do continue
    * end case
    '''
    if (centerS == 0 or centerD == 0):
        # reset initial
        initial_delay = 0
        cv2.imshow(window_name, frame)
        print("Some Object's not found")
        publish(client, "0000 0000 0")
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
            # Check if the 'X' button is clicked to exit the loop
        if cv2.getWindowProperty(window_name, cv2.WND_PROP_AUTOSIZE) < 1:
            break
    else:
        '''
        Found both object
        Initial subcycle
        '''
        frame_angle, frame_distance = draw_line(frame, centerS, centerD)
        # stop
        if (frame_distance < 80):
                publish(client, "0000 0000 0")
                cv2.imshow(window_name, frame)
                print("Stop from reach the destination")
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                # Check if the 'X' button is clicked to exit the loop
                if cv2.getWindowProperty(window_name, cv2.WND_PROP_AUTOSIZE) < 1:
                    break
        # step1
        if (initial_delay==0):
            initial_source = centerS
        # step2
        publish(client, "0000 0020 0")
        '''
        delay 30 frame: asume that camera is 30 fps, thus delay 1 sec
        '''
        initial_delay+=1
        if initial_delay < INITIAL_MAX:
            cv2.imshow(window_name, frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            # Check if the 'X' button is clicked to exit the loop
            if cv2.getWindowProperty(window_name, cv2.WND_PROP_AUTOSIZE) < 1:
                break
        else:
            if initial_delay == 30:
                initial_destiantion = centerS
                initial_delay+=1
            initial_angle,_ = angle_distance(initial_source,initial_destiantion)
            draw_initial_angle_line(frame, centerS, initial_angle)
            '''
            main subcycle ****
            '''
            # prove path: step1 get initial distance
            if (main_delay == 0):
                initial_distance = frame_distance
            # if the path made longer distance 5 frames delay++, if not reset main delay
            if (frame_distance > initial_distance):
                main_delay+=1
            else:
                main_delay = 0
            if (main_delay < MAIN_BUFFER):
                # step1
                path_angle = frame_angle - initial_angle
                #step2
                # Convert degrees to radians
                path_radians = math.radians(path_angle)
                # Calculate x and y
                x = int(SPEED * math.sin(path_radians))
                y = int(SPEED * math.cos(path_radians))
                center = 0
                msg = "{0:4.0f} {1:4.0f} {2:1.0f}".format(x, y, center)
                publish(client, msg)
                cv2.imshow(window_name, frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                # Check if the 'X' button is clicked to exit the loop
                if cv2.getWindowProperty(window_name, cv2.WND_PROP_AUTOSIZE) < 1:
                    break
            else:
                initial_delay = 0
                cv2.imshow(window_name, frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                # Check if the 'X' button is clicked to exit the loop
                if cv2.getWindowProperty(window_name, cv2.WND_PROP_AUTOSIZE) < 1:
                    break
publish(client, "0000 0000 0")
cap.release()
cv2.destroyAllWindows()