# This file contains two classes:
# Robolink()
# Item()
# These classes are the main objects used to create macros for RoboDK.
# An item is an object in the RoboDK tree (it can be either a robot, an object, a tool, a frame, a program, ...).

import struct
from robodk import *
from warnings import warn

ITEM_CASE_STATION=1
ITEM_CASE_ROBOT=2
ITEM_CASE_FRAME=3
ITEM_CASE_TOOL=4
ITEM_CASE_OBJECT=5
ITEM_CASE_TARGET=6
ITEM_CASE_PROGRAM=8

RUNMODE_SIMULATE=1      # performs the simulation moving the robot (default)
RUNMODE_QUICKVALIDATE=2 # performs a quick check to validate the robot movements
RUNMODE_MAKE_ROBOTPROG=3# makes the robot program
RUNMODE_RUN_REAL=4      # moves the real robot is it is connected

class Robolink:
    """This class is the link to allows to create macros and automate Robodk.
    Any interaction is made through \"items\" (Item() objects). An item is an object in the
    robodk tree (it can be either a robot, an object, a tool, a frame, a 
    program, ...)."""
    APPLICATION_DIR = ''    # file path to the robodk program (executable)
    SAFE_MODE = 1           # checks that provided items exist in memory
    AUTO_UPDATE = 0         # if AUTO_UPDATE is zero, the scene is rendered after every function call
    TIMEOUT = 5             # timeout for communication, in seconds
    COM = None              # tcpip com
    PORT_START = 20500      # port to start looking for app connection
    PORT_END = 20510        # port to stop looking for app connection
    PORT = -1
    #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    def is_connected(self):
        "Returns 1 if connection is valid, returns 0 if connection is invalid"
        if not self.COM: return 0
        connected = 1        
        #try:
        #    self.COM.settimeout(0)
        #    check = self.COM.recv(1)
        #except:
        #    connected = 0
        #    
        #self.COM.settimeout(self.TIMEOUT)
        return connected

    def check_connection(self):
        """If we are not connected it will attempt a connection, if it fails, it will throw an error"""
        if not self.is_connected() and self.Connect() < 1:
            raise Exception('Unable to connect')
        #To do: Clear input buffer.

    def check_status(self):
        """This function checks the status of the connection"""
        status = self.rec_int()
        if status > 0 and status < 10:
            strproblems = 'Unknown error'
            if status == 1:
                strproblems = 'Invalid item provided: The item identifier provided is not valid or it does not exist.'
            elif status == 2: #output warning
                strproblems = self.rec_line()
                print('WARNING: ' + strproblems)
                #warn(strproblems)# does not show where is the problem...
                return 0
            elif status == 3: #output error
                strproblems = self.rec_line()
                #raise Exception(msg)
            elif status == 9:
                strproblems = 'Invalid license. Contact us at: www.robodk.com'
            print(strproblems)
            raise Exception(strproblems)
        elif status == 0:
            # everything is OK
            status = status;
        else:
            raise Exception('Problems running function')
        return status

    def check_color(self, color):
        """Formats the color in a vector of size 4x1 and ranges [0,1]"""
        if not isinstance(color,list) or len(color) < 3 or len(color) > 4:
            raise Exception('The color vector must be a list of 3 or 4 values')
        if len(color) == 3:
            color.append(1)
        if max(color) > 1 or min(color) < -1:
            print("WARNING: Color provided is not in the range [0,1] ([r,g,b,a])")
        return color

    def send_line(self, string=None):
        """Sends a string of characters with a \\n"""
        self.COM.send(bytes(string+'\n','utf-8'))

    def rec_line(self):
        """Receives a string. It reads until if finds LF (\\n)"""
        string = ''
        chari = self.COM.recv(1).decode('utf-8')
        while chari != '\n':    #LF
            string = string + chari
            chari = self.COM.recv(1).decode('utf-8')
        return string

    def send_item(self, item):
        """Sends an item pointer"""
        if isinstance(item, Item):
            self.COM.send(struct.pack('>Q',item.item))#q=unsigned long long (64 bits), d=float64
            return
        self.COM.send(struct.pack('>Q',item))#q=unsigned long long (64 bits), d=float64

    def rec_item(self):
        """Receives an item pointer"""
        buffer = self.COM.recv(8)
        item = struct.unpack('>Q',buffer)#q=unsigned long long (64 bits), d=float64
        return Item(self,item[0])

    def send_pose(self, pose):
        """Sends a pose (4x4 matrix)"""
        if not pose.isHomogeneous():
            print("Warning: pose is not homogeneous!")
            print(pose)
        posebytes = b''
        for j in range(4):
            for i in range(4):
                posebytes = posebytes + struct.pack('>d',pose[i,j])
        self.COM.send(posebytes)

    def rec_pose(self):
        """Receives a pose (4x4 matrix)"""
        posebytes = self.COM.recv(16*8)
        posenums = struct.unpack('>16d',posebytes)
        pose = Mat(4,4)
        cnt = 0
        for j in range(4):
            for i in range(4):
                pose[i,j] = posenums[cnt]
                cnt = cnt + 1
        return pose
        
    def send_xyz(self, pos):
        """Sends an xyz vector"""
        posbytes = b''
        for i in range(3):
            posbytes = posbytes + struct.pack('>d',pos[i])
        self.COM.send(posbytes)

    def rec_xyz(self):
        """Receives an xyz vector"""
        posbytes = self.COM.recv(3*8)
        posnums = struct.unpack('>3d',posbytes)
        pos = [0,0,0]
        for i in range(3):
            pos[i] = posnums[i]
        return pos

    def send_int(self, num):
        """Sends an int (32 bits)"""
        if isinstance(num, float):
            num = round(num)
        elif not isinstance(num, int):
            num = num[0]
        self.COM.send(struct.pack('>i',num))

    def rec_int(self):
        """Receives an int (32 bits)"""
        buffer = self.COM.recv(4)
        num = struct.unpack('>i',buffer)
        return num[0]

    def send_array(self, values):
        """Sends an array of doubles"""
        if not isinstance(values,list):#if it is a Mat() with joints
            values = (values.tr()).rows[0];          
        nval = len(values)
        self.send_int(nval)        
        if nval > 0:
            buffer = b''
            for i in range(nval):
                buffer = buffer + struct.pack('>d',values[i])
            self.COM.send(buffer)

    def rec_array(self):
        """Receives an array of doubles"""
        nvalues = self.rec_int()
        if nvalues > 0:
            buffer = self.COM.recv(8*nvalues)
            values = list(struct.unpack('>'+str(nvalues)+'d',buffer))
            #values = fread(self.COM, nvalues, 'double')
        else:
            values = []
        return Mat(values)

    def send_matrix(self, mat):
        """Sends a 2 dimensional matrix (nxm)"""
        size = mat.size()
        self.send_int(size[0])
        self.send_int(size[1])
        for j in range(size[1]):
            matbytes = b''
            for i in range(size[0]):
                matbytes = matbytes + struct.pack('>d',mat[i,j])
            self.COM.send(matbytes)

    def rec_matrix(self):
        """Receives a 2 dimensional matrix (nxm)"""
        size1 = self.rec_int()
        size2 = self.rec_int()
        if size1 * size2 > 0:
            matbytes = self.COM.recv(size1*size2*8)
            matnums = struct.unpack('>'+str(size1*size2)+'d',matbytes)
            mat = Mat(size1,size2)
            cnt = 0
            for j in range(size2):
                for i in range(size1):
                    mat[i,j] = matnums[cnt]
                    cnt = cnt + 1
        else:
            mat = Mat([[]])
        return mat

    def moveX(self, target, itemrobot, movetype, blocking=True):
        """Performs a linear or joint movement. Use MoveJ or MoveL instead."""
        #self.check_connection();
        itemrobot.WaitMove()# checks connection
        command = 'MoveX'
        self.send_line(command)
        self.send_int(movetype)
        if isinstance(target,Item):# target is an item
            self.send_int(3)
            self.send_array([])
            self.send_item(target)
        elif isinstance(target,list) or target.size() != (4,4):# target are joints
            self.send_int(1)
            self.send_array(target)
            self.send_item(0)
        elif target.size() == (4,4):    # target is a pose
            self.send_int(2)
            mattr = target.tr()
            self.send_array(mattr.rows[0]+mattr.rows[1]+mattr.rows[2]+mattr.rows[3])
            self.send_item(0)
        else:
            raise Exception('Invalid input values')
        self.send_item(itemrobot)
        self.check_status()
        if blocking:
            itemrobot.WaitMove()

    #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    def __init__(self):
        """A connection is attempted upon creation of the object"""
        self.Connect()

    def Set_connection_params(self, safe_mode=1, auto_update=0, timeout=None):
        """Sets some behavior parameters: SAFE_MODE, AUTO_UPDATE and TIMEOUT.
        SAFE_MODE checks that item pointers provided by the user are valid.
        AUTO_UPDATE checks that item pointers provided by the user are valid.
        TIMEOUT is the timeout to wait for a response. Increase if you experience problems loading big files.
        If connection failed returns 0.
        In  1 (optional) : int -> SAFE_MODE (1=yes, 0=no)
        In  2 (optional) : int -> AUTO_UPDATE (1=yes, 0=no)
        In  3 (optional) : int -> TIMEOUT (1=yes, 0=no)
        Out 1 : int -> connection status (1=ok, 0=problems)
        Example:
            Set_connection_params(0,0); # Use for speed. Render() must be called to refresh the window.
            Set_connection_params(1,1); # Default behavior. Updates every time."""
        self.SAFE_MODE = safe_mode
        self.AUTO_UPDATE = auto_update
        self.TIMEOUT = timeout or self.TIMEOUT
        self.send_line('CMD_START')
        self.send_line(str(self.SAFE_MODE) + ' ' + str(self.AUTO_UPDATE))
        #fprintf(self.COM, sprintf('%i %i'), self.SAFE_MODE, self.AUTO_UPDATE))# appends LF
        response = self.rec_line()
        if response == 'READY':
            ok = 1
        else:
            ok = 0
        return ok

    def Connect(self):
        """Establishes a connection with robodk. robodk must be running, otherwise, the variable APPLICATION_DIR must be set properly.
        If the connection succeededs it returns 1, otherwise it returns 0"""
        import socket
        connected = 0
        for i in range(0,2):
            for port in range(self.PORT_START,self.PORT_END+1):
                self.COM = socket.socket(socket.AF_INET, socket.SOCK_STREAM)#'localhost', port, 'Timeout', self.TIMEOUT, 'BytesAvailableFcnMode', 'byte', 'InputBufferSize', 4000); 
                #self.COM.setblocking(1)#default is blocking
                self.COM.settimeout(self.TIMEOUT)
                try:
                    self.COM.connect(('localhost', port))                
                    connected = self.is_connected()
                    if connected > 0:
                        break
                except:
                    connected = connected

            if connected > 0:# if status is closed, try to open application
                self.PORT = port
                break;
            else:
                try:
                    import os
                    os.system(self.APPLICATION_DIR)
                    #sample: os.system(r"C:\Documents and Settings\flow_model\flow.exe")
                    #winopen(self.APPLICATION_DIR);
                    import time
                    time.sleep(2);
                except:
                    raise Exception(['application path is not correct or could not start: ',self.APPLICATION_DIR])

        if connected > 0 and not self.Set_connection_params():
            connected = 0
        return connected

    #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    # public methods
    def Item(self, name):
        """Returns an item by its name. If there is no exact match it will return the last closest match.
        check variables ITEM_CASE_*
        Example:
            RL = Robolink()
            item = RL.Get_Item('Robot') #(item equals ITEM_CASE_ROBOT)"""
        self.check_connection()
        command = 'G_Item'
        self.send_line(command)
        self.send_line(name)
        item = self.rec_item()#     item = fread(com, 2, 'ulong');% ulong is 32 bits!!!
        self.check_status()
        return item

    def ItemList(self):
        """Returns a list of names of all available items in the currently open station in robodk"""
        self.check_connection()
        command = 'G_List_Items'
        self.send_line(command)
        count = self.rec_int()
        retlist = []
        for i in range(count):
            namei = self.rec_line()
            print(i,'\t->\t',namei)
            retlist.append(namei)
        self.check_status()
        return retlist

    #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    def Copy(self, item):
        """Makes a copy of an item (same as Ctrl+C), which can be pasted (Ctrl+V) using Paste_Item().
        In 1 : item
        Example:
            RL = Robolink()
            object = RL.Item('My Object');
            object.Copy()         #RL.Copy(object); also works
            newobject = RL.Paste();
            newobject.setName('My Object (copy 1)');
            newobject = RL.Paste();
            newobject.setName('My Object (copy 2)');"""
        self.check_connection()
        command = 'Copy'
        self.send_line(command)
        self.send_item(item)
        self.check_status()

    def Paste(self, toparent=0):
        """Pastes the copied item (same as Ctrl+V). Needs to be used after Copy_Item(). See Copy_Item() for an example.
        In 1 (optional): item -> parent to paste to"""
        self.check_connection()
        command = 'Paste'
        self.send_line(command)
        self.send_item(toparent)
        newitem = self.rec_item()
        self.check_status()
        return newitem

    def AddFile(self, filename, parent=0):
        """Loads a file and attaches it to parent. It can be any file supported by robodk.
        Timeout may have to be increased if big files are loaded.
        In 1  : string -> absolute path of the file
        In 2 (optiona): item -> parent to attach
        Out 1 : item -> added item (0 if failed)
        Example:
            RL = Robolink()
            item = Add_File(r'C:\\Users\\Name\\Desktop\\object.step')
            RL.Set_Pose(item, transl(100,50,500))"""
        self.check_connection()
        command = 'Add'
        self.send_line(command)
        self.send_line(filename)
        self.send_item(parent)
        newitem = self.rec_item()
        self.check_status()
        return newitem

    def AddTarget(self, name, itemparent=0, itemrobot=0):
        """Adds a new target that can be reached with a robot.
        In  1 : string -> name of the target
        In  2 (optional): item -> parent to attach to (such as a frame)
        In  3 (optional): item -> main robot that will be used to go to self target
        Out 1 : item -> the new item created"""
        self.check_connection()
        command = 'Add_TARGET'
        self.send_line(command)
        self.send_line(name)
        self.send_item(itemparent)
        self.send_item(itemrobot)
        newitem = self.rec_item()
        self.check_status()
        return newitem

    def AddFrame(self, name, itemparent=0):
        """Adds a new Frame that can be referenced by a robot.
        In  1 : string -> name of the frame
        In  2 (optional): item -> parent to attach to (such as the rrobot base frame)
        Out 1 : item -> the new item created"""
        self.check_connection()
        command = 'Add_FRAME'
        self.send_line(command)
        self.send_line(name)
        self.send_item(itemparent)
        newitem = self.rec_item()
        self.check_status()
        return newitem

    def AddProgram(self, name, itemrobot=0):
        """Adds a new program.
        In  1 : string -> name of the program
        In  2 (optional): item -> robot that will be used
        Out 1 : item -> the new item created"""
        self.check_connection()
        command = 'Add_PROG'
        self.send_line(command)
        self.send_line(name)
        self.send_item(itemrobot)
        newitem = self.rec_item()
        self.check_status()
        return newitem

    #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    def RunProgram(self, fcn_param):
        """Adds a function call in the program output. RoboDK will handle the syntax when the code is generated for a specific robot. If the program exists it will also run the program in simulate mode.
        In  1 : fcn call -> string of the program to run
        Out 1 : this function always returns 0"""
        return self.RunCode(fcn_param, True)
    
    def RunCode(self, code, code_is_fcn_call=False):
        """Adds code to run in the program output. If the program exists it will also run the program in simulate mode.
        In  1 : code -> string of the code or program to run
        In  2 : code_is_fcn_call -> True if the code corresponds to a function call (same as RunProgram), if so, RoboDK will handle the syntax when the code is generated for a specific robot
        Out 1 : this function always returns 0"""
        self.check_connection()
        command = 'RunCode'
        self.send_line(command)
        self.send_int(code_is_fcn_call)
        self.send_line(code.replace('\r\n','<<br>>').replace('\n','<<br>>'))
        prog_status = self.rec_int()
        self.check_status()
        return prog_status
    
    def RunMessage(self, message, message_is_comment=False):
        """Shows a message or a comment in the output robot program.
        In  1 : string -> message or comment to show in the teach pendant
        Out 1 : int -> if message_is_comment is set to True (or 1) the message will appear only as a comment in the code"""
        print('Message: ' + message)
        self.check_connection()
        command = 'RunMessage'
        self.send_line(command)
        self.send_int(message_is_comment)
        self.send_line(message.replace('\r\n','<<br>>').replace('\n','<<br>>'))
        self.check_status()    

    def Render(self, always_render=False):
        """Renders the scene. This function turns off rendering unless always_render is set to true."""
        auto_render = not always_render;
        self.check_connection()
        command = 'Render'
        self.send_line(command)
        self.send_int(auto_render)
        self.check_status()

    def Collision(self):
        """Returns the number of pairs of objects that are currently in a collision state."""
        self.check_connection()
        command = 'Collision'
        self.send_line(command)
        ncollisions = self.rec_int()
        self.check_status()
        return ncollisions

    def setSimulationSpeed(self, speed):
        """Sets the current simulation speed. Set the speed to 1 for a real-time simulation. The slowest speed allowed is 0.001 times the real speed. Set to a high value (>100) for fast simulation results.""" 
        self.check_connection()
        command = 'SimulateSpeed'
        self.send_line(command)
        self.send_int(speed*1000)
        self.check_status()
    
    def setRunMode(self, run_mode=1):
        """Sets the behavior of the script. By default, robodk shows the path simulation for movement instructions (run_mode=1).
        Setting the run_mode to 2 allows to perform a quick check to see if the path is feasible.
        In   1 : int = RUNMODE
        RUNMODE_SIMULATE=1        performs the simulation moving the robot (default)
        RUNMODE_QUICKVALIDATE=2   performs a quick check to validate the robot movements
        RUNMODE_MAKE_ROBOTPROG=3  makes the robot program
        RUNMODE_RUN_REAL=4        moves the real robot is it is connected
        """
        self.check_connection()
        command = 'S_RunMode'
        self.send_line(command)
        self.send_int(run_mode)
        self.check_status()
        
    def RunMode(self):
        """Returns the behavior of the script. By default, robodk shows the path simulation for movement instructions (run_mode=1).
        If run_mode = 2, the script is performing a quick check to see if the path is feasible (usually managed by the GUI).
        If run_mode = 3, the script is generating the robot program (usually managed by the GUI).
        Out  1 : int = RUNMODE
        RUNMODE_SIMULATE=1        performs the simulation moving the robot (default)
        RUNMODE_QUICKVALIDATE=2   performs a quick check to validate the robot movements
        RUNMODE_MAKE_ROBOTPROG=3  makes the robot program
        RUNMODE_RUN_REAL=4        moves the real robot is it is connected
        """            
        self.check_connection()
        command = 'G_RunMode'
        self.send_line(command)
        runmode = self.rec_int()
        self.check_status()
        return runmode

    def getParam(self, param='PATH_OPENSTATION'):
        """Gets a global parameter from the RoboDK station.
        In  1 : string = parameter
        Out 1 : string = value
        Available parameters:
        PATH_OPENSTATION = folder path of the current .stn file
        FILE_OPENSTATION = file path of the current .stn file
        PATH_DESKTOP = folder path of the user's folder"""    
        self.check_connection()
        command = 'G_Param'
        self.send_line(command)
        self.send_line(param)
        value = self.rec_line()
        self.check_status()
        return value

    def ShowSequence(self, matrix):
        """Displays a sequence of joints
        In  1 : joint sequence as a 6xN matrix or instruction sequence as a 7xN matrix"""
        Item(self, 0).ShowSequence(matrix)

    def LaserTracker_Measure(self, estimate=[0,0,0], search=False):
        """Takes a laser tracker measurement with respect to the reference frame. If an estimate point is provided, the laser tracker will first move to those coordinates. If search is True, the tracker will search for a target.
        Returns the XYZ coordinates of target if it was found. Othewise it retuns None."""
        self.check_connection()
        command = 'MeasLT'
        self.send_line(command)
        self.send_xyz(estimate)
        self.send_int(1 if search else 0)
        xyz = self.rec_xyz()
        self.check_status()
        if xyz[0]*xyz[0] + xyz[1]*xyz[1] + xyz[2]*xyz[2] < 0.0001:
            return None
        
        return xyz

    def Collision_Line(self, p1, p2, ref=eye(4)):
        """Checks the collision between a line and the station. The line is composed by 2 points.
        In  1 : p1 -> start point of the line
        In  2 : p2 -> end point of the line
        In  3 : pose (optional) -> reference of the 2 points
        Out 1 : collision -> True if there is a collision, False otherwise
        Out 2 : item -> Item collided
        Out 3 : point -> collision point (station reference)"""
        p1abs = ref*p1;
        p2abs = ref*p2;        
        self.check_connection()
        command = 'CollisionLine'
        self.send_line(command)
        self.send_xyz(p1abs)
        self.send_xyz(p2abs)
        itempicked = self.rec_item()
        xyz = self.rec_xyz()
        collision = itempicked.Valid()
        self.check_status()
        return collision, itempicked, xyz



   
class Item():
    """The Item class represents an item in RoboDK station. An item can be a robot, a frame, a tool, an object, a target, ... any item visible in the station tree.
    An item can also be seen as a node where other items can be attached to (child items).
    Every item has one parent item/node and can have one or more child items/nodes"""
    
    def __init__(self, link, ptr_item=0):
        self.item = ptr_item
        self.link = link # important to keep this a reference and not a duplicate (otherwise it will establish a new connection at every call)

    def __repr__(self):
        if self.Valid():
            return ("RoboDK item (%i)" % self.item)
        else:
            return "RoboDK item (INVALID)"

    def RL(self):
        """Returns the RoboDK link Robolink()."""
        return self.link
    
    #"""Generic item calls"""
    def Type(self):
        """Returns an integer that represents the type of the item (robot, object, tool, frame, ...)
        Compare the returned value to ITEM_CASE_* variables"""
        self.link.check_connection()
        command = 'G_Item_Type'
        self.link.send_line(command)
        self.link.send_item(self)
        itemtype = self.link.rec_int()
        self.link.check_status()
        return itemtype
        
    def Copy(self):
        """Copy the item to the clipboard (same as Ctrl+C). Use together with Paste() to duplicate items."""
        self.link.Copy(self.item)
        
    def Paste(self):
        """Paste the item from the clipboard as a child of this item (same as Ctrl+V)
        Out 1: item -> new item pasted (created)"""
        return self.link.Paste(self.item)

    def AddGeometry(self, fromitem, pose):
        """Makes a copy of the geometry fromitem adding it at a given position (pose) relative to this item."""
        self.link.check_connection()
        command = 'CopyFaces'
        self.link.send_line(command)
        self.link.send_item(fromitem)
        self.link.send_item(self)
        self.link.send_pose(pose)        
        self.link.check_status()
        
    def Delete(self):
        """Deletes an item and its childs from the station.
        In  1 : item -> item to delete"""
        self.link.check_connection()
        command = 'Remove'
        self.link.send_line(command)
        self.link.send_item(self)
        self.link.check_status()
        self.item = 0

    def Valid(self):
        """Checks if the item is valid. An invalid item will be returned by an unsuccessful function call."""
        if self.item == 0: return False
        return True
    
    def setParent(self, parent):
        """Moves the item to another location item node "parent" (a different parent within the tree)
        In 1  : parent -> parent item to attach the item"""
        self.link.check_connection()
        command = 'S_Parent'
        self.link.send_line(command)
        self.link.send_item(self)
        self.link.send_item(parent)
        self.link.check_status()
        
    def setParentStatic(self, parent):
        """Moves the item to another location (parent) without changing the current position in the station
        In 1  : parent -> parent to attach the item"""
        self.link.check_connection()
        command = 'S_Parent_Static'
        self.link.send_line(command)
        self.link.send_item(self)
        self.link.send_item(parent)
        self.link.check_status()

    def AttachClosest(self):
        """Attaches the closest object to the provided tool (see also: Set_Parent_Static).
        Out  : item -> returns the item that was attached (item.Valid() is False if none found)"""
        self.link.check_connection()
        command = 'Attach_Closest'
        self.link.send_line(command)
        self.link.send_item(self)
        item_attached = self.link.rec_item()
        self.link.check_status()
        return item_attached

    def DetachClosest(self, parent=0):
        """Detaches the closest object attached to the tool (see also: setParentStatic).
        In 1 : parent item (optional) -> parent to leave the object."""
        self.link.check_connection()
        command = 'Detach_Closest'
        self.link.send_line(command)
        self.link.send_item(self)
        self.link.send_item(parent)
        item_detached = self.link.rec_item()
        self.link.check_status()
        return item_detached        

    def DetachAll(self, parent=0):
        """Detaches any object attached to a tool (see also: setParentStatic).
        In  1 : item (optional) -> parent to leave the objects"""
        self.link.check_connection()
        command = 'Detach_All'
        self.link.send_line(command)
        self.link.send_item(self)
        self.link.send_item(parent)
        self.link.check_status()

    def Parent(self):
        """Returns the parent item of the item.
        Out : parent -> parent of the item"""
        self.link.check_connection()
        command = 'G_Parent'
        self.link.send_line(command)
        self.link.send_item(self)
        parent = self.link.rec_item()
        self.link.check_status()
        return parent

    def Childs(self):
        """Returns a list of the item childs that are attached to the provided item.
        Out  : item x n -> list of child items"""
        self.link.check_connection()
        command = 'G_Childs'
        self.link.send_line(command)
        self.link.send_item(self)
        nitems = self.link.rec_int()
        itemlist = []
        for i in range(nitems):
            itemlist.append(self.link.rec_item())
        self.link.check_status()
        return itemlist

    #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    def Visible(self):
        """Returns 1 if the item is visible, otherwise, returns 0.
        Out : int -> visible (1) or not visible (0)"""
        self.link.check_connection()
        command = 'G_Visible'
        self.link.send_line(command)
        self.link.send_item(self)
        visible = self.link.rec_int()
        self.link.check_status()
        return visible

    def setVisible(self, visible, visible_frame=None):
        """Sets the item visiblity status
        In 1 : int -> set visible (True or 1) or not visible (False or 0)
        In 2 : int (optional) -> set visible frame (True or 1) or not visible (False or 0)"""        
        if visible_frame is None: visible_frame = visible
        self.link.check_connection()
        command = 'S_Visible'
        self.link.send_line(command)
        self.link.send_item(self)
        self.link.send_int(visible)
        self.link.send_int(visible_frame)
        self.link.check_status()

    def Name(self):
        """Returns the name of an item. The name of the item is always displayed in the RoboDK station tree
        Out : name (string)"""
        self.link.check_connection()
        command = 'G_Name'
        self.link.send_line(command)
        self.link.send_item(self)
        name = self.link.rec_line()
        self.link.check_status()
        return name

    def setName(self, name):
        """Sets the name of an item.
        In 1 : name (string)"""
        self.link.check_connection()
        command = 'S_Name'
        self.link.send_line(command)
        self.link.send_item(self)
        self.link.send_line(name)
        self.link.check_status()
        
    #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    def setPose(self, pose):
        """Sets the local position (pose) of an item. For example, the position of an object/frame/target with respect to its parent.
        In 1 : 4x4 homogeneous matrix (pose)"""
        self.link.check_connection()
        command = 'S_Hlocal'
        self.link.send_line(command)
        self.link.send_item(self)
        self.link.send_pose(pose)
        self.link.check_status()

    def Pose(self):
        """Returns the local position (pose) of an item. For example, the position of an object/frame/target with respect to its parent.
        Out : 4x4 homogeneous matrix (pose)"""
        self.link.check_connection()
        command = 'G_Hlocal'
        self.link.send_line(command)
        self.link.send_item(self)
        pose = self.link.rec_pose()
        self.link.check_status()
        return pose

    def setHtool(self, pose):
        """Sets the tool pose of a tool item.
        In 1 : 4x4 homogeneous matrix (pose)"""
        self.link.check_connection()
        command = 'S_Htool'
        self.link.send_line(command)
        self.link.send_item(self)
        self.link.send_pose(pose)
        self.link.check_status()

    def Htool(self):
        """Returns the tool pose of an item.
        Out 1 : 4x4 homogeneous matrix (pose)"""
        self.link.check_connection()
        command = 'G_Htool'
        self.link.send_line(command)
        self.link.send_item(self)
        pose = self.link.rec_pose()
        self.link.check_status()
        return pose

    def setPoseAbs(self, pose):
        """Sets the global position (pose) of an item. For example, the position of an object/frame/target with respect to the station origin.
        In  1 : 4x4 homogeneous matrix (pose)"""
        self.link.check_connection()
        command = 'S_Hlocal_Abs'
        self.link.send_line(command)
        self.link.send_item(self)
        self.link.send_pose(pose)
        self.link.check_status()

    def PoseAbs(self):
        """Returns the global position (pose) of an item. For example, the position of an object/frame/target with respect to the station origin.
        Out 1 : 4x4 homogeneous matrix (pose)"""
        self.link.check_connection()
        command = 'G_Hlocal_Abs'
        self.link.send_line(command)
        self.link.send_item(self)
        pose = self.link.rec_pose()
        self.link.check_status()
        return pose

    def Recolor(self, tocolor, fromcolor=None, tolerance=None):
        """Changes the color of a robot/object/tool. A color must must in the format COLOR=[R,G,B,(A=1)] where all values range from 0 to 1.
        Alpha (A) defaults to 1 (100% opaque). Set A to 0 to make an object transparent.
        In  1 : color -> color to change to
        In  2 (optional): color -> filter by self color
        In  3 (optional): int -> optional tolerance to use if a color filter is used (defaults to 0.1)"""
        self.link.check_connection()
        if not fromcolor:
            fromcolor = [0,0,0,0]
            tolerance = 2
        elif not tolerance:
            tolerance= 0.1
        if not (isinstance(tolerance,int) or isinstance(tolerance,float)):
            raise Exception("tolerance must be a scalar")
            
        tocolor = self.link.check_color(tocolor)
        fromcolor = self.link.check_color(fromcolor)
        command = 'Recolor'
        self.link.send_line(command)
        self.link.send_item(self)
        self.link.send_array([tolerance] + fromcolor + tocolor)
        self.link.check_status()

    def Scale(self, scale):
        """Apply a scale to an object to make it bigger or smaller.
        the scale can be uniform (if scale is a float value) or per axis (if scale is a vector [scale_x, scale_y, scale_z]).
        In  1 : scale -> scale to apply"""
        self.link.check_connection()
        if isinstance(scale,float) or isinstance(scale,int):
            scale = [scale, scale, scale]
        elif len(scale) > 3:
            scale = scale[:3]
        elif len(scale) < 3:
            raise Exception("scale must be a single value or a 3-vector value")
        command = 'Scale'
        self.link.send_line(command)
        self.link.send_item(self)
        self.link.send_array(scale)
        self.link.check_status()
        
    #"""Target item calls"""
    def setAsCartesianTarget(self):
        """Sets a target as a cartesian target. A cartesian target moves to cartesian coordinates."""
        self.link.check_connection()
        command = 'S_Target_As_RT'
        self.link.send_line(command)
        self.link.send_item(self)
        self.link.check_status()

    def setAsJointTarget(self):
        """Sets a target as a joint target. A joint target moves to a joints position without regarding the cartesian coordinates."""
        self.link.check_connection()
        command = 'S_Target_As_JT'
        self.link.send_line(command)
        self.link.send_item(self)
        self.link.check_status()

    #"""Robot item calls"""
    def Joints(self):
        """Returns the current joints of a robot or the joints of a target. If the item is a cartesian target, it returns the preferred joints (configuration) to go to that cartesian position.
        Out 1 : double x n -> joints matrix
        Example to convert a nx1 joint Matrix to a vector:
            joints = tr(robot.Joints())
            joints = joints.rows[0]"""
        self.link.check_connection()
        command = 'G_Thetas'
        self.link.send_line(command)
        self.link.send_item(self)
        joints = self.link.rec_array()
        self.link.check_status()
        return joints

    def setJoints(self, joints):
        """Sets the current joints of a robot or the joints of a target. It the item is a cartesian target, it returns the preferred joints (configuration) to go to that cartesian position.
        In  1 : double x n -> joints"""
        self.link.check_connection()
        command = 'S_Thetas'
        self.link.send_line(command)
        self.link.send_array(joints)
        self.link.send_item(self)
        self.link.check_status()

    def setFrame(self, frame):
        """Sets the frame of a robot (user frame). The frame can be either an item or a 4x4 Matrix.
        If "frame" is an item, it links the robot to the frame item. If frame is a 4x4 Matrix, it updates the linked pose of the robot frame.
        In  1 : item/pose -> frame item or 4x4 Matrix (pose of the reference frame)"""
        self.link.check_connection()
        if isinstance(frame,Item):
            command = 'S_Frame_ptr'
            self.link.send_line(command)
            self.link.send_item(frame)
        else:
            command = 'S_Frame'
            self.link.send_line(command)
            self.link.send_pose(frame)
        self.link.send_item(self)
        self.link.check_status()

    def setTool(self, tool):
        """Sets the tool pose of a robot. The tool pose can be either an item or a 4x4 Matrix.
        If "tool" is an item, it links the robot to the tool item. If tool is a 4x4 Matrix, it updates the linked pose of the robot tool.
        In  1 : item/pose -> tool item or 4x4 Matrix (pose of the tool frame)"""
        self.link.check_connection()
        if isinstance(tool,Item):
            command = 'S_Tool_ptr'
            self.link.send_line(command)
            self.link.send_item(tool)
        else:
            command = 'S_Tool'
            self.link.send_line(command)
            self.link.send_pose(tool)        
        self.link.send_item(self)
        self.link.check_status()

    def SolveFK(self, joints):
        """Computes the forward kinematics of the robot for the provided joints.
        In  1 : double x n -> joints
        Out 1 : 4x4 matrix -> pose of the robot tool with respect to the robot frame"""
        self.link.check_connection()
        command = 'G_FK'
        self.link.send_line(command)
        self.link.send_array(joints)
        self.link.send_item(self)
        pose = self.link.rec_pose()
        self.link.check_status()
        return pose

    def SolveIK(self, pose):
        """Computes the inverse kinematics for the specified robot and pose. The joints returned are the closest to the current robot configuration (see SolveIK_All()).
        In  1 : 4x4 matrix -> pose of the robot tool with respect to the robot frame
        Out 1 : double x n -> joints"""
        self.link.check_connection()
        command = 'G_IK'
        self.link.send_line(command)
        self.link.send_pose(pose)
        self.link.send_item(self)
        joints = self.link.rec_array()
        self.link.check_status()
        return joints

    def SolveIK_All(self, pose):
        """Computes the inverse kinematics for the specified robot and pose. The function returns all available joint solutions as a 2D matrix.
        In  1 : 4x4 matrix -> pose of the robot tool with respect to the robot frame
        Out 1 : double x n x m -> joint list (2D matrix)"""
        self.link.check_connection()
        command = 'G_IK_cmpl'
        self.link.send_line(command)
        self.link.send_pose(pose)
        self.link.send_item(self)
        joints_list = self.link.rec_matrix()
        if joints_list.size(0) > 6:
            joints_list = joints_list[:-2,:]
        self.link.check_status()
        return joints_list

    def MoveJ(self, target, blocking=True):
        """Moves a robot to a specific target ("Move Joint" mode). self function blocks until the robot finishes its movements.
        In  1 : joints/pose/item -> target to move to. It can be the robot joints (Nx1 or 1xN), the pose (4x4) or an item (item pointer)
        In  2 (optional): blocking -> True if we want the instruction to wait until the robot finished the movement (default=True)"""
        self.link.moveX(target, self, 1, blocking)

    def MoveL(self, target, blocking=True):
        """Moves a robot to a specific target ("Move Linear" mode). self function waits (blocks) until the robot finishes its movements.
        In  1 : joints/pose/item -> target to move to. It can be the robot joints (Nx1 or 1xN), the pose (4x4) or an item (item pointer)
        In  2 (optional): blocking -> True if we want the instruction to wait until the robot finished the movement (default=True)"""
        self.link.moveX(target, self, 2, blocking)

    def MoveJ_Collision(self, j1, j2, minstep_deg=-1):
        """Checks if a joint movement is free of collision.
        In  1 : joints -> start joints
        In  2 : joints -> destination joints
        In  3 (optional): maximum joint step in degrees
        Out : collision : returns 0 if the movement is free of collision. Otherwise it returns the number of pairs of objects that collided if there was a collision."""
        self.link.check_connection()
        command = 'CollisionMove'
        self.link.send_line(command)
        self.link.send_item(self)
        self.link.send_array(j1)
        self.link.send_array(j2)        
        self.link.send_int(minstep_deg*1000)
        collision = self.link.rec_int()
        self.link.check_status()
        return collision

    def setSpeed(self, speed, accel=-1):
        """Sets the speed and/or the acceleration of a robot.
        In  1 : speed -> speed in mm/s (-1 = no change)
        In  2 : accel (optional) -> acceleration in mm/s2 (-1 = no change)"""
        self.link.check_connection()
        command = 'S_Speed'
        self.link.send_line(command)
        self.link.send_int(speed*1000);
        self.link.send_int(accel*1000);        
        self.link.send_item(self)
        self.link.check_status()

    def setZoneData(self, zonedata):
        """Sets the robot zone data value.
        In  1 : zonedata value (int) (robot dependent, set to -1 for fine movements)"""
        self.link.check_connection()
        command = 'S_ZoneData'
        self.link.send_line(command)
        self.link.send_int(zonedata*1000);
        self.link.send_item(self)
        self.link.check_status()

    def ShowSequence(self, matrix):
        """Displays a sequence of joints
        In  1 : joint sequence as a 6xN matrix or instruction sequence as a 7xN matrix"""
        self.link.check_connection()
        command = 'Show_Seq'
        self.link.send_line(command)
        self.link.send_matrix(matrix);
        self.link.send_item(self)
        self.link.check_status()
    
    def Busy(self):
        """Checks if a robot is currently moving.
        Out 1 : int -> busy status (1=moving, 0=stopped)"""
        self.link.check_connection()
        command = 'IsBusy'
        self.link.send_line(command)
        self.link.send_item(self)
        busy = self.link.rec_int()
        self.link.check_status()
        return busy

    def WaitMove(self, timeout=300):
        """Waits (blocks) until the robot finishes its movement.
        In  1 (optional): timeout -> Max time to wait for robot to finish its movement (in seconds)"""
        self.link.check_connection()
        command = 'WaitMove'
        self.link.send_line(command)
        self.link.send_item(self)
        self.link.check_status()
        self.link.COM.settimeout(timeout)
        self.link.check_status()#will wait here
        self.link.COM.settimeout(self.link.TIMEOUT)
        #busy = self.link.Is_Busy(self.item)
        #while busy:
        #    busy = self.link.Is_Busy(self.item)

    #"""Program item calls"""
    def RunProgram(self):
        """Runs a program. It returns the number of instructions that can be executed successfully (a quick check is performed before the program starts)
        This is a non-blocking call.
        Out 1 : int -> number of instructions that can be executed"""
        self.link.check_connection()
        command = 'RunProg'
        self.link.send_line(command)
        self.link.send_item(self)
        prog_status = self.link.rec_int()
        self.link.check_status()
        return prog_status
        
    def addMoveJ(self, itemtarget):
        """Adds a new robot move joint instruction to a program.
        In  1 : item -> target to move to"""
        self.link.check_connection()
        command = 'Add_INSMOVE'
        self.link.send_line(command)
        self.link.send_item(itemtarget)
        self.link.send_item(self)
        self.link.send_int(1)
        self.link.check_status()

    def addMoveL(self, itemtarget):
        """Adds a new robot move linear instruction to a program.
        In  1 : item -> target to move to"""
        self.link.check_connection()
        command = 'Add_INSMOVE'
        self.link.send_line(command)
        self.link.send_item(itemtarget)
        self.link.send_item(self)
        self.link.send_int(2)
        self.link.check_status()
                

        
        
        
