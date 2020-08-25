''' ros2 shell wrapper'''
# shell
from subprocess import Popen, PIPE, STDOUT
from io import TextIOWrapper
from multiprocessing import Process, Queue
#ros2
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
#messages
from std_msgs.msg import String


class Shell:
    ''' run command in encapsulated shell'''
    def __init__(self):
        self.launch()
        # store commands to be send to shell
        self.q = Queue()

    def launch(self):
        '''start shell command in Popen'''
        self.proc = Popen(
            # command
            'python3 repeater.py',
            shell=True,
            stdin=PIPE,
            stdout=PIPE,
            stderr=STDOUT
            )
        self.stdin = TextIOWrapper(
            self.proc.stdin,
            encoding='utf-8',
            line_buffering=True
            )
        self.stdout = TextIOWrapper(
            self.proc.stdout,
            encoding='utf-8',
            )

    def relaunch(self):
        '''in case shell terminates'''
        self.proc.kill()
        self.proc.wait()
        self.launch()
        print('restarting shell')


def write_string(mav):
    '''send message from queue tu shell(threadsave)'''
    while True:
        # blocks till q is filled
        mav.stdin.write(mav.q.get() + '\n')


class ReadShell(Node):
    '''processing and publishing shell output'''
    def __init__(self, shell, name=None):
        super().__init__('mavros2_read')
        self.node = rclpy.create_node(name or type(self).__name__)
        self.shell = shell
        self.publisher = self.node.create_publisher(String, 'shell_output', 10)

        self.timer = self.create_timer(0.01, self.read_callback)

    def read_callback(self):
        '''check if new message has arrived'''
        output = self.shell.stdout.readline()
        # check if shell is still running, if not empty output and restart
        if self.shell.proc.poll() is not None and len(output) == 0:
            self.shell.relaunch()
        else:
            if len(output) > 0:

                # publish output
                message = String()
                #remove '\n'
                message.data = output[:-1]
                self.publisher.publish(message)


class WriteShell(Node):
    '''adds incomming messages to queue to be send to shell'''
    def __init__(self, shell, name=None):
        super().__init__('shell_write')
        self.shell = shell

        self.node = rclpy.create_node(name or type(self).__name__)
        self.generic_input = self.create_subscription(
            String,
            'shell_input',
            self.incomming_msg,
            10
            )

    def incomming_msg(self, msg):
        '''adds incoming message to input queue'''
        self.shell.q.put(msg.data)



def main(args=None):
    '''start interaction with shell'''
    #TODO init parameter
    # print(args)
    shell = Shell()

    Process(target=write_string, args=(shell,)).start()

    rclpy.init(args=args)
    shell_read = ReadShell(shell)
    shell_write = WriteShell(shell)
    executor = MultiThreadedExecutor()
    executor.add_node(shell_read)
    executor.add_node(shell_write)
    executor.spin()
    executor.shutdown()
    shell_read.destroy_node()
    shell_write.destroy_node()

if __name__ == '__main__':
    main()
