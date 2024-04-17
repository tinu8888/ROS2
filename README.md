# Ros 2 Publisher and Subscriber Python
### ~By Vivek Tiwari

**Note: ROS2 Installation required**
## Creates a Publisher in ROS2

1. Create a ROS2 workspace:

```bash
mkdir -p ~/my_prj/src
cd ~/my_prj/src
```
2. Create a new package:

```bash
ros2 pkg create --build-type ament_python my_package
```
3. Create file my_nodepub.py inside my_package/my_package/

```bash
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MyNode(Node):

    def __init__(self):
        super().__init__('my_node')
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello, ROS2 %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    node = MyNode()

    rclpy.spin(node)

    # Destroy the node explicitly
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

```

4. Dir tree will look like this:

```bash
my_package/
├── my_package/
│   ├── __init__.py
│   └── my_nodepub.py
├── package.xml
└── setup.py
```

5. Give executable permission:

```bash
chmod +x my_nodepub.py
```

6. Modify entry_points in  `setup.py`

```bash
    entry_points={
        'console_scripts': [
            'my_nodepub = my_package.my_nodepub:main',
        ],
    },
```
7. Build your package:

```bash
cd ~/my_prj
colcon build
```
8. Source your workspace:

```bash
source install/setup.bash
```
9. run Ros2 python pakage:

```bash
ros2 run my_package my_nodepub
```

10. Output  will be like this:

```bash
[INFO] [1713383462.206288842] [my_node]: Publishing: "Hello, ROS2 0"
[INFO] [1713383463.185656533] [my_node]: Publishing: "Hello, ROS2 1"
[INFO] [1713383464.185711677] [my_node]: Publishing: "Hello, ROS2 2"
[INFO] [1713383465.185665114] [my_node]: Publishing: "Hello, ROS2 3"
[INFO] [1713383466.185724134] [my_node]: Publishing: "Hello, ROS2 4"
```

## Creates a Subscriber in ROS2

1. Create file my_nodesub.py inside my_package/my_package/:
```bash
my_package/
├── my_package/
│   ├── __init__.py
│   └── my_nodepub.py
│   └── my_nodesub.py
├── package.xml
└── setup.py
```

```bash
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MySubscriber(Node):

    def __init__(self):
        super().__init__('my_subscriber')
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
def main(args=None):
    rclpy.init(args=args)

    my_subscriber = MySubscriber()

    rclpy.spin(my_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    my_subscriber.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
```

2. Modify entry_points in  `setup.py`

```bash
    entry_points={
        'console_scripts': [
            'my_nodepub = my_package.my_nodepub:main',
            'my_nodesub = my_package.my_nodesub:main',

        ],
    },
```
3. Give executable permission:

```bash
chmod +x my_nodesub.py
```
4. Build your package:

```bash
cd ~/my_prj
colcon build
```
5. Source your workspace:

```bash
source install/setup.bash
```

6. Run your nodes:
```bash
ros2 run my_package my_nodesub
```
7. Output  will be like this:

```bash
[INFO] [1713393248.419769036] [my_subscriber]: I heard: "Hello, ROS2 0"
[INFO] [1713393249.419560374] [my_subscriber]: I heard: "Hello, ROS2 1"
[INFO] [1713393250.419717666] [my_subscriber]: I heard: "Hello, ROS2 2"
[INFO] [1713393251.419509661] [my_subscriber]: I heard: "Hello, ROS2 3"
[INFO] [1713393252.419481923] [my_subscriber]: I heard: "Hello, ROS2 4"
```