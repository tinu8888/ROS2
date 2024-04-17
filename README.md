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