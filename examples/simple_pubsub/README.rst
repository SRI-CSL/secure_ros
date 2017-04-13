Simple talker listener example (`simple_pubsub`)
------------------------------------------------

This example is a simple example to run Secure ROS using the built-in talker and listener example in a linux machine. The ROS master and nodes run on the same machine. 

Requirements
~~~~~~~~~~~~

Install ROS and corresponding version of Secure ROS on Ubuntu (ROS Indigo on 14.04 and ROS Kinetic on 16.04). 

Using Secure ROS
~~~~~~~~~~~~~~~~

You need to source Secure ROS in each terminal. ::

  source /opt/secure_ros/indigo/setup.bash

Authorization configuration
~~~~~~~~~~~~~~~~~~~~~~~~~~~

The authorization file (``ros_auth_simple_pubsub.yaml``) only allows certain nodes and all topics to be published to and subscribed to from your machine (`machine1`, IP address: ``127.0.0.1``). 

Running the example
~~~~~~~~~~~~~~~~~~~

All the nodes are run on a single machine (`machine1`). Please start the master and nodes in separate consoles. Change the working directory to ``examples/simple_pubsub`` in all the cases. The ``ROS_AUTH_FILE`` path is resolved with respect to the current working directory. 

* Start the master node from the ``simple_pubsub`` directory. ::

    ROS_AUTH_FILE=ros_auth_simple_pubsub.yaml roscore

* Start the talker node. ::

    rosrun test_secure_ros talker.py

  You may also run the C++ version *instead* of the python version (``rosrun test_secure_ros talker2``)

* Start the listener node. ::

    rosrun test_secure_ros listener.py

  You may also run the C++ version *instead* of the python version (``rosrun test_secure_ros listener2``)


.. _simple_pubsub_test:

Test example
~~~~~~~~~~~~

You need to source Secure ROS in each terminal. ::

  source /opt/secure_ros/indigo/setup.bash

You may test Secure ROS by trying different ROS commands listed below. 

* Query rosmaster about topics ::

    rostopic list 

* Subscribing to topics ::

    rosrun test_secure_ros listener.py --anon 

* Publishing to topics ::

    rosrun test_secure_ros talker.py --anon

* Service client (e.g. ``/talker/get_loggers``) ::

    rosservice call /talker/get_loggers

* Get parameters ::

    rosparam list 
    rosparam get /rosdistro 

* Set parameters ::

    rosparam set /rosdistro "jade"
    rosparam get /rosdistro 

* Killing nodes ::

    rosnode kill -a 

If ``ROS_IP`` is not set, the ROS master is bound to all the network devices. In the test examples, ``ROS_MASTER_URI`` has the default value which is ``http://<hostname>:11311``. If you set ``ROS_MASTER_URI`` with the external IP address of your machine (e.g. ``http://192.168.10.201:11311``), then the requests are routed through the ethernet device (and not localhost).
Secure ROS will deny the requests since the external IP address (``192.168.10.201``) is not an authorized IP address for any of these topics.


Variations of talker listener example (`simple_pubsub`)
-------------------------------------------------------

In this section we list variations on the above example.

Standalone version 
~~~~~~~~~~~~~~~~~~

You may repeat the original example with ``ros_auth_simple_pubsub2.yaml`` where the external IP address has been added to the alias for `machine1`. (You will need to replace the IP address in ``ros_auth_simple_pubsub2.yaml`` with your actual IP address.)

To start the master node on `machine1`, ::

  ROS_AUTH_FILE=ros_auth_simple_pubsub2.yaml roscore

Start the talker on `machine1`, ::

  rosrun test_secure_ros talker.py

Start the listener on `machine1`, ::

  rosrun test_secure_ros listener.py

In this case, irrespective of whether you use the local (``http://localhost:11311``) or external (``http://192.168.10.201``) IP address in the ``ROS_MASTER_URI`` on `machine1`, you will be able to gain full access.

*However*, you will *not* be able to access the nodes from another machine (`machine2`, IP address e.g.: ``192.168.10.202``).

On machine2 (or any other machine on the network), set ``ROS_MASTER_URI``. :: 

  export ROS_MASTER_URI=http://192.168.10.201:11311/

You will also need to set ``ROS_IP`` if the hostname of `machine2` cannot be resolved from `machine1`. ::

  export ROS_IP=192.168.10.202

You may then run the commands in :ref:`simple_pubsub_test`. You should be unable to register with the master, subscribe, publish, or otherwise access information. 


Network version 
~~~~~~~~~~~~~~~

You may modify the previous example by adding a second authorized machine (e.g. `machine2` with IP address ``192.168.10.202``) to the subscribers for topic ``/chatter`` (``ros_auth_simple_pubsub_network.yaml``).

To start the master node on `machine1`, ::

  ROS_AUTH_FILE=ros_auth_simple_pubsub2.yaml roscore

Start the talker on `machine1`, ::

  rosrun test_secure_ros talker.py

Start the listener on `machine1`, ::

  rosrun test_secure_ros listener.py

On machine2, set ``ROS_MASTER_URI``. :: 

  export ROS_MASTER_URI=http://192.168.10.201:11311/

You will also need to set ``ROS_IP`` if the hostname of `machine2` cannot be resolved from `machine1`. ::

  export ROS_IP=192.168.10.202

You should then be able to subscribe to ``/chatter`` from `machine2`. ::

  rostopic echo /chatter

However you will be unable to subscribe to ``/counter`` from `machine2`. ::

  rostopic echo /chatter

You will also have restricted access to information from the master on `machine2`. E.g. `rostopic list` will list only a subset of the topics (i.e. the topics that `machine2` needs to know). 

