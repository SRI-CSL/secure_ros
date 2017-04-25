Simple talker listener example (`simple_pubsub`)
------------------------------------------------

This example is a simple example to run Secure ROS using the built-in talker and listener example in a linux machine. The ROS master and nodes run on the same machine. 

Requirements
~~~~~~~~~~~~

Install ROS and corresponding version of Secure ROS on Ubuntu (ROS Indigo on 14.04 and ROS Kinetic on 16.04). 

Using Secure ROS
~~~~~~~~~~~~~~~~

You need to source Secure ROS in each terminal. ::

  source /opt/secure_ros/kinetic/setup.bash

Authorization configuration
~~~~~~~~~~~~~~~~~~~~~~~~~~~

The authorization file (``ros_auth_simple_pubsub.yaml``) is shown below. It allows certain nodes and all topics to be published to and subscribed to from your machine (`machine1`, IP address: ``127.0.0.1``). ::

  aliases:
    machine1: [127.0.0.1]
  topics:
    /chatter:
      publishers: [machine1]
      subscribers: [machine1]
    /counter:
      publishers: [machine1]
      subscribers: [machine1]
  nodes:
    /talker: [machine1]
    /listener: [machine1]


Running the example
~~~~~~~~~~~~~~~~~~~

All the nodes are run on the same machine (`machine1`). Change the working directory to ``examples/simple_pubsub`` in all the cases and start the master and nodes in separate consoles. The ``ROS_AUTH_FILE`` path is resolved with respect to the current working directory. Note that if ``ROS_IP`` is not set, the ROS master is bound to all the network devices.  ``ROS_MASTER_URI`` has the default value ``http://<hostname>:11311`` which resolves to ``127.0.0.1`` in this case. ::

  unset ROS_IP
  unset ROS_MASTER_URI
  source /opt/secure_ros/kinetic/setup.bash

* Start the master node from the ``simple_pubsub`` directory. ::

    ROS_AUTH_FILE=ros_auth_simple_pubsub.yaml roscore

* Start the talker node. ::

    rosrun test_secure_ros talker.py

  You may also run the C++ version *instead* of the python version (``rosrun test_secure_ros talker2``)

* Start the listener node. ::

    rosrun test_secure_ros listener.py

  You may also run the C++ version *instead* of the python version (``rosrun test_secure_ros listener2``)


Test Secure ROS
~~~~~~~~~~~~~~~

We test Secure ROS by querying the master locally and externally as described in the following sections.

Test locally 
^^^^^^^^^^^^

You need to source Secure ROS in each terminal. ``ROS_MASTER_URI`` has the default value which is ``http://<hostname>:11311`` and the requests are routed through the local network. ::

  unset ROS_IP
  unset ROS_MASTER_URI
  source /opt/secure_ros/kinetic/setup.bash

Run the commands in :ref:`simple_pubsub_test`. You should have full access.

Test externally 
^^^^^^^^^^^^^^^

If you set ``ROS_MASTER_URI`` with the external IP address of your machine (e.g. ``http://192.168.10.201:11311``), then the requests are routed through the network and not loopback (``127.0.0.1``).

*You will need to replace the IP address* ``192.168.10.201`` *with your actual IP address.* ::

  source /opt/secure_ros/kinetic/setup.bash
  export ROS_IP=192.168.10.201
  export ROS_MASTER_URI=http://192.168.10.201:11311

Run the commands in :ref:`simple_pubsub_test`. Secure ROS will deny the requests since the external IP address (``192.168.10.201``) is not an authorized IP address for any of these topics. 

You may repeat this example with ``ros_auth_simple_pubsub2.yaml`` where the external IP address has been added to the alias for `machine1`. 
(You will need to replace the IP address ``192.168.10.201`` in ``ros_auth_simple_pubsub2.yaml`` with your actual IP address.) 
In this case, irrespective of whether you use the local (``http://localhost:11311``) or external IP address (``http://192.168.10.201:11311``) in the ``ROS_MASTER_URI`` on `machine1`, you will be able to gain full access.

*However*, you will *not* be able to access the nodes from another machine (`machine2`, IP address e.g.: ``192.168.10.202``).

On machine2 (or any other machine on the network), set ``ROS_MASTER_URI``. :: 

  source /opt/secure_ros/kinetic/setup.bash
  export ROS_MASTER_URI=http://192.168.10.201:11311/

You will also need to set ``ROS_IP`` if the hostname of `machine2` cannot be resolved from `machine1`. Please see `ROS Network set-up <http://wiki.ros.org/ROS/NetworkSetup>`_ for details. ::

  export ROS_IP=192.168.10.202

You may then run the commands in :ref:`simple_pubsub_test`. You should be unable to register with the master, subscribe, publish, or otherwise access information. 

Test from network
^^^^^^^^^^^^^^^^^

You may modify the previous example by adding a second authorized machine (e.g. `machine2` with IP address ``192.168.10.202``) to the subscribers for topic ``/chatter`` (``ros_auth_simple_pubsub_network.yaml``). Run the following commands in each terminal in `machine1`. You only have to set ``ROS_IP`` on all machines if all the hostnames cannot be resolved by all the machines as is standard `ROS Network set-up <http://wiki.ros.org/ROS/NetworkSetup>`_. ::

  source /opt/secure_ros/kinetic/setup.bash
  export ROS_MASTER_URI=http://192.168.10.201:11311
  export ROS_IP=192.168.10.201


To start the master node on `machine1`, ::

  ROS_AUTH_FILE=ros_auth_simple_pubsub2.yaml roscore

Start the talker on `machine1`, ::

  rosrun test_secure_ros talker.py

Start the listener on `machine1`, ::

  rosrun test_secure_ros listener.py

On machine2, set ``ROS_MASTER_URI``. :: 

  source /opt/secure_ros/kinetic/setup.bash
  export ROS_MASTER_URI=http://192.168.10.201:11311/

You will also need to set ``ROS_IP`` if the hostname of `machine2` cannot be resolved from `machine1`. ::

  export ROS_IP=192.168.10.202

You should then be able to subscribe to ``/chatter`` from `machine2`. ::

  rostopic echo /chatter

However you will be unable to subscribe to ``/counter`` from `machine2`. ::

  rostopic echo /chatter

You will also have restricted access to information from the master on `machine2`. E.g. `rostopic list` will list only a subset of the topics (i.e. the topics that `machine2` needs to know). 


.. _simple_pubsub_test:

ROS commands 
^^^^^^^^^^^^
You may test Secure ROS by trying ROS command line tools. 

* Query rosmaster about topics ::

    rostopic list 

* Subscribe to topics ::

    rosrun test_secure_ros listener.py --anon 

* Publish to topics ::

    rosrun test_secure_ros talker.py --anon

* Call service (e.g. ``/talker/get_loggers``) ::

    rosservice call /talker/get_loggers

* Get parameters ::

    rosparam list 
    rosparam get /rosdistro 

* Set parameters ::

    rosparam set /rosdistro "jade"
    rosparam get /rosdistro 

* Kill nodes ::

    rosnode kill -a 

