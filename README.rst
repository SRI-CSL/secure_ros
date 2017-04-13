Introduction
============

Secure ROS is a "fork" of core ROS packages to enable secure communication among ROS nodes (not to be confused with SROS, http://wiki.ros.org/SROS/). The main goal of Secure ROS is to enable secure communication for regular users of ROS. 

The modified packages are `rosmaster`, `rosgraph`, `roscpp`, `rospy`, `xmlrpcpp` and `nodelet`. 

Secure ROS uses IPSec in transport mode and modified versions of the ROS master, `rospy` module and `roscpp` library to ensure secure communication.  At run time, the user can specify authorized subscribers and publishers to topics, setters and getters to parameters and providers (servers) and requesters (clients) of services in the form of a YAML configuration file for the ROS master. Secure ROS allows only authorized nodes to connect to topics, services and parameters listed in the configuration file. 

Secure ROS keeps ROS public API intact, allowing user level ROS packages to be used without modification. If no YAML configuration is provided, Secure ROS behaves like regular ROS (with no security).

The Secure ROS website is at http://secure-ros.csl.sri.com/.

Installation
------------

Please install ROS first (see http://wiki.ros.org/ROS/Installation).
  
To install Secure ROS, download the debian package from http://secure-ros.csl.sri.com/download/ and install with `dpkg`.  E.g. 

::

  dpkg -i secure-ros-indigo-secure-ros_0.9.1-1_amd64.deb

The Secure ROS package files are installed in ``/opt/secure_ros/<release>/`` and includes modified versions of some ROS libraries and modules.

Usage
-----

The user can use the features of Secure ROS by sourcing ``/opt/secure_ros/<release>/setup.bash`` instead of ``/opt/ros/<release>/setup.bash``. This will automatically include all packages installed in ``/opt/ros/<release>/``. 

To enable secure communication, you also need to provide an authorization configuration file (relative to the working directory) when starting the ROS master. E.g. ::

  ROS_AUTH_FILE=ros_auth.yaml roscore

If the ``ROS_AUTH_FILE`` environment variable is not defined, the Secure ROS master behaves like the regular ROS master. The configuration file is described in the next section. For further details, please see the example. 

Authorization Configuration 
---------------------------

A sample configuration file from the `multi_pubsub` example is provided below. The authorization configuration YAML file is a dictionary with the following keys: ``aliases``, ``topics``, ``nodes``, ``parameters`` and ``services``. The ``topics`` key is required but the others are optional. These key-value pairs are described in detail below.

::

  aliases:
    vm2: [192.168.10.202]
    vm3: [192.168.10.203]
  topics:
    /chatter:
      publishers: [vm2]
      subscribers: [vm3]
    /counter:
      publishers: [vm2]
      subscribers: [vm3]
  nodes:
    /talker: [vm2]
    /listener: [vm3]

- ``aliases``: The value of the ``aliases`` key is a dict of `(key,value)` pairs where each keys is an alias and each value is the corresponding *list* of IP addresses. The IP address can also be a hostname that can be resolved by the master.

- ``topics``: The value of the ``topics`` key is a dict of `(key,value)` pairs where the keys are *all* the allowed topics excepting reserved topics. The reserved topics are ``rosout`` and ``rosout_agg``. Each topic value is another dictionary with two required keys: ``publishers`` and ``subscribers``. The value of the ``publisher`` and ``subscriber`` keys is a *list* of IP addresses that are the authorized publishers and subscribers to that topic respectively.

- ``nodes``: This is an optional key. The value of the key is a dict of `(key,value)` pairs where the keys are node names excepting reserved nodes. The reserved nodes are ``rosout``. Each value is list of the IP addresses from which that node can register. If a node is listed as a key, then that node may only register from one of the associated IP addresses. If a node is not listed as a key or if the ``nodes`` key is not present, then any node may register from the list of ``authorized_ip_addresses``.

- ``parameters``: This is an optional key. The value of the key is a dict of `(key,value)` pairs where the keys are parameter names (or parameter prefixes) other than the reserved parameters. The value of each key is another dict with two keys: ``setters`` (required) and ``getters`` (optional). The value of the ``setters`` and ``getters`` keys is a *list* of IP addresses that are the authorized setters and getters of that parameter respectively. If a parameter name is not listed, then that parameter may be set and accessed from any IP address in ``authorized_ip_addresses``. If a parameter does not have the ``getters`` key, that parameter may be accessed from any IP address in ``authorized_ip_addresses``. 

  The reserved parameters are: ``/run_id``, ``/rosversion``, ``/rosdistro``, ``/tcp_keepalive``, ``/use_sim_time``, ``/enable_statistics``, ``/statistics_window_min_elements``, ``/statistics_window_max_elements``, ``/statistics_window_min_size``, ``/statistics_window_max_size`` 

- ``services``: This is an optional key. The value of the key is a dict of `(key,value)` pairs. The keys are service names other than the reserved services. The value of each key is another dict with two keys: ``providers`` (required) and ``requesters`` (optional). The value of the ``providers`` and ``requesters`` keys is a *list* of IP addresses that are the authorized providers and requesters of that service respectively. If a service name is not listed, then that service may be set and accessed from any IP address in ``authorized_ip_addresses``. If a service does not have the ``requesters`` key, that service may be accessed from any IP address in ``authorized_ip_addresses``.

``authorized_ip_addresses`` is an internal set containing all the IP addresses that are listed in the authorization file. Any request from an IP address not on this list is, in general, discarded. This list is also used as an authorized list for nodes, parameters or services that are not explicitly listed in the file.

